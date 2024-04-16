/*
 * SPDX-License-Identifier: GPL-2.0-or-later
 *
 * Copyright (c) 2024 MemVerge Inc.
 *
 */

#include <sys/file.h>
#include "qemu/osdep.h"
#include "hw/irq.h"
#include "migration/vmstate.h"
#include "qapi/error.h"
#include "hw/cxl/cxl.h"
#include "hw/cxl/cxl_mailbox.h"
#include "hw/cxl/cxl_device.h"
#include "hw/pci/pcie.h"
#include "hw/pci/pcie_port.h"
#include "hw/qdev-properties.h"
#include "sysemu/hostmem.h"
#include "mhsld.h"

#define TYPE_CXL_MHSLD "cxl-mhsld"
OBJECT_DECLARE_TYPE(CXLMHSLDState, CXLMHSLDClass, CXL_MHSLD)

/*
 * CXL r3.0 section 7.6.7.5.1 - Get Multi-Headed Info (Opcode 5500h)
 *
 * This command retrieves the number of heads, number of supported LDs,
 * and Head-to-LD mapping of a Multi-Headed device.
 */
static CXLRetCode cmd_mhd_get_info(const struct cxl_cmd *cmd,
                                   uint8_t *payload_in,
                                   size_t len_in,
                                   uint8_t *payload_out,
                                   size_t *len_out,
                                   CXLCCI * cci)
{
    CXLMHSLDState *s = CXL_MHSLD(cci->d);
    MHDGetInfoInput *input = (void *)payload_in;
    MHDGetInfoOutput *output = (void *)payload_out;

    uint8_t start_ld = input->start_ld;
    uint8_t ldmap_len = input->ldmap_len;
    uint8_t i;

    if (start_ld >= s->mhd_state->nr_lds) {
        return CXL_MBOX_INVALID_INPUT;
    }

    output->nr_lds = s->mhd_state->nr_lds;
    output->nr_heads = s->mhd_state->nr_heads;
    output->resv1 = 0;
    output->start_ld = start_ld;
    output->resv2 = 0;

    for (i = 0; i < ldmap_len && (start_ld + i) < output->nr_lds; i++) {
        output->ldmap[i] = s->mhd_state->ldmap[start_ld + i];
    }
    output->ldmap_len = i;

    *len_out = sizeof(*output) + output->ldmap_len;
    return CXL_MBOX_SUCCESS;
}

static const struct cxl_cmd cxl_cmd_set_mhsld[256][256] = {
    [MHSLD_MHD][GET_MHD_INFO] = {"GET_MULTI_HEADED_INFO",
        cmd_mhd_get_info, 2, 0},
};

static Property cxl_mhsld_props[] = {
    DEFINE_PROP_UINT32("mhd-head", CXLMHSLDState, mhd_head, ~(0)),
    DEFINE_PROP_STRING("mhd-state_file", CXLMHSLDState, mhd_state_file),
    DEFINE_PROP_BOOL("mhd-init", CXLMHSLDState, mhd_init, false),
    DEFINE_PROP_END_OF_LIST(),
};

static int cxl_mhsld_state_open(const char *filename, int flags) {
    char name[128];
    snprintf(name, sizeof(name), "/%s", filename);
    return shm_open(name, flags, 0666);
}

static int cxl_mhsld_state_unlink(const char *filename) {
    char name[128];
    snprintf(name, sizeof(name), "/%s", filename);
    return shm_unlink(name);
}

static int cxl_mhsld_state_create(const char *filename, size_t size) {
    int fd, rc;

    fd = cxl_mhsld_state_open(filename, O_RDWR | O_CREAT);
    if (fd == -1)
        return -1;

    rc = ftruncate(fd, size);

    if (rc) {
        close(fd);
        return -1;
    }

    return fd;
}

static void cxl_mhsld_state_initialize(CXLMHSLDState *s, size_t dc_size) {
    if (!s->mhd_init)
        return;

    memset(s->mhd_state, 0, s->mhd_state_size);
    s->mhd_state->nr_heads = MHSLD_HEADS;
    s->mhd_state->nr_lds = MHSLD_HEADS;
    s->mhd_state->nr_blocks = dc_size / MHSLD_BLOCK_SZ;
}

static MHSLDSharedState *cxl_mhsld_state_map(CXLMHSLDState *s) {
    void *map;
    size_t size = s->mhd_state_size;
    int fd = s->mhd_state_fd;

    if (fd < 0)
        return NULL;

    map = mmap(NULL, size, PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0);
    if (map == MAP_FAILED)
        return NULL;

    return (MHSLDSharedState *)map;
}

static bool cxl_mhsld_state_set(CXLMHSLDState *s,
                                size_t block_start,
                                size_t block_count)
{
    uint8_t prev, *block;
    size_t i;
    bool fail = false;

    /*
     * Try to claim all extents from start -> start + count;
     * break early if a claimed extent is encountered
     */
    for (i = 0; i < block_count; ++i) {
        block = &s->mhd_state->blocks[block_start + i];
        prev = __sync_val_compare_and_swap(block, 0, (1 << s->mhd_head));
        if (prev != 0) {
            fail = true;
            break;
        }
    }

    if (!fail)
        return true;

    /* Roll back incomplete claims */
    for (;; --i) {
        block = &s->mhd_state->blocks[block_start + i];
        __sync_fetch_and_and(block, ~(1u << s->mhd_head));
        if (i == 0)
            break;
    }

    return false;
}

static void cxl_mhsld_state_clear(CXLMHSLDState *s,
                                  size_t block_start,
                                  size_t block_count)
{
    size_t i;
    uint8_t *block;

    for (i = 0; i < block_count; ++i) {
        block = &s->mhd_state->blocks[block_start + i];
        __sync_fetch_and_and(block, ~(1u << s->mhd_head));
    }
}

static bool cxl_mhsld_access_valid(PCIDevice *d, uint64_t addr, unsigned int size) {
    return true;
}

/*
 * Triggered during an add_capacity command to a CXL device:
 * takes a list of extent records and preallocates them,
 * in anticipation of a "dcd accept" response from the host.
 *
 * Extents that are not accepted by the host will be rolled
 * back later.
 */
static bool cxl_mhsld_reserve_extents_in_region(PCIDevice *pci_dev,
                                                CXLDCExtentRecordList *records,
                                                CXLDCRegion *region) {
    uint64_t len, dpa;
    bool rc;

    CXLMHSLDState *s = CXL_MHSLD(pci_dev);
    CXLDCExtentRecordList *list = records, *rollback = NULL;

    for (; list; list = list->next) {
        len = list->value->len;
        dpa = list->value->offset + region->base;

        /*
         * TODO:
         * The start-block calculation fails if regions have variable
         * block sizes -- we'd need to track region->start_block_idx
         * explicitly, and calculate offset/len relative to that.
         */
        rc = cxl_mhsld_state_set(s, dpa / region->block_size,
                                 len / region->block_size);

        if (!rc) {
            rollback = records;
            break;
        }
    }

    /* Setting the mhd state failed. Roll back the extents that were added */
    for (; rollback; rollback = rollback->next) {
        len = rollback->value->len;
        dpa = list->value->offset + region->base;

        cxl_mhsld_state_clear(s, dpa / region->block_size,
                              len / region->block_size);

        if (rollback == list)
            return false;
    }

    return true;
}

static bool cxl_mhsld_release_extent_in_region(PCIDevice *pci_dev,
                                               CXLDCRegion *region,
                                               uint64_t dpa,
                                               uint64_t len) {
    cxl_mhsld_state_clear(dpa / region->block_size, len / region->block_size);
    return true;
}

static bool cxl_mhsld_test_extent_block_backed(PCIDevice *pci_dev,
                                               CXLDCRegion *region,
                                               uint64_t dpa,
                                               uint64_t len) {
    size_t i;

    dpa = dpa / region->block_size;
    len = len / region->block_size;

    for (i = 0; i < len; ++i)
        if (s->mhd_state->blocks[dpa + i] != (1 << s->mhd_head))
            return false;

    return true;
}

static void cxl_mhsld_realize(PCIDevice *pci_dev, Error **errp)
{
    CXLMHSLDState *s = CXL_MHSLD(pci_dev);
    MemoryRegion *mr;
    int fd = -1;
    size_t dc_size;

    ct3_realize(pci_dev, errp);

    /* Get number of blocks from dcd size */
    mr = host_memory_backend_get_memory(s->ct3d.dc.host_dc);
    if (!mr)
        return;
    dc_size = memory_region_size(mr);
    if (!dc_size) {
        error_setg(errp, "MHSLD does not have dynamic capacity to manage");
        return;
    }

    s->mhd_state_size = (dc_size / MHSLD_BLOCK_SZ) + MHSLD_HEADER_SZ;

    /* Sanity check the head idx */
    if (s->mhd_head >= MHSLD_HEADS) {
        error_setg(errp, "MHD Head ID must be between 0-7");
        return;
    }

    /* Create the state file if this is the 'mhd_init' instance */
    if (s->mhd_init)
        fd = cxl_mhsld_state_create(s->mhd_state_file, s->mhd_state_size);
    else
        fd = cxl_mhsld_state_open(s->mhd_state_file, O_RDWR);

    if (fd < 0) {
        error_setg(errp, "failed to open mhsld state errno %d", errno);
        return;
    }

    s->mhd_state_fd = fd;

    /* Map the state and initialize it as needed */
    s->mhd_state = cxl_mhsld_state_map(s);
    if (!s->mhd_state) {
        error_setg(errp, "Failed to mmap mhd state file");
        close(fd);
        cxl_mhsld_state_unlink(s->mhd_state_file);
        return;
    }

    cxl_mhsld_state_initialize(s, dc_size);

    /* Set the LD ownership for this head to this system */
    s->mhd_state->ldmap[s->mhd_head] = s->mhd_head;
    return;
}


static void cxl_mhsld_exit(PCIDevice *pci_dev)
{
    CXLMHSLDState *s = CXL_MHSLD(pci_dev);

    ct3_exit(pci_dev);

    if (s->mhd_state_fd) {
        munmap(s->mhd_state, s->mhd_state_size);
        close(s->mhd_state_fd);
        cxl_mhsld_state_unlink(s->mhd_state_file);
        s->mhd_state = NULL;
    }
}

static void cxl_mhsld_reset(DeviceState *d)
{
    CXLMHSLDState *s = CXL_MHSLD(d);
    size_t blocks, i;

    ct3d_reset(d);
    cxl_add_cci_commands(&s->ct3d.cci, cxl_cmd_set_mhsld, 512);

    /*
     * Scan s->mhd_state->blocks for any byte with bit s->mhd_head set,
     * and clear it (release the capacity)
     */
    blocks = s->mhd_state->nr_blocks;
    for (i = 0; i < blocks; i++)
        s->mhd_state->blocks[i] &= ~(1 << s->mhd_head);
}

/* TODO: Implement shared device hooks
 *
 * Example: DCD-add events need to validate that the requested extent
 *          does not already have a mapping (or, if it does, it is
 *          a shared extent with the right tagging).
 *
 * Since this operates on the shared state, we will need to serialize
 * these callbacks across QEMU instances via a mutex in shared state.
 */

static void cxl_mhsld_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    PCIDeviceClass *pc = PCI_DEVICE_CLASS(klass);

    pc->realize = cxl_mhsld_realize;
    pc->exit = cxl_mhsld_exit;
    dc->reset = cxl_mhsld_reset;
    device_class_set_props(dc, cxl_mhsld_props);

    /* TODO: type3 class callbacks for multi-host access validation
     * CXLType3Class *cvc = CXL_TYPE3_CLASS(klass);
     * cvc->mhd_access_valid = mhsld_access_valid;
     */
    CXLType3Class *cvc = CXL_TYPE3_CLASS(klass);
    cvc->mhd_get_info = cmd_mhd_get_info;
    cvc->mhd_access_valid = cxl_mhsld_access_valid;
    cvc->mhd_reserve_extents_in_region = cxl_mhsld_reserve_extents_in_region;
    cvc->mhd_release_extent_in_region = cxl_mhsld_release_extent_in_region;
    cvc->mhd_test_extent_block_backed = cxl_mhsld_test_extent_block_backed;
}

static const TypeInfo cxl_mhsld_info = {
    .name = TYPE_CXL_MHSLD,
    .parent = TYPE_CXL_TYPE3,
    .class_size = sizeof(struct CXLMHSLDClass),
    .class_init = cxl_mhsld_class_init,
    .instance_size = sizeof(CXLMHSLDState),
    .interfaces = (InterfaceInfo[]) {
        { INTERFACE_CXL_DEVICE },
        { INTERFACE_PCIE_DEVICE },
        {}
    },
};

static void cxl_mhsld_register_types(void)
{
    type_register_static(&cxl_mhsld_info);
}

type_init(cxl_mhsld_register_types)
