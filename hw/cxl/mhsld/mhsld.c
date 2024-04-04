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

static void cxl_mhsld_realize(PCIDevice *pci_dev, Error **errp)
{
    CXLMHSLDState *s = CXL_MHSLD(pci_dev);
    MemoryRegion *mr;
    struct stat st;
    int fd = -1;
    size_t state_size;
    size_t dc_size;

    ct3_realize(pci_dev, errp);

    mr = host_memory_backend_get_memory(s->ct3d.dc.host_dc);
    if (!mr)
        return;
    dc_size = memory_region_size(mr);
    if (!dc_size) {
        error_setg(errp, "MHSLD does not have dynamic capacity to manage");
        return;
    }

    if (s->mhd_head >= MHSLD_HEADS) {
        error_setg(errp, "MHD Head ID must be between 0-7");
        goto cleanup_lock;
    }

    /* Create or Open the MHD State file */
    state_size = (dc_size / MHSLD_BLOCK_SZ) + sizeof(MHSLDSharedState);
    if (s->mhd_init) {
        fd = open(s->mhd_state_file, O_RDWR | O_CREAT, 0666);
        if (fd == -1) {
            error_setg(errp, "Failed to create mhd state file");
            return;
        }
        if (ftruncate(fd, state_size) == -1) {
            error_setg(errp, "Failed to set mhd state file size");
            goto cleanup;
        }
    } else {
        fd = open(s->mhd_state_file, O_RDWR);
        if (fd == -1) {
            error_setg(errp, "Failed to open mhd state file");
            return;
        }
        if (fstat(fd, &st) == -1) {
            error_setg(errp, "Failed to get mhd state file size");
            goto cleanup;
        }
        if (st.st_size < state_size) {
            error_setg(errp, "State file is too small to manage this device");
            goto cleanup;
        }
    }

    /* map the state and initialize it as needed */
    s->mhd_state_fd = fd;
    s->mhd_state_size = state_size;
    s->mhd_state = mmap(NULL, state_size, PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0);
    if (s->mhd_state == MAP_FAILED) {
        error_setg(errp, "Failed to mmap mhd state file");
        goto cleanup;
    }

    /* Acquire the state region for modification */
    if (flock(s->mhd_state_fd, LOCK_EX) == -1) {
        error_setg(errp, "Failed to acquire exclusive lock on mhd state file");
        goto cleanup_unmap;
    }

    /* If this is initializing the system, completely reset the mhd state file */
    if (s->mhd_init) {
        memset(s->mhd_state, 0, s->mhd_state_size);
        s->mhd_state->nr_heads = MHSLD_HEADS;
        s->mhd_state->nr_lds = MHSLD_HEADS;
        s->mhd_state->nr_blocks = dc_size / MHSLD_BLOCK_SZ;
    }

    /* Set the LD ownership for this head to this system */
    s->mhd_state->ldmap[s->mhd_head] = s->mhd_head;
    msync(s->mhd_state, s->mhd_state_size, MS_SYNC);
    flock(s->mhd_state_fd, LOCK_UN);
    return;

cleanup_lock:
    flock(s->mhd_state_fd, LOCK_UN);
cleanup_unmap:
    munmap(s->mhd_state, s->mhd_state_size);
cleanup:
    if (fd != -1)
        close(fd);
}


static void cxl_mhsld_exit(PCIDevice *pci_dev)
{
    CXLMHSLDState *s = CXL_MHSLD(pci_dev);

    ct3_exit(pci_dev);

    if (s->mhd_state_fd) {
        munmap(s->mhd_state, s->mhd_state_size);
        close(s->mhd_state_fd);
        s->mhd_state = NULL;
    }
}

/*
 * mhdcd_allocate_extents:
 *
 * 1. acquire flock(EX)
 * 2. for the extent requested, check whether all the bits in blocks are free
 *    a. if not, then return an error
 * 3. set the bit for all blocks belonging to the extent for this head id
 * 4. release flock(UN)
 *
 */

/*
 * mhdcd_deallocate_extents:
 * 1. acquire flock(EX)
 * 2. for the extents described, clear the bits in those blocks for this head id
 * 3. release flock(UN)
 */

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
    flock(s->mhd_state_fd, LOCK_EX);
    blocks = s->mhd_state->nr_blocks;
    for (i = 0; i < blocks; i++)
        s->mhd_state->blocks[i] &= ~(1 << s->mhd_head);
    msync(s->mhd_state, s->mhd_state_size, MS_SYNC);
    flock(s->mhd_state_fd, LOCK_UN);
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
     * cvc->mhdcd_allocate_extents = mhdcd_allocate_extents;
     * cvc->mhdcd_deallocate_extents = mhdcd_deallocate_extents;
     */
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
