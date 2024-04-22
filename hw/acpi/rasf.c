
#include "qemu/osdep.h"
#include "qemu/log.h"
#include "hw/irq.h"
#include "hw/qdev-properties.h"
#include "hw/sysbus.h"
#include "qom/object.h"

#include "hw/acpi/rasf.h"

typedef struct PccRasfData {
    uint32_t sig;
    /* ACPI 6.5 Table 14.10 Generic Communications Channel Command Field */
    uint16_t command;
#define RASF_CMD_EXECUTE 1
    uint16_t status;
    uint16_t version;

    /* ACPI 6.5 Table 5.77 Platform RAS Capabilities Bitmap */
    uint8_t ras_caps[16];
    uint8_t set_ras_caps[16];

#if defined(CONFIG_ACPI_RAS2_FT)
#define RASF_RAS_CAPS_PATROL_SCRUB 0x1
#define RASF_RAS_CAPS_LA2PA_TRANSLATION 0x2
#else
#define RASF_RAS_CAPS_SCRUB 0x1
#define RASF_RAS_CAPS_SCRUB_EXP_TO_SW 0x2
#endif

    uint16_t num_param_blocks;
    uint32_t set_ras_cap_stat;
#define RASF_RAS_CAP_STAT_SUCCESS 0
#define RASF_RAS_CAP_STAT_NOT_VALID 1
#define RASF_RAS_CAP_STAT_NOT_SUPPORTED 2
#define RASF_RAS_CAP_STAT_BUSY 3
#define RASF_RAS_CAP_STAT_FAILED_F 4
#define RASF_RAS_CAP_STAT_ABORTED 5
#define RASF_RAS_CAP_STAT_INVALID_DATA 6

    /* ACPI 6.5 Table 5-78 Parameter Block Structur for PATROL_SCRUB */
    struct {
        uint16_t type;
#define RASF_TYPE_PATROL_SCRUB 0
        uint16_t version; /* No version defined in spec!? */
        uint16_t length;
        uint16_t cmd;
#define RASF_PATROL_SCRUB_CMD_GET_PARAMS 1
#define RASF_PATROL_SCRUB_CMD_START 2
#define RASF_PATROL_SCRUB_CMD_STOP 3
        uint64_t in_addr_base;
        uint64_t in_addr_size;
        uint64_t out_addr_base;
        uint64_t out_addr_size;
#if defined(CONFIG_ACPI_RAS2_FT)
        uint32_t out_flags;
        uint32_t out_scrub_params;
        uint32_t in_scrub_params;
#else
        uint16_t out_flags;
        uint8_t in_speed;
#endif
    } QEMU_PACKED ras_pb;

} QEMU_PACKED PccRasfData;

typedef struct ACPIRASFState {
    SysBusDevice parent_obj;
    MemoryRegion mr;
    qemu_irq irq;
    PccRasfData data;
    struct {
        uint64_t base;
        uint64_t size;
        uint8_t flags;
#if defined(CONFIG_ACPI_RAS2_FT)
        uint8_t min_scrub_rate;
        uint8_t max_scrub_rate;
        uint8_t cur_scrub_rate;
        uint8_t en_background_patrol_scrub;
#endif
    } scrub_vals;
} ACPIRASFState;

DECLARE_INSTANCE_CHECKER(ACPIRASFState, ACPI_RASF_DEVICE, TYPE_ACPI_RASF);

static uint64_t pcc_read_reg(void *opaque, hwaddr offset, unsigned size)
{
    ACPIRASFState *s;
    uint8_t *data;

    s = ACPI_RASF_DEVICE(opaque);
    data = (uint8_t *)&s->data;
    if (offset + size <= sizeof(s->data)) {
        switch (size) {
        case 1:
            return *((uint8_t *)&data[offset]);
        case 2:
            return *((uint16_t *)&data[offset]);
        case 4:
            return *((uint32_t *)&data[offset]);
        case 8:
            return *((uint64_t *)&data[offset]);
        default:
            return 0;
        }
    }
    switch (offset) {
    case RASF_PCC_DOORBELL_OFFSET:
        return 0;
    case RASF_PCC_INT_ACK_OFFSET:
        return 0;
    default:
        return 0;
    }
}

static void rasf_doorbell(ACPIRASFState *s)
{
#if defined(CONFIG_ACPI_RAS2_FT)
    uint8_t scrub_rate;
#endif
    /* Hammer in some values, the OS should not have written but... */
    s->data.num_param_blocks = 1;
#if defined(CONFIG_ACPI_RAS2_FT)
    s->data.sig = ('R' << 24) | ('A' << 16) | ('S' << 8) | '2';
    s->data.ras_caps[0] = RASF_RAS_CAPS_PATROL_SCRUB;
#else
    s->data.sig = ('R' << 24) | ('A' << 16) | ('S' << 8) | 'F';
    s->data.ras_caps[0] = RASF_RAS_CAPS_SCRUB | RASF_RAS_CAPS_SCRUB_EXP_TO_SW;
#endif
    s->data.ras_pb.length = sizeof(s->data.ras_pb);

    switch (s->data.command) {
    case RASF_CMD_EXECUTE:
        break;
    default:
        return;
    }

    if (s->data.set_ras_caps[0] == 0) {
        /* Initial query only  - only fill in the caps */
        s->data.set_ras_cap_stat = RASF_RAS_CAP_STAT_SUCCESS;
        return;
    }

#if defined(CONFIG_ACPI_RAS2_FT)
    if (!(s->data.set_ras_caps[0] &
          (RASF_RAS_CAPS_PATROL_SCRUB))) {
        s->data.set_ras_cap_stat = RASF_RAS_CAP_STAT_INVALID_DATA;
        return;
    }
#else
    /* Not clear which bit should be set - accept either */
    if (!(s->data.set_ras_caps[0] &
          (RASF_RAS_CAPS_SCRUB | RASF_RAS_CAPS_SCRUB_EXP_TO_SW))) {
        s->data.set_ras_cap_stat = RASF_RAS_CAP_STAT_INVALID_DATA;
        return;
    }
#endif

    if (s->data.ras_pb.type != RASF_TYPE_PATROL_SCRUB) {
        s->data.set_ras_cap_stat = RASF_RAS_CAP_STAT_INVALID_DATA;
        return;
    }

    if (s->data.ras_pb.length != sizeof(s->data.ras_pb)) {
        s->data.set_ras_cap_stat = RASF_RAS_CAP_STAT_INVALID_DATA;
        return;
    }

    switch (s->data.ras_pb.cmd) {
    case RASF_PATROL_SCRUB_CMD_GET_PARAMS:
        s->data.ras_pb.out_addr_base = s->scrub_vals.base;
        s->data.ras_pb.out_addr_size = s->scrub_vals.size;
        s->data.ras_pb.out_flags = s->scrub_vals.flags;
#if defined(CONFIG_ACPI_RAS2_FT)
        s->data.ras_pb.out_scrub_params = (s->scrub_vals.max_scrub_rate << 16) |
					  (s->scrub_vals.min_scrub_rate << 8) |
					  s->scrub_vals.cur_scrub_rate;
#endif
        s->data.set_ras_cap_stat = RASF_RAS_CAP_STAT_SUCCESS;
        return;
    case RASF_PATROL_SCRUB_CMD_START:
        s->scrub_vals.base = s->data.ras_pb.in_addr_base;
        s->scrub_vals.size = s->data.ras_pb.in_addr_size;
#if defined(CONFIG_ACPI_RAS2_FT)
        s->scrub_vals.flags |= 1;
	scrub_rate = (s->data.ras_pb.in_scrub_params >> 8) & 0xFF;
	if ((scrub_rate < s->scrub_vals.min_scrub_rate) ||
	    (scrub_rate > s->scrub_vals.max_scrub_rate)) {
		s->data.set_ras_cap_stat = RASF_RAS_CAP_STAT_INVALID_DATA;
		return;
	}
	s->scrub_vals.cur_scrub_rate = scrub_rate;
	s->scrub_vals.en_background_patrol_scrub = s->data.ras_pb.in_scrub_params & 0x01;
#else
        /* Odd we have an input parameter that says if is already running */
        s->scrub_vals.flags = s->data.ras_pb.in_speed | 1;
#endif
        s->data.ras_pb.out_addr_base = s->data.ras_pb.in_addr_base;
        s->data.ras_pb.out_addr_size = s->data.ras_pb.in_addr_size;
        s->data.set_ras_cap_stat = RASF_RAS_CAP_STAT_SUCCESS;
        return;
    case RASF_PATROL_SCRUB_CMD_STOP:
        /* Set the running flag to off */
#if defined(CONFIG_ACPI_RAS2_FT)
        s->scrub_vals.flags &= ~0x1;
#else
        s->scrub_vals.flags = s->data.ras_pb.in_speed & ~0x1;
#endif
        s->data.set_ras_cap_stat = RASF_RAS_CAP_STAT_SUCCESS;
        return;
    default:
        s->data.set_ras_cap_stat = RASF_RAS_CAP_STAT_INVALID_DATA;
        return;
    }
}

static void pcc_write_reg(void *opaque, hwaddr offset, uint64_t value,
                          unsigned size)
{
    ACPIRASFState *s = ACPI_RASF_DEVICE(opaque);
    uint8_t *data = (uint8_t *)&s->data;

    if (offset + size <= sizeof(s->data)) {
        switch (size) {
        case 1:
            *((uint8_t *)&data[offset]) = value;
            return;
        case 2:
            *((uint16_t *)&data[offset]) = value;
            return;
        case 4:
            *((uint32_t *)&data[offset]) = value;
            return;
        case 8:
            *((uint64_t *)&data[offset]) = value;
            return;
        default:
            return;
        }

    }

    if (offset == RASF_PCC_DOORBELL_OFFSET) {
        rasf_doorbell(s);
        s->data.status = 1;
        qemu_irq_pulse(s->irq);
        return;
    }
    if (offset == RASF_PCC_INT_ACK_OFFSET) {
        /* Edge interrupt so nothing to do */
        return;
    }
    return;
}

static const MemoryRegionOps pcc_ops = {
    .read = pcc_read_reg,
    .write = pcc_write_reg,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid = {
        .min_access_size = 1,
        .max_access_size = 8,
        .unaligned = false,
    },
    .impl = {
        .min_access_size = 1,
        .max_access_size = 8,
    },
};

static void rasf_realize(DeviceState *dev, Error **errp)
{
    ACPIRASFState *s = ACPI_RASF_DEVICE(dev);

    /* Set the PCC RASF Communication channels to have some valid data */
    s->data = (PccRasfData) {
         /* Not clear who should write sig */
#if defined(CONFIG_ACPI_RAS2_FT)
        .sig = ('R' << 24) | ('A' << 16) | ('S' << 8) | '2',
        .ras_caps[0] = RASF_RAS_CAPS_PATROL_SCRUB,
#else
        .sig = ('R' << 24) | ('A' << 16) | ('S' << 8) | 'F',
        .ras_caps[0] = RASF_RAS_CAPS_SCRUB | RASF_RAS_CAPS_SCRUB_EXP_TO_SW,
#endif
        .status = 0x1,
        .num_param_blocks = 1,
        .ras_pb.length = 43,
    };

    /* Set scrubbing defaults */
    s->scrub_vals.base = 0x100000;
    s->scrub_vals.size = 0x200000;

#if defined(CONFIG_ACPI_RAS2_FT)
    s->scrub_vals.flags = 0;
    s->scrub_vals.min_scrub_rate = 1;
    s->scrub_vals.max_scrub_rate = 24;
    s->scrub_vals.cur_scrub_rate = 10;
#else
    s->scrub_vals.flags = (7 << 1) | 1;
#endif

    sysbus_init_irq(SYS_BUS_DEVICE(dev), &s->irq);
    memory_region_init_io(&s->mr, OBJECT(s), &pcc_ops, s, "pcc_chan", 0x1000);
    sysbus_init_mmio(SYS_BUS_DEVICE(dev), &s->mr);
}

static void rasf_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    dc->realize = rasf_realize;
}

static const TypeInfo rasf_info = {
    .name = TYPE_ACPI_RASF,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(ACPIRASFState),
    .class_init = rasf_class_init,
};

static void rasf_register_types(void)
{
    type_register_static(&rasf_info);
}
type_init(rasf_register_types);
