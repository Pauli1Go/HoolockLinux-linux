/*
 * Mobile NVMe support for Hx SoC based systems
 *
 * Copyright (C) 2020 Corellium LLC
 */

#include <linux/err.h>
#include <linux/iommu.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/delay.h>
#include <linux/io.h>
#include <linux/slab.h>
#include <linux/sizes.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/nvme.h>
#include <linux/pci.h>

#include "nvme.h"

#define REG_INIT                0x1800
#define  REG_INIT_REGULAR       0
#define  REG_INIT_BOOTLOADER    1
#define  REG_INIT_RESTRICTED    3
#define REG_SCRATCH_SIZE_REQ    0x1808
#define REG_SCRATCH_ALIGN_REQ   0x180c
#define REG_SCRATCH_BASE_LO     0x1810
#define REG_SCRATCH_BASE_HI     0x1814
#define REG_SCRATCH_SIZE        0x1818
#define REG_CORE_MASK           0x1824
#define REG_LOG_SIZE            0x1828
#define REG_BOOT_STATE          0x1b18
#define  REG_BOOT_STATE_MAGIC   0xbfbfbfbfu

#define MAX_REQS                36
#define MAX_REQ_PAGES           256
#define NVMMU_PAGE_SIZE         4096

/* in pci/controller/pcie-hx.c */
u64 pcie_hx_map_nvmmu(struct device *dev, unsigned tag, unsigned npages, u64 *page);

struct nvme_hx_quirk_data {
    u64 scratch_base_dma;
    u32 scratch_size;
    struct nvme_hx_quirk_req {
        struct nvme_hx_quirk_data *qd;
        u64 page[MAX_REQ_PAGES];
        unsigned npages;
    } req[MAX_REQS];
    DECLARE_BITMAP(used_req, MAX_REQS);
    unsigned last_req;
    spinlock_t used_req_lock;
};

static bool nvme_hx_dt_bool(const char *name)
{
    struct device_node *node;
    bool ret;

    node = of_find_compatible_node(NULL, NULL, "hx,pcie-h9p");
    if(!node)
        return false;

    ret = of_property_read_bool(node, name);
    of_node_put(node);
    return ret;
}

static int nvme_hx_check_boot_state(struct nvme_ctrl *ctrl)
{
    u32 val;
    ctrl->ops->reg_read32(ctrl, REG_BOOT_STATE, &val);
    return (val == REG_BOOT_STATE_MAGIC);
}

#if 0
void nvme_dump_regs(struct nvme_ctrl *ctrl)
{
    unsigned i, j;
    u32 data[8];

    for(i=0; i<0x200; i+=0x20) {
        for(j=0; j<8; j++)
            pci_read_config_dword(to_pci_dev(ctrl->dev), i + j * 4, &data[j]);
        pr_err("cfg-%03x  %08x %08x %08x %08x  %08x %08x %08x %08x\n", i,
            data[0], data[1], data[2], data[3], data[4], data[5], data[6], data[7]);
    }

    for(i=0; i<0x40; i+=0x20) {
        for(j=0; j<8; j++)
            ctrl->ops->reg_read32(ctrl, i + j * 4, &data[j]);
        pr_err("bar-%03x  %08x %08x %08x %08x  %08x %08x %08x %08x\n", i,
            data[0], data[1], data[2], data[3], data[4], data[5], data[6], data[7]);
    }

    for(i=0x1800; i<0x2000; i+=0x20) {
        for(j=0; j<8; j++)
            ctrl->ops->reg_read32(ctrl, i + j * 4, &data[j]);
        if(i < 0x1840 || (i >= 0x1900 && i <= 0x1940))
            pr_err("ex-%04x  %08x %08x %08x %08x  %08x %08x %08x %08x\n", i,
                data[0], data[1], data[2], data[3], data[4], data[5], data[6], data[7]);
    }
mdelay(1000);
}

static void nvme_hx_clear_scratch(struct device *dev, u64 base, u32 size)
{
    void *buf;

    while(size >= PAGE_SIZE) {
        buf = memremap(base, size, MEMREMAP_WB);
        if(!buf) {
            dev_err(dev, "failed to map NVMe scratch memory at 0x%llx for clearing.\n", base);
            return;
        }
        memset(buf, 0, PAGE_SIZE);
        memunmap(buf);
        base += PAGE_SIZE;
        size -= PAGE_SIZE;
    }
}
#endif

int nvme_hx_preinit(struct nvme_ctrl *ctrl, struct device *dev)
{
    u32 core_mask, log_size_reg, scratch_size, scratch_align, csts, scratch_iova;
    const u32 *of_addr = NULL;
    u64 log_size, scratch_base = 0, of_size = 0, scratch_base_dma;
    struct device_node *node;
    unsigned timeout = 2000, idx;
    struct nvme_hx_quirk_data *qd;
    int has_iova;

    dev_dbg(dev, "HX NVMe preinit start\n");

    pci_write_config_byte(to_pci_dev(ctrl->dev), 0x00D, 0x40);          /* latency timer */
    pci_write_config_dword(to_pci_dev(ctrl->dev), 0x17C, 0x10081008);   /* max snoop / no-snoop latency */
    pci_write_config_dword(to_pci_dev(ctrl->dev), 0x18C, 0x00000000);   /* L1 PM substates control 2 */
    pci_write_config_dword(to_pci_dev(ctrl->dev), 0x188, 0x40550000);   /* L1 PM substates control 1 */

    if(nvme_hx_check_boot_state(ctrl))
        dev_warn(dev, "NVMe controller may need reset.\n");

    ctrl->ops->reg_read32(ctrl, REG_CORE_MASK, &core_mask);
    if(core_mask == -1u) {
        dev_warn(dev, "invalid core mask, using default.\n");
        core_mask = 0;
    }

    ctrl->ops->reg_read32(ctrl, REG_LOG_SIZE, &log_size_reg);
    if(log_size_reg == -1u) {
        dev_warn(dev, "invalid log size, using default.\n");
        log_size_reg = 0;
    }
    log_size = ((u64)log_size_reg + 1) << 12;

    ctrl->ops->reg_read32(ctrl, REG_SCRATCH_SIZE_REQ, &scratch_size);
    ctrl->ops->reg_read32(ctrl, REG_SCRATCH_ALIGN_REQ, &scratch_align);

    ctrl->ops->reg_read32(ctrl, NVME_REG_CSTS, &csts);
    ctrl->ops->reg_write32(ctrl, NVME_REG_CC, 0);
    while(timeout) {
        ctrl->ops->reg_read32(ctrl, NVME_REG_CSTS, &csts);
        if(!(csts & (NVME_CSTS_RDY | NVME_CSTS_CFS)))
            break;
        timeout --;
        if(!timeout)
            dev_err(dev, "stopping device timed out, CSTS: 0x%08x\n", csts);
        usleep_range(1000, 2000);
    }

    node = of_find_compatible_node(NULL, NULL, "hx,nvme-scratch");
    if(node)
        of_addr = of_get_address(node, 0, &of_size, NULL);
    if(of_addr) {
        if(of_size < scratch_size) {
            dev_err(dev, "device tree reservation too small: 0x%llx, required: 0x%x.\n", of_size, scratch_size);
            of_size = 0;
        } else {
            scratch_size = of_size;
            scratch_base = of_translate_address(node, of_addr);
            if(scratch_base & (scratch_align - 1ul)) {
                dev_err(dev, "device tree reservation misaligned: 0x%llx, required: 0x%x.\n", scratch_base, scratch_align);
                scratch_base = 0;
                of_size = 0;
            }
        }
    } else
        dev_err(dev, "missing hx,nvme-scratch reservation in device tree.\n");

    if(!of_size) {
        dev_err(dev, "Hx NVMe device: scratch 0x%x@---[0x%x], log 0x%x:0x%llx\n", scratch_size, scratch_align, core_mask, log_size);
        return -ENOSPC;
    }

    has_iova = !of_property_read_u32(node, "iova", &scratch_iova);
    if(!has_iova) {
        scratch_base_dma = dma_map_resource(dev, scratch_base, scratch_size, DMA_BIDIRECTIONAL, 0);
        if(scratch_base_dma == DMA_MAPPING_ERROR) {
            dev_err(dev, "failed to map DMA region.\n");
            dev_err(dev, "Hx NVMe device: scratch 0x%x@0x%llx|---[0x%x], log 0x%x:0x%llx\n", scratch_size, scratch_base, scratch_align, core_mask, log_size);
            return -ENOSPC;
        }
    } else
        scratch_base_dma = scratch_iova;

    dev_dbg(dev, "Hx NVMe device: scratch 0x%x@0x%llx|0x%llx[0x%x], log 0x%x:0x%llx\n", scratch_size, scratch_base, scratch_base_dma, scratch_align, core_mask, log_size);

    dev_dbg(dev, "HX NVMe preinit: allocating quirk data\n");
    qd = devm_kzalloc(dev, sizeof(*qd), GFP_KERNEL);
    if(!qd) {
        dev_err(dev, "out of memory for Hx NVMe quirk data\n");
        return -ENOMEM;
    }
    qd->scratch_base_dma = scratch_base_dma;
    qd->scratch_size = scratch_size;
    for(idx=0; idx<MAX_REQS; idx++)
        qd->req[idx].qd = qd;
    ctrl->quirk_data = qd;

    spin_lock_init(&qd->used_req_lock);

    dev_dbg(dev, "HX NVMe preinit complete\n");
    if(nvme_hx_dt_bool("hx,stop-after-nvme-hx-preinit")) {
        dev_warn(dev, "HX NVMe probe stopped after HX preinit by device tree.\n");
        return -ECANCELED;
    }

    return 0;
}

int nvme_hx_preenable(struct nvme_ctrl *ctrl, struct device *dev)
{
    struct nvme_hx_quirk_data *qd = ctrl->quirk_data;

    dev_dbg(dev, "HX NVMe preenable start\n");
    if(!qd) {
        dev_err(dev, "Hx NVMe quirk data missing\n");
        return -EINVAL;
    }

    dev_dbg(dev, "HX NVMe preenable: before REG_INIT\n");
    ctrl->ops->reg_write32(ctrl, REG_INIT, REG_INIT_REGULAR);
    dev_dbg(dev, "HX NVMe preenable: before scratch base lo=0x%llx\n", qd->scratch_base_dma);
    ctrl->ops->reg_write32(ctrl, REG_SCRATCH_BASE_LO, qd->scratch_base_dma & 0xFFFFFFFFul);
    dev_dbg(dev, "HX NVMe preenable: before scratch base hi=0x%llx\n", qd->scratch_base_dma >> 32);
    ctrl->ops->reg_write32(ctrl, REG_SCRATCH_BASE_HI, qd->scratch_base_dma >> 32);
    dev_dbg(dev, "HX NVMe preenable: before scratch size=0x%x\n", qd->scratch_size);
    ctrl->ops->reg_write32(ctrl, REG_SCRATCH_SIZE, qd->scratch_size);
    dev_dbg(dev, "HX NVMe preenable complete\n");

    return 0;
}

u32 nvme_hx_max_req_size(struct nvme_ctrl *ctrl)
{
    return MAX_REQ_PAGES * (NVMMU_PAGE_SIZE / 512);
}

u32 nvme_hx_max_queue_depth(struct nvme_ctrl *ctrl)
{
    return MAX_REQS;
}

int nvme_hx_alloc_req(struct nvme_ctrl *ctrl, void **handle, unsigned *tag, u64 **pages)
{
    struct nvme_hx_quirk_data *qd = ctrl->quirk_data;
    struct nvme_hx_quirk_req *qr;
    unsigned long flags;
    unsigned idx;

    if(!qd)
        return -EOPNOTSUPP;

    spin_lock_irqsave(&qd->used_req_lock, flags);
    idx = find_next_zero_bit(qd->used_req, MAX_REQS, (qd->last_req + 1) % MAX_REQS);
    if(idx >= MAX_REQS)
        idx = find_first_zero_bit(qd->used_req, MAX_REQS);
    if(idx >= MAX_REQS) {
        spin_unlock_irqrestore(&qd->used_req_lock, flags);
        dev_warn_ratelimited(ctrl->dev,
            "HX NVMe flatdma request slots exhausted, retrying later\n");
        return -ENOMEM;
    }
    qd->last_req = idx;
    __set_bit(idx, qd->used_req);
    spin_unlock_irqrestore(&qd->used_req_lock, flags);

    qr = &qd->req[idx];
    qr->npages = 0;
    memset(qr->page, 0, sizeof(qr->page));

    *handle = qr;
    *tag = idx;
    *pages = qr->page;
    dev_dbg(ctrl->dev, "HX NVMe flatdma alloc: tag=%u\n", idx);
    return 0;
}

u64 nvme_hx_map_req(struct nvme_ctrl *ctrl, struct device *dev, void *handle, unsigned npages)
{
    struct nvme_hx_quirk_data *qd = ctrl->quirk_data;
    struct nvme_hx_quirk_req *qr = handle;
    unsigned idx;
    u64 iova;

    if(!qd || !qr)
        return 0;

    idx = qr - qd->req;
    if(idx >= MAX_REQS)
        return 0;

    qr->npages = npages;
    iova = pcie_hx_map_nvmmu(dev, idx, npages, qr->page);
    dev_dbg(ctrl->dev, "HX NVMe flatdma map: tag=%u npages=%u iova=0x%llx\n",
            idx, npages, iova);
    return iova;
}

void nvme_hx_free_req(struct nvme_ctrl *ctrl, struct device *dev, void *handle)
{
    struct nvme_hx_quirk_data *qd = ctrl->quirk_data;
    struct nvme_hx_quirk_req *qr = handle;
    unsigned long flags;
    unsigned idx;

    if(!qd || !qr)
        return;

    idx = qr - qd->req;
    if(idx >= MAX_REQS)
        return;

    pcie_hx_map_nvmmu(dev, idx, 0, NULL);
    spin_lock_irqsave(&qd->used_req_lock, flags);
    __clear_bit(idx, qd->used_req);
    spin_unlock_irqrestore(&qd->used_req_lock, flags);
    dev_dbg(ctrl->dev, "HX NVMe flatdma free: tag=%u\n", idx);
}

blk_status_t nvme_hx_map_data(struct nvme_ctrl *ctrl, struct request *req, struct nvme_command *cmnd)
{
    dev_warn_once(ctrl->dev,
        "Hx NVMe data mapping is not ported to the Hoolock 7.1 PCI NVMe path yet\n");
    return BLK_STS_NOTSUPP;
}

blk_status_t nvme_hx_unmap_data(struct nvme_ctrl *ctrl, struct request *req)
{
    return BLK_STS_NOTSUPP;
}
