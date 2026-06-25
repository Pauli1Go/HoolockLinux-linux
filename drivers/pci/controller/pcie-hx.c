#include <linux/delay.h>
#include <linux/clk.h>
#include <linux/msi.h>
#include <linux/pci-ecam.h>
#include <linux/irqdomain.h>
#include <linux/irqchip/chained_irq.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_pci.h>
#include <linux/gpio/consumer.h>
#include <linux/pm_runtime.h>
#include <linux/pinctrl/consumer.h>
#include <asm/io.h>

#include "pci-host-common.h"

#define NUM_MSI                 32
#define NUM_PORT                4

#define REG_PHY0_PORTSTAT(i)   (0x0100 + (i) * 0x0080)
#define REG_PHY1_PORTMASK       0x000C
#define REG_PORT_LTSSMCTL       0x0080
#define REG_PORT_IRQSTAT        0x0100
#define REG_PORT_IRQMASK        0x0104
#define REG_PORT_MSIVECBASE     0x0128
#define REG_PORT_ENABLE         0x0140
#define REG_PORT_LINKSTS        0x0208

#define REG_NVMMU_TCB_CTRL      0x0004
#define REG_NVMMU_TCB_BASE_LO   0x0008
#define REG_NVMMU_TCB_BASE_HI   0x000C
#define REG_NVMMU_TCB_TABLE_LO  0x0010
#define REG_NVMMU_TCB_TABLE_HI  0x0014
#define REG_NVMMU_SART_CTRL     0x0020
#define REG_NVMMU_SART_VA_BASE  0x0024
#define REG_NVMMU_SART_VA_END   0x0028
#define REG_NVMMU_SART_PA_BASE  0x002C

#define MSI_PER_PORT           (NUM_MSI / NUM_PORT)

#define HX_PMGR_PCIE_CORE       0x80318
#define HX_PMGR_PCIE_AUX        0x80320
#define HX_PMGR_PCIE_REF        0x80148
#define HX_PMGR_AUTO_ENABLE     0x10000000u

#define MAX_SGL_TAG             36
#define MAX_SGL_SIZE            256

#define TCB_PERM_WRITE          0x200
#define TCB_PERM_READ           0x100

u64 pcie_hx_map_nvmmu(struct device *dev, unsigned tag, unsigned npages, u64 *page);

struct pcie_hx {
    struct platform_device *pdev;
    void __iomem *base_config;
    void __iomem *base_phy[3];
    void __iomem *base_port[NUM_PORT];
    void __iomem *base_pcieclk_postup;
    void __iomem *base_pmgr;
    struct clk *clk[3];
    struct gpio_desc *perstn[NUM_PORT];
    struct gpio_desc *clkreqn[NUM_PORT];
    struct gpio_desc *debugio;
    struct pinctrl *pctrl;
    struct gpio_descs *devpwrio;
    struct {
        int num;
        u32 *seq;
    } devpwron[NUM_PORT];

    struct pci_host_bridge *bridge;
    struct pci_config_window *cfgwin;
    struct list_head cfgres;

    struct pcie_hx_sart {
        struct pcie_hx *pcie;
        void __iomem *base;

        uint64_t pa_base;
        uint32_t va_base;
        uint32_t size;

        size_t tcb_size, tcb_table_size, tcb_sgl_size;
        dma_addr_t tcb_dma, tcb_table_dma, tcb_sgl_dma;
        void *tcb, *tcb_table, *tcb_sgl;
    } sart[NUM_PORT];

    DECLARE_BITMAP(used_msi[NUM_PORT], MSI_PER_PORT);
    u64 msi_doorbell;
    spinlock_t used_msi_lock;
    struct irq_domain *irq_dom;
    struct pcie_hx_msi {
        struct pcie_hx *pcie;
        int virq;
    } msi[NUM_MSI];
    struct pcie_hx_port {
        struct pcie_hx *pcie;
    } port[NUM_MSI];
};

struct pcie_hx_tunable {
    uint32_t offset;
    uint32_t size;
    uint64_t mask;
    uint64_t data;
};

#include "pcie-hx-tunables.h"

static inline void rmwl(volatile void __iomem *addr, u32 clr, u32 set)
{
    writel((readl(addr) & ~clr) | set, addr);
}

static inline void rmww(volatile void __iomem *addr, u16 clr, u16 set)
{
    writew((readw(addr) & ~clr) | set, addr);
}

static inline void writerl(u32 val, volatile void __iomem *addr)
{
    writel(val, addr);
    readl(addr);
}

static inline u64 readsz(volatile void __iomem *addr, unsigned sz)
{
    switch(sz) {
    case 1: return readb(addr);
    case 2: return readw(addr);
    case 4: return readl(addr);
    case 8: return readq(addr);
    }
    return 0;
}

static inline void writesz(u64 val, volatile void __iomem *addr, unsigned sz)
{
    switch(sz) {
    case 1: writeb(val, addr); break;
    case 2: writew(val, addr); break;
    case 4: writel(val, addr); break;
    case 8: writeq(val, addr); break;
    }
}

static void pcie_hx_msi_isr(struct irq_desc *desc)
{
    struct irq_chip *chip = irq_desc_get_chip(desc);
    struct pcie_hx_msi *pciemsi = irq_desc_get_handler_data(desc);
    struct pcie_hx *pcie = pciemsi->pcie;
    unsigned idx = pciemsi - pcie->msi, virq;

    chained_irq_enter(chip, desc);
    spin_lock(&pcie->used_msi_lock);

    virq = irq_find_mapping(pcie->irq_dom, idx);
    generic_handle_irq(virq);

    spin_unlock(&pcie->used_msi_lock);
    chained_irq_exit(chip, desc);
}

static void pcie_hx_compose_msi_msg(struct irq_data *d, struct msi_msg *msg)
{
    struct pcie_hx *pcie = d->chip_data;
    pr_warn("pcie-hx MSI compose: irq=%u hwirq=%lu chip_data=%p\n",
        d->irq, d->hwirq, pcie);
    if(!pcie) {
        memset(msg, 0, sizeof(*msg));
        return;
    }
    msg->address_lo = lower_32_bits(pcie->msi_doorbell);
    msg->address_hi = upper_32_bits(pcie->msi_doorbell);
    msg->data = d->hwirq;
    pr_warn("pcie-hx MSI compose done: addr=%08x:%08x data=0x%x\n",
        msg->address_hi, msg->address_lo, msg->data);
}

static void pcie_hx_write_msi_msg(struct irq_data *d, struct msi_msg *msg)
{
    pr_warn("pcie-hx MSI write_msg: irq=%u hwirq=%lu chip=%s msg=%08x:%08x data=0x%x\n",
        d->irq, d->hwirq, d->chip && d->chip->name ? d->chip->name : "<none>",
        msg->address_hi, msg->address_lo, msg->data);
    pci_write_msi_msg(d->irq, msg);
    pr_warn("pcie-hx MSI write_msg done: irq=%u\n", d->irq);
}

static int pcie_hx_set_affinity(struct irq_data *d, const struct cpumask *mask, bool force)
{
    return -EINVAL;
}

static void pcie_hx_ack_irq(struct irq_data *d)
{
}

static void pcie_hx_mask_irq(struct irq_data *d)
{
    struct pcie_hx *pcie = d->chip_data;
    pr_warn("pcie-hx MSI mask: irq=%u hwirq=%lu chip_data=%p\n",
        d->irq, d->hwirq, pcie);
    if(!pcie || d->hwirq >= NUM_MSI || pcie->msi[d->hwirq].virq <= 0) {
        pr_warn("pcie-hx MSI mask skipped: hwirq=%lu parent_virq=%d\n",
            d->hwirq, pcie && d->hwirq < NUM_MSI ? pcie->msi[d->hwirq].virq : -1);
        return;
    }
    disable_irq(pcie->msi[d->hwirq].virq);
}

static void pcie_hx_unmask_irq(struct irq_data *d)
{
    struct pcie_hx *pcie = d->chip_data;
    pr_warn("pcie-hx MSI unmask: irq=%u hwirq=%lu chip_data=%p\n",
        d->irq, d->hwirq, pcie);
    if(!pcie || d->hwirq >= NUM_MSI || pcie->msi[d->hwirq].virq <= 0) {
        pr_warn("pcie-hx MSI unmask skipped: hwirq=%lu parent_virq=%d\n",
            d->hwirq, pcie && d->hwirq < NUM_MSI ? pcie->msi[d->hwirq].virq : -1);
        return;
    }
    enable_irq(pcie->msi[d->hwirq].virq);
}

static struct irq_chip pcie_hx_msi_chip = {
    .name = "MSI",
    .irq_ack = pcie_hx_ack_irq,
    .irq_mask = pcie_hx_mask_irq,
    .irq_unmask = pcie_hx_unmask_irq,
    .irq_compose_msi_msg = pcie_hx_compose_msi_msg,
    .irq_write_msi_msg = pcie_hx_write_msi_msg,
    .irq_set_affinity = pcie_hx_set_affinity,
};

static struct irq_chip pcie_hx_irq_chip = {
    .name = "MSI IRQ",
    .irq_ack = irq_chip_ack_parent,
    .irq_mask = irq_chip_mask_parent,
    .irq_unmask = irq_chip_unmask_parent,
    .irq_write_msi_msg = pcie_hx_write_msi_msg,
};

static struct msi_domain_info pcie_hx_msi_dom_info = {
    .flags = MSI_FLAG_MULTI_PCI_MSI | MSI_FLAG_PCI_MSIX | MSI_FLAG_USE_DEF_DOM_OPS | MSI_FLAG_USE_DEF_CHIP_OPS,
    .chip = &pcie_hx_irq_chip,
};

static unsigned pcie_hx_bus_to_port(struct pcie_hx *pcie, unsigned bus)
{
    unsigned port, bus0, bus1;
    u32 cfg;
    for(port=0; port<NUM_PORT; port++) {
        cfg = readl(pcie->base_config + (port << 15) + 0x18); /* read bus number word */
        bus0 = (cfg >> 8) & 255; /* secondary bus number */
        bus1 = (cfg >> 16) & 255; /* subordinate bus number */
        if(!bus0 || !bus1 || bus0 == 0xFF || bus1 == 0xFF)
            continue;
        if(bus >= bus0 && bus <= bus1)
            break;
    }
    return port;
}

static int pcie_hx_irq_domain_alloc(struct irq_domain *dom, unsigned int virq, unsigned int nr_irqs, void *args)
{
    struct pcie_hx *pcie = dom->host_data;
    msi_alloc_info_t *info = args;
    struct msi_desc *desc = info ? info->desc : NULL;
    struct pci_dev *pdev = NULL;
    unsigned long flags;
    int pos;
    unsigned bus = 0, port;

    if(desc && desc->dev && dev_is_pci(desc->dev)) {
        pdev = to_pci_dev(desc->dev);
        if(pdev->bus)
            bus = pdev->bus->number;
    }

    pr_warn("pcie-hx MSI alloc: virq=%u nr=%u desc=%p dev=%s hwirq=%llu bus=%u devfn=0x%02x\n",
        virq, nr_irqs, desc, desc && desc->dev ? dev_name(desc->dev) : "<none>",
        info ? (unsigned long long)info->hwirq : 0, bus, pdev ? pdev->devfn : 0xff);

    if(bus < 1) /* no MSIs on root ports */
        return -ENOSPC;

    if(nr_irqs > 1)
        return -ENOSPC;

    port = pcie_hx_bus_to_port(pcie, bus);
    if(port >= NUM_PORT)
        return -ENOSPC;

    spin_lock_irqsave(&pcie->used_msi_lock, flags);

    pos = find_first_zero_bit(pcie->used_msi[port], MSI_PER_PORT);
    if(pos >= MSI_PER_PORT) {
        spin_unlock_irqrestore(&pcie->used_msi_lock, flags);
        pr_warn("pcie-hx MSI alloc failed: port=%u has no free slots\n", port);
        return -ENOSPC;
    }
    __set_bit(pos, pcie->used_msi[port]);
    spin_unlock_irqrestore(&pcie->used_msi_lock, flags);
    irq_domain_set_info(dom, virq, pos + MSI_PER_PORT * port, &pcie_hx_msi_chip, pcie, handle_edge_irq, NULL, NULL);
    pr_warn("pcie-hx MSI alloc done: port=%u slot=%d hwirq=%u\n", port, pos, pos + MSI_PER_PORT * port);

    return 0;
}

static void pcie_hx_irq_domain_free(struct irq_domain *dom, unsigned int virq, unsigned int nr_irqs)
{
    unsigned long flags;
    struct irq_data *d = irq_domain_get_irq_data(dom, virq);
    struct pcie_hx *pcie = d ? d->chip_data : NULL;
    unsigned i, port;

    pr_warn("pcie-hx MSI free: virq=%u nr=%u data=%p chip_data=%p\n",
        virq, nr_irqs, d, pcie);
    if(!d || !pcie) {
        pr_warn("pcie-hx MSI free skipped: missing irq data\n");
        return;
    }

    spin_lock_irqsave(&pcie->used_msi_lock, flags);
    for(i=0; i<nr_irqs; i++) {
        port = (d->hwirq + i) / MSI_PER_PORT;
        pr_warn("pcie-hx MSI free slot: hwirq=%lu port=%u slot=%lu\n",
            d->hwirq + i, port, d->hwirq + i - port * MSI_PER_PORT);
        __clear_bit(d->hwirq + i - port * MSI_PER_PORT, pcie->used_msi[port]);
    }
    spin_unlock_irqrestore(&pcie->used_msi_lock, flags);
}

static const struct irq_domain_ops pcie_hx_irq_dom_ops = {
    .alloc = pcie_hx_irq_domain_alloc,
    .free = pcie_hx_irq_domain_free,
};

static int pcie_hx_config_read(struct pci_bus *bus, unsigned int devfn, int where, int size, u32 *val)
{
    struct pci_config_window *cfg = bus->sysdata;
    if(bus->number == cfg->busr.start && PCI_SLOT(devfn) >= NUM_PORT)
        return -ENODEV;
    return pci_generic_config_read(bus, devfn, where, size, val);
}

static int pcie_hx_config_write(struct pci_bus *bus, unsigned int devfn, int where, int size, u32 val)
{
    struct pci_config_window *cfg = bus->sysdata;
    if(bus->number == cfg->busr.start && PCI_SLOT(devfn) >= NUM_PORT)
        return -ENODEV;
    if(where <= 0x3C && where + size > 0x3C) /* intercept writes to IRQ line */
        val |= 0xFFu << ((0x3C - where) << 3);
    return pci_generic_config_write(bus, devfn, where, size, val);
}

static struct pci_ecam_ops pcie_hx_ecam_ops = {
    .bus_shift = 20,
    .pci_ops = {
        .map_bus = pci_ecam_map_bus,
        .read = pcie_hx_config_read,
        .write = pcie_hx_config_write,
    }
};

static irqreturn_t pcie_hx_portirq_isr(int irq, void *dev_id)
{
    struct pcie_hx_port *pcieport = dev_id;
    struct pcie_hx *pcie = pcieport->pcie;
    unsigned idx = pcieport - pcie->port;
    uint32_t mask;

    mask = readl(pcie->base_port[idx] + REG_PORT_IRQSTAT);
    writel(mask, pcie->base_port[idx] + REG_PORT_IRQSTAT);
    pr_err("pcie%d: got port irq 0x%x\n", idx, mask);

    return IRQ_HANDLED;
}

static irqreturn_t pcie_hx_sartirq_isr(int irq, void *dev_id)
{
    struct pcie_hx_sart *sart = dev_id;
    struct pcie_hx *pcie = sart->pcie;
    unsigned idx = sart - pcie->sart;

    pr_err("pcie%d: got SART irq\n", idx);

    return IRQ_HANDLED;
}

/* used from nvme/hx.c */
u64 pcie_hx_map_nvmmu(struct device *dev, unsigned tag, unsigned npages, u64 *page)
{
    struct device *rdev = dev;
    struct pcie_hx *pcie;
    unsigned port = 0, i;
    u32 *tcb;
    u32 *sgl;

    if(tag >= MAX_SGL_TAG || npages >= MAX_SGL_SIZE)
        return 0;

    if(!dev->bus)
        return 0;
    while(rdev && rdev->bus == dev->bus)
        rdev = rdev->parent;
    if(!rdev || !rdev->parent)
        return 0;
    rdev = rdev->parent;

    pcie = dev_get_drvdata(rdev);
    if(!pcie->sart[port].tcb)
        return 0;

    tcb = pcie->sart[port].tcb + (tag << 7);
    tcb[0] = TCB_PERM_WRITE | TCB_PERM_READ;
    tcb[1] = npages;
    if(npages >= 1)
        tcb[2] = page[0] >> 12;
    else
        tcb[2] = 0;

    sgl = pcie->sart[port].tcb_sgl + MAX_SGL_SIZE * 4 * tag;
    for(i=0; i<npages; i++)
        sgl[i] = page[i] >> 12;
    *(u64 *)&tcb[4] = pcie->sart[port].tcb_sgl_dma + MAX_SGL_SIZE * 4 * tag;

    if(!npages)
        writel(tag, pcie->sart[port].base + REG_NVMMU_TCB_CTRL);

    return 0x40000000ul + (tag << 23);
}

static int pcie_hx_link_up(struct pcie_hx *pcie, unsigned idx)
{
    uint32_t linksts = readl(pcie->base_port[idx] + REG_PORT_LINKSTS);
    linksts = (linksts >> 8) & 0x3F;
    if(linksts >= 0x11 && linksts <= 0x14)
        return 1;
    return 0;
}

static int pcie_hx_wait_poll(struct pcie_hx *pcie, unsigned idx, volatile void __iomem *addr, uint32_t mask, uint32_t min, uint32_t max, unsigned msec, const char *msg)
{
    uint32_t val = 0;
    while(msec --) {
        val = readl(addr);
        if((val & mask) >= min && (val & mask) <= max)
            return 0;
        usleep_range(1000, 2000);
    }
    pr_err("pcie%d: %s timed out (%08x/%08x/%08x-%08x)\n", idx, msg, val, mask, min, max);
    return 1;
}

static int pcie_hx_wait_gpio(struct pcie_hx *pcie, unsigned idx, struct gpio_desc *gpiod, unsigned req, unsigned msec, const char *msg)
{
    uint32_t val = 0;
    while(msec --) {
        val = gpiod_get_raw_value(gpiod);
        if(val == req)
            return 0;
        usleep_range(1000, 2000);
    }
    pr_err("pcie%d: %s timed out (%d/%d)\n", idx, msg, val, req);
    return 1;
}

static void pcie_hx_apply_tunables(struct pcie_hx *pcie, volatile void __iomem *base, const struct pcie_hx_tunable *tuna, unsigned count)
{
    unsigned i;
    uint64_t val;

    for(i=0; i<count; i++) {
        val = readsz(base + tuna[i].offset, tuna[i].size);
        if((val & tuna[i].mask) == tuna[i].data)
            continue;
        val &= ~tuna[i].mask;
        val |= tuna[i].data;
        writesz(val, base + tuna[i].offset, tuna[i].size);
    }
}

static unsigned pcie_hx_find_pcicap(struct pcie_hx *pcie, unsigned busdevfn, unsigned type)
{
    unsigned ptr, next;

    ptr = readl(pcie->base_config + (busdevfn << 12) + 0x034) & 0xFF;
    while(ptr) {
        next = readl(pcie->base_config + (busdevfn << 12) + ptr);
        if((next & 0xFF) == type)
            return ptr;
        ptr = (next >> 8) & 0xFF;
    }
    return 0;
}

static void pcie_hx_setup_sart(struct pcie_hx *pcie, unsigned idx)
{
    struct pcie_hx_sart *sart = &pcie->sart[idx];

    writerl(sart->tcb_dma, sart->base + REG_NVMMU_TCB_BASE_LO);
    writerl(sart->tcb_dma >> 32, sart->base + REG_NVMMU_TCB_BASE_HI);
    writerl(sart->tcb_table_dma, sart->base + REG_NVMMU_TCB_TABLE_LO);
    writerl(sart->tcb_table_dma >> 32, sart->base + REG_NVMMU_TCB_TABLE_HI);
    writerl(0x10000, sart->base + REG_NVMMU_TCB_CTRL);
    pcie_hx_wait_poll(pcie, idx, sart->base + REG_NVMMU_TCB_CTRL, 0x10, 0, 0, 250, "TCB start");

    writerl(sart->va_base - 0x80000000u, sart->base + REG_NVMMU_SART_VA_BASE);
    writerl(sart->va_base + ((sart->size + 0xFFFFFu) & -0x100000u) - 0x80100000u, sart->base + REG_NVMMU_SART_VA_END);
    writerl(sart->pa_base >> 20, sart->base + REG_NVMMU_SART_PA_BASE);
    writerl(1, sart->base + REG_NVMMU_SART_CTRL);
}

static void pcie_hx_port_pwron(struct pcie_hx *pcie, unsigned idx)
{
    int i;
    u32 *seq = pcie->devpwron[idx].seq;

    for(i=0; i<pcie->devpwron[idx].num; i++) {
        usleep_range(2500, 5000);
        if(seq[1] < 2)
            gpiod_direction_output(pcie->devpwrio->desc[seq[0]], seq[1]);
        else if(seq[1] == 2)
            gpiod_direction_input(pcie->devpwrio->desc[seq[0]]);
        usleep_range(2500, 5000);
        seq += 2;
    }
}

static bool pcie_hx_setup_port(struct pcie_hx *pcie, unsigned idx)
{
    struct device *dev = &pcie->pdev->dev;
    struct device_node *node = dev->of_node;
    unsigned ptr;

    if(pcie_hx_link_up(pcie, idx))
        return false;

    gpiod_direction_output(pcie->perstn[idx], 0);

    if(idx == 0 && of_property_read_bool(node, "hx,stop-after-port0-perst-low")) {
        dev_warn(dev, "HX PCIe probe stopped after PERST#0 low by device tree.\n");
        return true;
    }

    rmwl(pcie->base_phy[0] + 0x134 + 0x80 * idx, 1, 0);
    rmwl(pcie->base_phy[0] + 0x124 + 0x80 * idx, 0, 1);
    pcie_hx_wait_poll(pcie, idx, pcie->base_phy[0] + 0x28, 0x10, 0x10, 0x10, 250, "port init");

    if(idx == 0 && of_property_read_bool(node, "hx,stop-after-port0-init-poll")) {
        dev_warn(dev, "HX PCIe probe stopped after port0 init poll by device tree.\n");
        return true;
    }

    usleep_range(250, 1000);
    rmwl(pcie->base_phy[0] + 0x100 + 0x80 * idx, 0, 1);

    if(idx == 0 && of_property_read_bool(node, "hx,stop-after-port0-phy100-enable")) {
        dev_warn(dev, "HX PCIe probe stopped after port0 PHY 0x100 enable by device tree.\n");
        return true;
    }

    rmwl(pcie->base_phy[0] + 0x100 + 0x80 * idx, 0x100, 0);

    if(idx == 0 && of_property_read_bool(node, "hx,stop-after-port0-phy100-clear")) {
        dev_warn(dev, "HX PCIe probe stopped after port0 PHY 0x100 clear by device tree.\n");
        return true;
    }

    usleep_range(500, 1000);

    rmwl(pcie->base_phy[0] + 0x134 + 0x80 * idx, 0, 1);

    if(idx == 0 && of_property_read_bool(node, "hx,stop-after-port0-phy134-enable")) {
        dev_warn(dev, "HX PCIe probe stopped after port0 PHY 0x134 enable by device tree.\n");
        return true;
    }

    if(idx == 0 && of_property_read_bool(node, "hx,skip-port0-phy4020-write"))
        dev_warn(dev, "HX PCIe skipped port0 PHY 0x4020 write by device tree.\n");
    else {
        u32 phy4020_value = 3;

        if(idx == 0)
            of_property_read_u32(node, "hx,port0-phy4020-value", &phy4020_value);

        writel(phy4020_value, pcie->base_phy[0] + 0x4020 + 0x40 * idx);
    }

    if(idx == 0 && of_property_read_bool(node, "hx,stop-after-port0-phy4020-write")) {
        dev_warn(dev, "HX PCIe probe stopped after port0 PHY 0x4020 write by device tree.\n");
        return true;
    }

    rmwl(pcie->base_phy[0] + 0x124 + 0x80 * idx, 0x100, 0);

    if(idx == 0 && of_property_read_bool(node, "hx,stop-after-port0-phy124-clear")) {
        dev_warn(dev, "HX PCIe probe stopped after port0 PHY 0x124 clear by device tree.\n");
        return true;
    }

    if(idx == 0 && of_property_read_bool(node, "hx,stop-after-port0-early-phy")) {
        dev_warn(dev, "HX PCIe probe stopped after port0 early PHY sequence by device tree.\n");
        return true;
    }

    pcie_hx_port_pwron(pcie, idx);

    if(idx == 0 && of_property_read_bool(node, "hx,stop-after-port0-devpwr")) {
        dev_warn(dev, "HX PCIe probe stopped after port0 device power sequence by device tree.\n");
        return true;
    }

    if(pcie->sart[idx].base && !(idx == 0 && of_property_read_bool(node, "hx,skip-port0-sart-reinit")))
        pcie_hx_setup_sart(pcie, idx);
    else if(idx == 0 && of_property_read_bool(node, "hx,skip-port0-sart-reinit"))
        dev_warn(dev, "HX PCIe skipped port0 SART reinit by device tree.\n");

    if(idx == 0 && of_property_read_bool(node, "hx,stop-after-port0-sart-reinit")) {
        dev_warn(dev, "HX PCIe probe stopped after port0 SART reinit by device tree.\n");
        return true;
    }

    if(idx == 0 && of_property_read_bool(node, "hx,skip-port0-prelink-config")) {
        dev_warn(dev, "HX PCIe skipped port0 pre-link config access by device tree.\n");
        ptr = 0;
    } else {
        ptr = pcie_hx_find_pcicap(pcie, idx << 3, 0x10);

        if(idx == 0 && of_property_read_bool(node, "hx,stop-after-port0-cap-read")) {
            dev_warn(dev, "HX PCIe probe stopped after port0 PCIe capability read by device tree.\n");
            return true;
        }

        if(ptr)
            rmww(pcie->base_config + (idx << 15) + (ptr + 0x30), 15, idx ? 1 : 3); /* maximum speed: 2.5, 5.0, 8.0 GT/s */
        pcie_hx_apply_tunables(pcie, pcie->base_config + (idx << 15), pcie_hx_config_tunables, ARRAY_SIZE(pcie_hx_config_tunables));
        pcie_hx_apply_tunables(pcie, pcie->base_port[idx], pcie_hx_port_tunables, ARRAY_SIZE(pcie_hx_port_tunables));
        rmwl(pcie->base_config + (idx << 15) + 0x8e0, 0, 1);
    }

    if(idx == 0 && of_property_read_bool(node, "hx,stop-after-port0-prelink-config")) {
        dev_warn(dev, "HX PCIe probe stopped after port0 pre-link config stage by device tree.\n");
        return true;
    }

    writel(0xff002fff, pcie->base_port[idx] + REG_PORT_IRQMASK);

    if(idx == 0 && of_property_read_bool(node, "hx,stop-after-port0-irqmask-write")) {
        dev_warn(dev, "HX PCIe probe stopped after port0 IRQMASK write by device tree.\n");
        return true;
    }

    writel(0x00ffd000, pcie->base_port[idx] + REG_PORT_IRQSTAT);

    if(idx == 0 && of_property_read_bool(node, "hx,stop-after-port0-irq-setup")) {
        dev_warn(dev, "HX PCIe probe stopped after port0 IRQ setup by device tree.\n");
        return true;
    }

    rmwl(pcie->base_port[idx] + REG_PORT_ENABLE, 0, 0x80000000u);
    writel(0x31, pcie->base_port[idx] + 0x124);
    writel(idx * 0x10001 * MSI_PER_PORT, pcie->base_port[idx] + REG_PORT_MSIVECBASE);

    if(idx == 0 && of_property_read_bool(node, "hx,stop-after-port0-enable-setup")) {
        dev_warn(dev, "HX PCIe probe stopped after port0 enable setup by device tree.\n");
        return true;
    }

    usleep_range(250, 1000);
    pcie_hx_wait_gpio(pcie, idx, pcie->clkreqn[idx], 0, 250, "clkreq# wait");

    if(idx == 0 && of_property_read_bool(node, "hx,stop-after-port0-clkreq-wait")) {
        dev_warn(dev, "HX PCIe probe stopped after port0 CLKREQ wait by device tree.\n");
        return true;
    }

    gpiod_direction_output(pcie->perstn[idx], 1);
    usleep_range(250, 1000);

    if(idx == 0 && of_property_read_bool(node, "hx,stop-after-port0-perst-high")) {
        dev_warn(dev, "HX PCIe probe stopped after PERST#0 high by device tree.\n");
        return true;
    }

    pcie_hx_wait_poll(pcie, idx, pcie->base_phy[1] + REG_PHY1_PORTMASK, 1u << idx, 1u << idx, 1u << idx, 250, "port up");

    if(idx == 0 && of_property_read_bool(node, "hx,stop-after-port0-port-up-poll")) {
        dev_warn(dev, "HX PCIe probe stopped after port0 port-up poll by device tree.\n");
        return true;
    }

    rmwl(pcie->base_phy[2] + 0x180, 0, 0x4000);
    rmwl(pcie->base_phy[2] + 0x184, 0, 0x4000);
    rmwl(pcie->base_phy[2] + 0x90, 0xFFF, 100);
    rmwl(pcie->base_phy[2] + 0x98, 0xFFF, 25);

    if(idx == 0 && of_property_read_bool(node, "hx,stop-after-port0-phy2-common")) {
        dev_warn(dev, "HX PCIe probe stopped after port0 PHY2 common tuning by device tree.\n");
        return true;
    }

    rmwl(pcie->base_phy[2] + 0x10088 + 0x800 * idx, 0, 0x4000);
    writel(0, pcie->base_phy[2] + 0x10784 + 0x800 * idx);
    rmwl(pcie->base_phy[2] + 0x10004 + 0x800 * idx, 0xFFF, 0x600);
    writel(0x3105, pcie->base_phy[2] + 0x20788 + 0x800 * idx);
    rmwl(pcie->base_phy[2] + 0x207a0 + 0x800 * idx, 0xFF, 0x9F);
    rmwl(pcie->base_phy[2] + 0x207a8 + 0x800 * idx, 0xFF, 0x01);
    rmwl(pcie->base_phy[2] + 0x20400 + 0x800 * idx, 0x1F, 0x0A);
    writel(175, pcie->base_phy[2] + 0x2009c + 0x800 * idx);
    writel(175, pcie->base_phy[2] + 0x200dc + 0x800 * idx);
    writel(333, pcie->base_phy[2] + 0x200a0 + 0x800 * idx);
    writel(333, pcie->base_phy[2] + 0x200e0 + 0x800 * idx);
    writel(530, pcie->base_phy[2] + 0x200a4 + 0x800 * idx);
    writel(530, pcie->base_phy[2] + 0x200e4 + 0x800 * idx);
    writel(0, pcie->base_phy[2] + 0x20330 + 0x800 * idx);
    writel(0, pcie->base_phy[2] + 0x20340 + 0x800 * idx);
    writel(0, pcie->base_phy[2] + 0x20350 + 0x800 * idx);

    if(idx == 0 && of_property_read_bool(node, "hx,stop-after-port0-phy2-port")) {
        dev_warn(dev, "HX PCIe probe stopped after port0 PHY2 per-port tuning by device tree.\n");
        return true;
    }

    writel(0xff002f0f, pcie->base_port[idx] + REG_PORT_IRQMASK);

    if(idx == 0 && of_property_read_bool(node, "hx,stop-after-port0-final-irqmask-write")) {
        dev_warn(dev, "HX PCIe probe stopped after port0 final IRQMASK write by device tree.\n");
        return true;
    }

    usleep_range(5000, 10000);

    rmwl(pcie->base_port[idx] + REG_PORT_LTSSMCTL, 0, 1);

    if(idx == 0 && of_property_read_bool(node, "hx,stop-after-port0-ltssm-start")) {
        dev_warn(dev, "HX PCIe probe stopped after port0 LTSSM start by device tree.\n");
        return true;
    }

    pcie_hx_wait_poll(pcie, idx, pcie->base_port[idx] + REG_PORT_LINKSTS, 0x3F00, 0x1100, 0x1400, 500, "link up");

    if(idx == 0 && of_property_read_bool(node, "hx,stop-after-port0-link-up-poll")) {
        dev_warn(dev, "HX PCIe probe stopped after port0 link-up poll by device tree.\n");
        return true;
    }

    return false;
}

static bool pcie_hx_setup_ports(struct pcie_hx *pcie)
{
    struct device *dev = &pcie->pdev->dev;
    struct device_node *node = dev->of_node;
    unsigned port;

    gpiod_direction_output(pcie->debugio, 0);

    writel(0x10, pcie->base_phy[0] + 0x0004);

    rmwl(pcie->base_phy[0] + 0x124, 0, 1);
    pcie_hx_wait_poll(pcie, 0, pcie->base_phy[0] + 0x28, 0x10, 0x10, 0x10, 250, "global init 1");
    pcie_hx_wait_poll(pcie, 0, pcie->base_phy[0] + 0x28, 1, 1, 1, 250, "global init 2");
    writel(1, pcie->base_phy[0] + 0x34);
    pcie_hx_apply_tunables(pcie, pcie->base_phy[0], pcie_hx_phy_0_tunables, ARRAY_SIZE(pcie_hx_phy_0_tunables));
    writel(1, pcie->base_phy[0] + 0x14); /* this enables NVMe mode on port 0 */
    usleep_range(5000, 10000);
    writel(1, pcie->base_phy[0] + 0x24);
    usleep_range(500, 1000);

    if(of_property_read_bool(node, "hx,stop-after-global-init")) {
        dev_warn(dev, "HX PCIe probe stopped after global PHY init by device tree.\n");
        return true;
    }

    for(port=0; port<NUM_PORT; port++) {
        if(pcie->devpwron[port].num < 0)
            continue;
        if(pcie_hx_setup_port(pcie, port))
            return true;
    }

    return false;
}

static const char *const pcie_hx_clock_names[] = { "core", "aux", "ref" };

static int pcie_hx_enable_raw_pmgr_gate(struct pcie_hx *pcie, const char *name, u32 offset)
{
    struct device *dev = &pcie->pdev->dev;
    unsigned max = 10000;
    u32 val;

    val = readl(pcie->base_pmgr + offset);
    dev_warn(dev, "HX PCIe raw PMGR %s before: 0x%08x\n", name, val);

    val |= 0xf;
    writel(val, pcie->base_pmgr + offset);

    while(max--) {
        val = readl(pcie->base_pmgr + offset);
        if(((val >> 4) & 0xf) == 0xf) {
            writel(val | HX_PMGR_AUTO_ENABLE, pcie->base_pmgr + offset);
            dev_warn(dev, "HX PCIe raw PMGR %s enabled: 0x%08x\n", name, readl(pcie->base_pmgr + offset));
            return 0;
        }

        cpu_relax();
    }

    dev_err(dev, "HX PCIe raw PMGR %s failed to reach active state: 0x%08x\n", name, val);
    return -ETIMEDOUT;
}

static int pcie_hx_apply_raw_pmgr_gates(struct pcie_hx *pcie)
{
    struct device *dev = &pcie->pdev->dev;
    struct device_node *node = dev->of_node;
    int ret;

    if(!of_property_read_bool(node, "hx,apply-raw-pmgr-gates"))
        return 0;

    if(!pcie->base_pmgr) {
        dev_err(dev, "missing raw PMGR MMIO range.\n");
        return -EINVAL;
    }

    ret = pcie_hx_enable_raw_pmgr_gate(pcie, "core", HX_PMGR_PCIE_CORE);
    if(ret)
        return ret;

    ret = pcie_hx_enable_raw_pmgr_gate(pcie, "aux", HX_PMGR_PCIE_AUX);
    if(ret)
        return ret;

    ret = pcie_hx_enable_raw_pmgr_gate(pcie, "ref", HX_PMGR_PCIE_REF);
    if(ret)
        return ret;

    return 0;
}

static int pcie_hx_apply_pcieclk_postup(struct pcie_hx *pcie)
{
    struct device *dev = &pcie->pdev->dev;
    struct device_node *node = dev->of_node;

    if(!of_property_read_bool(node, "hx,apply-pcieclk-postup"))
        return 0;

    if(!pcie->base_pcieclk_postup) {
        dev_err(dev, "missing pcieclk post-up MMIO range.\n");
        return -EINVAL;
    }

    writel(0x00000007, pcie->base_pcieclk_postup + 0x0000);
    writel(0x80010005, pcie->base_pcieclk_postup + 0x000c);
    writel(0x00000003, pcie->base_pcieclk_postup + 0x4104);
    writel(0x00000003, pcie->base_pcieclk_postup + 0x4100);

    dev_warn(dev, "HX PCIe applied pcieclk post-up sequence by device tree.\n");
    return 0;
}

static int pcie_hx_pci_host_probe(struct pcie_hx *pcie)
{
    struct device *dev = &pcie->pdev->dev;
    struct device_node *node = dev->of_node;
    struct pci_host_bridge *bridge = pcie->bridge;
    struct pci_bus *bus, *child;
    int ret;

    dev_warn(dev, "HX PCIe entering pci_scan_root_bus_bridge.\n");
    pci_lock_rescan_remove();
    ret = pci_scan_root_bus_bridge(bridge);
    pci_unlock_rescan_remove();
    if(ret < 0) {
        dev_err(dev, "HX PCIe root bridge scan failed: %d\n", ret);
        return ret;
    }

    bus = bridge->bus;
    dev_warn(dev, "HX PCIe pci_scan_root_bus_bridge done: bus=%p number=%u children_empty=%d.\n",
        bus, bus ? bus->number : 0, bus ? list_empty(&bus->children) : -1);

    if(of_property_read_bool(node, "hx,stop-after-pci-scan-root")) {
        dev_warn(dev, "HX PCIe probe stopped after pci_scan_root_bus_bridge by device tree.\n");
        return 0;
    }

    if(of_property_read_bool(node, "hx,stop-after-pci-root-slot0")) {
        dev_warn(dev, "HX PCIe probe stopped after root slot 0 scan by device tree.\n");
        return 0;
    }

    if(!bus) {
        dev_err(dev, "HX PCIe root bridge scan returned no bus.\n");
        return -ENODEV;
    }

    if(bridge->preserve_config)
        pci_bus_claim_resources(bus);

    dev_warn(dev, "HX PCIe assigning unassigned root bus resources.\n");
    pci_assign_unassigned_root_bus_resources(bus);
    dev_warn(dev, "HX PCIe resource assignment done.\n");

    if(of_property_read_bool(node, "hx,stop-after-pci-assign-resources")) {
        dev_warn(dev, "HX PCIe probe stopped after root-bus resource assignment by device tree.\n");
        return 0;
    }

    list_for_each_entry(child, &bus->children, node) {
        dev_warn(dev, "HX PCIe configuring child bus %02x.\n", child->number);
        pcie_bus_configure_settings(child);
    }
    dev_warn(dev, "HX PCIe child bus setting configuration done.\n");

    if(of_property_read_bool(node, "hx,stop-after-pci-configure-settings")) {
        dev_warn(dev, "HX PCIe probe stopped after child-bus setting configuration by device tree.\n");
        return 0;
    }

    dev_warn(dev, "HX PCIe adding PCI devices.\n");
    pci_lock_rescan_remove();
    pci_bus_add_devices(bus);
    pci_unlock_rescan_remove();
    dev_warn(dev, "HX PCIe pci_bus_add_devices done.\n");

    if(of_property_read_bool(node, "hx,stop-after-pci-add-devices")) {
        dev_warn(dev, "HX PCIe probe stopped after pci_bus_add_devices by device tree.\n");
        return 0;
    }

    pm_runtime_set_active(&bridge->dev);
    pm_runtime_no_callbacks(&bridge->dev);
    devm_pm_runtime_enable(&bridge->dev);

    return 0;
}

static void pcie_hx_pm_disable(void *data)
{
    struct device *dev = data;

    pm_runtime_put_sync(dev);
    pm_runtime_disable(dev);
}

static int pcie_hx_probe_sart(struct pcie_hx *pcie)
{
    struct device *dev = &pcie->pdev->dev;
    struct device_node *node = dev->of_node, *mnode;
    u32 map[NUM_PORT] = { 0 }, miova;
    const u32 *of_addr;
    u64 msize, mphys;
    unsigned port, idx;
    int ret, pirq;

    ret = of_property_read_variable_u32_array(node, "sart-map", map, NUM_PORT, NUM_PORT);
    if(ret < 0) {
        dev_warn(dev, "NVMMU/SART is disabled.\n");
        return 0;
    }

    for(port=0; port<NUM_PORT; port++) {
        if(!map[port])
            continue;
        idx = map[port] - 1;

        pcie->sart[port].base = of_iomap(node, idx + 4 + NUM_PORT);
        if(!pcie->sart[port].base) {
            dev_err(dev, "failed to map NVMMU MMIO range %d.\n", idx);
            return -EINVAL;
        }

        pirq = platform_get_irq(pcie->pdev, NUM_PORT + NUM_MSI + idx);
        if(pirq < 0) {
            dev_err(dev, "failed to map NVMMU IRQ %d.\n", idx);
            return -EINVAL;
        }
        pcie->sart[port].pcie = pcie;
        if(devm_request_irq(dev, pirq, pcie_hx_sartirq_isr, 0, dev_name(dev), &pcie->sart[port]) < 0) {
            dev_err(dev, "failed to request NVMMU IRQ %d.\n", idx);
            return -EINVAL;
        }

        mnode = of_parse_phandle(node, "sart-regions", idx);
        if(!mnode) {
            dev_err(dev, "SART region %d not specified.\n", idx);
            return -EINVAL;
        }

        of_addr = of_get_address(mnode, 0, &msize, NULL);
        if(!of_addr || msize < 0x100000) {
            dev_err(dev, "SART region %d undefined or too small.\n", idx);
            return -EINVAL;
        }
        mphys = of_translate_address(mnode, of_addr);
        if(mphys & 0xFFFFF) {
            dev_err(dev, "SART region %d misaligned.\n", idx);
            return -EINVAL;
        }

        if(of_property_read_u32(mnode, "iova", &miova)) {
            dev_err(dev, "SART region %d missing iova property.\n", idx);
            return -EINVAL;
        }
        if(miova & 0xFFFFF) {
            dev_err(dev, "SART region %d IOVA misaligned.\n", idx);
            return -EINVAL;
        }

        pcie->sart[port].pa_base = mphys;
        pcie->sart[port].va_base = miova;
        pcie->sart[port].size = msize;

        pcie->sart[port].tcb_size = round_up(MAX_SGL_TAG * 0x80, PAGE_SIZE);
        pcie->sart[port].tcb_table_size = round_up(PAGE_SIZE * 16, PAGE_SIZE);
        pcie->sart[port].tcb_sgl_size = round_up(MAX_SGL_TAG * MAX_SGL_SIZE * 0x4, PAGE_SIZE);
        pcie->sart[port].tcb = dmam_alloc_attrs(dev, pcie->sart[port].tcb_size, &pcie->sart[port].tcb_dma, GFP_KERNEL | __GFP_ZERO, DMA_ATTR_WRITE_COMBINE);
        pcie->sart[port].tcb_table = dmam_alloc_attrs(dev, pcie->sart[port].tcb_table_size, &pcie->sart[port].tcb_table_dma, GFP_KERNEL | __GFP_ZERO, DMA_ATTR_WRITE_COMBINE);
        pcie->sart[port].tcb_sgl = dmam_alloc_attrs(dev, pcie->sart[port].tcb_sgl_size, &pcie->sart[port].tcb_sgl_dma, GFP_KERNEL | __GFP_ZERO, DMA_ATTR_WRITE_COMBINE);
        if(!pcie->sart[port].tcb || !pcie->sart[port].tcb_table || !pcie->sart[port].tcb_sgl) {
            dev_err(dev, "NVMMU table allocation failed.\n");
            return -EINVAL;
        }

        dev_warn(dev, "NVMMU port %d: SART 0x%llx@0x%x->0x%llx, TCB %ld@0x%llx, table %ld@0x%llx, SGL %ld@0x%llx\n", port, msize, miova, mphys,
            pcie->sart[port].tcb_size, pcie->sart[port].tcb_dma, pcie->sart[port].tcb_table_size, pcie->sart[port].tcb_table_dma, pcie->sart[port].tcb_sgl_size, pcie->sart[port].tcb_sgl_dma);
    }

    return 0;
}

static int pcie_hx_probe(struct platform_device *pdev)
{
    struct device *dev = &pdev->dev;
    struct device_node *node = dev->of_node;
    struct fwnode_handle *fwnode = of_fwnode_handle(node);
    struct pcie_hx *pcie;
    void __iomem *mmio[4 + NUM_PORT];
    struct irq_domain *msi_dom, *irq_dom;
    int virq[NUM_MSI], pirq, ret;
    char name[32];
    unsigned i;

    pcie = devm_kzalloc(dev, sizeof(*pcie), GFP_KERNEL);
    if(!pcie)
        return -ENOMEM;

    pm_runtime_enable(dev);
    ret = pm_runtime_resume_and_get(dev);
    if(ret < 0) {
        pm_runtime_disable(dev);
        return ret;
    }
    ret = devm_add_action_or_reset(dev, pcie_hx_pm_disable, dev);
    if(ret)
        return ret;

    for(i=0; i<4+NUM_PORT; i++) {
        mmio[i] = of_iomap(node, i);
        if(!mmio[i]) {
            dev_err(dev, "failed to map MMIO range %d.\n", i);
            return -EINVAL;
        }
    }
    pcie->base_config = mmio[0];
    for(i=0; i<3; i++)
        pcie->base_phy[i] = mmio[i + 1];
    for(i=0; i<NUM_PORT; i++)
        pcie->base_port[i] = mmio[i + 4];
    pcie->base_pcieclk_postup = of_iomap(node, 4 + NUM_PORT + 1);
    pcie->base_pmgr = of_iomap(node, 4 + NUM_PORT + 2);

    for(i=0; i<3; i++) {
        pcie->clk[i] = devm_clk_get(dev, pcie_hx_clock_names[i]);
        if(IS_ERR(pcie->clk[i]))
            return PTR_ERR(pcie->clk[i]);
    }

    for(i=0; i<NUM_PORT; i++) {
        pcie->perstn[i] = devm_gpiod_get_index(dev, "perst", i, 0);
        if(IS_ERR(pcie->perstn[i])) {
            if(PTR_ERR(pcie->perstn[i]) != -EPROBE_DEFER)
                dev_err(dev, "failed to get PERST#%d gpio: %ld\n", i, PTR_ERR(pcie->perstn[i]));
            return PTR_ERR(pcie->perstn[i]);
        }

        pcie->clkreqn[i] = devm_gpiod_get_index(dev, "clkreq", i, 0);
        if(IS_ERR(pcie->clkreqn[i])) {
            if(PTR_ERR(pcie->clkreqn[i]) != -EPROBE_DEFER)
                dev_err(dev, "failed to get PERST#%d gpio: %ld\n", i, PTR_ERR(pcie->clkreqn[i]));
            return PTR_ERR(pcie->clkreqn[i]);
        }
    }

    pcie->debugio = devm_gpiod_get_index(dev, "debug", 0, 0);
    if(IS_ERR(pcie->debugio)) {
        if(PTR_ERR(pcie->debugio) != -EPROBE_DEFER)
            dev_err(dev, "failed to get debug gpio: %ld\n", PTR_ERR(pcie->debugio));
        return PTR_ERR(pcie->debugio);
    }

    pcie->devpwrio = gpiod_get_array_optional(dev, "devpwr", 0);
    if(pcie->devpwrio && IS_ERR(pcie->devpwrio)) {
        if(PTR_ERR(pcie->devpwrio) != -EPROBE_DEFER)
            dev_err(dev, "failed to get device power gpio: %ld\n", PTR_ERR(pcie->devpwrio));
        return PTR_ERR(pcie->devpwrio);
    }

    for(i=0; i<NUM_PORT; i++) {
        sprintf(name, "devpwr-on-%d", i);
        ret = of_property_count_elems_of_size(node, name, 8);
        pcie->devpwron[i].num = ret;
        if(ret <= 0)
            continue;
        pcie->devpwron[i].seq = devm_kzalloc(dev, ret * 8, GFP_KERNEL);
        if(!pcie->devpwron[i].seq)
            return -ENOMEM;
        ret = of_property_read_variable_u32_array(node, name, pcie->devpwron[i].seq, ret * 2, ret * 2);
        if(ret < 0)
            return ret;
    }

    pcie->pctrl = devm_pinctrl_get_select_default(dev);

    pcie->pdev = pdev;
    platform_set_drvdata(pdev, pcie);

    dma_set_mask_and_coherent(&pdev->dev, DMA_BIT_MASK(64));

    for(i=0; i<NUM_PORT; i++) {
        pirq = platform_get_irq(pdev, i);
        if(pirq < 0) {
            dev_err(dev, "failed to map port IRQ %d\n", i);
            return -EINVAL;
        }
        pcie->port[i].pcie = pcie;
        if(devm_request_irq(dev, pirq, pcie_hx_portirq_isr, 0, dev_name(dev), &pcie->port[i]) < 0) {
            dev_err(dev, "failed to request port IRQ %d\n", i);
            return -EINVAL;
        }
    }

    for(i=0; i<NUM_MSI; i++) {
        virq[i] = platform_get_irq(pdev, NUM_PORT + i);
        if(virq[i] < 0) {
            dev_err(dev, "failed to map MSI IRQ %d\n", i);
            return -EINVAL;
        }
    }

    irq_dom = irq_domain_create_linear(fwnode, NUM_MSI, &pcie_hx_irq_dom_ops, pcie);
    if(!irq_dom) {
        dev_err(dev, "failed to create IRQ domain\n");
        return -ENOMEM;
    }

    msi_dom = msi_create_irq_domain(fwnode, &pcie_hx_msi_dom_info, irq_dom);
    if(!msi_dom) {
        dev_err(dev, "failed to create MSI domain\n");
        irq_domain_remove(irq_dom);
        return -ENOMEM;
    }

    pcie->irq_dom = irq_dom;
    spin_lock_init(&pcie->used_msi_lock);

    for(i=0; i<NUM_MSI; i++) {
        pcie->msi[i].pcie = pcie;
        pcie->msi[i].virq = virq[i];
        irq_set_chained_handler_and_data(virq[i], pcie_hx_msi_isr, &pcie->msi[i]);
        disable_irq(virq[i]);
    }

    for(i=0; i<3; i++) {
        ret = clk_prepare_enable(pcie->clk[i]);
        if(ret < 0)
            return ret;
    }

    ret = pcie_hx_apply_raw_pmgr_gates(pcie);
    if(ret)
        return ret;

    if(of_property_read_bool(node, "hx,stop-after-raw-pmgr-gates")) {
        dev_warn(dev, "HX PCIe probe stopped after raw PMGR gates by device tree.\n");
        return 0;
    }

    ret = pcie_hx_apply_pcieclk_postup(pcie);
    if(ret)
        return ret;

    if(of_property_read_bool(node, "hx,stop-after-pcieclk-postup")) {
        dev_warn(dev, "HX PCIe probe stopped after pcieclk post-up by device tree.\n");
        return 0;
    }

    if(of_property_read_u64(node, "msi-doorbell", &pcie->msi_doorbell))
        pcie->msi_doorbell = 0xBFFFF000ul;

    ret = pcie_hx_probe_sart(pcie);
    if(ret)
        return ret;

    if(of_property_read_bool(node, "hx,stop-after-sart")) {
        dev_warn(dev, "HX PCIe probe stopped after SART setup by device tree.\n");
        return 0;
    }

    if(pcie_hx_setup_ports(pcie))
        return 0;

    pcie->bridge = devm_pci_alloc_host_bridge(dev, 0);
    if(!pcie->bridge)
        return -ENOMEM;

    pcie->cfgwin = pci_host_common_ecam_create(dev, pcie->bridge, &pcie_hx_ecam_ops);
    if(IS_ERR(pcie->cfgwin))
        return PTR_ERR(pcie->cfgwin);

    pcie->bridge->sysdata = pcie->cfgwin;
    pcie->bridge->ops = &pcie_hx_ecam_ops.pci_ops;
    pcie->bridge->msi_domain = true;

    if(of_property_read_bool(node, "hx,stop-before-pci-host-probe")) {
        dev_warn(dev, "HX PCIe probe stopped before pci_host_probe by device tree.\n");
        return 0;
    }

    ret = pcie_hx_pci_host_probe(pcie);
    if(ret < 0)
        return ret;

    return 0;
}

static const struct of_device_id pcie_hx_ids[] = {
    { .compatible = "hx,pcie-h9p" },
    { },
};

static struct platform_driver pcie_hx_driver = {
    .probe = pcie_hx_probe,
    .driver = {
        .name = KBUILD_MODNAME,
        .of_match_table = pcie_hx_ids,
        .suppress_bind_attrs = true,
    },
};
builtin_platform_driver(pcie_hx_driver);
