// SPDX-License-Identifier: GPL-2.0
/*
 * PCIe host bridge driver for Apple H9P/T8010 SoCs.
 *
 * The controller exposes an ECAM-compatible root complex after the SoC-specific
 * power, clock and PHY sequence has brought a port out of reset. The hardware
 * differs enough from the Apple Silicon PCIe controller to keep the early H9P
 * bring-up sequence separate, while still using the generic PCI host bridge
 * and MSI subsystems.
 */

#include <linux/bitfield.h>
#include <linux/bitops.h>
#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/dma-mapping.h>
#include <linux/gpio/consumer.h>
#include <linux/interrupt.h>
#include <linux/irqchip/chained_irq.h>
#include <linux/irqdomain.h>
#include <linux/io.h>
#include <linux/iopoll.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/msi.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/pci.h>
#include <linux/pci-ecam.h>
#include <linux/pinctrl/consumer.h>
#include <linux/platform_device.h>
#include <linux/pm_domain.h>
#include <linux/sizes.h>

#include <linux/apple-h9p-pcie.h>

#include "pci-host-common.h"

#define H9P_NUM_PORTS			4
#define H9P_NUM_MSI			32
#define H9P_MSI_PER_PORT		(H9P_NUM_MSI / H9P_NUM_PORTS)

#define H9P_CFG_PORT_STRIDE		0x8000
#define H9P_CFG_PORT_MISC		0x08e0

#define H9P_PHY0_COMMON_CTL0		0x0004
#define H9P_PHY0_COMMON_CTL1		0x0014
#define H9P_PHY0_COMMON_CTL2		0x0024
#define H9P_PHY0_COMMON_CTL3		0x0034
#define H9P_PHY0_COMMON_CTL_ENABLE	BIT(0)
#define H9P_PHY0_COMMON_CTL_INIT	BIT(4)
#define H9P_PHY0_PORT_STRIDE		0x0080
#define H9P_PHY0_PORTSTAT(port)		(0x0100 + (port) * H9P_PHY0_PORT_STRIDE)
#define H9P_PHY0_PORT_CTL0(port)	(0x0100 + (port) * H9P_PHY0_PORT_STRIDE)
#define H9P_PHY0_PORT_CTL1(port)	(0x0124 + (port) * H9P_PHY0_PORT_STRIDE)
#define H9P_PHY0_PORT_CTL2(port)	(0x0134 + (port) * H9P_PHY0_PORT_STRIDE)
#define H9P_PHY0_COMMON_STAT		0x0028
#define H9P_PHY0_COMMON_STAT_INIT_DONE	BIT(4)
#define H9P_PHY0_COMMON_STAT_READY	BIT(0)
#define H9P_PHY0_PORT_LINK_RATE(port)	(0x4020 + (port) * 0x0040)
#define H9P_PHY1_PORTMASK		0x000c

#define H9P_PHY2_EQ_COMMON0		0x0180
#define H9P_PHY2_EQ_COMMON1		0x0184
#define H9P_PHY2_EQ_TIME0		0x0090
#define H9P_PHY2_EQ_TIME1		0x0098
#define H9P_PHY2_PORT_STRIDE		0x0800
#define H9P_PHY2_PORT(port, reg)	((reg) + (port) * H9P_PHY2_PORT_STRIDE)
#define H9P_PHY2_PORT_EQ_CTL		0x10088
#define H9P_PHY2_PORT_IDLE		0x10784
#define H9P_PHY2_PORT_EQ_PRESET	0x10004
#define H9P_PHY2_PORT_RX_CTL0		0x20788
#define H9P_PHY2_PORT_RX_CTL1		0x207a0
#define H9P_PHY2_PORT_RX_CTL2		0x207a8
#define H9P_PHY2_PORT_RX_CTL3		0x20400
#define H9P_PHY2_PORT_TIMER0		0x2009c
#define H9P_PHY2_PORT_TIMER1		0x200dc
#define H9P_PHY2_PORT_TIMER2		0x200a0
#define H9P_PHY2_PORT_TIMER3		0x200e0
#define H9P_PHY2_PORT_TIMER4		0x200a4
#define H9P_PHY2_PORT_TIMER5		0x200e4
#define H9P_PHY2_PORT_CLEAR0		0x20330
#define H9P_PHY2_PORT_CLEAR1		0x20340
#define H9P_PHY2_PORT_CLEAR2		0x20350

#define H9P_PORT_LTSSMCTL		0x0080
#define H9P_PORT_LTSSM_ENABLE		BIT(0)
#define H9P_PORT_IRQSTAT		0x0100
#define H9P_PORT_IRQMASK		0x0104
#define H9P_PORT_IRQMASK_PRE_LINK	0xff002fff
#define H9P_PORT_IRQSTAT_PRE_LINK	0x00ffd000
#define H9P_PORT_IRQMASK_LINK_UP	0xff002f0f
#define H9P_PORT_PWRCTL		0x0124
#define H9P_PORT_PWRCTL_INIT		0x31
#define H9P_PORT_MSIVECBASE		0x0128
#define H9P_PORT_ENABLE		0x0140
#define H9P_PORT_ENABLE_APPLE		BIT(31)
#define H9P_PORT_LINKSTS		0x0208
#define H9P_PORT_LINKSTS_LTSSM		GENMASK(13, 8)
#define H9P_PORT_LTSSM_DETECT		0x11
#define H9P_PORT_LTSSM_L0		0x14

#define H9P_LINK_SPEED_2_5GT		1
#define H9P_LINK_SPEED_8GT		3

#define H9P_PCIECLK_POSTUP0		0x0000
#define H9P_PCIECLK_POSTUP1		0x000c
#define H9P_PCIECLK_POSTUP2		0x4104
#define H9P_PCIECLK_POSTUP3		0x4100
#define H9P_PCIECLK_POSTUP0_VALUE	0x00000007
#define H9P_PCIECLK_POSTUP1_VALUE	0x80010005
#define H9P_PCIECLK_POSTUP2_VALUE	0x00000003
#define H9P_PCIECLK_POSTUP3_VALUE	0x00000003

#define H9P_NVMMU_TCB_CTRL		0x0004
#define H9P_NVMMU_TCB_BASE_LO		0x0008
#define H9P_NVMMU_TCB_BASE_HI		0x000c
#define H9P_NVMMU_TCB_TABLE_LO		0x0010
#define H9P_NVMMU_TCB_TABLE_HI		0x0014
#define H9P_NVMMU_SART_CTRL		0x0020
#define H9P_NVMMU_SART_VA_BASE		0x0024
#define H9P_NVMMU_SART_VA_END		0x0028
#define H9P_NVMMU_SART_PA_BASE		0x002c

#define H9P_NVMMU_TCB_BYTES		0x80
#define H9P_NVMMU_TCB_DWORDS		(H9P_NVMMU_TCB_BYTES / sizeof(u32))
#define H9P_NVMMU_SGL_WORDS		APPLE_H9P_NVMMU_MAX_PAGES
#define H9P_NVMMU_FLATDMA_BASE		0x40000000ULL
#define H9P_NVMMU_FLATDMA_STRIDE	SZ_8M
#define H9P_NVMMU_SART_ALIGNMENT	SZ_1M
#define H9P_NVMMU_TCB_READ		0x100
#define H9P_NVMMU_TCB_WRITE		0x200

#define H9P_DEFAULT_MSI_DOORBELL	0xbffff000ULL

struct apple_h9p_tunable {
	u32 offset;
	u32 size;
	u64 mask;
	u64 data;
};

static const struct apple_h9p_tunable h9p_phy0_tunables[] = {
	{ 0x0008, 4, 0x7f7f7f7f, 0x00000000 },
	{ 0x000c, 4, 0x00073f3f, 0x00043f00 },
	{ 0x0010, 4, 0x00000700, 0x00000000 },
	{ 0x0018, 4, 0x00ffffff, 0x000c0960 },
	{ 0x001c, 4, 0x00001fff, 0x0000092c },
	{ 0x002c, 4, 0x000000ff, 0x00000009 },
	{ 0x003c, 4, 0x80000000, 0x00000000 },
	{ 0x0100, 4, 0x31100010, 0x01000000 },
	{ 0x0108, 4, 0x00000707, 0x00000000 },
	{ 0x010c, 4, 0x00073f3f, 0x00043f00 },
	{ 0x0110, 4, 0x00000011, 0x00000001 },
	{ 0x0114, 4, 0x00000007, 0x00000000 },
	{ 0x0118, 4, 0x00073f3f, 0x00043f00 },
	{ 0x0120, 4, 0x0333003f, 0x0111000f },
	{ 0x0130, 4, 0x000000ff, 0x0000000f },
	{ 0x0138, 4, 0x0000007f, 0x0000003e },
	{ 0x0180, 4, 0x31100010, 0x01000000 },
	{ 0x0188, 4, 0x00000707, 0x00000000 },
	{ 0x018c, 4, 0x00073f3f, 0x00043f00 },
	{ 0x01a0, 4, 0x0333003f, 0x0111000f },
	{ 0x01b0, 4, 0x000000ff, 0x0000000f },
	{ 0x01b8, 4, 0x0000007f, 0x0000003e },
	{ 0x0200, 4, 0x31100010, 0x01000000 },
	{ 0x0208, 4, 0x00000707, 0x00000000 },
	{ 0x020c, 4, 0x00073f3f, 0x00043f00 },
	{ 0x0220, 4, 0x0333003f, 0x0111000f },
	{ 0x0230, 4, 0x000000ff, 0x0000000f },
	{ 0x0238, 4, 0x0000007f, 0x0000003e },
	{ 0x0280, 4, 0x31100010, 0x01000000 },
	{ 0x0288, 4, 0x00000707, 0x00000000 },
	{ 0x028c, 4, 0x00073f3f, 0x00043f00 },
	{ 0x02a0, 4, 0x0333003f, 0x0111000f },
	{ 0x02b0, 4, 0x000000ff, 0x0000000f },
	{ 0x02b8, 4, 0x0000007f, 0x0000003e },
	{ 0x0100, 4, 0x00000010, 0x00000010 },
	{ 0x0180, 4, 0x00000010, 0x00000000 },
	{ 0x0200, 4, 0x00000010, 0x00000000 },
	{ 0x0280, 4, 0x00000010, 0x00000000 },
};

static const struct apple_h9p_tunable h9p_config_tunables[] = {
	{ 0x0098, 4, 0x0000000f, 0x00000000 },
	{ 0x0164, 4, 0x00f8ff00, 0x00000000 },
	{ 0x08e0, 4, 0x00000005, 0x00000005 },
};

static const struct apple_h9p_tunable h9p_port_tunables[] = {
	{ 0x0090, 4, 0x000000ff, 0x00000028 },
	{ 0x0130, 4, 0x0000000d, 0x00000005 },
	{ 0x0134, 4, 0x00000001, 0x00000001 },
	{ 0x0138, 4, 0x00007f7f, 0x00000000 },
	{ 0x013c, 4, 0x00000002, 0x00000002 },
	{ 0x0140, 4, 0x0073ffff, 0x00704c4b },
};

struct apple_h9p_pcie {
	struct device *dev;
	struct platform_device *pdev;
	struct pci_host_bridge *bridge;
	struct pci_config_window *cfgwin;

	void __iomem *base_config;
	void __iomem *base_phy[3];
	void __iomem *base_port[H9P_NUM_PORTS];
	void __iomem *base_pcieclk_postup;

	struct clk_bulk_data clks[3];
	struct gpio_desc *perst[H9P_NUM_PORTS];
	struct gpio_desc *clkreq[H9P_NUM_PORTS];
	struct gpio_descs *devpwr;
	struct pinctrl *pinctrl;
	u32 enabled_ports;

	struct apple_h9p_nvmmu {
		struct apple_h9p_pcie *pcie;
		void __iomem *base;
		u64 pa_base;
		u32 va_base;
		u32 size;
		void *tcb;
		void *tcb_table;
		void *tcb_sgl;
		size_t tcb_size;
		size_t tcb_table_size;
		size_t tcb_sgl_size;
		dma_addr_t tcb_dma;
		dma_addr_t tcb_table_dma;
		dma_addr_t tcb_sgl_dma;
	} nvmmu[H9P_NUM_PORTS];

	struct device **pd_dev;
	struct device_link **pd_link;
	int pd_count;

	DECLARE_BITMAP(used_msi[H9P_NUM_PORTS], H9P_MSI_PER_PORT);
	u64 msi_doorbell;
	/* Protects the per-port MSI allocation bitmaps. */
	spinlock_t used_msi_lock;
	struct irq_domain *irq_dom;
	struct irq_domain *msi_dom;

	struct apple_h9p_msi {
		struct apple_h9p_pcie *pcie;
		int virq;
		bool disabled;
	} msi[H9P_NUM_MSI];
};

static inline void h9p_rmw(void __iomem *addr, u32 clear, u32 set)
{
	writel((readl(addr) & ~clear) | set, addr);
}

static inline void h9p_rmww(void __iomem *addr, u16 clear, u16 set)
{
	writew((readw(addr) & ~clear) | set, addr);
}

static inline u64 h9p_readsz(void __iomem *addr, u32 size)
{
	switch (size) {
	case 1:
		return readb(addr);
	case 2:
		return readw(addr);
	case 4:
		return readl(addr);
	case 8:
		return readq(addr);
	default:
		return 0;
	}
}

static inline void h9p_writesz(u64 value, void __iomem *addr, u32 size)
{
	switch (size) {
	case 1:
		writeb(value, addr);
		break;
	case 2:
		writew(value, addr);
		break;
	case 4:
		writel(value, addr);
		break;
	case 8:
		writeq(value, addr);
		break;
	}
}

static inline void h9p_writel_flush(u32 value, void __iomem *addr)
{
	writel(value, addr);
	readl(addr);
}

static void apple_h9p_pcie_detach_genpd(struct apple_h9p_pcie *pcie)
{
	int i;

	for (i = pcie->pd_count - 1; i >= 0; i--) {
		if (pcie->pd_link[i])
			device_link_del(pcie->pd_link[i]);
		if (!IS_ERR_OR_NULL(pcie->pd_dev[i]))
			dev_pm_domain_detach(pcie->pd_dev[i], true);
	}
}

static int apple_h9p_pcie_attach_genpd(struct apple_h9p_pcie *pcie)
{
	struct device *dev = pcie->dev;
	int i;

	pcie->pd_count = of_count_phandle_with_args(dev->of_node,
						    "power-domains",
						    "#power-domain-cells");
	if (pcie->pd_count <= 1)
		return 0;

	pcie->pd_dev = devm_kcalloc(dev, pcie->pd_count,
				    sizeof(*pcie->pd_dev), GFP_KERNEL);
	if (!pcie->pd_dev)
		return -ENOMEM;

	pcie->pd_link = devm_kcalloc(dev, pcie->pd_count,
				     sizeof(*pcie->pd_link), GFP_KERNEL);
	if (!pcie->pd_link)
		return -ENOMEM;

	for (i = 0; i < pcie->pd_count; i++) {
		pcie->pd_dev[i] = dev_pm_domain_attach_by_id(dev, i);
		if (IS_ERR(pcie->pd_dev[i])) {
			apple_h9p_pcie_detach_genpd(pcie);
			return PTR_ERR(pcie->pd_dev[i]);
		}

		pcie->pd_link[i] = device_link_add(dev, pcie->pd_dev[i],
						   DL_FLAG_STATELESS |
						   DL_FLAG_PM_RUNTIME |
						   DL_FLAG_RPM_ACTIVE);
		if (!pcie->pd_link[i]) {
			apple_h9p_pcie_detach_genpd(pcie);
			return -EINVAL;
		}
	}

	return 0;
}

static void apple_h9p_pcie_genpd_cleanup(void *data)
{
	apple_h9p_pcie_detach_genpd(data);
}

static void apple_h9p_pcie_clk_cleanup(void *data)
{
	struct apple_h9p_pcie *pcie = data;

	clk_bulk_disable_unprepare(ARRAY_SIZE(pcie->clks), pcie->clks);
}

static struct apple_h9p_pcie *apple_h9p_pcie_lookup(struct device *dev)
{
	struct pci_host_bridge *bridge = dev_get_drvdata(dev);

	return bridge ? pci_host_bridge_priv(bridge) : NULL;
}

static int apple_h9p_pcie_config_read(struct pci_bus *bus, unsigned int devfn,
				      int where, int size, u32 *val)
{
	struct pci_config_window *cfg = bus->sysdata;

	if (bus->number == cfg->busr.start && PCI_SLOT(devfn) >= H9P_NUM_PORTS)
		return PCIBIOS_DEVICE_NOT_FOUND;

	return pci_generic_config_read(bus, devfn, where, size, val);
}

static int apple_h9p_pcie_config_write(struct pci_bus *bus, unsigned int devfn,
				       int where, int size, u32 val)
{
	struct pci_config_window *cfg = bus->sysdata;

	if (bus->number == cfg->busr.start && PCI_SLOT(devfn) >= H9P_NUM_PORTS)
		return PCIBIOS_DEVICE_NOT_FOUND;

	if (where <= PCI_INTERRUPT_LINE && where + size > PCI_INTERRUPT_LINE)
		val |= 0xffu << ((PCI_INTERRUPT_LINE - where) << 3);

	return pci_generic_config_write(bus, devfn, where, size, val);
}

static unsigned int apple_h9p_pcie_bus_to_port(struct apple_h9p_pcie *pcie,
					       unsigned int bus)
{
	unsigned int port;

	for (port = 0; port < H9P_NUM_PORTS; port++) {
		u32 cfg, sec, sub;

		cfg = readl(pcie->base_config + port * H9P_CFG_PORT_STRIDE +
			    PCI_PRIMARY_BUS);
		sec = (cfg >> 8) & 0xff;
		sub = (cfg >> 16) & 0xff;

		if (!sec || !sub || sec == 0xff || sub == 0xff)
			continue;
		if (bus >= sec && bus <= sub)
			return port;
	}

	return H9P_NUM_PORTS;
}

static int apple_h9p_pcie_device_port(struct apple_h9p_pcie *pcie,
				      struct device *dev)
{
	struct pci_dev *pdev;

	if (!dev_is_pci(dev))
		return -ENODEV;

	pdev = to_pci_dev(dev);
	if (!pdev->bus)
		return -ENODEV;

	return apple_h9p_pcie_bus_to_port(pcie, pdev->bus->number);
}

static void apple_h9p_msi_compose_msg(struct irq_data *d, struct msi_msg *msg)
{
	struct apple_h9p_pcie *pcie = irq_data_get_irq_chip_data(d);

	if (!pcie) {
		memset(msg, 0, sizeof(*msg));
		return;
	}

	msg->address_lo = lower_32_bits(pcie->msi_doorbell);
	msg->address_hi = upper_32_bits(pcie->msi_doorbell);
	msg->data = d->hwirq;
}

static void apple_h9p_msi_write_msg(struct irq_data *d, struct msi_msg *msg)
{
	pci_write_msi_msg(d->irq, msg);
}

static int apple_h9p_msi_set_affinity(struct irq_data *d,
				      const struct cpumask *mask, bool force)
{
	return -EINVAL;
}

static void apple_h9p_msi_mask(struct irq_data *d)
{
	struct apple_h9p_pcie *pcie = irq_data_get_irq_chip_data(d);

	if (!pcie || d->hwirq >= H9P_NUM_MSI || pcie->msi[d->hwirq].virq <= 0)
		return;

	if (!pcie->msi[d->hwirq].disabled) {
		disable_irq_nosync(pcie->msi[d->hwirq].virq);
		pcie->msi[d->hwirq].disabled = true;
	}
}

static void apple_h9p_msi_unmask(struct irq_data *d)
{
	struct apple_h9p_pcie *pcie = irq_data_get_irq_chip_data(d);

	if (!pcie || d->hwirq >= H9P_NUM_MSI || pcie->msi[d->hwirq].virq <= 0)
		return;

	if (pcie->msi[d->hwirq].disabled) {
		enable_irq(pcie->msi[d->hwirq].virq);
		pcie->msi[d->hwirq].disabled = false;
	}
}

static void apple_h9p_msi_ack(struct irq_data *d)
{
}

static struct irq_chip apple_h9p_msi_chip = {
	.name = "Apple H9P PCIe MSI",
	.irq_ack = apple_h9p_msi_ack,
	.irq_mask = apple_h9p_msi_mask,
	.irq_unmask = apple_h9p_msi_unmask,
	.irq_compose_msi_msg = apple_h9p_msi_compose_msg,
	.irq_write_msi_msg = apple_h9p_msi_write_msg,
	.irq_set_affinity = apple_h9p_msi_set_affinity,
};

static void apple_h9p_msi_isr(struct irq_desc *desc)
{
	struct irq_chip *chip = irq_desc_get_chip(desc);
	struct apple_h9p_msi *msi = irq_desc_get_handler_data(desc);
	struct apple_h9p_pcie *pcie = msi->pcie;
	unsigned int idx = msi - pcie->msi;
	unsigned int virq;

	chained_irq_enter(chip, desc);
	virq = irq_find_mapping(pcie->irq_dom, idx);
	if (virq)
		generic_handle_irq(virq);
	chained_irq_exit(chip, desc);
}

static int apple_h9p_msi_alloc(struct irq_domain *domain, unsigned int virq,
			       unsigned int nr_irqs, void *args)
{
	struct apple_h9p_pcie *pcie = domain->host_data;
	msi_alloc_info_t *info = args;
	struct msi_desc *desc = info ? info->desc : NULL;
	struct pci_dev *pdev = NULL;
	unsigned long flags;
	unsigned int bus = 0;
	unsigned int port;
	int slot;

	if (nr_irqs != 1)
		return -ENOSPC;

	if (desc && desc->dev && dev_is_pci(desc->dev)) {
		pdev = to_pci_dev(desc->dev);
		if (pdev->bus)
			bus = pdev->bus->number;
	}

	if (bus < 1)
		return -ENOSPC;

	port = apple_h9p_pcie_bus_to_port(pcie, bus);
	if (port >= H9P_NUM_PORTS)
		return -ENOSPC;
	if (!(pcie->enabled_ports & BIT(port)))
		return -ENOSPC;

	spin_lock_irqsave(&pcie->used_msi_lock, flags);
	slot = find_first_zero_bit(pcie->used_msi[port], H9P_MSI_PER_PORT);
	if (slot >= H9P_MSI_PER_PORT) {
		spin_unlock_irqrestore(&pcie->used_msi_lock, flags);
		return -ENOSPC;
	}
	__set_bit(slot, pcie->used_msi[port]);
	spin_unlock_irqrestore(&pcie->used_msi_lock, flags);

	irq_domain_set_info(domain, virq, port * H9P_MSI_PER_PORT + slot,
			    &apple_h9p_msi_chip, pcie, handle_edge_irq,
			    NULL, NULL);
	return 0;
}

static void apple_h9p_msi_free(struct irq_domain *domain, unsigned int virq,
			       unsigned int nr_irqs)
{
	struct irq_data *d = irq_domain_get_irq_data(domain, virq);
	struct apple_h9p_pcie *pcie = d ? irq_data_get_irq_chip_data(d) : NULL;
	unsigned long flags;
	unsigned int i;

	if (!pcie || !d)
		return;

	spin_lock_irqsave(&pcie->used_msi_lock, flags);
	for (i = 0; i < nr_irqs; i++) {
		unsigned long hwirq = d->hwirq + i;
		unsigned int port = hwirq / H9P_MSI_PER_PORT;
		unsigned int slot = hwirq % H9P_MSI_PER_PORT;

		if (port < H9P_NUM_PORTS)
			__clear_bit(slot, pcie->used_msi[port]);
	}
	spin_unlock_irqrestore(&pcie->used_msi_lock, flags);
}

static const struct irq_domain_ops apple_h9p_msi_domain_ops = {
	.alloc = apple_h9p_msi_alloc,
	.free = apple_h9p_msi_free,
};

static struct irq_chip apple_h9p_msi_parent_chip = {
	.name = "Apple H9P PCIe MSI parent",
	.irq_ack = irq_chip_ack_parent,
	.irq_mask = irq_chip_mask_parent,
	.irq_unmask = irq_chip_unmask_parent,
	.irq_write_msi_msg = apple_h9p_msi_write_msg,
};

static struct msi_domain_info apple_h9p_msi_domain_info = {
	.flags = MSI_FLAG_MULTI_PCI_MSI | MSI_FLAG_PCI_MSIX |
		 MSI_FLAG_USE_DEF_DOM_OPS | MSI_FLAG_USE_DEF_CHIP_OPS |
		 MSI_FLAG_PCI_MSI_MASK_PARENT,
	.chip = &apple_h9p_msi_parent_chip,
};

static void apple_h9p_pcie_msi_cleanup(void *data)
{
	struct apple_h9p_pcie *pcie = data;
	unsigned int i;

	for (i = 0; i < H9P_NUM_MSI; i++) {
		if (pcie->msi[i].virq <= 0)
			continue;

		irq_set_chained_handler_and_data(pcie->msi[i].virq, NULL,
						 NULL);
		if (pcie->msi[i].disabled) {
			enable_irq(pcie->msi[i].virq);
			pcie->msi[i].disabled = false;
		}
	}

	if (pcie->msi_dom) {
		irq_domain_remove(pcie->msi_dom);
		pcie->msi_dom = NULL;
	}

	if (pcie->irq_dom) {
		irq_domain_remove(pcie->irq_dom);
		pcie->irq_dom = NULL;
	}
}

static int apple_h9p_pcie_setup_msi(struct apple_h9p_pcie *pcie)
{
	struct device *dev = pcie->dev;
	struct fwnode_handle *fwnode = dev_fwnode(dev);
	unsigned int i;
	int ret;

	pcie->irq_dom = irq_domain_create_linear(fwnode, H9P_NUM_MSI,
						 &apple_h9p_msi_domain_ops,
						 pcie);
	if (!pcie->irq_dom)
		return -ENOMEM;

	pcie->msi_dom = msi_create_irq_domain(fwnode,
					      &apple_h9p_msi_domain_info,
					      pcie->irq_dom);
	if (!pcie->msi_dom) {
		irq_domain_remove(pcie->irq_dom);
		pcie->irq_dom = NULL;
		return -ENOMEM;
	}

	ret = devm_add_action_or_reset(dev, apple_h9p_pcie_msi_cleanup,
				       pcie);
	if (ret)
		return ret;

	for (i = 0; i < H9P_NUM_MSI; i++) {
		int irq = platform_get_irq(pcie->pdev, H9P_NUM_PORTS + i);

		if (irq < 0)
			return irq;

		pcie->msi[i].pcie = pcie;
		pcie->msi[i].virq = irq;
		irq_set_chained_handler_and_data(irq, apple_h9p_msi_isr,
						 &pcie->msi[i]);
		disable_irq(irq);
		pcie->msi[i].disabled = true;
	}

	return 0;
}

static u64 apple_h9p_read_pci_cap(struct apple_h9p_pcie *pcie,
				  unsigned int busdevfn, u32 type)
{
	void __iomem *cfg = pcie->base_config + (busdevfn << 12);
	u32 ptr = readl(cfg + PCI_CAPABILITY_LIST) & 0xff;

	while (ptr) {
		u32 next = readl(cfg + ptr);

		if ((next & 0xff) == type)
			return ptr;
		ptr = (next >> 8) & 0xff;
	}

	return 0;
}

static int apple_h9p_wait(void __iomem *addr, u32 mask, u32 min, u32 max,
			  unsigned long timeout_us)
{
	u32 val;

	return readl_poll_timeout(addr, val, (val & mask) >= min &&
				  (val & mask) <= max, 1000, timeout_us);
}

static int apple_h9p_wait_gpio(struct gpio_desc *desc, int value,
			       unsigned long timeout_us)
{
	ktime_t timeout = ktime_add_us(ktime_get(), timeout_us);

	do {
		if (gpiod_get_raw_value(desc) == value)
			return 0;
		usleep_range(1000, 2000);
	} while (ktime_before(ktime_get(), timeout));

	return -ETIMEDOUT;
}

static irqreturn_t apple_h9p_nvmmu_irq(int irq, void *data)
{
	struct apple_h9p_nvmmu *nvmmu = data;
	struct apple_h9p_pcie *pcie = nvmmu->pcie;
	unsigned int port = nvmmu - pcie->nvmmu;

	dev_err_ratelimited(pcie->dev, "port %u NVMMU fault interrupt\n", port);
	return IRQ_HANDLED;
}

static int apple_h9p_setup_nvmmu_port(struct apple_h9p_pcie *pcie,
				      unsigned int port)
{
	struct apple_h9p_nvmmu *nvmmu = &pcie->nvmmu[port];
	struct device *dev = pcie->dev;
	struct device_node *mem_np;
	struct resource res;
	u32 iova;
	int irq;
	int ret;

	if (!nvmmu->base)
		return 0;

	mem_np = of_parse_phandle(dev->of_node, "memory-region", port);
	if (!mem_np)
		return dev_err_probe(dev, -EINVAL,
				     "port %u NVMMU missing memory-region\n",
				     port);

	ret = of_address_to_resource(mem_np, 0, &res);
	if (ret)
		goto out_put_node;

	ret = of_property_read_u32(dev->of_node, "apple,nvmmu-iova", &iova);
	if (ret)
		goto out_put_node;

	if (resource_size(&res) < H9P_NVMMU_SART_ALIGNMENT ||
	    !IS_ALIGNED(res.start, H9P_NVMMU_SART_ALIGNMENT) ||
	    !IS_ALIGNED(iova, H9P_NVMMU_SART_ALIGNMENT)) {
		ret = -EINVAL;
		goto out_put_node;
	}

	nvmmu->pcie = pcie;
	nvmmu->pa_base = res.start;
	nvmmu->va_base = iova;
	nvmmu->size = resource_size(&res);
	nvmmu->tcb_size = round_up(APPLE_H9P_NVMMU_MAX_REQS *
				   H9P_NVMMU_TCB_BYTES, PAGE_SIZE);
	nvmmu->tcb_table_size = PAGE_SIZE * 16;
	nvmmu->tcb_sgl_size = round_up(APPLE_H9P_NVMMU_MAX_REQS *
				       H9P_NVMMU_SGL_WORDS * sizeof(u32),
				       PAGE_SIZE);

	nvmmu->tcb = dmam_alloc_attrs(dev, nvmmu->tcb_size, &nvmmu->tcb_dma,
				      GFP_KERNEL | __GFP_ZERO,
				      DMA_ATTR_WRITE_COMBINE);
	if (!nvmmu->tcb) {
		ret = -ENOMEM;
		goto out_put_node;
	}

	nvmmu->tcb_table = dmam_alloc_attrs(dev, nvmmu->tcb_table_size,
					    &nvmmu->tcb_table_dma,
					    GFP_KERNEL | __GFP_ZERO,
					    DMA_ATTR_WRITE_COMBINE);
	if (!nvmmu->tcb_table) {
		ret = -ENOMEM;
		goto out_put_node;
	}

	nvmmu->tcb_sgl = dmam_alloc_attrs(dev, nvmmu->tcb_sgl_size,
					  &nvmmu->tcb_sgl_dma,
					  GFP_KERNEL | __GFP_ZERO,
					  DMA_ATTR_WRITE_COMBINE);
	if (!nvmmu->tcb_sgl) {
		ret = -ENOMEM;
		goto out_put_node;
	}

	h9p_writel_flush(lower_32_bits(nvmmu->tcb_dma),
			 nvmmu->base + H9P_NVMMU_TCB_BASE_LO);
	h9p_writel_flush(upper_32_bits(nvmmu->tcb_dma),
			 nvmmu->base + H9P_NVMMU_TCB_BASE_HI);
	h9p_writel_flush(lower_32_bits(nvmmu->tcb_table_dma),
			 nvmmu->base + H9P_NVMMU_TCB_TABLE_LO);
	h9p_writel_flush(upper_32_bits(nvmmu->tcb_table_dma),
			 nvmmu->base + H9P_NVMMU_TCB_TABLE_HI);
	h9p_writel_flush(0x10000, nvmmu->base + H9P_NVMMU_TCB_CTRL);

	ret = apple_h9p_wait(nvmmu->base + H9P_NVMMU_TCB_CTRL, 0x10, 0, 0,
			     250000);
	if (ret)
		goto out_put_node;

	h9p_writel_flush(nvmmu->va_base - 0x80000000U,
			 nvmmu->base + H9P_NVMMU_SART_VA_BASE);
	h9p_writel_flush(round_up(nvmmu->va_base + nvmmu->size,
				  H9P_NVMMU_SART_ALIGNMENT) - 0x80100000U,
			 nvmmu->base + H9P_NVMMU_SART_VA_END);
	h9p_writel_flush(nvmmu->pa_base >> 20,
			 nvmmu->base + H9P_NVMMU_SART_PA_BASE);
	h9p_writel_flush(1, nvmmu->base + H9P_NVMMU_SART_CTRL);

	irq = platform_get_irq_optional(pcie->pdev, H9P_NUM_PORTS +
					H9P_NUM_MSI + port);
	if (irq > 0) {
		ret = devm_request_irq(dev, irq, apple_h9p_nvmmu_irq, 0,
				       dev_name(dev), nvmmu);
		if (ret)
			goto out_put_node;
	} else if (irq != -ENXIO) {
		ret = irq;
		goto out_put_node;
	}

	dev_dbg(dev, "port %u NVMMU window %#x@%pa size %#x\n", port,
		nvmmu->va_base, &res.start, nvmmu->size);

out_put_node:
	of_node_put(mem_np);
	return ret;
}

static int apple_h9p_setup_nvmmu(struct apple_h9p_pcie *pcie)
{
	unsigned int port;
	int ret;

	for (port = 0; port < H9P_NUM_PORTS; port++) {
		if (!(pcie->enabled_ports & BIT(port)))
			continue;

		ret = apple_h9p_setup_nvmmu_port(pcie, port);
		if (ret)
			return dev_err_probe(pcie->dev, ret,
					     "port %u NVMMU setup failed\n",
					     port);
	}

	return 0;
}

int apple_h9p_pcie_map_nvmmu(struct device *dev, unsigned int tag,
			     const u64 *pages, unsigned int npages,
			     dma_addr_t *iova)
{
	struct apple_h9p_nvmmu *nvmmu;
	struct apple_h9p_pcie *pcie;
	struct device *host_dev = dev;
	unsigned int port;
	unsigned int i;
	u64 sgl_dma;
	u32 *tcb;
	u32 *sgl;
	int ret;

	if (tag >= APPLE_H9P_NVMMU_MAX_REQS ||
	    npages > APPLE_H9P_NVMMU_MAX_PAGES)
		return -EINVAL;
	if (npages && !pages)
		return -EINVAL;

	while (host_dev && host_dev->bus == dev->bus)
		host_dev = host_dev->parent;
	if (!host_dev || !host_dev->parent)
		return -ENODEV;

	pcie = apple_h9p_pcie_lookup(host_dev->parent);
	if (!pcie)
		return -ENODEV;

	ret = apple_h9p_pcie_device_port(pcie, dev);
	if (ret < 0)
		return ret;
	port = ret;
	if (port >= H9P_NUM_PORTS || !(pcie->enabled_ports & BIT(port)))
		return -ENODEV;

	nvmmu = &pcie->nvmmu[port];
	if (!nvmmu->base || !nvmmu->tcb || !nvmmu->tcb_sgl)
		return -EOPNOTSUPP;

	tcb = (u32 *)nvmmu->tcb + tag * H9P_NVMMU_TCB_DWORDS;
	sgl = (u32 *)nvmmu->tcb_sgl + tag * H9P_NVMMU_SGL_WORDS;
	memset(tcb, 0, H9P_NVMMU_TCB_BYTES);
	memset(sgl, 0, H9P_NVMMU_SGL_WORDS * sizeof(*sgl));

	if (npages) {
		tcb[0] = H9P_NVMMU_TCB_READ | H9P_NVMMU_TCB_WRITE;
		tcb[1] = npages;
		tcb[2] = pages[0] >> ilog2(APPLE_H9P_NVMMU_PAGE_SIZE);
		for (i = 0; i < npages; i++)
			sgl[i] = pages[i] >> ilog2(APPLE_H9P_NVMMU_PAGE_SIZE);

		sgl_dma = nvmmu->tcb_sgl_dma +
			  tag * H9P_NVMMU_SGL_WORDS * sizeof(*sgl);
		memcpy(&tcb[4], &sgl_dma, sizeof(sgl_dma));
		if (iova)
			*iova = H9P_NVMMU_FLATDMA_BASE +
				tag * H9P_NVMMU_FLATDMA_STRIDE;
	} else {
		dma_wmb();
		h9p_writel_flush(tag, nvmmu->base + H9P_NVMMU_TCB_CTRL);
		if (iova)
			*iova = 0;
		return 0;
	}

	dma_wmb();
	return 0;
}
EXPORT_SYMBOL_GPL(apple_h9p_pcie_map_nvmmu);

static void apple_h9p_apply_tunables(void __iomem *base,
				     const struct apple_h9p_tunable *tunables,
				     unsigned int count)
{
	unsigned int i;

	for (i = 0; i < count; i++) {
		u64 val = h9p_readsz(base + tunables[i].offset, tunables[i].size);

		if ((val & tunables[i].mask) == tunables[i].data)
			continue;
		val &= ~tunables[i].mask;
		val |= tunables[i].data;
		h9p_writesz(val, base + tunables[i].offset, tunables[i].size);
	}
}

static int apple_h9p_pcieclk_postup(struct apple_h9p_pcie *pcie)
{
	if (!pcie->base_pcieclk_postup)
		return 0;

	writel(H9P_PCIECLK_POSTUP0_VALUE,
	       pcie->base_pcieclk_postup + H9P_PCIECLK_POSTUP0);
	writel(H9P_PCIECLK_POSTUP1_VALUE,
	       pcie->base_pcieclk_postup + H9P_PCIECLK_POSTUP1);
	writel(H9P_PCIECLK_POSTUP2_VALUE,
	       pcie->base_pcieclk_postup + H9P_PCIECLK_POSTUP2);
	writel(H9P_PCIECLK_POSTUP3_VALUE,
	       pcie->base_pcieclk_postup + H9P_PCIECLK_POSTUP3);

	return 0;
}

static bool apple_h9p_link_up(struct apple_h9p_pcie *pcie, unsigned int port)
{
	u32 linksts = readl(pcie->base_port[port] + H9P_PORT_LINKSTS);

	linksts = FIELD_GET(H9P_PORT_LINKSTS_LTSSM, linksts);
	return linksts >= H9P_PORT_LTSSM_DETECT && linksts <= H9P_PORT_LTSSM_L0;
}

static int apple_h9p_setup_port(struct apple_h9p_pcie *pcie, unsigned int port)
{
	struct device *dev = pcie->dev;
	u64 cap;
	int ret;

	if (apple_h9p_link_up(pcie, port))
		return 0;

	gpiod_direction_output(pcie->perst[port], 0);

	h9p_rmw(pcie->base_phy[0] + H9P_PHY0_PORT_CTL2(port), 1, 0);
	h9p_rmw(pcie->base_phy[0] + H9P_PHY0_PORT_CTL1(port), 0, 1);

	ret = apple_h9p_wait(pcie->base_phy[0] + H9P_PHY0_COMMON_STAT,
			     H9P_PHY0_COMMON_STAT_INIT_DONE,
			     H9P_PHY0_COMMON_STAT_INIT_DONE,
			     H9P_PHY0_COMMON_STAT_INIT_DONE, 250000);
	if (ret)
		return dev_err_probe(dev, ret, "port %u init timeout\n", port);

	usleep_range(250, 1000);
	h9p_rmw(pcie->base_phy[0] + H9P_PHY0_PORT_CTL0(port), 0, 1);
	h9p_rmw(pcie->base_phy[0] + H9P_PHY0_PORT_CTL0(port), 0x100, 0);
	usleep_range(500, 1000);
	h9p_rmw(pcie->base_phy[0] + H9P_PHY0_PORT_CTL2(port), 0, 1);

	writel(port ? 0 : H9P_LINK_SPEED_8GT,
	       pcie->base_phy[0] + H9P_PHY0_PORT_LINK_RATE(port));
	h9p_rmw(pcie->base_phy[0] + H9P_PHY0_PORT_CTL1(port), 0x100, 0);

	cap = apple_h9p_read_pci_cap(pcie, port << 3, PCI_CAP_ID_EXP);
	if (cap)
		h9p_rmww(pcie->base_config + port * H9P_CFG_PORT_STRIDE +
			 cap + PCI_EXP_LNKCTL2, PCI_EXP_LNKCTL2_TLS,
			 port ? H9P_LINK_SPEED_2_5GT : H9P_LINK_SPEED_8GT);

	apple_h9p_apply_tunables(pcie->base_config + port * H9P_CFG_PORT_STRIDE,
				 h9p_config_tunables,
				 ARRAY_SIZE(h9p_config_tunables));
	apple_h9p_apply_tunables(pcie->base_port[port], h9p_port_tunables,
				 ARRAY_SIZE(h9p_port_tunables));

	h9p_rmw(pcie->base_config + port * H9P_CFG_PORT_STRIDE +
		H9P_CFG_PORT_MISC, 0, 1);

	writel(H9P_PORT_IRQMASK_PRE_LINK,
	       pcie->base_port[port] + H9P_PORT_IRQMASK);
	writel(H9P_PORT_IRQSTAT_PRE_LINK,
	       pcie->base_port[port] + H9P_PORT_IRQSTAT);

	h9p_rmw(pcie->base_port[port] + H9P_PORT_ENABLE, 0,
		H9P_PORT_ENABLE_APPLE);
	writel(H9P_PORT_PWRCTL_INIT, pcie->base_port[port] + H9P_PORT_PWRCTL);
	writel(port * 0x10001 * H9P_MSI_PER_PORT,
	       pcie->base_port[port] + H9P_PORT_MSIVECBASE);

	usleep_range(250, 1000);
	ret = apple_h9p_wait_gpio(pcie->clkreq[port], 0, 250000);
	if (ret)
		return dev_err_probe(dev, ret, "port %u CLKREQ# timeout\n",
				     port);

	gpiod_direction_output(pcie->perst[port], 1);
	usleep_range(250, 1000);

	ret = apple_h9p_wait(pcie->base_phy[1] + H9P_PHY1_PORTMASK,
			     BIT(port), BIT(port), BIT(port), 250000);
	if (ret)
		return dev_err_probe(dev, ret, "port %u PHY up timeout\n",
				     port);

	h9p_rmw(pcie->base_phy[2] + H9P_PHY2_EQ_COMMON0, 0, 0x4000);
	h9p_rmw(pcie->base_phy[2] + H9P_PHY2_EQ_COMMON1, 0, 0x4000);
	h9p_rmw(pcie->base_phy[2] + H9P_PHY2_EQ_TIME0, 0xfff, 100);
	h9p_rmw(pcie->base_phy[2] + H9P_PHY2_EQ_TIME1, 0xfff, 25);
	h9p_rmw(pcie->base_phy[2] + H9P_PHY2_PORT(port, H9P_PHY2_PORT_EQ_CTL),
		0, 0x4000);
	writel(0, pcie->base_phy[2] + H9P_PHY2_PORT(port, H9P_PHY2_PORT_IDLE));
	h9p_rmw(pcie->base_phy[2] +
		H9P_PHY2_PORT(port, H9P_PHY2_PORT_EQ_PRESET), 0xfff, 0x600);
	writel(0x3105, pcie->base_phy[2] +
	       H9P_PHY2_PORT(port, H9P_PHY2_PORT_RX_CTL0));
	h9p_rmw(pcie->base_phy[2] +
		H9P_PHY2_PORT(port, H9P_PHY2_PORT_RX_CTL1), 0xff, 0x9f);
	h9p_rmw(pcie->base_phy[2] +
		H9P_PHY2_PORT(port, H9P_PHY2_PORT_RX_CTL2), 0xff, 0x01);
	h9p_rmw(pcie->base_phy[2] +
		H9P_PHY2_PORT(port, H9P_PHY2_PORT_RX_CTL3), 0x1f, 0x0a);
	writel(175, pcie->base_phy[2] + H9P_PHY2_PORT(port, H9P_PHY2_PORT_TIMER0));
	writel(175, pcie->base_phy[2] + H9P_PHY2_PORT(port, H9P_PHY2_PORT_TIMER1));
	writel(333, pcie->base_phy[2] + H9P_PHY2_PORT(port, H9P_PHY2_PORT_TIMER2));
	writel(333, pcie->base_phy[2] + H9P_PHY2_PORT(port, H9P_PHY2_PORT_TIMER3));
	writel(530, pcie->base_phy[2] + H9P_PHY2_PORT(port, H9P_PHY2_PORT_TIMER4));
	writel(530, pcie->base_phy[2] + H9P_PHY2_PORT(port, H9P_PHY2_PORT_TIMER5));
	writel(0, pcie->base_phy[2] + H9P_PHY2_PORT(port, H9P_PHY2_PORT_CLEAR0));
	writel(0, pcie->base_phy[2] + H9P_PHY2_PORT(port, H9P_PHY2_PORT_CLEAR1));
	writel(0, pcie->base_phy[2] + H9P_PHY2_PORT(port, H9P_PHY2_PORT_CLEAR2));

	writel(H9P_PORT_IRQMASK_LINK_UP,
	       pcie->base_port[port] + H9P_PORT_IRQMASK);
	usleep_range(5000, 10000);

	h9p_rmw(pcie->base_port[port] + H9P_PORT_LTSSMCTL, 0,
		H9P_PORT_LTSSM_ENABLE);
	ret = apple_h9p_wait(pcie->base_port[port] + H9P_PORT_LINKSTS,
			     H9P_PORT_LINKSTS_LTSSM,
			     FIELD_PREP(H9P_PORT_LINKSTS_LTSSM,
					H9P_PORT_LTSSM_DETECT),
			     FIELD_PREP(H9P_PORT_LINKSTS_LTSSM,
					H9P_PORT_LTSSM_L0),
			     500000);
	if (ret)
		dev_warn(dev, "port %u link did not reach L0\n", port);

	return 0;
}

static int apple_h9p_setup_ports(struct apple_h9p_pcie *pcie)
{
	unsigned int port;
	int ret;

	writel(H9P_PHY0_COMMON_CTL_INIT,
	       pcie->base_phy[0] + H9P_PHY0_COMMON_CTL0);
	h9p_rmw(pcie->base_phy[0] + H9P_PHY0_PORT_CTL1(0), 0,
		H9P_PHY0_COMMON_CTL_ENABLE);

	ret = apple_h9p_wait(pcie->base_phy[0] + H9P_PHY0_COMMON_STAT,
			     H9P_PHY0_COMMON_STAT_INIT_DONE,
			     H9P_PHY0_COMMON_STAT_INIT_DONE,
			     H9P_PHY0_COMMON_STAT_INIT_DONE, 250000);
	if (ret)
		return dev_err_probe(pcie->dev, ret,
				     "global PHY init timeout\n");

	ret = apple_h9p_wait(pcie->base_phy[0] + H9P_PHY0_COMMON_STAT,
			     H9P_PHY0_COMMON_STAT_READY,
			     H9P_PHY0_COMMON_STAT_READY,
			     H9P_PHY0_COMMON_STAT_READY, 250000);
	if (ret)
		return dev_err_probe(pcie->dev, ret,
				     "global PHY ready timeout\n");

	writel(H9P_PHY0_COMMON_CTL_ENABLE,
	       pcie->base_phy[0] + H9P_PHY0_COMMON_CTL3);
	apple_h9p_apply_tunables(pcie->base_phy[0], h9p_phy0_tunables,
				 ARRAY_SIZE(h9p_phy0_tunables));
	writel(H9P_PHY0_COMMON_CTL_ENABLE,
	       pcie->base_phy[0] + H9P_PHY0_COMMON_CTL1);
	usleep_range(5000, 10000);
	writel(H9P_PHY0_COMMON_CTL_ENABLE,
	       pcie->base_phy[0] + H9P_PHY0_COMMON_CTL2);
	usleep_range(500, 1000);

	for (port = 0; port < H9P_NUM_PORTS; port++) {
		if (!(pcie->enabled_ports & BIT(port)))
			continue;

		ret = apple_h9p_setup_port(pcie, port);
		if (ret)
			return ret;
	}

	return 0;
}

static int apple_h9p_pcie_init(struct pci_config_window *cfg)
{
	struct apple_h9p_pcie *pcie = apple_h9p_pcie_lookup(cfg->parent);
	int ret;

	if (!pcie)
		return -ENODEV;

	pcie->cfgwin = cfg;
	pcie->base_config = cfg->win;

	ret = apple_h9p_pcieclk_postup(pcie);
	if (ret)
		return ret;

	ret = apple_h9p_setup_ports(pcie);
	if (ret)
		return ret;

	ret = apple_h9p_setup_nvmmu(pcie);
	return ret;
}

static const struct pci_ecam_ops apple_h9p_pcie_ecam_ops = {
	.bus_shift = 20,
	.init = apple_h9p_pcie_init,
	.pci_ops = {
		.map_bus = pci_ecam_map_bus,
		.read = apple_h9p_pcie_config_read,
		.write = apple_h9p_pcie_config_write,
	},
};

static int apple_h9p_pcie_map_resources(struct platform_device *pdev,
					struct apple_h9p_pcie *pcie)
{
	struct device *dev = &pdev->dev;
	unsigned int i;

	for (i = 0; i < 3; i++) {
		char name[8];

		snprintf(name, sizeof(name), "phy%u", i);
		pcie->base_phy[i] = devm_platform_ioremap_resource_byname(pdev,
									  name);
		if (IS_ERR(pcie->base_phy[i]))
			return PTR_ERR(pcie->base_phy[i]);
	}

	for (i = 0; i < H9P_NUM_PORTS; i++) {
		char name[8];
		struct resource *res;

		snprintf(name, sizeof(name), "port%u", i);
		pcie->base_port[i] = devm_platform_ioremap_resource_byname(pdev,
									   name);
		if (IS_ERR(pcie->base_port[i]))
			return PTR_ERR(pcie->base_port[i]);

		snprintf(name, sizeof(name), "nvmmu%u", i);
		res = platform_get_resource_byname(pdev, IORESOURCE_MEM, name);
		if (res) {
			pcie->nvmmu[i].base = devm_ioremap_resource(dev, res);
			if (IS_ERR(pcie->nvmmu[i].base))
				return PTR_ERR(pcie->nvmmu[i].base);
		}
	}

	pcie->base_pcieclk_postup =
		devm_platform_ioremap_resource_byname(pdev, "pcieclk-postup");
	if (IS_ERR(pcie->base_pcieclk_postup)) {
		if (PTR_ERR(pcie->base_pcieclk_postup) == -EINVAL)
			pcie->base_pcieclk_postup = NULL;
		else
			return dev_err_probe(dev,
					     PTR_ERR(pcie->base_pcieclk_postup),
					     "failed to map pcieclk post-up\n");
	}

	return 0;
}

static int apple_h9p_pcie_get_gpios(struct apple_h9p_pcie *pcie)
{
	struct device *dev = pcie->dev;
	unsigned int i;

	for (i = 0; i < H9P_NUM_PORTS; i++) {
		if (!(pcie->enabled_ports & BIT(i)))
			continue;

		pcie->perst[i] = devm_gpiod_get_index(dev, "reset", i,
						      GPIOD_OUT_LOW);
		if (IS_ERR(pcie->perst[i]))
			return dev_err_probe(dev, PTR_ERR(pcie->perst[i]),
					     "failed to get PERST#%u\n", i);

		pcie->clkreq[i] = devm_gpiod_get_index(dev, "clkreq", i,
						       GPIOD_IN);
		if (IS_ERR(pcie->clkreq[i]))
			return dev_err_probe(dev, PTR_ERR(pcie->clkreq[i]),
					     "failed to get CLKREQ#%u\n", i);
	}

	pcie->devpwr = devm_gpiod_get_array_optional(dev, "devpwr", GPIOD_ASIS);
	if (IS_ERR(pcie->devpwr))
		return dev_err_probe(dev, PTR_ERR(pcie->devpwr),
				     "failed to get device power GPIOs\n");

	return 0;
}

static int apple_h9p_pcie_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct pci_host_bridge *bridge;
	struct apple_h9p_pcie *pcie;
	int ret;

	bridge = devm_pci_alloc_host_bridge(dev, sizeof(*pcie));
	if (!bridge)
		return -ENOMEM;

	pcie = pci_host_bridge_priv(bridge);
	pcie->dev = dev;
	pcie->pdev = pdev;
	pcie->bridge = bridge;
	spin_lock_init(&pcie->used_msi_lock);

	ret = of_property_read_u32(dev->of_node, "apple,enabled-ports",
				   &pcie->enabled_ports);
	if (ret)
		pcie->enabled_ports = BIT(0);
	pcie->enabled_ports &= GENMASK(H9P_NUM_PORTS - 1, 0);
	if (!pcie->enabled_ports)
		return dev_err_probe(dev, -EINVAL, "no enabled ports\n");

	ret = dma_set_mask_and_coherent(dev, DMA_BIT_MASK(64));
	if (ret)
		return dev_err_probe(dev, ret, "failed to set DMA mask\n");

	ret = apple_h9p_pcie_attach_genpd(pcie);
	if (ret)
		return dev_err_probe(dev, ret, "failed to attach power domains\n");
	ret = devm_add_action_or_reset(dev, apple_h9p_pcie_genpd_cleanup, pcie);
	if (ret)
		return ret;

	pcie->clks[0].id = "core";
	pcie->clks[1].id = "aux";
	pcie->clks[2].id = "ref";
	ret = devm_clk_bulk_get(dev, ARRAY_SIZE(pcie->clks), pcie->clks);
	if (ret)
		return dev_err_probe(dev, ret, "failed to get clocks\n");

	ret = clk_bulk_prepare_enable(ARRAY_SIZE(pcie->clks), pcie->clks);
	if (ret)
		return dev_err_probe(dev, ret, "failed to enable clocks\n");
	ret = devm_add_action_or_reset(dev, apple_h9p_pcie_clk_cleanup, pcie);
	if (ret)
		return ret;

	pcie->pinctrl = devm_pinctrl_get_select_default(dev);
	if (PTR_ERR(pcie->pinctrl) == -ENODEV)
		pcie->pinctrl = NULL;
	else if (IS_ERR(pcie->pinctrl))
		return dev_err_probe(dev, PTR_ERR(pcie->pinctrl),
				     "failed to select pinctrl state\n");

	ret = apple_h9p_pcie_map_resources(pdev, pcie);
	if (ret)
		return ret;

	ret = apple_h9p_pcie_get_gpios(pcie);
	if (ret)
		return ret;

	ret = of_property_read_u64(dev->of_node, "apple,msi-doorbell",
				   &pcie->msi_doorbell);
	if (ret)
		pcie->msi_doorbell = H9P_DEFAULT_MSI_DOORBELL;

	ret = apple_h9p_pcie_setup_msi(pcie);
	if (ret)
		return dev_err_probe(dev, ret, "failed to set up MSI\n");

	return pci_host_common_init(pdev, bridge, &apple_h9p_pcie_ecam_ops);
}

static const struct of_device_id apple_h9p_pcie_of_match[] = {
	{ .compatible = "apple,t8010-pcie" },
	{ }
};
MODULE_DEVICE_TABLE(of, apple_h9p_pcie_of_match);

static struct platform_driver apple_h9p_pcie_driver = {
	.probe = apple_h9p_pcie_probe,
	.driver = {
		.name = "pcie-apple-h9p",
		.of_match_table = apple_h9p_pcie_of_match,
		.suppress_bind_attrs = true,
	},
};
module_platform_driver(apple_h9p_pcie_driver);

MODULE_DESCRIPTION("Apple H9P/T8010 PCIe host bridge driver");
MODULE_LICENSE("GPL");
