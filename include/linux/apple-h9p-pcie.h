/* SPDX-License-Identifier: GPL-2.0 */
#ifndef _LINUX_APPLE_H9P_PCIE_H
#define _LINUX_APPLE_H9P_PCIE_H

#include <linux/dma-mapping.h>
#include <linux/errno.h>
#include <linux/types.h>

struct device;

#define APPLE_H9P_NVMMU_MAX_REQS	36
#define APPLE_H9P_NVMMU_MAX_PAGES	256
#define APPLE_H9P_NVMMU_PAGE_SIZE	4096

#if IS_REACHABLE(CONFIG_PCIE_APPLE_H9P)
int apple_h9p_pcie_map_nvmmu(struct device *dev, unsigned int tag,
			     const u64 *pages, unsigned int npages,
			     dma_addr_t *iova);
#else
static inline int apple_h9p_pcie_map_nvmmu(struct device *dev,
					   unsigned int tag, const u64 *pages,
					   unsigned int npages, dma_addr_t *iova)
{
	return -EOPNOTSUPP;
}
#endif

#endif /* _LINUX_APPLE_H9P_PCIE_H */
