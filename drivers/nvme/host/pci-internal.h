/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Private nvme-pci structures shared with controller-specific glue.
 */

#ifndef _NVME_PCI_INTERNAL_H
#define _NVME_PCI_INTERNAL_H

#include <linux/blkdev.h>
#include <linux/blk-mq.h>
#include <linux/blk-mq-dma.h>
#include <linux/completion.h>
#include <linux/device.h>
#include <linux/mempool.h>
#include <linux/mutex.h>
#include <linux/pci.h>
#include <linux/spinlock.h>
#include <linux/types.h>

#include "nvme.h"

#define PCI_DEVICE_ID_APPLE_H9P_NVME	0x2002

#define NVME_MAX_NR_DESCRIPTORS	5

struct nvme_dev;
struct nvme_queue;

struct nvme_descriptor_pools {
	struct dma_pool *large;
	struct dma_pool *small;
};

struct nvme_pci_dma_ops {
	u32 quirks;
	int (*init)(struct nvme_dev *dev, int node);
	void (*exit)(struct nvme_dev *dev);
	int (*preinit)(struct nvme_dev *dev);
	int (*prepare_enable)(struct nvme_dev *dev);
	blk_status_t (*map_data)(struct request *req);
	bool (*unmap_data)(struct request *req);
	bool (*reuse_admin_irq)(struct nvme_dev *dev, struct pci_dev *pdev,
				struct nvme_queue *adminq);
	u32 (*queue_depth)(struct nvme_dev *dev, u32 depth);
	u32 (*max_hw_sectors)(struct nvme_dev *dev, u32 max_hw_sectors);
};

/*
 * Represents an NVM Express device.  Each nvme_dev is a PCI function.
 */
struct nvme_dev {
	struct nvme_queue *queues;
	struct blk_mq_tag_set tagset;
	struct blk_mq_tag_set admin_tagset;
	u32 __iomem *dbs;
	struct device *dev;
	unsigned int online_queues;
	unsigned int max_qid;
	unsigned int io_queues[HCTX_MAX_TYPES];
	unsigned int num_vecs;
	u32 q_depth;
	int io_sqes;
	u32 db_stride;
	void __iomem *bar;
	unsigned long bar_mapped_size;
	/* protects shutdown sequencing against reset and remove paths */
	struct mutex shutdown_lock;
	bool subsystem;
	u64 cmb_size;
	bool cmb_use_sqes;
	u32 cmbsz;
	u32 cmbloc;
	struct nvme_ctrl ctrl;
	u32 last_ps;
	bool hmb;
	struct sg_table *hmb_sgt;
	mempool_t *dmavec_mempool;
	const struct nvme_pci_dma_ops *dma_ops;
	void *dma_data;

	/* shadow doorbell buffer support: */
	__le32 *dbbuf_dbs;
	dma_addr_t dbbuf_dbs_dma_addr;
	__le32 *dbbuf_eis;
	dma_addr_t dbbuf_eis_dma_addr;

	/* host memory buffer support: */
	u64 host_mem_size;
	u32 nr_host_mem_descs;
	u32 host_mem_descs_size;
	dma_addr_t host_mem_descs_dma;
	struct nvme_host_mem_buf_desc *host_mem_descs;
	void **host_mem_desc_bufs;
	unsigned int nr_allocated_queues;
	unsigned int nr_write_queues;
	unsigned int nr_poll_queues;
	struct nvme_descriptor_pools descriptor_pools[];
};

/*
 * An NVM Express queue.  Each device has at least two (one for admin
 * commands and one for I/O commands).
 */
struct nvme_queue {
	struct nvme_dev *dev;
	struct nvme_descriptor_pools descriptor_pools;
	/* protects SQ tail updates */
	spinlock_t sq_lock;
	void *sq_cmds;
	/* protects CQ polling state; only used for poll queues */
	spinlock_t cq_poll_lock ____cacheline_aligned_in_smp;
	struct nvme_completion *cqes;
	dma_addr_t sq_dma_addr;
	dma_addr_t cq_dma_addr;
	u32 __iomem *q_db;
	u32 q_depth;
	u16 cq_vector;
	u16 sq_tail;
	u16 last_sq_tail;
	u16 cq_head;
	u16 qid;
	u8 cq_phase;
	u8 sqes;
	unsigned long flags;
#define NVMEQ_ENABLED		0
#define NVMEQ_SQ_CMB		1
#define NVMEQ_DELETE_ERROR	2
#define NVMEQ_POLLED		3
	__le32 *dbbuf_sq_db;
	__le32 *dbbuf_cq_db;
	__le32 *dbbuf_sq_ei;
	__le32 *dbbuf_cq_ei;
	struct completion delete_done;
};

static inline size_t nvme_pci_sq_size(const struct nvme_queue *q)
{
	return q->q_depth << q->sqes;
}

static inline size_t nvme_pci_cq_size(const struct nvme_queue *q)
{
	return q->q_depth * sizeof(struct nvme_completion);
}

/* bits for iod->flags */
enum nvme_iod_flags {
	/* this command has been aborted by the timeout handler */
	IOD_ABORTED		= 1U << 0,

	/* uses the small descriptor pool */
	IOD_SMALL_DESCRIPTOR	= 1U << 1,

	/* single segment dma mapping */
	IOD_SINGLE_SEGMENT	= 1U << 2,

	/* Data payload contains p2p memory */
	IOD_DATA_P2P		= 1U << 3,

	/* Metadata contains p2p memory */
	IOD_META_P2P		= 1U << 4,

	/* Data payload contains MMIO memory */
	IOD_DATA_MMIO		= 1U << 5,

	/* Metadata contains MMIO memory */
	IOD_META_MMIO		= 1U << 6,

	/* Metadata using non-coalesced MPTR */
	IOD_SINGLE_META_SEGMENT	= 1U << 7,
};

struct nvme_dma_vec {
	dma_addr_t addr;
	unsigned int len;
};

/*
 * The nvme_iod describes the data in an I/O.
 */
struct nvme_iod {
	struct nvme_request req;
	struct nvme_command cmd;
	u8 flags;
	u8 nr_descriptors;

	size_t total_len;
	struct dma_iova_state dma_state;
	void *descriptors[NVME_MAX_NR_DESCRIPTORS];
	struct nvme_dma_vec *dma_vecs;
	unsigned int nr_dma_vecs;

	dma_addr_t meta_dma;
	size_t meta_total_len;
	struct dma_iova_state meta_dma_state;
	struct nvme_sgl_desc *meta_descriptor;
	void *dma_private;
};

extern const struct nvme_pci_dma_ops nvme_pci_apple_h9p_ops;

#endif /* _NVME_PCI_INTERNAL_H */
