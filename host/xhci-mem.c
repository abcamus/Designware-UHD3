#include <xhci.h>
#include <usb.h>
#include <misc.h>
#include <errno.h>

#define CACHELINE_SIZE				CONFIG_SYS_CACHELINE_SIZE

static void *xhci_malloc(unsigned int size)
{
	void *ptr;
	size_t cacheline_size = max(XHCI_ALIGNMENT, CACHELINE_SIZE);

	//TODO: allo ptr
	/*
	ptr = memalign(cacheline_size, ALIGN(size, cacheline_size));
	BUG_ON(!ptr);
	memset(ptr, '\0', size);

	xhci_flush_cache((uintptr_t)ptr, size);
	*/
#ifndef EMBEDDED_PORT
	ptr = malloc(size);
	if (!ptr)
		debug("%s:%d: malloc error.\n", __func__, __LINE__);
#endif

	return ptr;
}

static struct xhci_segment *xhci_segment_alloc(void)
{
	struct xhci_segment *seg;

	//TODO: alloc segment
	//seg = (struct xhci_segment *)malloc(sizeof(struct xhci_segment));
	//BUG_ON(!seg);
#ifdef EMBEDDED_PORT
	seg = (struct xhci_segment *)SEG_ADDR;
#else
	seg = (struct xhci_segment *)malloc(sizeof(struct xhci_segment));
#endif

	//TODO: alloc your own trbs
	//seg->trbs = (union xhci_trb *)xhci_malloc(SEGMENT_SIZE);
#ifdef EMBEDDED_PORT
seg->trbs = (union xhci_trb *)SEGMENT_BASE;
#else
seg->trbs = (union xhci_trb *)malloc(SEGMENT_SIZE);
#endif

	seg->next = NULL;

	return seg;
}

static void xhci_link_segments(struct xhci_segment *prev,
				struct xhci_segment *next, bool link_trbs)
{
	u32 val;
	u64 val_64 = 0;

	if (!prev || !next)
		return;
	prev->next = next;
	if (link_trbs) {
		val_64 = (uintptr_t)next->trbs;
		prev->trbs[TRBS_PER_SEGMENT-1].link.segment_ptr = val_64;

		/*
		 * Set the last TRB in the segment to
		 * have a TRB type ID of Link TRB
		 */
		val = le32_to_cpu(prev->trbs[TRBS_PER_SEGMENT-1].link.control);
		val &= ~TRB_TYPE_BITMASK;
		val |= (TRB_LINK << TRB_TYPE_SHIFT);

		prev->trbs[TRBS_PER_SEGMENT-1].link.control = cpu_to_le32(val);
	}
}

static void xhci_initialize_ring_info(struct xhci_ring *ring)
{
	/*
	 * The ring is empty, so the enqueue pointer == dequeue pointer
	 */
	ring->enqueue = ring->first_seg->trbs;
	ring->enq_seg = ring->first_seg;
	ring->dequeue = ring->enqueue;
	ring->deq_seg = ring->first_seg;

	/*
	 * The ring is initialized to 0. The producer must write 1 to the
	 * cycle bit to handover ownership of the TRB, so PCS = 1.
	 * The consumer must compare CCS to the cycle bit to
	 * check ownership, so CCS = 1.
	 */
	ring->cycle_state = 1;
}

struct xhci_ring *xhci_ring_alloc(unsigned int num_segs, bool link_trbs)
{
	struct xhci_ring *ring;
	struct xhci_segment *prev;

	//TODO: alloc ring
	//ring = (struct xhci_ring *)malloc(sizeof(struct xhci_ring));
	//BUG_ON(!ring);
#ifdef ENBEDDED_PORT
	ring = (struct xhci_ring *)TEST_ADDR;
#else
	ring = (struct xhci_ring *)malloc(sizeof(struct xhci_ring));
#endif

	if (num_segs == 0)
		return ring;

	ring->first_seg = xhci_segment_alloc();
	//BUG_ON(!ring->first_seg);

	num_segs--;

	prev = ring->first_seg;
	while (num_segs > 0) {
		struct xhci_segment *next;

		debug("%s:%d.\n", __func__, __LINE__);
		next = xhci_segment_alloc();
		//BUG_ON(!next);

		xhci_link_segments(prev, next, link_trbs);

		prev = next;
		num_segs--;
	}

	debug("%s:%d.\n", __func__, __LINE__);
	xhci_link_segments(prev, ring->first_seg, link_trbs);
	if (link_trbs) {
		/* See section 4.9.2.1 and 6.4.4.1 */
		prev->trbs[TRBS_PER_SEGMENT-1].link.control |=
					cpu_to_le32(LINK_TOGGLE);
	}

	debug("%s:%d.\n", __func__, __LINE__);
	xhci_initialize_ring_info(ring);

	return ring;
}

/**
 * Allocates the Container context
 *
 * @param ctrl	Host controller data structure
 * @param type type of XHCI Container Context
 * @return NULL if failed else pointer to the context on success
 */
static struct xhci_container_ctx
		*xhci_alloc_container_ctx(struct xhci_ctrl *ctrl, int type)
{
	struct xhci_container_ctx *ctx;

	ctx = (struct xhci_container_ctx *)
		malloc(sizeof(struct xhci_container_ctx));
	BUG_ON(!ctx);

	BUG_ON((type != XHCI_CTX_TYPE_DEVICE) && (type != XHCI_CTX_TYPE_INPUT));
	ctx->type = type;
	ctx->size = (MAX_EP_CTX_NUM + 1) *
			CTX_SIZE(readl(&ctrl->hccr->cr_hccparams));
	if (type == XHCI_CTX_TYPE_INPUT)
		ctx->size += CTX_SIZE(readl(&ctrl->hccr->cr_hccparams));

	ctx->bytes = (u8 *)xhci_malloc(ctx->size);

	return ctx;
}

/**
 * invalidates the address passed till the length
 *
 * @param addr	pointer to memory region to be invalidates
 * @param len	the length of the cache line to be invalidated
 * @return none
 */
void xhci_inval_cache(uintptr_t addr, u32 len)
{
#if 0
	BUG_ON((void *)addr == NULL || len == 0);

	invalidate_dcache_range(addr & ~(CACHELINE_SIZE - 1),
				ALIGN(addr + len, CACHELINE_SIZE));
#endif
	return;
}

/**
 * Allocating virtual device
 *
 * @param udev	pointer to USB deivce structure
 * @return 0 on success else -1 on failure
 */
int xhci_alloc_virt_device(struct xhci_ctrl *ctrl, unsigned int slot_id)
{
	u64 byte_64 = 0;
	struct xhci_virt_device *virt_dev;

	/* Slot ID 0 is reserved */
	if (ctrl->devs[slot_id]) {
		printf("Virt dev for slot[%d] already allocated\n", slot_id);
		return -EEXIST;
	}

	//TODO: define platform specific devs
	ctrl->devs[slot_id] = (struct xhci_virt_device *)
					malloc(sizeof(struct xhci_virt_device));

	if (!ctrl->devs[slot_id]) {
		puts("Failed to allocate virtual device\n");
		return -ENOMEM;
	}

	memset(ctrl->devs[slot_id], 0, sizeof(struct xhci_virt_device));
	virt_dev = ctrl->devs[slot_id];

	/* Allocate the (output) device context that will be used in the HC. */
	virt_dev->out_ctx = xhci_alloc_container_ctx(ctrl,
					XHCI_CTX_TYPE_DEVICE);
	if (!virt_dev->out_ctx) {
		puts("Failed to allocate out context for virt dev\n");
		return -ENOMEM;
	}

	/* Allocate the (input) device context for address device command */
	virt_dev->in_ctx = xhci_alloc_container_ctx(ctrl,
					XHCI_CTX_TYPE_INPUT);
	if (!virt_dev->in_ctx) {
		puts("Failed to allocate in context for virt dev\n");
		return -ENOMEM;
	}

	/* Allocate endpoint 0 ring */
	virt_dev->eps[0].ring = xhci_ring_alloc(1, true);

	byte_64 = (uintptr_t)(virt_dev->out_ctx->bytes);

	/* Point to output device context in dcbaa. */
	ctrl->dcbaa->dev_context_ptrs[slot_id] = byte_64;

	xhci_flush_cache((uintptr_t)&ctrl->dcbaa->dev_context_ptrs[slot_id],
			 sizeof(__le64));
	return 0;
}

void xhci_flush_cache(uintptr_t addr, u32 len)
{
}

int xhci_mem_init(struct xhci_ctrl *ctrl, struct xhci_hccr *hccr,
					struct xhci_hcor *hcor)
{
	uint64_t val_64;
	uint64_t trb_64;
	uint32_t val;
	unsigned long deq;
	int i;
	struct xhci_segment *seg;

	/* DCBAA initialization */
	//TODO: platform specific dcbaa
	/*
	ctrl->dcbaa = (struct xhci_device_context_array *)
			xhci_malloc(sizeof(struct xhci_device_context_array));
	*/
#ifdef EMBEDDED_PORT
	//TODO: specify DCBAA addr
	ctrl->dcbaa = (struct xhci_device_context_array *)DCBAA_ADDR;
#else
	ctrl->dcbaa = malloc(sizeof(struct xhci_device_context_array));
#endif
	if (ctrl->dcbaa == NULL) {
		puts("unable to allocate DCBA\n");
		return -ENOMEM;
	}

	val_64 = (uintptr_t)ctrl->dcbaa;
	/* Set the pointer in DCBAA register */
	xhci_writeq(&hcor->or_dcbaap, val_64);

	/* Command ring control pointer register initialization */
	ctrl->cmd_ring = xhci_ring_alloc(1, true);

	/* Set the address in the Command Ring Control register */
	trb_64 = (uintptr_t)ctrl->cmd_ring->first_seg->trbs;

	val_64 = xhci_readq(&hcor->or_crcr);
	val_64 = (val_64 & (u64) CMD_RING_RSVD_BITS) |
		(trb_64 & (u64) ~CMD_RING_RSVD_BITS) |
		ctrl->cmd_ring->cycle_state;
	xhci_writeq(&hcor->or_crcr, val_64);

	/* write the address of db register */
	val = xhci_readl(&hccr->cr_dboff);
	val &= DBOFF_MASK;
	ctrl->dba = (struct xhci_doorbell_array *)((char *)hccr + val);

	/* write the address of runtime register */
	val = xhci_readl(&hccr->cr_rtsoff);
	val &= RTSOFF_MASK;
#ifdef LOCAL_SIM
	val = 0x100;
	debug("run regs off = 0x%x.\n", val);
#endif
	ctrl->run_regs = (struct xhci_run_regs *)((char *)hccr + val);

	/* writting the address of ir_set structure */
	ctrl->ir_set = &ctrl->run_regs->ir_set[0];

	/* Event ring does not maintain link TRB */
	ctrl->event_ring = xhci_ring_alloc(ERST_NUM_SEGS, false);

	debug("ERST NUM SEGS = %d.\n", ERST_NUM_SEGS);
	/*
	ctrl->erst.entries = (struct xhci_erst_entry *)
		xhci_malloc(sizeof(struct xhci_erst_entry) * ERST_NUM_SEGS);
	*/
	ctrl->erst.entries = (struct xhci_erst_entry *)malloc(sizeof(struct xhci_erst_entry));

	ctrl->erst.num_entries = ERST_NUM_SEGS;

	for (val = 0, seg = ctrl->event_ring->first_seg;
			val < ERST_NUM_SEGS;
			val++) {
		trb_64 = 0;
		trb_64 = (uintptr_t)seg->trbs;
		struct xhci_erst_entry *entry = &ctrl->erst.entries[val];
		xhci_writeq(&entry->seg_addr, trb_64);
		entry->seg_size = cpu_to_le32(TRBS_PER_SEGMENT);
		entry->rsvd = 0;
		seg = seg->next;
	}

	xhci_flush_cache((uintptr_t)ctrl->erst.entries,
			 ERST_NUM_SEGS * sizeof(struct xhci_erst_entry));

	deq = (unsigned long)ctrl->event_ring->dequeue;

	/* Update HC event ring dequeue pointer */
	xhci_writeq(&ctrl->ir_set->erst_dequeue,
				(u64)deq & (u64)~ERST_PTR_MASK);

	/* set ERST count with the number of entries in the segment table */
	val = xhci_readl(&ctrl->ir_set->erst_size);

	val &= ERST_SIZE_MASK;
	val |= ERST_NUM_SEGS;
	xhci_writel(&ctrl->ir_set->erst_size, val);

	/* this is the event ring segment table pointer */
	val_64 = xhci_readq(&ctrl->ir_set->erst_base);
	val_64 &= ERST_PTR_MASK;
	val_64 |= ((uintptr_t)(ctrl->erst.entries) & ~ERST_PTR_MASK);

	xhci_writeq(&ctrl->ir_set->erst_base, val_64);

	/* initializing the virtual devices to NULL */
	for (i = 0; i < MAX_HC_SLOTS; ++i)
		ctrl->devs[i] = NULL;

	/*
	 * Just Zero'ing this register completely,
	 * or some spurious Device Notification Events
	 * might screw things here.
	 */
	xhci_writel(&hcor->or_dnctrl, 0x0);

	return 0;
}

/**
 * Setup an xHCI virtual device for a Set Address command
 *
 * @param udev pointer to the Device Data Structure
 * @return returns negative value on failure else 0 on success
 */
void xhci_setup_addressable_virt_dev(struct xhci_ctrl *ctrl, int slot_id,
				     int speed, int hop_portnr)
{
	struct xhci_virt_device *virt_dev;
	struct xhci_ep_ctx *ep0_ctx;
	struct xhci_slot_ctx *slot_ctx;
	u32 port_num = 0;
	u64 trb_64 = 0;

	virt_dev = ctrl->devs[slot_id];

	BUG_ON(!virt_dev);

	/* Extract the EP0 and Slot Ctrl */
	ep0_ctx = xhci_get_ep_ctx(ctrl, virt_dev->in_ctx, 0);
	slot_ctx = xhci_get_slot_ctx(ctrl, virt_dev->in_ctx);

	/* Only the control endpoint is valid - one endpoint context */
	slot_ctx->dev_info |= cpu_to_le32(LAST_CTX(1) | 0);

	switch (speed) {
	case USB_SPEED_SUPER:
		slot_ctx->dev_info |= cpu_to_le32(SLOT_SPEED_SS);
		break;
	case USB_SPEED_HIGH:
		slot_ctx->dev_info |= cpu_to_le32(SLOT_SPEED_HS);
		break;
	case USB_SPEED_FULL:
		slot_ctx->dev_info |= cpu_to_le32(SLOT_SPEED_FS);
		break;
	case USB_SPEED_LOW:
		slot_ctx->dev_info |= cpu_to_le32(SLOT_SPEED_LS);
		break;
	default:
		/* Speed was set earlier, this shouldn't happen. */
		BUG();
	}

	port_num = hop_portnr;
	debug("port_num = %d\n", port_num);

	slot_ctx->dev_info2 |=
			cpu_to_le32(((port_num & ROOT_HUB_PORT_MASK) <<
				ROOT_HUB_PORT_SHIFT));

	/* Step 4 - ring already allocated */
	/* Step 5 */
	ep0_ctx->ep_info2 = cpu_to_le32(CTRL_EP << EP_TYPE_SHIFT);
	debug("SPEED = %d\n", speed);

	switch (speed) {
	case USB_SPEED_SUPER:
		ep0_ctx->ep_info2 |= cpu_to_le32(((512 & MAX_PACKET_MASK) <<
					MAX_PACKET_SHIFT));
		debug("Setting Packet size = 512bytes\n");
		break;
	case USB_SPEED_HIGH:
	/* USB core guesses at a 64-byte max packet first for FS devices */
	case USB_SPEED_FULL:
		ep0_ctx->ep_info2 |= cpu_to_le32(((64 & MAX_PACKET_MASK) <<
					MAX_PACKET_SHIFT));
		debug("Setting Packet size = 64bytes\n");
		break;
	case USB_SPEED_LOW:
		ep0_ctx->ep_info2 |= cpu_to_le32(((8 & MAX_PACKET_MASK) <<
					MAX_PACKET_SHIFT));
		debug("Setting Packet size = 8bytes\n");
		break;
	default:
		/* New speed? */
		BUG();
	}

	/* EP 0 can handle "burst" sizes of 1, so Max Burst Size field is 0 */
	ep0_ctx->ep_info2 |=
			cpu_to_le32(((0 & MAX_BURST_MASK) << MAX_BURST_SHIFT) |
			((3 & ERROR_COUNT_MASK) << ERROR_COUNT_SHIFT));

	trb_64 = (uintptr_t)virt_dev->eps[0].ring->first_seg->trbs;
	ep0_ctx->deq = cpu_to_le64(trb_64 | virt_dev->eps[0].ring->cycle_state);

	/* Steps 7 and 8 were done in xhci_alloc_virt_device() */

	xhci_flush_cache((uintptr_t)ep0_ctx, sizeof(struct xhci_ep_ctx));
	xhci_flush_cache((uintptr_t)slot_ctx, sizeof(struct xhci_slot_ctx));
}

/**
 * Give the input control context for the passed container context
 *
 * @param ctx	pointer to the context
 * @return pointer to the Input control context data
 */
struct xhci_input_control_ctx
		*xhci_get_input_control_ctx(struct xhci_container_ctx *ctx)
{
	BUG_ON(ctx->type != XHCI_CTX_TYPE_INPUT);
	return (struct xhci_input_control_ctx *)ctx->bytes;
}

/**
 * Give the slot context for the passed container context
 *
 * @param ctrl	Host controller data structure
 * @param ctx	pointer to the context
 * @return pointer to the slot control context data
 */
struct xhci_slot_ctx *xhci_get_slot_ctx(struct xhci_ctrl *ctrl,
				struct xhci_container_ctx *ctx)
{
	if (ctx->type == XHCI_CTX_TYPE_DEVICE)
		return (struct xhci_slot_ctx *)ctx->bytes;

	return (struct xhci_slot_ctx *)
		(ctx->bytes + CTX_SIZE(readl(&ctrl->hccr->cr_hccparams)));
}

/**
 * Copy output xhci_slot_ctx to the input xhci_slot_ctx.
 * Useful when you want to change one particular aspect of the endpoint
 * and then issue a configure endpoint command.
 * Only the context entries field matters, but
 * we'll copy the whole thing anyway.
 *
 * @param ctrl	Host controller data structure
 * @param in_ctx contains the inpout context
 * @param out_ctx contains the inpout context
 * @return none
 */
void xhci_slot_copy(struct xhci_ctrl *ctrl, struct xhci_container_ctx *in_ctx,
					struct xhci_container_ctx *out_ctx)
{
	struct xhci_slot_ctx *in_slot_ctx;
	struct xhci_slot_ctx *out_slot_ctx;

	in_slot_ctx = xhci_get_slot_ctx(ctrl, in_ctx);
	out_slot_ctx = xhci_get_slot_ctx(ctrl, out_ctx);

	in_slot_ctx->dev_info = out_slot_ctx->dev_info;
	in_slot_ctx->dev_info2 = out_slot_ctx->dev_info2;
	in_slot_ctx->tt_info = out_slot_ctx->tt_info;
	in_slot_ctx->dev_state = out_slot_ctx->dev_state;
}

/**
 * Copy output xhci_ep_ctx to the input xhci_ep_ctx copy.
 * Useful when you want to change one particular aspect of the endpoint
 * and then issue a configure endpoint command.
 *
 * @param ctrl	Host controller data structure
 * @param in_ctx contains the input context
 * @param out_ctx contains the input context
 * @param ep_index index of the end point
 * @return none
 */
void xhci_endpoint_copy(struct xhci_ctrl *ctrl,
			struct xhci_container_ctx *in_ctx,
			struct xhci_container_ctx *out_ctx,
			unsigned int ep_index)
{
	struct xhci_ep_ctx *out_ep_ctx;
	struct xhci_ep_ctx *in_ep_ctx;

	out_ep_ctx = xhci_get_ep_ctx(ctrl, out_ctx, ep_index);
	in_ep_ctx = xhci_get_ep_ctx(ctrl, in_ctx, ep_index);

	in_ep_ctx->ep_info = out_ep_ctx->ep_info;
	in_ep_ctx->ep_info2 = out_ep_ctx->ep_info2;
	in_ep_ctx->deq = out_ep_ctx->deq;
	in_ep_ctx->tx_info = out_ep_ctx->tx_info;
}

/**
 * Gets the EP context from based on the ep_index
 *
 * @param ctrl	Host controller data structure
 * @param ctx	context container
 * @param ep_index	index of the endpoint
 * @return pointer to the End point context
 */
struct xhci_ep_ctx *xhci_get_ep_ctx(struct xhci_ctrl *ctrl,
				    struct xhci_container_ctx *ctx,
				    unsigned int ep_index)
{
	/* increment ep index by offset of start of ep ctx array */
	ep_index++;
	if (ctx->type == XHCI_CTX_TYPE_INPUT)
		ep_index++;

	return (struct xhci_ep_ctx *)
		(ctx->bytes +
		(ep_index * CTX_SIZE(readl(&ctrl->hccr->cr_hccparams))));
}
