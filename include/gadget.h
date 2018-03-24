#ifndef __GADGET_H_
#define __GADGET_H_

#include <list.h>

struct usb_ep;

struct usb_request {
	void			*buf;
	unsigned		length;
	dma_addr_t		dma;

	unsigned		stream_id:16;
	unsigned		no_interrupt:1;
	unsigned		zero:1;
	unsigned		short_not_ok:1;

	void			(*complete)(struct usb_ep *ep,
					struct usb_request *req);
	void			*context;
	struct list_head	list;

	int			status;
	unsigned		actual;
};

struct usb_ep_ops {
	int (*enable) (struct usb_ep *ep,
		const struct usb_endpoint_descriptor *desc);
	int (*disable) (struct usb_ep *ep);

	struct usb_request *(*alloc_request) (struct usb_ep *ep,
		gfp_t gfp_flags);
	void (*free_request) (struct usb_ep *ep, struct usb_request *req);

	int (*queue) (struct usb_ep *ep, struct usb_request *req,
		gfp_t gfp_flags);
	int (*dequeue) (struct usb_ep *ep, struct usb_request *req);

	int (*set_halt) (struct usb_ep *ep, int value);
	int (*set_wedge)(struct usb_ep *ep);
	int (*fifo_status) (struct usb_ep *ep);
	void (*fifo_flush) (struct usb_ep *ep);
};

struct usb_ep {
	void			*driver_data;
	const char		*name;
	const struct usb_ep_ops	*ops;
	struct list_head	ep_list;
	unsigned		maxpacket:16;
	unsigned		maxpacket_limit:16;
	unsigned		max_streams:16;
	unsigned		maxburst:5;
	const struct usb_endpoint_descriptor	*desc;
	const struct usb_ss_ep_comp_descriptor	*comp_desc;
};

#endif
