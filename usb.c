#include <usb.h>
#include <dwc3.h>
#include <errno.h>
#include <delay.h>
#include <misc.h>
#include <io.h>
#include <xhci.h>
#include <ctype.h>

#include <blk.h>

static int usb_stor_curr_dev = -1;

#ifndef CONFIG_USB_MAX_CONTROLLER_COUNT
#define CONFIG_USB_MAX_CONTROLLER_COUNT 1
#endif

static LIST_HEAD(dwc3_list);
/* -------------------------------------------------------------------------- */

__weak void set_usbdrd_phy_ctrl() 
{
	return;
}

__weak void xxxx_usb3_phy_init()
{
	return;
}

static inline void *ERR_PTR(long error)
{
	return (void *) error;
}

static struct dwc3_event_buffer *dwc3_alloc_one_event_buffer(struct dwc3 *dwc,
		unsigned length)
{
	struct dwc3_event_buffer	*evt;

	//evt = devm_kzalloc(dwc->dev, sizeof(*evt), GFP_KERNEL);
	//TODO: alloc evt
#ifdef EMBEDDED_PORT
evt = (struct dwc3_event_buffer *)EVT_ADDR;	
#else
	evt = evt_base;
#endif
	if (!evt)
		return ERR_PTR(-ENOMEM);

	debug("%s:%d.\n", __func__, __LINE__);
	evt->dwc	= dwc;
	evt->length	= length;
	/*
	evt->buf	= dma_alloc_coherent(length,
					     (unsigned long *)&evt->dma);
	*/
	//TODO: alloc evt buffer
#ifdef EMBEDDED_PORT
evt->buf = (void *)EVT_BUF_ADDR;
#else
evt->buf = malloc(length);
evt->dma = (u64)evt->buf;
#endif
	if (!evt->buf)
		return ERR_PTR(-ENOMEM);

	return evt;
}

static int dwc3_alloc_event_buffers(struct dwc3 *dwc, unsigned length)
{
	int			num;
	int			i;

	num = DWC3_NUM_INT(dwc->hwparams.hwparams1);
	dwc->num_event_buffers = num;

	/*
	dwc->ev_buffs = memalign(CONFIG_SYS_CACHELINE_SIZE,
				 sizeof(*dwc->ev_buffs) * num);
	*/
	//TODO: alloc ev_buffs
#ifdef LOCAL_SIM
	num = 1;
	dwc->ev_buffs = malloc(sizeof(*dwc->ev_buffs)*num);
	dwc->num_event_buffers = num;
#endif
	if (!dwc->ev_buffs)
		return -ENOMEM;

	for (i = 0; i < num; i++) {
		struct dwc3_event_buffer	*evt;

		evt = dwc3_alloc_one_event_buffer(dwc, length);
		dwc->ev_buffs[i] = evt;
	}

	return 0;
}

static void dwc3_cache_hwparams(struct dwc3 *dwc)
{
	struct dwc3_hwparams	*parms = &dwc->hwparams;

	parms->hwparams0 = dwc3_readl(dwc->regs, DWC3_GHWPARAMS0);
	parms->hwparams1 = dwc3_readl(dwc->regs, DWC3_GHWPARAMS1);
	parms->hwparams2 = dwc3_readl(dwc->regs, DWC3_GHWPARAMS2);
	parms->hwparams3 = dwc3_readl(dwc->regs, DWC3_GHWPARAMS3);
	parms->hwparams4 = dwc3_readl(dwc->regs, DWC3_GHWPARAMS4);
	parms->hwparams5 = dwc3_readl(dwc->regs, DWC3_GHWPARAMS5);
	parms->hwparams6 = dwc3_readl(dwc->regs, DWC3_GHWPARAMS6);
	parms->hwparams7 = dwc3_readl(dwc->regs, DWC3_GHWPARAMS7);
	parms->hwparams8 = dwc3_readl(dwc->regs, DWC3_GHWPARAMS8);
}

static int dwc3_event_buffers_setup(struct dwc3 *dwc)
{
	struct dwc3_event_buffer	*evt;
	int				n;

	debug("num event buffers = %d.\n", dwc->num_event_buffers);
	for (n = 0; n < dwc->num_event_buffers; n++) {
		evt = dwc->ev_buffs[n];
		dev_dbg(dwc->dev, "Event buf %p dma %08llx length %d\n",
				evt->buf, (unsigned long long) evt->dma,
				evt->length);

		evt->lpos = 0;

		dwc3_writel(dwc->regs, DWC3_GEVNTADRLO(n),
				lower_32_bits(evt->dma));
		dwc3_writel(dwc->regs, DWC3_GEVNTADRHI(n),
				upper_32_bits(evt->dma));
		dwc3_writel(dwc->regs, DWC3_GEVNTSIZ(n),
				DWC3_GEVNTSIZ_SIZE(evt->length));
		dwc3_writel(dwc->regs, DWC3_GEVNTCOUNT(n), 0);
	}
	debug("%s:%d.\n", __func__, __LINE__);

	return 0;
}

static int dwc3_core_init_mode(struct dwc3 *dwc)
{
	int ret;

	switch (dwc->dr_mode) {
	case USB_DR_MODE_PERIPHERAL:
		dwc3_set_mode(dwc, DWC3_GCTL_PRTCAP_DEVICE);
		//TODO: enable this when device mode
#ifdef GADGET_TEST
		ret = dwc3_gadget_init(dwc);
		if (ret) {
			dev_err(dev, "failed to initialize gadget\n");
			return ret;
		}
#endif
		break;
	case USB_DR_MODE_HOST:
		dwc3_set_mode(dwc, DWC3_GCTL_PRTCAP_HOST);
		ret = dwc3_host_init(dwc);
		if (ret) {
			dev_err(dev, "failed to initialize host\n");
			return ret;
		}
		break;
	case USB_DR_MODE_OTG:
		dwc3_set_mode(dwc, DWC3_GCTL_PRTCAP_OTG);
		//TODO: enable this when otg mode
#ifdef OTG_TEST
		ret = dwc3_host_init(dwc);
		if (ret) {
			dev_err(dev, "failed to initialize host\n");
			return ret;
		}

		ret = dwc3_gadget_init(dwc);
		if (ret) {
			dev_err(dev, "failed to initialize gadget\n");
			return ret;
		}
#endif
		break;
	default:
		dev_err(dev, "Unsupported mode of operation %d\n", dwc->dr_mode);
		return -EINVAL;
	}

	return 0;
}

int dwc3_xxx_init(struct dwc3_device *dwc3_dev)
{
	struct dwc3		*dwc;
	u8			lpm_nyet_threshold;
	u8			tx_de_emphasis;
	u8			hird_threshold;

	int			ret;

	void			*mem;

	//mem = devm_kzalloc(dev, sizeof(*dwc) + DWC3_ALIGN_MASK, GFP_KERNEL);
	//TODO: alloc mem
#ifndef EMBEDDED_PORT
	mem = g_mem;
#endif
	if (!mem)
		return -ENOMEM;

#ifndef EMBEDDED_PORT
	dwc = dwc_base;
#else
	//TODO: alloc dwc
	//dwc = PTR_ALIGN(mem, DWC3_ALIGN_MASK + 1);
#endif
	dwc->mem = mem;

	dwc->regs = (void *)(uintptr_t)(dwc3_dev->base +
					DWC3_GLOBALS_REGS_START);

	/* default to highest possible threshold */
	lpm_nyet_threshold = 0xff;

	/* default to -3.5dB de-emphasis */
	tx_de_emphasis = 1;

	/*
	 * default to assert utmi_sleep_n and use maximum allowed HIRD
	 * threshold value of 0b1100
	 */
	hird_threshold = 12;

	dwc->maximum_speed = dwc3_dev->maximum_speed;
	dwc->has_lpm_erratum = dwc3_dev->has_lpm_erratum;
	if (dwc3_dev->lpm_nyet_threshold)
		lpm_nyet_threshold = dwc3_dev->lpm_nyet_threshold;
	dwc->is_utmi_l1_suspend = dwc3_dev->is_utmi_l1_suspend;
	if (dwc3_dev->hird_threshold)
		hird_threshold = dwc3_dev->hird_threshold;

	dwc->needs_fifo_resize = dwc3_dev->tx_fifo_resize;
	dwc->dr_mode = dwc3_dev->dr_mode;

	dwc->disable_scramble_quirk = dwc3_dev->disable_scramble_quirk;
	dwc->u2exit_lfps_quirk = dwc3_dev->u2exit_lfps_quirk;
	dwc->u2ss_inp3_quirk = dwc3_dev->u2ss_inp3_quirk;
	dwc->req_p1p2p3_quirk = dwc3_dev->req_p1p2p3_quirk;
	dwc->del_p1p2p3_quirk = dwc3_dev->del_p1p2p3_quirk;
	dwc->del_phy_power_chg_quirk = dwc3_dev->del_phy_power_chg_quirk;
	dwc->lfps_filter_quirk = dwc3_dev->lfps_filter_quirk;
	dwc->rx_detect_poll_quirk = dwc3_dev->rx_detect_poll_quirk;
	dwc->dis_u3_susphy_quirk = dwc3_dev->dis_u3_susphy_quirk;
	dwc->dis_u2_susphy_quirk = dwc3_dev->dis_u2_susphy_quirk;

	dwc->tx_de_emphasis_quirk = dwc3_dev->tx_de_emphasis_quirk;
	if (dwc3_dev->tx_de_emphasis)
		tx_de_emphasis = dwc3_dev->tx_de_emphasis;

	/* default to superspeed if no maximum_speed passed */
	if (dwc->maximum_speed == USB_SPEED_UNKNOWN)
		dwc->maximum_speed = USB_SPEED_SUPER;

	dwc->lpm_nyet_threshold = lpm_nyet_threshold;
	dwc->tx_de_emphasis = tx_de_emphasis;

	dwc->hird_threshold = hird_threshold
		| (dwc->is_utmi_l1_suspend << 4);

	dwc->index = dwc3_dev->index;

	dwc3_cache_hwparams(dwc);

	ret = dwc3_alloc_event_buffers(dwc, DWC3_EVENT_BUFFERS_SIZE);
	if (ret) {
		dev_err(dwc->dev, "failed to allocate event buffers\n");
		return -ENOMEM;
	}

	//TODO: default dr_mode is HOST MODE, modify yourself
	dwc->dr_mode = USB_DR_MODE_HOST;

	if (dwc->dr_mode == USB_DR_MODE_UNKNOWN)
		dwc->dr_mode = USB_DR_MODE_OTG;

	ret = dwc3_core_init(dwc);
	if (ret) {
		dev_err(dev, "failed to initialize core\n");
		goto err0;
	}

	debug("%s:%d.\n", __func__, __LINE__);
	ret = dwc3_event_buffers_setup(dwc);
	if (ret) {
		dev_err(dwc->dev, "failed to setup event buffers\n");
		goto err1;
	}

	debug("%s:%d.\n", __func__, __LINE__);
	ret = dwc3_core_init_mode(dwc);
	if (ret)
		goto err2;

	list_add_tail(&dwc->list, &dwc3_list);

	return 0;

err2:
	//TODO: platform specific
	//dwc3_event_buffers_cleanup(dwc);

err1:
	//TODO: platform specific
	//dwc3_core_exit(dwc);

err0:
	//TODO: platform specific
	//dwc3_free_event_buffers(dwc);

	return ret;
}

static struct dwc3_device dwc3_device_data = {
	.maximum_speed = USB_SPEED_SUPER,
	//TODO:
#ifdef EMBEDDED_PORT
	.base = 0x12400000,
#endif
	.dr_mode = USB_DR_MODE_PERIPHERAL,
	.index = 0,
};

/*
 * Called from xhci_hcd_init() in host/xhci-dev.c
 */
int board_usb_init(int index, enum usb_init_type init)
{
#if 0
	void *ctrl;
	int ret;
	set_usbdrd_phy_ctrl();
	xxxx_usb3_phy_init();
	debug("%s:%d.\n", __func__, __LINE__);
#ifndef ENBEDDED_PORT
	dwc3_device_data.base = dev_data_base;
#endif
	return dwc3_xxx_init(&dwc3_device_data);
#else
	return 0;
#endif
}

#define USB_BUFSIZ	512

static int asynch_allowed;
char usb_started = 0; /* flag for the started/stopped USB status */

static struct usb_device usb_dev[USB_MAX_DEVICE];
static int dev_index;

__weak int usb_alloc_device(struct usb_device *udev)
{
	return 0;
}

/* returns a pointer to the device with the index [index].
 * if the device is not assigned (dev->devnum==-1) returns NULL
 */
struct usb_device *usb_get_dev_index(int index)
{
	if (usb_dev[index].devnum == -1)
		return NULL;
	else
		return &usb_dev[index];
}

/*
 * disables the asynch behaviour of the control message. This is used for data
 * transfers that uses the exclusiv access to the control and bulk messages.
 * Returns the old value so it can be restored later.
 */
int usb_disable_asynch(int disable)
{
	int old_value = asynch_allowed;

	asynch_allowed = !disable;
	return old_value;
}

/***********************************************************************
 * Clears an endpoint
 * endp: endpoint number in bits 0-3;
 * direction flag in bit 7 (1 = IN, 0 = OUT)
 */
int usb_clear_halt(struct usb_device *dev, int pipe)
{
	int result;
	int endp = usb_pipeendpoint(pipe)|(usb_pipein(pipe)<<7);

	result = usb_control_msg(dev, usb_sndctrlpipe(dev, 0),
				 USB_REQ_CLEAR_FEATURE, USB_RECIP_ENDPOINT, 0,
				 endp, NULL, 0, USB_CNTL_TIMEOUT * 3);

	/* don't clear if failed */
	if (result < 0)
		return result;

	/*
	 * NOTE: we do not get status and verify reset was successful
	 * as some devices are reported to lock up upon this check..
	 */

	usb_endpoint_running(dev, usb_pipeendpoint(pipe), usb_pipeout(pipe));

	/* toggle is reset on clear */
	usb_settoggle(dev, usb_pipeendpoint(pipe), usb_pipeout(pipe), 0);
	return 0;
}

/*
 * submits a control message and waits for comletion (at least timeout * 1ms)
 * If timeout is 0, we don't wait for completion (used as example to set and
 * clear keyboards LEDs). For data transfers, (storage transfers) we don't
 * allow control messages with 0 timeout, by previousely resetting the flag
 * asynch_allowed (usb_disable_asynch(1)).
 * returns the transferred length if OK or -1 if error. The transferred length
 * and the current status are stored in the dev->act_len and dev->status.
 */
int usb_control_msg(struct usb_device *dev, unsigned int pipe,
			unsigned char request, unsigned char requesttype,
			unsigned short value, unsigned short index,
			void *data, unsigned short size, int timeout)
{
	ALLOC_CACHE_ALIGN_BUFFER(struct devrequest, setup_packet, 1);
	int err;

	if ((timeout == 0) && (!asynch_allowed)) {
		/* request for a asynch control pipe is not allowed */
		return -EINVAL;
	}

	/* set setup command */
	setup_packet->requesttype = requesttype;
	setup_packet->request = request;
	setup_packet->value = cpu_to_le16(value);
	setup_packet->index = cpu_to_le16(index);
	setup_packet->length = cpu_to_le16(size);
	debug("usb_control_msg: request: 0x%X, requesttype: 0x%X, " \
	      "value 0x%X index 0x%X length 0x%X\n",
	      request, requesttype, value, index, size);
	dev->status = USB_ST_NOT_PROC; /*not yet processed */

	err = submit_control_msg(dev, pipe, data, size, setup_packet);
	if (err < 0)
		return err;
	if (timeout == 0)
		return (int)size;

	/*
	 * Wait for status to update until timeout expires, USB driver
	 * interrupt handler may set the status when the USB operation has
	 * been completed.
	 */
	while (timeout--) {
		if (!((volatile unsigned long)dev->status & USB_ST_NOT_PROC))
			break;
		mdelay(1);
	}
	if (dev->status)
		return -1;

	return dev->act_len;

}

/*-------------------------------------------------------------------
 * submits bulk message, and waits for completion. returns 0 if Ok or
 * negative if Error.
 * synchronous behavior
 */
int usb_bulk_msg(struct usb_device *dev, unsigned int pipe,
			void *data, int len, int *actual_length, int timeout)
{
	if (len < 0)
		return -EINVAL;
	dev->status = USB_ST_NOT_PROC; /*not yet processed */
	if (submit_bulk_msg(dev, pipe, data, len) < 0)
		return -EIO;
	while (timeout--) {
		if (!((volatile unsigned long)dev->status & USB_ST_NOT_PROC))
			break;
		mdelay(1);
	}
	*actual_length = dev->act_len;
	if (dev->status == 0)
		return 0;
	else
		return -EIO;
}

/**********************************************************************
 * get_descriptor type
 */
static int usb_get_descriptor(struct usb_device *dev, unsigned char type,
			unsigned char index, void *buf, int size)
{
	return usb_control_msg(dev, usb_rcvctrlpipe(dev, 0),
			       USB_REQ_GET_DESCRIPTOR, USB_DIR_IN,
			       (type << 8) + index, 0, buf, size,
			       USB_CNTL_TIMEOUT);
}

static int get_descriptor_len(struct usb_device *dev, int len, int expect_len)
{
	__maybe_unused struct usb_device_descriptor *desc;
	ALLOC_CACHE_ALIGN_BUFFER(unsigned char, tmpbuf, USB_BUFSIZ);
	int err;

	desc = (struct usb_device_descriptor *)tmpbuf;

	err = usb_get_descriptor(dev, USB_DT_DEVICE, 0, desc, len);
	if (err < expect_len) {
		if (err < 0) {
			printf("unable to get device descriptor (error=%d)\n",
				err);
			return err;
		} else {
			printf("USB device descriptor short read (expected %i, got %i)\n",
				expect_len, err);
			return -EIO;
		}
	}
	memcpy(&dev->descriptor, tmpbuf, sizeof(dev->descriptor));

	return 0;
}

static int usb_setup_descriptor(struct usb_device *dev, bool do_read)
{
	/*
	 * This is a Windows scheme of initialization sequence, with double
	 * reset of the device (Linux uses the same sequence)
	 * Some equipment is said to work only with such init sequence; this
	 * patch is based on the work by Alan Stern:
	 * http://sourceforge.net/mailarchive/forum.php?
	 * thread_id=5729457&forum_id=5398
	 */

	/*
	 * send 64-byte GET-DEVICE-DESCRIPTOR request.  Since the descriptor is
	 * only 18 bytes long, this will terminate with a short packet.  But if
	 * the maxpacket size is 8 or 16 the device may be waiting to transmit
	 * some more, or keeps on retransmitting the 8 byte header.
	 */

	if (dev->speed == USB_SPEED_LOW) {
		dev->descriptor.bMaxPacketSize0 = 8;
		dev->maxpacketsize = PACKET_SIZE_8;
	} else {
		dev->descriptor.bMaxPacketSize0 = 64;
		dev->maxpacketsize = PACKET_SIZE_64;
	}
	dev->epmaxpacketin[0] = dev->descriptor.bMaxPacketSize0;
	dev->epmaxpacketout[0] = dev->descriptor.bMaxPacketSize0;

	if (do_read) {
		int err;

		/*
		 * Validate we've received only at least 8 bytes, not that we've
		 * received the entire descriptor. The reasoning is:
		 * - The code only uses fields in the first 8 bytes, so that's all we
		 *   need to have fetched at this stage.
		 * - The smallest maxpacket size is 8 bytes. Before we know the actual
		 *   maxpacket the device uses, the USB controller may only accept a
		 *   single packet. Consequently we are only guaranteed to receive 1
		 *   packet (at least 8 bytes) even in a non-error case.
		 *
		 * At least the DWC2 controller needs to be programmed with the number
		 * of packets in addition to the number of bytes. A request for 64
		 * bytes of data with the maxpacket guessed as 64 (above) yields a
		 * request for 1 packet.
		 */
		err = get_descriptor_len(dev, 64, 8);
		if (err)
			return err;
	}

	dev->epmaxpacketin[0] = dev->descriptor.bMaxPacketSize0;
	dev->epmaxpacketout[0] = dev->descriptor.bMaxPacketSize0;
	switch (dev->descriptor.bMaxPacketSize0) {
	case 8:
		dev->maxpacketsize  = PACKET_SIZE_8;
		break;
	case 16:
		dev->maxpacketsize = PACKET_SIZE_16;
		break;
	case 32:
		dev->maxpacketsize = PACKET_SIZE_32;
		break;
	case 64:
		dev->maxpacketsize = PACKET_SIZE_64;
		break;
	default:
		printf("usb_new_device: invalid max packet size\n");
		return -EIO;
	}

	return 0;
}

static int usb_hub_port_reset(struct usb_device *dev, struct usb_device *hub)
{
	if (!hub)
		usb_reset_root_port(dev);

	return 0;
}

/********************************************************************
 * set address of a device to the value in dev->devnum.
 * This can only be done by addressing the device via the default address (0)
 */
static int usb_set_address(struct usb_device *dev)
{
	debug("set address %d\n", dev->devnum);

	return usb_control_msg(dev, usb_snddefctrl(dev), USB_REQ_SET_ADDRESS,
			       0, (dev->devnum), 0, NULL, 0, USB_CNTL_TIMEOUT);
}

/********************************************************************
 * set interface number to interface
 */
int usb_set_interface(struct usb_device *dev, int interface, int alternate)
{
	struct usb_interface *if_face = NULL;
	int ret, i;

	for (i = 0; i < dev->config.desc.bNumInterfaces; i++) {
		if (dev->config.if_desc[i].desc.bInterfaceNumber == interface) {
			if_face = &dev->config.if_desc[i];
			break;
		}
	}
	if (!if_face) {
		printf("selecting invalid interface %d", interface);
		return -EINVAL;
	}
	/*
	 * We should return now for devices with only one alternate setting.
	 * According to 9.4.10 of the Universal Serial Bus Specification
	 * Revision 2.0 such devices can return with a STALL. This results in
	 * some USB sticks timeouting during initialization and then being
	 * unusable in U-Boot.
	 */
	if (if_face->num_altsetting == 1)
		return 0;

	ret = usb_control_msg(dev, usb_sndctrlpipe(dev, 0),
				USB_REQ_SET_INTERFACE, USB_RECIP_INTERFACE,
				alternate, interface, NULL, 0,
				USB_CNTL_TIMEOUT * 5);
	if (ret < 0)
		return ret;

	return 0;
}

static int usb_prepare_device(struct usb_device *dev, int addr, bool do_read,
			      struct usb_device *parent)
{
	int err;

	/*
	 * Allocate usb 3.0 device context.
	 * USB 3.0 (xHCI) protocol tries to allocate device slot
	 * and related data structures first. This call does that.
	 * Refer to sec 4.3.2 in xHCI spec rev1.0
	 */
	err = usb_alloc_device(dev);
	if (err) {
		printf("Cannot allocate device context to get SLOT_ID\n");
		return err;
	}
	err = usb_setup_descriptor(dev, do_read);
	if (err)
		return err;
	err = usb_hub_port_reset(dev, parent);
	if (err)
		return err;

	dev->devnum = addr;

	err = usb_set_address(dev); /* set address */

	if (err < 0) {
		printf("\n      USB device not accepting new address " \
			"(error=%lX)\n", dev->status);
		return err;
	}

	mdelay(10);	/* Let the SET_ADDRESS settle */

	return 0;
}

/**********************************************************************
 * gets len of configuration cfgno
 */
int usb_get_configuration_len(struct usb_device *dev, int cfgno)
{
	int result;
	ALLOC_CACHE_ALIGN_BUFFER(unsigned char, buffer, 9);
	struct usb_config_descriptor *config;

	config = (struct usb_config_descriptor *)&buffer[0];
	result = usb_get_descriptor(dev, USB_DT_CONFIG, cfgno, buffer, 9);
	if (result < 9) {
		if (result < 0)
			printf("unable to get descriptor, error %lX\n",
				dev->status);
		else
			printf("config descriptor too short " \
				"(expected %i, got %i)\n", 9, result);
		return -EIO;
	}
	return le16_to_cpu(config->wTotalLength);
}

/**********************************************************************
 * gets configuration cfgno and store it in the buffer
 */
int usb_get_configuration_no(struct usb_device *dev, int cfgno,
			     unsigned char *buffer, int length)
{
	int result;
	struct usb_config_descriptor *config;

	config = (struct usb_config_descriptor *)&buffer[0];
	result = usb_get_descriptor(dev, USB_DT_CONFIG, cfgno, buffer, length);
	debug("get_conf_no %d Result %d, wLength %d\n", cfgno, result,
	      le16_to_cpu(config->wTotalLength));
	config->wTotalLength = result; /* validated, with CPU byte order */

	return result;
}

/*******************************************************************************
 * Parse the config, located in buffer, and fills the dev->config structure.
 * Note that all little/big endian swapping are done automatically.
 * (wTotalLength has already been swapped and sanitized when it was read.)
 */
static int usb_parse_config(struct usb_device *dev,
			unsigned char *buffer, int cfgno)
{
	struct usb_descriptor_header *head;
	int index, ifno, epno, curr_if_num;
	u16 ep_wMaxPacketSize;
	struct usb_interface *if_desc = NULL;

	ifno = -1;
	epno = -1;
	curr_if_num = -1;

	dev->configno = cfgno;
	head = (struct usb_descriptor_header *) &buffer[0];
	if (head->bDescriptorType != USB_DT_CONFIG) {
		printf(" ERROR: NOT USB_CONFIG_DESC %x\n",
			head->bDescriptorType);
		return -EINVAL;
	}
	if (head->bLength != USB_DT_CONFIG_SIZE) {
		printf("ERROR: Invalid USB CFG length (%d)\n", head->bLength);
		return -EINVAL;
	}
	memcpy(&dev->config, head, USB_DT_CONFIG_SIZE);
	dev->config.no_of_if = 0;

	index = dev->config.desc.bLength;
	/* Ok the first entry must be a configuration entry,
	 * now process the others */
	head = (struct usb_descriptor_header *) &buffer[index];
	while (index + 1 < dev->config.desc.wTotalLength && head->bLength) {
		switch (head->bDescriptorType) {
		case USB_DT_INTERFACE:
			if (head->bLength != USB_DT_INTERFACE_SIZE) {
				printf("ERROR: Invalid USB IF length (%d)\n",
					head->bLength);
				break;
			}
			if (index + USB_DT_INTERFACE_SIZE >
			    dev->config.desc.wTotalLength) {
				puts("USB IF descriptor overflowed buffer!\n");
				break;
			}
			if (((struct usb_interface_descriptor *) \
			     head)->bInterfaceNumber != curr_if_num) {
				/* this is a new interface, copy new desc */
				ifno = dev->config.no_of_if;
				if (ifno >= USB_MAXINTERFACES) {
					puts("Too many USB interfaces!\n");
					/* try to go on with what we have */
					return -EINVAL;
				}
				if_desc = &dev->config.if_desc[ifno];
				dev->config.no_of_if++;
				memcpy(if_desc, head,
					USB_DT_INTERFACE_SIZE);
				if_desc->no_of_ep = 0;
				if_desc->num_altsetting = 1;
				curr_if_num =
				     if_desc->desc.bInterfaceNumber;
			} else {
				/* found alternate setting for the interface */
				if (ifno >= 0) {
					if_desc = &dev->config.if_desc[ifno];
					if_desc->num_altsetting++;
				}
			}
			break;
		case USB_DT_ENDPOINT:
			if (head->bLength != USB_DT_ENDPOINT_SIZE) {
				printf("ERROR: Invalid USB EP length (%d)\n",
					head->bLength);
				break;
			}
			if (index + USB_DT_ENDPOINT_SIZE >
			    dev->config.desc.wTotalLength) {
				puts("USB EP descriptor overflowed buffer!\n");
				break;
			}
			if (ifno < 0) {
				puts("Endpoint descriptor out of order!\n");
				break;
			}
			epno = dev->config.if_desc[ifno].no_of_ep;
			if_desc = &dev->config.if_desc[ifno];
			if (epno >= USB_MAXENDPOINTS) {
				printf("Interface %d has too many endpoints!\n",
					if_desc->desc.bInterfaceNumber);
				return -EINVAL;
			}
			/* found an endpoint */
			if_desc->no_of_ep++;
			memcpy(&if_desc->ep_desc[epno], head,
				USB_DT_ENDPOINT_SIZE);
			ep_wMaxPacketSize = get_unaligned(&dev->config.\
							if_desc[ifno].\
							ep_desc[epno].\
							wMaxPacketSize);
			put_unaligned(le16_to_cpu(ep_wMaxPacketSize),
					&dev->config.\
					if_desc[ifno].\
					ep_desc[epno].\
					wMaxPacketSize);
			debug("if %d, ep %d\n", ifno, epno);
			break;
		case USB_DT_SS_ENDPOINT_COMP:
			if (head->bLength != USB_DT_SS_EP_COMP_SIZE) {
				printf("ERROR: Invalid USB EPC length (%d)\n",
					head->bLength);
				break;
			}
			if (index + USB_DT_SS_EP_COMP_SIZE >
			    dev->config.desc.wTotalLength) {
				puts("USB EPC descriptor overflowed buffer!\n");
				break;
			}
			if (ifno < 0 || epno < 0) {
				puts("EPC descriptor out of order!\n");
				break;
			}
			if_desc = &dev->config.if_desc[ifno];
			memcpy(&if_desc->ss_ep_comp_desc[epno], head,
				USB_DT_SS_EP_COMP_SIZE);
			break;
		default:
			if (head->bLength == 0)
				return -EINVAL;

			debug("unknown Description Type : %x\n",
			      head->bDescriptorType);

#ifdef DEBUG
			{
				unsigned char *ch = (unsigned char *)head;
				int i;

				for (i = 0; i < head->bLength; i++)
					debug("%02X ", *ch++);
				debug("\n\n\n");
			}
#endif
			break;
		}
		index += head->bLength;
		head = (struct usb_descriptor_header *)&buffer[index];
	}
	return 0;
}

/*
 * The routine usb_set_maxpacket_ep() is extracted from the loop of routine
 * usb_set_maxpacket(), because the optimizer of GCC 4.x chokes on this routine
 * when it is inlined in 1 single routine. What happens is that the register r3
 * is used as loop-count 'i', but gets overwritten later on.
 * This is clearly a compiler bug, but it is easier to workaround it here than
 * to update the compiler (Occurs with at least several GCC 4.{1,2},x
 * CodeSourcery compilers like e.g. 2007q3, 2008q1, 2008q3 lite editions on ARM)
 *
 * NOTE: Similar behaviour was observed with GCC4.6 on ARMv5.
 */
static void noinline
usb_set_maxpacket_ep(struct usb_device *dev, int if_idx, int ep_idx)
{
	int b;
	struct usb_endpoint_descriptor *ep;
	u16 ep_wMaxPacketSize;

	ep = &dev->config.if_desc[if_idx].ep_desc[ep_idx];

	b = ep->bEndpointAddress & USB_ENDPOINT_NUMBER_MASK;
	ep_wMaxPacketSize = get_unaligned(&ep->wMaxPacketSize);

	if ((ep->bmAttributes & USB_ENDPOINT_XFERTYPE_MASK) ==
						USB_ENDPOINT_XFER_CONTROL) {
		/* Control => bidirectional */
		dev->epmaxpacketout[b] = ep_wMaxPacketSize;
		dev->epmaxpacketin[b] = ep_wMaxPacketSize;
		debug("##Control EP epmaxpacketout/in[%d] = %d\n",
		      b, dev->epmaxpacketin[b]);
	} else {
		if ((ep->bEndpointAddress & 0x80) == 0) {
			/* OUT Endpoint */
			if (ep_wMaxPacketSize > dev->epmaxpacketout[b]) {
				dev->epmaxpacketout[b] = ep_wMaxPacketSize;
				debug("##EP epmaxpacketout[%d] = %d\n",
				      b, dev->epmaxpacketout[b]);
			}
		} else {
			/* IN Endpoint */
			if (ep_wMaxPacketSize > dev->epmaxpacketin[b]) {
				dev->epmaxpacketin[b] = ep_wMaxPacketSize;
				debug("##EP epmaxpacketin[%d] = %d\n",
				      b, dev->epmaxpacketin[b]);
			}
		} /* if out */
	} /* if control */
}

/*
 * set the max packed value of all endpoints in the given configuration
 */
static int usb_set_maxpacket(struct usb_device *dev)
{
	int i, ii;

	for (i = 0; i < dev->config.desc.bNumInterfaces; i++)
		for (ii = 0; ii < dev->config.if_desc[i].desc.bNumEndpoints; ii++)
			usb_set_maxpacket_ep(dev, i, ii);

	return 0;
}

/********************************************************************
 * set configuration number to configuration
 */
static int usb_set_configuration(struct usb_device *dev, int configuration)
{
	int res;
	debug("set configuration %d\n", configuration);
	/* set setup command */
	res = usb_control_msg(dev, usb_sndctrlpipe(dev, 0),
				USB_REQ_SET_CONFIGURATION, 0,
				configuration, 0,
				NULL, 0, USB_CNTL_TIMEOUT);
	if (res == 0) {
		dev->toggle[0] = 0;
		dev->toggle[1] = 0;
		return 0;
	} else
		return -EIO;
}

/********************************************************************
 * get string index in buffer
 */
static int usb_get_string(struct usb_device *dev, unsigned short langid,
		   unsigned char index, void *buf, int size)
{
	int i;
	int result;

	for (i = 0; i < 3; ++i) {
		/* some devices are flaky */
		result = usb_control_msg(dev, usb_rcvctrlpipe(dev, 0),
			USB_REQ_GET_DESCRIPTOR, USB_DIR_IN,
			(USB_DT_STRING << 8) + index, langid, buf, size,
			USB_CNTL_TIMEOUT);

		if (result > 0)
			break;
	}

	return result;
}

static void usb_try_string_workarounds(unsigned char *buf, int *length)
{
	int newlength, oldlength = *length;

	for (newlength = 2; newlength + 1 < oldlength; newlength += 2)
		if (!isprint(buf[newlength]) || buf[newlength + 1])
			break;

	if (newlength > 2) {
		buf[0] = newlength;
		*length = newlength;
	}
}

static int usb_string_sub(struct usb_device *dev, unsigned int langid,
		unsigned int index, unsigned char *buf)
{
	int rc;

	/* Try to read the string descriptor by asking for the maximum
	 * possible number of bytes */
	rc = usb_get_string(dev, langid, index, buf, 255);

	/* If that failed try to read the descriptor length, then
	 * ask for just that many bytes */
	if (rc < 2) {
		rc = usb_get_string(dev, langid, index, buf, 2);
		if (rc == 2)
			rc = usb_get_string(dev, langid, index, buf, buf[0]);
	}

	if (rc >= 2) {
		if (!buf[0] && !buf[1])
			usb_try_string_workarounds(buf, &rc);

		/* There might be extra junk at the end of the descriptor */
		if (buf[0] < rc)
			rc = buf[0];

		rc = rc - (rc & 1); /* force a multiple of two */
	}

	if (rc < 2)
		rc = -EINVAL;

	return rc;
}

/********************************************************************
 * usb_string:
 * Get string index and translate it to ascii.
 * returns string length (> 0) or error (< 0)
 */
int usb_string(struct usb_device *dev, int index, char *buf, size_t size)
{
	ALLOC_CACHE_ALIGN_BUFFER(unsigned char, mybuf, USB_BUFSIZ);
	unsigned char *tbuf;
	int err;
	unsigned int u, idx;

	if (size <= 0 || !buf || !index)
		return -EINVAL;
	buf[0] = 0;
	tbuf = &mybuf[0];

	/* get langid for strings if it's not yet known */
	if (!dev->have_langid) {
		err = usb_string_sub(dev, 0, 0, tbuf);
		if (err < 0) {
			debug("error getting string descriptor 0 " \
			      "(error=%lx)\n", dev->status);
			return -EIO;
		} else if (tbuf[0] < 4) {
			debug("string descriptor 0 too short\n");
			return -EIO;
		} else {
			dev->have_langid = -1;
			dev->string_langid = tbuf[2] | (tbuf[3] << 8);
				/* always use the first langid listed */
			debug("USB device number %d default " \
			      "language ID 0x%x\n",
			      dev->devnum, dev->string_langid);
		}
	}

	err = usb_string_sub(dev, dev->string_langid, index, tbuf);
	if (err < 0)
		return err;

	size--;		/* leave room for trailing NULL char in output buffer */
	for (idx = 0, u = 2; u < err; u += 2) {
		if (idx >= size)
			break;
		if (tbuf[u+1])			/* high byte */
			buf[idx++] = '?';  /* non-ASCII character */
		else
			buf[idx++] = tbuf[u];
	}
	buf[idx] = 0;
	err = idx;
	return err;
}

int usb_select_config(struct usb_device *dev)
{
	unsigned char *tmpbuf = NULL;
	int err;

	err = get_descriptor_len(dev, USB_DT_DEVICE_SIZE, USB_DT_DEVICE_SIZE);
	if (err)
		return err;

	/* correct le values */
	le16_to_cpus(&dev->descriptor.bcdUSB);
	le16_to_cpus(&dev->descriptor.idVendor);
	le16_to_cpus(&dev->descriptor.idProduct);
	le16_to_cpus(&dev->descriptor.bcdDevice);

	/*
	 * Kingston DT Ultimate 32GB USB 3.0 seems to be extremely sensitive
	 * about this first Get Descriptor request. If there are any other
	 * requests in the first microframe, the stick crashes. Wait about
	 * one microframe duration here (1mS for USB 1.x , 125uS for USB 2.0).
	 */
	mdelay(1);

	/* only support for one config for now */
	err = usb_get_configuration_len(dev, 0);
	if (err >= 0) {
		tmpbuf = (unsigned char *)malloc_cache_aligned(err);
		if (!tmpbuf)
			err = -ENOMEM;
		else
			err = usb_get_configuration_no(dev, 0, tmpbuf, err);
	}
	if (err < 0) {
		printf("usb_new_device: Cannot read configuration, " \
		       "skipping device %04x:%04x\n",
		       dev->descriptor.idVendor, dev->descriptor.idProduct);
		free(tmpbuf);
		return err;
	}
	usb_parse_config(dev, tmpbuf, 0);
#ifdef LOCAL_SIM
	free(tmpbuf);
#endif
	usb_set_maxpacket(dev);
	/*
	 * we set the default configuration here
	 * This seems premature. If the driver wants a different configuration
	 * it will need to select itself.
	 */
	err = usb_set_configuration(dev, dev->config.desc.bConfigurationValue);
	if (err < 0) {
		printf("failed to set default configuration " \
			"len %d, status %lX\n", dev->act_len, dev->status);
		return err;
	}

	/*
	 * Wait until the Set Configuration request gets processed by the
	 * device. This is required by at least SanDisk Cruzer Pop USB 2.0
	 * and Kingston DT Ultimate 32GB USB 3.0 on DWC2 OTG controller.
	 */
	mdelay(10);

	debug("new device strings: Mfr=%d, Product=%d, SerialNumber=%d\n",
	      dev->descriptor.iManufacturer, dev->descriptor.iProduct,
	      dev->descriptor.iSerialNumber);
	memset(dev->mf, 0, sizeof(dev->mf));
	memset(dev->prod, 0, sizeof(dev->prod));
	memset(dev->serial, 0, sizeof(dev->serial));
	if (dev->descriptor.iManufacturer)
		usb_string(dev, dev->descriptor.iManufacturer,
			   dev->mf, sizeof(dev->mf));
	if (dev->descriptor.iProduct)
		usb_string(dev, dev->descriptor.iProduct,
			   dev->prod, sizeof(dev->prod));
	if (dev->descriptor.iSerialNumber)
		usb_string(dev, dev->descriptor.iSerialNumber,
			   dev->serial, sizeof(dev->serial));
	debug("Manufacturer %s\n", dev->mf);
	debug("Product      %s\n", dev->prod);
	debug("SerialNumber %s\n", dev->serial);

	return 0;
}

int usb_setup_device(struct usb_device *dev, bool do_read,
		     struct usb_device *parent)
{
	int addr;
	int ret;

	/* We still haven't set the Address yet */
	addr = dev->devnum;
	dev->devnum = 0;

	ret = usb_prepare_device(dev, addr, do_read, parent);
	if (ret)
		return ret;
	ret = usb_select_config(dev);

	return ret;
}

int usb_new_device(struct usb_device *dev)
{
	debug("%s:%d.\n", __func__, __LINE__);

	bool do_read = true;
	int err;

	/*
	 * XHCI needs to issue a Address device command to setup
	 * proper device context structures, before it can interact
	 * with the device. So a get_descriptor will fail before any
	 * of that is done for XHCI unlike EHCI.
	 */
#ifdef CONFIG_USB_XHCI_HCD
	do_read = false;
#endif
	err = usb_setup_device(dev, do_read, dev->parent);
	if (err)
		return err;

	/* Now probe if the device is a hub */
	err = usb_hub_probe(dev, 0);
	if (err < 0)
		return err;

	return 0;
}

int usb_alloc_new_device(struct udevice *controller, struct usb_device **devp)
{
	int i;
	debug("New Device %d\n", dev_index);
	if (dev_index == USB_MAX_DEVICE) {
		printf("ERROR, too many USB Devices, max=%d\n", USB_MAX_DEVICE);
		return -ENOSPC;
	}
	/* default Address is 0, real addresses start with 1 */
	usb_dev[dev_index].devnum = dev_index + 1;
	usb_dev[dev_index].maxchild = 0;
	for (i = 0; i < USB_MAXCHILDREN; i++)
		usb_dev[dev_index].children[i] = NULL;
	usb_dev[dev_index].parent = NULL;
	usb_dev[dev_index].controller = controller;
	dev_index++;
	*devp = &usb_dev[dev_index - 1];

	return 0;
}

int usb_init(void)
{
	void *ctrl;
	struct usb_device *dev;
	int i, start_index = 0;
	int controllers_initialized = 0;
	int ret;

	dev_index = 0;
	asynch_allowed = 1;
	//usb_hub_reset();

	/* first make all devices unknown */
	/*
	for (i = 0; i < USB_MAX_DEVICE; i++) {
		memset(&usb_dev[i], 0, sizeof(struct usb_device));
		usb_dev[i].devnum = -1;
	}
	*/

	/* init low_level USB */
	debug("%s:%d\n", __func__,__LINE__);
	for (i = 0; i < CONFIG_USB_MAX_CONTROLLER_COUNT; i++) {
		/* init low_level USB */
		debug("USB%d:   \n", i);
		// defined in host/xhci.c
		ret = usb_lowlevel_init(i, USB_INIT_HOST, &ctrl);
		if (ret == -ENODEV) {	/* No such device. */
			puts("Port not available.\n");
			controllers_initialized++;
			continue;
		}

		if (ret) {		/* Other error. */
			puts("lowlevel init failed\n");
			continue;
		}
		/*
		 * lowlevel init is OK, now scan the bus for devices
		 * i.e. search HUBs and configure them
		 */
		controllers_initialized++;
		start_index = dev_index;
		printf("scanning bus %d for devices... ", i);
		ret = usb_alloc_new_device(ctrl, &dev);
		if (ret)
			break;

		/*
		 * device 0 is always present
		 * (root hub, so let it analyze)
		 */
		//TODO: use usb_new_device to enumerate new device
		ret = usb_new_device(dev);
		if (ret)
			usb_free_device(dev->controller);

		if (start_index == dev_index) {
			puts("No USB Device found\n");
			continue;
		} else {
			printf("%d USB Device(s) found\n",
				dev_index - start_index);
		}

		usb_started = 1;
	}

	debug("scan end\n");
	/* if we were not able to find at least one working bus, bail out */
	if (controllers_initialized == 0)
		puts("USB error: all controllers failed lowlevel init\n");

	debug("%s:%d.\n", __func__, __LINE__);
	//return usb_started ? 0 : -ENODEV;
	return 0;
}

/*
 * returns the max packet size, depending on the pipe direction and
 * the configurations values
 */
int usb_maxpacket(struct usb_device *dev, unsigned long pipe)
{
	/* direction is out -> use emaxpacket out */
	if ((pipe & USB_DIR_IN) == 0)
		return dev->epmaxpacketout[((pipe>>15) & 0xf)];
	else
		return dev->epmaxpacketin[((pipe>>15) & 0xf)];
}

bool usb_device_has_child_on_port(struct usb_device *parent, int port)
{
#ifdef CONFIG_DM_USB
	return false;
#else
	return parent->children[port] != NULL;
#endif
}

static void do_usb_start(void)
{
	usb_stor_curr_dev = usb_stor_scan(1);
}


struct blk_desc *blk_get_devnum_by_type(enum if_type if_type, int devnum)
{}

static int msg_read()
{
	struct blk_desc *stor_dev;

	if (usb_stor_curr_dev < 0) {
		printf("no current device selected\n");
		return 1;
	}
	unsigned long addr = 0x00000000;
	unsigned long blk  = 0x00000000;
	unsigned long cnt  = 0x10;
	unsigned long n;
	printf("\nUSB read: device %d block # %ld, count %ld"
		" ... ", usb_stor_curr_dev, blk, cnt);
	stor_dev = blk_get_devnum_by_type(IF_TYPE_USB,
					  usb_stor_curr_dev);
	n = blk_dread(stor_dev, blk, cnt, (ulong *)addr);
	printf("%ld blocks read: %s\n", n,
		(n == cnt) ? "OK" : "ERROR");
	if (n == cnt)
		return 0;
}

#ifdef LOCAL_SIM
int main()
#else
//TODO: modify this API name
int main()
{
	int ret;
	printf("Start DWC3 Simulation.\n");
	ret = usb_init();
	if (ret)
		goto out;
	do_usb_start()	;
out:		
	return ret;
}
#endif
