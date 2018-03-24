#ifndef __USB_DEFS_H_
#define __USB_DEFS_H_

/* Sub STORAGE Classes */
#define US_SC_RBC              1		/* Typically, flash devices */
#define US_SC_8020             2		/* CD-ROM */
#define US_SC_QIC              3		/* QIC-157 Tapes */
#define US_SC_UFI              4		/* Floppy */
#define US_SC_8070             5		/* Removable media */
#define US_SC_SCSI             6		/* Transparent */
#define US_SC_MIN              US_SC_RBC
#define US_SC_MAX              US_SC_SCSI

/* STORAGE Protocols */
#define US_PR_CB               1		/* Control/Bulk w/o interrupt */
#define US_PR_CBI              0		/* Control/Bulk/Interrupt */
#define US_PR_BULK             0x50		/* bulk only */

/* USB types */
#define USB_TYPE_STANDARD   (0x00 << 5)
#define USB_TYPE_CLASS      (0x01 << 5)
#define USB_TYPE_VENDOR     (0x02 << 5)
#define USB_TYPE_RESERVED   (0x03 << 5)

/* USB recipients */
#define USB_RECIP_DEVICE      0x00
#define USB_RECIP_INTERFACE   0x01
#define USB_RECIP_ENDPOINT    0x02
#define USB_RECIP_OTHER       0x03


/* USB directions */
#define USB_DIR_OUT           0
#define USB_DIR_IN            0x80

/*
 * bmRequestType: USB Device Requests, table 9.2 USB 2.0 spec.
 * (shifted) direction/type/recipient.
 */
#define DeviceRequest \
	((USB_DIR_IN | USB_TYPE_STANDARD | USB_RECIP_DEVICE) << 8)

#define DeviceOutRequest \
	((USB_DIR_OUT | USB_TYPE_STANDARD | USB_RECIP_DEVICE) << 8)

#define InterfaceRequest \
	((USB_DIR_IN | USB_TYPE_STANDARD | USB_RECIP_INTERFACE) << 8)

#define EndpointRequest \
	((USB_DIR_IN | USB_TYPE_STANDARD | USB_RECIP_INTERFACE) << 8)

#define EndpointOutRequest \
	((USB_DIR_OUT | USB_TYPE_STANDARD | USB_RECIP_INTERFACE) << 8)

/* Descriptor types */
#define USB_DT_DEVICE        0x01
#define USB_DT_CONFIG        0x02
#define USB_DT_STRING        0x03
#define USB_DT_INTERFACE     0x04
#define USB_DT_ENDPOINT      0x05

#define USB_DT_HID          (USB_TYPE_CLASS | 0x01)
#define USB_DT_REPORT       (USB_TYPE_CLASS | 0x02)
#define USB_DT_PHYSICAL     (USB_TYPE_CLASS | 0x03)
#define USB_DT_HUB          (USB_TYPE_CLASS | 0x09)

/* Descriptor sizes per descriptor type */
#define USB_DT_DEVICE_SIZE      18
#define USB_DT_CONFIG_SIZE      9
#define USB_DT_INTERFACE_SIZE   9
#define USB_DT_ENDPOINT_SIZE    7
#define USB_DT_ENDPOINT_AUDIO_SIZE  9	/* Audio extension */
#define USB_DT_HUB_NONVAR_SIZE  7
#define USB_DT_HID_SIZE         9

/* Endpoints */
#define USB_ENDPOINT_NUMBER_MASK  0x0f	/* in bEndpointAddress */
#define USB_ENDPOINT_DIR_MASK     0x80

#define USB_ENDPOINT_XFERTYPE_MASK 0x03	/* in bmAttributes */
#define USB_ENDPOINT_XFER_CONTROL  0
#define USB_ENDPOINT_XFER_ISOC     1
#define USB_ENDPOINT_XFER_BULK     2
#define USB_ENDPOINT_XFER_INT      3

/* USB Packet IDs (PIDs) */
#define USB_PID_UNDEF_0             0xf0
#define USB_PID_OUT                 0xe1
#define USB_PID_ACK                 0xd2
#define USB_PID_DATA0               0xc3
#define USB_PID_UNDEF_4             0xb4
#define USB_PID_SOF                 0xa5
#define USB_PID_UNDEF_6             0x96
#define USB_PID_UNDEF_7             0x87
#define USB_PID_UNDEF_8             0x78
#define USB_PID_IN                  0x69
#define USB_PID_NAK                 0x5a
#define USB_PID_DATA1               0x4b
#define USB_PID_PREAMBLE            0x3c
#define USB_PID_SETUP               0x2d
#define USB_PID_STALL               0x1e
#define USB_PID_UNDEF_F             0x0f

/* Standard requests */
#define USB_REQ_GET_STATUS          0x00
#define USB_REQ_CLEAR_FEATURE       0x01
#define USB_REQ_SET_FEATURE         0x03
#define USB_REQ_SET_ADDRESS         0x05
#define USB_REQ_GET_DESCRIPTOR      0x06
#define USB_REQ_SET_DESCRIPTOR      0x07
#define USB_REQ_GET_CONFIGURATION   0x08
#define USB_REQ_SET_CONFIGURATION   0x09
#define USB_REQ_GET_INTERFACE       0x0A
#define USB_REQ_SET_INTERFACE       0x0B
#define USB_REQ_SYNCH_FRAME         0x0C

/* HID requests */
#define USB_REQ_GET_REPORT          0x01
#define USB_REQ_GET_IDLE            0x02
#define USB_REQ_GET_PROTOCOL        0x03
#define USB_REQ_SET_REPORT          0x09
#define USB_REQ_SET_IDLE            0x0A
#define USB_REQ_SET_PROTOCOL        0x0B

/* Device features */
#define USB_FEAT_HALT               0x00
#define USB_FEAT_WAKEUP             0x01
#define USB_FEAT_TEST               0x02

/* Test modes */
#define USB_TEST_MODE_J             0x01
#define USB_TEST_MODE_K             0x02
#define USB_TEST_MODE_SE0_NAK       0x03
#define USB_TEST_MODE_PACKET        0x04
#define USB_TEST_MODE_FORCE_ENABLE  0x05

/*
 * "pipe" definitions, use unsigned so we can compare reliably, since this
 * value is shifted up to bits 30/31.
 */
#define PIPE_ISOCHRONOUS    0U
#define PIPE_INTERRUPT      1U
#define PIPE_CONTROL        2U
#define PIPE_BULK           3U
#define PIPE_DEVEP_MASK     0x0007ff00

#define USB_ISOCHRONOUS    0
#define USB_INTERRUPT      1
#define USB_CONTROL        2
#define USB_BULK           3

#define USB_PIPE_TYPE_SHIFT	30
#define USB_PIPE_TYPE_MASK	(3 << USB_PIPE_TYPE_SHIFT)

#define USB_PIPE_DEV_SHIFT	8
#define USB_PIPE_DEV_MASK	(0x7f << USB_PIPE_DEV_SHIFT)

#define USB_PIPE_EP_SHIFT	15
#define USB_PIPE_EP_MASK	(0xf << USB_PIPE_EP_SHIFT)

/* USB-status codes: */
#define USB_ST_ACTIVE           0x1		/* TD is active */
#define USB_ST_STALLED          0x2		/* TD is stalled */
#define USB_ST_BUF_ERR          0x4		/* buffer error */
#define USB_ST_BABBLE_DET       0x8		/* Babble detected */
#define USB_ST_NAK_REC          0x10	/* NAK Received*/
#define USB_ST_CRC_ERR          0x20	/* CRC/timeout Error */
#define USB_ST_BIT_ERR          0x40	/* Bitstuff error */
#define USB_ST_NOT_PROC         0x80000000L	/* Not yet processed */

/*
 * CBI style
 */

#define US_CBI_ADSC		0

/* Command Block Wrapper */
struct umass_bbb_cbw {
	__u32		dCBWSignature;
#	define CBWSIGNATURE	0x43425355
	__u32		dCBWTag;
	__u32		dCBWDataTransferLength;
	__u8		bCBWFlags;
#	define CBWFLAGS_OUT	0x00
#	define CBWFLAGS_IN	0x80
#	define CBWFLAGS_SBZ	0x7f
	__u8		bCBWLUN;
	__u8		bCDBLength;
#	define CBWCDBLENGTH	16
	__u8		CBWCDB[CBWCDBLENGTH];
};
#define UMASS_BBB_CBW_SIZE	31

/* Command Status Wrapper */
struct umass_bbb_csw {
	__u32		dCSWSignature;
#	define CSWSIGNATURE	0x53425355
	__u32		dCSWTag;
	__u32		dCSWDataResidue;
	__u8		bCSWStatus;
#	define CSWSTATUS_GOOD	0x0
#	define CSWSTATUS_FAILED 0x1
#	define CSWSTATUS_PHASE	0x2
};
#define UMASS_BBB_CSW_SIZE	13

/*
 * BULK only
 */
#define US_BBB_RESET		0xff
#define US_BBB_GET_MAX_LUN	0xfe

#endif
