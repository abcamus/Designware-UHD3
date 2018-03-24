#ifndef __DWC3_PORT_H
#define __DWC3_PORT_H
#include <types.h>

/* Global constants */
#define DWC3_ENDPOINTS_NUM			32

#define DWC3_EVENT_TYPE_MASK			0xfe

#define DWC3_EVENT_TYPE_DEV			0
#define DWC3_EVENT_TYPE_CARKIT			3
#define DWC3_EVENT_TYPE_I2C			4

#define DWC3_DEVICE_EVENT_DISCONNECT		0
#define DWC3_DEVICE_EVENT_RESET			1
#define DWC3_DEVICE_EVENT_CONNECT_DONE		2
#define DWC3_DEVICE_EVENT_LINK_STATUS_CHANGE	3
#define DWC3_DEVICE_EVENT_WAKEUP		4
#define DWC3_DEVICE_EVENT_EOPF			6
#define DWC3_DEVICE_EVENT_SOF			7
#define DWC3_DEVICE_EVENT_ERRATIC_ERROR		9
#define DWC3_DEVICE_EVENT_CMD_CMPL		10
#define DWC3_DEVICE_EVENT_OVERFLOW		11

#define DWC3_GEVNTCOUNT_MASK			0xfffc
#define DWC3_GSNPSID_MASK			0xffff0000
#define DWC3_GSNPSID_SHIFT			16
#define DWC3_GSNPSREV_MASK			0xffff

#define DWC3_REVISION_MASK			0xffff

#define DWC3_REG_OFFSET				0xC100

struct g_event_buffer {
	u32 g_evntadrlo;
	u32 g_evntadrhi;
	u32 g_evntsiz;
	u32 g_evntcount;
};

enum usb_dr_mode {
	USB_DR_MODE_UNKNOWN,
	USB_DR_MODE_HOST,
	USB_DR_MODE_PERIPHERAL,
	USB_DR_MODE_OTG,
};

struct dwc3_device {
	unsigned long base;
	enum usb_dr_mode dr_mode;
	u32 maximum_speed;
	unsigned tx_fifo_resize:1;
	unsigned has_lpm_erratum;
	u8 lpm_nyet_threshold;
	unsigned is_utmi_l1_suspend;
	u8 hird_threshold;
	unsigned disable_scramble_quirk;
	unsigned u2exit_lfps_quirk;
	unsigned u2ss_inp3_quirk;
	unsigned req_p1p2p3_quirk;
	unsigned del_p1p2p3_quirk;
	unsigned del_phy_power_chg_quirk;
	unsigned lfps_filter_quirk;
	unsigned rx_detect_poll_quirk;
	unsigned dis_u3_susphy_quirk;
	unsigned dis_u2_susphy_quirk;
	unsigned tx_de_emphasis_quirk;
	unsigned tx_de_emphasis;
	int index;
};

struct d_physical_endpoint {
	u32 d_depcmdpar2;
	u32 d_depcmdpar1;
	u32 d_depcmdpar0;
	u32 d_depcmd;
};

#if 1
struct dwc3_regs {					/* offset: 0xC100 */
	u32 g_sbuscfg0;
	u32 g_sbuscfg1;
	u32 g_txthrcfg;
	u32 g_rxthrcfg;
	u32 g_ctl;

	u32 reserved1;

	u32 g_sts;

	u32 reserved2;

	u32 g_snpsid;
	u32 g_gpio;
	u32 g_uid;
	u32 g_uctl;
	u64 g_buserraddr;
	u64 g_prtbimap;

	u32 g_hwparams0;
	u32 g_hwparams1;
	u32 g_hwparams2;
	u32 g_hwparams3;
	u32 g_hwparams4;
	u32 g_hwparams5;
	u32 g_hwparams6;
	u32 g_hwparams7;

	u32 g_dbgfifospace;
	u32 g_dbgltssm;
	u32 g_dbglnmcc;
	u32 g_dbgbmu;
	u32 g_dbglspmux;
	u32 g_dbglsp;
	u32 g_dbgepinfo0;
	u32 g_dbgepinfo1;

	u64 g_prtbimap_hs;
	u64 g_prtbimap_fs;

	u32 reserved3[28];

	u32 g_usb2phycfg[16];
	u32 g_usb2i2cctl[16];
	u32 g_usb2phyacc[16];
	u32 g_usb3pipectl[16];

	u32 g_txfifosiz[32];
	u32 g_rxfifosiz[32];

	struct g_event_buffer g_evnt_buf[32];

	u32 g_hwparams8;

	u32 reserved4[11];

	u32 g_fladj;

	u32 reserved5[51];

	u32 d_cfg;
	u32 d_ctl;
	u32 d_evten;
	u32 d_sts;
	u32 d_gcmdpar;
	u32 d_gcmd;

	u32 reserved6[2];

	u32 d_alepena;

	u32 reserved7[55];

	struct d_physical_endpoint d_phy_ep_cmd[32];

	u32 reserved8[128];

	u32 o_cfg;
	u32 o_ctl;
	u32 o_evt;
	u32 o_evten;
	u32 o_sts;

	u32 reserved9[3];

	u32 adp_cfg;
	u32 adp_ctl;
	u32 adp_evt;
	u32 adp_evten;

	u32 bc_cfg;

	u32 reserved10;

	u32 bc_evt;
	u32 bc_evten;
};
#endif

#endif
