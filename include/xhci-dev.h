#ifndef __ARCH_XHCI_DEV_H_
#define __ARCH_XHCI_DEV_H_

#include <usb.h>
#include <xhci.h>

struct arto_dwc_wrapper {
	u32 revision;

	u32 reserve_1[3];

	u32 sysconfig; /* offset of 0x10 */

	u32 reserve_2[3];
	u16 reserve_3;

	u32 irqstatus_raw_0; /* offset of 0x24 */
	u32 irqstatus_0;
	u32 irqenable_set_0;
	u32 irqenable_clr_0;

	u32 irqstatus_raw_1; /* offset of 0x34 */
	u32 irqstatus_1;
	u32 irqenable_set_1;
	u32 irqenable_clr_1;

	u32 reserve_4[15];

	u32 utmi_otg_ctrl; /* offset of 0x80 */
	u32 utmi_otg_status;

	u32 reserve_5[30];

	u32 mram_offset; /* offset of 0x100 */
	u32 fladj;
	u32 dbg_config;
	u32 dbg_data;
	u32 dev_ebc_en;
};

/* XHCI PHY register structure */
struct arto_usb3_phy {
	u32 reserve1;
	u32 pll_status;
	u32 pll_go;
	u32 pll_config_1;
	u32 pll_config_2;
	u32 pll_config_3;
	u32 pll_ssc_config_1;
	u32 pll_ssc_config_2;
	u32 pll_config_4;
};

struct arto_xhci {
	struct arto_dwc_wrapper *otg_wrapper;
	struct arto_usb3_phy *usb3_phy;
	struct xhci_hccr *hcd;
	struct dwc3 *dwc3_reg;
};

/* USB PHY functions */
void omap_enable_phy(struct arto_xhci *omap);
void omap_reset_usb_phy(struct dwc3 *dwc3_reg);
void usb_phy_power(int on);


#endif
