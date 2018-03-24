#include <usb.h>
#include <xhci.h>
#include <xhci-dev.h>
#include <misc.h>

#define __weak	__attribute__((weak))

static struct arto_xhci arto;

__weak void usb_phy_power(int on)
{
	return;
}

static int artosyn_xhci_core_init(struct arto_xhci *arto)
{
	int ret = 0;

	usb_phy_power(1);
	//arto_enable_phy();
	//
	
	ret = dwc3_core_init(arto->dwc3_reg);

	dwc3_set_mode(arto->dwc3_reg, DWC3_GCTL_PRTCAP_HOST);

	return ret;
}

int xhci_hcd_init(int index, struct xhci_hccr **hccr, struct xhci_hcor **hcor)
{
	struct arto_xhci *ctx = &arto;
	int ret = 0;

#ifndef MY_XHCI_BASE
printf("MY XHCI BASE not defined: ");
#ifdef LOCAL_SIM
printf("Use Unix Environment.\n");
#define MY_XHCI_BASE		g_regs	
#endif
#else
	ctx->hcd = (struct xhci_hccr *)MY_XHCI_BASE;
#endif

#ifdef EMBEDDED_PORT
	ctx->dwc3_reg = (void *)DWC3_REG;
#endif
	ctx->dwc3_reg->regs = (struct dwc3_regs *)((char *)(ctx->hcd) + DWC3_REG_OFFSET);
	//TODO: specify PHY Base and WRAPPER
	//ctx->usb3_phy = (struct omap_usb3_phy *)OMAP_OCP1_SCP_BASE;
	//ctx->otg_wrapper = (struct omap_dwc_wrapper *)OMAP_OTG_WRAPPER_BASE;

	debug("%s:%d.\n", __func__, __LINE__);
	ret = board_usb_init(index, USB_INIT_HOST);
#ifdef EMBEDDED_PORT
	if (ret != 0) {
		puts("Failed to initialize board for USB\n");
		return ret;
	}
#endif

	debug("artosyn xhci core init.\n");
	ret = artosyn_xhci_core_init(ctx);
#ifdef EMBEDDED_PORT
	if (ret < 0) {
		//TODO: specify puts method
		puts("Failed to initialize xhci\n");
		return ret;
	}
#endif

	*hccr = (struct xhci_hccr *)(MY_XHCI_BASE);
	*hcor = (struct xhci_hcor *)( *hccr
				+ HC_LENGTH(xhci_readl(&(*hccr)->cr_capbase)));

	//TODO: specify debug method
	debug("arto-xhci: init hccr %p and hcor %p hc_length %d\n",
	      *hccr, *hcor,
	      (uint32_t)HC_LENGTH(xhci_readl(&(*hccr)->cr_capbase)));

	return ret;
}
