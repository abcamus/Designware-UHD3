#ifndef __DWC3_IO_H_
#define __DWC3_IO_H_

#include <usb.h>

//TODO: define your own dwc3_readl function
static inline u32 dwc3_readl(void __iomem *base, u32 offset)
{
	//TODO: define your own DWC3_GLOBALS_REGS_START
//#define DWC3_GLOBALS_REGS_START		0
	unsigned long offs = offset - DWC3_GLOBALS_REGS_START;
	u32 value;

	/*
	 * We requested the mem region starting from the Globals address
	 * space, see dwc3_probe in core.c.
	 * However, the offsets are given starting from xHCI address space.
	 */
	//TODO: define readl method
	value = readl(base + offs);

	return value;
}

static inline void dwc3_writel(void __iomem *base, u32 offset, u32 value)
{
	//TODO: define your own DWC3_GLOBALS_REGS_START
	unsigned long offs = offset - DWC3_GLOBALS_REGS_START;

	/*
	 * We requested the mem region starting from the Globals address
	 * space, see dwc3_probe in core.c.
	 * However, the offsets are given starting from xHCI address space.
	 */
	//TODO: define writel method
	//writel(value, base + offs);
}

#endif
