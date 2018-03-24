#ifndef __DWC3_PORT_CONFIG_
#define __DWC3_PORT_CONFIG_

//TODO: just for compile warning under 64-bit Linux Env, modify yourself
typedef unsigned long size_t;

#define NULL	0
#define offsetof(TYPE, MEMBER) ((size_t) &((TYPE *)0)->MEMBER)

#define MY_XHCI_BASE			  (0x60000000)
#define SYS_MEM_BASE_ADDR		0x80000000

// data structure mapping
#define DT_BASE					  (SYS_MEM_BASE_ADDR)
#define DWC3_REG				  (DT_BASE+0x0)
#define EVT_ADDR				  (DT_BASE+0x1000)
#define DCBAA_ADDR				(DT_BASE+0x3000)
#define SEG_ADDR				  (DT_BASE+0x4000)
#define SEGMENT_BASE			(DT_BASE+0x5000)

// buffer mapping
#define EVT_BUF_ADDR			(SYS_MEM_BASE_ADDR+0x2000)
#define SCRATCH_BUF				(SYS_MEM_BASE_ADDR+0x3000)

// print APIs
#define debug(args...)
#define printf(args...)
#define puts(args...)
#define dev_err(dev, args...)
#define dev_dbg(dev, args...)
#define dev_vdbg(dev, args...)
//TODO: specify your own serial output method
#define BUG(args...)
#define BUG_ON	BUG
#define get_timer(num)		num
//TODO: specify your own mem operation methods
#define memcpy(dst, src, size)
#define memset(addr, value, size)
#define memalign(align, size)	0
#define malloc(size)	(size)
#define free(args...)

#endif
