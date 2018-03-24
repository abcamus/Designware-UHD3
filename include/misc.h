#ifndef __MISC_H_
#define __MISC_H_

#define CONFIG_SYS_CACHELINE_SIZE		32

#define LOG2(x) (((x & 0xaaaaaaaa) ? 1 : 0) + ((x & 0xcccccccc) ? 2 : 0) + \
		 ((x & 0xf0f0f0f0) ? 4 : 0) + ((x & 0xff00ff00) ? 8 : 0) + \
		 ((x & 0xffff0000) ? 16 : 0))
#define LOG2_INVALID(type) ((type)((sizeof(type)<<3)-1))

#define uswap_16(x) \
	((((x) & 0xff00) >> 8) | \
	 (((x) & 0x00ff) << 8))
#define uswap_32(x) \
	((((x) & 0xff000000) >> 24) | \
	 (((x) & 0x00ff0000) >>  8) | \
	 (((x) & 0x0000ff00) <<  8) | \
	 (((x) & 0x000000ff) << 24))
#define _uswap_64(x, sfx) \
	((((x) & 0xff00000000000000##sfx) >> 56) | \
	 (((x) & 0x00ff000000000000##sfx) >> 40) | \
	 (((x) & 0x0000ff0000000000##sfx) >> 24) | \
	 (((x) & 0x000000ff00000000##sfx) >>  8) | \
	 (((x) & 0x00000000ff000000##sfx) <<  8) | \
	 (((x) & 0x0000000000ff0000##sfx) << 24) | \
	 (((x) & 0x000000000000ff00##sfx) << 40) | \
	 (((x) & 0x00000000000000ff##sfx) << 56))

#define cpu_to_le16(x)		(x)
#define cpu_to_le32(x)		(x)
#define cpu_to_le64(x)		(x)
#define le16_to_cpu(x)		(x)
#define le32_to_cpu(x)		(x)
#define le64_to_cpu(x)		(x)
#define le16_to_cpus(x)		(x)
#define be16_to_cpu(x)		uswap_16(x)
#define be32_to_cpu(x)		uswap_32(x)

#define upper_32_bits(n) ((u32)(((n) >> 16) >> 16))

/**
 * lower_32_bits - return bits 0-31 of a number
 * @n: the number we're accessing
 */
#define lower_32_bits(n) ((u32)(n))

//TODO: specify your own print class methods
//#define LOCAL_SIM
#define EMBEDDED_PORT
#ifdef EMBEDDED_PORT
#include <dwc3portconfig.h>
#else
#include <stdio.h>
#include <malloc.h>
#include <string.h>
#define debug(args...)			printf(args)
#define puts(args...)			printf(args)
#define dev_err(dev, args...)	printf(args)
#define dev_dbg(dev, args...)	printf(args)
#define dev_vdbg(dev, args...)	printf(args)
#define BUG()		printf("BUG") 
#define BUG_ON(args...)	
#define get_timer(num)		num
// Global Addrs, modified according to platform

#ifdef LOCAL_SIM
//char test_addr[0x10000];
#define test_addr	malloc(0x3000)
#ifndef g_regs
#define g_regs  test_addr
#endif
#ifndef g_mem
#define g_mem  test_addr
#endif
#ifndef dev_data_base
#define dev_data_base  test_addr
#endif
#ifndef dwc_base
#define dwc_base  test_addr
#endif
#ifndef evt_base
#define evt_base  test_addr
#endif
#if 0
#define EV_BUFFS  test_addr
#endif
#ifndef scratch_buf
#define scratch_buf  test_addr
#endif
#ifndef g_scratch_addr
#define g_scratch_addr  test_addr
#endif
#else
unsigned int g_regs[0x20000];
char g_mem[0x10000];
unsigned int dev_data_base[0x10000];
char dwc_base[0x100];
unsigned int evt_base[0x100];
char ev_buffs[0x1000];
char scratch_buf[0x1000];
char g_scratch_addr[0x1000];
#endif
#endif


#define get_unaligned(addr)		*addr
#define put_unaligned(val, addr)	*addr = val
#define __le16_to_cpu(val)	val

//TODO: compile related configs
#define __weak				__attribute__((weak))
#define __maybe_unused		__attribute__((unused))
#define  noinline	__attribute__((noinline))

//TODO: specify debug methods

//TODO: device APIs
#define usb_free_device(args...)
//#define usb_alloc_new_device(args...)
//#define usb_new_device(args...)

#define min(x, y) ({				\
	typeof(x) _min1 = (x);			\
	typeof(y) _min2 = (y);			\
	(void) (&_min1 == &_min2);		\
	_min1 < _min2 ? _min1 : _min2; })

#define max(x, y) ({				\
	typeof(x) _max1 = (x);			\
	typeof(y) _max2 = (y);			\
	(void) (&_max1 == &_max2);		\
	_max1 > _max2 ? _max1 : _max2; })

/**
 * clamp - return a value clamped to a given range with strict typechecking
 * @val: current value
 * @lo: lowest allowable value
 * @hi: highest allowable value
 *
 * This macro does strict typechecking of lo/hi to make sure they are of the
 * same type as val.  See the unnecessary pointer comparisons.
 */
#define clamp(val, lo, hi) min((typeof(val))max(val, lo), hi)

/*
 * ..and if you can't take the strict
 * types, you can specify one yourself.
 *
 * Or not use min/max/clamp at all, of course.
 */
#define min_t(type, x, y) ({			\
	type __min1 = (x);			\
	type __min2 = (y);			\
	__min1 < __min2 ? __min1: __min2; })

#define max_t(type, x, y) ({			\
	type __max1 = (x);			\
	type __max2 = (y);			\
	__max1 > __max2 ? __max1: __max2; })


#define TEST_ADDR	0x80000000

#define readl(addr)		*(volatile unsigned int *)(addr)
#define writel(val, addr)	*(volatile unsigned int *)(addr) = val


#define ARCH_DMA_MINALIGN	CONFIG_SYS_CACHELINE_SIZE
#define CONFIG_USB_XHCI_HCD

#define usb_reset_root_port(dev)

//TODO: all APIs need specified are here
#define simple_strtol(args...)	0
#define getenv(args...)		0
#define calloc(num, size)	malloc(num*size)

#endif
