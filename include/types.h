#ifndef _LINUX_TYPES_H
#define _LINUX_TYPES_H

typedef unsigned char bool;
typedef unsigned char uchar;

typedef signed char s8;
typedef unsigned char u8;

typedef signed short s16;
typedef unsigned short u16;

typedef signed int s32;
typedef unsigned int u32;

typedef signed long long s64;
typedef unsigned long long u64;

typedef u32 dma_addr_t;

typedef unsigned long phys_addr_t;
typedef unsigned long phys_size_t;

typedef __signed__ char __s8;
typedef unsigned char __u8;

typedef __signed__ short __s16;
typedef unsigned short __u16;

typedef __signed__ int __s32;
typedef unsigned int __u32;

typedef unsigned long long __u64;
typedef		__u64		uint64_t;

typedef		__u8		uint8_t;
typedef		__u16		uint16_t;
typedef		__u32		uint32_t;
typedef unsigned long	ulong;

#define __bitwise
typedef __u16 __bitwise __le16;
typedef __u16 __bitwise __be16;
typedef __u32 __bitwise __le32;
typedef __u32 __bitwise __be32;
typedef __u64 __bitwise __le64;
typedef __u64 __bitwise __be64;

typedef unsigned long int uintptr_t;

typedef unsigned	gfp_t;

#endif
