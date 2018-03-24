#ifndef __MEMALIGN_H_
#define __MEMALIGN_H_

#include <types.h>
#include <misc.h>

#define ALIGN(x,a)		__ALIGN_MASK((x),(typeof(x))(a)-1)
#define __ALIGN_MASK(x,mask)	(((x)+(mask))&~(mask))

#define ROUND(a,b)		(((a) + (b) - 1) & ~((b) - 1))

#define PAD_COUNT(s, pad) (((s) - 1) / (pad) + 1)
#define PAD_SIZE(s, pad) (PAD_COUNT(s, pad) * pad)
#define ALLOC_ALIGN_BUFFER_PAD(type, name, size, align, pad)		\
	char __##name[ROUND(PAD_SIZE((size) * sizeof(type), pad), align)  \
		      + (align - 1)];		\
									\
	type *name = (type *)ALIGN((uintptr_t)__##name, align)


#define ALLOC_ALIGN_BUFFER(type, name, size, align)		\
	ALLOC_ALIGN_BUFFER_PAD(type, name, size, align, 1)
#define ALLOC_CACHE_ALIGN_BUFFER(type, name, size)			\
	ALLOC_ALIGN_BUFFER(type, name, size, ARCH_DMA_MINALIGN)

/**
 * malloc_cache_aligned() - allocate a memory region aligned to cache line size
 *
 * This allocates memory at a cache-line boundary. The amount allocated may
 * be larger than requested as it is rounded up to the nearest multiple of the
 * cache-line size. This ensured that subsequent cache operations on this
 * memory (flush, invalidate) will not affect subsequently allocated regions.
 *
 * @size:	Minimum number of bytes to allocate
 *
 * @return pointer to new memory region, or NULL if there is no more memory
 * available.
 */
static inline void *malloc_cache_aligned(size_t size)
{
	//TODO: specify your own memalign
#ifdef LOCAL_SIM
	return malloc(size);
#else
	return memalign(ARCH_DMA_MINALIGN, ALIGN(size, ARCH_DMA_MINALIGN));
#endif
}

#endif
