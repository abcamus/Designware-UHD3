#ifndef __DELAY_H_
#define __DELAY_H_

//TODO: define mdelay
static inline void mdelay(unsigned long msec)
{
	while (msec--);
}

#endif
