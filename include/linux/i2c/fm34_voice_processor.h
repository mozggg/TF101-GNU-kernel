#ifndef __FM34_H__
#define __FM34_H__

#define FM34_IOC_MAGIC		0xf3
#define FM34_IOCTL_SENABLE	_IOW(FM34_IOC_MAGIC, 1, int)

#ifdef __KERNEL__

struct fm34_platform_data {
	int gpio_reset;
	int gpio_en;
};

#endif /* __KERNEL__ */

#endif /* __FM34_H__ */
