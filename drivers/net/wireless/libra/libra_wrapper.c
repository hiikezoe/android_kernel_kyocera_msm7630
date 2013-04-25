/*
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#include <linux/libra_sdioif.h>
#include <linux/mfd/pm8xxx/gpio.h>
#include <linux/netdevice.h>

void _set_bit_le(int nr, volatile unsigned long * p)
{
	_set_bit(nr, p);
}
EXPORT_SYMBOL(_set_bit_le);

#define PM8058_GPIO_PM_TO_SYS(pm_gpio)     (pm_gpio + NR_GPIO_IRQS)
int pm8058_gpio_config(int gpio, struct pm_gpio *param)
{
	return pm8xxx_gpio_config(PM8058_GPIO_PM_TO_SYS(gpio), param);
}
EXPORT_SYMBOL(pm8058_gpio_config);

extern struct net_device *alloc_etherdev_mqs(int sizeof_priv, unsigned int txqs,
					    unsigned int rxqs);
struct net_device * alloc_etherdev_mq(int sizeof_priv, unsigned int count)
{
	return alloc_etherdev_mqs(sizeof_priv, count, count);
}
EXPORT_SYMBOL(alloc_etherdev_mq);

void libra_sdio_set_wifi_power(int power)
{
}
EXPORT_SYMBOL(libra_sdio_set_wifi_power);
