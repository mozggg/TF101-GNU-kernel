/*
 * arch/arm/mach-tegra/board-harmony-sdhci.c
 *
 * Copyright (C) 2010 Google, Inc.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#include <linux/resource.h>
#include <linux/platform_device.h>
#include <linux/wlan_plat.h>
#include <linux/delay.h>
#include <linux/gpio.h>
#include <linux/clk.h>
#include <linux/err.h>
#include <linux/mmc/host.h>

#include <asm/mach-types.h>
#include <mach/irqs.h>
#include <mach/iomap.h>
#include <mach/sdhci.h>

#include "gpio-names.h"
#include "board.h"
#include "devices.h"

#if 0
#define TF101_WLAN_PWR	TEGRA_GPIO_PK5
#endif
#define TF101_WLAN_RST	TEGRA_GPIO_PK6

static void (*wifi_status_cb)(int card_present, void *dev_id);
static void *wifi_status_cb_devid;
static int tf101_wifi_status_register(void (*callback)(int , void *), void *);

static int tf101_wifi_reset(int on);
static int tf101_wifi_power(int on);
static int tf101_wifi_set_carddetect(int val);

static struct wifi_platform_data tf101_wifi_control = {
	.set_power	= tf101_wifi_power,
	.set_reset	= tf101_wifi_reset,
	.set_carddetect = tf101_wifi_set_carddetect,
};

static struct platform_device tf101_wifi_device = {
	.name		= "bcm4329_wlan",
	.id		= 1,
	.dev		= {
		.platform_data = &tf101_wifi_control,
	},
};

static struct embedded_sdio_data embedded_sdio_data1 = {
	.cccr   = {
		.sdio_vsn	= 2,
		.multi_block	= 1,
		.low_speed	= 0,
		.wide_bus	= 0,
		.high_power	= 1,
		.high_speed	= 1,
	},
	.cis  = {
		.vendor 	= 0x02d0,
		.device 	= 0x4329,
	},
};

static struct tegra_sdhci_platform_data tegra_sdhci_platform_data1 = {
	.mmc_data = {
		.register_status_notify	= tf101_wifi_status_register,
		.embedded_sdio = &embedded_sdio_data1,
		.built_in = 0,
	},
	.cd_gpio = -1,
	.wp_gpio = -1,
	.power_gpio = -1,
	.max_clk_limit = 40000000,
};

static struct tegra_sdhci_platform_data tegra_sdhci_platform_data3 = {
	.cd_gpio = TEGRA_GPIO_PI5,
	.wp_gpio = TEGRA_GPIO_PH1,
	.power_gpio = TEGRA_GPIO_PT3,
};

static struct tegra_sdhci_platform_data tegra_sdhci_platform_data4 = {
	.cd_gpio = -1,
	.wp_gpio = -1,
	.power_gpio = TEGRA_GPIO_PI6,
	.is_8bit = 1,
	.mmc_data = {
		.built_in = 1,
	}
};

static int tf101_wifi_status_register(
		void (*callback)(int card_present, void *dev_id),
		void *dev_id)
{
	pr_debug("%s: %p %p\n", __func__, callback, dev_id);
	if (wifi_status_cb)
		return -EAGAIN;
	wifi_status_cb = callback;
	wifi_status_cb_devid = dev_id;
	return 0;
}

static int tf101_wifi_set_carddetect(int val)
{
	pr_debug("%s: %d\n", __func__, val);
	if (wifi_status_cb)
		wifi_status_cb(val, wifi_status_cb_devid);
	else
		pr_warning("%s: Nobody to notify\n", __func__);
	return 0;
}

static int tf101_wifi_power(int on)
{
	pr_debug("%s: %d\n", __func__, on);

#if 0
	gpio_set_value(TF101_WLAN_PWR, on);
	mdelay(100);
#endif
	gpio_set_value(TF101_WLAN_RST, on);
	mdelay(200);

	return 0;
}

static int tf101_wifi_reset(int on)
{
	pr_debug("%s: do nothing\n", __func__);
	return 0;
}

static int __init tf101_wifi_init(void)
{
	//gpio_request(TF101_WLAN_PWR, "wlan_power");
	gpio_request(TF101_WLAN_RST, "wlan_rst");

	//tegra_gpio_enable(TF101_WLAN_PWR);
	tegra_gpio_enable(TF101_WLAN_RST);

	//gpio_direction_output(TF101_WLAN_PWR, 0);
	gpio_direction_output(TF101_WLAN_RST, 0);

	platform_device_register(&tf101_wifi_device);

	device_init_wakeup(&tf101_wifi_device.dev, 1);
	device_set_wakeup_enable(&tf101_wifi_device.dev, 0);

	return 0;
}

int __init tf101_sdhci_init(void)
{
	tegra_gpio_enable(tegra_sdhci_platform_data3.power_gpio);
	tegra_gpio_enable(tegra_sdhci_platform_data3.cd_gpio);
	tegra_gpio_enable(tegra_sdhci_platform_data3.wp_gpio);
	tegra_gpio_enable(tegra_sdhci_platform_data4.power_gpio);

	tegra_sdhci_device1.dev.platform_data = &tegra_sdhci_platform_data1;
	tegra_sdhci_device3.dev.platform_data = &tegra_sdhci_platform_data3;
	tegra_sdhci_device4.dev.platform_data = &tegra_sdhci_platform_data4;

	platform_device_register(&tegra_sdhci_device4);
	platform_device_register(&tegra_sdhci_device3);
	platform_device_register(&tegra_sdhci_device1);

	tf101_wifi_init();
	return 0;
}
