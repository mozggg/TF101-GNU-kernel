/*
 * arch/arm/mach-tegra/board-tf101-panel.c
 *
 * Copyright (c) 2010-2011, NVIDIA Corporation.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
 */

#include <linux/delay.h>
#include <linux/gpio.h>
#include <linux/regulator/consumer.h>
#include <linux/resource.h>
#include <asm/mach-types.h>
#include <linux/platform_device.h>
#include <linux/earlysuspend.h>
#include <linux/pwm_backlight.h>
#include <linux/nvhost.h>
#include <linux/mm.h>
#include <mach/nvmap.h>
#include <mach/irqs.h>
#include <mach/iomap.h>
#include <mach/dc.h>
#include <mach/fb.h>

#include "devices.h"
#include "gpio-names.h"
#include "board.h"

#define tf101_pnl_pwr_enb	TEGRA_GPIO_PC6
#define tf101_bl_enb		TEGRA_GPIO_PD4
#define tf101_lvds_shutdown	TEGRA_GPIO_PB2
#define tf101_hdmi_hpd	TEGRA_GPIO_PN7
#define tf101_hdmi_enb	TEGRA_GPIO_PV5

/*panel power on sequence timing*/
#define tf101_pnl_to_lvds_ms	0
#define tf101_lvds_to_bl_ms	200

static struct regulator *tf101_hdmi_reg = NULL;
static struct regulator *tf101_hdmi_pll = NULL;


static int tf101_backlight_init(struct device *dev) {
	int ret;

	ret = gpio_request(tf101_bl_enb, "backlight_enb");
	if (ret < 0)
		return ret;

	ret = gpio_direction_output(tf101_bl_enb, 1);
	if (ret < 0)
		gpio_free(tf101_bl_enb);
	else
		tegra_gpio_enable(tf101_bl_enb);

	return ret;
};

static void tf101_backlight_exit(struct device *dev) {
	gpio_set_value(tf101_bl_enb, 0);
	gpio_free(tf101_bl_enb);
	tegra_gpio_disable(tf101_bl_enb);
}

static int tf101_backlight_notify(struct device *unused, int brightness)
{
	gpio_set_value(tf101_bl_enb, !!brightness);
	return brightness;
}

static int tf101_disp1_check_fb(struct device *dev, struct fb_info *info);

static struct platform_pwm_backlight_data tf101_backlight_data = {
	.pwm_id		= 2,
	.max_brightness	= 255,
	.dft_brightness	= 157,
	.pwm_period_ns	= 4000000,
	.init		= tf101_backlight_init,
	.exit		= tf101_backlight_exit,
	.notify		= tf101_backlight_notify,
	/* Only toggle backlight on fb blank notifications for disp1 */
	.check_fb   = tf101_disp1_check_fb,
};

static struct platform_device tf101_backlight_device = {
	.name	= "pwm-backlight",
	.id	= -1,
	.dev	= {
		.platform_data = &tf101_backlight_data,
	},
};

static int tf101_panel_enable(void)
{
	struct regulator *reg = regulator_get(NULL, "vdd_ldo4");

	if (reg) {
		regulator_enable(reg);
		regulator_put(reg);
	}

	gpio_set_value(tf101_pnl_pwr_enb, 1);
	mdelay(tf101_pnl_to_lvds_ms);
	gpio_set_value(tf101_lvds_shutdown, 1);
	mdelay(tf101_lvds_to_bl_ms);
	return 0;
}

static int tf101_panel_disable(void)
{
	gpio_set_value(tf101_lvds_shutdown, 0);
	gpio_set_value(tf101_pnl_pwr_enb, 0);
	return 0;
}

static int tf101_hdmi_enable(void)
{
	if (!tf101_hdmi_reg) {
		tf101_hdmi_reg = regulator_get(NULL, "avdd_hdmi"); /* LD07 */
		if (IS_ERR_OR_NULL(tf101_hdmi_reg)) {
			pr_err("hdmi: couldn't get regulator avdd_hdmi\n");
			tf101_hdmi_reg = NULL;
			return PTR_ERR(tf101_hdmi_reg);
		}
	}
	regulator_enable(tf101_hdmi_reg);

	if (!tf101_hdmi_pll) {
		tf101_hdmi_pll = regulator_get(NULL, "avdd_hdmi_pll"); /* LD08 */
		if (IS_ERR_OR_NULL(tf101_hdmi_pll)) {
			pr_err("hdmi: couldn't get regulator avdd_hdmi_pll\n");
			tf101_hdmi_pll = NULL;
			regulator_disable(tf101_hdmi_reg);
			tf101_hdmi_reg = NULL;
			return PTR_ERR(tf101_hdmi_pll);
		}
	}
	regulator_enable(tf101_hdmi_pll);
	return 0;
}

static int tf101_hdmi_disable(void)
{
	regulator_disable(tf101_hdmi_reg);
	regulator_disable(tf101_hdmi_pll);
	return 0;
}

static struct resource tf101_disp1_resources[] = {
	{
		.name	= "irq",
		.start	= INT_DISPLAY_GENERAL,
		.end	= INT_DISPLAY_GENERAL,
		.flags	= IORESOURCE_IRQ,
	},
	{
		.name	= "regs",
		.start	= TEGRA_DISPLAY_BASE,
		.end	= TEGRA_DISPLAY_BASE + TEGRA_DISPLAY_SIZE-1,
		.flags	= IORESOURCE_MEM,
	},
	{
		.name	= "fbmem",
		.flags	= IORESOURCE_MEM,
	},
};

static struct resource tf101_disp2_resources[] = {
	{
		.name	= "irq",
		.start	= INT_DISPLAY_B_GENERAL,
		.end	= INT_DISPLAY_B_GENERAL,
		.flags	= IORESOURCE_IRQ,
	},
	{
		.name	= "regs",
		.start	= TEGRA_DISPLAY2_BASE,
		.end	= TEGRA_DISPLAY2_BASE + TEGRA_DISPLAY2_SIZE - 1,
		.flags	= IORESOURCE_MEM,
	},
	{
		.name	= "fbmem",
		.flags	= IORESOURCE_MEM,
	},
	{
		.name	= "hdmi_regs",
		.start	= TEGRA_HDMI_BASE,
		.end	= TEGRA_HDMI_BASE + TEGRA_HDMI_SIZE - 1,
		.flags	= IORESOURCE_MEM,
	},
};

static struct tegra_dc_mode tf101_panel_modes[] = {
	{
		.pclk = 72072000,
		.h_ref_to_sync = 11,
		.v_ref_to_sync = 1,
		.h_sync_width = 58,
		.v_sync_width = 4,
		.h_back_porch = 58,
		.v_back_porch = 4,
		.h_active = 1280,
		.v_active = 800,
		.h_front_porch = 58,
		.v_front_porch = 4,
	},
};

static struct tegra_fb_data tf101_fb_data = {
	.win		= 0,
	.xres		= 1280,
	.yres		= 800,
	.bits_per_pixel	= 32,
	.flags		= TEGRA_FB_FLIP_ON_PROBE,
};

static struct tegra_fb_data tf101_hdmi_fb_data = {
	.win		= 0,
	.xres		= 1280,
	.yres		= 800,
	.bits_per_pixel	= 32,
	.flags		= TEGRA_FB_FLIP_ON_PROBE,
};

static struct tegra_dc_out tf101_disp1_out = {
	.type		= TEGRA_DC_OUT_RGB,

	.align		= TEGRA_DC_ALIGN_MSB,
	.order		= TEGRA_DC_ORDER_RED_BLUE,
	.depth		= 18,
	.dither		= TEGRA_DC_ORDERED_DITHER,

	.modes	 	= tf101_panel_modes,
	.n_modes 	= ARRAY_SIZE(tf101_panel_modes),

	.enable		= tf101_panel_enable,
	.disable	= tf101_panel_disable,
};

static struct tegra_dc_out tf101_disp2_out = {
	.type		= TEGRA_DC_OUT_HDMI,
	.flags		= TEGRA_DC_OUT_HOTPLUG_HIGH,

	.dcc_bus	= 1,
	.hotplug_gpio	= tf101_hdmi_hpd,

	.max_pixclock	= KHZ2PICOS(148500),

	.align		= TEGRA_DC_ALIGN_MSB,
	.order		= TEGRA_DC_ORDER_RED_BLUE,

	.enable		= tf101_hdmi_enable,
	.disable	= tf101_hdmi_disable,
};

static struct tegra_dc_platform_data tf101_disp1_pdata = {
	.flags		= TEGRA_DC_FLAG_ENABLED,
	.default_out	= &tf101_disp1_out,
	.fb		= &tf101_fb_data,
};

static struct tegra_dc_platform_data tf101_disp2_pdata = {
	.flags		= 0,
	.default_out	= &tf101_disp2_out,
	.fb		= &tf101_hdmi_fb_data,
};

static struct nvhost_device tf101_disp1_device = {
	.name		= "tegradc",
	.id		= 0,
	.resource	= tf101_disp1_resources,
	.num_resources	= ARRAY_SIZE(tf101_disp1_resources),
	.dev = {
		.platform_data = &tf101_disp1_pdata,
	},
};

static int tf101_disp1_check_fb(struct device *dev, struct fb_info *info)
{
	return info->device == &tf101_disp1_device.dev;
}

static struct nvhost_device tf101_disp2_device = {
	.name		= "tegradc",
	.id		= 1,
	.resource	= tf101_disp2_resources,
	.num_resources	= ARRAY_SIZE(tf101_disp2_resources),
	.dev = {
		.platform_data = &tf101_disp2_pdata,
	},
};

static struct nvmap_platform_carveout tf101_carveouts[] = {
	[0] = NVMAP_HEAP_CARVEOUT_IRAM_INIT,
	[1] = {
		.name		= "generic-0",
		.usage_mask	= NVMAP_HEAP_CARVEOUT_GENERIC,
		.buddy_size	= SZ_32K,
	},
};

static struct nvmap_platform_data tf101_nvmap_data = {
	.carveouts	= tf101_carveouts,
	.nr_carveouts	= ARRAY_SIZE(tf101_carveouts),
};

static struct platform_device tf101_nvmap_device = {
	.name	= "tegra-nvmap",
	.id	= -1,
	.dev	= {
		.platform_data = &tf101_nvmap_data,
	},
};

static struct platform_device *tf101_gfx_devices[] __initdata = {
	&tf101_nvmap_device,
#ifdef CONFIG_TEGRA_GRHOST
	&tegra_grhost_device,
#endif
	&tegra_pwfm2_device,
	&tf101_backlight_device,
};

#ifdef CONFIG_HAS_EARLYSUSPEND
/* put early_suspend/late_resume handlers here for the display in order
 * to keep the code out of the display driver, keeping it closer to upstream
 */
struct early_suspend tf101_panel_early_suspender;

static void tf101_panel_early_suspend(struct early_suspend *h)
{
	/* power down LCD, add use a black screen for HDMI */
	if (num_registered_fb > 0)
		fb_blank(registered_fb[0], FB_BLANK_POWERDOWN);
	if (num_registered_fb > 1)
		fb_blank(registered_fb[1], FB_BLANK_NORMAL);
#ifdef CONFIG_TEGRA_CONVSERVATIVE_GOV_ON_EARLYSUPSEND
	cpufreq_save_default_governor();
	cpufreq_set_conservative_governor();
	cpufreq_set_conservative_governor_param("up_threshold",
			SET_CONSERVATIVE_GOVERNOR_UP_THRESHOLD);

	cpufreq_set_conservative_governor_param("down_threshold",
			SET_CONSERVATIVE_GOVERNOR_DOWN_THRESHOLD);

	cpufreq_set_conservative_governor_param("freq_step",
		SET_CONSERVATIVE_GOVERNOR_FREQ_STEP);
#endif
}

static void tf101_panel_late_resume(struct early_suspend *h)
{
	unsigned i;
#ifdef CONFIG_TEGRA_CONVSERVATIVE_GOV_ON_EARLYSUPSEND
	cpufreq_restore_default_governor();
#endif
	for (i = 0; i < num_registered_fb; i++)
		fb_blank(registered_fb[i], FB_BLANK_UNBLANK);
}
#endif

void tf101_clear_framebuffer(unsigned long base, unsigned long size)
{
	void __iomem *io;

	BUG_ON(PAGE_ALIGN((unsigned long)base) != (unsigned long)base);
	BUG_ON(PAGE_ALIGN(size) != size);

	io = ioremap(base, size);
	if (!io) {
		pr_err("%s: Failed to map framebuffer\n", __func__);
		return;
	}

	memset(io, 0, size);
	iounmap(io);
}

int __init tf101_panel_init(void)
{
	int err;
	struct resource *res;

	gpio_request(tf101_pnl_pwr_enb, "pnl_pwr_enb");
	gpio_direction_output(tf101_pnl_pwr_enb, 1);
	tegra_gpio_enable(tf101_pnl_pwr_enb);

	gpio_request(tf101_lvds_shutdown, "lvds_shdn");
	gpio_direction_output(tf101_lvds_shutdown, 1);
	tegra_gpio_enable(tf101_lvds_shutdown);

	tegra_gpio_enable(tf101_hdmi_enb);
	gpio_request(tf101_hdmi_enb, "hdmi_5v_en");
	gpio_direction_output(tf101_hdmi_enb, 1);

	tegra_gpio_enable(tf101_hdmi_hpd);
	gpio_request(tf101_hdmi_hpd, "hdmi_hpd");
	gpio_direction_input(tf101_hdmi_hpd);

#ifdef CONFIG_HAS_EARLYSUSPEND
	tf101_panel_early_suspender.suspend = tf101_panel_early_suspend;
	tf101_panel_early_suspender.resume = tf101_panel_late_resume;
	tf101_panel_early_suspender.level = EARLY_SUSPEND_LEVEL_DISABLE_FB;
	register_early_suspend(&tf101_panel_early_suspender);
#endif

	tf101_carveouts[1].base = tegra_carveout_start;
	tf101_carveouts[1].size = tegra_carveout_size;

	err = platform_add_devices(tf101_gfx_devices,
				   ARRAY_SIZE(tf101_gfx_devices));

	tf101_clear_framebuffer(tegra_fb_start, tegra_fb_size);

#if defined(CONFIG_TEGRA_GRHOST) && defined(CONFIG_TEGRA_DC)
	res = nvhost_get_resource_byname(&tf101_disp1_device,
		IORESOURCE_MEM, "fbmem");
	res->start = tegra_fb_start;
	res->end = tegra_fb_start + tegra_fb_size - 1;

	res = nvhost_get_resource_byname(&tf101_disp2_device,
		IORESOURCE_MEM, "fbmem");
	res->start = tegra_fb2_start;
	res->end = tegra_fb2_start + tegra_fb2_size - 1;

	if (!err)
		err = nvhost_device_register(&tf101_disp1_device);

	if (!err)
		err = nvhost_device_register(&tf101_disp2_device);
#endif

	return err;
}

