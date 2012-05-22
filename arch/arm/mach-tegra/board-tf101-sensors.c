/*
 * arch/arm/mach-tegra/board-tf101-sensors.c
 *
 * Copyright (c) 2011, NVIDIA CORPORATION, All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are
 * met:
 *
 * Redistributions of source code must retain the above copyright notice,
 * this list of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright
 * notice, this list of conditions and the following disclaimer in the
 * documentation and/or other materials provided with the distribution.
 *
 * Neither the name of NVIDIA CORPORATION nor the names of its contributors
 * may be used to endorse or promote products derived from this software
 * without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS
 * IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED
 * TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A
 * PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 * HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED
 * TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
 * PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
 * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
 * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include <linux/delay.h>
#include <linux/i2c.h>
#include <linux/mpu.h>
#include <linux/i2c/pca954x.h>
#include <linux/i2c/pca953x.h>
#include <linux/nct1008.h>
#include <linux/err.h>
#include <linux/regulator/consumer.h>
#include <linux/nvhost.h>
#include <linux/nvhost_ioctl.h>
#include <linux/platform_device.h>
#include <linux/dma-mapping.h>
#include <linux/clk.h>

#include <mach/gpio.h>
#include <mach/iomap.h>

#include <media/soc_camera.h>
#include <media/tegra_v4l2_camera.h>
#include <generated/mach-types.h>

#include "gpio-names.h"
#include "board.h"
#include "board-tf101.h"
#include "cpu-tegra.h"

#include "../../../drivers/video/tegra/host/host1x/host1x_syncpt.h"

#define LIGHT_IRQ_GPIO          TEGRA_GPIO_PZ2
#define PROX_IRQ_GPIO           TEGRA_GPIO_PG0
#define AKM8975_IRQ_GPIO        TEGRA_GPIO_PN5
#define CAMERA_POWER_GPIO       TEGRA_GPIO_PK3
#define CAMERA_CSI_MUX_SEL_GPIO TEGRA_GPIO_PBB4
#define AC_PRESENT_GPIO         TEGRA_GPIO_PV3
#define CAP_IRQ_GPIO            TEGRA_GPIO_PX4
#define NCT1008_THERM2_GPIO     TEGRA_GPIO_PN6
#define YUV_SENSOR_OE_L_GPIO    TEGRA_GPIO_PL2
#define YUV_SENSOR_RST_GPIO     TEGRA_GPIO_PU2

static struct regulator *reg_p_cam_avdd; /* LDO0 */
static struct regulator *reg_tegra_cam;    /* LDO6 */

static struct clk *clk_vi;
static struct clk *clk_vi_sensor;
static struct clk *clk_csi;
static struct clk *clk_isp;
static struct clk *clk_csus;

enum {
	GPIO_FREE = 0,
	GPIO_REQUESTED,
};

struct camera_gpios {
	const char *name;
	int gpio;
	int enabled;
        int milliseconds;
        int requested;
};
#define CAMERA_GPIO(_name, _gpio, _enabled, _milliseconds, _requested)		\
	{						                        \
		.name = _name,				                        \
		.gpio = _gpio,				                        \
		.enabled = _enabled,			                        \
		.milliseconds = _milliseconds,				        \
		.requested = _requested,			                \
	}

static struct camera_gpios yuv_sensor_gpio_keys[] = {
	[0] = CAMERA_GPIO("mipi_power_en", AVDD_DSI_CSI_ENB_GPIO, 1, 0, GPIO_FREE),
	[1] = CAMERA_GPIO("cam_power_en", CAMERA_POWER_GPIO, 0, 0, GPIO_FREE),
	[2] = CAMERA_GPIO("cam_csi_sel", CAMERA_CSI_MUX_SEL_GPIO, 0, 0, GPIO_FREE),
//	[1] = CAMERA_GPIO("yuv_sensor_oe_l", YUV_SENSOR_OE_L_GPIO, 0, 0, GPIO_FREE),
	[3] = CAMERA_GPIO("yuv_sensor_rst_lo", YUV_SENSOR_RST_GPIO, 1, 0, GPIO_FREE),	//Kenji+
};

static int yuv_sensor_power_on(void)
{
	int ret;
	int i;

  printk("yuv_sensor_power_on+\n");
  if (!reg_tegra_cam) {
//    reg_tegra_cam = regulator_get(NULL, "tegra_camera");
    reg_tegra_cam = regulator_get(NULL, "vcsi");
    if (IS_ERR_OR_NULL(reg_tegra_cam)) {
      pr_err("EP101_ov5640_power_on LDO6: p_tegra_cam failed\n");
      regulator_put(reg_p_cam_avdd);
      reg_tegra_cam = NULL;
      return PTR_ERR(reg_tegra_cam);
    }
    regulator_set_voltage(reg_tegra_cam, 1800000, 1800000);
    pr_err("EP101_ov5640_power_on LDO6: p_tegra_cam OK\n");
    regulator_enable(reg_tegra_cam);
  }
  msleep(1);
  if (!reg_p_cam_avdd) {
    reg_p_cam_avdd = regulator_get(NULL, "p_cam_avdd");
    if (IS_ERR_OR_NULL(reg_p_cam_avdd)) {
      pr_err("EP101_ov5640_power_on LDO0: p_cam_avdd failed\n");
      reg_p_cam_avdd = NULL;
      return PTR_ERR(reg_p_cam_avdd);
    }
    regulator_set_voltage(reg_p_cam_avdd, 2850000, 2850000);
    pr_err("EP101_ov5640_power_on LDO0: p_cam_avdd OK\n");
    regulator_enable(reg_p_cam_avdd);
  }
  msleep(5);
  for (i = 0; i < ARRAY_SIZE(yuv_sensor_gpio_keys); i++) {
    tegra_gpio_enable(yuv_sensor_gpio_keys[i].gpio);
    pr_info("gpio %d set to %d\n",yuv_sensor_gpio_keys[i].gpio, yuv_sensor_gpio_keys[i].enabled);
    ret = gpio_request(yuv_sensor_gpio_keys[i].gpio,
    yuv_sensor_gpio_keys[i].name);
    if (ret < 0) {
      pr_err("%s: gpio_request failed for gpio #%d\n",
      __func__, i);
      goto fail;
    }
    msleep(1);
    gpio_direction_output(yuv_sensor_gpio_keys[i].gpio,
    yuv_sensor_gpio_keys[i].enabled);
    gpio_export(yuv_sensor_gpio_keys[i].gpio, false);
  }
  printk("yuv_sensor_power_on-\n");
	return 0;
fail:
  regulator_disable(reg_p_cam_avdd);
  regulator_disable(reg_tegra_cam);
  regulator_put(reg_p_cam_avdd);
  regulator_put(reg_tegra_cam);
	while (i--)
		gpio_free(yuv_sensor_gpio_keys[i].gpio);
	return ret;
}

static int yuv_sensor_power_off(void)
{
  int i;

  printk("yuv_sensor_power_off+\n");
  for (i=ARRAY_SIZE(yuv_sensor_gpio_keys)-1;i>=0; i--) {
    pr_info("gpio %d set to %d\n",yuv_sensor_gpio_keys[i].gpio, !(yuv_sensor_gpio_keys[i].enabled));
    gpio_direction_output(yuv_sensor_gpio_keys[i].gpio,
    !(yuv_sensor_gpio_keys[i].enabled));
    gpio_export(yuv_sensor_gpio_keys[i].gpio, false);
  }
  msleep(1);
  if(reg_p_cam_avdd){
    regulator_disable(reg_p_cam_avdd);
    regulator_put(reg_p_cam_avdd);
    reg_p_cam_avdd = NULL;
  	pr_err("EP101_ov5640_power_off LDO0: p_cam_avdd OK\n");
  }
  msleep(1);
  if(reg_tegra_cam){
  	regulator_disable(reg_tegra_cam);
  	regulator_put(reg_tegra_cam);
    reg_tegra_cam = NULL;
  	pr_err("EP101_ov5640_power_off LDO6: p_tegra_cam OK\n");
  }

  i = ARRAY_SIZE(yuv_sensor_gpio_keys);
  while (i--)
    gpio_free(yuv_sensor_gpio_keys[i].gpio);
  printk("yuv_sensor_power_off-\n");
  return 0;
}

#if 0
struct yuv_sensor_platform_data yuv_sensor_data = {
	.power_on = yuv_sensor_power_on,
	.power_off = yuv_sensor_power_off,
};
#endif

#define FRONT_CAMERA_POWER_GPIO	TEGRA_GPIO_PK4
#define FRONT_YUV_SENSOR_RST_GPIO     TEGRA_GPIO_PU3

static struct camera_gpios yuv_front_sensor_gpio_keys[] = {
	[0] = CAMERA_GPIO("mipi_power_en", AVDD_DSI_CSI_ENB_GPIO, 1, 0, GPIO_FREE),
	[1] = CAMERA_GPIO("cam_power_en", FRONT_CAMERA_POWER_GPIO, 0, 0, GPIO_FREE),
	[2] = CAMERA_GPIO("yuv_sensor_rst_lo", FRONT_YUV_SENSOR_RST_GPIO, 1, 0, GPIO_FREE),
};

static int yuv_front_sensor_power_on(void)
{
	int ret;
	int i;
  printk("yuv_front_sensor_power_on+\n");
  if (!reg_tegra_cam) {
//    reg_tegra_cam = regulator_get(NULL, "tegra_camera");
    reg_tegra_cam = regulator_get(NULL, "vcsi");
    if (IS_ERR_OR_NULL(reg_tegra_cam)) {
      pr_err("IO Power Failed LDO6: p_tegra_cam failed\n");
      regulator_put(reg_p_cam_avdd);
      reg_tegra_cam = NULL;
      return PTR_ERR(reg_tegra_cam);
    }
    regulator_set_voltage(reg_tegra_cam, 1800000, 1800000);
    pr_err("EP101_mi1040_power_on LDO6: p_tegra_cam OK\n");
    regulator_enable(reg_tegra_cam);
  }
  if (!reg_p_cam_avdd) {
    reg_p_cam_avdd = regulator_get(NULL, "p_cam_avdd");
    if (IS_ERR_OR_NULL(reg_p_cam_avdd)) {
      pr_err("AVDD Failed: p_cam_avdd failed\n");
      reg_p_cam_avdd = NULL;
      return PTR_ERR(reg_p_cam_avdd);
    }
    regulator_set_voltage(reg_p_cam_avdd, 2850000, 2850000);
    pr_err("EP101_mi1040_power_on LDO0: p_cam_avdd OK\n");
    regulator_enable(reg_p_cam_avdd);
  }

  for (i = 0; i < ARRAY_SIZE(yuv_front_sensor_gpio_keys); i++) {
    tegra_gpio_enable(yuv_front_sensor_gpio_keys[i].gpio);
    pr_info("gpio %d set to %d\n",yuv_front_sensor_gpio_keys[i].gpio,
      yuv_front_sensor_gpio_keys[i].enabled);
    ret = gpio_request(yuv_front_sensor_gpio_keys[i].gpio,
      yuv_front_sensor_gpio_keys[i].name);
    if (ret < 0) {
      pr_err("%s: gpio_request failed for gpio #%d\n",
      __func__, i);
      goto fail;
    }
    gpio_direction_output(yuv_front_sensor_gpio_keys[i].gpio,
    yuv_front_sensor_gpio_keys[i].enabled);
    gpio_export(yuv_front_sensor_gpio_keys[i].gpio, false);
  }
  printk("yuv_front_sensor_power_on-\n");
	return 0;
fail:
  regulator_disable(reg_p_cam_avdd);
  regulator_disable(reg_tegra_cam);
  regulator_put(reg_p_cam_avdd);
  regulator_put(reg_tegra_cam);
	while (i--)
		gpio_free(yuv_sensor_gpio_keys[i].gpio);
	return ret;
}

static int yuv_front_sensor_power_off(void)
{
  int i;

  printk("yuv_front_sensor_power_off+\n");
  for (i=ARRAY_SIZE(yuv_front_sensor_gpio_keys)-1;i>=0; i--) {
    pr_info("gpio %d set to %d\n",yuv_front_sensor_gpio_keys[i].gpio, !(yuv_front_sensor_gpio_keys[i].enabled));
    gpio_direction_output(yuv_front_sensor_gpio_keys[i].gpio,
    !(yuv_front_sensor_gpio_keys[i].enabled));
    gpio_export(yuv_front_sensor_gpio_keys[i].gpio, false);
  }
  if(reg_p_cam_avdd){
  	regulator_disable(reg_p_cam_avdd);
    regulator_put(reg_p_cam_avdd);
    reg_p_cam_avdd = NULL;
  	pr_err("EP101_mi1040_power_off LDO0: p_cam_avdd OK\n");
  }
  if(reg_tegra_cam){
  	regulator_disable(reg_tegra_cam);
  	regulator_put(reg_tegra_cam);
    reg_tegra_cam = NULL;
  	pr_err("EP101_mi1040_power_off LDO6: p_tegra_cam OK\n");
  }

  i = ARRAY_SIZE(yuv_front_sensor_gpio_keys);
  while (i--)
    gpio_free(yuv_front_sensor_gpio_keys[i].gpio);
  printk("yuv_front_sensor_power_off-\n");
  return 0;
}

#if 0
struct yuv_sensor_platform_data yuv_front_sensor_data = {
	.power_on = yuv_front_sensor_power_on,
	.power_off = yuv_front_sensor_power_off,
};
#endif

static void tf101_nct1008_init(void)
{
	tegra_gpio_enable(NCT1008_THERM2_GPIO);
	gpio_request(NCT1008_THERM2_GPIO, "temp_alert");
	gpio_direction_input(NCT1008_THERM2_GPIO);
}

static struct nct1008_platform_data tf101_nct1008_pdata = {
	.supported_hwrev = true,
	.ext_range = false,
	.conv_rate = 0x08,
	.offset = 0,
	.hysteresis = 0,
	.shutdown_ext_limit = 90,
	.shutdown_local_limit = 96,
	.throttling_ext_limit = 49,
	.alarm_fn = tegra_throttling_enable,
};

static const struct i2c_board_info tf101_i2c2_board_info[] = {
	{
		I2C_BOARD_INFO("bq20z75", 0x0B),
	},
#ifdef CONFIG_INPUT_ASUSEC
	{
		I2C_BOARD_INFO("asusec", 0x19),
		.irq = TEGRA_GPIO_TO_IRQ(TEGRA_GPIO_PS2),
	},
#endif
	{
		I2C_BOARD_INFO("al3000a", 0x1c),
		.irq = TEGRA_GPIO_TO_IRQ(LIGHT_IRQ_GPIO),
	},
	{
		I2C_BOARD_INFO("prox_lds6202", 0x2c),
		.irq = TEGRA_GPIO_TO_IRQ(PROX_IRQ_GPIO),
	},
};

static struct i2c_board_info tf101_i2c3_board_info[] = {
	{
		I2C_BOARD_INFO("ov5640", 0x3c),
	},
#if 0
	{
		I2C_BOARD_INFO("mi1040", 0x48), /* MT9D115 */
		.platform_data = &yuv_front_sensor_data,
 	},
#endif
};

static struct i2c_board_info tf101_i2c4_board_info[] = {
	{
		I2C_BOARD_INFO("nct1008", 0x4C),
		.irq = TEGRA_GPIO_TO_IRQ(NCT1008_THERM2_GPIO),
		.platform_data = &tf101_nct1008_pdata,
	},
};

static int tegra_camera_enable(struct nvhost_device *ndev)
{
	/* Turn on relevant clocks. */
	clk_enable(clk_vi);
	clk_enable(clk_vi_sensor);
	clk_enable(clk_csi);
	clk_enable(clk_isp);
	clk_enable(clk_csus);

	return yuv_sensor_power_on();
}

static void tegra_camera_disable(struct nvhost_device *ndev)
{
	yuv_sensor_power_off();

	/* Turn off relevant clocks. */
	clk_disable(clk_vi);
	clk_disable(clk_vi_sensor);
	clk_disable(clk_csi);
	clk_disable(clk_isp);
	clk_disable(clk_csus);
}

static struct soc_camera_link ov5640_iclink = {
	.bus_id         = 0,
	.i2c_adapter_id = 3,
	.board_info     = &tf101_i2c3_board_info[0],
	.module_name    = "ov5640",
};

static struct platform_device soc_camera = {
	.name   = "soc-camera-pdrv",
	.id     = 0,
	.dev    = {
		.platform_data = &ov5640_iclink,
	},
};

static struct tegra_camera_platform_data tegra_camera_platform_data = {
	.enable_camera          = tegra_camera_enable,
	.disable_camera         = tegra_camera_disable,
	.flip_v                 = 1,
	.flip_h                 = 0,
};

static struct resource tegra_camera_resources[] = {
	{
		.name   = "regs",
		.start  = TEGRA_VI_BASE,
		.end    = TEGRA_VI_BASE + TEGRA_VI_SIZE - 1,
		.flags  = IORESOURCE_MEM,
	},
};

static u64 tegra_camera_dma_mask = DMA_BIT_MASK(32);

struct nvhost_device tegra_camera_device = {
	.name           = "tegra-camera",
	.id             = 0,
	.resource       = tegra_camera_resources,
	.num_resources  = ARRAY_SIZE(tegra_camera_resources),
	.dev            = {
		.dma_mask               = &tegra_camera_dma_mask,
		.coherent_dma_mask      = 0xffffffff,
		.platform_data          = &tegra_camera_platform_data,
	},
	.syncpts       = BIT(NVSYNCPT_CSI_VI_0) | BIT(NVSYNCPT_CSI_VI_1) |
	                 BIT(NVSYNCPT_VI_ISP_0) | BIT(NVSYNCPT_VI_ISP_1) |
			 BIT(NVSYNCPT_VI_ISP_2) | BIT(NVSYNCPT_VI_ISP_3) |
			 BIT(NVSYNCPT_VI_ISP_4),
	.modulemutexes = BIT(8),
	.exclusive     = true,
	NVHOST_MODULE_NO_POWERGATE_IDS,
	NVHOST_DEFAULT_CLOCKGATE_DELAY,
	.moduleid      = NVHOST_MODULE_VI,
};

#ifdef CONFIG_MPU_SENSORS_MPU3050
static struct mpu_platform_data mpu_pdata = {
	.int_config  = 0x10,
	.orientation = { 0, 1, 0, -1, 0, 0, 0, 0, 1 },  /* Orientation matrix for MPU on tf101 */
	.level_shifter = 0,

#ifdef CONFIG_MPU_SENSORS_KXTF9
	.accel = {
		.address     = MPU_ACCEL_ADDR,
		.irq         = TEGRA_GPIO_TO_IRQ(TEGRA_GPIO_PN4),
		.adapt_num   = 0,
		.bus         = EXT_SLAVE_BUS_SECONDARY,
		.orientation = { 1, 0, 0, 0, 1, 0, 0, 0, 1 },
	},
#endif
#ifdef CONFIG_MPU_SENSORS_AMI306
	.compass = {
		.address     = MPU_COMPASS_ADDR,
		.irq         = TEGRA_GPIO_TO_IRQ(TEGRA_GPIO_PN5),
		.adapt_num   = 0,
		.bus         = EXT_SLAVE_BUS_PRIMARY,
		.orientation = { -1, 0, 0, 0, 1, 0, 0, 0, -1 },
        },
#endif
};

static struct i2c_board_info __initdata mpu3050_i2c0_boardinfo[] = {
	{
		I2C_BOARD_INFO(MPU_GYRO_NAME, 0x68),
		.irq = TEGRA_GPIO_TO_IRQ(TEGRA_GPIO_PZ4),
		.platform_data = &mpu_pdata,
	},
#ifdef CONFIG_MPU_SENSORS_KXTF9
	{
		I2C_BOARD_INFO(MPU_ACCEL_NAME, MPU_ACCEL_ADDR),
		.irq = TEGRA_GPIO_TO_IRQ(TEGRA_GPIO_PN4),
		.platform_data = &mpu_pdata.accel,
	},
#endif
#ifdef CONFIG_MPU_SENSORS_AMI306
	{
		I2C_BOARD_INFO(MPU_COMPASS_NAME, MPU_COMPASS_ADDR),
		.irq = TEGRA_GPIO_TO_IRQ(TEGRA_GPIO_PN5),
		.platform_data = &mpu_pdata.compass,
	},
#endif
};

static void tf101_mpuirq_init(void)
{
	tegra_gpio_enable(TEGRA_GPIO_PZ4);
	gpio_request(TEGRA_GPIO_PZ4, MPU_GYRO_NAME);
	gpio_direction_input(TEGRA_GPIO_PZ4);

	tegra_gpio_enable(TEGRA_GPIO_PN4);
	gpio_request(TEGRA_GPIO_PN4, MPU_ACCEL_NAME);
	gpio_direction_input(TEGRA_GPIO_PN4);

	tegra_gpio_enable(TEGRA_GPIO_PN5);
	gpio_request(TEGRA_GPIO_PN5, MPU_COMPASS_NAME);
	gpio_direction_input(TEGRA_GPIO_PN5);
}
#endif

int __init tf101_sensors_init(void)
{
	void __iomem *misc_base = IO_ADDRESS(TEGRA_APB_MISC_BASE);

#ifdef CONFIG_MPU_SENSORS_MPU3050
	tf101_mpuirq_init();
#endif
	tf101_nct1008_init();

	clk_vi = clk_get_sys("tegra_camera", "vi");
	if (!clk_vi)
		pr_warn("Failed to get vi clock\n");

	clk_vi_sensor = clk_get_sys("tegra_camera", "vi_sensor");
	if (!clk_vi_sensor)
		pr_warn("Failed to get vi_sensor clock\n");

	clk_csi = clk_get_sys("tegra_camera", "csi");
	if (!clk_csi)
		pr_warn("Failed to get csi clock\n");

	clk_isp = clk_get_sys("tegra_camera", "isp");
	if (!clk_isp)
		pr_warn("Failed to get isp clock\n");

	clk_csus = clk_get_sys("tegra_camera", "csus");
	if (!clk_csus)
		pr_warn("Failed to get csus clock\n");

	clk_set_rate(clk_vi, 150000000);
	clk_set_rate(clk_vi_sensor, 150000000);
	clk_set_rate(clk_csus, 6000000);

	writel(readl(misc_base + 0x42c) | 0x1, misc_base + 0x42c);

	nvhost_device_register(&tegra_camera_device);
	platform_device_register(&soc_camera);

	i2c_register_board_info(2, tf101_i2c2_board_info,
		ARRAY_SIZE(tf101_i2c2_board_info));

	//i2c_register_board_info(3, tf101_i2c3_board_info,
	//	ARRAY_SIZE(tf101_i2c3_board_info));

	i2c_register_board_info(4, tf101_i2c4_board_info,
		ARRAY_SIZE(tf101_i2c4_board_info));

#ifdef CONFIG_MPU_SENSORS_MPU3050
	i2c_register_board_info(0, mpu3050_i2c0_boardinfo,
		ARRAY_SIZE(mpu3050_i2c0_boardinfo));
#endif

	return 0;
}
