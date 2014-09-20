/*
 * Copyright (C) 2012-2013 Freescale Semiconductor, Inc. All Rights Reserved.
 */

/*
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.

 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.

 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
 */

/*!
 * @file mxc_ldb.c
 *
 * @brief This file contains the LDB driver device interface and fops
 * functions.
 */
#include <linux/types.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/err.h>
#include <linux/clk.h>
#include <linux/console.h>
#include <linux/io.h>
#include <linux/ipu.h>
#include <linux/mxcfb.h>
#include <linux/regulator/consumer.h>
#include <linux/spinlock.h>
#include <linux/of_device.h>
#include <linux/mod_devicetable.h>
#include "mxc_dispdrv.h"

#define DISPDRV_LDB	"ldb"

#define LDB_BGREF_RMODE_MASK		0x00008000
#define LDB_BGREF_RMODE_INT		0x00008000
#define LDB_BGREF_RMODE_EXT		0x0

#define LDB_DI1_VS_POL_MASK		0x00000400
#define LDB_DI1_VS_POL_ACT_LOW		0x00000400
#define LDB_DI1_VS_POL_ACT_HIGH		0x0
#define LDB_DI0_VS_POL_MASK		0x00000200
#define LDB_DI0_VS_POL_ACT_LOW		0x00000200
#define LDB_DI0_VS_POL_ACT_HIGH		0x0

#define LDB_BIT_MAP_CH1_MASK		0x00000100
#define LDB_BIT_MAP_CH1_JEIDA		0x00000100
#define LDB_BIT_MAP_CH1_SPWG		0x0
#define LDB_BIT_MAP_CH0_MASK		0x00000040
#define LDB_BIT_MAP_CH0_JEIDA		0x00000040
#define LDB_BIT_MAP_CH0_SPWG		0x0

#define LDB_DATA_WIDTH_CH1_MASK		0x00000080
#define LDB_DATA_WIDTH_CH1_24		0x00000080
#define LDB_DATA_WIDTH_CH1_18		0x0
#define LDB_DATA_WIDTH_CH0_MASK		0x00000020
#define LDB_DATA_WIDTH_CH0_24		0x00000020
#define LDB_DATA_WIDTH_CH0_18		0x0

#define LDB_CH1_MODE_MASK		0x0000000C
#define LDB_CH1_MODE_EN_TO_DI1		0x0000000C
#define LDB_CH1_MODE_EN_TO_DI0		0x00000004
#define LDB_CH1_MODE_DISABLE		0x0
#define LDB_CH0_MODE_MASK		0x00000003
#define LDB_CH0_MODE_EN_TO_DI1		0x00000003
#define LDB_CH0_MODE_EN_TO_DI0		0x00000001
#define LDB_CH0_MODE_DISABLE		0x0

#define LDB_SPLIT_MODE_EN		0x00000010

enum {
	IMX6_LDB,
};

enum {
	LDB_IMX6 = 1,
};

struct fsl_mxc_ldb_platform_data {
	int devtype;
	u32 ext_ref;
#define LDB_SPL_DI0	1
#define LDB_SPL_DI1	2
#define LDB_DUL_DI0	3
#define LDB_DUL_DI1	4
#define LDB_SIN0	5
#define LDB_SIN1	6
#define LDB_SEP0	7
#define LDB_SEP1	8
	int mode;
	int ipu_id;
	int disp_id;

	/*only work for separate mode*/
	int sec_ipu_id;
	int sec_disp_id;
};

struct ldb_data {
	struct platform_device *pdev;
	struct mxc_dispdrv_handle *disp_ldb;
	uint32_t *reg;
	uint32_t *control_reg;
	uint32_t *gpr3_reg;
	uint32_t control_reg_data;
	struct regulator *lvds_bg_reg;
	int mode;
	bool inited;
	struct ldb_setting {
		struct clk *di_clk;
		struct clk *ldb_di_clk;
		struct clk *div_3_5_clk;
		struct clk *div_7_clk;
		struct clk *div_sel_clk;
		bool active;
		bool clk_en;
		int ipu;
		int di;
		uint32_t ch_mask;
		uint32_t ch_val;
	} setting[2];
	struct notifier_block nb;
};

static int g_ldb_mode;

static struct fb_videomode ldb_modedb[] = {
	{
	 "LDB-WXGA", 60, 1280, 800, 14065,
	 40, 40,
	 10, 3,
	 80, 10,
	 0,
	 FB_VMODE_NONINTERLACED,
	 FB_MODE_IS_DETAILED,},
	{
	 "LDB-XGA", 60, 1024, 768, 15385,
	 220, 40,
	 21, 7,
	 60, 10,
	 0,
	 FB_VMODE_NONINTERLACED,
	 FB_MODE_IS_DETAILED,},
	{
	 "LDB-1080P60", 60, 1920, 1080, 7692,
	 100, 40,
	 30, 3,
	 10, 2,
	 0,
	 FB_VMODE_NONINTERLACED,
	 FB_MODE_IS_DETAILED,},
};
static int ldb_modedb_sz = ARRAY_SIZE(ldb_modedb);

static inline int is_imx6_ldb(struct fsl_mxc_ldb_platform_data *plat_data)
{
	return (plat_data->devtype == LDB_IMX6);
}

static int bits_per_pixel(int pixel_fmt)
{
	switch (pixel_fmt) {
	case IPU_PIX_FMT_BGR24:
	case IPU_PIX_FMT_RGB24:
		return 24;
		break;
	case IPU_PIX_FMT_BGR666:
	case IPU_PIX_FMT_RGB666:
	case IPU_PIX_FMT_LVDS666:
		return 18;
		break;
	default:
		break;
	}
	return 0;
}

static int valid_mode(int pixel_fmt)
{
	return ((pixel_fmt == IPU_PIX_FMT_RGB24) ||
		(pixel_fmt == IPU_PIX_FMT_BGR24) ||
		(pixel_fmt == IPU_PIX_FMT_LVDS666) ||
		(pixel_fmt == IPU_PIX_FMT_RGB666) ||
		(pixel_fmt == IPU_PIX_FMT_BGR666));
}

static int parse_ldb_mode(char *mode)
{
	int ldb_mode;

	if (!strcmp(mode, "spl0"))
		ldb_mode = LDB_SPL_DI0;
	else if (!strcmp(mode, "spl1"))
		ldb_mode = LDB_SPL_DI1;
	else if (!strcmp(mode, "dul0"))
		ldb_mode = LDB_DUL_DI0;
	else if (!strcmp(mode, "dul1"))
		ldb_mode = LDB_DUL_DI1;
	else if (!strcmp(mode, "sin0"))
		ldb_mode = LDB_SIN0;
	else if (!strcmp(mode, "sin1"))
		ldb_mode = LDB_SIN1;
	else if (!strcmp(mode, "sep0"))
		ldb_mode = LDB_SEP0;
	else if (!strcmp(mode, "sep1"))
		ldb_mode = LDB_SEP1;
	else
		ldb_mode = -EINVAL;

	return ldb_mode;
}

#ifndef MODULE
/*
 *    "ldb=spl0/1"       --      split mode on DI0/1
 *    "ldb=dul0/1"       --      dual mode on DI0/1
 *    "ldb=sin0/1"       --      single mode on LVDS0/1
 *    "ldb=sep0/1" 	 --      separate mode begin from LVDS0/1
 *
 *    there are two LVDS channels(LVDS0 and LVDS1) which can transfer video
 *    datas, there two channels can be used as split/dual/single/separate mode.
 *
 *    split mode means display data from DI0 or DI1 will send to both channels
 *    LVDS0+LVDS1.
 *    dual mode means display data from DI0 or DI1 will be duplicated on LVDS0
 *    and LVDS1, it said, LVDS0 and LVDS1 has the same content.
 *    single mode means only work for DI0/DI1->LVDS0 or DI0/DI1->LVDS1.
 *    separate mode means you can make DI0/DI1->LVDS0 and DI0/DI1->LVDS1 work
 *    at the same time.
 */
static int __init ldb_setup(char *options)
{
	g_ldb_mode = parse_ldb_mode(options);
	return (g_ldb_mode < 0) ? 0 : 1;
}
__setup("ldb=", ldb_setup);
#endif

static int ldb_get_of_property(struct platform_device *pdev,
				struct fsl_mxc_ldb_platform_data *plat_data)
{
	struct device_node *np = pdev->dev.of_node;
	int err;
	u32 ipu_id, disp_id;
	u32 sec_ipu_id, sec_disp_id;
	char *mode;
	u32 ext_ref;

	err = of_property_read_string(np, "mode", (const char **)&mode);
	if (err) {
		dev_dbg(&pdev->dev, "get of property mode fail\n");
		return err;
	}
	err = of_property_read_u32(np, "ext_ref", &ext_ref);
	if (err) {
		dev_dbg(&pdev->dev, "get of property ext_ref fail\n");
		return err;
	}
	err = of_property_read_u32(np, "ipu_id", &ipu_id);
	if (err) {
		dev_dbg(&pdev->dev, "get of property ipu_id fail\n");
		return err;
	}
	err = of_property_read_u32(np, "disp_id", &disp_id);
	if (err) {
		dev_dbg(&pdev->dev, "get of property disp_id fail\n");
		return err;
	}
	err = of_property_read_u32(np, "sec_ipu_id", &sec_ipu_id);
	if (err) {
		dev_dbg(&pdev->dev, "get of property sec_ipu_id fail\n");
		return err;
	}
	err = of_property_read_u32(np, "sec_disp_id", &sec_disp_id);
	if (err) {
		dev_dbg(&pdev->dev, "get of property sec_disp_id fail\n");
		return err;
	}

	plat_data->mode = parse_ldb_mode(mode);
	plat_data->ext_ref = ext_ref;
	plat_data->ipu_id = ipu_id;
	plat_data->disp_id = disp_id;
	plat_data->sec_ipu_id = sec_ipu_id;
	plat_data->sec_disp_id = sec_disp_id;

	return err;
}

static int find_ldb_setting(struct ldb_data *ldb, struct fb_info *fbi)
{
	char *id_di[] = {
		 "DISP3 BG",
		 "DISP3 BG - DI1",
		};
	char id[16];
	int i;

	for (i = 0; i < 2; i++) {
		if (ldb->setting[i].active) {
			memset(id, 0, 16);
			memcpy(id, id_di[ldb->setting[i].di],
				strlen(id_di[ldb->setting[i].di]));
			id[4] += ldb->setting[i].ipu;
			if (!strcmp(id, fbi->fix.id))
				return i;
		}
	}
	return -EINVAL;
}

static int ldb_disp_setup(struct mxc_dispdrv_handle *disp, struct fb_info *fbi)
{
	uint32_t reg, val;
	uint32_t pixel_clk, rounded_pixel_clk;
	struct clk *ldb_clk_parent;
	struct ldb_data *ldb = mxc_dispdrv_getdata(disp);
	int setting_idx, di;
	int ret;

	setting_idx = find_ldb_setting(ldb, fbi);
	if (setting_idx < 0)
		return setting_idx;

	di = ldb->setting[setting_idx].di;

	/* restore channel mode setting */
	val = readl(ldb->control_reg);
	val |= ldb->setting[setting_idx].ch_val;
	writel(val, ldb->control_reg);
	dev_dbg(&ldb->pdev->dev, "LDB setup, control reg:0x%x\n",
			readl(ldb->control_reg));

	/* vsync setup */
	reg = readl(ldb->control_reg);
	if (fbi->var.sync & FB_SYNC_VERT_HIGH_ACT) {
		if (di == 0)
			reg = (reg & ~LDB_DI0_VS_POL_MASK)
				| LDB_DI0_VS_POL_ACT_HIGH;
		else
			reg = (reg & ~LDB_DI1_VS_POL_MASK)
				| LDB_DI1_VS_POL_ACT_HIGH;
	} else {
		if (di == 0)
			reg = (reg & ~LDB_DI0_VS_POL_MASK)
				| LDB_DI0_VS_POL_ACT_LOW;
		else
			reg = (reg & ~LDB_DI1_VS_POL_MASK)
				| LDB_DI1_VS_POL_ACT_LOW;
	}
	writel(reg, ldb->control_reg);

	/* clk setup */
	if (ldb->setting[setting_idx].clk_en)
		 clk_disable_unprepare(ldb->setting[setting_idx].ldb_di_clk);
	pixel_clk = (PICOS2KHZ(fbi->var.pixclock)) * 1000UL;
	ldb_clk_parent = clk_get_parent(ldb->setting[setting_idx].ldb_di_clk);
	if (IS_ERR(ldb_clk_parent)) {
		dev_err(&ldb->pdev->dev, "get ldb di parent clk fail\n");
		return PTR_ERR(ldb_clk_parent);
	}
	if ((ldb->mode == LDB_SPL_DI0) || (ldb->mode == LDB_SPL_DI1))
		ret = clk_set_rate(ldb_clk_parent, pixel_clk * 7 / 2);
	else
		ret = clk_set_rate(ldb_clk_parent, pixel_clk * 7);
	if (ret < 0) {
		dev_err(&ldb->pdev->dev, "set ldb parent clk fail:%d\n", ret);
		return ret;
	}
	rounded_pixel_clk = clk_round_rate(ldb->setting[setting_idx].ldb_di_clk,
						pixel_clk);
	dev_dbg(&ldb->pdev->dev, "pixel_clk:%d, rounded_pixel_clk:%d\n",
			pixel_clk, rounded_pixel_clk);
	ret = clk_set_rate(ldb->setting[setting_idx].ldb_di_clk,
				rounded_pixel_clk);
	if (ret < 0) {
		dev_err(&ldb->pdev->dev, "set ldb di clk fail:%d\n", ret);
		return ret;
	}
	ret = clk_prepare_enable(ldb->setting[setting_idx].ldb_di_clk);
	if (ret < 0) {
		dev_err(&ldb->pdev->dev, "enable ldb di clk fail:%d\n", ret);
		return ret;
	}

	if (!ldb->setting[setting_idx].clk_en)
		ldb->setting[setting_idx].clk_en = true;

	return 0;
}

int ldb_fb_event(struct notifier_block *nb, unsigned long val, void *v)
{
	struct ldb_data *ldb = container_of(nb, struct ldb_data, nb);
	struct fb_event *event = v;
	struct fb_info *fbi = event->info;
	int index;
	uint32_t data;

	index = find_ldb_setting(ldb, fbi);
	if (index < 0)
		return 0;

	fbi->mode = (struct fb_videomode *)fb_match_mode(&fbi->var,
			&fbi->modelist);

	if (!fbi->mode) {
		dev_warn(&ldb->pdev->dev,
				"LDB: can not find mode for xres=%d, yres=%d\n",
				fbi->var.xres, fbi->var.yres);
		if (ldb->setting[index].clk_en) {
			clk_disable(ldb->setting[index].ldb_di_clk);
			ldb->setting[index].clk_en = false;
			data = readl(ldb->control_reg);
			data &= ~ldb->setting[index].ch_mask;
			writel(data, ldb->control_reg);
		}
		return 0;
	}

	switch (val) {
	case FB_EVENT_BLANK:
	{
		if (*((int *)event->data) == FB_BLANK_UNBLANK) {
			if (!ldb->setting[index].clk_en) {
				clk_enable(ldb->setting[index].ldb_di_clk);
				ldb->setting[index].clk_en = true;
			}
		} else {
			if (ldb->setting[index].clk_en) {
				clk_disable(ldb->setting[index].ldb_di_clk);
				ldb->setting[index].clk_en = false;
				data = readl(ldb->control_reg);
				data &= ~ldb->setting[index].ch_mask;
				writel(data, ldb->control_reg);
				dev_dbg(&ldb->pdev->dev,
					"LDB blank, control reg:0x%x\n",
						readl(ldb->control_reg));
			}
		}
		break;
	}
	case FB_EVENT_SUSPEND:
		if (ldb->setting[index].clk_en) {
			clk_disable(ldb->setting[index].ldb_di_clk);
			ldb->setting[index].clk_en = false;
		}
		break;
	default:
		break;
	}
	return 0;
}

#define LVDS_MUX_CTL_WIDTH	2
#define LVDS_MUX_CTL_MASK	3
#define LVDS0_MUX_CTL_OFFS	6
#define LVDS1_MUX_CTL_OFFS	8
#define LVDS0_MUX_CTL_MASK	(LVDS_MUX_CTL_MASK << 6)
#define LVDS1_MUX_CTL_MASK	(LVDS_MUX_CTL_MASK << 8)
#define ROUTE_IPU_DI(ipu, di)	(((ipu << 1) | di) & LVDS_MUX_CTL_MASK)
static int ldb_ipu_ldb_route(int ipu, int di, struct ldb_data *ldb)
{
	uint32_t reg;
	int channel;
	int shift;
	int mode = ldb->mode;

	reg = readl(ldb->gpr3_reg);
	if (mode < LDB_SIN0) {
		reg &= ~(LVDS0_MUX_CTL_MASK | LVDS1_MUX_CTL_MASK);
		reg |= (ROUTE_IPU_DI(ipu, di) << LVDS0_MUX_CTL_OFFS) |
			(ROUTE_IPU_DI(ipu, di) << LVDS1_MUX_CTL_OFFS);
		dev_dbg(&ldb->pdev->dev,
			"Dual/Split mode both channels route to IPU%d-DI%d\n",
			ipu, di);
	} else if ((mode == LDB_SIN0) || (mode == LDB_SIN1)) {
		reg &= ~(LVDS0_MUX_CTL_MASK | LVDS1_MUX_CTL_MASK);
		channel = mode - LDB_SIN0;
		shift = LVDS0_MUX_CTL_OFFS + channel * LVDS_MUX_CTL_WIDTH;
		reg |= ROUTE_IPU_DI(ipu, di) << shift;
		dev_dbg(&ldb->pdev->dev,
			"Single mode channel %d route to IPU%d-DI%d\n",
				channel, ipu, di);
	} else {
		static bool first = true;

		if (first) {
			if (mode == LDB_SEP0) {
				reg &= ~LVDS0_MUX_CTL_MASK;
				channel = 0;
			} else {
				reg &= ~LVDS1_MUX_CTL_MASK;
				channel = 1;
			}
			first = false;
		} else {
			if (mode == LDB_SEP0) {
				reg &= ~LVDS1_MUX_CTL_MASK;
				channel = 1;
			} else {
				reg &= ~LVDS0_MUX_CTL_MASK;
				channel = 0;
			}
		}

		shift = LVDS0_MUX_CTL_OFFS + channel * LVDS_MUX_CTL_WIDTH;
		reg |= ROUTE_IPU_DI(ipu, di) << shift;

		dev_dbg(&ldb->pdev->dev,
			"Separate mode channel %d route to IPU%d-DI%d\n",
			channel, ipu, di);
	}
	writel(reg, ldb->gpr3_reg);

	return 0;
}

static int ldb_disp_init(struct mxc_dispdrv_handle *disp,
	struct mxc_dispdrv_setting *setting)
{
	int ret = 0, i, lvds_channel = 0;
	struct ldb_data *ldb = mxc_dispdrv_getdata(disp);
	struct fsl_mxc_ldb_platform_data *plat_data = ldb->pdev->dev.platform_data;
	struct resource *res;
	uint32_t reg, setting_idx;
	uint32_t ch_mask = 0, ch_val = 0;
	uint32_t ipu_id, disp_id;
	char di_clk[] = "ipu1_di0_sel";
	char ldb_clk[] = "ldb_di0";
	char div_3_5_clk[] = "di0_div_3_5";
	char div_7_clk[] = "di0_div_7";
	char div_sel_clk[] = "di0_div_sel";

	/* if input format not valid, make RGB666 as default*/
	if (!valid_mode(setting->if_fmt)) {
		dev_warn(&ldb->pdev->dev, "Input pixel format not valid"
					" use default RGB666\n");
		setting->if_fmt = IPU_PIX_FMT_RGB666;
	}

	if (!ldb->inited) {
		setting_idx = 0;
		res = platform_get_resource(ldb->pdev, IORESOURCE_MEM, 0);
		if (!res) {
			dev_err(&ldb->pdev->dev, "get iomem fail.\n");
			return -ENOMEM;
		}

		ldb->reg = devm_ioremap(&ldb->pdev->dev, res->start,
					resource_size(res));
		ldb->control_reg = ldb->reg + 2;
		ldb->gpr3_reg = ldb->reg + 3;

		/* ipu selected by platform data setting */
		setting->dev_id = plat_data->ipu_id;

		reg = readl(ldb->control_reg);

		/* refrence resistor select */
		reg &= ~LDB_BGREF_RMODE_MASK;
		if (plat_data->ext_ref)
			reg |= LDB_BGREF_RMODE_EXT;
		else
			reg |= LDB_BGREF_RMODE_INT;

		/* TODO: now only use SPWG data mapping for both channel */
		reg &= ~(LDB_BIT_MAP_CH0_MASK | LDB_BIT_MAP_CH1_MASK);
		reg |= LDB_BIT_MAP_CH0_SPWG | LDB_BIT_MAP_CH1_SPWG;

		/* channel mode setting */
		reg &= ~(LDB_CH0_MODE_MASK | LDB_CH1_MODE_MASK);
		reg &= ~(LDB_DATA_WIDTH_CH0_MASK | LDB_DATA_WIDTH_CH1_MASK);

		if (bits_per_pixel(setting->if_fmt) == 24)
			reg |= LDB_DATA_WIDTH_CH0_24 | LDB_DATA_WIDTH_CH1_24;
		else
			reg |= LDB_DATA_WIDTH_CH0_18 | LDB_DATA_WIDTH_CH1_18;

		if (g_ldb_mode >= LDB_SPL_DI0)
			ldb->mode = g_ldb_mode;
		else
			ldb->mode = plat_data->mode;

		if ((ldb->mode == LDB_SIN0) || (ldb->mode == LDB_SIN1)) {
			ret = ldb->mode - LDB_SIN0;
			if (plat_data->disp_id != ret) {
				dev_warn(&ldb->pdev->dev,
					"change IPU DI%d to IPU DI%d for LDB "
					"channel%d.\n",
					plat_data->disp_id, ret, ret);
				plat_data->disp_id = ret;
			}
		} else if (((ldb->mode == LDB_SEP0) || (ldb->mode == LDB_SEP1))
				&& is_imx6_ldb(plat_data)) {
			if (plat_data->disp_id == plat_data->sec_disp_id) {
				dev_err(&ldb->pdev->dev,
					"For LVDS separate mode,"
					"two DIs should be different!\n");
				return -EINVAL;
			}

			if (((!plat_data->disp_id) && (ldb->mode == LDB_SEP1))
				|| ((plat_data->disp_id) &&
					(ldb->mode == LDB_SEP0))) {
				dev_dbg(&ldb->pdev->dev,
					"LVDS separate mode:"
					"swap DI configuration!\n");
				ipu_id = plat_data->ipu_id;
				disp_id = plat_data->disp_id;
				plat_data->ipu_id = plat_data->sec_ipu_id;
				plat_data->disp_id = plat_data->sec_disp_id;
				plat_data->sec_ipu_id = ipu_id;
				plat_data->sec_disp_id = disp_id;
			}
		}

		if (ldb->mode == LDB_SPL_DI0) {
			reg |= LDB_SPLIT_MODE_EN | LDB_CH0_MODE_EN_TO_DI0
				| LDB_CH1_MODE_EN_TO_DI0;
			setting->disp_id = 0;
		} else if (ldb->mode == LDB_SPL_DI1) {
			reg |= LDB_SPLIT_MODE_EN | LDB_CH0_MODE_EN_TO_DI1
				| LDB_CH1_MODE_EN_TO_DI1;
			setting->disp_id = 1;
		} else if (ldb->mode == LDB_DUL_DI0) {
			reg &= ~LDB_SPLIT_MODE_EN;
			reg |= LDB_CH0_MODE_EN_TO_DI0 | LDB_CH1_MODE_EN_TO_DI0;
			setting->disp_id = 0;
		} else if (ldb->mode == LDB_DUL_DI1) {
			reg &= ~LDB_SPLIT_MODE_EN;
			reg |= LDB_CH0_MODE_EN_TO_DI1 | LDB_CH1_MODE_EN_TO_DI1;
			setting->disp_id = 1;
		} else if (ldb->mode == LDB_SIN0) {
			reg &= ~LDB_SPLIT_MODE_EN;
			setting->disp_id = plat_data->disp_id;
			if (setting->disp_id == 0)
				reg |= LDB_CH0_MODE_EN_TO_DI0;
			else
				reg |= LDB_CH0_MODE_EN_TO_DI1;
			ch_mask = LDB_CH0_MODE_MASK;
			ch_val = reg & LDB_CH0_MODE_MASK;
		} else if (ldb->mode == LDB_SIN1) {
			reg &= ~LDB_SPLIT_MODE_EN;
			setting->disp_id = plat_data->disp_id;
			if (setting->disp_id == 0)
				reg |= LDB_CH1_MODE_EN_TO_DI0;
			else
				reg |= LDB_CH1_MODE_EN_TO_DI1;
			ch_mask = LDB_CH1_MODE_MASK;
			ch_val = reg & LDB_CH1_MODE_MASK;
		} else { /* separate mode*/
			setting->disp_id = plat_data->disp_id;

			/* first output is LVDS0 or LVDS1 */
			if (ldb->mode == LDB_SEP0)
				lvds_channel = 0;
			else
				lvds_channel = 1;

			reg &= ~LDB_SPLIT_MODE_EN;

			if ((lvds_channel == 0) && (setting->disp_id == 0))
				reg |= LDB_CH0_MODE_EN_TO_DI0;
			else if ((lvds_channel == 0) && (setting->disp_id == 1))
				reg |= LDB_CH0_MODE_EN_TO_DI1;
			else if ((lvds_channel == 1) && (setting->disp_id == 0))
				reg |= LDB_CH1_MODE_EN_TO_DI0;
			else
				reg |= LDB_CH1_MODE_EN_TO_DI1;
			ch_mask = lvds_channel ? LDB_CH1_MODE_MASK :
					LDB_CH0_MODE_MASK;
			ch_val = reg & ch_mask;

			if (bits_per_pixel(setting->if_fmt) == 24) {
				if (lvds_channel == 0)
					reg &= ~LDB_DATA_WIDTH_CH1_24;
				else
					reg &= ~LDB_DATA_WIDTH_CH0_24;
			} else {
				if (lvds_channel == 0)
					reg &= ~LDB_DATA_WIDTH_CH1_18;
				else
					reg &= ~LDB_DATA_WIDTH_CH0_18;
			}
		}

		writel(reg, ldb->control_reg);
		if (ldb->mode <  LDB_SIN0) {
			ch_mask = LDB_CH0_MODE_MASK | LDB_CH1_MODE_MASK;
			ch_val = reg & (LDB_CH0_MODE_MASK | LDB_CH1_MODE_MASK);
		}
	} else { /* second time for separate mode */
		if ((ldb->mode == LDB_SPL_DI0) ||
			(ldb->mode == LDB_SPL_DI1) ||
			(ldb->mode == LDB_DUL_DI0) ||
			(ldb->mode == LDB_DUL_DI1) ||
			(ldb->mode == LDB_SIN0) ||
			(ldb->mode == LDB_SIN1)) {
			dev_err(&ldb->pdev->dev, "for second ldb disp"
					"ldb mode should in separate mode\n");
			return -EINVAL;
		}

		setting_idx = 1;
		if (is_imx6_ldb(plat_data)) {
			setting->dev_id = plat_data->sec_ipu_id;
			setting->disp_id = plat_data->sec_disp_id;
		} else {
			setting->dev_id = plat_data->ipu_id;
			setting->disp_id = !plat_data->disp_id;
		}
		if (setting->disp_id == ldb->setting[0].di) {
			dev_err(&ldb->pdev->dev, "Err: for second ldb disp in"
				"separate mode, DI should be different!\n");
			return -EINVAL;
		}

		/* second output is LVDS0 or LVDS1 */
		if (ldb->mode == LDB_SEP0)
			lvds_channel = 1;
		else
			lvds_channel = 0;

		reg = readl(ldb->control_reg);
		if ((lvds_channel == 0) && (setting->disp_id == 0))
			reg |= LDB_CH0_MODE_EN_TO_DI0;
		else if ((lvds_channel == 0) && (setting->disp_id == 1))
			reg |= LDB_CH0_MODE_EN_TO_DI1;
		else if ((lvds_channel == 1) && (setting->disp_id == 0))
			reg |= LDB_CH1_MODE_EN_TO_DI0;
		else
			reg |= LDB_CH1_MODE_EN_TO_DI1;
		ch_mask = lvds_channel ?  LDB_CH1_MODE_MASK :
				LDB_CH0_MODE_MASK;
		ch_val = reg & ch_mask;

		if (bits_per_pixel(setting->if_fmt) == 24) {
			if (lvds_channel == 0)
				reg |= LDB_DATA_WIDTH_CH0_24;
			else
				reg |= LDB_DATA_WIDTH_CH1_24;
		} else {
			if (lvds_channel == 0)
				reg |= LDB_DATA_WIDTH_CH0_18;
			else
				reg |= LDB_DATA_WIDTH_CH1_18;
		}
		writel(reg, ldb->control_reg);
	}

	/* get clocks */
	if (is_imx6_ldb(plat_data) &&
		((ldb->mode == LDB_SEP0) || (ldb->mode == LDB_SEP1))) {
		ldb_clk[6] += lvds_channel;
		div_3_5_clk[2] += lvds_channel;
		div_7_clk[2] += lvds_channel;
		div_sel_clk[2] += lvds_channel;
	} else {
		ldb_clk[6] += setting->disp_id;
		div_3_5_clk[2] += setting->disp_id;
		div_7_clk[2] += setting->disp_id;
		div_sel_clk[2] += setting->disp_id;
	}
	ldb->setting[setting_idx].ldb_di_clk = clk_get(&ldb->pdev->dev,
							ldb_clk);
	if (IS_ERR(ldb->setting[setting_idx].ldb_di_clk)) {
		dev_err(&ldb->pdev->dev, "get ldb clk failed\n");
		return PTR_ERR(ldb->setting[setting_idx].ldb_di_clk);
	}

	ldb->setting[setting_idx].div_3_5_clk = clk_get(&ldb->pdev->dev,
							div_3_5_clk);
	if (IS_ERR(ldb->setting[setting_idx].div_3_5_clk)) {
		dev_err(&ldb->pdev->dev, "get div 3.5 clk failed\n");
		return PTR_ERR(ldb->setting[setting_idx].div_3_5_clk);
	}
	ldb->setting[setting_idx].div_7_clk = clk_get(&ldb->pdev->dev,
							div_7_clk);
	if (IS_ERR(ldb->setting[setting_idx].div_7_clk)) {
		dev_err(&ldb->pdev->dev, "get div 7 clk failed\n");
		return PTR_ERR(ldb->setting[setting_idx].div_7_clk);
	}

	ldb->setting[setting_idx].div_sel_clk = clk_get(&ldb->pdev->dev,
							div_sel_clk);
	if (IS_ERR(ldb->setting[setting_idx].div_sel_clk)) {
		dev_err(&ldb->pdev->dev, "get div sel clk failed\n");
		return PTR_ERR(ldb->setting[setting_idx].div_sel_clk);
	}

	di_clk[3] += setting->dev_id;
	di_clk[7] += setting->disp_id;
	ldb->setting[setting_idx].di_clk = clk_get(&ldb->pdev->dev,
							di_clk);
	if (IS_ERR(ldb->setting[setting_idx].di_clk)) {
		dev_err(&ldb->pdev->dev, "get di clk failed\n");
		return PTR_ERR(ldb->setting[setting_idx].di_clk);
	}

	ldb->setting[setting_idx].ch_mask = ch_mask;
	ldb->setting[setting_idx].ch_val = ch_val;

	if (is_imx6_ldb(plat_data))
		ldb_ipu_ldb_route(setting->dev_id, setting->disp_id, ldb);

	/* must use spec video mode defined by driver */
	ret = fb_find_mode(&setting->fbi->var, setting->fbi, setting->dft_mode_str,
				ldb_modedb, ldb_modedb_sz, NULL, setting->default_bpp);
	if (ret != 1)
		fb_videomode_to_var(&setting->fbi->var, &ldb_modedb[0]);

	INIT_LIST_HEAD(&setting->fbi->modelist);
	for (i = 0; i < ldb_modedb_sz; i++) {
		struct fb_videomode m;
		fb_var_to_videomode(&m, &setting->fbi->var);
		if (fb_mode_is_equal(&m, &ldb_modedb[i])) {
			fb_add_videomode(&ldb_modedb[i],
					&setting->fbi->modelist);
			break;
		}
	}

	ldb->setting[setting_idx].ipu = setting->dev_id;
	ldb->setting[setting_idx].di = setting->disp_id;

	return ret;
}

static int ldb_post_disp_init(struct mxc_dispdrv_handle *disp,
				int ipu_id, int disp_id)
{
	struct ldb_data *ldb = mxc_dispdrv_getdata(disp);
	int setting_idx = ldb->inited ? 1 : 0;
	int ret = 0;

	if (!ldb->inited) {
		ldb->nb.notifier_call = ldb_fb_event;
		fb_register_client(&ldb->nb);
	}

	ret = clk_set_parent(ldb->setting[setting_idx].di_clk,
			ldb->setting[setting_idx].ldb_di_clk);
	if (ret) {
		dev_err(&ldb->pdev->dev, "fail to set ldb_di clk as"
			"the parent of ipu_di clk\n");
		return ret;
	}

	if ((ldb->mode == LDB_SPL_DI0) || (ldb->mode == LDB_SPL_DI1)) {
		ret = clk_set_parent(ldb->setting[setting_idx].div_sel_clk,
				ldb->setting[setting_idx].div_3_5_clk);
		if (ret) {
			dev_err(&ldb->pdev->dev, "fail to set div 3.5 clk as"
				"the parent of div sel clk\n");
			return ret;
		}
	} else {
		ret = clk_set_parent(ldb->setting[setting_idx].div_sel_clk,
				ldb->setting[setting_idx].div_7_clk);
		if (ret) {
			dev_err(&ldb->pdev->dev, "fail to set div 7 clk as"
				"the parent of div sel clk\n");
			return ret;
		}
	}

	/* save active ldb setting for fb notifier */
	ldb->setting[setting_idx].active = true;

	ldb->inited = true;
	return ret;
}

static void ldb_disp_deinit(struct mxc_dispdrv_handle *disp)
{
	struct ldb_data *ldb = mxc_dispdrv_getdata(disp);
	int i;

	writel(0, ldb->control_reg);

	for (i = 0; i < 2; i++) {
		clk_disable(ldb->setting[i].ldb_di_clk);
		clk_put(ldb->setting[i].ldb_di_clk);
		clk_put(ldb->setting[i].div_3_5_clk);
		clk_put(ldb->setting[i].div_7_clk);
		clk_put(ldb->setting[i].div_sel_clk);
	}

	fb_unregister_client(&ldb->nb);
}

static struct mxc_dispdrv_driver ldb_drv = {
	.name 	= DISPDRV_LDB,
	.init 	= ldb_disp_init,
	.post_init = ldb_post_disp_init,
	.deinit	= ldb_disp_deinit,
	.setup = ldb_disp_setup,
};

static int ldb_suspend(struct platform_device *pdev, pm_message_t state)
{
	struct ldb_data *ldb = dev_get_drvdata(&pdev->dev);
	uint32_t	data;

	if (!ldb->inited)
		return 0;
	data = readl(ldb->control_reg);
	ldb->control_reg_data = data;
	data &= ~(LDB_CH0_MODE_MASK | LDB_CH1_MODE_MASK);
	writel(data, ldb->control_reg);

	return 0;
}

static int ldb_resume(struct platform_device *pdev)
{
	struct ldb_data *ldb = dev_get_drvdata(&pdev->dev);

	if (!ldb->inited)
		return 0;
	writel(ldb->control_reg_data, ldb->control_reg);

	return 0;
}

static struct platform_device_id imx_ldb_devtype[] = {
	{
		.name = "ldb-imx6",
		.driver_data = LDB_IMX6,
	}, {
		/* sentinel */
	}
};

static const struct of_device_id imx_ldb_dt_ids[] = {
	{ .compatible = "fsl,imx6q-ldb", .data = &imx_ldb_devtype[IMX6_LDB],},
	{ /* sentinel */ }
};

/*!
 * This function is called by the driver framework to initialize the LDB
 * device.
 *
 * @param	dev	The device structure for the LDB passed in by the
 *			driver framework.
 *
 * @return      Returns 0 on success or negative error code on error
 */
static int ldb_probe(struct platform_device *pdev)
{
	int ret = 0;
	struct ldb_data *ldb;
	struct fsl_mxc_ldb_platform_data *plat_data;
	const struct of_device_id *of_id =
			of_match_device(imx_ldb_dt_ids, &pdev->dev);

	dev_dbg(&pdev->dev, "%s enter\n", __func__);
	ldb = devm_kzalloc(&pdev->dev, sizeof(struct ldb_data), GFP_KERNEL);
	if (!ldb)
		return -ENOMEM;

	plat_data = devm_kzalloc(&pdev->dev,
				sizeof(struct fsl_mxc_ldb_platform_data),
				GFP_KERNEL);
	if (!plat_data)
		return -ENOMEM;
	pdev->dev.platform_data = plat_data;
	if (of_id)
		pdev->id_entry = of_id->data;
	plat_data->devtype = pdev->id_entry->driver_data;

	ret = ldb_get_of_property(pdev, plat_data);
	if (ret < 0) {
		dev_err(&pdev->dev, "get ldb of property fail\n");
		return ret;
	}

	ldb->pdev = pdev;
	ldb->disp_ldb = mxc_dispdrv_register(&ldb_drv);
	mxc_dispdrv_setdata(ldb->disp_ldb, ldb);

	dev_set_drvdata(&pdev->dev, ldb);

	dev_dbg(&pdev->dev, "%s exit\n", __func__);
	return ret;
}

static int ldb_remove(struct platform_device *pdev)
{
	struct ldb_data *ldb = dev_get_drvdata(&pdev->dev);

	if (!ldb->inited)
		return 0;
	mxc_dispdrv_puthandle(ldb->disp_ldb);
	mxc_dispdrv_unregister(ldb->disp_ldb);
	return 0;
}

static struct platform_driver mxcldb_driver = {
	.driver = {
		.name = "mxc_ldb",
		.of_match_table	= imx_ldb_dt_ids,
	},
	.probe = ldb_probe,
	.remove = ldb_remove,
	.suspend = ldb_suspend,
	.resume = ldb_resume,
};

static int __init ldb_init(void)
{
	return platform_driver_register(&mxcldb_driver);
}

static void __exit ldb_uninit(void)
{
	platform_driver_unregister(&mxcldb_driver);
}

module_init(ldb_init);
module_exit(ldb_uninit);

MODULE_AUTHOR("Freescale Semiconductor, Inc.");
MODULE_DESCRIPTION("MXC LDB driver");
MODULE_LICENSE("GPL");
