/*
 * adv7610.c -- MXC video capture driver for i.MX boards with
 *               adv7610 HDMI receiver
 *
 * Copyright (C) 2013 3DR Corporation
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307 USA
 *
 */

//#define DEBUG
#define pr_fmt(fmt) KBUILD_MODNAME ": " fmt

#include <linux/init.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/fsl_devices.h>
#include <linux/mfd/core.h>
#include <linux/mfd/syscon.h>
#include <linux/mfd/syscon/imx6q-iomuxc-gpr.h>
#include <linux/module.h>
#include <linux/of_device.h>
#include <linux/platform_device.h>
#include <linux/regmap.h>
#include <linux/types.h>
#include <linux/videodev2.h>
#include <linux/workqueue.h>
#include <linux/i2c.h>
#include <linux/of_gpio.h>
#include <linux/time.h>

#include <media/v4l2-chip-ident.h>
#include "v4l2-int-device.h"

#include "mxc_v4l2_capture.h"

#define USE_16BIT
//#define SUPPORT_1080P

/*!
 * Maintains the information on the current state of the sensor.
 */
static int rst_gpio;

struct sensor {
	struct sensor_data sen;
} adv7610_data;

/* video resolution data */
typedef struct {
	int width;
	int height;
	int fps;
	bool interlaced;
} adv7610_vidout_fmt_t;

#define ADV7610_CHIP_ID_HIGH_BYTE	0xEA
#define ADV7610_CHIP_ID_LOW_BYTE	0xEB
#define MAX_LOCK_TRIES 1000

#define CEC_MAP_ADDR 		0x80
#define INFOFRAME_MAP_ADDR 	0x7C
#define KSV_MAP_ADDR 		0x64
#define EDID_MAP_ADDR 		0x6C
#define HDMI_MAP_ADDR 		0x68
#define CP_MAP_ADDR 		0x44
#define DPLL_MAP_ADDR 		0x4C

struct adv7610_i2c_map {
	struct	i2c_client *io;
	struct	i2c_client *cec;
	struct	i2c_client *infoframe;
	struct	i2c_client *dpll;
	struct	i2c_client *ksv;
	struct	i2c_client *edid;
	struct	i2c_client *hdmi;
	struct  i2c_client *cp;
};

static struct adv7610_i2c_map adv7610_i2c_clients = {NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL};

wait_queue_head_t i2c_wait;

static int adv7610_write(struct i2c_client *client, u8 reg, u8 val);
static inline int adv7610_read(struct i2c_client *client, u8 reg);

/***********************************************************************
 * mxc_v4l2_capture interface.
 ***********************************************************************/
//Get the video output format from the ADV7610
int adv7610_get_vidout_fmt(adv7610_vidout_fmt_t *fmt)
{
	int ch1Fcl, count;

	//Determine if there is an HDMI input, if so read the current format
	if(adv7610_read(adv7610_i2c_clients.io, 0x6F) & 0x01)
	{
		pr_debug("ADV7610 HDMI cable detected\n");

		//Pull data from the i2c here to determine the format.
		//Read the line width
		count = 0;
		while(!(adv7610_read(adv7610_i2c_clients.io, 0x6A) & 0x01)){
			if(++count > MAX_LOCK_TRIES)
			{
				pr_debug("ADV7610 DE regen filter no lock\n");
				goto default_exit;
			}
		}  //wait for the DE Regen filter to lock
		
		count = 0;
		while(!(adv7610_read(adv7610_i2c_clients.io, 0x6A) & 0x02)){
			if(++count > MAX_LOCK_TRIES)	
			{
				pr_debug("ADV7610 vert filter no lock\n");
				goto default_exit; 
			}
		}  //wait for the vert filter to lock
	
		//msleep(300);	//wait 300ms for some reason before the values are good...
		fmt->width = (adv7610_read(adv7610_i2c_clients.hdmi, 0x07) & 0x1F) << 8;
		fmt->width |= adv7610_read(adv7610_i2c_clients.hdmi, 0x08);

		//read the field height
		fmt->height = (adv7610_read(adv7610_i2c_clients.hdmi, 0x09) & 0x1F) << 8;
		fmt->height |= adv7610_read(adv7610_i2c_clients.hdmi, 0x0A);

		//Determine if its interlaced
		fmt->interlaced = (bool)(adv7610_read(adv7610_i2c_clients.hdmi, 0x0B) & 0x20);

		pr_debug("ADV7610 read %ix%i%s\n",fmt->width,fmt->height,(fmt->interlaced?"i":"p"));

		//Fps comes from the CH1_FCL
		count = 0;
		while(!(adv7610_read(adv7610_i2c_clients.cp, 0xB1) & 0x80)){
			if(++count > MAX_LOCK_TRIES)
			{
				pr_debug("ADV7610 CH1_STDI_DVALID not high\n");
				goto default_exit; 
			}

		}  //wait for the CH1_STDI_DVALID to be high

		ch1Fcl = (adv7610_read(adv7610_i2c_clients.cp, 0xB8) & 0x1F) << 8;
		ch1Fcl |= adv7610_read(adv7610_i2c_clients.cp, 0xB9); 
		fmt->fps = ((111861  + (ch1Fcl - 1))/ ch1Fcl);	//A little integer division rounding
	}
	else
	{
		pr_debug("ADV7610 no HDMI cable detected\n");
default_exit:
		pr_debug("ADV7610 returning default resolution\n");
		//Use default values
		fmt->width = 640;
		fmt->height = 480;
		fmt->interlaced = false;
		fmt->fps = 60;
	}
	

	pr_debug("ADV7610 returned %ix%i%s@%i\n",
		fmt->width, fmt->height,
		(fmt->interlaced?"i":"p"),
		fmt->fps);

	return 0;
}

static void adv7610_hard_reset(void)
{
	pr_debug("ADV7610 hard reset\n");

        /* camera reset */
        gpio_set_value(rst_gpio, 1);

        gpio_set_value(rst_gpio, 0);
        msleep(1);

        gpio_set_value(rst_gpio, 1);
        msleep(5);
}


/***********************************************************************
 * IOCTL Functions from v4l2_int_ioctl_desc.
 ***********************************************************************/

/*!
 * ioctl_g_ifparm - V4L2 sensor interface handler for vidioc_int_g_ifparm_num
 * s: pointer to standard V4L2 device structure
 * p: pointer to standard V4L2 vidioc_int_g_ifparm_num ioctl structure
 *
 * Gets slave interface parameters.
 * Calculates the required xclk value to support the requested
 * clock parameters in p.  This value is returned in the p
 * parameter.
 *
 * vidioc_int_g_ifparm returns platform-specific information about the
 * interface settings used by the sensor.
 *
 * Called on open.
 */
static int ioctl_g_ifparm(struct v4l2_int_device *s, struct v4l2_ifparm *p)
{
	adv7610_vidout_fmt_t fmt;

	if (s == NULL) {
		pr_err("   ERROR!! no slave device set!\n");
		return -ENODEV;
	}

	if (adv7610_get_vidout_fmt(&fmt))
		return -ENODEV;
	pr_debug("%s: %dx%d%c@%dfps\n", __func__, fmt.height, fmt.width,
		fmt.interlaced?'i':'p', fmt.fps);

	/* Initialize structure to 0s then set any non-0 values. */
	memset(p, 0, sizeof(*p));
#ifdef USE_16BIT
    p->if_type = V4L2_IF_TYPE_BT1120_PROGRESSIVE_SDR;
//    p->u.bt1120.clock_curr = 1;
//    p->u.bt1120.mode = V4L2_IF_TYPE_BT1120_MODE_BT_8BIT;
#else
	p->if_type = V4L2_IF_TYPE_BT656; /* This is the only possibility. */
	p->u.bt656.clock_curr = (fmt.interlaced)?0:1;
	p->u.bt656.mode = V4L2_IF_TYPE_BT656_MODE_NOBT_8BIT;
#endif

	return 0;
}

/*!
 * ioctl_g_parm - V4L2 sensor interface handler for VIDIOC_G_PARM ioctl
 * @s: pointer to standard V4L2 device structure
 * @a: pointer to standard V4L2 VIDIOC_G_PARM ioctl structure
 *
 * Returns the sensor's video CAPTURE parameters.
 */
static int ioctl_g_parm(struct v4l2_int_device *s, struct v4l2_streamparm *a)
{
	struct sensor *sensor = s->priv;
	struct v4l2_captureparm *cparm = &a->parm.capture;

	switch (a->type) {
	/* These are all the possible cases. */
	case V4L2_BUF_TYPE_VIDEO_CAPTURE:
		pr_debug("   type is V4L2_BUF_TYPE_VIDEO_CAPTURE\n");
		memset(a, 0, sizeof(*a));
		a->type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
		cparm->capability = sensor->sen.streamcap.capability;
		cparm->timeperframe = sensor->sen.streamcap.timeperframe;
		cparm->capturemode = sensor->sen.streamcap.capturemode;
		break;

	case V4L2_BUF_TYPE_VIDEO_OUTPUT:
	case V4L2_BUF_TYPE_VIDEO_OVERLAY:
	case V4L2_BUF_TYPE_VBI_CAPTURE:
	case V4L2_BUF_TYPE_VBI_OUTPUT:
	case V4L2_BUF_TYPE_SLICED_VBI_CAPTURE:
	case V4L2_BUF_TYPE_SLICED_VBI_OUTPUT:
		break;

	default:
		pr_debug("ioctl_g_parm:type is unknown %d\n", a->type);
		break;
	}

	return 0;
}

/*!
 * ioctl_s_parm - V4L2 sensor interface handler for VIDIOC_S_PARM ioctl
 * @s: pointer to standard V4L2 device structure
 * @a: pointer to standard V4L2 VIDIOC_S_PARM ioctl structure
 *
 * Configures the sensor to use the input parameters, if possible.  If
 * not possible, reverts to the old parameters and returns the
 * appropriate error code.
 *
 * This driver cannot change these settings.
 */
static int ioctl_s_parm(struct v4l2_int_device *s, struct v4l2_streamparm *a)
{
	switch (a->type) {
	/* These are all the possible cases. */
	case V4L2_BUF_TYPE_VIDEO_CAPTURE:
	case V4L2_BUF_TYPE_VIDEO_OUTPUT:
	case V4L2_BUF_TYPE_VIDEO_OVERLAY:
	case V4L2_BUF_TYPE_VBI_CAPTURE:
	case V4L2_BUF_TYPE_VBI_OUTPUT:
	case V4L2_BUF_TYPE_SLICED_VBI_CAPTURE:
	case V4L2_BUF_TYPE_SLICED_VBI_OUTPUT:
		break;

	default:
		pr_debug("   type is unknown - %d\n", a->type);
		break;
	}

	return 0;
}

/*!
 * ioctl_g_fmt_cap - V4L2 sensor interface handler for ioctl_g_fmt_cap
 * @s: pointer to standard V4L2 device structure
 * @f: pointer to standard V4L2 v4l2_format structure
 *
 * Returns the sensor's current pixel format in the v4l2_format
 * parameter.
 */
static int ioctl_g_fmt_cap(struct v4l2_int_device *s, struct v4l2_format *f)
{
	struct sensor *sensor = s->priv;
	adv7610_vidout_fmt_t fmt;

	if (adv7610_get_vidout_fmt(&fmt))
		return -ENODEV;

	sensor->sen.pix.height = fmt.height;
	sensor->sen.pix.width = fmt.width;
	sensor->sen.streamcap.timeperframe.denominator = fmt.fps;
	sensor->sen.streamcap.timeperframe.numerator = 1;
	switch (f->type) {
	case V4L2_BUF_TYPE_VIDEO_CAPTURE:
		f->fmt.pix = sensor->sen.pix;
		break;

	case V4L2_BUF_TYPE_PRIVATE: {
		}
		break;

	default:
		f->fmt.pix = sensor->sen.pix;
		break;
	}

	return 0;
}

/*!
 * ioctl_enum_framesizes - V4L2 sensor interface handler for
 *			   VIDIOC_ENUM_FRAMESIZES ioctl
 * @s: pointer to standard V4L2 device structure
 * @fsize: standard V4L2 VIDIOC_ENUM_FRAMESIZES ioctl structure
 *
 * Return 0 if successful, otherwise -EINVAL.
 */
static int ioctl_enum_framesizes(struct v4l2_int_device *s,
				 struct v4l2_frmsizeenum *fsize)
{
	struct sensor *sensor = s->priv;
	adv7610_vidout_fmt_t fmt;

	if (fsize->index >= 1)
		return -EINVAL;

    if (adv7610_get_vidout_fmt(&fmt))
		return -ENODEV;

	sensor->sen.pix.height = fmt.height;
	sensor->sen.pix.width = fmt.width;
	fsize->discrete.width = sensor->sen.pix.width;
	fsize->discrete.height  = sensor->sen.pix.height;

	return 0;
}

/*!
 * ioctl_g_chip_ident - V4L2 sensor interface handler for
 *			VIDIOC_DBG_G_CHIP_IDENT ioctl
 * @s: pointer to standard V4L2 device structure
 * @id: pointer to int
 *
 * Return 0.
 */
static int ioctl_g_chip_ident(struct v4l2_int_device *s, int *id)
{
	((struct v4l2_dbg_chip_ident *)id)->match.type =
					V4L2_CHIP_MATCH_I2C_DRIVER;
	strcpy(((struct v4l2_dbg_chip_ident *)id)->match.name,
						"adv7610_decoder");
	((struct v4l2_dbg_chip_ident *)id)->ident = V4L2_IDENT_ADV7610;

	return 0;
}

/*!
 * This structure defines all the ioctls for this module.
 */
static struct v4l2_int_ioctl_desc adv7610_ioctl_desc[] = {

/* {vidioc_int_s_power_num, (v4l2_int_ioctl_func*)ioctl_s_power}, */
	{vidioc_int_g_ifparm_num, (v4l2_int_ioctl_func*)ioctl_g_ifparm},
	{vidioc_int_g_fmt_cap_num, (v4l2_int_ioctl_func*)ioctl_g_fmt_cap},
	/*!
	 * If the requested format is supported, configures the HW to use that
	 * format, returns error code if format not supported or HW can't be
	 * correctly configured.
	 */
	{vidioc_int_g_parm_num, (v4l2_int_ioctl_func*)ioctl_g_parm},
	{vidioc_int_s_parm_num, (v4l2_int_ioctl_func*)ioctl_s_parm},
	{vidioc_int_enum_framesizes_num,
				(v4l2_int_ioctl_func *) ioctl_enum_framesizes},
	{vidioc_int_g_chip_ident_num,
				(v4l2_int_ioctl_func *)ioctl_g_chip_ident},
};

static struct v4l2_int_slave adv7610_slave = {
	.ioctls = adv7610_ioctl_desc,
	.num_ioctls = ARRAY_SIZE(adv7610_ioctl_desc),
};

static struct v4l2_int_device adv7610_int_device = {
	.module = THIS_MODULE,
	.name = "adv7610-video",
	.type = v4l2_int_type_slave,
	.u = {
		.slave = &adv7610_slave,
	},
};

/*! Write one register of a ADV7610 i2c slave device.
 *
 *  @param *reg         register in the device we wish to access.
 *
 *  @return                    0 if success, an error code otherwise.
 */
static int adv7610_write(struct i2c_client *client, u8 reg, u8 val)
{
        s32 ret;

        //pr_debug("adv7610 write %02X %02X\n",reg,val);
        ret = i2c_smbus_write_byte_data(client, reg, val);
        if (ret < 0) {
                dev_dbg(&client->dev,
                        "%s:write reg error:reg=%2x,val=%2x ret:%i\n", __func__,
                        reg, val, ret);
                return -1;
        }
        return 0;
}

/*! Read one register from a ADV7610 i2c slave device.
 *
 *  @param *reg         register in the device we wish to access.
 *
 *  @return                    0 if success, an error code otherwise.
 */
static inline int adv7610_read(struct i2c_client *client, u8 reg)
{
        int val;

        val = i2c_smbus_read_byte_data(client, reg);
        if (val < 0) {
                dev_dbg(&client->dev,
                        "%s:read reg error: reg=%2x\n", __func__, reg);
                return -1;
        }
        return val;
}
#ifdef SUPPORT_1080P
static const int edid[256] = {
    0x00,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0x00,0x06,0xD4,0x00,0x00,0x00,0x00,0x00,0x00,
    0x00,0x16,0x01,0x03,0x81,0x46,0x27,0x78,0x0A,0x32,0x30,0xA1,0x54,0x52,0x9E,0x26,
    0x0A,0x49,0x4B,0xA3,0x8,0x00,0x81,0xC0,0x81,0x00,0x01,0x001,0x81,0x40,0x01,0x01,
    0x01,0x01,0x01,0x01,0x01,0x01,0x00,0x00,0x00,0xFF,0x00,0x31,0x32,0x33,0x34,0x35,
    0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
    0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0xFC,0x00,0x33,
    0x44,0x52,0x0A,0x20,0x20,0x20,0x20,0x20,0x20,0x20,0x20,0x20,0x00,0x00,0x00,0xFD,
    0x00,0x38,0x4B,0x20,0x44,0x11,0x00,0x0A,0x20,0x20,0x20,0x20,0x20,0x20,0x01,0xB2,
    0x02,0x03,0x19,0x71,0x45,0x84,0x3E,0x05,0x10,0x22,0x26,0x15,0x07,0x50,0x09,0x07,
    0x01,0x67,0x03,0x0C,0x00,0x10,0x00,0x00,0x1E,0x01,0x1D,0x00,0x72,0x51,0xD0,0x1E,
    0x20,0x6E,0x28,0x55,0x00,0xC4,0x8E,0x21,0x00,0x00,0x1E,0x8C,0x0A,0xD0,0x8A,0x20,
    0xE0,0x2D,0x10,0x10,0x3E,0x96,0x00,0xC4,0x8E,0x21,0x00,0x00,0x18,0x00,0x00,0x00,
    0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
    0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
    0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
    0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0xE5,
};

#else
static const int edid[256] = {
    0x00,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0x00,0x06,0xD4,0x00,0x00,0x00,0x00,0x00,0x00,
    0x00,0x16,0x01,0x03,0x81,0x46,0x27,0x78,0x0A,0x32,0x30,0xA1,0x54,0x52,0x9E,0x26,
    0x0A,0x49,0x4B,0xA3,0x08,0x00,0x81,0xC0,0x81,0x00,0x01,0x01,0x81,0x40,0x01,0x01,
    0x01,0x01,0x01,0x01,0x01,0x01,0x00,0x00,0x00,0xFF,0x00,0x31,0x32,0x33,0x34,0x35,
    0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
    0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0xFC,0x00,0x33,
    0x44,0x52,0x0A,0x20,0x20,0x20,0x20,0x20,0x20,0x20,0x20,0x20,0x00,0x00,0x00,0xFD,
    0x00,0x38,0x4B,0x20,0x44,0x11,0x00,0x0A,0x20,0x20,0x20,0x20,0x20,0x20,0x01,0xB2,
    0x02,0x03,0x1A,0x70,0x46,0x04,0x3E,0x03,0x13,0x12,0x11,0x26,0x15,0x07,0x50,0x09,
    0x07,0x01,0x67,0x03,0x0C,0x00,0x10,0x00,0x00,0x1E,0x01,0x1D,0x00,0x72,0x51,0xD0,
    0x1E,0x20,0x6E,0x28,0x55,0x00,0xC4,0x8E,0x21,0x00,0x00,0x1E,0x8C,0x0A,0xD0,0x8A,
    0x20,0xE0,0x2D,0x10,0x10,0x3E,0x96,0x00,0xC4,0x8E,0x21,0x00,0x00,0x18,0x00,0x00,
    0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
    0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
    0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
    0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x62,
};
#endif

static int adv7610_hw_init(struct i2c_client *client)
{
    int i, chip_id_high, chip_id_low;

    //Make sure its there...
    chip_id_high = adv7610_read(client,ADV7610_CHIP_ID_HIGH_BYTE);
    if (chip_id_high != 0x20) {
        pr_warning("adv7610 is not found\n");
        return -ENODEV;
    }
    chip_id_low = adv7610_read(client,ADV7610_CHIP_ID_LOW_BYTE);
    if (chip_id_low != 0x51) {
        pr_warning("adv7610 is not found\n");
        return -ENODEV;
    }

    dev_dbg(&client->dev,
            "In adv7610:adv7610_hw_init\n");

    adv7610_hard_reset(); //Hardware reset first

    adv7610_write(client, 0xFF, 0x80); //Perform a software reset.
    msleep(100);

    //Set up the map addresses
    adv7610_write(client,	0xF4,	CEC_MAP_ADDR);   //cec map i2c address
    adv7610_write(client,	0xF5,	INFOFRAME_MAP_ADDR);     //infofrmae map i2c address
    adv7610_write(client,	0xF8,	DPLL_MAP_ADDR);  //dpll map i2c address
    adv7610_write(client,	0xF9,	KSV_MAP_ADDR);   //ksv map i2c address
    adv7610_write(client,	0xFA,	EDID_MAP_ADDR);  //edid map i2c address
    adv7610_write(client,	0xFB,	HDMI_MAP_ADDR);  //hdmi map i2c address
    adv7610_write(client,	0xFD,	CP_MAP_ADDR);    //cp map i2c address


    //i2c dummy clients based on the above map addresses
    adv7610_i2c_clients.io = 	client;
    adv7610_i2c_clients.cec = 	i2c_new_dummy(client->adapter, CEC_MAP_ADDR >> 1);
    adv7610_i2c_clients.infoframe = i2c_new_dummy(client->adapter, INFOFRAME_MAP_ADDR >> 1);
    adv7610_i2c_clients.dpll = 	i2c_new_dummy(client->adapter, DPLL_MAP_ADDR >> 1);
    adv7610_i2c_clients.ksv = 	i2c_new_dummy(client->adapter, KSV_MAP_ADDR >> 1);
    adv7610_i2c_clients.edid = 	i2c_new_dummy(client->adapter, EDID_MAP_ADDR >> 1);
    adv7610_i2c_clients.hdmi= 	i2c_new_dummy(client->adapter, HDMI_MAP_ADDR >> 1);
    adv7610_i2c_clients.cp = 	i2c_new_dummy(client->adapter, CP_MAP_ADDR >> 1);

    //Set the AUTO_EDID to turn on when the internal EDID is enabled
    adv7610_write(adv7610_i2c_clients.hdmi, 0x6C, 0xA0);
    
    adv7610_write(adv7610_i2c_clients.ksv, 0x77, 0x00); //Disable the Internal EDID
    //Set the EDID value
    for(i=0; i<=0xFF; ++i)
        adv7610_write(adv7610_i2c_clients.edid,i,edid[i]);
    adv7610_write(adv7610_i2c_clients.ksv, 0x77, 0x00); //Set the Most Significant Bit of the SPA location to 0
    adv7610_write(adv7610_i2c_clients.ksv, 0x52, 0x20); //Set the SPA for port B.
    adv7610_write(adv7610_i2c_clients.ksv, 0x53, 0x00); //Set the SPA for port B.
    adv7610_write(adv7610_i2c_clients.ksv, 0x70, 0x9E); //Set the Least Significant Byte of the SPA location
    adv7610_write(adv7610_i2c_clients.ksv, 0x74, 0x03); //Enable the Internal EDID for Ports

    adv7610_write(adv7610_i2c_clients.hdmi, 0x00, 0x00); //Select port A (shouldn't matter, only 1 port)
#ifdef USE_16BIT
    adv7610_write(adv7610_i2c_clients.io, 0x03, 0x80);// 16-bit SDR ITU-656 mode
    adv7610_write(adv7610_i2c_clients.io, 0x19, 0x89);
#else
    adv7610_write(adv7610_i2c_clients.io, 0x03, 0x00);// 8-bit SDR ITU-656 mode
    adv7610_write(adv7610_i2c_clients.io, 0x19, 0xC0);// Enable LLC DLL, double clock
    adv7610_write(adv7610_i2c_clients.io, 0x33, 0x40);// Enable LLC_DLL_MUX
    adv7610_write(adv7610_i2c_clients.io, 0x06, 0xA1); // Invert CLK
#endif

    adv7610_write(adv7610_i2c_clients.io, 0x14, 0x55); //med-low drive strength (including clock)
    adv7610_write(adv7610_i2c_clients.cp, 0xBA, 0x01); //Set HDMI FreeRun
    adv7610_write(adv7610_i2c_clients.io, 0x0B, 0x44);// Power up part
    adv7610_write(adv7610_i2c_clients.io, 0x0c, 0x42); //Power up part
    adv7610_write(adv7610_i2c_clients.io, 0x15, 0xB8); // Disable Tristate of clock and data 

    adv7610_write(adv7610_i2c_clients.ksv, 0x40, 0x81); // Disable HDCP 1.1 features
    adv7610_write(adv7610_i2c_clients.hdmi, 0x9B, 0x03); // ADI recommended setting
    adv7610_write(adv7610_i2c_clients.hdmi, 0xC1, 0x01); // ADI recommended setting
    adv7610_write(adv7610_i2c_clients.hdmi, 0xC2, 0x01); // ADI recommended setting
    adv7610_write(adv7610_i2c_clients.hdmi, 0xC3, 0x01); // ADI recommended setting
    adv7610_write(adv7610_i2c_clients.hdmi, 0xC4, 0x01); // ADI recommended setting
    adv7610_write(adv7610_i2c_clients.hdmi, 0xC5, 0x01); // ADI recommended setting
    adv7610_write(adv7610_i2c_clients.hdmi, 0xC6, 0x01); // ADI recommended setting
    adv7610_write(adv7610_i2c_clients.hdmi, 0xC7, 0x01); // ADI recommended setting
    adv7610_write(adv7610_i2c_clients.hdmi, 0xC8, 0x01); // ADI recommended setting
    adv7610_write(adv7610_i2c_clients.hdmi, 0xC9, 0x01); // ADI recommended setting
    adv7610_write(adv7610_i2c_clients.hdmi, 0xCA, 0x01); // ADI recommended setting
    adv7610_write(adv7610_i2c_clients.hdmi, 0xCB, 0x01); // ADI recommended setting
    adv7610_write(adv7610_i2c_clients.hdmi, 0xCC, 0x01); // ADI recommended setting
    adv7610_write(adv7610_i2c_clients.hdmi, 0x00, 0x00); // Set HDMI Input Port A
    adv7610_write(adv7610_i2c_clients.hdmi, 0x83, 0xFE); // Enable clock terminator for port A
    adv7610_write(adv7610_i2c_clients.hdmi, 0x6F, 0x08); // ADI recommended setting
    adv7610_write(adv7610_i2c_clients.hdmi, 0x85, 0x1F); // ADI recommended setting
    adv7610_write(adv7610_i2c_clients.hdmi, 0x87, 0x70); // ADI recommended setting
    adv7610_write(adv7610_i2c_clients.hdmi, 0x8D, 0x04); // LFG
    adv7610_write(adv7610_i2c_clients.hdmi, 0x8E, 0x1E); // HFG
    adv7610_write(adv7610_i2c_clients.hdmi, 0x1A, 0x8A); // unmute audio
    adv7610_write(adv7610_i2c_clients.hdmi, 0x57, 0xDA); // ADI recommended setting
    adv7610_write(adv7610_i2c_clients.hdmi, 0x58, 0x01); // ADI recommended setting
    adv7610_write(adv7610_i2c_clients.hdmi, 0x75, 0x10); // DDC drive strength

    adv7610_write(adv7610_i2c_clients.io, 0x1, 0x6); // Prim_Mode =110b HDMI-GR
    adv7610_write(adv7610_i2c_clients.io, 0x2, 0xF5); // Auto CSC, RGB out, Set op_656 bit

    //Free run should be black, not blue
    adv7610_write(adv7610_i2c_clients.cp, 0xBF, 0x16); //Turn on DEF_COL_MAN_VAL
    adv7610_write(adv7610_i2c_clients.cp, 0xC0, 0x00); //Free run color CHA (Y)
    adv7610_write(adv7610_i2c_clients.cp, 0xC1, 0x80); //Free run color CHB (U)
    adv7610_write(adv7610_i2c_clients.cp, 0xC2, 0x80); //Free run color CHC (V)

    return 0;
}

static int adv7610_video_probe(struct i2c_client *client,
			const struct i2c_device_id *id)
{
	struct sensor *sens = &adv7610_data;
	struct device *dev = &client->dev;
	int ret;

	dev_dbg(dev, "%s\n", __func__);

	init_waitqueue_head(&i2c_wait);

	/* Set initial values for the sensor struct. */
	memset(sens, 0, sizeof(adv7610_data));
	sens->sen.streamcap.timeperframe.denominator = 0;
	sens->sen.streamcap.timeperframe.numerator = 0;
	sens->sen.pix.width = 0;
	sens->sen.pix.height = 0;
#ifdef USE_16BIT
	sens->sen.pix.pixelformat = V4L2_PIX_FMT_YUYV;
#else
	sens->sen.pix.pixelformat = V4L2_PIX_FMT_UYVY;  /* YUV422 */
#endif
	sens->sen.on = true;

        /* request reset pin */
        rst_gpio = of_get_named_gpio(dev->of_node, "reset-gpio", 0);
        if (!gpio_is_valid(rst_gpio)) {
                dev_warn(dev, "no sensor reset pin available");
                return -EINVAL;
        }
        ret = devm_gpio_request_one(dev, rst_gpio, GPIOF_OUT_INIT_HIGH,
                                        "adv7610_reset");
        if (ret < 0)
                return ret;

	ret = of_property_read_u32(dev->of_node, "csi_id",
				   &(sens->sen.csi));
	if (ret) {
		dev_err(dev, "csi_id invalid\n");
		return ret;
	}

	dev_dbg(dev, "IPU1_CSI%d\n", sens->sen.csi);
	dev_dbg(dev, "type is %d (expect %d)\n",
		 adv7610_int_device.type, v4l2_int_type_slave);
	dev_dbg(dev, "num ioctls is %d\n",
		 adv7610_int_device.u.slave->num_ioctls);

	//Attempt to connect to the device
	ret = adv7610_hw_init(client);
	if(ret == -ENODEV)
		return ret;

    /* This function attaches this structure to the /dev/video<n> device */
    adv7610_int_device.priv = sens;
    ret = v4l2_int_device_register(&adv7610_int_device);

	return ret;
}

static int adv7610_video_remove(struct i2c_client *client)
{
	v4l2_int_device_unregister(&adv7610_int_device);

	return 0;
}

static const struct i2c_device_id adv7610_id[] = {
	{"adv7610", 0},
	{},
};

MODULE_DEVICE_TABLE(i2c, adv7610_id);

static struct i2c_driver adv7610_i2c_driver = {
	.driver = {
		  .owner = THIS_MODULE,
		  .name  = "adv7610",
		  },
	.probe  = adv7610_video_probe,
	.remove = adv7610_video_remove,
	.id_table = adv7610_id,
};

/*!
 * adv7610 init function
 * Called by insmod adv7610_hdmi.ko.
 *
 * @return  Error code indicating success or failure
 */
static __init int adv7610_init(void)
{
	u8 err;

	err = i2c_add_driver(&adv7610_i2c_driver);
	if (err != 0)
		pr_err("%s:driver registration failed, error=%d\n",
			__func__, err);

	return err;
}

/*!
 * ADV7610 cleanup function
 * Called on rmmod adv7610_hdmi.ko
 *
 * @return  Error code indicating success or failure
 */
static void __exit adv7610_clean(void)
{
	if (adv7610_i2c_clients.infoframe != NULL)
		i2c_unregister_device(adv7610_i2c_clients.infoframe);
	if (adv7610_i2c_clients.cec != NULL)
		i2c_unregister_device(adv7610_i2c_clients.cec);
	if (adv7610_i2c_clients.dpll != NULL)
		i2c_unregister_device(adv7610_i2c_clients.dpll);
	if (adv7610_i2c_clients.ksv != NULL)
		i2c_unregister_device(adv7610_i2c_clients.ksv);
	if (adv7610_i2c_clients.edid != NULL)
		i2c_unregister_device(adv7610_i2c_clients.edid);
	if (adv7610_i2c_clients.hdmi != NULL)
		i2c_unregister_device(adv7610_i2c_clients.hdmi);
	if (adv7610_i2c_clients.cp != NULL)
		i2c_unregister_device(adv7610_i2c_clients.cp);

	i2c_del_driver(&adv7610_i2c_driver);
}

module_init(adv7610_init);
module_exit(adv7610_clean);

MODULE_AUTHOR("Allan Matthew <amatthew@3drobotics.com>");
MODULE_DESCRIPTION("ADV7610 hdmi receiver MXC video capture driver");
MODULE_LICENSE("GPL");
MODULE_ALIAS("CSI");
