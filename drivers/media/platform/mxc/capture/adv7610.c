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

#define DEBUG
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

#include <media/v4l2-chip-ident.h>
#include <media/v4l2-int-device.h>

#include "mxc_v4l2_capture.h"

/*!
 * Maintains the information on the current state of the sensor.
 */
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

static struct adv7610_i2c_map adv7610_i2c_clients;

static int adv7610_write(struct i2c_client *client, u8 reg, u8 val);
static inline int adv7610_read(struct i2c_client *client, u8 reg);

/***********************************************************************
 * mxc_v4l2_capture interface.
 ***********************************************************************/

//Get the video output format from the ADV7610
int adv7610_get_vidout_fmt(adv7610_vidout_fmt_t *fmt)
{
	int val, width, height;

	//Pull data from the i2c here to determine the format.

	//Read the line width
	val = adv7610_read(adv7610_i2c_clients.hdmi, 0x07);
	width = (val & 0x1F) << 8;
	val = adv7610_read(adv7610_i2c_clients.hdmi, 0x08);
	width |= val;
	fmt->width = width;

	//read the field height
	val = adv7610_read(adv7610_i2c_clients.hdmi, 0x09);
	height = (val & 0x1F) << 8;
	val = adv7610_read(adv7610_i2c_clients.hdmi, 0x0A);
	height |= val;
	fmt->height = height;

	//Determine if its interlaced
	val = adv7610_read(adv7610_i2c_clients.hdmi, 0x0B);
	fmt->interlaced = (bool)(val & 0x20);

	//Fixed 30fps for now
	fmt->fps=30;

	//Not quite working yet...
	fmt->width = 1280;
	fmt->height = 720;
	fmt->interlaced = false;

	pr_debug("ADV7610 returned %ix%i%s@%i\n",
		fmt->width, fmt->height,
		(fmt->interlaced?"i":"p"),
		fmt->fps);


	return 0;
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
	p->if_type = V4L2_IF_TYPE_BT656; /* This is the only possibility. */
	p->u.bt656.clock_curr = (fmt.interlaced)?0:1;
	p->u.bt656.mode = V4L2_IF_TYPE_BT656_MODE_NOBT_8BIT;

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

	pr_debug("%s: %dx%d%c@%dfps\n", __func__, fmt.width, fmt.height,
		fmt.interlaced?'i':'p', fmt.fps);
	sensor->sen.pix.height = fmt.height;
	sensor->sen.pix.width = fmt.width;
	sensor->sen.streamcap.timeperframe.denominator = fmt.fps;
	sensor->sen.streamcap.timeperframe.numerator = 1;
	switch (f->type) {
	case V4L2_BUF_TYPE_VIDEO_CAPTURE:
		pr_debug("   Returning size of %dx%d\n",
			 sensor->sen.pix.width, sensor->sen.pix.height);
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

	pr_debug("%s: %dx%d%c@%dfps\n", __func__, fmt.width, fmt.height,
		fmt.interlaced?'i':'p', fmt.fps);
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

static int edid[256] = {
	0x0,	0xFF,	0xFF,	0xFF,	0xFF,	0xFF,	0xFF,	0x0,	0x6,	0x8F,	0x7,	0x11,
	0x1,	0x0,	0x0,	0x0,	0x17,	0x11,	0x1,	0x3,	0x80,	0x0C,	0x9,	0x78,
	0x0A,	0x1E,	0xAC,	0x98,	0x59,	0x56,	0x85,	0x28,	0x29,	0x52,	0x57,	0x0,
	0x0,	0x0,	0x1,	0x1,	0x1,	0x1,	0x1,	0x1,	0x1,	0x1,	0x1,	0x1,
	0x1,	0x1,	0x1,	0x1,	0x1,	0x1,	0x8C,	0x0A,	0xD0,	0x8A,	0x20,	0xE0,
	0x2D,	0x10,	0x10,	0x3E,	0x96,	0x0,	0x81,	0x60,	0x0,	0x0,	0x0,	0x18,
	0x1,	0x1D,	0x80,	0x18,	0x71,	0x1C,	0x16,	0x20,	0x58,	0x2C,	0x25,	0x0,
	0x81,	0x49,	0x0,	0x0,	0x0,	0x9E,	0x0,	0x0,	0x0,	0xFC,	0x0,	0x56,
	0x41,	0x2D,	0x31,	0x38,	0x30,	0x39,	0x41,	0x0A,	0x20,	0x20,	0x20,	0x20,
	0x0,	0x0,	0x0,	0xFD,	0x0,	0x17,	0x3D,	0x0D,	0x2E,	0x11,	0x0,	0x0A,
	0x20,	0x20,	0x20,	0x20,	0x20,	0x20,	0x1,	0x1C,	0x2,	0x3,	0x34,	0x71,
	0x4D,	0x82,	0x5,	0x4,	0x1,	0x10,	0x11,	0x14,	0x13,	0x1F,	0x6,	0x15,
	0x3,	0x12,	0x35,	0x0F,	0x7F,	0x7,	0x17,	0x1F,	0x38,	0x1F,	0x7,	0x30,
	0x2F,	0x7,	0x72,	0x3F,	0x7F,	0x72,	0x57,	0x7F,	0x0,	0x37,	0x7F,	0x72,
	0x83,	0x4F,	0x0,	0x0,	0x67,	0x3,	0x0C,	0x0,	0x10,	0x0,	0x88,	0x2D,
	0x0,	0x0,	0x0,	0xFF,	0x0,	0x0A,	0x20,	0x20,	0x20,	0x20,	0x20,	0x20,
	0x20,	0x20,	0x20,	0x20,	0x20,	0x20,	0x0,	0x0,	0x0,	0xFF,	0x0,	0x0A,
	0x20,	0x20,	0x20,	0x20,	0x20,	0x20,	0x20,	0x20,	0x20,	0x20,	0x20,	0x20,
	0x0,	0x0,	0x0,	0xFF,	0x0,	0x0A,	0x20,	0x20,	0x20,	0x20,	0x20,	0x20,	
	0x20,	0x20,	0x20,	0x20,	0x20,	0x20,	0x0,	0x0,	0x0,	0x0,	0x0,	0x0,
	0x0,	0x0,	0x0,	0x0,	0x0,	0x0,	0x0,	0x0,	0x0,	0x0,	0x0,	0x0,
	0x0,	0x0,	0x0,	0xDA,
};

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
 
        adv7610_write(client, 0xFF, 0x80); //Perform a reset.
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

	adv7610_write(adv7610_i2c_clients.ksv, 0x77, 0x00); //Disable the Internal EDID

	for(i=0; i<=0xFF; ++i)
		adv7610_write(adv7610_i2c_clients.edid,i,edid[i]);

	adv7610_write(adv7610_i2c_clients.ksv, 0x77, 0x00); //Set the Most Significant Bit of the SPA location to 0
	adv7610_write(adv7610_i2c_clients.ksv, 0x52, 0x20); //Set the SPA for port B.
	adv7610_write(adv7610_i2c_clients.ksv, 0x53, 0x00); //Set the SPA for port B.
	adv7610_write(adv7610_i2c_clients.ksv, 0x70, 0x9E); //Set the Least Significant Byte of the SPA location
	adv7610_write(adv7610_i2c_clients.ksv, 0x74, 0x03); //Enable the Internal EDID for Ports

        adv7610_write(adv7610_i2c_clients.io, 0x03, 0x00);// 8-bit SDR ITU-656 mode

        //adv7610_write(adv7610_i2c_clients.io, 0x06, 0xA7); // Invert VS + HS + CLK
        adv7610_write(adv7610_i2c_clients.io, 0x06, 0xA1); // Invert CLK
        //adv7610_write(adv7610_i2c_clients.io, 0x06, 0xAF); // Invert everything
        //adv7610_write(adv7610_i2c_clients.io, 0x06, 0xA6); // Invert VS + HS

        adv7610_write(adv7610_i2c_clients.io, 0x19, 0xC0);// Enable LLC DLL, double clock
        adv7610_write(adv7610_i2c_clients.io, 0x33, 0x40);// Enable LLC_DLL_MUX
        adv7610_write(adv7610_i2c_clients.cp, 0xBA, 0x01); //Set HDMI FreeRun
	adv7610_write(adv7610_i2c_clients.io, 0x14, 0x7F); //Max Drive Strength

        adv7610_write(adv7610_i2c_clients.io, 0x0B, 0x44);// Power up part
        adv7610_write(adv7610_i2c_clients.io, 0x0c, 0x42); //Power up part
        adv7610_write(adv7610_i2c_clients.io, 0x15, 0xB8); // Disable Tristate of clock and data only
        //adv7610_write(adv7610_i2c_clients.io, 0x15, 0xB0); // Disable Tristate of video and clock pins

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

	adv7610_write(adv7610_i2c_clients.io, 0x01, 0x25);   //30fps, HDMI COMP
	adv7610_write(adv7610_i2c_clients.io, 0x00, 0x13);   //720p
	
	return 0;
}

static int adv7610_video_probe(struct i2c_client *client,
			const struct i2c_device_id *id)
{
	struct sensor *sens = &adv7610_data;
	struct device *dev = &client->dev;
	int ret;

	dev_dbg(dev, "%s\n", __func__);

	/* Set initial values for the sensor struct. */
	memset(sens, 0, sizeof(adv7610_data));
	sens->sen.streamcap.timeperframe.denominator = 0;
	sens->sen.streamcap.timeperframe.numerator = 0;
	sens->sen.pix.width = 0;
	sens->sen.pix.height = 0;
	sens->sen.pix.pixelformat = V4L2_PIX_FMT_UYVY;  /* YUV422 */
	sens->sen.on = true;

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
        i2c_unregister_device(adv7610_i2c_clients.infoframe);
        i2c_unregister_device(adv7610_i2c_clients.cec);
        i2c_unregister_device(adv7610_i2c_clients.dpll);
        i2c_unregister_device(adv7610_i2c_clients.ksv);
        i2c_unregister_device(adv7610_i2c_clients.edid);
        i2c_unregister_device(adv7610_i2c_clients.hdmi);
        i2c_unregister_device(adv7610_i2c_clients.cp);

	i2c_del_driver(&adv7610_i2c_driver);
}

module_init(adv7610_init);
module_exit(adv7610_clean);

MODULE_AUTHOR("Allan Matthew <amatthew@3drobotics.com>");
MODULE_DESCRIPTION("ADV7610 hdmi receiver MXC video capture driver");
MODULE_LICENSE("GPL");
MODULE_ALIAS("CSI");
