/*
 * Support for gc0310 Camera Sensor.
 *
 * Copyright (c) 2012 Intel Corporation. All Rights Reserved.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License version
 * 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA
 * 02110-1301, USA.
 *
 */


#ifndef __GC0310_H__
#define __GC0310_H__

#include <linux/kernel.h>
#include <linux/types.h>
#include <linux/i2c.h>
#include <linux/delay.h>
#include <linux/videodev2.h>
#include <linux/spinlock.h>
#include <media/v4l2-subdev.h>
#include <media/v4l2-device.h>
#include <media/v4l2-chip-ident.h>
#include <linux/v4l2-mediabus.h>
#include <media/media-entity.h>
#include <linux/atomisp_platform.h>
#include <linux/atomisp.h>

#define V4L2_IDENT_GC0310 8245


/* system registers */
#define GC0310_REG_SENSOR_ID_H		0xf0
#define GC0310_REG_PLL_MODE1		0xf7
#define GC0310_REG_PLL_MODE2		0xf8
#define GC0310_REG_PAGE_SELECT		0xfe

/* P0 registers */
#define GC0310_REG_COARSE_EXPOSURE_H	0x03
#define GC0310_REG_COARSE_EXPOSURE_L	0x04
#define GC0310_REG_HB_H			0x05
#define GC0310_REG_HB_L			0x06
#define GC0310_REG_VB_H			0x07
#define GC0310_REG_VB_L			0x08
#define GC0310_REG_ROW_START_H		0x09
#define GC0310_REG_ROW_START_L		0x0a
#define GC0310_REG_COL_START_H		0x0b
#define GC0310_REG_COL_START_L		0x0c
#define GC0310_REG_HEIGHT_H		0x0d
#define GC0310_REG_HEIGHT_L		0x0e
#define GC0310_REG_WIDTH_H		0x0f
#define GC0310_REG_WIDTH_L		0x10
#define GC0310_REG_SH_DELAY		0x11
#define GC0310_REG_A_GAIN		0x48
#define GC0310_REG_D_GAIN		0x71

#define I2C_RETRY_COUNT		5
#define MSG_LEN_OFFSET		1 /* 8-bits addr */


/* GC0310_DEVICE_ID */
#define GC0310_MOD_ID_H		0xa3

#define GC0310_FINE_INTG_TIME_MIN 0
#define GC0310_FINE_INTG_TIME_MAX_MARGIN 0
#define GC0310_COARSE_INTG_TIME_MIN 1
#define GC0310_COARSE_INTG_TIME_MAX_MARGIN 6


#define GC0310_FOCAL_LENGTH_NUM	208	/*2.08mm*/
#define GC0310_FOCAL_LENGTH_DEM	100
#define GC0310_F_NUMBER_DEFAULT_NUM	24
#define GC0310_F_NUMBER_DEM	10
#define GC0310_WAIT_STAT_TIMEOUT	100
#define GC0310_FLICKER_MODE_50HZ	1
#define GC0310_FLICKER_MODE_60HZ	2
/*
 * focal length bits definition:
 * bits 31-16: numerator, bits 15-0: denominator
 */
#define GC0310_FOCAL_LENGTH_DEFAULT 0xD00064

/*
 * current f-number bits definition:
 * bits 31-16: numerator, bits 15-0: denominator
 */
#define GC0310_F_NUMBER_DEFAULT 0x18000a

/*
 * f-number range bits definition:
 * bits 31-24: max f-number numerator
 * bits 23-16: max f-number denominator
 * bits 15-8: min f-number numerator
 * bits 7-0: min f-number denominator
 */
#define GC0310_F_NUMBER_RANGE 0x180a180a

/* Supported resolutions */
enum {
	GC0310_RES_CIF,
	GC0310_RES_VGA,
};

#define GC0310_RES_VGA_SIZE_H		640
#define GC0310_RES_VGA_SIZE_V		480
#define GC0310_RES_CIF_SIZE_H		352
#define GC0310_RES_CIF_SIZE_V		288
#define GC0310_RES_QVGA_SIZE_H		320
#define GC0310_RES_QVGA_SIZE_V		240
#define GC0310_RES_QCIF_SIZE_H		176
#define GC0310_RES_QCIF_SIZE_V		144


/* #defines for register writes and register array processing */
#define SENSOR_WAIT_MS		0
#define GC0310_8BIT		1
#define GC0310_16BIT		2
#define GC0310_32BIT		4
#define SENSOR_TABLE_END	0xFF

struct gc0310_reg_cmd {
	u16 cmd;
	u16 addr;
	u16 val;
};

struct gc0310_device {
	struct v4l2_subdev sd;
	struct media_pad pad;
	struct v4l2_mbus_framefmt format;

	struct camera_sensor_platform_data *platform_data;
	int real_model_id;
	int nctx;
	int power;

	unsigned int bus_width;
	unsigned int mode;
	unsigned int field_inv;
	unsigned int field_sel;
	unsigned int ycseq;
	unsigned int conv422;
	unsigned int bpat;
	unsigned int hpol;
	unsigned int vpol;
	unsigned int edge;
	unsigned int bls;
	unsigned int gamma;
	unsigned int cconv;
	unsigned int res;
	unsigned int dwn_sz;
	unsigned int blc;
	unsigned int agc;
	unsigned int awb;
	unsigned int aec;
	/* extention SENSOR version 2 */
	unsigned int cie_profile;

	/* extention SENSOR version 3 */
	unsigned int flicker_freq;

	/* extension SENSOR version 4 */
	unsigned int smia_mode;
	unsigned int mipi_mode;

	/* Add name here to load shared library */
	unsigned int type;

	/*Number of MIPI lanes*/
	unsigned int mipi_lanes;
	char name[32];

	u8 lightfreq;
};

struct gc0310_res_struct {
	u8 *desc;
	int res;
	int width;
	int height;
	int fps;
	int skip_frames;
	bool used;
	u16 pixels_per_line;
	u16 lines_per_frame;
	u8 bin_factor_x;
	u8 bin_factor_y;
	u8 bin_mode;
};

struct gc0310_control {
	struct v4l2_queryctrl qc;
	int (*query)(struct v4l2_subdev *sd, s32 *value);
	int (*tweak)(struct v4l2_subdev *sd, int value);
};

/*
 * Modes supported by the gc0310 driver.
 * Please, keep them in ascending order.
 */
static struct gc0310_res_struct gc0310_res[] = {
	{
	.desc	= "VGA",
	.res	= GC0310_RES_VGA,
	.width	= 656,
	.height	= 496,
	.fps	= 16.7,
	.used	= 0,
	.skip_frames = 2,

	.pixels_per_line = 0x03e8,
	.lines_per_frame = 0x0240,
	.bin_factor_x = 1,
	.bin_factor_y = 1,
	.bin_mode = 0,
	},
};
#define N_RES (ARRAY_SIZE(gc0310_res))

static const struct i2c_device_id gc0310_id[] = {
	{"gc0310", 0},
	{}
};

static struct gc0310_reg_cmd const gc0310_suspend[] = {
	{SENSOR_TABLE_END, 0, 0}
};

static struct gc0310_reg_cmd const gc0310_streaming[] = {
	{SENSOR_TABLE_END, 0, 0}
};

static struct gc0310_reg_cmd const gc0310_standby_reg[] = {
	{SENSOR_TABLE_END, 0, 0}
};

static struct gc0310_reg_cmd const gc0310_wakeup_reg[] = {
	{SENSOR_TABLE_END, 0, 0}
};

static struct gc0310_reg_cmd const gc0310_chgstat_reg[] = {
	{SENSOR_TABLE_END, 0, 0}
};
static struct gc0310_reg_cmd const gc0310_qcif_init[] = {

	{SENSOR_TABLE_END, 0, 0}
};

static struct gc0310_reg_cmd const gc0310_qvga_init[] = {

	{SENSOR_TABLE_END, 0, 0}
};

static struct gc0310_reg_cmd const gc0310_vga_init[] = {

	{SENSOR_TABLE_END, 0, 0}
};



static struct gc0310_reg_cmd const gc0310_common[] = {

/////////////////////////////////////////////////
/////////////////	system reg	/////////////////
/////////////////////////////////////////////////
{GC0310_8BIT, 0xfe, 0xf0},
{GC0310_8BIT, 0xfe, 0xf0},
{GC0310_8BIT, 0xfe, 0x00},
{GC0310_8BIT, 0xfc, 0x0e},
{GC0310_8BIT, 0xfc, 0x0e},
{GC0310_8BIT, 0xf2, 0x80},
{GC0310_8BIT, 0xf3, 0x00},
{GC0310_8BIT, 0xf7, 0x33},
{GC0310_8BIT, 0xf8, 0x05},
{GC0310_8BIT, 0xf9, 0x0e},
{GC0310_8BIT, 0xfa, 0x11},
/////////////////////////////////////////////////
///////////////////   MIPI	 ////////////////////
/////////////////////////////////////////////////
{GC0310_8BIT, 0xfe, 0x03},
{GC0310_8BIT, 0x01, 0x03},
{GC0310_8BIT, 0x02, 0x11},
{GC0310_8BIT, 0x03, 0x94},
{GC0310_8BIT, 0x04, 0x01},
{GC0310_8BIT, 0x05, 0x00},
{GC0310_8BIT, 0x06, 0x80},
{GC0310_8BIT, 0x11, 0x2a},
{GC0310_8BIT, 0x12, 0x90},
{GC0310_8BIT, 0x13, 0x02},
{GC0310_8BIT, 0x15, 0x12},
{GC0310_8BIT, 0x17, 0x01},
{GC0310_8BIT, 0x40, 0x08},
{GC0310_8BIT, 0x41, 0x00},
{GC0310_8BIT, 0x42, 0x00},
{GC0310_8BIT, 0x43, 0x00},
{GC0310_8BIT, 0x21, 0x02},
{GC0310_8BIT, 0x22, 0x02},
{GC0310_8BIT, 0x23, 0x01},
{GC0310_8BIT, 0x29, 0x01},
{GC0310_8BIT, 0x2A, 0x25},
{GC0310_8BIT, 0x2B, 0x02},
{GC0310_8BIT, 0xfe, 0x00},
/////////////////////////////////////////////////
/////////////////////////////////////////////////
/////////////////	CISCTL reg	/////////////////
/////////////////////////////////////////////////
{GC0310_8BIT, 0x00, 0x2f},
{GC0310_8BIT, 0x01, 0x0f},
{GC0310_8BIT, 0x02, 0x04},
{GC0310_8BIT, 0x4f, 0x00},
{GC0310_8BIT, 0x03, 0x01},
{GC0310_8BIT, 0x04, 0xec},
{GC0310_8BIT, 0x05, 0x00},
{GC0310_8BIT, 0x06, 0xa8},
{GC0310_8BIT, 0x07, 0x00},
{GC0310_8BIT, 0x08, 0x20},
{GC0310_8BIT, 0x09, 0x00},
{GC0310_8BIT, 0x0a, 0x00},
{GC0310_8BIT, 0x0b, 0x00},
{GC0310_8BIT, 0x0c, 0x00},
{GC0310_8BIT, 0x0d, 0x01},
{GC0310_8BIT, 0x0e, 0xf2},
{GC0310_8BIT, 0x0f, 0x02},
{GC0310_8BIT, 0x10, 0x94},
{GC0310_8BIT, 0x17, 0x14},
{GC0310_8BIT, 0x18, 0x1a},
{GC0310_8BIT, 0x19, 0x14},
{GC0310_8BIT, 0x1b, 0x48},
{GC0310_8BIT, 0x1e, 0x6b},
{GC0310_8BIT, 0x1f, 0x28},
{GC0310_8BIT, 0x20, 0x89},
{GC0310_8BIT, 0x21, 0x49},
{GC0310_8BIT, 0x22, 0xb0},
{GC0310_8BIT, 0x23, 0x04},
{GC0310_8BIT, 0x24, 0x16},
{GC0310_8BIT, 0x34, 0x20},
/////////////////////////////////////////////////
////////////////////   BLK	 ////////////////////
/////////////////////////////////////////////////
{GC0310_8BIT, 0x26, 0x23},
{GC0310_8BIT, 0x28, 0xff},
{GC0310_8BIT, 0x29, 0x00},
{GC0310_8BIT, 0x32, 0x09}, //Exp_rate_darkc
{GC0310_8BIT, 0x33, 0x18}, //offset ratio
{GC0310_8BIT, 0x37, 0x20}, //darkcurrent ratio
{GC0310_8BIT, 0x2a, 0x00},
{GC0310_8BIT, 0x2b, 0x00},
{GC0310_8BIT, 0x2c, 0x00},
{GC0310_8BIT, 0x2d, 0x00},
{GC0310_8BIT, 0x2e, 0x00},
{GC0310_8BIT, 0x2f, 0x00},
{GC0310_8BIT, 0x30, 0x00},
{GC0310_8BIT, 0x31, 0x00},
{GC0310_8BIT, 0x47, 0x80},
{GC0310_8BIT, 0x4e, 0x66},
{GC0310_8BIT, 0xa8, 0x02},
{GC0310_8BIT, 0xa9, 0x80},
/////////////////////////////////////////////////
//////////////////	 ISP reg  ///////////////////
/////////////////////////////////////////////////
{GC0310_8BIT, 0x40, 0x06},
{GC0310_8BIT, 0x41, 0x00},
{GC0310_8BIT, 0x42, 0x04},
{GC0310_8BIT, 0x44, 0x18},
{GC0310_8BIT, 0x46, 0x02},
{GC0310_8BIT, 0x49, 0x03},
{GC0310_8BIT, 0x4c, 0x20},
{GC0310_8BIT, 0x50, 0x01},
{GC0310_8BIT, 0x51, 0x00},
{GC0310_8BIT, 0x52, 0x00},
{GC0310_8BIT, 0x53, 0x00},
{GC0310_8BIT, 0x54, 0x01},
{GC0310_8BIT, 0x55, 0x01},
{GC0310_8BIT, 0x56, 0xf0},
{GC0310_8BIT, 0x57, 0x02},
{GC0310_8BIT, 0x58, 0x90},
/////////////////////////////////////////////////
///////////////////   GAIN	 ////////////////////
/////////////////////////////////////////////////
{GC0310_8BIT, 0x70, 0x50},// global gain
{GC0310_8BIT, 0x71, 0x20},// pregain gain
{GC0310_8BIT, 0x72, 0x40},// post gain
{GC0310_8BIT, 0x5a, 0x84},
{GC0310_8BIT, 0x5b, 0xc9},
{GC0310_8BIT, 0x5c, 0xed},
{GC0310_8BIT, 0x77, 0x40},// R gain
{GC0310_8BIT, 0x78, 0x40},// G gain
{GC0310_8BIT, 0x79, 0x40},// B gain
{GC0310_8BIT, 0x48, 0x00},
{GC0310_8BIT, 0xfe, 0x01},
{GC0310_8BIT, 0x0a, 0x45}, //[7]col gain mode
{GC0310_8BIT, 0x3e, 0x40},
{GC0310_8BIT, 0x3f, 0x5c},
{GC0310_8BIT, 0x40, 0x7b},
{GC0310_8BIT, 0x41, 0xbd},
{GC0310_8BIT, 0x42, 0xf6},
{GC0310_8BIT, 0x43, 0x63},
{GC0310_8BIT, 0x03, 0x60},
{GC0310_8BIT, 0x44, 0x03},

////////////////////////////////////////////
/////////////////dark sun//////////////////
///////////////////////////////////////////
{GC0310_8BIT, 0xfe, 0x01},
{GC0310_8BIT, 0x45, 0xa4},
{GC0310_8BIT, 0x46, 0xf0},//sun vaule th
{GC0310_8BIT, 0x48, 0x03},//sun mode
{GC0310_8BIT, 0x4f, 0x60},//sun_clamp
{GC0310_8BIT, 0xfe, 0x00},

{SENSOR_TABLE_END, 0, 0}
};

static struct gc0310_reg_cmd const gc0310_antiflicker_50hz[] = {
	{SENSOR_TABLE_END, 0, 0}
};

static struct gc0310_reg_cmd const gc0310_antiflicker_60hz[] = {
	{SENSOR_TABLE_END, 0, 0}
};

static struct gc0310_reg_cmd const gc0310_iq[] = {
	{SENSOR_TABLE_END, 0, 0}
};

#endif
