/*
 * Support for gc2155 Camera Sensor.
 *
 * Copyright (c) 2014 ASUSTeK COMPUTER INC. All Rights Reserved.
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

#define TWO_LANE 0

#define V4L2_IDENT_GC2155 8245

#define GC2155_AWB_STEADY	(1<<0)	/* awb steady */
#define GC2155_AE_READY	(1<<3)	/* ae status ready */

/* mask to set sensor vert_flip and horz_mirror */
#define GC2155_VFLIP_MASK	0x0002
#define GC2155_HFLIP_MASK	0x0001
#define GC2155_FLIP_EN		1
#define GC2155_FLIP_DIS		0

/* sensor register that control sensor read-mode and mirror */
#define GC2155_READ_MODE	0xC834
/* sensor ae-track status register */
#define GC2155_AE_TRACK_STATUS	0xA800
/* sensor awb status register */
#define GC2155_AWB_STATUS	0xAC00

/* gc2155 system registers */
#define REG_CHIP_ID_H		0xF0
#define REG_CHIP_ID_L		0xF1
#define REG_PLL_MODE1		0xF7
#define REG_PLL_MODE2		0xF8
#define REG_RST_AND_PG_SELECT	0xFE

/* gc2155 page0 registers */
#define REG_EXPO_COARSE_H	0x03
#define REG_EXPO_COARSE_L	0x04
#define REG_H_BLANK_H		0x05
#define REG_H_BLANK_L		0x06
#define REG_V_BLANK_H		0x07
#define REG_V_BLANK_L		0x08
#define REG_ROW_START_H		0x09
#define REG_ROW_START_L		0x0A
#define REG_COL_START_H		0x0B
#define REG_COL_START_L		0x0C
#define REG_WIN_HEIGHT_H	0x0D
#define REG_WIN_HEIGHT_L	0x0E
#define REG_WIN_WIDTH_H		0x0F
#define REG_WIN_WIDTH_L		0x10
#define REG_SH_DELAY_H		0x11
#define REG_SH_DELAY_L		0x12

#define REG_DIGITAL_PREGAIN		0xB1
#define REG_DIGITAL_POSTGAIN		0xB2

#define I2C_RETRY_COUNT		5
#define MSG_LEN_OFFSET		1 /*8-bits addr*/



/* GC2155_CHIP_ID VALUE*/
#define GC2155_MOD_ID		0x2155

#define GC2155_FINE_INTG_TIME_MIN 0
#define GC2155_FINE_INTG_TIME_MAX_MARGIN 0
#define GC2155_COARSE_INTG_TIME_MIN 1
#define GC2155_COARSE_INTG_TIME_MAX_MARGIN 6


#define GC2155_BPAT_RGRGGBGB	(1 << 0)
#define GC2155_BPAT_GRGRBGBG	(1 << 1)
#define GC2155_BPAT_GBGBRGRG	(1 << 2)
#define GC2155_BPAT_BGBGGRGR	(1 << 3)

#define GC2155_FOCAL_LENGTH_NUM	282	/* 2.82 mm */
#define GC2155_FOCAL_LENGTH_DEM	100
#define GC2155_F_NUMBER_DEFAULT_NUM	28
#define GC2155_F_NUMBER_DEM	10
#define GC2155_FLICKER_MODE_50HZ	1
#define GC2155_FLICKER_MODE_60HZ	2
/*
 * focal length bits definition:
 * bits 31-16: numerator, bits 15-0: denominator
 */
#define GC2155_FOCAL_LENGTH_DEFAULT 0xD00064

/*
 * current f-number bits definition:
 * bits 31-16: numerator, bits 15-0: denominator
 */
#define GC2155_F_NUMBER_DEFAULT 0x18000a

/*
 * f-number range bits definition:
 * bits 31-24: max f-number numerator
 * bits 23-16: max f-number denominator
 * bits 15-8: min f-number numerator
 * bits 7-0: min f-number denominator
 */
#define GC2155_F_NUMBER_RANGE 0x180a180a

/* Supported resolutions */
enum {
	GC2155_RES_CIF,
	GC2155_RES_VGA,
	GC2155_RES_2M,
};

#define GC2155_RES_2M_SIZE_H		1600
#define GC2155_RES_2M_SIZE_V		1200
#define GC2155_RES_VGA_SIZE_H		640
#define GC2155_RES_VGA_SIZE_V		480
#define GC2155_RES_CIF_SIZE_H		352
#define GC2155_RES_CIF_SIZE_V		288
#define GC2155_RES_QVGA_SIZE_H		320
#define GC2155_RES_QVGA_SIZE_V		240
#define GC2155_RES_QCIF_SIZE_H		176
#define GC2155_RES_QCIF_SIZE_V		144

#define RATIO_SHIFT_BITS		13

/* #defines for register writes and register array processing */
#define SENSOR_WAIT_MS		0
#define GC2155_8BIT		1
#define GC2155_16BIT		2
#define GC2155_32BIT		4
#define SENSOR_TABLE_END	0xFF

struct gc2155_reg_cmd {
	u16 cmd;
	u16 addr;
	u16 val;
};


struct gc2155_device {
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

       struct attribute_group sensor_i2c_attribute; //Add for ATD read camera status+++
};

struct gc2155_res_struct {
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

struct gc2155_control {
	struct v4l2_queryctrl qc;
	int (*query)(struct v4l2_subdev *sd, s32 *value);
	int (*tweak)(struct v4l2_subdev *sd, int value);
};


/*
 * Modes supported by the gc2155 driver.
 * Please, keep them in ascending order.
 */
static struct gc2155_res_struct gc2155_res[] = {
	{
	.desc	= "gc2155_2M_21fps",
	.res	= GC2155_RES_2M,
	.width	= 1616,
	.height	= 1216,
	.fps	= 21,
	.used	= 0,
	.skip_frames = 1,

	/* Fake values. Re-calculate this value in gc2155.c */
	.pixels_per_line = 1632,
	.lines_per_frame = 1248,

	.bin_factor_x = 1,
	.bin_factor_y = 1,
	.bin_mode = 0,
	},

};
#define N_RES (ARRAY_SIZE(gc2155_res))

static const struct i2c_device_id gc2155_id[] = {
	{"gc2155", 0},
	{}
};

static struct gc2155_reg_cmd const gc2155_suspend[] = {

    {SENSOR_TABLE_END, 0x0, 0x0}
};

static struct gc2155_reg_cmd const gc2155_streaming[] = {

	{SENSOR_TABLE_END, 0x0, 0x0}
};

static struct gc2155_reg_cmd const gc2155_standby_reg[] = {

	{SENSOR_TABLE_END, 0x0, 0x0}
};

static struct gc2155_reg_cmd const gc2155_wakeup_reg[] = {

	{SENSOR_TABLE_END, 0x0, 0x0}
};

static struct gc2155_reg_cmd const gc2155_chgstat_reg[] = {

	{SENSOR_TABLE_END, 0x0, 0x0}
};
static struct gc2155_reg_cmd const gc2155_cif_init[] = {

	{SENSOR_TABLE_END, 0x0, 0x0}
};

static struct gc2155_reg_cmd const gc2155_qvga_init[] = {

	{SENSOR_TABLE_END, 0x0, 0x0}
};

static struct gc2155_reg_cmd const gc2155_vga_init[] = {

    {SENSOR_TABLE_END, 0x0, 0x0}
};


static struct gc2155_reg_cmd const gc2155_common[] = {
{GC2155_8BIT, 0xfe, 0xf0},
{GC2155_8BIT, 0xfe, 0xf0},
{GC2155_8BIT, 0xfe, 0xf0},
{GC2155_8BIT, 0xfc, 0x06},
{GC2155_8BIT, 0xf6, 0x00},
{GC2155_8BIT, 0xf7, 0x1d},
{GC2155_8BIT, 0xf8, 0x85},
{GC2155_8BIT, 0xfa, 0x00},
{GC2155_8BIT, 0xf9, 0x8e},
{GC2155_8BIT, 0xf2, 0x00},
{GC2155_8BIT, 0xfe, 0x00},
{GC2155_8BIT, 0x03, 0x04},
{GC2155_8BIT, 0x04, 0x00},
{GC2155_8BIT, 0x05, 0x01},
{GC2155_8BIT, 0x06, 0x18},
{GC2155_8BIT, 0x07, 0x00},
{GC2155_8BIT, 0x08, 0x20},
{GC2155_8BIT, 0x09, 0x00},
{GC2155_8BIT, 0x0a, 0x00},
{GC2155_8BIT, 0x0b, 0x00},
{GC2155_8BIT, 0x0c, 0x00},
{GC2155_8BIT, 0x0d, 0x04},
{GC2155_8BIT, 0x0e, 0xd0},
{GC2155_8BIT, 0x0f, 0x06},
{GC2155_8BIT, 0x10, 0x62},
{GC2155_8BIT, 0x12, 0x2e},
{GC2155_8BIT, 0x17, 0x14},
{GC2155_8BIT, 0x18, 0x0a},
{GC2155_8BIT, 0x19, 0x0b},
{GC2155_8BIT, 0x1a, 0x09},
{GC2155_8BIT, 0x1b, 0x4b},
{GC2155_8BIT, 0x1c, 0x07},
{GC2155_8BIT, 0x1d, 0x10},
{GC2155_8BIT, 0x1e, 0x98},
{GC2155_8BIT, 0x1f, 0x78},
{GC2155_8BIT, 0x20, 0x05},
{GC2155_8BIT, 0x21, 0x40},
{GC2155_8BIT, 0x22, 0xf0},
{GC2155_8BIT, 0x24, 0x16},
{GC2155_8BIT, 0x25, 0x01},
{GC2155_8BIT, 0x26, 0x10},
{GC2155_8BIT, 0x2d, 0x40},
{GC2155_8BIT, 0x30, 0x01},
{GC2155_8BIT, 0x31, 0x90},
{GC2155_8BIT, 0x33, 0x04},
{GC2155_8BIT, 0x34, 0x01},
{GC2155_8BIT, 0x5c, 0x07},
{GC2155_8BIT, 0x80, 0x06},
{GC2155_8BIT, 0x81, 0x80},
{GC2155_8BIT, 0x82, 0x30},
{GC2155_8BIT, 0x83, 0x00},
{GC2155_8BIT, 0x84, 0x17},
{GC2155_8BIT, 0x85, 0x08},
{GC2155_8BIT, 0x86, 0x36},
{GC2155_8BIT, 0x88, 0x03},
{GC2155_8BIT, 0x89, 0x03},
{GC2155_8BIT, 0x8a, 0x00},
{GC2155_8BIT, 0x8b, 0x00},
{GC2155_8BIT, 0xb0, 0x70},
{GC2155_8BIT, 0xb6, 0x00}, /* P0:0xb6: AEC enable */
{GC2155_8BIT, 0x90, 0x01},
{GC2155_8BIT, 0x91, 0x00},
{GC2155_8BIT, 0x92, 0x00},
{GC2155_8BIT, 0x93, 0x00},
{GC2155_8BIT, 0x94, 0x00},
{GC2155_8BIT, 0x95, 0x04},
{GC2155_8BIT, 0x96, 0xc0},
{GC2155_8BIT, 0x97, 0x06},
{GC2155_8BIT, 0x98, 0x50},
{GC2155_8BIT, 0x9a, 0x02},
{GC2155_8BIT, 0x18, 0x0a},
{GC2155_8BIT, 0x40, 0x43},
{GC2155_8BIT, 0x41, 0x28},
{GC2155_8BIT, 0x42, 0x60},
{GC2155_8BIT, 0x43, 0x54},
{GC2155_8BIT, 0x5e, 0x00},
{GC2155_8BIT, 0x5f, 0x00},
{GC2155_8BIT, 0x60, 0x00},
{GC2155_8BIT, 0x61, 0x00},
{GC2155_8BIT, 0x62, 0x00},
{GC2155_8BIT, 0x63, 0x00},
{GC2155_8BIT, 0x64, 0x00},
{GC2155_8BIT, 0x65, 0x00},
{GC2155_8BIT, 0x66, 0x20},
{GC2155_8BIT, 0x67, 0x20},
{GC2155_8BIT, 0x68, 0x20},
{GC2155_8BIT, 0x69, 0x20},
{GC2155_8BIT, 0x6a, 0x00},
{GC2155_8BIT, 0x6b, 0x00},
{GC2155_8BIT, 0x6c, 0x00},
{GC2155_8BIT, 0x6d, 0x00},
{GC2155_8BIT, 0x6e, 0x00},
{GC2155_8BIT, 0x6f, 0x00},
{GC2155_8BIT, 0x70, 0x00},
{GC2155_8BIT, 0x71, 0x00},
{GC2155_8BIT, 0x72, 0xf0},
{GC2155_8BIT, 0xfe, 0x02},
{GC2155_8BIT, 0x49, 0x00},
{GC2155_8BIT, 0x4b, 0x02},
{GC2155_8BIT, 0xfe, 0x00},
{GC2155_8BIT, 0xfe, 0x02},
{GC2155_8BIT, 0x82, 0x05},
{GC2155_8BIT, 0x83, 0x04},
{GC2155_8BIT, 0x86, 0x80},
{GC2155_8BIT, 0x89, 0x80},
{GC2155_8BIT, 0x8a, 0x60},
{GC2155_8BIT, 0x8b, 0x30},
{GC2155_8BIT, 0xfe, 0x01},
{GC2155_8BIT, 0x21, 0x14},
{GC2155_8BIT, 0xfe, 0x02},
{GC2155_8BIT, 0xd1, 0x20},
{GC2155_8BIT, 0xd2, 0x20},
{GC2155_8BIT, 0xdd, 0x80},
{GC2155_8BIT, 0xde, 0x84},
{GC2155_8BIT, 0xfe, 0x00},
{GC2155_8BIT, 0xf2, 0x0f},
{GC2155_8BIT, 0xfe, 0x00},
{GC2155_8BIT, 0xfe, 0x03},
{GC2155_8BIT, 0x01, 0x83},//discontin
{GC2155_8BIT, 0x02, 0x00},//[2:0] clk_diff_[6:4] lane0_diff
{GC2155_8BIT, 0x03, 0x10},
{GC2155_8BIT, 0x04, 0x20},
{GC2155_8BIT, 0x05, 0x00},
{GC2155_8BIT, 0x06, 0x88},
{GC2155_8BIT, 0x10, 0x00},
{GC2155_8BIT, 0x11, 0x2b},
{GC2155_8BIT, 0x12, 0xe4},
{GC2155_8BIT, 0x13, 0x07},
{GC2155_8BIT, 0x15, 0x11},//discontin
{GC2155_8BIT, 0x17, 0xf1},
{GC2155_8BIT, 0x22, 0x02},//clk prepare
{GC2155_8BIT, 0x26, 0x06},//08_clk teot
{GC2155_8BIT, 0x29, 0x02},//01_data prepare
{GC2155_8BIT, 0x2b, 0x06},//08_data teot
{GC2155_8BIT, 0xfe, 0x00},

{SENSOR_TABLE_END, 0x0, 0x0}
};

static struct gc2155_reg_cmd const gc2155_antiflicker_50hz[] = {
	 {SENSOR_TABLE_END, 0, 0}
};

static struct gc2155_reg_cmd const gc2155_antiflicker_60hz[] = {
	 {SENSOR_TABLE_END, 0, 0}
};

static struct gc2155_reg_cmd const gc2155_iq[] = {
	 {SENSOR_TABLE_END, 0, 0}
};

