/*
 * Support for gc0339 Camera Sensor.
 *
 * Copyright (c) 2013 ASUSTeK COMPUTER INC. All Rights Reserved.
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

#ifndef __A1040_H__
#define __A1040_H__

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

#define V4L2_IDENT_GC0339 8245

#define MT9P111_REV3
#define FULLINISUPPORT

/* #defines for register writes and register array processing */
#define MISENSOR_8BIT		1
#define MISENSOR_16BIT		2
#define MISENSOR_32BIT		4

#define MISENSOR_FWBURST0	0x80
#define MISENSOR_FWBURST1	0x81
#define MISENSOR_FWBURST4	0x84
#define MISENSOR_FWBURST	0x88

#define MISENSOR_TOK_TERM	0xf000	/* terminating token for reg list */
#define MISENSOR_TOK_DELAY	0xfe00	/* delay token for reg list */
#define MISENSOR_TOK_FWLOAD	0xfd00	/* token indicating load FW */
#define MISENSOR_TOK_POLL	0xfc00	/* token indicating poll instruction */
#define MISENSOR_TOK_RMW	0x0010  /* RMW operation */
#define MISENSOR_TOK_MASK	0xfff0
#define MISENSOR_AWB_STEADY	(1<<0)	/* awb steady */
#define MISENSOR_AE_READY	(1<<3)	/* ae status ready */

/* mask to set sensor read_mode via misensor_rmw_reg */
#define MISENSOR_R_MODE_MASK	0x0330
/* mask to set sensor vert_flip and horz_mirror */
#define MISENSOR_VFLIP_MASK	0x0002
#define MISENSOR_HFLIP_MASK	0x0001
#define MISENSOR_FLIP_EN	1
#define MISENSOR_FLIP_DIS	0

/* bits set to set sensor read_mode via misensor_rmw_reg */
#define MISENSOR_SKIPPING_SET	0x0011
#define MISENSOR_SUMMING_SET	0x0033
#define MISENSOR_NORMAL_SET	0x0000

/* sensor register that control sensor read-mode and mirror */
#define MISENSOR_READ_MODE	0xC834
/* sensor ae-track status register */
#define MISENSOR_AE_TRACK_STATUS	0xA800
/* sensor awb status register */
#define MISENSOR_AWB_STATUS	0xAC00
/* sensor coarse integration time register */
#define MISENSOR_COARSE_INTEGRATION_TIME_H 0x03
#define MISENSOR_COARSE_INTEGRATION_TIME_L 0x04

//registers
#define REG_SW_RESET                    0x301A
#define REG_SW_STREAM                   0xDC00
#define REG_SCCB_CTRL                   0x3100
#define REG_SC_CMMN_CHIP_ID             0x0000
#define REG_V_START                     0x06 
#define REG_H_START                     0x08
#define REG_WIDTH_H                      0x0B
#define REG_WIDTH_L                      0x0C
#define REG_HEIGHT_H                      0x09
#define REG_HEIGHT_L                      0x0A
#define REG_SH_DELAY                     0x12
#define REG_DUMMY_H                     0x0F
#define REG_H_DUMMY_L                     0x01
#define REG_V_DUMMY_L                     0x02
#define REG_PIXEL_CLK                   0xc808
#define REG_TIMING_VTS                  0xc812
#define REG_TIMING_HTS                  0xc814
#define REG_EXPO_COARSE                 0x03
#define REG_EXPO_FINE                   0x3014
#define REG_GAIN                        0x50
#define REG_ANALOGGAIN                  0x305F
#define REG_ADDR_ACESSS                 0x098E
#define REG_COMM_Register               0x0080

#define SENSOR_DETECTED		1
#define SENSOR_NOT_DETECTED	0

#define I2C_RETRY_COUNT		5
#define MSG_LEN_OFFSET		1 /*8-bits addr*/

#ifndef MIPI_CONTROL
#define MIPI_CONTROL		0x3400	/* MIPI_Control */
#endif

//Consistent value
#define VALUE_V_START                     0
#define VALUE_H_START                     0
#define VALUE_V_END                       480
#define VALUE_H_END                       640
#define VALUE_V_OUTPUT                       480
#define VALUE_H_OUTPUT                       640
#define VALUE_PIXEL_CLK                   96

/* GPIO pin on Moorestown */
#define GPIO_SCLK_25		44
#define GPIO_STB_PIN		47

#define GPIO_STDBY_PIN		49   /* ab:new */
#define GPIO_RESET_PIN		50

/* System control register for Aptina A-1040SOC*/
#define GC0339_PID		0x0

/* MT9P111_DEVICE_ID */
#define GC0339_MOD_ID		0xc8

#define GC0339_FINE_INTG_TIME_MIN 0
#define GC0339_FINE_INTG_TIME_MAX_MARGIN 0
#define GC0339_COARSE_INTG_TIME_MIN 1
#define GC0339_COARSE_INTG_TIME_MAX_MARGIN 6

/* ulBPat; */

#define GC0339_BPAT_RGRGGBGB	(1 << 0)
#define GC0339_BPAT_GRGRBGBG	(1 << 1)
#define GC0339_BPAT_GBGBRGRG	(1 << 2)
#define GC0339_BPAT_BGBGGRGR	(1 << 3)

#define GC0339_FOCAL_LENGTH_NUM	169	/*1.69mm*/
#define GC0339_FOCAL_LENGTH_DEM	100
#define GC0339_F_NUMBER_DEFAULT_NUM	24
#define GC0339_F_NUMBER_DEM	10
#define GC0339_WAIT_STAT_TIMEOUT	100
#define GC0339_FLICKER_MODE_50HZ	1
#define GC0339_FLICKER_MODE_60HZ	2
/*
 * focal length bits definition:
 * bits 31-16: numerator, bits 15-0: denominator
 */
#define GC0339_FOCAL_LENGTH_DEFAULT 0xA90064

/*
 * current f-number bits definition:
 * bits 31-16: numerator, bits 15-0: denominator
 */
#define GC0339_F_NUMBER_DEFAULT 0x18000a

/*
 * f-number range bits definition:
 * bits 31-24: max f-number numerator
 * bits 23-16: max f-number denominator
 * bits 15-8: min f-number numerator
 * bits 7-0: min f-number denominator
 */
#define GC0339_F_NUMBER_RANGE 0x180a180a

/* Supported resolutions */
enum {
	GC0339_RES_CIF,
	GC0339_RES_VGA,
};

#define GC0339_RES_VGA_SIZE_H		640
#define GC0339_RES_VGA_SIZE_V		480
#define GC0339_RES_CIF_SIZE_H		 	352
#define GC0339_RES_CIF_SIZE_V			288
#define GC0339_RES_QVGA_SIZE_H		320
#define GC0339_RES_QVGA_SIZE_V		240
#define GC0339_RES_QCIF_SIZE_H		176
#define GC0339_RES_QCIF_SIZE_V		144

/* completion status polling requirements, usage based on Aptina .INI Rev2 */
enum poll_reg {
	NO_POLLING,
	PRE_POLLING,
	POST_POLLING,
};
/*
 * struct misensor_reg - MI sensor  register format
 * @length: length of the register
 * @reg: 16-bit offset to register
 * @val: 8/16/32-bit register value
 * Define a structure for sensor register initialization values
 */
struct misensor_reg {
	u32 length;
	u32 reg;
	u32 val;	/* value or for read/mod/write, AND mask */
	u32 val2;	/* optional; for rmw, OR mask */
};

/*
 * struct misensor_fwreg - Firmware burst command
 * @type: FW burst or 8/16 bit register
 * @addr: 16-bit offset to register or other values depending on type
 * @valx: data value for burst (or other commands)
 *
 * Define a structure for sensor register initialization values
 */
struct misensor_fwreg {
	u32	type;	/* type of value, register or FW burst string */
	u32	addr;	/* target address */
	u32	val0;
	u32	val1;
	u32	val2;
	u32	val3;
	u32	val4;
	u32	val5;
	u32	val6;
	u32	val7;
};

struct regval_list {
	u16 reg_num;
	u8 value;
};

struct gc0339_device {
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

struct gc0339_format_struct {
	u8 *desc;
	u32 pixelformat;
	struct regval_list *regs;
};

struct gc0339_res_struct {
	u8 *desc;
	int res;
	int width;
	int height;
	int fps;
	int skip_frames;
	bool used;
	struct regval_list *regs;
	u16 pixels_per_line;
	u16 lines_per_frame;
	u8 bin_factor_x;
	u8 bin_factor_y;
	u8 bin_mode;
};

struct gc0339_control {
	struct v4l2_queryctrl qc;
	int (*query)(struct v4l2_subdev *sd, s32 *value);
	int (*tweak)(struct v4l2_subdev *sd, int value);
};

/* 2 bytes used for address: 256 bytes total */
#define GC0339_MAX_WRITE_BUF_SIZE	254
struct gc0339_write_buffer {
	u16 addr;
	u8 data[GC0339_MAX_WRITE_BUF_SIZE];
};

struct gc0339_write_ctrl {
	int index;
	struct gc0339_write_buffer buffer;
};

/*
 * Modes supported by the gc0339 driver.
 * Please, keep them in ascending order.
 */
static struct gc0339_res_struct gc0339_res[] = {
	/*{
	.desc	= "CIF",
	.res	= GC0339_RES_CIF,
	.width	= 368,
	.height	= 304,
	.fps	= 30,
	.used	= 0,
	.regs	= NULL,
	.skip_frames = 1,

	.pixels_per_line = 0x01F0, // consistent with regs arrays
	.lines_per_frame = 0x014F, // consistent with regs arrays
	.bin_factor_x = 1,
	.bin_factor_y = 1,
	.bin_mode = 0,
	},*/
	{
	.desc	= "VGA",
	.res	= GC0339_RES_VGA,
	.width	= 656,
	.height	= 496,
	.fps	= 30,
	.used	= 0,
	.regs	= NULL,
	.skip_frames = 3,

	.pixels_per_line = 0x0314, // consistent with regs arrays
	.lines_per_frame = 0x0213, // consistent with regs arrays
	.bin_factor_x = 1,
	.bin_factor_y = 1,
	.bin_mode = 0,
	},
};
#define N_RES (ARRAY_SIZE(gc0339_res))

static const struct i2c_device_id gc0339_id[] = {
	{"gc0339", 0},
	{}
};

static struct misensor_reg const gc0339_suspend[] = {
	 {MISENSOR_TOK_TERM, 0, 0}
};

static struct misensor_reg const gc0339_streaming[] = {
	 {MISENSOR_TOK_TERM, 0, 0}
};

static struct misensor_reg const gc0339_standby_reg[] = {
	 {MISENSOR_TOK_TERM, 0, 0}
};

static struct misensor_reg const gc0339_wakeup_reg[] = {
	 {MISENSOR_TOK_TERM, 0, 0}
};

static struct misensor_reg const gc0339_chgstat_reg[] = {
	{MISENSOR_TOK_TERM, 0, 0}
};
static struct misensor_reg const gc0339_qcif_init[] = {

	{MISENSOR_TOK_TERM, 0, 0}
};

static struct misensor_reg const gc0339_qvga_init[] = {

	{MISENSOR_TOK_TERM, 0, 0}
};

static struct misensor_reg const gc0339_vga_init[] = {

	{MISENSOR_TOK_TERM, 0, 0}
};



static struct misensor_reg const gc0339_common[] = {

	{MISENSOR_TOK_TERM, 0, 0}
};

static struct misensor_reg const gc0339_antiflicker_50hz[] = {
	 {MISENSOR_TOK_TERM, 0, 0}
};

static struct misensor_reg const gc0339_antiflicker_60hz[] = {
	 {MISENSOR_TOK_TERM, 0, 0}
};

static struct misensor_reg const gc0339_iq[] = {
	 {MISENSOR_TOK_TERM, 0, 0}
};

#endif
