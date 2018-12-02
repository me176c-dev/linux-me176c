/* 
* Support for gc0310 Camera Sensor.
 *
 * Copyright (c) 2012 Intel Corporation. All Rights Reserved.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License version
 * as published by the Free Software Foundation.
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

#include <linux/module.h>
#include <linux/types.h>
#include <linux/kernel.h>
#include <linux/mm.h>
#include <linux/string.h>
#include <linux/errno.h>
#include <linux/init.h>
#include <linux/kmod.h>
#include <linux/device.h>
#include <linux/fs.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/delay.h>
#include <linux/i2c.h>
#include <linux/gpio.h>
#include <media/v4l2-device.h>
#include <media/v4l2-chip-ident.h>

#include <linux/debugfs.h>

#include "gc0310.h"

#define to_gc0310_sensor(sd) container_of(sd, struct gc0310_device, sd)

#define GC0310_ANALOG_GAIN_1 64  // 1.00x
#define GC0310_ANALOG_GAIN_2 92  // 1.40x
#define GC0310_ANALOG_GAIN_3 123  // 2.00x
#define GC0310_ANALOG_GAIN_4 190  // 2.80x
#define GC0310_ANALOG_GAIN_5 246  // 4.00x
#define GC0310_ANALOG_GAIN_6 355  // 5.60x
#define GC0310_ANALOG_GAIN_7 482  // 8.00x

/*
 * TODO: use debug parameter to actually define when debug messages should
 * be printed.
 */
static int debug;
module_param(debug, int, 0644);
MODULE_PARM_DESC(debug, "Debug level (0-1)");

/* ASUS dbgfs */
static u8 atd_node = 1;
static unsigned int WhoAmI = 310;
static struct i2c_client *g_client;
static u16 g_by_file = 0x0;

static int
gc0310_read_reg(struct i2c_client *client, u16 data_length, u8 reg, u8 *val)
{
	int err;
	struct i2c_msg msg[2];
	unsigned char data[2];
	if (!client->adapter) {
		v4l2_err(client, "%s error, no client->adapter\n", __func__);
		return -ENODEV;
	}

	if (data_length != GC0310_8BIT && data_length != GC0310_16BIT
					 && data_length != GC0310_32BIT) {
		v4l2_err(client, "%s error, invalid data length\n", __func__);
		return -EINVAL;
	}

	msg[0].addr = client->addr;
	msg[0].flags = 0;
	msg[0].len = MSG_LEN_OFFSET;
	msg[0].buf = data;

	/* high byte goes out first */
	data[0] = (u8) (reg & 0xff);

	msg[1].addr = client->addr;
	msg[1].len = data_length;
	msg[1].flags = I2C_M_RD;
	msg[1].buf = data+1;

	err = i2c_transfer(client->adapter, msg, 2);

	if (err >= 0) {
		*val = 0;
		/* high byte comes first */
		if (data_length == GC0310_8BIT)
			*val = data[1];

		return 0;
	}

	dev_err(&client->dev, "read from offset 0x%x error %d", reg, err);
	return err;
}

static int
gc0310_write_reg(struct i2c_client *client, u16 data_length, u8 reg, u8 val)
{
	int num_msg;
	struct i2c_msg msg;
	unsigned char data[4] = {0};
	int retry = 0;

	if (!client->adapter) {
		v4l2_err(client, "%s error, no client->adapter\n", __func__);
		return -ENODEV;
	}

	if (data_length != GC0310_8BIT && data_length != GC0310_16BIT
					 && data_length != GC0310_32BIT) {
		v4l2_err(client, "%s error, invalid data_length\n", __func__);
		return -EINVAL;
	}

	memset(&msg, 0, sizeof(msg));

again:

	data[0] = (u8) reg;
	data[1] = (u8) (val & 0xff);

	msg.addr = client->addr;
	msg.flags = 0;
	msg.len = 1 + data_length;
	msg.buf = data;

	if (data_length == GC0310_8BIT)
		data[2] = (u8)(val);

	num_msg = i2c_transfer(client->adapter, &msg, 1);

	/*
	* HACK: Need some delay here for Rev 2 sensors otherwise some
	* registers do not seem to load correctly.
	*/
	mdelay(1);

	if (num_msg >= 0)
		return 0;

	dev_err(&client->dev, "write error: wrote 0x%x to offset 0x%x error %d",
		val, reg, num_msg);
	if (retry <= I2C_RETRY_COUNT) {
		dev_dbg(&client->dev, "retrying... %d", retry);
		retry++;
		msleep(20);
		goto again;
	}

	return num_msg;
}

/*
 * gc0310_write_reg_array - Initializes a list of gc0310 registers
 * @client: i2c driver client structure
 * @reglist: list of registers to be written
 */
static int gc0310_write_reg_array(struct i2c_client *client,
				const struct gc0310_reg_cmd reglist[])
{
	int err;
	const struct gc0310_reg_cmd *next;
	u16 val;

	for (next = reglist; next->cmd != SENSOR_TABLE_END; next++) {

		if (next->cmd == SENSOR_WAIT_MS) {
			msleep(next->val);
			continue;
		}

		val = next->val;

		if (next->cmd == GC0310_8BIT)
			err = gc0310_write_reg(client, GC0310_8BIT, next->addr, val);
		else
			pr_info("%s: Unknown cmd: %d\n", __func__, next->cmd);

		if (err) {
			pr_info("%s: err=%d\n", __func__, err);
			return err;
		}
	}
	return 0;
}

static int gc0310_set_suspend(struct v4l2_subdev *sd)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);

	gc0310_write_reg(client, GC0310_8BIT, GC0310_REG_PAGE_SELECT, 0x03);

	gc0310_write_reg(client, GC0310_8BIT, 0x16, 0x05);

	msleep(200);

	gc0310_write_reg(client, GC0310_8BIT, 0x10, 0x84); /* 8bit raw disable*/

	gc0310_write_reg(client, GC0310_8BIT, 0x01, 0x00);

	gc0310_write_reg(client, GC0310_8BIT, GC0310_REG_PAGE_SELECT, 0x00);

	msleep(50);

	return 0;
}

static int gc0310_set_streaming(struct v4l2_subdev *sd)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);

	gc0310_write_reg(client, GC0310_8BIT, 0xfe, 0x30);

	gc0310_write_reg(client, GC0310_8BIT, GC0310_REG_PAGE_SELECT, 0x03);

	gc0310_write_reg(client, GC0310_8BIT, 0x16, 0x09);

	gc0310_write_reg(client, GC0310_8BIT, 0x10, 0x94); /* 8bit raw enable*/

	gc0310_write_reg(client, GC0310_8BIT, 0x01, 0x03);

	gc0310_write_reg(client, GC0310_8BIT, GC0310_REG_PAGE_SELECT, 0x00);

	return 0;
}

static int gc0310_init_common(struct v4l2_subdev *sd)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	int ret = 0;

	pr_info("%s()++\n",__func__);

	ret = gc0310_write_reg_array(client, gc0310_common);

	if (ret)
		pr_info("%s: write seq fail\n", __func__);

	pr_info("%s()--\n",__func__);
	return ret;
}

static u8 char2u8(u8 *pC)
{
	u8 ret_val = 0;

	u8 h4b, l4b;

	if ((pC[0] >= 'a') && (pC[0] <= 'f'))
		h4b = pC[0] - 'a' + 10;
	else if ((pC[0] >= 'A') && (pC[0] <= 'F'))
		h4b = pC[0] - 'A' + 10;
	else if ((pC[0] >= '0') && (pC[0] <= '9'))
		h4b = pC[0] - '0';
	else {
		pr_info("%s: convert h4b error\n", __func__);
		return 0xFF;
	}

	if ((pC[1] >= 'a') && (pC[1] <= 'f'))
		l4b = pC[1] - 'a' + 10;
	else if ((pC[1] >= 'A') && (pC[1] <= 'F'))
		l4b = pC[1] - 'A' + 10;
	else if ((pC[1] >= '0') && (pC[1] <= '9'))
		l4b = pC[1] - '0';
	else {
		pr_info("%s: convert l4b error\n", __func__);
		return 0xFF;
	}

	ret_val = (h4b << 4) | l4b;

	return ret_val;
}

static int gc0310_init_common_by_file(struct v4l2_subdev *sd)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	int ret = 0;
	struct file *fp = NULL;
	struct inode *inode;
	mm_segment_t old_fs;
	int bootbin_size = 0;
	u8 *pReg;
	u8 *pVal;
	u8 *pTmp;

	u8 Reg, Val;
	pr_info("%s()++\n",__func__);

	fp = filp_open("/data/data/gc0310.txt", O_RDONLY, 0);

	if ( !IS_ERR_OR_NULL(fp) ) {
		pr_info("filp_open success fp:%p\n", fp);
		inode = fp->f_dentry->d_inode;
		bootbin_size = inode->i_size;
//		printk("%s: fp->f_dentry->d_inode->i_size=%d\n", __FUNCTION__, bootbin_size);
		pReg = kmalloc(2, GFP_KERNEL);
		pVal = kmalloc(2, GFP_KERNEL);
		pTmp = kmalloc(10, GFP_KERNEL);
		old_fs = get_fs();
		set_fs(KERNEL_DS);
		if(fp->f_op != NULL && fp->f_op->read != NULL){
			int byte_count= 0;
			int i = 0;
			for (i = 0; i < (bootbin_size/7) + 1; i++) {
				byte_count += fp->f_op->read(fp, pReg, 2, &fp->f_pos);
				byte_count += fp->f_op->read(fp, pTmp, 1, &fp->f_pos);
				byte_count += fp->f_op->read(fp, pVal, 2, &fp->f_pos);
				byte_count += fp->f_op->read(fp, pTmp, 2, &fp->f_pos);

				Reg = char2u8(pReg);
				Val = char2u8(pVal);
				pr_info("%s(%d): 0x%02x 0x%02x\n", __func__, __LINE__, Reg, Val);
				ret |= gc0310_write_reg(client, GC0310_8BIT, Reg,Val);
			}

			if (byte_count <= 0) {
				pr_info("%s: EOF or error. last byte_count= %d;\n", __func__, byte_count);
			} else
				pr_info("%s: init file size= %d bytes\n", __func__, bootbin_size);
		}
		set_fs(old_fs);
		filp_close(fp, NULL);
		kfree(pReg);
		kfree(pVal);
		kfree(pTmp);
	} else if(PTR_ERR(fp) == -ENOENT) {
		pr_err("%s: /data/data/gc0310.txt not found error\n", __func__);
		ret = -EINVAL;
	} else {
		pr_err("%s: /data/data/gc0310.txt open error\n", __func__);
		ret = -EINVAL;
	}

	if (ret)
		pr_info("%s: write seq fail\n", __func__);

	pr_info("%s()--\n",__func__);
	return ret;
}

static int power_up(struct v4l2_subdev *sd)
{
	struct gc0310_device *dev = to_gc0310_sensor(sd);
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	int ret;

	if (NULL == dev->platform_data) {
		dev_err(&client->dev, "no camera_sensor_platform_data");
		return -ENODEV;
	}

	/* power control */
	ret = dev->platform_data->power_ctrl(sd, 1);
	if (ret)
		goto fail_power;


	usleep_range(5000, 6000);

	/*
	 * according to DS, 44ms is needed between power up and first i2c
	 * commend
	 */
	msleep(50);

	return 0;

fail_power:
	dev->platform_data->power_ctrl(sd, 0);
	dev_err(&client->dev, "sensor power-up failed\n");

	return ret;
}

static int power_down(struct v4l2_subdev *sd)
{
	struct gc0310_device *dev = to_gc0310_sensor(sd);
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	int ret;


	if (NULL == dev->platform_data) {
		dev_err(&client->dev, "no camera_sensor_platform_data");
		return -ENODEV;
	}

	/* power control */
	ret = dev->platform_data->power_ctrl(sd, 0);
	if (ret)
		dev_err(&client->dev, "vprog failed.\n");

	/*according to DS, 20ms is needed after power down*/
	msleep(20);

	return ret;
}

static int gc0310_s_power(struct v4l2_subdev *sd, int power)
{
	pr_info("%s(%d) %s\n", __func__, __LINE__, power ? ("on") : ("off"));
	if (power == 0)
		return power_down(sd);
	else {
		if (power_up(sd))
			return -EINVAL;

		if(g_by_file)
			return gc0310_init_common_by_file(sd);
		else
			return gc0310_init_common(sd);
	}
}

static int gc0310_try_res(u32 *w, u32 *h)
{
	int i;

	/*
	* The mode list is in ascending order. We're done as soon as
	* we have found the first equal or bigger size.
	*/
	for (i = 0; i < N_RES; i++) {
		if ((gc0310_res[i].width >= *w) &&
                (gc0310_res[i].height >= *h))
			break;
	}

	/*
	* If no mode was found, it means we can provide only a smaller size.
	* Returning the biggest one available in this case.
	*/
	if (i == N_RES)
		i--;

	*w = gc0310_res[i].width;
	*h = gc0310_res[i].height;

	return 0;
}

static struct gc0310_res_struct *gc0310_to_res(u32 w, u32 h)
{
	int  index;

	for (index = 0; index < N_RES; index++) {
		if ((gc0310_res[index].width == w) &&
                (gc0310_res[index].height == h))
			break;
	}

	/* No mode found */
	if (index >= N_RES)
		return NULL;

	return &gc0310_res[index];
}

static int gc0310_try_mbus_fmt(struct v4l2_subdev *sd,
				struct v4l2_mbus_framefmt *fmt)
{
	fmt->code = V4L2_MBUS_FMT_SGRBG8_1X8;
	return gc0310_try_res(&fmt->width, &fmt->height);
}

static int gc0310_res2size(unsigned int res, int *h_size, int *v_size)
{
	unsigned short hsize;
	unsigned short vsize;

	switch (res) {
	case GC0310_RES_CIF:
		hsize = GC0310_RES_CIF_SIZE_H;
		vsize = GC0310_RES_CIF_SIZE_V;
		break;
	case GC0310_RES_VGA:
		hsize = GC0310_RES_VGA_SIZE_H;
		vsize = GC0310_RES_VGA_SIZE_V;
		break;
	default:
		WARN(1, "%s: Resolution 0x%08x unknown\n", __func__, res);
		return -EINVAL;
	}

	if (h_size != NULL)
		*h_size = hsize;
	if (v_size != NULL)
		*v_size = vsize;

	return 0;
}

static int gc0310_get_intg_factor(struct i2c_client *client,
				struct camera_mipi_info *info,
				const struct gc0310_res_struct *res)
{
	struct atomisp_sensor_mode_data *buf = &info->data;
	const unsigned int ext_clk_freq_hz = 19200000;
	u8 reg_val, reg_val2, div2en, div;
	u16 val;
	int ret;
	dev_dbg(&client->dev, "%s\n", __func__);

	if (info == NULL)
		return -EINVAL;

	ret = gc0310_read_reg(client, GC0310_8BIT,  GC0310_REG_PLL_MODE1, &div2en);
	div2en = div2en & 0x2;
	ret = gc0310_read_reg(client, GC0310_8BIT,  GC0310_REG_PLL_MODE2, &div);
	div = div & 0x3F;

	if (div2en)
		buf->vt_pix_clk_freq_mhz = (ext_clk_freq_hz >> 2) * (div + 1);
	else
		buf->vt_pix_clk_freq_mhz = (ext_clk_freq_hz >> 1) * (div + 1);

	/* get integration time */
	buf->coarse_integration_time_min = GC0310_COARSE_INTG_TIME_MIN;
	buf->coarse_integration_time_max_margin =
					GC0310_COARSE_INTG_TIME_MAX_MARGIN;

	buf->fine_integration_time_min = GC0310_FINE_INTG_TIME_MIN;
	buf->fine_integration_time_max_margin =
					GC0310_FINE_INTG_TIME_MAX_MARGIN;

	buf->fine_integration_time_def = GC0310_FINE_INTG_TIME_MIN;

	/* get the cropping and output resolution to ISP for this mode. */
	ret = gc0310_read_reg(client, GC0310_8BIT,  GC0310_REG_COL_START_H, &reg_val);
	ret |= gc0310_read_reg(client, GC0310_8BIT,  GC0310_REG_COL_START_L, &reg_val2);
	if (ret) {
		pr_info("%s: Read COL_START Fail\n", __func__);
		return ret;
	}
	buf->crop_horizontal_start = ((reg_val << 8) & 0x300) | reg_val2;

	ret = gc0310_read_reg(client, GC0310_8BIT,  GC0310_REG_ROW_START_H, &reg_val);
	ret |= gc0310_read_reg(client, GC0310_8BIT,  GC0310_REG_ROW_START_L, &reg_val2);
	if (ret) {
		pr_info("%s: Read ROW_START Fail\n", __func__);
		return ret;
	}
	buf->crop_vertical_start = ((reg_val << 8) & 0x100) | reg_val2;

	val = 0;
	ret = gc0310_read_reg(client, GC0310_8BIT,  GC0310_REG_WIDTH_H, &reg_val);
	val = reg_val;
	val = (val << 8) & 0x300;
	ret |= gc0310_read_reg(client, GC0310_8BIT,  GC0310_REG_WIDTH_L, &reg_val);
	val += reg_val;
	if (ret) {
		pr_info("%s: Read WIN_WIDTH Fail\n", __func__);
		return ret;
	}
	buf->output_width = val;
	buf->crop_horizontal_end = buf->crop_horizontal_start + val - 1;

	val = 0;
	ret = gc0310_read_reg(client, GC0310_8BIT,  GC0310_REG_HEIGHT_H, &reg_val);
	val = reg_val;
	val = (val << 8) & 0x100;
	ret |= gc0310_read_reg(client, GC0310_8BIT,  GC0310_REG_HEIGHT_L, &reg_val);
	val += reg_val;
	if (ret) {
		pr_info("%s: Read WIN_HEIGHT Fail\n", __func__);
		return ret;
	}
	buf->output_height = val;
	buf->crop_vertical_end = buf->crop_vertical_start + val - 1;

	val = 0;
	ret = gc0310_read_reg(client, GC0310_8BIT,  GC0310_REG_HB_H, &reg_val);
	val = reg_val;
	val = (val << 8) & 0xF00;
	ret = gc0310_read_reg(client, GC0310_8BIT,  GC0310_REG_HB_L, &reg_val);
	val += reg_val;

	ret = gc0310_read_reg(client, GC0310_8BIT,  GC0310_REG_SH_DELAY, &reg_val);
	if (ret)
		return ret;
	val += reg_val;
	buf->line_length_pck = buf->output_width + val + 4;

	val = 0;
	ret = gc0310_read_reg(client, GC0310_8BIT,  GC0310_REG_VB_H, &reg_val);
	val = reg_val;
	val = (val << 8) & 0xF00;
	ret = gc0310_read_reg(client, GC0310_8BIT,  GC0310_REG_VB_L, &reg_val);
	val += reg_val;

	buf->frame_length_lines = buf->output_height + val;

	buf->read_mode = res->bin_mode;
	buf->binning_factor_x = 1;
	buf->binning_factor_y = 1;
	return 0;
}

static int gc0310_get_mbus_fmt(struct v4l2_subdev *sd,
				struct v4l2_mbus_framefmt *fmt)
{
	struct gc0310_device *dev = to_gc0310_sensor(sd);
	int width, height;
	int ret;

	fmt->code = V4L2_MBUS_FMT_SGRBG8_1X8;

	ret = gc0310_res2size(dev->res, &width, &height);
	if (ret)
		return ret;
	fmt->width = width;
	fmt->height = height;

	return 0;
}

static int gc0310_set_mbus_fmt(struct v4l2_subdev *sd,
				struct v4l2_mbus_framefmt *fmt)
{
	struct i2c_client *c = v4l2_get_subdevdata(sd);
	struct gc0310_device *dev = to_gc0310_sensor(sd);
	struct gc0310_res_struct *res_index;
	u32 width = fmt->width;
	u32 height = fmt->height;
	struct camera_mipi_info *gc0310_info = NULL;
	int ret;
	pr_info("%s()++\n",__func__);

	gc0310_info = v4l2_get_subdev_hostdata(sd);
	pr_info("gc0310_info %d %d %d %d\n",
		gc0310_info->port, gc0310_info->input_format,
		gc0310_info->num_lanes, gc0310_info->raw_bayer_order);

	if (gc0310_info == NULL)
		return -EINVAL;

	gc0310_try_res(&width, &height);
	res_index = gc0310_to_res(width, height);

	/* Sanity check */
	if (unlikely(!res_index)) {
		WARN_ON(1);
		return -EINVAL;
	}

	switch (res_index->res) {
	case GC0310_RES_CIF:
		pr_info("gc0310_set_mbus_fmt: CIF\n");

		break;
	case GC0310_RES_VGA:
		pr_info("gc0310_set_mbus_fmt: VGA\n");

		break;
	default:
		v4l2_err(sd, "set resolution: %d failed!\n", res_index->res);
		return -EINVAL;
	}

	ret = gc0310_get_intg_factor(c, gc0310_info,
					&gc0310_res[res_index->res]);
	if (ret) {
		dev_err(&c->dev, "failed to get integration_factor\n");
		return -EINVAL;
	}

	/*
	* gc0310 - we don't poll for context switch
	* because it does not happen with streaming disabled.
	*/
	dev->res = res_index->res;

	fmt->width = width;
	fmt->height = height;
	fmt->code = V4L2_MBUS_FMT_SGRBG8_1X8;
	return 0;
}

/* TODO: Update to SOC functions, remove exposure and gain */
static int gc0310_g_focal(struct v4l2_subdev *sd, s32 *val)
{
	*val = (GC0310_FOCAL_LENGTH_NUM << 16) | GC0310_FOCAL_LENGTH_DEM;
	return 0;
}

static int gc0310_g_fnumber(struct v4l2_subdev *sd, s32 *val)
{
	/*const f number for gc0310*/
	*val = (GC0310_F_NUMBER_DEFAULT_NUM << 16) | GC0310_F_NUMBER_DEM;
	return 0;
}

static int gc0310_g_fnumber_range(struct v4l2_subdev *sd, s32 *val)
{
	*val = (GC0310_F_NUMBER_DEFAULT_NUM << 24) |
		(GC0310_F_NUMBER_DEM << 16) |
		(GC0310_F_NUMBER_DEFAULT_NUM << 8) | GC0310_F_NUMBER_DEM;
	return 0;
}

static int gc0310_s_freq(struct v4l2_subdev *sd, s32 val)
{
	struct i2c_client *c = v4l2_get_subdevdata(sd);
	struct gc0310_device *dev = to_gc0310_sensor(sd);
	int ret;

	if (val != GC0310_FLICKER_MODE_50HZ &&
			val != GC0310_FLICKER_MODE_60HZ)
		return -EINVAL;

	if (val == GC0310_FLICKER_MODE_50HZ) {
		ret = gc0310_write_reg_array(c, gc0310_antiflicker_50hz);
		if (ret < 0)
			return ret;
	} else {
		ret = gc0310_write_reg_array(c, gc0310_antiflicker_60hz);
		if (ret < 0)
			return ret;
	}

	if (ret == 0)
		dev->lightfreq = val;

	return ret;
}

static long gc0310_s_exposure(struct v4l2_subdev *sd,
        struct atomisp_exposure *exposure)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	int ret = 0;
	unsigned int coarse_integration = 0;
	unsigned int AnalogGain, DigitalGain;

	u8 expo_coarse_h, expo_coarse_l;
	//Short terms workarond, need to be fixed
#ifdef ASUS_PROJECT_ME176C_L
	return 0;
#endif
	pr_info("%s(0x%X 0x%X 0x%X)\n", __func__,
			exposure->integration_time[0], exposure->gain[0], exposure->gain[1]);

	coarse_integration = exposure->integration_time[0];
	AnalogGain = exposure->gain[0];
	DigitalGain = exposure->gain[1];

	/* P0:0x48 [2:0] Analog gain
	 *  0: 1.4X
	 *  1: 2X
	 *  2: 2.8X
	 *  3: 4X
	 *  4: 5.6X
	 *  5: 8X
	 */
	gc0310_write_reg(client, GC0310_8BIT, GC0310_REG_A_GAIN, AnalogGain);

	/* P0:0x71 [7:0] Digital gain(pregain)
	 * [7]: 4X
	 * [6]: 2X
	 * ...
	 * [1]: 1/16 X
	 * [0]: 1/32 X
	 */
	gc0310_write_reg(client, GC0310_8BIT, GC0310_REG_D_GAIN, DigitalGain);

	if(!coarse_integration) coarse_integration = 1; /* avoid 0 */
	if(coarse_integration < 1) coarse_integration = 1;
	if(coarse_integration > 4095) coarse_integration = 4095;// 2^13

	expo_coarse_h = (u8)(coarse_integration >> 8);
	expo_coarse_l = (u8)(coarse_integration & 0xff);


	ret = gc0310_write_reg(client, GC0310_8BIT,
		GC0310_REG_COARSE_EXPOSURE_H, expo_coarse_h);
	ret |= gc0310_write_reg(client, GC0310_8BIT,
		GC0310_REG_COARSE_EXPOSURE_L, expo_coarse_l);

	if (ret) {
		 v4l2_err(client, "%s: fail to set exposure time\n", __func__);
		 return -EINVAL;
	}

	return ret;
}

static long gc0310_ioctl(struct v4l2_subdev *sd, unsigned int cmd, void *arg)
{
	switch (cmd) {
	case ATOMISP_IOC_S_EXPOSURE:
		return gc0310_s_exposure(sd, arg);
	default:
		return -EINVAL;
	}
	return 0;
}


/* This returns the exposure time being used. This should only be used
   for filling in EXIF data, not for actual image processing. */
static int gc0310_g_exposure(struct v4l2_subdev *sd, s32 *value)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	u16 coarse;
	u8 reg_val_h, reg_val_l;
	int ret;

	/* the fine integration time is currently not calculated */
	ret = gc0310_read_reg(client, GC0310_8BIT,
				GC0310_REG_COARSE_EXPOSURE_H, &reg_val_h);
	if (ret)
		return ret;

	coarse = ((u16)(reg_val_h & 0x0f)) << 8;

	ret = gc0310_read_reg(client, GC0310_8BIT,
				GC0310_REG_COARSE_EXPOSURE_L, &reg_val_l);
	if (ret)
		return ret;

	coarse |= reg_val_l;

	*value = coarse;
	return 0;
}

static struct gc0310_control gc0310_controls[] = {
	{
		.qc = {
			.id = V4L2_CID_FOCAL_ABSOLUTE,
			.type = V4L2_CTRL_TYPE_INTEGER,
			.name = "focal length",
			.minimum = GC0310_FOCAL_LENGTH_DEFAULT,
			.maximum = GC0310_FOCAL_LENGTH_DEFAULT,
			.step = 0x01,
			.default_value = GC0310_FOCAL_LENGTH_DEFAULT,
			.flags = 0,
		},
		.query = gc0310_g_focal,
	},
	{
		.qc = {
			.id = V4L2_CID_FNUMBER_ABSOLUTE,
			.type = V4L2_CTRL_TYPE_INTEGER,
			.name = "f-number",
			.minimum = GC0310_F_NUMBER_DEFAULT,
			.maximum = GC0310_F_NUMBER_DEFAULT,
			.step = 0x01,
			.default_value = GC0310_F_NUMBER_DEFAULT,
			.flags = 0,
		},
		.query = gc0310_g_fnumber,
	},
	{
		.qc = {
			.id = V4L2_CID_FNUMBER_RANGE,
			.type = V4L2_CTRL_TYPE_INTEGER,
			.name = "f-number range",
			.minimum = GC0310_F_NUMBER_RANGE,
			.maximum =  GC0310_F_NUMBER_RANGE,
			.step = 0x01,
			.default_value = GC0310_F_NUMBER_RANGE,
			.flags = 0,
		},
		.query = gc0310_g_fnumber_range,
	},
	{
		.qc = {
			.id = V4L2_CID_POWER_LINE_FREQUENCY,
			.type = V4L2_CTRL_TYPE_MENU,
			.name = "Light frequency filter",
			.minimum = 1,
			.maximum =  2, /* 1: 50Hz, 2:60Hz */
			.step = 1,
			.default_value = 1,
			.flags = 0,
		},
		.tweak = gc0310_s_freq,
	},
	{
		.qc = {
			.id = V4L2_CID_EXPOSURE_ABSOLUTE,
			.type = V4L2_CTRL_TYPE_INTEGER,
			.name = "exposure",
			.minimum = 0x0,
			.maximum = 0xffff,
			.step = 0x01,
			.default_value = 0x00,
			.flags = 0,
		},
		.query = gc0310_g_exposure,
	},

};
#define N_CONTROLS (ARRAY_SIZE(gc0310_controls))

static struct gc0310_control *gc0310_find_control(__u32 id)
{
	int i;

	for (i = 0; i < N_CONTROLS; i++) {
		if (gc0310_controls[i].qc.id == id)
			return &gc0310_controls[i];
	}
	return NULL;
}

#define ASUS_DETECT_CHIP_ID 1

#if ASUS_DETECT_CHIP_ID
static int gc0310_detect(struct gc0310_device *dev, struct i2c_client *client)
{
	struct i2c_adapter *adapter = client->adapter;
	u8 retvalue;

	if (!i2c_check_functionality(adapter, I2C_FUNC_I2C)) {
		dev_err(&client->dev, "%s: i2c error", __func__);
		return -ENODEV;
	}
	gc0310_read_reg(client, GC0310_8BIT, (u8)GC0310_REG_SENSOR_ID_H, &retvalue);
	dev->real_model_id = retvalue;
	printk("detect gc0310 module ID = %x\n", retvalue);

	if (retvalue != GC0310_MOD_ID_H) {
		dev_err(&client->dev, "%s: failed: client->addr = %x\n",
			__func__, client->addr);
		return -ENODEV;
	}

	return 0;
}
#endif


static int
gc0310_s_config(struct v4l2_subdev *sd, int irq, void *platform_data)
{
	struct gc0310_device *dev = to_gc0310_sensor(sd);
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	int ret;

	if (NULL == platform_data)
		return -ENODEV;

	dev->platform_data =
	    (struct camera_sensor_platform_data *)platform_data;

	if (dev->platform_data->platform_init) {
		ret = dev->platform_data->platform_init(client);
		if (ret) {
			v4l2_err(client, "gc0310 platform init err\n");
			return ret;
		}
	}

#if ASUS_DETECT_CHIP_ID
	/* power off the module, then power on it in future
	 * as first power on by board may not fulfill the
	 * power on sequqence needed by the module
	 */
	ret = gc0310_s_power(sd, 0);
	if (ret) {
		v4l2_err(client, "gc0310 power-off err");
		return ret;
	}

	ret = gc0310_s_power(sd, 1);
	if (ret) {
		v4l2_err(client, "gc0310 power-up err");
		gc0310_s_power(sd, 0);
		return ret;
	}
#endif

	ret = dev->platform_data->csi_cfg(sd, 1);
	if (ret)
		goto fail_csi_cfg;

#if ASUS_DETECT_CHIP_ID
	/* config & detect sensor */
	ret = gc0310_detect(dev, client);
	if (ret) {
		v4l2_err(client, "gc0310_detect err s_config.\n");
		goto fail_detect;
	}

/*
	ret = gc0310_init_common(sd);
	if (ret) {
		v4l2_err(client, "gc0310_init_common err s_config.\n");
		goto fail_detect;
	}
*/
	ret = gc0310_set_suspend(sd);
	if (ret) {
		v4l2_err(client, "gc0310 suspend err");
		return ret;
	}

	ret = gc0310_s_power(sd, 0);
	if (ret) {
		v4l2_err(client, "gc0310 power down err");
		return ret;
	}
#endif
	return 0;

fail_csi_cfg:
	dev->platform_data->csi_cfg(sd, 0);

#if ASUS_DETECT_CHIP_ID
fail_detect:
	gc0310_s_power(sd, 0);
	dev_err(&client->dev, "sensor power-gating failed\n");
#endif
	return ret;

}

static int gc0310_queryctrl(struct v4l2_subdev *sd, struct v4l2_queryctrl *qc)
{
	struct gc0310_control *ctrl = gc0310_find_control(qc->id);

	if (ctrl == NULL)
		return -EINVAL;
	*qc = ctrl->qc;
	return 0;
}

static int gc0310_s_parm(struct v4l2_subdev *sd,
			struct v4l2_streamparm *param)
{
	return 0;
}

static int gc0310_g_ctrl(struct v4l2_subdev *sd, struct v4l2_control *ctrl)
{
	struct gc0310_control *octrl = gc0310_find_control(ctrl->id);
	int ret;

	if (octrl == NULL)
		return -EINVAL;

	ret = octrl->query(sd, &ctrl->value);
	if (ret < 0)
		return ret;

	return 0;
}

static int gc0310_s_ctrl(struct v4l2_subdev *sd, struct v4l2_control *ctrl)
{
	struct gc0310_control *octrl = gc0310_find_control(ctrl->id);
	int ret;

	if (!octrl || !octrl->tweak)
		return -EINVAL;

	ret = octrl->tweak(sd, ctrl->value);
	if (ret < 0)
		return ret;

	return 0;
}

static int gc0310_s_stream(struct v4l2_subdev *sd, int enable)
{
	int ret;

	if (enable) {
		printk("gc0310_s_stream: Stream On\n");
		ret = gc0310_set_streaming(sd);
	} else {
		printk("gc0310_s_stream: Stream Off\n");
		ret = gc0310_set_suspend(sd);
	}

	return ret;
}

static int
gc0310_enum_framesizes(struct v4l2_subdev *sd, struct v4l2_frmsizeenum *fsize)
{
	unsigned int index = fsize->index;

	if (index >= N_RES)
		return -EINVAL;

	fsize->type = V4L2_FRMSIZE_TYPE_DISCRETE;
	fsize->discrete.width = gc0310_res[index].width;
	fsize->discrete.height = gc0310_res[index].height;

	/* FIXME: Wrong way to know used mode */
	fsize->reserved[0] = gc0310_res[index].used;

	return 0;
}

static int gc0310_enum_frameintervals(struct v4l2_subdev *sd,
        struct v4l2_frmivalenum *fival)
{
	unsigned int index = fival->index;
	int i;

	if (index >= N_RES)
		return -EINVAL;

	/* find out the first equal or bigger size */
	for (i = 0; i < N_RES; i++) {
		if ((gc0310_res[i].width >= fival->width) &&
                (gc0310_res[i].height >= fival->height))
			break;
	}
	if (i == N_RES)
		i--;

	index = i;

	fival->type = V4L2_FRMIVAL_TYPE_DISCRETE;
	fival->discrete.numerator = 1;
	fival->discrete.denominator = gc0310_res[index].fps;

	return 0;
}

static int
gc0310_g_chip_ident(struct v4l2_subdev *sd, struct v4l2_dbg_chip_ident *chip)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);

	return v4l2_chip_ident_i2c_client(client, chip, V4L2_IDENT_GC0310, 0);
}

static int gc0310_enum_mbus_code(struct v4l2_subdev *sd,
				  struct v4l2_subdev_fh *fh,
				  struct v4l2_subdev_mbus_code_enum *code)
{
	if (code->index)
		return -EINVAL;

	code->code = V4L2_MBUS_FMT_SGRBG8_1X8;

	return 0;
}

static int gc0310_enum_frame_size(struct v4l2_subdev *sd,
	struct v4l2_subdev_fh *fh,
	struct v4l2_subdev_frame_size_enum *fse)
{
	unsigned int index = fse->index;


	if (index >= N_RES)
		return -EINVAL;

	fse->min_width = gc0310_res[index].width;
	fse->min_height = gc0310_res[index].height;
	fse->max_width = gc0310_res[index].width;
	fse->max_height = gc0310_res[index].height;

	return 0;
}

static struct v4l2_mbus_framefmt *
__gc0310_get_pad_format(struct gc0310_device *sensor,
			 struct v4l2_subdev_fh *fh, unsigned int pad,
			 enum v4l2_subdev_format_whence which)
{
	struct i2c_client *client = v4l2_get_subdevdata(&sensor->sd);

	if (pad != 0) {
		dev_err(&client->dev,  "%s err. pad %x\n", __func__, pad);
		return NULL;
	}

	switch (which) {
	case V4L2_SUBDEV_FORMAT_TRY:
		return v4l2_subdev_get_try_format(fh, pad);
	case V4L2_SUBDEV_FORMAT_ACTIVE:
		return &sensor->format;
	default:
		return NULL;
	}
}

static int
gc0310_get_pad_format(struct v4l2_subdev *sd, struct v4l2_subdev_fh *fh,
		       struct v4l2_subdev_format *fmt)
{
	struct gc0310_device *snr = to_gc0310_sensor(sd);
	struct v4l2_mbus_framefmt *format =
			__gc0310_get_pad_format(snr, fh, fmt->pad, fmt->which);

	if (format == NULL)
		return -EINVAL;
	fmt->format = *format;

	return 0;
}

static int
gc0310_set_pad_format(struct v4l2_subdev *sd, struct v4l2_subdev_fh *fh,
		       struct v4l2_subdev_format *fmt)
{
	struct gc0310_device *snr = to_gc0310_sensor(sd);
	struct v4l2_mbus_framefmt *format =
			__gc0310_get_pad_format(snr, fh, fmt->pad, fmt->which);

	if (format == NULL)
		return -EINVAL;

	if (fmt->which == V4L2_SUBDEV_FORMAT_ACTIVE)
		snr->format = fmt->format;

	return 0;
}

static int gc0310_g_skip_frames(struct v4l2_subdev *sd, u32 *frames)
{
	int index;
	struct gc0310_device *snr = to_gc0310_sensor(sd);

	if (frames == NULL)
		return -EINVAL;

	for (index = 0; index < N_RES; index++) {
		if (gc0310_res[index].res == snr->res)
			break;
	}

	if (index >= N_RES)
		return -EINVAL;

	*frames = gc0310_res[index].skip_frames;

	return 0;
}

static const struct v4l2_subdev_video_ops gc0310_video_ops = {
	.try_mbus_fmt = gc0310_try_mbus_fmt,
	.s_mbus_fmt = gc0310_set_mbus_fmt,
	.g_mbus_fmt = gc0310_get_mbus_fmt,
	.s_stream = gc0310_s_stream,
	.enum_framesizes = gc0310_enum_framesizes,
	.enum_frameintervals = gc0310_enum_frameintervals,
	.s_parm = gc0310_s_parm,
};

static struct v4l2_subdev_sensor_ops gc0310_sensor_ops = {
	.g_skip_frames	= gc0310_g_skip_frames,
};

static const struct v4l2_subdev_core_ops gc0310_core_ops = {
	.g_chip_ident = gc0310_g_chip_ident,
	.queryctrl = gc0310_queryctrl,
	.g_ctrl = gc0310_g_ctrl,
	.s_ctrl = gc0310_s_ctrl,
	.s_power = gc0310_s_power,
	.ioctl = gc0310_ioctl,
};

/* REVISIT: Do we need pad operations? */
static const struct v4l2_subdev_pad_ops gc0310_pad_ops = {
	.enum_mbus_code = gc0310_enum_mbus_code,
	.enum_frame_size = gc0310_enum_frame_size,
	.get_fmt = gc0310_get_pad_format,
	.set_fmt = gc0310_set_pad_format,
};

static const struct v4l2_subdev_ops gc0310_ops = {
	.core = &gc0310_core_ops,
	.video = &gc0310_video_ops,
	.pad = &gc0310_pad_ops,
	.sensor = &gc0310_sensor_ops,
};

static const struct media_entity_operations gc0310_entity_ops = {
	.link_setup = NULL,
};


static int gc0310_remove(struct i2c_client *client)
{
	struct gc0310_device *dev;
	struct v4l2_subdev *sd = i2c_get_clientdata(client);

	dev = container_of(sd, struct gc0310_device, sd);
	dev->platform_data->csi_cfg(sd, 0);
	if (dev->platform_data->platform_deinit)
		dev->platform_data->platform_deinit();
	v4l2_device_unregister_subdev(sd);
	media_entity_cleanup(&dev->sd.entity);
	kfree(dev);
	return 0;
}

/* --------------------------- */
static int dbg_gc0310_vga_status_open(
	struct inode *inode, struct file *file)
{
	file->private_data = inode->i_private;
	return 0;
}

static ssize_t dbg_gc0310_vga_status_read(
	struct file *file,
	char __user *buf,
	size_t count,
	loff_t *ppos)
{
	int len = 0;
	int tot = 0;
	char debug_buf[256];
	int dlen = sizeof(debug_buf);
	char *bp = debug_buf;

	u8 id = 0xFF;
	int err = 0;
	struct v4l2_subdev *sd = i2c_get_clientdata(g_client);

	pr_info("%s: buf=%p, count=%d, ppos=%p; *ppos= %d\n",
		__func__, buf, (int)count, ppos, (int)*ppos);

	if (*ppos)
		return 0;	/* the end */

	/* Power-on  */
	power_up(sd);

	err = gc0310_read_reg(g_client, GC0310_8BIT, (u8)GC0310_REG_SENSOR_ID_H, &id);

	printk("detect gc0310 module ID = %x\n", id);

	if (err) {
		len = snprintf(bp, dlen, "0\n");
		tot += len; bp += len; dlen -= len;
		pr_info("%s(%d): err=%d\n", __func__, __LINE__, err);
	} else if (id == GC0310_MOD_ID_H) {
		len = snprintf(bp, dlen, "1\n");
		tot += len; bp += len; dlen -= len;
	} else {
		len = snprintf(bp, dlen, "0\n");
		tot += len; bp += len; dlen -= len;
		pr_info("%s(%d): wrong chip_id: 0x%X\n",
			__func__, __LINE__, id);
	}

	/* Power-off */
	power_down(sd);

	if (copy_to_user(buf, debug_buf, tot))
		return -EFAULT;
	if (tot < 0)
		return 0;
	*ppos += tot;	/* increase offset */
	return tot;
}

static const struct file_operations dbg_gc0310_vga_status_fops = {
	.open		= dbg_gc0310_vga_status_open,
	.read		= dbg_gc0310_vga_status_read,
};

static int gc0310_dbgfs_init(void)
{
	struct dentry *debugfs_dir;
	debugfs_dir = debugfs_create_dir("camera1", NULL);

	debugfs_create_u8("vga_status", 0644, debugfs_dir, &atd_node);
	debugfs_create_x16("by_file", 0644, debugfs_dir, &g_by_file);
	debugfs_create_u32("sensor_id", 0644, debugfs_dir, &WhoAmI);

	return 0;
}
/* --------------------------- */

static int gc0310_probe(struct i2c_client *client,
				const struct i2c_device_id *id)
{
	struct gc0310_device *dev;
	int ret;

	/* Setup sensor configuration structure */
	dev = kzalloc(sizeof(*dev), GFP_KERNEL);
	if (!dev) {
		dev_err(&client->dev, "out of memory\n");
		return -ENOMEM;
	}

	v4l2_i2c_subdev_init(&dev->sd, client, &gc0310_ops);

	if (client->dev.platform_data) {
		ret = gc0310_s_config(&dev->sd, client->irq,
				       client->dev.platform_data);
		if (ret) {
			v4l2_device_unregister_subdev(&dev->sd);
			kfree(dev);
			return ret;
		}
	}

	/*TODO add format code here*/
	dev->sd.flags |= V4L2_SUBDEV_FL_HAS_DEVNODE;
	dev->pad.flags = MEDIA_PAD_FL_SOURCE;
	dev->format.code = V4L2_MBUS_FMT_SGRBG8_1X8;
	dev->sd.entity.type = MEDIA_ENT_T_V4L2_SUBDEV_SENSOR;

	/* REVISIT: Do we need media controller? */
	ret = media_entity_init(&dev->sd.entity, 1, &dev->pad, 0);
	if (ret) {
		gc0310_remove(client);
		return ret;
	}

	/* set res index to be invalid */
	dev->res = -1;

	/* dbgfs init */
	g_client = client;
	gc0310_dbgfs_init();

	return 0;
}

MODULE_DEVICE_TABLE(i2c, gc0310_id);

static struct i2c_driver gc0310_driver = {
	.driver = {
		.owner = THIS_MODULE,
		.name = "gc0310"
	},
	.probe = gc0310_probe,
	.remove = gc0310_remove,
	.id_table = gc0310_id,
};

static __init int init_gc0310(void)
{

	return i2c_add_driver(&gc0310_driver);
}

static __exit void exit_gc0310(void)
{

	i2c_del_driver(&gc0310_driver);
}

module_init(init_gc0310);
module_exit(exit_gc0310);

MODULE_AUTHOR("Qi Jing <qix.jing@intel.com>");
MODULE_LICENSE("GPL");
