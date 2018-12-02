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
#include "gc2155.h"

//#include <linux/board_asustek.h>

#define PROJECT_ID_FE375CG	0x1
#define PROJECT_ID_FE375CXG	0x4

#define to_gc2155_sensor(sd) container_of(sd, struct gc2155_device, sd)

/*
 * TODO: use debug parameter to actually define when debug messages should
 * be printed.
 */
static int debug;
module_param(debug, int, 0644);
MODULE_PARM_DESC(debug, "Debug level (0-1)");

//Add for ATD read camera status+++
static unsigned int ATD_gc2155_status = 0;  //Add for ATD read camera status

static struct i2c_client *g_client;
static u16 g_by_file = 0x00;

static ssize_t gc2155_show_status(struct device *dev,struct device_attribute *attr,char *buf)
{
	pr_info("%s: get gc2155 status (%d) !!\n", __func__, ATD_gc2155_status);
	//Check sensor connect status, just do it  in begining for ATD camera status

	return sprintf(buf,"%d\n", ATD_gc2155_status);
}

static DEVICE_ATTR(gc2155_status, S_IRUGO,gc2155_show_status,NULL);

static struct attribute *gc2155_attributes[] = {
	&dev_attr_gc2155_status.attr,
	NULL
};
//Add for ATD read camera status---


static int
gc2155_read_reg(struct i2c_client *client, u16 data_length, u8 reg, u8 *val)
{
	int err;
	struct i2c_msg msg[2];
	unsigned char data[2];

	if (!client->adapter) {
		v4l2_err(client, "%s error, no client->adapter\n", __func__);
		return -ENODEV;
	}

	if (data_length != GC2155_8BIT && data_length != GC2155_16BIT
					 && data_length != GC2155_32BIT) {
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
		if (data_length == GC2155_8BIT)
			*val = data[1];
		/*else if (data_length == GC2155_16BIT)
			*val = data[1] + (data[0] << 8);
		else
			*val = data[3] + (data[2] << 8) +
				(data[1] << 16) + (data[0] << 24);*/

		return 0;
	}

	dev_err(&client->dev, "read from offset 0x%x error %d", reg, err);
	return err;
}

static int
gc2155_write_reg(struct i2c_client *client, u16 data_length, u8 reg, u8 val)
{
	int num_msg;
	struct i2c_msg msg;
	unsigned char data[4] = {0};
	int retry = 0;

	/* TODO: Should we deal with 16bit or 32bit data ? */

	/* pr_info("%s(ff %02x %02x)\n", __func__, reg, val); */

	if (!client->adapter) {
		v4l2_err(client, "%s error, no client->adapter\n", __func__);
		return -ENODEV;
	}

	if (data_length != GC2155_8BIT && data_length != GC2155_16BIT
					 && data_length != GC2155_32BIT) {
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

	/* high byte goes out first
	wreg = (u16 *)data;
	*wreg = cpu_to_be16(reg);*/

	if (data_length == GC2155_8BIT)
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
	if (retry < I2C_RETRY_COUNT) {
		dev_dbg(&client->dev, "retrying... %d", retry);
		retry++;
		msleep(20);
		goto again;
	}

	return num_msg;
}

/*
 * gc2155_write_reg_array - Initializes a list of gc2155 registers
 * @client: i2c driver client structure
 * @reglist: list of registers to be written
 */

static int gc2155_write_reg_array(struct i2c_client *client,
				const struct gc2155_reg_cmd reglist[])
{
	int err;
	const struct gc2155_reg_cmd *next;
	u16 val;

	for (next = reglist; next->cmd != SENSOR_TABLE_END; next++) {

		if (next->cmd == SENSOR_WAIT_MS) {
			msleep(next->val);
			continue;
		}

		val = next->val;

		if (next->cmd == GC2155_8BIT)
			err = gc2155_write_reg(client, GC2155_8BIT, next->addr, val);
		else
			pr_info("%s: Unknown cmd: %d\n", __func__, next->cmd);

		if (err) {
			pr_info("%s: err=%d\n", __func__, err);
			return err;
		}
	}
	return 0;
}

static int gc2155_set_suspend(struct v4l2_subdev *sd)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	pr_info("%s()\n",__func__);

	gc2155_write_reg(client, GC2155_8BIT,  REG_RST_AND_PG_SELECT, 0x03);

	/* TODO: project id detect here. */
	gc2155_write_reg(client, GC2155_8BIT,  0x16, 0x05);
	msleep(200);
#if TWO_LANE
	gc2155_write_reg(client, GC2155_8BIT,  0x10, 0x01);
#else
	gc2155_write_reg(client, GC2155_8BIT,  0x10, 0x00);
#endif
	gc2155_write_reg(client, GC2155_8BIT,  0x01, 0x80);
	gc2155_write_reg(client, GC2155_8BIT,  REG_RST_AND_PG_SELECT, 0x00);
	return 0;
}

static int gc2155_set_streaming(struct v4l2_subdev *sd)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	pr_info("%s()\n",__func__);

	/* mipi reset */
	gc2155_write_reg(client, GC2155_8BIT,  REG_RST_AND_PG_SELECT, 0x30);

	/* TODO: project id detect here. */
	gc2155_write_reg(client, GC2155_8BIT,  REG_RST_AND_PG_SELECT, 0x03);
	gc2155_write_reg(client, GC2155_8BIT,  0x16, 0x09);
#if TWO_LANE
	gc2155_write_reg(client, GC2155_8BIT,  0x10, 0x91);
#else
	gc2155_write_reg(client, GC2155_8BIT,  0x10, 0x90);
#endif
	gc2155_write_reg(client, GC2155_8BIT,  0x01, 0x83);
	gc2155_write_reg(client, GC2155_8BIT,  REG_RST_AND_PG_SELECT, 0x00);
	return 0;
}

static int gc2155_init_common(struct v4l2_subdev *sd)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	int ret = 0;

	pr_info("%s()++\n", __func__);

	ret = gc2155_write_reg_array(client, gc2155_common);

	if (ret)
		pr_info("%s: write seq fail\n", __func__);

	pr_info("%s()--\n", __func__);
	return ret;
}

static u8 char2u8(u8 *p_c)
{
	u8 ret_val = 0;

	u8 h4b, l4b;

	if ((p_c[0] >= 'a') && (p_c[0] <= 'f'))
		h4b = p_c[0] - 'a' + 10;
	else if ((p_c[0] >= 'A') && (p_c[0] <= 'F'))
		h4b = p_c[0] - 'A' + 10;
	else if ((p_c[0] >= '0') && (p_c[0] <= '9'))
		h4b = p_c[0] - '0';
	else {
		pr_info("%s: convert h4b error\n", __func__);
		return 0xFF;
	}

	if ((p_c[1] >= 'a') && (p_c[1] <= 'f'))
		l4b = p_c[1] - 'a' + 10;
	else if ((p_c[1] >= 'A') && (p_c[1] <= 'F'))
		l4b = p_c[1] - 'A' + 10;
	else if ((p_c[1] >= '0') && (p_c[1] <= '9'))
		l4b = p_c[1] - '0';
	else {
		pr_info("%s: convert l4b error\n", __func__);
		return 0xFF;
	}

	ret_val = (h4b << 4) | l4b;

	return ret_val;
}

static int gc2155_init_common_by_file(struct v4l2_subdev *sd)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	int ret = 0;
	struct file *fp = NULL;
	struct inode *inode;
	mm_segment_t old_fs;
	int bootbin_size = 0;
	u8 *p_reg;
	u8 *p_val;
	u8 *p_tmp;

	u8 reg, val;
	pr_info("%s()++\n", __func__);

	fp = filp_open("/system/bin/gc2155.txt", O_RDONLY, 0);

	if (!IS_ERR_OR_NULL(fp)) {
		pr_info("filp_open success fp:%p\n", fp);
		inode = fp->f_dentry->d_inode;
		bootbin_size = inode->i_size;
		/*
		printk("%s: fp->f_dentry->d_inode->i_size=%d\n", __FUNCTION__,
			bootbin_size);
		*/
		p_reg = kmalloc(2, GFP_KERNEL);
		p_val = kmalloc(2, GFP_KERNEL);
		p_tmp = kmalloc(10, GFP_KERNEL);
		old_fs = get_fs();
		set_fs(KERNEL_DS);
		if (fp->f_op != NULL && fp->f_op->read != NULL) {
			int byte_count = 0;
			int i = 0;
			for (i = 0; i < (bootbin_size/7) + 1; i++) {
				byte_count += fp->f_op->read(fp, p_reg, 2, &fp->f_pos);
				byte_count += fp->f_op->read(fp, p_tmp, 1, &fp->f_pos);
				byte_count += fp->f_op->read(fp, p_val, 2, &fp->f_pos);
				byte_count += fp->f_op->read(fp, p_tmp, 2, &fp->f_pos);

				reg = char2u8(p_reg);
				val = char2u8(p_val);
				pr_info("%s(%d): 0x%02x 0x%02x\n", __func__, __LINE__, reg, val);
				ret |= gc2155_write_reg(client, GC2155_8BIT,
					reg, val);
			}

			if (byte_count <= 0)
				pr_info("%s: EOF or error. last byte_count= %d;\n", __func__, byte_count);
			else
				pr_info("%s: init file size= %d bytes\n", __func__, bootbin_size);

		}
		set_fs(old_fs);
		filp_close(fp, NULL);
		kfree(p_reg);
		kfree(p_val);
		kfree(p_tmp);
	} else if (PTR_ERR(fp) == -ENOENT) {
		pr_err("%s: /sdcard/gc2155.txt not found error\n", __func__);
		ret = -EINVAL;
	} else {
		pr_err("%s: /sdcard/gc2155.txt open error\n", __func__);
		ret = -EINVAL;
	}

	if (ret)
		pr_info("%s: write seq fail\n", __func__);

	pr_info("%s()--\n", __func__);
	return ret;
}

static int power_up(struct v4l2_subdev *sd)
{
	struct gc2155_device *dev = to_gc2155_sensor(sd);
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

	return 0;

fail_power:
	dev->platform_data->power_ctrl(sd, 0);
	dev_err(&client->dev, "sensor power-up failed\n");

	return ret;
}

static int power_down(struct v4l2_subdev *sd)
{
	struct gc2155_device *dev = to_gc2155_sensor(sd);
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

static int gc2155_s_power(struct v4l2_subdev *sd, int power)
{
	pr_info("%s(%d) %s\n", __func__, __LINE__, power ? ("on") : ("off"));
	if (power == 0)
		return power_down(sd);
	else {
		if (power_up(sd))
			return -EINVAL;

		if (g_by_file)
			return gc2155_init_common_by_file(sd);
		else
			return gc2155_init_common(sd);
	}
}

/*
 * distance - calculate the distance
 * @res: resolution
 * @w: width
 * @h: height
 *
 * Get the gap between resolution and w/h.
 * res->width/height smaller than w/h wouldn't be considered.
 * Returns the value of gap or -1 if fail.
 */
static int distance(struct gc2155_res_struct *res, u32 w, u32 h)
{
	unsigned int w_ratio = ((res->width << RATIO_SHIFT_BITS)/w);
	unsigned int h_ratio;
	int match;

	if (h == 0)
		return -1;
	h_ratio = ((res->height << RATIO_SHIFT_BITS) / h);
	if (h_ratio == 0)
		return -1;

	if ((res->width == w) && (res->height == h))
		pr_info("%s(%d): res %dx%d exactly identical.\n",
			__func__, __LINE__, w, h);

	match   = abs(((w_ratio << RATIO_SHIFT_BITS) / h_ratio)
			- ((int)(1 << RATIO_SHIFT_BITS)));

	if ((w_ratio < (int)(1 << RATIO_SHIFT_BITS))
	    || (h_ratio < (int)(1 << RATIO_SHIFT_BITS))) {
		return -1;
	}
	return match;
}

static int gc2155_try_res(u32 *w, u32 *h)
{
	int i;
	int idx = N_RES-1;
	int dist;
	int min_dist = INT_MAX;
	struct gc2155_res_struct *tmp_res = NULL;

	pr_info("%s++, w=%d, h=%d\n", __func__, *w, *h);

	/*
	 * The mode list is in ascending order. We're done as soon as
	 * we have found the first equal or bigger size.
	 */
	for (i = 0; i < N_RES; i++) {
		tmp_res = &gc2155_res[i];
		dist = distance(tmp_res, *w, *h);
		pr_info("%s: %d: %s: dist= %d\n", __func__, i,
				gc2155_res[i].desc, dist);
		if (dist == -1)
			continue;
		if (dist < min_dist) {
			min_dist = dist;
			idx = i;
		}
	}

	/*
	 * If no mode was found, it means we can provide only a smaller size.
	 * Returning the biggest one available in this case.
	 */
	if (idx == N_RES)
		idx--;

	*w = gc2155_res[idx].width;
	*h = gc2155_res[idx].height;

	pr_info("tried width = %d, height = %d\n", *w, *h);

	return 0;
}

static struct gc2155_res_struct *gc2155_to_res(u32 w, u32 h)
{
	int  index;

	pr_info("%s++\n", __func__);

	for (index = 0; index < N_RES; index++) {
		if ((gc2155_res[index].width == w) &&
			(gc2155_res[index].height == h))
			break;
	}

	/* No mode found */
	if (index >= N_RES) {
		pr_info("%s-- no mode found\n", __func__);
		return NULL;
	}
	pr_info("%s-- gc2155_res[%d] is selected\n", __func__, index);
	return &gc2155_res[index];
}

static int gc2155_try_mbus_fmt(struct v4l2_subdev *sd,
				struct v4l2_mbus_framefmt *fmt)
{
	pr_info("%s++\n", __func__);
	fmt->code = V4L2_MBUS_FMT_SRGGB10_1X10;
	return gc2155_try_res(&fmt->width, &fmt->height);
}

static int gc2155_res2size(unsigned int res, int *h_size, int *v_size)
{
	unsigned short hsize;
	unsigned short vsize;

	pr_info("%s++\n", __func__);

	switch (res) {
	case GC2155_RES_CIF:
		hsize = GC2155_RES_CIF_SIZE_H;
		vsize = GC2155_RES_CIF_SIZE_V;
		break;
	case GC2155_RES_VGA:
		hsize = GC2155_RES_VGA_SIZE_H;
		vsize = GC2155_RES_VGA_SIZE_V;
		break;
	case GC2155_RES_2M:
		hsize = GC2155_RES_2M_SIZE_H;
		vsize = GC2155_RES_2M_SIZE_V;
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

static int gc2155_get_intg_factor(struct i2c_client *client,
				struct camera_mipi_info *info,
				const struct gc2155_res_struct *res)
{
	struct atomisp_sensor_mode_data *buf = &info->data;

	unsigned int hb, vb, sh_delay;
	u8 reg_val, reg_val2, div, div2en;
	int ret;

	pr_info("%s()++: TODO\n", __func__);
	dev_err(&client->dev, "%s\n", __func__);

	if (info == NULL)
		return -EINVAL;

	ret = gc2155_read_reg(client, GC2155_8BIT,  REG_RST_AND_PG_SELECT, &reg_val);
	pr_info("%s(%d): page = %d\n", __func__, __LINE__, reg_val);

	/* if(div2en)
	 *   pclk = (mclk/4) * (div2 + 1)
	 * else
	 *   pclk = (mclk/2) * (div2 + 1)
	 *
	 *  div2en = bit[1] of [0xf7]
	 *  div2 = bit[0-5] of [0xf8]
	 */
	ret = gc2155_read_reg(client, GC2155_8BIT,  REG_PLL_MODE1, &div2en);
	ret = gc2155_read_reg(client, GC2155_8BIT,  REG_PLL_MODE2, &div);
	if (div2en & 0x2)
		buf->vt_pix_clk_freq_mhz = (19200000 >> 2) * ((div & 0x3F) + 1);
	else
		buf->vt_pix_clk_freq_mhz = (19200000 >> 1) * ((div & 0x3F) + 1);

	pr_info("vt_pix_clk_freq_mhz = %d; div=0x%x\n", buf->vt_pix_clk_freq_mhz, div);

	/* get integration time. DIT doesn't use these vales. */
	buf->coarse_integration_time_min = GC2155_COARSE_INTG_TIME_MIN;
	buf->coarse_integration_time_max_margin =
					GC2155_COARSE_INTG_TIME_MAX_MARGIN;

	buf->fine_integration_time_min = GC2155_FINE_INTG_TIME_MIN;
	buf->fine_integration_time_max_margin =
					GC2155_FINE_INTG_TIME_MAX_MARGIN;

	buf->fine_integration_time_def = GC2155_FINE_INTG_TIME_MIN;

	/* crop_horizontal_start */
	ret = gc2155_read_reg(client, GC2155_8BIT, REG_COL_START_H, &reg_val);
	ret |= gc2155_read_reg(client, GC2155_8BIT, REG_COL_START_L, &reg_val2);

	if (ret) {
		pr_info("Read COL_START fail\n");
		return ret;
	} else {
		buf->crop_horizontal_start = ((reg_val << 8) & 0x0700) | reg_val2;
		pr_info("crop_horizontal_start = %d\n", buf->crop_horizontal_start);
	}

	/* crop_vertical_start */
	ret = gc2155_read_reg(client, GC2155_8BIT, REG_ROW_START_H, &reg_val);
	ret |= gc2155_read_reg(client, GC2155_8BIT, REG_ROW_START_L, &reg_val2);

	if (ret) {
		pr_info("Read ROW_START fail\n");
		return ret;
	} else {
		buf->crop_vertical_start = ((reg_val << 8) & 0x0700) | reg_val2;
		pr_info("crop_vertical_start = %d\n", buf->crop_vertical_start);
	}

	/* output_width */
	ret = gc2155_read_reg(client, GC2155_8BIT, REG_WIN_WIDTH_H, &reg_val);
	ret |= gc2155_read_reg(client, GC2155_8BIT, REG_WIN_WIDTH_L, &reg_val2);

	if (ret) {
		pr_info("Read WIN_WIDTH fail\n");
		return ret;
	} else {
		buf->output_width = ((reg_val << 8) & 0x0700) | reg_val2;
		pr_info("output_width = %d\n", buf->output_width);
	}

	/* crop_horizontal_end */
	buf->crop_horizontal_end =
		buf->crop_horizontal_start + buf->output_width - 1;
	pr_info("crop_horizontal_end = %d\n", buf->crop_horizontal_end);

	/* output_height */
	ret = gc2155_read_reg(client, GC2155_8BIT, REG_WIN_HEIGHT_H, &reg_val);
	ret |= gc2155_read_reg(client, GC2155_8BIT, REG_WIN_HEIGHT_L, &reg_val2);

	if (ret) {
		pr_info("Read WIN_HEIGHT fail\n");
		return ret;
	} else {
		buf->output_height = ((reg_val << 8) & 0x0700) | reg_val2;
		pr_info("output_height = %d\n", buf->output_height);
	}

	/* crop_vertical_end */
	buf->crop_vertical_end =
		buf->crop_vertical_start + buf->output_height - 1;
	pr_info("crop_vertical_end = %d\n", buf->crop_vertical_end);

	/* H Blank */
	ret = gc2155_read_reg(client, GC2155_8BIT, REG_H_BLANK_H, &reg_val);
	ret |= gc2155_read_reg(client, GC2155_8BIT, REG_H_BLANK_L, &reg_val2);
	if (ret) {
		pr_info("Read H_BLANK fail\n");
		return ret;
	} else {
		hb = ((reg_val << 8) & 0x0F00) | reg_val2;
		pr_info("hb = %d\n", hb);
	}

	/* Sh_delay */
	ret = gc2155_read_reg(client, GC2155_8BIT, REG_SH_DELAY_H, &reg_val);
	ret |= gc2155_read_reg(client, GC2155_8BIT, REG_SH_DELAY_L, &reg_val2);
	if (ret) {
		pr_info("Read SH_DELAY fail\n");
		return ret;
	} else {
		sh_delay = ((reg_val << 8) & 0x0300) | reg_val2;
		pr_info("sh_delay = %d\n", sh_delay);
	}

	/* line_length_pck(row_time):
	 *  row_time = Hb + Sh_delay + win_width + 4.
	 *
	 *   Hb: HBlank or dummy pixel, Setting by register P0:0x05 and P0:0x06.
	 *   Sh_delay: Setting by registerP0:0x11[9:8], P0:0x12[7:0].
	 *   win_width: Setting by register 0x0f and P0:0x10, win_width = 1600,
	 *   final_output_width + 8. So for UXGA, we should set win_width as 1616.
	 */

	/* gc2155 spec: buf->line_length_pck = hb + sh_delay + buf->output_width + 4;
	 * intel : buf->line_length_pck = (hb + sh_delay + (buf->output_width + 16)/2 + 4) << 1;
	 */
	buf->line_length_pck = (hb + sh_delay + (buf->output_width + 16)/2 + 4) << 1;
	pr_info("line_length_pck = %d\n", buf->line_length_pck);

	/* V Blank */
	ret = gc2155_read_reg(client, GC2155_8BIT, REG_V_BLANK_H, &reg_val);
	ret |= gc2155_read_reg(client, GC2155_8BIT, REG_V_BLANK_L, &reg_val2);
	if (ret) {
		pr_info("Read V_BLANK fail\n");
		return ret;
	} else {
		vb = ((reg_val << 8) & 0x1F00) | reg_val2;
		pr_info("vb = %d\n", vb);
	}

	/* frame_length_lines (Frame time, Ft)
	 * Ft = VB + Vt + 8 (unit is row_time)
	 *  VB = Bt + St + Et, Vblank/Dummy line, from P0:0x07 and P0:0x08.
	 *   Bt: Blank time, VSYNC no active time.
	 *   St: Start time, setting by register P0:0x13
	 *   Et: End time, setting by register P0:0x14
	 *  Vt: valid line time. UXGA is 1200, Vt = win_height - 8, win_height
	 *      is setting by register P0:0x0d and P0:0x0e(1232).
	 */
	/* gc2155 spec: buf->frame_length_lines = vb + buf->output_height;
	 * intel: buf->frame_length_lines = vb + buf->output_height + 32;
	 */
	buf->frame_length_lines = vb + buf->output_height + 32;
	pr_info("frame_length_lines = %d\n", buf->frame_length_lines);

	buf->read_mode = res->bin_mode;
	buf->binning_factor_x = 1;
	buf->binning_factor_y = 1;
	return 0;
}

static int gc2155_get_mbus_fmt(struct v4l2_subdev *sd,
				struct v4l2_mbus_framefmt *fmt)
{
	struct gc2155_device *dev = to_gc2155_sensor(sd);

	int width, height;
	int ret;

	pr_info("%s++\n", __func__);

	fmt->code = V4L2_MBUS_FMT_SRGGB10_1X10;

	ret = gc2155_res2size(dev->res, &width, &height);
	if (ret)
		return ret;
	fmt->width = width;
	fmt->height = height;

	return 0;
}

static int gc2155_set_mbus_fmt(struct v4l2_subdev *sd,
				  struct v4l2_mbus_framefmt *fmt)
{
	struct i2c_client *c = v4l2_get_subdevdata(sd);
	struct gc2155_device *dev = to_gc2155_sensor(sd);
	struct gc2155_res_struct *res_index;
	u32 width = fmt->width;
	u32 height = fmt->height;
	struct camera_mipi_info *gc2155_info = NULL;
	int ret;
	pr_info("%s()++\n", __func__);

	gc2155_info = v4l2_get_subdev_hostdata(sd);
	pr_info("gc2155_info %d %d %d %d\n",
		gc2155_info->port, gc2155_info->input_format,
		gc2155_info->num_lanes, gc2155_info->raw_bayer_order);

	if (gc2155_info == NULL)
		return -EINVAL;

	gc2155_try_res(&width, &height);
	res_index = gc2155_to_res(width, height);

	/* Sanity check */
	if (unlikely(!res_index)) {
		WARN_ON(1);
		return -EINVAL;
	}

	switch (res_index->res) {
	case GC2155_RES_CIF:
		pr_info("gc2155_set_mbus_fmt: CIF\n");
		ret = gc2155_write_reg_array(c, gc2155_cif_init);
		if (ret) {
			pr_info("%s: set CIF seq FAIL.\n", __func__);
			return -EINVAL;
		}
		break;
	case GC2155_RES_VGA:
		pr_info("gc2155_set_mbus_fmt: VGA\n");
		ret = gc2155_write_reg_array(c, gc2155_vga_init);
		if (ret) {
			pr_info("%s: set VGA seq FAIL.\n", __func__);
			return -EINVAL;
		}
		break;
	case GC2155_RES_2M:
		pr_info("gc2155_set_mbus_fmt: 2M\n");
		ret = gc2155_write_reg_array(c, gc2155_2M_init);
		if (ret) {
			pr_info("%s: set 2M seq FAIL.\n", __func__);
			return -EINVAL;
		}
		break;
	default:
		v4l2_err(sd, "set resolution: %d failed!\n", res_index->res);
		return -EINVAL;
	}

	ret = gc2155_get_intg_factor(c, gc2155_info,
					&gc2155_res[res_index->res]);
	if (ret) {
		dev_err(&c->dev, "failed to get integration_factor\n");
		return -EINVAL;
	}

	/*
	 * gc2155 - we don't poll for context switch
	 * because it does not happen with streaming disabled.
	 */
	dev->res = res_index->res;

	fmt->width = width;
	fmt->height = height;
	fmt->code = V4L2_MBUS_FMT_SRGGB10_1X10;
	pr_info("%s()--\n", __func__);
	return 0;
}

/* TODO: Update to SOC functions, remove exposure and gain */
static int gc2155_g_focal(struct v4l2_subdev *sd, s32 *val)
{
	*val = (GC2155_FOCAL_LENGTH_NUM << 16) | GC2155_FOCAL_LENGTH_DEM;
	return 0;
}

static int gc2155_g_fnumber(struct v4l2_subdev *sd, s32 *val)
{
	/*const f number for gc2155*/
	*val = (GC2155_F_NUMBER_DEFAULT_NUM << 16) | GC2155_F_NUMBER_DEM;
	return 0;
}

static int gc2155_g_fnumber_range(struct v4l2_subdev *sd, s32 *val)
{
	*val = (GC2155_F_NUMBER_DEFAULT_NUM << 24) |
		(GC2155_F_NUMBER_DEM << 16) |
		(GC2155_F_NUMBER_DEFAULT_NUM << 8) | GC2155_F_NUMBER_DEM;
	return 0;
}

static int gc2155_s_freq(struct v4l2_subdev *sd, s32  val)
{
	struct i2c_client *c = v4l2_get_subdevdata(sd);
	struct gc2155_device *dev = to_gc2155_sensor(sd);
	int ret;
	pr_info("%s++: TODO\n", __func__);
	if (val != GC2155_FLICKER_MODE_50HZ &&
			val != GC2155_FLICKER_MODE_60HZ)
		return -EINVAL;

	if (val == GC2155_FLICKER_MODE_50HZ) {
		ret = gc2155_write_reg_array(c, gc2155_antiflicker_50hz);
		if (ret < 0)
			return ret;
	} else {
		ret = gc2155_write_reg_array(c, gc2155_antiflicker_60hz);
		if (ret < 0)
			return ret;
	}

	if (ret == 0)
		dev->lightfreq = val;

	return ret;
}


static long gc2155_s_exposure(struct v4l2_subdev *sd,
					struct atomisp_exposure *exposure)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	u8 tmp;
	int ret = 0;

	unsigned int coarse_integration = 0;

	unsigned int analog_gain, digital_gain;

	u8 expo_coarse_h, expo_coarse_l;

	/*dev_err(&client->dev, "%s(0x%X 0x%X 0x%X)\n", __func__,
		exposure->integration_time[0], exposure->gain[0], exposure->gain[1]);*/

	coarse_integration = exposure->integration_time[0];
	/*
	fine_integration = ExposureTime.FineIntegrationTime;
	FrameLengthLines = ExposureTime.FrameLengthLines;
	*/
	analog_gain = exposure->gain[0];
	digital_gain = exposure->gain[1];

	expo_coarse_h = (u8)((coarse_integration >> 8) & 0x1F);
	expo_coarse_l = (u8)(coarse_integration & 0xff);


	ret = gc2155_read_reg(client, GC2155_8BIT,  REG_RST_AND_PG_SELECT, &tmp);
	//pr_info("%s(%d): page = %d\n", __func__, __LINE__, tmp);

	ret = gc2155_write_reg(client, GC2155_8BIT, REG_RST_AND_PG_SELECT, 0x0);

	ret = gc2155_read_reg(client, GC2155_8BIT,  REG_RST_AND_PG_SELECT, &tmp);
	//pr_info("%s(%d): page = %d\n", __func__, __LINE__, tmp);

	ret = gc2155_write_reg(client, GC2155_8BIT, REG_EXPO_COARSE_H, expo_coarse_h);
	ret = gc2155_write_reg(client, GC2155_8BIT, REG_EXPO_COARSE_L, expo_coarse_l);

	if (ret) {
		v4l2_err(client, "%s: fail to set exposure time\n", __func__);
		return -EINVAL;
	}

	/* Set Digital gain
	 * Controlled by AEC, can be manually controlled when disable AEC
	 * P0:0xb1 Auto_pregain
	 * P0:0xb2 Auto_postgain
	 */
	/* Set Analog Gain
	 *  Aec close: p0:0xb6 [0]
	 *  Set gain to P0:0x25
	 *  000: 1X
	 *  001: 1.4X
	 *  010: 2X
	 *  011: 2.8X
	 *  100: 4X
	 *  101: 5.6X
	 *  110: 8X
	 */

	ret = gc2155_write_reg(client, GC2155_8BIT, 0xb1, digital_gain);
	ret = gc2155_write_reg(client, GC2155_8BIT, 0x25, analog_gain);

	/*pr_info("%s, setting {Again, Dgain}= {0x%x, 0x%x}\n", __func__,
		analog_gain, digital_gain);*/

	if (ret) {
		v4l2_err(client, "%s: fail to set AnalogGainToWrite\n", __func__);
		return -EINVAL;
	}

	return ret;
}

static long gc2155_ioctl(struct v4l2_subdev *sd, unsigned int cmd, void *arg)
{
	switch (cmd) {
	case ATOMISP_IOC_S_EXPOSURE:
		/* pr_info("%s--: ATOMISP_IOC_S_EXPOSURE\n", __func__); */
		return gc2155_s_exposure(sd, arg);
	default:
		pr_info("%s-- not supported: 0x%x\n", __func__, cmd);
		return -EINVAL;
	}
	return 0;
}


/* This returns the exposure time being used. This should only be used
	for filling in EXIF data, not for actual image processing. */
static int gc2155_g_exposure(struct v4l2_subdev *sd, s32 *value)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	u16 coarse;
	u8 reg_val_h, reg_val_l;
	int ret;

	/* the fine integration time is currently not calculated */
	ret = gc2155_read_reg(client, GC2155_8BIT,
				REG_EXPO_COARSE_H, &reg_val_h);
	if (ret) {
		pr_info("%s: read exp fail\n", __func__);
		return ret;
	}
	coarse = ((u16)(reg_val_h & 0x1f)) << 8;

	ret = gc2155_read_reg(client, GC2155_8BIT,
				REG_EXPO_COARSE_L, &reg_val_l);
	if (ret) {
		pr_info("%s: read exp fail\n", __func__);
		return ret;
	}
	coarse |= reg_val_l;

	*value = coarse;
	return 0;
}

static struct gc2155_control gc2155_controls[] = {
/*
	{
		.qc = {
			.id = V4L2_CID_VFLIP,
			.type = V4L2_CTRL_TYPE_INTEGER,
			.name = "Image v-Flip",
			.minimum = 0,
			.maximum = 1,
			.step = 1,
			.default_value = 0,
		},
		.query = gc2155_g_vflip,
		.tweak = gc2155_t_vflip,
	},
	{
		.qc = {
			.id = V4L2_CID_HFLIP,
			.type = V4L2_CTRL_TYPE_INTEGER,
			.name = "Image h-Flip",
			.minimum = 0,
			.maximum = 1,
			.step = 1,
			.default_value = 0,
		},
		.query = gc2155_g_hflip,
		.tweak = gc2155_t_hflip,
	},
*/
	{
		.qc = {
			.id = V4L2_CID_FOCAL_ABSOLUTE,
			.type = V4L2_CTRL_TYPE_INTEGER,
			.name = "focal length",
			.minimum = GC2155_FOCAL_LENGTH_DEFAULT,
			.maximum = GC2155_FOCAL_LENGTH_DEFAULT,
			.step = 0x01,
			.default_value = GC2155_FOCAL_LENGTH_DEFAULT,
			.flags = 0,
		},
		.query = gc2155_g_focal,
	},
	{
		.qc = {
			.id = V4L2_CID_FNUMBER_ABSOLUTE,
			.type = V4L2_CTRL_TYPE_INTEGER,
			.name = "f-number",
			.minimum = GC2155_F_NUMBER_DEFAULT,
			.maximum = GC2155_F_NUMBER_DEFAULT,
			.step = 0x01,
			.default_value = GC2155_F_NUMBER_DEFAULT,
			.flags = 0,
		},
		.query = gc2155_g_fnumber,
	},
	{
		.qc = {
			.id = V4L2_CID_FNUMBER_RANGE,
			.type = V4L2_CTRL_TYPE_INTEGER,
			.name = "f-number range",
			.minimum = GC2155_F_NUMBER_RANGE,
			.maximum =  GC2155_F_NUMBER_RANGE,
			.step = 0x01,
			.default_value = GC2155_F_NUMBER_RANGE,
			.flags = 0,
		},
		.query = gc2155_g_fnumber_range,
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
		.tweak = gc2155_s_freq,
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
		.query = gc2155_g_exposure,
	},

};
#define N_CONTROLS (ARRAY_SIZE(gc2155_controls))

static struct gc2155_control *gc2155_find_control(__u32 id)
{
	int i;

	for (i = 0; i < N_CONTROLS; i++) {
		if (gc2155_controls[i].qc.id == id)
			return &gc2155_controls[i];
	}
	return NULL;
}

static int gc2155_detect(struct gc2155_device *dev, struct i2c_client *client)
{
	struct i2c_adapter *adapter = client->adapter;
	u8 chipid_h, chipid_l;

	pr_info("%s++\n", __func__);

	if (!i2c_check_functionality(adapter, I2C_FUNC_I2C)) {
		dev_err(&client->dev, "%s: i2c error", __func__);
		return -ENODEV;
	}

	gc2155_read_reg(client, GC2155_8BIT, REG_CHIP_ID_H, &chipid_h);

	gc2155_read_reg(client, GC2155_8BIT, REG_CHIP_ID_L, &chipid_l);
	pr_info("%s(%d): chipid_h= 0x%x; chipid_l= 0x%x\n",
		__func__, __LINE__, chipid_h, chipid_l);

	dev->real_model_id = ((chipid_h << 8) & 0xff00) | chipid_l;
	pr_info("%s(%d): detect module ID = %x\n",
			__func__, __LINE__, dev->real_model_id);

	if (dev->real_model_id != GC2155_MOD_ID) {
		ATD_gc2155_status = 0;
		dev_err(&client->dev, "%s: failed: client->addr = %x\n",
						__func__, client->addr);
		return -ENODEV;
	} else {
		ATD_gc2155_status = 1;
	}
	pr_info("%s--\n", __func__);
	return 0;
}

static int
gc2155_s_config(struct v4l2_subdev *sd, int irq, void *platform_data)
{
	struct gc2155_device *dev = to_gc2155_sensor(sd);
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	int ret;

	pr_info("%s++\n", __func__);

	if (NULL == platform_data)
		return -ENODEV;

	dev->platform_data =
		(struct camera_sensor_platform_data *)platform_data;

	if (dev->platform_data->platform_init) {
		ret = dev->platform_data->platform_init(client);
		if (ret) {
			v4l2_err(client, "gc2155 platform init err\n");
			return ret;
		}
	}
	power_down(sd);

	ret = gc2155_s_power(sd, 1);
	if (ret) {
		v4l2_err(client, "gc2155 power-up err");
		gc2155_s_power(sd, 0);
		return ret;
	}

	/* config & detect sensor */
	ret = gc2155_detect(dev, client);
	if (ret) {
		v4l2_err(client, "gc2155_detect err s_config.\n");
		goto fail_detect;
	}

	ret = dev->platform_data->csi_cfg(sd, 1);
	if (ret)
		goto fail_csi_cfg;

	ret = gc2155_set_suspend(sd);
	if (ret) {
		v4l2_err(client, "gc2155 suspend err");
		return ret;
	}

	ret = gc2155_s_power(sd, 0);
	if (ret) {
		v4l2_err(client, "gc2155 power down err");
		return ret;
	}

	return 0;

fail_csi_cfg:
	dev->platform_data->csi_cfg(sd, 0);
fail_detect:
	gc2155_s_power(sd, 0);
	dev_err(&client->dev, "sensor power-gating failed\n");
	return ret;
}

static int gc2155_queryctrl(struct v4l2_subdev *sd, struct v4l2_queryctrl *qc)
{
	struct gc2155_control *ctrl = gc2155_find_control(qc->id);
	pr_info("%s++\n", __func__);

	if (ctrl == NULL)
		return -EINVAL;
	*qc = ctrl->qc;
	return 0;
}


static int gc2155_s_parm(struct v4l2_subdev *sd,
			struct v4l2_streamparm *param)
{
	pr_info("%s++\n", __func__);
	return 0;
}

static int gc2155_g_ctrl(struct v4l2_subdev *sd, struct v4l2_control *ctrl)
{
	struct gc2155_control *octrl = gc2155_find_control(ctrl->id);
	int ret;

	if (octrl == NULL)
		return -EINVAL;

	ret = octrl->query(sd, &ctrl->value);
	if (ret < 0)
		return ret;

	return 0;
}

static int gc2155_s_ctrl(struct v4l2_subdev *sd, struct v4l2_control *ctrl)
{
	struct gc2155_control *octrl = gc2155_find_control(ctrl->id);
	int ret;

	pr_info("%s++\n", __func__);

	if (!octrl || !octrl->tweak)
		return -EINVAL;

	ret = octrl->tweak(sd, ctrl->value);
	if (ret < 0)
		return ret;

	return 0;
}

static int gc2155_s_stream(struct v4l2_subdev *sd, int enable)
{
	int ret;

	if (enable) {
		pr_info("gc2155_s_stream: Stream On\n");
		ret = gc2155_set_streaming(sd);
	} else {
		pr_info("gc2155_s_stream: Stream Off\n");
		ret = gc2155_set_suspend(sd);
	}
        msleep(10);
	return ret;
}

static int
gc2155_enum_framesizes(struct v4l2_subdev *sd, struct v4l2_frmsizeenum *fsize)
{
	unsigned int index = fsize->index;

	pr_info("%s++\n", __func__);

	if (index >= N_RES)
		return -EINVAL;

	fsize->type = V4L2_FRMSIZE_TYPE_DISCRETE;
	fsize->discrete.width = gc2155_res[index].width;
	fsize->discrete.height = gc2155_res[index].height;

	/* FIXME: Wrong way to know used mode */
	fsize->reserved[0] = gc2155_res[index].used;

	return 0;
}

static int gc2155_enum_frameintervals(struct v4l2_subdev *sd,
						struct v4l2_frmivalenum *fival)
{
	unsigned int index = fival->index;
	int i;

	pr_info("%s++\n", __func__);

	if (index >= N_RES) {
		pr_info("%s-- error\n", __func__);
		return -EINVAL;
	}
	/* find out the first equal or bigger size */
	for (i = 0; i < N_RES; i++) {
		if ((gc2155_res[i].width >= fival->width) &&
			(gc2155_res[i].height >= fival->height))
			break;
	}
	if (i == N_RES)
		i--;
	pr_info("%s(%d): fival: {wxh}={%dx%d}\n", __func__, __LINE__,
		fival->width, fival->height);
	index = i;
	pr_info("%s(%d): gc2155_res[%d]: {wxh}={%dx%d}\n", __func__, __LINE__,
		index, gc2155_res[index].width, gc2155_res[index].height);
	fival->type = V4L2_FRMIVAL_TYPE_DISCRETE;
	fival->discrete.numerator = 1;
	fival->discrete.denominator = gc2155_res[index].fps;
	pr_info("%s--\n", __func__);
	return 0;
}

static int
gc2155_g_chip_ident(struct v4l2_subdev *sd, struct v4l2_dbg_chip_ident *chip)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	pr_info("%s++\n", __func__);

	return v4l2_chip_ident_i2c_client(client, chip, V4L2_IDENT_GC2155, 0);
}

static int gc2155_enum_mbus_code(struct v4l2_subdev *sd,
				  struct v4l2_subdev_fh *fh,
				  struct v4l2_subdev_mbus_code_enum *code)
{
	pr_info("%s++\n", __func__);
	if (code->index) {
		pr_info("%s-- error\n", __func__);
		return -EINVAL;
	}
	code->code = V4L2_MBUS_FMT_SRGGB10_1X10;
	pr_info("%s--\n", __func__);
	return 0;
}

static int gc2155_enum_frame_size(struct v4l2_subdev *sd,
	struct v4l2_subdev_fh *fh,
	struct v4l2_subdev_frame_size_enum *fse)
{
	unsigned int index = fse->index;

	pr_info("%s++\n", __func__);

	if (index >= N_RES)
		return -EINVAL;

	fse->min_width = gc2155_res[index].width;
	fse->min_height = gc2155_res[index].height;
	fse->max_width = gc2155_res[index].width;
	fse->max_height = gc2155_res[index].height;

	return 0;
}

static struct v4l2_mbus_framefmt *
__gc2155_get_pad_format(struct gc2155_device *sensor,
			 struct v4l2_subdev_fh *fh, unsigned int pad,
			 enum v4l2_subdev_format_whence which)
{
	struct i2c_client *client = v4l2_get_subdevdata(&sensor->sd);
	pr_info("%s++\n", __func__);

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
gc2155_get_pad_format(struct v4l2_subdev *sd, struct v4l2_subdev_fh *fh,
				struct v4l2_subdev_format *fmt)
{
	struct gc2155_device *snr = to_gc2155_sensor(sd);
	struct v4l2_mbus_framefmt *format =
			__gc2155_get_pad_format(snr, fh, fmt->pad, fmt->which);
	pr_info("%s++\n", __func__);

	if (format == NULL)
		return -EINVAL;
	fmt->format = *format;

	return 0;
}

static int
gc2155_set_pad_format(struct v4l2_subdev *sd, struct v4l2_subdev_fh *fh,
				struct v4l2_subdev_format *fmt)
{
	struct gc2155_device *snr = to_gc2155_sensor(sd);
	struct v4l2_mbus_framefmt *format =
			__gc2155_get_pad_format(snr, fh, fmt->pad, fmt->which);
	pr_info("%s++\n", __func__);

	if (format == NULL)
		return -EINVAL;

	if (fmt->which == V4L2_SUBDEV_FORMAT_ACTIVE)
		snr->format = fmt->format;

	return 0;
}

static int gc2155_g_skip_frames(struct v4l2_subdev *sd, u32 *frames)
{
	int index;
	struct gc2155_device *snr = to_gc2155_sensor(sd);
	pr_info("%s++\n",__func__);

	if (frames == NULL)
		return -EINVAL;

	for (index = 0; index < N_RES; index++) {
		if (gc2155_res[index].res == snr->res)
			break;
	}

	if (index >= N_RES)
		return -EINVAL;

	*frames = gc2155_res[index].skip_frames;

	return 0;
}

static const struct v4l2_subdev_video_ops gc2155_video_ops = {
	.try_mbus_fmt = gc2155_try_mbus_fmt,
	.s_mbus_fmt = gc2155_set_mbus_fmt,
	.g_mbus_fmt = gc2155_get_mbus_fmt,
	.s_stream = gc2155_s_stream,
	.enum_framesizes = gc2155_enum_framesizes,
	.enum_frameintervals = gc2155_enum_frameintervals,
	.s_parm = gc2155_s_parm,
};

static struct v4l2_subdev_sensor_ops gc2155_sensor_ops = {
	.g_skip_frames	= gc2155_g_skip_frames,
};

static const struct v4l2_subdev_core_ops gc2155_core_ops = {
	.g_chip_ident = gc2155_g_chip_ident,
	.queryctrl = gc2155_queryctrl,
	.g_ctrl = gc2155_g_ctrl,
	.s_ctrl = gc2155_s_ctrl,
	.s_power = gc2155_s_power,
	.ioctl = gc2155_ioctl,
};

/* REVISIT: Do we need pad operations? */
static const struct v4l2_subdev_pad_ops gc2155_pad_ops = {
	.enum_mbus_code = gc2155_enum_mbus_code,
	.enum_frame_size = gc2155_enum_frame_size,
	.get_fmt = gc2155_get_pad_format,
	.set_fmt = gc2155_set_pad_format,
};

static const struct v4l2_subdev_ops gc2155_ops = {
	.core = &gc2155_core_ops,
	.video = &gc2155_video_ops,
	.pad = &gc2155_pad_ops,
	.sensor = &gc2155_sensor_ops,
};

static const struct media_entity_operations gc2155_entity_ops = {
	.link_setup = NULL,
};


static int gc2155_remove(struct i2c_client *client)
{
	struct gc2155_device *dev;
	struct v4l2_subdev *sd = i2c_get_clientdata(client);

	dev = container_of(sd, struct gc2155_device, sd);
	dev->platform_data->csi_cfg(sd, 0);
	if (dev->platform_data->platform_deinit)
		dev->platform_data->platform_deinit();
	v4l2_device_unregister_subdev(sd);
	media_entity_cleanup(&dev->sd.entity);
	kfree(dev);
	return 0;
}

/*++++++++++ dbgfs ++++++++++*/
static int dbg_set_gc2155_reg_open(struct inode *inode, struct file *file)
{
	file->private_data = inode->i_private;
	return 0;
}

static ssize_t dbg_set_gc2155_reg_write(
	struct file *file,
	const char __user *buf,
	size_t count,
	loff_t *ppos)
{
	char debug_buf[256];
	int cnt;
	unsigned int ofst = 0, reg_val = 0;
	u8 reg, val;
	int buf_count = 0;

	u8 i2c_buf[16];
	u8 *b_ptr = i2c_buf;
	int err = 0;

	/*
	pr_info("%s: buf=%p, count=%d, ppos=%p\n", __func__, buf, count, ppos);
	*/
	if (count > sizeof(debug_buf))
		return -EFAULT;
	if (copy_from_user(debug_buf, buf, count))
		return -EFAULT;
	debug_buf[count] = '\0';	/* end of string */

	cnt = sscanf(debug_buf, "%x %x", &ofst, &reg_val);

	reg = ofst & 0xFF;
	val = reg_val & 0xFF;


	*b_ptr++ = reg_val;
	buf_count++;
	pr_info("write [0x%x]= 0x%x\n", reg, val);
	err = gc2155_write_reg(g_client, GC2155_8BIT, reg, val);
	if (err)
		pr_info("dbg write error\n");


	return count;
}


static const struct file_operations dbg_set_gc2155_reg_fops = {
	.open		= dbg_set_gc2155_reg_open,
	.write		= dbg_set_gc2155_reg_write,
};

static int gc2155_dbgfs_init(void)
{
	struct dentry *debugfs_dir;
	debugfs_dir = debugfs_create_dir("camera0", NULL);
	debugfs_create_u32("camera_status",
		0644, debugfs_dir, &ATD_gc2155_status);

	return 0;
}
/*---------- dbgfs ----------*/

static int gc2155_probe(struct i2c_client *client,
				const struct i2c_device_id *id)
{
	struct gc2155_device *dev;
	int ret;
	//int project_id = PROJECT_ID_INVALID;

	pr_info("%s()++\n", __func__);

#if 0
	/* Project id check. */
	project_id = asustek_get_project_id();
	switch(project_id) {
		case PROJECT_ID_FE375CG:
		case PROJECT_ID_FE375CXG:
		break;

		default:
			pr_info("gc2155 is unsupported on this board(0x%x)\n",
				project_id);
		return -EINVAL;
	}
#endif

	/* Setup sensor configuration structure */
	dev = kzalloc(sizeof(*dev), GFP_KERNEL);
	if (!dev) {
		dev_err(&client->dev, "out of memory\n");
		return -ENOMEM;
	}

	v4l2_i2c_subdev_init(&dev->sd, client, &gc2155_ops);

	//Add for ATD read camera status+++
	dev->sensor_i2c_attribute.attrs = gc2155_attributes;

	// Register sysfs hooks
	ret = sysfs_create_group(&client->dev.kobj, &dev->sensor_i2c_attribute);
	if (ret) {
		dev_err(&client->dev, "Not able to create the sysfs\n");
		return ret;
	}
	//Add for ATD read camera status---

	if (client->dev.platform_data) {
		ret = gc2155_s_config(&dev->sd, client->irq,
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
	dev->format.code = V4L2_MBUS_FMT_SRGGB10_1X10;
	dev->sd.entity.type = MEDIA_ENT_T_V4L2_SUBDEV_SENSOR;


	/* REVISIT: Do we need media controller? */
	ret = media_entity_init(&dev->sd.entity, 1, &dev->pad, 0);
	if (ret) {
		gc2155_remove(client);
		return ret;
	}

	/* set res index to be invalid */
	dev->res = -1;

	/* dbgfs for ATD */
	g_client = client;
	gc2155_dbgfs_init(); //for second source

	pr_info("%s()--\n", __func__);
	return 0;
}

MODULE_DEVICE_TABLE(i2c, gc2155_id);

static struct i2c_driver gc2155_driver = {
	.driver = {
		.owner = THIS_MODULE,
		.name = "gc2155"
	},
	.probe = gc2155_probe,
	.remove = gc2155_remove,
	.id_table = gc2155_id,
};

static __init int init_gc2155(void)
{
	return i2c_add_driver(&gc2155_driver);
}

static __exit void exit_gc2155(void)
{
	i2c_del_driver(&gc2155_driver);
}

module_init(init_gc2155);
module_exit(exit_gc2155);

MODULE_AUTHOR("Adogu Huang <adogu_huang@asus.com>");
MODULE_LICENSE("GPL");
