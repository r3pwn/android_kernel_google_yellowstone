/*
 * ov7251.c - ov7251 sensor driver
 *
 * Copyright (c) 2014, NVIDIA CORPORATION, All Rights Reserved.
 *
 * Contributors:
 *  Jerry Chang <jerchang@nvidia.com>
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms and conditions of the GNU General Public License,
 * version 2, as published by the Free Software Foundation.
 *
 * This program is distributed in the hope it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <linux/delay.h>
#include <linux/fs.h>
#include <linux/i2c.h>
#include <linux/clk.h>
#include <linux/miscdevice.h>
#include <linux/slab.h>
#include <linux/uaccess.h>
#include <media/ov7251.h>
#include <linux/regulator/consumer.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/of_gpio.h>
#include <media/nvc.h>
#include <linux/gpio.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/debugfs.h>
#include <../../../../arch/arm/mach-tegra/common.h>
#include <linux/proc_fs.h>
#include <linux/seq_file.h>

#include <misc/sh-ldisc.h>

#define SIZEOF_I2C_TRANSBUF 32

struct ov7251_reg {
	u16 addr;
	u16 val;
};

struct ov7251_info {
	struct miscdevice			miscdev_info;
	struct ov7251_power_rail	power;
	struct i2c_client			*i2c_client;
	struct clk					*mclk;
	struct ov7251_platform_data *pdata;
	struct ov7251_sensordata	sensor_data;
	int mode;
	u8 i2c_trans_buf[SIZEOF_I2C_TRANSBUF];
	atomic_t in_use;
#ifdef CONFIG_DEBUG_FS
	struct dentry			*debugfs_root;
	u32						debug_i2c_offset;
#endif
};

#define OV7251_TABLE_WAIT_MS 0
#define OV7251_WAIT_MS 10
#define OV7251_TABLE_END 1
#define OV7251_MAX_RETRIES 3

static struct ov7251_reg mode_640x480[] = {
	{0x0103, 0x01},
	{0x0100, 0x00},
	{0x3005, 0x02},
	{0x3012, 0xc0},
	{0x3013, 0xd2},
	{0x3014, 0x04},
	{0x3016, 0xf0},
	{0x3017, 0xf0},
	{0x3018, 0xf0},
	{0x301a, 0xf0},
	{0x301b, 0xf0},
	{0x301c, 0xf0},
	{0x3023, 0x07},
	{0x3037, 0xf0},
	{0x3098, 0x04},
	{0x3099, 0x50}, /*28*/
	{0x309a, 0x05},
	{0x309b, 0x04},
	{0x30b0, 0x0a},
	{0x30b1, 0x01},
	{0x30b3, 0x32}, /*0x64*/
	{0x30b4, 0x05}, /*0x03*/
	{0x30b5, 0x05},
	{0x3106, 0x12},
	{0x3500, 0x00},
	{0x3501, 0x1f},
	{0x3502, 0x80},
	{0x3503, 0x07},
	{0x3509, 0x10},
	{0x350b, 0x10},
	{0x3600, 0x1c},
	{0x3602, 0x62},
	{0x3620, 0xb7},
	{0x3622, 0x04},
	{0x3626, 0x21},
	{0x3627, 0x30},
	{0x3634, 0x41},
	{0x3636, 0x00},
	{0x3662, 0x01},
	{0x3664, 0xf0},
	{0x3669, 0x1a},
	{0x366a, 0x00},
	{0x366b, 0x50},
	{0x3705, 0xc1},
	{0x3709, 0x40},
	{0x373c, 0x08},
	{0x3742, 0x00},
	{0x3788, 0x00},
	{0x37a8, 0x01},
	{0x37a9, 0xc0},
	{0x3800, 0x00},
	{0x3801, 0x04},
	{0x3802, 0x00},
	{0x3803, 0x04},
	{0x3804, 0x02},
	{0x3805, 0x8b},
	{0x3806, 0x01},
	{0x3807, 0xeb},
	{0x3808, 0x02},
	{0x3809, 0x80},
	{0x380a, 0x01},
	{0x380b, 0xe0},
	{0x380c, 0x03},
	{0x380d, 0xa0},
	{0x380e, 0x06},
	{0x380f, 0xbc},
	{0x3810, 0x00},
	{0x3811, 0x04},
	{0x3812, 0x00},
	{0x3813, 0x05},
	{0x3814, 0x11},
	{0x3815, 0x11},
	{0x3820, 0x00},
	{0x3821, 0x00},
	{0x382f, 0xc4},
	{0x3832, 0xff},
	{0x3833, 0xff},
	{0x3834, 0x00},
	{0x3835, 0x05},
	{0x3837, 0x00},
	{0x3b80, 0x00},
	{0x3b81, 0xa5},
	{0x3b82, 0x10},
	{0x3b83, 0x00},
	{0x3b84, 0x08},
	{0x3b85, 0x00},
	{0x3b86, 0x01},
	{0x3b87, 0x00},
	{0x3b88, 0x00},
	{0x3b89, 0x00},
	{0x3b8a, 0x00},
	{0x3b8b, 0x05},
	{0x3b8c, 0x00},
	{0x3b8d, 0x00},
	{0x3b8e, 0x00},
	{0x3b8f, 0x1a},
	{0x3b94, 0x05},
	{0x3b95, 0xf2},
	{0x3b96, 0x40},
	{0x3c00, 0x89},
	{0x3c01, 0xab},
	{0x3c02, 0x01},
	{0x3c03, 0x00},
	{0x3c04, 0x00},
	{0x3c05, 0x03},
	{0x3c06, 0x00},
	{0x3c07, 0x05},
	{0x3c0c, 0x00},
	{0x3c0d, 0x00},
	{0x3c0e, 0x00},
	{0x3c0f, 0x00},
	{0x4001, 0xc2},
	{0x4004, 0x04},
	{0x4005, 0x20},
	{0x404e, 0x01},
	{0x4300, 0xff},
	{0x4301, 0x00},
        {0x4311, 0x80}, /* Restate default VSYNC_WIDTH_H */
        {0x4312, 0x00}, /* Restate default VSYNC_WIDTH_L */
        {0x4314, 0x00}, /* Restate default VSYNC_DELAY1 [23:16] */
        {0x4315, 0x00}, /* Default was 0x01 VSYNC_DELAY2 [15:8] */
        {0x4316, 0x00}, /* Restate default VSYNC_DELAY3 [7:0] */
	{0x4600, 0x00},
	{0x4601, 0x4e},
	{0x4800, 0x64}, /*bit[5]=0 as MIPI continuous clock*/
	{0x4801, 0x0f},
	{0x4806, 0x0f},
	{0x4819, 0xaa},
	{0x4823, 0x3e},
	{0x4837, 0x32}, /*0x19*/
	{0x4a0d, 0x00},
	{0x5000, 0x85},
	{0x5001, 0x80},
	{OV7251_TABLE_WAIT_MS, OV7251_WAIT_MS},
	{OV7251_TABLE_END, 0x00}
};

enum {
	OV7251_MODE_640x480,
};

static struct ov7251_reg *mode_table[] = {
	[OV7251_MODE_640x480] = mode_640x480,
};

#define OV7251_ENTER_GROUP_HOLD(group_hold) \
	do {	\
		if (group_hold) {   \
			reg_list[offset].addr = 0x3208; \
			reg_list[offset].val = 0x00;\
			offset++;  \
		}   \
	} while (0)

#define OV7251_LEAVE_GROUP_HOLD(group_hold) \
	do {	\
		if (group_hold) {   \
			reg_list[offset].addr = 0x3208; \
			reg_list[offset].val = 0x10;\
			offset++; \
			reg_list[offset].addr = 0x3208; \
			reg_list[offset].val = 0xA0;\
			offset++; \
		} \
	} while (0)

static void ov7251_mclk_disable(struct ov7251_info *info)
{
	dev_dbg(&info->i2c_client->dev, "%s: disable MCLK\n", __func__);
	clk_disable_unprepare(info->mclk);
}

static int ov7251_mclk_enable(struct ov7251_info *info)
{
	int err;
	unsigned long mclk_init_rate = 24000000;

	dev_dbg(&info->i2c_client->dev, "%s: enable MCLK with %lu Hz\n",
		__func__, mclk_init_rate);

	err = clk_set_rate(info->mclk, mclk_init_rate);
	if (!err)
		err = clk_prepare_enable(info->mclk);
	return err;
}

static inline int ov7251_get_frame_length_regs(struct ov7251_reg *regs,
						u32 frame_length)
{
	regs->addr = 0x380e;
	regs->val = (frame_length >> 8) & 0xff;
	(regs + 1)->addr = 0x380f;
	(regs + 1)->val = (frame_length) & 0xff;
	return 2;
}

static inline int ov7251_get_coarse_time_regs(struct ov7251_reg *regs,
					       u32 coarse_time)
{
	regs->addr = 0x3500;
	regs->val = (coarse_time >> 12) & 0xff;
	(regs + 1)->addr = 0x3501;
	(regs + 1)->val = (coarse_time >> 4) & 0xff;
	(regs + 2)->addr = 0x3502;
	(regs + 2)->val = (coarse_time & 0xf) << 4;
	return 3;
}

static inline int ov7251_get_gain_reg(struct ov7251_reg *regs, u16 gain)
{
	regs->addr = 0x350b;
	regs->val = gain & 0xff;
	return 1;
}

static int ov7251_read_reg(struct i2c_client *client, u16 addr, u8 *val)
{
	int err;
	struct i2c_msg msg[2];
	unsigned char data[3];

	if (!client->adapter)
		return -ENODEV;

	msg[0].addr = client->addr;
	msg[0].flags = 0;
	msg[0].len = 2;
	msg[0].buf = data;

	/* high byte goes out first */
	data[0] = (u8) (addr >> 8);
	data[1] = (u8) (addr & 0xff);

	msg[1].addr = client->addr;
	msg[1].flags = I2C_M_RD;
	msg[1].len = 1;
	msg[1].buf = data + 2;

	err = i2c_transfer(client->adapter, msg, 2);

	if (err != 2)

		return -EINVAL;

	*val = data[2];

	return 0;
}

static int ov7251_write_reg(struct i2c_client *client, u16 addr, u8 val)
{
	int err;
	struct i2c_msg msg;
	unsigned char data[3];
	int retry = 0;

	if (!client->adapter)
		return -ENODEV;
	dev_info(&client->dev, "%s\n", __func__);

	data[0] = (u8) (addr >> 8);
	data[1] = (u8) (addr & 0xff);
	data[2] = (u8) (val & 0xff);

	msg.addr = client->addr;
	msg.flags = 0;
	msg.len = 3;
	msg.buf = data;

	do {
		err = i2c_transfer(client->adapter, &msg, 1);
		if (err == 1)
			return 0;
		retry++;
		pr_err("ov7251: i2c transfer failed, retrying %x %x\n",
			addr, val);

		usleep_range(3000, 3100);
	} while (retry <= OV7251_MAX_RETRIES);

	return err;
}

static int ov7251_write_bulk_reg(struct i2c_client *client, u8 *data, int len)
{
	int err;
	struct i2c_msg msg;

	if (!client->adapter)
		return -ENODEV;

	msg.addr = client->addr;
	msg.flags = 0;
	msg.len = len;
	msg.buf = data;

	err = i2c_transfer(client->adapter, &msg, 1);
	if (err == 1)
		return 0;

	pr_err("ov7251: i2c bulk transfer failed at %x\n",
		(int)data[0] << 8 | data[1]);

	return err;
}

static int ov7251_write_table(struct ov7251_info *info,
			      const struct ov7251_reg table[],
			      const struct ov7251_reg override_list[],
			      int num_override_regs)
{
	int err;
	const struct ov7251_reg *next, *n_next;
	u8 *b_ptr = info->i2c_trans_buf;
	unsigned int buf_filled = 0;
	unsigned int i;
	u16 val;

	for (next = table; next->addr != OV7251_TABLE_END; next++) {
		if (next->addr == OV7251_TABLE_WAIT_MS) {
			msleep(next->val);
			continue;
		}

		val = next->val;
		/* When an override list is passed in, replace the reg */
		/* value to write if the reg is in the list            */
		if (override_list) {
			for (i = 0; i < num_override_regs; i++) {
				if (next->addr == override_list[i].addr) {
					val = override_list[i].val;
					break;
				}
			}
		}

		if (!buf_filled) {
			b_ptr = info->i2c_trans_buf;
			*b_ptr++ = next->addr >> 8;
			*b_ptr++ = next->addr & 0xff;
			buf_filled = 2;
		}
		*b_ptr++ = val;
		buf_filled++;

		n_next = next + 1;
		if (n_next->addr != OV7251_TABLE_END &&
			n_next->addr != OV7251_TABLE_WAIT_MS &&
			buf_filled < SIZEOF_I2C_TRANSBUF &&
			n_next->addr == next->addr + 1) {
			continue;
		}

		err = ov7251_write_bulk_reg(info->i2c_client,
			info->i2c_trans_buf, buf_filled);
		if (err)
			return err;
		buf_filled = 0;
	}
	return 0;
}

static int ov7251_set_mode(struct ov7251_info *info, struct ov7251_mode *mode)
{
	int sensor_mode;
	int err;
	struct ov7251_reg reg_list[6];
	int offset = 0;

	if (mode->xres == 640 && mode->yres == 480) {
		sensor_mode = OV7251_MODE_640x480;
		pr_info("ov7251_set_mode 640x480\n");
	} else {
		pr_err("There is no this resolution no support %dX%d!!",
			mode->xres, mode->yres);
		return 1;
	}

	/* get a list of override regs for the asking frame length, */
	/* coarse integration time, and gain.                       */
	offset += ov7251_get_frame_length_regs(reg_list + offset,
					       mode->frame_length);
	offset += ov7251_get_coarse_time_regs(reg_list + offset,
					      mode->coarse_time);
	offset += ov7251_get_gain_reg(reg_list + offset, mode->gain);

	err = ov7251_write_table(info, mode_table[sensor_mode], reg_list,
				 offset);
	if (err)
		return err;

	info->mode = sensor_mode;

	return 0;
}

static int ov7251_set_frame_length(struct ov7251_info *info, u32 frame_length)
{
	int ret;
	struct ov7251_reg reg_list[2];
	u8 *b_ptr = info->i2c_trans_buf;

	ov7251_get_frame_length_regs(reg_list, frame_length);

	*b_ptr++ = reg_list[0].addr >> 8;
	*b_ptr++ = reg_list[0].addr & 0xff;
	*b_ptr++ = reg_list[0].val & 0xff;
	*b_ptr++ = reg_list[1].val & 0xff;
	ret = ov7251_write_bulk_reg(info->i2c_client, info->i2c_trans_buf, 4);

	return ret;
}

static int ov7251_set_coarse_time(struct ov7251_info *info, u32 coarse_time)
{
	int ret;
	struct ov7251_reg reg_list[3];
	u8 *b_ptr = info->i2c_trans_buf;

	dev_info(&info->i2c_client->dev, "coarse_time 0x%x\n", coarse_time);
	ov7251_get_coarse_time_regs(reg_list, coarse_time);

	*b_ptr++ = reg_list[0].addr >> 8;
	*b_ptr++ = reg_list[0].addr & 0xff;
	*b_ptr++ = reg_list[0].val & 0xff;
	*b_ptr++ = reg_list[1].val & 0xff;
	*b_ptr++ = reg_list[2].val & 0xff;

	ret = ov7251_write_bulk_reg(info->i2c_client, info->i2c_trans_buf, 5);
	return ret;

}

static int ov7251_set_gain(struct ov7251_info *info, u16 gain)
{
	int ret;
	u8 *b_ptr = info->i2c_trans_buf;

	struct ov7251_reg reg_list[1];
	ov7251_get_gain_reg(reg_list, gain);

	*b_ptr++ = reg_list[0].addr >> 8;
	*b_ptr++ = reg_list[0].addr & 0xff;
	*b_ptr++ = reg_list[0].val & 0xff;
	ret = ov7251_write_bulk_reg(info->i2c_client, info->i2c_trans_buf, 3);
	return ret;
}

static int ov7251_set_group_hold(struct ov7251_info *info,
			struct ov7251_grouphold *gh)
{
	int err = 0;
	struct ov7251_reg reg_list[12];
	int offset = 0;
	bool group_hold = true;

	OV7251_ENTER_GROUP_HOLD(group_hold);
	if (gh->gain_enable)
		offset += ov7251_get_gain_reg(reg_list + offset,
					  gh->gain);
	if (gh->frame_length_enable)
		offset += ov7251_get_frame_length_regs(reg_list + offset,
						  gh->frame_length);
	if (gh->coarse_time_enable)
		offset += ov7251_get_coarse_time_regs(reg_list + offset,
			gh->coarse_time);
	OV7251_LEAVE_GROUP_HOLD(group_hold);

	reg_list[offset].addr = OV7251_TABLE_END;
	err |= ov7251_write_table(info, reg_list, NULL, 0);

	return err;
}

static int ov7251_get_sensor_id(struct ov7251_info *info)
{
	int ret = 0;
	int i;
	u8  bak;

	dev_info(&info->i2c_client->dev, "%s\n", __func__);
	if (info->sensor_data.fuse_id_size)
		return 0;

	/* select bank 31 */
	ov7251_write_reg(info->i2c_client, 0x3d84, 31);
	for (i = 0; i < 8; i++) {
		ret |= ov7251_read_reg(info->i2c_client, 0x300A + i, &bak);
		info->sensor_data.fuse_id[i] = bak;
	}

	if (!ret)
		info->sensor_data.fuse_id_size = 8;

	return ret;
}

static int ov7251_get_status(struct ov7251_info *info, u8 *status)
{
	int err;

	*status = 0;
	err = ov7251_read_reg(info->i2c_client, 0x002, status);
	return err;
}

static long ov7251_ioctl(struct file *file,
			 unsigned int cmd, unsigned long arg)
{
	int err;
	struct ov7251_info *info = file->private_data;

	switch (cmd) {
	case OV7251_IOCTL_SET_MODE:
	{
		struct ov7251_mode mode;
		if (copy_from_user(&mode,
				   (const void __user *)arg,
				   sizeof(struct ov7251_mode))) {
			return -EFAULT;
		}

		return ov7251_set_mode(info, &mode);
	}
	case OV7251_IOCTL_SET_FRAME_LENGTH:
		return ov7251_set_frame_length(info, (u32)arg);
	case OV7251_IOCTL_SET_COARSE_TIME:
		return ov7251_set_coarse_time(info, (u32)arg);
	case OV7251_IOCTL_SET_GAIN:
		return ov7251_set_gain(info, (u16)arg);
	case OV7251_IOCTL_SET_GROUP_HOLD:
	{
		struct ov7251_grouphold gh;
		if (copy_from_user(&gh,
				(const void __user *)arg,
				sizeof(struct ov7251_grouphold))) {
			return -EFAULT;
		}
		return ov7251_set_group_hold(info, &gh);
	}
	case OV7251_IOCTL_GET_STATUS:
	{
		u8 status;

		err = ov7251_get_status(info, &status);
		if (err)
			return err;
		if (copy_to_user((void __user *)arg, &status,
				 2)) {
			return -EFAULT;
		}
		return 0;
	}
	case OV7251_IOCTL_GET_FUSEID:
	{
		err = ov7251_get_sensor_id(info);
		if (err)
			return err;
		if (copy_to_user((void __user *)arg,
				&info->sensor_data,
				sizeof(struct ov7251_sensordata))) {
			return -EFAULT;
		}
		return 0;
	}
	case OV7251_IOCTL_START_STREAM:
	{
		return ov7251_write_reg(info->i2c_client, 0x100, 1);
	}
	case OV7251_IOCTL_GET_FRAMEINFO:
	{
		struct frameinfo info;
		int ret;

		if (copy_from_user(&info, (const void __user *) arg,
				   sizeof(info))) {
			pr_err("cannot get frameinfo argument from userland");
			return -EFAULT;
		}

		ret = sh_get_frameinfo(DEVICE_FISHEYE_CAMERA, &info);

		if (copy_to_user((void __user *) arg, &info, sizeof(info))) {
			pr_err("cannot copy frameinfo back to userspace");
			return -EFAULT;
		}

		return ret;
	}
	default:
		return -EINVAL;
	}

	return 0;
}

static int ov7251_open(struct inode *inode, struct file *file)
{
	struct miscdevice	*miscdev = file->private_data;
	struct ov7251_info	*info;

	info = container_of(miscdev, struct ov7251_info, miscdev_info);
	/* check if the device is in use */
	if (atomic_xchg(&info->in_use, 1)) {
		dev_info(&info->i2c_client->dev, "%s:BUSY!\n", __func__);
		return -EBUSY;
	}
	file->private_data = info;

	if (info->pdata && info->pdata->power_on) {
		ov7251_mclk_enable(info);
		info->pdata->power_on(&info->power, info->pdata->sh_device);
	} else {
		dev_err(&info->i2c_client->dev,
			"%s:no valid power_on function.\n", __func__);
		return -EEXIST;
	}

	return 0;
}

int ov7251_release(struct inode *inode, struct file *file)
{
	struct ov7251_info *info = file->private_data;

	if (info->pdata && info->pdata->power_off) {
		ov7251_mclk_disable(info);
		info->pdata->power_off(&info->power, info->pdata->sh_device);
	}
	file->private_data = NULL;

	/* warn if device is already released */
	WARN_ON(!atomic_xchg(&info->in_use, 0));
	return 0;
}


static int ov7251_power_put(struct ov7251_power_rail *pw)
{
	return 0;
}

static int __maybe_unused ov7251_regulator_get(struct ov7251_info *info,
	struct regulator **vreg, char vreg_name[])
{
	struct regulator *reg = NULL;
	int err = 0;

	reg = regulator_get(&info->i2c_client->dev, vreg_name);
	if (unlikely(IS_ERR(reg))) {
		dev_err(&info->i2c_client->dev, "%s %s ERR: %d\n",
			__func__, vreg_name, (int)reg);
		err = PTR_ERR(reg);
		reg = NULL;
	} else
		dev_dbg(&info->i2c_client->dev, "%s: %s\n",
			__func__, vreg_name);

	*vreg = reg;
	return err;
}

static int ov7251_power_get(struct ov7251_info *info)
{
	return 0;
}

static const struct file_operations ov7251_fileops = {
	.owner = THIS_MODULE,
	.open = ov7251_open,
	.unlocked_ioctl = ov7251_ioctl,
	.release = ov7251_release,
};

static struct miscdevice ov7251_device = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "ov7251",
	.fops = &ov7251_fileops,
};

#ifdef CONFIG_DEBUG_FS
static int ov7251_stats_show(struct seq_file *s, void *data)
{
	struct ov7251_info *info = (struct ov7251_info *)(s->private);

	seq_printf(s, "%-20s : %-20s\n", "Name", "ov7251-debugfs-testing");
	seq_printf(s, "%-20s : 0x%X\n", "Current i2c-offset Addr",
			info->debug_i2c_offset);
	return 0;
}

static int ov7251_stats_open(struct inode *inode, struct file *file)
{
	return single_open(file, ov7251_stats_show, inode->i_private);
}

static const struct file_operations ov7251_stats_fops = {
	.open       = ov7251_stats_open,
	.read       = seq_read,
	.llseek     = seq_lseek,
	.release    = single_release,
};

static int debug_i2c_offset_w(void *data, u64 val)
{
	struct ov7251_info *info = (struct ov7251_info *)(data);
	dev_info(&info->i2c_client->dev,
			"ov7251:%s setting i2c offset to 0x%X\n",
			__func__, (u32)val);
	info->debug_i2c_offset = (u32)val;
	dev_info(&info->i2c_client->dev,
			"ov7251:%s new i2c offset is 0x%X\n", __func__,
			info->debug_i2c_offset);
	return 0;
}

static int debug_i2c_offset_r(void *data, u64 *val)
{
	struct ov7251_info *info = (struct ov7251_info *)(data);
	*val = (u64)info->debug_i2c_offset;
	dev_info(&info->i2c_client->dev,
			"ov7251:%s reading i2c offset is 0x%X\n", __func__,
			info->debug_i2c_offset);
	return 0;
}

static int debug_i2c_read(void *data, u64 *val)
{
	struct ov7251_info *info = (struct ov7251_info *)(data);
	u8 temp1 = 0;
	u8 temp2 = 0;
	dev_info(&info->i2c_client->dev,
			"ov7251:%s reading offset 0x%X\n", __func__,
			info->debug_i2c_offset);
	if (ov7251_read_reg(info->i2c_client,
				info->debug_i2c_offset, &temp1)
		|| ov7251_read_reg(info->i2c_client,
			info->debug_i2c_offset+1, &temp2)) {
		dev_err(&info->i2c_client->dev,
				"ov7251:%s failed\n", __func__);
		return -EIO;
	}
	dev_info(&info->i2c_client->dev,
			"ov7251:%s read value is 0x%04X\n", __func__,
			temp1<<8 | temp2);
	*val = (u64)(temp1<<8 | temp2);
	return 0;
}

static int debug_i2c_write(void *data, u64 val)
{
	struct ov7251_info *info = (struct ov7251_info *)(data);
	dev_info(&info->i2c_client->dev,
			"ov7251:%s writing 0x%X to offset 0x%X\n", __func__,
			(u8)val, info->debug_i2c_offset);
	if (ov7251_write_reg(info->i2c_client,
				info->debug_i2c_offset, (u8)val)) {
		dev_err(&info->i2c_client->dev, "ov7251:%s failed\n", __func__);
		return -EIO;
	}
	return 0;
}

DEFINE_SIMPLE_ATTRIBUTE(i2c_offset_fops, debug_i2c_offset_r,
		debug_i2c_offset_w, "0x%llx\n");
DEFINE_SIMPLE_ATTRIBUTE(i2c_read_fops, debug_i2c_read,
		/*debug_i2c_dummy_w*/ NULL, "0x%llx\n");
DEFINE_SIMPLE_ATTRIBUTE(i2c_write_fops, /*debug_i2c_dummy_r*/NULL,
		debug_i2c_write, "0x%llx\n");

static int ov7251_debug_init(struct ov7251_info *info)
{
	dev_dbg(&info->i2c_client->dev, "%s", __func__);

	info->debugfs_root = debugfs_create_dir(ov7251_device.name, NULL);

	if (!info->debugfs_root)
		goto err_out;

	if (!debugfs_create_file("stats", S_IRUGO,
			info->debugfs_root, info, &ov7251_stats_fops))
		goto err_out;

	if (!debugfs_create_file("offset", S_IRUGO | S_IWUSR,
			info->debugfs_root, info, &i2c_offset_fops))
		goto err_out;

	if (!debugfs_create_file("read", S_IRUGO,
			info->debugfs_root, info, &i2c_read_fops))
		goto err_out;

	if (!debugfs_create_file("write", S_IWUSR,
			info->debugfs_root, info, &i2c_write_fops))
		goto err_out;

	return 0;

err_out:
	dev_err(&info->i2c_client->dev, "ERROR:%s failed", __func__);
	debugfs_remove_recursive(info->debugfs_root);
	return -ENOMEM;
}
#endif

static int ov7251_probe(struct i2c_client *client,
			const struct i2c_device_id *id)
{
	struct ov7251_info *info;
	const char *mclk_name;
	int err = 0;

	pr_info("ov7251: probing sensor.\n");

	info = devm_kzalloc(&client->dev,
		sizeof(struct ov7251_info), GFP_KERNEL);
	if (!info) {
		pr_err("ov7251:%s:Unable to allocate memory!\n", __func__);
		return -ENOMEM;
	}

	info->pdata = client->dev.platform_data;
	info->i2c_client = client;
	atomic_set(&info->in_use, 0);
	info->mode = -1;

	mclk_name = info->pdata->mclk_name ?
		    info->pdata->mclk_name : "default_mclk";
	info->mclk = devm_clk_get(&client->dev, mclk_name);
	if (IS_ERR(info->mclk)) {
		dev_err(&client->dev, "%s: unable to get clock %s\n",
			__func__, mclk_name);
		return PTR_ERR(info->mclk);
	}

	ov7251_power_get(info);

	memcpy(&info->miscdev_info,
		&ov7251_device,
		sizeof(struct miscdevice));

	err = misc_register(&info->miscdev_info);
	if (err) {
		ov7251_power_put(&info->power);
		pr_err("ov7251:%s:Unable to register misc device!\n",
		__func__);
	}

	i2c_set_clientdata(client, info);

#ifdef CONFIG_DEBUG_FS
	ov7251_debug_init(info);
#endif

	return err;
}

static int ov7251_remove(struct i2c_client *client)
{
	struct ov7251_info *info = i2c_get_clientdata(client);

	ov7251_power_put(&info->power);
	misc_deregister(&ov7251_device);
#ifdef CONFIG_DEBUG_FS
	debugfs_remove_recursive(info->debugfs_root);
#endif
	return 0;
}

static const struct i2c_device_id ov7251_id[] = {
	{ "ov7251", 0 },
	{ },
};

MODULE_DEVICE_TABLE(i2c, ov7251_id);

static struct i2c_driver ov7251_i2c_driver = {
	.driver = {
		.name = "ov7251",
		.owner = THIS_MODULE,
	},
	.probe = ov7251_probe,
	.remove = ov7251_remove,
	.id_table = ov7251_id,
};

static int __init ov7251_init(void)
{
	pr_info("ov7251 sensor driver loading\n");
	return i2c_add_driver(&ov7251_i2c_driver);
}

static void __exit ov7251_exit(void)
{
	i2c_del_driver(&ov7251_i2c_driver);
}

module_init(ov7251_init);
module_exit(ov7251_exit);
MODULE_LICENSE("GPL v2");
