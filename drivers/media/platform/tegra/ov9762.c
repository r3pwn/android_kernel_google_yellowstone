/*
 * ov9762.c - ov9762 sensor driver
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
#include <media/ov9762.h>
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

struct ov9762_reg {
	u16 addr;
	u16 val;
};

struct ov9762_info {
	struct miscdevice			miscdev_info;
	struct ov9762_power_rail	power;
	struct i2c_client			*i2c_client;
	struct clk					*mclk;
	struct ov9762_platform_data *pdata;
	struct ov9762_sensordata	sensor_data;
	int mode;
	u8 i2c_trans_buf[SIZEOF_I2C_TRANSBUF];
	atomic_t in_use;
#ifdef CONFIG_DEBUG_FS
	struct dentry			*debugfs_root;
	u32						debug_i2c_offset;
#endif
};

#define OV9762_TABLE_WAIT_MS 0
#define OV9762_WAIT_MS 10
#define OV9762_TABLE_END 1
#define OV9762_MAX_RETRIES 3

static struct ov9762_reg mode_1384x968[] = {
	{0x0103, 0x01},
	{0x0101, 0x01},
	{0x0340, 0x04}, /* VTS */
	{0x0341, 0x30}, /* VTS */
	{0x0342, 0x07}, /* HTS */
	{0x0343, 0x48}, /* HTS */
	{0x0344, 0x00},
	{0x0345, 0x04},
	{0x0346, 0x00},
	{0x0347, 0x04},
	{0x0348, 0x05},
	{0x0349, 0x73},
	{0x034a, 0x03},
	{0x034b, 0xd3},
	{0x034c, 0x05}, /* x_output_size */
	{0x034d, 0x68}, /* x_output_size */
	{0x034e, 0x03}, /* y_output_size */
	{0x034f, 0xc8}, /* y_output_size */
	{0x0381, 0x01},
	{0x0383, 0x01},
	{0x0385, 0x01},
	{0x0387, 0x01},
	{0x3012, 0x08}, /* MIPI control */
	{0x3013, 0x90}, /* MIPI control */
	{0x3014, 0xc4}, /* MIPI control */
	{0x301f, 0x83}, /* MIPI control */
	{0x3081, 0x02}, /* PLL control */
	{0x3090, 0x01}, /* PLL control */
	{0x3091, 0x27}, /* PLL control */
	{0x3094, 0x00}, /* PLL control */
	{0x3098, 0x03}, /* PLL control */
	{0x3099, 0x1e}, /* PLL control */
	{0x309a, 0x04}, /* PLL control */
	{0x30b3, 0x99}, /* PLL control */
	{0x30b4, 0x02}, /* PLL control */
	{0x3503, 0x07}, /* Gain/Exposure control */
	{0x350a, 0x00},
	{0x350b, 0x10},
	{0x3620, 0x34},
	{0x3622, 0x2b},
	{0x3626, 0x34},
	{0x3631, 0xd5},
	{0x3660, 0x80},
	{0x3662, 0x14},
	{0x3663, 0xf0},
	{0x3667, 0xa0},
	{0x3680, 0xf4},
	{0x3704, 0x40},
	{0x3705, 0x1e},
	{0x3708, 0x63},
	{0x3709, 0x12},
	{0x3719, 0x17},
	{0x3739, 0x55},
	{0x373c, 0x0c},
	{0x3781, 0x82},
	{0x378f, 0x55},
	{0x3810, 0x00},
	{0x3811, 0x04},
	{0x3812, 0x00},
	{0x3813, 0x04},
	{0x3820, 0x00},
	{0x3821, 0x00},
	{0x3841, 0xc4},
	{0x3842, 0xff},
	{0x3843, 0xff},
	{0x3a02, 0x21}, /* AEC control */
	{0x3a03, 0x60}, /* AEC control */
	{0x3b00, 0x00},
	{0x4000, 0x43}, /* BLC control */
	{0x4004, 0xc8}, /* BLC control */
	{0x4008, 0x42},
	{0x4009, 0x10},
	{0x404f, 0x8f},
	{0x4307, 0x30},
	{0x4513, 0x60},
	{0x4520, 0xb0},
	{0x4582, 0x00},
	{0x4602, 0x00},
	{0x4603, 0x00},
	{0x4605, 0x00},
	{0x4608, 0x02},
	{0x4837, 0x15}, /* MIPI control */
	{0x4d07, 0x04}, /* Temperature sensor */
	{0x5000, 0x30}, /* ISP control */
	{0x5103, 0x17}, /* ISP control */
	{0x5780, 0x1c}, /* DPC control */
	{0x5782, 0x03}, /* DPC control */
	{0x5783, 0x08}, /* DPC control */
	{0x5784, 0x20}, /* DPC control */
	{0x5787, 0x10}, /* DPC control */
	{0x5788, 0x18}, /* DPC control */
	{0x5a90, 0x0c}, /* ISP control */
	{OV9762_TABLE_WAIT_MS, OV9762_WAIT_MS},
	{OV9762_TABLE_END, 0x00}
};

enum {
	OV9762_MODE_1384x968,
};

static struct ov9762_reg *mode_table[] = {
	[OV9762_MODE_1384x968] = mode_1384x968,
};

#define ov9762_ENTER_GROUP_HOLD(group_hold) \
	do {	\
		if (group_hold) {   \
			reg_list[offset].addr = 0x3208; \
			reg_list[offset].val = 0x00;\
			offset++;  \
		}   \
	} while (0)

#define ov9762_LEAVE_GROUP_HOLD(group_hold) \
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

static void ov9762_mclk_disable(struct ov9762_info *info)
{
	dev_dbg(&info->i2c_client->dev, "%s: disable MCLK\n", __func__);
	clk_disable_unprepare(info->mclk);
}

static int ov9762_mclk_enable(struct ov9762_info *info)
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

static inline int ov9762_get_frame_length_regs(struct ov9762_reg *regs,
						u32 frame_length)
{
	regs->addr = 0x0340;
	regs->val = (frame_length >> 8) & 0xff;
	(regs + 1)->addr = 0x0341;
	(regs + 1)->val = (frame_length) & 0xff;
	return 2;
}

static inline int ov9762_get_coarse_time_regs(struct ov9762_reg *regs,
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

static inline int ov9762_get_gain_reg(struct ov9762_reg *regs, u16 gain)
{
	regs->addr = 0x350b;
	regs->val = gain & 0xff;
	return 1;
}

static int ov9762_read_reg(struct i2c_client *client, u16 addr, u8 *val)
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

static int ov9762_write_reg(struct i2c_client *client, u16 addr, u8 val)
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
		pr_err("ov9762: i2c transfer failed, retrying %x %x\n",
			addr, val);

		usleep_range(3000, 3100);
	} while (retry <= OV9762_MAX_RETRIES);

	return err;
}

static int ov9762_write_bulk_reg(struct i2c_client *client, u8 *data, int len)
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

	pr_err("ov9762: i2c bulk transfer failed at %x\n",
		(int)data[0] << 8 | data[1]);

	return err;
}

static int ov9762_write_table(struct ov9762_info *info,
			      const struct ov9762_reg table[],
			      const struct ov9762_reg override_list[],
			      int num_override_regs)
{
	int err;
	const struct ov9762_reg *next, *n_next;
	u8 *b_ptr = info->i2c_trans_buf;
	unsigned int buf_filled = 0;
	unsigned int i;
	u16 val;

	for (next = table; next->addr != OV9762_TABLE_END; next++) {
		if (next->addr == OV9762_TABLE_WAIT_MS) {
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
		if (n_next->addr != OV9762_TABLE_END &&
			n_next->addr != OV9762_TABLE_WAIT_MS &&
			buf_filled < SIZEOF_I2C_TRANSBUF &&
			n_next->addr == next->addr + 1) {
			continue;
		}

		err = ov9762_write_bulk_reg(info->i2c_client,
			info->i2c_trans_buf, buf_filled);
		if (err)
			return err;
		buf_filled = 0;
	}
	return 0;
}

static int ov9762_set_mode(struct ov9762_info *info, struct ov9762_mode *mode)
{
	int sensor_mode;
	int err;
	struct ov9762_reg reg_list[6];
	int offset = 0;
	
	if (mode->xres == 1384 && mode->yres == 968) {
		sensor_mode = OV9762_MODE_1384x968;
		pr_info("ov9762_set_mode 1384x968\n");
	} else {
		pr_err("There is no this resolution no support %dX%d!!",
			mode->xres, mode->yres);
		return 1;
	}

	/* get a list of override regs for the asking frame length, */
	/* coarse integration time, and gain.                       */
	offset += ov9762_get_frame_length_regs(reg_list + offset,
					       mode->frame_length);
	offset += ov9762_get_coarse_time_regs(reg_list + offset,
					      mode->coarse_time);
	offset += ov9762_get_gain_reg(reg_list + offset, mode->gain);

	err = ov9762_write_table(info, mode_table[sensor_mode], reg_list,
				 offset);
	if (err)
		return err;

	info->mode = sensor_mode;
	
	return 0;
}

static int ov9762_set_frame_length(struct ov9762_info *info, u32 frame_length)
{
	int ret;
	struct ov9762_reg reg_list[2];
	u8 *b_ptr = info->i2c_trans_buf;

	ov9762_get_frame_length_regs(reg_list, frame_length);

	*b_ptr++ = reg_list[0].addr >> 8;
	*b_ptr++ = reg_list[0].addr & 0xff;
	*b_ptr++ = reg_list[0].val & 0xff;
	*b_ptr++ = reg_list[1].val & 0xff;
	ret = ov9762_write_bulk_reg(info->i2c_client, info->i2c_trans_buf, 4);

	return ret;
}

static int ov9762_set_coarse_time(struct ov9762_info *info, u32 coarse_time)
{
	int ret;
	struct ov9762_reg reg_list[3];
	u8 *b_ptr = info->i2c_trans_buf;

	dev_info(&info->i2c_client->dev, "coarse_time 0x%x\n", coarse_time);
	ov9762_get_coarse_time_regs(reg_list, coarse_time);

	*b_ptr++ = reg_list[0].addr >> 8;
	*b_ptr++ = reg_list[0].addr & 0xff;
	*b_ptr++ = reg_list[0].val & 0xff;
	*b_ptr++ = reg_list[1].val & 0xff;
	*b_ptr++ = reg_list[2].val & 0xff;

	ret = ov9762_write_bulk_reg(info->i2c_client, info->i2c_trans_buf, 5);
	return ret;

}

static int ov9762_set_gain(struct ov9762_info *info, u16 gain)
{
	int ret;
	u8 *b_ptr = info->i2c_trans_buf;

	struct ov9762_reg reg_list[1];
	ov9762_get_gain_reg(reg_list, gain);

	*b_ptr++ = reg_list[0].addr >> 8;
	*b_ptr++ = reg_list[0].addr & 0xff;
	*b_ptr++ = reg_list[0].val & 0xff;
	ret = ov9762_write_bulk_reg(info->i2c_client, info->i2c_trans_buf, 3);
	return ret;
}

static int ov9762_set_group_hold(struct ov9762_info *info,
			struct ov9762_grouphold *gh)
{
	int err = 0;
	struct ov9762_reg reg_list[12];
	int offset = 0;
	bool group_hold = true;

	ov9762_ENTER_GROUP_HOLD(group_hold);
	if (gh->gain_enable)
		offset += ov9762_get_gain_reg(reg_list + offset,
					  gh->gain);
	if (gh->frame_length_enable)
		offset += ov9762_get_frame_length_regs(reg_list + offset,
						  gh->frame_length);
	if (gh->coarse_time_enable)
		offset += ov9762_get_coarse_time_regs(reg_list + offset,
			gh->coarse_time);
	ov9762_LEAVE_GROUP_HOLD(group_hold);

	reg_list[offset].addr = OV9762_TABLE_END;
	err |= ov9762_write_table(info, reg_list, NULL, 0);

	return err;
}

static int ov9762_get_sensor_id(struct ov9762_info *info)
{
	int ret = 0;
	int i;
	u8  bak;

	dev_info(&info->i2c_client->dev, "%s\n", __func__);
	//if (info->sensor_data.fuse_id_size)
	//	return 0;

	/* select bank 31 */
	ov9762_write_reg(info->i2c_client, 0x3d84, 31);
	for (i = 0; i < 8; i++) {
		ret |= ov9762_read_reg(info->i2c_client, 0x300A + i, &bak);
		info->sensor_data.fuse_id[i] = bak;
	}

	if (!ret)
		info->sensor_data.fuse_id_size = 8;

	return ret;
}

static int ov9762_get_status(struct ov9762_info *info, u8 *status)
{
	int err;

	*status = 0;
	err = ov9762_read_reg(info->i2c_client, 0x002, status);
	return err;
}

static long ov9762_ioctl(struct file *file,
			 unsigned int cmd, unsigned long arg)
{
	int err;
	struct ov9762_info *info = file->private_data;

	switch (cmd) {
	case OV9762_IOCTL_SET_MODE:
	{
		struct ov9762_mode mode;
		if (copy_from_user(&mode,
				   (const void __user *)arg,
				   sizeof(struct ov9762_mode))) {
			return -EFAULT;
		}

		return ov9762_set_mode(info, &mode);
	}
	case OV9762_IOCTL_SET_FRAME_LENGTH:
		return ov9762_set_frame_length(info, (u32)arg);
	case OV9762_IOCTL_SET_COARSE_TIME:
		return ov9762_set_coarse_time(info, (u32)arg);
	case OV9762_IOCTL_SET_GAIN:
		return ov9762_set_gain(info, (u16)arg);
	case OV9762_IOCTL_SET_GROUP_HOLD:
	{
		struct ov9762_grouphold gh;
		if (copy_from_user(&gh,
				(const void __user *)arg,
				sizeof(struct ov9762_grouphold))) {
			return -EFAULT;
		}
		return ov9762_set_group_hold(info, &gh);
	}
	case OV9762_IOCTL_GET_STATUS:
	{
		u8 status;

		err = ov9762_get_status(info, &status);
		if (err)
			return err;
		if (copy_to_user((void __user *)arg, &status,
				 2)) {
			return -EFAULT;
		}
		return 0;
	}
	case OV9762_IOCTL_GET_FUSEID:
	{
		err = ov9762_get_sensor_id(info);
		if (err)
			return err;
		if (copy_to_user((void __user *)arg,
				&info->sensor_data,
				sizeof(struct ov9762_sensordata))) {
			return -EFAULT;
		}
		return 0;
	}
	case OV9762_IOCTL_START_STREAM:
	{
		return ov9762_write_reg(info->i2c_client, 0x100, 1);
	}
	case OV9762_IOCTL_GET_FRAMEINFO:
	{
		struct frameinfo info;
		int ret;

		if (copy_from_user(&info, (const void __user *) arg,
				   sizeof(info))) {
			pr_err("cannot get frameinfo argument from userland");
			return -EFAULT;
		}

		ret = sh_get_frameinfo(DEVICE_FRONT_CAMERA, &info);

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

static int ov9762_open(struct inode *inode, struct file *file)
{
	struct miscdevice	*miscdev = file->private_data;
	struct ov9762_info	*info;

	info = container_of(miscdev, struct ov9762_info, miscdev_info);
	/* check if the device is in use */
	if (atomic_xchg(&info->in_use, 1)) {
		dev_info(&info->i2c_client->dev, "%s:BUSY!\n", __func__);
		return -EBUSY;
	}
	file->private_data = info;

	if (info->pdata && info->pdata->power_on) {
		ov9762_mclk_enable(info);
		info->pdata->power_on(&info->power, info->pdata->sh_device);
	} else {
		dev_err(&info->i2c_client->dev,
			"%s:no valid power_on function.\n", __func__);
		return -EEXIST;
	}

	return 0;
}

int ov9762_release(struct inode *inode, struct file *file)
{
	struct ov9762_info *info = file->private_data;

	if (info->pdata && info->pdata->power_off) {
		ov9762_mclk_disable(info);
		info->pdata->power_off(&info->power, info->pdata->sh_device);
	}
	file->private_data = NULL;

	/* warn if device is already released */
	WARN_ON(!atomic_xchg(&info->in_use, 0));
	return 0;
}


static int ov9762_power_put(struct ov9762_power_rail *pw)
{
	if (likely(pw->avdd_hv))
		regulator_put(pw->avdd_hv);

	if (likely(pw->vdd_lv))
		regulator_put(pw->vdd_lv);

	if (likely(pw->dvdd))
		regulator_put(pw->dvdd);

	pw->avdd_hv = NULL;
	pw->vdd_lv = NULL;
	pw->dvdd = NULL;

	return 0;
}

static int ov9762_regulator_get(struct ov9762_info *info,
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

static int ov9762_power_get(struct ov9762_info *info)
{
	struct ov9762_power_rail *pw = &info->power;

	ov9762_regulator_get(info, &pw->vdd_lv, "vdig_ov9762"); /*LDO2*/
	ov9762_regulator_get(info, &pw->avdd_hv, "avdd_ov9762");/*LDO5*/
	ov9762_regulator_get(info, &pw->dvdd, "dvdd_ov9762");   /*LDO7*/

	return 0;
}

static const struct file_operations ov9762_fileops = {
	.owner = THIS_MODULE,
	.open = ov9762_open,
	.unlocked_ioctl = ov9762_ioctl,
	.release = ov9762_release,
};

static struct miscdevice ov9762_device = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "ov9762",
	.fops = &ov9762_fileops,
};

#ifdef CONFIG_DEBUG_FS
static int ov9762_stats_show(struct seq_file *s, void *data)
{
	struct ov9762_info *info = (struct ov9762_info *)(s->private);

	seq_printf(s, "%-20s : %-20s\n", "Name", "ov9762-debugfs-testing");
	seq_printf(s, "%-20s : 0x%X\n", "Current i2c-offset Addr",
			info->debug_i2c_offset);
	return 0;
}

static int ov9762_stats_open(struct inode *inode, struct file *file)
{
	return single_open(file, ov9762_stats_show, inode->i_private);
}

static const struct file_operations ov9762_stats_fops = {
	.open       = ov9762_stats_open,
	.read       = seq_read,
	.llseek     = seq_lseek,
	.release    = single_release,
};

static int debug_i2c_offset_w(void *data, u64 val)
{
	struct ov9762_info *info = (struct ov9762_info *)(data);
	dev_info(&info->i2c_client->dev,
			"ov9762:%s setting i2c offset to 0x%X\n",
			__func__, (u32)val);
	info->debug_i2c_offset = (u32)val;
	dev_info(&info->i2c_client->dev,
			"ov9762:%s new i2c offset is 0x%X\n", __func__,
			info->debug_i2c_offset);
	return 0;
}

static int debug_i2c_offset_r(void *data, u64 *val)
{
	struct ov9762_info *info = (struct ov9762_info *)(data);
	*val = (u64)info->debug_i2c_offset;
	dev_info(&info->i2c_client->dev,
			"ov9762:%s reading i2c offset is 0x%X\n", __func__,
			info->debug_i2c_offset);
	return 0;
}

static int debug_i2c_read(void *data, u64 *val)
{
	struct ov9762_info *info = (struct ov9762_info *)(data);
	u8 temp1 = 0;
	u8 temp2 = 0;
	dev_info(&info->i2c_client->dev,
			"ov9762:%s reading offset 0x%X\n", __func__,
			info->debug_i2c_offset);
	if (ov9762_read_reg(info->i2c_client,
				info->debug_i2c_offset, &temp1)
		|| ov9762_read_reg(info->i2c_client,
			info->debug_i2c_offset+1, &temp2)) {
		dev_err(&info->i2c_client->dev,
				"ov9762:%s failed\n", __func__);
		return -EIO;
	}
	dev_info(&info->i2c_client->dev,
			"ov9762:%s read value is 0x%04X\n", __func__,
			temp1<<8 | temp2);
	*val = (u64)(temp1<<8 | temp2);
	return 0;
}

static int debug_i2c_write(void *data, u64 val)
{
	struct ov9762_info *info = (struct ov9762_info *)(data);
	dev_info(&info->i2c_client->dev,
			"ov9762:%s writing 0x%X to offset 0x%X\n", __func__,
			(u8)val, info->debug_i2c_offset);
	if (ov9762_write_reg(info->i2c_client,
				info->debug_i2c_offset, (u8)val)) {
		dev_err(&info->i2c_client->dev, "ov9762:%s failed\n", __func__);
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

static int ov9762_debug_init(struct ov9762_info *info)
{
	dev_dbg(&info->i2c_client->dev, "%s", __func__);

	info->debugfs_root = debugfs_create_dir(ov9762_device.name, NULL);

	if (!info->debugfs_root)
		goto err_out;

	if (!debugfs_create_file("stats", S_IRUGO,
			info->debugfs_root, info, &ov9762_stats_fops))
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

static int ov9762_probe(struct i2c_client *client,
			const struct i2c_device_id *id)
{
	struct ov9762_info *info;
	const char *mclk_name;
	int err = 0;

	pr_info("ov9762: probing sensor.\n");

	info = devm_kzalloc(&client->dev,
		sizeof(struct ov9762_info), GFP_KERNEL);
	if (!info) {
		pr_err("ov9762:%s:Unable to allocate memory!\n", __func__);
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

	ov9762_power_get(info);

	memcpy(&info->miscdev_info,
		&ov9762_device,
		sizeof(struct miscdevice));

	err = misc_register(&info->miscdev_info);
	if (err) {
		ov9762_power_put(&info->power);
		pr_err("ov9762:%s:Unable to register misc device!\n",
		__func__);
	}

	i2c_set_clientdata(client, info);

	pr_info("ov9762: probing sensor done.\n");
#ifdef CONFIG_DEBUG_FS
	ov9762_debug_init(info);
#endif

	return err;
}

static int ov9762_remove(struct i2c_client *client)
{
	struct ov9762_info *info = i2c_get_clientdata(client);

	ov9762_power_put(&info->power);
	misc_deregister(&ov9762_device);
#ifdef CONFIG_DEBUG_FS
	debugfs_remove_recursive(info->debugfs_root);
#endif
	return 0;
}

static const struct i2c_device_id ov9762_id[] = {
	{ "ov9762", 0 },
	{ },
};

MODULE_DEVICE_TABLE(i2c, ov9762_id);

static struct i2c_driver ov9762_i2c_driver = {
	.driver = {
		.name = "ov9762",
		.owner = THIS_MODULE,
	},
	.probe = ov9762_probe,
	.remove = ov9762_remove,
	.id_table = ov9762_id,
};

static int __init ov9762_init(void)
{
	pr_info("ov9762 sensor driver loading\n");
	return i2c_add_driver(&ov9762_i2c_driver);
}

static void __exit ov9762_exit(void)
{
	i2c_del_driver(&ov9762_i2c_driver);
}

module_init(ov9762_init);
module_exit(ov9762_exit);
MODULE_LICENSE("GPL v2");
