/*
 * Copyright (c) 2011-2014, NVIDIA CORPORATION, All Rights Reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA
 * 02111-1307, USA
 */

#ifndef __DW9714_H__
#define __DW9714_H__

#include <media/nvc_focus.h>
#include <media/nvc.h>

/* See notes in the nvc.h file on the GPIO usage */
enum dw9714_gpio_type {
	DW9714_GPIO_TYPE_PWRDN = 0,
};

struct dw9714_power_rail {
	struct regulator *vdd;
	struct regulator *vdd_i2c;
};

struct dw9714_platform_data {
	int cfg;
	int num;
	int sync;
	const char *dev_name;
	struct nvc_focus_nvc (*nvc);
	struct nvc_focus_cap (*cap);
	struct dw9714_pdata_info (*info);
	int gpio_count;
	struct nvc_gpio_pdata *gpio;
	int (*power_on)(struct dw9714_power_rail *pw);
	int (*power_off)(struct dw9714_power_rail *pw);
};

struct dw9714_pdata_info {
	float focal_length;
	float fnumber;
	__u32 settle_time;
	__s16 pos_low;
	__s16 pos_high;
	__s16 limit_low;
	__s16 limit_high;
	int move_timeoutms;
	__u32 focus_hyper_ratio;
	__u32 focus_hyper_div;
};

#endif /* __DW9714_H__ */
