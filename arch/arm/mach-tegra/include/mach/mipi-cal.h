/*
 * Copyright (C) 2014 NVIDIA Corporation.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#ifndef __MACH_TEGRA_MIPI_CAL_H
#define __MACH_TEGRA_MIPI_CAL_H

#ifndef CONFIG_ARCH_TEGRA_2x_SOC
#ifndef CONFIG_ARCH_TEGRA_3x_SOC

int mipi_cal_trylock(void);
void mipi_cal_unlock(void);

#endif
#endif

#endif
