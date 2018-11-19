/*
 * arch/arm/mach-tegra/board-touch-raydium_spi.c
 *
 * Copyright (C) 2012-2014, NVIDIA Corporation.  All Rights Reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
 */

#include <linux/errno.h>
#include <linux/kernel.h>
#include <linux/spi/spi.h>
#include <linux/delay.h>
#include <linux/gpio.h>
#include <linux/spi/rm31080a_ts.h>

/*#include <mach/gpio-tegra.h>*/
#ifdef EPROBE_DEFER
int __init touch_init_raydium(int irq_gpio, int reset_gpio,
				struct rm_spi_ts_platform_data *rm31080ts_data,
				struct spi_board_info *rm31080a_spi_board,
				int asize)
{
	int err = RETURN_OK;
	gpio_request(irq_gpio, "raydium-irq");
	gpio_direction_input(irq_gpio);

	gpio_request(reset_gpio, "raydium-reset");
	gpio_direction_output(reset_gpio, 0);

	rm31080ts_data->gpio_reset = reset_gpio;

	rm31080a_spi_board->irq = gpio_to_irq(irq_gpio);

	spi_register_board_info(rm31080a_spi_board, asize);
	pr_info("Raydium - touch platform_id :  %d\n",
			rm31080ts_data->platform_id);

	return err;
}
#else
struct rm_spi_ts_platform_data rm31080ts_data = {
	.gpio_reset = 0,
	.config = 0,
};

struct spi_board_info rm31080a_spi_board[1] = {
	{
		.modalias = "rm_ts_spidev",
		.bus_num = 0,
		.chip_select = 0,
		.max_speed_hz = 9 * 1000 * 1000,
		.mode = SPI_MODE_0,
		.platform_data = &rm31080ts_data,
	},
};


int __init touch_init_raydium_and_spi(int irq_gpio, int reset_gpio,
	int platform, u16 bus_num, u16 chip_select)
{
	int err = RETURN_OK;

	/*tegra_gpio_enable(irq_gpio); removed by nvidia */
	gpio_request(irq_gpio, "raydium-irq");
	gpio_direction_input(irq_gpio);

	/*tegra_gpio_enable(reset_gpio); removed by nvidia */
	gpio_request(reset_gpio, "raydium-reset");
	gpio_direction_output(reset_gpio, 0);

	rm31080ts_data.gpio_reset = reset_gpio;

	usleep_range(5000, 6000);/*msleep(5); */
	gpio_set_value(reset_gpio, 1);
	usleep_range(5000, 6000);/*msleep(5); */

	rm31080a_spi_board[0].irq = gpio_to_irq(irq_gpio);
	rm31080a_spi_board[0].bus_num = bus_num;
	rm31080a_spi_board[0].chip_select = chip_select;

	rm31080ts_data.platform_id = platform;

	switch (platform) {
	case RM_PLATFORM_K007:/*0x00*/
		pr_info("Raydium Kai PCB based touch init\n");
		break;
	case RM_PLATFORM_K107:/*0x01*/
		pr_info("Raydium Kai On-Board touch init\n");
		break;
	case RM_PLATFORM_C210:/*0x02*/
		pr_info("Raydium cardhu touch init\n");
		break;
	case RM_PLATFORM_D010:/*0x03*/
		pr_info("Raydium dalmore touch init\n");
		break;
	case RM_PLATFORM_P005:/*0x04*/
		pr_info("Raydium pluto touch init\n");
		break;
	case RM_PLATFORM_R005:/*0x05*/
		pr_info("Raydium roth touch init\n");
		break;
	case RM_PLATFORM_RAYPRJ: /*0x80*/
		pr_info("Raydium touch init\n");
		break;
	default:
		pr_err("touch_id error, no touch\n");
		err = -ENODEV;
		break;
	}

	if (!err)
		spi_register_board_info(rm31080a_spi_board,
				ARRAY_SIZE(rm31080a_spi_board));

	return err;
}

int __init touch_init_raydium(int irq_gpio, int reset_gpio, int platform)
{
	u16 bus_num = 0;
	u16 chip_select = 0;
	return touch_init_raydium_and_spi(irq_gpio, reset_gpio, platform,
		bus_num, chip_select);
}
#endif
