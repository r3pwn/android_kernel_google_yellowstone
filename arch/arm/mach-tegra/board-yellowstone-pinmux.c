/*
 * arch/arm/mach-tegra/board-yellowstone-pinmux.c
 *
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

#include <linux/bug.h>
#include <linux/export.h>
#include <linux/kernel.h>
#include <mach/pinmux.h>
#include <mach/pinmux-defines.h>
#include <mach/pinmux-t12.h>

#include <linux/init.h>
#include <linux/gpio.h>
#include <mach/gpio-tegra.h>

#include "board.h"
#include "devices.h"
#include "gpio-names.h"
#include "common.h"
#include "board-yellowstone-pinmux-t12x.h"

static struct tegra_drive_pingroup_config yellowstone_drive_pinmux[] = {

    /* SDMMC1 */
    SET_DRIVE(SDIO1, ENABLE, DISABLE, DIV_1, 32, 42, FASTEST, FASTEST),

    /* SDMMC3 */
    SET_DRIVE(SDIO3, ENABLE, DISABLE, DIV_1, 20, 36, FASTEST, FASTEST),

    /* SDMMC4 */
    SET_DRIVE_WITH_TYPE(GMA, ENABLE, DISABLE, DIV_1, 1, 2, FASTEST, FASTEST, 1),
};

static struct tegra_pingroup_config yellowstone_pinmux_common_pvt[] = {
	DEFAULT_PINMUX(DP_HPD, RSVD2, NORMAL, NORMAL, INPUT),
};

static struct gpio_init_pin_info init_gpio_mode_yellowstone_common_pvt[] = {

	/* CAP_RDY */
	GPIO_INIT_PIN_MODE(TEGRA_GPIO_PFF0, true, 0),
};

static void yellowstone_gpio_init_configure(void)
{
    int len;
    int i;
    struct gpio_init_pin_info *pins_info;

    printk("%s\n", __func__);
    len = ARRAY_SIZE(init_gpio_mode_yellowstone_common);
    pins_info = init_gpio_mode_yellowstone_common;

    for (i = 0; i < len; ++i) {
        tegra_gpio_init_configure(pins_info->gpio_nr,
            pins_info->is_input, pins_info->value);
        pins_info++;
    }
	/* External pull up resistor 100k for RDY pin in PVT */
	if(get_cci_hw_id() >= PVT)
	    tegra_gpio_init_configure(init_gpio_mode_yellowstone_common_pvt->gpio_nr,
			init_gpio_mode_yellowstone_common_pvt->is_input,
			init_gpio_mode_yellowstone_common_pvt->value);
}

void yellowstone_pinmux_init(void)
{
    printk("%s\n", __func__);
    yellowstone_gpio_init_configure();

    tegra_pinmux_config_table(yellowstone_pinmux_common,
                    ARRAY_SIZE(yellowstone_pinmux_common));

	if(get_cci_hw_id() >= PVT)
		tegra_pinmux_config_table(yellowstone_pinmux_common_pvt,
                    ARRAY_SIZE(yellowstone_pinmux_common_pvt));

    tegra_drive_pinmux_config_table(yellowstone_drive_pinmux, ARRAY_SIZE(yellowstone_drive_pinmux));

    tegra_pinmux_config_table(unused_pins_lowpower,
        ARRAY_SIZE(unused_pins_lowpower));
}
EXPORT_SYMBOL(yellowstone_pinmux_init);
