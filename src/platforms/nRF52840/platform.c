/*
 * This file is part of the Black Magic Debug project.
 *
 * Copyright (C) 2011  Black Sphere Technologies Ltd.
 * Written by Gareth McMullin <gareth@blacksphere.co.nz>
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */
#include "general.h"
#include "gdb_if.h"
#include "version.h"

#include "gdb_packet.h"
#include "gdb_main.h"
#include "target.h"
#include "exception.h"
#include "gdb_packet.h"
#include "morse.h"

#include <assert.h>
#include <sys/time.h>
#include <sys/unistd.h>

#include <nrf_gpio.h>
#include <nrf_delay.h>

#include "nrf_main.h"

void platform_init(int argc, char **argv)
{
	(void) argc;
	(void) argv;

	gpio_clear(_, SWCLK_PIN);
	gpio_clear(_, SWDIO_PIN);

	nrf_gpio_cfg_output(SWCLK_PIN);
	nrf_gpio_cfg_output(SWDIO_PIN);

	assert(gdb_if_init() == 0);
}

void platform_buffer_flush(void)
{
	;
}

void platform_srst_set_val(bool assert)
{
	(void)assert;
}

bool platform_srst_get_val(void) { return false; }

const char *platform_target_voltage(void)
{
	return "not supported";
}

uint32_t platform_time_ms(void)
{
	extern volatile uint32_t ms_count;
	return ms_count;
}

void platform_delay(uint32_t ms)
{
	nrf_delay_ms(ms);
}

int platform_hwversion(void)
{
	return 0;
}
