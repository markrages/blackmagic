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

#ifndef __PLATFORM_H
#define __PLATFORM_H

#undef PRIx32
#define PRIx32 "x"

#undef SCNx32
#define SCNx32 "x"

#define NO_USB_PLEASE

void platform_buffer_flush(void);

#include "boards.h"
#include "bsp.h"

#define LED_RUN_STATE     (BSP_BOARD_LED_0)
#define LED_IDLE_STATE    (BSP_BOARD_LED_1)
#define LED_ERROR_STATE   (BSP_BOARD_LED_2)
#define LED_UNUSED        (BSP_BOARD_LED_3)

#define SET_LED_STATE(led, state) do {	\
		if (state) { \
			bsp_board_led_on(led); \
		} else { \
			bsp_board_led_off(led); \
		} \
	} while (0);

#define SET_RUN_STATE(state) SET_LED_STATE(LED_RUN_STATE, state)
#define SET_IDLE_STATE(state) SET_LED_STATE(LED_IDLE_STATE, state)
#define SET_ERROR_STATE(state) SET_LED_STATE(LED_ERROR_STATE, state)
#define DEBUG(x, ...) do { ; } while (0)

#include "timing.h"

#include <nrf_gpio.h>

// I wish this define was renamed... it really means "no STM32 stuff please"
#define LIBFTDI

#define TMS_PIN NRF_GPIO_PIN_MAP(0, 24) // pin 0.24
#define TDI_PIN NRF_GPIO_PIN_MAP(0, 9)
#define TDO_PIN NRF_GPIO_PIN_MAP(0, 29)
#define TCK_PIN NRF_GPIO_PIN_MAP(0, 2)

#define SWDIO_DIR_PIN NRF_GPIO_PIN_MAP(0, 20)
#define SWDCK_DIR_PIN NRF_GPIO_PIN_MAP(0, 20)

#define SWDIO_PIN TMS_PIN
#define SWCLK_PIN TCK_PIN

#define gpio_set(port, pin) nrf_gpio_pin_set(pin)
#define gpio_clear(port, pin) nrf_gpio_pin_clear(pin)
#define gpio_get(port, pin) nrf_gpio_pin_read(pin)
#define gpio_set_val(port, pin, value) do {	  \
		if (value) { \
			gpio_set(port, pin); \
		} else { \
			gpio_clear(port, pin); \
		} \
	} while (0)

#define SWDIO_MODE_FLOAT() do {	  \
		nrf_gpio_cfg_input(SWDIO_PIN, NRF_GPIO_PIN_NOPULL); \
		gpio_clear(_, SWDIO_DIR_PIN); \
	} while (0)

#define SWDIO_MODE_DRIVE() do {	  \
		gpio_set(_, SWDIO_DIR_PIN); \
		nrf_gpio_cfg_output(SWDIO_PIN); \
	} while (0)

#define TMS_SET_MODE() do { \
		nrf_gpio_cfg_output(TMS_PIN); \
	} while (0)


#define PLATFORM_HAS_DEBUG // do we?

#endif
