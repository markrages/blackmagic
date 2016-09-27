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

#include "esp/uart.h"

#include "FreeRTOS.h"
#include "task.h"

void platform_init(int argc, char **argv)
{
  (void) argc;
  (void) argv;

  struct ip_info ipconfig;

  do {
    //get ip info of ESP8266 station
    sdk_wifi_get_ip_info(STATION_IF, &ipconfig);

  } while (ipconfig.ip.addr==0);

  printf("IP ADDR " IPSTR "\n", IP2STR(&ipconfig));

  assert(gdb_if_init() == 0);

}

void platform_srst_set_val(bool assert)
{
	(void)assert;
	platform_buffer_flush();
}

bool platform_srst_get_val(void) { return false; }

void platform_buffer_flush(void)
{
  ;
}

const char *platform_target_voltage(void)
{
  return "not supported";
}

uint32_t platform_time_ms(void)
{
  return xTaskGetTickCount() / portTICK_RATE_MS;
}

#define vTaskDelayMs(ms)	vTaskDelay((ms)/portTICK_RATE_MS)

void platform_delay(uint32_t ms)
{
  vTaskDelayMs(ms);
}

int platform_hwversion(void)
{
  return 0;
}

/* This is a transplanted main() from main.c */
void main_task(void *parameters)
{
  (void) parameters;

  platform_init(0, NULL);

       while (true) {

 	  volatile struct exception e;
		TRY_CATCH(e, EXCEPTION_ALL) {
			gdb_main();
		}
		if (e.type) {
			gdb_putpacketz("EFF");
			target_list_free();
			morse("TARGET LOST.", 1);
		}
	}

	/* Should never get here */
}

#include "espressif/esp_wifi.h"
#include "ssid_config.h"

void user_init(void)
{
    uart_set_baud(0, 115200);
    printf("SDK version:%s\n", sdk_system_get_sdk_version());

    struct sdk_station_config config = {
        .ssid = WIFI_SSID,
        .password = WIFI_PASS,
    };

    /* required to call wifi_set_opmode before station_set_config */
    sdk_wifi_set_opmode(STATION_MODE);
    sdk_wifi_station_set_config(&config);

    xTaskCreate(&main_task, (signed char *)"main", 4*256, NULL, 2, NULL);
}
