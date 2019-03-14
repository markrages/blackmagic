/**
 * Copyright (c) 2017 - 2018, Nordic Semiconductor ASA
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form, except as embedded into a Nordic
 *    Semiconductor ASA integrated circuit in a product or a software update for
 *    such product, must reproduce the above copyright notice, this list of
 *    conditions and the following disclaimer in the documentation and/or other
 *    materials provided with the distribution.
 *
 * 3. Neither the name of Nordic Semiconductor ASA nor the names of its
 *    contributors may be used to endorse or promote products derived from this
 *    software without specific prior written permission.
 *
 * 4. This software, with or without modification, must only be used with a
 *    Nordic Semiconductor ASA integrated circuit.
 *
 * 5. Any software provided in binary form under this license must not be reverse
 *    engineered, decompiled, modified and/or disassembled.
 *
 * THIS SOFTWARE IS PROVIDED BY NORDIC SEMICONDUCTOR ASA "AS IS" AND ANY EXPRESS
 * OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY, NONINFRINGEMENT, AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL NORDIC SEMICONDUCTOR ASA OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
 * GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
 * OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */
#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdio.h>

#include "nrf.h"
#include "nrf_drv_usbd.h"
#include "nrf_drv_clock.h"
#include "nrf_gpio.h"
#include "nrf_delay.h"
#include "nrf_drv_power.h"

#include "app_error.h"
#include "app_util.h"
#include "app_usbd_core.h"
#include "nrf_dfu_trigger_usb.h"
#include "app_usbd.h"
#include "app_usbd_string_desc.h"
#include "app_usbd_cdc_acm.h"
#include "app_usbd_serial_num.h"

#include "boards.h"
#include "bsp.h"
#include "bsp_cli.h"
#include "nrf_cli.h"
#include "nrf_cli_uart.h"

#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"

#define LIBFTDI
#include "gdb_if.h"

#include "morse.h"
#include "platform.h"

/**
 * @brief CLI interface over UART
 */
NRF_CLI_UART_DEF(m_cli_uart_transport, 0, 64, 16);
NRF_CLI_DEF(m_cli_uart,
            "uart_cli:~$ ",
            &m_cli_uart_transport.transport,
            '\r',
            4);

/**@file
 * @defgroup usbd_cdc_acm_example main.c
 * @{
 * @ingroup usbd_cdc_acm_example
 * @brief USBD CDC ACM example
 *
 */

#define BTN_CDC_DATA_SEND       0
#define BTN_CDC_NOTIFY_SEND     1

#define BTN_CDC_DATA_KEY_RELEASE        (bsp_event_t)(BSP_EVENT_KEY_LAST + 1)

/**
 * @brief Enable power USB detection
 *
 * Configure if example supports USB port connection
 */
#ifndef USBD_POWER_DETECTION
#define USBD_POWER_DETECTION true
#endif


static void cdc_acm_user_ev_handler(app_usbd_class_inst_t const * p_inst,
                                    app_usbd_cdc_acm_user_event_t event);
static void cdc_acm_user_ev_handler2(app_usbd_class_inst_t const * p_inst,
                                     app_usbd_cdc_acm_user_event_t event);

#define CDC_ACM_COMM_INTERFACE  0
#define CDC_ACM_COMM_EPIN       NRF_DRV_USBD_EPIN2

#define CDC_ACM_DATA_INTERFACE  1
#define CDC_ACM_DATA_EPIN       NRF_DRV_USBD_EPIN1
#define CDC_ACM_DATA_EPOUT      NRF_DRV_USBD_EPOUT1

#define CDC_ACM2_COMM_INTERFACE  2
#define CDC_ACM2_COMM_EPIN       NRF_DRV_USBD_EPIN3

#define CDC_ACM2_DATA_INTERFACE  3
#define CDC_ACM2_DATA_EPIN       NRF_DRV_USBD_EPIN4
#define CDC_ACM2_DATA_EPOUT      NRF_DRV_USBD_EPOUT2

/**
 * @brief CDC_ACM class instances
 * */
APP_USBD_CDC_ACM_GLOBAL_DEF(m_app_cdc_acm,
                            APP_USER_BMP_UART,
                            cdc_acm_user_ev_handler,
                            CDC_ACM_COMM_INTERFACE,
                            CDC_ACM_DATA_INTERFACE,
                            CDC_ACM_COMM_EPIN,
                            CDC_ACM_DATA_EPIN,
                            CDC_ACM_DATA_EPOUT,
                            APP_USBD_CDC_COMM_PROTOCOL_AT_V250
);

APP_USBD_CDC_ACM_GLOBAL_DEF(m_app_cdc_acm2,
                            APP_USER_BMP_GDB,
                            cdc_acm_user_ev_handler2,
                            CDC_ACM2_COMM_INTERFACE,
                            CDC_ACM2_DATA_INTERFACE,
                            CDC_ACM2_COMM_EPIN,
                            CDC_ACM2_DATA_EPIN,
                            CDC_ACM2_DATA_EPOUT,
                            APP_USBD_CDC_COMM_PROTOCOL_AT_V250
);



#define READ_SIZE 1

static char m_rx_buffer[READ_SIZE];
static char m_tx_buffer[NRF_DRV_USBD_EPSIZE];

static char m_rx_buffer2[READ_SIZE];
static char m_tx_buffer2[NRF_DRV_USBD_EPSIZE];

static bool m_send_flag = 0;

/**
 * @brief User event handler @ref app_usbd_cdc_acm_user_ev_handler_t (headphones)
 * */
static void cdc_acm_user_ev_handler(app_usbd_class_inst_t const * p_inst,
                                    app_usbd_cdc_acm_user_event_t event)
{
    app_usbd_cdc_acm_t const * p_cdc_acm = app_usbd_cdc_acm_class_get(p_inst);

    switch (event)
    {
        case APP_USBD_CDC_ACM_USER_EVT_PORT_OPEN:
        {
	        //bsp_board_led_on(LED_CDC_ACM_OPEN);

            /*Setup first transfer*/
            ret_code_t ret = app_usbd_cdc_acm_read(&m_app_cdc_acm,
                                                   m_rx_buffer,
                                                   READ_SIZE);
            UNUSED_VARIABLE(ret);

            break;
        }
        case APP_USBD_CDC_ACM_USER_EVT_PORT_CLOSE:
	        //bsp_board_led_off(LED_CDC_ACM_OPEN);
            break;
        case APP_USBD_CDC_ACM_USER_EVT_TX_DONE:
	        //bsp_board_led_invert(LED_CDC_ACM_TX);
            break;
        case APP_USBD_CDC_ACM_USER_EVT_RX_DONE:
        {
            ret_code_t ret;
            NRF_LOG_INFO("Bytes waiting: %d", app_usbd_cdc_acm_bytes_stored(p_cdc_acm));
            do
            {
                /*Get amount of data transfered*/
                size_t size = app_usbd_cdc_acm_rx_size(p_cdc_acm);
                NRF_LOG_INFO("RX: size: %lu char: %c", size, m_rx_buffer[0]);

                /* Fetch data until internal buffer is empty */
                ret = app_usbd_cdc_acm_read(&m_app_cdc_acm,
                                            m_rx_buffer,
                                            READ_SIZE);
            } while (ret == NRF_SUCCESS);

            //bsp_board_led_invert(LED_CDC_ACM_RX);
            break;
        }
        default:
            break;
    }
}

#define CIRCBUF_SIZE (2048)
uint8_t gdb_bytes_incoming[CIRCBUF_SIZE];
int gdb_bytes_in_head = 0;
int gdb_bytes_in_tail = 0;

uint8_t gdb_bytes_outgoing[CIRCBUF_SIZE];
int gdb_bytes_out_head = 0;
int gdb_bytes_out_tail = 0;
static bool m_send_flag2 = 0;

void nrf_put_byte(uint8_t b)
{
	gdb_bytes_outgoing[gdb_bytes_out_tail++] = b;
	if (gdb_bytes_out_tail == CIRCBUF_SIZE)
		gdb_bytes_out_tail = 0;
	m_send_flag2 = true;
}

bool nrf_get_byte(uint8_t *b)
{
	if (gdb_bytes_in_head == gdb_bytes_in_tail)
		return false;

	*b = gdb_bytes_incoming[gdb_bytes_in_head++];
	if (gdb_bytes_in_head == CIRCBUF_SIZE)
		gdb_bytes_in_head = 0;
	return true;
}

/**
 * @brief User event handler @ref app_usbd_cdc_acm_user_ev_handler_t (headphones)
 * */
static void cdc_acm_user_ev_handler2(app_usbd_class_inst_t const * p_inst,
                                     app_usbd_cdc_acm_user_event_t event)
{
    app_usbd_cdc_acm_t const * p_cdc_acm = app_usbd_cdc_acm_class_get(p_inst);

    switch (event)
    {
        case APP_USBD_CDC_ACM_USER_EVT_PORT_OPEN:
        {
	        //            bsp_board_led_on(LED_CDC_ACM_OPEN);

            /*Setup first transfer*/
            ret_code_t ret = app_usbd_cdc_acm_read(&m_app_cdc_acm2,
                                                   m_rx_buffer2,
                                                   READ_SIZE);

            UNUSED_VARIABLE(ret);

            gdb_bytes_in_tail =
	            gdb_bytes_in_head =
	            gdb_bytes_out_tail =
	            gdb_bytes_out_head = 0;

            break;
        }
        case APP_USBD_CDC_ACM_USER_EVT_PORT_CLOSE:
	        //bsp_board_led_off(LED_CDC_ACM_OPEN);
            break;
        case APP_USBD_CDC_ACM_USER_EVT_TX_DONE:
	        //bsp_board_led_invert(LED_CDC_ACM_TX);
            break;
        case APP_USBD_CDC_ACM_USER_EVT_RX_DONE:
        {
            ret_code_t ret;
            NRF_LOG_INFO("Bytes waiting: %d", app_usbd_cdc_acm_bytes_stored(p_cdc_acm));
            do
            {
                /*Get amount of data transfered*/
                size_t size = app_usbd_cdc_acm_rx_size(p_cdc_acm);
                NRF_LOG_INFO("RX: size: %lu char: %c", size, m_rx_buffer2[0]);

                for (int i=0; i<size; i++) {

	                gdb_bytes_incoming[gdb_bytes_in_tail++] = m_rx_buffer2[i];
	                if (gdb_bytes_in_tail == CIRCBUF_SIZE)
		                gdb_bytes_in_tail = 0;
                }

                /* Fetch data until internal buffer is empty */
                ret = app_usbd_cdc_acm_read(&m_app_cdc_acm2,
                                            m_rx_buffer2,
                                            READ_SIZE);

            } while (ret == NRF_SUCCESS);

            //bsp_board_led_invert(LED_CDC_ACM_RX);
            break;
        }
        default:
            break;
    }
}


static void usbd_user_ev_handler(app_usbd_event_type_t event)
{
    switch (event)
    {
        case APP_USBD_EVT_DRV_SUSPEND:
	        //bsp_board_led_off(LED_USB_RESUME);
            break;
        case APP_USBD_EVT_DRV_RESUME:
	        //bsp_board_led_on(LED_USB_RESUME);
            break;
        case APP_USBD_EVT_STARTED:
            break;
        case APP_USBD_EVT_STOPPED:
	        app_usbd_disable();
            bsp_board_leds_off();
            break;
        case APP_USBD_EVT_POWER_DETECTED:
            NRF_LOG_INFO("USB power detected");

            if (!nrf_drv_usbd_is_enabled())
            {
                app_usbd_enable();
            }
            break;
        case APP_USBD_EVT_POWER_REMOVED:
            NRF_LOG_INFO("USB power removed");
            app_usbd_stop();
            break;
        case APP_USBD_EVT_POWER_READY:
            NRF_LOG_INFO("USB ready");
            app_usbd_start();
            break;
        default:
            break;
    }
}

static void bsp_event_callback(bsp_event_t ev)
{
    ret_code_t ret;
    switch ((unsigned int)ev)
    {
        case CONCAT_2(BSP_EVENT_KEY_, BTN_CDC_DATA_SEND):
        {
            m_send_flag = 0;
            break;
        }

        case BTN_CDC_DATA_KEY_RELEASE :
        {
            m_send_flag = 1;
            break;
        }

        case CONCAT_2(BSP_EVENT_KEY_, BTN_CDC_NOTIFY_SEND):
        {
            ret = app_usbd_cdc_acm_serial_state_notify(&m_app_cdc_acm,
                                                       APP_USBD_CDC_ACM_SERIAL_STATE_BREAK,
                                                       false);
            UNUSED_VARIABLE(ret);
            break;
        }

        default:
            return; // no implementation needed
    }
}

static void init_bsp(void)
{
    ret_code_t ret;
    ret = bsp_init(BSP_INIT_BUTTONS, bsp_event_callback);
    APP_ERROR_CHECK(ret);

    UNUSED_RETURN_VALUE(bsp_event_to_button_action_assign(BTN_CDC_DATA_SEND,
                                                          BSP_BUTTON_ACTION_RELEASE,
                                                          BTN_CDC_DATA_KEY_RELEASE));

    /* Configure LEDs */
    bsp_board_init(BSP_INIT_LEDS);
}


#define TX_PIN_NUMBER (15)
#define RX_PIN_NUMBER (17)

static void init_cli(void)
{
    ret_code_t ret;
    ret = bsp_cli_init(bsp_event_callback);
    APP_ERROR_CHECK(ret);
    nrf_drv_uart_config_t uart_config = NRF_DRV_UART_DEFAULT_CONFIG;
    uart_config.pseltxd = TX_PIN_NUMBER;
    uart_config.pselrxd = RX_PIN_NUMBER;
    uart_config.hwfc    = NRF_UART_HWFC_DISABLED;
    ret = nrf_cli_init(&m_cli_uart, &uart_config, true, true, NRF_LOG_SEVERITY_INFO);
    APP_ERROR_CHECK(ret);
    ret = nrf_cli_start(&m_cli_uart);
    APP_ERROR_CHECK(ret);
}

volatile uint32_t ms_count = 0;
APP_TIMER_DEF(m_repeated_timer_id);

static void repeated_timer_handler(void *p_context)
{
	(void)p_context;
	ms_count += 10;

	static int morse_ct = 0;
	if (10 == ++morse_ct) {
		SET_ERROR_STATE(morse_update());
		morse_ct = 0;
	}
}

void nrf_init(void)
{
    ret_code_t ret;
    static const app_usbd_config_t usbd_config = {
        .ev_state_proc = usbd_user_ev_handler
    };

    ret = NRF_LOG_INIT(NULL);
    APP_ERROR_CHECK(ret);

    ret = nrf_drv_clock_init();
    APP_ERROR_CHECK(ret);

    nrf_drv_clock_lfclk_request(NULL);

    while(!nrf_drv_clock_lfclk_is_running())
    {
        /* Just waiting */
    }

    ret = app_timer_init();
    APP_ERROR_CHECK(ret);

    // Create timers
    ret = app_timer_create(&m_repeated_timer_id,
                           APP_TIMER_MODE_REPEATED,
                           repeated_timer_handler);
    APP_ERROR_CHECK(ret);

    ret = app_timer_start(m_repeated_timer_id, 328, NULL);
    APP_ERROR_CHECK(ret);

    app_usbd_serial_num_generate();

    ret = app_usbd_init(&usbd_config);
    APP_ERROR_CHECK(ret);

    app_usbd_class_inst_t const * class_cdc_acm = app_usbd_cdc_acm_class_inst_get(&m_app_cdc_acm);
    ret = app_usbd_class_append(class_cdc_acm);
    APP_ERROR_CHECK(ret);

    app_usbd_class_inst_t const * class_cdc_acm2 = app_usbd_cdc_acm_class_inst_get(&m_app_cdc_acm2);
    ret = app_usbd_class_append(class_cdc_acm2);
    APP_ERROR_CHECK(ret);

    init_bsp();
    init_cli();

    NRF_LOG_INFO("USBD CDC ACM example started.");

     if (USBD_POWER_DETECTION)
    {
        ret = app_usbd_power_events_enable();
        APP_ERROR_CHECK(ret);
    }
    else
    {
        NRF_LOG_INFO("No USB power detection enabled\r\nStarting USB now");

        app_usbd_enable();
        app_usbd_start();
    }
}

void nrf_mainloop(void)
{
	ret_code_t ret;

	while (app_usbd_event_queue_process())
		{
			/* Nothing to do */
		}

	if(m_send_flag)
		{
			static int  frame_counter;

			size_t size = sprintf(m_tx_buffer, "Hi %u |in %d - %d | out %d - %d | %d\r\n", frame_counter, gdb_bytes_in_tail, gdb_bytes_in_head, gdb_bytes_out_tail, gdb_bytes_out_head, 0);

			ret = app_usbd_cdc_acm_write(&m_app_cdc_acm, m_tx_buffer, size);
			if (ret == NRF_SUCCESS)
				{
					++frame_counter;
				}
		}
	if(m_send_flag2)
		{
			size_t size = 0;

			int head = gdb_bytes_out_head;
			while ((size < NRF_DRV_USBD_EPSIZE-8) &&
			       (head != gdb_bytes_out_tail))
				{
					m_tx_buffer2[size++] = gdb_bytes_outgoing[head++];
					if (head == CIRCBUF_SIZE)
						head = 0;
				}

			ret = app_usbd_cdc_acm_write(&m_app_cdc_acm2, m_tx_buffer2, size);
			if (ret == NRF_SUCCESS)
				{
					gdb_bytes_out_head = head;
				}
		}

	nrf_cli_process(&m_cli_uart);

	UNUSED_RETURN_VALUE(NRF_LOG_PROCESS());
	/* Sleep CPU only if there was no interrupt since last loop processing */
	//__WFE();
}


int gdb_if_init(void)
{
	nrf_init();
	return 0;
}

unsigned char gdb_if_getchar(void)
{
	uint8_t ret;
	while (!nrf_get_byte(&ret)) {
		nrf_mainloop();
	}

	return ret;
}

unsigned char gdb_if_getchar_to(int timeout)
{
	uint32_t start_time = ms_count;
	uint8_t ret = 0;
	do {
		nrf_mainloop();
		if (nrf_get_byte(&ret))
			break;
	} while ((ms_count - start_time) < (unsigned)timeout);

	return ret;
}

void gdb_if_putchar(unsigned char c, int flush)
{
	nrf_put_byte(c);

	// did we fill up?
	if ((gdb_bytes_out_tail + 1)%CIRCBUF_SIZE == gdb_bytes_out_head) {
		nrf_mainloop();
	}

	if (flush) {
		do {
			nrf_mainloop();
		} while	(gdb_bytes_out_tail != gdb_bytes_out_head);
	}
}


/** @} */
