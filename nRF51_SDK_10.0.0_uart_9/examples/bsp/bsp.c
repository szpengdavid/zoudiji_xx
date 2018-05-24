/* Copyright (c) 2014 Nordic Semiconductor. All Rights Reserved.
 *
 * The information contained herein is property of Nordic Semiconductor ASA.
 * Terms and conditions of usage are described in detail in NORDIC
 * SEMICONDUCTOR STANDARD SOFTWARE LICENSE AGREEMENT.
 *
 * Licensees are granted free, non-transferable use of the information. NO
 * WARRANTY of ANY KIND is provided. This heading must NOT be removed from
 * the file.
 *
 */

#include "bsp.h"
#include <stddef.h>
#include <stdio.h>
#include "nordic_common.h"
#include "nrf.h"
#include "nrf_gpio.h"
#include "nrf_error.h"

#define ADVERTISING_LED_ON_INTERVAL            200
#define ADVERTISING_LED_OFF_INTERVAL           1800

#define ADVERTISING_DIRECTED_LED_ON_INTERVAL   200
#define ADVERTISING_DIRECTED_LED_OFF_INTERVAL  200

#define ADVERTISING_WHITELIST_LED_ON_INTERVAL  200
#define ADVERTISING_WHITELIST_LED_OFF_INTERVAL 800

#define ADVERTISING_SLOW_LED_ON_INTERVAL       400
#define ADVERTISING_SLOW_LED_OFF_INTERVAL      4000

#define BONDING_INTERVAL                       100

#define SENT_OK_INTERVAL                       100
#define SEND_ERROR_INTERVAL                    500

#define RCV_OK_INTERVAL                        100
#define RCV_ERROR_INTERVAL                     500

#define ALERT_INTERVAL                         200

extern bool flag_LED_show;





#if BUTTONS_NUMBER > 0

//°´¼üÁ´
static const uint32_t m_buttons_list[BUTTONS_NUMBER] = BUTTONS_LIST;

#endif // BUTTONS_NUMBER > 0

#define BSP_MS_TO_TICK(MS) (m_app_ticks_per_100ms * (MS / 100))

#ifdef BSP_LED_2_MASK
#define ALERT_LED_MASK BSP_LED_2_MASK
#else
#define ALERT_LED_MASK BSP_LED_1_MASK
#endif // LED_2_MASK

uint32_t bsp_buttons_state_get(uint32_t * p_buttons_state)
{
    uint32_t result = NRF_SUCCESS;

    *p_buttons_state = 0;
#if BUTTONS_NUMBER > 0
    uint32_t buttons = ~NRF_GPIO->IN;
    uint32_t cnt;

    for (cnt = 0; cnt < BUTTONS_NUMBER; cnt++)
    {
        if (buttons & (1 << m_buttons_list[cnt]))
        {
            *p_buttons_state |= 1 << cnt;
        }
    }
#endif // BUTTONS_NUMBER > 0

    return result;
}














