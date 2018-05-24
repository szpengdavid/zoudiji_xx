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

/**@file
 *
 * @defgroup bsp_btn_ble BSP: BLE Button Module
 * @{
 * @ingroup bsp
 *
 * @brief Module for controlling BLE behavior through button actions.
 *
 * @details The application must propagate BLE events to the BLE Button Module.
 * Based on these events, the BLE Button Module configures the Board Support Package
 * to generate BSP events for certain button actions. These BSP events should then be
 * handled by the application's BSP event handler.
 *
 */

#ifndef BSP_BTN_BLE_H__
#define BSP_BTN_BLE_H__

#include <stdint.h>
#include "ble.h"
#include "bsp.h"

/**@brief BLE Button Module error handler type. */
typedef void (*bsp_btn_ble_error_handler_t) (uint32_t nrf_error);



/**@brief Function for setting up wakeup buttons before going into sleep mode.
 *
 * @retval NRF_SUCCESS  If the buttons were prepared successfully. Otherwise, a propagated error
 *                      code is returned.
 */
uint32_t bsp_btn_ble_sleep_mode_prepare(void);

/**@brief Function for handling the application's BLE stack events.
 *
 * @details This function handles all events from the BLE stack that are of interest to this module.
 *
 * @param[in] p_ble_evt BLE stack event.
 */
void bsp_btn_ble_on_ble_evt(ble_evt_t * p_ble_evt);

#endif /* BSP_BTN_BLE_H__ */

/** @} */
