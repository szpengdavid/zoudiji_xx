

#include "bsp_btn_ble.h"
#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include "ble.h"
#include "bsp.h"


#define BTN_ID_WAKEUP             0  /**< ID of button used to wake up the application. */
#define BTN_ID_SLEEP              0  /**< ID of button used to put the application into sleep mode. */
#define BTN_ID_DISCONNECT         0  /**< ID of button used to gracefully terminate a connection on long press. */
#define BTN_ID_WAKEUP_BOND_DELETE 1  /**< ID of button used to wake up the application and delete all bonding information. */
#define BTN_ID_WHITELIST_OFF      1  /**< ID of button used to turn off usage of the whitelist. */

#define BTN_ACTION_SLEEP          BSP_BUTTON_ACTION_RELEASE    /**< Button action used to put the application into sleep mode. */
#define BTN_ACTION_DISCONNECT     BSP_BUTTON_ACTION_LONG_PUSH  /**< Button action used to gracefully terminate a connection on long press. */
#define BTN_ACTION_WHITELIST_OFF  BSP_BUTTON_ACTION_LONG_PUSH  /**< Button action used to turn off usage of the whitelist. */



/**@brief This macro will return from the current function if err_code
 *        is not NRF_SUCCESS.
 */
#define RETURN_ON_ERROR(err_code)  \
do                                 \
{                                  \
    if ((err_code) != NRF_SUCCESS) \
    {                              \
        return err_code;           \
    }                              \
}                                  \
while(0)


/**@brief This macro will return from the current function if err_code
 *        is not NRF_SUCCESS or NRF_ERROR_INVALID_PARAM.
 */
#define RETURN_ON_ERROR_NOT_INVALID_PARAM(err_code)                             \
do                                                                              \
{                                                                               \
    if (((err_code) != NRF_SUCCESS) && ((err_code) != NRF_ERROR_INVALID_PARAM)) \
    {                                                                           \
        return err_code;                                                        \
    }                                                                           \
}                                                                               \
while(0)


/**@brief This macro will return from the current function if err_code
 *        is not NRF_SUCCESS or NRF_ERROR_NOT_SUPPORTED.
 */
#define RETURN_ON_ERROR_NOT_NOT_SUPPORTED(err_code)                             \
do                                                                              \
{                                                                               \
    if (((err_code) != NRF_SUCCESS) && ((err_code) != NRF_ERROR_NOT_SUPPORTED)) \
    {                                                                           \
        return err_code;                                                        \
    }                                                                           \
}                                                                               \
while(0)


/**@brief This macro will call the registered error handler if err_code
 *        is not NRF_SUCCESS and the error handler is not NULL.
 */
#define CALL_HANDLER_ON_ERROR(err_code)                           \
do                                                                \
{                                                                 \
    if (((err_code) != NRF_SUCCESS) && (m_error_handler != NULL)) \
    {                                                             \
        m_error_handler(err_code);                                \
    }                                                             \
}                                                                 \
while(0)


static bsp_btn_ble_error_handler_t m_error_handler = NULL; /**< Error handler registered by the user. */
static uint32_t                    m_num_connections = 0;  /**< Number of connections the device is currently in. */









void bsp_btn_ble_on_ble_evt(ble_evt_t * p_ble_evt)
{

    switch (p_ble_evt->header.evt_id)
    {
        case BLE_GAP_EVT_CONNECTED:
            if (m_num_connections == 0)
            {
            }

            m_num_connections++;
            break;

        case BLE_GAP_EVT_DISCONNECTED:
            m_num_connections--;

            if (m_num_connections == 0)
            {
            }
            break;

        default:
            break;
    }
}

