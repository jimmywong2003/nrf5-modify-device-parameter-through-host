/**
 * Copyright (c) 2014 - 2019, Nordic Semiconductor ASA
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
/** @file
 *
 * @defgroup ble_sdk_uart_over_ble_main main.c
 * @{
 * @ingroup  ble_sdk_app_nus_eval
 * @brief    UART over BLE application main file.
 *
 * This file contains the source code for a sample application that uses the Nordic UART service.
 * This application uses the @ref srvlib_conn_params module.
 */


#include <stdint.h>
#include <string.h>
#include "nordic_common.h"
#include "nrf.h"
#include "ble_hci.h"
#include "ble_advdata.h"
#include "ble_advertising.h"
#include "ble_conn_params.h"
#include "nrf_sdh.h"
#include "nrf_sdh_soc.h"
#include "nrf_sdh_ble.h"
#include "nrf_ble_gatt.h"
#include "nrf_ble_qwr.h"
#include "app_timer.h"

#include "ble_nus.h"
#include "ble_bas.h"
#include "ble_tcs.h"

#include "app_uart.h"
#include "app_util_platform.h"
#include "bsp_btn_ble.h"
#include "nrf_pwr_mgmt.h"

#include "nrf_drv_saadc.h"

#include "app_scheduler.h"
#include "nrf_delay.h"

#if defined (UART_PRESENT)
#include "nrf_uart.h"
#endif
#if defined (UARTE_PRESENT)
#include "nrf_uarte.h"
#endif

#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"
#include "macros_common.h"


#define APP_BLE_CONN_CFG_TAG            1                                           /**< A tag identifying the SoftDevice BLE configuration. */

#define NRF_BL_CONNECT_MODE_BUTTON_PIN BSP_BUTTON_0

#define TX_POWER_BUTTON BSP_BUTTON_1
#define APP_STATE_BUTTON BSP_BUTTON_2

#define DEVICE_NAME                     "UART_ADV"                                    /**< Name of device. Will be included in the advertising data. */
#define NORDIC_COMPANY_ID               0x0059                                      /**< Nordic Semiconductor ASA company identifier. */

#define BUTTON_DETECTION_DELAY APP_TIMER_TICKS(50) /**< Delay from a GPIOTE event until a button is reported as pushed (in number of timer ticks). */

#define NUS_SERVICE_UUID_TYPE           BLE_UUID_TYPE_VENDOR_BEGIN                  /**< UUID type for the Nordic UART Service (vendor specific). */
#define TCS_SERVICE_UUID_TYPE           BLE_UUID_TYPE_VENDOR_BEGIN+1

#define APP_BLE_OBSERVER_PRIO           3                                           /**< Application's BLE observer priority. You shouldn't need to modify this value. */
#define APP_SOC_OBSERVER_PRIO           1

#define APP_ADV_INTERVAL                64                                          /**< The advertising interval (in units of 0.625 ms. This value corresponds to 40 ms). */

#define APP_ADV_DURATION                0                                       /**< The advertising duration (180 seconds) in units of 10 milliseconds. */

#define APP_ADV_INTERVAL_MS             380                                         /**< The advertising interval in ms. */
#define APP_ADV_TIMEOUT_IN_SECONDS      180                                         /**< The advertising timeout in s. */

#define MIN_CONN_INTERVAL_MS            7.5                                         /**< Minimum acceptable connection interval in ms. */
#define MAX_CONN_INTERVAL_MS            30                                          /**< Maximum acceptable connection interval in ms. */
#define SLAVE_LATENCY                   0                                           /**< Slave latency. */
#define CONN_SUP_TIMEOUT_MS             3200                                        /**< Connection supervisory timeout (4 seconds), Supervision Timeout uses 10 ms units. */
#define FIRST_CONN_PARAMS_UPDATE_DELAY  APP_TIMER_TICKS(5000)  /**< Time from initiating event (connect or start of notification) to first time sd_ble_gap_conn_param_update is called (1 second). */
#define NEXT_CONN_PARAMS_UPDATE_DELAY   APP_TIMER_TICKS(30000) /**< Time between each call to sd_ble_gap_conn_param_update after the first call (30 seconds). */
#define MAX_CONN_PARAMS_UPDATE_COUNT    3                                           /**< Number of attempts before giving up the connection parameter negotiation. */

// #define MIN_CONN_INTERVAL               MSEC_TO_UNITS(20, UNIT_1_25_MS)             /**< Minimum acceptable connection interval (20 ms), Connection interval uses 1.25 ms units. */
// #define MAX_CONN_INTERVAL               MSEC_TO_UNITS(75, UNIT_1_25_MS)             /**< Maximum acceptable connection interval (75 ms), Connection interval uses 1.25 ms units. */
//#define SLAVE_LATENCY                   0                                           /**< Slave latency. */
//#define CONN_SUP_TIMEOUT                MSEC_TO_UNITS(4000, UNIT_10_MS)             /**< Connection supervisory timeout (4 seconds), Supervision Timeout uses 10 ms units. */
// #define FIRST_CONN_PARAMS_UPDATE_DELAY  APP_TIMER_TICKS(5000)                       /**< Time from initiating event (connect or start of notification) to first time sd_ble_gap_conn_param_update is called (5 seconds). */
// #define NEXT_CONN_PARAMS_UPDATE_DELAY   APP_TIMER_TICKS(30000)                      /**< Time between each call to sd_ble_gap_conn_param_update after the first call (30 seconds). */
// #define MAX_CONN_PARAMS_UPDATE_COUNT    3                                           /**< Number of attempts before giving up the connection parameter negotiation. */


#define NON_CONNECTABLE_ADV_INTERVAL    MSEC_TO_UNITS(100, UNIT_0_625_MS)  /**< The advertising interval for non-connectable advertisement (100 ms). This value can vary between 100ms to 10.24s). */

#define APP_BEACON_INFO_LENGTH          0x17                               /**< Total length of information advertised by the Beacon. */
#define APP_ADV_DATA_LENGTH             0x15                               /**< Length of manufacturer specific data in the advertisement. */
#define APP_DEVICE_TYPE                 0x02                               /**< 0x02 refers to Beacon. */
#define APP_MEASURED_RSSI               0xC3                               /**< The Beacon's measured RSSI at 1 meter distance in dBm. */
#define APP_COMPANY_IDENTIFIER          0x0059                             /**< Company identifier for Nordic Semiconductor ASA. as per www.bluetooth.org. */
#define APP_MAJOR_VALUE                 0x01, 0x02                         /**< Major value used to identify Beacons. */
#define APP_MINOR_VALUE                 0x03, 0x04                         /**< Minor value used to identify Beacons. */
#define APP_BEACON_UUID                 0x01, 0x12, 0x23, 0x34, \
        0x45, 0x56, 0x67, 0x78, \
        0x89, 0x9a, 0xab, 0xbc, \
        0xcd, 0xde, 0xef, 0xf0                                            /**< Proprietary UUID for Beacon. */

#if defined(USE_UICR_FOR_MAJ_MIN_VALUES)
#define MAJ_VAL_OFFSET_IN_BEACON_INFO   18                                 /**< Position of the MSB of the Major Value in m_beacon_info array. */
#define UICR_ADDRESS                    0x10001080                         /**< Address of the UICR register used by this example. The major and minor versions to be encoded into the advertising data will be picked up from this location. */
#endif


static uint8_t m_beacon_info[APP_BEACON_INFO_LENGTH] =                    /**< Information advertised by the Beacon. */
{
        APP_DEVICE_TYPE, // Manufacturer specific information. Specifies the device type in this
                         // implementation.
        APP_ADV_DATA_LENGTH, // Manufacturer specific information. Specifies the length of the
                             // manufacturer specific data in this implementation.
        APP_BEACON_UUID, // 128 bit UUID value.
        APP_MAJOR_VALUE, // Major arbitrary value that can be used to distinguish between Beacons.
        APP_MINOR_VALUE, // Minor arbitrary value that can be used to distinguish between Beacons.
        APP_MEASURED_RSSI // Manufacturer specific information. The Beacon's measured TX power in
                          // this implementation.
};


#define BATTERY_LEVEL_MEAS_INTERVAL         APP_TIMER_TICKS(2000)                   /**< Battery level measurement interval (ticks). */
#define MIN_BATTERY_LEVEL                   81                                      /**< Minimum simulated battery level. */
#define MAX_BATTERY_LEVEL                   100                                     /**< Maximum simulated 7battery level. */
#define BATTERY_LEVEL_INCREMENT             1                                       /**< Increment between each simulated battery level measurement. */

#define ADC_REF_VOLTAGE_IN_MILLIVOLTS   600                                     /**< Reference voltage (in milli volts) used by ADC while doing conversion. */
#define ADC_PRE_SCALING_COMPENSATION    6                                       /**< The ADC is configured to use VDD with 1/3 prescaling as input. And hence the result of conversion is to be multiplied by 3 to get the actual value of the battery voltage.*/
#define DIODE_FWD_VOLT_DROP_MILLIVOLTS  270                                     /**< Typical forward voltage drop of the diode . */
#define ADC_RES_10BIT                   1024                                    /**< Maximum digital value for 10-bit ADC conversion. */


#define DEAD_BEEF                       0xDEADBEEF                                  /**< Value used as error code on stack dump, can be used to identify stack location on stack unwind. */

#define UART_TX_BUF_SIZE                256                                         /**< UART TX buffer size. */
#define UART_RX_BUF_SIZE                256                                         /**< UART RX buffer size. */

#define SCHED_MAX_EVENT_DATA_SIZE           APP_TIMER_SCHED_EVENT_DATA_SIZE            /**< Maximum size of scheduler events. */
#ifdef SVCALL_AS_NORMAL_FUNCTION
#define SCHED_QUEUE_SIZE                    20                                         /**< Maximum number of events in the scheduler queue. More is needed in case of Serialization. */
#else
#define SCHED_QUEUE_SIZE                    10                                         /**< Maximum number of events in the scheduler queue. */
#endif

#define TX_POWER_LEVEL                  (4)                                    /**< TX Power Level value. This will be set both in the TX Power service, in the advertising data, and also used to set the radio transmit power. */

/**@brief Thingy default beacon configuration. Eddystone url */
#define THINGY_BEACON_ADV_INTERVAL      760                 /**< The Beacon's advertising interval, in milliseconds*/
#define THINGY_BEACON_URL_DEFAULT       "\x03goo.gl/pIWdir" /**< https://goo.gl/pIWdir short for https://developer.nordicsemi.com/thingy/52/ */
#define THINGY_BEACON_URL_LEN           14

#define THINGY_DEFAULT_PASSWORD         "0000"

#define THINGY_ADV_PAYLOAD_DEFAULT      "01234567890123456789012"
#define THINGY_ADV_PAYLOAD_LEN           23

/**@brief Thingy FW version.
 * 0xFF indicates a custom build from source.
   Version numbers are changed for releases. */
#define THINGY_FW_VERSION_MAJOR     (0xFF)
#define THINGY_FW_VERSION_MINOR     (0xFF)
#define THINGY_FW_VERSION_PATCH     (0xFF)

/**@brief Thingy default configuration. */
#define THINGY_CONFIG_DEFAULT                         \
        {                                                     \
                .dev_name =                                       \
                {                                                 \
                        .name = DEVICE_NAME,                          \
                        .len = 10                                     \
                },                                                \
                .adv_params =                                     \
                {                                                 \
                        .interval = MSEC_TO_UNITS(APP_ADV_INTERVAL_MS, UNIT_0_625_MS),                  \
                        .timeout = APP_ADV_TIMEOUT_IN_SECONDS         \
                },                                                \
                .conn_params =                                    \
                {                                                 \
                        .min_conn_int  = (uint16_t)MSEC_TO_UNITS(MIN_CONN_INTERVAL_MS, UNIT_1_25_MS),   \
                        .max_conn_int  = MSEC_TO_UNITS(MAX_CONN_INTERVAL_MS, UNIT_1_25_MS),             \
                        .slave_latency = SLAVE_LATENCY,                                                 \
                        .sup_timeout   = MSEC_TO_UNITS(CONN_SUP_TIMEOUT_MS, UNIT_10_MS)                 \
                },                                                \
                .eddystone_url =                                  \
                {                                                 \
                        .data = THINGY_BEACON_URL_DEFAULT,            \
                        .len  = THINGY_BEACON_URL_LEN                 \
                },                                                \
                .fw_version =                                     \
                {                                                 \
                        .major = THINGY_FW_VERSION_MAJOR,             \
                        .minor = THINGY_FW_VERSION_MINOR,             \
                        .patch = THINGY_FW_VERSION_PATCH              \
                },                                                \
                .mtu =                                            \
                {                                                 \
                        .req = 0x00,                                  \
                        .size = 23                                    \
                },                                                 \
                .tx_power =                                        \
                {                                                   \
                        .tx_power = TX_POWER_LEVEL,                 \
                },                                                  \
                .pwd =                                            \
                {                                                 \
                        .data = THINGY_DEFAULT_PASSWORD,           \
                },                                                 \
                .adv_payload =                                  \
                {                                                 \
                        .data = THINGY_ADV_PAYLOAD_DEFAULT,            \
                        .len  = THINGY_ADV_PAYLOAD_LEN                 \
                },                                                    \
        }


#ifdef NRF_PWD_BLE_ENABLED

APP_TIMER_DEF(m_pwd_timer_id);                                          /**< Battery timer. */
#define PASSWORD_TIMEOUT_INTERVAL         APP_TIMER_TICKS(NRF_PWD_TIMEOUT_PERIOD)                   /**< Battery level measurement interval (ticks). */

static bool m_pwd_is_verified = false;

#endif


#define SUPPORT_FUNC_MAC_ADDR_STR_LEN 6

static ble_gap_adv_params_t m_adv_params;                                  /**< Parameters to be passed to the stack when starting advertising. */
static uint8_t m_adv_handle = BLE_GAP_ADV_SET_HANDLE_NOT_SET;              /**< Advertising handle used to identify an advertising set. */
static uint8_t m_enc_advdata[BLE_GAP_ADV_SET_DATA_SIZE_MAX];               /**< Buffer for storing an encoded advertising set. */

static int8_t m_tx_power = TX_POWER_LEVEL;

BLE_BAS_DEF(m_bas);                                                 /**< Structure used to identify the battery service. */
BLE_NUS_DEF(m_nus, NRF_SDH_BLE_TOTAL_LINK_COUNT);                                   /**< BLE NUS service instance. */
BLE_TCS_DEF(m_tcs);

NRF_BLE_GATT_DEF(m_gatt);                                                           /**< GATT module instance. */
NRF_BLE_QWR_DEF(m_qwr);                                                             /**< Context for the Queued Write module.*/
BLE_ADVERTISING_DEF(m_advertising);                                                 /**< Advertising module instance. */

APP_TIMER_DEF(m_battery_timer_id);                                  /**< Battery timer. */

/* Battery detection service */
static nrf_saadc_value_t adc_buf[2];
static void on_bas_evt(ble_bas_t * p_bas, ble_bas_evt_t * p_evt);

/* Thingy Configure Service */
static ble_tcs_params_t         * m_ble_config;
static const ble_tcs_params_t m_ble_default_config = THINGY_CONFIG_DEFAULT;

static ble_tcs_mtu_t m_mtu;

static bool m_flash_disconnect = false;
static bool m_major_minor_fw_ver_changed = false;

static uint16_t m_conn_handle          = BLE_CONN_HANDLE_INVALID;                   /**< Handle of the current connection. */
static uint16_t m_ble_nus_max_data_len = BLE_GATT_ATT_MTU_DEFAULT - 3;              /**< Maximum length of data (in bytes) that can be transmitted to the peer by the Nordic UART service module. */

static ble_uuid_t m_adv_uuids[]          =                                          /**< Universally unique service identifier. */
{
        {BLE_UUID_NUS_SERVICE, NUS_SERVICE_UUID_TYPE}
};

/**@brief Macro to convert the result of ADC conversion in millivolts.
 *
 * @param[in]  ADC_VALUE   ADC result.
 *
 * @retval     Result converted to millivolts.
 */
#define ADC_RESULT_IN_MILLI_VOLTS(ADC_VALUE) \
        ((((ADC_VALUE) *ADC_REF_VOLTAGE_IN_MILLIVOLTS) / ADC_RES_10BIT) * ADC_PRE_SCALING_COMPENSATION)


/**@brief Struct that contains pointers to the encoded advertising data. */
static ble_gap_adv_data_t m_adv_data =
{
        .adv_data =
        {
                .p_data = m_enc_advdata,
                .len    = BLE_GAP_ADV_SET_DATA_SIZE_MAX
        },
        .scan_rsp_data =
        {
                .p_data = NULL,
                .len    = 0

        }
};



/**@brief Function for initializing button used to enter DFU mode.
 */
static void enter_button_init(void)
{
        nrf_gpio_cfg_sense_input(NRF_BL_CONNECT_MODE_BUTTON_PIN,
                                 BUTTON_PULL,
                                 NRF_GPIO_PIN_SENSE_LOW);
}

/**@brief Function for checking whether to enter DFU mode or not.
 */
static bool connect_adv_enter_check(void)
{
        if (nrf_gpio_pin_read(NRF_BL_CONNECT_MODE_BUTTON_PIN) == 0)
        {
                //NRF_LOG_DEBUG("DFU mode requested via button.");
                return true;
        }
        return false;
}

/**@brief Function for handling the ADC interrupt.
 *
 * @details  This function will fetch the conversion result from the ADC, convert the value into
 *           percentage and send it to peer.
 */
void saadc_event_handler(nrf_drv_saadc_evt_t const * p_event)
{
        if (p_event->type == NRF_DRV_SAADC_EVT_DONE)
        {
                nrf_saadc_value_t adc_result;
                uint16_t batt_lvl_in_milli_volts;
                uint8_t percentage_batt_lvl;
                uint32_t err_code;

                adc_result = p_event->data.done.p_buffer[0];

                err_code = nrf_drv_saadc_buffer_convert(p_event->data.done.p_buffer, 1);
                APP_ERROR_CHECK(err_code);

                batt_lvl_in_milli_volts = ADC_RESULT_IN_MILLI_VOLTS(adc_result) +
                                          DIODE_FWD_VOLT_DROP_MILLIVOLTS;
                percentage_batt_lvl = battery_level_in_percent(batt_lvl_in_milli_volts);

                NRF_LOG_DEBUG("Battery service value : %03d %%", percentage_batt_lvl);

                if (m_conn_handle != BLE_CONN_HANDLE_INVALID)
                {
                        err_code = ble_bas_battery_level_update(&m_bas, percentage_batt_lvl, BLE_CONN_HANDLE_ALL);
                        if ((err_code != NRF_SUCCESS) &&
                            (err_code != NRF_ERROR_INVALID_STATE) &&
                            (err_code != NRF_ERROR_RESOURCES) &&
                            (err_code != NRF_ERROR_BUSY) &&
                            (err_code != BLE_ERROR_GATTS_SYS_ATTR_MISSING)
                            )
                        {
                                APP_ERROR_HANDLER(err_code);
                        }
                }
        }
}

/**@brief Function for configuring ADC to do battery level conversion.
 */
static void adc_configure(void)
{
        ret_code_t err_code = nrf_drv_saadc_init(NULL, saadc_event_handler);
        APP_ERROR_CHECK(err_code);

        nrf_saadc_channel_config_t config =
                NRF_DRV_SAADC_DEFAULT_CHANNEL_CONFIG_SE(NRF_SAADC_INPUT_VDD);

        err_code = nrf_drv_saadc_channel_init(0, &config);
        APP_ERROR_CHECK(err_code);

        err_code = nrf_drv_saadc_buffer_convert(&adc_buf[0], 1);
        APP_ERROR_CHECK(err_code);

        err_code = nrf_drv_saadc_buffer_convert(&adc_buf[1], 1);
        APP_ERROR_CHECK(err_code);
}

/**@brief Function for handling the Battery measurement timer timeout.
 *
 * @details This function will be called each time the battery level measurement timer expires.
 *          This function will start the ADC.
 *
 * @param[in] p_context   Pointer used for passing some arbitrary information (context) from the
 *                        app_start_timer() call to the timeout handler.
 */
static void battery_level_meas_timeout_handler(void * p_context)
{
        UNUSED_PARAMETER(p_context);

        ret_code_t err_code;
        err_code = nrf_drv_saadc_sample();
        APP_ERROR_CHECK(err_code);
}

static void password_timeout_handler(void *p_context)
{
        UNUSED_PARAMETER(p_context);

        if (m_pwd_is_verified == false && m_conn_handle != BLE_CONN_HANDLE_INVALID)
        {
                NRF_LOG_INFO("Verify the password failure!! Disconnect the LINK!!!");
                /* Disconnect from the peer. */
                ret_code_t err_code = sd_ble_gap_disconnect(m_conn_handle, BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
                VERIFY_SUCCESS(err_code);
        }
}


/**@brief Check if flash is currently being accessed.
 */
static bool flash_access_ongoing(void)
{
        if (nrf_fstorage_is_busy(NULL))
        {
                NRF_LOG_INFO("Waiting until all flash operations are completed.");
                return true;
        }
        else
                return false;
}

/**@brief Function for assert macro callback.
 *
 * @details This function will be called in case of an assert in the SoftDevice.
 *
 * @warning This handler is an example only and does not fit a final product. You need to analyse
 *          how your product is supposed to react in case of Assert.
 * @warning On assert from the SoftDevice, the system can only recover on reset.
 *
 * @param[in] line_num    Line number of the failing ASSERT call.
 * @param[in] p_file_name File name of the failing ASSERT call.
 */
void assert_nrf_callback(uint16_t line_num, const uint8_t * p_file_name)
{
        app_error_handler(DEAD_BEEF, line_num, p_file_name);
}

/**@brief Function for initializing the timer module.
 */
static void timers_init(void)
{
        ret_code_t err_code = app_timer_init();
        APP_ERROR_CHECK(err_code);

        // Create battery timer.
        err_code = app_timer_create(&m_battery_timer_id,
                                    APP_TIMER_MODE_REPEATED,
                                    battery_level_meas_timeout_handler);
        APP_ERROR_CHECK(err_code);

#ifdef NRF_PWD_BLE_ENABLED
        // Create battery timer.
        err_code = app_timer_create(&m_pwd_timer_id,
                                    APP_TIMER_MODE_SINGLE_SHOT,
                                    password_timeout_handler);
        APP_ERROR_CHECK(err_code);
#endif

}

static void application_timer_start(void)
{
        // Start battery timer
        ret_code_t err_code =app_timer_start(m_battery_timer_id, BATTERY_LEVEL_MEAS_INTERVAL, NULL);
        APP_ERROR_CHECK(err_code);

}


/**@brief Function for changing the tx power.
 */
static void tx_power_set(void)
{
        int tx_power = m_ble_config->tx_power.tx_power;
        NRF_LOG_INFO("TX Power set = %d", tx_power);
        ret_code_t err_code = sd_ble_gap_tx_power_set(BLE_GAP_TX_POWER_ROLE_ADV, m_adv_handle, tx_power);
        APP_ERROR_CHECK(err_code);
}

/**@brief Checks the current version of the FW against the previous version stored in flash.
 * If a major or minor FW change is detected, modules must reinitialize their flash storage.
 *
 * @note: If the FW version is changed while erasing all flash, a FW change cannot be detected.
 */
static uint32_t device_config_verify(void)
{
        bool update_flash = false;
        uint32_t err_code;

        bool fw_version_major_changed = ( m_ble_config->fw_version.major != m_ble_default_config.fw_version.major );
        bool fw_version_minor_changed = ( m_ble_config->fw_version.minor != m_ble_default_config.fw_version.minor );
        bool fw_version_patch_changed = ( m_ble_config->fw_version.patch != m_ble_default_config.fw_version.patch );

        ble_tcs_fw_version_t prev_fw_version = m_ble_config->fw_version;

        if ( fw_version_major_changed || fw_version_minor_changed || fw_version_patch_changed)
        {
                m_ble_config->fw_version.major = m_ble_default_config.fw_version.major;
                m_ble_config->fw_version.minor = m_ble_default_config.fw_version.minor;
                m_ble_config->fw_version.patch = m_ble_default_config.fw_version.patch;

                update_flash = true;

                if(fw_version_major_changed || fw_version_minor_changed)
                {
                        update_flash = false;
                        m_major_minor_fw_ver_changed = true;

                        err_code = m_ble_flash_config_store(&m_ble_default_config);
                        APP_ERROR_CHECK(err_code);
                }
        }

        NRF_LOG_INFO("m_ble: Current FW: v%d.%d.%d \r\n",
                     m_ble_default_config.fw_version.major, m_ble_default_config.fw_version.minor, m_ble_default_config.fw_version.patch);

        if(m_major_minor_fw_ver_changed)
        {
                NRF_LOG_INFO("m_ble: Major or minor FW version changed. Prev. FW (from flash): v%d.%d.%d \r\n",
                             prev_fw_version.major, prev_fw_version.minor, prev_fw_version.patch);
        }

        NRF_LOG_INFO("m_ble: TX Power = %d", m_ble_config->tx_power.tx_power);

        NRF_LOG_INFO("Password %c%c%c%c", m_ble_config->pwd.data[0], m_ble_config->pwd.data[1], m_ble_config->pwd.data[2], m_ble_config->pwd.data[3]);

        // Check Eddystone URL length.
        if (m_ble_config->eddystone_url.len > 17)
        {
                memcpy(m_ble_config->eddystone_url.data, m_ble_default_config.eddystone_url.data, m_ble_default_config.eddystone_url.len);
                m_ble_config->eddystone_url.len = m_ble_default_config.eddystone_url.len;
                update_flash = true;
        }

        if (update_flash)
        {
                err_code = m_ble_flash_config_store(m_ble_config, true);
                APP_ERROR_CHECK(err_code);
        }

        return NRF_SUCCESS;
}


/**@brief Function for the GAP initialization.
 *
 * @details This function will set up all the necessary GAP (Generic Access Profile) parameters of
 *          the device. It also sets the permissions and appearance.
 */
static void gap_params_init(bool load_setting)
{
        uint32_t err_code;
        ble_gap_conn_params_t gap_conn_params;
        ble_gap_conn_sec_mode_t sec_mode;

        BLE_GAP_CONN_SEC_MODE_SET_OPEN(&sec_mode);

        if (load_setting)
        {
                if (m_ble_config)
                {
                        err_code = sd_ble_gap_device_name_set(&sec_mode,
                                                              m_ble_config->dev_name.name,
                                                              strlen((const char *)m_ble_config->dev_name.name));
                        APP_ERROR_CHECK(err_code);
                        NRF_LOG_INFO("gap_params device name = %s", m_ble_config->dev_name.name);
                }
        }
        else
        {
                err_code = sd_ble_gap_device_name_set(&sec_mode,
                                                      (const uint8_t *) DEVICE_NAME,
                                                      strlen(DEVICE_NAME));
                APP_ERROR_CHECK(err_code);
        }

        memset(&gap_conn_params, 0, sizeof(gap_conn_params));
        if (load_setting)
        {

                gap_conn_params.min_conn_interval = m_ble_config->conn_params.min_conn_int;
                gap_conn_params.max_conn_interval = m_ble_config->conn_params.max_conn_int;
                gap_conn_params.slave_latency     = m_ble_config->conn_params.slave_latency;
                gap_conn_params.conn_sup_timeout  = m_ble_config->conn_params.sup_timeout;
        }
        else
        {
                gap_conn_params.min_conn_interval = MIN_CONN_INTERVAL_MS;
                gap_conn_params.max_conn_interval = MAX_CONN_INTERVAL_MS;
                gap_conn_params.slave_latency     = SLAVE_LATENCY;
                gap_conn_params.conn_sup_timeout  = CONN_SUP_TIMEOUT_MS;
        }
        err_code = sd_ble_gap_ppcp_set(&gap_conn_params);
        APP_ERROR_CHECK(err_code);

        ble_gap_addr_t ble_address = {.addr_type = BLE_GAP_ADDR_TYPE_RANDOM_STATIC,
                                      .addr_id_peer = 0,
                                      .addr = {0xC3,0x11,0x99,0x33,0x44,0xFF}};
        err_code = sd_ble_gap_addr_set(&ble_address);
}


/**@brief Function for handling Queued Write Module errors.
 *
 * @details A pointer to this function will be passed to each service which may need to inform the
 *          application about an error.
 *
 * @param[in]   nrf_error   Error code containing information about what went wrong.
 */
static void nrf_qwr_error_handler(uint32_t nrf_error)
{
        APP_ERROR_HANDLER(nrf_error);
}


/**@brief Function for handling the data from the Nordic UART Service.
 *
 * @details This function will process the data received from the Nordic UART BLE Service and send
 *          it to the UART module.
 *
 * @param[in] p_evt       Nordic UART Service event.
 */
/**@snippet [Handling the data received over BLE] */
static void nus_data_handler(ble_nus_evt_t * p_evt)
{

        if (p_evt->type == BLE_NUS_EVT_RX_DATA)
        {
                uint32_t err_code;

                NRF_LOG_INFO("Received data from BLE NUS. Writing data on UART.");
                NRF_LOG_HEXDUMP_INFO(p_evt->params.rx_data.p_data, p_evt->params.rx_data.length);

                for (uint32_t i = 0; i < p_evt->params.rx_data.length; i++)
                {
                        do
                        {
                                err_code = app_uart_put(p_evt->params.rx_data.p_data[i]);
                                if ((err_code != NRF_SUCCESS) && (err_code != NRF_ERROR_BUSY))
                                {
                                        NRF_LOG_ERROR("Failed receiving NUS message. Error 0x%x. ", err_code);
                                        APP_ERROR_CHECK(err_code);
                                }
                        } while (err_code == NRF_ERROR_BUSY);
                }
                if (p_evt->params.rx_data.p_data[p_evt->params.rx_data.length - 1] == '\r')
                {
                        while (app_uart_put('\n') == NRF_ERROR_BUSY);
                }
        }

}
/**@snippet [Handling the data received over BLE] */

static void ble_on_sys_evt(uint32_t sys_evt, void *p_context)
{
        switch(sys_evt)
        {
        case NRF_EVT_FLASH_OPERATION_ERROR:
        case NRF_EVT_FLASH_OPERATION_SUCCESS:
                //if (s_waiting_for_flash)
        {
                if (!nrf_fstorage_is_busy(NULL))
                {
                        uint32_t err_code;
                        NRF_LOG_DEBUG("Flash Ready.");

                        if (m_conn_handle != BLE_CONN_HANDLE_INVALID)
                        {
                                // Disconnect on GATT Server timeout event.
                                err_code = sd_ble_gap_disconnect(m_conn_handle,
                                                                 BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
                                APP_ERROR_CHECK(err_code);
                        }

                        nrf_delay_ms(1000);
                        nrf_pwr_mgmt_shutdown(NRF_PWR_MGMT_SHUTDOWN_RESET);

                }
        }
        break;
        default:
                /* Ignore */
                break;
        }
}

NRF_SDH_SOC_OBSERVER(ble_soc_observer, APP_SOC_OBSERVER_PRIO, ble_on_sys_evt, NULL);

/**@brief Function for handling thingy configuration events.
 */
static void tcs_evt_handler (ble_tcs_t        * p_tcs,
                             ble_tcs_evt_type_t evt_type,
                             uint8_t          * p_data,
                             uint16_t length)
{
        bool update_flash = false;

        if (evt_type == BLE_TCS_EVT_PWD_VERIFY)
        {
                if (strncmp(p_data, m_ble_config->pwd.data, length)==0)
                {
                        m_pwd_is_verified = true;
                        NRF_LOG_INFO("Verification Pass!!");
                }
        }

        if (m_pwd_is_verified)
        {
                switch (evt_type)
                {
                case BLE_TCS_EVT_DEV_NAME:
                        if (length <= BLE_TCS_DEVICE_NAME_LEN_MAX)
                        {
                                memcpy(m_ble_config->dev_name.name, p_data, length);
                                m_ble_config->dev_name.name[length] = 0;
                                m_ble_config->dev_name.len = length;
                                update_flash = true;

                                NRF_LOG_INFO("Change the Name: %s", m_ble_config->dev_name.name)
                        }
                        break;
                case BLE_TCS_EVT_ADV_PARAM:
                        if (length == sizeof(ble_tcs_adv_params_t))
                        {
                                NRF_LOG_INFO("Update the Advertising parameter!");
                                NRF_LOG_HEXDUMP_INFO(p_data, length);
                                memcpy(&m_ble_config->adv_params, p_data, length);
                                update_flash = true;
                        }
                        break;
                case BLE_TCS_EVT_CONN_PARAM:
                        if (length == sizeof(ble_tcs_conn_params_t))
                        {
                                uint32_t err_code;
                                ble_gap_conn_params_t gap_conn_params;

                                memcpy(&m_ble_config->conn_params, p_data, length);
                                memset(&gap_conn_params, 0, sizeof(gap_conn_params));

                                gap_conn_params.min_conn_interval = m_ble_config->conn_params.min_conn_int;
                                gap_conn_params.max_conn_interval = m_ble_config->conn_params.max_conn_int;
                                gap_conn_params.slave_latency     = m_ble_config->conn_params.slave_latency;
                                gap_conn_params.conn_sup_timeout  = m_ble_config->conn_params.sup_timeout;

                                err_code = ble_conn_params_change_conn_params(m_conn_handle, &gap_conn_params);
                                APP_ERROR_CHECK(err_code);

                                update_flash = true;
                        }
                        break;
                case BLE_TCS_EVT_BEACON:
                        if (length <= BLE_TCS_BEACON_LEN_MAX)
                        {
                                uint32_t err_code;

                                memcpy(m_ble_config->eddystone_url.data, p_data, length);
                                m_ble_config->eddystone_url.len = length;
                                update_flash = true;

//                        err_code = timeslot_init();
//                        APP_ERROR_CHECK(err_code);
                        }
                        break;
                case BLE_TCS_EVT_MTU:
                        if (length == sizeof(ble_tcs_mtu_t))
                        {
                                uint32_t err_code;
                                ble_tcs_mtu_t * p_mtu = (ble_tcs_mtu_t *)p_data;

                                if (p_mtu->req == TCS_MTU_REQ_EXCHANGE)
                                {
                                        NRF_LOG_INFO("tcs_evt_handler: TCS_MTU_REQ_EXCHANGE - %d\r\n", p_mtu->size);
                                        err_code = sd_ble_gattc_exchange_mtu_request(m_conn_handle, p_mtu->size);
                                        if (err_code == NRF_SUCCESS)
                                        {
                                                memcpy(&m_mtu, p_data, length);
                                        }
                                        else
                                        {
                                                err_code = ble_tcs_mtu_set(&m_tcs, &m_mtu);
                                                APP_ERROR_CHECK(err_code);
                                        }
                                }
                                else
                                {
                                        err_code = ble_tcs_mtu_set(&m_tcs, &m_mtu);
                                        APP_ERROR_CHECK(err_code);
                                }
                        }
                        break;

                case BLE_TCS_EVT_TX_POWER:
                        NRF_LOG_INFO("BLE_TCS_EVT_TX_POWER %d", sizeof(ble_tcs_tx_power_t));
                        if (length == sizeof(ble_tcs_tx_power_t))
                        {
                                uint32_t err_code;
                                memcpy(&m_ble_config->tx_power.tx_power, p_data, length);

                                NRF_LOG_INFO("Store TX Power");
                                NRF_LOG_HEXDUMP_INFO(p_data, length);
                                update_flash = true;
                        }
                        break;
                case BLE_TCS_EVT_PWD:
                        NRF_LOG_INFO("BLE_TCS_EVT_PWD %d", sizeof(ble_tcs_pwd_t));
                        if (length == sizeof(ble_tcs_pwd_t))
                        {
                                // uint32_t err_code;
                                memcpy(m_ble_config->pwd.data, p_data, length);
                                // ble_tcs_pwd_t * p_tx_power = (ble_tcs_pwd_t *)p_data;
                                // m_ble_config->tx_power.tx_power = *p_tx_power;

                                NRF_LOG_HEXDUMP_INFO(p_data, length);
                                update_flash = true;
                        }
                        break;

                case BLE_TCS_EVT_ADV_PAYLOAD:
                        NRF_LOG_INFO("BLE_TCS_EVT_ADV_PAYLOAD");
                        if (length <= BLE_TCS_ADV_PAYLOAD_LEN_MAX)
                        {
                                uint32_t err_code;

                                memcpy(m_ble_config->adv_payload.data, p_data, length);
                                m_ble_config->adv_payload.len = length;

                                NRF_LOG_HEXDUMP_INFO(p_data, length);
                                update_flash = true;
                        }
                        break;


                }
        }

        if (update_flash)
        {
                uint32_t err_code;

                err_code = m_ble_flash_config_store(m_ble_config, false);
                APP_ERROR_CHECK(err_code);
        }
}

/**@brief Function for initializing the Battery Service.
 */
static void bas_init(void)
{
        ret_code_t err_code;
        ble_bas_init_t bas_init_obj;

        memset(&bas_init_obj, 0, sizeof(bas_init_obj));

        bas_init_obj.evt_handler          = on_bas_evt;
        bas_init_obj.support_notification = true;
        bas_init_obj.p_report_ref         = NULL;
        bas_init_obj.initial_batt_level   = 100;

        bas_init_obj.bl_rd_sec        = SEC_OPEN;
        bas_init_obj.bl_cccd_wr_sec   = SEC_OPEN;
        bas_init_obj.bl_report_rd_sec = SEC_OPEN;

        err_code = ble_bas_init(&m_bas, &bas_init_obj);
        APP_ERROR_CHECK(err_code);
}

static void nus_init(void)
{
        uint32_t err_code;
        ble_nus_init_t nus_init;
        memset(&nus_init, 0, sizeof(nus_init));

        nus_init.data_handler = nus_data_handler;

        err_code = ble_nus_init(&m_nus, &nus_init);
        APP_ERROR_CHECK(err_code);

}

static void tcs_init(void)
{
        ble_tcs_init_t tcs_init;
        uint32_t err_code;

        tcs_init.p_init_vals = m_ble_config;
        tcs_init.evt_handler = tcs_evt_handler;

        err_code = ble_tcs_init(&m_tcs, &tcs_init);
        APP_ERROR_CHECK(err_code);
}


/**@brief Function for initializing services that will be used by the application.
 */
static void services_init(void)
{
        uint32_t err_code;

        nrf_ble_qwr_init_t qwr_init = {0};

        // Initialize Queued Write Module.
        qwr_init.error_handler = nrf_qwr_error_handler;

        err_code = nrf_ble_qwr_init(&m_qwr, &qwr_init);
        APP_ERROR_CHECK(err_code);

        tcs_init();

        // Initialize NUS.
        nus_init();

        bas_init();


}


/**@brief Function for handling an event from the Connection Parameters Module.
 *
 * @details This function will be called for all events in the Connection Parameters Module
 *          which are passed to the application.
 *
 * @note All this function does is to disconnect. This could have been done by simply setting
 *       the disconnect_on_fail config parameter, but instead we use the event handler
 *       mechanism to demonstrate its use.
 *
 * @param[in] p_evt  Event received from the Connection Parameters Module.
 */
static void on_conn_params_evt(ble_conn_params_evt_t * p_evt)
{
        uint32_t err_code;

        if (p_evt->evt_type == BLE_CONN_PARAMS_EVT_FAILED)
        {
                err_code = sd_ble_gap_disconnect(m_conn_handle, BLE_HCI_CONN_INTERVAL_UNACCEPTABLE);
                APP_ERROR_CHECK(err_code);
        }
}


/**@brief Function for handling errors from the Connection Parameters module.
 *
 * @param[in] nrf_error  Error code containing information about what went wrong.
 */
static void conn_params_error_handler(uint32_t nrf_error)
{
        APP_ERROR_HANDLER(nrf_error);
}


/**@brief Function for initializing the Connection Parameters module.
 */
static void conn_params_init(void)
{
        uint32_t err_code;
        ble_conn_params_init_t cp_init;

        memset(&cp_init, 0, sizeof(cp_init));

        cp_init.p_conn_params                  = NULL;
        cp_init.first_conn_params_update_delay = FIRST_CONN_PARAMS_UPDATE_DELAY;
        cp_init.next_conn_params_update_delay  = NEXT_CONN_PARAMS_UPDATE_DELAY;
        cp_init.max_conn_params_update_count   = MAX_CONN_PARAMS_UPDATE_COUNT;
        cp_init.start_on_notify_cccd_handle    = BLE_GATT_HANDLE_INVALID;
        cp_init.disconnect_on_fail             = false;
        cp_init.evt_handler                    = on_conn_params_evt;
        cp_init.error_handler                  = conn_params_error_handler;

        err_code = ble_conn_params_init(&cp_init);
        APP_ERROR_CHECK(err_code);
}


/**@brief Function for putting the chip into sleep mode.
 *
 * @note This function will not return.
 */
static void sleep_mode_enter(void)
{
        uint32_t err_code = bsp_indication_set(BSP_INDICATE_IDLE);
        APP_ERROR_CHECK(err_code);

        // Prepare wakeup buttons.
        err_code = bsp_btn_ble_sleep_mode_prepare();
        APP_ERROR_CHECK(err_code);

        // Go to system-off mode (this function will not return; wakeup will cause a reset).
        err_code = sd_power_system_off();
        APP_ERROR_CHECK(err_code);
}


/**@brief Function for handling advertising events.
 *
 * @details This function will be called for advertising events which are passed to the application.
 *
 * @param[in] ble_adv_evt  Advertising event.
 */
static void on_adv_evt(ble_adv_evt_t ble_adv_evt)
{
        uint32_t err_code;

        switch (ble_adv_evt)
        {
        case BLE_ADV_EVT_FAST:
                err_code = bsp_indication_set(BSP_INDICATE_ADVERTISING);
                APP_ERROR_CHECK(err_code);
                break;
        case BLE_ADV_EVT_IDLE:
                // sleep_mode_enter();
                break;
        default:
                break;
        }
}

/**@brief Function for handling the Battery Service events.
 *
 * @details This function will be called for all Battery Service events which are passed to the
 |          application.
 *
 * @param[in] p_bas  Battery Service structure.
 * @param[in] p_evt  Event received from the Battery Service.
 */
static void on_bas_evt(ble_bas_t * p_bas, ble_bas_evt_t * p_evt)
{
        ret_code_t err_code;

        switch (p_evt->evt_type)
        {
        case BLE_BAS_EVT_NOTIFICATION_ENABLED:
                // // Start battery timer
                // err_code = app_timer_start(m_battery_timer_id, BATTERY_LEVEL_MEAS_INTERVAL, NULL);
                // APP_ERROR_CHECK(err_code);
                break; // BLE_BAS_EVT_NOTIFICATION_ENABLED

        case BLE_BAS_EVT_NOTIFICATION_DISABLED:
                err_code = app_timer_stop(m_battery_timer_id);
                APP_ERROR_CHECK(err_code);
                break; // BLE_BAS_EVT_NOTIFICATION_DISABLED

        default:
                // No implementation needed.
                break;
        }
}



/**@brief Function for handling BLE events.
 *
 * @param[in]   p_ble_evt   Bluetooth stack event.
 * @param[in]   p_context   Unused.
 */
static void ble_evt_handler(ble_evt_t const * p_ble_evt, void * p_context)
{
        uint32_t err_code;

        switch (p_ble_evt->header.evt_id)
        {
        case BLE_GAP_EVT_CONNECTED:
                NRF_LOG_INFO("Connected");
                err_code = bsp_indication_set(BSP_INDICATE_CONNECTED);
                APP_ERROR_CHECK(err_code);
                m_conn_handle = p_ble_evt->evt.gap_evt.conn_handle;
                err_code = nrf_ble_qwr_conn_handle_assign(&m_qwr, m_conn_handle);
                APP_ERROR_CHECK(err_code);

#ifdef NRF_PWD_BLE_ENABLED
                m_pwd_is_verified = false;
                ret_code_t err_code =app_timer_start(m_pwd_timer_id, PASSWORD_TIMEOUT_INTERVAL, NULL);
                APP_ERROR_CHECK(err_code);
#endif

                break;

        case BLE_GAP_EVT_DISCONNECTED:
                NRF_LOG_INFO("Disconnected");
                // LED indication will be changed when advertising starts.
                m_conn_handle = BLE_CONN_HANDLE_INVALID;
                break;

        case BLE_GAP_EVT_PHY_UPDATE_REQUEST:
        {
                NRF_LOG_DEBUG("PHY update request.");
                ble_gap_phys_t const phys =
                {
                        .rx_phys = BLE_GAP_PHY_AUTO,
                        .tx_phys = BLE_GAP_PHY_AUTO,
                };
                err_code = sd_ble_gap_phy_update(p_ble_evt->evt.gap_evt.conn_handle, &phys);
                APP_ERROR_CHECK(err_code);
        } break;

        case BLE_GAP_EVT_SEC_PARAMS_REQUEST:
                // Pairing not supported
                err_code = sd_ble_gap_sec_params_reply(m_conn_handle, BLE_GAP_SEC_STATUS_PAIRING_NOT_SUPP, NULL, NULL);
                APP_ERROR_CHECK(err_code);
                break;

        case BLE_GATTS_EVT_SYS_ATTR_MISSING:
                // No system attributes have been stored.
                err_code = sd_ble_gatts_sys_attr_set(m_conn_handle, NULL, 0, 0);
                APP_ERROR_CHECK(err_code);
                break;

        case BLE_GATTC_EVT_TIMEOUT:
                // Disconnect on GATT Client timeout event.
                err_code = sd_ble_gap_disconnect(p_ble_evt->evt.gattc_evt.conn_handle,
                                                 BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
                APP_ERROR_CHECK(err_code);
                break;

        case BLE_GATTS_EVT_TIMEOUT:
                // Disconnect on GATT Server timeout event.
                err_code = sd_ble_gap_disconnect(p_ble_evt->evt.gatts_evt.conn_handle,
                                                 BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
                APP_ERROR_CHECK(err_code);
                break;

        default:
                // No implementation needed.
                break;
        }
}


/**@brief Function for the Event Scheduler initialization.
 */
static void scheduler_init(void)
{
        APP_SCHED_INIT(SCHED_MAX_EVENT_DATA_SIZE, SCHED_QUEUE_SIZE);
}



/**@brief Function for the SoftDevice initialization.
 *
 * @details This function initializes the SoftDevice and the BLE event interrupt.
 */
static void ble_stack_init(void)
{
        ret_code_t err_code;

        err_code = nrf_sdh_enable_request();
        APP_ERROR_CHECK(err_code);

        // Configure the BLE stack using the default settings.
        // Fetch the start address of the application RAM.
        uint32_t ram_start = 0;
        err_code = nrf_sdh_ble_default_cfg_set(APP_BLE_CONN_CFG_TAG, &ram_start);
        APP_ERROR_CHECK(err_code);

        // Enable BLE stack.
        err_code = nrf_sdh_ble_enable(&ram_start);
        APP_ERROR_CHECK(err_code);


        err_code = sd_power_mode_set(NRF_POWER_MODE_LOWPWR);
        APP_ERROR_CHECK(err_code);

        err_code = sd_power_dcdc_mode_set(NRF_POWER_DCDC_ENABLE);
        APP_ERROR_CHECK(err_code);

        // Register a handler for BLE events.
        NRF_SDH_BLE_OBSERVER(m_ble_observer, APP_BLE_OBSERVER_PRIO, ble_evt_handler, NULL);
}


/**@brief Function for handling events from the GATT library. */
void gatt_evt_handler(nrf_ble_gatt_t * p_gatt, nrf_ble_gatt_evt_t const * p_evt)
{
        uint32_t data_length;
        if ((m_conn_handle == p_evt->conn_handle) && (p_evt->evt_id == NRF_BLE_GATT_EVT_ATT_MTU_UPDATED))
        {
                data_length = p_evt->params.att_mtu_effective - OPCODE_LENGTH - HANDLE_LENGTH;
                NRF_LOG_INFO("Data len is set to 0x%X(%d)", m_ble_nus_max_data_len, m_ble_nus_max_data_len);
                m_ble_nus_max_data_len = data_length;
        }
        else if ((m_conn_handle == p_evt->conn_handle) && (p_evt->evt_id == NRF_BLE_GATT_EVT_DATA_LENGTH_UPDATED))
        {
                data_length = p_evt->params.att_mtu_effective - OPCODE_LENGTH - HANDLE_LENGTH - 4;
                NRF_LOG_INFO("gatt_event: Data len is set to 0x%X (%d)", data_length, data_length);
                m_ble_nus_max_data_len = data_length;
        }
        NRF_LOG_DEBUG("ATT MTU exchange completed. central 0x%x peripheral 0x%x",
                      p_gatt->att_mtu_desired_central,
                      p_gatt->att_mtu_desired_periph);
}


/**@brief Function for initializing the GATT library. */
void gatt_init(void)
{
        ret_code_t err_code;

        err_code = nrf_ble_gatt_init(&m_gatt, gatt_evt_handler);
        APP_ERROR_CHECK(err_code);

        err_code = nrf_ble_gatt_att_mtu_periph_set(&m_gatt, NRF_SDH_BLE_GATT_MAX_MTU_SIZE);
        APP_ERROR_CHECK(err_code);
}

/**@brief   Function for handling app_uart events.
 *
 * @details This function will receive a single character from the app_uart module and append it to
 *          a string. The string will be be sent over BLE when the last character received was a
 *          'new line' '\n' (hex 0x0A) or if the string has reached the maximum data length.
 */
/**@snippet [Handling the data received over UART] */
void uart_event_handle(app_uart_evt_t * p_event)
{
        static uint8_t data_array[BLE_NUS_MAX_DATA_LEN];
        static uint8_t index = 0;
        uint32_t err_code;

        switch (p_event->evt_type)
        {
        case APP_UART_DATA_READY:
                UNUSED_VARIABLE(app_uart_get(&data_array[index]));
                index++;

                if ((data_array[index - 1] == '\n') ||
                    (data_array[index - 1] == '\r') ||
                    (index >= m_ble_nus_max_data_len))
                {
                        if (index > 1)
                        {
                                NRF_LOG_DEBUG("Ready to send data over BLE NUS");
                                NRF_LOG_HEXDUMP_DEBUG(data_array, index);

                                do
                                {
                                        uint16_t length = (uint16_t)index;
                                        err_code = ble_nus_data_send(&m_nus, data_array, &length, m_conn_handle);
                                        if ((err_code != NRF_ERROR_INVALID_STATE) &&
                                            (err_code != NRF_ERROR_RESOURCES) &&
                                            (err_code != NRF_ERROR_NOT_FOUND))
                                        {
                                                APP_ERROR_CHECK(err_code);
                                        }
                                } while (err_code == NRF_ERROR_RESOURCES);
                        }

                        index = 0;
                }
                break;

        case APP_UART_COMMUNICATION_ERROR:
                APP_ERROR_HANDLER(p_event->data.error_communication);
                break;

        case APP_UART_FIFO_ERROR:
                APP_ERROR_HANDLER(p_event->data.error_code);
                break;

        default:
                break;
        }
}
/**@snippet [Handling the data received over UART] */


/**@brief  Function for initializing the UART module.
 */
/**@snippet [UART Initialization] */
static void uart_init(void)
{
        uint32_t err_code;
        app_uart_comm_params_t const comm_params =
        {
                .rx_pin_no    = RX_PIN_NUMBER,
                .tx_pin_no    = TX_PIN_NUMBER,
                .rts_pin_no   = RTS_PIN_NUMBER,
                .cts_pin_no   = CTS_PIN_NUMBER,
                .flow_control = APP_UART_FLOW_CONTROL_DISABLED,
                .use_parity   = false,
#if defined (UART_PRESENT)
                .baud_rate    = NRF_UART_BAUDRATE_115200
#else
                .baud_rate    = NRF_UARTE_BAUDRATE_115200
#endif
        };

        APP_UART_FIFO_INIT(&comm_params,
                           UART_RX_BUF_SIZE,
                           UART_TX_BUF_SIZE,
                           uart_event_handle,
                           APP_IRQ_PRIORITY_LOWEST,
                           err_code);
        APP_ERROR_CHECK(err_code);
}
/**@snippet [UART Initialization] */



/**@brief Function for initializing the Advertising functionality.
 *
 * @details Encodes the required advertising data and passes it to the stack.
 *          Also builds a structure to be passed to the stack when starting advertising.
 */
static void non_connectable_advertising_init(void)
{
        uint32_t err_code;
        ble_advdata_t advdata;
        uint8_t flags = BLE_GAP_ADV_FLAG_BR_EDR_NOT_SUPPORTED;

        ble_advdata_manuf_data_t manuf_specific_data;

        manuf_specific_data.company_identifier = APP_COMPANY_IDENTIFIER;

#if defined(USE_UICR_FOR_MAJ_MIN_VALUES)
        // If USE_UICR_FOR_MAJ_MIN_VALUES is defined, the major and minor values will be read from the
        // UICR instead of using the default values. The major and minor values obtained from the UICR
        // are encoded into advertising data in big endian order (MSB First).
        // To set the UICR used by this example to a desired value, write to the address 0x10001080
        // using the nrfjprog tool. The command to be used is as follows.
        // nrfjprog --snr <Segger-chip-Serial-Number> --memwr 0x10001080 --val <your major/minor value>
        // For example, for a major value and minor value of 0xabcd and 0x0102 respectively, the
        // the following command should be used.
        // nrfjprog --snr <Segger-chip-Serial-Number> --memwr 0x10001080 --val 0xabcd0102
        uint16_t major_value = ((*(uint32_t *)UICR_ADDRESS) & 0xFFFF0000) >> 16;
        uint16_t minor_value = ((*(uint32_t *)UICR_ADDRESS) & 0x0000FFFF);

        uint8_t index = MAJ_VAL_OFFSET_IN_BEACON_INFO;

        m_beacon_info[index++] = MSB_16(major_value);
        m_beacon_info[index++] = LSB_16(major_value);

        m_beacon_info[index++] = MSB_16(minor_value);
        m_beacon_info[index++] = LSB_16(minor_value);
#endif

        manuf_specific_data.data.p_data = (uint8_t *) m_beacon_info;
        manuf_specific_data.data.size   = APP_BEACON_INFO_LENGTH;

        // Build and set advertising data.
        memset(&advdata, 0, sizeof(advdata));

        advdata.name_type             = BLE_ADVDATA_NO_NAME;
        advdata.flags                 = flags;
        advdata.p_manuf_specific_data = &manuf_specific_data;

        // Initialize advertising parameters (used when starting advertising).
        memset(&m_adv_params, 0, sizeof(m_adv_params));

        m_adv_params.properties.type = BLE_GAP_ADV_TYPE_NONCONNECTABLE_NONSCANNABLE_UNDIRECTED;
        m_adv_params.p_peer_addr     = NULL;// Undirected advertisement.
        m_adv_params.filter_policy   = BLE_GAP_ADV_FP_ANY;
        m_adv_params.interval        = NON_CONNECTABLE_ADV_INTERVAL;
        m_adv_params.duration        = 0;   // Never time out.

        err_code = ble_advdata_encode(&advdata, m_adv_data.adv_data.p_data, &m_adv_data.adv_data.len);
        APP_ERROR_CHECK(err_code);

        err_code = sd_ble_gap_adv_set_configure(&m_adv_handle, &m_adv_data, &m_adv_params);
        APP_ERROR_CHECK(err_code);


}



/**@brief Function for initializing the Advertising functionality.
 */
static void connectable_advertising_init(void)
{
        uint32_t err_code;
        ble_advertising_init_t init;

        memset(&init, 0, sizeof(init));

        init.advdata.name_type          = BLE_ADVDATA_FULL_NAME;
        init.advdata.include_appearance = false;
        init.advdata.flags              = BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE;

        init.srdata.uuids_complete.uuid_cnt = sizeof(m_adv_uuids) / sizeof(m_adv_uuids[0]);
        init.srdata.uuids_complete.p_uuids  = m_adv_uuids;

        init.config.ble_adv_fast_enabled  = true;
        init.config.ble_adv_fast_interval = APP_ADV_INTERVAL;
        init.config.ble_adv_fast_timeout  = APP_ADV_DURATION;
        init.evt_handler = on_adv_evt;

        err_code = ble_advertising_init(&m_advertising, &init);
        APP_ERROR_CHECK(err_code);

        ble_advertising_conn_cfg_tag_set(&m_advertising, APP_BLE_CONN_CFG_TAG);
}

/**@brief Function for handling events from the button handler module.
 *
 * @param[in] pin_no        The pin that the event applies to.
 * @param[in] button_action The button action (press/release).
 */
static void button_event_handler(uint8_t pin_no, uint8_t button_action)
{
        ret_code_t err_code;

        switch (pin_no)
        {

        case TX_POWER_BUTTON:
                if (button_action == APP_BUTTON_PUSH)
                {
                        {
                                err_code = sd_ble_gap_tx_power_set(BLE_GAP_TX_POWER_ROLE_ADV, m_adv_handle, m_tx_power);
                                APP_ERROR_CHECK(err_code);
                        }
                }
                break;

        case APP_STATE_BUTTON:
                break;

        default:
                APP_ERROR_HANDLER(pin_no);
                break;
        }
}


/**@brief Function for initializing the button handler module.
 */
static void buttons_init(void)
{
        ret_code_t err_code;

        //The array must be static because a pointer to it will be saved in the button handler module.
        static app_button_cfg_t buttons[] =
        {
                {TX_POWER_BUTTON, false, BUTTON_PULL, button_event_handler},
                {APP_STATE_BUTTON, false, BUTTON_PULL, button_event_handler},
        };

        err_code = app_button_init(buttons, ARRAY_SIZE(buttons),
                                   BUTTON_DETECTION_DELAY);
        APP_ERROR_CHECK(err_code);
}


/**@brief Function for initializing the nrf log module.
 */
static void log_init(void)
{
        ret_code_t err_code = NRF_LOG_INIT(NULL);
        APP_ERROR_CHECK(err_code);

        NRF_LOG_DEFAULT_BACKENDS_INIT();
}


/**@brief Function for initializing power management.
 */
static void power_management_init(void)
{
        ret_code_t err_code;
        err_code = nrf_pwr_mgmt_init();
        APP_ERROR_CHECK(err_code);
}


/**@brief Function for placing the application in low power state while waiting for events.
 */
#define FPU_EXCEPTION_MASK 0x0000009F
/**@brief Function for handling the idle state (main loop).
 *
 * @details If there is no pending log operation, then sleep until next the next event occurs.
 */
static void idle_state_handle(void)
{
        __set_FPSCR(__get_FPSCR()  & ~(FPU_EXCEPTION_MASK));
        (void) __get_FPSCR();
        NVIC_ClearPendingIRQ(FPU_IRQn);


        app_sched_execute();
        while(NRF_LOG_PROCESS());
        //UNUSED_RETURN_VALUE(NRF_LOG_PROCESS());
        nrf_pwr_mgmt_run();
}


/**@brief Function for starting advertising.
 */
static void connect_advertising_start(void)
{
        uint32_t err_code = ble_advertising_start(&m_advertising, BLE_ADV_MODE_FAST);
        APP_ERROR_CHECK(err_code);
}

/**@brief Function for starting advertising.
 */
static void non_connect_advertising_start(void)
{
        ret_code_t err_code;

        tx_power_set();

        err_code = sd_ble_gap_adv_start(m_adv_handle, APP_BLE_CONN_CFG_TAG);
        APP_ERROR_CHECK(err_code);

        err_code = bsp_indication_set(BSP_INDICATE_ADVERTISING);
        APP_ERROR_CHECK(err_code);
}

/**@brief Application main function.
 */
int main(void)
{
        bool erase_bonds;
        bool connect_mode_enter = false;
        uint32_t err_code = NRF_SUCCESS;

        /* enable instruction cache */
        NRF_NVMC->ICACHECNF = (NVMC_ICACHECNF_CACHEEN_Enabled << NVMC_ICACHECNF_CACHEEN_Pos) +
                              (NVMC_ICACHECNF_CACHEPROFEN_Disabled << NVMC_ICACHECNF_CACHEPROFEN_Pos);

        // Initialize.
        uart_init();
        log_init();
        timers_init();
        // Enable the SADDC to measure the battery
        adc_configure();
        /**@brief Load configuration from flash. */
        err_code = m_ble_flash_init(&m_ble_default_config, &m_ble_config);
        if (err_code != NRF_SUCCESS)
        {
                //jimmy
                NRF_LOG_ERROR(" m_ble_flash_init failed - %d\r\n", err_code);
                APP_ERROR_CHECK(err_code);
        }
        err_code = device_config_verify();
        if (err_code != NRF_SUCCESS)
        {
                NRF_LOG_ERROR("Thingy_config_verify failed - %d\r\n", err_code);
                APP_ERROR_CHECK(err_code);
        }
        enter_button_init();
        connect_mode_enter       = connect_adv_enter_check();
        buttons_init();
        power_management_init();
        ble_stack_init();
        scheduler_init();
        application_timer_start();
        if (connect_mode_enter)
        {
                gap_params_init(true);
                gatt_init();
                services_init();
                connectable_advertising_init();
                conn_params_init();
                connect_advertising_start();

                NRF_LOG_INFO("Connected Advertising!");
        }
        else
        {
                gap_params_init(true);
                non_connectable_advertising_init();
                non_connect_advertising_start();

                NRF_LOG_INFO("Non-Connected Advertising!");

        }

        // Enter main loop.
        for (;;)
        {
                idle_state_handle();
        }
}


/**
 * @}
 */
