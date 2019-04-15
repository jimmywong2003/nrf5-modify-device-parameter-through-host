#ifndef BLE_TCS_H__
#define BLE_TCS_H__

#include "ble.h"
#include "ble_srv_common.h"
#include "app_util_platform.h"
#include <string.h>
#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

#define BLE_UUID_TCS_SERVICE 0x0100                      /**< The UUID of the Thingy Configuration Service. */
#define BLE_TCS_MAX_DATA_LEN (BLE_GATT_ATT_MTU_DEFAULT - 3) /**< Maximum length of data (in bytes) that can be transmitted to the peer by the Thingy Configuration service module. */

#define BLE_TCS_DEVICE_NAME_LEN_MAX 10
#define BLE_TCS_BEACON_LEN_MAX      17
#define BLE_TCS_BEACON_LEN_MIN       3

#define BLE_TCS_PWD_LEN_MAX    4

#define BLE_TCS_ADV_PAYLOAD_LEN_MAX 24
#define BLE_TCS_ADV_PAYLOAD_LEN_MIN 24

#define TCS_ADV_PARAMS_INTERVAL_MIN 32UL
#define TCS_ADV_PARAMS_INTERVAL_MAX 8000UL
#define TCS_ADV_PARAMS_TIMEOUT_MIN  0UL
#define TCS_ADV_PARAMS_TIMEOUT_MAX  180UL

#define TCS_MTU_REQ_EXCHANGE    0x01
#define TCS_MTU_REQ_MIN         0UL
#define TCS_MTU_REQ_MAX         1UL
#define TCS_MTU_SIZE_MIN        23UL
#define TCS_MTU_SIZE_MAX        276UL



#ifdef __GNUC__
    #ifdef PACKED
        #undef PACKED
    #endif
    #define PACKED(TYPE) TYPE __attribute__ ((packed))
#endif

#define BLE_TCS_DEF(_name)                                                                          \
        static ble_tcs_t _name;                                                                             \
        NRF_SDH_BLE_OBSERVER(_name ## _obs,                                                                 \
                             BLE_TCS_BLE_OBSERVER_PRIO,                                                     \
                             ble_tcs_on_ble_evt, &_name)

typedef PACKED ( struct
{
        uint8_t name[BLE_TCS_DEVICE_NAME_LEN_MAX+1];
        uint8_t len;
}) ble_tcs_dev_name_t;

typedef PACKED ( struct
{
        uint16_t interval;
        uint8_t timeout;
}) ble_tcs_adv_params_t;

typedef uint16_t ble_tcs_appear_t;

typedef PACKED ( struct
{
        uint16_t min_conn_int;
        uint16_t max_conn_int;
        uint16_t slave_latency;
        uint16_t sup_timeout;
}) ble_tcs_conn_params_t;

typedef PACKED ( struct
{
        uint8_t req;
        uint16_t size;
}) ble_tcs_mtu_t;

typedef PACKED ( struct
{
        uint8_t major;
        uint8_t minor;
        uint8_t patch;
}) ble_tcs_fw_version_t;

typedef PACKED ( struct
{
        uint8_t data[BLE_TCS_BEACON_LEN_MAX];
        uint8_t len;
}) ble_tcs_eddystone_url_t;

typedef PACKED ( struct
{
        int tx_power;
}) ble_tcs_tx_power_t;

typedef PACKED ( struct
{
        uint8_t data[BLE_TCS_PWD_LEN_MAX];
}) ble_tcs_pwd_t;

typedef PACKED ( struct
{
        uint8_t data[BLE_TCS_ADV_PAYLOAD_LEN_MAX];
        uint8_t len;
}) ble_tcs_adv_payload_t;

typedef struct
{
        ble_tcs_dev_name_t dev_name;
        ble_tcs_adv_params_t adv_params;
        ble_tcs_conn_params_t conn_params;
        ble_tcs_eddystone_url_t eddystone_url;
        ble_tcs_fw_version_t fw_version;
        ble_tcs_mtu_t mtu;
        ble_tcs_tx_power_t tx_power;
        ble_tcs_pwd_t pwd;
        ble_tcs_adv_payload_t adv_payload;
        ble_tcs_pwd_t pwd_verify;
}ble_tcs_params_t;

typedef enum
{
        BLE_TCS_EVT_DEV_NAME = 0,
        BLE_TCS_EVT_ADV_PARAM,
        BLE_TCS_EVT_CONN_PARAM,
        BLE_TCS_EVT_BEACON,
        BLE_TCS_EVT_MTU,
        BLE_TCS_EVT_TX_POWER,
        BLE_TCS_EVT_PWD,
        BLE_TCS_EVT_ADV_PAYLOAD,
        BLE_TCS_EVT_PWD_VERIFY,
}ble_tcs_evt_type_t;


/* Forward declaration of the ble_tcs_t type. */
typedef struct ble_tcs_s ble_tcs_t;

/**@brief Thingy Configuration Service event handler type. */
typedef void (*ble_tcs_evt_handler_t) (ble_tcs_t        * p_tcs,
                                       ble_tcs_evt_type_t evt_type,
                                       uint8_t          * p_data,
                                       uint16_t length);

/**@brief Thingy Configuration Service initialization structure.
 *
 * @details This structure contains the initialization information for the service. The application
 * must fill this structure and pass it to the service using the @ref ble_tcs_init function.
 */
typedef struct
{
        ble_tcs_params_t      * p_init_vals;
        ble_tcs_evt_handler_t evt_handler; /**< Event handler to be called for handling received data. */
} ble_tcs_init_t;

/**@brief Thingy Configuration Service structure.
 *
 * @details This structure contains status information related to the service.
 */
struct ble_tcs_s
{
        uint8_t uuid_type;                                 /**< UUID type for Thingy Configuration Service Base UUID. */
        uint16_t service_handle;                           /**< Handle of Thingy Configuration Service (as provided by the S110 SoftDevice). */
        ble_gatts_char_handles_t dev_name_handles;         /**< Handles related to the temperature characteristic (as provided by the S132 SoftDevice). */
        ble_gatts_char_handles_t adv_param_handles;        /**< Handles related to the pressure characteristic (as provided by the S132 SoftDevice). */
        ble_gatts_char_handles_t conn_param_handles;       /**< Handles related to the config characteristic (as provided by the S132 SoftDevice). */
        ble_gatts_char_handles_t beacon_handles;
        ble_gatts_char_handles_t fwv_handles;
        ble_gatts_char_handles_t mtu_handles;
        ble_gatts_char_handles_t tx_power_handles;
        ble_gatts_char_handles_t pwd_handles;
        ble_gatts_char_handles_t adv_payload_handles;
        ble_gatts_char_handles_t verify_handles;
        uint16_t conn_handle;                              /**< Handle of the current connection (as provided by the S110 SoftDevice). BLE_CONN_HANDLE_INVALID if not in a connection. */
        ble_tcs_evt_handler_t evt_handler;                 /**< Event handler to be called for handling received data. */
};

/**@brief Function for initializing the Thingy Configuration Service.
 *
 * @param[out] p_tcs      Thingy Configuration Service structure. This structure must be supplied
 *                        by the application. It is initialized by this function and will
 *                        later be used to identify this particular service instance.
 * @param[in] p_tcs_init  Information needed to initialize the service.
 *
 * @retval NRF_SUCCESS If the service was successfully initialized. Otherwise, an error code is returned.
 * @retval NRF_ERROR_NULL If either of the pointers p_tcs or p_tcs_init is NULL.
 */
uint32_t ble_tcs_init(ble_tcs_t * p_tcs, const ble_tcs_init_t * p_tcs_init);

/**@brief Function for handling the Thingy Configuration Service's BLE events.
 *
 * @details The Thingy Configuration Service expects the application to call this function each time an
 * event is received from the S110 SoftDevice. This function processes the event if it
 * is relevant and calls the Thingy Configuration Service event handler of the
 * application if necessary.
 *
 * @param[in] p_tcs       Thingy Configuration Service structure.
 * @param[in] p_ble_evt   Event received from the S110 SoftDevice.
 */
void ble_tcs_on_ble_evt(ble_evt_t const * p_ble_evt, void * p_context);

/**@brief Function for setting the MTU char data.
 *
 * @details This function updates the MTU characteristic.
 *
 * @param[in] p_tcs       Pointer to the Thingy Configuration Service structure.
 * @param[in] p_data      Pointer to the MTU data.
 *
 * @retval NRF_SUCCESS If the string was sent successfully. Otherwise, an error code is returned.
 */
uint32_t ble_tcs_mtu_set(ble_tcs_t * p_tcs, ble_tcs_mtu_t * p_data);

#endif //
