/* CustomBLE.h */

#ifndef CUSTOM_BLE_H
#define CUSTOM_BLE_H

#include <stdint.h>
#include "bluenrg_lp_gatt_aci.h"
#include "bluenrg_lp_gap_aci.h"
#include "hci.h"
#include "bluenrg_lp_aci.h"
#include "bluenrg_lp_hci_le.h"
#include "bluenrg_lp_events.h"
#include "hci_const.h"
#include "hci_tl.h"
#include "bluenrg_utils.h"

/* Definitions -------------------------------------------------------------- */

#define ADV_DATA_UID_SIZE 3
#define ADV_DATA_HEADER_SIZE 8
#define ADV_DATA_PAYLOAD_SIZE 15
#define ADV_DATA_MESSAGE_SIZE \
    (ADV_DATA_UID_SIZE + ADV_DATA_HEADER_SIZE + ADV_DATA_PAYLOAD_SIZE)

#define BLE_MANUFACTURER_STM_L      0x30
#define BLE_MANUFACTURER_STM_H      0x00



/* Typedefs ----------------------------------------------------------------- */

typedef uint8_t (*ble_callback_t)(void);

typedef struct
{
    uint8_t bsdk_version;
    uint8_t device_id;
    uint8_t firmware_id;
    uint8_t payload_id;
    uint8_t payload[ADV_DATA_PAYLOAD_SIZE];
    uint8_t size;
    void (*callback)(void);  // Callback function pointer
} ble_beacon_data_t;

/* Function Declarations ---------------------------------------------------- */

/**
 * @brief Initializes the BLE manager and sets up the basic configurations.
 * @retval BLE_STATUS_SUCCESS on success, error code otherwise.
 */
uint8_t ble_manager_init(void);

/**
 * @brief Sets the payload data for the beacon.
 * @param beacon_data Pointer to the beacon data structure.
 * @param payload_buffer Pointer to the payload data buffer.
 * @param size Size of the payload data.
 * @retval BLE_STATUS_SUCCESS on success, error code otherwise.
 */
uint8_t ble_beacon_data_set_payload(ble_beacon_data_t *beacon_data,
                                    uint8_t *payload_buffer, uint32_t size);

/**
 * @brief Sends the beacon data through advertising.
 * @param beacon_data Pointer to the beacon data structure.
 * @retval BLE_STATUS_SUCCESS on success, error code otherwise.
 */
uint8_t ble_beacon_send(ble_beacon_data_t *beacon_data);

tBleStatus DisableAdvertising(void);
/**
 * @brief Callback function for beaconing. Can be overridden by the user.
 * @retval BLE_STATUS_SUCCESS to continue sending, other values to stop.
 */
__weak uint8_t ble_beacon_callback(void);

void CustomBLE_TIM3_Handler(void);

#endif // CUSTOM_BLE_H
