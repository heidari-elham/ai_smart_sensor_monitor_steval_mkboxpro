
#include "CustomBLE.h"

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

#define BLE_MANUFACTURER_STM_L 		0x30
#define BLE_MANUFACTURER_STM_H 		0x00

static uint8_t adv_data[31];



/* Function Definitions ----------------------------------------------------- */
uint8_t ble_manager_init(void)
{
    uint8_t ret;
    uint16_t service_handle, dev_name_char_handle, appearance_char_handle;

    // Initialize the HCI transport layer
    hci_init(NULL, NULL);

    // Reset the Link Layer on an LE Controller
    hci_reset();

    // Delay required for booting the BLE device
    HAL_Delay(2000);

    // Read and display the BLE MAC address
    uint8_t bdaddr[] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
    uint8_t baddr_len = CONFIG_DATA_PUBADDR_LEN;

    ret = aci_hal_read_config_data(0x80, &baddr_len, bdaddr);
    if (ret != BLE_STATUS_SUCCESS) {
        printf("Failed to read BLE MAC address\r\n");
        return ret;
    }

    printf("BLE MAC address is %02X:%02X:%02X:%02X:%02X:%02X\r\n",
           bdaddr[5], bdaddr[4], bdaddr[3], bdaddr[2], bdaddr[1], bdaddr[0]);

    // Validate the random static MAC address format
    if ((bdaddr[5] & 0xC0U) != 0xC0U) {
        printf("Static Random address not well formed\r\n");
        return BLE_STATUS_ERROR;
    }

    // Set the read MAC address as the advertising address
    ret = aci_hal_write_config_data(CONFIG_DATA_PUBADDR_OFFSET, baddr_len, bdaddr);
    if (ret != BLE_STATUS_SUCCESS) {
        printf("Failed to set BLE advertising address\r\n");
        return ret;
    }

    printf("BLE advertising address successfully set\r\n");

    const char customBoardName[] = "SSM";


    // Initialize GAP with specified role and parameters
    ret = aci_gap_init(GAP_BROADCASTER_ROLE,
    		           0x00,
					   (uint8_t)strlen(customBoardName),
                       STATIC_RANDOM_ADDR,
					   &service_handle,
					   &dev_name_char_handle,
					   &appearance_char_handle);

    if (ret != BLE_STATUS_SUCCESS) {
        printf("GAP_Init failed\r\n");
        return ret;
    }

    ret = aci_gatt_srv_write_handle_value_nwk(dev_name_char_handle + 1,
                                              0,  // Offset from the start
                                              (uint8_t) strlen(customBoardName), // Length of the name
                                              (uint8_t *) customBoardName); // Custom name as a byte array

    if (ret != BLE_STATUS_SUCCESS) {
    	return ret;
    }

    // Initialize GATT Server
    ret = aci_gatt_srv_init();
    if (ret != BLE_STATUS_SUCCESS) {
        printf("GATT_Init failed\r\n");
        return ret;
    }

    // Set transmit power level (optional)
    ret = aci_hal_set_tx_power_level(1, 7);
    if (ret != BLE_STATUS_SUCCESS)
      {return ret;}


	/* put device in non connectable mode */
	ret = aci_gap_set_advertising_configuration(
	    0,                                   // Advertising handle (single handle for broadcast)
		GAP_MODE_BROADCAST,                  // Advertising mode set to broadcast
		ADV_PROP_LEGACY,                     // Advertising properties (legacy, non-connectable)
	    160,                                 // Minimum advertising interval (100 ms in 0.625 ms steps)
	    160,                                 // Maximum advertising interval (100 ms in 0.625 ms steps)
	    ADV_CH_ALL,                          // Advertising on all three channels
		PUBLIC_ADDR,                         // Peer address type (not used for broadcast)
		NULL,                               // Peer address (not used for broadcast)
	    ADV_NO_WHITE_LIST_USE,               // No whitelist, broadcast openly
	    0,                                   // Advertising SID (unused in legacy advertising)
	    LE_1M_PHY,                           // Primary PHY for legacy advertising (1M PHY)
	    0,                                   // Secondary maximum skip (not applicable in legacy)
	    LE_1M_PHY,                           // Secondary PHY (not used for legacy)
	    0,                                   // Advertising power (use default)
	    0                                    // Do not use anonymous advertising
	);
    if (ret != BLE_STATUS_SUCCESS)
        return ret;


    return BLE_STATUS_SUCCESS;
}


uint8_t ble_beacon_data_set_payload(ble_beacon_data_t *beacon_data,
									uint8_t *payload_buffer, uint32_t size)
{
	memset(beacon_data->payload, 0x00, ADV_DATA_PAYLOAD_SIZE);
	memcpy(beacon_data->payload, payload_buffer, size);
	beacon_data->size = size;

	return BLE_STATUS_SUCCESS;
}



uint8_t ble_beacon_send(ble_beacon_data_t *beacon_data)
{
	uint8_t ret;
	uint8_t adv_data_size = ADV_DATA_HEADER_SIZE + beacon_data->size ;
	uint8_t adv_data_message_size =
	ADV_DATA_UID_SIZE + ADV_DATA_HEADER_SIZE + beacon_data->size;


	adv_data[0] = adv_data_size;
	adv_data[1] = AD_TYPE_MANUFACTURER_SPECIFIC_DATA;
	adv_data[2] = BLE_MANUFACTURER_STM_L;
	adv_data[3] = BLE_MANUFACTURER_STM_H;
	adv_data[4] = beacon_data->bsdk_version;
	adv_data[5] = beacon_data->device_id;
	adv_data[6] = beacon_data->firmware_id;
	adv_data[7] = beacon_data->payload_id;

	memcpy(&(adv_data[8]),
		   beacon_data->payload, beacon_data->size);


	/* Set the beacon manufacturer specific data*/
	ret = aci_gap_set_advertising_data_nwk(0, ADV_COMPLETE_DATA, adv_data_message_size, adv_data);
	if (ret != BLE_STATUS_SUCCESS)
		return ret;



	Advertising_Set_Parameters_t Advertising_Set_Parameters[1];
	Advertising_Set_Parameters[0].Advertising_Handle = 0;
	Advertising_Set_Parameters[0].Duration = 0;
	Advertising_Set_Parameters[0].Max_Extended_Advertising_Events = 0;

	/* Enable advertising */
	ret = aci_gap_set_advertising_enable (ENABLE, 1, Advertising_Set_Parameters);
	if (ret != BLE_STATUS_SUCCESS)
		return ret;



	return BLE_STATUS_SUCCESS;
}


/**
 * @brief Disables BLE advertising.
 * @return BLE_STATUS_SUCCESS if the operation was successful, error code otherwise.
 */
tBleStatus DisableAdvertising(void) {
    tBleStatus ret;
    Advertising_Set_Parameters_t Advertising_Set_Parameters[1];

    // Set the advertising handle and parameters
    Advertising_Set_Parameters[0].Advertising_Handle = 0;
    Advertising_Set_Parameters[0].Duration = 0;
    Advertising_Set_Parameters[0].Max_Extended_Advertising_Events = 0;

    // Disable advertising
    ret = aci_gap_set_advertising_enable(DISABLE, 1, Advertising_Set_Parameters);
    if (ret != BLE_STATUS_SUCCESS) {
        // Log or handle the error as needed
        return ret;
    }

    return BLE_STATUS_SUCCESS;
}

/**
 * @brief  This method can be set to describe beaconing
 * @param number of times to repeat sending beacon
 * @retval send another or stop
 */
__weak uint8_t ble_beacon_callback(void)
{
	HAL_Delay(1000);

	return BLE_STATUS_SUCCESS;
}
