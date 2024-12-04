/* Header guard ------------------------------------------------------------- */

#ifndef APP_SSM_H
#define APP_SSM_H

#ifdef __cplusplus
extern "C"
{
#endif

/* Definitions -------------------------------------------------------------- */

// Define the smart sensor type as MLC only
#define SSM_SMART_SENSOR_MLC 0
#define SSM_SMART_SENSOR_ISPU 1
#define SSM_SMART_SENSOR_BOTH 2

// Set the configuration to use only MLC
#define SSM_CONFIG_SMART_SENSOR SSM_SMART_SENSOR_MLC

#define SSM_CONFIG_BLUEST_SDK_PROTOCOL_VERSION	0x03
#define SSM_CONFIG_BLUEST_SDK_V3_DEVICE_ID		0x09
#define SSM_CONFIG_BLUEST_SDK_V3_FW_ID			0xff
#define SSM_CONFIG_BLUEST_SDK_V3_PAYLOAD_ID		0xff
#define SSM_PAYLOAD_ID_MLC 0x01
#define SSM_PAYLOAD_ID_ISPU 0x02
/* Function prototypes ------------------------------------------------------ */

// Initialize the Smart Sensor Monitor
void UART_Debug_Print(const char *msg);
void MX_ssm_Init(void);
void MX_ssm_Process(void);
void Enter_Stop1Mode(void);
void Exit_Stop1Mode(void);
void SystemPower_Config(void);
// Process data from the Smart Sensor Monitor


#ifdef __cplusplus
}
#endif

#endif /* APP_SSM_H */

/* -------------------------------------------------------------------------- */
