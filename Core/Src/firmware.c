/*
 * firmware.c
 *
 *  Created on: Aug 22, 2024
 *      Author: Ellie
 */

/* Includes ------------------------------------------------------------------*/

#include <stdint.h>
#include <stdio.h>

#include "main.h"

#include "Firmware.h"

#include "CustomBLE.h"

#if (SSM_CONFIG_SMART_SENSOR == SSM_SMART_SENSOR_MLC)
#include "ssm_smart_sensor_mlc.h"
#elif (SSM_CONFIG_SMART_SENSOR == SSM_SMART_SENSOR_ISPU)
#include "ssm_smart_sensor_ispu.h"
#elif (SSM_CONFIG_SMART_SENSOR == SSM_SMART_SENSOR_BOTH)
#include "ssm_smart_sensor_ispu.h"
#include "ssm_smart_sensor_mlc.h"
#endif /* SSM_CONFIG_SMART_SENSOR */



/* Global variables --------------------------------------------------------- */
static ble_beacon_data_t beacon_data;
static volatile uint8_t smart_sensor_event = 0;
uint16_t mlcCharHandle;
static volatile uint8_t led_off_event = 0;

extern UART_HandleTypeDef huart4;
extern SPI_HandleTypeDef hspi2;
extern SPI_HandleTypeDef hspi3;
extern TIM_HandleTypeDef htim3;

void SystemPower_Config(void);
static void handle_event(uint8_t payload_id, uint8_t *prediction);

void MX_ssm_Init(void)
{
	int32_t ret;

	SystemPower_Config();

	printf("===== Smart Sensor Monitor =====\n");

#if (SSM_CONFIG_SMART_SENSOR == SSM_SMART_SENSOR_MLC)

	ret = ssm_smart_sensor_mlc_init();
	if (ret != BSP_ERROR_NONE)
	{
		Error_Handler();
	}

	printf("===== mlc initialization Done =====\n");

	ret = ssm_smart_sensor_mlc_configure_ucf();
	if (ret != BSP_ERROR_NONE)
	{
		Error_Handler();
	}

	printf("===== ucf configuration Done =====\n");

#elif (SSM_CONFIG_SMART_SENSOR == SSM_SMART_SENSOR_ISPU)

	ret = ssm_smart_sensor_ispu_init();
	if (ret != BSP_ERROR_NONE)
		Error_Handler();

	printf("===== ispu initialization Done =====\n");

	ret = ssm_smart_sensor_ispu_configure_ucf();
	if (ret != BSP_ERROR_NONE)
		Error_Handler();

	printf("===== ucf configuration Done =====\n");

#elif (SSM_CONFIG_SMART_SENSOR == SSM_SMART_SENSOR_BOTH)
	ret = ssm_smart_sensor_mlc_init();
	if (ret != BSP_ERROR_NONE)
		Error_Handler();

	ret = ssm_smart_sensor_ispu_init();
	if (ret != BSP_ERROR_NONE)
		Error_Handler();

	printf("===== mlc and ispu initialization Done =====\n");

	ret = ssm_smart_sensor_mlc_configure_ucf();
	if (ret != BSP_ERROR_NONE)
	{
		Error_Handler();
	}
	ret = ssm_smart_sensor_ispu_configure_ucf();
	if (ret != BSP_ERROR_NONE)
		Error_Handler();

	printf("===== ucf configuration Done for both =====\n");
#endif /* SSM_CONFIG_SMART_SENSOR */

	ret = ble_manager_init();
	if (ret != BLE_STATUS_SUCCESS)
		Error_Handler();

	beacon_data.bsdk_version = SSM_CONFIG_BLUEST_SDK_PROTOCOL_VERSION;
	beacon_data.device_id = SSM_CONFIG_BLUEST_SDK_V3_DEVICE_ID;
	beacon_data.firmware_id = SSM_CONFIG_BLUEST_SDK_V3_FW_ID;
	beacon_data.payload_id = SSM_CONFIG_BLUEST_SDK_V3_PAYLOAD_ID;
	beacon_data.callback = ble_beacon_callback;

	printf("===== BLE initialization Done =====\n");

	}



void MX_ssm_Process(void)
{

	int32_t ret;

	if ((smart_sensor_event & 0x01) > 0) {

		smart_sensor_event &= ~0x01;
		uint8_t mlc_prediction;


		ret = ssm_smart_sensor_mlc_handle_event(&mlc_prediction);
		if (ret != BSP_ERROR_NONE)
			Error_Handler();

		handle_event(SSM_PAYLOAD_ID_MLC, mlc_prediction);

	}
	if ((smart_sensor_event & 0x10) > 0) {

		smart_sensor_event &= ~0x10;
		uint8_t ispu_prediction;


		ret = ssm_smart_sensor_ispu_handle_event(&ispu_prediction);
		if (ret != BSP_ERROR_NONE)
			Error_Handler();

		handle_event(SSM_PAYLOAD_ID_ISPU, ispu_prediction);

	}



	if (led_off_event == 1) {

		led_off_event = 0;
		HAL_TIM_Base_Stop_IT(&htim3);
		DisableAdvertising();
		HAL_GPIO_WritePin(GPIOF, GPIO_PIN_6, RESET);

	}

	if (htim3.State == HAL_TIM_STATE_BUSY) {
		// suspend the MCU and wait for interrupt
	    __WFI();
	}
	else {
		// stop the MCU and wait for interrupt
		Enter_Stop1Mode();

		Exit_Stop1Mode();
	}


}

/**
* @brief  Timer interrupt callback
 * @param None
 * @retval None
 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if(htim->Instance == TIM3){
		led_off_event = 1;
	}
}


/**
 * @brief  This method is used to enter StopMode1
 * @param None
 * @retval None
 */
void Enter_Stop1Mode(void)
{
	// De-initialize peripherals for power saving
	HAL_SPI_DeInit(&hspi1);

	// for DEBUG
	HAL_UART_DeInit(&huart4);
#if (SSM_CONFIG_SMART_SENSOR == SSM_SMART_SENSOR_MLC)
	HAL_SPI_DeInit(&hspi2);
#elif (SSM_CONFIG_SMART_SENSOR == SSM_SMART_SENSOR_ISPU)
	HAL_SPI_DeInit(&hspi3);
#elif (SSM_CONFIG_SMART_SENSOR == SSM_SMART_SENSOR_BOTH)
	HAL_SPI_DeInit(&hspi2);
	HAL_SPI_DeInit(&hspi3);
#endif /* SSM_CONFIG_SMART_SENSOR */

	// Suspend SystemTick before entering sleep mode
	HAL_SuspendTick();

	// Enter Stop Mode 1, exit over interrupt
	HAL_PWREx_EnableUltraLowPowerMode();
	HAL_PWREx_EnterSTOP1Mode(PWR_STOPENTRY_WFI);

}

/**
 * @brief  This method restores all peripherals after exiting StopMode1
 * Must be called direct after Enter_Stop1Mode()
 * @param None
 * @retval None
 */
void Exit_Stop1Mode(void)
{

	// Re-initialize the system clock
	SysTick_Config(SystemCoreClock / 1000);

	// Enable Systick again
	HAL_ResumeTick();

// Re-initialize peripherals
#if (SSM_CONFIG_SMART_SENSOR == SSM_SMART_SENSOR_MLC)
	HAL_SPI_Init(&hspi2);
#elif (SSM_CONFIG_SMART_SENSOR == SSM_SMART_SENSOR_ISPU)
	HAL_SPI_Init(&hspi3);
#elif (SSM_CONFIG_SMART_SENSOR == SSM_SMART_SENSOR_BOTH)
	HAL_SPI_Init(&hspi2);
	HAL_SPI_Init(&hspi3);
#endif /* SSM_CONFIG_SMART_SENSOR */

	// for DEBUG
	HAL_UART_Init(&huart4);

	HAL_SPI_Init(&hspi1);


}


/**
 * @brief  EXTI line detection callback.
 * @param  uint16_t GPIO_Pin Specifies the pin connected EXTI line
 * @retval None
 */
void HAL_GPIO_EXTI_Rising_Callback(uint16_t GPIO_Pin)
{

	if (GPIO_Pin == GPIO_PIN_4)
	{
		smart_sensor_event |= 0x01;
	}
	else if (GPIO_Pin == GPIO_PIN_2) {

		smart_sensor_event |= 0x10;
	}

}

static void handle_event(uint8_t payload_id, uint8_t *prediction) {
    // Set GPIO to indicate event handling
    HAL_GPIO_WritePin(GPIOF, GPIO_PIN_6, SET);

    // Log the prediction and payload ID for debugging
    printf("Payload ID: %d, Prediction: %d\n", payload_id, prediction);

    // Prepare BLE payload and send it
    beacon_data.payload_id = payload_id;
    uint8_t payload[] = {0};
    payload[0] = prediction;
    ble_beacon_data_set_payload(&beacon_data, payload, sizeof(payload));
    ble_beacon_send(&beacon_data);

    // Start the timer interrupt
    HAL_TIM_Base_Start_IT(&htim3);
}

/**
 * @brief  Implementation of printf() function
 * @param None
 * @retval None
 */
#ifdef __GNUC__
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif
PUTCHAR_PROTOTYPE
{
  HAL_UART_Transmit(&huart4, (uint8_t *)&ch, 1, HAL_MAX_DELAY);
  return ch;
}


/**
 * @brief Power Configuration
 * @retval None
 */
void SystemPower_Config(void)
{
	HAL_PWREx_EnableVddIO2();
	HAL_PWREx_DisableUCPDDeadBattery();

}
