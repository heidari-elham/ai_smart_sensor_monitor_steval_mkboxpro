/*
 * ssm_smart_sensor_mlc.c
 *
 *  Created on: Aug 23, 2024
 *      Author: ellie
 */

/* Includes ----------------------------------------------------------------- */

#include "mlc.h"
#include "custom_motion_sensors_ex.h"



int32_t ssm_smart_sensor_mlc_init(void)
{
	int32_t ret;
	ret = CUSTOM_MOTION_SENSOR_Init(CUSTOM_LSM6DSV16X_0, MOTION_ACCELERO);
	if (ret != BSP_ERROR_NONE)
	    return ret;
	return BSP_ERROR_NONE;
}

int32_t ssm_smart_sensor_mlc_configure_ucf(void)
{
	int32_t ret = 0;

	/* Load UCF configuration */
	for (uint16_t i = 0; i < (sizeof(mlc_configuration) / sizeof(ucf_line_t)); i++) {
		ret |= CUSTOM_MOTION_SENSOR_Write_Register(
			CUSTOM_LSM6DSV16X_0,
			mlc_configuration[i].address,
			mlc_configuration[i].data
		);
	}

	if (ret != BSP_ERROR_NONE)
		return ret;

	return BSP_ERROR_NONE;
}

int32_t ssm_smart_sensor_mlc_handle_event(uint8_t* output)
{
	int32_t ret;

	ret = CUSTOM_MOTION_SENSOR_Write_Register(
		CUSTOM_LSM6DSV16X_0,
		LSM6DSV16X_FUNC_CFG_ACCESS,
		0x80
	);
	if (ret != BSP_ERROR_NONE)
		return ret;

	ret = CUSTOM_MOTION_SENSOR_Read_Register(
		CUSTOM_LSM6DSV16X_0,
		LSM6DSV16X_MLC1_SRC,
		output
	);
	if (ret != BSP_ERROR_NONE)
		return ret;

	ret = CUSTOM_MOTION_SENSOR_Write_Register(
		CUSTOM_LSM6DSV16X_0,
		LSM6DSV16X_FUNC_CFG_ACCESS,
		0x00
	);
	if (ret != BSP_ERROR_NONE)
		return ret;

	return BSP_ERROR_NONE;
}
