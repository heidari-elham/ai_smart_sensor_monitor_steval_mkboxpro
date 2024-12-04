/*
 * ssm_smart_sensor_ispu.c
 *
 *  Created on: Nov 28, 2024
 *      Author: ellie
 */
/* Includes ----------------------------------------------------------------- */
#include "custom_ispu_motion_sensors_ex.h"

#include "ssm_smart_sensor_ispu.h"

#include "ispu.h"

/* Definitions -------------------------------------------------------------- */

#define SSM_ISPU_PREDICTION_REG ISM330IS_ISPU_DOUT_00_L
#define SSM_ISPU_FUNC_CFG_ENABLE 0x80
#define SSM_ISPU_FUNC_CFG_DISABLE 0x00

/* Function definitions ----------------------------------------------------- */

int32_t ssm_smart_sensor_ispu_init(void)
{
	int32_t ret;

	ret = CUSTOM_ISPU_MOTION_SENSOR_Init(CUSTOM_ISM330IS_0, MOTION_ACCELERO);
	if (ret != BSP_ERROR_NONE)
		return ret;

	return BSP_ERROR_NONE;
}


int32_t ssm_smart_sensor_ispu_configure_ucf(void)
{
	int32_t ret = 0;

	for (uint16_t i = 0; i < sizeof(ispu_conf) / sizeof(ucf_line_ext_t); i++)
	{
		if (ispu_conf[i].op == MEMS_UCF_OP_WRITE)
		{
			ret |= CUSTOM_ISPU_MOTION_SENSOR_Write_Register(
				CUSTOM_ISM330IS_0,
				ispu_conf[i].address,
				ispu_conf[i].data
			);
		}
		else if (ispu_conf[i].op == MEMS_UCF_OP_DELAY)
		{
			HAL_Delay(ispu_conf[i].data);
		}
	}

	if (ret != BSP_ERROR_NONE)
		return ret;

	return BSP_ERROR_NONE;
}

int32_t ssm_smart_sensor_ispu_handle_event(uint8_t *output)
{
	int32_t ret;

	ret = CUSTOM_ISPU_MOTION_SENSOR_Write_Register(
		CUSTOM_ISM330IS_0,
		ISM330IS_FUNC_CFG_ACCESS,
		SSM_ISPU_FUNC_CFG_ENABLE
	);
	if (ret != BSP_ERROR_NONE)
		return ret;

	ret = CUSTOM_ISPU_MOTION_SENSOR_Read_Register(
		CUSTOM_ISM330IS_0,
		SSM_ISPU_PREDICTION_REG,
		output
	);
	if (ret != BSP_ERROR_NONE)
		return ret;

	ret = CUSTOM_ISPU_MOTION_SENSOR_Write_Register(
		CUSTOM_ISM330IS_0,
		ISM330IS_FUNC_CFG_ACCESS,
		SSM_ISPU_FUNC_CFG_DISABLE
	);
	if (ret != BSP_ERROR_NONE)
		return ret;

	return BSP_ERROR_NONE;
}

/* -------------------------------------------------------------------------- */
