/*
 * ssm_smart_sensor_ispu.h
 *
 *  Created on: Nov 28, 2024
 *      Author: ellie
 */

#ifndef INC_SSM_SMART_SENSOR_ISPU_H_
#define INC_SSM_SMART_SENSOR_ISPU_H_

/*
 * ssm_smart_sensor_ispu.h
 *
 *  Created on: Nov 28, 2024
 *      Author: ellie
 */

#ifndef SSM_SMART_SENSOR_ISPU_H
#define SSM_SMART_SENSOR_ISPU_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ----------------------------------------------------------------- */
#include <stdint.h>
#include "custom_ispu_motion_sensors_ex.h"

/* Definitions -------------------------------------------------------------- */
#define SSM_ISPU_PREDICTION_REG      ISM330IS_ISPU_DOUT_00_L
#define SSM_ISPU_FUNC_CFG_ENABLE     0x80
#define SSM_ISPU_FUNC_CFG_DISABLE    0x00

/* Function Prototypes ------------------------------------------------------ */

/**
 * @brief Initialize the smart sensor ISPU.
 *
 * @return int32_t BSP_ERROR_NONE on success, or an error code on failure.
 */
int32_t ssm_smart_sensor_ispu_init(void);

/**
 * @brief Configure the ISPU using the UCF configuration.
 *
 * @return int32_t BSP_ERROR_NONE on success, or an error code on failure.
 */
int32_t ssm_smart_sensor_ispu_configure_ucf(void);

/**
 * @brief Handle an event from the ISPU and read the prediction result.
 *
 * @param[out] output Pointer to a variable to store the prediction output.
 *
 * @return int32_t BSP_ERROR_NONE on success, or an error code on failure.
 */
int32_t ssm_smart_sensor_ispu_handle_event(uint8_t *output);

#ifdef __cplusplus
}
#endif

#endif /* SSM_SMART_SENSOR_ISPU_H */

#endif /* INC_SSM_SMART_SENSOR_ISPU_H_ */
