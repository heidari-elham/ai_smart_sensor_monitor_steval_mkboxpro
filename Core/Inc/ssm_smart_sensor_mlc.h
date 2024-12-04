/*
 * ssm_smart_sensor_mlc.h
 *
 *  Created on: Aug 23, 2024
 *      Author: ellie
 */

#ifndef INC_SSM_SMART_SENSOR_MLC_H_
#define INC_SSM_SMART_SENSOR_MLC_H_

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ----------------------------------------------------------------- */

#include <stdint.h>

/* Function prototypes ------------------------------------------------------ */

int32_t ssm_smart_sensor_mlc_init(void);
int32_t ssm_smart_sensor_mlc_configure_ucf(void);
int32_t ssm_smart_sensor_mlc_handle_event(uint8_t *output);

#ifdef __cplusplus
}
#endif


#endif /* INC_SSM_SMART_SENSOR_MLC_H_ */
