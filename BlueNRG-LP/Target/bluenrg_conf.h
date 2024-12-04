/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    Target/bluenrg_conf.h
  * @author  System Research & Applications Team - Agrate/Catania Lab.
  * @brief   BLE configuration file
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  *
  ******************************************************************************
  */

/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef BLUENRG_CONF_H
#define BLUENRG_CONF_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/

#include "stm32u5xx_hal.h"
#include <string.h>

/*---------- Maximum Advertising Interval (for a number N, Time = N x 0.625 msec) -----------*/
#define ADV_INTERV_MAX      1920
/*---------- Number of Bytes reserved for HCI Max Payload -----------*/
#define HCI_MAX_PAYLOAD_SIZE      260
/*---------- Number of incoming packets added to the list of packets to read -----------*/
#define HCI_READ_PACKET_NUM_MAX      10
/*---------- Minimum Advertising Interval (for a number N, Time = N x 0.625 msec) -----------*/
#define ADV_INTERV_MIN      1600
/*---------- Print messages from BLueNRG-LP files at middleware level -----------*/
#define BLUENRGLP_DEBUG      1
/*---------- Number of Bytes reserved for HCI Read Packet -----------*/
#define HCI_READ_PACKET_SIZE      260
/*---------- HCI Default Timeout -----------*/
#define HCI_DEFAULT_TIMEOUT_MS        1000

#define BLUENRG_memcpy                memcpy
#define BLUENRG_memset                memset
#define BLUENRG_memcmp                memcmp

#if BLUENRGLP_DEBUG
/**
  * User can change here printf with a custom implementation.
  * For example:
  * #define BLUENRG_PRINTF(...)   STBOX1_PRINTF(__VA_ARGS__)
  */
#include <stdio.h>
#define BLUENRG_PRINTF(...)         printf(__VA_ARGS__)
#else /* BLUENRGLP_DEBUG */
#define BLUENRG_PRINTF(...)
#endif /* BLUENRGLP_DEBUG */

#ifdef __cplusplus
}
#endif
#endif /* BLUENRG_CONF_H */
