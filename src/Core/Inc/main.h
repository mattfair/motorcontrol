/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.h
 * @brief          : Header for main.c file.
 *                   This file contains the common defines of the application.
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
 * All rights reserved.</center></h2>
 *
 * This software component is licensed by ST under BSD 3-Clause license,
 * the "License"; You may not use this file except in compliance with the
 * License. You may obtain a copy of the License at:
 *                        opensource.org/licenses/BSD-3-Clause
 *
 ******************************************************************************
 */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <reg/udral/physics/dynamics/translation/LinearTs_0_1.h>
#include <reg/udral/physics/electricity/PowerTs_0_1.h>
#include <reg/udral/service/actuator/common/Feedback_0_1.h>
#include <reg/udral/service/actuator/common/Status_0_1.h>
#include <reg/udral/service/actuator/common/_0_1.h>
#include <reg/udral/service/common/Readiness_0_1.h>
#include <uavcan/_register/Access_1_0.h>
#include <uavcan/_register/List_1_0.h>
#include <uavcan/node/ExecuteCommand_1_1.h>
#include <uavcan/node/GetInfo_1_0.h>
#include <uavcan/node/Heartbeat_1_0.h>
#include <uavcan/node/port/List_0_1.h>
#include <uavcan/pnp/NodeIDAllocationData_1_0.h>
#include "canard.h"

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */
    // forward declare state struct
    typedef struct State State;

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define B1_Pin GPIO_PIN_13
#define B1_GPIO_Port GPIOC
#define USART_TX_Pin GPIO_PIN_2
#define USART_TX_GPIO_Port GPIOA
#define USART_RX_Pin GPIO_PIN_3
#define USART_RX_GPIO_Port GPIOA
#define TMS_Pin GPIO_PIN_13
#define TMS_GPIO_Port GPIOA
#define TCK_Pin GPIO_PIN_14
#define TCK_GPIO_Port GPIOA
#define SWO_Pin GPIO_PIN_3
#define SWO_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

#define KILO 1000L
#define MEGA ( (int64_t)KILO * KILO )

    CanardMicrosecond GetMonotonicMicroseconds( void );
    const char* PortToName( const CanardPortID port_id );
    const char* KindToName( const CanardTransferKind kind );
    void ProcessMessageServoSetpoint( State* const state,
                                      const reg_udral_physics_dynamics_translation_Linear_0_1* const msg );
    void ProcessMessageServiceReadiness( State* const state,
                                         const reg_udral_service_common_Readiness_0_1* const msg,
                                         const CanardMicrosecond monotonic_time );
    void ProcessMessagePlugAndPlayNodeIDAllocation( State* const state,
                                                    const uavcan_pnp_NodeIDAllocationData_1_0* const msg );
    void ProcessMessagePlugAndPlayNodeIDAllocation( State* const state,
                                                    const uavcan_pnp_NodeIDAllocationData_1_0* const msg );
    uavcan_node_GetInfo_Response_1_0 ProcessRequestNodeGetInfo();
    uavcan_register_Access_Response_1_0 ProcessRequestRegisterAccess( const uavcan_register_Access_Request_1_0* req );
    uavcan_node_ExecuteCommand_Response_1_1 ProcessRequestExecuteCommand(
        const uavcan_node_ExecuteCommand_Request_1_1* req );

    void Send( State* const state,
               const CanardMicrosecond tx_deadline_usec,
               const CanardTransferMetadata* const metadata,
               const size_t payload_size,
               const void* const payload );

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
