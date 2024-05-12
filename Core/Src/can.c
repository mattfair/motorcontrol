/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file    can.c
 * @brief   This file provides code for the configuration
 *          of the CAN instances.
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
 ******************************************************************************
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "can.h"

/* USER CODE BEGIN 0 */
#include "hw_config.h"
#include "math_ops.h"
#include "user_config.h"
/* USER CODE END 0 */

CAN_HandleTypeDef hcan1;

/* CAN1 init function */
void MX_CAN1_Init(void)
{
  /* USER CODE BEGIN CAN1_Init 0 */

  /* USER CODE END CAN1_Init 0 */

  /* USER CODE BEGIN CAN1_Init 1 */

  /* USER CODE END CAN1_Init 1 */
  hcan1.Instance = CAN1;
  hcan1.Init.Prescaler = 3;
  hcan1.Init.Mode = CAN_MODE_NORMAL;
  hcan1.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan1.Init.TimeSeg1 = CAN_BS1_12TQ;
  hcan1.Init.TimeSeg2 = CAN_BS2_2TQ;
  hcan1.Init.TimeTriggeredMode = DISABLE;
  hcan1.Init.AutoBusOff = DISABLE;
  hcan1.Init.AutoWakeUp = ENABLE;
  hcan1.Init.AutoRetransmission = DISABLE;
  hcan1.Init.ReceiveFifoLocked = DISABLE;
  hcan1.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN1_Init 2 */

  /* USER CODE END CAN1_Init 2 */
}

void HAL_CAN_MspInit(CAN_HandleTypeDef *canHandle)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if (canHandle->Instance == CAN1)
  {
    /* USER CODE BEGIN CAN1_MspInit 0 */

    /* USER CODE END CAN1_MspInit 0 */
    /* CAN1 clock enable */
    __HAL_RCC_CAN1_CLK_ENABLE();

    __HAL_RCC_GPIOB_CLK_ENABLE();
    /**CAN1 GPIO Configuration
    PB8     ------> CAN1_RX
    PB9     ------> CAN1_TX
    */
    GPIO_InitStruct.Pin = GPIO_PIN_8 | GPIO_PIN_9;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF9_CAN1;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    /* CAN1 interrupt Init */
    HAL_NVIC_SetPriority(CAN1_TX_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(CAN1_TX_IRQn);
    HAL_NVIC_SetPriority(CAN1_RX0_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(CAN1_RX0_IRQn);
    HAL_NVIC_SetPriority(CAN1_RX1_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(CAN1_RX1_IRQn);
    HAL_NVIC_SetPriority(CAN1_SCE_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(CAN1_SCE_IRQn);
    /* USER CODE BEGIN CAN1_MspInit 1 */

    /* USER CODE END CAN1_MspInit 1 */
  }
}

void HAL_CAN_MspDeInit(CAN_HandleTypeDef *canHandle)
{
  if (canHandle->Instance == CAN1)
  {
    /* USER CODE BEGIN CAN1_MspDeInit 0 */

    /* USER CODE END CAN1_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_CAN1_CLK_DISABLE();

    /**CAN1 GPIO Configuration
    PB8     ------> CAN1_RX
    PB9     ------> CAN1_TX
    */
    HAL_GPIO_DeInit(GPIOB, GPIO_PIN_8 | GPIO_PIN_9);

    /* CAN1 interrupt Deinit */
    HAL_NVIC_DisableIRQ(CAN1_TX_IRQn);
    HAL_NVIC_DisableIRQ(CAN1_RX0_IRQn);
    HAL_NVIC_DisableIRQ(CAN1_RX1_IRQn);
    HAL_NVIC_DisableIRQ(CAN1_SCE_IRQn);
    /* USER CODE BEGIN CAN1_MspDeInit 1 */

    /* USER CODE END CAN1_MspDeInit 1 */
  }
}

/* USER CODE BEGIN 1 */

void HAL_CAN_Reset(CAN_HandleTypeDef *hcan)
{
  // First, stop the CAN peripheral
  if (HAL_CAN_Stop(hcan) != HAL_OK)
  {
    printf("Failed to stop CAN\r\n");
  }

  // Deinitialize the MSP (Low Level Init)
  HAL_CAN_MspDeInit(hcan);

  // Reset the CAN peripheral
  __HAL_RCC_CAN1_FORCE_RESET();
  __HAL_RCC_CAN1_RELEASE_RESET();

  // Re-initialize the MSP
  HAL_CAN_MspInit(hcan);

  if (HAL_CAN_Init(hcan) != HAL_OK)
  {
    printf("Failed to re-initialize CAN\r\n");
  }

  // Start the CAN peripheral
  if (HAL_CAN_Start(hcan) != HAL_OK)
  {
    printf("Failed to start CAN\r\n");
  }

  // Reconfigure filters if necessary
  CAN_FilterTypeDef CAN_FilterConfig = {0};
  CAN_FilterConfig.FilterBank = 0;
  CAN_FilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;
  CAN_FilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;
  CAN_FilterConfig.FilterIdHigh = 0x0000;
  CAN_FilterConfig.FilterIdLow = 0x0000;
  CAN_FilterConfig.FilterMaskIdHigh = 0x0000;
  CAN_FilterConfig.FilterMaskIdLow = 0x0000;
  CAN_FilterConfig.FilterFIFOAssignment = CAN_RX_FIFO0;
  CAN_FilterConfig.FilterActivation = ENABLE;

  if (HAL_CAN_ConfigFilter(hcan, &CAN_FilterConfig) != HAL_OK)
  {
    printf("Failed to configure filters\r\n");
  }
}

void can_rx_init(CANRxMessage *msg)
{
  msg->filter.FilterFIFOAssignment = CAN_FILTER_FIFO0;  // set fifo assignment
  msg->filter.FilterIdHigh = CAN_ID << 5;               // CAN ID
  msg->filter.FilterIdLow = 0x0;
  msg->filter.FilterMaskIdHigh = 0xFFF;
  msg->filter.FilterMaskIdLow = 0;
  msg->filter.FilterMode = CAN_FILTERMODE_IDMASK;
  msg->filter.FilterScale = CAN_FILTERSCALE_32BIT;
  msg->filter.FilterActivation = ENABLE;
  HAL_CAN_ConfigFilter(&CAN_H, &msg->filter);
}

void can_tx_init(CANTxMessage *msg)
{
  msg->tx_header.DLC = 7;           // message size of 7 byte
  msg->tx_header.IDE = CAN_ID_STD;  // set identifier to standard
  msg->tx_header.RTR =
      CAN_RTR_DATA;  // set data type to remote transmission request?
  msg->tx_header.StdId = CAN_MASTER;  // recipient CAN ID
}

/// CAN Reply Packet Structure ///
/// 16 bit position, between -4*pi and 4*pi
/// 12 bit velocity, between -30 and + 30 rad/s
/// 12 bit current, between -40 and 40;
/// CAN Packet is 5 8-bit words
/// Formatted as follows.  For each quantity, bit 0 is LSB
/// 0: [position[15-8]]
/// 1: [position[7-0]]
/// 2: [velocity[11-4]]
/// 3: [velocity[3-0], current[11-8]]
/// 4: [current[7-0]]
void pack_reply(CANTxMessage *msg, uint8_t id, float p, float v, float t,
                float vb)
{
  int p_int = float_to_uint(p, P_MIN, P_MAX, 16);
  int v_int = float_to_uint(v, V_MIN, V_MAX, 12);
  int t_int = float_to_uint(t, -(I_MAX + SENSE_BUFFER) * KT * GR,
                            (I_MAX + SENSE_BUFFER) * KT * GR, 12);
  int vb_int = float_to_uint(vb, VB_MIN, VB_MAX, 8);
  msg->data[0] = id;
  msg->data[1] = p_int >> 8;
  msg->data[2] = p_int & 0xFF;
  msg->data[3] = v_int >> 4;
  msg->data[4] = ((v_int & 0xF) << 4) + (t_int >> 8);
  msg->data[5] = t_int & 0xFF;
  msg->data[6] = vb_int;

  printf("Sending CAN Message %02X %02X %02X %02X %02X %02X %02X\n\r",
         msg->data[0], msg->data[1], msg->data[2], msg->data[3], msg->data[4],
         msg->data[5], msg->data[6]);
}

/// CAN Command Packet Structure ///
/// 16 bit position command, between -4*pi and 4*pi
/// 12 bit velocity command, between -30 and + 30 rad/s
/// 12 bit kp, between 0 and 500 N-m/rad
/// 12 bit kd, between 0 and 100 N-m*s/rad
/// 12 bit feed forward torque, between -18 and 18 N-m
/// CAN Packet is 8 8-bit words
/// Formatted as follows.  For each quantity, bit 0 is LSB
/// 0: [position[15-8]]
/// 1: [position[7-0]]
/// 2: [velocity[11-4]]
/// 3: [velocity[3-0], kp[11-8]]
/// 4: [kp[7-0]]
/// 5: [kd[11-4]]
/// 6: [kd[3-0], torque[11-8]]
/// 7: [torque[7-0]]
void unpack_cmd(CANRxMessage msg, float *commands)
{  // ControllerStruct * controller){
  printf("Received CAN Message %02X %02X %02X %02X %02X %02X %02X %02X\n\r",
         msg.data[0], msg.data[1], msg.data[2], msg.data[3], msg.data[4],
         msg.data[5], msg.data[6], msg.data[7]);
  int p_int = (msg.data[0] << 8) | msg.data[1];
  int v_int = (msg.data[2] << 4) | (msg.data[3] >> 4);
  int kp_int = ((msg.data[3] & 0xF) << 8) | msg.data[4];
  int kd_int = (msg.data[5] << 4) | (msg.data[6] >> 4);
  int t_int = ((msg.data[6] & 0xF) << 8) | msg.data[7];

  commands[0] = uint_to_float(p_int, P_MIN, P_MAX, 16);
  commands[1] = uint_to_float(v_int, V_MIN, V_MAX, 12);
  commands[2] = uint_to_float(kp_int, KP_MIN, KP_MAX, 12);
  commands[3] = uint_to_float(kd_int, KD_MIN, KD_MAX, 12);
  commands[4] = uint_to_float(t_int, -I_MAX * KT * GR, I_MAX * KT * GR, 12);
  // printf("Received   ");
  // printf("%.3f  %.3f  %.3f  %.3f  %.3f   %.3f", controller->p_des,
  // controller->v_des, controller->kp, controller->kd, controller->t_ff,
  // controller->i_q_ref); printf("\n\r");
}

void PrintCANErrorMessages(CAN_HandleTypeDef *hcan)
{
  uint32_t error = HAL_CAN_GetError(hcan);

  // Check for no error first
  if (error == HAL_CAN_ERROR_NONE)
  {
    printf("No CAN errors.\r\n");
  }
  else
  {
    // General error flags
    if (error & HAL_CAN_ERROR_EWG)
    {
      printf("CAN Error Warning.\r\n");
    }
    if (error & HAL_CAN_ERROR_EPV)
    {
      printf("CAN Error Passive.\r\n");
    }
    if (error & HAL_CAN_ERROR_BOF)
    {
      printf("CAN Bus-Off Error.\r\n");
    }
    if (error & HAL_CAN_ERROR_STF)
    {
      printf("CAN Stuff Error.\r\n");
    }
    if (error & HAL_CAN_ERROR_FOR)
    {
      printf("CAN Form Error.\r\n");
    }
    if (error & HAL_CAN_ERROR_ACK)
    {
      printf("CAN Acknowledgment Error.\r\n");
    }
    if (error & HAL_CAN_ERROR_BR)
    {
      printf("CAN Bit Recessive Error.\r\n");
    }
    if (error & HAL_CAN_ERROR_BD)
    {
      printf("CAN Bit Dominant Error.\r\n");
    }
    if (error & HAL_CAN_ERROR_CRC)
    {
      printf("CAN CRC Error.\r\n");
    }
    if (error & HAL_CAN_ERROR_RX_FOV0)
    {
      printf("CAN RX FIFO 0 Overrun Error.\r\n");
    }
    if (error & HAL_CAN_ERROR_RX_FOV1)
    {
      printf("CAN RX FIFO 1 Overrun Error.\r\n");
    }
    if (error & HAL_CAN_ERROR_TX_ALST0)
    {
      printf("CAN TX Mailbox 0 Arbitration Lost.\r\n");
    }
    if (error & HAL_CAN_ERROR_TX_TERR0)
    {
      printf("CAN TX Mailbox 0 Transmission Error.\r\n");
    }
    if (error & HAL_CAN_ERROR_TX_ALST1)
    {
      printf("CAN TX Mailbox 1 Arbitration Lost.\r\n");
    }
    if (error & HAL_CAN_ERROR_TX_TERR1)
    {
      printf("CAN TX Mailbox 1 Transmission Error.\r\n");
    }
    if (error & HAL_CAN_ERROR_TX_ALST2)
    {
      printf("CAN TX Mailbox 2 Arbitration Lost.\r\n");
    }
    if (error & HAL_CAN_ERROR_TX_TERR2)
    {
      printf("CAN TX Mailbox 2 Transmission Error.\r\n");
    }
    if (error & HAL_CAN_ERROR_TIMEOUT)
    {
      printf("CAN Timeout Error.\r\n");
    }

    // Read the Error Status Register
    uint32_t esr = hcan->Instance->ESR;
    printf("Transmit Error Counter: %lu\r\n", (esr >> 16) & 0xFF);
    printf("Receive Error Counter: %lu\r\n", (esr >> 24) & 0xFF);
    printf("Error Status Register Flags: 0x%08X\r\n",
           esr & 0x00FFFFFF);  // Mask out irrelevant bits

    // Read the status of the CAN controller from the Status Register
    uint32_t tsr = hcan->Instance->TSR;
    printf("Status Register: 0x%08X\r\n", tsr);
    if (tsr & CAN_TSR_TME0)
    {
      printf("Mailbox 0 is empty.\r\n");
    }
    if (tsr & CAN_TSR_TME1)
    {
      printf("Mailbox 1 is empty.\r\n");
    }
    if (tsr & CAN_TSR_TME2)
    {
      printf("Mailbox 2 is empty.\r\n");
    }
  }
}

void PrintCANStatusRegisters(CAN_HandleTypeDef *hcan)
{
  if (!hcan)
  {
    printf("CAN handle is NULL\r\n");
    return;
  }

  // Master Control Register (MCR)
  printf("MCR: 0x%08X\r\n", (unsigned int)hcan->Instance->MCR);
  printf("  Initialization Request (INRQ): %s\r\n",
         (hcan->Instance->MCR & CAN_MCR_INRQ) ? "SET" : "RESET");
  printf("  Sleep Mode Request (SLEEP): %s\r\n",
         (hcan->Instance->MCR & CAN_MCR_SLEEP) ? "SET" : "RESET");

  // Master Status Register (MSR)
  printf("MSR: 0x%08X\r\n", (unsigned int)hcan->Instance->MSR);
  printf("  CAN Initialization OK (INAK): %s\r\n",
         (hcan->Instance->MSR & CAN_MSR_INAK) ? "SET" : "RESET");
  printf("  Sleep Acknowledge (SLAK): %s\r\n",
         (hcan->Instance->MSR & CAN_MSR_SLAK) ? "SET" : "RESET");

  // Transmit Status Register (TSR)
  printf("TSR: 0x%08X\r\n", (unsigned int)hcan->Instance->TSR);
  printf("  Transmit Mailbox 0 Empty (TME0): %s\r\n",
         (hcan->Instance->TSR & CAN_TSR_TME0) ? "EMPTY" : "FULL");
  printf("  Transmit Mailbox 1 Empty (TME1): %s\r\n",
         (hcan->Instance->TSR & CAN_TSR_TME1) ? "EMPTY" : "FULL");
  printf("  Transmit Mailbox 2 Empty (TME2): %s\r\n",
         (hcan->Instance->TSR & CAN_TSR_TME2) ? "EMPTY" : "FULL");
  printf("  Request Completed Mailbox 0 (RQCP0): %s\r\n",
         (hcan->Instance->TSR & CAN_TSR_RQCP0) ? "YES" : "NO");
  printf("  Transmission OK Mailbox 0 (TXOK0): %s\r\n",
         (hcan->Instance->TSR & CAN_TSR_TXOK0) ? "YES" : "NO");

  // Receive FIFO 0 and 1 Registers (RF0R, RF1R)
  printf("RF0R: 0x%08X\r\n", (unsigned int)hcan->Instance->RF0R);
  printf("  Receive FIFO 0 Full (FULL0): %s\r\n",
         (hcan->Instance->RF0R & CAN_RF0R_FULL0) ? "FULL" : "NOT FULL");
  printf("  Receive FIFO 0 Overrun (FOVR0): %s\r\n",
         (hcan->Instance->RF0R & CAN_RF0R_FOVR0) ? "OVERRUN" : "NO OVERRUN");
  printf("RF1R: 0x%08X\r\n", (unsigned int)hcan->Instance->RF1R);
  printf("  Receive FIFO 1 Full (FULL1): %s\r\n",
         (hcan->Instance->RF1R & CAN_RF1R_FULL1) ? "FULL" : "NOT FULL");
  printf("  Receive FIFO 1 Overrun (FOVR1): %s\r\n",
         (hcan->Instance->RF1R & CAN_RF1R_FOVR1) ? "OVERRUN" : "NO OVERRUN");

  // Error Status Register (ESR)
  printf("ESR: 0x%08X\r\n", (unsigned int)hcan->Instance->ESR);
  printf("  Error Passive (EPVF): %s\r\n",
         (hcan->Instance->ESR & CAN_ESR_EPVF) ? "ERROR PASSIVE" : "NORMAL");
  printf("  Bus-off Status (BOFF): %s\r\n",
         (hcan->Instance->ESR & CAN_ESR_BOFF) ? "BUS-OFF" : "CONNECTED");
  printf("  Transmit Error Counter (TEC): %u\r\n",
         (unsigned int)((hcan->Instance->ESR & CAN_ESR_TEC) >> 16));
  printf("  Receive Error Counter (REC): %u\r\n",
         (unsigned int)((hcan->Instance->ESR & CAN_ESR_REC) >> 24));
}

void CanErrorCallback(CAN_HandleTypeDef *hcan)
{
  printf("CAN Error Callback\r\n");
  PrintCANErrorMessages(hcan);
  PrintCANStatusRegisters(hcan);
}

/* USER CODE END 1 */
