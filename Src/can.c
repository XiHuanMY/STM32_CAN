/**
  ******************************************************************************
  * File Name          : CAN.c
  * Description        : This file provides code for the configuration
  *                      of the CAN instances.
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

/* Includes ------------------------------------------------------------------*/
#include "can.h"

/* USER CODE BEGIN 0 */
extern UART_HandleTypeDef huart2;
extern uint8_t rx_buffer[200] ;
/* USER CODE END 0 */

CAN_HandleTypeDef hcan;

/* CAN init function */
void MX_CAN_Init(void)
{
	
  hcan.Instance = CAN1;
  hcan.Init.Prescaler = 24;
  hcan.Init.Mode = CAN_MODE_NORMAL;
  hcan.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan.Init.TimeSeg1 = CAN_BS1_3TQ;
  hcan.Init.TimeSeg2 = CAN_BS2_2TQ;
  hcan.Init.TimeTriggeredMode = DISABLE;
  hcan.Init.AutoBusOff = DISABLE;
  hcan.Init.AutoWakeUp = DISABLE;
  hcan.Init.AutoRetransmission = DISABLE;
  hcan.Init.ReceiveFifoLocked = DISABLE;
  hcan.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan) != HAL_OK)
  {
    Error_Handler();
  }
	//HAL_CAN_Start(&hcan);
}

void HAL_CAN_MspInit(CAN_HandleTypeDef* canHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if(canHandle->Instance==CAN1)
  {
  /* USER CODE BEGIN CAN1_MspInit 0 */

  /* USER CODE END CAN1_MspInit 0 */
    /* CAN1 clock enable */
    __HAL_RCC_CAN1_CLK_ENABLE();
  
    __HAL_RCC_GPIOB_CLK_ENABLE();
    /**CAN GPIO Configuration    
    PB8     ------> CAN_RX
    PB9     ------> CAN_TX 
    */
//		GPIO_PinRemapConfig(GPIO_Remap1_CAN1,ENABLE);
    GPIO_InitStruct.Pin = GPIO_PIN_8;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_PIN_9;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    __HAL_AFIO_REMAP_CAN1_2();

    /* CAN1 interrupt Init */
    HAL_NVIC_SetPriority(USB_LP_CAN1_RX0_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(USB_LP_CAN1_RX0_IRQn);
  /* USER CODE BEGIN CAN1_MspInit 1 */

  /* USER CODE END CAN1_MspInit 1 */
  }
}

void HAL_CAN_MspDeInit(CAN_HandleTypeDef* canHandle)
{

  if(canHandle->Instance==CAN1)
  {
  /* USER CODE BEGIN CAN1_MspDeInit 0 */

  /* USER CODE END CAN1_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_CAN1_CLK_DISABLE();
  
    /**CAN GPIO Configuration    
    PB8     ------> CAN_RX
    PB9     ------> CAN_TX 
    */
    HAL_GPIO_DeInit(GPIOB, GPIO_PIN_8|GPIO_PIN_9);

    /* CAN1 interrupt Deinit */
    HAL_NVIC_DisableIRQ(USB_LP_CAN1_RX0_IRQn);
  /* USER CODE BEGIN CAN1_MspDeInit 1 */

  /* USER CODE END CAN1_MspDeInit 1 */
  }
} 

/* USER CODE BEGIN 1 */
HAL_StatusTypeDef CAN_Filter_Config(void)
{	
	HAL_StatusTypeDef  HAL_Status;  
	/*##-2- Configure the CAN1 Filter ###########################################*/
	CAN_FilterTypeDef sFilterConfig;
	sFilterConfig.FilterIdHigh = 0;
	sFilterConfig.FilterIdLow = 0;
	sFilterConfig.FilterMaskIdHigh = 0;
	sFilterConfig.FilterMaskIdLow = 0;
	sFilterConfig.FilterBank = 0;
	sFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;
	sFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;
	sFilterConfig.FilterFIFOAssignment = CAN_FILTER_FIFO0;
	sFilterConfig.FilterActivation = ENABLE;
	sFilterConfig.SlaveStartFilterBank = 14;
	HAL_CAN_ConfigFilter(&hcan, &sFilterConfig);
	HAL_Status=HAL_CAN_Start(&hcan);  //开启CAN  
	if(HAL_Status!=HAL_OK)
	{  
		
		return HAL_Status;
	}    
	HAL_Status=HAL_CAN_ActivateNotification(&hcan,   CAN_IT_RX_FIFO0_MSG_PENDING);  
	if(HAL_Status!=HAL_OK)
	{  
		
		return HAL_Status;
	}  
	return HAL_OK;
}
uint8_t tempCanTxdata2[100];
bool CAN_SendNormalMsg(CAN_HandleTypeDef* hcanx, uint32_t StdId, uint8_t* pdata, uint8_t length)
{
		CAN_TxHeaderTypeDef TxMessage;
		HAL_StatusTypeDef	HAL_RetVal;
		TxMessage.StdId = StdId;      //帧ID为传入参数的CAN_ID
    TxMessage.IDE = CAN_ID_STD;    //标准帧
    TxMessage.RTR = CAN_RTR_DATA;  //数据帧
    TxMessage.DLC = 0x08;          //帧长度初始化为8
	
	for(uint8_t Txcount=0;Txcount<length;)
	{
		uint8_t remain=length-Txcount;
		if(remain>=8)
		{
			TxMessage.DLC = 0x08;
			
		}
		else
		{
			TxMessage.DLC = remain;
		}
		uint32_t count_overtime=0;
		while(HAL_CAN_GetTxMailboxesFreeLevel(hcanx)==0)	//HAL_CAN_GetTxMailboxesFreeLevel(hcan);
		{
			count_overtime++;
			if(count_overtime>5000)
				break;
		}
		HAL_RetVal=HAL_CAN_AddTxMessage(hcanx,&TxMessage,&pdata[Txcount],(uint32_t*)tempCanTxdata2); 

		if(HAL_RetVal!=HAL_OK)
		{
			return false;
		}	
		Txcount+=TxMessage.DLC;
	}
	return true;
}
extern uint8_t recv_end_flag;
uint32_t can_test_count[10]={0};
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
  if(hcan->Instance==CAN1)
  {
	   CAN_RxHeaderTypeDef RxMessage1;
	   CAN_RecvMsg can_recvmsg;
     HAL_CAN_GetRxMessage(hcan,CAN_RX_FIFO0,&RxMessage1,can_recvmsg.Data);
	  
		
		
		HAL_UART_Transmit(&huart2,can_recvmsg.Data,RxMessage1.DLC,200);//接收数据打印出来
	  //CAN1_Hit_Analysis(&RxMessage1,&can_recvmsg);
  }
}



/* USER CODE END 1 */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
