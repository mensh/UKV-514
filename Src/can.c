/**
  ******************************************************************************
  * File Name          : CAN.c
  * Description        : This file provides code for the configuration
  *                      of the CAN instances.
  ******************************************************************************
  *
  * COPYRIGHT(c) 2015 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "can.h"

#include "gpio.h"
#include "string.h"
//#include "cmsis_os.h"
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

CAN_HandleTypeDef hcan1;

/* CAN1 init function */
void MX_CAN1_Init(void)
{
	static CanRxMsgTypeDef       RxMessage;
	static CanTxMsgTypeDef       TxMessage;
	CAN_FilterConfTypeDef  sFilterConfig;
  hcan1.Instance = CAN1;
  hcan1.Init.Prescaler = 5;
	hcan1.pTxMsg = &TxMessage;
  hcan1.pRxMsg = &RxMessage;
  hcan1.Init.Mode = CAN_MODE_NORMAL;
  hcan1.Init.SJW = CAN_SJW_4TQ;
  hcan1.Init.BS1 = CAN_BS1_2TQ;
  hcan1.Init.BS2 = CAN_BS2_3TQ;
  hcan1.Init.TTCM = DISABLE;
  hcan1.Init.ABOM = DISABLE;
  hcan1.Init.AWUM = DISABLE;
  hcan1.Init.NART = DISABLE;
  hcan1.Init.RFLM = DISABLE;
  hcan1.Init.TXFP = DISABLE;
  HAL_CAN_Init(&hcan1);

	  
   /*##-2- Configure the CAN Filter ###########################################*/
  sFilterConfig.FilterNumber = 0;
  sFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;
  sFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;
  sFilterConfig.FilterIdHigh = 0x46A<<5;
  sFilterConfig.FilterIdLow = 0x0000;
  sFilterConfig.FilterMaskIdHigh = 0x46A<<5;
  sFilterConfig.FilterMaskIdLow = 0x0000;
  sFilterConfig.FilterFIFOAssignment = 0;
  sFilterConfig.FilterActivation = ENABLE;
  sFilterConfig.BankNumber = 14;
	 HAL_CAN_ConfigFilter(&hcan1, &sFilterConfig);
	
	 sFilterConfig.FilterNumber = 1;
  sFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;
  sFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;
  sFilterConfig.FilterIdHigh = 0x44B<<5;
  sFilterConfig.FilterIdLow = 0x0000;
  sFilterConfig.FilterMaskIdHigh = 0x44B<<5;
  sFilterConfig.FilterMaskIdLow = 0x0000;
  sFilterConfig.FilterFIFOAssignment = 0;
  sFilterConfig.FilterActivation = ENABLE;
  sFilterConfig.BankNumber = 14;
	HAL_CAN_ConfigFilter(&hcan1, &sFilterConfig);
	

  
	HAL_CAN_Receive_IT(&hcan1, CAN_FIFO0);
}

void HAL_CAN_MspInit(CAN_HandleTypeDef* hcan)
{

  GPIO_InitTypeDef GPIO_InitStruct;
  if(hcan->Instance==CAN1)
  {
  /* USER CODE BEGIN CAN1_MspInit 0 */

  /* USER CODE END CAN1_MspInit 0 */
    /* Peripheral clock enable */
    __CAN1_CLK_ENABLE();
  
    /**CAN1 GPIO Configuration    
    PB8     ------> CAN1_RX
    PB9     ------> CAN1_TX 
    */
    GPIO_InitStruct.Pin = GPIO_PIN_8|GPIO_PIN_9;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF9_CAN1;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    /* Peripheral interrupt init*/
    HAL_NVIC_SetPriority(CAN1_TX_IRQn,5, 0);
    HAL_NVIC_EnableIRQ(CAN1_TX_IRQn);
    HAL_NVIC_SetPriority(CAN1_RX0_IRQn, 6, 0);
    HAL_NVIC_EnableIRQ(CAN1_RX0_IRQn);
    HAL_NVIC_SetPriority(CAN1_RX1_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(CAN1_RX1_IRQn);
  /* USER CODE BEGIN CAN1_MspInit 1 */

  /* USER CODE END CAN1_MspInit 1 */
  }
}

void HAL_CAN_MspDeInit(CAN_HandleTypeDef* hcan)
{

  if(hcan->Instance==CAN1)
  {
  /* USER CODE BEGIN CAN1_MspDeInit 0 */

  /* USER CODE END CAN1_MspDeInit 0 */
    /* Peripheral clock disable */
    __CAN1_CLK_DISABLE();
  
    /**CAN1 GPIO Configuration    
    PB8     ------> CAN1_RX
    PB9     ------> CAN1_TX 
    */
    HAL_GPIO_DeInit(GPIOB, GPIO_PIN_8|GPIO_PIN_9);

    /* Peripheral interrupt Deinit*/
    HAL_NVIC_DisableIRQ(CAN1_TX_IRQn);

    HAL_NVIC_DisableIRQ(CAN1_RX0_IRQn);

    HAL_NVIC_DisableIRQ(CAN1_RX1_IRQn);

  }
  /* USER CODE BEGIN CAN1_MspDeInit 1 */

  /* USER CODE END CAN1_MspDeInit 1 */
} 

uint8_t iTablo;
uint8_t DVR;
void HAL_CAN_RxCpltCallback(CAN_HandleTypeDef *CanHandle)
{
	     if (CanHandle->pRxMsg->StdId == 0x46A)   
       {
				iTablo=((CanHandle->pRxMsg->Data[1])&0x02)>>1;
			 }
			 if (CanHandle->pRxMsg->StdId == 0x44B)
			 {
				 DVR = ((CanHandle->pRxMsg->Data[2])&0x40)>>6;
			 }
			HAL_CAN_Receive_IT(CanHandle, CAN_FIFO0);
			hcan1.Instance->IER=hcan1.Instance->IER | 1<<1;
}


extern uint8_t bit_vsk_1_canal_plati;
extern uint8_t bit_vsk_2_canal_plati;
extern uint8_t bit_vsk_1_kz_obriv;
extern uint8_t bit_vsk_2_kz_obriv;
extern uint8_t usb_connect_can;
uint32_t timing;


void Thread_CAN_out  (void)
{

   if (timing==0)
	 {
//		 
//		 	     if (hcan1.pRxMsg->StdId == 0x46A)   
//       {
//				iTablo=((hcan1.pRxMsg->Data[2])&0x02)>>1;
//			 }
//			 if (hcan1.pRxMsg->StdId == 0x44B)
//			 {
//				 DVR = ((hcan1.pRxMsg->Data[2])&0x40)>>6;
//			 }
//			HAL_CAN_Receive(&hcan1, CAN_FIFO0,800);
		 
		 
			memset(hcan1.pTxMsg,0,sizeof(CanTxMsgTypeDef));	
			hcan1.pTxMsg->StdId = 0x54A;
			hcan1.pTxMsg->DLC = 5; 
			hcan1.pTxMsg->IDE = CAN_ID_STD;
			if (bit_vsk_1_canal_plati==0 /*&& bit_vsk_1_kz_obriv==0*/) hcan1.pTxMsg->Data[0]=vibr_1_canal_to_can&0xfffe;
			else hcan1.pTxMsg->Data[0]=0x0000;
		 
			if (bit_vsk_2_canal_plati==0 /*&& bit_vsk_2_kz_obriv==0*/) hcan1.pTxMsg->Data[2]=vibr_2_canal_to_can&0xfffe;
			else hcan1.pTxMsg->Data[2]=0x0000;
		 
			hcan1.pTxMsg->Data[4]=OV_1_canal<<0|OV_2_canal<<1|bit_vsk_1_canal_plati<<2|bit_vsk_2_canal_plati<<3|bit_vsk_1_kz_obriv<<4|bit_vsk_2_kz_obriv<<5|usb_connect_can<<6;
			HAL_CAN_Transmit(&hcan1,200);
			timing = 100 ;
		}
}

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
