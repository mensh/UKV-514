#include "can.h"
#include "dma.h"
#include "rng.h"
#include "spi.h"
#include "gpio.h"
#include "core.h"
#include "bhd2settings.h"
#include "timCore.h"
#include "dac.h"
#include "math.h"
#include "adc.h"


#define RELE6_ON   		HAL_GPIO_WritePin(GPIOB,GPIO_PIN_15,GPIO_PIN_SET);	
#define RELE6_OFF   	HAL_GPIO_WritePin(GPIOB,GPIO_PIN_15,GPIO_PIN_RESET);	

#define RELE4_ON   		HAL_GPIO_WritePin(GPIOE,GPIO_PIN_6,GPIO_PIN_SET);	
#define RELE4_OFF   	HAL_GPIO_WritePin(GPIOE,GPIO_PIN_6,GPIO_PIN_RESET);	

#define RELE2_ON   		HAL_GPIO_WritePin(GPIOE,GPIO_PIN_4,GPIO_PIN_SET);	
#define RELE2_OFF   	HAL_GPIO_WritePin(GPIOE,GPIO_PIN_4,GPIO_PIN_RESET);	

#define RELE4_ON   		HAL_GPIO_WritePin(GPIOE,GPIO_PIN_6,GPIO_PIN_SET);	
#define RELE4_OFF   	HAL_GPIO_WritePin(GPIOE,GPIO_PIN_6,GPIO_PIN_RESET);	

#define RELE1_ON   		HAL_GPIO_WritePin(GPIOE,GPIO_PIN_3,GPIO_PIN_SET);	
#define RELE1_OFF   	HAL_GPIO_WritePin(GPIOE,GPIO_PIN_3,GPIO_PIN_RESET);	

#define RELE3_ON   		HAL_GPIO_WritePin(GPIOE,GPIO_PIN_5,GPIO_PIN_SET);	
#define RELE3_OFF   	HAL_GPIO_WritePin(GPIOE,GPIO_PIN_5,GPIO_PIN_RESET);	




uint16_t DATA4DAC[360]={0};


void Sinus_calculate(float Delitel)
{
	
	uint16_t i;
  for(i=0; i<360; i++)
  {
    DATA4DAC[i] = (uint16_t)((sin(i*(3.141592/180.f)) + 2.f)*4095.f/3.3)/Delitel;
  }
}




void RELE_OFF (void)
{
	RELE1_OFF;
	RELE2_OFF;
	RELE3_OFF;
	RELE4_OFF;
	//RELE5_OFF;
	RELE6_OFF;
}


uint8_t VSK_ZK_OBRIV_run=1;
uint16_t rawADC[4];

uint8_t bit_vsk_1_canal_plati=0;
uint8_t bit_vsk_2_canal_plati=0;

uint8_t bit_vsk_1_kz_obriv=0;
uint8_t bit_vsk_2_kz_obriv=0;

extern uint8_t DVR;

uint32_t Timer_VSK;

uint32_t counter_ADC1=0;
uint16_t mass_ADC1[128];
uint8_t start_ADC1=0;
uint8_t ADC1_data_ready=0;
float RMS_1;
float data_temp;

void SKZ_1 (void)
{
	uint16_t i;
	
	if (counter_ADC1<128 && start_ADC1==1) 
	{
		mass_ADC1[counter_ADC1]=rawADC[0];
		counter_ADC1++;
	}
	else if (counter_ADC1>=128) 
	{
		ADC1_data_ready=1;
		for (i=0;i<counter_ADC1;i++)
		{
			data_temp=(mass_ADC1[i]*3.3/4096)-1.65;
			RMS_1=RMS_1+((data_temp*data_temp));
		}
		RMS_1=sqrt(RMS_1/counter_ADC1);
	}
}


uint32_t counter_ADC2=0;
uint16_t mass_ADC2[128];
uint8_t start_ADC2=0;
uint8_t ADC2_data_ready=0;
float RMS_2;


void SKZ_2 (void)
{
	uint16_t i;
	
	if (counter_ADC2<128 && start_ADC2==1) 
	{
		mass_ADC2[counter_ADC2]=rawADC[1];
		counter_ADC2++;
	}
	else if (counter_ADC2>=128) 
	{
		ADC2_data_ready=1;
		for (i=0;i<counter_ADC2;i++)
		{
			data_temp=(mass_ADC2[i]*3.3/4096)-1.65;
			RMS_2=RMS_2+((data_temp*data_temp));
		}
		RMS_2=sqrt(RMS_2/counter_ADC2);
	}
}





//void DMA2_Stream1_IRQHandler(void)
//{
//  /* USER CODE BEGIN DMA2_Stream1_IRQn 0 */

//  /* USER CODE END DMA2_Stream1_IRQn 0 */
//  HAL_DMA_IRQHandler(&hdma_adc3);
//  /* USER CODE BEGIN DMA2_Stream1_IRQn 1 */

//  /* USER CODE END DMA2_Stream1_IRQn 1 */
//}




void Thread_Control  (void)
{
	static 	uint8_t State=0;
	static uint8_t Vsk_timer_run=0;
	//HAL_DAC_Start_DMA(&hdac,DAC_CHANNEL_1,(uint32_t*)DATA4DAC,360,DAC_ALIGN_12B_R);	
	static uint8_t Compl=0;

		if (DVR==0 && Compl==0)
		{
		switch(State)
		{
			case(0):
			{	
				RELE_OFF();
				HAL_DAC_Start_DMA(&hdac,DAC_CHANNEL_1,(uint32_t*)DATA4DAC,360,DAC_ALIGN_12B_R);	
				HAL_ADC_Start_DMA(&hadc3, (uint32_t*)&rawADC, 4);
				State=1;
				break;
			}
			
			
			case(1):
			{
			
				if (Vsk_timer_run==0)
				{
					Sinus_calculate(3.5);
				//KZ OBRIV VNUTRENIH CEPEI 1 CANAL
					htim.Init.Period=0x7ff/3/2.6;
					HAL_TIM_Base_Init(&htim);
					RELE2_ON;  
					RELE6_OFF;	
					if (iTablo==1) Timer_VSK = (10000);
					else  Timer_VSK = (10000);
					Vsk_timer_run=1;
				}
				if (Timer_VSK==0)
				{
					if (vibr_1_canal_to_can/128.f<40)  bit_vsk_1_canal_plati=1;
					else bit_vsk_1_canal_plati=0;
					
					RELE2_OFF;
					State=2;
					Vsk_timer_run=0;
				}
				break;
			}
			case(2):
			{
				//KZ OBRIV VNUTRENIH CEPEI 2 CANAL
	
				if (Vsk_timer_run==0)
				{
					Sinus_calculate(16);
					htim.Init.Period=0x7ff*3;
					HAL_TIM_Base_Init(&htim);
					RELE4_ON;
					RELE6_ON;
					
					if (iTablo==1) Timer_VSK = (10000);
					else  Timer_VSK = (10000);
					Vsk_timer_run=1;
				}
				if (Timer_VSK==0)
				{
					if (vibr_2_canal_to_can/128.f<40)  bit_vsk_2_canal_plati=1;
					else bit_vsk_2_canal_plati=0;
					RELE4_OFF;
					State=3;
					Vsk_timer_run=0;
				}
				break;
			}
			case(3):
			{
				if (1) 
				{
	
				if (Vsk_timer_run==0) 
				{
					//DATCHIK 1 CANAL
					htim.Init.Period=0x7ff/10;
					HAL_TIM_Base_Init(&htim);
					VSK_ZK_OBRIV_run=0;
					Sinus_calculate(1);
					RELE2_OFF;
					RELE6_OFF;
					RELE1_ON;
					start_ADC1=1;
					Timer_VSK=(10000);
					Vsk_timer_run=1;
				}
				if (ADC1_data_ready==0) SKZ_1(); 
				
				if (Timer_VSK==0)
				{	
					if (RMS_1<0.15 || RMS_1>0.4) bit_vsk_1_kz_obriv=1;
					//else bit_vsk_1_kz_obriv
					RELE1_OFF;
					State=4;
					Vsk_timer_run=0;
				}
				}
				break;
			}
			case (4):
			{
				if (1) 
				{
				//DATCHIK 2 CANAL
			
				if (Vsk_timer_run==0) 
				{
					RELE4_OFF;
					RELE6_ON;
					RELE3_ON;
					Timer_VSK=(10000);	
					start_ADC2=1;
					Vsk_timer_run=1;
				}
				if (ADC2_data_ready==0) SKZ_2(); 
				
				if (Timer_VSK==0)
				{
					if (RMS_2<0.15 || RMS_2>0.29) bit_vsk_2_kz_obriv=1;
					HAL_DAC_Stop(&hdac,DAC_CHANNEL_1);
					RELE6_OFF;
					RELE3_OFF;
					State=5;
					Vsk_timer_run=0;
					VSK_ZK_OBRIV_run=1;
				}
				}
				break;
			}
			case(5):
			{
				
				if (iTablo==1 && Compl==1) 
				{
					State=0;
					Compl = 0;
				}
				Compl = 1;
				break;
			}
		}
	}

	
}

