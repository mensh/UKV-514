//#include "cmsis_os.h"
#include "stm32f2xx_hal.h"
#include "timCore.h"
#include "FIFO.h"
//#include "adc.h"
//#include "can.h"
//#include "dac.h"
//#include "dma.h"
//#include "spi.h"
//#include "tim.h"
//#include "usart.h"
//#include "usb_device.h"
//#include "gpio.h"
//#include "fsmc.h"
//#include "iir_filter.h"
//#include "filter_1_canal.h"
//#include "filter_2_canal.h"
#include "math.h"
#include "ringbuffer.h"
#include "fileSystem.h"
#include "core.h"

extern tdPagesQueue pagesQueue;
//fifo_t* fifo_buf;		

uint32_t k,z;
extern SPI_HandleTypeDef hspi1;
extern SPI_HandleTypeDef hspi3;

//RINGBUFFER_NEW(rb, 6000);

#define CS_ADC1_UP   		HAL_GPIO_WritePin(GPIOB,GPIO_PIN_2,GPIO_PIN_SET);	
#define CS_ADC1_DOWN   	HAL_GPIO_WritePin(GPIOB,GPIO_PIN_2,GPIO_PIN_RESET);	

#define CS_ADC2_UP   		HAL_GPIO_WritePin(GPIOC,GPIO_PIN_5,GPIO_PIN_SET);	
#define CS_ADC2_DOWN   	HAL_GPIO_WritePin(GPIOC,GPIO_PIN_5,GPIO_PIN_RESET);	

//const double   (*f_mass_point1_DEN[1])[3]={DEN_1};
//const double   (*f_mass_point1_NUM[1])[3]={NUM_1};
//const int *f_mass_point1_NL[1]={NL_1};
//const int *f_mass_point1_DL[1]={DL_1};

//const double  (*f_mass_point2_DEN[1])[3]={DEN_2};
//const double  (*f_mass_point2_NUM[1])[3]={NUM_2};
//const int *f_mass_point2_NL[1]={NL_2};
//const int *f_mass_point2_DL[1]={DL_2};

float ADC1_DATA;
float ADC2_DATA;

#define kFilteringFactor_2_canal 0.05
#define kFilteringFactor_1_canal 0.07
#define K_2_canal 800.f
#define K_1_canal 1911.f 	 //FS  - 4000Hz => usred 3 sec=3*4000
#define SIZE_1_canal 6000
#define SIZE_2_canal 1200

#define condensator_1_canal 2200.0e-12
#define coeff_usilen_1_canal 20.0 
#define tip_dat_1_canal  20.0e-12
#define AparatCoeff_1_canal	 (((coeff_usilen_1_canal*tip_dat_1_canal)/(condensator_1_canal))/2.0)
#define Const_1_canal		 (1.0/(AparatCoeff_1_canal))
#define TOR_COEFF_1_CANAL 1.44055
#define WINDOW_COEFF_1_canal 1.0*1.2879177*0.97826086956521739


#define condensator_2_canal 2200.0e-12
#define coeff_usilen_2_canal 40.0 
#define tip_dat_2_canal  20.0e-12
#define AparatCoeff_2_canal	 (((coeff_usilen_2_canal*tip_dat_2_canal)/(condensator_2_canal))/2.0)
#define Const_2_canal		 (1.0/(  AparatCoeff_2_canal))
#define TOR_COEFF_2_CANAL 1.0
#define WINDOW_COEFF_2_canal 1.0*2.5*1.0416666*0.868194842*1.03668261*1.01246105919003115


#define VIBR_TO_PERCENT_1_CANAL (1.f/0.45f) 
#define VIBR_TO_PERCENT_2_CANAL (1.f/0.65f) 

float K1=1, K2=1;
uint16_t   vibr_2_canal_to_can=0,vibr_1_canal_to_can=0;


float integral_2_canal=0.f,integral_1_canal=0.f;
float accelX_2_canal=0.f,accelX_1_canal=0.f;
float rollingX_2_canal=0.f,rollingX_1_canal=0.f;			
float pik_2_canal=0.f,pik_1_canal=0.f;
float integral_old_2_canal=0.f,integral_old_1_canal=0.f;
float vibr_2_canal=0.f,vibr_1_canal=0.f;
float  History_1_canal[13],History_2_canal[13];

uint32_t Timer_OV_1_canal=10000, Timer_OV_2_canal=10000;
uint8_t OV_1_canal=0, OV_2_canal=0;

float buffer_2_canal_av[SIZE_2_canal],buffer_1_canal_av[SIZE_1_canal];
uint32_t   indexs_2_canal=0,indexs_1_canal=0;
float	Sum_2_canal,Sum_1_canal;

int16_t ADC1_code;
int16_t ADC2_code;

void FIFO (void)
{

uint32_t i;
indexs_2_canal = 0;
indexs_1_canal=0;
Sum_1_canal = 0.f;
Sum_2_canal = 0.f;
for(i = 0; i < SIZE_1_canal; i++) buffer_1_canal_av[i] = 0.f;	 
for(i = 0; i < SIZE_2_canal; i++) buffer_2_canal_av[i] = 0.f;	 
for(i=0;i<13;i++ )History_1_canal[i]=0.f;
for(i=0;i<13;i++ )History_2_canal[i]=0.f;
	
integral_2_canal=0.f;integral_1_canal=0.f;
accelX_2_canal=0.f;accelX_1_canal=0.f;
rollingX_2_canal=0.f;rollingX_1_canal=0.f;			
pik_2_canal=0.f;pik_1_canal=0.f;
integral_old_2_canal=0.f;integral_old_1_canal=0.f;
vibr_2_canal=0.f;vibr_1_canal=0.f;	
	
	
}


//??????? ?? ????? ??? 1 ??????
void AddNewValue_1_canal(float newValue)
{
Sum_1_canal -= buffer_1_canal_av[indexs_1_canal];
buffer_1_canal_av[indexs_1_canal] = newValue;
Sum_1_canal += newValue;
indexs_1_canal = (indexs_1_canal + 1 < SIZE_1_canal) ? indexs_1_canal + 1: 0;
}

//??????? ?? ????? ??? 1 ??????
void AddNewValue_2_canal(float newValue)
{
Sum_2_canal -= buffer_2_canal_av[indexs_2_canal];	
buffer_2_canal_av[indexs_2_canal] = newValue;
Sum_2_canal += newValue;
indexs_2_canal = (indexs_2_canal + 1 < SIZE_2_canal) ? indexs_2_canal + 1: 0;
}

float getAvarage_1_canal(void)
{
return Sum_1_canal / SIZE_1_canal;
}


float getAvarage_2_canal(void)
{
return (Sum_2_canal/SIZE_2_canal);
}

extern float iir_1_canal(float NewSample);
extern float iir_2_canal(float NewSample);

int16_t Read_ADC_1 (void)
{
	uint8_t RX[5];
	int16_t code_ADC;
	uint16_t code;

	CS_ADC1_UP;
	__nop();
	__nop();
	__nop();
	__nop();
	CS_ADC1_DOWN;
		HAL_SPI_Receive(&hspi1,RX,2,200);
		while (HAL_SPI_GetState(&hspi1) != HAL_SPI_STATE_READY)
		{
     //osThreadYield();
		}
	code = (RX[0]<<8)|(RX[1]);
	code_ADC = code << 4 ;
	return (code_ADC >> 4);
}


int16_t Read_ADC_2 (void)
{

	uint8_t RX[5];
	int16_t code_ADC;
	uint16_t code;

	CS_ADC2_UP;
	__nop();
	__nop();
	__nop();
	__nop();
	CS_ADC2_DOWN;
	
	HAL_SPI_Receive(&hspi3,RX,2,200);
	while (HAL_SPI_GetState(&hspi3) != HAL_SPI_STATE_READY)
  {
   //  osThreadYield();
  }
	code = (RX[0]<<8)|(RX[1]);
	code_ADC = code << 4 ;
	return (code_ADC >> 4);
}


void OV (void)
{
	if (Timer_OV_1_canal==0) OV_1_canal=1;
	else OV_1_canal=0;
	
	if (Timer_OV_2_canal==0) OV_2_canal=1;
	else OV_2_canal=0;
}

void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi)
{
    if (hspi->Instance==hspi1.Instance)
    {
   // CS_ADC1_UP;
    }
    
    if (hspi->Instance==hspi3.Instance)
    {
    //CE_UP;
    }
}

extern uint8_t VSK_ZK_OBRIV_run;

extern RingBuffer *rb;


void TIM7_IRQHandler(void)
{
	//tdMemoryPage *pPage = &pagesQueue.page[ pagesQueue.eye % CORE_PAGES_QUEUE_SIZE ];
	uint8_t mass[5];
	if (VSK_ZK_OBRIV_run==1)
	{
		//CS_ADC2_UP;
		ADC2_code = Read_ADC_2();
		ADC2_DATA = ADC2_code*20.f/4095.f;
		//integral_2_canal=IIR_filter(ADC2_DATA, History_2_canal, DEN_2_canal, NUM_2_canal, 8 ,NL_2_canal,DL_2_canal);
		integral_2_canal=iir_2_canal(ADC2_DATA);
		rollingX_2_canal = (integral_2_canal * kFilteringFactor_2_canal) + (rollingX_2_canal * (1.f - kFilteringFactor_2_canal));
		accelX_2_canal += ((integral_2_canal-rollingX_2_canal)+integral_old_2_canal)*0.5f*1000.f*9.8f*0.0025; 
		integral_old_2_canal=(integral_2_canal-rollingX_2_canal);
		pik_2_canal=(pik_2_canal+((fabsf(accelX_2_canal)-pik_2_canal))/K_2_canal);
		AddNewValue_2_canal (pik_2_canal);	
		vibr_2_canal=getAvarage_2_canal();
		vibr_2_canal=(vibr_2_canal*Const_2_canal*WINDOW_COEFF_2_canal)*K2;
		if (vibr_2_canal>96.0) vibr_2_canal=96.0;
		vibr_2_canal_to_can=(vibr_2_canal*128.f*VIBR_TO_PERCENT_2_CANAL);		
		
		mass[0]=0x02;
		mass[1]=(ADC2_code&0x00FF)>>0;
		mass[2]=(ADC2_code&0xFF00)>>8;
		mass[3]=(vibr_2_canal_to_can&0x00FF)>>0;
		mass[4]=(vibr_2_canal_to_can&0xFF00)>>8;
		rbPushN(rb,mass,5);
		//fifoPushBytes(fifo_buf,mass,5);
//		ringbuffer_write(&rb,mass,5);
		if (vibr_2_canal<=65) Timer_OV_2_canal=10000;
	}
	else 
	{
		vibr_2_canal_to_can=0;
		FIFO();
	}
		HAL_TIM_IRQHandler(&htim7);

}


/**
* @brief This function handles TIM3 global interrupt.
*/
void TIM3_IRQHandler(void)
{
		uint8_t mass[5];
	if (VSK_ZK_OBRIV_run==1)
	{
		ADC1_code = Read_ADC_1();
			ADC1_DATA = ADC1_code*20.f/4095.f;
	//	integral_1_canal=IIR_filter(ADC1_DATA, History_1_canal, DEN_1_canal, NUM_1_canal, 8 ,NL_1_canal,DL_1_canal);
		integral_1_canal=iir_1_canal(ADC1_DATA)	;
		rollingX_1_canal = (integral_1_canal * kFilteringFactor_1_canal) + (rollingX_1_canal * (1.f - kFilteringFactor_1_canal));
		accelX_1_canal += ((integral_1_canal-rollingX_1_canal)+integral_old_1_canal)*0.5f*1000.f*9.8f*0.00025; 
		integral_old_1_canal=(integral_1_canal-rollingX_1_canal);
		pik_1_canal=(pik_1_canal+((fabsf(accelX_1_canal)-pik_1_canal))/K_1_canal);
		AddNewValue_1_canal (pik_1_canal);	
		vibr_1_canal=getAvarage_1_canal();
		vibr_1_canal=(vibr_1_canal*Const_1_canal*WINDOW_COEFF_1_canal)*K1;
		if (vibr_1_canal>67.0) vibr_1_canal=67.0;
		vibr_1_canal_to_can=(vibr_1_canal*128.f*VIBR_TO_PERCENT_1_CANAL);		
		
		mass[0]=0x01;
		mass[1]=(ADC1_code&0x00FF)>>0;
		mass[2]=(ADC1_code&0xFF00)>>8;
		mass[3]=(vibr_1_canal_to_can&0x00FF)>>0;
		mass[4]=(vibr_1_canal_to_can&0xFF00)>>8;
		rbPushN(rb,mass,5);
//		ringbuffer_write(&rb,mass,5);
		//fifoPushBytes(fifo_buf,mass,5);
		if (vibr_1_canal<=45) Timer_OV_1_canal=10000;
	}
	else 
	{
		vibr_1_canal_to_can=0;
		FIFO();
	}
		HAL_TIM_IRQHandler(&htim3);

}










void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
//	const	double   (*DEN_1_canal)[3],(*NUM_1_canal)[3];
//	const int *NL_1_canal, *DL_1_canal;
//	
//	const	double  (*DEN_2_canal)[3],(*NUM_2_canal)[3];
//	const int *NL_2_canal, *DL_2_canal;
//	
//	DEN_1_canal=(f_mass_point1_DEN[0]);
//	NUM_1_canal=(f_mass_point1_NUM[0]);
//	NL_1_canal=(f_mass_point1_NL[0]);
//	DL_1_canal=(f_mass_point1_DL[0]);
//	
//	DEN_2_canal=(f_mass_point2_DEN[0]);
//	NUM_2_canal=(f_mass_point2_NUM[0]);
//	NL_2_canal=(f_mass_point2_NL[0]);
//	DL_2_canal=(f_mass_point2_DL[0]);
//	
//	
//	if (htim->Instance==TIM3)    //2000HZ
//	{
//	
//	
//	//CS_ADC1_DOWN;
//		

//	}

//	if (htim->Instance==TIM7) //400Hz
//	{

//		//CS_ADC2_DOWN;
//	}
	
	OV();
}


