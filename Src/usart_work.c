/* Includes ------------------------------------------------------------------*/
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
#include "FIFO.h"
#include "adc.h"
#include "usart.h"
#include "cfg_storage.h"
#include "flash_sec.h"
#include <string.h>

extern UART_HandleTypeDef huart4;
extern float K1, K2;
#define COUNTOF(__BUFFER__)   (sizeof(__BUFFER__) / sizeof(*(__BUFFER__)))
#define RXBUFFERSIZE                     10
#define TXSTARTMESSAGESIZE               (COUNTOF(aTxStartMessage) - 1)
#define TXENDMESSAGESIZE                 (COUNTOF(aTxEndMessage) - 1)
/* USER CODE BEGIN 4 */

uint8_t aRxBuffer[RXBUFFERSIZE];

#define ReadWord       0xA0
#define WriteWord      0x80
#define Flash_UpTime   0x00
#define FlashCoefReg   0x08
#define Flash_RTC      0x10

#define QuantityWORD 	3 //(QuantitySingl_Param)
#define amountParam     (QuantityWORD+1)  //
#define WconutNosrc16    (QuantityWORD*2)//
#define src16InPut      (QuantityWORD)    //


//#define amountParam 2
//#define amountWord (amountParam*2)
#define baseAdress 0x080e0000


uint8_t bBuf_fill, bErorFlash;
uint8_t Mu8_USART_Data[5];
uint16_t u16_adrdata ;
uint8_t Mu8_output[5];
uint16_t u16_dateFlash[amountParam];

	
uint16_t Crc16( unsigned char *pcBlock, unsigned short len )
{
    uint16_t crc = 0xFFFF;
    unsigned char i;
 
    while( len-- )
    {
        crc ^= *pcBlock++ << 8;
 
        for( i = 0; i < 8; i++ )
        crc = crc & 0x8000 ? ( crc << 1 ) ^ 0x1021 : crc << 1;
    }
 
    return crc;
}



uint16_t flash_read(uint32_t address) {
return (*(__IO uint16_t*) address);
}



	


//??????? ????????? true ????? ????? ??????? ??? ?????? ??????.

uint8_t flash_ready(void) {
return !(FLASH->SR & FLASH_SR_BSY);
}


void Flash_program_16 (uint16_t Data, uint32_t Address)
{
	//uint16_t *Data_Work;
	uint16_t i;
	uint16_t temp_datas[amountParam];
	FLASH_EraseInitTypeDef EraseInitStruct;
	uint32_t PageError ;
	EraseInitStruct.NbSectors = 1;
  EraseInitStruct.TypeErase = TYPEERASE_SECTORS;
	EraseInitStruct.Sector = FLASH_SECTOR_11;
	EraseInitStruct.VoltageRange = VOLTAGE_RANGE_3;


	while(!flash_ready()){}
	//Reed memory 
	for (i=0;i<amountParam;i++)
	{
		temp_datas[i]=flash_read(baseAdress+i*2);
	}
	//Replace data 
	temp_datas[Address] = Data;
	
	HAL_FLASH_Unlock();
	FLASH_Erase_Sector(FLASH_SECTOR_11, VOLTAGE_RANGE_3);
	HAL_FLASH_Lock();
	HAL_FLASH_Unlock();
	while(!flash_ready()){}

		//Save data 
		for (i=0;i<amountParam;i++)
		{
			__HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_EOP | FLASH_FLAG_OPERR |
                FLASH_FLAG_WRPERR | FLASH_FLAG_PGAERR |
                FLASH_FLAG_PGPERR | FLASH_FLAG_PGSERR);
			HAL_FLASH_Program(TYPEPROGRAM_HALFWORD, baseAdress+(i*2), temp_datas[i]);
				while(!flash_ready()){}
		
		}

		HAL_FLASH_Lock();
		while(!flash_ready()){};
}



void Flash_program (uint16_t *Data, uint16_t Size)
{
	//uint16_t *Data_Work;
	uint16_t i;
	uint16_t* tmp;
	tmp = (uint16_t*) Data;
	HAL_FLASH_Unlock();
	for (i=0;i<Size;i++)
	{
	HAL_FLASH_Program(TYPEPROGRAM_HALFWORD, 0x080e0000+(i*2), *(tmp+i));
	}
	HAL_FLASH_Lock();
}
#define  Koef_K1		0.00006103515625	
void Init_Koeff (void)
{
//	uint16_t DAta[4];
//	DAta[0]=0x5555;
//	DAta[1]=0x5678;
//	DAta[2]=0x9abc;
//	DAta[3]=0xdef0;
//	
//	
	K1 = flash_read(baseAdress)*Koef_K1;
	K2=flash_read(baseAdress+2)*Koef_K1;

//	HAL_FLASH_Unlock();
//	Flash_program_16(0x8888,0x0001);
////	HAL_FLASH_Program(TYPEPROGRAM_HALFWORD, 0x080e0000, 0x7777);
////	Flash_program (DAta,4);
//	HAL_FLASH_Lock();
//	data = flash_read(0x080e0000);
	
}


uint8_t Fn_Monitor( uint8_t *u8_mass,uint16_t *Mu16_Read_dateFlash ){
  static uint8_t u8_start = 0;

// static U16 u16_count = 0; 
 uint8_t *p_elem_mass, /*u8_summa,*/ Mu8_output[5], i;
 uint16_t u16_adrdata ;
 p_elem_mass = u8_mass;
	for (i=0;i<5;i++) Mu8_output[i]=0;
	//u8_summa = 	*p_elem_mass + *(p_elem_mass + 1) + *(p_elem_mass + 2) + *(p_elem_mass + 3);
	if(/**(p_elem_mass + 5) == u8_summa	*/1	){
		if(u8_start != 0x20){
		//========== start ??????????? ??????? ==========
			if(1/*(*p_elem_mass & 0xE0) == 0x20*/){
			Mu8_output[0] = 0x20;   		// === CMD=001, NF=0 
				Mu8_output[1] = 0x01;			// === ???????? ???????
				Mu8_output[2] = 0;
				Mu8_output[3] = 0x2; 			// === ?????? ?????? 2 ?????
				Mu8_output[4] =  Mu8_output[0] + Mu8_output[1] + Mu8_output[2] + Mu8_output[3]; 
				// ---- ???????? ??????? ?????? -----
		//		for(i=0;i<=4;i++){	  
		//			AT91F_US_PutChar(AT91C_BASE_US0,Mu8_output[i]);
		//			while(!AT91F_US_TxReady(AT91C_BASE_US0));
		//		}
				HAL_UART_Transmit(&huart4, (uint8_t*)Mu8_output, 5,2000);
//			while(txDoneFlag) {};
//				txDoneFlag = false;
				// ----------------------------------
//				AT91F_SPI_CfgPCS(AT91C_BASE_SPI0, 1);
						
//				u16_count = 0;
				u8_start = 0x20 ;
			}
       //========== end ??????????? ??????? ==========
		}
		else{
				if((*p_elem_mass & 0xE0) == ReadWord){
		
					u16_adrdata	= ((uint16_t)(*p_elem_mass & 0x07) << 8 ) | 	(*(p_elem_mass + 1));
//					for (i=0;i<amountParam;i++)
//					{
//					p_last[i] = cfg_pool_get(&cfg_pool);
//					s_last[i] = cfg_stor_get(&cfg_stor);
//					}
//					
					Mu16_Read_dateFlash[u16_adrdata] = flash_read(baseAdress+u16_adrdata*2);  // === ?????? ????????? ??????
					Mu8_output[0] = *(p_elem_mass);
					Mu8_output[1] = *(p_elem_mass+1);
					Mu8_output[2] = Mu16_Read_dateFlash[u16_adrdata];
					Mu8_output[3] = Mu16_Read_dateFlash[u16_adrdata] >> 8;
					Mu8_output[4] =  Mu8_output[0] + Mu8_output[1] + Mu8_output[2] + Mu8_output[3];
					
					// ---- ???????? ??????? ?????? ----- 
				//	for(i=0;i<=4;i++){	 
				//		AT91F_US_PutChar(AT91C_BASE_US0,Mu8_output[i]);
				//		while(!AT91F_US_TxReady(AT91C_BASE_US0));
				//	}
				HAL_UART_Transmit(&huart4, Mu8_output, 5,2000);
					
//				while(!txDoneFlag) {};
//				txDoneFlag = false;
					// ----------------------------------
				}
			// ==== end ?????????? ?????? ????? ?? flash =============
  
			//========== start ?????? ?????? ????? ? flash ===========
				if((*p_elem_mass & 0xE0) == WriteWord){
			
					u16_adrdata	= ((uint16_t)(*p_elem_mass & 0x07) << 8 ) | (*(p_elem_mass + 1));
					Mu16_Read_dateFlash[u16_adrdata] =	(((uint16_t)(*(p_elem_mass + 3))) << 8 ) | (*(p_elem_mass + 2));
					Mu16_Read_dateFlash[src16InPut] =	Crc16((unsigned char*)Mu16_Read_dateFlash,WconutNosrc16);
					Flash_program_16(Mu16_Read_dateFlash[u16_adrdata],u16_adrdata);
					Flash_program_16( Mu16_Read_dateFlash[src16InPut],src16InPut);
					Mu16_Read_dateFlash[u16_adrdata] = flash_read(baseAdress+u16_adrdata*2); 
					Mu16_Read_dateFlash[src16InPut]  = flash_read(baseAdress+src16InPut*2);	  // === ?????? ??????????? ?????
					
					//Init_Koeff();
			//		t.k[u16_adrdata] = Mu16_Read_dateFlash[u16_adrdata];
			//		t.k[u16_adrdata+1] = Mu16_Read_dateFlash[src16InPut];	
			//		cfg_pool_commit(&cfg_pool, &t); 
			//		cfg_stor_commit(&cfg_stor, &t);		
					//cfg_test_storage();
			//		write_flash_word( u16_adrdata, Mu16_Read_dateFlash[u16_adrdata]); // === ?????? ?????????? ??????
				//	write_flash_word( src16InPut, Mu16_Read_dateFlash[src16InPut]);	  // === ?????? ??????????? ?????
			//		Mu16_Read_dateFlash[u16_adrdata] = read_flash_word(u16_adrdata);  // === ?????? ????????? ??????
			//		Mu16_Read_dateFlash[src16InPut]  = read_flash_word(src16InPut);	  // === ?????? ??????????? ?????
			
				
					// ---- ???????? ??????????? ????? ------
					
//					if(Crc16((unsigned char*)Mu16_Read_dateFlash,WconutNosrc16) !=  Mu16_Read_dateFlash[src16InPut]){
//						Mu16_Read_dateFlash[u16_adrdata] = 0;
//						bErorFlash = 0; 
//					}
//					else bErorFlash = 1;
					// --------------------------------------
					Mu8_output[0] = *(p_elem_mass);
					Mu8_output[1] = *(p_elem_mass + 1);
					Mu8_output[2] = Mu16_Read_dateFlash[u16_adrdata];
					Mu8_output[3] = Mu16_Read_dateFlash[u16_adrdata] >> 8;
					Mu8_output[4] =  Mu8_output[0] + Mu8_output[1] + Mu8_output[2] + Mu8_output[3];
					// ---- ???????? ??????? ?????? -----
				//	for(i=0;i<=4;i++){	  
				//		AT91F_US_PutChar(AT91C_BASE_US0,Mu8_output[i]);
					//	while(!AT91F_US_TxReady(AT91C_BASE_US0));
				//	}
					HAL_UART_Transmit(&huart4, (uint8_t*)Mu8_output, 5,2000);
//		while(!txDoneFlag) {};
//				txDoneFlag = false;
					// ----------------------------------
				}
			//========== end ?????? ?????? ????? ? flash =============

			// ==== ????????? ?????? ?????????? ??????? ===
			if((*p_elem_mass & 0xF4) == 0xF4){
					Mu8_output[0] = 0xF4;
					Mu8_output[1] = 0;
					Mu8_output[2] = 0;
					Mu8_output[3] = 0;
					Mu8_output[4] = 0xF4;
			//		for(i=0;i<5;i++){	  //---- ???????? ??????? ??????
			//			AT91F_US_PutChar(AT91C_BASE_US0,Mu8_output[i]);
			//			while(!AT91F_US_TxReady(AT91C_BASE_US0));
			//		}
				HAL_UART_Transmit(&huart4, (uint8_t*)Mu8_output, 5,200);
//				while(!txDoneFlag) {};
//				txDoneFlag = false;
					u8_start = 0xF4;
			}
			// ============================================
		
		}
	}
	return	u8_start;
};
	
	
	
void Work_Usart (void)
{
 HAL_UART_Receive_DMA(&huart4, (uint8_t *)aRxBuffer, 5);
}


uint16_t data;



uint16_t coeff;













void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  /* Turn LED2 on: Transfer in reception process is correct */
  Fn_Monitor(aRxBuffer,u16_dateFlash);
}


