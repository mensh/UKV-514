#ifndef __SPI_H_
#define __SPI_H_



/*===============================================================================================*/
/*===============================================================================================*/
// Comments



/*===============================================================================================*/
/*===============================================================================================*/
// Includes
#include "stdint.h"
#include "spi.h"
#include "tdStatus.h"
#include "stm32f2xx_hal.h"
#include "stm32f2xx_hal_spi.h"



/*===============================================================================================*/
/*===============================================================================================*/
// Defines
#define SPI_DRIVER_SPI_TxRx_COMPLETE_CALLBACK_ENABLE 0

#define SPI_DRIVER_TRANSMITION_TIMEOUT_ENABLE 0
#define SPI_DRIVER_TRANSMITION_TIMEOUT_MS 10

#define SPI_DRIVER_CHANNELS 3
#define SPI_DRIVER_CHANNEL_1 0
#define SPI_DRIVER_CHANNEL_2 1
#define SPI_DRIVER_CHANNEL_3 2



/*===============================================================================================*/
/*===============================================================================================*/
// Data types, structures, ect.
typedef enum {Down = 0, Up = 1} tdEnumSpiCSState;

typedef struct
	{
	GPIO_TypeDef *pGPIO;
	uint32_t pin;
	} tdSpiCS;

typedef struct
	{
	SPI_HandleTypeDef *pSpi;
	tdSpiCS chipSelect;
	uint8_t channel;
	} tdSpiDevice;

typedef struct
	{
	tdSpiDevice *pChannel[ SPI_DRIVER_CHANNELS ];
	} tdSpiDriver;



/*===============================================================================================*/
/*===============================================================================================*/
tdStatus SPI_DRIVER_initDevice(tdSpiDevice *_pDevice, SPI_HandleTypeDef *_pSpiHandle, tdSpiCS *_pCs);

tdStatus SPI_DRIVER_dataTransmition(tdSpiDevice *_pDevice, uint8_t *_pTxData, uint8_t *_pRxData, uint16_t _size);



/*===============================================================================================*/
/*===============================================================================================*/



#endif
