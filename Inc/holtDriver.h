#ifndef __HOLT_DRIVER_H_
#define __HOLT_DRIVER_H_



/*===============================================================================================*/
/*===============================================================================================*/
// Comments



/*===============================================================================================*/
/*===============================================================================================*/
// Includes
#include "stdint.h"
#include "tdStatus.h"
#include "spiDriver.h"
#include "bhd2settings.h"


/*===============================================================================================*/
/*===============================================================================================*/
// Defines
#define HOLT_DRIVER_HI3598_DATA_CHANNEL_SHIFT 4
#define HOLT_DRIVER_HI3598_DATA_CHANNEL_MASK 0x0F
#define HOLT_DRIVER_HI3598_DATA_OP_CODE_SHIFT 0
#define HOLT_DRIVER_HI3598_DATA_OP_CODE_MASK 0x0F

#define HOLT_DRIVER_FIFO_STATUS_EMPTY 0x00
#define HOLT_DRIVER_FIFO_STATUS_NOT_FULL 0x00

#define HOLT_DRIVER_HI3598_DATA_BYTE_MASK 0xFF

#define HOLT_DRIVER_HI3598_DATA_OP_INSTRUCTION_SIZE 1
#define HOLT_DRIVER_HI3598_DATA_CONFIGURATION_SIZE  2
#define HOLT_DRIVER_HI3598_DATA_WORD_SIZE 4
#define HOLT_DRIVER_HI3598_DATA_STATUS_SIZE 2

#define HOLT_DRIVER_HI3598_DATA_RECEIVER_FIFO_SIZE 4
#define HOLT_DRIVER_HI3598_DATA_RECEIVER_FIFO_ONE_WORD 1

#define HOLT_DRIVER_HI3598_CHANNEL_ENUMERATION_START 1
#define HOLT_DRIVER_HI3598_CHANNELS_AMOUNT 8

#define HOLT_DRIVER_HI3598_CONFIGURATION_SPEED_BIT 													0
#define HOLT_DRIVER_HI3598_CONFIGURATION_R_FLAG_DEFENITION_BIT							1
#define HOLT_DRIVER_HI3598_CONFIGURATION_LABEL_RECOGNATION_BIT							2
#define HOLT_DRIVER_HI3598_CONFIGURATION_RESET_RECEIVER_BIT									3
#define HOLT_DRIVER_HI3598_CONFIGURATION_PARITY_CHECK_BIT										4
#define HOLT_DRIVER_HI3598_CONFIGURATION_SELF_TEST_BIT											5
#define HOLT_DRIVER_HI3598_CONFIGURATION_RECEIVER_DECODER_BIT								6
#define HOLT_DRIVER_HI3598_CONFIGURATION_ARINC_BIT_10_REQUIRED_STATE_BIT		7
#define HOLT_DRIVER_HI3598_CONFIGURATION_ARINC_BIT_9_REQUIRED_STATE_BIT			8
#define HOLT_DRIVER_HI3598_CONFIGURATION_ARINC_LABEL_ORDER_BIT							9

#define HOLT_DRIVER_HI3598_CONFIGURATION_SPEED_HIGH													(0 << HOLT_DRIVER_HI3598_CONFIGURATION_SPEED_BIT)
#define HOLT_DRIVER_HI3598_CONFIGURATION_SPEED_LOW 													(1 << HOLT_DRIVER_HI3598_CONFIGURATION_SPEED_BIT)
#define HOLT_DRIVER_HI3598_CONFIGURATION_R_FLAG_DEFENITION_NOT_EMPTY				(0 << HOLT_DRIVER_HI3598_CONFIGURATION_R_FLAG_DEFENITION_BIT)
#define HOLT_DRIVER_HI3598_CONFIGURATION_R_FLAG_DEFENITION_FULL							(1 << HOLT_DRIVER_HI3598_CONFIGURATION_R_FLAG_DEFENITION_BIT)
#define HOLT_DRIVER_HI3598_CONFIGURATION_LABEL_RECOGNATION_DISABLE					(0 << HOLT_DRIVER_HI3598_CONFIGURATION_LABEL_RECOGNATION_BIT)
#define HOLT_DRIVER_HI3598_CONFIGURATION_LABEL_RECOGNATION_ENABLE						(1 << HOLT_DRIVER_HI3598_CONFIGURATION_LABEL_RECOGNATION_BIT)
#define HOLT_DRIVER_HI3598_CONFIGURATION_RESET_RECEIVER_FALSE								(0 << HOLT_DRIVER_HI3598_CONFIGURATION_RESET_RECEIVER_BIT)
#define HOLT_DRIVER_HI3598_CONFIGURATION_RESET_RECEIVER_TRUE								(1 << HOLT_DRIVER_HI3598_CONFIGURATION_RESET_RECEIVER_BIT)
#define HOLT_DRIVER_HI3598_CONFIGURATION_PARITY_CHECK_DISABLE								(0 << HOLT_DRIVER_HI3598_CONFIGURATION_PARITY_CHECK_BIT)
#define HOLT_DRIVER_HI3598_CONFIGURATION_PARITY_CHECK_ENABLE								(1 << HOLT_DRIVER_HI3598_CONFIGURATION_PARITY_CHECK_BIT)
#define HOLT_DRIVER_HI3598_CONFIGURATION_SELF_TEST_ENABLE										(0 << HOLT_DRIVER_HI3598_CONFIGURATION_SELF_TEST_BIT)
#define HOLT_DRIVER_HI3598_CONFIGURATION_SELF_TEST_DISABLE									(1 << HOLT_DRIVER_HI3598_CONFIGURATION_SELF_TEST_BIT)
#define HOLT_DRIVER_HI3598_CONFIGURATION_RECEIVER_DECODER_DISABLE						(0 << HOLT_DRIVER_HI3598_CONFIGURATION_RECEIVER_DECODER_BIT)
#define HOLT_DRIVER_HI3598_CONFIGURATION_RECEIVER_DECODER_ENABLE						(1 << HOLT_DRIVER_HI3598_CONFIGURATION_RECEIVER_DECODER_BIT)
#define HOLT_DRIVER_HI3598_CONFIGURATION_ARINC_BIT_10_REQUIRED_STATE_FALSE	(0 << HOLT_DRIVER_HI3598_CONFIGURATION_ARINC_BIT_10_REQUIRED_STATE_BIT)
#define HOLT_DRIVER_HI3598_CONFIGURATION_ARINC_BIT_10_REQUIRED_STATE_TRUE		(1 << HOLT_DRIVER_HI3598_CONFIGURATION_ARINC_BIT_10_REQUIRED_STATE_BIT)
#define HOLT_DRIVER_HI3598_CONFIGURATION_ARINC_BIT_9_REQUIRED_STATE_FALSE		(0 << HOLT_DRIVER_HI3598_CONFIGURATION_ARINC_BIT_9_REQUIRED_STATE_BIT)
#define HOLT_DRIVER_HI3598_CONFIGURATION_ARINC_BIT_9_REQUIRED_STATE_TRUE		(1 << HOLT_DRIVER_HI3598_CONFIGURATION_ARINC_BIT_9_REQUIRED_STATE_BIT)
#define HOLT_DRIVER_HI3598_CONFIGURATION_LABEL_ORDER_REVERSED								(0 << HOLT_DRIVER_HI3598_CONFIGURATION_ARINC_LABEL_ORDER_BIT)
#define HOLT_DRIVER_HI3598_CONFIGURATION_LABEL_ORDER_DIRECT									(1 << HOLT_DRIVER_HI3598_CONFIGURATION_ARINC_LABEL_ORDER_BIT)

#define HOLT_DRIVER_CHANNELS_READY_PINS (GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7)

#ifdef USB_HARDWARE_FS
#define HOLT_DRIVER_CHANNELS_READY_PORT GPIOC
#else
#define HOLT_DRIVER_CHANNELS_READY_PORT GPIOF
#endif

#define HOLT_DRIVER_CHANNELS_READY (HOLT_DRIVER_CHANNELS_READY_PORT->IDR & HOLT_DRIVER_CHANNELS_READY_PINS)



/*===============================================================================================*/
/*===============================================================================================*/
// Data types, structures, ect.

typedef enum { setConfiguration = 0x04, readConfiguration = 0x05, readRxFifo = 0x03, readStatus = 0x06, reset = 0x07, loopBackTxSpeedHigh = 0x08, loopBackTxSpeedLow = 0x09 } tdHoltOpCode;


typedef struct
	{
	uint8_t notEmpty;
	uint8_t full;
	} tdHoltStatus;



/*===============================================================================================*/
/*===============================================================================================*/
tdStatus HOLT_DRIVER_dataTransmition(tdSpiDevice *_pDevice, uint8_t *_pTxData, uint8_t *_pRxData, uint16_t _size);
tdStatus HOLT_DRIVER_setConfiguration(tdSpiDevice *_pDevice, uint8_t _channel, uint16_t _configuration);
tdStatus HOLT_DRIVER_readConfiguration(tdSpiDevice *_pDevice, uint8_t _channel, uint16_t *_pConfiguration);
tdStatus HOLT_DRIVER_readFifoWord(tdSpiDevice *_pDevice, uint8_t _channel, uint32_t *_word);
tdStatus HOLT_DRIVER_readFifoData(tdSpiDevice *_pDevice, uint8_t _channel, uint8_t *_data);
tdStatus HOLT_DRIVER_readStatus(tdSpiDevice *_pDevice, tdHoltStatus *_pStatus);
tdStatus HOLT_DRIVER_masterReset(tdSpiDevice *_pDevice);
tdStatus HOLT_DRIVER_writeLoopBackFifoWord(tdSpiDevice *_pDevice, uint8_t _speedHigh, uint32_t _word);



/*===============================================================================================*/
/*===============================================================================================*/



#endif
