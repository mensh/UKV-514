#ifndef __HOLT_CORE_H_
#define __HOLT_CORE_H_



/*===============================================================================================*/
/*===============================================================================================*/
// Comments



/*===============================================================================================*/
/*===============================================================================================*/
// Includes
#include "stdint.h"
#include "tdStatus.h"
#include "holtDriver.h"

/*===============================================================================================*/
/*===============================================================================================*/
// Defines
#define HOLT_CORE_READ_BYTES_AMOUNT 5
#define HOLT_CORE_CHANNELS_AMOUNT 8

#define HOLT_CORE_LABELS_AMOUNT 256 /* 2 ^ 8 - one byte label */

#define HOLT_CORE_USB_SLOT_TD_SIZE 2
#define HOLT_CORE_USB_SLOT_TD_ICHN_SHIFT 0
#define HOLT_CORE_USB_SLOT_TD_FREQ_SHIFT 1

#define CORE_HOLT3598_CONFIGURATION ( HOLT_DRIVER_HI3598_CONFIGURATION_SELF_TEST_DISABLE | HOLT_DRIVER_HI3598_CONFIGURATION_LABEL_ORDER_DIRECT )

/*===============================================================================================*/
/*===============================================================================================*/
// Data types, structures, ect.

typedef enum { frequency_100_kHz = 0, frequency_12_5_kHz = 2, frequency_notSet } tdArincFreq;
typedef enum { configOk = 0x00, configIndexError = 0x01, configFreqError = 0x02, configDataTypeError = 0x03, configAmountError = 0x04, configDuplicateChannels = 0x05 } tdArincConfigurationStatus;
typedef enum { notActiveState = 0x00, activeState = 0x01, sameState = 0x03 } tdChannelState;

typedef struct
	{
	uint32_t activeTimeEnd;
	uint8_t enable;
	uint8_t active;
	tdChannelState state;
	} tdArincChannelStatus;

typedef struct
	{
	uint32_t data[ HOLT_CORE_LABELS_AMOUNT ];
	uint32_t okTimeEnd;
	tdArincChannelStatus status;
	tdArincFreq freq;
	} tdArincChannel;

typedef struct
	{
	uint8_t high;
	uint8_t low;
	} tdArincReceiverFaild;

typedef struct
	{
	uint32_t activeChannelsAmount;
	tdArincChannel channel[ HOLT_CORE_CHANNELS_AMOUNT ];
	uint8_t reversedLabels[ HOLT_CORE_LABELS_AMOUNT ];
	uint8_t enabledChannels;
	tdArincReceiverFaild receiverFailedFreq;
	} tdArinc;


/*===============================================================================================*/
/*===============================================================================================*/
void HOLT_CORE_init(tdSpiDevice *_pDevice, tdArinc *_pArinc);
tdArincConfigurationStatus HOLT_CORE_handleSlotTD(tdArinc *_pArinc, uint8_t *_pData, uint8_t _slotsAmount);
tdStatus HOLT_CORE_configurateChannel(tdSpiDevice *_pDevice, uint8_t _channel, uint16_t _configuration);
tdStatus HOLT_CORE_readWordsFormChannel(tdSpiDevice *_pDevice, tdArinc *_pArinc, uint8_t _channel, uint8_t _wordsToRead, uint8_t *_pBuffer, uint32_t *_pCurrentSize, uint32_t _maxSize);
tdStatus HOLT_CORE_handleFifoEvents(tdSpiDevice *_pDevice, tdArinc *_pArinc, uint8_t _events, uint8_t _wordsToRead, uint8_t *_pBuffer, uint32_t *_pCurrentSize, uint32_t _maxSize);
tdStatus HOLT_CORE_ReedAvailableData(tdSpiDevice *_pDevice, tdArinc *_pArinc, uint8_t *_pBuffer, uint32_t *_pCurrentSize, uint32_t _maxSize);
tdStatus HOLT_CORE_generateReversedLabels(tdArinc *_pArinc);
tdStatus HOLT_CORE_holtSelfTest(tdSpiDevice *_pDevice, tdArincReceiverFaild *_pFailStats);

extern tdSpiDevice HOLT3598;



/*===============================================================================================*/
/*===============================================================================================*/



#endif
