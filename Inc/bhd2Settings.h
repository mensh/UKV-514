#ifndef __BHD2_SETTINGS_H_
#define __BHD2_SETTINGS_H_



/*===============================================================================================*/
/*===============================================================================================*/
// Comments



/*===============================================================================================*/
/*===============================================================================================*/
// Includes



/*===============================================================================================*/
/*===============================================================================================*/
// Defines

/* DEBUG */
//#define DEBUG_PRINT

#define BHD2_HARDWARE_NEW 0

/* USB HARDWARE AND MODE */
#if BHD2_HARDWARE_NEW == 0
	#define USB_HARDWARE_FS
#else
	#define USB_HARDWARE_HS_VBUS_DETECT_BY_PIN
	#define USB_HARDWARE_HS_MC_REFCLK
	#define USB_HARDWARE_HS_MC_REFCLK_24_MHz
	//#define USB_HARDWARE_HS_MC_REFCLK_26_MHz
#endif

/* BHD2 Modefication and trigger to detect active channel(s) */
//#define BHD2_ACTIVE_CHANNEL_COUNTER_HANDLER
//#define BHD2_ACTIVE_CHANNEL_148FM
//#define BHD2_ACTIVE_CHANNEL_450S

#ifdef BHD2_ACTIVE_CHANNEL_450S
	#define BHD2_A_C_450S_TRIGGER_CHANNEL_1 0
	#define BHD2_A_C_450S_TRIGGER_CHANNEL_2 1
	#define BHD2_A_C_450S_TRIGGER_ON_1_LABEL 0x08
	#define BHD2_A_C_450S_TRIGGER_ON_1_BIT (1 << 13)
	#define BHD2_A_C_450S_TRIGGER_ON_2_LABEL 0x08
	#define BHD2_A_C_450S_TRIGGER_ON_2_BIT (1 << 14)
	#define BHD2_A_C_450S_TRIGGER_OFF_1_LABEL 0x0D
	#define BHD2_A_C_450S_TRIGGER_OFF_1_BIT (1 << 14)
	#define BHD2_A_C_450S_TRIGGER_OFF_2_LABEL 0xE4
	#define BHD2_A_C_450S_TRIGGER_OFF_2_VALUE (0x0533)

	#define BHD2_ACTIVE_CHANNEL_450S_TRIGGER_SIGNAL_STATE(_data, _channel, _label, _bit) ( (_data->channel[ _channel ].data[ _label ] & _bit) ? 1 : 0 )
	#define BHD2_ACTIVE_CHANNEL_450S_TRIGGER_SIGNAL_VALUE(_data, _channel, _label) ( (_data->channel[ _channel ].data[ _label ] >> 13) & 0xFFFF )
#endif

/* other */
#define BHD2_SYNCHRONIZE_ARINC_ACTIVE_CHANNELS
#define BHD2_ARINC_CHANNEL_OK_TIMEOUT_MS 2000
#define BHD2_ARINC_CHANNEL_ACTIVE_TIMEOUT_MS 3000
//#define BHD2_ARINC_CORE_HOLT_GET_STATUS_VIA_SPI
//#define BHD2_ARINC_CORE_HOLT_ENABLE_SELFTEST



/*===============================================================================================*/
/*===============================================================================================*/
// Data types, structures, ect.



/*===============================================================================================*/
/*===============================================================================================*/



/*===============================================================================================*/
/*===============================================================================================*/



#endif
