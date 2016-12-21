#ifndef __TIM_CORE_H__
#define __TIM_CORE_H__



/*===============================================================================================*/
/*===============================================================================================*/
// Comments



/*===============================================================================================*/
/*===============================================================================================*/
// Includes
#include "stm32f2xx_hal.h"
#include "stdint.h"
#include "bhd2settings.h"


/*===============================================================================================*/
/*===============================================================================================*/
// Defines



/*===============================================================================================*/
/*===============================================================================================*/
// Data types, structures, ect.



/*===============================================================================================*/
/*===============================================================================================*/
#if !defined( USB_HARDWARE_FS ) && defined( USB_HARDWARE_HS_MC_REFCLK )
void TIM_CORE_initTim2(void);
#endif
void TIM_CORE_initTim7(void);
void TIM_CORE_setCallback(TIM_HandleTypeDef *htim, void (*pCallback)(void));
void TIM_CORE_getCallback(TIM_HandleTypeDef *htim, void (**_pCallback)() );


void MX_TIM7_Init(void);
void TIM6_Config(void);
void MX_TIM3_Init(void);


extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim7;
extern TIM_HandleTypeDef htim;
/*===============================================================================================*/
/*===============================================================================================*/



#endif