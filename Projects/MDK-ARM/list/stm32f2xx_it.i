#line 1 "..\\..\\Src\\stm32f2xx_it.c"
































 
 
#line 1 "..\\..\\Drivers\\STM32F2xx_HAL_Driver\\Inc\\stm32f2xx_hal.h"




































  

 







 
#line 1 "..\\..\\Inc\\stm32f2xx_hal_conf.h"
































 

 







 
 

 


 































#line 87 "..\\..\\Inc\\stm32f2xx_hal_conf.h"

 




 












 








 





 

 


 






 



 
 

 

 

 
#line 151 "..\\..\\Inc\\stm32f2xx_hal_conf.h"

 





 

 

 

 





 




#line 185 "..\\..\\Inc\\stm32f2xx_hal_conf.h"





 















 


 

#line 1 "..\\..\\Drivers\\STM32F2xx_HAL_Driver\\Inc\\stm32f2xx_hal_rcc.h"



































 

 







 
#line 1 "..\\..\\Drivers\\STM32F2xx_HAL_Driver\\Inc\\stm32f2xx_hal_def.h"




































 

 







 
#line 1 "..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32F2xx\\Include\\stm32f2xx.h"













































 



 



 
    






   


 



 

#line 79 "..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32F2xx\\Include\\stm32f2xx.h"
   


 
#line 91 "..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32F2xx\\Include\\stm32f2xx.h"



 
#line 103 "..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32F2xx\\Include\\stm32f2xx.h"
                                             


 



 

#line 1 "..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32F2xx\\Include\\stm32f215xx.h"








































 



 



 
    






  



 



 







 
   


 




 
typedef enum
{
 
  NonMaskableInt_IRQn         = -14,     
  MemoryManagement_IRQn       = -12,     
  BusFault_IRQn               = -11,     
  UsageFault_IRQn             = -10,     
  SVCall_IRQn                 = -5,      
  DebugMonitor_IRQn           = -4,      
  PendSV_IRQn                 = -2,      
  SysTick_IRQn                = -1,      
 
  WWDG_IRQn                   = 0,       
  PVD_IRQn                    = 1,       
  TAMP_STAMP_IRQn             = 2,       
  RTC_WKUP_IRQn               = 3,       
  FLASH_IRQn                  = 4,       
  RCC_IRQn                    = 5,       
  EXTI0_IRQn                  = 6,       
  EXTI1_IRQn                  = 7,       
  EXTI2_IRQn                  = 8,       
  EXTI3_IRQn                  = 9,       
  EXTI4_IRQn                  = 10,      
  DMA1_Stream0_IRQn           = 11,      
  DMA1_Stream1_IRQn           = 12,      
  DMA1_Stream2_IRQn           = 13,      
  DMA1_Stream3_IRQn           = 14,      
  DMA1_Stream4_IRQn           = 15,      
  DMA1_Stream5_IRQn           = 16,      
  DMA1_Stream6_IRQn           = 17,      
  ADC_IRQn                    = 18,      
  CAN1_TX_IRQn                = 19,      
  CAN1_RX0_IRQn               = 20,      
  CAN1_RX1_IRQn               = 21,      
  CAN1_SCE_IRQn               = 22,      
  EXTI9_5_IRQn                = 23,      
  TIM1_BRK_TIM9_IRQn          = 24,      
  TIM1_UP_TIM10_IRQn          = 25,      
  TIM1_TRG_COM_TIM11_IRQn     = 26,      
  TIM1_CC_IRQn                = 27,      
  TIM2_IRQn                   = 28,      
  TIM3_IRQn                   = 29,      
  TIM4_IRQn                   = 30,      
  I2C1_EV_IRQn                = 31,      
  I2C1_ER_IRQn                = 32,      
  I2C2_EV_IRQn                = 33,      
  I2C2_ER_IRQn                = 34,        
  SPI1_IRQn                   = 35,      
  SPI2_IRQn                   = 36,      
  USART1_IRQn                 = 37,      
  USART2_IRQn                 = 38,      
  USART3_IRQn                 = 39,      
  EXTI15_10_IRQn              = 40,      
  RTC_Alarm_IRQn              = 41,      
  OTG_FS_WKUP_IRQn            = 42,          
  TIM8_BRK_TIM12_IRQn         = 43,      
  TIM8_UP_TIM13_IRQn          = 44,      
  TIM8_TRG_COM_TIM14_IRQn     = 45,      
  TIM8_CC_IRQn                = 46,      
  DMA1_Stream7_IRQn           = 47,      
  FSMC_IRQn                   = 48,      
  SDIO_IRQn                   = 49,      
  TIM5_IRQn                   = 50,      
  SPI3_IRQn                   = 51,      
  UART4_IRQn                  = 52,      
  UART5_IRQn                  = 53,      
  TIM6_DAC_IRQn               = 54,      
  TIM7_IRQn                   = 55,      
  DMA2_Stream0_IRQn           = 56,      
  DMA2_Stream1_IRQn           = 57,      
  DMA2_Stream2_IRQn           = 58,      
  DMA2_Stream3_IRQn           = 59,      
  DMA2_Stream4_IRQn           = 60,      
  CAN2_TX_IRQn                = 63,      
  CAN2_RX0_IRQn               = 64,      
  CAN2_RX1_IRQn               = 65,      
  CAN2_SCE_IRQn               = 66,      
  OTG_FS_IRQn                 = 67,      
  DMA2_Stream5_IRQn           = 68,      
  DMA2_Stream6_IRQn           = 69,      
  DMA2_Stream7_IRQn           = 70,      
  USART6_IRQn                 = 71,      
  I2C3_EV_IRQn                = 72,      
  I2C3_ER_IRQn                = 73,      
  OTG_HS_EP1_OUT_IRQn         = 74,      
  OTG_HS_EP1_IN_IRQn          = 75,      
  OTG_HS_WKUP_IRQn            = 76,      
  OTG_HS_IRQn                 = 77,      
  CRYP_IRQn                   = 79,      
  HASH_RNG_IRQn               = 80       
} IRQn_Type;



 

#line 1 "..\\..\\Drivers\\CMSIS\\Include\\core_cm3.h"
 







 

























 
























 




 


 

 













#line 104 "..\\..\\Drivers\\CMSIS\\Include\\core_cm3.h"


 







#line 134 "..\\..\\Drivers\\CMSIS\\Include\\core_cm3.h"

#line 1 "C:\\Keil_v5\\ARM\\ARMCC\\Bin\\..\\include\\stdint.h"
 
 





 









     
#line 27 "C:\\Keil_v5\\ARM\\ARMCC\\Bin\\..\\include\\stdint.h"
     











#line 46 "C:\\Keil_v5\\ARM\\ARMCC\\Bin\\..\\include\\stdint.h"





 

     

     
typedef   signed          char int8_t;
typedef   signed short     int int16_t;
typedef   signed           int int32_t;
typedef   signed       __int64 int64_t;

     
typedef unsigned          char uint8_t;
typedef unsigned short     int uint16_t;
typedef unsigned           int uint32_t;
typedef unsigned       __int64 uint64_t;

     

     
     
typedef   signed          char int_least8_t;
typedef   signed short     int int_least16_t;
typedef   signed           int int_least32_t;
typedef   signed       __int64 int_least64_t;

     
typedef unsigned          char uint_least8_t;
typedef unsigned short     int uint_least16_t;
typedef unsigned           int uint_least32_t;
typedef unsigned       __int64 uint_least64_t;

     

     
typedef   signed           int int_fast8_t;
typedef   signed           int int_fast16_t;
typedef   signed           int int_fast32_t;
typedef   signed       __int64 int_fast64_t;

     
typedef unsigned           int uint_fast8_t;
typedef unsigned           int uint_fast16_t;
typedef unsigned           int uint_fast32_t;
typedef unsigned       __int64 uint_fast64_t;

     




typedef   signed           int intptr_t;
typedef unsigned           int uintptr_t;


     
typedef   signed     long long intmax_t;
typedef unsigned     long long uintmax_t;




     

     





     





     





     

     





     





     





     

     





     





     





     

     






     






     






     

     


     


     


     

     
#line 216 "C:\\Keil_v5\\ARM\\ARMCC\\Bin\\..\\include\\stdint.h"

     



     






     
    
 



#line 241 "C:\\Keil_v5\\ARM\\ARMCC\\Bin\\..\\include\\stdint.h"

     







     










     











#line 305 "C:\\Keil_v5\\ARM\\ARMCC\\Bin\\..\\include\\stdint.h"






 
#line 136 "..\\..\\Drivers\\CMSIS\\Include\\core_cm3.h"
#line 1 "..\\..\\Drivers\\CMSIS\\Include\\core_cmInstr.h"
 







 

























 






 



 


 









 







 







 






 








 







 







 









 









 

__attribute__((section(".rev16_text"))) static __inline __asm uint32_t __REV16(uint32_t value)
{
  rev16 r0, r0
  bx lr
}








 

__attribute__((section(".revsh_text"))) static __inline __asm int32_t __REVSH(int32_t value)
{
  revsh r0, r0
  bx lr
}










 










 











 









 









 









 











 











 











 







 










 










 









 






#line 685 "..\\..\\Drivers\\CMSIS\\Include\\core_cmInstr.h"

   

#line 137 "..\\..\\Drivers\\CMSIS\\Include\\core_cm3.h"
#line 1 "..\\..\\Drivers\\CMSIS\\Include\\core_cmFunc.h"
 







 

























 






 



 


 





 
 






 
static __inline uint32_t __get_CONTROL(void)
{
  register uint32_t __regControl         __asm("control");
  return(__regControl);
}







 
static __inline void __set_CONTROL(uint32_t control)
{
  register uint32_t __regControl         __asm("control");
  __regControl = control;
}







 
static __inline uint32_t __get_IPSR(void)
{
  register uint32_t __regIPSR          __asm("ipsr");
  return(__regIPSR);
}







 
static __inline uint32_t __get_APSR(void)
{
  register uint32_t __regAPSR          __asm("apsr");
  return(__regAPSR);
}







 
static __inline uint32_t __get_xPSR(void)
{
  register uint32_t __regXPSR          __asm("xpsr");
  return(__regXPSR);
}







 
static __inline uint32_t __get_PSP(void)
{
  register uint32_t __regProcessStackPointer  __asm("psp");
  return(__regProcessStackPointer);
}







 
static __inline void __set_PSP(uint32_t topOfProcStack)
{
  register uint32_t __regProcessStackPointer  __asm("psp");
  __regProcessStackPointer = topOfProcStack;
}







 
static __inline uint32_t __get_MSP(void)
{
  register uint32_t __regMainStackPointer     __asm("msp");
  return(__regMainStackPointer);
}







 
static __inline void __set_MSP(uint32_t topOfMainStack)
{
  register uint32_t __regMainStackPointer     __asm("msp");
  __regMainStackPointer = topOfMainStack;
}







 
static __inline uint32_t __get_PRIMASK(void)
{
  register uint32_t __regPriMask         __asm("primask");
  return(__regPriMask);
}







 
static __inline void __set_PRIMASK(uint32_t priMask)
{
  register uint32_t __regPriMask         __asm("primask");
  __regPriMask = (priMask);
}








 







 








 
static __inline uint32_t  __get_BASEPRI(void)
{
  register uint32_t __regBasePri         __asm("basepri");
  return(__regBasePri);
}







 
static __inline void __set_BASEPRI(uint32_t basePri)
{
  register uint32_t __regBasePri         __asm("basepri");
  __regBasePri = (basePri & 0xff);
}







 
static __inline uint32_t __get_FAULTMASK(void)
{
  register uint32_t __regFaultMask       __asm("faultmask");
  return(__regFaultMask);
}







 
static __inline void __set_FAULTMASK(uint32_t faultMask)
{
  register uint32_t __regFaultMask       __asm("faultmask");
  __regFaultMask = (faultMask & (uint32_t)1);
}




#line 307 "..\\..\\Drivers\\CMSIS\\Include\\core_cmFunc.h"


#line 632 "..\\..\\Drivers\\CMSIS\\Include\\core_cmFunc.h"

 


#line 138 "..\\..\\Drivers\\CMSIS\\Include\\core_cm3.h"








 
#line 168 "..\\..\\Drivers\\CMSIS\\Include\\core_cm3.h"

 






 
#line 184 "..\\..\\Drivers\\CMSIS\\Include\\core_cm3.h"

 












 


 





 


 
typedef union
{
  struct
  {

    uint32_t _reserved0:27;               





    uint32_t Q:1;                         
    uint32_t V:1;                         
    uint32_t C:1;                         
    uint32_t Z:1;                         
    uint32_t N:1;                         
  } b;                                    
  uint32_t w;                             
} APSR_Type;



 
typedef union
{
  struct
  {
    uint32_t ISR:9;                       
    uint32_t _reserved0:23;               
  } b;                                    
  uint32_t w;                             
} IPSR_Type;



 
typedef union
{
  struct
  {
    uint32_t ISR:9;                       

    uint32_t _reserved0:15;               





    uint32_t T:1;                         
    uint32_t IT:2;                        
    uint32_t Q:1;                         
    uint32_t V:1;                         
    uint32_t C:1;                         
    uint32_t Z:1;                         
    uint32_t N:1;                         
  } b;                                    
  uint32_t w;                             
} xPSR_Type;



 
typedef union
{
  struct
  {
    uint32_t nPRIV:1;                     
    uint32_t SPSEL:1;                     
    uint32_t FPCA:1;                      
    uint32_t _reserved0:29;               
  } b;                                    
  uint32_t w;                             
} CONTROL_Type;

 






 


 
typedef struct
{
  volatile uint32_t ISER[8];                  
       uint32_t RESERVED0[24];
  volatile uint32_t ICER[8];                  
       uint32_t RSERVED1[24];
  volatile uint32_t ISPR[8];                  
       uint32_t RESERVED2[24];
  volatile uint32_t ICPR[8];                  
       uint32_t RESERVED3[24];
  volatile uint32_t IABR[8];                  
       uint32_t RESERVED4[56];
  volatile uint8_t  IP[240];                  
       uint32_t RESERVED5[644];
  volatile  uint32_t STIR;                     
}  NVIC_Type;

 



 






 


 
typedef struct
{
  volatile const  uint32_t CPUID;                    
  volatile uint32_t ICSR;                     
  volatile uint32_t VTOR;                     
  volatile uint32_t AIRCR;                    
  volatile uint32_t SCR;                      
  volatile uint32_t CCR;                      
  volatile uint8_t  SHP[12];                  
  volatile uint32_t SHCSR;                    
  volatile uint32_t CFSR;                     
  volatile uint32_t HFSR;                     
  volatile uint32_t DFSR;                     
  volatile uint32_t MMFAR;                    
  volatile uint32_t BFAR;                     
  volatile uint32_t AFSR;                     
  volatile const  uint32_t PFR[2];                   
  volatile const  uint32_t DFR;                      
  volatile const  uint32_t ADR;                      
  volatile const  uint32_t MMFR[4];                  
  volatile const  uint32_t ISAR[5];                  
       uint32_t RESERVED0[5];
  volatile uint32_t CPACR;                    
} SCB_Type;

 















 






























 




#line 411 "..\\..\\Drivers\\CMSIS\\Include\\core_cm3.h"

 





















 









 


















 










































 









 









 















 






 


 
typedef struct
{
       uint32_t RESERVED0[1];
  volatile const  uint32_t ICTR;                     

  volatile uint32_t ACTLR;                    



} SCnSCB_Type;

 



 










 






 


 
typedef struct
{
  volatile uint32_t CTRL;                     
  volatile uint32_t LOAD;                     
  volatile uint32_t VAL;                      
  volatile const  uint32_t CALIB;                    
} SysTick_Type;

 












 



 



 









 






 


 
typedef struct
{
  volatile  union
  {
    volatile  uint8_t    u8;                   
    volatile  uint16_t   u16;                  
    volatile  uint32_t   u32;                  
  }  PORT [32];                           
       uint32_t RESERVED0[864];
  volatile uint32_t TER;                      
       uint32_t RESERVED1[15];
  volatile uint32_t TPR;                      
       uint32_t RESERVED2[15];
  volatile uint32_t TCR;                      
       uint32_t RESERVED3[29];
  volatile  uint32_t IWR;                      
  volatile const  uint32_t IRR;                      
  volatile uint32_t IMCR;                     
       uint32_t RESERVED4[43];
  volatile  uint32_t LAR;                      
  volatile const  uint32_t LSR;                      
       uint32_t RESERVED5[6];
  volatile const  uint32_t PID4;                     
  volatile const  uint32_t PID5;                     
  volatile const  uint32_t PID6;                     
  volatile const  uint32_t PID7;                     
  volatile const  uint32_t PID0;                     
  volatile const  uint32_t PID1;                     
  volatile const  uint32_t PID2;                     
  volatile const  uint32_t PID3;                     
  volatile const  uint32_t CID0;                     
  volatile const  uint32_t CID1;                     
  volatile const  uint32_t CID2;                     
  volatile const  uint32_t CID3;                     
} ITM_Type;

 



 



























 



 



 



 









   






 


 
typedef struct
{
  volatile uint32_t CTRL;                     
  volatile uint32_t CYCCNT;                   
  volatile uint32_t CPICNT;                   
  volatile uint32_t EXCCNT;                   
  volatile uint32_t SLEEPCNT;                 
  volatile uint32_t LSUCNT;                   
  volatile uint32_t FOLDCNT;                  
  volatile const  uint32_t PCSR;                     
  volatile uint32_t COMP0;                    
  volatile uint32_t MASK0;                    
  volatile uint32_t FUNCTION0;                
       uint32_t RESERVED0[1];
  volatile uint32_t COMP1;                    
  volatile uint32_t MASK1;                    
  volatile uint32_t FUNCTION1;                
       uint32_t RESERVED1[1];
  volatile uint32_t COMP2;                    
  volatile uint32_t MASK2;                    
  volatile uint32_t FUNCTION2;                
       uint32_t RESERVED2[1];
  volatile uint32_t COMP3;                    
  volatile uint32_t MASK3;                    
  volatile uint32_t FUNCTION3;                
} DWT_Type;

 






















































 



 



 



 



 



 



 



























   






 


 
typedef struct
{
  volatile uint32_t SSPSR;                    
  volatile uint32_t CSPSR;                    
       uint32_t RESERVED0[2];
  volatile uint32_t ACPR;                     
       uint32_t RESERVED1[55];
  volatile uint32_t SPPR;                     
       uint32_t RESERVED2[131];
  volatile const  uint32_t FFSR;                     
  volatile uint32_t FFCR;                     
  volatile const  uint32_t FSCR;                     
       uint32_t RESERVED3[759];
  volatile const  uint32_t TRIGGER;                  
  volatile const  uint32_t FIFO0;                    
  volatile const  uint32_t ITATBCTR2;                
       uint32_t RESERVED4[1];
  volatile const  uint32_t ITATBCTR0;                
  volatile const  uint32_t FIFO1;                    
  volatile uint32_t ITCTRL;                   
       uint32_t RESERVED5[39];
  volatile uint32_t CLAIMSET;                 
  volatile uint32_t CLAIMCLR;                 
       uint32_t RESERVED7[8];
  volatile const  uint32_t DEVID;                    
  volatile const  uint32_t DEVTYPE;                  
} TPI_Type;

 



 



 












 






 



 





















 



 





















 



 



 


















 






   







 


 
typedef struct
{
  volatile const  uint32_t TYPE;                     
  volatile uint32_t CTRL;                     
  volatile uint32_t RNR;                      
  volatile uint32_t RBAR;                     
  volatile uint32_t RASR;                     
  volatile uint32_t RBAR_A1;                  
  volatile uint32_t RASR_A1;                  
  volatile uint32_t RBAR_A2;                  
  volatile uint32_t RASR_A2;                  
  volatile uint32_t RBAR_A3;                  
  volatile uint32_t RASR_A3;                  
} MPU_Type;

 









 









 



 









 






























 







 


 
typedef struct
{
  volatile uint32_t DHCSR;                    
  volatile  uint32_t DCRSR;                    
  volatile uint32_t DCRDR;                    
  volatile uint32_t DEMCR;                    
} CoreDebug_Type;

 




































 






 







































 






 

 
#line 1242 "..\\..\\Drivers\\CMSIS\\Include\\core_cm3.h"

#line 1251 "..\\..\\Drivers\\CMSIS\\Include\\core_cm3.h"






 










 

 



 




 










 
static __inline void NVIC_SetPriorityGrouping(uint32_t PriorityGroup)
{
  uint32_t reg_value;
  uint32_t PriorityGroupTmp = (PriorityGroup & (uint32_t)0x07);                

  reg_value  =  ((SCB_Type *) ((0xE000E000UL) + 0x0D00UL) )->AIRCR;                                                    
  reg_value &= ~((0xFFFFUL << 16) | (7UL << 8));              
  reg_value  =  (reg_value                                 |
                ((uint32_t)0x5FA << 16) |
                (PriorityGroupTmp << 8));                                      
  ((SCB_Type *) ((0xE000E000UL) + 0x0D00UL) )->AIRCR =  reg_value;
}







 
static __inline uint32_t NVIC_GetPriorityGrouping(void)
{
  return ((((SCB_Type *) ((0xE000E000UL) + 0x0D00UL) )->AIRCR & (7UL << 8)) >> 8);    
}







 
static __inline void NVIC_EnableIRQ(IRQn_Type IRQn)
{
  ((NVIC_Type *) ((0xE000E000UL) + 0x0100UL) )->ISER[((uint32_t)(IRQn) >> 5)] = (1 << ((uint32_t)(IRQn) & 0x1F));  
}







 
static __inline void NVIC_DisableIRQ(IRQn_Type IRQn)
{
  ((NVIC_Type *) ((0xE000E000UL) + 0x0100UL) )->ICER[((uint32_t)(IRQn) >> 5)] = (1 << ((uint32_t)(IRQn) & 0x1F));  
}











 
static __inline uint32_t NVIC_GetPendingIRQ(IRQn_Type IRQn)
{
  return((uint32_t) ((((NVIC_Type *) ((0xE000E000UL) + 0x0100UL) )->ISPR[(uint32_t)(IRQn) >> 5] & (1 << ((uint32_t)(IRQn) & 0x1F)))?1:0));  
}







 
static __inline void NVIC_SetPendingIRQ(IRQn_Type IRQn)
{
  ((NVIC_Type *) ((0xE000E000UL) + 0x0100UL) )->ISPR[((uint32_t)(IRQn) >> 5)] = (1 << ((uint32_t)(IRQn) & 0x1F));  
}







 
static __inline void NVIC_ClearPendingIRQ(IRQn_Type IRQn)
{
  ((NVIC_Type *) ((0xE000E000UL) + 0x0100UL) )->ICPR[((uint32_t)(IRQn) >> 5)] = (1 << ((uint32_t)(IRQn) & 0x1F));  
}










 
static __inline uint32_t NVIC_GetActive(IRQn_Type IRQn)
{
  return((uint32_t)((((NVIC_Type *) ((0xE000E000UL) + 0x0100UL) )->IABR[(uint32_t)(IRQn) >> 5] & (1 << ((uint32_t)(IRQn) & 0x1F)))?1:0));  
}










 
static __inline void NVIC_SetPriority(IRQn_Type IRQn, uint32_t priority)
{
  if(IRQn < 0) {
    ((SCB_Type *) ((0xE000E000UL) + 0x0D00UL) )->SHP[((uint32_t)(IRQn) & 0xF)-4] = ((priority << (8 - 4)) & 0xff); }  
  else {
    ((NVIC_Type *) ((0xE000E000UL) + 0x0100UL) )->IP[(uint32_t)(IRQn)] = ((priority << (8 - 4)) & 0xff);    }         
}












 
static __inline uint32_t NVIC_GetPriority(IRQn_Type IRQn)
{

  if(IRQn < 0) {
    return((uint32_t)(((SCB_Type *) ((0xE000E000UL) + 0x0D00UL) )->SHP[((uint32_t)(IRQn) & 0xF)-4] >> (8 - 4)));  }  
  else {
    return((uint32_t)(((NVIC_Type *) ((0xE000E000UL) + 0x0100UL) )->IP[(uint32_t)(IRQn)]           >> (8 - 4)));  }  
}













 
static __inline uint32_t NVIC_EncodePriority (uint32_t PriorityGroup, uint32_t PreemptPriority, uint32_t SubPriority)
{
  uint32_t PriorityGroupTmp = (PriorityGroup & 0x07);           
  uint32_t PreemptPriorityBits;
  uint32_t SubPriorityBits;

  PreemptPriorityBits = ((7 - PriorityGroupTmp) > 4) ? 4 : 7 - PriorityGroupTmp;
  SubPriorityBits     = ((PriorityGroupTmp + 4) < 7) ? 0 : PriorityGroupTmp - 7 + 4;

  return (
           ((PreemptPriority & ((1 << (PreemptPriorityBits)) - 1)) << SubPriorityBits) |
           ((SubPriority     & ((1 << (SubPriorityBits    )) - 1)))
         );
}













 
static __inline void NVIC_DecodePriority (uint32_t Priority, uint32_t PriorityGroup, uint32_t* pPreemptPriority, uint32_t* pSubPriority)
{
  uint32_t PriorityGroupTmp = (PriorityGroup & 0x07);           
  uint32_t PreemptPriorityBits;
  uint32_t SubPriorityBits;

  PreemptPriorityBits = ((7 - PriorityGroupTmp) > 4) ? 4 : 7 - PriorityGroupTmp;
  SubPriorityBits     = ((PriorityGroupTmp + 4) < 7) ? 0 : PriorityGroupTmp - 7 + 4;

  *pPreemptPriority = (Priority >> SubPriorityBits) & ((1 << (PreemptPriorityBits)) - 1);
  *pSubPriority     = (Priority                   ) & ((1 << (SubPriorityBits    )) - 1);
}





 
static __inline void NVIC_SystemReset(void)
{
  __dsb(0xF);                                                     
 
  ((SCB_Type *) ((0xE000E000UL) + 0x0D00UL) )->AIRCR  = ((0x5FA << 16)      |
                 (((SCB_Type *) ((0xE000E000UL) + 0x0D00UL) )->AIRCR & (7UL << 8)) |
                 (1UL << 2));                    
  __dsb(0xF);                                                      
  while(1);                                                     
}

 



 




 

















 
static __inline uint32_t SysTick_Config(uint32_t ticks)
{
  if ((ticks - 1) > (0xFFFFFFUL << 0))  return (1);       

  ((SysTick_Type *) ((0xE000E000UL) + 0x0010UL) )->LOAD  = ticks - 1;                                   
  NVIC_SetPriority (SysTick_IRQn, (1<<4) - 1);   
  ((SysTick_Type *) ((0xE000E000UL) + 0x0010UL) )->VAL   = 0;                                           
  ((SysTick_Type *) ((0xE000E000UL) + 0x0010UL) )->CTRL  = (1UL << 2) |
                   (1UL << 1)   |
                   (1UL << 0);                     
  return (0);                                                   
}



 



 




 

extern volatile int32_t ITM_RxBuffer;                     












 
static __inline uint32_t ITM_SendChar (uint32_t ch)
{
  if ((((ITM_Type *) (0xE0000000UL) )->TCR & (1UL << 0))                  &&       
      (((ITM_Type *) (0xE0000000UL) )->TER & (1UL << 0)        )                    )      
  {
    while (((ITM_Type *) (0xE0000000UL) )->PORT[0].u32 == 0);
    ((ITM_Type *) (0xE0000000UL) )->PORT[0].u8 = (uint8_t) ch;
  }
  return (ch);
}








 
static __inline int32_t ITM_ReceiveChar (void) {
  int32_t ch = -1;                            

  if (ITM_RxBuffer != 0x5AA55AA5) {
    ch = ITM_RxBuffer;
    ITM_RxBuffer = 0x5AA55AA5;        
  }

  return (ch);
}








 
static __inline int32_t ITM_CheckChar (void) {

  if (ITM_RxBuffer == 0x5AA55AA5) {
    return (0);                                  
  } else {
    return (1);                                  
  }
}

 





#line 180 "..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32F2xx\\Include\\stm32f215xx.h"
#line 1 "..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32F2xx\\Include\\system_stm32f2xx.h"



































  



 



   
  


 









 



 




 
  






 
extern uint32_t SystemCoreClock;           




 



 



 



 



 



 
  
extern void SystemInit(void);
extern void SystemCoreClockUpdate(void);


 









 
  


   
 
#line 181 "..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32F2xx\\Include\\stm32f215xx.h"
#line 182 "..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32F2xx\\Include\\stm32f215xx.h"



    



 

typedef struct
{
  volatile uint32_t SR;      
  volatile uint32_t CR1;           
  volatile uint32_t CR2;     
  volatile uint32_t SMPR1;   
  volatile uint32_t SMPR2;   
  volatile uint32_t JOFR1;   
  volatile uint32_t JOFR2;   
  volatile uint32_t JOFR3;   
  volatile uint32_t JOFR4;   
  volatile uint32_t HTR;     
  volatile uint32_t LTR;     
  volatile uint32_t SQR1;    
  volatile uint32_t SQR2;    
  volatile uint32_t SQR3;    
  volatile uint32_t JSQR;    
  volatile uint32_t JDR1;    
  volatile uint32_t JDR2;    
  volatile uint32_t JDR3;    
  volatile uint32_t JDR4;    
  volatile uint32_t DR;      
} ADC_TypeDef;

typedef struct
{
  volatile uint32_t CSR;     
  volatile uint32_t CCR;     
  volatile uint32_t CDR;    
 
} ADC_Common_TypeDef;




 

typedef struct
{
  volatile uint32_t TIR;   
  volatile uint32_t TDTR;  
  volatile uint32_t TDLR;  
  volatile uint32_t TDHR;  
} CAN_TxMailBox_TypeDef;



 
  
typedef struct
{
  volatile uint32_t RIR;   
  volatile uint32_t RDTR;  
  volatile uint32_t RDLR;  
  volatile uint32_t RDHR;  
} CAN_FIFOMailBox_TypeDef;



 
  
typedef struct
{
  volatile uint32_t FR1;  
  volatile uint32_t FR2;  
} CAN_FilterRegister_TypeDef;



 
  
typedef struct
{
  volatile uint32_t              MCR;                  
  volatile uint32_t              MSR;                  
  volatile uint32_t              TSR;                  
  volatile uint32_t              RF0R;                 
  volatile uint32_t              RF1R;                 
  volatile uint32_t              IER;                  
  volatile uint32_t              ESR;                  
  volatile uint32_t              BTR;                  
  uint32_t                   RESERVED0[88];        
  CAN_TxMailBox_TypeDef      sTxMailBox[3];        
  CAN_FIFOMailBox_TypeDef    sFIFOMailBox[2];      
  uint32_t                   RESERVED1[12];        
  volatile uint32_t              FMR;                  
  volatile uint32_t              FM1R;                 
  uint32_t                   RESERVED2;            
  volatile uint32_t              FS1R;                 
  uint32_t                   RESERVED3;            
  volatile uint32_t              FFA1R;                
  uint32_t                   RESERVED4;            
  volatile uint32_t              FA1R;                 
  uint32_t                   RESERVED5[8];          
  CAN_FilterRegister_TypeDef sFilterRegister[28];  
} CAN_TypeDef;



 

typedef struct
{
  volatile uint32_t DR;          
  volatile uint8_t  IDR;         
  uint8_t       RESERVED0;   
  uint16_t      RESERVED1;   
  volatile uint32_t CR;          
} CRC_TypeDef;



 

typedef struct
{
  volatile uint32_t CR;        
  volatile uint32_t SWTRIGR;   
  volatile uint32_t DHR12R1;   
  volatile uint32_t DHR12L1;   
  volatile uint32_t DHR8R1;    
  volatile uint32_t DHR12R2;   
  volatile uint32_t DHR12L2;   
  volatile uint32_t DHR8R2;    
  volatile uint32_t DHR12RD;   
  volatile uint32_t DHR12LD;   
  volatile uint32_t DHR8RD;    
  volatile uint32_t DOR1;      
  volatile uint32_t DOR2;      
  volatile uint32_t SR;        
} DAC_TypeDef;



 

typedef struct
{
  volatile uint32_t IDCODE;   
  volatile uint32_t CR;       
  volatile uint32_t APB1FZ;   
  volatile uint32_t APB2FZ;   
}DBGMCU_TypeDef;




 

typedef struct
{
  volatile uint32_t CR;      
  volatile uint32_t NDTR;    
  volatile uint32_t PAR;     
  volatile uint32_t M0AR;    
  volatile uint32_t M1AR;    
  volatile uint32_t FCR;     
} DMA_Stream_TypeDef;

typedef struct
{
  volatile uint32_t LISR;    
  volatile uint32_t HISR;    
  volatile uint32_t LIFCR;   
  volatile uint32_t HIFCR;   
} DMA_TypeDef;




 

typedef struct
{
  volatile uint32_t IMR;     
  volatile uint32_t EMR;     
  volatile uint32_t RTSR;    
  volatile uint32_t FTSR;    
  volatile uint32_t SWIER;   
  volatile uint32_t PR;      
} EXTI_TypeDef;



 

typedef struct
{
  volatile uint32_t ACR;       
  volatile uint32_t KEYR;      
  volatile uint32_t OPTKEYR;   
  volatile uint32_t SR;        
  volatile uint32_t CR;        
  volatile uint32_t OPTCR;     
} FLASH_TypeDef;




 

typedef struct
{
  volatile uint32_t BTCR[8];        
} FSMC_Bank1_TypeDef; 



 
  
typedef struct
{
  volatile uint32_t BWTR[7];     
} FSMC_Bank1E_TypeDef;



 
  
typedef struct
{
  volatile uint32_t PCR2;        
  volatile uint32_t SR2;         
  volatile uint32_t PMEM2;       
  volatile uint32_t PATT2;       
  uint32_t      RESERVED0;   
  volatile uint32_t ECCR2;       
  uint32_t      RESERVED1;   
  uint32_t      RESERVED2;   
  volatile uint32_t PCR3;        
  volatile uint32_t SR3;         
  volatile uint32_t PMEM3;       
  volatile uint32_t PATT3;       
  uint32_t      RESERVED3;   
  volatile uint32_t ECCR3;       
} FSMC_Bank2_3_TypeDef;



 
  
typedef struct
{
  volatile uint32_t PCR4;        
  volatile uint32_t SR4;         
  volatile uint32_t PMEM4;       
  volatile uint32_t PATT4;       
  volatile uint32_t PIO4;        
} FSMC_Bank4_TypeDef; 




 

typedef struct
{
  volatile uint32_t MODER;     
  volatile uint32_t OTYPER;    
  volatile uint32_t OSPEEDR;   
  volatile uint32_t PUPDR;     
  volatile uint32_t IDR;       
  volatile uint32_t ODR;       
  volatile uint16_t BSRRL;     
  volatile uint16_t BSRRH;     
  volatile uint32_t LCKR;      
  volatile uint32_t AFR[2];    
} GPIO_TypeDef;



 
  
typedef struct
{
  volatile uint32_t MEMRMP;        
  volatile uint32_t PMC;           
  volatile uint32_t EXTICR[4];     
  uint32_t      RESERVED[2];    
  volatile uint32_t CMPCR;         
} SYSCFG_TypeDef;



 

typedef struct
{
  volatile uint32_t CR1;         
  volatile uint32_t CR2;         
  volatile uint32_t OAR1;        
  volatile uint32_t OAR2;        
  volatile uint32_t DR;          
  volatile uint32_t SR1;         
  volatile uint32_t SR2;         
  volatile uint32_t CCR;         
  volatile uint32_t TRISE;       
  volatile uint32_t FLTR;        
} I2C_TypeDef;



 

typedef struct
{
  volatile uint32_t KR;    
  volatile uint32_t PR;    
  volatile uint32_t RLR;   
  volatile uint32_t SR;    
} IWDG_TypeDef;



 

typedef struct
{
  volatile uint32_t CR;    
  volatile uint32_t CSR;   
} PWR_TypeDef;



 

typedef struct
{
  volatile uint32_t CR;             
  volatile uint32_t PLLCFGR;        
  volatile uint32_t CFGR;           
  volatile uint32_t CIR;            
  volatile uint32_t AHB1RSTR;       
  volatile uint32_t AHB2RSTR;       
  volatile uint32_t AHB3RSTR;       
  uint32_t      RESERVED0;      
  volatile uint32_t APB1RSTR;       
  volatile uint32_t APB2RSTR;       
  uint32_t      RESERVED1[2];   
  volatile uint32_t AHB1ENR;        
  volatile uint32_t AHB2ENR;        
  volatile uint32_t AHB3ENR;        
  uint32_t      RESERVED2;      
  volatile uint32_t APB1ENR;        
  volatile uint32_t APB2ENR;        
  uint32_t      RESERVED3[2];   
  volatile uint32_t AHB1LPENR;      
  volatile uint32_t AHB2LPENR;      
  volatile uint32_t AHB3LPENR;      
  uint32_t      RESERVED4;      
  volatile uint32_t APB1LPENR;      
  volatile uint32_t APB2LPENR;      
  uint32_t      RESERVED5[2];   
  volatile uint32_t BDCR;           
  volatile uint32_t CSR;            
  uint32_t      RESERVED6[2];   
  volatile uint32_t SSCGR;          
  volatile uint32_t PLLI2SCFGR;     

} RCC_TypeDef;



 

typedef struct
{
  volatile uint32_t TR;       
  volatile uint32_t DR;       
  volatile uint32_t CR;       
  volatile uint32_t ISR;      
  volatile uint32_t PRER;     
  volatile uint32_t WUTR;     
  volatile uint32_t CALIBR;   
  volatile uint32_t ALRMAR;   
  volatile uint32_t ALRMBR;   
  volatile uint32_t WPR;      
  uint32_t RESERVED1;     
  uint32_t RESERVED2;     
  volatile uint32_t TSTR;     
  volatile uint32_t TSDR;     
  uint32_t RESERVED3;     
  uint32_t RESERVED4;     
  volatile uint32_t TAFCR;    
  uint32_t RESERVED5;     
  uint32_t RESERVED6;     
  uint32_t RESERVED7;     
  volatile uint32_t BKP0R;    
  volatile uint32_t BKP1R;    
  volatile uint32_t BKP2R;    
  volatile uint32_t BKP3R;    
  volatile uint32_t BKP4R;    
  volatile uint32_t BKP5R;    
  volatile uint32_t BKP6R;    
  volatile uint32_t BKP7R;    
  volatile uint32_t BKP8R;    
  volatile uint32_t BKP9R;    
  volatile uint32_t BKP10R;   
  volatile uint32_t BKP11R;   
  volatile uint32_t BKP12R;   
  volatile uint32_t BKP13R;   
  volatile uint32_t BKP14R;   
  volatile uint32_t BKP15R;   
  volatile uint32_t BKP16R;   
  volatile uint32_t BKP17R;   
  volatile uint32_t BKP18R;   
  volatile uint32_t BKP19R;   
} RTC_TypeDef;




 

typedef struct
{
  volatile uint32_t POWER;           
  volatile uint32_t CLKCR;           
  volatile uint32_t ARG;             
  volatile uint32_t CMD;             
  volatile const uint32_t  RESPCMD;         
  volatile const uint32_t  RESP1;           
  volatile const uint32_t  RESP2;           
  volatile const uint32_t  RESP3;           
  volatile const uint32_t  RESP4;           
  volatile uint32_t DTIMER;          
  volatile uint32_t DLEN;            
  volatile uint32_t DCTRL;           
  volatile const uint32_t  DCOUNT;          
  volatile const uint32_t  STA;             
  volatile uint32_t ICR;             
  volatile uint32_t MASK;            
  uint32_t      RESERVED0[2];    
  volatile const uint32_t  FIFOCNT;         
  uint32_t      RESERVED1[13];   
  volatile uint32_t FIFO;            
} SDIO_TypeDef;



 

typedef struct
{
  volatile uint32_t CR1;         
  volatile uint32_t CR2;         
  volatile uint32_t SR;          
  volatile uint32_t DR;          
  volatile uint32_t CRCPR;       
  volatile uint32_t RXCRCR;      
  volatile uint32_t TXCRCR;      
  volatile uint32_t I2SCFGR;     
  volatile uint32_t I2SPR;       
} SPI_TypeDef;



 

typedef struct
{
  volatile uint32_t CR1;          
  volatile uint32_t CR2;          
  volatile uint32_t SMCR;         
  volatile uint32_t DIER;         
  volatile uint32_t SR;           
  volatile uint32_t EGR;          
  volatile uint32_t CCMR1;        
  volatile uint32_t CCMR2;        
  volatile uint32_t CCER;         
  volatile uint32_t CNT;          
  volatile uint32_t PSC;          
  volatile uint32_t ARR;          
  volatile uint32_t RCR;          
  volatile uint32_t CCR1;         
  volatile uint32_t CCR2;         
  volatile uint32_t CCR3;         
  volatile uint32_t CCR4;         
  volatile uint32_t BDTR;         
  volatile uint32_t DCR;          
  volatile uint32_t DMAR;         
  volatile uint32_t OR;           
} TIM_TypeDef;



 
 
typedef struct
{
  volatile uint32_t SR;          
  volatile uint32_t DR;          
  volatile uint32_t BRR;         
  volatile uint32_t CR1;         
  volatile uint32_t CR2;         
  volatile uint32_t CR3;         
  volatile uint32_t GTPR;        
} USART_TypeDef;



 

typedef struct
{
  volatile uint32_t CR;    
  volatile uint32_t CFR;   
  volatile uint32_t SR;    
} WWDG_TypeDef;



 

typedef struct
{
  volatile uint32_t CR;      
  volatile uint32_t SR;      
  volatile uint32_t DR;      
  volatile uint32_t DOUT;    
  volatile uint32_t DMACR;   
  volatile uint32_t IMSCR;   
  volatile uint32_t RISR;    
  volatile uint32_t MISR;    
  volatile uint32_t K0LR;    
  volatile uint32_t K0RR;    
  volatile uint32_t K1LR;    
  volatile uint32_t K1RR;    
  volatile uint32_t K2LR;    
  volatile uint32_t K2RR;    
  volatile uint32_t K3LR;    
  volatile uint32_t K3RR;    
  volatile uint32_t IV0LR;   
  volatile uint32_t IV0RR;   
  volatile uint32_t IV1LR;   
  volatile uint32_t IV1RR;   
} CRYP_TypeDef;



 
  
typedef struct 
{
  volatile uint32_t CR;         
  volatile uint32_t DIN;        
  volatile uint32_t STR;        
  volatile uint32_t HR[5];      
  volatile uint32_t IMR;        
  volatile uint32_t SR;         
  uint32_t  RESERVED[52];   
  volatile uint32_t CSR[51];      
} HASH_TypeDef;



 
  
typedef struct 
{
  volatile uint32_t CR;   
  volatile uint32_t SR;   
  volatile uint32_t DR;   
} RNG_TypeDef;


 


 
typedef struct
{
  volatile uint32_t GOTGCTL;       
  volatile uint32_t GOTGINT;       
  volatile uint32_t GAHBCFG;       
  volatile uint32_t GUSBCFG;       
  volatile uint32_t GRSTCTL;       
  volatile uint32_t GINTSTS;       
  volatile uint32_t GINTMSK;       
  volatile uint32_t GRXSTSR;       
  volatile uint32_t GRXSTSP;       
  volatile uint32_t GRXFSIZ;       
  volatile uint32_t DIEPTXF0_HNPTXFSIZ;    
  volatile uint32_t HNPTXSTS;      
  uint32_t Reserved30[2];      
  volatile uint32_t GCCFG;         
  volatile uint32_t CID;           
  uint32_t  Reserved40[48];    
  volatile uint32_t HPTXFSIZ;  
  volatile uint32_t DIEPTXF[0x0F]; 
}
USB_OTG_GlobalTypeDef;





 
typedef struct 
{
  volatile uint32_t DCFG;          
  volatile uint32_t DCTL;          
  volatile uint32_t DSTS;          
  uint32_t Reserved0C;            
  volatile uint32_t DIEPMSK;    
  volatile uint32_t DOEPMSK;   
  volatile uint32_t DAINT;      
  volatile uint32_t DAINTMSK;  
  uint32_t  Reserved20;           
  uint32_t Reserved9;        
  volatile uint32_t DVBUSDIS;     
  volatile uint32_t DVBUSPULSE;   
  volatile uint32_t DTHRCTL;      
  volatile uint32_t DIEPEMPMSK;  
  volatile uint32_t DEACHINT;     
  volatile uint32_t DEACHMSK;       
  uint32_t Reserved40;       
  volatile uint32_t DINEP1MSK;   
  uint32_t  Reserved44[15];       
  volatile uint32_t DOUTEP1MSK;     
}
USB_OTG_DeviceTypeDef;




 
typedef struct 
{
  volatile uint32_t DIEPCTL;  
  uint32_t Reserved04;              
  volatile uint32_t DIEPINT;  
  uint32_t Reserved0C;              
  volatile uint32_t DIEPTSIZ;  
  volatile uint32_t DIEPDMA;  
  volatile uint32_t DTXFSTS; 
  uint32_t Reserved18;              
}
USB_OTG_INEndpointTypeDef;




 
typedef struct 
{
  volatile uint32_t DOEPCTL;        
  uint32_t Reserved04;          
  volatile uint32_t DOEPINT;        
  uint32_t Reserved0C;          
  volatile uint32_t DOEPTSIZ;       
  volatile uint32_t DOEPDMA;        
  uint32_t Reserved18[2];       
}
USB_OTG_OUTEndpointTypeDef;




 
typedef struct 
{
  volatile uint32_t HCFG;              
  volatile uint32_t HFIR;       
  volatile uint32_t HFNUM;          
  uint32_t Reserved40C;                    
  volatile uint32_t HPTXSTS;    
  volatile uint32_t HAINT;    
  volatile uint32_t HAINTMSK;    
}
USB_OTG_HostTypeDef;




 
typedef struct
{
  volatile uint32_t HCCHAR;
  volatile uint32_t HCSPLT;
  volatile uint32_t HCINT;
  volatile uint32_t HCINTMSK;
  volatile uint32_t HCTSIZ;
  volatile uint32_t HCDMA;
  uint32_t Reserved[2];
}
USB_OTG_HostChannelTypeDef;

    


 
#line 897 "..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32F2xx\\Include\\stm32f215xx.h"

 




 





 
#line 935 "..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32F2xx\\Include\\stm32f215xx.h"

 
#line 952 "..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32F2xx\\Include\\stm32f215xx.h"

 
#line 984 "..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32F2xx\\Include\\stm32f215xx.h"

 





 





 


 



#line 1016 "..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32F2xx\\Include\\stm32f215xx.h"



 
  


   
#line 1102 "..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32F2xx\\Include\\stm32f215xx.h"








 



 
  
  

 
    
 
 
 

 
 
 
 
 
 
#line 1136 "..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32F2xx\\Include\\stm32f215xx.h"

 
#line 1162 "..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32F2xx\\Include\\stm32f215xx.h"
  
 
#line 1188 "..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32F2xx\\Include\\stm32f215xx.h"

 
#line 1226 "..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32F2xx\\Include\\stm32f215xx.h"

 
#line 1268 "..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32F2xx\\Include\\stm32f215xx.h"

 


 


 


 


 


 


 
#line 1317 "..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32F2xx\\Include\\stm32f215xx.h"

 
#line 1355 "..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32F2xx\\Include\\stm32f215xx.h"

 
#line 1393 "..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32F2xx\\Include\\stm32f215xx.h"

 
#line 1422 "..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32F2xx\\Include\\stm32f215xx.h"

 


 


 


 


 



 
#line 1458 "..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32F2xx\\Include\\stm32f215xx.h"

 
#line 1480 "..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32F2xx\\Include\\stm32f215xx.h"

 



 
 
 
 
 
 
 
#line 1502 "..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32F2xx\\Include\\stm32f215xx.h"
 
#line 1512 "..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32F2xx\\Include\\stm32f215xx.h"

 
#line 1530 "..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32F2xx\\Include\\stm32f215xx.h"











 





 





 
#line 1568 "..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32F2xx\\Include\\stm32f215xx.h"

 












 
#line 1598 "..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32F2xx\\Include\\stm32f215xx.h"


 
 






 




 





 





 






 




 





 





 






   




 





 





 





 




 





 





 





 




 





 





 
 



 
#line 1740 "..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32F2xx\\Include\\stm32f215xx.h"

 
#line 1757 "..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32F2xx\\Include\\stm32f215xx.h"

 
#line 1774 "..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32F2xx\\Include\\stm32f215xx.h"

 
#line 1791 "..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32F2xx\\Include\\stm32f215xx.h"

 
#line 1825 "..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32F2xx\\Include\\stm32f215xx.h"

 
#line 1859 "..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32F2xx\\Include\\stm32f215xx.h"

 
#line 1893 "..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32F2xx\\Include\\stm32f215xx.h"

 
#line 1927 "..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32F2xx\\Include\\stm32f215xx.h"

 
#line 1961 "..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32F2xx\\Include\\stm32f215xx.h"

 
#line 1995 "..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32F2xx\\Include\\stm32f215xx.h"

 
#line 2029 "..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32F2xx\\Include\\stm32f215xx.h"

 
#line 2063 "..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32F2xx\\Include\\stm32f215xx.h"

 
#line 2097 "..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32F2xx\\Include\\stm32f215xx.h"

 
#line 2131 "..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32F2xx\\Include\\stm32f215xx.h"

 
#line 2165 "..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32F2xx\\Include\\stm32f215xx.h"

 
#line 2199 "..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32F2xx\\Include\\stm32f215xx.h"

 
#line 2233 "..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32F2xx\\Include\\stm32f215xx.h"

 
#line 2267 "..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32F2xx\\Include\\stm32f215xx.h"

 
#line 2301 "..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32F2xx\\Include\\stm32f215xx.h"

 
#line 2335 "..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32F2xx\\Include\\stm32f215xx.h"

 
#line 2369 "..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32F2xx\\Include\\stm32f215xx.h"

 
#line 2403 "..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32F2xx\\Include\\stm32f215xx.h"

 
#line 2437 "..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32F2xx\\Include\\stm32f215xx.h"

 
#line 2471 "..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32F2xx\\Include\\stm32f215xx.h"

 
#line 2505 "..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32F2xx\\Include\\stm32f215xx.h"

 
#line 2539 "..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32F2xx\\Include\\stm32f215xx.h"

 
#line 2573 "..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32F2xx\\Include\\stm32f215xx.h"

 
#line 2607 "..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32F2xx\\Include\\stm32f215xx.h"

 
#line 2641 "..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32F2xx\\Include\\stm32f215xx.h"

 
#line 2675 "..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32F2xx\\Include\\stm32f215xx.h"

 
#line 2709 "..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32F2xx\\Include\\stm32f215xx.h"

 
#line 2743 "..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32F2xx\\Include\\stm32f215xx.h"

 
 
 
 
 
 



 



 


 
 
 
 
 
 


#line 2780 "..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32F2xx\\Include\\stm32f215xx.h"

#line 2789 "..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32F2xx\\Include\\stm32f215xx.h"
 





 


 


 


 



 
 
 
 
 
 









































 



 


 


 


 


 


 


 



 



 



 


 


 



 
 
 
 
 

 
 
 
 
 
  
#line 2946 "..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32F2xx\\Include\\stm32f215xx.h"

 
#line 2965 "..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32F2xx\\Include\\stm32f215xx.h"

  
#line 2976 "..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32F2xx\\Include\\stm32f215xx.h"

  
#line 2998 "..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32F2xx\\Include\\stm32f215xx.h"

  
#line 3020 "..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32F2xx\\Include\\stm32f215xx.h"

  
#line 3042 "..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32F2xx\\Include\\stm32f215xx.h"

  
#line 3064 "..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32F2xx\\Include\\stm32f215xx.h"


 
 
 
 
 
 
#line 3092 "..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32F2xx\\Include\\stm32f215xx.h"

 
#line 3114 "..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32F2xx\\Include\\stm32f215xx.h"

 
#line 3136 "..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32F2xx\\Include\\stm32f215xx.h"

 
#line 3158 "..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32F2xx\\Include\\stm32f215xx.h"

 
#line 3180 "..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32F2xx\\Include\\stm32f215xx.h"

 
#line 3202 "..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32F2xx\\Include\\stm32f215xx.h"

 
 
 
 
 
 
#line 3218 "..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32F2xx\\Include\\stm32f215xx.h"

#line 3226 "..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32F2xx\\Include\\stm32f215xx.h"

 
#line 3235 "..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32F2xx\\Include\\stm32f215xx.h"

 
#line 3251 "..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32F2xx\\Include\\stm32f215xx.h"

 
#line 3283 "..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32F2xx\\Include\\stm32f215xx.h"

 
 
 
 
 
 











#line 3311 "..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32F2xx\\Include\\stm32f215xx.h"

 











#line 3334 "..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32F2xx\\Include\\stm32f215xx.h"

 











#line 3357 "..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32F2xx\\Include\\stm32f215xx.h"

 











#line 3380 "..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32F2xx\\Include\\stm32f215xx.h"

 








































 








































 








































 








































 


































 


































 


































 


































 



























 



























 



























 
#line 3777 "..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32F2xx\\Include\\stm32f215xx.h"

 
#line 3786 "..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32F2xx\\Include\\stm32f215xx.h"

 
#line 3795 "..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32F2xx\\Include\\stm32f215xx.h"

 
#line 3806 "..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32F2xx\\Include\\stm32f215xx.h"

#line 3816 "..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32F2xx\\Include\\stm32f215xx.h"

#line 3826 "..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32F2xx\\Include\\stm32f215xx.h"

#line 3836 "..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32F2xx\\Include\\stm32f215xx.h"

 
#line 3847 "..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32F2xx\\Include\\stm32f215xx.h"

#line 3857 "..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32F2xx\\Include\\stm32f215xx.h"

#line 3867 "..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32F2xx\\Include\\stm32f215xx.h"

#line 3877 "..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32F2xx\\Include\\stm32f215xx.h"

 
#line 3888 "..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32F2xx\\Include\\stm32f215xx.h"

#line 3898 "..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32F2xx\\Include\\stm32f215xx.h"

#line 3908 "..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32F2xx\\Include\\stm32f215xx.h"

#line 3918 "..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32F2xx\\Include\\stm32f215xx.h"

 
#line 3929 "..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32F2xx\\Include\\stm32f215xx.h"

#line 3939 "..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32F2xx\\Include\\stm32f215xx.h"

#line 3949 "..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32F2xx\\Include\\stm32f215xx.h"

#line 3959 "..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32F2xx\\Include\\stm32f215xx.h"

 
#line 3970 "..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32F2xx\\Include\\stm32f215xx.h"

#line 3980 "..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32F2xx\\Include\\stm32f215xx.h"

#line 3990 "..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32F2xx\\Include\\stm32f215xx.h"

#line 4000 "..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32F2xx\\Include\\stm32f215xx.h"

 
#line 4011 "..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32F2xx\\Include\\stm32f215xx.h"

#line 4021 "..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32F2xx\\Include\\stm32f215xx.h"

#line 4031 "..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32F2xx\\Include\\stm32f215xx.h"

#line 4041 "..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32F2xx\\Include\\stm32f215xx.h"

 
#line 4052 "..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32F2xx\\Include\\stm32f215xx.h"

#line 4062 "..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32F2xx\\Include\\stm32f215xx.h"

#line 4072 "..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32F2xx\\Include\\stm32f215xx.h"

#line 4082 "..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32F2xx\\Include\\stm32f215xx.h"

 


 


 
 
 
 
 
 
































































 
#line 4176 "..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32F2xx\\Include\\stm32f215xx.h"

 
































































 
































































 
#line 4324 "..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32F2xx\\Include\\stm32f215xx.h"
 
#line 4341 "..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32F2xx\\Include\\stm32f215xx.h"

 
#line 4359 "..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32F2xx\\Include\\stm32f215xx.h"
 
#line 4376 "..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32F2xx\\Include\\stm32f215xx.h"

 
#line 4410 "..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32F2xx\\Include\\stm32f215xx.h"

 
 
 
 
 
 
#line 4432 "..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32F2xx\\Include\\stm32f215xx.h"

 
#line 4441 "..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32F2xx\\Include\\stm32f215xx.h"

 



 





 
 
 
 
 
 
#line 4472 "..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32F2xx\\Include\\stm32f215xx.h"

 
#line 4481 "..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32F2xx\\Include\\stm32f215xx.h"







 



#line 4502 "..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32F2xx\\Include\\stm32f215xx.h"



 



 


 
#line 4527 "..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32F2xx\\Include\\stm32f215xx.h"

 
#line 4537 "..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32F2xx\\Include\\stm32f215xx.h"

 




 


 



 
 
 
 
 
 


 





 


 



 
 
 
 
 
 











 
#line 4597 "..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32F2xx\\Include\\stm32f215xx.h"




 
#line 4608 "..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32F2xx\\Include\\stm32f215xx.h"

 
 
 
 
 
 



#line 4624 "..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32F2xx\\Include\\stm32f215xx.h"

#line 4634 "..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32F2xx\\Include\\stm32f215xx.h"

#line 4643 "..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32F2xx\\Include\\stm32f215xx.h"

 
#line 4652 "..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32F2xx\\Include\\stm32f215xx.h"

#line 4663 "..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32F2xx\\Include\\stm32f215xx.h"















 
 








 








 






#line 4713 "..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32F2xx\\Include\\stm32f215xx.h"

 











 











 
#line 4745 "..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32F2xx\\Include\\stm32f215xx.h"

 




















 
#line 4774 "..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32F2xx\\Include\\stm32f215xx.h"

#line 4782 "..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32F2xx\\Include\\stm32f215xx.h"

#line 4789 "..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32F2xx\\Include\\stm32f215xx.h"



 
#line 4806 "..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32F2xx\\Include\\stm32f215xx.h"

 


  




 



 
#line 4843 "..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32F2xx\\Include\\stm32f215xx.h"

 
#line 4856 "..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32F2xx\\Include\\stm32f215xx.h"

 


 
#line 4874 "..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32F2xx\\Include\\stm32f215xx.h"




 





 



 
#line 4912 "..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32F2xx\\Include\\stm32f215xx.h"

 
#line 4927 "..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32F2xx\\Include\\stm32f215xx.h"

 
#line 4947 "..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32F2xx\\Include\\stm32f215xx.h"

 





 



 
#line 4982 "..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32F2xx\\Include\\stm32f215xx.h"

 
#line 4997 "..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32F2xx\\Include\\stm32f215xx.h"

 











 
#line 5021 "..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32F2xx\\Include\\stm32f215xx.h"

 





 
#line 5039 "..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32F2xx\\Include\\stm32f215xx.h"






 
 
 
 
 
 



 






 
 
 
 
 
 
#line 5094 "..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32F2xx\\Include\\stm32f215xx.h"

 
#line 5124 "..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32F2xx\\Include\\stm32f215xx.h"

 
#line 5150 "..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32F2xx\\Include\\stm32f215xx.h"

 
#line 5165 "..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32F2xx\\Include\\stm32f215xx.h"

 



 


 



 
#line 5218 "..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32F2xx\\Include\\stm32f215xx.h"

 
#line 5260 "..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32F2xx\\Include\\stm32f215xx.h"

 


 
#line 5292 "..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32F2xx\\Include\\stm32f215xx.h"

 
#line 5312 "..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32F2xx\\Include\\stm32f215xx.h"

 
#line 5320 "..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32F2xx\\Include\\stm32f215xx.h"

 


 


 


 


 


 


 


 


 


 


 


 


 


 


 


 


 


 


 


 




 
 
 
 
 
 




 












 


 






#line 5423 "..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32F2xx\\Include\\stm32f215xx.h"

 


 


 


 


 


 


 


 


 
















 


 
#line 5493 "..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32F2xx\\Include\\stm32f215xx.h"

 
#line 5508 "..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32F2xx\\Include\\stm32f215xx.h"

 
#line 5534 "..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32F2xx\\Include\\stm32f215xx.h"

 


 


 
 
 
 
 
 









#line 5566 "..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32F2xx\\Include\\stm32f215xx.h"

 
#line 5575 "..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32F2xx\\Include\\stm32f215xx.h"

 
#line 5586 "..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32F2xx\\Include\\stm32f215xx.h"

 


 


 


 


 





















 



                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                  
                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                   
                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                   
                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                   
                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                   
                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                   
                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                   



                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                  
                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                   






                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                               
#line 5653 "..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32F2xx\\Include\\stm32f215xx.h"


                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                               
#line 5665 "..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32F2xx\\Include\\stm32f215xx.h"


                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                               
#line 5677 "..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32F2xx\\Include\\stm32f215xx.h"


                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                               
#line 5689 "..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32F2xx\\Include\\stm32f215xx.h"
                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                  
 






  
#line 5707 "..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32F2xx\\Include\\stm32f215xx.h"


  
#line 5719 "..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32F2xx\\Include\\stm32f215xx.h"


  
#line 5731 "..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32F2xx\\Include\\stm32f215xx.h"


  
#line 5743 "..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32F2xx\\Include\\stm32f215xx.h"

 




           


  
#line 5762 "..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32F2xx\\Include\\stm32f215xx.h"


  
#line 5774 "..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32F2xx\\Include\\stm32f215xx.h"


  
#line 5786 "..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32F2xx\\Include\\stm32f215xx.h"


  
#line 5798 "..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32F2xx\\Include\\stm32f215xx.h"

 






  
#line 5815 "..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32F2xx\\Include\\stm32f215xx.h"


  
#line 5826 "..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32F2xx\\Include\\stm32f215xx.h"


  
#line 5837 "..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32F2xx\\Include\\stm32f215xx.h"


  
#line 5848 "..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32F2xx\\Include\\stm32f215xx.h"

   



 
 
 
 
 
 
















 









#line 5893 "..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32F2xx\\Include\\stm32f215xx.h"

 

























 
#line 5936 "..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32F2xx\\Include\\stm32f215xx.h"

 
#line 5950 "..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32F2xx\\Include\\stm32f215xx.h"

 
#line 5960 "..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32F2xx\\Include\\stm32f215xx.h"

 




























 





















 




























 





















 
#line 6079 "..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32F2xx\\Include\\stm32f215xx.h"

 


 


 


 


 


 


 


 


 
#line 6114 "..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32F2xx\\Include\\stm32f215xx.h"





#line 6125 "..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32F2xx\\Include\\stm32f215xx.h"

 
#line 6133 "..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32F2xx\\Include\\stm32f215xx.h"

#line 6140 "..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32F2xx\\Include\\stm32f215xx.h"

 


 
#line 6151 "..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32F2xx\\Include\\stm32f215xx.h"


 
 
 
 
 
 
#line 6169 "..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32F2xx\\Include\\stm32f215xx.h"

 


 



 
#line 6193 "..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32F2xx\\Include\\stm32f215xx.h"

 
#line 6202 "..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32F2xx\\Include\\stm32f215xx.h"







 
#line 6222 "..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32F2xx\\Include\\stm32f215xx.h"

 
#line 6233 "..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32F2xx\\Include\\stm32f215xx.h"



 
 
 
 
 
 
#line 6250 "..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32F2xx\\Include\\stm32f215xx.h"



 
#line 6262 "..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32F2xx\\Include\\stm32f215xx.h"







 



 
 
 
 
 
 



 









 
#line 6310 "..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32F2xx\\Include\\stm32f215xx.h"
 


 






 
 
 
 
 
 
#line 6336 "..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32F2xx\\Include\\stm32f215xx.h"

 






 






#line 6359 "..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32F2xx\\Include\\stm32f215xx.h"









 




 
#line 6380 "..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32F2xx\\Include\\stm32f215xx.h"

 





#line 6396 "..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32F2xx\\Include\\stm32f215xx.h"

 


 



 








 


#line 6424 "..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32F2xx\\Include\\stm32f215xx.h"

 

#line 6434 "..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32F2xx\\Include\\stm32f215xx.h"

#line 6453 "..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32F2xx\\Include\\stm32f215xx.h"

 






#line 6469 "..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32F2xx\\Include\\stm32f215xx.h"

 
#line 6479 "..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32F2xx\\Include\\stm32f215xx.h"

 


#line 6492 "..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32F2xx\\Include\\stm32f215xx.h"

#line 6502 "..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32F2xx\\Include\\stm32f215xx.h"

 


 
#line 6514 "..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32F2xx\\Include\\stm32f215xx.h"

 
#line 6542 "..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32F2xx\\Include\\stm32f215xx.h"

 
#line 6570 "..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32F2xx\\Include\\stm32f215xx.h"

 



 


 





 



 

#line 6596 "..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32F2xx\\Include\\stm32f215xx.h"























 

#line 6627 "..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32F2xx\\Include\\stm32f215xx.h"























 


 


 





 


 


#line 6677 "..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32F2xx\\Include\\stm32f215xx.h"

#line 6686 "..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32F2xx\\Include\\stm32f215xx.h"

 



#line 6702 "..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32F2xx\\Include\\stm32f215xx.h"

#line 6714 "..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32F2xx\\Include\\stm32f215xx.h"

 


 



 
#line 6729 "..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32F2xx\\Include\\stm32f215xx.h"

 



 


 
#line 6747 "..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32F2xx\\Include\\stm32f215xx.h"

 
#line 6758 "..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32F2xx\\Include\\stm32f215xx.h"
















 
#line 6786 "..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32F2xx\\Include\\stm32f215xx.h"

 



 










#line 6813 "..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32F2xx\\Include\\stm32f215xx.h"

 


#line 6824 "..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32F2xx\\Include\\stm32f215xx.h"









#line 6844 "..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32F2xx\\Include\\stm32f215xx.h"

 

#line 6855 "..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32F2xx\\Include\\stm32f215xx.h"

#line 6864 "..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32F2xx\\Include\\stm32f215xx.h"







 
#line 6883 "..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32F2xx\\Include\\stm32f215xx.h"

 
#line 6896 "..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32F2xx\\Include\\stm32f215xx.h"

 
#line 6909 "..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32F2xx\\Include\\stm32f215xx.h"

 




 
#line 6922 "..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32F2xx\\Include\\stm32f215xx.h"

 


 


 


 



 

#line 6952 "..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32F2xx\\Include\\stm32f215xx.h"

 
#line 6960 "..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32F2xx\\Include\\stm32f215xx.h"

 








 






  



 



 
 
 




 


 
 


 


 
#line 7019 "..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32F2xx\\Include\\stm32f215xx.h"

 
#line 7030 "..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32F2xx\\Include\\stm32f215xx.h"

 




 



 



 


 


 




 




 
#line 7075 "..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32F2xx\\Include\\stm32f215xx.h"

 
#line 7089 "..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32F2xx\\Include\\stm32f215xx.h"

 
#line 7099 "..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32F2xx\\Include\\stm32f215xx.h"

 
#line 7107 "..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32F2xx\\Include\\stm32f215xx.h"

 
#line 7115 "..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32F2xx\\Include\\stm32f215xx.h"

 



 
#line 7127 "..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32F2xx\\Include\\stm32f215xx.h"

 
#line 7137 "..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32F2xx\\Include\\stm32f215xx.h"

 
#line 7145 "..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32F2xx\\Include\\stm32f215xx.h"

 
#line 7153 "..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32F2xx\\Include\\stm32f215xx.h"

 
#line 7161 "..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32F2xx\\Include\\stm32f215xx.h"

 
#line 7173 "..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32F2xx\\Include\\stm32f215xx.h"

 
#line 7183 "..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32F2xx\\Include\\stm32f215xx.h"

 



 
#line 7195 "..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32F2xx\\Include\\stm32f215xx.h"

 




 
#line 7258 "..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32F2xx\\Include\\stm32f215xx.h"

 
#line 7270 "..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32F2xx\\Include\\stm32f215xx.h"

 





 
#line 7284 "..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32F2xx\\Include\\stm32f215xx.h"

 





 





 
#line 7304 "..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32F2xx\\Include\\stm32f215xx.h"

 


 





  



 



 







 

#line 116 "..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32F2xx\\Include\\stm32f2xx.h"
#line 123 "..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32F2xx\\Include\\stm32f2xx.h"



 



  
typedef enum 
{
  RESET = 0, 
  SET = !RESET
} FlagStatus, ITStatus;

typedef enum 
{
  DISABLE = 0, 
  ENABLE = !DISABLE
} FunctionalState;


typedef enum 
{
  ERROR = 0, 
  SUCCESS = !ERROR
} ErrorStatus;



 




 



















 










 



 
  



 
#line 49 "..\\..\\Drivers\\STM32F2xx_HAL_Driver\\Inc\\stm32f2xx_hal_def.h"

 



   
typedef enum 
{
  HAL_OK       = 0x00,
  HAL_ERROR    = 0x01,
  HAL_BUSY     = 0x02,
  HAL_TIMEOUT  = 0x03
} HAL_StatusTypeDef;



 
typedef enum 
{
  HAL_UNLOCKED = 0x00,
  HAL_LOCKED   = 0x01  
} HAL_LockTypeDef;

 















#line 102 "..\\..\\Drivers\\STM32F2xx_HAL_Driver\\Inc\\stm32f2xx_hal_def.h"







#line 117 "..\\..\\Drivers\\STM32F2xx_HAL_Driver\\Inc\\stm32f2xx_hal_def.h"

  
 
#line 141 "..\\..\\Drivers\\STM32F2xx_HAL_Driver\\Inc\\stm32f2xx_hal_def.h"







 
#line 48 "..\\..\\Drivers\\STM32F2xx_HAL_Driver\\Inc\\stm32f2xx_hal_rcc.h"



 



 

  



 
typedef struct
{
  uint32_t PLLState;   
 

  uint32_t PLLSource;  
            

  uint32_t PLLM;       
         

  uint32_t PLLN;       
 

  uint32_t PLLP;       
 

  uint32_t PLLQ;       
 

}RCC_PLLInitTypeDef;



 
typedef struct
{
  uint32_t OscillatorType;       
 

  uint32_t HSEState;             
 

  uint32_t LSEState;             
 
                                          
  uint32_t HSIState;             
 

  uint32_t HSICalibrationValue;  
 
                               
  uint32_t LSIState;             
 

  RCC_PLLInitTypeDef PLL;               

}RCC_OscInitTypeDef;



 
typedef struct
{
  uint32_t ClockType;             
 

  uint32_t SYSCLKSource;          
 

  uint32_t AHBCLKDivider;         
 

  uint32_t APB1CLKDivider;        
 

  uint32_t APB2CLKDivider;        
 

}RCC_ClkInitTypeDef;

 


 




 

 
 



 


 


 



 
 




 
 



 



 
 




 


 


 


 







 



 









 



 








 



 








 



 






 



 






 



 







 



 






 



 



#line 295 "..\\..\\Drivers\\STM32F2xx_HAL_Driver\\Inc\\stm32f2xx_hal_rcc.h"






 



 








 
  


 









  



 
#line 342 "..\\..\\Drivers\\STM32F2xx_HAL_Driver\\Inc\\stm32f2xx_hal_rcc.h"








  
  


 











  



 
#line 403 "..\\..\\Drivers\\STM32F2xx_HAL_Driver\\Inc\\stm32f2xx_hal_rcc.h"


 



 




 



 






 



 









 



 









 



 











 



 
#line 481 "..\\..\\Drivers\\STM32F2xx_HAL_Driver\\Inc\\stm32f2xx_hal_rcc.h"


 
  








 
 





 


 
#line 512 "..\\..\\Drivers\\STM32F2xx_HAL_Driver\\Inc\\stm32f2xx_hal_rcc.h"




 



  




 



 




 





    
 





 
#line 563 "..\\..\\Drivers\\STM32F2xx_HAL_Driver\\Inc\\stm32f2xx_hal_rcc.h"

#line 579 "..\\..\\Drivers\\STM32F2xx_HAL_Driver\\Inc\\stm32f2xx_hal_rcc.h"





 



                                        








 







 







 
#line 637 "..\\..\\Drivers\\STM32F2xx_HAL_Driver\\Inc\\stm32f2xx_hal_rcc.h"

#line 661 "..\\..\\Drivers\\STM32F2xx_HAL_Driver\\Inc\\stm32f2xx_hal_rcc.h"






 
#line 681 "..\\..\\Drivers\\STM32F2xx_HAL_Driver\\Inc\\stm32f2xx_hal_rcc.h"

#line 695 "..\\..\\Drivers\\STM32F2xx_HAL_Driver\\Inc\\stm32f2xx_hal_rcc.h"


   
#line 713 "..\\..\\Drivers\\STM32F2xx_HAL_Driver\\Inc\\stm32f2xx_hal_rcc.h"

#line 729 "..\\..\\Drivers\\STM32F2xx_HAL_Driver\\Inc\\stm32f2xx_hal_rcc.h"


 










 
#line 767 "..\\..\\Drivers\\STM32F2xx_HAL_Driver\\Inc\\stm32f2xx_hal_rcc.h"

#line 793 "..\\..\\Drivers\\STM32F2xx_HAL_Driver\\Inc\\stm32f2xx_hal_rcc.h"


 
#line 808 "..\\..\\Drivers\\STM32F2xx_HAL_Driver\\Inc\\stm32f2xx_hal_rcc.h"

#line 821 "..\\..\\Drivers\\STM32F2xx_HAL_Driver\\Inc\\stm32f2xx_hal_rcc.h"


   











 
#line 854 "..\\..\\Drivers\\STM32F2xx_HAL_Driver\\Inc\\stm32f2xx_hal_rcc.h"

#line 873 "..\\..\\Drivers\\STM32F2xx_HAL_Driver\\Inc\\stm32f2xx_hal_rcc.h"






 












 









 
#line 926 "..\\..\\Drivers\\STM32F2xx_HAL_Driver\\Inc\\stm32f2xx_hal_rcc.h"
                                      
#line 950 "..\\..\\Drivers\\STM32F2xx_HAL_Driver\\Inc\\stm32f2xx_hal_rcc.h"






 
#line 970 "..\\..\\Drivers\\STM32F2xx_HAL_Driver\\Inc\\stm32f2xx_hal_rcc.h"

#line 984 "..\\..\\Drivers\\STM32F2xx_HAL_Driver\\Inc\\stm32f2xx_hal_rcc.h"















 








 










 





















 

















 




 























 


                                                   








 









 





























 












 




 















 








      







 





 











 












 













 













 




 



















 




 




 
#line 1 "..\\..\\Drivers\\STM32F2xx_HAL_Driver\\Inc\\stm32f2xx_hal_rcc_ex.h"



































  

 







 
#line 48 "..\\..\\Drivers\\STM32F2xx_HAL_Driver\\Inc\\stm32f2xx_hal_rcc_ex.h"



 



  

  



 
typedef struct
{
  uint32_t PLLI2SN;    

 

  uint32_t PLLI2SR;    

 

}RCC_PLLI2SInitTypeDef;



 
typedef struct
{
  uint32_t PeriphClockSelection; 
 

  RCC_PLLI2SInitTypeDef PLLI2S;  
 

  uint32_t RTCClockSelection;      
 

  uint8_t TIMPresSelection;      
 

}RCC_PeriphCLKInitTypeDef;


 


 



 







 



 




 



 
     
 





 

#line 143 "..\\..\\Drivers\\STM32F2xx_HAL_Driver\\Inc\\stm32f2xx_hal_rcc_ex.h"



 
#line 163 "..\\..\\Drivers\\STM32F2xx_HAL_Driver\\Inc\\stm32f2xx_hal_rcc_ex.h"





 
















 







 
















 


 


 






 

#line 236 "..\\..\\Drivers\\STM32F2xx_HAL_Driver\\Inc\\stm32f2xx_hal_rcc_ex.h"






 















 


 
HAL_StatusTypeDef HAL_RCCEx_PeriphCLKConfig(RCC_PeriphCLKInitTypeDef  *PeriphClkInit);
void HAL_RCCEx_GetPeriphCLKConfig(RCC_PeriphCLKInitTypeDef  *PeriphClkInit);



  



 
  






 
#line 1287 "..\\..\\Drivers\\STM32F2xx_HAL_Driver\\Inc\\stm32f2xx_hal_rcc.h"

 
                              
 
void HAL_RCC_DeInit(void);
HAL_StatusTypeDef HAL_RCC_OscConfig(RCC_OscInitTypeDef *RCC_OscInitStruct);
HAL_StatusTypeDef HAL_RCC_ClockConfig(RCC_ClkInitTypeDef *RCC_ClkInitStruct, uint32_t FLatency);

 
void     HAL_RCC_MCOConfig(uint32_t RCC_MCOx, uint32_t RCC_MCOSource, uint32_t RCC_MCODiv);
void     HAL_RCC_EnableCSS(void);
void     HAL_RCC_DisableCSS(void);
uint32_t HAL_RCC_GetSysClockFreq(void);
uint32_t HAL_RCC_GetHCLKFreq(void);
uint32_t HAL_RCC_GetPCLK1Freq(void);
uint32_t HAL_RCC_GetPCLK2Freq(void);
void     HAL_RCC_GetOscConfig(RCC_OscInitTypeDef *RCC_OscInitStruct);
void     HAL_RCC_GetClockConfig(RCC_ClkInitTypeDef *RCC_ClkInitStruct, uint32_t *pFLatency);

 
void HAL_RCC_NMI_IRQHandler(void);

  
void HAL_RCC_CCSCallback(void);



  



 







 
#line 213 "..\\..\\Inc\\stm32f2xx_hal_conf.h"


#line 1 "..\\..\\Drivers\\STM32F2xx_HAL_Driver\\Inc\\stm32f2xx_hal_gpio.h"



































  

 







 
#line 48 "..\\..\\Drivers\\STM32F2xx_HAL_Driver\\Inc\\stm32f2xx_hal_gpio.h"



 



  

 



  
typedef struct
{
  uint32_t Pin;       
 

  uint32_t Mode;      
 

  uint32_t Pull;      
 

  uint32_t Speed;     
 

  uint32_t Alternate;  
 
}GPIO_InitTypeDef;



 
typedef enum
{
  GPIO_PIN_RESET = 0,
  GPIO_PIN_SET
}GPIO_PinState;


 



 



 
#line 116 "..\\..\\Drivers\\STM32F2xx_HAL_Driver\\Inc\\stm32f2xx_hal_gpio.h"

#line 134 "..\\..\\Drivers\\STM32F2xx_HAL_Driver\\Inc\\stm32f2xx_hal_gpio.h"


 










  







    



 



  
#line 176 "..\\..\\Drivers\\STM32F2xx_HAL_Driver\\Inc\\stm32f2xx_hal_gpio.h"



 



   









 

 


   








 
  


 

 






 







 







 







 


 
#line 1 "..\\..\\Drivers\\STM32F2xx_HAL_Driver\\Inc\\stm32f2xx_hal_gpio_ex.h"



































  

 







 
#line 48 "..\\..\\Drivers\\STM32F2xx_HAL_Driver\\Inc\\stm32f2xx_hal_gpio_ex.h"



 



  

 

 



  
  


 



  








  





  






  







  






  




  




  






  






  








  





  






  






  






  


#line 209 "..\\..\\Drivers\\STM32F2xx_HAL_Driver\\Inc\\stm32f2xx_hal_gpio_ex.h"
                          


  



 

 
  




  



  
  






 
#line 249 "..\\..\\Drivers\\STM32F2xx_HAL_Driver\\Inc\\stm32f2xx_hal_gpio.h"

  
 
void  HAL_GPIO_Init(GPIO_TypeDef  *GPIOx, GPIO_InitTypeDef *GPIO_Init);
void  HAL_GPIO_DeInit(GPIO_TypeDef  *GPIOx, uint32_t GPIO_Pin);

 
GPIO_PinState HAL_GPIO_ReadPin(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin);
void          HAL_GPIO_WritePin(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin, GPIO_PinState PinState);
void          HAL_GPIO_TogglePin(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin);
void          HAL_GPIO_EXTI_IRQHandler(uint16_t GPIO_Pin);
void          HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin);



  



  







 
#line 217 "..\\..\\Inc\\stm32f2xx_hal_conf.h"


#line 1 "..\\..\\Drivers\\STM32F2xx_HAL_Driver\\Inc\\stm32f2xx_hal_dma.h"



































  

 







 
#line 48 "..\\..\\Drivers\\STM32F2xx_HAL_Driver\\Inc\\stm32f2xx_hal_dma.h"



 



  

  



 
typedef struct
{
  uint32_t Channel;              
 
                               
  uint32_t Direction;            

 

  uint32_t PeriphInc;            
   
                               
  uint32_t MemInc;               
 
  
  uint32_t PeriphDataAlignment;  
    

  uint32_t MemDataAlignment;     
 
                               
  uint32_t Mode;                 


  

  uint32_t Priority;             
 

  uint32_t FIFOMode;             


 
                               
  uint32_t FIFOThreshold;        
 
   
  uint32_t MemBurst;             



 
  
  uint32_t PeriphBurst;          



 
  
}DMA_InitTypeDef;



  
typedef enum
{
  HAL_DMA_STATE_RESET             = 0x00,     
  HAL_DMA_STATE_READY             = 0x01,   
  HAL_DMA_STATE_READY_MEM0        = 0x11,   
  HAL_DMA_STATE_READY_MEM1        = 0x21,    
  HAL_DMA_STATE_READY_HALF_MEM0   = 0x31,   
  HAL_DMA_STATE_READY_HALF_MEM1   = 0x41,     
  HAL_DMA_STATE_BUSY              = 0x02,   
  HAL_DMA_STATE_BUSY_MEM0         = 0x12,   
  HAL_DMA_STATE_BUSY_MEM1         = 0x22,          
  HAL_DMA_STATE_TIMEOUT           = 0x03,     
  HAL_DMA_STATE_ERROR             = 0x04,   
  
}HAL_DMA_StateTypeDef;



  
typedef enum
{
  HAL_DMA_FULL_TRANSFER      = 0x00,     
  HAL_DMA_HALF_TRANSFER      = 0x01,     

}HAL_DMA_LevelCompleteTypeDef;




  
typedef struct __DMA_HandleTypeDef
{  
  DMA_Stream_TypeDef         *Instance;                                                     
  
  DMA_InitTypeDef            Init;                                                           
  
  HAL_LockTypeDef            Lock;                                                            
  
  volatile HAL_DMA_StateTypeDef  State;                                                         
  
  void                       *Parent;                                                         
    
  void                       (* XferCpltCallback)( struct __DMA_HandleTypeDef * hdma);      
  
  void                       (* XferHalfCpltCallback)( struct __DMA_HandleTypeDef * hdma);  
  
  void                       (* XferM1CpltCallback)( struct __DMA_HandleTypeDef * hdma);    
  
  void                       (* XferErrorCallback)( struct __DMA_HandleTypeDef * hdma);     

  volatile uint32_t              ErrorCode;                                                     
  
}DMA_HandleTypeDef;    

 



 



  







 



  
#line 199 "..\\..\\Drivers\\STM32F2xx_HAL_Driver\\Inc\\stm32f2xx_hal_dma.h"

#line 208 "..\\..\\Drivers\\STM32F2xx_HAL_Driver\\Inc\\stm32f2xx_hal_dma.h"


 



  









 
    


  



      
        


  







  



  







 



  









  




 









 



  









 



 











  



 







  



 











  



  











  



  











 



 







 



  
#line 412 "..\\..\\Drivers\\STM32F2xx_HAL_Driver\\Inc\\stm32f2xx_hal_dma.h"


 
  


 
  
 











 






 






 


 





 
#line 470 "..\\..\\Drivers\\STM32F2xx_HAL_Driver\\Inc\\stm32f2xx_hal_dma.h"





       
#line 490 "..\\..\\Drivers\\STM32F2xx_HAL_Driver\\Inc\\stm32f2xx_hal_dma.h"





 
#line 510 "..\\..\\Drivers\\STM32F2xx_HAL_Driver\\Inc\\stm32f2xx_hal_dma.h"





 
#line 530 "..\\..\\Drivers\\STM32F2xx_HAL_Driver\\Inc\\stm32f2xx_hal_dma.h"





 
#line 550 "..\\..\\Drivers\\STM32F2xx_HAL_Driver\\Inc\\stm32f2xx_hal_dma.h"













 

















 
















 














 














 




















 







 



 
#line 1 "..\\..\\Drivers\\STM32F2xx_HAL_Driver\\Inc\\stm32f2xx_hal_dma_ex.h"



































 

 







 
#line 48 "..\\..\\Drivers\\STM32F2xx_HAL_Driver\\Inc\\stm32f2xx_hal_dma_ex.h"



 



  

 



  
typedef enum
{
  MEMORY0      = 0x00,     
  MEMORY1      = 0x01,     

}HAL_DMA_MemoryTypeDef;

  
 

 
 
HAL_StatusTypeDef HAL_DMAEx_MultiBufferStart(DMA_HandleTypeDef *hdma, uint32_t SrcAddress, uint32_t DstAddress, uint32_t SecondMemAddress, uint32_t DataLength);
HAL_StatusTypeDef HAL_DMAEx_MultiBufferStart_IT(DMA_HandleTypeDef *hdma, uint32_t SrcAddress, uint32_t DstAddress, uint32_t SecondMemAddress, uint32_t DataLength);
HAL_StatusTypeDef HAL_DMAEx_ChangeMemory(DMA_HandleTypeDef *hdma, uint32_t Address, HAL_DMA_MemoryTypeDef memory);



 



 







 
#line 663 "..\\..\\Drivers\\STM32F2xx_HAL_Driver\\Inc\\stm32f2xx_hal_dma.h"

 
  
 
HAL_StatusTypeDef HAL_DMA_Init(DMA_HandleTypeDef *hdma); 
HAL_StatusTypeDef HAL_DMA_DeInit(DMA_HandleTypeDef *hdma);

 
HAL_StatusTypeDef HAL_DMA_Start (DMA_HandleTypeDef *hdma, uint32_t SrcAddress, uint32_t DstAddress, uint32_t DataLength);
HAL_StatusTypeDef HAL_DMA_Start_IT(DMA_HandleTypeDef *hdma, uint32_t SrcAddress, uint32_t DstAddress, uint32_t DataLength);
HAL_StatusTypeDef HAL_DMA_Abort(DMA_HandleTypeDef *hdma);
HAL_StatusTypeDef HAL_DMA_PollForTransfer(DMA_HandleTypeDef *hdma, uint32_t CompleteLevel, uint32_t Timeout);
void              HAL_DMA_IRQHandler(DMA_HandleTypeDef *hdma);

 
HAL_DMA_StateTypeDef HAL_DMA_GetState(DMA_HandleTypeDef *hdma);
uint32_t             HAL_DMA_GetError(DMA_HandleTypeDef *hdma);



  



 







 
#line 221 "..\\..\\Inc\\stm32f2xx_hal_conf.h"


#line 1 "..\\..\\Drivers\\STM32F2xx_HAL_Driver\\Inc\\stm32f2xx_hal_cortex.h"



































  

 







 
#line 48 "..\\..\\Drivers\\STM32F2xx_HAL_Driver\\Inc\\stm32f2xx_hal_cortex.h"



 



  
 
 



 




 

#line 78 "..\\..\\Drivers\\STM32F2xx_HAL_Driver\\Inc\\stm32f2xx_hal_cortex.h"













 



 






 

 







 
#line 122 "..\\..\\Drivers\\STM32F2xx_HAL_Driver\\Inc\\stm32f2xx_hal_cortex.h"



 

 
 
 
void HAL_NVIC_SetPriorityGrouping(uint32_t PriorityGroup);
void HAL_NVIC_SetPriority(IRQn_Type IRQn, uint32_t PreemptPriority, uint32_t SubPriority);
void HAL_NVIC_EnableIRQ(IRQn_Type IRQn);
void HAL_NVIC_DisableIRQ(IRQn_Type IRQn);
void HAL_NVIC_SystemReset(void);
uint32_t HAL_SYSTICK_Config(uint32_t TicksNumb);

 
uint32_t HAL_NVIC_GetPriorityGrouping(void);
void HAL_NVIC_GetPriority(IRQn_Type IRQn, uint32_t PriorityGroup, uint32_t* pPreemptPriority, uint32_t* pSubPriority);
uint32_t HAL_NVIC_GetPendingIRQ(IRQn_Type IRQn);
void HAL_NVIC_SetPendingIRQ(IRQn_Type IRQn);
void HAL_NVIC_ClearPendingIRQ(IRQn_Type IRQn);
uint32_t HAL_NVIC_GetActive(IRQn_Type IRQn);
void HAL_SYSTICK_CLKSourceConfig(uint32_t CLKSource);
void HAL_SYSTICK_IRQHandler(void);
void HAL_SYSTICK_Callback(void);



  



 
  





 

 
#line 225 "..\\..\\Inc\\stm32f2xx_hal_conf.h"


#line 1 "..\\..\\Drivers\\STM32F2xx_HAL_Driver\\Inc\\stm32f2xx_hal_adc.h"



































 

 







 
#line 48 "..\\..\\Drivers\\STM32F2xx_HAL_Driver\\Inc\\stm32f2xx_hal_adc.h"



 



  

 
   


  
typedef enum
{
  HAL_ADC_STATE_RESET                   = 0x00,     
  HAL_ADC_STATE_READY                   = 0x01,     
  HAL_ADC_STATE_BUSY                    = 0x02,      
  HAL_ADC_STATE_BUSY_REG                = 0x12,     
  HAL_ADC_STATE_BUSY_INJ                = 0x22,     
  HAL_ADC_STATE_BUSY_INJ_REG            = 0x32,     
  HAL_ADC_STATE_TIMEOUT                 = 0x03,     
  HAL_ADC_STATE_ERROR                   = 0x04,     
  HAL_ADC_STATE_EOC                     = 0x05,     
  HAL_ADC_STATE_EOC_REG                 = 0x15,     
  HAL_ADC_STATE_EOC_INJ                 = 0x25,     
  HAL_ADC_STATE_EOC_INJ_REG             = 0x35,     
  HAL_ADC_STATE_AWD                     = 0x06     

}HAL_ADC_StateTypeDef;



  
typedef struct
{
  uint32_t ClockPrescaler;        

 
  uint32_t Resolution;            
 
  uint32_t DataAlign;             
 
  uint32_t ScanConvMode;          

  
  uint32_t EOCSelection;          

 
  uint32_t ContinuousConvMode;    
 
  uint32_t DMAContinuousRequests; 
  
  uint32_t NbrOfConversion;       

 
  uint32_t DiscontinuousConvMode; 

 
  uint32_t NbrOfDiscConversion;   

 
  uint32_t ExternalTrigConvEdge;  
 
  uint32_t ExternalTrigConv;      
  
}ADC_InitTypeDef;



  
typedef struct
{
  ADC_TypeDef                   *Instance;                    

  ADC_InitTypeDef               Init;                         

  volatile uint32_t                 NbrOfCurrentConversionRank;   

  DMA_HandleTypeDef             *DMA_Handle;                  

  HAL_LockTypeDef               Lock;                         

  volatile HAL_ADC_StateTypeDef     State;                        

  volatile uint32_t                 ErrorCode;                    
}ADC_HandleTypeDef;



  
typedef struct 
{
  uint32_t Channel;        
 
  uint32_t Rank;           
 
  uint32_t SamplingTime;   
 
  uint32_t Offset;          
}ADC_ChannelConfTypeDef;



  
typedef struct
{
  uint32_t WatchdogMode;      
 
  uint32_t HighThreshold;     
      
  uint32_t LowThreshold;      
 
  uint32_t Channel;           

       
  uint32_t ITMode;            

 
  uint32_t WatchdogNumber;     
}ADC_AnalogWDGConfTypeDef;

 



 




  






   




  
#line 201 "..\\..\\Drivers\\STM32F2xx_HAL_Driver\\Inc\\stm32f2xx_hal_adc.h"


  



  
#line 224 "..\\..\\Drivers\\STM32F2xx_HAL_Driver\\Inc\\stm32f2xx_hal_adc.h"

#line 241 "..\\..\\Drivers\\STM32F2xx_HAL_Driver\\Inc\\stm32f2xx_hal_adc.h"


  



  











  



  











  



  
#line 296 "..\\..\\Drivers\\STM32F2xx_HAL_Driver\\Inc\\stm32f2xx_hal_adc.h"

#line 313 "..\\..\\Drivers\\STM32F2xx_HAL_Driver\\Inc\\stm32f2xx_hal_adc.h"


  



  







  



  
#line 351 "..\\..\\Drivers\\STM32F2xx_HAL_Driver\\Inc\\stm32f2xx_hal_adc.h"





#line 375 "..\\..\\Drivers\\STM32F2xx_HAL_Driver\\Inc\\stm32f2xx_hal_adc.h"


  



  
#line 390 "..\\..\\Drivers\\STM32F2xx_HAL_Driver\\Inc\\stm32f2xx_hal_adc.h"

#line 399 "..\\..\\Drivers\\STM32F2xx_HAL_Driver\\Inc\\stm32f2xx_hal_adc.h"


  

  

  









  



  







 



  
#line 439 "..\\..\\Drivers\\STM32F2xx_HAL_Driver\\Inc\\stm32f2xx_hal_adc.h"

#line 447 "..\\..\\Drivers\\STM32F2xx_HAL_Driver\\Inc\\stm32f2xx_hal_adc.h"


  
    


  









  
    


  
#line 474 "..\\..\\Drivers\\STM32F2xx_HAL_Driver\\Inc\\stm32f2xx_hal_adc.h"


  



  









 



  



  



  



  



  



  



  



  



  







    
  


  

 




 






 






 







 







 







 







 







 






 






 






 






 






 







 







 






 







 







 






 


 
#line 1 "..\\..\\Drivers\\STM32F2xx_HAL_Driver\\Inc\\stm32f2xx_hal_adc_ex.h"



































 

 







 
#line 48 "..\\..\\Drivers\\STM32F2xx_HAL_Driver\\Inc\\stm32f2xx_hal_adc_ex.h"



 



  

 
   


  
typedef struct 
{
  uint32_t InjectedChannel;                
  
  uint32_t InjectedRank;                   
  
  uint32_t InjectedSamplingTime;           
 
  uint32_t InjectedOffset;                 
 
  uint32_t InjectedNbrOfConversion;        

 
  uint32_t AutoInjectedConv;               
 
  uint32_t InjectedDiscontinuousConvMode;  
 
  uint32_t ExternalTrigInjecConvEdge;      
 
  uint32_t ExternalTrigInjecConv;          
 
}ADC_InjectionConfTypeDef;



  
typedef struct
{
  uint32_t Mode;              
 
  uint32_t DMAAccessMode;     
 
  uint32_t TwoSamplingDelay;  
 
}ADC_MultiModeTypeDef;

 



 




  
#line 121 "..\\..\\Drivers\\STM32F2xx_HAL_Driver\\Inc\\stm32f2xx_hal_adc_ex.h"

#line 135 "..\\..\\Drivers\\STM32F2xx_HAL_Driver\\Inc\\stm32f2xx_hal_adc_ex.h"


  



  











  



  











  



  
#line 190 "..\\..\\Drivers\\STM32F2xx_HAL_Driver\\Inc\\stm32f2xx_hal_adc_ex.h"

#line 207 "..\\..\\Drivers\\STM32F2xx_HAL_Driver\\Inc\\stm32f2xx_hal_adc_ex.h"


  



  







 
    


  



  



  



  
 


  

 







 



 

 
HAL_StatusTypeDef HAL_ADCEx_InjectedStop(ADC_HandleTypeDef* hadc);
HAL_StatusTypeDef HAL_ADCEx_InjectedPollForConversion(ADC_HandleTypeDef* hadc, uint32_t Timeout);
HAL_StatusTypeDef HAL_ADCEx_InjectedStop_IT(ADC_HandleTypeDef* hadc);
HAL_StatusTypeDef HAL_ADCEx_InjectedStart(ADC_HandleTypeDef* hadc);
HAL_StatusTypeDef HAL_ADCEx_InjectedStart_IT(ADC_HandleTypeDef* hadc);
uint32_t          HAL_ADCEx_InjectedGetValue(ADC_HandleTypeDef* hadc, uint32_t InjectedRank);
HAL_StatusTypeDef HAL_ADCEx_MultiModeStart_DMA(ADC_HandleTypeDef* hadc, uint32_t* pData, uint32_t Length);
HAL_StatusTypeDef HAL_ADCEx_MultiModeStop_DMA(ADC_HandleTypeDef* hadc);
uint32_t          HAL_ADCEx_MultiModeGetValue(ADC_HandleTypeDef* hadc);
void       HAL_ADCEx_InjectedConvCpltCallback(ADC_HandleTypeDef* hadc);

 
HAL_StatusTypeDef HAL_ADCEx_InjectedConfigChannel(ADC_HandleTypeDef* hadc,ADC_InjectionConfTypeDef* sConfigInjected);
HAL_StatusTypeDef HAL_ADCEx_MultiModeConfigChannel(ADC_HandleTypeDef* hadc, ADC_MultiModeTypeDef* multimode);



  



 








 
#line 685 "..\\..\\Drivers\\STM32F2xx_HAL_Driver\\Inc\\stm32f2xx_hal_adc.h"

 
 
HAL_StatusTypeDef HAL_ADC_Init(ADC_HandleTypeDef* hadc);
HAL_StatusTypeDef HAL_ADC_DeInit(ADC_HandleTypeDef *hadc);
void       HAL_ADC_MspInit(ADC_HandleTypeDef* hadc);
void       HAL_ADC_MspDeInit(ADC_HandleTypeDef* hadc);

 
HAL_StatusTypeDef HAL_ADC_Start(ADC_HandleTypeDef* hadc);
HAL_StatusTypeDef HAL_ADC_Stop(ADC_HandleTypeDef* hadc);
HAL_StatusTypeDef HAL_ADC_PollForConversion(ADC_HandleTypeDef* hadc, uint32_t Timeout);

HAL_StatusTypeDef HAL_ADC_PollForEvent(ADC_HandleTypeDef* hadc, uint32_t EventType, uint32_t Timeout);

HAL_StatusTypeDef HAL_ADC_Start_IT(ADC_HandleTypeDef* hadc);
HAL_StatusTypeDef HAL_ADC_Stop_IT(ADC_HandleTypeDef* hadc);

void              HAL_ADC_IRQHandler(ADC_HandleTypeDef* hadc);

HAL_StatusTypeDef HAL_ADC_Start_DMA(ADC_HandleTypeDef* hadc, uint32_t* pData, uint32_t Length);
HAL_StatusTypeDef HAL_ADC_Stop_DMA(ADC_HandleTypeDef* hadc);

uint32_t          HAL_ADC_GetValue(ADC_HandleTypeDef* hadc);

void       HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc);
void       HAL_ADC_ConvHalfCpltCallback(ADC_HandleTypeDef* hadc);
void       HAL_ADC_LevelOutOfWindowCallback(ADC_HandleTypeDef* hadc);
void       HAL_ADC_ErrorCallback(ADC_HandleTypeDef *hadc);

 
HAL_StatusTypeDef HAL_ADC_ConfigChannel(ADC_HandleTypeDef* hadc, ADC_ChannelConfTypeDef* sConfig);
HAL_StatusTypeDef HAL_ADC_AnalogWDGConfig(ADC_HandleTypeDef* hadc, ADC_AnalogWDGConfTypeDef* AnalogWDGConfig);

 
HAL_ADC_StateTypeDef HAL_ADC_GetState(ADC_HandleTypeDef* hadc);
uint32_t             HAL_ADC_GetError(ADC_HandleTypeDef *hadc);



  



 








 
#line 229 "..\\..\\Inc\\stm32f2xx_hal_conf.h"


#line 1 "..\\..\\Drivers\\STM32F2xx_HAL_Driver\\Inc\\stm32f2xx_hal_can.h"



































 

 







 
#line 48 "..\\..\\Drivers\\STM32F2xx_HAL_Driver\\Inc\\stm32f2xx_hal_can.h"



 



 

 



  
typedef enum
{
  HAL_CAN_STATE_RESET             = 0x00,   
  HAL_CAN_STATE_READY             = 0x01,     
  HAL_CAN_STATE_BUSY              = 0x02,        
  HAL_CAN_STATE_BUSY_TX           = 0x12,      
  HAL_CAN_STATE_BUSY_RX           = 0x22,    
  HAL_CAN_STATE_BUSY_TX_RX        = 0x32,   
  HAL_CAN_STATE_TIMEOUT           = 0x03,   
  HAL_CAN_STATE_ERROR             = 0x04      

}HAL_CAN_StateTypeDef;



 
typedef struct
{
  uint32_t Prescaler;  
 
  
  uint32_t Mode;       
 

  uint32_t SJW;        


 

  uint32_t BS1;        
 
                             
  uint32_t BS2;        
 
  
  uint32_t TTCM;       
 
  
  uint32_t ABOM;       
 

  uint32_t AWUM;       
 

  uint32_t NART;       
 

  uint32_t RFLM;       
 

  uint32_t TXFP;       
 
}CAN_InitTypeDef;



 
typedef struct
{
  uint32_t FilterIdHigh;          

  
                                              
  uint32_t FilterIdLow;           

  

  uint32_t FilterMaskIdHigh;      


  

  uint32_t FilterMaskIdLow;       


  

  uint32_t FilterFIFOAssignment;  
 
  
  uint32_t FilterNumber;          
 

  uint32_t FilterMode;            
 

  uint32_t FilterScale;           
 

  uint32_t FilterActivation;      
 
                                       
  uint32_t BankNumber;            
  
  
}CAN_FilterConfTypeDef;



 
typedef struct
{
  uint32_t StdId;    
  
                        
  uint32_t ExtId;    
  
                        
  uint32_t IDE;      
 

  uint32_t RTR;      
                          

  uint32_t DLC;      
 

  uint32_t Data[8];  
 
   
}CanTxMsgTypeDef;



 
typedef struct
{
  uint32_t StdId;       
  

  uint32_t ExtId;       
  

  uint32_t IDE;         
 

  uint32_t RTR;         
 

  uint32_t DLC;         
 

  uint32_t Data[8];     
 

  uint32_t FMI;         
 
                        
  uint32_t FIFONumber;  
 
                       
}CanRxMsgTypeDef;



  
typedef struct
{
  CAN_TypeDef                 *Instance;   
  
  CAN_InitTypeDef             Init;        
  
  CanTxMsgTypeDef*            pTxMsg;      

  CanRxMsgTypeDef*            pRxMsg;      
  
  volatile HAL_CAN_StateTypeDef   State;       
  
  HAL_LockTypeDef             Lock;        
  
  volatile uint32_t               ErrorCode;   
  
}CAN_HandleTypeDef;

 



 



 
#line 255 "..\\..\\Drivers\\STM32F2xx_HAL_Driver\\Inc\\stm32f2xx_hal_can.h"


   




 




 



 











 




 









 



 
#line 319 "..\\..\\Drivers\\STM32F2xx_HAL_Driver\\Inc\\stm32f2xx_hal_can.h"




 



 
#line 336 "..\\..\\Drivers\\STM32F2xx_HAL_Driver\\Inc\\stm32f2xx_hal_can.h"




 



 



 



 



 



 







 



 







 



 






 




 



 



 



 






 



 






 



 






 



 







 



 






 



 

 

 

 
#line 480 "..\\..\\Drivers\\STM32F2xx_HAL_Driver\\Inc\\stm32f2xx_hal_can.h"

 






 




 

 




#line 506 "..\\..\\Drivers\\STM32F2xx_HAL_Driver\\Inc\\stm32f2xx_hal_can.h"
                               








 

  


  


 
#line 530 "..\\..\\Drivers\\STM32F2xx_HAL_Driver\\Inc\\stm32f2xx_hal_can.h"

 



 






 




#line 554 "..\\..\\Drivers\\STM32F2xx_HAL_Driver\\Inc\\stm32f2xx_hal_can.h"

#line 561 "..\\..\\Drivers\\STM32F2xx_HAL_Driver\\Inc\\stm32f2xx_hal_can.h"


 

 

 


 






 

 






 







 







 





























 
#line 640 "..\\..\\Drivers\\STM32F2xx_HAL_Driver\\Inc\\stm32f2xx_hal_can.h"



























 
#line 674 "..\\..\\Drivers\\STM32F2xx_HAL_Driver\\Inc\\stm32f2xx_hal_can.h"









 







 












 








 













 


   
   

  
HAL_StatusTypeDef HAL_CAN_Init(CAN_HandleTypeDef* hcan);
HAL_StatusTypeDef HAL_CAN_ConfigFilter(CAN_HandleTypeDef* hcan, CAN_FilterConfTypeDef* sFilterConfig);
HAL_StatusTypeDef HAL_CAN_DeInit(CAN_HandleTypeDef* hcan);
void HAL_CAN_MspInit(CAN_HandleTypeDef* hcan);
void HAL_CAN_MspDeInit(CAN_HandleTypeDef* hcan);

 
HAL_StatusTypeDef HAL_CAN_Transmit(CAN_HandleTypeDef *hcan, uint32_t Timeout);
HAL_StatusTypeDef HAL_CAN_Transmit_IT(CAN_HandleTypeDef *hcan);
HAL_StatusTypeDef HAL_CAN_Receive(CAN_HandleTypeDef *hcan, uint8_t FIFONumber, uint32_t Timeout);
HAL_StatusTypeDef HAL_CAN_Receive_IT(CAN_HandleTypeDef *hcan, uint8_t FIFONumber);
HAL_StatusTypeDef HAL_CAN_Sleep(CAN_HandleTypeDef *hcan);
HAL_StatusTypeDef HAL_CAN_WakeUp(CAN_HandleTypeDef *hcan);

 
void HAL_CAN_IRQHandler(CAN_HandleTypeDef* hcan);
uint32_t HAL_CAN_GetError(CAN_HandleTypeDef *hcan);
HAL_CAN_StateTypeDef HAL_CAN_GetState(CAN_HandleTypeDef* hcan);

void HAL_CAN_TxCpltCallback(CAN_HandleTypeDef* hcan);
void HAL_CAN_RxCpltCallback(CAN_HandleTypeDef* hcan);
void HAL_CAN_ErrorCallback(CAN_HandleTypeDef *hcan);



 



 








 
#line 233 "..\\..\\Inc\\stm32f2xx_hal_conf.h"






















#line 1 "..\\..\\Drivers\\STM32F2xx_HAL_Driver\\Inc\\stm32f2xx_hal_flash.h"



































  

 







 
#line 48 "..\\..\\Drivers\\STM32F2xx_HAL_Driver\\Inc\\stm32f2xx_hal_flash.h"



 



  

  


 
typedef enum
{ 
  FLASH_ERROR_RD =  0x01,
  FLASH_ERROR_PGS = 0x02,
  FLASH_ERROR_PGP = 0x04,
  FLASH_ERROR_PGA = 0x08,
  FLASH_ERROR_WRP = 0x10,
  FLASH_ERROR_OPERATION = 0x20
}FLASH_ErrorTypeDef;



 
typedef enum 
{
  FLASH_PROC_NONE = 0, 
  FLASH_PROC_SECTERASE,
  FLASH_PROC_MASSERASE,
  FLASH_PROC_PROGRAM
} FLASH_ProcedureTypeDef;




 
typedef struct
{
  volatile FLASH_ProcedureTypeDef ProcedureOnGoing;    
  
  volatile uint32_t               NbSectorsToErase;    
  
  volatile uint8_t                VoltageForErase;     
  
  volatile uint32_t               Sector;              
  
  volatile uint32_t               Address;             
  
  HAL_LockTypeDef             Lock;                

  volatile FLASH_ErrorTypeDef     ErrorCode;           

}FLASH_ProcessTypeDef;



  
 



   





  












 




  
#line 144 "..\\..\\Drivers\\STM32F2xx_HAL_Driver\\Inc\\stm32f2xx_hal_flash.h"



 
  



  





   



 







  



  







  



  



  



  



  



  




  
  
 






  





  





  





  





  





  





  






 






 









   









   















 














 


 
#line 1 "..\\..\\Drivers\\STM32F2xx_HAL_Driver\\Inc\\stm32f2xx_hal_flash_ex.h"



































  

 







 
#line 48 "..\\..\\Drivers\\STM32F2xx_HAL_Driver\\Inc\\stm32f2xx_hal_flash_ex.h"



 



  

  



 
typedef struct
{
  uint32_t TypeErase;   
 

  uint32_t Sector;      
         
  
  uint32_t NbSectors;   
            
                                                          
  uint32_t VoltageRange;
         
  
} FLASH_EraseInitTypeDef;



 
typedef struct
{
  uint32_t OptionType;   
 

  uint32_t WRPState;     
 

  uint32_t WRPSector;         
 

  uint32_t RDPLevel;     
 

  uint32_t BORLevel;     
 

  uint8_t  USERConfig;   
 

} FLASH_OBProgramInitTypeDef;

 



 



  








 
  


  









                              


 
  


  








 
  


  









 
  


 


  
 





  
  


  





  
  


  





  




  





     



   
#line 220 "..\\..\\Drivers\\STM32F2xx_HAL_Driver\\Inc\\stm32f2xx_hal_flash_ex.h"


 



 
#line 235 "..\\..\\Drivers\\STM32F2xx_HAL_Driver\\Inc\\stm32f2xx_hal_flash_ex.h"

#line 244 "..\\..\\Drivers\\STM32F2xx_HAL_Driver\\Inc\\stm32f2xx_hal_flash_ex.h"


  
  


 
#line 263 "..\\..\\Drivers\\STM32F2xx_HAL_Driver\\Inc\\stm32f2xx_hal_flash_ex.h"



#line 272 "..\\..\\Drivers\\STM32F2xx_HAL_Driver\\Inc\\stm32f2xx_hal_flash_ex.h"








  



 
#line 298 "..\\..\\Drivers\\STM32F2xx_HAL_Driver\\Inc\\stm32f2xx_hal_flash_ex.h"




  

 

 

 
HAL_StatusTypeDef HAL_FLASHEx_Erase(FLASH_EraseInitTypeDef *pEraseInit, uint32_t *SectorError);
HAL_StatusTypeDef HAL_FLASHEx_Erase_IT(FLASH_EraseInitTypeDef *pEraseInit);
HAL_StatusTypeDef HAL_FLASHEx_OBProgram(FLASH_OBProgramInitTypeDef *pOBInit);
void              HAL_FLASHEx_OBGetConfig(FLASH_OBProgramInitTypeDef *pOBInit);

void FLASH_Erase_Sector(uint32_t Sector, uint8_t VoltageRange);



  



 



   






 
#line 322 "..\\..\\Drivers\\STM32F2xx_HAL_Driver\\Inc\\stm32f2xx_hal_flash.h"

 
 
HAL_StatusTypeDef   HAL_FLASH_Program(uint32_t TypeProgram, uint32_t Address, uint64_t Data);
HAL_StatusTypeDef   HAL_FLASH_Program_IT(uint32_t TypeProgram, uint32_t Address, uint64_t Data);
 
void                HAL_FLASH_IRQHandler(void);
  
void         HAL_FLASH_EndOfOperationCallback(uint32_t ReturnValue);
void         HAL_FLASH_OperationErrorCallback(uint32_t ReturnValue);

 
HAL_StatusTypeDef   HAL_FLASH_Unlock(void);
HAL_StatusTypeDef   HAL_FLASH_Lock(void);
HAL_StatusTypeDef   HAL_FLASH_OB_Unlock(void);
HAL_StatusTypeDef   HAL_FLASH_OB_Lock(void);
 
HAL_StatusTypeDef   HAL_FLASH_OB_Launch(void);

 
FLASH_ErrorTypeDef  HAL_FLASH_GetError(void);

HAL_StatusTypeDef FLASH_WaitForLastOperation(uint32_t Timeout);



  



 







 
#line 257 "..\\..\\Inc\\stm32f2xx_hal_conf.h"










#line 1 "..\\..\\Drivers\\STM32F2xx_HAL_Driver\\Inc\\stm32f2xx_hal_nand.h"



































  

 







#line 1 "..\\..\\Drivers\\STM32F2xx_HAL_Driver\\Inc\\stm32f2xx_ll_fsmc.h"



































  

 







 
#line 48 "..\\..\\Drivers\\STM32F2xx_HAL_Driver\\Inc\\stm32f2xx_ll_fsmc.h"



 



  

  












  
typedef struct
{
  uint32_t NSBank;                       
   
                                                    
  uint32_t DataAddressMux;               

 
  
  uint32_t MemoryType;                   

 
                                              
  uint32_t MemoryDataWidth;              
 
  
  uint32_t BurstAccessMode;              

 
                                               
  uint32_t WaitSignalPolarity;           

 
  
  uint32_t WrapMode;                     

 
  
  uint32_t WaitSignalActive;             


 
  
  uint32_t WriteOperation;               
 
  
  uint32_t WaitSignal;                   

 
  
  uint32_t ExtendedMode;                 
 
  
  uint32_t AsynchronousWait;             

 
  
  uint32_t WriteBurst;                   
                                      

}FSMC_NORSRAM_InitTypeDef;




 
typedef struct
{
  uint32_t AddressSetupTime;             


 
  
  uint32_t AddressHoldTime;              


 
  
  uint32_t DataSetupTime;                



 
  
  uint32_t BusTurnAroundDuration;        


 
  
  uint32_t CLKDivision;                  


 
  
  uint32_t DataLatency;                  





 
  
  uint32_t AccessMode;                   
 
  
}FSMC_NORSRAM_TimingTypeDef;



  
typedef struct
{
  uint32_t NandBank;               
            
  
  uint32_t Waitfeature;            
 
  
  uint32_t MemoryDataWidth;        
 
  
  uint32_t EccComputation;         
 
  
  uint32_t ECCPageSize;            
 
  
  uint32_t TCLRSetupTime;          

 
  
  uint32_t TARSetupTime;           

 
                                     
}FSMC_NAND_InitTypeDef;  



 
typedef struct
{
  uint32_t SetupTime;            



 
  
  uint32_t WaitSetupTime;        



 
  
  uint32_t HoldSetupTime;        




 
  
  uint32_t HiZSetupTime;         



 
  
}FSMC_NAND_PCC_TimingTypeDef;



  
typedef struct
{
  uint32_t Waitfeature;            
 
  
  uint32_t TCLRSetupTime;          

 
  
  uint32_t TARSetupTime;           

 
                                     
}FSMC_PCCARD_InitTypeDef;  

 



  
  


 











 



 








 



 











 



 










 



 




 



 








 
    



 







 



 







 



 







 



 







 



 








 



 







 



 








   



 









 
  


 









 
  


 



 



 



 



 



 



 



 



 



 



 



   



 











 
    


   



 



   








 



 







 



 




 



 







 



 







 



 
#line 590 "..\\..\\Drivers\\STM32F2xx_HAL_Driver\\Inc\\stm32f2xx_ll_fsmc.h"

#line 597 "..\\..\\Drivers\\STM32F2xx_HAL_Driver\\Inc\\stm32f2xx_ll_fsmc.h"


 



 



 



 



 



 



 



 



 



 



 



 



   
    


   
  



 




 



 




 
  
  

 




   



 




  
   



   












 
    



  











                               



 


 





 
 





  







  




  
  




 
 





   


                                         






                                           


                                         
                                        


  
  



 





  






  




 
  



  











   













 


                                                                                                                             











 













 











  











  












 












 




  

 

 
 
HAL_StatusTypeDef  FSMC_NORSRAM_Init(FSMC_Bank1_TypeDef *Device, FSMC_NORSRAM_InitTypeDef *Init);
HAL_StatusTypeDef  FSMC_NORSRAM_Timing_Init(FSMC_Bank1_TypeDef *Device, FSMC_NORSRAM_TimingTypeDef *Timing, uint32_t Bank);
HAL_StatusTypeDef  FSMC_NORSRAM_Extended_Timing_Init(FSMC_Bank1E_TypeDef *Device, FSMC_NORSRAM_TimingTypeDef *Timing, uint32_t Bank, uint32_t ExtendedMode);
HAL_StatusTypeDef  FSMC_NORSRAM_DeInit(FSMC_Bank1_TypeDef *Device, FSMC_Bank1E_TypeDef *ExDevice, uint32_t Bank);

 
HAL_StatusTypeDef  FSMC_NORSRAM_WriteOperation_Enable(FSMC_Bank1_TypeDef *Device, uint32_t Bank);
HAL_StatusTypeDef  FSMC_NORSRAM_WriteOperation_Disable(FSMC_Bank1_TypeDef *Device, uint32_t Bank);

 
 
HAL_StatusTypeDef  FSMC_NAND_Init(FSMC_Bank2_3_TypeDef *Device, FSMC_NAND_InitTypeDef *Init);
HAL_StatusTypeDef  FSMC_NAND_CommonSpace_Timing_Init(FSMC_Bank2_3_TypeDef *Device, FSMC_NAND_PCC_TimingTypeDef *Timing, uint32_t Bank);
HAL_StatusTypeDef  FSMC_NAND_AttributeSpace_Timing_Init(FSMC_Bank2_3_TypeDef *Device, FSMC_NAND_PCC_TimingTypeDef *Timing, uint32_t Bank);
HAL_StatusTypeDef  FSMC_NAND_DeInit(FSMC_Bank2_3_TypeDef *Device, uint32_t Bank);

 
HAL_StatusTypeDef  FSMC_NAND_ECC_Enable(FSMC_Bank2_3_TypeDef *Device, uint32_t Bank);
HAL_StatusTypeDef  FSMC_NAND_ECC_Disable(FSMC_Bank2_3_TypeDef *Device, uint32_t Bank);
HAL_StatusTypeDef  FSMC_NAND_GetECC(FSMC_Bank2_3_TypeDef *Device, uint32_t *ECCval, uint32_t Bank, uint32_t Timeout);

 
 
HAL_StatusTypeDef  FSMC_PCCARD_Init(FSMC_Bank4_TypeDef *Device, FSMC_PCCARD_InitTypeDef *Init);
HAL_StatusTypeDef  FSMC_PCCARD_CommonSpace_Timing_Init(FSMC_Bank4_TypeDef *Device, FSMC_NAND_PCC_TimingTypeDef *Timing);
HAL_StatusTypeDef  FSMC_PCCARD_AttributeSpace_Timing_Init(FSMC_Bank4_TypeDef *Device, FSMC_NAND_PCC_TimingTypeDef *Timing);
HAL_StatusTypeDef  FSMC_PCCARD_IOSpace_Timing_Init(FSMC_Bank4_TypeDef *Device, FSMC_NAND_PCC_TimingTypeDef *Timing); 
HAL_StatusTypeDef  FSMC_PCCARD_DeInit(FSMC_Bank4_TypeDef *Device);

 





#line 971 "..\\..\\Drivers\\STM32F2xx_HAL_Driver\\Inc\\stm32f2xx_ll_fsmc.h"








#line 991 "..\\..\\Drivers\\STM32F2xx_HAL_Driver\\Inc\\stm32f2xx_ll_fsmc.h"

#line 1004 "..\\..\\Drivers\\STM32F2xx_HAL_Driver\\Inc\\stm32f2xx_ll_fsmc.h"

























  



 
  






 
#line 47 "..\\..\\Drivers\\STM32F2xx_HAL_Driver\\Inc\\stm32f2xx_hal_nand.h"



 



  

  
 



  
typedef enum
{
  HAL_NAND_STATE_RESET     = 0x00,   
  HAL_NAND_STATE_READY     = 0x01,   
  HAL_NAND_STATE_BUSY      = 0x02,   
  HAL_NAND_STATE_ERROR     = 0x03    

}HAL_NAND_StateTypeDef;
   


 
typedef struct
{
    
  
  uint8_t Maker_Id; 
  
  uint8_t Device_Id;
  
  uint8_t Third_Id;
  
  uint8_t Fourth_Id;
  
}NAND_IDTypeDef;



 
typedef struct 
{
  uint16_t Page;      
  
  uint16_t Zone;     
  
  uint16_t Block;   
 
}NAND_AddressTypedef;



  
typedef struct
{
  uint32_t PageSize;        
  
  uint32_t SpareAreaSize;     
  
  uint32_t BlockSize;         
  
  uint32_t BlockNbr;        
  
  uint32_t ZoneSize;        
   
}NAND_InfoTypeDef;



    
typedef struct
{
  FSMC_Bank2_3_TypeDef             *Instance;   
  
  FSMC_NAND_InitTypeDef         Init;        

  HAL_LockTypeDef              Lock;         

  volatile HAL_NAND_StateTypeDef   State;       
  
  NAND_InfoTypeDef             Info;        
  
}NAND_HandleTypeDef;


 


  











 
#line 158 "..\\..\\Drivers\\STM32F2xx_HAL_Driver\\Inc\\stm32f2xx_hal_nand.h"



 
  
 





 






 





 

 
HAL_StatusTypeDef  HAL_NAND_Init(NAND_HandleTypeDef *hnand, FSMC_NAND_PCC_TimingTypeDef *ComSpace_Timing, FSMC_NAND_PCC_TimingTypeDef *AttSpace_Timing);
HAL_StatusTypeDef  HAL_NAND_DeInit(NAND_HandleTypeDef *hnand);
void        HAL_NAND_MspInit(NAND_HandleTypeDef *hnand);
void        HAL_NAND_MspDeInit(NAND_HandleTypeDef *hnand);
void               HAL_NAND_IRQHandler(NAND_HandleTypeDef *hnand);
void        HAL_NAND_ITCallback(NAND_HandleTypeDef *hnand);

 
HAL_StatusTypeDef  HAL_NAND_Read_ID(NAND_HandleTypeDef *hnand, NAND_IDTypeDef *pNAND_ID);
HAL_StatusTypeDef  HAL_NAND_Reset(NAND_HandleTypeDef *hnand);
HAL_StatusTypeDef  HAL_NAND_Read_Page(NAND_HandleTypeDef *hnand, NAND_AddressTypedef *pAddress, uint8_t *pBuffer, uint32_t NumPageToRead);
HAL_StatusTypeDef  HAL_NAND_Write_Page(NAND_HandleTypeDef *hnand, NAND_AddressTypedef *pAddress, uint8_t *pBuffer, uint32_t NumPageToWrite);
HAL_StatusTypeDef  HAL_NAND_Read_SpareArea(NAND_HandleTypeDef *hnand, NAND_AddressTypedef *pAddress, uint8_t *pBuffer, uint32_t NumSpareAreaToRead);
HAL_StatusTypeDef  HAL_NAND_Write_SpareArea(NAND_HandleTypeDef *hnand, NAND_AddressTypedef *pAddress, uint8_t *pBuffer, uint32_t NumSpareAreaTowrite);
HAL_StatusTypeDef  HAL_NAND_Erase_Block(NAND_HandleTypeDef *hnand, NAND_AddressTypedef *pAddress);
uint32_t           HAL_NAND_Read_Status(NAND_HandleTypeDef *hnand);
uint32_t           HAL_NAND_Address_Inc(NAND_HandleTypeDef *hnand, NAND_AddressTypedef *pAddress);

 
HAL_StatusTypeDef  HAL_NAND_ECC_Enable(NAND_HandleTypeDef *hnand);
HAL_StatusTypeDef  HAL_NAND_ECC_Disable(NAND_HandleTypeDef *hnand);
HAL_StatusTypeDef  HAL_NAND_GetECC(NAND_HandleTypeDef *hnand, uint32_t *ECCval, uint32_t Timeout);

 
HAL_NAND_StateTypeDef HAL_NAND_GetState(NAND_HandleTypeDef *hnand);
uint32_t HAL_NAND_Read_Status(NAND_HandleTypeDef *hnand);



  



  







 
#line 269 "..\\..\\Inc\\stm32f2xx_hal_conf.h"






















#line 1 "..\\..\\Drivers\\STM32F2xx_HAL_Driver\\Inc\\stm32f2xx_hal_pwr.h"



































  

 







 
#line 48 "..\\..\\Drivers\\STM32F2xx_HAL_Driver\\Inc\\stm32f2xx_hal_pwr.h"



 



  

 


 
typedef struct
{
  uint32_t PVDLevel;   
 

  uint32_t Mode;      
 
}PWR_PVDTypeDef;

 
 


 
 




 



 



 



 
 




 


   


 
 


 





 



  
#line 131 "..\\..\\Drivers\\STM32F2xx_HAL_Driver\\Inc\\stm32f2xx_hal_pwr.h"


    
 


 
#line 144 "..\\..\\Drivers\\STM32F2xx_HAL_Driver\\Inc\\stm32f2xx_hal_pwr.h"


  



 







 
    


 





 



 





 



 







 



  
  
 


 
  


















 







 









 








 








 








 




  

 
#line 1 "..\\..\\Drivers\\STM32F2xx_HAL_Driver\\Inc\\stm32f2xx_hal_pwr_ex.h"



































  

 







 
#line 48 "..\\..\\Drivers\\STM32F2xx_HAL_Driver\\Inc\\stm32f2xx_hal_pwr_ex.h"



 



  

  
 
 
 
void              HAL_PWREx_EnableFlashPowerDown(void);
void              HAL_PWREx_DisableFlashPowerDown(void); 
HAL_StatusTypeDef HAL_PWREx_EnableBkUpReg(void);
HAL_StatusTypeDef HAL_PWREx_DisableBkUpReg(void); 



  



 
  







 
#line 273 "..\\..\\Drivers\\STM32F2xx_HAL_Driver\\Inc\\stm32f2xx_hal_pwr.h"

 

 
void        HAL_PWR_DeInit(void);
void        HAL_PWR_EnableBkUpAccess(void);
void        HAL_PWR_DisableBkUpAccess(void);

 
void        HAL_PWR_PVDConfig(PWR_PVDTypeDef *sConfigPVD);
void        HAL_PWR_EnablePVD(void);
void        HAL_PWR_DisablePVD(void);
void        HAL_PWR_EnableWakeUpPin(uint32_t WakeUpPinx);
void        HAL_PWR_DisableWakeUpPin(uint32_t WakeUpPinx);

void        HAL_PWR_EnterSTOPMode(uint32_t Regulator, uint8_t STOPEntry);
void        HAL_PWR_EnterSLEEPMode(uint32_t Regulator, uint8_t SLEEPEntry);
void        HAL_PWR_EnterSTANDBYMode(void);

void        HAL_PWR_PVD_IRQHandler(void);
void        HAL_PWR_PVDCallback(void);




  



 
  







 
#line 293 "..\\..\\Inc\\stm32f2xx_hal_conf.h"


#line 1 "..\\..\\Drivers\\STM32F2xx_HAL_Driver\\Inc\\stm32f2xx_hal_rng.h"



































  

 







 
#line 48 "..\\..\\Drivers\\STM32F2xx_HAL_Driver\\Inc\\stm32f2xx_hal_rng.h"



 



  

  



  
typedef enum
{
  HAL_RNG_STATE_RESET     = 0x00,   
  HAL_RNG_STATE_READY     = 0x01,   
  HAL_RNG_STATE_BUSY      = 0x02,    
  HAL_RNG_STATE_TIMEOUT   = 0x03,   
  HAL_RNG_STATE_ERROR     = 0x04    
    
}HAL_RNG_StateTypeDef;



  
typedef struct
{
  RNG_TypeDef                 *Instance;    
  
  HAL_LockTypeDef             Lock;        
  
  volatile HAL_RNG_StateTypeDef   State;       
  
}RNG_HandleTypeDef;

 



 



  







 




  









 



  
  
 





 






 







 







 

    




 

    




 











 


 

 
HAL_StatusTypeDef HAL_RNG_Init(RNG_HandleTypeDef *hrng);
HAL_StatusTypeDef HAL_RNG_DeInit (RNG_HandleTypeDef *hrng);
void HAL_RNG_MspInit(RNG_HandleTypeDef *hrng);
void HAL_RNG_MspDeInit(RNG_HandleTypeDef *hrng);

 
uint32_t HAL_RNG_GetRandomNumber(RNG_HandleTypeDef *hrng);
uint32_t HAL_RNG_GetRandomNumber_IT(RNG_HandleTypeDef *hrng);
void HAL_RNG_IRQHandler(RNG_HandleTypeDef *hrng);
void HAL_RNG_ReadyCallback(RNG_HandleTypeDef* hrng);
void HAL_RNG_ErrorCallback(RNG_HandleTypeDef *hrng);

 
HAL_RNG_StateTypeDef HAL_RNG_GetState(RNG_HandleTypeDef *hrng);



  



  







 
#line 297 "..\\..\\Inc\\stm32f2xx_hal_conf.h"










#line 1 "..\\..\\Drivers\\STM32F2xx_HAL_Driver\\Inc\\stm32f2xx_hal_spi.h"



































  

 







 
#line 48 "..\\..\\Drivers\\STM32F2xx_HAL_Driver\\Inc\\stm32f2xx_hal_spi.h"



 



 

 



 
typedef struct
{
  uint32_t Mode;               
 

  uint32_t Direction;          
 

  uint32_t DataSize;           
 

  uint32_t CLKPolarity;        
 

  uint32_t CLKPhase;           
 

  uint32_t NSS;                

 

  uint32_t BaudRatePrescaler;  



 

  uint32_t FirstBit;           
 

  uint32_t TIMode;             
 

  uint32_t CRCCalculation;     
 

  uint32_t CRCPolynomial;      
 

}SPI_InitTypeDef;



 
typedef enum
{
  HAL_SPI_STATE_RESET      = 0x00,   
  HAL_SPI_STATE_READY      = 0x01,   
  HAL_SPI_STATE_BUSY       = 0x02,   
  HAL_SPI_STATE_BUSY_TX    = 0x12,   
  HAL_SPI_STATE_BUSY_RX    = 0x22,   
  HAL_SPI_STATE_BUSY_TX_RX = 0x32,   
  HAL_SPI_STATE_ERROR      = 0x03    
    
}HAL_SPI_StateTypeDef;



  
typedef enum
{
  HAL_SPI_ERROR_NONE      = 0x00,     
  HAL_SPI_ERROR_MODF      = 0x01,     
  HAL_SPI_ERROR_CRC       = 0x02,     
  HAL_SPI_ERROR_OVR       = 0x04,     
  HAL_SPI_ERROR_FRE       = 0x08,     
  HAL_SPI_ERROR_DMA       = 0x10,     
  HAL_SPI_ERROR_FLAG      = 0x20      

}HAL_SPI_ErrorTypeDef;



 
typedef struct __SPI_HandleTypeDef
{
  SPI_TypeDef                *Instance;     

  SPI_InitTypeDef            Init;          

  uint8_t                    *pTxBuffPtr;   

  uint16_t                   TxXferSize;    
  
  uint16_t                   TxXferCount;   

  uint8_t                    *pRxBuffPtr;   

  uint16_t                   RxXferSize;    

  uint16_t                   RxXferCount;   

  DMA_HandleTypeDef          *hdmatx;       

  DMA_HandleTypeDef          *hdmarx;       

  void                       (*RxISR)(struct __SPI_HandleTypeDef * hspi);  

  void                       (*TxISR)(struct __SPI_HandleTypeDef * hspi);  

  HAL_LockTypeDef            Lock;          

  volatile HAL_SPI_StateTypeDef  State;         

  volatile HAL_SPI_ErrorTypeDef  ErrorCode;          

}SPI_HandleTypeDef;

 



 



 







 



 















 



 







  



 







 



 







 



 









  



 
#line 269 "..\\..\\Drivers\\STM32F2xx_HAL_Driver\\Inc\\stm32f2xx_hal_spi.h"

#line 278 "..\\..\\Drivers\\STM32F2xx_HAL_Driver\\Inc\\stm32f2xx_hal_spi.h"


  



 







 



 







 



 







 



 





 



 
#line 338 "..\\..\\Drivers\\STM32F2xx_HAL_Driver\\Inc\\stm32f2xx_hal_spi.h"



 



 

 










 












 















 






 






 







 







 














 

 
HAL_StatusTypeDef HAL_SPI_Init(SPI_HandleTypeDef *hspi);
HAL_StatusTypeDef HAL_SPI_DeInit (SPI_HandleTypeDef *hspi);
void HAL_SPI_MspInit(SPI_HandleTypeDef *hspi);
void HAL_SPI_MspDeInit(SPI_HandleTypeDef *hspi);

 
HAL_StatusTypeDef HAL_SPI_Transmit(SPI_HandleTypeDef *hspi, uint8_t *pData, uint16_t Size, uint32_t Timeout);
HAL_StatusTypeDef HAL_SPI_Receive(SPI_HandleTypeDef *hspi, uint8_t *pData, uint16_t Size, uint32_t Timeout);
HAL_StatusTypeDef HAL_SPI_TransmitReceive(SPI_HandleTypeDef *hspi, uint8_t *pTxData, uint8_t *pRxData, uint16_t Size, uint32_t Timeout);
HAL_StatusTypeDef HAL_SPI_Transmit_IT(SPI_HandleTypeDef *hspi, uint8_t *pData, uint16_t Size);
HAL_StatusTypeDef HAL_SPI_Receive_IT(SPI_HandleTypeDef *hspi, uint8_t *pData, uint16_t Size);
HAL_StatusTypeDef HAL_SPI_TransmitReceive_IT(SPI_HandleTypeDef *hspi, uint8_t *pTxData, uint8_t *pRxData, uint16_t Size);
HAL_StatusTypeDef HAL_SPI_Transmit_DMA(SPI_HandleTypeDef *hspi, uint8_t *pData, uint16_t Size);
HAL_StatusTypeDef HAL_SPI_Receive_DMA(SPI_HandleTypeDef *hspi, uint8_t *pData, uint16_t Size);
HAL_StatusTypeDef HAL_SPI_TransmitReceive_DMA(SPI_HandleTypeDef *hspi, uint8_t *pTxData, uint8_t *pRxData, uint16_t Size);
void HAL_SPI_IRQHandler(SPI_HandleTypeDef *hspi);
void HAL_SPI_TxCpltCallback(SPI_HandleTypeDef *hspi);
void HAL_SPI_RxCpltCallback(SPI_HandleTypeDef *hspi);
void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi);
void HAL_SPI_ErrorCallback(SPI_HandleTypeDef *hspi);

 
HAL_SPI_StateTypeDef HAL_SPI_GetState(SPI_HandleTypeDef *hspi);
HAL_SPI_ErrorTypeDef HAL_SPI_GetError(SPI_HandleTypeDef *hspi);



  



 
  






 
#line 309 "..\\..\\Inc\\stm32f2xx_hal_conf.h"


#line 1 "..\\..\\Drivers\\STM32F2xx_HAL_Driver\\Inc\\stm32f2xx_hal_tim.h"



































  

 







 
#line 48 "..\\..\\Drivers\\STM32F2xx_HAL_Driver\\Inc\\stm32f2xx_hal_tim.h"



 



  

  



 
typedef struct
{
  uint32_t Prescaler;         
 

  uint32_t CounterMode;       
 

  uint32_t Period;            

  

  uint32_t ClockDivision;     
 

  uint32_t RepetitionCounter;  






 
} TIM_Base_InitTypeDef;



 

typedef struct
{                                 
  uint32_t OCMode;        
 

  uint32_t Pulse;         
                           

  uint32_t OCPolarity;    
 

  uint32_t OCNPolarity;   

 
  
  uint32_t OCFastMode;   

 


  uint32_t OCIdleState;   

 

  uint32_t OCNIdleState;  

 
} TIM_OC_InitTypeDef;  



 
typedef struct
{                               
  uint32_t OCMode;        
 

  uint32_t Pulse;         
                           

  uint32_t OCPolarity;    
 

  uint32_t OCNPolarity;   

 

  uint32_t OCIdleState;   

 

  uint32_t OCNIdleState;  

 

  uint32_t ICPolarity;    
 

  uint32_t ICSelection;   
 

  uint32_t ICFilter;      
   
} TIM_OnePulse_InitTypeDef;  




 

typedef struct
{                                  
  uint32_t  ICPolarity;   
 

  uint32_t ICSelection;  
 

  uint32_t ICPrescaler;  
 

  uint32_t ICFilter;     
 
} TIM_IC_InitTypeDef;



 

typedef struct
{
  uint32_t EncoderMode;   
 
                                  
  uint32_t IC1Polarity;   
 

  uint32_t IC1Selection;  
 

  uint32_t IC1Prescaler;  
 

  uint32_t IC1Filter;     
 
                                  
  uint32_t IC2Polarity;   
 

  uint32_t IC2Selection;  
 

  uint32_t IC2Prescaler;  
 

  uint32_t IC2Filter;     
                                  
} TIM_Encoder_InitTypeDef;



  
typedef struct
{
  uint32_t ClockSource;     
  
  uint32_t ClockPolarity;   
 
  uint32_t ClockPrescaler;  
 
  uint32_t ClockFilter;    
                                    
}TIM_ClockConfigTypeDef;



  
typedef struct
{ 
  uint32_t ClearInputState;      
   
  uint32_t ClearInputSource;     
  
  uint32_t ClearInputPolarity;   
 
  uint32_t ClearInputPrescaler;  
 
  uint32_t ClearInputFilter;    
  
}TIM_ClearInputConfigTypeDef;



  
typedef struct {
  uint32_t  SlaveMode;         
  
  uint32_t  InputTrigger;      
 
  uint32_t  TriggerPolarity;   
 
  uint32_t  TriggerPrescaler;  
 
  uint32_t  TriggerFilter;     
   

}TIM_SlaveConfigTypeDef;



  
typedef enum
{
  HAL_TIM_STATE_RESET             = 0x00,     
  HAL_TIM_STATE_READY             = 0x01,     
  HAL_TIM_STATE_BUSY              = 0x02,         
  HAL_TIM_STATE_TIMEOUT           = 0x03,       
  HAL_TIM_STATE_ERROR             = 0x04                                                                                   
}HAL_TIM_StateTypeDef;



  
typedef enum
{
  HAL_TIM_ACTIVE_CHANNEL_1        = 0x01,     
  HAL_TIM_ACTIVE_CHANNEL_2        = 0x02,     
  HAL_TIM_ACTIVE_CHANNEL_3        = 0x04,        
  HAL_TIM_ACTIVE_CHANNEL_4        = 0x08,     
  HAL_TIM_ACTIVE_CHANNEL_CLEARED  = 0x00          
}HAL_TIM_ActiveChannel;



  
typedef struct
{
  TIM_TypeDef                 *Instance;       
  TIM_Base_InitTypeDef        Init;           
  HAL_TIM_ActiveChannel       Channel;         
  DMA_HandleTypeDef           *hdma[7];      
 
  HAL_LockTypeDef             Lock;           
  volatile HAL_TIM_StateTypeDef   State;            
}TIM_HandleTypeDef;

 


 



 





 



 




 



                 






 



 














  
  


 










 



 

#line 378 "..\\..\\Drivers\\STM32F2xx_HAL_Driver\\Inc\\stm32f2xx_hal_tim.h"



                              
#line 388 "..\\..\\Drivers\\STM32F2xx_HAL_Driver\\Inc\\stm32f2xx_hal_tim.h"


 



 








  


 







  


 








  
  


 








 



 
  







 



 







  



 







  



 






                                 





                                 





                                      





  




 




    





  



 












  



 












  



 







  


  
#line 577 "..\\..\\Drivers\\STM32F2xx_HAL_Driver\\Inc\\stm32f2xx_hal_tim.h"


    


  
#line 591 "..\\..\\Drivers\\STM32F2xx_HAL_Driver\\Inc\\stm32f2xx_hal_tim.h"



#line 602 "..\\..\\Drivers\\STM32F2xx_HAL_Driver\\Inc\\stm32f2xx_hal_tim.h"


 





 

#line 620 "..\\..\\Drivers\\STM32F2xx_HAL_Driver\\Inc\\stm32f2xx_hal_tim.h"



 
    


 

#line 638 "..\\..\\Drivers\\STM32F2xx_HAL_Driver\\Inc\\stm32f2xx_hal_tim.h"
  


  



  
                                
#line 659 "..\\..\\Drivers\\STM32F2xx_HAL_Driver\\Inc\\stm32f2xx_hal_tim.h"

#line 672 "..\\..\\Drivers\\STM32F2xx_HAL_Driver\\Inc\\stm32f2xx_hal_tim.h"


 



  
#line 689 "..\\..\\Drivers\\STM32F2xx_HAL_Driver\\Inc\\stm32f2xx_hal_tim.h"

#line 700 "..\\..\\Drivers\\STM32F2xx_HAL_Driver\\Inc\\stm32f2xx_hal_tim.h"


    



 













 


                 











  


 




   



 







 



 






  



 
#line 779 "..\\..\\Drivers\\STM32F2xx_HAL_Driver\\Inc\\stm32f2xx_hal_tim.h"


 
  


 




  



   







 
  


 







 


 











   


                          







 


 







 


 







   
  


   
#line 875 "..\\..\\Drivers\\STM32F2xx_HAL_Driver\\Inc\\stm32f2xx_hal_tim.h"

#line 884 "..\\..\\Drivers\\STM32F2xx_HAL_Driver\\Inc\\stm32f2xx_hal_tim.h"
      
   


  


 













  



 







  


 

#line 948 "..\\..\\Drivers\\STM32F2xx_HAL_Driver\\Inc\\stm32f2xx_hal_tim.h"


   



 













 



                 











 



 




   

  

 









  
  


 

#line 1053 "..\\..\\Drivers\\STM32F2xx_HAL_Driver\\Inc\\stm32f2xx_hal_tim.h"


  



 

#line 1097 "..\\..\\Drivers\\STM32F2xx_HAL_Driver\\Inc\\stm32f2xx_hal_tim.h"


  


 




  



 
#line 1119 "..\\..\\Drivers\\STM32F2xx_HAL_Driver\\Inc\\stm32f2xx_hal_tim.h"


  



 






  



    
  
 





 






 




 







 
#line 1175 "..\\..\\Drivers\\STM32F2xx_HAL_Driver\\Inc\\stm32f2xx_hal_tim.h"


                           




 
#line 1193 "..\\..\\Drivers\\STM32F2xx_HAL_Driver\\Inc\\stm32f2xx_hal_tim.h"

#line 1200 "..\\..\\Drivers\\STM32F2xx_HAL_Driver\\Inc\\stm32f2xx_hal_tim.h"


















                          












 








 








 
















 
#line 1273 "..\\..\\Drivers\\STM32F2xx_HAL_Driver\\Inc\\stm32f2xx_hal_tim.h"
                            

















 






 
#line 1 "..\\..\\Drivers\\STM32F2xx_HAL_Driver\\Inc\\stm32f2xx_hal_tim_ex.h"



































  

 







 
#line 48 "..\\..\\Drivers\\STM32F2xx_HAL_Driver\\Inc\\stm32f2xx_hal_tim_ex.h"



 



  

  



 

typedef struct
{
                                  
  uint32_t IC1Polarity;            
 
                                                                   
  uint32_t IC1Prescaler;        
 
                                  
  uint32_t IC1Filter;           
   
  uint32_t Commutation_Delay;  
                               
} TIM_HallSensor_InitTypeDef;



  
typedef struct {
  uint32_t  MasterOutputTrigger;   
  
  uint32_t  MasterSlaveMode;       
 
}TIM_MasterConfigTypeDef;



  
typedef struct
{
  uint32_t OffStateRunMode;	        
 
  uint32_t OffStateIDLEMode;	      
 
  uint32_t LockLevel;	 	            
                              
  uint32_t DeadTime;	 	            
 
  uint32_t BreakState;	 	          
 
  uint32_t BreakPolarity;	 	        
 
  uint32_t AutomaticOutput;	 	      
            
}TIM_BreakDeadTimeConfigTypeDef;

 


 



 

#line 128 "..\\..\\Drivers\\STM32F2xx_HAL_Driver\\Inc\\stm32f2xx_hal_tim_ex.h"

#line 139 "..\\..\\Drivers\\STM32F2xx_HAL_Driver\\Inc\\stm32f2xx_hal_tim_ex.h"



  



    
  
 

 

 
HAL_StatusTypeDef HAL_TIMEx_HallSensor_Init(TIM_HandleTypeDef* htim, TIM_HallSensor_InitTypeDef* sConfig);
HAL_StatusTypeDef HAL_TIMEx_HallSensor_DeInit(TIM_HandleTypeDef* htim);

void HAL_TIMEx_HallSensor_MspInit(TIM_HandleTypeDef* htim);
void HAL_TIMEx_HallSensor_MspDeInit(TIM_HandleTypeDef* htim);

  
HAL_StatusTypeDef HAL_TIMEx_HallSensor_Start(TIM_HandleTypeDef* htim);
HAL_StatusTypeDef HAL_TIMEx_HallSensor_Stop(TIM_HandleTypeDef* htim);
 
HAL_StatusTypeDef HAL_TIMEx_HallSensor_Start_IT(TIM_HandleTypeDef* htim);
HAL_StatusTypeDef HAL_TIMEx_HallSensor_Stop_IT(TIM_HandleTypeDef* htim);
 
HAL_StatusTypeDef HAL_TIMEx_HallSensor_Start_DMA(TIM_HandleTypeDef* htim, uint32_t *pData, uint16_t Length);
HAL_StatusTypeDef HAL_TIMEx_HallSensor_Stop_DMA(TIM_HandleTypeDef* htim);

 
 
HAL_StatusTypeDef HAL_TIMEx_OCN_Start(TIM_HandleTypeDef* htim, uint32_t Channel);
HAL_StatusTypeDef HAL_TIMEx_OCN_Stop(TIM_HandleTypeDef* htim, uint32_t Channel);

 
HAL_StatusTypeDef HAL_TIMEx_OCN_Start_IT(TIM_HandleTypeDef* htim, uint32_t Channel);
HAL_StatusTypeDef HAL_TIMEx_OCN_Stop_IT(TIM_HandleTypeDef* htim, uint32_t Channel);

 
HAL_StatusTypeDef HAL_TIMEx_OCN_Start_DMA(TIM_HandleTypeDef* htim, uint32_t Channel, uint32_t *pData, uint16_t Length);
HAL_StatusTypeDef HAL_TIMEx_OCN_Stop_DMA(TIM_HandleTypeDef* htim, uint32_t Channel);

 
 
HAL_StatusTypeDef HAL_TIMEx_PWMN_Start(TIM_HandleTypeDef* htim, uint32_t Channel);
HAL_StatusTypeDef HAL_TIMEx_PWMN_Stop(TIM_HandleTypeDef* htim, uint32_t Channel);

 
HAL_StatusTypeDef HAL_TIMEx_PWMN_Start_IT(TIM_HandleTypeDef* htim, uint32_t Channel);
HAL_StatusTypeDef HAL_TIMEx_PWMN_Stop_IT(TIM_HandleTypeDef* htim, uint32_t Channel);
 
HAL_StatusTypeDef HAL_TIMEx_PWMN_Start_DMA(TIM_HandleTypeDef* htim, uint32_t Channel, uint32_t *pData, uint16_t Length);
HAL_StatusTypeDef HAL_TIMEx_PWMN_Stop_DMA(TIM_HandleTypeDef* htim, uint32_t Channel);

 
 
HAL_StatusTypeDef HAL_TIMEx_OnePulseN_Start(TIM_HandleTypeDef* htim, uint32_t OutputChannel);
HAL_StatusTypeDef HAL_TIMEx_OnePulseN_Stop(TIM_HandleTypeDef* htim, uint32_t OutputChannel);

 
HAL_StatusTypeDef HAL_TIMEx_OnePulseN_Start_IT(TIM_HandleTypeDef* htim, uint32_t OutputChannel);
HAL_StatusTypeDef HAL_TIMEx_OnePulseN_Stop_IT(TIM_HandleTypeDef* htim, uint32_t OutputChannel);

 
HAL_StatusTypeDef HAL_TIMEx_ConfigCommutationEvent(TIM_HandleTypeDef* htim, uint32_t  InputTrigger, uint32_t  CommutationSource);
HAL_StatusTypeDef HAL_TIMEx_ConfigCommutationEvent_IT(TIM_HandleTypeDef* htim, uint32_t  InputTrigger, uint32_t  CommutationSource);
HAL_StatusTypeDef HAL_TIMEx_ConfigCommutationEvent_DMA(TIM_HandleTypeDef* htim, uint32_t  InputTrigger, uint32_t  CommutationSource);
HAL_StatusTypeDef HAL_TIMEx_MasterConfigSynchronization(TIM_HandleTypeDef* htim, TIM_MasterConfigTypeDef * sMasterConfig);
HAL_StatusTypeDef HAL_TIMEx_ConfigBreakDeadTime(TIM_HandleTypeDef* htim, TIM_BreakDeadTimeConfigTypeDef *sBreakDeadTimeConfig);
HAL_StatusTypeDef HAL_TIMEx_RemapConfig(TIM_HandleTypeDef* htim, uint32_t Remap);

 
void HAL_TIMEx_CommutationCallback(TIM_HandleTypeDef* htim);
void HAL_TIMEx_BreakCallback(TIM_HandleTypeDef* htim);
void HAL_TIMEx_DMACommutationCplt(DMA_HandleTypeDef *hdma);

 
HAL_TIM_StateTypeDef HAL_TIMEx_HallSensor_GetState(TIM_HandleTypeDef* htim);



  



  
  






 
#line 1300 "..\\..\\Drivers\\STM32F2xx_HAL_Driver\\Inc\\stm32f2xx_hal_tim.h"

 

 
HAL_StatusTypeDef HAL_TIM_Base_Init(TIM_HandleTypeDef *htim);
HAL_StatusTypeDef HAL_TIM_Base_DeInit(TIM_HandleTypeDef *htim);
void HAL_TIM_Base_MspInit(TIM_HandleTypeDef *htim);
void HAL_TIM_Base_MspDeInit(TIM_HandleTypeDef *htim);
 
HAL_StatusTypeDef HAL_TIM_Base_Start(TIM_HandleTypeDef *htim);
HAL_StatusTypeDef HAL_TIM_Base_Stop(TIM_HandleTypeDef *htim);
 
HAL_StatusTypeDef HAL_TIM_Base_Start_IT(TIM_HandleTypeDef *htim);
HAL_StatusTypeDef HAL_TIM_Base_Stop_IT(TIM_HandleTypeDef *htim);
 
HAL_StatusTypeDef HAL_TIM_Base_Start_DMA(TIM_HandleTypeDef *htim, uint32_t *pData, uint16_t Length);
HAL_StatusTypeDef HAL_TIM_Base_Stop_DMA(TIM_HandleTypeDef *htim);

 
HAL_StatusTypeDef HAL_TIM_OC_Init(TIM_HandleTypeDef *htim);
HAL_StatusTypeDef HAL_TIM_OC_DeInit(TIM_HandleTypeDef *htim);
void HAL_TIM_OC_MspInit(TIM_HandleTypeDef *htim);
void HAL_TIM_OC_MspDeInit(TIM_HandleTypeDef *htim);
 
HAL_StatusTypeDef HAL_TIM_OC_Start(TIM_HandleTypeDef *htim, uint32_t Channel);
HAL_StatusTypeDef HAL_TIM_OC_Stop(TIM_HandleTypeDef *htim, uint32_t Channel);
 
HAL_StatusTypeDef HAL_TIM_OC_Start_IT(TIM_HandleTypeDef *htim, uint32_t Channel);
HAL_StatusTypeDef HAL_TIM_OC_Stop_IT(TIM_HandleTypeDef *htim, uint32_t Channel);
 
HAL_StatusTypeDef HAL_TIM_OC_Start_DMA(TIM_HandleTypeDef *htim, uint32_t Channel, uint32_t *pData, uint16_t Length);
HAL_StatusTypeDef HAL_TIM_OC_Stop_DMA(TIM_HandleTypeDef *htim, uint32_t Channel);

 
HAL_StatusTypeDef HAL_TIM_PWM_Init(TIM_HandleTypeDef *htim);
HAL_StatusTypeDef HAL_TIM_PWM_DeInit(TIM_HandleTypeDef *htim);
void HAL_TIM_PWM_MspInit(TIM_HandleTypeDef *htim);
void HAL_TIM_PWM_MspDeInit(TIM_HandleTypeDef *htim);
 
HAL_StatusTypeDef HAL_TIM_PWM_Start(TIM_HandleTypeDef *htim, uint32_t Channel);
HAL_StatusTypeDef HAL_TIM_PWM_Stop(TIM_HandleTypeDef *htim, uint32_t Channel);
 
HAL_StatusTypeDef HAL_TIM_PWM_Start_IT(TIM_HandleTypeDef *htim, uint32_t Channel);
HAL_StatusTypeDef HAL_TIM_PWM_Stop_IT(TIM_HandleTypeDef *htim, uint32_t Channel);
 
HAL_StatusTypeDef HAL_TIM_PWM_Start_DMA(TIM_HandleTypeDef *htim, uint32_t Channel, uint32_t *pData, uint16_t Length);
HAL_StatusTypeDef HAL_TIM_PWM_Stop_DMA(TIM_HandleTypeDef *htim, uint32_t Channel);

 
HAL_StatusTypeDef HAL_TIM_IC_Init(TIM_HandleTypeDef *htim);
HAL_StatusTypeDef HAL_TIM_IC_DeInit(TIM_HandleTypeDef *htim);
void HAL_TIM_IC_MspInit(TIM_HandleTypeDef *htim);
void HAL_TIM_IC_MspDeInit(TIM_HandleTypeDef *htim);
 
HAL_StatusTypeDef HAL_TIM_IC_Start(TIM_HandleTypeDef *htim, uint32_t Channel);
HAL_StatusTypeDef HAL_TIM_IC_Stop(TIM_HandleTypeDef *htim, uint32_t Channel);
 
HAL_StatusTypeDef HAL_TIM_IC_Start_IT(TIM_HandleTypeDef *htim, uint32_t Channel);
HAL_StatusTypeDef HAL_TIM_IC_Stop_IT(TIM_HandleTypeDef *htim, uint32_t Channel);
 
HAL_StatusTypeDef HAL_TIM_IC_Start_DMA(TIM_HandleTypeDef *htim, uint32_t Channel, uint32_t *pData, uint16_t Length);
HAL_StatusTypeDef HAL_TIM_IC_Stop_DMA(TIM_HandleTypeDef *htim, uint32_t Channel);

 
HAL_StatusTypeDef HAL_TIM_OnePulse_Init(TIM_HandleTypeDef *htim, uint32_t OnePulseMode);
HAL_StatusTypeDef HAL_TIM_OnePulse_DeInit(TIM_HandleTypeDef *htim);
void HAL_TIM_OnePulse_MspInit(TIM_HandleTypeDef *htim);
void HAL_TIM_OnePulse_MspDeInit(TIM_HandleTypeDef *htim);
 
HAL_StatusTypeDef HAL_TIM_OnePulse_Start(TIM_HandleTypeDef *htim, uint32_t OutputChannel);
HAL_StatusTypeDef HAL_TIM_OnePulse_Stop(TIM_HandleTypeDef *htim, uint32_t OutputChannel);

 
HAL_StatusTypeDef HAL_TIM_OnePulse_Start_IT(TIM_HandleTypeDef *htim, uint32_t OutputChannel);
HAL_StatusTypeDef HAL_TIM_OnePulse_Stop_IT(TIM_HandleTypeDef *htim, uint32_t OutputChannel);

 
HAL_StatusTypeDef HAL_TIM_Encoder_Init(TIM_HandleTypeDef *htim,  TIM_Encoder_InitTypeDef* sConfig);
HAL_StatusTypeDef HAL_TIM_Encoder_DeInit(TIM_HandleTypeDef *htim);
void HAL_TIM_Encoder_MspInit(TIM_HandleTypeDef *htim);
void HAL_TIM_Encoder_MspDeInit(TIM_HandleTypeDef *htim);
  
HAL_StatusTypeDef HAL_TIM_Encoder_Start(TIM_HandleTypeDef *htim, uint32_t Channel);
HAL_StatusTypeDef HAL_TIM_Encoder_Stop(TIM_HandleTypeDef *htim, uint32_t Channel);
 
HAL_StatusTypeDef HAL_TIM_Encoder_Start_IT(TIM_HandleTypeDef *htim, uint32_t Channel);
HAL_StatusTypeDef HAL_TIM_Encoder_Stop_IT(TIM_HandleTypeDef *htim, uint32_t Channel);
 
HAL_StatusTypeDef HAL_TIM_Encoder_Start_DMA(TIM_HandleTypeDef *htim, uint32_t Channel, uint32_t *pData1, uint32_t *pData2, uint16_t Length);
HAL_StatusTypeDef HAL_TIM_Encoder_Stop_DMA(TIM_HandleTypeDef *htim, uint32_t Channel);

 
void HAL_TIM_IRQHandler(TIM_HandleTypeDef *htim);

 
HAL_StatusTypeDef HAL_TIM_OC_ConfigChannel(TIM_HandleTypeDef *htim, TIM_OC_InitTypeDef* sConfig, uint32_t Channel);
HAL_StatusTypeDef HAL_TIM_PWM_ConfigChannel(TIM_HandleTypeDef *htim, TIM_OC_InitTypeDef* sConfig, uint32_t Channel);
HAL_StatusTypeDef HAL_TIM_IC_ConfigChannel(TIM_HandleTypeDef *htim, TIM_IC_InitTypeDef* sConfig, uint32_t Channel);
HAL_StatusTypeDef HAL_TIM_OnePulse_ConfigChannel(TIM_HandleTypeDef *htim, TIM_OnePulse_InitTypeDef* sConfig, uint32_t OutputChannel,  uint32_t InputChannel);
HAL_StatusTypeDef HAL_TIM_ConfigOCrefClear(TIM_HandleTypeDef *htim, TIM_ClearInputConfigTypeDef * sClearInputConfig, uint32_t Channel);
HAL_StatusTypeDef HAL_TIM_ConfigClockSource(TIM_HandleTypeDef *htim, TIM_ClockConfigTypeDef * sClockSourceConfig);    
HAL_StatusTypeDef HAL_TIM_ConfigTI1Input(TIM_HandleTypeDef *htim, uint32_t TI1_Selection);
HAL_StatusTypeDef HAL_TIM_SlaveConfigSynchronization(TIM_HandleTypeDef *htim, TIM_SlaveConfigTypeDef * sSlaveConfig);
HAL_StatusTypeDef HAL_TIM_DMABurst_WriteStart(TIM_HandleTypeDef *htim, uint32_t BurstBaseAddress, uint32_t BurstRequestSrc,                                               uint32_t  *BurstBuffer, uint32_t  BurstLength);

HAL_StatusTypeDef HAL_TIM_DMABurst_WriteStop(TIM_HandleTypeDef *htim, uint32_t BurstRequestSrc);
HAL_StatusTypeDef HAL_TIM_DMABurst_ReadStart(TIM_HandleTypeDef *htim, uint32_t BurstBaseAddress, uint32_t BurstRequestSrc,                                               uint32_t  *BurstBuffer, uint32_t  BurstLength);

HAL_StatusTypeDef HAL_TIM_DMABurst_ReadStop(TIM_HandleTypeDef *htim, uint32_t BurstRequestSrc);
HAL_StatusTypeDef HAL_TIM_GenerateEvent(TIM_HandleTypeDef *htim, uint32_t EventSource);
uint32_t HAL_TIM_ReadCapturedValue(TIM_HandleTypeDef *htim, uint32_t Channel);

 
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim);
void HAL_TIM_OC_DelayElapsedCallback(TIM_HandleTypeDef *htim);
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim);
void HAL_TIM_PWM_PulseFinishedCallback(TIM_HandleTypeDef *htim);
void HAL_TIM_TriggerCallback(TIM_HandleTypeDef *htim);
void HAL_TIM_ErrorCallback(TIM_HandleTypeDef *htim);

 
HAL_TIM_StateTypeDef HAL_TIM_Base_GetState(TIM_HandleTypeDef *htim);
HAL_TIM_StateTypeDef HAL_TIM_OC_GetState(TIM_HandleTypeDef *htim);
HAL_TIM_StateTypeDef HAL_TIM_PWM_GetState(TIM_HandleTypeDef *htim);
HAL_TIM_StateTypeDef HAL_TIM_IC_GetState(TIM_HandleTypeDef *htim);
HAL_TIM_StateTypeDef HAL_TIM_OnePulse_GetState(TIM_HandleTypeDef *htim);
HAL_TIM_StateTypeDef HAL_TIM_Encoder_GetState(TIM_HandleTypeDef *htim);

void TIM_Base_SetConfig(TIM_TypeDef *TIMx, TIM_Base_InitTypeDef *Structure);
void TIM_TI1_SetConfig(TIM_TypeDef *TIMx, uint32_t TIM_ICPolarity, uint32_t TIM_ICSelection, uint32_t TIM_ICFilter);
void TIM_OC2_SetConfig(TIM_TypeDef *TIMx, TIM_OC_InitTypeDef *OC_Config);
void HAL_TIM_DMADelayPulseCplt(DMA_HandleTypeDef *hdma);
void HAL_TIM_DMAError(DMA_HandleTypeDef *hdma);
void HAL_TIM_DMACaptureCplt(DMA_HandleTypeDef *hdma);
void TIM_CCxChannelCmd(TIM_TypeDef* TIMx, uint32_t Channel, uint32_t ChannelState);



  



  
  






 
#line 313 "..\\..\\Inc\\stm32f2xx_hal_conf.h"






















#line 1 "..\\..\\Drivers\\STM32F2xx_HAL_Driver\\Inc\\stm32f2xx_hal_pcd.h"



































  

 







 
#line 1 "..\\..\\Drivers\\STM32F2xx_HAL_Driver\\Inc\\stm32f2xx_ll_usb.h"



































  

 







 
#line 48 "..\\..\\Drivers\\STM32F2xx_HAL_Driver\\Inc\\stm32f2xx_ll_usb.h"



 



  

  



   
typedef enum 
{
   USB_OTG_DEVICE_MODE  = 0,
   USB_OTG_HOST_MODE    = 1,
   USB_OTG_DRD_MODE     = 2
   
}USB_OTG_ModeTypeDef;



  
typedef enum {
  URB_IDLE = 0,
  URB_DONE,
  URB_NOTREADY,
  URB_NYET,
  URB_ERROR,
  URB_STALL
    
}USB_OTG_URBStateTypeDef;



  
typedef enum {
  HC_IDLE = 0,
  HC_XFRC,
  HC_HALTED,
  HC_NAK,
  HC_NYET,
  HC_STALL,
  HC_XACTERR,  
  HC_BBLERR,   
  HC_DATATGLERR
    
}USB_OTG_HCStateTypeDef;



 
typedef struct
{
  uint32_t dev_endpoints;        

     
  
  uint32_t Host_channels;        

        

  uint32_t speed;                
         
                               
  uint32_t dma_enable;                        

  uint32_t ep0_mps;              
               
                       
  uint32_t phy_itface;           
  
                                
  uint32_t Sof_enable;                 
                               
  uint32_t low_power_enable;           
                          
  uint32_t vbus_sensing_enable;    

  uint32_t use_dedicated_ep1;           
  
  uint32_t use_external_vbus;        
  
}USB_OTG_CfgTypeDef;

typedef struct
{
  uint8_t   num;            
  
                                
  uint8_t   is_in;          
  
  
  uint8_t   is_stall;       
  
  
  uint8_t   type;           
  
                                
  uint8_t   data_pid_start; 
 
                                
  uint8_t   even_odd_frame; 
 
                                
  uint16_t  tx_fifo_num;    
 
                                
  uint32_t  maxpacket;      
 

  uint8_t   *xfer_buff;      
                                
  uint32_t  dma_addr;        
  
  uint32_t  xfer_len;        
  
  uint32_t  xfer_count;      

}USB_OTG_EPTypeDef;

typedef struct
{
  uint8_t   dev_addr ;     
  

  uint8_t   ch_num;        
  
                                
  uint8_t   ep_num;        
  
                                
  uint8_t   ep_is_in;      
  
                                
  uint8_t   speed;         
 
                                
  uint8_t   do_ping;        
  
  uint8_t   process_ping;   

  uint8_t   ep_type;       
 
                                
  uint16_t  max_packet;    
 
                                
  uint8_t   data_pid;      
 
                                
  uint8_t   *xfer_buff;     
  
  uint32_t  xfer_len;       
  
  uint32_t  xfer_count;     
  
  uint8_t   toggle_in;     
 
                                
  uint8_t   toggle_out;    
 
  
  uint32_t  dma_addr;       
  
  uint32_t  ErrCnt;         
  
  USB_OTG_URBStateTypeDef  urb_state;  
  
  
  USB_OTG_HCStateTypeDef   state;     
  
                                             
}USB_OTG_HCTypeDef;
  
 



 



 





 



   






 
  


    




 
  


 





 



 






 
  


   






 



 






 



 





 



 







 



 







 



   





 
    


     





   



 





       









    











 


    



 
HAL_StatusTypeDef USB_CoreInit(USB_OTG_GlobalTypeDef *USBx, USB_OTG_CfgTypeDef Init);
HAL_StatusTypeDef USB_DevInit(USB_OTG_GlobalTypeDef *USBx, USB_OTG_CfgTypeDef Init);
HAL_StatusTypeDef USB_EnableGlobalInt(USB_OTG_GlobalTypeDef *USBx);
HAL_StatusTypeDef USB_DisableGlobalInt(USB_OTG_GlobalTypeDef *USBx);
HAL_StatusTypeDef USB_SetCurrentMode(USB_OTG_GlobalTypeDef *USBx , USB_OTG_ModeTypeDef mode);
HAL_StatusTypeDef USB_SetDevSpeed(USB_OTG_GlobalTypeDef *USBx , uint8_t speed);
HAL_StatusTypeDef USB_FlushRxFifo (USB_OTG_GlobalTypeDef *USBx);
HAL_StatusTypeDef USB_FlushTxFifo (USB_OTG_GlobalTypeDef *USBx, uint32_t num );
HAL_StatusTypeDef USB_ActivateEndpoint(USB_OTG_GlobalTypeDef *USBx, USB_OTG_EPTypeDef *ep);
HAL_StatusTypeDef USB_DeactivateEndpoint(USB_OTG_GlobalTypeDef *USBx, USB_OTG_EPTypeDef *ep);
HAL_StatusTypeDef USB_ActivateDedicatedEndpoint(USB_OTG_GlobalTypeDef *USBx, USB_OTG_EPTypeDef *ep);
HAL_StatusTypeDef USB_DeactivateDedicatedEndpoint(USB_OTG_GlobalTypeDef *USBx, USB_OTG_EPTypeDef *ep);
HAL_StatusTypeDef USB_EPStartXfer(USB_OTG_GlobalTypeDef *USBx , USB_OTG_EPTypeDef *ep, uint8_t dma);
HAL_StatusTypeDef USB_EP0StartXfer(USB_OTG_GlobalTypeDef *USBx , USB_OTG_EPTypeDef *ep, uint8_t dma);
HAL_StatusTypeDef USB_WritePacket(USB_OTG_GlobalTypeDef *USBx, uint8_t *src, uint8_t ch_ep_num, uint16_t len, uint8_t dma);
void *            USB_ReadPacket(USB_OTG_GlobalTypeDef *USBx, uint8_t *dest, uint16_t len);
HAL_StatusTypeDef USB_EPSetStall(USB_OTG_GlobalTypeDef *USBx , USB_OTG_EPTypeDef *ep);
HAL_StatusTypeDef USB_EPClearStall(USB_OTG_GlobalTypeDef *USBx , USB_OTG_EPTypeDef *ep);
HAL_StatusTypeDef USB_SetDevAddress (USB_OTG_GlobalTypeDef *USBx, uint8_t address);
HAL_StatusTypeDef USB_DevConnect (USB_OTG_GlobalTypeDef *USBx);
HAL_StatusTypeDef USB_DevDisconnect (USB_OTG_GlobalTypeDef *USBx);
HAL_StatusTypeDef USB_StopDevice(USB_OTG_GlobalTypeDef *USBx);
HAL_StatusTypeDef USB_ActivateSetup (USB_OTG_GlobalTypeDef *USBx);
HAL_StatusTypeDef USB_EP0_OutStart(USB_OTG_GlobalTypeDef *USBx, uint8_t dma, uint8_t *psetup);
uint8_t           USB_GetDevSpeed(USB_OTG_GlobalTypeDef *USBx);
uint32_t          USB_GetMode(USB_OTG_GlobalTypeDef *USBx);
uint32_t          USB_ReadInterrupts (USB_OTG_GlobalTypeDef *USBx);
uint32_t          USB_ReadDevAllOutEpInterrupt (USB_OTG_GlobalTypeDef *USBx);
uint32_t          USB_ReadDevOutEPInterrupt (USB_OTG_GlobalTypeDef *USBx , uint8_t epnum);
uint32_t          USB_ReadDevAllInEpInterrupt (USB_OTG_GlobalTypeDef *USBx);
uint32_t          USB_ReadDevInEPInterrupt (USB_OTG_GlobalTypeDef *USBx , uint8_t epnum);
void              USB_ClearInterrupts (USB_OTG_GlobalTypeDef *USBx, uint32_t interrupt);

HAL_StatusTypeDef USB_HostInit (USB_OTG_GlobalTypeDef *USBx, USB_OTG_CfgTypeDef cfg);
HAL_StatusTypeDef USB_InitFSLSPClkSel(USB_OTG_GlobalTypeDef *USBx , uint8_t freq);
HAL_StatusTypeDef USB_ResetPort(USB_OTG_GlobalTypeDef *USBx);
HAL_StatusTypeDef USB_DriveVbus (USB_OTG_GlobalTypeDef *USBx, uint8_t state);
uint32_t          USB_GetHostSpeed (USB_OTG_GlobalTypeDef *USBx);
uint32_t          USB_GetCurrentFrame (USB_OTG_GlobalTypeDef *USBx);
HAL_StatusTypeDef USB_HC_Init(USB_OTG_GlobalTypeDef *USBx,  
                                  uint8_t ch_num,
                                  uint8_t epnum,
                                  uint8_t dev_address,
                                  uint8_t speed,
                                  uint8_t ep_type,
                                  uint16_t mps);
HAL_StatusTypeDef USB_HC_StartXfer(USB_OTG_GlobalTypeDef *USBx, USB_OTG_HCTypeDef *hc, uint8_t dma);
uint32_t          USB_HC_ReadInterrupt (USB_OTG_GlobalTypeDef *USBx);
HAL_StatusTypeDef USB_HC_Halt(USB_OTG_GlobalTypeDef *USBx , uint8_t hc_num);
HAL_StatusTypeDef USB_DoPing(USB_OTG_GlobalTypeDef *USBx , uint8_t ch_num);
HAL_StatusTypeDef USB_StopHost(USB_OTG_GlobalTypeDef *USBx);



  



 
  







 
#line 48 "..\\..\\Drivers\\STM32F2xx_HAL_Driver\\Inc\\stm32f2xx_hal_pcd.h"
   


 



  

  

   

   
typedef enum 
{
  PCD_READY    = 0x00,
  PCD_ERROR    = 0x01,
  PCD_BUSY     = 0x02,
  PCD_TIMEOUT  = 0x03
} PCD_StateTypeDef;


typedef USB_OTG_GlobalTypeDef  PCD_TypeDef;
typedef USB_OTG_CfgTypeDef     PCD_InitTypeDef;
typedef USB_OTG_EPTypeDef      PCD_EPTypeDef ;                          



  
typedef struct
{
  PCD_TypeDef             *Instance;     
  PCD_InitTypeDef         Init;        
  PCD_EPTypeDef           IN_ep[15];   
  PCD_EPTypeDef           OUT_ep[15];   
  HAL_LockTypeDef         Lock;        
  volatile PCD_StateTypeDef   State;       
  uint32_t                Setup[12];   
  void                    *pData;           
  
} PCD_HandleTypeDef;
  
 


 



 





 
  
  

 




 
  


  




 



  
  
 




 


   










                                                      

                                                         






















                                                      


















                                                      











 

 

 
HAL_StatusTypeDef HAL_PCD_Init(PCD_HandleTypeDef *hpcd);
HAL_StatusTypeDef HAL_PCD_DeInit (PCD_HandleTypeDef *hpcd);
void HAL_PCD_MspInit(PCD_HandleTypeDef *hpcd);
void HAL_PCD_MspDeInit(PCD_HandleTypeDef *hpcd);

 
  
HAL_StatusTypeDef HAL_PCD_Start(PCD_HandleTypeDef *hpcd);
HAL_StatusTypeDef HAL_PCD_Stop(PCD_HandleTypeDef *hpcd);
void HAL_PCD_IRQHandler(PCD_HandleTypeDef *hpcd);

void HAL_PCD_DataOutStageCallback(PCD_HandleTypeDef *hpcd, uint8_t epnum);
void HAL_PCD_DataInStageCallback(PCD_HandleTypeDef *hpcd, uint8_t epnum);
void HAL_PCD_SetupStageCallback(PCD_HandleTypeDef *hpcd);
void HAL_PCD_SOFCallback(PCD_HandleTypeDef *hpcd);
void HAL_PCD_ResetCallback(PCD_HandleTypeDef *hpcd);
void HAL_PCD_SuspendCallback(PCD_HandleTypeDef *hpcd);
void HAL_PCD_ResumeCallback(PCD_HandleTypeDef *hpcd);
void HAL_PCD_ISOOUTIncompleteCallback(PCD_HandleTypeDef *hpcd, uint8_t epnum);
void HAL_PCD_ISOINIncompleteCallback(PCD_HandleTypeDef *hpcd, uint8_t epnum);
void HAL_PCD_ConnectCallback(PCD_HandleTypeDef *hpcd);
void HAL_PCD_DisconnectCallback(PCD_HandleTypeDef *hpcd);



 
HAL_StatusTypeDef HAL_PCD_DevConnect(PCD_HandleTypeDef *hpcd);
HAL_StatusTypeDef HAL_PCD_DevDisconnect(PCD_HandleTypeDef *hpcd);
HAL_StatusTypeDef HAL_PCD_SetAddress(PCD_HandleTypeDef *hpcd, uint8_t address);
HAL_StatusTypeDef HAL_PCD_EP_Open(PCD_HandleTypeDef *hpcd, uint8_t ep_addr, uint16_t ep_mps, uint8_t ep_type);
HAL_StatusTypeDef HAL_PCD_EP_Close(PCD_HandleTypeDef *hpcd, uint8_t ep_addr);
HAL_StatusTypeDef HAL_PCD_EP_Receive(PCD_HandleTypeDef *hpcd, uint8_t ep_addr, uint8_t *pBuf, uint32_t len);
HAL_StatusTypeDef HAL_PCD_EP_Transmit(PCD_HandleTypeDef *hpcd, uint8_t ep_addr, uint8_t *pBuf, uint32_t len);
uint16_t          HAL_PCD_EP_GetRxCount(PCD_HandleTypeDef *hpcd, uint8_t ep_addr);
HAL_StatusTypeDef HAL_PCD_EP_SetStall(PCD_HandleTypeDef *hpcd, uint8_t ep_addr);
HAL_StatusTypeDef HAL_PCD_EP_ClrStall(PCD_HandleTypeDef *hpcd, uint8_t ep_addr);
HAL_StatusTypeDef HAL_PCD_EP_Flush(PCD_HandleTypeDef *hpcd, uint8_t ep_addr);
HAL_StatusTypeDef HAL_PCD_SetTxFiFo(PCD_HandleTypeDef *hpcd, uint8_t fifo, uint16_t size);
HAL_StatusTypeDef HAL_PCD_SetRxFiFo(PCD_HandleTypeDef *hpcd, uint16_t size);
HAL_StatusTypeDef HAL_PCD_ActiveRemoteWakeup(PCD_HandleTypeDef *hpcd);
HAL_StatusTypeDef HAL_PCD_DeActiveRemoteWakeup(PCD_HandleTypeDef *hpcd);
 
PCD_StateTypeDef HAL_PCD_GetState(PCD_HandleTypeDef *hpcd);



  



  








 
#line 337 "..\\..\\Inc\\stm32f2xx_hal_conf.h"






 
#line 359 "..\\..\\Inc\\stm32f2xx_hal_conf.h"








 
#line 49 "..\\..\\Drivers\\STM32F2xx_HAL_Driver\\Inc\\stm32f2xx_hal.h"



 



  

 
 
 


 
#line 86 "..\\..\\Drivers\\STM32F2xx_HAL_Driver\\Inc\\stm32f2xx_hal.h"

#line 109 "..\\..\\Drivers\\STM32F2xx_HAL_Driver\\Inc\\stm32f2xx_hal.h"


 



 





 





 




 

 
HAL_StatusTypeDef HAL_Init(void);
HAL_StatusTypeDef HAL_DeInit(void);
void HAL_MspInit(void);
void HAL_MspDeInit(void);

 
void     HAL_IncTick(void);
void     HAL_Delay(volatile uint32_t Delay);
uint32_t HAL_GetTick(void);
uint32_t HAL_GetHalVersion(void);
uint32_t HAL_GetREVID(void);
uint32_t HAL_GetDEVID(void);
void HAL_EnableDBGSleepMode(void);
void HAL_DisableDBGSleepMode(void);
void HAL_EnableDBGStopMode(void);
void HAL_DisableDBGStopMode(void);
void HAL_EnableDBGStandbyMode(void);
void HAL_DisableDBGStandbyMode(void);
void HAL_EnableCompensationCell(void);
void HAL_DisableCompensationCell(void);



  



  
  






 
#line 36 "..\\..\\Src\\stm32f2xx_it.c"
#line 37 "..\\..\\Src\\stm32f2xx_it.c"
#line 1 "..\\..\\Inc\\stm32f2xx_it.h"
































 

 







 
 
 
 
 

void DMA2_Stream2_IRQHandler(void);
void DMA2_Stream3_IRQHandler(void);
void DMA1_Stream0_IRQHandler(void);
void SysTick_Handler(void);
void SPI3_IRQHandler(void);
void OTG_HS_IRQHandler(void);
void DMA1_Stream5_IRQHandler(void);
void SPI1_IRQHandler(void);







 
#line 38 "..\\..\\Src\\stm32f2xx_it.c"

 

extern PCD_HandleTypeDef hpcd_USB_OTG_HS;
extern DMA_HandleTypeDef hdma_spi1_rx;
extern DMA_HandleTypeDef hdma_spi1_tx;
extern DMA_HandleTypeDef hdma_spi3_rx;
extern DMA_HandleTypeDef hdma_spi3_tx;
extern SPI_HandleTypeDef hspi1;
extern SPI_HandleTypeDef hspi3;
extern TIM_HandleTypeDef htim7;

 
 
 



 
void SysTick_Handler(void)
	{
	HAL_IncTick();
	HAL_SYSTICK_IRQHandler();
	}



 
void OTG_HS_IRQHandler(void)
	{
	HAL_NVIC_ClearPendingIRQ(OTG_HS_IRQn);
	HAL_PCD_IRQHandler(&hpcd_USB_OTG_HS);
	}



 
void SPI1_IRQHandler(void)
	{
	HAL_NVIC_ClearPendingIRQ(SPI1_IRQn);
	HAL_SPI_IRQHandler(&hspi1);
	}



 
void SPI3_IRQHandler(void)
	{
	HAL_NVIC_ClearPendingIRQ(SPI3_IRQn);		
	HAL_SPI_IRQHandler(&hspi3);
	}



 
void DMA2_Stream2_IRQHandler(void)
	{
	HAL_NVIC_ClearPendingIRQ(DMA2_Stream2_IRQn);
	HAL_DMA_IRQHandler(&hdma_spi1_rx);
	}



 
void DMA2_Stream3_IRQHandler(void)
	{
	HAL_NVIC_ClearPendingIRQ(DMA2_Stream3_IRQn);
	HAL_DMA_IRQHandler(&hdma_spi1_tx);
	}



 
void DMA1_Stream0_IRQHandler(void)
	{
	HAL_NVIC_ClearPendingIRQ(DMA1_Stream0_IRQn);
	HAL_DMA_IRQHandler(&hdma_spi3_rx);
	}



 
void DMA1_Stream5_IRQHandler(void)
	{
	HAL_NVIC_ClearPendingIRQ(DMA1_Stream5_IRQn);
	HAL_DMA_IRQHandler(&hdma_spi3_tx);
	}

void TIM7_IRQHandler(void)
	{
	HAL_TIM_IRQHandler(&htim7);
	}


 
