#line 1 "Source\\main.c"
#line 1 ".\\Include\\stm32f4xx.h"




































  



 



 
    






  


 
  


 







 





#line 82 ".\\Include\\stm32f4xx.h"







            








 










 
#line 118 ".\\Include\\stm32f4xx.h"
                                             


 



 



 









 
 



 
typedef enum IRQn
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
  ETH_IRQn                    = 61,      
  ETH_WKUP_IRQn               = 62,      
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
  DCMI_IRQn                   = 78,      
  CRYP_IRQn                   = 79,      
  HASH_RNG_IRQn               = 80,       
  FPU_IRQn                    = 81       
} IRQn_Type;



 

#line 1 "C:\\STM32F4-Discovery_FW_V1.1.0\\Libraries\\CMSIS\\Include\\core_cm4.h"
 




















 
























 













 




 






 

 











#line 100 "C:\\STM32F4-Discovery_FW_V1.1.0\\Libraries\\CMSIS\\Include\\core_cm4.h"

 
#line 113 "C:\\STM32F4-Discovery_FW_V1.1.0\\Libraries\\CMSIS\\Include\\core_cm4.h"

#line 142 "C:\\STM32F4-Discovery_FW_V1.1.0\\Libraries\\CMSIS\\Include\\core_cm4.h"

#line 1 "C:\\Keil\\ARM\\ARMCC\\bin\\..\\include\\stdint.h"
 
 





 










#line 26 "C:\\Keil\\ARM\\ARMCC\\bin\\..\\include\\stdint.h"







 

     

     
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

     
typedef   signed       __int64 intmax_t;
typedef unsigned       __int64 uintmax_t;




     

     





     





     





     

     





     





     





     

     





     





     





     

     


     


     


     

     


     


     


     

     



     



     


     
    
 



#line 197 "C:\\Keil\\ARM\\ARMCC\\bin\\..\\include\\stdint.h"

     







     










     











#line 261 "C:\\Keil\\ARM\\ARMCC\\bin\\..\\include\\stdint.h"



 



#line 144 "C:\\STM32F4-Discovery_FW_V1.1.0\\Libraries\\CMSIS\\Include\\core_cm4.h"
#line 1 "C:\\STM32F4-Discovery_FW_V1.1.0\\Libraries\\CMSIS\\Include\\core_cmInstr.h"
 




















 





 



 


 









 







 







 






 








 







 







 









 









 
static __inline __asm uint32_t __REV16(uint32_t value)
{
  rev16 r0, r0
  bx lr
}








 
static __inline __asm int32_t __REVSH(int32_t value)
{
  revsh r0, r0
  bx lr
}










 









 









 









 











 











 











 







 










 










 









 






#line 582 "C:\\STM32F4-Discovery_FW_V1.1.0\\Libraries\\CMSIS\\Include\\core_cmInstr.h"

   

#line 145 "C:\\STM32F4-Discovery_FW_V1.1.0\\Libraries\\CMSIS\\Include\\core_cm4.h"
#line 1 "C:\\STM32F4-Discovery_FW_V1.1.0\\Libraries\\CMSIS\\Include\\core_cmFunc.h"
 




















 





 



 


 





 
 






 
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











 
static __inline uint32_t __get_FPSCR(void)
{

  register uint32_t __regfpscr         __asm("fpscr");
  return(__regfpscr);



}







 
static __inline void __set_FPSCR(uint32_t fpscr)
{

  register uint32_t __regfpscr         __asm("fpscr");
  __regfpscr = (fpscr);

}




#line 605 "C:\\STM32F4-Discovery_FW_V1.1.0\\Libraries\\CMSIS\\Include\\core_cmFunc.h"

 


#line 146 "C:\\STM32F4-Discovery_FW_V1.1.0\\Libraries\\CMSIS\\Include\\core_cm4.h"
#line 1 "C:\\STM32F4-Discovery_FW_V1.1.0\\Libraries\\CMSIS\\Include\\core_cm4_simd.h"
 




















 











 


 



 


 

 
#line 106 "C:\\STM32F4-Discovery_FW_V1.1.0\\Libraries\\CMSIS\\Include\\core_cm4_simd.h"








 



#line 693 "C:\\STM32F4-Discovery_FW_V1.1.0\\Libraries\\CMSIS\\Include\\core_cm4_simd.h"

 




#line 147 "C:\\STM32F4-Discovery_FW_V1.1.0\\Libraries\\CMSIS\\Include\\core_cm4.h"








 
#line 182 "C:\\STM32F4-Discovery_FW_V1.1.0\\Libraries\\CMSIS\\Include\\core_cm4.h"

 
#line 191 "C:\\STM32F4-Discovery_FW_V1.1.0\\Libraries\\CMSIS\\Include\\core_cm4.h"

 





 









 





 


 
typedef union
{
  struct
  {



    uint32_t _reserved0:16;               
    uint32_t GE:4;                        
    uint32_t _reserved1:7;                

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



    uint32_t _reserved0:7;                
    uint32_t GE:4;                        
    uint32_t _reserved1:4;                

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
} ITM_Type;

 



 



























   







 


 
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
       uint32_t RESERVED0[1];
  volatile uint32_t FPCCR;                    
  volatile uint32_t FPCAR;                    
  volatile uint32_t FPDSCR;                   
  volatile const  uint32_t MVFR0;                    
  volatile const  uint32_t MVFR1;                    
} FPU_Type;

 



























 



 












 
























 












 







 


 
typedef struct
{
  volatile uint32_t DHCSR;                    
  volatile  uint32_t DCRSR;                    
  volatile uint32_t DCRDR;                    
  volatile uint32_t DEMCR;                    
} CoreDebug_Type;

 




































 






 







































 




 

 
#line 986 "C:\\STM32F4-Discovery_FW_V1.1.0\\Libraries\\CMSIS\\Include\\core_cm4.h"

#line 993 "C:\\STM32F4-Discovery_FW_V1.1.0\\Libraries\\CMSIS\\Include\\core_cm4.h"











 





 






 



 



 










 
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
 
  ((NVIC_Type *) ((0xE000E000UL) + 0x0100UL) )->ISER[(uint32_t)((int32_t)IRQn) >> 5] = (uint32_t)(1 << ((uint32_t)((int32_t)IRQn) & (uint32_t)0x1F));  
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
  if (ticks > (0xFFFFFFUL << 0))  return (1);             

  ((SysTick_Type *) ((0xE000E000UL) + 0x0010UL) )->LOAD  = (ticks & (0xFFFFFFUL << 0)) - 1;       
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
  if ((((CoreDebug_Type *) (0xE000EDF0UL))->DEMCR & (1UL << 24))  &&       
      (((ITM_Type *) (0xE0000000UL) )->TCR & (1UL << 0))                  &&       
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

 





#line 246 ".\\Include\\stm32f4xx.h"
#line 1 ".\\Include\\system_stm32f4xx.h"



















  



 



   
  


 









 



 




 

extern uint32_t SystemCoreClock;           




 



 



 



 



 



 
  
extern void SystemInit(void);
extern void SystemCoreClockUpdate(void);


 









 
  


   
 
#line 247 ".\\Include\\stm32f4xx.h"
#line 248 ".\\Include\\stm32f4xx.h"



   
 
typedef int32_t  s32;
typedef int16_t s16;
typedef int8_t  s8;

typedef const int32_t sc32;   
typedef const int16_t sc16;   
typedef const int8_t sc8;    

typedef volatile int32_t  vs32;
typedef volatile int16_t  vs16;
typedef volatile int8_t   vs8;

typedef volatile const int32_t vsc32;   
typedef volatile const int16_t vsc16;   
typedef volatile const int8_t vsc8;    

typedef uint32_t  u32;
typedef uint16_t u16;
typedef uint8_t  u8;

typedef const uint32_t uc32;   
typedef const uint16_t uc16;   
typedef const uint8_t uc8;    

typedef volatile uint32_t  vu32;
typedef volatile uint16_t vu16;
typedef volatile uint8_t  vu8;

typedef volatile const uint32_t vuc32;   
typedef volatile const uint16_t vuc16;   
typedef volatile const uint8_t vuc8;    

typedef enum {RESET = 0, SET = !RESET} FlagStatus, ITStatus;

typedef enum {DISABLE = 0, ENABLE = !DISABLE} FunctionalState;


typedef enum {ERROR = 0, SUCCESS = !ERROR} ErrorStatus;



 



    



 

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
  volatile uint32_t SR;        
  volatile uint32_t RISR;      
  volatile uint32_t IER;       
  volatile uint32_t MISR;      
  volatile uint32_t ICR;       
  volatile uint32_t ESCR;      
  volatile uint32_t ESUR;      
  volatile uint32_t CWSTRTR;   
  volatile uint32_t CWSIZER;   
  volatile uint32_t DR;        
} DCMI_TypeDef;



 

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
  volatile uint32_t MACCR;
  volatile uint32_t MACFFR;
  volatile uint32_t MACHTHR;
  volatile uint32_t MACHTLR;
  volatile uint32_t MACMIIAR;
  volatile uint32_t MACMIIDR;
  volatile uint32_t MACFCR;
  volatile uint32_t MACVLANTR;              
  uint32_t      RESERVED0[2];
  volatile uint32_t MACRWUFFR;              
  volatile uint32_t MACPMTCSR;
  uint32_t      RESERVED1[2];
  volatile uint32_t MACSR;                  
  volatile uint32_t MACIMR;
  volatile uint32_t MACA0HR;
  volatile uint32_t MACA0LR;
  volatile uint32_t MACA1HR;
  volatile uint32_t MACA1LR;
  volatile uint32_t MACA2HR;
  volatile uint32_t MACA2LR;
  volatile uint32_t MACA3HR;
  volatile uint32_t MACA3LR;                
  uint32_t      RESERVED2[40];
  volatile uint32_t MMCCR;                  
  volatile uint32_t MMCRIR;
  volatile uint32_t MMCTIR;
  volatile uint32_t MMCRIMR;
  volatile uint32_t MMCTIMR;                
  uint32_t      RESERVED3[14];
  volatile uint32_t MMCTGFSCCR;             
  volatile uint32_t MMCTGFMSCCR;
  uint32_t      RESERVED4[5];
  volatile uint32_t MMCTGFCR;
  uint32_t      RESERVED5[10];
  volatile uint32_t MMCRFCECR;
  volatile uint32_t MMCRFAECR;
  uint32_t      RESERVED6[10];
  volatile uint32_t MMCRGUFCR;
  uint32_t      RESERVED7[334];
  volatile uint32_t PTPTSCR;
  volatile uint32_t PTPSSIR;
  volatile uint32_t PTPTSHR;
  volatile uint32_t PTPTSLR;
  volatile uint32_t PTPTSHUR;
  volatile uint32_t PTPTSLUR;
  volatile uint32_t PTPTSAR;
  volatile uint32_t PTPTTHR;
  volatile uint32_t PTPTTLR;
  volatile uint32_t RESERVED8;
  volatile uint32_t PTPTSSR;
  uint32_t      RESERVED9[565];
  volatile uint32_t DMABMR;
  volatile uint32_t DMATPDR;
  volatile uint32_t DMARPDR;
  volatile uint32_t DMARDLAR;
  volatile uint32_t DMATDLAR;
  volatile uint32_t DMASR;
  volatile uint32_t DMAOMR;
  volatile uint32_t DMAIER;
  volatile uint32_t DMAMFBOCR;
  volatile uint32_t DMARSWTR;
  uint32_t      RESERVED10[8];
  volatile uint32_t DMACHTDR;
  volatile uint32_t DMACHRDR;
  volatile uint32_t DMACHTBAR;
  volatile uint32_t DMACHRBAR;
} ETH_TypeDef;



 

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
} FSMC_Bank2_TypeDef;



 
  
typedef struct
{
  volatile uint32_t PCR3;        
  volatile uint32_t SR3;         
  volatile uint32_t PMEM3;       
  volatile uint32_t PATT3;       
  uint32_t      RESERVED0;   
  volatile uint32_t ECCR3;       
} FSMC_Bank3_TypeDef;



 
  
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
  volatile uint16_t CR1;         
  uint16_t      RESERVED0;   
  volatile uint16_t CR2;         
  uint16_t      RESERVED1;   
  volatile uint16_t OAR1;        
  uint16_t      RESERVED2;   
  volatile uint16_t OAR2;        
  uint16_t      RESERVED3;   
  volatile uint16_t DR;          
  uint16_t      RESERVED4;   
  volatile uint16_t SR1;         
  uint16_t      RESERVED5;   
  volatile uint16_t SR2;         
  uint16_t      RESERVED6;   
  volatile uint16_t CCR;         
  uint16_t      RESERVED7;   
  volatile uint16_t TRISE;       
  uint16_t      RESERVED8;   
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
  volatile uint32_t SSR;      
  volatile uint32_t SHIFTR;   
  volatile uint32_t TSTR;     
  volatile uint32_t TSDR;     
  volatile uint32_t TSSSR;    
  volatile uint32_t CALR;     
  volatile uint32_t TAFCR;    
  volatile uint32_t ALRMASSR; 
  volatile uint32_t ALRMBSSR; 
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
  volatile uint16_t CR1;         
  uint16_t      RESERVED0;   
  volatile uint16_t CR2;         
  uint16_t      RESERVED1;   
  volatile uint16_t SR;          
  uint16_t      RESERVED2;   
  volatile uint16_t DR;          
  uint16_t      RESERVED3;   
  volatile uint16_t CRCPR;       
  uint16_t      RESERVED4;   
  volatile uint16_t RXCRCR;      
  uint16_t      RESERVED5;   
  volatile uint16_t TXCRCR;      
  uint16_t      RESERVED6;   
  volatile uint16_t I2SCFGR;     
  uint16_t      RESERVED7;   
  volatile uint16_t I2SPR;       
  uint16_t      RESERVED8;   
} SPI_TypeDef;



 

typedef struct
{
  volatile uint16_t CR1;          
  uint16_t      RESERVED0;    
  volatile uint16_t CR2;          
  uint16_t      RESERVED1;    
  volatile uint16_t SMCR;         
  uint16_t      RESERVED2;    
  volatile uint16_t DIER;         
  uint16_t      RESERVED3;    
  volatile uint16_t SR;           
  uint16_t      RESERVED4;    
  volatile uint16_t EGR;          
  uint16_t      RESERVED5;    
  volatile uint16_t CCMR1;        
  uint16_t      RESERVED6;    
  volatile uint16_t CCMR2;        
  uint16_t      RESERVED7;    
  volatile uint16_t CCER;         
  uint16_t      RESERVED8;    
  volatile uint32_t CNT;          
  volatile uint16_t PSC;          
  uint16_t      RESERVED9;    
  volatile uint32_t ARR;          
  volatile uint16_t RCR;          
  uint16_t      RESERVED10;   
  volatile uint32_t CCR1;         
  volatile uint32_t CCR2;         
  volatile uint32_t CCR3;         
  volatile uint32_t CCR4;         
  volatile uint16_t BDTR;         
  uint16_t      RESERVED11;   
  volatile uint16_t DCR;          
  uint16_t      RESERVED12;   
  volatile uint16_t DMAR;         
  uint16_t      RESERVED13;   
  volatile uint16_t OR;           
  uint16_t      RESERVED14;   
} TIM_TypeDef;



 
 
typedef struct
{
  volatile uint16_t SR;          
  uint16_t      RESERVED0;   
  volatile uint16_t DR;          
  uint16_t      RESERVED1;   
  volatile uint16_t BRR;         
  uint16_t      RESERVED2;   
  volatile uint16_t CR1;         
  uint16_t      RESERVED3;   
  volatile uint16_t CR2;         
  uint16_t      RESERVED4;   
  volatile uint16_t CR3;         
  uint16_t      RESERVED5;   
  volatile uint16_t GTPR;        
  uint16_t      RESERVED6;   
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



 
  


 
#line 1015 ".\\Include\\stm32f4xx.h"







 




 





 
#line 1061 ".\\Include\\stm32f4xx.h"

 
#line 1078 ".\\Include\\stm32f4xx.h"

 
#line 1115 ".\\Include\\stm32f4xx.h"

 





 






 




 
  


   
#line 1222 ".\\Include\\stm32f4xx.h"



 



 
  
  

 
    
 
 
 

 
 
 
 
 
 
#line 1251 ".\\Include\\stm32f4xx.h"

 
#line 1277 ".\\Include\\stm32f4xx.h"
  
 
#line 1303 ".\\Include\\stm32f4xx.h"

 
#line 1341 ".\\Include\\stm32f4xx.h"

 
#line 1383 ".\\Include\\stm32f4xx.h"

 


 


 


 


 


 


 
#line 1432 ".\\Include\\stm32f4xx.h"

 
#line 1470 ".\\Include\\stm32f4xx.h"

 
#line 1508 ".\\Include\\stm32f4xx.h"

 
#line 1537 ".\\Include\\stm32f4xx.h"

 


 


 


 


 



 
#line 1573 ".\\Include\\stm32f4xx.h"

 
#line 1595 ".\\Include\\stm32f4xx.h"

 



 
 
 
 
 
 
 
#line 1616 ".\\Include\\stm32f4xx.h"

 
#line 1627 ".\\Include\\stm32f4xx.h"

 
#line 1645 ".\\Include\\stm32f4xx.h"











 





 





 
#line 1683 ".\\Include\\stm32f4xx.h"

 












 
#line 1704 ".\\Include\\stm32f4xx.h"

 
 






 




 





 





 






 




 





 





 






   




 





 





 





 




 





 





 





 




 





 





 
 


 
#line 1844 ".\\Include\\stm32f4xx.h"

 
#line 1861 ".\\Include\\stm32f4xx.h"

 
#line 1878 ".\\Include\\stm32f4xx.h"

 
#line 1895 ".\\Include\\stm32f4xx.h"

 
#line 1929 ".\\Include\\stm32f4xx.h"

 
#line 1963 ".\\Include\\stm32f4xx.h"

 
#line 1997 ".\\Include\\stm32f4xx.h"

 
#line 2031 ".\\Include\\stm32f4xx.h"

 
#line 2065 ".\\Include\\stm32f4xx.h"

 
#line 2099 ".\\Include\\stm32f4xx.h"

 
#line 2133 ".\\Include\\stm32f4xx.h"

 
#line 2167 ".\\Include\\stm32f4xx.h"

 
#line 2201 ".\\Include\\stm32f4xx.h"

 
#line 2235 ".\\Include\\stm32f4xx.h"

 
#line 2269 ".\\Include\\stm32f4xx.h"

 
#line 2303 ".\\Include\\stm32f4xx.h"

 
#line 2337 ".\\Include\\stm32f4xx.h"

 
#line 2371 ".\\Include\\stm32f4xx.h"

 
#line 2405 ".\\Include\\stm32f4xx.h"

 
#line 2439 ".\\Include\\stm32f4xx.h"

 
#line 2473 ".\\Include\\stm32f4xx.h"

 
#line 2507 ".\\Include\\stm32f4xx.h"

 
#line 2541 ".\\Include\\stm32f4xx.h"

 
#line 2575 ".\\Include\\stm32f4xx.h"

 
#line 2609 ".\\Include\\stm32f4xx.h"

 
#line 2643 ".\\Include\\stm32f4xx.h"

 
#line 2677 ".\\Include\\stm32f4xx.h"

 
#line 2711 ".\\Include\\stm32f4xx.h"

 
#line 2745 ".\\Include\\stm32f4xx.h"

 
#line 2779 ".\\Include\\stm32f4xx.h"

 
#line 2813 ".\\Include\\stm32f4xx.h"

 
#line 2847 ".\\Include\\stm32f4xx.h"

 
 
 
 
 
 



 



 


 
 
 
 
 
 


#line 2884 ".\\Include\\stm32f4xx.h"

#line 2893 ".\\Include\\stm32f4xx.h"
 





 


 


 


 



 
 
 
 
 
 









































 



 


 


 


 


 


 


 



 



 



 


 


 



 
 
 
 
 

 
 
 
 
 
 
#line 3029 ".\\Include\\stm32f4xx.h"

 




 






 






 






 






 
 
 
 
 
  
#line 3104 ".\\Include\\stm32f4xx.h"

 
#line 3123 ".\\Include\\stm32f4xx.h"

  
#line 3134 ".\\Include\\stm32f4xx.h"

  
#line 3156 ".\\Include\\stm32f4xx.h"

  
#line 3178 ".\\Include\\stm32f4xx.h"

  
#line 3200 ".\\Include\\stm32f4xx.h"

  
#line 3222 ".\\Include\\stm32f4xx.h"

 
 
 
 
 
 
#line 3249 ".\\Include\\stm32f4xx.h"

 
#line 3271 ".\\Include\\stm32f4xx.h"

 
#line 3293 ".\\Include\\stm32f4xx.h"

 
#line 3315 ".\\Include\\stm32f4xx.h"

 
#line 3337 ".\\Include\\stm32f4xx.h"

 
#line 3359 ".\\Include\\stm32f4xx.h"

 
 
 
 
 
 
#line 3375 ".\\Include\\stm32f4xx.h"

#line 3383 ".\\Include\\stm32f4xx.h"

 
#line 3392 ".\\Include\\stm32f4xx.h"

 
#line 3406 ".\\Include\\stm32f4xx.h"

 
#line 3436 ".\\Include\\stm32f4xx.h"

 
 
 
 
 
 











#line 3464 ".\\Include\\stm32f4xx.h"

 











#line 3487 ".\\Include\\stm32f4xx.h"

 











#line 3510 ".\\Include\\stm32f4xx.h"

 











#line 3533 ".\\Include\\stm32f4xx.h"

 








































 








































 








































 








































 


































 


































 


































 


































 



























 



























 



























 
#line 3930 ".\\Include\\stm32f4xx.h"

 
#line 3939 ".\\Include\\stm32f4xx.h"

 
#line 3948 ".\\Include\\stm32f4xx.h"

 
#line 3959 ".\\Include\\stm32f4xx.h"

#line 3969 ".\\Include\\stm32f4xx.h"

#line 3979 ".\\Include\\stm32f4xx.h"

#line 3989 ".\\Include\\stm32f4xx.h"

 
#line 4000 ".\\Include\\stm32f4xx.h"

#line 4010 ".\\Include\\stm32f4xx.h"

#line 4020 ".\\Include\\stm32f4xx.h"

#line 4030 ".\\Include\\stm32f4xx.h"

 
#line 4041 ".\\Include\\stm32f4xx.h"

#line 4051 ".\\Include\\stm32f4xx.h"

#line 4061 ".\\Include\\stm32f4xx.h"

#line 4071 ".\\Include\\stm32f4xx.h"

 
#line 4082 ".\\Include\\stm32f4xx.h"

#line 4092 ".\\Include\\stm32f4xx.h"

#line 4102 ".\\Include\\stm32f4xx.h"

#line 4112 ".\\Include\\stm32f4xx.h"

 
#line 4123 ".\\Include\\stm32f4xx.h"

#line 4133 ".\\Include\\stm32f4xx.h"

#line 4143 ".\\Include\\stm32f4xx.h"

#line 4153 ".\\Include\\stm32f4xx.h"

 
#line 4164 ".\\Include\\stm32f4xx.h"

#line 4174 ".\\Include\\stm32f4xx.h"

#line 4184 ".\\Include\\stm32f4xx.h"

#line 4194 ".\\Include\\stm32f4xx.h"

 
#line 4205 ".\\Include\\stm32f4xx.h"

#line 4215 ".\\Include\\stm32f4xx.h"

#line 4225 ".\\Include\\stm32f4xx.h"

#line 4235 ".\\Include\\stm32f4xx.h"

 


 


 
 
 
 
 
 
































































 
#line 4329 ".\\Include\\stm32f4xx.h"

 
































































 
































































 
#line 4477 ".\\Include\\stm32f4xx.h"
 
#line 4494 ".\\Include\\stm32f4xx.h"

 
#line 4512 ".\\Include\\stm32f4xx.h"
 
#line 4529 ".\\Include\\stm32f4xx.h"

 
#line 4563 ".\\Include\\stm32f4xx.h"

 
 
 
 
 
 
#line 4584 ".\\Include\\stm32f4xx.h"

 
#line 4593 ".\\Include\\stm32f4xx.h"

 



 





 
 
 
 
 
 
#line 4624 ".\\Include\\stm32f4xx.h"

 
#line 4633 ".\\Include\\stm32f4xx.h"







 



#line 4654 ".\\Include\\stm32f4xx.h"



 



 


 
#line 4679 ".\\Include\\stm32f4xx.h"

 
#line 4689 ".\\Include\\stm32f4xx.h"

 




 


 
 
 
 
 
 


 





 


 



 
 
 
 
 
 












 
#line 4746 ".\\Include\\stm32f4xx.h"




 


 
#line 4761 ".\\Include\\stm32f4xx.h"
 


 
 
 
 
 
 



#line 4779 ".\\Include\\stm32f4xx.h"

#line 4789 ".\\Include\\stm32f4xx.h"

#line 4798 ".\\Include\\stm32f4xx.h"

 
#line 4807 ".\\Include\\stm32f4xx.h"

#line 4818 ".\\Include\\stm32f4xx.h"















 
 








 








 






#line 4868 ".\\Include\\stm32f4xx.h"

 











 











 
#line 4900 ".\\Include\\stm32f4xx.h"

 




















 
#line 4943 ".\\Include\\stm32f4xx.h"

 
#line 4959 ".\\Include\\stm32f4xx.h"

 






 


 
#line 4994 ".\\Include\\stm32f4xx.h"

 
#line 5007 ".\\Include\\stm32f4xx.h"
 


 
#line 5031 ".\\Include\\stm32f4xx.h"

 






 


 
#line 5066 ".\\Include\\stm32f4xx.h"

 
#line 5081 ".\\Include\\stm32f4xx.h"

 
#line 5105 ".\\Include\\stm32f4xx.h"

 






 


 
#line 5140 ".\\Include\\stm32f4xx.h"

 
#line 5155 ".\\Include\\stm32f4xx.h"

 











 
#line 5179 ".\\Include\\stm32f4xx.h"

 





 



 
 
 
 
 
 



 






 
 
 
 
 
 
#line 5239 ".\\Include\\stm32f4xx.h"

 
#line 5269 ".\\Include\\stm32f4xx.h"

 
#line 5297 ".\\Include\\stm32f4xx.h"

 
#line 5314 ".\\Include\\stm32f4xx.h"

 



 


 



 
#line 5367 ".\\Include\\stm32f4xx.h"

 
#line 5409 ".\\Include\\stm32f4xx.h"

 


 


 



 
#line 5448 ".\\Include\\stm32f4xx.h"

 
#line 5468 ".\\Include\\stm32f4xx.h"

 


 
#line 5486 ".\\Include\\stm32f4xx.h"

 
#line 5506 ".\\Include\\stm32f4xx.h"

 
#line 5514 ".\\Include\\stm32f4xx.h"

 
#line 5522 ".\\Include\\stm32f4xx.h"

 


 


 


 


 


 


 


 


 


 


 


 


 


 


 


 


 


 


 


 


 
 
 
 
 
 




 












 


 






#line 5623 ".\\Include\\stm32f4xx.h"

 


 


 


 


 


 


 


 


 
















 


 
#line 5693 ".\\Include\\stm32f4xx.h"

 
#line 5708 ".\\Include\\stm32f4xx.h"

 
#line 5734 ".\\Include\\stm32f4xx.h"

 


 


 
 
 
 
 
 









#line 5766 ".\\Include\\stm32f4xx.h"

 
#line 5774 ".\\Include\\stm32f4xx.h"

 
#line 5784 ".\\Include\\stm32f4xx.h"

 


 


 


 


 





















 




 
 
 
 
 
   




 

 


 






  
#line 5856 ".\\Include\\stm32f4xx.h"


  
#line 5868 ".\\Include\\stm32f4xx.h"


  
#line 5880 ".\\Include\\stm32f4xx.h"


  
#line 5892 ".\\Include\\stm32f4xx.h"

 






  
#line 5910 ".\\Include\\stm32f4xx.h"


  
#line 5922 ".\\Include\\stm32f4xx.h"


  
#line 5934 ".\\Include\\stm32f4xx.h"


  
#line 5946 ".\\Include\\stm32f4xx.h"

 




           


  
#line 5965 ".\\Include\\stm32f4xx.h"


  
#line 5977 ".\\Include\\stm32f4xx.h"


  
#line 5989 ".\\Include\\stm32f4xx.h"


  
#line 6001 ".\\Include\\stm32f4xx.h"

 






  
#line 6018 ".\\Include\\stm32f4xx.h"


  
#line 6029 ".\\Include\\stm32f4xx.h"


  
#line 6040 ".\\Include\\stm32f4xx.h"


  
#line 6051 ".\\Include\\stm32f4xx.h"

   



 
 
 
 
 
 
















 









#line 6096 ".\\Include\\stm32f4xx.h"

 

























 
#line 6139 ".\\Include\\stm32f4xx.h"

 
#line 6153 ".\\Include\\stm32f4xx.h"

 
#line 6163 ".\\Include\\stm32f4xx.h"

 




























 





















 




























 





















 
#line 6282 ".\\Include\\stm32f4xx.h"

 


 


 


 


 


 


 


 


 
#line 6317 ".\\Include\\stm32f4xx.h"





#line 6328 ".\\Include\\stm32f4xx.h"

 
#line 6336 ".\\Include\\stm32f4xx.h"

#line 6343 ".\\Include\\stm32f4xx.h"

 


 
#line 6354 ".\\Include\\stm32f4xx.h"


 
 
 
 
 
 
#line 6372 ".\\Include\\stm32f4xx.h"

 


 



 
#line 6396 ".\\Include\\stm32f4xx.h"

 
#line 6405 ".\\Include\\stm32f4xx.h"







 
#line 6425 ".\\Include\\stm32f4xx.h"

 
#line 6436 ".\\Include\\stm32f4xx.h"



 
 
 
 
 
 
#line 6453 ".\\Include\\stm32f4xx.h"



 
#line 6465 ".\\Include\\stm32f4xx.h"







 



 
 
 
 
 
 



 









 
#line 6513 ".\\Include\\stm32f4xx.h"
 


 






 
 
 
 
 
 
#line 6557 ".\\Include\\stm32f4xx.h"

 
#line 6573 ".\\Include\\stm32f4xx.h"

 


 


 
#line 6591 ".\\Include\\stm32f4xx.h"
  
 


 
#line 6607 ".\\Include\\stm32f4xx.h"

 



  


 








 

  
#line 6634 ".\\Include\\stm32f4xx.h"

 






 



 


 


 
#line 6663 ".\\Include\\stm32f4xx.h"

 


 
#line 6678 ".\\Include\\stm32f4xx.h"

 


 
#line 6693 ".\\Include\\stm32f4xx.h"

 


 
 
 

 
#line 6708 ".\\Include\\stm32f4xx.h"

 




 




 




 




 


 


 


 


 


 


 
 
 

 
#line 6761 ".\\Include\\stm32f4xx.h"

#line 6768 ".\\Include\\stm32f4xx.h"

 


 


 



 


 



 


 


 


 



 
 
 

 
#line 6843 ".\\Include\\stm32f4xx.h"

 


 


 


 


 




   
#line 6894 ".\\Include\\stm32f4xx.h"

 
#line 6920 ".\\Include\\stm32f4xx.h"

 
#line 6937 ".\\Include\\stm32f4xx.h"

 





 


 


 


 




 

 

  

#line 1 "C:\\STM32F4-Discovery_FW_V1.1.0\\Libraries\\STM32F4xx_StdPeriph_Driver\\inc\\stm32f4xx_conf.h"



















  

 




 




 
 
#line 1 "C:\\STM32F4-Discovery_FW_V1.1.0\\Libraries\\STM32F4xx_StdPeriph_Driver\\inc\\stm32f4xx_adc.h"




















 

 







 
#line 1 ".\\Include\\stm32f4xx.h"




































  



 



 
    
#line 6995 ".\\Include\\stm32f4xx.h"



 

  

 

 
#line 33 "C:\\STM32F4-Discovery_FW_V1.1.0\\Libraries\\STM32F4xx_StdPeriph_Driver\\inc\\stm32f4xx_adc.h"



 



  

 



  
typedef struct
{
  uint32_t ADC_Resolution;                
                                    
  FunctionalState ADC_ScanConvMode;       


  
  FunctionalState ADC_ContinuousConvMode; 

 
  uint32_t ADC_ExternalTrigConvEdge;      


 
  uint32_t ADC_ExternalTrigConv;          


 
  uint32_t ADC_DataAlign;                 

 
  uint8_t  ADC_NbrOfConversion;           


 
}ADC_InitTypeDef;
  


  
typedef struct 
{
  uint32_t ADC_Mode;                      

                                               
  uint32_t ADC_Prescaler;                 

 
  uint32_t ADC_DMAAccessMode;             


 
  uint32_t ADC_TwoSamplingDelay;          

 
  
}ADC_CommonInitTypeDef;


 



  






  
#line 135 "C:\\STM32F4-Discovery_FW_V1.1.0\\Libraries\\STM32F4xx_StdPeriph_Driver\\inc\\stm32f4xx_adc.h"


  




  
#line 151 "C:\\STM32F4-Discovery_FW_V1.1.0\\Libraries\\STM32F4xx_StdPeriph_Driver\\inc\\stm32f4xx_adc.h"


  




  
#line 167 "C:\\STM32F4-Discovery_FW_V1.1.0\\Libraries\\STM32F4xx_StdPeriph_Driver\\inc\\stm32f4xx_adc.h"
                                     


  




  
#line 208 "C:\\STM32F4-Discovery_FW_V1.1.0\\Libraries\\STM32F4xx_StdPeriph_Driver\\inc\\stm32f4xx_adc.h"
                                     


  




  
#line 225 "C:\\STM32F4-Discovery_FW_V1.1.0\\Libraries\\STM32F4xx_StdPeriph_Driver\\inc\\stm32f4xx_adc.h"
                                      


  




  
#line 242 "C:\\STM32F4-Discovery_FW_V1.1.0\\Libraries\\STM32F4xx_StdPeriph_Driver\\inc\\stm32f4xx_adc.h"


  




  
#line 282 "C:\\STM32F4-Discovery_FW_V1.1.0\\Libraries\\STM32F4xx_StdPeriph_Driver\\inc\\stm32f4xx_adc.h"


  




  






  




  
#line 321 "C:\\STM32F4-Discovery_FW_V1.1.0\\Libraries\\STM32F4xx_StdPeriph_Driver\\inc\\stm32f4xx_adc.h"





#line 345 "C:\\STM32F4-Discovery_FW_V1.1.0\\Libraries\\STM32F4xx_StdPeriph_Driver\\inc\\stm32f4xx_adc.h"


  




  
#line 369 "C:\\STM32F4-Discovery_FW_V1.1.0\\Libraries\\STM32F4xx_StdPeriph_Driver\\inc\\stm32f4xx_adc.h"


  




  
#line 385 "C:\\STM32F4-Discovery_FW_V1.1.0\\Libraries\\STM32F4xx_StdPeriph_Driver\\inc\\stm32f4xx_adc.h"
                                            


  




  
#line 426 "C:\\STM32F4-Discovery_FW_V1.1.0\\Libraries\\STM32F4xx_StdPeriph_Driver\\inc\\stm32f4xx_adc.h"


  




  
#line 442 "C:\\STM32F4-Discovery_FW_V1.1.0\\Libraries\\STM32F4xx_StdPeriph_Driver\\inc\\stm32f4xx_adc.h"


  




  
#line 464 "C:\\STM32F4-Discovery_FW_V1.1.0\\Libraries\\STM32F4xx_StdPeriph_Driver\\inc\\stm32f4xx_adc.h"


  




  
#line 478 "C:\\STM32F4-Discovery_FW_V1.1.0\\Libraries\\STM32F4xx_StdPeriph_Driver\\inc\\stm32f4xx_adc.h"


  




  
#line 492 "C:\\STM32F4-Discovery_FW_V1.1.0\\Libraries\\STM32F4xx_StdPeriph_Driver\\inc\\stm32f4xx_adc.h"
  
#line 500 "C:\\STM32F4-Discovery_FW_V1.1.0\\Libraries\\STM32F4xx_StdPeriph_Driver\\inc\\stm32f4xx_adc.h"


  




  



  




  



  




  



  




  



  




  



  




  



  




  



  




  

 
   

   
void ADC_DeInit(void);

 
void ADC_Init(ADC_TypeDef* ADCx, ADC_InitTypeDef* ADC_InitStruct);
void ADC_StructInit(ADC_InitTypeDef* ADC_InitStruct);
void ADC_CommonInit(ADC_CommonInitTypeDef* ADC_CommonInitStruct);
void ADC_CommonStructInit(ADC_CommonInitTypeDef* ADC_CommonInitStruct);
void ADC_Cmd(ADC_TypeDef* ADCx, FunctionalState NewState);

 
void ADC_AnalogWatchdogCmd(ADC_TypeDef* ADCx, uint32_t ADC_AnalogWatchdog);
void ADC_AnalogWatchdogThresholdsConfig(ADC_TypeDef* ADCx, uint16_t HighThreshold,uint16_t LowThreshold);
void ADC_AnalogWatchdogSingleChannelConfig(ADC_TypeDef* ADCx, uint8_t ADC_Channel);

 
void ADC_TempSensorVrefintCmd(FunctionalState NewState);
void ADC_VBATCmd(FunctionalState NewState);

 
void ADC_RegularChannelConfig(ADC_TypeDef* ADCx, uint8_t ADC_Channel, uint8_t Rank, uint8_t ADC_SampleTime);
void ADC_SoftwareStartConv(ADC_TypeDef* ADCx);
FlagStatus ADC_GetSoftwareStartConvStatus(ADC_TypeDef* ADCx);
void ADC_EOCOnEachRegularChannelCmd(ADC_TypeDef* ADCx, FunctionalState NewState);
void ADC_ContinuousModeCmd(ADC_TypeDef* ADCx, FunctionalState NewState);
void ADC_DiscModeChannelCountConfig(ADC_TypeDef* ADCx, uint8_t Number);
void ADC_DiscModeCmd(ADC_TypeDef* ADCx, FunctionalState NewState);
uint16_t ADC_GetConversionValue(ADC_TypeDef* ADCx);
uint32_t ADC_GetMultiModeConversionValue(void);

 
void ADC_DMACmd(ADC_TypeDef* ADCx, FunctionalState NewState);
void ADC_DMARequestAfterLastTransferCmd(ADC_TypeDef* ADCx, FunctionalState NewState);
void ADC_MultiModeDMARequestAfterLastTransferCmd(FunctionalState NewState);

 
void ADC_InjectedChannelConfig(ADC_TypeDef* ADCx, uint8_t ADC_Channel, uint8_t Rank, uint8_t ADC_SampleTime);
void ADC_InjectedSequencerLengthConfig(ADC_TypeDef* ADCx, uint8_t Length);
void ADC_SetInjectedOffset(ADC_TypeDef* ADCx, uint8_t ADC_InjectedChannel, uint16_t Offset);
void ADC_ExternalTrigInjectedConvConfig(ADC_TypeDef* ADCx, uint32_t ADC_ExternalTrigInjecConv);
void ADC_ExternalTrigInjectedConvEdgeConfig(ADC_TypeDef* ADCx, uint32_t ADC_ExternalTrigInjecConvEdge);
void ADC_SoftwareStartInjectedConv(ADC_TypeDef* ADCx);
FlagStatus ADC_GetSoftwareStartInjectedConvCmdStatus(ADC_TypeDef* ADCx);
void ADC_AutoInjectedConvCmd(ADC_TypeDef* ADCx, FunctionalState NewState);
void ADC_InjectedDiscModeCmd(ADC_TypeDef* ADCx, FunctionalState NewState);
uint16_t ADC_GetInjectedConversionValue(ADC_TypeDef* ADCx, uint8_t ADC_InjectedChannel);

 
void ADC_ITConfig(ADC_TypeDef* ADCx, uint16_t ADC_IT, FunctionalState NewState);
FlagStatus ADC_GetFlagStatus(ADC_TypeDef* ADCx, uint8_t ADC_FLAG);
void ADC_ClearFlag(ADC_TypeDef* ADCx, uint8_t ADC_FLAG);
ITStatus ADC_GetITStatus(ADC_TypeDef* ADCx, uint16_t ADC_IT);
void ADC_ClearITPendingBit(ADC_TypeDef* ADCx, uint16_t ADC_IT);









  



  

 
#line 35 "C:\\STM32F4-Discovery_FW_V1.1.0\\Libraries\\STM32F4xx_StdPeriph_Driver\\inc\\stm32f4xx_conf.h"
#line 1 "C:\\STM32F4-Discovery_FW_V1.1.0\\Libraries\\STM32F4xx_StdPeriph_Driver\\inc\\stm32f4xx_can.h"




















 

 







 
#line 33 "C:\\STM32F4-Discovery_FW_V1.1.0\\Libraries\\STM32F4xx_StdPeriph_Driver\\inc\\stm32f4xx_can.h"



 



 

 






 
typedef struct
{
  uint16_t CAN_Prescaler;   
 
  
  uint8_t CAN_Mode;         
 

  uint8_t CAN_SJW;          


 

  uint8_t CAN_BS1;          

 

  uint8_t CAN_BS2;          
 
  
  FunctionalState CAN_TTCM; 
 
  
  FunctionalState CAN_ABOM;  
 

  FunctionalState CAN_AWUM;  
 

  FunctionalState CAN_NART;  
 

  FunctionalState CAN_RFLM;  
 

  FunctionalState CAN_TXFP;  
 
} CAN_InitTypeDef;



 
typedef struct
{
  uint16_t CAN_FilterIdHigh;         

 

  uint16_t CAN_FilterIdLow;          

 

  uint16_t CAN_FilterMaskIdHigh;     


 

  uint16_t CAN_FilterMaskIdLow;      


 

  uint16_t CAN_FilterFIFOAssignment; 
 
  
  uint8_t CAN_FilterNumber;           

  uint8_t CAN_FilterMode;            
 

  uint8_t CAN_FilterScale;           
 

  FunctionalState CAN_FilterActivation; 
 
} CAN_FilterInitTypeDef;



 
typedef struct
{
  uint32_t StdId;  
 

  uint32_t ExtId;  
 

  uint8_t IDE;     

 

  uint8_t RTR;     

 

  uint8_t DLC;     

 

  uint8_t Data[8]; 
 
} CanTxMsg;



 
typedef struct
{
  uint32_t StdId;  
 

  uint32_t ExtId;  
 

  uint8_t IDE;     

 

  uint8_t RTR;     

 

  uint8_t DLC;     
 

  uint8_t Data[8]; 
 

  uint8_t FMI;     

 
} CanRxMsg;

 



 



 





 




 



 












 


 


   










 
  



   





 



 









 



 
#line 283 "C:\\STM32F4-Discovery_FW_V1.1.0\\Libraries\\STM32F4xx_StdPeriph_Driver\\inc\\stm32f4xx_can.h"




 



 
#line 300 "C:\\STM32F4-Discovery_FW_V1.1.0\\Libraries\\STM32F4xx_StdPeriph_Driver\\inc\\stm32f4xx_can.h"




 



 



 



 



 



 







 



 







 



 





 




 



 



 



 






 



 





 




 



 




 




 



 





 	






 



 






 



 



 	




 



 



 




 




                                                          
#line 475 "C:\\STM32F4-Discovery_FW_V1.1.0\\Libraries\\STM32F4xx_StdPeriph_Driver\\inc\\stm32f4xx_can.h"


 



 

 

 

 




 
#line 499 "C:\\STM32F4-Discovery_FW_V1.1.0\\Libraries\\STM32F4xx_StdPeriph_Driver\\inc\\stm32f4xx_can.h"

 



 

 





#line 520 "C:\\STM32F4-Discovery_FW_V1.1.0\\Libraries\\STM32F4xx_StdPeriph_Driver\\inc\\stm32f4xx_can.h"








 

  


  


 
#line 543 "C:\\STM32F4-Discovery_FW_V1.1.0\\Libraries\\STM32F4xx_StdPeriph_Driver\\inc\\stm32f4xx_can.h"

 



 






 





#line 568 "C:\\STM32F4-Discovery_FW_V1.1.0\\Libraries\\STM32F4xx_StdPeriph_Driver\\inc\\stm32f4xx_can.h"

#line 575 "C:\\STM32F4-Discovery_FW_V1.1.0\\Libraries\\STM32F4xx_StdPeriph_Driver\\inc\\stm32f4xx_can.h"


 



 

 
   

  
void CAN_DeInit(CAN_TypeDef* CANx);

  
uint8_t CAN_Init(CAN_TypeDef* CANx, CAN_InitTypeDef* CAN_InitStruct);
void CAN_FilterInit(CAN_FilterInitTypeDef* CAN_FilterInitStruct);
void CAN_StructInit(CAN_InitTypeDef* CAN_InitStruct);
void CAN_SlaveStartBank(uint8_t CAN_BankNumber); 
void CAN_DBGFreeze(CAN_TypeDef* CANx, FunctionalState NewState);
void CAN_TTComModeCmd(CAN_TypeDef* CANx, FunctionalState NewState);

 
uint8_t CAN_Transmit(CAN_TypeDef* CANx, CanTxMsg* TxMessage);
uint8_t CAN_TransmitStatus(CAN_TypeDef* CANx, uint8_t TransmitMailbox);
void CAN_CancelTransmit(CAN_TypeDef* CANx, uint8_t Mailbox);

 
void CAN_Receive(CAN_TypeDef* CANx, uint8_t FIFONumber, CanRxMsg* RxMessage);
void CAN_FIFORelease(CAN_TypeDef* CANx, uint8_t FIFONumber);
uint8_t CAN_MessagePending(CAN_TypeDef* CANx, uint8_t FIFONumber);

 
uint8_t CAN_OperatingModeRequest(CAN_TypeDef* CANx, uint8_t CAN_OperatingMode);
uint8_t CAN_Sleep(CAN_TypeDef* CANx);
uint8_t CAN_WakeUp(CAN_TypeDef* CANx);

 
uint8_t CAN_GetLastErrorCode(CAN_TypeDef* CANx);
uint8_t CAN_GetReceiveErrorCounter(CAN_TypeDef* CANx);
uint8_t CAN_GetLSBTransmitErrorCounter(CAN_TypeDef* CANx);

 
void CAN_ITConfig(CAN_TypeDef* CANx, uint32_t CAN_IT, FunctionalState NewState);
FlagStatus CAN_GetFlagStatus(CAN_TypeDef* CANx, uint32_t CAN_FLAG);
void CAN_ClearFlag(CAN_TypeDef* CANx, uint32_t CAN_FLAG);
ITStatus CAN_GetITStatus(CAN_TypeDef* CANx, uint32_t CAN_IT);
void CAN_ClearITPendingBit(CAN_TypeDef* CANx, uint32_t CAN_IT);









 



 

 
#line 36 "C:\\STM32F4-Discovery_FW_V1.1.0\\Libraries\\STM32F4xx_StdPeriph_Driver\\inc\\stm32f4xx_conf.h"
#line 1 "C:\\STM32F4-Discovery_FW_V1.1.0\\Libraries\\STM32F4xx_StdPeriph_Driver\\inc\\stm32f4xx_crc.h"




















 

 







 
#line 33 "C:\\STM32F4-Discovery_FW_V1.1.0\\Libraries\\STM32F4xx_StdPeriph_Driver\\inc\\stm32f4xx_crc.h"



 



 

 
 



 



 

 
   

void CRC_ResetDR(void);
uint32_t CRC_CalcCRC(uint32_t Data);
uint32_t CRC_CalcBlockCRC(uint32_t pBuffer[], uint32_t BufferLength);
uint32_t CRC_GetCRC(void);
void CRC_SetIDRegister(uint8_t IDValue);
uint8_t CRC_GetIDRegister(void);









 



 

 
#line 37 "C:\\STM32F4-Discovery_FW_V1.1.0\\Libraries\\STM32F4xx_StdPeriph_Driver\\inc\\stm32f4xx_conf.h"
#line 1 "C:\\STM32F4-Discovery_FW_V1.1.0\\Libraries\\STM32F4xx_StdPeriph_Driver\\inc\\stm32f4xx_cryp.h"




















 

 







 
#line 33 "C:\\STM32F4-Discovery_FW_V1.1.0\\Libraries\\STM32F4xx_StdPeriph_Driver\\inc\\stm32f4xx_cryp.h"



 



  

 



  
typedef struct
{
  uint16_t CRYP_AlgoDir;   
 
  uint16_t CRYP_AlgoMode;  

 
  uint16_t CRYP_DataType;  
  
  uint16_t CRYP_KeySize;   

 
}CRYP_InitTypeDef;



  
typedef struct
{
  uint32_t CRYP_Key0Left;   
  uint32_t CRYP_Key0Right;  
  uint32_t CRYP_Key1Left;   
  uint32_t CRYP_Key1Right;  
  uint32_t CRYP_Key2Left;   
  uint32_t CRYP_Key2Right;  
  uint32_t CRYP_Key3Left;   
  uint32_t CRYP_Key3Right;  
}CRYP_KeyInitTypeDef;


  
typedef struct
{
  uint32_t CRYP_IV0Left;   
  uint32_t CRYP_IV0Right;  
  uint32_t CRYP_IV1Left;   
  uint32_t CRYP_IV1Right;  
}CRYP_IVInitTypeDef;



  
typedef struct
{
   
  uint32_t CR_bits9to2;
   
  uint32_t CRYP_IV0LR;
  uint32_t CRYP_IV0RR;
  uint32_t CRYP_IV1LR;
  uint32_t CRYP_IV1RR;
   
  uint32_t CRYP_K0LR;
  uint32_t CRYP_K0RR;
  uint32_t CRYP_K1LR;
  uint32_t CRYP_K1RR;
  uint32_t CRYP_K2LR;
  uint32_t CRYP_K2RR;
  uint32_t CRYP_K3LR;
  uint32_t CRYP_K3RR;
}CRYP_Context;


 



 



 







  
 


 

 



 



 





#line 154 "C:\\STM32F4-Discovery_FW_V1.1.0\\Libraries\\STM32F4xx_StdPeriph_Driver\\inc\\stm32f4xx_cryp.h"


  
 


 
#line 169 "C:\\STM32F4-Discovery_FW_V1.1.0\\Libraries\\STM32F4xx_StdPeriph_Driver\\inc\\stm32f4xx_cryp.h"


 
                                     


 
#line 182 "C:\\STM32F4-Discovery_FW_V1.1.0\\Libraries\\STM32F4xx_StdPeriph_Driver\\inc\\stm32f4xx_cryp.h"


 



 
#line 201 "C:\\STM32F4-Discovery_FW_V1.1.0\\Libraries\\STM32F4xx_StdPeriph_Driver\\inc\\stm32f4xx_cryp.h"

#line 209 "C:\\STM32F4-Discovery_FW_V1.1.0\\Libraries\\STM32F4xx_StdPeriph_Driver\\inc\\stm32f4xx_cryp.h"


 



 







 



 





 



 





  



  

 
 

 
void CRYP_DeInit(void);

 
void CRYP_Init(CRYP_InitTypeDef* CRYP_InitStruct);
void CRYP_StructInit(CRYP_InitTypeDef* CRYP_InitStruct);
void CRYP_KeyInit(CRYP_KeyInitTypeDef* CRYP_KeyInitStruct);
void CRYP_KeyStructInit(CRYP_KeyInitTypeDef* CRYP_KeyInitStruct);
void CRYP_IVInit(CRYP_IVInitTypeDef* CRYP_IVInitStruct);
void CRYP_IVStructInit(CRYP_IVInitTypeDef* CRYP_IVInitStruct);
void CRYP_Cmd(FunctionalState NewState);

 
void CRYP_DataIn(uint32_t Data);
uint32_t CRYP_DataOut(void);
void CRYP_FIFOFlush(void);

 
ErrorStatus CRYP_SaveContext(CRYP_Context* CRYP_ContextSave,
                             CRYP_KeyInitTypeDef* CRYP_KeyInitStruct);
void CRYP_RestoreContext(CRYP_Context* CRYP_ContextRestore);

 
void CRYP_DMACmd(uint8_t CRYP_DMAReq, FunctionalState NewState);

 
void CRYP_ITConfig(uint8_t CRYP_IT, FunctionalState NewState);
ITStatus CRYP_GetITStatus(uint8_t CRYP_IT);
FlagStatus CRYP_GetFlagStatus(uint8_t CRYP_FLAG);

 
ErrorStatus CRYP_AES_ECB(uint8_t Mode,
                         uint8_t *Key, uint16_t Keysize,
                         uint8_t *Input, uint32_t Ilength,
                         uint8_t *Output);

ErrorStatus CRYP_AES_CBC(uint8_t Mode,
                         uint8_t InitVectors[16],
                         uint8_t *Key, uint16_t Keysize,
                         uint8_t *Input, uint32_t Ilength,
                         uint8_t *Output);

ErrorStatus CRYP_AES_CTR(uint8_t Mode,
                         uint8_t InitVectors[16],
                         uint8_t *Key, uint16_t Keysize,
                         uint8_t *Input, uint32_t Ilength,
                         uint8_t *Output);

 
ErrorStatus CRYP_TDES_ECB(uint8_t Mode,
                           uint8_t Key[24], 
                           uint8_t *Input, uint32_t Ilength,
                           uint8_t *Output);

ErrorStatus CRYP_TDES_CBC(uint8_t Mode,
                          uint8_t Key[24],
                          uint8_t InitVectors[8],
                          uint8_t *Input, uint32_t Ilength,
                          uint8_t *Output);

 
ErrorStatus CRYP_DES_ECB(uint8_t Mode,
                         uint8_t Key[8],
                         uint8_t *Input, uint32_t Ilength,
                         uint8_t *Output);

ErrorStatus CRYP_DES_CBC(uint8_t Mode,
                         uint8_t Key[8],
                         uint8_t InitVectors[8],
                         uint8_t *Input,uint32_t Ilength,
                         uint8_t *Output);









 



  

 
#line 38 "C:\\STM32F4-Discovery_FW_V1.1.0\\Libraries\\STM32F4xx_StdPeriph_Driver\\inc\\stm32f4xx_conf.h"
#line 1 "C:\\STM32F4-Discovery_FW_V1.1.0\\Libraries\\STM32F4xx_StdPeriph_Driver\\inc\\stm32f4xx_dac.h"




















 

 







 
#line 33 "C:\\STM32F4-Discovery_FW_V1.1.0\\Libraries\\STM32F4xx_StdPeriph_Driver\\inc\\stm32f4xx_dac.h"



 



 

 



 

typedef struct
{
  uint32_t DAC_Trigger;                      
 

  uint32_t DAC_WaveGeneration;               

 

  uint32_t DAC_LFSRUnmask_TriangleAmplitude; 

 

  uint32_t DAC_OutputBuffer;                 
 
}DAC_InitTypeDef;

 



 



 

#line 83 "C:\\STM32F4-Discovery_FW_V1.1.0\\Libraries\\STM32F4xx_StdPeriph_Driver\\inc\\stm32f4xx_dac.h"




#line 96 "C:\\STM32F4-Discovery_FW_V1.1.0\\Libraries\\STM32F4xx_StdPeriph_Driver\\inc\\stm32f4xx_dac.h"



 



 

#line 111 "C:\\STM32F4-Discovery_FW_V1.1.0\\Libraries\\STM32F4xx_StdPeriph_Driver\\inc\\stm32f4xx_dac.h"


 



 

#line 143 "C:\\STM32F4-Discovery_FW_V1.1.0\\Libraries\\STM32F4xx_StdPeriph_Driver\\inc\\stm32f4xx_dac.h"

#line 168 "C:\\STM32F4-Discovery_FW_V1.1.0\\Libraries\\STM32F4xx_StdPeriph_Driver\\inc\\stm32f4xx_dac.h"


 



 







 



 







 



 

#line 206 "C:\\STM32F4-Discovery_FW_V1.1.0\\Libraries\\STM32F4xx_StdPeriph_Driver\\inc\\stm32f4xx_dac.h"


 



 







 



 




 
  


    





  



  
  





 



 

 
   

   
void DAC_DeInit(void);

 
void DAC_Init(uint32_t DAC_Channel, DAC_InitTypeDef* DAC_InitStruct);
void DAC_StructInit(DAC_InitTypeDef* DAC_InitStruct);
void DAC_Cmd(uint32_t DAC_Channel, FunctionalState NewState);
void DAC_SoftwareTriggerCmd(uint32_t DAC_Channel, FunctionalState NewState);
void DAC_DualSoftwareTriggerCmd(FunctionalState NewState);
void DAC_WaveGenerationCmd(uint32_t DAC_Channel, uint32_t DAC_Wave, FunctionalState NewState);
void DAC_SetChannel1Data(uint32_t DAC_Align, uint16_t Data);
void DAC_SetChannel2Data(uint32_t DAC_Align, uint16_t Data);
void DAC_SetDualChannelData(uint32_t DAC_Align, uint16_t Data2, uint16_t Data1);
uint16_t DAC_GetDataOutputValue(uint32_t DAC_Channel);

 
void DAC_DMACmd(uint32_t DAC_Channel, FunctionalState NewState);

 
void DAC_ITConfig(uint32_t DAC_Channel, uint32_t DAC_IT, FunctionalState NewState);
FlagStatus DAC_GetFlagStatus(uint32_t DAC_Channel, uint32_t DAC_FLAG);
void DAC_ClearFlag(uint32_t DAC_Channel, uint32_t DAC_FLAG);
ITStatus DAC_GetITStatus(uint32_t DAC_Channel, uint32_t DAC_IT);
void DAC_ClearITPendingBit(uint32_t DAC_Channel, uint32_t DAC_IT);









 



 

 
#line 39 "C:\\STM32F4-Discovery_FW_V1.1.0\\Libraries\\STM32F4xx_StdPeriph_Driver\\inc\\stm32f4xx_conf.h"
#line 1 "C:\\STM32F4-Discovery_FW_V1.1.0\\Libraries\\STM32F4xx_StdPeriph_Driver\\inc\\stm32f4xx_dbgmcu.h"



















 

 







 
#line 32 "C:\\STM32F4-Discovery_FW_V1.1.0\\Libraries\\STM32F4xx_StdPeriph_Driver\\inc\\stm32f4xx_dbgmcu.h"



 



  

 
 



  





#line 70 "C:\\STM32F4-Discovery_FW_V1.1.0\\Libraries\\STM32F4xx_StdPeriph_Driver\\inc\\stm32f4xx_dbgmcu.h"

#line 77 "C:\\STM32F4-Discovery_FW_V1.1.0\\Libraries\\STM32F4xx_StdPeriph_Driver\\inc\\stm32f4xx_dbgmcu.h"


  

 
  
uint32_t DBGMCU_GetREVID(void);
uint32_t DBGMCU_GetDEVID(void);
void DBGMCU_Config(uint32_t DBGMCU_Periph, FunctionalState NewState);
void DBGMCU_APB1PeriphConfig(uint32_t DBGMCU_Periph, FunctionalState NewState);
void DBGMCU_APB2PeriphConfig(uint32_t DBGMCU_Periph, FunctionalState NewState);









  



  

 
#line 40 "C:\\STM32F4-Discovery_FW_V1.1.0\\Libraries\\STM32F4xx_StdPeriph_Driver\\inc\\stm32f4xx_conf.h"
#line 1 "C:\\STM32F4-Discovery_FW_V1.1.0\\Libraries\\STM32F4xx_StdPeriph_Driver\\inc\\stm32f4xx_dcmi.h"



















 

 







 
#line 32 "C:\\STM32F4-Discovery_FW_V1.1.0\\Libraries\\STM32F4xx_StdPeriph_Driver\\inc\\stm32f4xx_dcmi.h"



 



  

 


  
typedef struct
{
  uint16_t DCMI_CaptureMode;      
 

  uint16_t DCMI_SynchroMode;      
 

  uint16_t DCMI_PCKPolarity;      
 

  uint16_t DCMI_VSPolarity;       
 

  uint16_t DCMI_HSPolarity;       
 

  uint16_t DCMI_CaptureRate;      
 

  uint16_t DCMI_ExtendedDataMode; 
 
} DCMI_InitTypeDef;



  
typedef struct
{
  uint16_t DCMI_VerticalStartLine;      
 

  uint16_t DCMI_HorizontalOffsetCount;  
 

  uint16_t DCMI_VerticalLineCount;      
 

  uint16_t DCMI_CaptureCount;           

 
} DCMI_CROPInitTypeDef;



  
typedef struct
{
  uint8_t DCMI_FrameStartCode;  
  uint8_t DCMI_LineStartCode;   
  uint8_t DCMI_LineEndCode;     
  uint8_t DCMI_FrameEndCode;    
} DCMI_CodesInitTypeDef;

 



 



  
#line 114 "C:\\STM32F4-Discovery_FW_V1.1.0\\Libraries\\STM32F4xx_StdPeriph_Driver\\inc\\stm32f4xx_dcmi.h"


  




  
#line 128 "C:\\STM32F4-Discovery_FW_V1.1.0\\Libraries\\STM32F4xx_StdPeriph_Driver\\inc\\stm32f4xx_dcmi.h"


  




  






  




  






  




  






  




  
#line 178 "C:\\STM32F4-Discovery_FW_V1.1.0\\Libraries\\STM32F4xx_StdPeriph_Driver\\inc\\stm32f4xx_dcmi.h"


  




  
#line 194 "C:\\STM32F4-Discovery_FW_V1.1.0\\Libraries\\STM32F4xx_StdPeriph_Driver\\inc\\stm32f4xx_dcmi.h"


  




  
#line 213 "C:\\STM32F4-Discovery_FW_V1.1.0\\Libraries\\STM32F4xx_StdPeriph_Driver\\inc\\stm32f4xx_dcmi.h"


  




  


  





  







  
#line 256 "C:\\STM32F4-Discovery_FW_V1.1.0\\Libraries\\STM32F4xx_StdPeriph_Driver\\inc\\stm32f4xx_dcmi.h"
                                



  



  

 
  

  
void DCMI_DeInit(void);

 
void DCMI_Init(DCMI_InitTypeDef* DCMI_InitStruct);
void DCMI_StructInit(DCMI_InitTypeDef* DCMI_InitStruct);
void DCMI_CROPConfig(DCMI_CROPInitTypeDef* DCMI_CROPInitStruct);
void DCMI_CROPCmd(FunctionalState NewState);
void DCMI_SetEmbeddedSynchroCodes(DCMI_CodesInitTypeDef* DCMI_CodesInitStruct);
void DCMI_JPEGCmd(FunctionalState NewState);

 
void DCMI_Cmd(FunctionalState NewState);
void DCMI_CaptureCmd(FunctionalState NewState);
uint32_t DCMI_ReadData(void);

 
void DCMI_ITConfig(uint16_t DCMI_IT, FunctionalState NewState);
FlagStatus DCMI_GetFlagStatus(uint16_t DCMI_FLAG);
void DCMI_ClearFlag(uint16_t DCMI_FLAG);
ITStatus DCMI_GetITStatus(uint16_t DCMI_IT);
void DCMI_ClearITPendingBit(uint16_t DCMI_IT);









  



  

 
#line 41 "C:\\STM32F4-Discovery_FW_V1.1.0\\Libraries\\STM32F4xx_StdPeriph_Driver\\inc\\stm32f4xx_conf.h"
#line 1 "C:\\STM32F4-Discovery_FW_V1.1.0\\Libraries\\STM32F4xx_StdPeriph_Driver\\inc\\stm32f4xx_dma.h"




















  

 







 
#line 33 "C:\\STM32F4-Discovery_FW_V1.1.0\\Libraries\\STM32F4xx_StdPeriph_Driver\\inc\\stm32f4xx_dma.h"



 



 

 



 

typedef struct
{
  uint32_t DMA_Channel;            
 
 
  uint32_t DMA_PeripheralBaseAddr;  

  uint32_t DMA_Memory0BaseAddr;    

 

  uint32_t DMA_DIR;                

 

  uint32_t DMA_BufferSize;         

 

  uint32_t DMA_PeripheralInc;      
 

  uint32_t DMA_MemoryInc;          
 

  uint32_t DMA_PeripheralDataSize; 
 

  uint32_t DMA_MemoryDataSize;     
 

  uint32_t DMA_Mode;               


 

  uint32_t DMA_Priority;           
 

  uint32_t DMA_FIFOMode;          


 

  uint32_t DMA_FIFOThreshold;      
 

  uint32_t DMA_MemoryBurst;        


 

  uint32_t DMA_PeripheralBurst;    


   
}DMA_InitTypeDef;

 



 

#line 128 "C:\\STM32F4-Discovery_FW_V1.1.0\\Libraries\\STM32F4xx_StdPeriph_Driver\\inc\\stm32f4xx_dma.h"






  
#line 143 "C:\\STM32F4-Discovery_FW_V1.1.0\\Libraries\\STM32F4xx_StdPeriph_Driver\\inc\\stm32f4xx_dma.h"

#line 152 "C:\\STM32F4-Discovery_FW_V1.1.0\\Libraries\\STM32F4xx_StdPeriph_Driver\\inc\\stm32f4xx_dma.h"


  




  









  




  



  




  







  




  







  




  









  




  









  




  







  




  











  




  







  




  











  




  











  




  











  




 
#line 340 "C:\\STM32F4-Discovery_FW_V1.1.0\\Libraries\\STM32F4xx_StdPeriph_Driver\\inc\\stm32f4xx_dma.h"

#line 347 "C:\\STM32F4-Discovery_FW_V1.1.0\\Libraries\\STM32F4xx_StdPeriph_Driver\\inc\\stm32f4xx_dma.h"


  



 
#line 394 "C:\\STM32F4-Discovery_FW_V1.1.0\\Libraries\\STM32F4xx_StdPeriph_Driver\\inc\\stm32f4xx_dma.h"




#line 418 "C:\\STM32F4-Discovery_FW_V1.1.0\\Libraries\\STM32F4xx_StdPeriph_Driver\\inc\\stm32f4xx_dma.h"


  




  









  




  
#line 481 "C:\\STM32F4-Discovery_FW_V1.1.0\\Libraries\\STM32F4xx_StdPeriph_Driver\\inc\\stm32f4xx_dma.h"





#line 506 "C:\\STM32F4-Discovery_FW_V1.1.0\\Libraries\\STM32F4xx_StdPeriph_Driver\\inc\\stm32f4xx_dma.h"


  




  







  




  







  




  






  



  

 
  

  
void DMA_DeInit(DMA_Stream_TypeDef* DMAy_Streamx);

 
void DMA_Init(DMA_Stream_TypeDef* DMAy_Streamx, DMA_InitTypeDef* DMA_InitStruct);
void DMA_StructInit(DMA_InitTypeDef* DMA_InitStruct);
void DMA_Cmd(DMA_Stream_TypeDef* DMAy_Streamx, FunctionalState NewState);

 
void DMA_PeriphIncOffsetSizeConfig(DMA_Stream_TypeDef* DMAy_Streamx, uint32_t DMA_Pincos);
void DMA_FlowControllerConfig(DMA_Stream_TypeDef* DMAy_Streamx, uint32_t DMA_FlowCtrl);

 
void DMA_SetCurrDataCounter(DMA_Stream_TypeDef* DMAy_Streamx, uint16_t Counter);
uint16_t DMA_GetCurrDataCounter(DMA_Stream_TypeDef* DMAy_Streamx);

 
void DMA_DoubleBufferModeConfig(DMA_Stream_TypeDef* DMAy_Streamx, uint32_t Memory1BaseAddr,
                                uint32_t DMA_CurrentMemory);
void DMA_DoubleBufferModeCmd(DMA_Stream_TypeDef* DMAy_Streamx, FunctionalState NewState);
void DMA_MemoryTargetConfig(DMA_Stream_TypeDef* DMAy_Streamx, uint32_t MemoryBaseAddr,
                            uint32_t DMA_MemoryTarget);
uint32_t DMA_GetCurrentMemoryTarget(DMA_Stream_TypeDef* DMAy_Streamx);

 
FunctionalState DMA_GetCmdStatus(DMA_Stream_TypeDef* DMAy_Streamx);
uint32_t DMA_GetFIFOStatus(DMA_Stream_TypeDef* DMAy_Streamx);
FlagStatus DMA_GetFlagStatus(DMA_Stream_TypeDef* DMAy_Streamx, uint32_t DMA_FLAG);
void DMA_ClearFlag(DMA_Stream_TypeDef* DMAy_Streamx, uint32_t DMA_FLAG);
void DMA_ITConfig(DMA_Stream_TypeDef* DMAy_Streamx, uint32_t DMA_IT, FunctionalState NewState);
ITStatus DMA_GetITStatus(DMA_Stream_TypeDef* DMAy_Streamx, uint32_t DMA_IT);
void DMA_ClearITPendingBit(DMA_Stream_TypeDef* DMAy_Streamx, uint32_t DMA_IT);









 



 


 
#line 42 "C:\\STM32F4-Discovery_FW_V1.1.0\\Libraries\\STM32F4xx_StdPeriph_Driver\\inc\\stm32f4xx_conf.h"
#line 1 "C:\\STM32F4-Discovery_FW_V1.1.0\\Libraries\\STM32F4xx_StdPeriph_Driver\\inc\\stm32f4xx_exti.h"




















 

 







 
#line 33 "C:\\STM32F4-Discovery_FW_V1.1.0\\Libraries\\STM32F4xx_StdPeriph_Driver\\inc\\stm32f4xx_exti.h"



 



 

 



 

typedef enum
{
  EXTI_Mode_Interrupt = 0x00,
  EXTI_Mode_Event = 0x04
}EXTIMode_TypeDef;





 

typedef enum
{
  EXTI_Trigger_Rising = 0x08,
  EXTI_Trigger_Falling = 0x0C,  
  EXTI_Trigger_Rising_Falling = 0x10
}EXTITrigger_TypeDef;






 

typedef struct
{
  uint32_t EXTI_Line;               
 
   
  EXTIMode_TypeDef EXTI_Mode;       
 

  EXTITrigger_TypeDef EXTI_Trigger; 
 

  FunctionalState EXTI_LineCmd;     
  
}EXTI_InitTypeDef;

 



 



 

#line 122 "C:\\STM32F4-Discovery_FW_V1.1.0\\Libraries\\STM32F4xx_StdPeriph_Driver\\inc\\stm32f4xx_exti.h"
                                          


#line 137 "C:\\STM32F4-Discovery_FW_V1.1.0\\Libraries\\STM32F4xx_StdPeriph_Driver\\inc\\stm32f4xx_exti.h"
                    


 



 

 
 

 
void EXTI_DeInit(void);

 
void EXTI_Init(EXTI_InitTypeDef* EXTI_InitStruct);
void EXTI_StructInit(EXTI_InitTypeDef* EXTI_InitStruct);
void EXTI_GenerateSWInterrupt(uint32_t EXTI_Line);

 
FlagStatus EXTI_GetFlagStatus(uint32_t EXTI_Line);
void EXTI_ClearFlag(uint32_t EXTI_Line);
ITStatus EXTI_GetITStatus(uint32_t EXTI_Line);
void EXTI_ClearITPendingBit(uint32_t EXTI_Line);









 



 

 
#line 43 "C:\\STM32F4-Discovery_FW_V1.1.0\\Libraries\\STM32F4xx_StdPeriph_Driver\\inc\\stm32f4xx_conf.h"
#line 1 "C:\\STM32F4-Discovery_FW_V1.1.0\\Libraries\\STM32F4xx_StdPeriph_Driver\\inc\\stm32f4xx_flash.h"




















 

 







 
#line 33 "C:\\STM32F4-Discovery_FW_V1.1.0\\Libraries\\STM32F4xx_StdPeriph_Driver\\inc\\stm32f4xx_flash.h"



 



  

 


  
typedef enum
{ 
  FLASH_BUSY = 1,
  FLASH_ERROR_PGS,
  FLASH_ERROR_PGP,
  FLASH_ERROR_PGA,
  FLASH_ERROR_WRP,
  FLASH_ERROR_PROGRAM,
  FLASH_ERROR_OPERATION,
  FLASH_COMPLETE
}FLASH_Status;

 



   



  
#line 75 "C:\\STM32F4-Discovery_FW_V1.1.0\\Libraries\\STM32F4xx_StdPeriph_Driver\\inc\\stm32f4xx_flash.h"

#line 84 "C:\\STM32F4-Discovery_FW_V1.1.0\\Libraries\\STM32F4xx_StdPeriph_Driver\\inc\\stm32f4xx_flash.h"


  



  











  



  
#line 127 "C:\\STM32F4-Discovery_FW_V1.1.0\\Libraries\\STM32F4xx_StdPeriph_Driver\\inc\\stm32f4xx_flash.h"


  



  
#line 147 "C:\\STM32F4-Discovery_FW_V1.1.0\\Libraries\\STM32F4xx_StdPeriph_Driver\\inc\\stm32f4xx_flash.h"




 



 


  
 





  



  





  



  





  




  





 
  


   
#line 207 "C:\\STM32F4-Discovery_FW_V1.1.0\\Libraries\\STM32F4xx_StdPeriph_Driver\\inc\\stm32f4xx_flash.h"


 



  





  



  
#line 236 "C:\\STM32F4-Discovery_FW_V1.1.0\\Libraries\\STM32F4xx_StdPeriph_Driver\\inc\\stm32f4xx_flash.h"


 



 







  



  







  



  



  






  

 
  
 
 
void FLASH_SetLatency(uint32_t FLASH_Latency);
void FLASH_PrefetchBufferCmd(FunctionalState NewState);
void FLASH_InstructionCacheCmd(FunctionalState NewState);
void FLASH_DataCacheCmd(FunctionalState NewState);
void FLASH_InstructionCacheReset(void);
void FLASH_DataCacheReset(void);

    
void FLASH_Unlock(void);
void FLASH_Lock(void);
FLASH_Status FLASH_EraseSector(uint32_t FLASH_Sector, uint8_t VoltageRange);
FLASH_Status FLASH_EraseAllSectors(uint8_t VoltageRange);
FLASH_Status FLASH_ProgramDoubleWord(uint32_t Address, uint64_t Data);
FLASH_Status FLASH_ProgramWord(uint32_t Address, uint32_t Data);
FLASH_Status FLASH_ProgramHalfWord(uint32_t Address, uint16_t Data);
FLASH_Status FLASH_ProgramByte(uint32_t Address, uint8_t Data);

  
void FLASH_OB_Unlock(void);
void FLASH_OB_Lock(void);
void FLASH_OB_WRPConfig(uint32_t OB_WRP, FunctionalState NewState);
void FLASH_OB_RDPConfig(uint8_t OB_RDP);
void FLASH_OB_UserConfig(uint8_t OB_IWDG, uint8_t OB_STOP, uint8_t OB_STDBY);
void FLASH_OB_BORConfig(uint8_t OB_BOR);
FLASH_Status FLASH_OB_Launch(void);
uint8_t FLASH_OB_GetUser(void);
uint16_t FLASH_OB_GetWRP(void);
FlagStatus FLASH_OB_GetRDP(void);
uint8_t FLASH_OB_GetBOR(void);

 
void FLASH_ITConfig(uint32_t FLASH_IT, FunctionalState NewState);
FlagStatus FLASH_GetFlagStatus(uint32_t FLASH_FLAG);
void FLASH_ClearFlag(uint32_t FLASH_FLAG);
FLASH_Status FLASH_GetStatus(void);
FLASH_Status FLASH_WaitForLastOperation(void);









  



  

 
#line 44 "C:\\STM32F4-Discovery_FW_V1.1.0\\Libraries\\STM32F4xx_StdPeriph_Driver\\inc\\stm32f4xx_conf.h"
#line 1 "C:\\STM32F4-Discovery_FW_V1.1.0\\Libraries\\STM32F4xx_StdPeriph_Driver\\inc\\stm32f4xx_fsmc.h"




















 

 







 
#line 33 "C:\\STM32F4-Discovery_FW_V1.1.0\\Libraries\\STM32F4xx_StdPeriph_Driver\\inc\\stm32f4xx_fsmc.h"



 



 

 



 
typedef struct
{
  uint32_t FSMC_AddressSetupTime;       


 

  uint32_t FSMC_AddressHoldTime;        


 

  uint32_t FSMC_DataSetupTime;          


 

  uint32_t FSMC_BusTurnAroundDuration;  


 

  uint32_t FSMC_CLKDivision;            

 

  uint32_t FSMC_DataLatency;            





 

  uint32_t FSMC_AccessMode;             
 
}FSMC_NORSRAMTimingInitTypeDef;



 
typedef struct
{
  uint32_t FSMC_Bank;                
 

  uint32_t FSMC_DataAddressMux;      

 

  uint32_t FSMC_MemoryType;          

 

  uint32_t FSMC_MemoryDataWidth;     
 

  uint32_t FSMC_BurstAccessMode;     

 

  uint32_t FSMC_AsynchronousWait;     

                                           

  uint32_t FSMC_WaitSignalPolarity;  

 

  uint32_t FSMC_WrapMode;            

 

  uint32_t FSMC_WaitSignalActive;    


 

  uint32_t FSMC_WriteOperation;      
 

  uint32_t FSMC_WaitSignal;          

 

  uint32_t FSMC_ExtendedMode;        
 

  uint32_t FSMC_WriteBurst;          
  

  FSMC_NORSRAMTimingInitTypeDef* FSMC_ReadWriteTimingStruct;    

  FSMC_NORSRAMTimingInitTypeDef* FSMC_WriteTimingStruct;            
}FSMC_NORSRAMInitTypeDef;



 
typedef struct
{
  uint32_t FSMC_SetupTime;      



 

  uint32_t FSMC_WaitSetupTime;  



 

  uint32_t FSMC_HoldSetupTime;  




 

  uint32_t FSMC_HiZSetupTime;   



 
}FSMC_NAND_PCCARDTimingInitTypeDef;



 
typedef struct
{
  uint32_t FSMC_Bank;              
 

  uint32_t FSMC_Waitfeature;      
 

  uint32_t FSMC_MemoryDataWidth;  
 

  uint32_t FSMC_ECC;              
 

  uint32_t FSMC_ECCPageSize;      
 

  uint32_t FSMC_TCLRSetupTime;    

 

  uint32_t FSMC_TARSetupTime;     

  

  FSMC_NAND_PCCARDTimingInitTypeDef*  FSMC_CommonSpaceTimingStruct;     

  FSMC_NAND_PCCARDTimingInitTypeDef*  FSMC_AttributeSpaceTimingStruct;  
}FSMC_NANDInitTypeDef;



 

typedef struct
{
  uint32_t FSMC_Waitfeature;    
 

  uint32_t FSMC_TCLRSetupTime;  

 

  uint32_t FSMC_TARSetupTime;   

  

  
  FSMC_NAND_PCCARDTimingInitTypeDef*  FSMC_CommonSpaceTimingStruct;  

  FSMC_NAND_PCCARDTimingInitTypeDef*  FSMC_AttributeSpaceTimingStruct;    
  
  FSMC_NAND_PCCARDTimingInitTypeDef*  FSMC_IOSpaceTimingStruct;    
}FSMC_PCCARDInitTypeDef;

 



 



 






 



   




 



     



 



















 



 







 



 

#line 308 "C:\\STM32F4-Discovery_FW_V1.1.0\\Libraries\\STM32F4xx_StdPeriph_Driver\\inc\\stm32f4xx_fsmc.h"


 



 







 



 







 
    


 






 



 






 



 






 



 






 



 






 



 






 



 







 



 







 



 



 



 



 



 



 



 



 



 



 



 



 



 
#line 485 "C:\\STM32F4-Discovery_FW_V1.1.0\\Libraries\\STM32F4xx_StdPeriph_Driver\\inc\\stm32f4xx_fsmc.h"


 



 
  


 



 






 




 






 



 
#line 535 "C:\\STM32F4-Discovery_FW_V1.1.0\\Libraries\\STM32F4xx_StdPeriph_Driver\\inc\\stm32f4xx_fsmc.h"


 



 



 



 



 



 



 



 



 



 



 



 



 



 
#line 597 "C:\\STM32F4-Discovery_FW_V1.1.0\\Libraries\\STM32F4xx_StdPeriph_Driver\\inc\\stm32f4xx_fsmc.h"


 



 
#line 612 "C:\\STM32F4-Discovery_FW_V1.1.0\\Libraries\\STM32F4xx_StdPeriph_Driver\\inc\\stm32f4xx_fsmc.h"




 



 



 

 
  

 
void FSMC_NORSRAMDeInit(uint32_t FSMC_Bank);
void FSMC_NORSRAMInit(FSMC_NORSRAMInitTypeDef* FSMC_NORSRAMInitStruct);
void FSMC_NORSRAMStructInit(FSMC_NORSRAMInitTypeDef* FSMC_NORSRAMInitStruct);
void FSMC_NORSRAMCmd(uint32_t FSMC_Bank, FunctionalState NewState);

 
void FSMC_NANDDeInit(uint32_t FSMC_Bank);
void FSMC_NANDInit(FSMC_NANDInitTypeDef* FSMC_NANDInitStruct);
void FSMC_NANDStructInit(FSMC_NANDInitTypeDef* FSMC_NANDInitStruct);
void FSMC_NANDCmd(uint32_t FSMC_Bank, FunctionalState NewState);
void FSMC_NANDECCCmd(uint32_t FSMC_Bank, FunctionalState NewState);
uint32_t FSMC_GetECC(uint32_t FSMC_Bank);

 
void FSMC_PCCARDDeInit(void);
void FSMC_PCCARDInit(FSMC_PCCARDInitTypeDef* FSMC_PCCARDInitStruct);
void FSMC_PCCARDStructInit(FSMC_PCCARDInitTypeDef* FSMC_PCCARDInitStruct);
void FSMC_PCCARDCmd(FunctionalState NewState);

 
void FSMC_ITConfig(uint32_t FSMC_Bank, uint32_t FSMC_IT, FunctionalState NewState);
FlagStatus FSMC_GetFlagStatus(uint32_t FSMC_Bank, uint32_t FSMC_FLAG);
void FSMC_ClearFlag(uint32_t FSMC_Bank, uint32_t FSMC_FLAG);
ITStatus FSMC_GetITStatus(uint32_t FSMC_Bank, uint32_t FSMC_IT);
void FSMC_ClearITPendingBit(uint32_t FSMC_Bank, uint32_t FSMC_IT);








 



  

 
#line 45 "C:\\STM32F4-Discovery_FW_V1.1.0\\Libraries\\STM32F4xx_StdPeriph_Driver\\inc\\stm32f4xx_conf.h"
#line 1 "C:\\STM32F4-Discovery_FW_V1.1.0\\Libraries\\STM32F4xx_StdPeriph_Driver\\inc\\stm32f4xx_hash.h"




















 

 







 
#line 33 "C:\\STM32F4-Discovery_FW_V1.1.0\\Libraries\\STM32F4xx_StdPeriph_Driver\\inc\\stm32f4xx_hash.h"



 



  

 



  
typedef struct
{
  uint32_t HASH_AlgoSelection; 
 
  uint32_t HASH_AlgoMode;      
 
  uint32_t HASH_DataType;      

 
  uint32_t HASH_HMACKeyType;   
 
}HASH_InitTypeDef;



  
typedef struct
{
  uint32_t Data[5];      
 
} HASH_MsgDigest; 



  
typedef struct
{
  uint32_t HASH_IMR; 
  uint32_t HASH_STR;      
  uint32_t HASH_CR;     
  uint32_t HASH_CSR[51];       
}HASH_Context;

 



  



  







 



  







 



   











 



  







 



   




 



   





				   


 



   

















  



  

 
  
  
 
void HASH_DeInit(void);

 
void HASH_Init(HASH_InitTypeDef* HASH_InitStruct);
void HASH_StructInit(HASH_InitTypeDef* HASH_InitStruct);
void HASH_Reset(void);

 
void HASH_DataIn(uint32_t Data);
uint8_t HASH_GetInFIFOWordsNbr(void);
void HASH_SetLastWordValidBitsNbr(uint16_t ValidNumber);
void HASH_StartDigest(void);
void HASH_GetDigest(HASH_MsgDigest* HASH_MessageDigest);

 
void HASH_SaveContext(HASH_Context* HASH_ContextSave);
void HASH_RestoreContext(HASH_Context* HASH_ContextRestore);

 
void HASH_DMACmd(FunctionalState NewState);

 
void HASH_ITConfig(uint8_t HASH_IT, FunctionalState NewState);
FlagStatus HASH_GetFlagStatus(uint16_t HASH_FLAG);
void HASH_ClearFlag(uint16_t HASH_FLAG);
ITStatus HASH_GetITStatus(uint8_t HASH_IT);
void HASH_ClearITPendingBit(uint8_t HASH_IT);

 
ErrorStatus HASH_SHA1(uint8_t *Input, uint32_t Ilen, uint8_t Output[20]);
ErrorStatus HMAC_SHA1(uint8_t *Key, uint32_t Keylen,
                      uint8_t *Input, uint32_t Ilen,
                      uint8_t Output[20]);

 
ErrorStatus HASH_MD5(uint8_t *Input, uint32_t Ilen, uint8_t Output[16]);
ErrorStatus HMAC_MD5(uint8_t *Key, uint32_t Keylen,
                     uint8_t *Input, uint32_t Ilen,
                     uint8_t Output[16]);









  



  

 
#line 46 "C:\\STM32F4-Discovery_FW_V1.1.0\\Libraries\\STM32F4xx_StdPeriph_Driver\\inc\\stm32f4xx_conf.h"
#line 1 "C:\\STM32F4-Discovery_FW_V1.1.0\\Libraries\\STM32F4xx_StdPeriph_Driver\\inc\\stm32f4xx_gpio.h"




















 

 







 
#line 33 "C:\\STM32F4-Discovery_FW_V1.1.0\\Libraries\\STM32F4xx_StdPeriph_Driver\\inc\\stm32f4xx_gpio.h"



 



  

 

#line 53 "C:\\STM32F4-Discovery_FW_V1.1.0\\Libraries\\STM32F4xx_StdPeriph_Driver\\inc\\stm32f4xx_gpio.h"
                                                                


    
typedef enum
{ 
  GPIO_Mode_IN   = 0x00,  
  GPIO_Mode_OUT  = 0x01,  
  GPIO_Mode_AF   = 0x02,  
  GPIO_Mode_AN   = 0x03   
}GPIOMode_TypeDef;





   
typedef enum
{ 
  GPIO_OType_PP = 0x00,
  GPIO_OType_OD = 0x01
}GPIOOType_TypeDef;





   
typedef enum
{ 
  GPIO_Speed_2MHz   = 0x00,  
  GPIO_Speed_25MHz  = 0x01,  
  GPIO_Speed_50MHz  = 0x02,  
  GPIO_Speed_100MHz = 0x03   
}GPIOSpeed_TypeDef;





  
typedef enum
{ 
  GPIO_PuPd_NOPULL = 0x00,
  GPIO_PuPd_UP     = 0x01,
  GPIO_PuPd_DOWN   = 0x02
}GPIOPuPd_TypeDef;





  
typedef enum
{ 
  Bit_RESET = 0,
  Bit_SET
}BitAction;





  
typedef struct
{
  uint32_t GPIO_Pin;              
 

  GPIOMode_TypeDef GPIO_Mode;     
 

  GPIOSpeed_TypeDef GPIO_Speed;   
 

  GPIOOType_TypeDef GPIO_OType;   
 

  GPIOPuPd_TypeDef GPIO_PuPd;     
 
}GPIO_InitTypeDef;

 



  



  
#line 161 "C:\\STM32F4-Discovery_FW_V1.1.0\\Libraries\\STM32F4xx_StdPeriph_Driver\\inc\\stm32f4xx_gpio.h"

#line 179 "C:\\STM32F4-Discovery_FW_V1.1.0\\Libraries\\STM32F4xx_StdPeriph_Driver\\inc\\stm32f4xx_gpio.h"


  




  
#line 203 "C:\\STM32F4-Discovery_FW_V1.1.0\\Libraries\\STM32F4xx_StdPeriph_Driver\\inc\\stm32f4xx_gpio.h"

#line 220 "C:\\STM32F4-Discovery_FW_V1.1.0\\Libraries\\STM32F4xx_StdPeriph_Driver\\inc\\stm32f4xx_gpio.h"


  



  


  








  





  






  







  






  





  




  







  






  








  





  




  






  




  


#line 345 "C:\\STM32F4-Discovery_FW_V1.1.0\\Libraries\\STM32F4xx_StdPeriph_Driver\\inc\\stm32f4xx_gpio.h"


  



 
    








 



 

 
  

 
void GPIO_DeInit(GPIO_TypeDef* GPIOx);

 
void GPIO_Init(GPIO_TypeDef* GPIOx, GPIO_InitTypeDef* GPIO_InitStruct);
void GPIO_StructInit(GPIO_InitTypeDef* GPIO_InitStruct);
void GPIO_PinLockConfig(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin);

 
uint8_t GPIO_ReadInputDataBit(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin);
uint16_t GPIO_ReadInputData(GPIO_TypeDef* GPIOx);
uint8_t GPIO_ReadOutputDataBit(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin);
uint16_t GPIO_ReadOutputData(GPIO_TypeDef* GPIOx);
void GPIO_SetBits(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin);
void GPIO_ResetBits(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin);
void GPIO_WriteBit(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin, BitAction BitVal);
void GPIO_Write(GPIO_TypeDef* GPIOx, uint16_t PortVal);
void GPIO_ToggleBits(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin);

 
void GPIO_PinAFConfig(GPIO_TypeDef* GPIOx, uint16_t GPIO_PinSource, uint8_t GPIO_AF);









  



  

 
#line 47 "C:\\STM32F4-Discovery_FW_V1.1.0\\Libraries\\STM32F4xx_StdPeriph_Driver\\inc\\stm32f4xx_conf.h"
#line 1 "C:\\STM32F4-Discovery_FW_V1.1.0\\Libraries\\STM32F4xx_StdPeriph_Driver\\inc\\stm32f4xx_i2c.h"




















  

 







 
#line 33 "C:\\STM32F4-Discovery_FW_V1.1.0\\Libraries\\STM32F4xx_StdPeriph_Driver\\inc\\stm32f4xx_i2c.h"



 



 

 



 

typedef struct
{
  uint32_t I2C_ClockSpeed;          
 

  uint16_t I2C_Mode;                
 

  uint16_t I2C_DutyCycle;           
 

  uint16_t I2C_OwnAddress1;         
 

  uint16_t I2C_Ack;                 
 

  uint16_t I2C_AcknowledgedAddress; 
 
}I2C_InitTypeDef;

 




 






 

#line 89 "C:\\STM32F4-Discovery_FW_V1.1.0\\Libraries\\STM32F4xx_StdPeriph_Driver\\inc\\stm32f4xx_i2c.h"


 



 







  



 







 



 







 



 







  



 

#line 163 "C:\\STM32F4-Discovery_FW_V1.1.0\\Libraries\\STM32F4xx_StdPeriph_Driver\\inc\\stm32f4xx_i2c.h"


 



 







  



 







 



 







  



 







  



 

#line 233 "C:\\STM32F4-Discovery_FW_V1.1.0\\Libraries\\STM32F4xx_StdPeriph_Driver\\inc\\stm32f4xx_i2c.h"



#line 243 "C:\\STM32F4-Discovery_FW_V1.1.0\\Libraries\\STM32F4xx_StdPeriph_Driver\\inc\\stm32f4xx_i2c.h"


 



 



 

#line 262 "C:\\STM32F4-Discovery_FW_V1.1.0\\Libraries\\STM32F4xx_StdPeriph_Driver\\inc\\stm32f4xx_i2c.h"



 

#line 281 "C:\\STM32F4-Discovery_FW_V1.1.0\\Libraries\\STM32F4xx_StdPeriph_Driver\\inc\\stm32f4xx_i2c.h"



#line 295 "C:\\STM32F4-Discovery_FW_V1.1.0\\Libraries\\STM32F4xx_StdPeriph_Driver\\inc\\stm32f4xx_i2c.h"


 



 





 








 
 

























 

 


 





























 

  
 


 
 

 







 

























 

    
 



 



 



























 

  
 

 


 
 


 






 

#line 501 "C:\\STM32F4-Discovery_FW_V1.1.0\\Libraries\\STM32F4xx_StdPeriph_Driver\\inc\\stm32f4xx_i2c.h"


 



 




 



 




 



 

 
  

 
void I2C_DeInit(I2C_TypeDef* I2Cx);

 
void I2C_Init(I2C_TypeDef* I2Cx, I2C_InitTypeDef* I2C_InitStruct);
void I2C_StructInit(I2C_InitTypeDef* I2C_InitStruct);
void I2C_Cmd(I2C_TypeDef* I2Cx, FunctionalState NewState);
void I2C_GenerateSTART(I2C_TypeDef* I2Cx, FunctionalState NewState);
void I2C_GenerateSTOP(I2C_TypeDef* I2Cx, FunctionalState NewState);
void I2C_Send7bitAddress(I2C_TypeDef* I2Cx, uint8_t Address, uint8_t I2C_Direction);
void I2C_AcknowledgeConfig(I2C_TypeDef* I2Cx, FunctionalState NewState);
void I2C_OwnAddress2Config(I2C_TypeDef* I2Cx, uint8_t Address);
void I2C_DualAddressCmd(I2C_TypeDef* I2Cx, FunctionalState NewState);
void I2C_GeneralCallCmd(I2C_TypeDef* I2Cx, FunctionalState NewState);
void I2C_SoftwareResetCmd(I2C_TypeDef* I2Cx, FunctionalState NewState);
void I2C_StretchClockCmd(I2C_TypeDef* I2Cx, FunctionalState NewState);
void I2C_FastModeDutyCycleConfig(I2C_TypeDef* I2Cx, uint16_t I2C_DutyCycle);
void I2C_NACKPositionConfig(I2C_TypeDef* I2Cx, uint16_t I2C_NACKPosition);
void I2C_SMBusAlertConfig(I2C_TypeDef* I2Cx, uint16_t I2C_SMBusAlert);
void I2C_ARPCmd(I2C_TypeDef* I2Cx, FunctionalState NewState);

  
void I2C_SendData(I2C_TypeDef* I2Cx, uint8_t Data);
uint8_t I2C_ReceiveData(I2C_TypeDef* I2Cx);

  
void I2C_TransmitPEC(I2C_TypeDef* I2Cx, FunctionalState NewState);
void I2C_PECPositionConfig(I2C_TypeDef* I2Cx, uint16_t I2C_PECPosition);
void I2C_CalculatePEC(I2C_TypeDef* I2Cx, FunctionalState NewState);
uint8_t I2C_GetPEC(I2C_TypeDef* I2Cx);

 
void I2C_DMACmd(I2C_TypeDef* I2Cx, FunctionalState NewState);
void I2C_DMALastTransferCmd(I2C_TypeDef* I2Cx, FunctionalState NewState);

 
uint16_t I2C_ReadRegister(I2C_TypeDef* I2Cx, uint8_t I2C_Register);
void I2C_ITConfig(I2C_TypeDef* I2Cx, uint16_t I2C_IT, FunctionalState NewState);




















































































 





 
ErrorStatus I2C_CheckEvent(I2C_TypeDef* I2Cx, uint32_t I2C_EVENT);




 
uint32_t I2C_GetLastEvent(I2C_TypeDef* I2Cx);




 
FlagStatus I2C_GetFlagStatus(I2C_TypeDef* I2Cx, uint32_t I2C_FLAG);


void I2C_ClearFlag(I2C_TypeDef* I2Cx, uint32_t I2C_FLAG);
ITStatus I2C_GetITStatus(I2C_TypeDef* I2Cx, uint32_t I2C_IT);
void I2C_ClearITPendingBit(I2C_TypeDef* I2Cx, uint32_t I2C_IT);









  



  

 
#line 48 "C:\\STM32F4-Discovery_FW_V1.1.0\\Libraries\\STM32F4xx_StdPeriph_Driver\\inc\\stm32f4xx_conf.h"
#line 1 "C:\\STM32F4-Discovery_FW_V1.1.0\\Libraries\\STM32F4xx_StdPeriph_Driver\\inc\\stm32f4xx_iwdg.h"




















 

 







 
#line 33 "C:\\STM32F4-Discovery_FW_V1.1.0\\Libraries\\STM32F4xx_StdPeriph_Driver\\inc\\stm32f4xx_iwdg.h"



 



 

 
 



 
  


 






 



 
#line 77 "C:\\STM32F4-Discovery_FW_V1.1.0\\Libraries\\STM32F4xx_StdPeriph_Driver\\inc\\stm32f4xx_iwdg.h"


 



 






 



 

 
 

 
void IWDG_WriteAccessCmd(uint16_t IWDG_WriteAccess);
void IWDG_SetPrescaler(uint8_t IWDG_Prescaler);
void IWDG_SetReload(uint16_t Reload);
void IWDG_ReloadCounter(void);

 
void IWDG_Enable(void);

 
FlagStatus IWDG_GetFlagStatus(uint16_t IWDG_FLAG);









 



 

 
#line 49 "C:\\STM32F4-Discovery_FW_V1.1.0\\Libraries\\STM32F4xx_StdPeriph_Driver\\inc\\stm32f4xx_conf.h"
#line 1 "C:\\STM32F4-Discovery_FW_V1.1.0\\Libraries\\STM32F4xx_StdPeriph_Driver\\inc\\stm32f4xx_pwr.h"




















  

 







 
#line 33 "C:\\STM32F4-Discovery_FW_V1.1.0\\Libraries\\STM32F4xx_StdPeriph_Driver\\inc\\stm32f4xx_pwr.h"



 



  

 
 



  



  

#line 61 "C:\\STM32F4-Discovery_FW_V1.1.0\\Libraries\\STM32F4xx_StdPeriph_Driver\\inc\\stm32f4xx_pwr.h"







 

  


 







 



 







 




 


 



 









 



 








 



 

 
  

  
void PWR_DeInit(void);

  
void PWR_BackupAccessCmd(FunctionalState NewState);

  
void PWR_PVDLevelConfig(uint32_t PWR_PVDLevel);
void PWR_PVDCmd(FunctionalState NewState);

  
void PWR_WakeUpPinCmd(FunctionalState NewState);

  
void PWR_BackupRegulatorCmd(FunctionalState NewState);
void PWR_MainRegulatorModeConfig(uint32_t PWR_Regulator_Voltage);

  
void PWR_FlashPowerDownCmd(FunctionalState NewState);

  
void PWR_EnterSTOPMode(uint32_t PWR_Regulator, uint8_t PWR_STOPEntry);
void PWR_EnterSTANDBYMode(void);

  
FlagStatus PWR_GetFlagStatus(uint32_t PWR_FLAG);
void PWR_ClearFlag(uint32_t PWR_FLAG);









 



 

 
#line 50 "C:\\STM32F4-Discovery_FW_V1.1.0\\Libraries\\STM32F4xx_StdPeriph_Driver\\inc\\stm32f4xx_conf.h"
#line 1 "C:\\STM32F4-Discovery_FW_V1.1.0\\Libraries\\STM32F4xx_StdPeriph_Driver\\inc\\stm32f4xx_rcc.h"



















 

 







 
#line 32 "C:\\STM32F4-Discovery_FW_V1.1.0\\Libraries\\STM32F4xx_StdPeriph_Driver\\inc\\stm32f4xx_rcc.h"



 



  

 
typedef struct
{
  uint32_t SYSCLK_Frequency;  
  uint32_t HCLK_Frequency;    
  uint32_t PCLK1_Frequency;   
  uint32_t PCLK2_Frequency;   
}RCC_ClocksTypeDef;

 



 
  


 







  
  


 
#line 79 "C:\\STM32F4-Discovery_FW_V1.1.0\\Libraries\\STM32F4xx_StdPeriph_Driver\\inc\\stm32f4xx_rcc.h"
 




  
  


 
#line 95 "C:\\STM32F4-Discovery_FW_V1.1.0\\Libraries\\STM32F4xx_StdPeriph_Driver\\inc\\stm32f4xx_rcc.h"


  
  


 
#line 116 "C:\\STM32F4-Discovery_FW_V1.1.0\\Libraries\\STM32F4xx_StdPeriph_Driver\\inc\\stm32f4xx_rcc.h"


  
  


 
#line 131 "C:\\STM32F4-Discovery_FW_V1.1.0\\Libraries\\STM32F4xx_StdPeriph_Driver\\inc\\stm32f4xx_rcc.h"


  
  


 
#line 151 "C:\\STM32F4-Discovery_FW_V1.1.0\\Libraries\\STM32F4xx_StdPeriph_Driver\\inc\\stm32f4xx_rcc.h"


  
  


 







  
  


 
#line 234 "C:\\STM32F4-Discovery_FW_V1.1.0\\Libraries\\STM32F4xx_StdPeriph_Driver\\inc\\stm32f4xx_rcc.h"


  
  


 






  
  


  
#line 278 "C:\\STM32F4-Discovery_FW_V1.1.0\\Libraries\\STM32F4xx_StdPeriph_Driver\\inc\\stm32f4xx_rcc.h"


  
  


   
#line 291 "C:\\STM32F4-Discovery_FW_V1.1.0\\Libraries\\STM32F4xx_StdPeriph_Driver\\inc\\stm32f4xx_rcc.h"


  
  


  




  
  


  
#line 331 "C:\\STM32F4-Discovery_FW_V1.1.0\\Libraries\\STM32F4xx_StdPeriph_Driver\\inc\\stm32f4xx_rcc.h"


  
  


  
#line 354 "C:\\STM32F4-Discovery_FW_V1.1.0\\Libraries\\STM32F4xx_StdPeriph_Driver\\inc\\stm32f4xx_rcc.h"


  
  


 
#line 372 "C:\\STM32F4-Discovery_FW_V1.1.0\\Libraries\\STM32F4xx_StdPeriph_Driver\\inc\\stm32f4xx_rcc.h"
                                   





  
  


 
#line 394 "C:\\STM32F4-Discovery_FW_V1.1.0\\Libraries\\STM32F4xx_StdPeriph_Driver\\inc\\stm32f4xx_rcc.h"
                                   





  
  


 
#line 426 "C:\\STM32F4-Discovery_FW_V1.1.0\\Libraries\\STM32F4xx_StdPeriph_Driver\\inc\\stm32f4xx_rcc.h"


  



  

 
  

 
void RCC_DeInit(void);

 
void RCC_HSEConfig(uint8_t RCC_HSE);
ErrorStatus RCC_WaitForHSEStartUp(void);
void RCC_AdjustHSICalibrationValue(uint8_t HSICalibrationValue);
void RCC_HSICmd(FunctionalState NewState);
void RCC_LSEConfig(uint8_t RCC_LSE);
void RCC_LSICmd(FunctionalState NewState);

void RCC_PLLConfig(uint32_t RCC_PLLSource, uint32_t PLLM, uint32_t PLLN, uint32_t PLLP, uint32_t PLLQ);
void RCC_PLLCmd(FunctionalState NewState);
void RCC_PLLI2SConfig(uint32_t PLLI2SN, uint32_t PLLI2SR);
void RCC_PLLI2SCmd(FunctionalState NewState);

void RCC_ClockSecuritySystemCmd(FunctionalState NewState);
void RCC_MCO1Config(uint32_t RCC_MCO1Source, uint32_t RCC_MCO1Div);
void RCC_MCO2Config(uint32_t RCC_MCO2Source, uint32_t RCC_MCO2Div);

 
void RCC_SYSCLKConfig(uint32_t RCC_SYSCLKSource);
uint8_t RCC_GetSYSCLKSource(void);
void RCC_HCLKConfig(uint32_t RCC_SYSCLK);
void RCC_PCLK1Config(uint32_t RCC_HCLK);
void RCC_PCLK2Config(uint32_t RCC_HCLK);
void RCC_GetClocksFreq(RCC_ClocksTypeDef* RCC_Clocks);

 
void RCC_RTCCLKConfig(uint32_t RCC_RTCCLKSource);
void RCC_RTCCLKCmd(FunctionalState NewState);
void RCC_BackupResetCmd(FunctionalState NewState);
void RCC_I2SCLKConfig(uint32_t RCC_I2SCLKSource); 

void RCC_AHB1PeriphClockCmd(uint32_t RCC_AHB1Periph, FunctionalState NewState);
void RCC_AHB2PeriphClockCmd(uint32_t RCC_AHB2Periph, FunctionalState NewState);
void RCC_AHB3PeriphClockCmd(uint32_t RCC_AHB3Periph, FunctionalState NewState);
void RCC_APB1PeriphClockCmd(uint32_t RCC_APB1Periph, FunctionalState NewState);
void RCC_APB2PeriphClockCmd(uint32_t RCC_APB2Periph, FunctionalState NewState);

void RCC_AHB1PeriphResetCmd(uint32_t RCC_AHB1Periph, FunctionalState NewState);
void RCC_AHB2PeriphResetCmd(uint32_t RCC_AHB2Periph, FunctionalState NewState);
void RCC_AHB3PeriphResetCmd(uint32_t RCC_AHB3Periph, FunctionalState NewState);
void RCC_APB1PeriphResetCmd(uint32_t RCC_APB1Periph, FunctionalState NewState);
void RCC_APB2PeriphResetCmd(uint32_t RCC_APB2Periph, FunctionalState NewState);

void RCC_AHB1PeriphClockLPModeCmd(uint32_t RCC_AHB1Periph, FunctionalState NewState);
void RCC_AHB2PeriphClockLPModeCmd(uint32_t RCC_AHB2Periph, FunctionalState NewState);
void RCC_AHB3PeriphClockLPModeCmd(uint32_t RCC_AHB3Periph, FunctionalState NewState);
void RCC_APB1PeriphClockLPModeCmd(uint32_t RCC_APB1Periph, FunctionalState NewState);
void RCC_APB2PeriphClockLPModeCmd(uint32_t RCC_APB2Periph, FunctionalState NewState);

 
void RCC_ITConfig(uint8_t RCC_IT, FunctionalState NewState);
FlagStatus RCC_GetFlagStatus(uint8_t RCC_FLAG);
void RCC_ClearFlag(void);
ITStatus RCC_GetITStatus(uint8_t RCC_IT);
void RCC_ClearITPendingBit(uint8_t RCC_IT);









  



  

 
#line 51 "C:\\STM32F4-Discovery_FW_V1.1.0\\Libraries\\STM32F4xx_StdPeriph_Driver\\inc\\stm32f4xx_conf.h"
#line 1 "C:\\STM32F4-Discovery_FW_V1.1.0\\Libraries\\STM32F4xx_StdPeriph_Driver\\inc\\stm32f4xx_rng.h"




















 

 







 
#line 33 "C:\\STM32F4-Discovery_FW_V1.1.0\\Libraries\\STM32F4xx_StdPeriph_Driver\\inc\\stm32f4xx_rng.h"



 



  

 
  



 
  


  











  



   







  



  

 
  

  
void RNG_DeInit(void);

 
void RNG_Cmd(FunctionalState NewState);

 
uint32_t RNG_GetRandomNumber(void);

 
void RNG_ITConfig(FunctionalState NewState);
FlagStatus RNG_GetFlagStatus(uint8_t RNG_FLAG);
void RNG_ClearFlag(uint8_t RNG_FLAG);
ITStatus RNG_GetITStatus(uint8_t RNG_IT);
void RNG_ClearITPendingBit(uint8_t RNG_IT);









  



  

 
#line 52 "C:\\STM32F4-Discovery_FW_V1.1.0\\Libraries\\STM32F4xx_StdPeriph_Driver\\inc\\stm32f4xx_conf.h"
#line 1 "C:\\STM32F4-Discovery_FW_V1.1.0\\Libraries\\STM32F4xx_StdPeriph_Driver\\inc\\stm32f4xx_rtc.h"




















 

 







 
#line 33 "C:\\STM32F4-Discovery_FW_V1.1.0\\Libraries\\STM32F4xx_StdPeriph_Driver\\inc\\stm32f4xx_rtc.h"



 



  

 



  
typedef struct
{
  uint32_t RTC_HourFormat;   
 
  
  uint32_t RTC_AsynchPrediv; 
 
  
  uint32_t RTC_SynchPrediv;  
 
}RTC_InitTypeDef;



 
typedef struct
{
  uint8_t RTC_Hours;    


 

  uint8_t RTC_Minutes;  
 
  
  uint8_t RTC_Seconds;  
 

  uint8_t RTC_H12;      
 
}RTC_TimeTypeDef; 



 
typedef struct
{
  uint8_t RTC_WeekDay; 
 
  
  uint8_t RTC_Month;   
 

  uint8_t RTC_Date;     
 
  
  uint8_t RTC_Year;     
 
}RTC_DateTypeDef;



 
typedef struct
{
  RTC_TimeTypeDef RTC_AlarmTime;      

  uint32_t RTC_AlarmMask;            
 

  uint32_t RTC_AlarmDateWeekDaySel;  
 
  
  uint8_t RTC_AlarmDateWeekDay;      



 
}RTC_AlarmTypeDef;

 



  




  






  



  

 


  




  




  



  







  



  






  



  




  



  

 
#line 205 "C:\\STM32F4-Discovery_FW_V1.1.0\\Libraries\\STM32F4xx_StdPeriph_Driver\\inc\\stm32f4xx_rtc.h"



  



  
  
#line 228 "C:\\STM32F4-Discovery_FW_V1.1.0\\Libraries\\STM32F4xx_StdPeriph_Driver\\inc\\stm32f4xx_rtc.h"


  




  
#line 244 "C:\\STM32F4-Discovery_FW_V1.1.0\\Libraries\\STM32F4xx_StdPeriph_Driver\\inc\\stm32f4xx_rtc.h"



  




  








  




  
#line 274 "C:\\STM32F4-Discovery_FW_V1.1.0\\Libraries\\STM32F4xx_StdPeriph_Driver\\inc\\stm32f4xx_rtc.h"



  



  







  

  

  
#line 343 "C:\\STM32F4-Discovery_FW_V1.1.0\\Libraries\\STM32F4xx_StdPeriph_Driver\\inc\\stm32f4xx_rtc.h"


  



  





  



  
#line 373 "C:\\STM32F4-Discovery_FW_V1.1.0\\Libraries\\STM32F4xx_StdPeriph_Driver\\inc\\stm32f4xx_rtc.h"


  



  






  



  




 







  



  






  




  








  

 

  






  



  
#line 453 "C:\\STM32F4-Discovery_FW_V1.1.0\\Libraries\\STM32F4xx_StdPeriph_Driver\\inc\\stm32f4xx_rtc.h"
                                          


  



  
#line 468 "C:\\STM32F4-Discovery_FW_V1.1.0\\Libraries\\STM32F4xx_StdPeriph_Driver\\inc\\stm32f4xx_rtc.h"



  



  




 



  











  



  
#line 509 "C:\\STM32F4-Discovery_FW_V1.1.0\\Libraries\\STM32F4xx_StdPeriph_Driver\\inc\\stm32f4xx_rtc.h"



  



  


#line 529 "C:\\STM32F4-Discovery_FW_V1.1.0\\Libraries\\STM32F4xx_StdPeriph_Driver\\inc\\stm32f4xx_rtc.h"


  



  
#line 560 "C:\\STM32F4-Discovery_FW_V1.1.0\\Libraries\\STM32F4xx_StdPeriph_Driver\\inc\\stm32f4xx_rtc.h"



 

  

  
#line 576 "C:\\STM32F4-Discovery_FW_V1.1.0\\Libraries\\STM32F4xx_StdPeriph_Driver\\inc\\stm32f4xx_rtc.h"







 



  





 



  






  



  






  



  







  



  






  



  




 



 

#line 693 "C:\\STM32F4-Discovery_FW_V1.1.0\\Libraries\\STM32F4xx_StdPeriph_Driver\\inc\\stm32f4xx_rtc.h"


  



  






  



  
#line 733 "C:\\STM32F4-Discovery_FW_V1.1.0\\Libraries\\STM32F4xx_StdPeriph_Driver\\inc\\stm32f4xx_rtc.h"


  



  
#line 746 "C:\\STM32F4-Discovery_FW_V1.1.0\\Libraries\\STM32F4xx_StdPeriph_Driver\\inc\\stm32f4xx_rtc.h"









  



  





  



  

 
  

 
ErrorStatus RTC_DeInit(void);

 
ErrorStatus RTC_Init(RTC_InitTypeDef* RTC_InitStruct);
void RTC_StructInit(RTC_InitTypeDef* RTC_InitStruct);
void RTC_WriteProtectionCmd(FunctionalState NewState);
ErrorStatus RTC_EnterInitMode(void);
void RTC_ExitInitMode(void);
ErrorStatus RTC_WaitForSynchro(void);
ErrorStatus RTC_RefClockCmd(FunctionalState NewState);
void RTC_BypassShadowCmd(FunctionalState NewState);

 
ErrorStatus RTC_SetTime(uint32_t RTC_Format, RTC_TimeTypeDef* RTC_TimeStruct);
void RTC_TimeStructInit(RTC_TimeTypeDef* RTC_TimeStruct);
void RTC_GetTime(uint32_t RTC_Format, RTC_TimeTypeDef* RTC_TimeStruct);
uint32_t RTC_GetSubSecond(void);
ErrorStatus RTC_SetDate(uint32_t RTC_Format, RTC_DateTypeDef* RTC_DateStruct);
void RTC_DateStructInit(RTC_DateTypeDef* RTC_DateStruct);
void RTC_GetDate(uint32_t RTC_Format, RTC_DateTypeDef* RTC_DateStruct);

 
void RTC_SetAlarm(uint32_t RTC_Format, uint32_t RTC_Alarm, RTC_AlarmTypeDef* RTC_AlarmStruct);
void RTC_AlarmStructInit(RTC_AlarmTypeDef* RTC_AlarmStruct);
void RTC_GetAlarm(uint32_t RTC_Format, uint32_t RTC_Alarm, RTC_AlarmTypeDef* RTC_AlarmStruct);
ErrorStatus RTC_AlarmCmd(uint32_t RTC_Alarm, FunctionalState NewState);
void RTC_AlarmSubSecondConfig(uint32_t RTC_Alarm, uint32_t RTC_AlarmSubSecondValue, uint32_t RTC_AlarmSubSecondMask);
uint32_t RTC_GetAlarmSubSecond(uint32_t RTC_Alarm);

 
void RTC_WakeUpClockConfig(uint32_t RTC_WakeUpClock);
void RTC_SetWakeUpCounter(uint32_t RTC_WakeUpCounter);
uint32_t RTC_GetWakeUpCounter(void);
ErrorStatus RTC_WakeUpCmd(FunctionalState NewState);

 
void RTC_DayLightSavingConfig(uint32_t RTC_DayLightSaving, uint32_t RTC_StoreOperation);
uint32_t RTC_GetStoreOperation(void);

 
void RTC_OutputConfig(uint32_t RTC_Output, uint32_t RTC_OutputPolarity);

 
ErrorStatus RTC_CoarseCalibConfig(uint32_t RTC_CalibSign, uint32_t Value);
ErrorStatus RTC_CoarseCalibCmd(FunctionalState NewState);
void RTC_CalibOutputCmd(FunctionalState NewState);
void RTC_CalibOutputConfig(uint32_t RTC_CalibOutput);
ErrorStatus RTC_SmoothCalibConfig(uint32_t RTC_SmoothCalibPeriod, 
                                  uint32_t RTC_SmoothCalibPlusPulses,
                                  uint32_t RTC_SmouthCalibMinusPulsesValue);

 
void RTC_TimeStampCmd(uint32_t RTC_TimeStampEdge, FunctionalState NewState);
void RTC_GetTimeStamp(uint32_t RTC_Format, RTC_TimeTypeDef* RTC_StampTimeStruct,
                                      RTC_DateTypeDef* RTC_StampDateStruct);
uint32_t RTC_GetTimeStampSubSecond(void);

 
void RTC_TamperTriggerConfig(uint32_t RTC_Tamper, uint32_t RTC_TamperTrigger);
void RTC_TamperCmd(uint32_t RTC_Tamper, FunctionalState NewState);
void RTC_TamperFilterConfig(uint32_t RTC_TamperFilter);
void RTC_TamperSamplingFreqConfig(uint32_t RTC_TamperSamplingFreq);
void RTC_TamperPinsPrechargeDuration(uint32_t RTC_TamperPrechargeDuration);
void RTC_TimeStampOnTamperDetectionCmd(FunctionalState NewState);
void RTC_TamperPullUpCmd(FunctionalState NewState);

 
void RTC_WriteBackupRegister(uint32_t RTC_BKP_DR, uint32_t Data);
uint32_t RTC_ReadBackupRegister(uint32_t RTC_BKP_DR);


 
void RTC_TamperPinSelection(uint32_t RTC_TamperPin);
void RTC_TimeStampPinSelection(uint32_t RTC_TimeStampPin);
void RTC_OutputTypeConfig(uint32_t RTC_OutputType);

 
ErrorStatus RTC_SynchroShiftConfig(uint32_t RTC_ShiftAdd1S, uint32_t RTC_ShiftSubFS);

 
void RTC_ITConfig(uint32_t RTC_IT, FunctionalState NewState);
FlagStatus RTC_GetFlagStatus(uint32_t RTC_FLAG);
void RTC_ClearFlag(uint32_t RTC_FLAG);
ITStatus RTC_GetITStatus(uint32_t RTC_IT);
void RTC_ClearITPendingBit(uint32_t RTC_IT);









  



  

 
#line 53 "C:\\STM32F4-Discovery_FW_V1.1.0\\Libraries\\STM32F4xx_StdPeriph_Driver\\inc\\stm32f4xx_conf.h"
#line 1 "C:\\STM32F4-Discovery_FW_V1.1.0\\Libraries\\STM32F4xx_StdPeriph_Driver\\inc\\stm32f4xx_sdio.h"




















 

 







 
#line 33 "C:\\STM32F4-Discovery_FW_V1.1.0\\Libraries\\STM32F4xx_StdPeriph_Driver\\inc\\stm32f4xx_sdio.h"



 



 

 

typedef struct
{
  uint32_t SDIO_ClockEdge;            
 

  uint32_t SDIO_ClockBypass;          

 

  uint32_t SDIO_ClockPowerSave;       

 

  uint32_t SDIO_BusWide;              
 

  uint32_t SDIO_HardwareFlowControl;  
 

  uint8_t SDIO_ClockDiv;              
 
                                           
} SDIO_InitTypeDef;

typedef struct
{
  uint32_t SDIO_Argument;  


 

  uint32_t SDIO_CmdIndex;   

  uint32_t SDIO_Response;  
 

  uint32_t SDIO_Wait;      
 

  uint32_t SDIO_CPSM;      

 
} SDIO_CmdInitTypeDef;

typedef struct
{
  uint32_t SDIO_DataTimeOut;     

  uint32_t SDIO_DataLength;      
 
  uint32_t SDIO_DataBlockSize;  
 
 
  uint32_t SDIO_TransferDir;    

 
 
  uint32_t SDIO_TransferMode;   
 
 
  uint32_t SDIO_DPSM;           

 
} SDIO_DataInitTypeDef;


 



 



 







 



 







  



 







 



 









 



 







 



 






  




 

#line 219 "C:\\STM32F4-Discovery_FW_V1.1.0\\Libraries\\STM32F4xx_StdPeriph_Driver\\inc\\stm32f4xx_sdio.h"


  



 




 



 

#line 242 "C:\\STM32F4-Discovery_FW_V1.1.0\\Libraries\\STM32F4xx_StdPeriph_Driver\\inc\\stm32f4xx_sdio.h"


 



 








 



 






  



 

#line 280 "C:\\STM32F4-Discovery_FW_V1.1.0\\Libraries\\STM32F4xx_StdPeriph_Driver\\inc\\stm32f4xx_sdio.h"


 



 




 



 

#line 327 "C:\\STM32F4-Discovery_FW_V1.1.0\\Libraries\\STM32F4xx_StdPeriph_Driver\\inc\\stm32f4xx_sdio.h"


 



 







 



 







 



 






 



 

#line 418 "C:\\STM32F4-Discovery_FW_V1.1.0\\Libraries\\STM32F4xx_StdPeriph_Driver\\inc\\stm32f4xx_sdio.h"



#line 445 "C:\\STM32F4-Discovery_FW_V1.1.0\\Libraries\\STM32F4xx_StdPeriph_Driver\\inc\\stm32f4xx_sdio.h"





 



 







 



 

 
 
 
void SDIO_DeInit(void);

 
void SDIO_Init(SDIO_InitTypeDef* SDIO_InitStruct);
void SDIO_StructInit(SDIO_InitTypeDef* SDIO_InitStruct);
void SDIO_ClockCmd(FunctionalState NewState);
void SDIO_SetPowerState(uint32_t SDIO_PowerState);
uint32_t SDIO_GetPowerState(void);

 
void SDIO_SendCommand(SDIO_CmdInitTypeDef *SDIO_CmdInitStruct);
void SDIO_CmdStructInit(SDIO_CmdInitTypeDef* SDIO_CmdInitStruct);
uint8_t SDIO_GetCommandResponse(void);
uint32_t SDIO_GetResponse(uint32_t SDIO_RESP);

 
void SDIO_DataConfig(SDIO_DataInitTypeDef* SDIO_DataInitStruct);
void SDIO_DataStructInit(SDIO_DataInitTypeDef* SDIO_DataInitStruct);
uint32_t SDIO_GetDataCounter(void);
uint32_t SDIO_ReadData(void);
void SDIO_WriteData(uint32_t Data);
uint32_t SDIO_GetFIFOCount(void);

 
void SDIO_StartSDIOReadWait(FunctionalState NewState);
void SDIO_StopSDIOReadWait(FunctionalState NewState);
void SDIO_SetSDIOReadWaitMode(uint32_t SDIO_ReadWaitMode);
void SDIO_SetSDIOOperation(FunctionalState NewState);
void SDIO_SendSDIOSuspendCmd(FunctionalState NewState);

 
void SDIO_CommandCompletionCmd(FunctionalState NewState);
void SDIO_CEATAITCmd(FunctionalState NewState);
void SDIO_SendCEATACmd(FunctionalState NewState);

 
void SDIO_DMACmd(FunctionalState NewState);

 
void SDIO_ITConfig(uint32_t SDIO_IT, FunctionalState NewState);
FlagStatus SDIO_GetFlagStatus(uint32_t SDIO_FLAG);
void SDIO_ClearFlag(uint32_t SDIO_FLAG);
ITStatus SDIO_GetITStatus(uint32_t SDIO_IT);
void SDIO_ClearITPendingBit(uint32_t SDIO_IT);









 



 

 
#line 54 "C:\\STM32F4-Discovery_FW_V1.1.0\\Libraries\\STM32F4xx_StdPeriph_Driver\\inc\\stm32f4xx_conf.h"
#line 1 "C:\\STM32F4-Discovery_FW_V1.1.0\\Libraries\\STM32F4xx_StdPeriph_Driver\\inc\\stm32f4xx_spi.h"




















  

 







 
#line 33 "C:\\STM32F4-Discovery_FW_V1.1.0\\Libraries\\STM32F4xx_StdPeriph_Driver\\inc\\stm32f4xx_spi.h"



 



  

 



 

typedef struct
{
  uint16_t SPI_Direction;           
 

  uint16_t SPI_Mode;                
 

  uint16_t SPI_DataSize;            
 

  uint16_t SPI_CPOL;                
 

  uint16_t SPI_CPHA;                
 

  uint16_t SPI_NSS;                 

 
 
  uint16_t SPI_BaudRatePrescaler;   



 

  uint16_t SPI_FirstBit;            
 

  uint16_t SPI_CRCPolynomial;        
}SPI_InitTypeDef;



 

typedef struct
{

  uint16_t I2S_Mode;         
 

  uint16_t I2S_Standard;     
 

  uint16_t I2S_DataFormat;   
 

  uint16_t I2S_MCLKOutput;   
 

  uint32_t I2S_AudioFreq;    
 

  uint16_t I2S_CPOL;         
 
}I2S_InitTypeDef;

 



 

























 
  
#line 147 "C:\\STM32F4-Discovery_FW_V1.1.0\\Libraries\\STM32F4xx_StdPeriph_Driver\\inc\\stm32f4xx_spi.h"


 



 







 



 







  



 







 



 







 



 







  



 

#line 231 "C:\\STM32F4-Discovery_FW_V1.1.0\\Libraries\\STM32F4xx_StdPeriph_Driver\\inc\\stm32f4xx_spi.h"


  



 







 



 

#line 259 "C:\\STM32F4-Discovery_FW_V1.1.0\\Libraries\\STM32F4xx_StdPeriph_Driver\\inc\\stm32f4xx_spi.h"


 
  



 

#line 278 "C:\\STM32F4-Discovery_FW_V1.1.0\\Libraries\\STM32F4xx_StdPeriph_Driver\\inc\\stm32f4xx_spi.h"


 
  


 

#line 294 "C:\\STM32F4-Discovery_FW_V1.1.0\\Libraries\\STM32F4xx_StdPeriph_Driver\\inc\\stm32f4xx_spi.h"


 



 







 



 

#line 324 "C:\\STM32F4-Discovery_FW_V1.1.0\\Libraries\\STM32F4xx_StdPeriph_Driver\\inc\\stm32f4xx_spi.h"






 
            


 







 



 






 



 







 



 






 



 







 



 























 



 

#line 431 "C:\\STM32F4-Discovery_FW_V1.1.0\\Libraries\\STM32F4xx_StdPeriph_Driver\\inc\\stm32f4xx_spi.h"

#line 438 "C:\\STM32F4-Discovery_FW_V1.1.0\\Libraries\\STM32F4xx_StdPeriph_Driver\\inc\\stm32f4xx_spi.h"


 



 




 



 

#line 474 "C:\\STM32F4-Discovery_FW_V1.1.0\\Libraries\\STM32F4xx_StdPeriph_Driver\\inc\\stm32f4xx_spi.h"


 
  


 

 
  

  
void SPI_I2S_DeInit(SPI_TypeDef* SPIx);

 
void SPI_Init(SPI_TypeDef* SPIx, SPI_InitTypeDef* SPI_InitStruct);
void I2S_Init(SPI_TypeDef* SPIx, I2S_InitTypeDef* I2S_InitStruct);
void SPI_StructInit(SPI_InitTypeDef* SPI_InitStruct);
void I2S_StructInit(I2S_InitTypeDef* I2S_InitStruct);
void SPI_Cmd(SPI_TypeDef* SPIx, FunctionalState NewState);
void I2S_Cmd(SPI_TypeDef* SPIx, FunctionalState NewState);
void SPI_DataSizeConfig(SPI_TypeDef* SPIx, uint16_t SPI_DataSize);
void SPI_BiDirectionalLineConfig(SPI_TypeDef* SPIx, uint16_t SPI_Direction);
void SPI_NSSInternalSoftwareConfig(SPI_TypeDef* SPIx, uint16_t SPI_NSSInternalSoft);
void SPI_SSOutputCmd(SPI_TypeDef* SPIx, FunctionalState NewState);
void SPI_TIModeCmd(SPI_TypeDef* SPIx, FunctionalState NewState);

void I2S_FullDuplexConfig(SPI_TypeDef* I2Sxext, I2S_InitTypeDef* I2S_InitStruct);

  
void SPI_I2S_SendData(SPI_TypeDef* SPIx, uint16_t Data);
uint16_t SPI_I2S_ReceiveData(SPI_TypeDef* SPIx);

 
void SPI_CalculateCRC(SPI_TypeDef* SPIx, FunctionalState NewState);
void SPI_TransmitCRC(SPI_TypeDef* SPIx);
uint16_t SPI_GetCRC(SPI_TypeDef* SPIx, uint8_t SPI_CRC);
uint16_t SPI_GetCRCPolynomial(SPI_TypeDef* SPIx);

 
void SPI_I2S_DMACmd(SPI_TypeDef* SPIx, uint16_t SPI_I2S_DMAReq, FunctionalState NewState);

 
void SPI_I2S_ITConfig(SPI_TypeDef* SPIx, uint8_t SPI_I2S_IT, FunctionalState NewState);
FlagStatus SPI_I2S_GetFlagStatus(SPI_TypeDef* SPIx, uint16_t SPI_I2S_FLAG);
void SPI_I2S_ClearFlag(SPI_TypeDef* SPIx, uint16_t SPI_I2S_FLAG);
ITStatus SPI_I2S_GetITStatus(SPI_TypeDef* SPIx, uint8_t SPI_I2S_IT);
void SPI_I2S_ClearITPendingBit(SPI_TypeDef* SPIx, uint8_t SPI_I2S_IT);









 



 

 
#line 55 "C:\\STM32F4-Discovery_FW_V1.1.0\\Libraries\\STM32F4xx_StdPeriph_Driver\\inc\\stm32f4xx_conf.h"
#line 1 "C:\\STM32F4-Discovery_FW_V1.1.0\\Libraries\\STM32F4xx_StdPeriph_Driver\\inc\\stm32f4xx_syscfg.h"




















 

 







 
#line 33 "C:\\STM32F4-Discovery_FW_V1.1.0\\Libraries\\STM32F4xx_StdPeriph_Driver\\inc\\stm32f4xx_syscfg.h"



 



  

 
 
  


  



  
#line 61 "C:\\STM32F4-Discovery_FW_V1.1.0\\Libraries\\STM32F4xx_StdPeriph_Driver\\inc\\stm32f4xx_syscfg.h"
                                      
#line 71 "C:\\STM32F4-Discovery_FW_V1.1.0\\Libraries\\STM32F4xx_StdPeriph_Driver\\inc\\stm32f4xx_syscfg.h"


  




  
#line 111 "C:\\STM32F4-Discovery_FW_V1.1.0\\Libraries\\STM32F4xx_StdPeriph_Driver\\inc\\stm32f4xx_syscfg.h"


  




  




   






  




  







  



  

 
  
 
void SYSCFG_DeInit(void);
void SYSCFG_MemoryRemapConfig(uint8_t SYSCFG_MemoryRemap);
void SYSCFG_EXTILineConfig(uint8_t EXTI_PortSourceGPIOx, uint8_t EXTI_PinSourcex);
void SYSCFG_ETH_MediaInterfaceConfig(uint32_t SYSCFG_ETH_MediaInterface); 
void SYSCFG_CompensationCellCmd(FunctionalState NewState); 
FlagStatus SYSCFG_GetCompensationCellStatus(void);









  



  

 
#line 56 "C:\\STM32F4-Discovery_FW_V1.1.0\\Libraries\\STM32F4xx_StdPeriph_Driver\\inc\\stm32f4xx_conf.h"
#line 1 "C:\\STM32F4-Discovery_FW_V1.1.0\\Libraries\\STM32F4xx_StdPeriph_Driver\\inc\\stm32f4xx_tim.h"




















 

 







 
#line 33 "C:\\STM32F4-Discovery_FW_V1.1.0\\Libraries\\STM32F4xx_StdPeriph_Driver\\inc\\stm32f4xx_tim.h"



 



  

 




 

typedef struct
{
  uint16_t TIM_Prescaler;         
 

  uint16_t TIM_CounterMode;       
 

  uint32_t TIM_Period;            

  

  uint16_t TIM_ClockDivision;     
 

  uint8_t TIM_RepetitionCounter;  






 
} TIM_TimeBaseInitTypeDef; 



 

typedef struct
{
  uint16_t TIM_OCMode;        
 

  uint16_t TIM_OutputState;   
 

  uint16_t TIM_OutputNState;  

 

  uint32_t TIM_Pulse;         
 

  uint16_t TIM_OCPolarity;    
 

  uint16_t TIM_OCNPolarity;   

 

  uint16_t TIM_OCIdleState;   

 

  uint16_t TIM_OCNIdleState;  

 
} TIM_OCInitTypeDef;



 

typedef struct
{

  uint16_t TIM_Channel;      
 

  uint16_t TIM_ICPolarity;   
 

  uint16_t TIM_ICSelection;  
 

  uint16_t TIM_ICPrescaler;  
 

  uint16_t TIM_ICFilter;     
 
} TIM_ICInitTypeDef;




 

typedef struct
{

  uint16_t TIM_OSSRState;        
 

  uint16_t TIM_OSSIState;        
 

  uint16_t TIM_LOCKLevel;        
  

  uint16_t TIM_DeadTime;         

 

  uint16_t TIM_Break;            
 

  uint16_t TIM_BreakPolarity;    
 

  uint16_t TIM_AutomaticOutput;  
 
} TIM_BDTRInitTypeDef;

 



 

#line 183 "C:\\STM32F4-Discovery_FW_V1.1.0\\Libraries\\STM32F4xx_StdPeriph_Driver\\inc\\stm32f4xx_tim.h"
                                          
#line 196 "C:\\STM32F4-Discovery_FW_V1.1.0\\Libraries\\STM32F4xx_StdPeriph_Driver\\inc\\stm32f4xx_tim.h"
                                     
 
#line 206 "C:\\STM32F4-Discovery_FW_V1.1.0\\Libraries\\STM32F4xx_StdPeriph_Driver\\inc\\stm32f4xx_tim.h"
 
#line 213 "C:\\STM32F4-Discovery_FW_V1.1.0\\Libraries\\STM32F4xx_StdPeriph_Driver\\inc\\stm32f4xx_tim.h"
 


 
#line 225 "C:\\STM32F4-Discovery_FW_V1.1.0\\Libraries\\STM32F4xx_StdPeriph_Driver\\inc\\stm32f4xx_tim.h"
                                






 

#line 254 "C:\\STM32F4-Discovery_FW_V1.1.0\\Libraries\\STM32F4xx_StdPeriph_Driver\\inc\\stm32f4xx_tim.h"


 



 







  



 





                                 




                                 







  



 

#line 303 "C:\\STM32F4-Discovery_FW_V1.1.0\\Libraries\\STM32F4xx_StdPeriph_Driver\\inc\\stm32f4xx_tim.h"


 



 

#line 321 "C:\\STM32F4-Discovery_FW_V1.1.0\\Libraries\\STM32F4xx_StdPeriph_Driver\\inc\\stm32f4xx_tim.h"


  



 







 



 
  






 



 







  



 







  



 







  



 







  



 







  



 







  



 







  



 

#line 445 "C:\\STM32F4-Discovery_FW_V1.1.0\\Libraries\\STM32F4xx_StdPeriph_Driver\\inc\\stm32f4xx_tim.h"


  



 







 



 







  



 







  



 







  



 

#line 507 "C:\\STM32F4-Discovery_FW_V1.1.0\\Libraries\\STM32F4xx_StdPeriph_Driver\\inc\\stm32f4xx_tim.h"


  



 

#line 523 "C:\\STM32F4-Discovery_FW_V1.1.0\\Libraries\\STM32F4xx_StdPeriph_Driver\\inc\\stm32f4xx_tim.h"


  



 

#line 539 "C:\\STM32F4-Discovery_FW_V1.1.0\\Libraries\\STM32F4xx_StdPeriph_Driver\\inc\\stm32f4xx_tim.h"


  



 

#line 556 "C:\\STM32F4-Discovery_FW_V1.1.0\\Libraries\\STM32F4xx_StdPeriph_Driver\\inc\\stm32f4xx_tim.h"

#line 565 "C:\\STM32F4-Discovery_FW_V1.1.0\\Libraries\\STM32F4xx_StdPeriph_Driver\\inc\\stm32f4xx_tim.h"


  



 

#line 613 "C:\\STM32F4-Discovery_FW_V1.1.0\\Libraries\\STM32F4xx_StdPeriph_Driver\\inc\\stm32f4xx_tim.h"


  



 

#line 657 "C:\\STM32F4-Discovery_FW_V1.1.0\\Libraries\\STM32F4xx_StdPeriph_Driver\\inc\\stm32f4xx_tim.h"


  



 

#line 673 "C:\\STM32F4-Discovery_FW_V1.1.0\\Libraries\\STM32F4xx_StdPeriph_Driver\\inc\\stm32f4xx_tim.h"



  



 

#line 690 "C:\\STM32F4-Discovery_FW_V1.1.0\\Libraries\\STM32F4xx_StdPeriph_Driver\\inc\\stm32f4xx_tim.h"


  



 

#line 718 "C:\\STM32F4-Discovery_FW_V1.1.0\\Libraries\\STM32F4xx_StdPeriph_Driver\\inc\\stm32f4xx_tim.h"


  



 







  



  






 



 







  



 







  



 

#line 779 "C:\\STM32F4-Discovery_FW_V1.1.0\\Libraries\\STM32F4xx_StdPeriph_Driver\\inc\\stm32f4xx_tim.h"


  




 

#line 797 "C:\\STM32F4-Discovery_FW_V1.1.0\\Libraries\\STM32F4xx_StdPeriph_Driver\\inc\\stm32f4xx_tim.h"
  


  



 

#line 812 "C:\\STM32F4-Discovery_FW_V1.1.0\\Libraries\\STM32F4xx_StdPeriph_Driver\\inc\\stm32f4xx_tim.h"


  



 







  



 





                                     


  



 







  



 

#line 873 "C:\\STM32F4-Discovery_FW_V1.1.0\\Libraries\\STM32F4xx_StdPeriph_Driver\\inc\\stm32f4xx_tim.h"


  



 

#line 889 "C:\\STM32F4-Discovery_FW_V1.1.0\\Libraries\\STM32F4xx_StdPeriph_Driver\\inc\\stm32f4xx_tim.h"


  



 







  


 














#line 931 "C:\\STM32F4-Discovery_FW_V1.1.0\\Libraries\\STM32F4xx_StdPeriph_Driver\\inc\\stm32f4xx_tim.h"



  


 

#line 963 "C:\\STM32F4-Discovery_FW_V1.1.0\\Libraries\\STM32F4xx_StdPeriph_Driver\\inc\\stm32f4xx_tim.h"



  



 




  



 




  



 

#line 1008 "C:\\STM32F4-Discovery_FW_V1.1.0\\Libraries\\STM32F4xx_StdPeriph_Driver\\inc\\stm32f4xx_tim.h"


 



 

 
  

 
void TIM_DeInit(TIM_TypeDef* TIMx);
void TIM_TimeBaseInit(TIM_TypeDef* TIMx, TIM_TimeBaseInitTypeDef* TIM_TimeBaseInitStruct);
void TIM_TimeBaseStructInit(TIM_TimeBaseInitTypeDef* TIM_TimeBaseInitStruct);
void TIM_PrescalerConfig(TIM_TypeDef* TIMx, uint16_t Prescaler, uint16_t TIM_PSCReloadMode);
void TIM_CounterModeConfig(TIM_TypeDef* TIMx, uint16_t TIM_CounterMode);
void TIM_SetCounter(TIM_TypeDef* TIMx, uint32_t Counter);
void TIM_SetAutoreload(TIM_TypeDef* TIMx, uint32_t Autoreload);
uint32_t TIM_GetCounter(TIM_TypeDef* TIMx);
uint16_t TIM_GetPrescaler(TIM_TypeDef* TIMx);
void TIM_UpdateDisableConfig(TIM_TypeDef* TIMx, FunctionalState NewState);
void TIM_UpdateRequestConfig(TIM_TypeDef* TIMx, uint16_t TIM_UpdateSource);
void TIM_ARRPreloadConfig(TIM_TypeDef* TIMx, FunctionalState NewState);
void TIM_SelectOnePulseMode(TIM_TypeDef* TIMx, uint16_t TIM_OPMode);
void TIM_SetClockDivision(TIM_TypeDef* TIMx, uint16_t TIM_CKD);
void TIM_Cmd(TIM_TypeDef* TIMx, FunctionalState NewState);

 
void TIM_OC1Init(TIM_TypeDef* TIMx, TIM_OCInitTypeDef* TIM_OCInitStruct);
void TIM_OC2Init(TIM_TypeDef* TIMx, TIM_OCInitTypeDef* TIM_OCInitStruct);
void TIM_OC3Init(TIM_TypeDef* TIMx, TIM_OCInitTypeDef* TIM_OCInitStruct);
void TIM_OC4Init(TIM_TypeDef* TIMx, TIM_OCInitTypeDef* TIM_OCInitStruct);
void TIM_OCStructInit(TIM_OCInitTypeDef* TIM_OCInitStruct);
void TIM_SelectOCxM(TIM_TypeDef* TIMx, uint16_t TIM_Channel, uint16_t TIM_OCMode);
void TIM_SetCompare1(TIM_TypeDef* TIMx, uint32_t Compare1);
void TIM_SetCompare2(TIM_TypeDef* TIMx, uint32_t Compare2);
void TIM_SetCompare3(TIM_TypeDef* TIMx, uint32_t Compare3);
void TIM_SetCompare4(TIM_TypeDef* TIMx, uint32_t Compare4);
void TIM_ForcedOC1Config(TIM_TypeDef* TIMx, uint16_t TIM_ForcedAction);
void TIM_ForcedOC2Config(TIM_TypeDef* TIMx, uint16_t TIM_ForcedAction);
void TIM_ForcedOC3Config(TIM_TypeDef* TIMx, uint16_t TIM_ForcedAction);
void TIM_ForcedOC4Config(TIM_TypeDef* TIMx, uint16_t TIM_ForcedAction);
void TIM_OC1PreloadConfig(TIM_TypeDef* TIMx, uint16_t TIM_OCPreload);
void TIM_OC2PreloadConfig(TIM_TypeDef* TIMx, uint16_t TIM_OCPreload);
void TIM_OC3PreloadConfig(TIM_TypeDef* TIMx, uint16_t TIM_OCPreload);
void TIM_OC4PreloadConfig(TIM_TypeDef* TIMx, uint16_t TIM_OCPreload);
void TIM_OC1FastConfig(TIM_TypeDef* TIMx, uint16_t TIM_OCFast);
void TIM_OC2FastConfig(TIM_TypeDef* TIMx, uint16_t TIM_OCFast);
void TIM_OC3FastConfig(TIM_TypeDef* TIMx, uint16_t TIM_OCFast);
void TIM_OC4FastConfig(TIM_TypeDef* TIMx, uint16_t TIM_OCFast);
void TIM_ClearOC1Ref(TIM_TypeDef* TIMx, uint16_t TIM_OCClear);
void TIM_ClearOC2Ref(TIM_TypeDef* TIMx, uint16_t TIM_OCClear);
void TIM_ClearOC3Ref(TIM_TypeDef* TIMx, uint16_t TIM_OCClear);
void TIM_ClearOC4Ref(TIM_TypeDef* TIMx, uint16_t TIM_OCClear);
void TIM_OC1PolarityConfig(TIM_TypeDef* TIMx, uint16_t TIM_OCPolarity);
void TIM_OC1NPolarityConfig(TIM_TypeDef* TIMx, uint16_t TIM_OCNPolarity);
void TIM_OC2PolarityConfig(TIM_TypeDef* TIMx, uint16_t TIM_OCPolarity);
void TIM_OC2NPolarityConfig(TIM_TypeDef* TIMx, uint16_t TIM_OCNPolarity);
void TIM_OC3PolarityConfig(TIM_TypeDef* TIMx, uint16_t TIM_OCPolarity);
void TIM_OC3NPolarityConfig(TIM_TypeDef* TIMx, uint16_t TIM_OCNPolarity);
void TIM_OC4PolarityConfig(TIM_TypeDef* TIMx, uint16_t TIM_OCPolarity);
void TIM_CCxCmd(TIM_TypeDef* TIMx, uint16_t TIM_Channel, uint16_t TIM_CCx);
void TIM_CCxNCmd(TIM_TypeDef* TIMx, uint16_t TIM_Channel, uint16_t TIM_CCxN);

 
void TIM_ICInit(TIM_TypeDef* TIMx, TIM_ICInitTypeDef* TIM_ICInitStruct);
void TIM_ICStructInit(TIM_ICInitTypeDef* TIM_ICInitStruct);
void TIM_PWMIConfig(TIM_TypeDef* TIMx, TIM_ICInitTypeDef* TIM_ICInitStruct);
uint32_t TIM_GetCapture1(TIM_TypeDef* TIMx);
uint32_t TIM_GetCapture2(TIM_TypeDef* TIMx);
uint32_t TIM_GetCapture3(TIM_TypeDef* TIMx);
uint32_t TIM_GetCapture4(TIM_TypeDef* TIMx);
void TIM_SetIC1Prescaler(TIM_TypeDef* TIMx, uint16_t TIM_ICPSC);
void TIM_SetIC2Prescaler(TIM_TypeDef* TIMx, uint16_t TIM_ICPSC);
void TIM_SetIC3Prescaler(TIM_TypeDef* TIMx, uint16_t TIM_ICPSC);
void TIM_SetIC4Prescaler(TIM_TypeDef* TIMx, uint16_t TIM_ICPSC);

 
void TIM_BDTRConfig(TIM_TypeDef* TIMx, TIM_BDTRInitTypeDef *TIM_BDTRInitStruct);
void TIM_BDTRStructInit(TIM_BDTRInitTypeDef* TIM_BDTRInitStruct);
void TIM_CtrlPWMOutputs(TIM_TypeDef* TIMx, FunctionalState NewState);
void TIM_SelectCOM(TIM_TypeDef* TIMx, FunctionalState NewState);
void TIM_CCPreloadControl(TIM_TypeDef* TIMx, FunctionalState NewState);

 
void TIM_ITConfig(TIM_TypeDef* TIMx, uint16_t TIM_IT, FunctionalState NewState);
void TIM_GenerateEvent(TIM_TypeDef* TIMx, uint16_t TIM_EventSource);
FlagStatus TIM_GetFlagStatus(TIM_TypeDef* TIMx, uint16_t TIM_FLAG);
void TIM_ClearFlag(TIM_TypeDef* TIMx, uint16_t TIM_FLAG);
ITStatus TIM_GetITStatus(TIM_TypeDef* TIMx, uint16_t TIM_IT);
void TIM_ClearITPendingBit(TIM_TypeDef* TIMx, uint16_t TIM_IT);
void TIM_DMAConfig(TIM_TypeDef* TIMx, uint16_t TIM_DMABase, uint16_t TIM_DMABurstLength);
void TIM_DMACmd(TIM_TypeDef* TIMx, uint16_t TIM_DMASource, FunctionalState NewState);
void TIM_SelectCCDMA(TIM_TypeDef* TIMx, FunctionalState NewState);

 
void TIM_InternalClockConfig(TIM_TypeDef* TIMx);
void TIM_ITRxExternalClockConfig(TIM_TypeDef* TIMx, uint16_t TIM_InputTriggerSource);
void TIM_TIxExternalClockConfig(TIM_TypeDef* TIMx, uint16_t TIM_TIxExternalCLKSource,
                                uint16_t TIM_ICPolarity, uint16_t ICFilter);
void TIM_ETRClockMode1Config(TIM_TypeDef* TIMx, uint16_t TIM_ExtTRGPrescaler, uint16_t TIM_ExtTRGPolarity,
                             uint16_t ExtTRGFilter);
void TIM_ETRClockMode2Config(TIM_TypeDef* TIMx, uint16_t TIM_ExtTRGPrescaler, 
                             uint16_t TIM_ExtTRGPolarity, uint16_t ExtTRGFilter);

 
void TIM_SelectInputTrigger(TIM_TypeDef* TIMx, uint16_t TIM_InputTriggerSource);
void TIM_SelectOutputTrigger(TIM_TypeDef* TIMx, uint16_t TIM_TRGOSource);
void TIM_SelectSlaveMode(TIM_TypeDef* TIMx, uint16_t TIM_SlaveMode);
void TIM_SelectMasterSlaveMode(TIM_TypeDef* TIMx, uint16_t TIM_MasterSlaveMode);
void TIM_ETRConfig(TIM_TypeDef* TIMx, uint16_t TIM_ExtTRGPrescaler, uint16_t TIM_ExtTRGPolarity,
                   uint16_t ExtTRGFilter);

    
void TIM_EncoderInterfaceConfig(TIM_TypeDef* TIMx, uint16_t TIM_EncoderMode,
                                uint16_t TIM_IC1Polarity, uint16_t TIM_IC2Polarity);
void TIM_SelectHallSensor(TIM_TypeDef* TIMx, FunctionalState NewState);

 
void TIM_RemapConfig(TIM_TypeDef* TIMx, uint16_t TIM_Remap);









  



 

 
#line 57 "C:\\STM32F4-Discovery_FW_V1.1.0\\Libraries\\STM32F4xx_StdPeriph_Driver\\inc\\stm32f4xx_conf.h"
#line 1 "C:\\STM32F4-Discovery_FW_V1.1.0\\Libraries\\STM32F4xx_StdPeriph_Driver\\inc\\stm32f4xx_usart.h"




















  

 







 
#line 33 "C:\\STM32F4-Discovery_FW_V1.1.0\\Libraries\\STM32F4xx_StdPeriph_Driver\\inc\\stm32f4xx_usart.h"



 



  

  



  
  
typedef struct
{
  uint32_t USART_BaudRate;            



 

  uint16_t USART_WordLength;          
 

  uint16_t USART_StopBits;            
 

  uint16_t USART_Parity;              




 
 
  uint16_t USART_Mode;                
 

  uint16_t USART_HardwareFlowControl; 

 
} USART_InitTypeDef;



  
  
typedef struct
{

  uint16_t USART_Clock;   
 

  uint16_t USART_CPOL;    
 

  uint16_t USART_CPHA;    
 

  uint16_t USART_LastBit; 

 
} USART_ClockInitTypeDef;

 



  
  
#line 110 "C:\\STM32F4-Discovery_FW_V1.1.0\\Libraries\\STM32F4xx_StdPeriph_Driver\\inc\\stm32f4xx_usart.h"








  
  


                                    




  



  
  
#line 141 "C:\\STM32F4-Discovery_FW_V1.1.0\\Libraries\\STM32F4xx_StdPeriph_Driver\\inc\\stm32f4xx_usart.h"


  



  
  
#line 155 "C:\\STM32F4-Discovery_FW_V1.1.0\\Libraries\\STM32F4xx_StdPeriph_Driver\\inc\\stm32f4xx_usart.h"


  



  
  





  



  
#line 182 "C:\\STM32F4-Discovery_FW_V1.1.0\\Libraries\\STM32F4xx_StdPeriph_Driver\\inc\\stm32f4xx_usart.h"


  



  






  



 
  






  



 







 



 







  



 
  
#line 249 "C:\\STM32F4-Discovery_FW_V1.1.0\\Libraries\\STM32F4xx_StdPeriph_Driver\\inc\\stm32f4xx_usart.h"



 



 

#line 270 "C:\\STM32F4-Discovery_FW_V1.1.0\\Libraries\\STM32F4xx_StdPeriph_Driver\\inc\\stm32f4xx_usart.h"


 



 







  



 







 



 
  







 



 







  



 

#line 342 "C:\\STM32F4-Discovery_FW_V1.1.0\\Libraries\\STM32F4xx_StdPeriph_Driver\\inc\\stm32f4xx_usart.h"
                              








  



  

 
   

  
void USART_DeInit(USART_TypeDef* USARTx);

 
void USART_Init(USART_TypeDef* USARTx, USART_InitTypeDef* USART_InitStruct);
void USART_StructInit(USART_InitTypeDef* USART_InitStruct);
void USART_ClockInit(USART_TypeDef* USARTx, USART_ClockInitTypeDef* USART_ClockInitStruct);
void USART_ClockStructInit(USART_ClockInitTypeDef* USART_ClockInitStruct);
void USART_Cmd(USART_TypeDef* USARTx, FunctionalState NewState);
void USART_SetPrescaler(USART_TypeDef* USARTx, uint8_t USART_Prescaler);
void USART_OverSampling8Cmd(USART_TypeDef* USARTx, FunctionalState NewState);
void USART_OneBitMethodCmd(USART_TypeDef* USARTx, FunctionalState NewState);

  
void USART_SendData(USART_TypeDef* USARTx, uint16_t Data);
uint16_t USART_ReceiveData(USART_TypeDef* USARTx);

 
void USART_SetAddress(USART_TypeDef* USARTx, uint8_t USART_Address);
void USART_WakeUpConfig(USART_TypeDef* USARTx, uint16_t USART_WakeUp);
void USART_ReceiverWakeUpCmd(USART_TypeDef* USARTx, FunctionalState NewState);

 
void USART_LINBreakDetectLengthConfig(USART_TypeDef* USARTx, uint16_t USART_LINBreakDetectLength);
void USART_LINCmd(USART_TypeDef* USARTx, FunctionalState NewState);
void USART_SendBreak(USART_TypeDef* USARTx);

 
void USART_HalfDuplexCmd(USART_TypeDef* USARTx, FunctionalState NewState);

 
void USART_SmartCardCmd(USART_TypeDef* USARTx, FunctionalState NewState);
void USART_SmartCardNACKCmd(USART_TypeDef* USARTx, FunctionalState NewState);
void USART_SetGuardTime(USART_TypeDef* USARTx, uint8_t USART_GuardTime);

 
void USART_IrDAConfig(USART_TypeDef* USARTx, uint16_t USART_IrDAMode);
void USART_IrDACmd(USART_TypeDef* USARTx, FunctionalState NewState);

 
void USART_DMACmd(USART_TypeDef* USARTx, uint16_t USART_DMAReq, FunctionalState NewState);

 
void USART_ITConfig(USART_TypeDef* USARTx, uint16_t USART_IT, FunctionalState NewState);
FlagStatus USART_GetFlagStatus(USART_TypeDef* USARTx, uint16_t USART_FLAG);
void USART_ClearFlag(USART_TypeDef* USARTx, uint16_t USART_FLAG);
ITStatus USART_GetITStatus(USART_TypeDef* USARTx, uint16_t USART_IT);
void USART_ClearITPendingBit(USART_TypeDef* USARTx, uint16_t USART_IT);









  



  

 
#line 58 "C:\\STM32F4-Discovery_FW_V1.1.0\\Libraries\\STM32F4xx_StdPeriph_Driver\\inc\\stm32f4xx_conf.h"
#line 1 "C:\\STM32F4-Discovery_FW_V1.1.0\\Libraries\\STM32F4xx_StdPeriph_Driver\\inc\\stm32f4xx_wwdg.h"




















 

 







 
#line 33 "C:\\STM32F4-Discovery_FW_V1.1.0\\Libraries\\STM32F4xx_StdPeriph_Driver\\inc\\stm32f4xx_wwdg.h"



 



  

 
 



  
  


 
  
#line 63 "C:\\STM32F4-Discovery_FW_V1.1.0\\Libraries\\STM32F4xx_StdPeriph_Driver\\inc\\stm32f4xx_wwdg.h"



  



  

 
 
  
   
void WWDG_DeInit(void);

 
void WWDG_SetPrescaler(uint32_t WWDG_Prescaler);
void WWDG_SetWindowValue(uint8_t WindowValue);
void WWDG_EnableIT(void);
void WWDG_SetCounter(uint8_t Counter);

 
void WWDG_Enable(uint8_t Counter);

 
FlagStatus WWDG_GetFlagStatus(void);
void WWDG_ClearFlag(void);









  



  

 
#line 59 "C:\\STM32F4-Discovery_FW_V1.1.0\\Libraries\\STM32F4xx_StdPeriph_Driver\\inc\\stm32f4xx_conf.h"
#line 1 "C:\\STM32F4-Discovery_FW_V1.1.0\\Libraries\\STM32F4xx_StdPeriph_Driver\\inc\\misc.h"




















 

 







 
#line 33 "C:\\STM32F4-Discovery_FW_V1.1.0\\Libraries\\STM32F4xx_StdPeriph_Driver\\inc\\misc.h"



 



 

 



 

typedef struct
{
  uint8_t NVIC_IRQChannel;                    


 

  uint8_t NVIC_IRQChannelPreemptionPriority;  


 

  uint8_t NVIC_IRQChannelSubPriority;         


 

  FunctionalState NVIC_IRQChannelCmd;         

    
} NVIC_InitTypeDef;
 
 



 



 







 



 

#line 98 "C:\\STM32F4-Discovery_FW_V1.1.0\\Libraries\\STM32F4xx_StdPeriph_Driver\\inc\\misc.h"


 



 

#line 116 "C:\\STM32F4-Discovery_FW_V1.1.0\\Libraries\\STM32F4xx_StdPeriph_Driver\\inc\\misc.h"















 



 







 



 

 
 

void NVIC_PriorityGroupConfig(uint32_t NVIC_PriorityGroup);
void NVIC_Init(NVIC_InitTypeDef* NVIC_InitStruct);
void NVIC_SetVectorTable(uint32_t NVIC_VectTab, uint32_t Offset);
void NVIC_SystemLPConfig(uint8_t LowPowerMode, FunctionalState NewState);
void SysTick_CLKSourceConfig(uint32_t SysTick_CLKSource);









 



 

 
#line 60 "C:\\STM32F4-Discovery_FW_V1.1.0\\Libraries\\STM32F4xx_StdPeriph_Driver\\inc\\stm32f4xx_conf.h"

 
 



 
   



 
 

 
#line 91 "C:\\STM32F4-Discovery_FW_V1.1.0\\Libraries\\STM32F4xx_StdPeriph_Driver\\inc\\stm32f4xx_conf.h"



 
#line 6966 ".\\Include\\stm32f4xx.h"




 

















 









 

  

 

 
#line 2 "Source\\main.c"
#line 3 "Source\\main.c"
#line 1 "C:\\STM32F4-Discovery_FW_V1.1.0\\Libraries\\CMSIS\\Include\\arm_math.h"



























 
































































































 




 








 









 



 






































































 



 



 



 


 






 



 





#line 1 "C:\\STM32F4-Discovery_FW_V1.1.0\\Libraries\\CMSIS\\Include\\core_cm4.h"
 




















 








#line 149 "C:\\STM32F4-Discovery_FW_V1.1.0\\Libraries\\CMSIS\\Include\\core_cm4.h"

#line 1375 "C:\\STM32F4-Discovery_FW_V1.1.0\\Libraries\\CMSIS\\Include\\core_cm4.h"

#line 258 "C:\\STM32F4-Discovery_FW_V1.1.0\\Libraries\\CMSIS\\Include\\arm_math.h"
#line 266 "C:\\STM32F4-Discovery_FW_V1.1.0\\Libraries\\CMSIS\\Include\\arm_math.h"

#line 1 "C:\\Keil\\ARM\\ARMCC\\bin\\..\\include\\string.h"
 
 
 
 




 








 












#line 38 "C:\\Keil\\ARM\\ARMCC\\bin\\..\\include\\string.h"


  
  typedef unsigned int size_t;








extern __declspec(__nothrow) void *memcpy(void * __restrict  ,
                    const void * __restrict  , size_t  ) __attribute__((__nonnull__(1,2)));
   




 
extern __declspec(__nothrow) void *memmove(void *  ,
                    const void *  , size_t  ) __attribute__((__nonnull__(1,2)));
   







 
extern __declspec(__nothrow) char *strcpy(char * __restrict  , const char * __restrict  ) __attribute__((__nonnull__(1,2)));
   




 
extern __declspec(__nothrow) char *strncpy(char * __restrict  , const char * __restrict  , size_t  ) __attribute__((__nonnull__(1,2)));
   





 

extern __declspec(__nothrow) char *strcat(char * __restrict  , const char * __restrict  ) __attribute__((__nonnull__(1,2)));
   




 
extern __declspec(__nothrow) char *strncat(char * __restrict  , const char * __restrict  , size_t  ) __attribute__((__nonnull__(1,2)));
   






 






 

extern __declspec(__nothrow) int memcmp(const void *  , const void *  , size_t  ) __attribute__((__nonnull__(1,2)));
   





 
extern __declspec(__nothrow) int strcmp(const char *  , const char *  ) __attribute__((__nonnull__(1,2)));
   




 
extern __declspec(__nothrow) int strncmp(const char *  , const char *  , size_t  ) __attribute__((__nonnull__(1,2)));
   






 
extern __declspec(__nothrow) int strcasecmp(const char *  , const char *  ) __attribute__((__nonnull__(1,2)));
   





 
extern __declspec(__nothrow) int strncasecmp(const char *  , const char *  , size_t  ) __attribute__((__nonnull__(1,2)));
   






 
extern __declspec(__nothrow) int strcoll(const char *  , const char *  ) __attribute__((__nonnull__(1,2)));
   







 

extern __declspec(__nothrow) size_t strxfrm(char * __restrict  , const char * __restrict  , size_t  ) __attribute__((__nonnull__(2)));
   













 


#line 185 "C:\\Keil\\ARM\\ARMCC\\bin\\..\\include\\string.h"
extern __declspec(__nothrow) void *memchr(const void *  , int  , size_t  ) __attribute__((__nonnull__(1)));

   





 

#line 201 "C:\\Keil\\ARM\\ARMCC\\bin\\..\\include\\string.h"
extern __declspec(__nothrow) char *strchr(const char *  , int  ) __attribute__((__nonnull__(1)));

   




 

extern __declspec(__nothrow) size_t strcspn(const char *  , const char *  ) __attribute__((__nonnull__(1,2)));
   




 

#line 224 "C:\\Keil\\ARM\\ARMCC\\bin\\..\\include\\string.h"
extern __declspec(__nothrow) char *strpbrk(const char *  , const char *  ) __attribute__((__nonnull__(1,2)));

   




 

#line 239 "C:\\Keil\\ARM\\ARMCC\\bin\\..\\include\\string.h"
extern __declspec(__nothrow) char *strrchr(const char *  , int  ) __attribute__((__nonnull__(1)));

   





 

extern __declspec(__nothrow) size_t strspn(const char *  , const char *  ) __attribute__((__nonnull__(1,2)));
   



 

#line 262 "C:\\Keil\\ARM\\ARMCC\\bin\\..\\include\\string.h"
extern __declspec(__nothrow) char *strstr(const char *  , const char *  ) __attribute__((__nonnull__(1,2)));

   





 

extern __declspec(__nothrow) char *strtok(char * __restrict  , const char * __restrict  ) __attribute__((__nonnull__(2)));
extern __declspec(__nothrow) char *_strtok_r(char *  , const char *  , char **  ) __attribute__((__nonnull__(2,3)));

extern __declspec(__nothrow) char *strtok_r(char *  , const char *  , char **  ) __attribute__((__nonnull__(2,3)));

   

































 

extern __declspec(__nothrow) void *memset(void *  , int  , size_t  ) __attribute__((__nonnull__(1)));
   



 
extern __declspec(__nothrow) char *strerror(int  );
   





 
extern __declspec(__nothrow) size_t strlen(const char *  ) __attribute__((__nonnull__(1)));
   



 

extern __declspec(__nothrow) size_t strlcpy(char *  , const char *  , size_t  ) __attribute__((__nonnull__(1,2)));
   
















 

extern __declspec(__nothrow) size_t strlcat(char *  , const char *  , size_t  ) __attribute__((__nonnull__(1,2)));
   






















 

extern __declspec(__nothrow) void _membitcpybl(void *  , const void *  , int  , int  , size_t  ) __attribute__((__nonnull__(1,2)));
extern __declspec(__nothrow) void _membitcpybb(void *  , const void *  , int  , int  , size_t  ) __attribute__((__nonnull__(1,2)));
extern __declspec(__nothrow) void _membitcpyhl(void *  , const void *  , int  , int  , size_t  ) __attribute__((__nonnull__(1,2)));
extern __declspec(__nothrow) void _membitcpyhb(void *  , const void *  , int  , int  , size_t  ) __attribute__((__nonnull__(1,2)));
extern __declspec(__nothrow) void _membitcpywl(void *  , const void *  , int  , int  , size_t  ) __attribute__((__nonnull__(1,2)));
extern __declspec(__nothrow) void _membitcpywb(void *  , const void *  , int  , int  , size_t  ) __attribute__((__nonnull__(1,2)));
extern __declspec(__nothrow) void _membitmovebl(void *  , const void *  , int  , int  , size_t  ) __attribute__((__nonnull__(1,2)));
extern __declspec(__nothrow) void _membitmovebb(void *  , const void *  , int  , int  , size_t  ) __attribute__((__nonnull__(1,2)));
extern __declspec(__nothrow) void _membitmovehl(void *  , const void *  , int  , int  , size_t  ) __attribute__((__nonnull__(1,2)));
extern __declspec(__nothrow) void _membitmovehb(void *  , const void *  , int  , int  , size_t  ) __attribute__((__nonnull__(1,2)));
extern __declspec(__nothrow) void _membitmovewl(void *  , const void *  , int  , int  , size_t  ) __attribute__((__nonnull__(1,2)));
extern __declspec(__nothrow) void _membitmovewb(void *  , const void *  , int  , int  , size_t  ) __attribute__((__nonnull__(1,2)));
    














































 







#line 494 "C:\\Keil\\ARM\\ARMCC\\bin\\..\\include\\string.h"



 

#line 269 "C:\\STM32F4-Discovery_FW_V1.1.0\\Libraries\\CMSIS\\Include\\arm_math.h"
#line 1 "C:\\Keil\\ARM\\ARMCC\\bin\\..\\include\\math.h"




 





 












 








 






#line 48 "C:\\Keil\\ARM\\ARMCC\\bin\\..\\include\\math.h"

#line 62 "C:\\Keil\\ARM\\ARMCC\\bin\\..\\include\\math.h"

   




 















 
#line 93 "C:\\Keil\\ARM\\ARMCC\\bin\\..\\include\\math.h"











 





extern __softfp unsigned __ARM_dcmp4(double  , double  );
extern __softfp unsigned __ARM_fcmp4(float  , float  );
    




 

extern __declspec(__nothrow) __softfp int __ARM_fpclassifyf(float  );
extern __declspec(__nothrow) __softfp int __ARM_fpclassify(double  );
     
     

static inline __declspec(__nothrow) __softfp int __ARM_isfinitef(float __x)
{
    return (((*(unsigned *)&(__x)) >> 23) & 0xff) != 0xff;
}
static inline __declspec(__nothrow) __softfp int __ARM_isfinite(double __x)
{
    return (((*(1 + (unsigned *)&(__x))) >> 20) & 0x7ff) != 0x7ff;
}
     
     

static inline __declspec(__nothrow) __softfp int __ARM_isinff(float __x)
{
    return ((*(unsigned *)&(__x)) << 1) == 0xff000000;
}
static inline __declspec(__nothrow) __softfp int __ARM_isinf(double __x)
{
    return (((*(1 + (unsigned *)&(__x))) << 1) == 0xffe00000) && ((*(unsigned *)&(__x)) == 0);
}
     
     

static inline __declspec(__nothrow) __softfp int __ARM_islessgreaterf(float __x, float __y)
{
    unsigned __f = __ARM_fcmp4(__x, __y) >> 28;
    return (__f == 8) || (__f == 2);  
}
static inline __declspec(__nothrow) __softfp int __ARM_islessgreater(double __x, double __y)
{
    unsigned __f = __ARM_dcmp4(__x, __y) >> 28;
    return (__f == 8) || (__f == 2);  
}
    


 

static inline __declspec(__nothrow) __softfp int __ARM_isnanf(float __x)
{
    return (0x7f800000 - ((*(unsigned *)&(__x)) & 0x7fffffff)) >> 31;
}
static inline __declspec(__nothrow) __softfp int __ARM_isnan(double __x)
{
    unsigned __xf = (*(1 + (unsigned *)&(__x))) | (((*(unsigned *)&(__x)) == 0) ? 0 : 1);
    return (0x7ff00000 - (__xf & 0x7fffffff)) >> 31;
}
     
     

static inline __declspec(__nothrow) __softfp int __ARM_isnormalf(float __x)
{
    unsigned __xe = ((*(unsigned *)&(__x)) >> 23) & 0xff;
    return (__xe != 0xff) && (__xe != 0);
}
static inline __declspec(__nothrow) __softfp int __ARM_isnormal(double __x)
{
    unsigned __xe = ((*(1 + (unsigned *)&(__x))) >> 20) & 0x7ff;
    return (__xe != 0x7ff) && (__xe != 0);
}
     
     

static inline __declspec(__nothrow) __softfp int __ARM_signbitf(float __x)
{
    return (*(unsigned *)&(__x)) >> 31;
}
static inline __declspec(__nothrow) __softfp int __ARM_signbit(double __x)
{
    return (*(1 + (unsigned *)&(__x))) >> 31;
}
     
     








#line 211 "C:\\Keil\\ARM\\ARMCC\\bin\\..\\include\\math.h"



   
  typedef float float_t;
  typedef double double_t;







extern const int math_errhandling;



extern __declspec(__nothrow) double acos(double  );
    
    
    
extern __declspec(__nothrow) double asin(double  );
    
    
    
    

extern __declspec(__nothrow) __pure double atan(double  );
    
    

extern __declspec(__nothrow) double atan2(double  , double  );
    
    
    
    

extern __declspec(__nothrow) double cos(double  );
    
    
    
    
extern __declspec(__nothrow) double sin(double  );
    
    
    
    

extern void __use_accurate_range_reduction(void);
    
    

extern __declspec(__nothrow) double tan(double  );
    
    
    
    

extern __declspec(__nothrow) double cosh(double  );
    
    
    
    
extern __declspec(__nothrow) double sinh(double  );
    
    
    
    
    

extern __declspec(__nothrow) __pure double tanh(double  );
    
    

extern __declspec(__nothrow) double exp(double  );
    
    
    
    
    

extern __declspec(__nothrow) double frexp(double  , int *  ) __attribute__((__nonnull__(2)));
    
    
    
    
    
    

extern __declspec(__nothrow) double ldexp(double  , int  );
    
    
    
    
extern __declspec(__nothrow) double log(double  );
    
    
    
    
    
extern __declspec(__nothrow) double log10(double  );
    
    
    
extern __declspec(__nothrow) double modf(double  , double *  ) __attribute__((__nonnull__(2)));
    
    
    
    

extern __declspec(__nothrow) double pow(double  , double  );
    
    
    
    
    
    
extern __declspec(__nothrow) double sqrt(double  );
    
    
    




    inline double _sqrt(double __x) { return sqrt(__x); }


    inline float _sqrtf(float __x) { return __sqrtf(__x); }



    



 

extern __declspec(__nothrow) __pure double ceil(double  );
    
    
extern __declspec(__nothrow) __pure double fabs(double  );
    
    

extern __declspec(__nothrow) __pure double floor(double  );
    
    

extern __declspec(__nothrow) double fmod(double  , double  );
    
    
    
    
    

    









 



extern __declspec(__nothrow) double acosh(double  );
    

 
extern __declspec(__nothrow) double asinh(double  );
    

 
extern __declspec(__nothrow) double atanh(double  );
    

 
extern __declspec(__nothrow) double cbrt(double  );
    

 
inline __declspec(__nothrow) __pure double copysign(double __x, double __y)
    

 
{
    (*(1 + (unsigned *)&(__x))) = ((*(1 + (unsigned *)&(__x))) & 0x7fffffff) | ((*(1 + (unsigned *)&(__y))) & 0x80000000);
    return __x;
}
inline __declspec(__nothrow) __pure float copysignf(float __x, float __y)
    

 
{
    (*(unsigned *)&(__x)) = ((*(unsigned *)&(__x)) & 0x7fffffff) | ((*(unsigned *)&(__y)) & 0x80000000);
    return __x;
}
extern __declspec(__nothrow) double erf(double  );
    

 
extern __declspec(__nothrow) double erfc(double  );
    

 
extern __declspec(__nothrow) double expm1(double  );
    

 



    

 






#line 445 "C:\\Keil\\ARM\\ARMCC\\bin\\..\\include\\math.h"


extern __declspec(__nothrow) double hypot(double  , double  );
    




 
extern __declspec(__nothrow) int ilogb(double  );
    

 
extern __declspec(__nothrow) int ilogbf(float  );
    

 
extern __declspec(__nothrow) int ilogbl(long double  );
    

 







    

 





    



 





    



 





    

 





    



 





    



 





    



 





    

 





    

 





    


 

extern __declspec(__nothrow) double lgamma (double  );
    


 
extern __declspec(__nothrow) double log1p(double  );
    

 
extern __declspec(__nothrow) double logb(double  );
    

 
extern __declspec(__nothrow) float logbf(float  );
    

 
extern __declspec(__nothrow) long double logbl(long double  );
    

 
extern __declspec(__nothrow) double nextafter(double  , double  );
    


 
extern __declspec(__nothrow) float nextafterf(float  , float  );
    


 
extern __declspec(__nothrow) long double nextafterl(long double  , long double  );
    


 
extern __declspec(__nothrow) double nexttoward(double  , long double  );
    


 
extern __declspec(__nothrow) float nexttowardf(float  , long double  );
    


 
extern __declspec(__nothrow) long double nexttowardl(long double  , long double  );
    


 
extern __declspec(__nothrow) double remainder(double  , double  );
    

 
extern __declspec(__nothrow) __pure double rint(double  );
    

 
extern __declspec(__nothrow) double scalbln(double  , long int  );
    

 
extern __declspec(__nothrow) float scalblnf(float  , long int  );
    

 
extern __declspec(__nothrow) long double scalblnl(long double  , long int  );
    

 
extern __declspec(__nothrow) double scalbn(double  , int  );
    

 
extern __declspec(__nothrow) float scalbnf(float  , int  );
    

 
extern __declspec(__nothrow) long double scalbnl(long double  , int  );
    

 




    

 



 
extern __declspec(__nothrow) __pure float _fabsf(float);  
inline __declspec(__nothrow) __pure float fabsf(float __f) { return _fabsf(__f); }
extern __declspec(__nothrow) float sinf(float  );
extern __declspec(__nothrow) float cosf(float  );
extern __declspec(__nothrow) float tanf(float  );
extern __declspec(__nothrow) float acosf(float  );
extern __declspec(__nothrow) float asinf(float  );
extern __declspec(__nothrow) float atanf(float  );
extern __declspec(__nothrow) float atan2f(float  , float  );
extern __declspec(__nothrow) float sinhf(float  );
extern __declspec(__nothrow) float coshf(float  );
extern __declspec(__nothrow) float tanhf(float  );
extern __declspec(__nothrow) float expf(float  );
extern __declspec(__nothrow) float logf(float  );
extern __declspec(__nothrow) float log10f(float  );
extern __declspec(__nothrow) float powf(float  , float  );
extern __declspec(__nothrow) float sqrtf(float  );
extern __declspec(__nothrow) float ldexpf(float  , int  );
extern __declspec(__nothrow) float frexpf(float  , int *  ) __attribute__((__nonnull__(2)));
extern __declspec(__nothrow) __pure float ceilf(float  );
extern __declspec(__nothrow) __pure float floorf(float  );
extern __declspec(__nothrow) float fmodf(float  , float  );
extern __declspec(__nothrow) float modff(float  , float *  ) __attribute__((__nonnull__(2)));

 
 













 
__declspec(__nothrow) long double acosl(long double );
__declspec(__nothrow) long double asinl(long double );
__declspec(__nothrow) long double atanl(long double );
__declspec(__nothrow) long double atan2l(long double , long double );
__declspec(__nothrow) long double ceill(long double );
__declspec(__nothrow) long double cosl(long double );
__declspec(__nothrow) long double coshl(long double );
__declspec(__nothrow) long double expl(long double );
__declspec(__nothrow) long double fabsl(long double );
__declspec(__nothrow) long double floorl(long double );
__declspec(__nothrow) long double fmodl(long double , long double );
__declspec(__nothrow) long double frexpl(long double , int* ) __attribute__((__nonnull__(2)));
__declspec(__nothrow) long double ldexpl(long double , int );
__declspec(__nothrow) long double logl(long double );
__declspec(__nothrow) long double log10l(long double );
__declspec(__nothrow) long double modfl(long double  , long double *  ) __attribute__((__nonnull__(2)));
__declspec(__nothrow) long double powl(long double , long double );
__declspec(__nothrow) long double sinl(long double );
__declspec(__nothrow) long double sinhl(long double );
__declspec(__nothrow) long double sqrtl(long double );
__declspec(__nothrow) long double tanl(long double );
__declspec(__nothrow) long double tanhl(long double );





 
extern __declspec(__nothrow) float acoshf(float  );
__declspec(__nothrow) long double acoshl(long double );
extern __declspec(__nothrow) float asinhf(float  );
__declspec(__nothrow) long double asinhl(long double );
extern __declspec(__nothrow) float atanhf(float  );
__declspec(__nothrow) long double atanhl(long double );
__declspec(__nothrow) long double copysignl(long double , long double );
extern __declspec(__nothrow) float cbrtf(float  );
__declspec(__nothrow) long double cbrtl(long double );
extern __declspec(__nothrow) float erff(float  );
__declspec(__nothrow) long double erfl(long double );
extern __declspec(__nothrow) float erfcf(float  );
__declspec(__nothrow) long double erfcl(long double );
extern __declspec(__nothrow) float expm1f(float  );
__declspec(__nothrow) long double expm1l(long double );
extern __declspec(__nothrow) float log1pf(float  );
__declspec(__nothrow) long double log1pl(long double );
extern __declspec(__nothrow) float hypotf(float  , float  );
__declspec(__nothrow) long double hypotl(long double , long double );
extern __declspec(__nothrow) float lgammaf(float  );
__declspec(__nothrow) long double lgammal(long double );
extern __declspec(__nothrow) float remainderf(float  , float  );
__declspec(__nothrow) long double remainderl(long double , long double );
extern __declspec(__nothrow) float rintf(float  );
__declspec(__nothrow) long double rintl(long double );






 
extern __declspec(__nothrow) double exp2(double  );  
extern __declspec(__nothrow) float exp2f(float  );
__declspec(__nothrow) long double exp2l(long double );
extern __declspec(__nothrow) double fdim(double  , double  );
extern __declspec(__nothrow) float fdimf(float  , float  );
__declspec(__nothrow) long double fdiml(long double , long double );
#line 769 "C:\\Keil\\ARM\\ARMCC\\bin\\..\\include\\math.h"
extern __declspec(__nothrow) double fma(double  , double  , double  );
extern __declspec(__nothrow) float fmaf(float  , float  , float  );
inline __declspec(__nothrow) long double fmal(long double __x, long double __y, long double __z)     { return (long double)fma((double)__x, (double)__y, (double)__z); }

extern __declspec(__nothrow) __pure double fmax(double  , double  );
extern __declspec(__nothrow) __pure float fmaxf(float  , float  );
__declspec(__nothrow) long double fmaxl(long double , long double );
extern __declspec(__nothrow) __pure double fmin(double  , double  );
extern __declspec(__nothrow) __pure float fminf(float  , float  );
__declspec(__nothrow) long double fminl(long double , long double );
extern __declspec(__nothrow) double log2(double  );  
extern __declspec(__nothrow) float log2f(float  );
__declspec(__nothrow) long double log2l(long double );
extern __declspec(__nothrow) long lrint(double  );
extern __declspec(__nothrow) long lrintf(float  );
inline __declspec(__nothrow) long lrintl(long double __x)     { return lrint((double)__x); }

extern __declspec(__nothrow) __int64 llrint(double  );
extern __declspec(__nothrow) __int64 llrintf(float  );
inline __declspec(__nothrow) __int64 llrintl(long double __x)     { return llrint((double)__x); }

extern __declspec(__nothrow) long lround(double  );
extern __declspec(__nothrow) long lroundf(float  );
inline __declspec(__nothrow) long lroundl(long double __x)     { return lround((double)__x); }

extern __declspec(__nothrow) __int64 llround(double  );
extern __declspec(__nothrow) __int64 llroundf(float  );
inline __declspec(__nothrow) __int64 llroundl(long double __x)     { return llround((double)__x); }

extern __declspec(__nothrow) __pure double nan(const char * );
extern __declspec(__nothrow) __pure float nanf(const char * );
inline __declspec(__nothrow) __pure long double nanl(const char *__t)     { return (long double)nan(__t); }
#line 808 "C:\\Keil\\ARM\\ARMCC\\bin\\..\\include\\math.h"
extern __declspec(__nothrow) __pure double nearbyint(double  );
extern __declspec(__nothrow) __pure float nearbyintf(float  );
__declspec(__nothrow) long double nearbyintl(long double );
extern  double remquo(double  , double  , int * );
extern  float remquof(float  , float  , int * );
inline long double remquol(long double __x, long double __y, int *__q)     { return (long double)remquo((double)__x, (double)__y, __q); }

extern __declspec(__nothrow) __pure double round(double  );
extern __declspec(__nothrow) __pure float roundf(float  );
__declspec(__nothrow) long double roundl(long double );
extern __declspec(__nothrow) double tgamma(double  );  
extern __declspec(__nothrow) float tgammaf(float  );
__declspec(__nothrow) long double tgammal(long double );
extern __declspec(__nothrow) __pure double trunc(double  );
extern __declspec(__nothrow) __pure float truncf(float  );
__declspec(__nothrow) long double truncl(long double );






#line 980 "C:\\Keil\\ARM\\ARMCC\\bin\\..\\include\\math.h"











#line 1182 "C:\\Keil\\ARM\\ARMCC\\bin\\..\\include\\math.h"



 

#line 270 "C:\\STM32F4-Discovery_FW_V1.1.0\\Libraries\\CMSIS\\Include\\arm_math.h"






  

 






  

 





  

 
   
   



  

 

  typedef enum
    {
      ARM_MATH_SUCCESS = 0,               
      ARM_MATH_ARGUMENT_ERROR = -1,       
      ARM_MATH_LENGTH_ERROR = -2,         
      ARM_MATH_SIZE_MISMATCH = -3,        
      ARM_MATH_NANINF = -4,               
      ARM_MATH_SINGULAR = -5,             
      ARM_MATH_TEST_FAILURE = -6          
    } arm_status;

  

 
  typedef int8_t q7_t;

  

 
  typedef int16_t q15_t;

  

 
  typedef int32_t q31_t;

  

 
  typedef int64_t q63_t;

  

 
  typedef float float32_t;

  

 
  typedef double float64_t;

  

 


#line 359 "C:\\STM32F4-Discovery_FW_V1.1.0\\Libraries\\CMSIS\\Include\\arm_math.h"


   

 


#line 378 "C:\\STM32F4-Discovery_FW_V1.1.0\\Libraries\\CMSIS\\Include\\arm_math.h"


  

 
  static __inline q31_t clip_q63_to_q31(
					q63_t x)
  {
    return ((q31_t) (x >> 32) != ((q31_t) x >> 31)) ?
      ((0x7FFFFFFF ^ ((q31_t) (x >> 63)))) : (q31_t) x;
  }

  

 
  static __inline q15_t clip_q63_to_q15(
					q63_t x)
  {
    return ((q31_t) (x >> 32) != ((q31_t) x >> 31)) ?
      ((0x7FFF ^ ((q15_t) (x >> 63)))) : (q15_t) (x >> 15);
  }

  

 
  static __inline q7_t clip_q31_to_q7(
				      q31_t x)
  {
    return ((q31_t) (x >> 24) != ((q31_t) x >> 23)) ?
      ((0x7F ^ ((q7_t) (x >> 31)))) : (q7_t) x;
  }

  

 
  static __inline q15_t clip_q31_to_q15(
					q31_t x)
  {
    return ((q31_t) (x >> 16) != ((q31_t) x >> 15)) ?
      ((0x7FFF ^ ((q15_t) (x >> 31)))) : (q15_t) x;
  }

  

 

  static __inline q63_t mult32x64(
				  q63_t x,
				  q31_t y)
  {
    return ((((q63_t) (x & 0x00000000FFFFFFFF) * y) >> 32) +
            (((q63_t) (x >> 32) * y)));
  }






#line 458 "C:\\STM32F4-Discovery_FW_V1.1.0\\Libraries\\CMSIS\\Include\\arm_math.h"

  

 

  static __inline uint32_t arm_recip_q31(
					 q31_t in,
					 q31_t * dst,
					 q31_t * pRecipTable)
  {

    uint32_t out, tempVal;
    uint32_t index, i;
    uint32_t signBits;

    if(in > 0)
      {
	signBits = __clz(in) - 1;
      }
    else
      {
	signBits = __clz(-in) - 1;
      }

     
    in = in << signBits;

     
    index = (uint32_t) (in >> 24u);
    index = (index & 0x0000003F);

     
    out = pRecipTable[index];

     
     
    for (i = 0u; i < 2u; i++)
      {
	tempVal = (q31_t) (((q63_t) in * out) >> 31u);
	tempVal = 0x7FFFFFFF - tempVal;
	 
	
	out = (q31_t) clip_q63_to_q31(((q63_t) out * tempVal) >> 30u);
      }

     
    *dst = out;

     
    return (signBits + 1u);

  }

  

 
  static __inline uint32_t arm_recip_q15(
					 q15_t in,
					 q15_t * dst,
					 q15_t * pRecipTable)
  {

    uint32_t out = 0, tempVal = 0;
    uint32_t index = 0, i = 0;
    uint32_t signBits = 0;

    if(in > 0)
      {
	signBits = __clz(in) - 17;
      }
    else
      {
	signBits = __clz(-in) - 17;
      }

     
    in = in << signBits;

     
    index = in >> 8;
    index = (index & 0x0000003F);

     
    out = pRecipTable[index];

     
     
    for (i = 0; i < 2; i++)
      {
	tempVal = (q15_t) (((q31_t) in * out) >> 15);
	tempVal = 0x7FFF - tempVal;
	 
	out = (q15_t) (((q31_t) out * tempVal) >> 14);
      }

     
    *dst = out;

     
    return (signBits + 1);

  }


  

 
#line 604 "C:\\STM32F4-Discovery_FW_V1.1.0\\Libraries\\CMSIS\\Include\\arm_math.h"



  

 
#line 971 "C:\\STM32F4-Discovery_FW_V1.1.0\\Libraries\\CMSIS\\Include\\arm_math.h"


  

 
  typedef struct
  {
    uint16_t numTaps;         
    q7_t *pState;             
    q7_t *pCoeffs;            
  } arm_fir_instance_q7;

  

 
  typedef struct
  {
    uint16_t numTaps;          
    q15_t *pState;             
    q15_t *pCoeffs;            
  } arm_fir_instance_q15;

  

 
  typedef struct
  {
    uint16_t numTaps;          
    q31_t *pState;             
    q31_t *pCoeffs;            
  } arm_fir_instance_q31;

  

 
  typedef struct
  {
    uint16_t numTaps;      
    float32_t *pState;     
    float32_t *pCoeffs;    
  } arm_fir_instance_f32;


  






 
  void arm_fir_q7(
		  const arm_fir_instance_q7 * S,
		   q7_t * pSrc,
		  q7_t * pDst,
		  uint32_t blockSize);


  







 
  void arm_fir_init_q7(
		       arm_fir_instance_q7 * S,
		       uint16_t numTaps,
		       q7_t * pCoeffs,
		       q7_t * pState,
		       uint32_t blockSize);


  






 
  void arm_fir_q15(
		   const arm_fir_instance_q15 * S,
		    q15_t * pSrc,
		   q15_t * pDst,
		   uint32_t blockSize);

  






 
  void arm_fir_fast_q15(
			const arm_fir_instance_q15 * S,
			 q15_t * pSrc,
			q15_t * pDst,
			uint32_t blockSize);

  








 
   
       arm_status arm_fir_init_q15(
			      arm_fir_instance_q15 * S,
			      uint16_t numTaps,
			      q15_t * pCoeffs,
			      q15_t * pState,
			      uint32_t blockSize);

  






 
  void arm_fir_q31(
		   const arm_fir_instance_q31 * S,
		    q31_t * pSrc,
		   q31_t * pDst,
		   uint32_t blockSize);

  






 
  void arm_fir_fast_q31(
			const arm_fir_instance_q31 * S,
			 q31_t * pSrc,
			q31_t * pDst,
			uint32_t blockSize);

  







 
  void arm_fir_init_q31(
			arm_fir_instance_q31 * S,
			uint16_t numTaps,
			q31_t * pCoeffs,
			q31_t * pState,
			uint32_t blockSize);

  






 
  void arm_fir_f32(
		   const arm_fir_instance_f32 * S,
		    float32_t * pSrc,
		   float32_t * pDst,
		   uint32_t blockSize);

  







 
  void arm_fir_init_f32(
			arm_fir_instance_f32 * S,
			uint16_t numTaps,
			float32_t * pCoeffs,
			float32_t * pState,
			uint32_t blockSize);


  

 
  typedef struct
  {
    int8_t numStages;          
    q15_t *pState;             
    q15_t *pCoeffs;            
    int8_t postShift;          

  } arm_biquad_casd_df1_inst_q15;


  

 
  typedef struct
  {
    uint32_t numStages;       
    q31_t *pState;            
    q31_t *pCoeffs;           
    uint8_t postShift;        

  } arm_biquad_casd_df1_inst_q31;

  

 
  typedef struct
  {
    uint32_t numStages;          
    float32_t *pState;           
    float32_t *pCoeffs;          


  } arm_biquad_casd_df1_inst_f32;



  






 

  void arm_biquad_cascade_df1_q15(
				  const arm_biquad_casd_df1_inst_q15 * S,
				   q15_t * pSrc,
				  q15_t * pDst,
				  uint32_t blockSize);

  







 

  void arm_biquad_cascade_df1_init_q15(
				       arm_biquad_casd_df1_inst_q15 * S,
				       uint8_t numStages,
				       q15_t * pCoeffs,
				       q15_t * pState,
				       int8_t postShift);


  






 

  void arm_biquad_cascade_df1_fast_q15(
				       const arm_biquad_casd_df1_inst_q15 * S,
				        q15_t * pSrc,
				       q15_t * pDst,
				       uint32_t blockSize);


  






 

  void arm_biquad_cascade_df1_q31(
				  const arm_biquad_casd_df1_inst_q31 * S,
				   q31_t * pSrc,
				  q31_t * pDst,
				  uint32_t blockSize);

  






 

  void arm_biquad_cascade_df1_fast_q31(
				       const arm_biquad_casd_df1_inst_q31 * S,
				        q31_t * pSrc,
				       q31_t * pDst,
				       uint32_t blockSize);

  







 

  void arm_biquad_cascade_df1_init_q31(
				       arm_biquad_casd_df1_inst_q31 * S,
				       uint8_t numStages,
				       q31_t * pCoeffs,
				       q31_t * pState,
				       int8_t postShift);

  






 

  void arm_biquad_cascade_df1_f32(
				  const arm_biquad_casd_df1_inst_f32 * S,
				   float32_t * pSrc,
				  float32_t * pDst,
				  uint32_t blockSize);

  






 

  void arm_biquad_cascade_df1_init_f32(
				       arm_biquad_casd_df1_inst_f32 * S,
				       uint8_t numStages,
				       float32_t * pCoeffs,
				       float32_t * pState);


  

 

  typedef struct
  {
    uint16_t numRows;      
    uint16_t numCols;      
    float32_t *pData;      
  } arm_matrix_instance_f32;

  

 

  typedef struct
  {
    uint16_t numRows;      
    uint16_t numCols;      
    q15_t *pData;          

  } arm_matrix_instance_q15;

  

 

  typedef struct
  {
    uint16_t numRows;      
    uint16_t numCols;      
    q31_t *pData;          

  } arm_matrix_instance_q31;



  






 

  arm_status arm_mat_add_f32(
			     const arm_matrix_instance_f32 * pSrcA,
			     const arm_matrix_instance_f32 * pSrcB,
			     arm_matrix_instance_f32 * pDst);

  






 

  arm_status arm_mat_add_q15(
			     const arm_matrix_instance_q15 * pSrcA,
			     const arm_matrix_instance_q15 * pSrcB,
			     arm_matrix_instance_q15 * pDst);

  






 

  arm_status arm_mat_add_q31(
			     const arm_matrix_instance_q31 * pSrcA,
			     const arm_matrix_instance_q31 * pSrcB,
			     arm_matrix_instance_q31 * pDst);


  





 

  arm_status arm_mat_trans_f32(
			       const arm_matrix_instance_f32 * pSrc,
			       arm_matrix_instance_f32 * pDst);


  





 

  arm_status arm_mat_trans_q15(
			       const arm_matrix_instance_q15 * pSrc,
			       arm_matrix_instance_q15 * pDst);

  





 

  arm_status arm_mat_trans_q31(
			       const arm_matrix_instance_q31 * pSrc,
			       arm_matrix_instance_q31 * pDst);


  






 

  arm_status arm_mat_mult_f32(
			      const arm_matrix_instance_f32 * pSrcA,
			      const arm_matrix_instance_f32 * pSrcB,
			      arm_matrix_instance_f32 * pDst);

  






 

  arm_status arm_mat_mult_q15(
			      const arm_matrix_instance_q15 * pSrcA,
			      const arm_matrix_instance_q15 * pSrcB,
			      arm_matrix_instance_q15 * pDst,
			      q15_t * pState);

  







 

  arm_status arm_mat_mult_fast_q15(
				   const arm_matrix_instance_q15 * pSrcA,
				   const arm_matrix_instance_q15 * pSrcB,
				   arm_matrix_instance_q15 * pDst,
				   q15_t * pState);

  






 

  arm_status arm_mat_mult_q31(
			      const arm_matrix_instance_q31 * pSrcA,
			      const arm_matrix_instance_q31 * pSrcB,
			      arm_matrix_instance_q31 * pDst);

  






 

  arm_status arm_mat_mult_fast_q31(
				   const arm_matrix_instance_q31 * pSrcA,
				   const arm_matrix_instance_q31 * pSrcB,
				   arm_matrix_instance_q31 * pDst);


  






 

  arm_status arm_mat_sub_f32(
			     const arm_matrix_instance_f32 * pSrcA,
			     const arm_matrix_instance_f32 * pSrcB,
			     arm_matrix_instance_f32 * pDst);

  






 

  arm_status arm_mat_sub_q15(
			     const arm_matrix_instance_q15 * pSrcA,
			     const arm_matrix_instance_q15 * pSrcB,
			     arm_matrix_instance_q15 * pDst);

  






 

  arm_status arm_mat_sub_q31(
			     const arm_matrix_instance_q31 * pSrcA,
			     const arm_matrix_instance_q31 * pSrcB,
			     arm_matrix_instance_q31 * pDst);

  






 

  arm_status arm_mat_scale_f32(
			       const arm_matrix_instance_f32 * pSrc,
			       float32_t scale,
			       arm_matrix_instance_f32 * pDst);

  







 

  arm_status arm_mat_scale_q15(
			       const arm_matrix_instance_q15 * pSrc,
			       q15_t scaleFract,
			       int32_t shift,
			       arm_matrix_instance_q15 * pDst);

  







 

  arm_status arm_mat_scale_q31(
			       const arm_matrix_instance_q31 * pSrc,
			       q31_t scaleFract,
			       int32_t shift,
			       arm_matrix_instance_q31 * pDst);


  






 

  void arm_mat_init_q31(
			arm_matrix_instance_q31 * S,
			uint16_t nRows,
			uint16_t nColumns,
			q31_t   *pData);

  






 

  void arm_mat_init_q15(
			arm_matrix_instance_q15 * S,
			uint16_t nRows,
			uint16_t nColumns,
			q15_t    *pData);

  






 

  void arm_mat_init_f32(
			arm_matrix_instance_f32 * S,
			uint16_t nRows,
			uint16_t nColumns,
			float32_t   *pData);



  

 
  typedef struct
  {
    q15_t A0; 	  




    q31_t A1;            

    q15_t state[3];        
    q15_t Kp;            
    q15_t Ki;            
    q15_t Kd;            
  } arm_pid_instance_q15;

  

 
  typedef struct
  {
    q31_t A0;             
    q31_t A1;             
    q31_t A2;             
    q31_t state[3];       
    q31_t Kp;             
    q31_t Ki;             
    q31_t Kd;             

  } arm_pid_instance_q31;

  

 
  typedef struct
  {
    float32_t A0;           
    float32_t A1;           
    float32_t A2;           
    float32_t state[3];     
    float32_t Kp;                
    float32_t Ki;                
    float32_t Kd;                
  } arm_pid_instance_f32;



  




 
  void arm_pid_init_f32(
			arm_pid_instance_f32 * S,
			int32_t resetStateFlag);

  



 
  void arm_pid_reset_f32(
			 arm_pid_instance_f32 * S);


  




 
  void arm_pid_init_q31(
			arm_pid_instance_q31 * S,
			int32_t resetStateFlag);

 
  



 

  void arm_pid_reset_q31(
			 arm_pid_instance_q31 * S);

  




 
  void arm_pid_init_q15(
			arm_pid_instance_q15 * S,
			int32_t resetStateFlag);

  



 
  void arm_pid_reset_q15(
			 arm_pid_instance_q15 * S);


  

 
  typedef struct
  {
    uint32_t nValues;
    float32_t x1;
    float32_t xSpacing;
    float32_t *pYData;           
  } arm_linear_interp_instance_f32;

  

 

  typedef struct
  {
    uint16_t numRows;	 
    uint16_t numCols;	 
    float32_t *pData;	 
  } arm_bilinear_interp_instance_f32;

   

 

  typedef struct
  {
    uint16_t numRows;	 
    uint16_t numCols;	 
    q31_t *pData;	 
  } arm_bilinear_interp_instance_q31;

   

 

  typedef struct
  {
    uint16_t numRows;	 
    uint16_t numCols;	 
    q15_t *pData;	 
  } arm_bilinear_interp_instance_q15;

   

 

  typedef struct
  {
    uint16_t numRows; 	 
    uint16_t numCols;	 
    q7_t *pData;		 
  } arm_bilinear_interp_instance_q7;


  






 

  void arm_mult_q7(
		    q7_t * pSrcA,
		    q7_t * pSrcB,
		   q7_t * pDst,
		   uint32_t blockSize);

  






 

  void arm_mult_q15(
		     q15_t * pSrcA,
		     q15_t * pSrcB,
		    q15_t * pDst,
		    uint32_t blockSize);

  






 

  void arm_mult_q31(
		     q31_t * pSrcA,
		     q31_t * pSrcB,
		    q31_t * pDst,
		    uint32_t blockSize);

  






 

  void arm_mult_f32(
		     float32_t * pSrcA,
		     float32_t * pSrcB,
		    float32_t * pDst,
		    uint32_t blockSize);


  

 

  typedef struct
  {
    uint16_t  fftLen;                 
    uint8_t   ifftFlag;               
    uint8_t   bitReverseFlag;         
    q15_t     *pTwiddle;              
    uint16_t  *pBitRevTable;          
    uint16_t  twidCoefModifier;       
    uint16_t  bitRevFactor;           
  } arm_cfft_radix4_instance_q15;

  

 

  typedef struct
  {
    uint16_t    fftLen;               
    uint8_t     ifftFlag;             
    uint8_t     bitReverseFlag;       
    q31_t       *pTwiddle;            
    uint16_t    *pBitRevTable;        
    uint16_t    twidCoefModifier;     
    uint16_t    bitRevFactor;         
  } arm_cfft_radix4_instance_q31;

  

 

  typedef struct
  {
    uint16_t     fftLen;                
    uint8_t      ifftFlag;              
    uint8_t      bitReverseFlag;        
    float32_t    *pTwiddle;             
    uint16_t     *pBitRevTable;         
    uint16_t     twidCoefModifier;      
    uint16_t     bitRevFactor;          
	float32_t    onebyfftLen;           
  } arm_cfft_radix4_instance_f32;

  




 

  void arm_cfft_radix4_q15(
			   const arm_cfft_radix4_instance_q15 * S,
			   q15_t * pSrc);

  






 

  arm_status arm_cfft_radix4_init_q15(
				      arm_cfft_radix4_instance_q15 * S,
				      uint16_t fftLen,
				      uint8_t ifftFlag,
				      uint8_t bitReverseFlag);

  




 

  void arm_cfft_radix4_q31(
			   const arm_cfft_radix4_instance_q31 * S,
			   q31_t * pSrc);

  






 
  
  arm_status arm_cfft_radix4_init_q31(
				      arm_cfft_radix4_instance_q31 * S,
				      uint16_t fftLen,
				      uint8_t ifftFlag,
				      uint8_t bitReverseFlag);

  




 

  void arm_cfft_radix4_f32(
			   const arm_cfft_radix4_instance_f32 * S,
			   float32_t * pSrc);

  






 
  
  arm_status arm_cfft_radix4_init_f32(
				      arm_cfft_radix4_instance_f32 * S,
				      uint16_t fftLen,
				      uint8_t ifftFlag,
				      uint8_t bitReverseFlag);



  

 

  






 
  
  void arm_radix4_butterfly_f32(
				float32_t * pSrc,
				uint16_t fftLen,
				float32_t * pCoef,
				uint16_t twidCoefModifier);

  







 
  
  void arm_radix4_butterfly_inverse_f32(
					float32_t * pSrc,
					uint16_t fftLen,
					float32_t * pCoef,
					uint16_t twidCoefModifier,
					float32_t onebyfftLen);

  






 

  void arm_bitreversal_f32(
			   float32_t *pSrc,
			   uint16_t fftSize,
			   uint16_t bitRevFactor,
			   uint16_t *pBitRevTab);

  






 
  
  void arm_radix4_butterfly_q31(
				q31_t *pSrc,
				uint32_t fftLen,
				q31_t *pCoef,
				uint32_t twidCoefModifier);

  






 
  
  void arm_radix4_butterfly_inverse_q31(
					q31_t * pSrc,
					uint32_t fftLen,
					q31_t * pCoef,
					uint32_t twidCoefModifier);
  
  






 

  void arm_bitreversal_q31(
			   q31_t * pSrc,
			   uint32_t fftLen,
			   uint16_t bitRevFactor,
			   uint16_t *pBitRevTab);

  






 

  void arm_radix4_butterfly_q15(
				q15_t *pSrc16,
				uint32_t fftLen,
				q15_t *pCoef16,
				uint32_t twidCoefModifier);

  






 

  void arm_radix4_butterfly_inverse_q15(
					q15_t *pSrc16,
					uint32_t fftLen,
					q15_t *pCoef16,
					uint32_t twidCoefModifier);

  






 

  void arm_bitreversal_q15(
			   q15_t * pSrc,
			   uint32_t fftLen,
			   uint16_t bitRevFactor,
			   uint16_t *pBitRevTab);

  

 

  typedef struct
  {
    uint32_t fftLenReal;                       
    uint32_t fftLenBy2;                        
    uint8_t  ifftFlagR;                        
	uint8_t  bitReverseFlagR;                  
    uint32_t twidCoefRModifier;                  
    q15_t    *pTwiddleAReal;                   
    q15_t    *pTwiddleBReal;                   
    arm_cfft_radix4_instance_q15 *pCfft;	   
  } arm_rfft_instance_q15;

  

 

  typedef struct
  {
    uint32_t fftLenReal;                         
    uint32_t fftLenBy2;                          
    uint8_t  ifftFlagR;                          
	uint8_t  bitReverseFlagR;                    
    uint32_t twidCoefRModifier;                  
    q31_t    *pTwiddleAReal;                     
    q31_t    *pTwiddleBReal;                     
    arm_cfft_radix4_instance_q31 *pCfft;         
  } arm_rfft_instance_q31;

  

 

  typedef struct
  {
    uint32_t  fftLenReal;                        
    uint16_t  fftLenBy2;                         
    uint8_t   ifftFlagR;                         
    uint8_t   bitReverseFlagR;                   
	uint32_t  twidCoefRModifier;                 
    float32_t *pTwiddleAReal;                    
    float32_t *pTwiddleBReal;                    
    arm_cfft_radix4_instance_f32 *pCfft;         
  } arm_rfft_instance_f32;

  





 

  void arm_rfft_q15(
		    const arm_rfft_instance_q15 * S,
		    q15_t * pSrc,
		    q15_t * pDst);

  







 

  arm_status arm_rfft_init_q15(
			       arm_rfft_instance_q15 * S,
			       arm_cfft_radix4_instance_q15 * S_CFFT,
			       uint32_t fftLenReal,
			       uint32_t ifftFlagR,
			       uint32_t bitReverseFlag);

  





 

  void arm_rfft_q31(
		    const arm_rfft_instance_q31 * S,
		    q31_t * pSrc,
		    q31_t * pDst);

  







 

  arm_status arm_rfft_init_q31(
			       arm_rfft_instance_q31 * S,
			       arm_cfft_radix4_instance_q31 * S_CFFT,
			       uint32_t fftLenReal,
			       uint32_t ifftFlagR,
			       uint32_t bitReverseFlag);

  







 

  arm_status arm_rfft_init_f32(
			       arm_rfft_instance_f32 * S,
			       arm_cfft_radix4_instance_f32 * S_CFFT,
			       uint32_t fftLenReal,
			       uint32_t ifftFlagR,
			       uint32_t bitReverseFlag);

  





 

  void arm_rfft_f32(
		    const arm_rfft_instance_f32 * S,
		    float32_t * pSrc,
		    float32_t * pDst);

  

 

  typedef struct
  {
    uint16_t N;                          
    uint16_t Nby2;                       
    float32_t normalize;                 
    float32_t *pTwiddle;                 
    float32_t *pCosFactor;               
    arm_rfft_instance_f32 *pRfft;         
    arm_cfft_radix4_instance_f32 *pCfft;  
  } arm_dct4_instance_f32;

  








 

  arm_status arm_dct4_init_f32(
			       arm_dct4_instance_f32 * S,
			       arm_rfft_instance_f32 * S_RFFT,
			       arm_cfft_radix4_instance_f32 * S_CFFT,
			       uint16_t N,
			       uint16_t Nby2,
			       float32_t normalize);

  





 

  void arm_dct4_f32(
		    const arm_dct4_instance_f32 * S,
		    float32_t * pState,
		    float32_t * pInlineBuffer);

  

 

  typedef struct
  {
    uint16_t N;                          
    uint16_t Nby2;                       
    q31_t normalize;                     
    q31_t *pTwiddle;                     
    q31_t *pCosFactor;                   
    arm_rfft_instance_q31 *pRfft;         
    arm_cfft_radix4_instance_q31 *pCfft;  
  } arm_dct4_instance_q31;

  








 

  arm_status arm_dct4_init_q31(
			       arm_dct4_instance_q31 * S,
			       arm_rfft_instance_q31 * S_RFFT,
			       arm_cfft_radix4_instance_q31 * S_CFFT,
			       uint16_t N,
			       uint16_t Nby2,
			       q31_t normalize);

  





 

  void arm_dct4_q31(
		    const arm_dct4_instance_q31 * S,
		    q31_t * pState,
		    q31_t * pInlineBuffer);

  

 

  typedef struct
  {
    uint16_t N;                          
    uint16_t Nby2;                       
    q15_t normalize;                     
    q15_t *pTwiddle;                     
    q15_t *pCosFactor;                   
    arm_rfft_instance_q15 *pRfft;         
    arm_cfft_radix4_instance_q15 *pCfft;  
  } arm_dct4_instance_q15;

  








 

  arm_status arm_dct4_init_q15(
			       arm_dct4_instance_q15 * S,
			       arm_rfft_instance_q15 * S_RFFT,
			       arm_cfft_radix4_instance_q15 * S_CFFT,
			       uint16_t N,
			       uint16_t Nby2,
			       q15_t normalize);

  





 

  void arm_dct4_q15(
		    const arm_dct4_instance_q15 * S,
		    q15_t * pState,
		    q15_t * pInlineBuffer);

  






 

  void arm_add_f32(
		   float32_t * pSrcA,
		   float32_t * pSrcB,
		   float32_t * pDst,
		   uint32_t blockSize);

  






 

  void arm_add_q7(
		  q7_t * pSrcA,
		  q7_t * pSrcB,
		  q7_t * pDst,
		  uint32_t blockSize);

  






 

  void arm_add_q15(
		    q15_t * pSrcA,
		    q15_t * pSrcB,
		   q15_t * pDst,
		   uint32_t blockSize);

  






 

  void arm_add_q31(
		    q31_t * pSrcA,
		    q31_t * pSrcB,
		   q31_t * pDst,
		   uint32_t blockSize);

  






 

  void arm_sub_f32(
		    float32_t * pSrcA,
		    float32_t * pSrcB,
		   float32_t * pDst,
		   uint32_t blockSize);

  






 

  void arm_sub_q7(
		   q7_t * pSrcA,
		   q7_t * pSrcB,
		  q7_t * pDst,
		  uint32_t blockSize);

  






 

  void arm_sub_q15(
		    q15_t * pSrcA,
		    q15_t * pSrcB,
		   q15_t * pDst,
		   uint32_t blockSize);

  






 

  void arm_sub_q31(
		    q31_t * pSrcA,
		    q31_t * pSrcB,
		   q31_t * pDst,
		   uint32_t blockSize);

  






 

  void arm_scale_f32(
		      float32_t * pSrc,
		     float32_t scale,
		     float32_t * pDst,
		     uint32_t blockSize);

  







 

  void arm_scale_q7(
		     q7_t * pSrc,
		    q7_t scaleFract,
		    int8_t shift,
		    q7_t * pDst,
		    uint32_t blockSize);

  







 

  void arm_scale_q15(
		      q15_t * pSrc,
		     q15_t scaleFract,
		     int8_t shift,
		     q15_t * pDst,
		     uint32_t blockSize);

  







 

  void arm_scale_q31(
		      q31_t * pSrc,
		     q31_t scaleFract,
		     int8_t shift,
		     q31_t * pDst,
		     uint32_t blockSize);

  





 

  void arm_abs_q7(
		   q7_t * pSrc,
		  q7_t * pDst,
		  uint32_t blockSize);

  





 

  void arm_abs_f32(
		    float32_t * pSrc,
		   float32_t * pDst,
		   uint32_t blockSize);

  





 

  void arm_abs_q15(
		    q15_t * pSrc,
		   q15_t * pDst,
		   uint32_t blockSize);

  





 

  void arm_abs_q31(
		    q31_t * pSrc,
		   q31_t * pDst,
		   uint32_t blockSize);

  






 

  void arm_dot_prod_f32(
			 float32_t * pSrcA,
			 float32_t * pSrcB,
			uint32_t blockSize,
			float32_t * result);

  






 

  void arm_dot_prod_q7(
		        q7_t * pSrcA,
		        q7_t * pSrcB,
		       uint32_t blockSize,
		       q31_t * result);

  






 

  void arm_dot_prod_q15(
			 q15_t * pSrcA,
			 q15_t * pSrcB,
			uint32_t blockSize,
			q63_t * result);

  






 

  void arm_dot_prod_q31(
			 q31_t * pSrcA,
			 q31_t * pSrcB,
			uint32_t blockSize,
			q63_t * result);

  






 

  void arm_shift_q7(
		     q7_t * pSrc,
		    int8_t shiftBits,
		    q7_t * pDst,
		    uint32_t blockSize);

  






 

  void arm_shift_q15(
		      q15_t * pSrc,
		     int8_t shiftBits,
		     q15_t * pDst,
		     uint32_t blockSize);

  






 

  void arm_shift_q31(
		      q31_t * pSrc,
		     int8_t shiftBits,
		     q31_t * pDst,
		     uint32_t blockSize);

  






 

  void arm_offset_f32(
		       float32_t * pSrc,
		      float32_t offset,
		      float32_t * pDst,
		      uint32_t blockSize);

  






 

  void arm_offset_q7(
		      q7_t * pSrc,
		     q7_t offset,
		     q7_t * pDst,
		     uint32_t blockSize);

  






 

  void arm_offset_q15(
		       q15_t * pSrc,
		      q15_t offset,
		      q15_t * pDst,
		      uint32_t blockSize);

  






 

  void arm_offset_q31(
		       q31_t * pSrc,
		      q31_t offset,
		      q31_t * pDst,
		      uint32_t blockSize);

  





 

  void arm_negate_f32(
		       float32_t * pSrc,
		      float32_t * pDst,
		      uint32_t blockSize);

  





 

  void arm_negate_q7(
		      q7_t * pSrc,
		     q7_t * pDst,
		     uint32_t blockSize);

  





 

  void arm_negate_q15(
		       q15_t * pSrc,
		      q15_t * pDst,
		      uint32_t blockSize);

  





 

  void arm_negate_q31(
		       q31_t * pSrc,
		      q31_t * pDst,
		      uint32_t blockSize);
  





 
  void arm_copy_f32(
		     float32_t * pSrc,
		    float32_t * pDst,
		    uint32_t blockSize);

  





 
  void arm_copy_q7(
		    q7_t * pSrc,
		   q7_t * pDst,
		   uint32_t blockSize);

  





 
  void arm_copy_q15(
		     q15_t * pSrc,
		    q15_t * pDst,
		    uint32_t blockSize);

  





 
  void arm_copy_q31(
		     q31_t * pSrc,
		    q31_t * pDst,
		    uint32_t blockSize);
  





 
  void arm_fill_f32(
		     float32_t value,
		    float32_t * pDst,
		    uint32_t blockSize);

  





 
  void arm_fill_q7(
		    q7_t value,
		   q7_t * pDst,
		   uint32_t blockSize);

  





 
  void arm_fill_q15(
		     q15_t value,
		    q15_t * pDst,
		    uint32_t blockSize);

  





 
  void arm_fill_q31(
		     q31_t value,
		    q31_t * pDst,
		    uint32_t blockSize);









  

  void arm_conv_f32(
		     float32_t * pSrcA,
		    uint32_t srcALen,
		     float32_t * pSrcB,
		    uint32_t srcBLen,
		    float32_t * pDst);









 

  void arm_conv_q15(
		     q15_t * pSrcA,
		    uint32_t srcALen,
		     q15_t * pSrcB,
		    uint32_t srcBLen,
		    q15_t * pDst);

  







 

  void arm_conv_fast_q15(
			  q15_t * pSrcA,
			 uint32_t srcALen,
			  q15_t * pSrcB,
			 uint32_t srcBLen,
			 q15_t * pDst);

  







 

  void arm_conv_q31(
		     q31_t * pSrcA,
		    uint32_t srcALen,
		     q31_t * pSrcB,
		    uint32_t srcBLen,
		    q31_t * pDst);

  







 

  void arm_conv_fast_q31(
			  q31_t * pSrcA,
			 uint32_t srcALen,
			  q31_t * pSrcB,
			 uint32_t srcBLen,
			 q31_t * pDst);

  







 

  void arm_conv_q7(
		    q7_t * pSrcA,
		   uint32_t srcALen,
		    q7_t * pSrcB,
		   uint32_t srcBLen,
		   q7_t * pDst);

  









 

  arm_status arm_conv_partial_f32(
				   float32_t * pSrcA,
				  uint32_t srcALen,
				   float32_t * pSrcB,
				  uint32_t srcBLen,
				  float32_t * pDst,
				  uint32_t firstIndex,
				  uint32_t numPoints);

  









 

  arm_status arm_conv_partial_q15(
				   q15_t * pSrcA,
				  uint32_t srcALen,
				   q15_t * pSrcB,
				  uint32_t srcBLen,
				  q15_t * pDst,
				  uint32_t firstIndex,
				  uint32_t numPoints);

  









 

  arm_status arm_conv_partial_fast_q15(
				        q15_t * pSrcA,
				       uint32_t srcALen,
				        q15_t * pSrcB,
				       uint32_t srcBLen,
				       q15_t * pDst,
				       uint32_t firstIndex,
				       uint32_t numPoints);

  









 

  arm_status arm_conv_partial_q31(
				   q31_t * pSrcA,
				  uint32_t srcALen,
				   q31_t * pSrcB,
				  uint32_t srcBLen,
				  q31_t * pDst,
				  uint32_t firstIndex,
				  uint32_t numPoints);


  









 

  arm_status arm_conv_partial_fast_q31(
				        q31_t * pSrcA,
				       uint32_t srcALen,
				        q31_t * pSrcB,
				       uint32_t srcBLen,
				       q31_t * pDst,
				       uint32_t firstIndex,
				       uint32_t numPoints);

  









 

  arm_status arm_conv_partial_q7(
				  q7_t * pSrcA,
				 uint32_t srcALen,
				  q7_t * pSrcB,
				 uint32_t srcBLen,
				 q7_t * pDst,
				 uint32_t firstIndex,
				 uint32_t numPoints);


  

 

  typedef struct
  {
    uint8_t M;                       
    uint16_t numTaps;                
    q15_t *pCoeffs;                   
    q15_t *pState;                    
  } arm_fir_decimate_instance_q15;

  

 

  typedef struct
  {
    uint8_t M;                   
    uint16_t numTaps;            
    q31_t *pCoeffs;               
    q31_t *pState;                

  } arm_fir_decimate_instance_q31;

  

 

  typedef struct
  {
    uint8_t M;                           
    uint16_t numTaps;                    
    float32_t *pCoeffs;                   
    float32_t *pState;                    

  } arm_fir_decimate_instance_f32;



  






 

  void arm_fir_decimate_f32(
			    const arm_fir_decimate_instance_f32 * S,
			     float32_t * pSrc,
			    float32_t * pDst,
			    uint32_t blockSize);


  









 

  arm_status arm_fir_decimate_init_f32(
				       arm_fir_decimate_instance_f32 * S,
				       uint16_t numTaps,
				       uint8_t M,
				       float32_t * pCoeffs,
				       float32_t * pState,
				       uint32_t blockSize);

  






 

  void arm_fir_decimate_q15(
			    const arm_fir_decimate_instance_q15 * S,
			     q15_t * pSrc,
			    q15_t * pDst,
			    uint32_t blockSize);

  






 

  void arm_fir_decimate_fast_q15(
				 const arm_fir_decimate_instance_q15 * S,
				  q15_t * pSrc,
				 q15_t * pDst,
				 uint32_t blockSize);



  









 

  arm_status arm_fir_decimate_init_q15(
				       arm_fir_decimate_instance_q15 * S,
				       uint16_t numTaps,
				       uint8_t M,
				       q15_t * pCoeffs,
				       q15_t * pState,
				       uint32_t blockSize);

  






 

  void arm_fir_decimate_q31(
			    const arm_fir_decimate_instance_q31 * S,
			     q31_t * pSrc,
			    q31_t * pDst,
			    uint32_t blockSize);

  






 

  void arm_fir_decimate_fast_q31(
				 arm_fir_decimate_instance_q31 * S,
				  q31_t * pSrc,
				 q31_t * pDst,
				 uint32_t blockSize);


  









 

  arm_status arm_fir_decimate_init_q31(
				       arm_fir_decimate_instance_q31 * S,
				       uint16_t numTaps,
				       uint8_t M,
				       q31_t * pCoeffs,
				       q31_t * pState,
				       uint32_t blockSize);



  

 

  typedef struct
  {
    uint8_t L;                       
    uint16_t phaseLength;            
    q15_t *pCoeffs;                  
    q15_t *pState;                   
  } arm_fir_interpolate_instance_q15;

  

 

  typedef struct
  {
    uint8_t L;                       
    uint16_t phaseLength;            
    q31_t *pCoeffs;                   
    q31_t *pState;                    
  } arm_fir_interpolate_instance_q31;

  

 

  typedef struct
  {
    uint8_t L;                      
    uint16_t phaseLength;           
    float32_t *pCoeffs;              
    float32_t *pState;               
  } arm_fir_interpolate_instance_f32;


  






 

  void arm_fir_interpolate_q15(
			       const arm_fir_interpolate_instance_q15 * S,
			        q15_t * pSrc,
			       q15_t * pDst,
			       uint32_t blockSize);


  









 

  arm_status arm_fir_interpolate_init_q15(
					  arm_fir_interpolate_instance_q15 * S,
					  uint8_t L,
					  uint16_t numTaps,
					  q15_t * pCoeffs,
					  q15_t * pState,
					  uint32_t blockSize);

  






 

  void arm_fir_interpolate_q31(
			       const arm_fir_interpolate_instance_q31 * S,
			        q31_t * pSrc,
			       q31_t * pDst,
			       uint32_t blockSize);

  









 

  arm_status arm_fir_interpolate_init_q31(
					  arm_fir_interpolate_instance_q31 * S,
					  uint8_t L,
					  uint16_t numTaps,
					  q31_t * pCoeffs,
					  q31_t * pState,
					  uint32_t blockSize);


  






 

  void arm_fir_interpolate_f32(
			       const arm_fir_interpolate_instance_f32 * S,
			        float32_t * pSrc,
			       float32_t * pDst,
			       uint32_t blockSize);

  









 

  arm_status arm_fir_interpolate_init_f32(
					  arm_fir_interpolate_instance_f32 * S,
					  uint8_t L,
					  uint16_t numTaps,
					  float32_t * pCoeffs,
					  float32_t * pState,
					  uint32_t blockSize);

  

 

  typedef struct
  {
    uint8_t numStages;        
    q63_t *pState;            
    q31_t *pCoeffs;           
    uint8_t postShift;        

  } arm_biquad_cas_df1_32x64_ins_q31;


  





 

  void arm_biquad_cas_df1_32x64_q31(
				    const arm_biquad_cas_df1_32x64_ins_q31 * S,
				     q31_t * pSrc,
				    q31_t * pDst,
				    uint32_t blockSize);


  






 

  void arm_biquad_cas_df1_32x64_init_q31(
					 arm_biquad_cas_df1_32x64_ins_q31 * S,
					 uint8_t numStages,
					 q31_t * pCoeffs,
					 q63_t * pState,
					 uint8_t postShift);



  

 

  typedef struct
  {
    uint8_t   numStages;        
    float32_t *pState;          
    float32_t *pCoeffs;         
  } arm_biquad_cascade_df2T_instance_f32;


  






 

  void arm_biquad_cascade_df2T_f32(
				   const arm_biquad_cascade_df2T_instance_f32 * S,
				    float32_t * pSrc,
				   float32_t * pDst,
				   uint32_t blockSize);


  






 

  void arm_biquad_cascade_df2T_init_f32(
					arm_biquad_cascade_df2T_instance_f32 * S,
					uint8_t numStages,
					float32_t * pCoeffs,
					float32_t * pState);



  

 

  typedef struct
  {
    uint16_t numStages;                           
    q15_t *pState;                                
    q15_t *pCoeffs;                               
  } arm_fir_lattice_instance_q15;

  

 

  typedef struct
  {
    uint16_t numStages;                           
    q31_t *pState;                                
    q31_t *pCoeffs;                               
  } arm_fir_lattice_instance_q31;

  

 

  typedef struct
  {
    uint16_t numStages;                   
    float32_t *pState;                    
    float32_t *pCoeffs;                   
  } arm_fir_lattice_instance_f32;

  






 

  void arm_fir_lattice_init_q15(
				arm_fir_lattice_instance_q15 * S,
				uint16_t numStages,
				q15_t * pCoeffs,
				q15_t * pState);


  






 
  void arm_fir_lattice_q15(
			   const arm_fir_lattice_instance_q15 * S,
			    q15_t * pSrc,
			   q15_t * pDst,
			   uint32_t blockSize);

  






 

  void arm_fir_lattice_init_q31(
				arm_fir_lattice_instance_q31 * S,
				uint16_t numStages,
				q31_t * pCoeffs,
				q31_t * pState);


  






 

  void arm_fir_lattice_q31(
			   const arm_fir_lattice_instance_q31 * S,
			    q31_t * pSrc,
			   q31_t * pDst,
			   uint32_t blockSize);








 

  void arm_fir_lattice_init_f32(
				arm_fir_lattice_instance_f32 * S,
				uint16_t numStages,
				float32_t * pCoeffs,
				float32_t * pState);

  






 

  void arm_fir_lattice_f32(
			   const arm_fir_lattice_instance_f32 * S,
			    float32_t * pSrc,
			   float32_t * pDst,
			   uint32_t blockSize);

  

 
  typedef struct
  {
    uint16_t numStages;                          
    q15_t *pState;                               
    q15_t *pkCoeffs;                             
    q15_t *pvCoeffs;                             
  } arm_iir_lattice_instance_q15;

  

 
  typedef struct
  {
    uint16_t numStages;                          
    q31_t *pState;                               
    q31_t *pkCoeffs;                             
    q31_t *pvCoeffs;                             
  } arm_iir_lattice_instance_q31;

  

 
  typedef struct
  {
    uint16_t numStages;                          
    float32_t *pState;                           
    float32_t *pkCoeffs;                         
    float32_t *pvCoeffs;                         
  } arm_iir_lattice_instance_f32;

  






 

  void arm_iir_lattice_f32(
			   const arm_iir_lattice_instance_f32 * S,
			    float32_t * pSrc,
			   float32_t * pDst,
			   uint32_t blockSize);

  








 

  void arm_iir_lattice_init_f32(
				arm_iir_lattice_instance_f32 * S,
				uint16_t numStages,
				float32_t *pkCoeffs,
				float32_t *pvCoeffs,
				float32_t *pState,
				uint32_t blockSize);


  






 

  void arm_iir_lattice_q31(
			   const arm_iir_lattice_instance_q31 * S,
			    q31_t * pSrc,
			   q31_t * pDst,
			   uint32_t blockSize);


  








 

  void arm_iir_lattice_init_q31(
				arm_iir_lattice_instance_q31 * S,
				uint16_t numStages,
				q31_t *pkCoeffs,
				q31_t *pvCoeffs,
				q31_t *pState,
				uint32_t blockSize);


  






 

  void arm_iir_lattice_q15(
			   const arm_iir_lattice_instance_q15 * S,
			    q15_t * pSrc,
			   q15_t * pDst,
			   uint32_t blockSize);











 

  void arm_iir_lattice_init_q15(
				arm_iir_lattice_instance_q15 * S,
				uint16_t numStages,
				q15_t *pkCoeffs,
				q15_t *pvCoeffs,
				q15_t *pState,
				uint32_t blockSize);

  

 

  typedef struct
  {
    uint16_t numTaps;     
    float32_t *pState;    
    float32_t *pCoeffs;   
    float32_t mu;         
  } arm_lms_instance_f32;

  








 

  void arm_lms_f32(
		   const arm_lms_instance_f32 * S,
		    float32_t * pSrc,
		    float32_t * pRef,
		   float32_t * pOut,
		   float32_t * pErr,
		   uint32_t blockSize);

  








 

  void arm_lms_init_f32(
			arm_lms_instance_f32 * S,
			uint16_t numTaps,
			float32_t * pCoeffs,
			float32_t * pState,
			float32_t mu,
			uint32_t blockSize);

  

 

  typedef struct
  {
    uint16_t numTaps;     
    q15_t *pState;        
    q15_t *pCoeffs;       
    q15_t mu;             
    uint32_t postShift;   
  } arm_lms_instance_q15;


  









 

  void arm_lms_init_q15(
			arm_lms_instance_q15 * S,
			uint16_t numTaps,
			q15_t * pCoeffs,
			q15_t * pState,
			q15_t mu,
			uint32_t blockSize,
			uint32_t postShift);

  








 

  void arm_lms_q15(
		   const arm_lms_instance_q15 * S,
		    q15_t * pSrc,
		    q15_t * pRef,
		   q15_t * pOut,
		   q15_t * pErr,
		   uint32_t blockSize);


  

 

  typedef struct
  {
    uint16_t numTaps;     
    q31_t *pState;        
    q31_t *pCoeffs;       
    q31_t mu;             
    uint32_t postShift;   

  } arm_lms_instance_q31;

  








 

  void arm_lms_q31(
		   const arm_lms_instance_q31 * S,
		    q31_t * pSrc,
		    q31_t * pRef,
		   q31_t * pOut,
		   q31_t * pErr,
		   uint32_t blockSize);

  









 

  void arm_lms_init_q31(
			arm_lms_instance_q31 * S,
			uint16_t numTaps,
			q31_t *pCoeffs,
			q31_t *pState,
			q31_t mu,
			uint32_t blockSize,
			uint32_t postShift);

  

 

  typedef struct
  {
    uint16_t  numTaps;     
    float32_t *pState;     
    float32_t *pCoeffs;    
    float32_t mu;         
    float32_t energy;     
    float32_t x0;         
  } arm_lms_norm_instance_f32;

  








 

  void arm_lms_norm_f32(
			arm_lms_norm_instance_f32 * S,
			 float32_t * pSrc,
			 float32_t * pRef,
			float32_t * pOut,
			float32_t * pErr,
			uint32_t blockSize);

  








 

  void arm_lms_norm_init_f32(
			     arm_lms_norm_instance_f32 * S,
			     uint16_t numTaps,
			     float32_t * pCoeffs,
			     float32_t * pState,
			     float32_t mu,
			     uint32_t blockSize);


  

 
  typedef struct
  {
    uint16_t numTaps;      
    q31_t *pState;         
    q31_t *pCoeffs;        
    q31_t mu;              
    uint8_t postShift;     
    q31_t *recipTable;     
    q31_t energy;          
    q31_t x0;              
  } arm_lms_norm_instance_q31;

  








 

  void arm_lms_norm_q31(
			arm_lms_norm_instance_q31 * S,
			 q31_t * pSrc,
			 q31_t * pRef,
			q31_t * pOut,
			q31_t * pErr,
			uint32_t blockSize);

  









 

  void arm_lms_norm_init_q31(
			     arm_lms_norm_instance_q31 * S,
			     uint16_t numTaps,
			     q31_t * pCoeffs,
			     q31_t * pState,
			     q31_t mu,
			     uint32_t blockSize,
			     uint8_t postShift);

  

 

  typedef struct
  {
    uint16_t numTaps;     
    q15_t *pState;         
    q15_t *pCoeffs;        
    q15_t mu;             
    uint8_t postShift;    
    q15_t *recipTable;    
    q15_t energy;         
    q15_t x0;             
  } arm_lms_norm_instance_q15;

  








 

  void arm_lms_norm_q15(
			arm_lms_norm_instance_q15 * S,
			 q15_t * pSrc,
			 q15_t * pRef,
			q15_t * pOut,
			q15_t * pErr,
			uint32_t blockSize);


  









 

  void arm_lms_norm_init_q15(
			     arm_lms_norm_instance_q15 * S,
			     uint16_t numTaps,
			     q15_t * pCoeffs,
			     q15_t * pState,
			     q15_t mu,
			     uint32_t blockSize,
			     uint8_t postShift);

  







 

  void arm_correlate_f32(
			  float32_t * pSrcA,
			 uint32_t srcALen,
			  float32_t * pSrcB,
			 uint32_t srcBLen,
			 float32_t * pDst);

  







 

  void arm_correlate_q15(
			  q15_t * pSrcA,
			 uint32_t srcALen,
			  q15_t * pSrcB,
			 uint32_t srcBLen,
			 q15_t * pDst);

  







 

  void arm_correlate_fast_q15(
			       q15_t * pSrcA,
			      uint32_t srcALen,
			       q15_t * pSrcB,
			      uint32_t srcBLen,
			      q15_t * pDst);

  







 

  void arm_correlate_q31(
			  q31_t * pSrcA,
			 uint32_t srcALen,
			  q31_t * pSrcB,
			 uint32_t srcBLen,
			 q31_t * pDst);

  







 

  void arm_correlate_fast_q31(
			       q31_t * pSrcA,
			      uint32_t srcALen,
			       q31_t * pSrcB,
			      uint32_t srcBLen,
			      q31_t * pDst);

  







 

  void arm_correlate_q7(
			 q7_t * pSrcA,
			uint32_t srcALen,
			 q7_t * pSrcB,
			uint32_t srcBLen,
			q7_t * pDst);

  

 
  typedef struct
  {
    uint16_t numTaps;              
    uint16_t stateIndex;           
    float32_t *pState;             
    float32_t *pCoeffs;            
    uint16_t maxDelay;             
    int32_t *pTapDelay;            
  } arm_fir_sparse_instance_f32;

  

 

  typedef struct
  {
    uint16_t numTaps;              
    uint16_t stateIndex;           
    q31_t *pState;                 
    q31_t *pCoeffs;                
    uint16_t maxDelay;             
    int32_t *pTapDelay;            
  } arm_fir_sparse_instance_q31;

  

 

  typedef struct
  {
    uint16_t numTaps;              
    uint16_t stateIndex;           
    q15_t *pState;                 
    q15_t *pCoeffs;                
    uint16_t maxDelay;             
    int32_t *pTapDelay;            
  } arm_fir_sparse_instance_q15;

  

 

  typedef struct
  {
    uint16_t numTaps;              
    uint16_t stateIndex;           
    q7_t *pState;                  
    q7_t *pCoeffs;                 
    uint16_t maxDelay;             
    int32_t *pTapDelay;            
  } arm_fir_sparse_instance_q7;

  







 

  void arm_fir_sparse_f32(
			  arm_fir_sparse_instance_f32 * S,
			   float32_t * pSrc,
			  float32_t * pDst,
			  float32_t * pScratchIn,
			  uint32_t blockSize);

  









 

  void arm_fir_sparse_init_f32(
			       arm_fir_sparse_instance_f32 * S,
			       uint16_t numTaps,
			       float32_t * pCoeffs,
			       float32_t * pState,
			       int32_t * pTapDelay,
			       uint16_t maxDelay,
			       uint32_t blockSize);

  







 

  void arm_fir_sparse_q31(
			  arm_fir_sparse_instance_q31 * S,
			   q31_t * pSrc,
			  q31_t * pDst,
			  q31_t * pScratchIn,
			  uint32_t blockSize);

  









 

  void arm_fir_sparse_init_q31(
			       arm_fir_sparse_instance_q31 * S,
			       uint16_t numTaps,
			       q31_t * pCoeffs,
			       q31_t * pState,
			       int32_t * pTapDelay,
			       uint16_t maxDelay,
			       uint32_t blockSize);

  








 

  void arm_fir_sparse_q15(
			  arm_fir_sparse_instance_q15 * S,
			   q15_t * pSrc,
			  q15_t * pDst,
			  q15_t * pScratchIn,
			  q31_t * pScratchOut,
			  uint32_t blockSize);


  









 

  void arm_fir_sparse_init_q15(
			       arm_fir_sparse_instance_q15 * S,
			       uint16_t numTaps,
			       q15_t * pCoeffs,
			       q15_t * pState,
			       int32_t * pTapDelay,
			       uint16_t maxDelay,
			       uint32_t blockSize);

  








 

  void arm_fir_sparse_q7(
			 arm_fir_sparse_instance_q7 * S,
			  q7_t * pSrc,
			 q7_t * pDst,
			 q7_t * pScratchIn,
			 q31_t * pScratchOut,
			 uint32_t blockSize);

  









 

  void arm_fir_sparse_init_q7(
			      arm_fir_sparse_instance_q7 * S,
			      uint16_t numTaps,
			      q7_t * pCoeffs,
			      q7_t * pState,
			      int32_t *pTapDelay,
			      uint16_t maxDelay,
			      uint32_t blockSize);


  





 

  void arm_sin_cos_f32(
		       float32_t theta,
		       float32_t *pSinVal,
		       float32_t *pCcosVal);

  





 

  void arm_sin_cos_q31(
		       q31_t theta,
		       q31_t *pSinVal,
		       q31_t *pCosVal);


  





 

  void arm_cmplx_conj_f32(
			   float32_t * pSrc,
			  float32_t * pDst,
			  uint32_t numSamples);

  





 

  void arm_cmplx_conj_q31(
			   q31_t * pSrc,
			  q31_t * pDst,
			  uint32_t numSamples);

  





 

  void arm_cmplx_conj_q15(
			   q15_t * pSrc,
			  q15_t * pDst,
			  uint32_t numSamples);



  





 

  void arm_cmplx_mag_squared_f32(
				  float32_t * pSrc,
				 float32_t * pDst,
				 uint32_t numSamples);

  





 

  void arm_cmplx_mag_squared_q31(
				  q31_t * pSrc,
				 q31_t * pDst,
				 uint32_t numSamples);

  





 

  void arm_cmplx_mag_squared_q15(
				  q15_t * pSrc,
				 q15_t * pDst,
				 uint32_t numSamples);


 

 

  






















































 

  


 

  




 


  static __inline float32_t arm_pid_f32(
					arm_pid_instance_f32 * S,
					float32_t in)
  {
    float32_t out;

     
    out = (S->A0 * in) +
      (S->A1 * S->state[0]) + (S->A2 * S->state[1]) + (S->state[2]);

     
    S->state[1] = S->state[0];
    S->state[0] = in;
    S->state[2] = out;

     
    return (out);

  }

  












 

  static __inline q31_t arm_pid_q31(
				    arm_pid_instance_q31 * S,
				    q31_t in)
  {
    q63_t acc;
	q31_t out;

     
    acc = (q63_t) S->A0 * in;

     
    acc += (q63_t) S->A1 * S->state[0];

     
    acc += (q63_t) S->A2 * S->state[1];

     
    out = (q31_t) (acc >> 31u);

     
    out += S->state[2];

     
    S->state[1] = S->state[0];
    S->state[0] = in;
    S->state[2] = out;

     
    return (out);

  }

  













 

  static __inline q15_t arm_pid_q15(
				    arm_pid_instance_q15 * S,
				    q15_t in)
  {
    q63_t acc;
    q15_t out;

     

#line 4809 "C:\\STM32F4-Discovery_FW_V1.1.0\\Libraries\\CMSIS\\Include\\arm_math.h"
				
     
    acc = (q31_t) __smuad(S->A0, in);
	


#line 4822 "C:\\STM32F4-Discovery_FW_V1.1.0\\Libraries\\CMSIS\\Include\\arm_math.h"

     
    acc = __smlald(S->A1, (q31_t)(*(int32_t * *) & (S->state)), acc);



     
    acc += (q31_t) S->state[2] << 15;

     
    out = (q15_t) (__ssat((acc >> 15), 16));

     
    S->state[1] = S->state[0];
    S->state[0] = in;
    S->state[2] = out;

     
    return (out);

  }
  
  

 


  





 

  arm_status arm_mat_inverse_f32(
				 const arm_matrix_instance_f32 * src,
				 arm_matrix_instance_f32 * dst);

  
 
  

 


  



















 

  


 

  







 

  static __inline void arm_clarke_f32(
				      float32_t Ia,
				      float32_t Ib,
				      float32_t * pIalpha,
				      float32_t * pIbeta)
  {
     
    *pIalpha = Ia;

     
    *pIbeta = ((float32_t) 0.57735026919 * Ia + (float32_t) 1.15470053838 * Ib);

  }

  












 

  static __inline void arm_clarke_q31(
				      q31_t Ia,
				      q31_t Ib,
				      q31_t * pIalpha,
				      q31_t * pIbeta)
  {
    q31_t product1, product2;                     

     
    *pIalpha = Ia;

     
    product1 = (q31_t) (((q63_t) Ia * 0x24F34E8B) >> 30);

     
    product2 = (q31_t) (((q63_t) Ib * 0x49E69D16) >> 30);

     
    *pIbeta = __qadd(product1, product2);
  }

  

 

  





 
  void arm_q7_to_q31(
		     q7_t * pSrc,
		     q31_t * pDst,
		     uint32_t blockSize);


 

  

 

  













 

  


 

   






 


  static __inline void arm_inv_clarke_f32(
					  float32_t Ialpha,
					  float32_t Ibeta,
					  float32_t * pIa,
					  float32_t * pIb)
  {
     
    *pIa = Ialpha;

     
    *pIb = -0.5 * Ialpha + (float32_t) 0.8660254039 *Ibeta;

  }

  












 

  static __inline void arm_inv_clarke_q31(
					  q31_t Ialpha,
					  q31_t Ibeta,
					  q31_t * pIa,
					  q31_t * pIb)
  {
    q31_t product1, product2;                     

     
    *pIa = Ialpha;

     
    product1 = (q31_t) (((q63_t) (Ialpha) * (0x40000000)) >> 31);

     
    product2 = (q31_t) (((q63_t) (Ibeta) * (0x6ED9EBA1)) >> 31);

     
    *pIb = __qsub(product2, product1);

  }

  

 

  





 
  void arm_q7_to_q15(
		      q7_t * pSrc,
		     q15_t * pDst,
		     uint32_t blockSize);

  

  

 

  





















 

  


 

  











 

  static __inline void arm_park_f32(
				    float32_t Ialpha,
				    float32_t Ibeta,
				    float32_t * pId,
				    float32_t * pIq,
				    float32_t sinVal,
				    float32_t cosVal)
  {
     
    *pId = Ialpha * cosVal + Ibeta * sinVal;

     
    *pIq = -Ialpha * sinVal + Ibeta * cosVal;

  }

  














 


  static __inline void arm_park_q31(
				    q31_t Ialpha,
				    q31_t Ibeta,
				    q31_t * pId,
				    q31_t * pIq,
				    q31_t sinVal,
				    q31_t cosVal)
  {
    q31_t product1, product2;                     
    q31_t product3, product4;                     

     
    product1 = (q31_t) (((q63_t) (Ialpha) * (cosVal)) >> 31);

     
    product2 = (q31_t) (((q63_t) (Ibeta) * (sinVal)) >> 31);


     
    product3 = (q31_t) (((q63_t) (Ialpha) * (sinVal)) >> 31);

     
    product4 = (q31_t) (((q63_t) (Ibeta) * (cosVal)) >> 31);

     
    *pId = __qadd(product1, product2);

     
    *pIq = __qsub(product4, product3);
  }

  

 

  





 
  void arm_q7_to_float(
		        q7_t * pSrc,
		       float32_t * pDst,
		       uint32_t blockSize);

 
  

 

  














 

  


 

   








 

  static __inline void arm_inv_park_f32(
					float32_t Id,
					float32_t Iq,
					float32_t * pIalpha,
					float32_t * pIbeta,
					float32_t sinVal,
					float32_t cosVal)
  {
     
    *pIalpha = Id * cosVal - Iq * sinVal;

     
    *pIbeta = Id * sinVal + Iq * cosVal;

  }


  














 


  static __inline void arm_inv_park_q31(
					q31_t Id,
					q31_t Iq,
					q31_t * pIalpha,
					q31_t * pIbeta,
					q31_t sinVal,
					q31_t cosVal)
  {
    q31_t product1, product2;                     
    q31_t product3, product4;                     

     
    product1 = (q31_t) (((q63_t) (Id) * (cosVal)) >> 31);

     
    product2 = (q31_t) (((q63_t) (Iq) * (sinVal)) >> 31);


     
    product3 = (q31_t) (((q63_t) (Id) * (sinVal)) >> 31);

     
    product4 = (q31_t) (((q63_t) (Iq) * (cosVal)) >> 31);

     
    *pIalpha = __qsub(product1, product2);

     
    *pIbeta = __qadd(product4, product3);

  }

  

 

   
  





 
  void arm_q31_to_float(
			 q31_t * pSrc,
			float32_t * pDst,
			uint32_t blockSize);

  

 

  





























 

  


 

  





 

  static __inline float32_t arm_linear_interp_f32(
						  arm_linear_interp_instance_f32 * S,
						  float32_t x)
  {

	  float32_t y;
	  float32_t x0, x1;						 
	  float32_t y0, y1;	  					 
	  float32_t xSpacing = S->xSpacing;		 
	  int32_t i;  							 
	  float32_t *pYData = S->pYData;	     

	   
	  i =   (x - S->x1) / xSpacing;

	  if(i < 0)
	  {
	      
		 y = pYData[0];
	  }
	  else if(i >= S->nValues)
	  {
	  	   
	  	  y = pYData[S->nValues-1];	
	  }
	  else
	  {	 
	  	   
		  x0 = S->x1 + i * xSpacing;
		  x1 = S->x1 + (i +1) * xSpacing;
		 
		  
		  y0 = pYData[i];
		  y1 = pYData[i + 1];
		
		   
		  y = y0 + (x - x0) * ((y1 - y0)/(x1-x0));	
		
	  }

       
	  return (y);
  }

   











 


  static __inline q31_t arm_linear_interp_q31(q31_t *pYData,
					      q31_t x, uint32_t nValues)
  {
    q31_t y;                                    
    q31_t y0, y1;                                 
    q31_t fract;                                  
    int32_t index;                               
    
     
     
     
    index = ((x & 0xFFF00000) >> 20);

	if(index >= (nValues - 1))
	{
		return(pYData[nValues - 1]);
	}
	else if(index < 0)
	{
		return(pYData[0]);
	}
	else
	{

	     
	     
	    fract = (x & 0x000FFFFF) << 11;
	
	     
	    y0 = pYData[index];
	    y1 = pYData[index + 1u];
	
	     
	    y = ((q31_t) ((q63_t) y0 * (0x7FFFFFFF - fract) >> 32));
	
	     
	    y += ((q31_t) (((q63_t) y1 * fract) >> 32));
	
	     
	    return (y << 1u);

	}

  }

  











 


  static __inline q15_t arm_linear_interp_q15(q15_t *pYData, q31_t x, uint32_t nValues)
  {
    q63_t y;                                    
    q15_t y0, y1;                               
    q31_t fract;                                
    int32_t index;                              

     
     
     
    index = ((x & 0xFFF00000) >> 20u); 

	if(index >= (nValues - 1))
	{
		return(pYData[nValues - 1]);
	}
	else if(index < 0)
	{
		return(pYData[0]);
	}
	else
	{	
	     
	     
	    fract = (x & 0x000FFFFF);
	
	     
	    y0 = pYData[index];
	    y1 = pYData[index + 1u];
	
	     
	    y = ((q63_t) y0 * (0xFFFFF - fract));
	
	     
	    y += ((q63_t) y1 * (fract));
	
	     
	    return (y >> 20);
	}


  }

  










 


  static __inline q7_t arm_linear_interp_q7(q7_t *pYData, q31_t x,  uint32_t nValues)
  {
    q31_t y;                                    
    q7_t y0, y1;                                  
    q31_t fract;                                  
    int32_t index;                               
    
     
     
     
    index = ((x & 0xFFF00000) >> 20u);


    if(index >= (nValues - 1))
	{
		return(pYData[nValues - 1]);
	}
	else if(index < 0)
	{
		return(pYData[0]);
	}
	else
	{

	     
	     
	    fract = (x & 0x000FFFFF);
	
	     
	    y0 = pYData[index];
	    y1 = pYData[index + 1u];
	
	     
	    y = ((y0 * (0xFFFFF - fract)));
	
	     
	    y += (y1 * fract);
	
	     
	    return (y >> 20u);

	}

  }
  

 

  



 

  float32_t arm_sin_f32(
			 float32_t x);

  



 

  q31_t arm_sin_q31(
		     q31_t x);

  



 

  q15_t arm_sin_q15(
		     q15_t x);

  



 

  float32_t arm_cos_f32(
			 float32_t x);

  



 

  q31_t arm_cos_q31(
		     q31_t x);

  



 

  q15_t arm_cos_q15(
		     q15_t x);


  

 


  

















 


  


 

  





 

  static __inline arm_status  arm_sqrt_f32(
					  float32_t in, float32_t *pOut)
  {
  	if(in > 0)
	{



		*pOut = __sqrtf(in);




		return (ARM_MATH_SUCCESS);
	}
  	else
	{
		*pOut = 0.0f;
		return (ARM_MATH_ARGUMENT_ERROR);
	}

  }


  





 
  arm_status arm_sqrt_q31(
		      q31_t in, q31_t *pOut);

  





 
  arm_status arm_sqrt_q15(
		      q15_t in, q15_t *pOut);

  

 






  

 

  static __inline void arm_circularWrite_f32(
					     int32_t * circBuffer,
					     int32_t L,
					     uint16_t * writeOffset,
					     int32_t bufferInc,
					     const int32_t * src,
					     int32_t srcInc,
					     uint32_t blockSize)
  {
    uint32_t i = 0u;
    int32_t wOffset;

    
 
    wOffset = *writeOffset;

     
    i = blockSize;

    while(i > 0u)
      {
	 
	circBuffer[wOffset] = *src;

	 
	src += srcInc;

	 
	wOffset += bufferInc;
	if(wOffset >= L)
	  wOffset -= L;

	 
	i--;
      }

     
    *writeOffset = wOffset;
  }



  

 
  static __inline void arm_circularRead_f32(
					    int32_t * circBuffer,
					    int32_t L,
					    int32_t * readOffset,
					    int32_t bufferInc,
					    int32_t * dst,
					    int32_t * dst_base,
					    int32_t dst_length,
					    int32_t dstInc,
					    uint32_t blockSize)
  {
    uint32_t i = 0u;
    int32_t rOffset, dst_end;

    
 
    rOffset = *readOffset;
    dst_end = (int32_t) (dst_base + dst_length);

     
    i = blockSize;

    while(i > 0u)
      {
	 
	*dst = circBuffer[rOffset];

	 
	dst += dstInc;

	if(dst == (int32_t *) dst_end)
	  {
	    dst = dst_base;
	  }

	 
	rOffset += bufferInc;

	if(rOffset >= L)
	  {
	    rOffset -= L;
	  }

	 
	i--;
      }

     
    *readOffset = rOffset;
  }

  

 

  static __inline void arm_circularWrite_q15(
					     q15_t * circBuffer,
					     int32_t L,
					     uint16_t * writeOffset,
					     int32_t bufferInc,
					     const q15_t * src,
					     int32_t srcInc,
					     uint32_t blockSize)
  {
    uint32_t i = 0u;
    int32_t wOffset;

    
 
    wOffset = *writeOffset;

     
    i = blockSize;

    while(i > 0u)
      {
	 
	circBuffer[wOffset] = *src;

	 
	src += srcInc;

	 
	wOffset += bufferInc;
	if(wOffset >= L)
	  wOffset -= L;

	 
	i--;
      }

     
    *writeOffset = wOffset;
  }



  

 
  static __inline void arm_circularRead_q15(
					    q15_t * circBuffer,
					    int32_t L,
					    int32_t * readOffset,
					    int32_t bufferInc,
					    q15_t * dst,
					    q15_t * dst_base,
					    int32_t dst_length,
					    int32_t dstInc,
					    uint32_t blockSize)
  {
    uint32_t i = 0;
    int32_t rOffset, dst_end;

    
 
    rOffset = *readOffset;

    dst_end = (int32_t) (dst_base + dst_length);

     
    i = blockSize;

    while(i > 0u)
      {
	 
	*dst = circBuffer[rOffset];

	 
	dst += dstInc;

	if(dst == (q15_t *) dst_end)
	  {
	    dst = dst_base;
	  }

	 
	rOffset += bufferInc;

	if(rOffset >= L)
	  {
	    rOffset -= L;
	  }

	 
	i--;
      }

     
    *readOffset = rOffset;
  }


  

 

  static __inline void arm_circularWrite_q7(
					    q7_t * circBuffer,
					    int32_t L,
					    uint16_t * writeOffset,
					    int32_t bufferInc,
					    const q7_t * src,
					    int32_t srcInc,
					    uint32_t blockSize)
  {
    uint32_t i = 0u;
    int32_t wOffset;

    
 
    wOffset = *writeOffset;

     
    i = blockSize;

    while(i > 0u)
      {
	 
	circBuffer[wOffset] = *src;

	 
	src += srcInc;

	 
	wOffset += bufferInc;
	if(wOffset >= L)
	  wOffset -= L;

	 
	i--;
      }

     
    *writeOffset = wOffset;
  }



  

 
  static __inline void arm_circularRead_q7(
					   q7_t * circBuffer,
					   int32_t L,
					   int32_t * readOffset,
					   int32_t bufferInc,
					   q7_t * dst,
					   q7_t * dst_base,
					   int32_t dst_length,
					   int32_t dstInc,
					   uint32_t blockSize)
  {
    uint32_t i = 0;
    int32_t rOffset, dst_end;

    
 
    rOffset = *readOffset;

    dst_end = (int32_t) (dst_base + dst_length);

     
    i = blockSize;

    while(i > 0u)
      {
	 
	*dst = circBuffer[rOffset];

	 
	dst += dstInc;

	if(dst == (q7_t *) dst_end)
	  {
	    dst = dst_base;
	  }

	 
	rOffset += bufferInc;

	if(rOffset >= L)
	  {
	    rOffset -= L;
	  }

	 
	i--;
      }

     
    *readOffset = rOffset;
  }


  





 

  void arm_power_q31(
		      q31_t * pSrc,
		     uint32_t blockSize,
		     q63_t * pResult);

  





 

  void arm_power_f32(
		      float32_t * pSrc,
		     uint32_t blockSize,
		     float32_t * pResult);

  





 

  void arm_power_q15(
		      q15_t * pSrc,
		     uint32_t blockSize,
		     q63_t * pResult);

  





 

  void arm_power_q7(
		     q7_t * pSrc,
		    uint32_t blockSize,
		    q31_t * pResult);

  





 

  void arm_mean_q7(
		    q7_t * pSrc,
		   uint32_t blockSize,
		   q7_t * pResult);

  





 
  void arm_mean_q15(
		     q15_t * pSrc,
		    uint32_t blockSize,
		    q15_t * pResult);

  





 
  void arm_mean_q31(
		     q31_t * pSrc,
		    uint32_t blockSize,
		    q31_t * pResult);

  





 
  void arm_mean_f32(
		     float32_t * pSrc,
		    uint32_t blockSize,
		    float32_t * pResult);

  





 

  void arm_var_f32(
		    float32_t * pSrc,
		   uint32_t blockSize,
		   float32_t * pResult);

  





 

  void arm_var_q31(
		    q31_t * pSrc,
		   uint32_t blockSize,
		   q63_t * pResult);

  





 

  void arm_var_q15(
		    q15_t * pSrc,
		   uint32_t blockSize,
		   q31_t * pResult);

  





 

  void arm_rms_f32(
		    float32_t * pSrc,
		   uint32_t blockSize,
		   float32_t * pResult);

  





 

  void arm_rms_q31(
		    q31_t * pSrc,
		   uint32_t blockSize,
		   q31_t * pResult);

  





 

  void arm_rms_q15(
		    q15_t * pSrc,
		   uint32_t blockSize,
		   q15_t * pResult);

  





 

  void arm_std_f32(
		    float32_t * pSrc,
		   uint32_t blockSize,
		   float32_t * pResult);

  





 

  void arm_std_q31(
		    q31_t * pSrc,
		   uint32_t blockSize,
		   q31_t * pResult);

  





 

  void arm_std_q15(
		    q15_t * pSrc,
		   uint32_t blockSize,
		   q15_t * pResult);

  





 

  void arm_cmplx_mag_f32(
			  float32_t * pSrc,
			 float32_t * pDst,
			 uint32_t numSamples);

  





 

  void arm_cmplx_mag_q31(
			  q31_t * pSrc,
			 q31_t * pDst,
			 uint32_t numSamples);

  





 

  void arm_cmplx_mag_q15(
			  q15_t * pSrc,
			 q15_t * pDst,
			 uint32_t numSamples);

  







 

  void arm_cmplx_dot_prod_q15(
			       q15_t * pSrcA,
			       q15_t * pSrcB,
			      uint32_t numSamples,
			      q31_t * realResult,
			      q31_t * imagResult);

  







 

  void arm_cmplx_dot_prod_q31(
			       q31_t * pSrcA,
			       q31_t * pSrcB,
			      uint32_t numSamples,
			      q63_t * realResult,
			      q63_t * imagResult);

  







 

  void arm_cmplx_dot_prod_f32(
			       float32_t * pSrcA,
			       float32_t * pSrcB,
			      uint32_t numSamples,
			      float32_t * realResult,
			      float32_t * imagResult);

  






 

  void arm_cmplx_mult_real_q15(
			        q15_t * pSrcCmplx,
			        q15_t * pSrcReal,
			       q15_t * pCmplxDst,
			       uint32_t numSamples);

  






 

  void arm_cmplx_mult_real_q31(
			        q31_t * pSrcCmplx,
			        q31_t * pSrcReal,
			       q31_t * pCmplxDst,
			       uint32_t numSamples);

  






 

  void arm_cmplx_mult_real_f32(
			        float32_t * pSrcCmplx,
			        float32_t * pSrcReal,
			       float32_t * pCmplxDst,
			       uint32_t numSamples);

  






 

  void arm_min_q7(
		   q7_t * pSrc,
		  uint32_t blockSize,
		  q7_t * result,
		  uint32_t * index);

  






 

  void arm_min_q15(
		    q15_t * pSrc,
		   uint32_t blockSize,
		   q15_t * pResult,
		   uint32_t * pIndex);

  






 
  void arm_min_q31(
		    q31_t * pSrc,
		   uint32_t blockSize,
		   q31_t * pResult,
		   uint32_t * pIndex);

  






 

  void arm_min_f32(
		    float32_t * pSrc,
		   uint32_t blockSize,
		   float32_t * pResult,
		   uint32_t * pIndex);








 

  void arm_max_q7(
		   q7_t * pSrc,
		  uint32_t blockSize,
		  q7_t * pResult,
		  uint32_t * pIndex);








 

  void arm_max_q15(
		    q15_t * pSrc,
		   uint32_t blockSize,
		   q15_t * pResult,
		   uint32_t * pIndex);








 

  void arm_max_q31(
		    q31_t * pSrc,
		   uint32_t blockSize,
		   q31_t * pResult,
		   uint32_t * pIndex);








 

  void arm_max_f32(
		    float32_t * pSrc,
		   uint32_t blockSize,
		   float32_t * pResult,
		   uint32_t * pIndex);

  






 

  void arm_cmplx_mult_cmplx_q15(
			        q15_t * pSrcA,
			        q15_t * pSrcB,
			       q15_t * pDst,
			       uint32_t numSamples);

  






 

  void arm_cmplx_mult_cmplx_q31(
			        q31_t * pSrcA,
			        q31_t * pSrcB,
			       q31_t * pDst,
			       uint32_t numSamples);

  






 

  void arm_cmplx_mult_cmplx_f32(
			        float32_t * pSrcA,
			        float32_t * pSrcB,
			       float32_t * pDst,
			       uint32_t numSamples);

  





 
  void arm_float_to_q31(
			       float32_t * pSrc,
			      q31_t * pDst,
			      uint32_t blockSize);

  





 
  void arm_float_to_q15(
			       float32_t * pSrc,
			      q15_t * pDst,
			      uint32_t blockSize);

  





 
  void arm_float_to_q7(
			      float32_t * pSrc,
			     q7_t * pDst,
			     uint32_t blockSize);


  





 
  void arm_q31_to_q15(
		       q31_t * pSrc,
		      q15_t * pDst,
		      uint32_t blockSize);

  





 
  void arm_q31_to_q7(
		      q31_t * pSrc,
		     q7_t * pDst,
		     uint32_t blockSize);

  





 
  void arm_q15_to_float(
			 q15_t * pSrc,
			float32_t * pDst,
			uint32_t blockSize);


  





 
  void arm_q15_to_q31(
		       q15_t * pSrc,
		      q31_t * pDst,
		      uint32_t blockSize);


  





 
  void arm_q15_to_q7(
		      q15_t * pSrc,
		     q7_t * pDst,
		     uint32_t blockSize);


  

 

  

















































 

  


 

  






 

  
  static __inline float32_t arm_bilinear_interp_f32(
						    const arm_bilinear_interp_instance_f32 * S,
						    float32_t X,
						    float32_t Y)
  {
    float32_t out;
    float32_t f00, f01, f10, f11;
    float32_t *pData = S->pData;
    int32_t xIndex, yIndex, index;
    float32_t xdiff, ydiff;
    float32_t b1, b2, b3, b4;

    xIndex = (int32_t) X;
    yIndex = (int32_t) Y;

	 
	 
	if(xIndex < 0 || xIndex > (S->numRows-1) || yIndex < 0  || yIndex > ( S->numCols-1))
	{
		return(0);
	}
	
     
    index = (xIndex - 1) + (yIndex-1) *  S->numCols ;


     
    f00 = pData[index];
    f01 = pData[index + 1];

     
    index = (xIndex-1) + (yIndex) * S->numCols;


     
    f10 = pData[index];
    f11 = pData[index + 1];

     
    b1 = f00;
    b2 = f01 - f00;
    b3 = f10 - f00;
    b4 = f00 - f01 - f10 + f11;

     
    xdiff = X - xIndex;

     
    ydiff = Y - yIndex;

     
     out = b1 + b2 * xdiff + b3 * ydiff + b4 * xdiff * ydiff;

    
    return (out);

  }

  






 

  static __inline q31_t arm_bilinear_interp_q31(
						arm_bilinear_interp_instance_q31 * S,
						q31_t X,
						q31_t Y)
  {
    q31_t out;                                    
    q31_t acc = 0;                                
    q31_t xfract, yfract;                         
    q31_t x1, x2, y1, y2;                         
    int32_t rI, cI;                              
    q31_t *pYData = S->pData;                     
    uint32_t nCols = S->numCols;                  


     
     
     
    rI = ((X & 0xFFF00000) >> 20u);

     
     
     
    cI = ((Y & 0xFFF00000) >> 20u);

	 
	 
	if(rI < 0 || rI > (S->numRows-1) || cI < 0  || cI > ( S->numCols-1))
	{
		return(0);
	}

     
     
    xfract = (X & 0x000FFFFF) << 11u;

     
    x1 = pYData[(rI) + nCols * (cI)];
    x2 = pYData[(rI) + nCols * (cI) + 1u];

     
     
    yfract = (Y & 0x000FFFFF) << 11u;

     
    y1 = pYData[(rI) + nCols * (cI + 1)];
    y2 = pYData[(rI) + nCols * (cI + 1) + 1u];

     
    out = ((q31_t) (((q63_t) x1 * (0x7FFFFFFF - xfract)) >> 32));
    acc = ((q31_t) (((q63_t) out * (0x7FFFFFFF - yfract)) >> 32));

     
    out = ((q31_t) ((q63_t) x2 * (0x7FFFFFFF - yfract) >> 32));
    acc += ((q31_t) ((q63_t) out * (xfract) >> 32));

     
    out = ((q31_t) ((q63_t) y1 * (0x7FFFFFFF - xfract) >> 32));
    acc += ((q31_t) ((q63_t) out * (yfract) >> 32));

     
    out = ((q31_t) ((q63_t) y2 * (xfract) >> 32));
    acc += ((q31_t) ((q63_t) out * (yfract) >> 32));

     
    return (acc << 2u);

  }

  





 

  static __inline q15_t arm_bilinear_interp_q15(
						arm_bilinear_interp_instance_q15 * S,
						q31_t X,
						q31_t Y)
  {
    q63_t acc = 0;                                
    q31_t out;                                    
    q15_t x1, x2, y1, y2;                         
    q31_t xfract, yfract;                         
    int32_t rI, cI;                              
    q15_t *pYData = S->pData;                     
    uint32_t nCols = S->numCols;                  

     
     
     
    rI = ((X & 0xFFF00000) >> 20);

     
     
     
    cI = ((Y & 0xFFF00000) >> 20);

	 
	 
	if(rI < 0 || rI > (S->numRows-1) || cI < 0  || cI > ( S->numCols-1))
	{
		return(0);
	}

     
     
    xfract = (X & 0x000FFFFF);

     
    x1 = pYData[(rI) + nCols * (cI)];
    x2 = pYData[(rI) + nCols * (cI) + 1u];


     
     
    yfract = (Y & 0x000FFFFF);

     
    y1 = pYData[(rI) + nCols * (cI + 1)];
    y2 = pYData[(rI) + nCols * (cI + 1) + 1u];

     

     
     
    out = (q31_t) (((q63_t) x1 * (0xFFFFF - xfract)) >> 4u);
    acc = ((q63_t) out * (0xFFFFF - yfract));

     
    out = (q31_t) (((q63_t) x2 * (0xFFFFF - yfract)) >> 4u);
    acc += ((q63_t) out * (xfract));

     
    out = (q31_t) (((q63_t) y1 * (0xFFFFF - xfract)) >> 4u);
    acc += ((q63_t) out * (yfract));

     
    out = (q31_t) (((q63_t) y2 * (xfract)) >> 4u);
    acc += ((q63_t) out * (yfract));

     
     
    return (acc >> 36);

  }

  





 

  static __inline q7_t arm_bilinear_interp_q7(
					      arm_bilinear_interp_instance_q7 * S,
					      q31_t X,
					      q31_t Y)
  {
    q63_t acc = 0;                                
    q31_t out;                                    
    q31_t xfract, yfract;                         
    q7_t x1, x2, y1, y2;                          
    int32_t rI, cI;                              
    q7_t *pYData = S->pData;                      
    uint32_t nCols = S->numCols;                  

     
     
     
    rI = ((X & 0xFFF00000) >> 20);

     
     
     
    cI = ((Y & 0xFFF00000) >> 20);

	 
	 
	if(rI < 0 || rI > (S->numRows-1) || cI < 0  || cI > ( S->numCols-1))
	{
		return(0);
	}

     
     
    xfract = (X & 0x000FFFFF);

     
    x1 = pYData[(rI) + nCols * (cI)];
    x2 = pYData[(rI) + nCols * (cI) + 1u];


     
     
    yfract = (Y & 0x000FFFFF);

     
    y1 = pYData[(rI) + nCols * (cI + 1)];
    y2 = pYData[(rI) + nCols * (cI + 1) + 1u];

     
    out = ((x1 * (0xFFFFF - xfract)));
    acc = (((q63_t) out * (0xFFFFF - yfract)));

     
    out = ((x2 * (0xFFFFF - yfract)));
    acc += (((q63_t) out * (xfract)));

     
    out = ((y1 * (0xFFFFF - xfract)));
    acc += (((q63_t) out * (yfract)));

     
    out = ((y2 * (yfract)));
    acc += (((q63_t) out * (xfract)));

     
    return (acc >> 40);

  }

  

 

















 
#line 4 "Source\\main.c"
#line 1 "C:\\STM32F4-Discovery_FW_V1.1.0\\Utilities\\STM32F4-Discovery\\stm32f4_discovery.h"




















  
  
 






                                              
 
#line 33 "C:\\STM32F4-Discovery_FW_V1.1.0\\Utilities\\STM32F4-Discovery\\stm32f4_discovery.h"
   


 
  


 
      


  



 
typedef enum 
{
  LED4 = 0,
  LED3 = 1,
  LED5 = 2,
  LED6 = 3
} Led_TypeDef;

typedef enum 
{  
  BUTTON_USER = 0,
} Button_TypeDef;

typedef enum 
{  
  BUTTON_MODE_GPIO = 0,
  BUTTON_MODE_EXTI = 1
} ButtonMode_TypeDef;     


  



  



 





  



  



  





  
  


   




 
#line 114 "C:\\STM32F4-Discovery_FW_V1.1.0\\Utilities\\STM32F4-Discovery\\stm32f4_discovery.h"


  
  


   


  




 
void STM_EVAL_LEDInit(Led_TypeDef Led);
void STM_EVAL_LEDOn(Led_TypeDef Led);
void STM_EVAL_LEDOff(Led_TypeDef Led);
void STM_EVAL_LEDToggle(Led_TypeDef Led);
void STM_EVAL_PBInit(Button_TypeDef Button, ButtonMode_TypeDef Button_Mode);
uint32_t STM_EVAL_PBGetState(Button_TypeDef Button);


 
  







  



  



 

 

 
#line 5 "Source\\main.c"
#line 1 "C:\\Keil\\ARM\\ARMCC\\bin\\..\\include\\stdio.h"
 
 
 





 






 













#line 38 "C:\\Keil\\ARM\\ARMCC\\bin\\..\\include\\stdio.h"


  
  typedef unsigned int size_t;    








 
 

 
  typedef struct __va_list __va_list;





   




 




typedef struct __fpos_t_struct {
    unsigned __int64 __pos;
    



 
    struct {
        unsigned int __state1, __state2;
    } __mbstate;
} fpos_t;
   


 


   

 

typedef struct __FILE FILE;
   






 

extern FILE __stdin, __stdout, __stderr;
extern FILE *__aeabi_stdin, *__aeabi_stdout, *__aeabi_stderr;

#line 129 "C:\\Keil\\ARM\\ARMCC\\bin\\..\\include\\stdio.h"
    

    

    





     



   


 


   


 

   



 

   


 




   


 





    


 






extern __declspec(__nothrow) int remove(const char *  ) __attribute__((__nonnull__(1)));
   





 
extern __declspec(__nothrow) int rename(const char *  , const char *  ) __attribute__((__nonnull__(1,2)));
   








 
extern __declspec(__nothrow) FILE *tmpfile(void);
   




 
extern __declspec(__nothrow) char *tmpnam(char *  );
   











 

extern __declspec(__nothrow) int fclose(FILE *  ) __attribute__((__nonnull__(1)));
   







 
extern __declspec(__nothrow) int fflush(FILE *  );
   







 
extern __declspec(__nothrow) FILE *fopen(const char * __restrict  ,
                           const char * __restrict  ) __attribute__((__nonnull__(1,2)));
   








































 
extern __declspec(__nothrow) FILE *freopen(const char * __restrict  ,
                    const char * __restrict  ,
                    FILE * __restrict  ) __attribute__((__nonnull__(2,3)));
   








 
extern __declspec(__nothrow) void setbuf(FILE * __restrict  ,
                    char * __restrict  ) __attribute__((__nonnull__(1)));
   




 
extern __declspec(__nothrow) int setvbuf(FILE * __restrict  ,
                   char * __restrict  ,
                   int  , size_t  ) __attribute__((__nonnull__(1)));
   















 
#pragma __printf_args
extern __declspec(__nothrow) int fprintf(FILE * __restrict  ,
                    const char * __restrict  , ...) __attribute__((__nonnull__(1,2)));
   


















 
#pragma __printf_args
extern __declspec(__nothrow) int _fprintf(FILE * __restrict  ,
                     const char * __restrict  , ...) __attribute__((__nonnull__(1,2)));
   



 
#pragma __printf_args
extern __declspec(__nothrow) int printf(const char * __restrict  , ...) __attribute__((__nonnull__(1)));
   




 
#pragma __printf_args
extern __declspec(__nothrow) int _printf(const char * __restrict  , ...) __attribute__((__nonnull__(1)));
   



 
#pragma __printf_args
extern __declspec(__nothrow) int sprintf(char * __restrict  , const char * __restrict  , ...) __attribute__((__nonnull__(1,2)));
   






 
#pragma __printf_args
extern __declspec(__nothrow) int _sprintf(char * __restrict  , const char * __restrict  , ...) __attribute__((__nonnull__(1,2)));
   



 

#pragma __printf_args
extern __declspec(__nothrow) int snprintf(char * __restrict  , size_t  ,
                     const char * __restrict  , ...) __attribute__((__nonnull__(3)));
   















 

#pragma __printf_args
extern __declspec(__nothrow) int _snprintf(char * __restrict  , size_t  ,
                      const char * __restrict  , ...) __attribute__((__nonnull__(3)));
   



 
#pragma __scanf_args
extern __declspec(__nothrow) int fscanf(FILE * __restrict  ,
                    const char * __restrict  , ...) __attribute__((__nonnull__(1,2)));
   






























 
#pragma __scanf_args
extern __declspec(__nothrow) int _fscanf(FILE * __restrict  ,
                     const char * __restrict  , ...) __attribute__((__nonnull__(1,2)));
   



 
#pragma __scanf_args
extern __declspec(__nothrow) int scanf(const char * __restrict  , ...) __attribute__((__nonnull__(1)));
   






 
#pragma __scanf_args
extern __declspec(__nothrow) int _scanf(const char * __restrict  , ...) __attribute__((__nonnull__(1)));
   



 
#pragma __scanf_args
extern __declspec(__nothrow) int sscanf(const char * __restrict  ,
                    const char * __restrict  , ...) __attribute__((__nonnull__(1,2)));
   








 
#pragma __scanf_args
extern __declspec(__nothrow) int _sscanf(const char * __restrict  ,
                     const char * __restrict  , ...) __attribute__((__nonnull__(1,2)));
   



 

 
extern __declspec(__nothrow) int vfscanf(FILE * __restrict  , const char * __restrict  , __va_list) __attribute__((__nonnull__(1,2)));
extern __declspec(__nothrow) int vscanf(const char * __restrict  , __va_list) __attribute__((__nonnull__(1)));
extern __declspec(__nothrow) int vsscanf(const char * __restrict  , const char * __restrict  , __va_list) __attribute__((__nonnull__(1,2)));

extern __declspec(__nothrow) int _vfscanf(FILE * __restrict  , const char * __restrict  , __va_list) __attribute__((__nonnull__(1,2)));
extern __declspec(__nothrow) int _vscanf(const char * __restrict  , __va_list) __attribute__((__nonnull__(1)));
extern __declspec(__nothrow) int _vsscanf(const char * __restrict  , const char * __restrict  , __va_list) __attribute__((__nonnull__(1,2)));

extern __declspec(__nothrow) int vprintf(const char * __restrict  , __va_list  ) __attribute__((__nonnull__(1)));
   





 
extern __declspec(__nothrow) int _vprintf(const char * __restrict  , __va_list  ) __attribute__((__nonnull__(1)));
   



 
extern __declspec(__nothrow) int vfprintf(FILE * __restrict  ,
                    const char * __restrict  , __va_list  ) __attribute__((__nonnull__(1,2)));
   






 
extern __declspec(__nothrow) int vsprintf(char * __restrict  ,
                     const char * __restrict  , __va_list  ) __attribute__((__nonnull__(1,2)));
   






 

extern __declspec(__nothrow) int vsnprintf(char * __restrict  , size_t  ,
                     const char * __restrict  , __va_list  ) __attribute__((__nonnull__(3)));
   







 

extern __declspec(__nothrow) int _vsprintf(char * __restrict  ,
                      const char * __restrict  , __va_list  ) __attribute__((__nonnull__(1,2)));
   



 
extern __declspec(__nothrow) int _vfprintf(FILE * __restrict  ,
                     const char * __restrict  , __va_list  ) __attribute__((__nonnull__(1,2)));
   



 
extern __declspec(__nothrow) int _vsnprintf(char * __restrict  , size_t  ,
                      const char * __restrict  , __va_list  ) __attribute__((__nonnull__(3)));
   



 
extern __declspec(__nothrow) int fgetc(FILE *  ) __attribute__((__nonnull__(1)));
   







 
extern __declspec(__nothrow) char *fgets(char * __restrict  , int  ,
                    FILE * __restrict  ) __attribute__((__nonnull__(1,3)));
   










 
extern __declspec(__nothrow) int fputc(int  , FILE *  ) __attribute__((__nonnull__(2)));
   







 
extern __declspec(__nothrow) int fputs(const char * __restrict  , FILE * __restrict  ) __attribute__((__nonnull__(1,2)));
   




 
extern __declspec(__nothrow) int getc(FILE *  ) __attribute__((__nonnull__(1)));
   







 




    extern __declspec(__nothrow) int (getchar)(void);

   





 
extern __declspec(__nothrow) char *gets(char *  ) __attribute__((__nonnull__(1)));
   









 
extern __declspec(__nothrow) int putc(int  , FILE *  ) __attribute__((__nonnull__(2)));
   





 




    extern __declspec(__nothrow) int (putchar)(int  );

   



 
extern __declspec(__nothrow) int puts(const char *  ) __attribute__((__nonnull__(1)));
   





 
extern __declspec(__nothrow) int ungetc(int  , FILE *  ) __attribute__((__nonnull__(2)));
   






















 

extern __declspec(__nothrow) size_t fread(void * __restrict  ,
                    size_t  , size_t  , FILE * __restrict  ) __attribute__((__nonnull__(1,4)));
   











 

extern __declspec(__nothrow) size_t __fread_bytes_avail(void * __restrict  ,
                    size_t  , FILE * __restrict  ) __attribute__((__nonnull__(1,3)));
   











 

extern __declspec(__nothrow) size_t fwrite(const void * __restrict  ,
                    size_t  , size_t  , FILE * __restrict  ) __attribute__((__nonnull__(1,4)));
   







 

extern __declspec(__nothrow) int fgetpos(FILE * __restrict  , fpos_t * __restrict  ) __attribute__((__nonnull__(1,2)));
   








 
extern __declspec(__nothrow) int fseek(FILE *  , long int  , int  ) __attribute__((__nonnull__(1)));
   














 
extern __declspec(__nothrow) int fsetpos(FILE * __restrict  , const fpos_t * __restrict  ) __attribute__((__nonnull__(1,2)));
   










 
extern __declspec(__nothrow) long int ftell(FILE *  ) __attribute__((__nonnull__(1)));
   











 
extern __declspec(__nothrow) void rewind(FILE *  ) __attribute__((__nonnull__(1)));
   





 

extern __declspec(__nothrow) void clearerr(FILE *  ) __attribute__((__nonnull__(1)));
   




 

extern __declspec(__nothrow) int feof(FILE *  ) __attribute__((__nonnull__(1)));
   


 
extern __declspec(__nothrow) int ferror(FILE *  ) __attribute__((__nonnull__(1)));
   


 
extern __declspec(__nothrow) void perror(const char *  );
   









 

extern __declspec(__nothrow) int _fisatty(FILE *   ) __attribute__((__nonnull__(1)));
    
 

extern __declspec(__nothrow) void __use_no_semihosting_swi(void);
extern __declspec(__nothrow) void __use_no_semihosting(void);
    





 











#line 948 "C:\\Keil\\ARM\\ARMCC\\bin\\..\\include\\stdio.h"



 

#line 6 "Source\\main.c"
void TimingDelay_Decrement(void);
static volatile uint32_t TimingDelay; 





 
void Delay(volatile uint32_t nTime)
{ 
  TimingDelay = nTime;

  while(TimingDelay != 0);
}

int main()
{
	Delay(100);
	printf("Hello World!\n");
	return 0;
}
