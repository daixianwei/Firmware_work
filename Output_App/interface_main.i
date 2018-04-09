#line 1 "Management\\Interface\\Interface_main.c"





 

 
#line 1 ".\\Management\\DisplayDriver\\Font.h"





 



 
#line 1 ".\\System\\CM3\\stm32f10x.h"







































 



 



 
    
#line 56 ".\\System\\CM3\\stm32f10x.h"
  


 
  


 

#line 75 ".\\System\\CM3\\stm32f10x.h"


















 





#line 107 ".\\System\\CM3\\stm32f10x.h"







            
#line 122 ".\\System\\CM3\\stm32f10x.h"





 






 
#line 143 ".\\System\\CM3\\stm32f10x.h"



 



 



 
#line 162 ".\\System\\CM3\\stm32f10x.h"




 
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
  TAMPER_IRQn                 = 2,       
  RTC_IRQn                    = 3,       
  FLASH_IRQn                  = 4,       
  RCC_IRQn                    = 5,       
  EXTI0_IRQn                  = 6,       
  EXTI1_IRQn                  = 7,       
  EXTI2_IRQn                  = 8,       
  EXTI3_IRQn                  = 9,       
  EXTI4_IRQn                  = 10,      
  DMA1_Channel1_IRQn          = 11,      
  DMA1_Channel2_IRQn          = 12,      
  DMA1_Channel3_IRQn          = 13,      
  DMA1_Channel4_IRQn          = 14,      
  DMA1_Channel5_IRQn          = 15,      
  DMA1_Channel6_IRQn          = 16,      
  DMA1_Channel7_IRQn          = 17,      

#line 221 ".\\System\\CM3\\stm32f10x.h"

#line 242 ".\\System\\CM3\\stm32f10x.h"

#line 270 ".\\System\\CM3\\stm32f10x.h"

#line 296 ".\\System\\CM3\\stm32f10x.h"


  ADC1_2_IRQn                 = 18,      
  USB_HP_CAN1_TX_IRQn         = 19,      
  USB_LP_CAN1_RX0_IRQn        = 20,      
  CAN1_RX1_IRQn               = 21,      
  CAN1_SCE_IRQn               = 22,      
  EXTI9_5_IRQn                = 23,      
  TIM1_BRK_IRQn               = 24,      
  TIM1_UP_IRQn                = 25,      
  TIM1_TRG_COM_IRQn           = 26,      
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
  RTCAlarm_IRQn               = 41,      
  USBWakeUp_IRQn              = 42,      
  TIM8_BRK_IRQn               = 43,      
  TIM8_UP_IRQn                = 44,      
  TIM8_TRG_COM_IRQn           = 45,      
  TIM8_CC_IRQn                = 46,      
  ADC3_IRQn                   = 47,      
  FSMC_IRQn                   = 48,      
  SDIO_IRQn                   = 49,      
  TIM5_IRQn                   = 50,      
  SPI3_IRQn                   = 51,      
  UART4_IRQn                  = 52,      
  UART5_IRQn                  = 53,      
  TIM6_IRQn                   = 54,      
  TIM7_IRQn                   = 55,      
  DMA2_Channel1_IRQn          = 56,      
  DMA2_Channel2_IRQn          = 57,      
  DMA2_Channel3_IRQn          = 58,      
  DMA2_Channel4_5_IRQn        = 59       


#line 381 ".\\System\\CM3\\stm32f10x.h"

#line 426 ".\\System\\CM3\\stm32f10x.h"

#line 472 ".\\System\\CM3\\stm32f10x.h"
} IRQn_Type;



 

#line 1 ".\\System\\CM3\\core_cm3.h"
 




















 




































 

 
 
 
 
 
 
 
 








 











#line 1 "D:\\Keil4.0\\ARM\\RV31\\Inc\\stdint.h"
 
 





 









#line 25 "D:\\Keil4.0\\ARM\\RV31\\Inc\\stdint.h"







 

     

     
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




     

     





     





     





     

     





     





     





     

     





     





     





     

     


     


     


     

     


     


     


     

     



     



     


     
    
 



#line 196 "D:\\Keil4.0\\ARM\\RV31\\Inc\\stdint.h"

     







     










     











#line 260 "D:\\Keil4.0\\ARM\\RV31\\Inc\\stdint.h"



 


#line 90 ".\\System\\CM3\\core_cm3.h"

















 

#line 116 ".\\System\\CM3\\core_cm3.h"





 


 





 
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
} SCB_Type;                                                

 












 






























 






 





















 









 


















 
































                                     









 









 









 














   





 
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
  volatile uint32_t IWR;                           
  volatile uint32_t IRR;                           
  volatile uint32_t IMCR;                          
       uint32_t RESERVED4[43];                                  
  volatile uint32_t LAR;                           
  volatile uint32_t LSR;                           
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
       uint32_t RESERVED0;
  volatile const  uint32_t ICTR;                          



       uint32_t RESERVED1;

} InterruptType_Type;

 



 








   


#line 613 ".\\System\\CM3\\core_cm3.h"





 
typedef struct
{
  volatile uint32_t DHCSR;                         
  volatile  uint32_t DCRSR;                         
  volatile uint32_t DCRDR;                         
  volatile uint32_t DEMCR;                         
} CoreDebug_Type;

 




































 






 






































   


 
#line 720 ".\\System\\CM3\\core_cm3.h"

#line 727 ".\\System\\CM3\\core_cm3.h"






   




 





#line 757 ".\\System\\CM3\\core_cm3.h"


 


 




#line 782 ".\\System\\CM3\\core_cm3.h"


 
 
 
 








 
extern uint32_t __get_PSP(void);








 
extern void __set_PSP(uint32_t topOfProcStack);








 
extern uint32_t __get_MSP(void);








 
extern void __set_MSP(uint32_t topOfMainStack);








 
extern uint32_t __REV16(uint16_t value);








 
extern int32_t __REVSH(int16_t value);


#line 932 ".\\System\\CM3\\core_cm3.h"





 








 
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







 
static __inline uint32_t __get_FAULTMASK(void)
{
  register uint32_t __regFaultMask       __asm("faultmask");
  return(__regFaultMask);
}







 
static __inline void __set_FAULTMASK(uint32_t faultMask)
{
  register uint32_t __regFaultMask       __asm("faultmask");
  __regFaultMask = (faultMask & 1);
}







 
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





#line 1444 ".\\System\\CM3\\core_cm3.h"







 
 

 











 
static __inline void NVIC_SetPriorityGrouping(uint32_t PriorityGroup)
{
  uint32_t reg_value;
  uint32_t PriorityGroupTmp = (PriorityGroup & 0x07);                          
  
  reg_value  =  ((SCB_Type *) ((0xE000E000) + 0x0D00))->AIRCR;                                                    
  reg_value &= ~((0xFFFFul << 16) | (7ul << 8));              
  reg_value  =  (reg_value                       |
                (0x5FA << 16) | 
                (PriorityGroupTmp << 8));                                      
  ((SCB_Type *) ((0xE000E000) + 0x0D00))->AIRCR =  reg_value;
}








 
static __inline uint32_t NVIC_GetPriorityGrouping(void)
{
  return ((((SCB_Type *) ((0xE000E000) + 0x0D00))->AIRCR & (7ul << 8)) >> 8);    
}








 
static __inline void NVIC_EnableIRQ(IRQn_Type IRQn)
{
  ((NVIC_Type *) ((0xE000E000) + 0x0100))->ISER[((uint32_t)(IRQn) >> 5)] = (1 << ((uint32_t)(IRQn) & 0x1F));  
}








 
static __inline void NVIC_DisableIRQ(IRQn_Type IRQn)
{
  ((NVIC_Type *) ((0xE000E000) + 0x0100))->ICER[((uint32_t)(IRQn) >> 5)] = (1 << ((uint32_t)(IRQn) & 0x1F));  
}









 
static __inline uint32_t NVIC_GetPendingIRQ(IRQn_Type IRQn)
{
  return((uint32_t) ((((NVIC_Type *) ((0xE000E000) + 0x0100))->ISPR[(uint32_t)(IRQn) >> 5] & (1 << ((uint32_t)(IRQn) & 0x1F)))?1:0));  
}








 
static __inline void NVIC_SetPendingIRQ(IRQn_Type IRQn)
{
  ((NVIC_Type *) ((0xE000E000) + 0x0100))->ISPR[((uint32_t)(IRQn) >> 5)] = (1 << ((uint32_t)(IRQn) & 0x1F));  
}








 
static __inline void NVIC_ClearPendingIRQ(IRQn_Type IRQn)
{
  ((NVIC_Type *) ((0xE000E000) + 0x0100))->ICPR[((uint32_t)(IRQn) >> 5)] = (1 << ((uint32_t)(IRQn) & 0x1F));  
}









 
static __inline uint32_t NVIC_GetActive(IRQn_Type IRQn)
{
  return((uint32_t)((((NVIC_Type *) ((0xE000E000) + 0x0100))->IABR[(uint32_t)(IRQn) >> 5] & (1 << ((uint32_t)(IRQn) & 0x1F)))?1:0));  
}












 
static __inline void NVIC_SetPriority(IRQn_Type IRQn, uint32_t priority)
{
  if(IRQn < 0) {
    ((SCB_Type *) ((0xE000E000) + 0x0D00))->SHP[((uint32_t)(IRQn) & 0xF)-4] = ((priority << (8 - 4)) & 0xff); }  
  else {
    ((NVIC_Type *) ((0xE000E000) + 0x0100))->IP[(uint32_t)(IRQn)] = ((priority << (8 - 4)) & 0xff);    }         
}















 
static __inline uint32_t NVIC_GetPriority(IRQn_Type IRQn)
{

  if(IRQn < 0) {
    return((uint32_t)(((SCB_Type *) ((0xE000E000) + 0x0D00))->SHP[((uint32_t)(IRQn) & 0xF)-4] >> (8 - 4)));  }  
  else {
    return((uint32_t)(((NVIC_Type *) ((0xE000E000) + 0x0100))->IP[(uint32_t)(IRQn)]           >> (8 - 4)));  }  
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



 












 
static __inline uint32_t SysTick_Config(uint32_t ticks)
{ 
  if (ticks > (0xFFFFFFul << 0))  return (1);             
                                                               
  ((SysTick_Type *) ((0xE000E000) + 0x0010))->LOAD  = (ticks & (0xFFFFFFul << 0)) - 1;       
  NVIC_SetPriority (SysTick_IRQn, (1<<4) - 1);   
  ((SysTick_Type *) ((0xE000E000) + 0x0010))->VAL   = 0;                                           
  ((SysTick_Type *) ((0xE000E000) + 0x0010))->CTRL  = (1ul << 2) | 
                   (1ul << 1)   | 
                   (1ul << 0);                     
  return (0);                                                   
}






 





 
static __inline void NVIC_SystemReset(void)
{
  ((SCB_Type *) ((0xE000E000) + 0x0D00))->AIRCR  = ((0x5FA << 16)      | 
                 (((SCB_Type *) ((0xE000E000) + 0x0D00))->AIRCR & (7ul << 8)) | 
                 (1ul << 2));                    
  __dsb(0);                                                                    
  while(1);                                                     
}

   



 






 
 

extern volatile int ITM_RxBuffer;                     












 
static __inline uint32_t ITM_SendChar (uint32_t ch)
{
  if ((((CoreDebug_Type *) (0xE000EDF0))->DEMCR & (1ul << 24))  &&       
      (((ITM_Type *) (0xE0000000))->TCR & (1ul << 0))                  &&       
      (((ITM_Type *) (0xE0000000))->TER & (1ul << 0)        )                    )      
  {
    while (((ITM_Type *) (0xE0000000))->PORT[0].u32 == 0);
    ((ITM_Type *) (0xE0000000))->PORT[0].u8 = (uint8_t) ch;
  }  
  return (ch);
}










 
static __inline int ITM_ReceiveChar (void) {
  int ch = -1;                                

  if (ITM_RxBuffer != 0x5AA55AA5) {
    ch = ITM_RxBuffer;
    ITM_RxBuffer = 0x5AA55AA5;        
  }
  
  return (ch); 
}









 
static __inline int ITM_CheckChar (void) {

  if (ITM_RxBuffer == 0x5AA55AA5) {
    return (0);                                  
  } else {
    return (1);                                  
  }
}

   






   



 
#line 479 ".\\System\\CM3\\stm32f10x.h"
#line 1 ".\\System\\CM3\\system_stm32f10x.h"



















 



 



   
  


 









 



 




 

extern uint32_t SystemCoreClock;           



 



 



 



 



 



 
  
extern void SystemInit(void);
extern void SystemCoreClockUpdate(void);


 









 
  


   
 
#line 480 ".\\System\\CM3\\stm32f10x.h"
#line 481 ".\\System\\CM3\\stm32f10x.h"



   

 
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


typedef enum {FALSE = 0, TRUE = !FALSE} bool;


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
  uint32_t  RESERVED0;
  volatile uint16_t DR1;
  uint16_t  RESERVED1;
  volatile uint16_t DR2;
  uint16_t  RESERVED2;
  volatile uint16_t DR3;
  uint16_t  RESERVED3;
  volatile uint16_t DR4;
  uint16_t  RESERVED4;
  volatile uint16_t DR5;
  uint16_t  RESERVED5;
  volatile uint16_t DR6;
  uint16_t  RESERVED6;
  volatile uint16_t DR7;
  uint16_t  RESERVED7;
  volatile uint16_t DR8;
  uint16_t  RESERVED8;
  volatile uint16_t DR9;
  uint16_t  RESERVED9;
  volatile uint16_t DR10;
  uint16_t  RESERVED10; 
  volatile uint16_t RTCCR;
  uint16_t  RESERVED11;
  volatile uint16_t CR;
  uint16_t  RESERVED12;
  volatile uint16_t CSR;
  uint16_t  RESERVED13[5];
  volatile uint16_t DR11;
  uint16_t  RESERVED14;
  volatile uint16_t DR12;
  uint16_t  RESERVED15;
  volatile uint16_t DR13;
  uint16_t  RESERVED16;
  volatile uint16_t DR14;
  uint16_t  RESERVED17;
  volatile uint16_t DR15;
  uint16_t  RESERVED18;
  volatile uint16_t DR16;
  uint16_t  RESERVED19;
  volatile uint16_t DR17;
  uint16_t  RESERVED20;
  volatile uint16_t DR18;
  uint16_t  RESERVED21;
  volatile uint16_t DR19;
  uint16_t  RESERVED22;
  volatile uint16_t DR20;
  uint16_t  RESERVED23;
  volatile uint16_t DR21;
  uint16_t  RESERVED24;
  volatile uint16_t DR22;
  uint16_t  RESERVED25;
  volatile uint16_t DR23;
  uint16_t  RESERVED26;
  volatile uint16_t DR24;
  uint16_t  RESERVED27;
  volatile uint16_t DR25;
  uint16_t  RESERVED28;
  volatile uint16_t DR26;
  uint16_t  RESERVED29;
  volatile uint16_t DR27;
  uint16_t  RESERVED30;
  volatile uint16_t DR28;
  uint16_t  RESERVED31;
  volatile uint16_t DR29;
  uint16_t  RESERVED32;
  volatile uint16_t DR30;
  uint16_t  RESERVED33; 
  volatile uint16_t DR31;
  uint16_t  RESERVED34;
  volatile uint16_t DR32;
  uint16_t  RESERVED35;
  volatile uint16_t DR33;
  uint16_t  RESERVED36;
  volatile uint16_t DR34;
  uint16_t  RESERVED37;
  volatile uint16_t DR35;
  uint16_t  RESERVED38;
  volatile uint16_t DR36;
  uint16_t  RESERVED39;
  volatile uint16_t DR37;
  uint16_t  RESERVED40;
  volatile uint16_t DR38;
  uint16_t  RESERVED41;
  volatile uint16_t DR39;
  uint16_t  RESERVED42;
  volatile uint16_t DR40;
  uint16_t  RESERVED43;
  volatile uint16_t DR41;
  uint16_t  RESERVED44;
  volatile uint16_t DR42;
  uint16_t  RESERVED45;    
} BKP_TypeDef;
  


 

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
  volatile uint32_t MCR;
  volatile uint32_t MSR;
  volatile uint32_t TSR;
  volatile uint32_t RF0R;
  volatile uint32_t RF1R;
  volatile uint32_t IER;
  volatile uint32_t ESR;
  volatile uint32_t BTR;
  uint32_t  RESERVED0[88];
  CAN_TxMailBox_TypeDef sTxMailBox[3];
  CAN_FIFOMailBox_TypeDef sFIFOMailBox[2];
  uint32_t  RESERVED1[12];
  volatile uint32_t FMR;
  volatile uint32_t FM1R;
  uint32_t  RESERVED2;
  volatile uint32_t FS1R;
  uint32_t  RESERVED3;
  volatile uint32_t FFA1R;
  uint32_t  RESERVED4;
  volatile uint32_t FA1R;
  uint32_t  RESERVED5[8];

  CAN_FilterRegister_TypeDef sFilterRegister[14];



} CAN_TypeDef;



 
typedef struct
{
  volatile uint32_t CFGR;
  volatile uint32_t OAR;
  volatile uint32_t PRES;
  volatile uint32_t ESR;
  volatile uint32_t CSR;
  volatile uint32_t TXD;
  volatile uint32_t RXD;  
} CEC_TypeDef;



 

typedef struct
{
  volatile uint32_t DR;
  volatile uint8_t  IDR;
  uint8_t   RESERVED0;
  uint16_t  RESERVED1;
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



} DAC_TypeDef;



 

typedef struct
{
  volatile uint32_t IDCODE;
  volatile uint32_t CR;	
}DBGMCU_TypeDef;



 

typedef struct
{
  volatile uint32_t CCR;
  volatile uint32_t CNDTR;
  volatile uint32_t CPAR;
  volatile uint32_t CMAR;
} DMA_Channel_TypeDef;

typedef struct
{
  volatile uint32_t ISR;
  volatile uint32_t IFCR;
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
       uint32_t RESERVED0[2];
  volatile uint32_t MACRWUFFR;              
  volatile uint32_t MACPMTCSR;
       uint32_t RESERVED1[2];
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
       uint32_t RESERVED2[40];
  volatile uint32_t MMCCR;                  
  volatile uint32_t MMCRIR;
  volatile uint32_t MMCTIR;
  volatile uint32_t MMCRIMR;
  volatile uint32_t MMCTIMR;                
       uint32_t RESERVED3[14];
  volatile uint32_t MMCTGFSCCR;             
  volatile uint32_t MMCTGFMSCCR;
       uint32_t RESERVED4[5];
  volatile uint32_t MMCTGFCR;
       uint32_t RESERVED5[10];
  volatile uint32_t MMCRFCECR;
  volatile uint32_t MMCRFAECR;
       uint32_t RESERVED6[10];
  volatile uint32_t MMCRGUFCR;
       uint32_t RESERVED7[334];
  volatile uint32_t PTPTSCR;
  volatile uint32_t PTPSSIR;
  volatile uint32_t PTPTSHR;
  volatile uint32_t PTPTSLR;
  volatile uint32_t PTPTSHUR;
  volatile uint32_t PTPTSLUR;
  volatile uint32_t PTPTSAR;
  volatile uint32_t PTPTTHR;
  volatile uint32_t PTPTTLR;
       uint32_t RESERVED8[567];
  volatile uint32_t DMABMR;
  volatile uint32_t DMATPDR;
  volatile uint32_t DMARPDR;
  volatile uint32_t DMARDLAR;
  volatile uint32_t DMATDLAR;
  volatile uint32_t DMASR;
  volatile uint32_t DMAOMR;
  volatile uint32_t DMAIER;
  volatile uint32_t DMAMFBOCR;
       uint32_t RESERVED9[9];
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
  volatile uint32_t AR;
  volatile uint32_t RESERVED;
  volatile uint32_t OBR;
  volatile uint32_t WRPR;
#line 924 ".\\System\\CM3\\stm32f10x.h"
} FLASH_TypeDef;



 
  
typedef struct
{
  volatile uint16_t RDP;
  volatile uint16_t USER;
  volatile uint16_t Data0;
  volatile uint16_t Data1;
  volatile uint16_t WRP0;
  volatile uint16_t WRP1;
  volatile uint16_t WRP2;
  volatile uint16_t WRP3;
} OB_TypeDef;



 

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
  uint32_t  RESERVED0;   
  volatile uint32_t ECCR2; 
} FSMC_Bank2_TypeDef;  



 
  
typedef struct
{
  volatile uint32_t PCR3;
  volatile uint32_t SR3;
  volatile uint32_t PMEM3;
  volatile uint32_t PATT3;
  uint32_t  RESERVED0;   
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
  volatile uint32_t CRL;
  volatile uint32_t CRH;
  volatile uint32_t IDR;
  volatile uint32_t ODR;
  volatile uint32_t BSRR;
  volatile uint32_t BRR;
  volatile uint32_t LCKR;
} GPIO_TypeDef;



 

typedef struct
{
  volatile uint32_t EVCR;
  volatile uint32_t MAPR;
  volatile uint32_t EXTICR[4];
  uint32_t RESERVED0;
  volatile uint32_t MAPR2;  
} AFIO_TypeDef;


 

typedef struct
{
  volatile uint16_t CR1;
  uint16_t  RESERVED0;
  volatile uint16_t CR2;
  uint16_t  RESERVED1;
  volatile uint16_t OAR1;
  uint16_t  RESERVED2;
  volatile uint16_t OAR2;
  uint16_t  RESERVED3;
  volatile uint16_t DR;
  uint16_t  RESERVED4;
  volatile uint16_t SR1;
  uint16_t  RESERVED5;
  volatile uint16_t SR2;
  uint16_t  RESERVED6;
  volatile uint16_t CCR;
  uint16_t  RESERVED7;
  volatile uint16_t TRISE;
  uint16_t  RESERVED8;
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
  volatile uint32_t CFGR;
  volatile uint32_t CIR;
  volatile uint32_t APB2RSTR;
  volatile uint32_t APB1RSTR;
  volatile uint32_t AHBENR;
  volatile uint32_t APB2ENR;
  volatile uint32_t APB1ENR;
  volatile uint32_t BDCR;
  volatile uint32_t CSR;










} RCC_TypeDef;



 

typedef struct
{
  volatile uint16_t CRH;
  uint16_t  RESERVED0;
  volatile uint16_t CRL;
  uint16_t  RESERVED1;
  volatile uint16_t PRLH;
  uint16_t  RESERVED2;
  volatile uint16_t PRLL;
  uint16_t  RESERVED3;
  volatile uint16_t DIVH;
  uint16_t  RESERVED4;
  volatile uint16_t DIVL;
  uint16_t  RESERVED5;
  volatile uint16_t CNTH;
  uint16_t  RESERVED6;
  volatile uint16_t CNTL;
  uint16_t  RESERVED7;
  volatile uint16_t ALRH;
  uint16_t  RESERVED8;
  volatile uint16_t ALRL;
  uint16_t  RESERVED9;
} RTC_TypeDef;



 

typedef struct
{
  volatile uint32_t POWER;
  volatile uint32_t CLKCR;
  volatile uint32_t ARG;
  volatile uint32_t CMD;
  volatile const uint32_t RESPCMD;
  volatile const uint32_t RESP1;
  volatile const uint32_t RESP2;
  volatile const uint32_t RESP3;
  volatile const uint32_t RESP4;
  volatile uint32_t DTIMER;
  volatile uint32_t DLEN;
  volatile uint32_t DCTRL;
  volatile const uint32_t DCOUNT;
  volatile const uint32_t STA;
  volatile uint32_t ICR;
  volatile uint32_t MASK;
  uint32_t  RESERVED0[2];
  volatile const uint32_t FIFOCNT;
  uint32_t  RESERVED1[13];
  volatile uint32_t FIFO;
} SDIO_TypeDef;



 

typedef struct
{
  volatile uint16_t CR1;
  uint16_t  RESERVED0;
  volatile uint16_t CR2;
  uint16_t  RESERVED1;
  volatile uint16_t SR;
  uint16_t  RESERVED2;
  volatile uint16_t DR;
  uint16_t  RESERVED3;
  volatile uint16_t CRCPR;
  uint16_t  RESERVED4;
  volatile uint16_t RXCRCR;
  uint16_t  RESERVED5;
  volatile uint16_t TXCRCR;
  uint16_t  RESERVED6;
  volatile uint16_t I2SCFGR;
  uint16_t  RESERVED7;
  volatile uint16_t I2SPR;
  uint16_t  RESERVED8;  
} SPI_TypeDef;



 

typedef struct
{
  volatile uint16_t CR1;
  uint16_t  RESERVED0;
  volatile uint16_t CR2;
  uint16_t  RESERVED1;
  volatile uint16_t SMCR;
  uint16_t  RESERVED2;
  volatile uint16_t DIER;
  uint16_t  RESERVED3;
  volatile uint16_t SR;
  uint16_t  RESERVED4;
  volatile uint16_t EGR;
  uint16_t  RESERVED5;
  volatile uint16_t CCMR1;
  uint16_t  RESERVED6;
  volatile uint16_t CCMR2;
  uint16_t  RESERVED7;
  volatile uint16_t CCER;
  uint16_t  RESERVED8;
  volatile uint16_t CNT;
  uint16_t  RESERVED9;
  volatile uint16_t PSC;
  uint16_t  RESERVED10;
  volatile uint16_t ARR;
  uint16_t  RESERVED11;
  volatile uint16_t RCR;
  uint16_t  RESERVED12;
  volatile uint16_t CCR1;
  uint16_t  RESERVED13;
  volatile uint16_t CCR2;
  uint16_t  RESERVED14;
  volatile uint16_t CCR3;
  uint16_t  RESERVED15;
  volatile uint16_t CCR4;
  uint16_t  RESERVED16;
  volatile uint16_t BDTR;
  uint16_t  RESERVED17;
  volatile uint16_t DCR;
  uint16_t  RESERVED18;
  volatile uint16_t DMAR;
  uint16_t  RESERVED19;
} TIM_TypeDef;



 
 
typedef struct
{
  volatile uint16_t SR;
  uint16_t  RESERVED0;
  volatile uint16_t DR;
  uint16_t  RESERVED1;
  volatile uint16_t BRR;
  uint16_t  RESERVED2;
  volatile uint16_t CR1;
  uint16_t  RESERVED3;
  volatile uint16_t CR2;
  uint16_t  RESERVED4;
  volatile uint16_t CR3;
  uint16_t  RESERVED5;
  volatile uint16_t GTPR;
  uint16_t  RESERVED6;
} USART_TypeDef;



 

typedef struct
{
  volatile uint32_t CR;
  volatile uint32_t CFR;
  volatile uint32_t SR;
} WWDG_TypeDef;



 
  


 











 




#line 1316 ".\\System\\CM3\\stm32f10x.h"

#line 1339 ".\\System\\CM3\\stm32f10x.h"



#line 1358 ".\\System\\CM3\\stm32f10x.h"




















 
  


   

#line 1458 ".\\System\\CM3\\stm32f10x.h"



 



 
  
  

 
    
 
 
 

 
 
 
 
 

 



 



 


 
 
 
 
 

 











 
#line 1519 ".\\System\\CM3\\stm32f10x.h"




 





 
 
 
 
 

 


 


 


 


 


 


 


 


 


 


 


 


 


 


 


 


 


 


 


 


 


 


 


 


 


 


 


 


 


 


 


 


 


 


 


 


 


 


 


 


 


 


 





 



 






 
 
 
 
 

 
#line 1695 ".\\System\\CM3\\stm32f10x.h"

#line 1702 ".\\System\\CM3\\stm32f10x.h"

 
 








 








 






#line 1738 ".\\System\\CM3\\stm32f10x.h"

 











 











 













 






#line 1854 ".\\System\\CM3\\stm32f10x.h"




#line 1874 ".\\System\\CM3\\stm32f10x.h"

 





#line 1887 ".\\System\\CM3\\stm32f10x.h"

 
#line 1906 ".\\System\\CM3\\stm32f10x.h"

#line 1915 ".\\System\\CM3\\stm32f10x.h"

 
#line 1923 ".\\System\\CM3\\stm32f10x.h"



















#line 1948 ".\\System\\CM3\\stm32f10x.h"












 













#line 1980 ".\\System\\CM3\\stm32f10x.h"





#line 1994 ".\\System\\CM3\\stm32f10x.h"

#line 2001 ".\\System\\CM3\\stm32f10x.h"

#line 2011 ".\\System\\CM3\\stm32f10x.h"











 


















#line 2047 ".\\System\\CM3\\stm32f10x.h"

 
#line 2055 ".\\System\\CM3\\stm32f10x.h"



















#line 2080 ".\\System\\CM3\\stm32f10x.h"












 













#line 2112 ".\\System\\CM3\\stm32f10x.h"





#line 2126 ".\\System\\CM3\\stm32f10x.h"

#line 2133 ".\\System\\CM3\\stm32f10x.h"

#line 2143 ".\\System\\CM3\\stm32f10x.h"











 








 








   
#line 2182 ".\\System\\CM3\\stm32f10x.h"

#line 2277 ".\\System\\CM3\\stm32f10x.h"

#line 2304 ".\\System\\CM3\\stm32f10x.h"
 
 
 
 
 
 

 




































































 




































































 
#line 2466 ".\\System\\CM3\\stm32f10x.h"

 
#line 2484 ".\\System\\CM3\\stm32f10x.h"

 
#line 2502 ".\\System\\CM3\\stm32f10x.h"

#line 2519 ".\\System\\CM3\\stm32f10x.h"

 
#line 2537 ".\\System\\CM3\\stm32f10x.h"

 
#line 2556 ".\\System\\CM3\\stm32f10x.h"

 

 






 
#line 2583 ".\\System\\CM3\\stm32f10x.h"






 








 









 








 








 









 










 




#line 2658 ".\\System\\CM3\\stm32f10x.h"

 










#line 2689 ".\\System\\CM3\\stm32f10x.h"

 





 
#line 2704 ".\\System\\CM3\\stm32f10x.h"

 
#line 2713 ".\\System\\CM3\\stm32f10x.h"

   
#line 2722 ".\\System\\CM3\\stm32f10x.h"

 
#line 2731 ".\\System\\CM3\\stm32f10x.h"

 





 
#line 2746 ".\\System\\CM3\\stm32f10x.h"

 
#line 2755 ".\\System\\CM3\\stm32f10x.h"

   
#line 2764 ".\\System\\CM3\\stm32f10x.h"

 
#line 2773 ".\\System\\CM3\\stm32f10x.h"

 





 
#line 2788 ".\\System\\CM3\\stm32f10x.h"

 
#line 2797 ".\\System\\CM3\\stm32f10x.h"

   
#line 2806 ".\\System\\CM3\\stm32f10x.h"

 
#line 2815 ".\\System\\CM3\\stm32f10x.h"

 





 
#line 2830 ".\\System\\CM3\\stm32f10x.h"

 
#line 2839 ".\\System\\CM3\\stm32f10x.h"

   
#line 2848 ".\\System\\CM3\\stm32f10x.h"

 
#line 2857 ".\\System\\CM3\\stm32f10x.h"

#line 2866 ".\\System\\CM3\\stm32f10x.h"

#line 2875 ".\\System\\CM3\\stm32f10x.h"

#line 2885 ".\\System\\CM3\\stm32f10x.h"

 
 
 
 
 

 





 


 


 




 
 
 
 
 

 
#line 2949 ".\\System\\CM3\\stm32f10x.h"

 
#line 2984 ".\\System\\CM3\\stm32f10x.h"

 
#line 3019 ".\\System\\CM3\\stm32f10x.h"

 
#line 3054 ".\\System\\CM3\\stm32f10x.h"

 
#line 3089 ".\\System\\CM3\\stm32f10x.h"

 





 





 





 





 





 





 





 





 






 
#line 3156 ".\\System\\CM3\\stm32f10x.h"

 



 









 
#line 3180 ".\\System\\CM3\\stm32f10x.h"




 




 
#line 3196 ".\\System\\CM3\\stm32f10x.h"

 





 
#line 3218 ".\\System\\CM3\\stm32f10x.h"

 
 





 
#line 3233 ".\\System\\CM3\\stm32f10x.h"
 
#line 3240 ".\\System\\CM3\\stm32f10x.h"

 




 






 


 


 


 
 
 
 
 

 
#line 3289 ".\\System\\CM3\\stm32f10x.h"

 
#line 3311 ".\\System\\CM3\\stm32f10x.h"

 
#line 3333 ".\\System\\CM3\\stm32f10x.h"

 
#line 3355 ".\\System\\CM3\\stm32f10x.h"

 
#line 3377 ".\\System\\CM3\\stm32f10x.h"

 
#line 3399 ".\\System\\CM3\\stm32f10x.h"

 
 
 
 
 

 
#line 3435 ".\\System\\CM3\\stm32f10x.h"

 
#line 3465 ".\\System\\CM3\\stm32f10x.h"

 
#line 3475 ".\\System\\CM3\\stm32f10x.h"















 
#line 3499 ".\\System\\CM3\\stm32f10x.h"















 
#line 3523 ".\\System\\CM3\\stm32f10x.h"















 
#line 3547 ".\\System\\CM3\\stm32f10x.h"















 
#line 3571 ".\\System\\CM3\\stm32f10x.h"















 
#line 3595 ".\\System\\CM3\\stm32f10x.h"















 
#line 3619 ".\\System\\CM3\\stm32f10x.h"















 


 


 


 


 


 


 


 


 


 



 


 


 



 


 


 


 



 


 


 


 


 
 
 
 
 

 






 
#line 3720 ".\\System\\CM3\\stm32f10x.h"

#line 3729 ".\\System\\CM3\\stm32f10x.h"















  
 
#line 3752 ".\\System\\CM3\\stm32f10x.h"


















 








































 


















































 


 


 


 


 


 


 
#line 3887 ".\\System\\CM3\\stm32f10x.h"

#line 3894 ".\\System\\CM3\\stm32f10x.h"

#line 3901 ".\\System\\CM3\\stm32f10x.h"

#line 3908 ".\\System\\CM3\\stm32f10x.h"







 
#line 3922 ".\\System\\CM3\\stm32f10x.h"

#line 3929 ".\\System\\CM3\\stm32f10x.h"

#line 3936 ".\\System\\CM3\\stm32f10x.h"

#line 3943 ".\\System\\CM3\\stm32f10x.h"

#line 3950 ".\\System\\CM3\\stm32f10x.h"

#line 3957 ".\\System\\CM3\\stm32f10x.h"

 
#line 3965 ".\\System\\CM3\\stm32f10x.h"

#line 3972 ".\\System\\CM3\\stm32f10x.h"

#line 3979 ".\\System\\CM3\\stm32f10x.h"

#line 3986 ".\\System\\CM3\\stm32f10x.h"

#line 3993 ".\\System\\CM3\\stm32f10x.h"

#line 4000 ".\\System\\CM3\\stm32f10x.h"

 
#line 4008 ".\\System\\CM3\\stm32f10x.h"

#line 4015 ".\\System\\CM3\\stm32f10x.h"

#line 4022 ".\\System\\CM3\\stm32f10x.h"

#line 4029 ".\\System\\CM3\\stm32f10x.h"





 


 


 


 


 



 
 
 
 
 

 









































 



 


 


 


 


 


 


 



 



 



 


 


 



 
 
 
 
 
 





 






 


 
#line 4171 ".\\System\\CM3\\stm32f10x.h"

 
#line 4181 ".\\System\\CM3\\stm32f10x.h"

 


 


 
 
 
 
 

 
















 









#line 4229 ".\\System\\CM3\\stm32f10x.h"

 

























 
#line 4272 ".\\System\\CM3\\stm32f10x.h"

 
#line 4286 ".\\System\\CM3\\stm32f10x.h"

 
#line 4296 ".\\System\\CM3\\stm32f10x.h"

 




























 





















 




























 





















 
#line 4415 ".\\System\\CM3\\stm32f10x.h"

 


 


 


 


 


 


 


 


 
#line 4450 ".\\System\\CM3\\stm32f10x.h"





#line 4461 ".\\System\\CM3\\stm32f10x.h"

 
#line 4469 ".\\System\\CM3\\stm32f10x.h"

#line 4476 ".\\System\\CM3\\stm32f10x.h"

 


 
 
 
 
 

 




 
#line 4498 ".\\System\\CM3\\stm32f10x.h"

 


 


 


 


 


 


 


 


 
 
 
 
 

 


 





 


 



 
 
 
 
 

 
#line 4560 ".\\System\\CM3\\stm32f10x.h"



 
#line 4572 ".\\System\\CM3\\stm32f10x.h"







 


 
 
 
 
 

 











#line 4610 ".\\System\\CM3\\stm32f10x.h"

 











#line 4633 ".\\System\\CM3\\stm32f10x.h"

 











#line 4656 ".\\System\\CM3\\stm32f10x.h"

 











#line 4679 ".\\System\\CM3\\stm32f10x.h"

 








































 








































 








































 








































 


































 


































 


































 


































 



























 



























 



























 
#line 5076 ".\\System\\CM3\\stm32f10x.h"

 
#line 5085 ".\\System\\CM3\\stm32f10x.h"

 
#line 5094 ".\\System\\CM3\\stm32f10x.h"

 
#line 5105 ".\\System\\CM3\\stm32f10x.h"

#line 5115 ".\\System\\CM3\\stm32f10x.h"

#line 5125 ".\\System\\CM3\\stm32f10x.h"

#line 5135 ".\\System\\CM3\\stm32f10x.h"

 
#line 5146 ".\\System\\CM3\\stm32f10x.h"

#line 5156 ".\\System\\CM3\\stm32f10x.h"

#line 5166 ".\\System\\CM3\\stm32f10x.h"

#line 5176 ".\\System\\CM3\\stm32f10x.h"

 
#line 5187 ".\\System\\CM3\\stm32f10x.h"

#line 5197 ".\\System\\CM3\\stm32f10x.h"

#line 5207 ".\\System\\CM3\\stm32f10x.h"

#line 5217 ".\\System\\CM3\\stm32f10x.h"

 
#line 5228 ".\\System\\CM3\\stm32f10x.h"

#line 5238 ".\\System\\CM3\\stm32f10x.h"

#line 5248 ".\\System\\CM3\\stm32f10x.h"

#line 5258 ".\\System\\CM3\\stm32f10x.h"

 
#line 5269 ".\\System\\CM3\\stm32f10x.h"

#line 5279 ".\\System\\CM3\\stm32f10x.h"

#line 5289 ".\\System\\CM3\\stm32f10x.h"

#line 5299 ".\\System\\CM3\\stm32f10x.h"

 
#line 5310 ".\\System\\CM3\\stm32f10x.h"

#line 5320 ".\\System\\CM3\\stm32f10x.h"

#line 5330 ".\\System\\CM3\\stm32f10x.h"

#line 5340 ".\\System\\CM3\\stm32f10x.h"

 
#line 5351 ".\\System\\CM3\\stm32f10x.h"

#line 5361 ".\\System\\CM3\\stm32f10x.h"

#line 5371 ".\\System\\CM3\\stm32f10x.h"

#line 5381 ".\\System\\CM3\\stm32f10x.h"

 


 


 
 
 
 
 

 




 












 


 






#line 5429 ".\\System\\CM3\\stm32f10x.h"

 


 


 


 


 


 


 


 


 
















 


 
#line 5499 ".\\System\\CM3\\stm32f10x.h"

 
#line 5514 ".\\System\\CM3\\stm32f10x.h"

 
#line 5540 ".\\System\\CM3\\stm32f10x.h"

 


 


 
 
 
 
 

 
 























 























 























 























 























 























 























 























 
 
#line 5761 ".\\System\\CM3\\stm32f10x.h"

 
#line 5773 ".\\System\\CM3\\stm32f10x.h"

 






 
#line 5790 ".\\System\\CM3\\stm32f10x.h"



     


 
 


 


 


 


 


 


 


 


 

 


 


 


 


 


 


 


 


 

 


 


 


 


 


 


 


 


 


 


 


 


 


 


 


 


 

 


 


 


 


 


 


 


 


 

 


#line 5934 ".\\System\\CM3\\stm32f10x.h"



 


#line 5946 ".\\System\\CM3\\stm32f10x.h"



 


#line 5958 ".\\System\\CM3\\stm32f10x.h"



 


#line 5970 ".\\System\\CM3\\stm32f10x.h"



 


#line 5982 ".\\System\\CM3\\stm32f10x.h"



 


#line 5994 ".\\System\\CM3\\stm32f10x.h"



 


#line 6006 ".\\System\\CM3\\stm32f10x.h"



 


#line 6018 ".\\System\\CM3\\stm32f10x.h"



 

 


#line 6032 ".\\System\\CM3\\stm32f10x.h"



 


#line 6044 ".\\System\\CM3\\stm32f10x.h"



 


#line 6056 ".\\System\\CM3\\stm32f10x.h"



 


#line 6068 ".\\System\\CM3\\stm32f10x.h"



 


#line 6080 ".\\System\\CM3\\stm32f10x.h"



 


#line 6092 ".\\System\\CM3\\stm32f10x.h"



 


#line 6104 ".\\System\\CM3\\stm32f10x.h"



 


#line 6116 ".\\System\\CM3\\stm32f10x.h"



 


#line 6128 ".\\System\\CM3\\stm32f10x.h"



 


#line 6140 ".\\System\\CM3\\stm32f10x.h"



 


#line 6152 ".\\System\\CM3\\stm32f10x.h"



 


#line 6164 ".\\System\\CM3\\stm32f10x.h"



 


#line 6176 ".\\System\\CM3\\stm32f10x.h"



 


#line 6188 ".\\System\\CM3\\stm32f10x.h"



 


#line 6200 ".\\System\\CM3\\stm32f10x.h"



 


#line 6212 ".\\System\\CM3\\stm32f10x.h"



 
 
 
 
 

 
 
#line 6232 ".\\System\\CM3\\stm32f10x.h"

 
#line 6243 ".\\System\\CM3\\stm32f10x.h"

 
#line 6261 ".\\System\\CM3\\stm32f10x.h"











 





 





 
#line 6299 ".\\System\\CM3\\stm32f10x.h"

 












 
#line 6320 ".\\System\\CM3\\stm32f10x.h"

 
 






 




 





 





 






 




 





 





 






   




 





 





 





 




 





 





 





 




 





 





 
 


 
#line 6460 ".\\System\\CM3\\stm32f10x.h"

 
#line 6477 ".\\System\\CM3\\stm32f10x.h"

 
#line 6494 ".\\System\\CM3\\stm32f10x.h"

 
#line 6511 ".\\System\\CM3\\stm32f10x.h"

 
#line 6545 ".\\System\\CM3\\stm32f10x.h"

 
#line 6579 ".\\System\\CM3\\stm32f10x.h"

 
#line 6613 ".\\System\\CM3\\stm32f10x.h"

 
#line 6647 ".\\System\\CM3\\stm32f10x.h"

 
#line 6681 ".\\System\\CM3\\stm32f10x.h"

 
#line 6715 ".\\System\\CM3\\stm32f10x.h"

 
#line 6749 ".\\System\\CM3\\stm32f10x.h"

 
#line 6783 ".\\System\\CM3\\stm32f10x.h"

 
#line 6817 ".\\System\\CM3\\stm32f10x.h"

 
#line 6851 ".\\System\\CM3\\stm32f10x.h"

 
#line 6885 ".\\System\\CM3\\stm32f10x.h"

 
#line 6919 ".\\System\\CM3\\stm32f10x.h"

 
#line 6953 ".\\System\\CM3\\stm32f10x.h"

 
#line 6987 ".\\System\\CM3\\stm32f10x.h"

 
#line 7021 ".\\System\\CM3\\stm32f10x.h"

 
#line 7055 ".\\System\\CM3\\stm32f10x.h"

 
#line 7089 ".\\System\\CM3\\stm32f10x.h"

 
#line 7123 ".\\System\\CM3\\stm32f10x.h"

 
#line 7157 ".\\System\\CM3\\stm32f10x.h"

 
#line 7191 ".\\System\\CM3\\stm32f10x.h"

 
#line 7225 ".\\System\\CM3\\stm32f10x.h"

 
#line 7259 ".\\System\\CM3\\stm32f10x.h"

 
#line 7293 ".\\System\\CM3\\stm32f10x.h"

 
#line 7327 ".\\System\\CM3\\stm32f10x.h"

 
#line 7361 ".\\System\\CM3\\stm32f10x.h"

 
#line 7395 ".\\System\\CM3\\stm32f10x.h"

 
#line 7429 ".\\System\\CM3\\stm32f10x.h"

 
#line 7463 ".\\System\\CM3\\stm32f10x.h"

 
 
 
 
 

 









#line 7490 ".\\System\\CM3\\stm32f10x.h"

 
#line 7498 ".\\System\\CM3\\stm32f10x.h"

 
#line 7508 ".\\System\\CM3\\stm32f10x.h"

 


 


 


 


 





















 




 
 
 
 
 

 
#line 7569 ".\\System\\CM3\\stm32f10x.h"

 
#line 7578 ".\\System\\CM3\\stm32f10x.h"







 



#line 7599 ".\\System\\CM3\\stm32f10x.h"



 



 


 
#line 7624 ".\\System\\CM3\\stm32f10x.h"

 
#line 7634 ".\\System\\CM3\\stm32f10x.h"

 




 


 
 
 
 
 

 
#line 7660 ".\\System\\CM3\\stm32f10x.h"

 


 



 
#line 7684 ".\\System\\CM3\\stm32f10x.h"

 
#line 7693 ".\\System\\CM3\\stm32f10x.h"







 
#line 7713 ".\\System\\CM3\\stm32f10x.h"

 
#line 7724 ".\\System\\CM3\\stm32f10x.h"



 
 
 
 
 

 


#line 7753 ".\\System\\CM3\\stm32f10x.h"

 









#line 7787 ".\\System\\CM3\\stm32f10x.h"

 
 
 
 
 

 









 


 


 





 
#line 7827 ".\\System\\CM3\\stm32f10x.h"

 


 









 


 

 



 



 



 



 



 



 



 



#line 8291 ".\\System\\CM3\\stm32f10x.h"



 

 

  

#line 1 ".\\System\\FWlib\\inc\\stm32f10x_conf.h"


















 

 



 
 
 
 
 
 
 
 
 
 
 
#line 1 ".\\System\\FWlib\\inc\\stm32f10x_fsmc.h"




















 

 







 
#line 1 ".\\System\\CM3\\stm32f10x.h"







































 



 



 
    
#line 8331 ".\\System\\CM3\\stm32f10x.h"



 

  

 

 
#line 33 ".\\System\\FWlib\\inc\\stm32f10x_fsmc.h"



 



 



 



 

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



 



 



 






 



   




 



     



 



















 



 








 



 

#line 317 ".\\System\\FWlib\\inc\\stm32f10x_fsmc.h"



 



 








 



 







 
  


 







 
  


 








 



 








 



 








 



 





                              


 



 







 



 









 



 







 



 





 



 





 



 





 



 





 



 





 



 





 



 

#line 521 ".\\System\\FWlib\\inc\\stm32f10x_fsmc.h"



 



 
  


 



 








 




 








 



 

#line 577 ".\\System\\FWlib\\inc\\stm32f10x_fsmc.h"



 



 





 



 





 



 





 



 





 



 





 



 





 



 

#line 653 ".\\System\\FWlib\\inc\\stm32f10x_fsmc.h"


 



 

#line 669 ".\\System\\FWlib\\inc\\stm32f10x_fsmc.h"





 



 



 



 



 



 

void FSMC_NORSRAMDeInit(uint32_t FSMC_Bank);
void FSMC_NANDDeInit(uint32_t FSMC_Bank);
void FSMC_PCCARDDeInit(void);
void FSMC_NORSRAMInit(FSMC_NORSRAMInitTypeDef* FSMC_NORSRAMInitStruct);
void FSMC_NANDInit(FSMC_NANDInitTypeDef* FSMC_NANDInitStruct);
void FSMC_PCCARDInit(FSMC_PCCARDInitTypeDef* FSMC_PCCARDInitStruct);
void FSMC_NORSRAMStructInit(FSMC_NORSRAMInitTypeDef* FSMC_NORSRAMInitStruct);
void FSMC_NANDStructInit(FSMC_NANDInitTypeDef* FSMC_NANDInitStruct);
void FSMC_PCCARDStructInit(FSMC_PCCARDInitTypeDef* FSMC_PCCARDInitStruct);
void FSMC_NORSRAMCmd(uint32_t FSMC_Bank, FunctionalState NewState);
void FSMC_NANDCmd(uint32_t FSMC_Bank, FunctionalState NewState);
void FSMC_PCCARDCmd(FunctionalState NewState);
void FSMC_NANDECCCmd(uint32_t FSMC_Bank, FunctionalState NewState);
uint32_t FSMC_GetECC(uint32_t FSMC_Bank);
void FSMC_ITConfig(uint32_t FSMC_Bank, uint32_t FSMC_IT, FunctionalState NewState);
FlagStatus FSMC_GetFlagStatus(uint32_t FSMC_Bank, uint32_t FSMC_FLAG);
void FSMC_ClearFlag(uint32_t FSMC_Bank, uint32_t FSMC_FLAG);
ITStatus FSMC_GetITStatus(uint32_t FSMC_Bank, uint32_t FSMC_IT);
void FSMC_ClearITPendingBit(uint32_t FSMC_Bank, uint32_t FSMC_IT);








 



 



  

 
#line 37 ".\\System\\FWlib\\inc\\stm32f10x_conf.h"
#line 1 ".\\System\\FWlib\\inc\\stm32f10x_gpio.h"




















 

 







 
#line 33 ".\\System\\FWlib\\inc\\stm32f10x_gpio.h"



 



 



 

#line 53 ".\\System\\FWlib\\inc\\stm32f10x_gpio.h"
                                     


 

typedef enum
{ 
  GPIO_Speed_10MHz = 1,
  GPIO_Speed_2MHz, 
  GPIO_Speed_50MHz
}GPIOSpeed_TypeDef;





 

typedef enum
{ GPIO_Mode_AIN = 0x0,
  GPIO_Mode_IN_FLOATING = 0x04,
  GPIO_Mode_IPD = 0x28,
  GPIO_Mode_IPU = 0x48,
  GPIO_Mode_Out_OD = 0x14,
  GPIO_Mode_Out_PP = 0x10,
  GPIO_Mode_AF_OD = 0x1C,
  GPIO_Mode_AF_PP = 0x18
}GPIOMode_TypeDef;








 

typedef struct
{
  uint16_t GPIO_Pin;             
 

  GPIOSpeed_TypeDef GPIO_Speed;  
 

  GPIOMode_TypeDef GPIO_Mode;    
 
}GPIO_InitTypeDef;




 

typedef enum
{ Bit_RESET = 0,
  Bit_SET
}BitAction;





 



 



 

#line 144 ".\\System\\FWlib\\inc\\stm32f10x_gpio.h"



#line 163 ".\\System\\FWlib\\inc\\stm32f10x_gpio.h"



 



 

#line 204 ".\\System\\FWlib\\inc\\stm32f10x_gpio.h"







#line 217 ".\\System\\FWlib\\inc\\stm32f10x_gpio.h"






#line 245 ".\\System\\FWlib\\inc\\stm32f10x_gpio.h"
                              


  



 

#line 266 ".\\System\\FWlib\\inc\\stm32f10x_gpio.h"

#line 274 ".\\System\\FWlib\\inc\\stm32f10x_gpio.h"



 



 

#line 299 ".\\System\\FWlib\\inc\\stm32f10x_gpio.h"

#line 316 ".\\System\\FWlib\\inc\\stm32f10x_gpio.h"



 



  








                                                 


 



 



 



 

void GPIO_DeInit(GPIO_TypeDef* GPIOx);
void GPIO_AFIODeInit(void);
void GPIO_Init(GPIO_TypeDef* GPIOx, GPIO_InitTypeDef* GPIO_InitStruct);
void GPIO_StructInit(GPIO_InitTypeDef* GPIO_InitStruct);
uint8_t GPIO_ReadInputDataBit(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin);
uint16_t GPIO_ReadInputData(GPIO_TypeDef* GPIOx);
uint8_t GPIO_ReadOutputDataBit(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin);
uint16_t GPIO_ReadOutputData(GPIO_TypeDef* GPIOx);
void GPIO_SetBits(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin);
void GPIO_ResetBits(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin);
void GPIO_WriteBit(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin, BitAction BitVal);
void GPIO_Write(GPIO_TypeDef* GPIOx, uint16_t PortVal);
void GPIO_PinLockConfig(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin);
void GPIO_EventOutputConfig(uint8_t GPIO_PortSource, uint8_t GPIO_PinSource);
void GPIO_EventOutputCmd(FunctionalState NewState);
void GPIO_PinRemapConfig(uint32_t GPIO_Remap, FunctionalState NewState);
void GPIO_EXTILineConfig(uint8_t GPIO_PortSource, uint8_t GPIO_PinSource);
void GPIO_ETH_MediaInterfaceConfig(uint32_t GPIO_ETH_MediaInterface);








 



 



 

 
#line 38 ".\\System\\FWlib\\inc\\stm32f10x_conf.h"
 
 
 
#line 1 ".\\System\\FWlib\\inc\\stm32f10x_rcc.h"




















 

 







 
#line 33 ".\\System\\FWlib\\inc\\stm32f10x_rcc.h"



 



 



 

typedef struct
{
  uint32_t SYSCLK_Frequency;   
  uint32_t HCLK_Frequency;     
  uint32_t PCLK1_Frequency;    
  uint32_t PCLK2_Frequency;    
  uint32_t ADCCLK_Frequency;   
}RCC_ClocksTypeDef;



 



 



 









  



 



#line 94 ".\\System\\FWlib\\inc\\stm32f10x_rcc.h"



  



 
#line 126 ".\\System\\FWlib\\inc\\stm32f10x_rcc.h"

#line 141 ".\\System\\FWlib\\inc\\stm32f10x_rcc.h"


 



 
#line 175 ".\\System\\FWlib\\inc\\stm32f10x_rcc.h"


 




 
#line 196 ".\\System\\FWlib\\inc\\stm32f10x_rcc.h"


 

#line 283 ".\\System\\FWlib\\inc\\stm32f10x_rcc.h"




 

#line 295 ".\\System\\FWlib\\inc\\stm32f10x_rcc.h"


 



 

#line 317 ".\\System\\FWlib\\inc\\stm32f10x_rcc.h"


  



 

#line 333 ".\\System\\FWlib\\inc\\stm32f10x_rcc.h"


 



 

#line 347 ".\\System\\FWlib\\inc\\stm32f10x_rcc.h"

#line 364 ".\\System\\FWlib\\inc\\stm32f10x_rcc.h"




 




 








 
#line 396 ".\\System\\FWlib\\inc\\stm32f10x_rcc.h"


#line 423 ".\\System\\FWlib\\inc\\stm32f10x_rcc.h"
  



 

#line 435 ".\\System\\FWlib\\inc\\stm32f10x_rcc.h"


 



 








 



 

#line 462 ".\\System\\FWlib\\inc\\stm32f10x_rcc.h"


 



 







#line 489 ".\\System\\FWlib\\inc\\stm32f10x_rcc.h"


 



 

#line 518 ".\\System\\FWlib\\inc\\stm32f10x_rcc.h"




  



 

#line 553 ".\\System\\FWlib\\inc\\stm32f10x_rcc.h"
 




 



 







#line 586 ".\\System\\FWlib\\inc\\stm32f10x_rcc.h"



 



 

#line 606 ".\\System\\FWlib\\inc\\stm32f10x_rcc.h"

#line 625 ".\\System\\FWlib\\inc\\stm32f10x_rcc.h"




 



 



 



 



 

void RCC_DeInit(void);
void RCC_HSEConfig(uint32_t RCC_HSE);
ErrorStatus RCC_WaitForHSEStartUp(void);
void RCC_AdjustHSICalibrationValue(uint8_t HSICalibrationValue);
void RCC_HSICmd(FunctionalState NewState);
void RCC_PLLConfig(uint32_t RCC_PLLSource, uint32_t RCC_PLLMul);
void RCC_PLLCmd(FunctionalState NewState);





#line 666 ".\\System\\FWlib\\inc\\stm32f10x_rcc.h"

void RCC_SYSCLKConfig(uint32_t RCC_SYSCLKSource);
uint8_t RCC_GetSYSCLKSource(void);
void RCC_HCLKConfig(uint32_t RCC_SYSCLK);
void RCC_PCLK1Config(uint32_t RCC_HCLK);
void RCC_PCLK2Config(uint32_t RCC_HCLK);
void RCC_ITConfig(uint8_t RCC_IT, FunctionalState NewState);


 void RCC_USBCLKConfig(uint32_t RCC_USBCLKSource);




void RCC_ADCCLKConfig(uint32_t RCC_PCLK2);






void RCC_LSEConfig(uint8_t RCC_LSE);
void RCC_LSICmd(FunctionalState NewState);
void RCC_RTCCLKConfig(uint32_t RCC_RTCCLKSource);
void RCC_RTCCLKCmd(FunctionalState NewState);
void RCC_GetClocksFreq(RCC_ClocksTypeDef* RCC_Clocks);
void RCC_AHBPeriphClockCmd(uint32_t RCC_AHBPeriph, FunctionalState NewState);
void RCC_APB2PeriphClockCmd(uint32_t RCC_APB2Periph, FunctionalState NewState);
void RCC_APB1PeriphClockCmd(uint32_t RCC_APB1Periph, FunctionalState NewState);





void RCC_APB2PeriphResetCmd(uint32_t RCC_APB2Periph, FunctionalState NewState);
void RCC_APB1PeriphResetCmd(uint32_t RCC_APB1Periph, FunctionalState NewState);
void RCC_BackupResetCmd(FunctionalState NewState);
void RCC_ClockSecuritySystemCmd(FunctionalState NewState);
void RCC_MCOConfig(uint8_t RCC_MCO);
FlagStatus RCC_GetFlagStatus(uint8_t RCC_FLAG);
void RCC_ClearFlag(void);
ITStatus RCC_GetITStatus(uint8_t RCC_IT);
void RCC_ClearITPendingBit(uint8_t RCC_IT);








 



 



  

 
#line 42 ".\\System\\FWlib\\inc\\stm32f10x_conf.h"
 
 
#line 1 ".\\System\\FWlib\\inc\\stm32f10x_spi.h"




















 

 







 
#line 33 ".\\System\\FWlib\\inc\\stm32f10x_spi.h"



 



  



 



 

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



 



 










 
  
#line 136 ".\\System\\FWlib\\inc\\stm32f10x_spi.h"


 



 







 



 







  



 







 



 







 



 







  



 

#line 220 ".\\System\\FWlib\\inc\\stm32f10x_spi.h"


  



 







 



 

#line 248 ".\\System\\FWlib\\inc\\stm32f10x_spi.h"


 



 

#line 266 ".\\System\\FWlib\\inc\\stm32f10x_spi.h"


 



 

#line 282 ".\\System\\FWlib\\inc\\stm32f10x_spi.h"


  



 







 



 

#line 312 ".\\System\\FWlib\\inc\\stm32f10x_spi.h"






  



 







 



 






 



 







 



 






 



 







 



 

#line 396 ".\\System\\FWlib\\inc\\stm32f10x_spi.h"


 



 

#line 417 ".\\System\\FWlib\\inc\\stm32f10x_spi.h"


 



 




 



 



 



 



 

void SPI_I2S_DeInit(SPI_TypeDef* SPIx);
void SPI_Init(SPI_TypeDef* SPIx, SPI_InitTypeDef* SPI_InitStruct);
void I2S_Init(SPI_TypeDef* SPIx, I2S_InitTypeDef* I2S_InitStruct);
void SPI_StructInit(SPI_InitTypeDef* SPI_InitStruct);
void I2S_StructInit(I2S_InitTypeDef* I2S_InitStruct);
void SPI_Cmd(SPI_TypeDef* SPIx, FunctionalState NewState);
void I2S_Cmd(SPI_TypeDef* SPIx, FunctionalState NewState);
void SPI_I2S_ITConfig(SPI_TypeDef* SPIx, uint8_t SPI_I2S_IT, FunctionalState NewState);
void SPI_I2S_DMACmd(SPI_TypeDef* SPIx, uint16_t SPI_I2S_DMAReq, FunctionalState NewState);
void SPI_I2S_SendData(SPI_TypeDef* SPIx, uint16_t Data);
uint16_t SPI_I2S_ReceiveData(SPI_TypeDef* SPIx);
void SPI_NSSInternalSoftwareConfig(SPI_TypeDef* SPIx, uint16_t SPI_NSSInternalSoft);
void SPI_SSOutputCmd(SPI_TypeDef* SPIx, FunctionalState NewState);
void SPI_DataSizeConfig(SPI_TypeDef* SPIx, uint16_t SPI_DataSize);
void SPI_TransmitCRC(SPI_TypeDef* SPIx);
void SPI_CalculateCRC(SPI_TypeDef* SPIx, FunctionalState NewState);
uint16_t SPI_GetCRC(SPI_TypeDef* SPIx, uint8_t SPI_CRC);
uint16_t SPI_GetCRCPolynomial(SPI_TypeDef* SPIx);
void SPI_BiDirectionalLineConfig(SPI_TypeDef* SPIx, uint16_t SPI_Direction);
FlagStatus SPI_I2S_GetFlagStatus(SPI_TypeDef* SPIx, uint16_t SPI_I2S_FLAG);
void SPI_I2S_ClearFlag(SPI_TypeDef* SPIx, uint16_t SPI_I2S_FLAG);
ITStatus SPI_I2S_GetITStatus(SPI_TypeDef* SPIx, uint8_t SPI_I2S_IT);
void SPI_I2S_ClearITPendingBit(SPI_TypeDef* SPIx, uint8_t SPI_I2S_IT);








 



 



 

 
#line 45 ".\\System\\FWlib\\inc\\stm32f10x_conf.h"
 
 
 
#line 1 ".\\System\\FWlib\\inc\\misc.h"




















 

 







 
#line 33 ".\\System\\FWlib\\inc\\misc.h"
#line 1 ".\\Main\\comDef.h"





 



 
#line 12 ".\\Main\\comDef.h"
#line 1 ".\\System\\FWlib\\inc\\stm32f10x_adc.h"




















 

 







 
#line 33 ".\\System\\FWlib\\inc\\stm32f10x_adc.h"



 



 



 



 

typedef struct
{
  uint32_t ADC_Mode;                      

 

  FunctionalState ADC_ScanConvMode;       

 

  FunctionalState ADC_ContinuousConvMode; 

 

  uint32_t ADC_ExternalTrigConv;          

 

  uint32_t ADC_DataAlign;                 
 

  uint8_t ADC_NbrOfChannel;               

 
}ADC_InitTypeDef;


 



 










 

#line 104 ".\\System\\FWlib\\inc\\stm32f10x_adc.h"

#line 115 ".\\System\\FWlib\\inc\\stm32f10x_adc.h"


 



 

#line 129 ".\\System\\FWlib\\inc\\stm32f10x_adc.h"




#line 139 ".\\System\\FWlib\\inc\\stm32f10x_adc.h"

#line 154 ".\\System\\FWlib\\inc\\stm32f10x_adc.h"


 



 







 



 

#line 192 ".\\System\\FWlib\\inc\\stm32f10x_adc.h"




#line 205 ".\\System\\FWlib\\inc\\stm32f10x_adc.h"


 



 

#line 229 ".\\System\\FWlib\\inc\\stm32f10x_adc.h"


 



 

















#line 266 ".\\System\\FWlib\\inc\\stm32f10x_adc.h"


 



 

#line 282 ".\\System\\FWlib\\inc\\stm32f10x_adc.h"


 



 

#line 297 ".\\System\\FWlib\\inc\\stm32f10x_adc.h"

#line 305 ".\\System\\FWlib\\inc\\stm32f10x_adc.h"


 



 











 



 

#line 338 ".\\System\\FWlib\\inc\\stm32f10x_adc.h"


 



 





 



 





 



 





 



 





  




 




 



 





 



 





 



 



 



 



 

void ADC_DeInit(ADC_TypeDef* ADCx);
void ADC_Init(ADC_TypeDef* ADCx, ADC_InitTypeDef* ADC_InitStruct);
void ADC_StructInit(ADC_InitTypeDef* ADC_InitStruct);
void ADC_Cmd(ADC_TypeDef* ADCx, FunctionalState NewState);
void ADC_DMACmd(ADC_TypeDef* ADCx, FunctionalState NewState);
void ADC_ITConfig(ADC_TypeDef* ADCx, uint16_t ADC_IT, FunctionalState NewState);
void ADC_ResetCalibration(ADC_TypeDef* ADCx);
FlagStatus ADC_GetResetCalibrationStatus(ADC_TypeDef* ADCx);
void ADC_StartCalibration(ADC_TypeDef* ADCx);
FlagStatus ADC_GetCalibrationStatus(ADC_TypeDef* ADCx);
void ADC_SoftwareStartConvCmd(ADC_TypeDef* ADCx, FunctionalState NewState);
FlagStatus ADC_GetSoftwareStartConvStatus(ADC_TypeDef* ADCx);
void ADC_DiscModeChannelCountConfig(ADC_TypeDef* ADCx, uint8_t Number);
void ADC_DiscModeCmd(ADC_TypeDef* ADCx, FunctionalState NewState);
void ADC_RegularChannelConfig(ADC_TypeDef* ADCx, uint8_t ADC_Channel, uint8_t Rank, uint8_t ADC_SampleTime);
void ADC_ExternalTrigConvCmd(ADC_TypeDef* ADCx, FunctionalState NewState);
uint16_t ADC_GetConversionValue(ADC_TypeDef* ADCx);
uint32_t ADC_GetDualModeConversionValue(void);
void ADC_AutoInjectedConvCmd(ADC_TypeDef* ADCx, FunctionalState NewState);
void ADC_InjectedDiscModeCmd(ADC_TypeDef* ADCx, FunctionalState NewState);
void ADC_ExternalTrigInjectedConvConfig(ADC_TypeDef* ADCx, uint32_t ADC_ExternalTrigInjecConv);
void ADC_ExternalTrigInjectedConvCmd(ADC_TypeDef* ADCx, FunctionalState NewState);
void ADC_SoftwareStartInjectedConvCmd(ADC_TypeDef* ADCx, FunctionalState NewState);
FlagStatus ADC_GetSoftwareStartInjectedConvCmdStatus(ADC_TypeDef* ADCx);
void ADC_InjectedChannelConfig(ADC_TypeDef* ADCx, uint8_t ADC_Channel, uint8_t Rank, uint8_t ADC_SampleTime);
void ADC_InjectedSequencerLengthConfig(ADC_TypeDef* ADCx, uint8_t Length);
void ADC_SetInjectedOffset(ADC_TypeDef* ADCx, uint8_t ADC_InjectedChannel, uint16_t Offset);
uint16_t ADC_GetInjectedConversionValue(ADC_TypeDef* ADCx, uint8_t ADC_InjectedChannel);
void ADC_AnalogWatchdogCmd(ADC_TypeDef* ADCx, uint32_t ADC_AnalogWatchdog);
void ADC_AnalogWatchdogThresholdsConfig(ADC_TypeDef* ADCx, uint16_t HighThreshold, uint16_t LowThreshold);
void ADC_AnalogWatchdogSingleChannelConfig(ADC_TypeDef* ADCx, uint8_t ADC_Channel);
void ADC_TempSensorVrefintCmd(FunctionalState NewState);
FlagStatus ADC_GetFlagStatus(ADC_TypeDef* ADCx, uint8_t ADC_FLAG);
void ADC_ClearFlag(ADC_TypeDef* ADCx, uint8_t ADC_FLAG);
ITStatus ADC_GetITStatus(ADC_TypeDef* ADCx, uint16_t ADC_IT);
void ADC_ClearITPendingBit(ADC_TypeDef* ADCx, uint16_t ADC_IT);









 



 



 

 
#line 13 ".\\Main\\comDef.h"
#line 1 ".\\System\\FWlib\\inc\\stm32f10x_bkp.h"




















 

 







 
#line 33 ".\\System\\FWlib\\inc\\stm32f10x_bkp.h"



 



 



 



 



 



 







 



 

#line 78 ".\\System\\FWlib\\inc\\stm32f10x_bkp.h"


 



 

#line 128 ".\\System\\FWlib\\inc\\stm32f10x_bkp.h"

#line 143 ".\\System\\FWlib\\inc\\stm32f10x_bkp.h"




 



 



 



 



 

void BKP_DeInit(void);
void BKP_TamperPinLevelConfig(uint16_t BKP_TamperPinLevel);
void BKP_TamperPinCmd(FunctionalState NewState);
void BKP_ITConfig(FunctionalState NewState);
void BKP_RTCOutputConfig(uint16_t BKP_RTCOutputSource);
void BKP_SetRTCCalibrationValue(uint8_t CalibrationValue);
void BKP_WriteBackupRegister(uint16_t BKP_DR, uint16_t Data);
uint16_t BKP_ReadBackupRegister(uint16_t BKP_DR);
FlagStatus BKP_GetFlagStatus(void);
void BKP_ClearFlag(void);
ITStatus BKP_GetITStatus(void);
void BKP_ClearITPendingBit(void);








 



 



 

 
#line 14 ".\\Main\\comDef.h"
#line 1 ".\\System\\FWlib\\inc\\stm32f10x_can.h"




















 

 







 
#line 33 ".\\System\\FWlib\\inc\\stm32f10x_can.h"



 



 



 






 

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



 



 



 






 



 












 





   










 
  



   







 



 










 



 

#line 301 ".\\System\\FWlib\\inc\\stm32f10x_can.h"




 



 

#line 319 ".\\System\\FWlib\\inc\\stm32f10x_can.h"





 



 





 



 







 



 








 



 









 



 







 



 



 



 








 



 







 



 







 



 








 



 








 



 






 



 






 




   
                                                                
#line 493 ".\\System\\FWlib\\inc\\stm32f10x_can.h"




 



 

 
 

 




 
#line 518 ".\\System\\FWlib\\inc\\stm32f10x_can.h"

 



 

 





#line 539 ".\\System\\FWlib\\inc\\stm32f10x_can.h"








 

  


 


  


 
#line 565 ".\\System\\FWlib\\inc\\stm32f10x_can.h"

 



 






 





#line 590 ".\\System\\FWlib\\inc\\stm32f10x_can.h"

#line 597 ".\\System\\FWlib\\inc\\stm32f10x_can.h"



 



 
#line 621 ".\\System\\FWlib\\inc\\stm32f10x_can.h"



 



 



 



 



 
  
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








 



 



 

 
#line 15 ".\\Main\\comDef.h"
#line 1 ".\\System\\FWlib\\inc\\stm32f10x_cec.h"




















 

 







 
#line 33 ".\\System\\FWlib\\inc\\stm32f10x_cec.h"



 



 
  



 
   


  
typedef struct
{
  uint16_t CEC_BitTimingMode; 
 
  uint16_t CEC_BitPeriodMode; 
 
}CEC_InitTypeDef;



 



  
  


  







 



  







  




  
#line 100 ".\\System\\FWlib\\inc\\stm32f10x_cec.h"


  




  



  



  




 



 
   


  
#line 136 ".\\System\\FWlib\\inc\\stm32f10x_cec.h"



  
#line 147 ".\\System\\FWlib\\inc\\stm32f10x_cec.h"


                               
#line 157 ".\\System\\FWlib\\inc\\stm32f10x_cec.h"



  



  



 
 


 



  
void CEC_DeInit(void);
void CEC_Init(CEC_InitTypeDef* CEC_InitStruct);
void CEC_Cmd(FunctionalState NewState);
void CEC_ITConfig(FunctionalState NewState);
void CEC_OwnAddressConfig(uint8_t CEC_OwnAddress);
void CEC_SetPrescaler(uint16_t CEC_Prescaler);
void CEC_SendDataByte(uint8_t Data);
uint8_t CEC_ReceiveDataByte(void);
void CEC_StartOfMessage(void);
void CEC_EndOfMessageCmd(FunctionalState NewState);
FlagStatus CEC_GetFlagStatus(uint32_t CEC_FLAG);
void CEC_ClearFlag(uint32_t CEC_FLAG);
ITStatus CEC_GetITStatus(uint8_t CEC_IT);
void CEC_ClearITPendingBit(uint16_t CEC_IT);









  



  



  

 
#line 16 ".\\Main\\comDef.h"
#line 1 ".\\System\\FWlib\\inc\\stm32f10x_conf.h"


















 

 
#line 75 ".\\System\\FWlib\\inc\\stm32f10x_conf.h"

 
#line 17 ".\\Main\\comDef.h"
#line 1 ".\\System\\FWlib\\inc\\stm32f10x_crc.h"




















 

 







 
#line 33 ".\\System\\FWlib\\inc\\stm32f10x_crc.h"



 



 



 



 



 



 



 



 



 

void CRC_ResetDR(void);
uint32_t CRC_CalcCRC(uint32_t Data);
uint32_t CRC_CalcBlockCRC(uint32_t pBuffer[], uint32_t BufferLength);
uint32_t CRC_GetCRC(void);
void CRC_SetIDRegister(uint8_t IDValue);
uint8_t CRC_GetIDRegister(void);








 



 



 

 
#line 18 ".\\Main\\comDef.h"
#line 1 ".\\System\\FWlib\\inc\\stm32f10x_dac.h"




















 

 







 
#line 33 ".\\System\\FWlib\\inc\\stm32f10x_dac.h"



 



 



 



 

typedef struct
{
  uint32_t DAC_Trigger;                      
 

  uint32_t DAC_WaveGeneration;               

 

  uint32_t DAC_LFSRUnmask_TriangleAmplitude; 

 

  uint32_t DAC_OutputBuffer;                 
 
}DAC_InitTypeDef;



 



 



 

#line 94 ".\\System\\FWlib\\inc\\stm32f10x_dac.h"

#line 104 ".\\System\\FWlib\\inc\\stm32f10x_dac.h"



 



 

#line 119 ".\\System\\FWlib\\inc\\stm32f10x_dac.h"


 



 

#line 151 ".\\System\\FWlib\\inc\\stm32f10x_dac.h"

#line 176 ".\\System\\FWlib\\inc\\stm32f10x_dac.h"


 



 







 



 







 



 

#line 214 ".\\System\\FWlib\\inc\\stm32f10x_dac.h"


 



 







 



 




 
#line 261 ".\\System\\FWlib\\inc\\stm32f10x_dac.h"



 



 



 



 

void DAC_DeInit(void);
void DAC_Init(uint32_t DAC_Channel, DAC_InitTypeDef* DAC_InitStruct);
void DAC_StructInit(DAC_InitTypeDef* DAC_InitStruct);
void DAC_Cmd(uint32_t DAC_Channel, FunctionalState NewState);



void DAC_DMACmd(uint32_t DAC_Channel, FunctionalState NewState);
void DAC_SoftwareTriggerCmd(uint32_t DAC_Channel, FunctionalState NewState);
void DAC_DualSoftwareTriggerCmd(FunctionalState NewState);
void DAC_WaveGenerationCmd(uint32_t DAC_Channel, uint32_t DAC_Wave, FunctionalState NewState);
void DAC_SetChannel1Data(uint32_t DAC_Align, uint16_t Data);
void DAC_SetChannel2Data(uint32_t DAC_Align, uint16_t Data);
void DAC_SetDualChannelData(uint32_t DAC_Align, uint16_t Data2, uint16_t Data1);
uint16_t DAC_GetDataOutputValue(uint32_t DAC_Channel);
#line 299 ".\\System\\FWlib\\inc\\stm32f10x_dac.h"








 



 



 

 
#line 19 ".\\Main\\comDef.h"
#line 1 ".\\System\\FWlib\\inc\\stm32f10x_dma.h"




















 

 







 
#line 33 ".\\System\\FWlib\\inc\\stm32f10x_dma.h"



 



 



 



 

typedef struct
{
  uint32_t DMA_PeripheralBaseAddr;  

  uint32_t DMA_MemoryBaseAddr;      

  uint32_t DMA_DIR;                
 

  uint32_t DMA_BufferSize;         

 

  uint32_t DMA_PeripheralInc;      
 

  uint32_t DMA_MemoryInc;          
 

  uint32_t DMA_PeripheralDataSize; 
 

  uint32_t DMA_MemoryDataSize;     
 

  uint32_t DMA_Mode;               


 

  uint32_t DMA_Priority;           
 

  uint32_t DMA_M2M;                
 
}DMA_InitTypeDef;



 



 

#line 107 ".\\System\\FWlib\\inc\\stm32f10x_dma.h"



 







 



 







 



 







 



 

#line 154 ".\\System\\FWlib\\inc\\stm32f10x_dma.h"


 



 

#line 168 ".\\System\\FWlib\\inc\\stm32f10x_dma.h"


 



 






 



 

#line 195 ".\\System\\FWlib\\inc\\stm32f10x_dma.h"


 



 







 



 






#line 248 ".\\System\\FWlib\\inc\\stm32f10x_dma.h"

#line 269 ".\\System\\FWlib\\inc\\stm32f10x_dma.h"



#line 296 ".\\System\\FWlib\\inc\\stm32f10x_dma.h"



 



 
#line 332 ".\\System\\FWlib\\inc\\stm32f10x_dma.h"

#line 353 ".\\System\\FWlib\\inc\\stm32f10x_dma.h"



#line 380 ".\\System\\FWlib\\inc\\stm32f10x_dma.h"


 



 





 



 



 



 



 

void DMA_DeInit(DMA_Channel_TypeDef* DMAy_Channelx);
void DMA_Init(DMA_Channel_TypeDef* DMAy_Channelx, DMA_InitTypeDef* DMA_InitStruct);
void DMA_StructInit(DMA_InitTypeDef* DMA_InitStruct);
void DMA_Cmd(DMA_Channel_TypeDef* DMAy_Channelx, FunctionalState NewState);
void DMA_ITConfig(DMA_Channel_TypeDef* DMAy_Channelx, uint32_t DMA_IT, FunctionalState NewState);
void DMA_SetCurrDataCounter(DMA_Channel_TypeDef* DMAy_Channelx, uint16_t DataNumber); 
uint16_t DMA_GetCurrDataCounter(DMA_Channel_TypeDef* DMAy_Channelx);
FlagStatus DMA_GetFlagStatus(uint32_t DMAy_FLAG);
void DMA_ClearFlag(uint32_t DMAy_FLAG);
ITStatus DMA_GetITStatus(uint32_t DMAy_IT);
void DMA_ClearITPendingBit(uint32_t DMAy_IT);








 



 



 

 
#line 20 ".\\Main\\comDef.h"
#line 1 ".\\System\\FWlib\\inc\\stm32f10x_exti.h"




















 

 







 
#line 33 ".\\System\\FWlib\\inc\\stm32f10x_exti.h"



 



 



 



 

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



 



 



 

#line 124 ".\\System\\FWlib\\inc\\stm32f10x_exti.h"
                                          
#line 136 ".\\System\\FWlib\\inc\\stm32f10x_exti.h"

                    


 



 



 



 



 

void EXTI_DeInit(void);
void EXTI_Init(EXTI_InitTypeDef* EXTI_InitStruct);
void EXTI_StructInit(EXTI_InitTypeDef* EXTI_InitStruct);
void EXTI_GenerateSWInterrupt(uint32_t EXTI_Line);
FlagStatus EXTI_GetFlagStatus(uint32_t EXTI_Line);
void EXTI_ClearFlag(uint32_t EXTI_Line);
ITStatus EXTI_GetITStatus(uint32_t EXTI_Line);
void EXTI_ClearITPendingBit(uint32_t EXTI_Line);








 



 



 

 
#line 21 ".\\Main\\comDef.h"
#line 1 ".\\System\\FWlib\\inc\\stm32f10x_flash.h"




















 

 







 
#line 33 ".\\System\\FWlib\\inc\\stm32f10x_flash.h"



 



 



 



 

typedef enum
{ 
  FLASH_BUSY = 1,
  FLASH_ERROR_PG,
  FLASH_ERROR_WRP,
  FLASH_COMPLETE,
  FLASH_TIMEOUT
}FLASH_Status;



 



 



 

#line 77 ".\\System\\FWlib\\inc\\stm32f10x_flash.h"


 



 







 



 







 



 

 
#line 118 ".\\System\\FWlib\\inc\\stm32f10x_flash.h"

 
#line 144 ".\\System\\FWlib\\inc\\stm32f10x_flash.h"

 
#line 211 ".\\System\\FWlib\\inc\\stm32f10x_flash.h"











 



 







 



 







 



 





#line 270 ".\\System\\FWlib\\inc\\stm32f10x_flash.h"


 


 
#line 291 ".\\System\\FWlib\\inc\\stm32f10x_flash.h"






 



 
#line 333 ".\\System\\FWlib\\inc\\stm32f10x_flash.h"





 
#line 346 ".\\System\\FWlib\\inc\\stm32f10x_flash.h"



 



 



 



 



 

 
void FLASH_SetLatency(uint32_t FLASH_Latency);
void FLASH_HalfCycleAccessCmd(uint32_t FLASH_HalfCycleAccess);
void FLASH_PrefetchBufferCmd(uint32_t FLASH_PrefetchBuffer);
void FLASH_Unlock(void);
void FLASH_Lock(void);
FLASH_Status FLASH_ErasePage(uint32_t Page_Address);
FLASH_Status FLASH_EraseAllPages(void);
FLASH_Status FLASH_EraseOptionBytes(void);
FLASH_Status FLASH_ProgramWord(uint32_t Address, uint32_t Data);
FLASH_Status FLASH_ProgramHalfWord(uint32_t Address, uint16_t Data);
FLASH_Status FLASH_ProgramOptionByteData(uint32_t Address, uint8_t Data);
FLASH_Status FLASH_EnableWriteProtection(uint32_t FLASH_Pages);
FLASH_Status FLASH_ReadOutProtection(FunctionalState NewState);
FLASH_Status FLASH_UserOptionByteConfig(uint16_t OB_IWDG, uint16_t OB_STOP, uint16_t OB_STDBY);
uint32_t FLASH_GetUserOptionByte(void);
uint32_t FLASH_GetWriteProtectionOptionByte(void);
FlagStatus FLASH_GetReadOutProtectionStatus(void);
FlagStatus FLASH_GetPrefetchBufferStatus(void);
void FLASH_ITConfig(uint32_t FLASH_IT, FunctionalState NewState);
FlagStatus FLASH_GetFlagStatus(uint32_t FLASH_FLAG);
void FLASH_ClearFlag(uint32_t FLASH_FLAG);
FLASH_Status FLASH_GetStatus(void);
FLASH_Status FLASH_WaitForLastOperation(uint32_t Timeout);

 
void FLASH_UnlockBank1(void);
void FLASH_LockBank1(void);
FLASH_Status FLASH_EraseAllBank1Pages(void);
FLASH_Status FLASH_GetBank1Status(void);
FLASH_Status FLASH_WaitForLastBank1Operation(uint32_t Timeout);

#line 408 ".\\System\\FWlib\\inc\\stm32f10x_flash.h"








 



 



 

 
#line 22 ".\\Main\\comDef.h"
#line 23 ".\\Main\\comDef.h"
#line 24 ".\\Main\\comDef.h"
#line 1 ".\\System\\FWlib\\inc\\stm32f10x_i2c.h"




















 

 







 
#line 33 ".\\System\\FWlib\\inc\\stm32f10x_i2c.h"



 



 



 



 

typedef struct
{
  uint32_t I2C_ClockSpeed;          
 

  uint16_t I2C_Mode;                
 

  uint16_t I2C_DutyCycle;           
 

  uint16_t I2C_OwnAddress1;         
 

  uint16_t I2C_Ack;                 
 

  uint16_t I2C_AcknowledgedAddress; 
 
}I2C_InitTypeDef;



  




 





 

#line 92 ".\\System\\FWlib\\inc\\stm32f10x_i2c.h"


 



 







  



 







 



 







 



 







  



 

#line 166 ".\\System\\FWlib\\inc\\stm32f10x_i2c.h"


 



 







 



 







  



 







  



 







  



 

#line 236 ".\\System\\FWlib\\inc\\stm32f10x_i2c.h"



#line 246 ".\\System\\FWlib\\inc\\stm32f10x_i2c.h"


 



 



 

#line 265 ".\\System\\FWlib\\inc\\stm32f10x_i2c.h"



 

#line 284 ".\\System\\FWlib\\inc\\stm32f10x_i2c.h"



#line 298 ".\\System\\FWlib\\inc\\stm32f10x_i2c.h"


 



 




 







 
 

























 

 


 





























 

  
 


 
 

 






 
























 

    
 



 



 



























 

  
 

 


 
 


 


 

#line 496 ".\\System\\FWlib\\inc\\stm32f10x_i2c.h"


 



 




 



 




 



 



 



 



 

void I2C_DeInit(I2C_TypeDef* I2Cx);
void I2C_Init(I2C_TypeDef* I2Cx, I2C_InitTypeDef* I2C_InitStruct);
void I2C_StructInit(I2C_InitTypeDef* I2C_InitStruct);
void I2C_Cmd(I2C_TypeDef* I2Cx, FunctionalState NewState);
void I2C_DMACmd(I2C_TypeDef* I2Cx, FunctionalState NewState);
void I2C_DMALastTransferCmd(I2C_TypeDef* I2Cx, FunctionalState NewState);
void I2C_GenerateSTART(I2C_TypeDef* I2Cx, FunctionalState NewState);
void I2C_GenerateSTOP(I2C_TypeDef* I2Cx, FunctionalState NewState);
void I2C_AcknowledgeConfig(I2C_TypeDef* I2Cx, FunctionalState NewState);
void I2C_OwnAddress2Config(I2C_TypeDef* I2Cx, uint8_t Address);
void I2C_DualAddressCmd(I2C_TypeDef* I2Cx, FunctionalState NewState);
void I2C_GeneralCallCmd(I2C_TypeDef* I2Cx, FunctionalState NewState);
void I2C_ITConfig(I2C_TypeDef* I2Cx, uint16_t I2C_IT, FunctionalState NewState);
void I2C_SendData(I2C_TypeDef* I2Cx, uint8_t Data);
uint8_t I2C_ReceiveData(I2C_TypeDef* I2Cx);
void I2C_Send7bitAddress(I2C_TypeDef* I2Cx, uint8_t Address, uint8_t I2C_Direction);
uint16_t I2C_ReadRegister(I2C_TypeDef* I2Cx, uint8_t I2C_Register);
void I2C_SoftwareResetCmd(I2C_TypeDef* I2Cx, FunctionalState NewState);
void I2C_NACKPositionConfig(I2C_TypeDef* I2Cx, uint16_t I2C_NACKPosition);
void I2C_SMBusAlertConfig(I2C_TypeDef* I2Cx, uint16_t I2C_SMBusAlert);
void I2C_TransmitPEC(I2C_TypeDef* I2Cx, FunctionalState NewState);
void I2C_PECPositionConfig(I2C_TypeDef* I2Cx, uint16_t I2C_PECPosition);
void I2C_CalculatePEC(I2C_TypeDef* I2Cx, FunctionalState NewState);
uint8_t I2C_GetPEC(I2C_TypeDef* I2Cx);
void I2C_ARPCmd(I2C_TypeDef* I2Cx, FunctionalState NewState);
void I2C_StretchClockCmd(I2C_TypeDef* I2Cx, FunctionalState NewState);
void I2C_FastModeDutyCycleConfig(I2C_TypeDef* I2Cx, uint16_t I2C_DutyCycle);













































































 





 
ErrorStatus I2C_CheckEvent(I2C_TypeDef* I2Cx, uint32_t I2C_EVENT);




 
uint32_t I2C_GetLastEvent(I2C_TypeDef* I2Cx);




 
FlagStatus I2C_GetFlagStatus(I2C_TypeDef* I2Cx, uint32_t I2C_FLAG);



 

void I2C_ClearFlag(I2C_TypeDef* I2Cx, uint32_t I2C_FLAG);
ITStatus I2C_GetITStatus(I2C_TypeDef* I2Cx, uint32_t I2C_IT);
void I2C_ClearITPendingBit(I2C_TypeDef* I2Cx, uint32_t I2C_IT);








  



  



  

 
#line 25 ".\\Main\\comDef.h"
#line 1 ".\\System\\FWlib\\inc\\stm32f10x_iwdg.h"




















 

 







 
#line 33 ".\\System\\FWlib\\inc\\stm32f10x_iwdg.h"



 



 



 



 



 



 







 



 

#line 84 ".\\System\\FWlib\\inc\\stm32f10x_iwdg.h"


 



 







 



 



 



 



 

void IWDG_WriteAccessCmd(uint16_t IWDG_WriteAccess);
void IWDG_SetPrescaler(uint8_t IWDG_Prescaler);
void IWDG_SetReload(uint16_t Reload);
void IWDG_ReloadCounter(void);
void IWDG_Enable(void);
FlagStatus IWDG_GetFlagStatus(uint16_t IWDG_FLAG);








 



 



 

 
#line 26 ".\\Main\\comDef.h"
#line 1 ".\\System\\FWlib\\inc\\stm32f10x_pwr.h"




















 

 







 
#line 33 ".\\System\\FWlib\\inc\\stm32f10x_pwr.h"



 



  



  



  



  



  

#line 70 ".\\System\\FWlib\\inc\\stm32f10x_pwr.h"


 



 







 



 




 


 



 










 



 



 



 



 

void PWR_DeInit(void);
void PWR_BackupAccessCmd(FunctionalState NewState);
void PWR_PVDCmd(FunctionalState NewState);
void PWR_PVDLevelConfig(uint32_t PWR_PVDLevel);
void PWR_WakeUpPinCmd(FunctionalState NewState);
void PWR_EnterSTOPMode(uint32_t PWR_Regulator, uint8_t PWR_STOPEntry);
void PWR_EnterSTANDBYMode(void);
FlagStatus PWR_GetFlagStatus(uint32_t PWR_FLAG);
void PWR_ClearFlag(uint32_t PWR_FLAG);








 



 



 

 
#line 27 ".\\Main\\comDef.h"
#line 28 ".\\Main\\comDef.h"
#line 1 ".\\System\\FWlib\\inc\\stm32f10x_rtc.h"




















 

 







 
#line 33 ".\\System\\FWlib\\inc\\stm32f10x_rtc.h"



 



  



  



  



 



 

#line 64 ".\\System\\FWlib\\inc\\stm32f10x_rtc.h"


  



 

#line 82 ".\\System\\FWlib\\inc\\stm32f10x_rtc.h"



 



 



 



 



 

void RTC_ITConfig(uint16_t RTC_IT, FunctionalState NewState);
void RTC_EnterConfigMode(void);
void RTC_ExitConfigMode(void);
uint32_t  RTC_GetCounter(void);
void RTC_SetCounter(uint32_t CounterValue);
void RTC_SetPrescaler(uint32_t PrescalerValue);
void RTC_SetAlarm(uint32_t AlarmValue);
uint32_t  RTC_GetDivider(void);
void RTC_WaitForLastTask(void);
void RTC_WaitForSynchro(void);
FlagStatus RTC_GetFlagStatus(uint16_t RTC_FLAG);
void RTC_ClearFlag(uint16_t RTC_FLAG);
ITStatus RTC_GetITStatus(uint16_t RTC_IT);
void RTC_ClearITPendingBit(uint16_t RTC_IT);








 



 



 

 
#line 29 ".\\Main\\comDef.h"
#line 1 ".\\System\\FWlib\\inc\\stm32f10x_sdio.h"




















 

 







 
#line 33 ".\\System\\FWlib\\inc\\stm32f10x_sdio.h"



 



 



 

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



  



 



 







 



 







  



 







 



 









 



 







 



 






  




 

#line 222 ".\\System\\FWlib\\inc\\stm32f10x_sdio.h"


  



 




 



 

#line 245 ".\\System\\FWlib\\inc\\stm32f10x_sdio.h"


 



 








 



 






  



 

#line 283 ".\\System\\FWlib\\inc\\stm32f10x_sdio.h"


 



 




 



 

#line 330 ".\\System\\FWlib\\inc\\stm32f10x_sdio.h"


 



 







 



 







 



 






 



 

#line 421 ".\\System\\FWlib\\inc\\stm32f10x_sdio.h"



#line 448 ".\\System\\FWlib\\inc\\stm32f10x_sdio.h"





 



 







 



 



 



 



 

void SDIO_DeInit(void);
void SDIO_Init(SDIO_InitTypeDef* SDIO_InitStruct);
void SDIO_StructInit(SDIO_InitTypeDef* SDIO_InitStruct);
void SDIO_ClockCmd(FunctionalState NewState);
void SDIO_SetPowerState(uint32_t SDIO_PowerState);
uint32_t SDIO_GetPowerState(void);
void SDIO_ITConfig(uint32_t SDIO_IT, FunctionalState NewState);
void SDIO_DMACmd(FunctionalState NewState);
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
FlagStatus SDIO_GetFlagStatus(uint32_t SDIO_FLAG);
void SDIO_ClearFlag(uint32_t SDIO_FLAG);
ITStatus SDIO_GetITStatus(uint32_t SDIO_IT);
void SDIO_ClearITPendingBit(uint32_t SDIO_IT);








 



 



 

 
#line 30 ".\\Main\\comDef.h"
#line 31 ".\\Main\\comDef.h"
#line 1 ".\\System\\FWlib\\inc\\stm32f10x_tim.h"




















 

 







 
#line 33 ".\\System\\FWlib\\inc\\stm32f10x_tim.h"



 



  



  




 

typedef struct
{
  uint16_t TIM_Prescaler;         
 

  uint16_t TIM_CounterMode;       
 

  uint16_t TIM_Period;            

  

  uint16_t TIM_ClockDivision;     
 

  uint8_t TIM_RepetitionCounter;  






 
} TIM_TimeBaseInitTypeDef;       



 

typedef struct
{
  uint16_t TIM_OCMode;        
 

  uint16_t TIM_OutputState;   
 

  uint16_t TIM_OutputNState;  

 

  uint16_t TIM_Pulse;         
 

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



 

#line 186 ".\\System\\FWlib\\inc\\stm32f10x_tim.h"

 



 






 
#line 205 ".\\System\\FWlib\\inc\\stm32f10x_tim.h"
									                                 
 
#line 216 ".\\System\\FWlib\\inc\\stm32f10x_tim.h"

                                             
#line 225 ".\\System\\FWlib\\inc\\stm32f10x_tim.h"

 
#line 236 ".\\System\\FWlib\\inc\\stm32f10x_tim.h"

 
#line 249 ".\\System\\FWlib\\inc\\stm32f10x_tim.h"

                                         
#line 266 ".\\System\\FWlib\\inc\\stm32f10x_tim.h"

 
#line 279 ".\\System\\FWlib\\inc\\stm32f10x_tim.h"
                                                                                                                                                                                                                          


  



 

#line 308 ".\\System\\FWlib\\inc\\stm32f10x_tim.h"


 



 







  



 

#line 341 ".\\System\\FWlib\\inc\\stm32f10x_tim.h"


  



 

#line 355 ".\\System\\FWlib\\inc\\stm32f10x_tim.h"


 



 

#line 373 ".\\System\\FWlib\\inc\\stm32f10x_tim.h"


  



 







 



 
  






 



 







  



 







  



 







  



 







  



 







  



 







  



 







  



 

#line 497 ".\\System\\FWlib\\inc\\stm32f10x_tim.h"


  



 







 



 







  



 







  



 







  



 

#line 561 ".\\System\\FWlib\\inc\\stm32f10x_tim.h"


  



 

#line 577 ".\\System\\FWlib\\inc\\stm32f10x_tim.h"


  



 

#line 593 ".\\System\\FWlib\\inc\\stm32f10x_tim.h"


  



 

#line 610 ".\\System\\FWlib\\inc\\stm32f10x_tim.h"

#line 619 ".\\System\\FWlib\\inc\\stm32f10x_tim.h"


  



 

#line 665 ".\\System\\FWlib\\inc\\stm32f10x_tim.h"


  



 

#line 709 ".\\System\\FWlib\\inc\\stm32f10x_tim.h"


  



 

#line 725 ".\\System\\FWlib\\inc\\stm32f10x_tim.h"



  



 

#line 742 ".\\System\\FWlib\\inc\\stm32f10x_tim.h"


  



 

#line 770 ".\\System\\FWlib\\inc\\stm32f10x_tim.h"


  



 

#line 784 ".\\System\\FWlib\\inc\\stm32f10x_tim.h"


  



  






 



 







  



 







  



 

#line 833 ".\\System\\FWlib\\inc\\stm32f10x_tim.h"


  




 

#line 851 ".\\System\\FWlib\\inc\\stm32f10x_tim.h"



  



 

#line 866 ".\\System\\FWlib\\inc\\stm32f10x_tim.h"


  



 







  



 





                                     


  



 







  



 

#line 927 ".\\System\\FWlib\\inc\\stm32f10x_tim.h"


  



 

#line 943 ".\\System\\FWlib\\inc\\stm32f10x_tim.h"


  



 







  



 

#line 987 ".\\System\\FWlib\\inc\\stm32f10x_tim.h"
                               
                               



  



 




  



 




  



 

#line 1034 ".\\System\\FWlib\\inc\\stm32f10x_tim.h"


 



 



 



  



 

void TIM_DeInit(TIM_TypeDef* TIMx);
void TIM_TimeBaseInit(TIM_TypeDef* TIMx, TIM_TimeBaseInitTypeDef* TIM_TimeBaseInitStruct);
void TIM_OC1Init(TIM_TypeDef* TIMx, TIM_OCInitTypeDef* TIM_OCInitStruct);
void TIM_OC2Init(TIM_TypeDef* TIMx, TIM_OCInitTypeDef* TIM_OCInitStruct);
void TIM_OC3Init(TIM_TypeDef* TIMx, TIM_OCInitTypeDef* TIM_OCInitStruct);
void TIM_OC4Init(TIM_TypeDef* TIMx, TIM_OCInitTypeDef* TIM_OCInitStruct);
void TIM_ICInit(TIM_TypeDef* TIMx, TIM_ICInitTypeDef* TIM_ICInitStruct);
void TIM_PWMIConfig(TIM_TypeDef* TIMx, TIM_ICInitTypeDef* TIM_ICInitStruct);
void TIM_BDTRConfig(TIM_TypeDef* TIMx, TIM_BDTRInitTypeDef *TIM_BDTRInitStruct);
void TIM_TimeBaseStructInit(TIM_TimeBaseInitTypeDef* TIM_TimeBaseInitStruct);
void TIM_OCStructInit(TIM_OCInitTypeDef* TIM_OCInitStruct);
void TIM_ICStructInit(TIM_ICInitTypeDef* TIM_ICInitStruct);
void TIM_BDTRStructInit(TIM_BDTRInitTypeDef* TIM_BDTRInitStruct);
void TIM_Cmd(TIM_TypeDef* TIMx, FunctionalState NewState);
void TIM_CtrlPWMOutputs(TIM_TypeDef* TIMx, FunctionalState NewState);
void TIM_ITConfig(TIM_TypeDef* TIMx, uint16_t TIM_IT, FunctionalState NewState);
void TIM_GenerateEvent(TIM_TypeDef* TIMx, uint16_t TIM_EventSource);
void TIM_DMAConfig(TIM_TypeDef* TIMx, uint16_t TIM_DMABase, uint16_t TIM_DMABurstLength);
void TIM_DMACmd(TIM_TypeDef* TIMx, uint16_t TIM_DMASource, FunctionalState NewState);
void TIM_InternalClockConfig(TIM_TypeDef* TIMx);
void TIM_ITRxExternalClockConfig(TIM_TypeDef* TIMx, uint16_t TIM_InputTriggerSource);
void TIM_TIxExternalClockConfig(TIM_TypeDef* TIMx, uint16_t TIM_TIxExternalCLKSource,
                                uint16_t TIM_ICPolarity, uint16_t ICFilter);
void TIM_ETRClockMode1Config(TIM_TypeDef* TIMx, uint16_t TIM_ExtTRGPrescaler, uint16_t TIM_ExtTRGPolarity,
                             uint16_t ExtTRGFilter);
void TIM_ETRClockMode2Config(TIM_TypeDef* TIMx, uint16_t TIM_ExtTRGPrescaler, 
                             uint16_t TIM_ExtTRGPolarity, uint16_t ExtTRGFilter);
void TIM_ETRConfig(TIM_TypeDef* TIMx, uint16_t TIM_ExtTRGPrescaler, uint16_t TIM_ExtTRGPolarity,
                   uint16_t ExtTRGFilter);
void TIM_PrescalerConfig(TIM_TypeDef* TIMx, uint16_t Prescaler, uint16_t TIM_PSCReloadMode);
void TIM_CounterModeConfig(TIM_TypeDef* TIMx, uint16_t TIM_CounterMode);
void TIM_SelectInputTrigger(TIM_TypeDef* TIMx, uint16_t TIM_InputTriggerSource);
void TIM_EncoderInterfaceConfig(TIM_TypeDef* TIMx, uint16_t TIM_EncoderMode,
                                uint16_t TIM_IC1Polarity, uint16_t TIM_IC2Polarity);
void TIM_ForcedOC1Config(TIM_TypeDef* TIMx, uint16_t TIM_ForcedAction);
void TIM_ForcedOC2Config(TIM_TypeDef* TIMx, uint16_t TIM_ForcedAction);
void TIM_ForcedOC3Config(TIM_TypeDef* TIMx, uint16_t TIM_ForcedAction);
void TIM_ForcedOC4Config(TIM_TypeDef* TIMx, uint16_t TIM_ForcedAction);
void TIM_ARRPreloadConfig(TIM_TypeDef* TIMx, FunctionalState NewState);
void TIM_SelectCOM(TIM_TypeDef* TIMx, FunctionalState NewState);
void TIM_SelectCCDMA(TIM_TypeDef* TIMx, FunctionalState NewState);
void TIM_CCPreloadControl(TIM_TypeDef* TIMx, FunctionalState NewState);
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
void TIM_SelectOCxM(TIM_TypeDef* TIMx, uint16_t TIM_Channel, uint16_t TIM_OCMode);
void TIM_UpdateDisableConfig(TIM_TypeDef* TIMx, FunctionalState NewState);
void TIM_UpdateRequestConfig(TIM_TypeDef* TIMx, uint16_t TIM_UpdateSource);
void TIM_SelectHallSensor(TIM_TypeDef* TIMx, FunctionalState NewState);
void TIM_SelectOnePulseMode(TIM_TypeDef* TIMx, uint16_t TIM_OPMode);
void TIM_SelectOutputTrigger(TIM_TypeDef* TIMx, uint16_t TIM_TRGOSource);
void TIM_SelectSlaveMode(TIM_TypeDef* TIMx, uint16_t TIM_SlaveMode);
void TIM_SelectMasterSlaveMode(TIM_TypeDef* TIMx, uint16_t TIM_MasterSlaveMode);
void TIM_SetCounter(TIM_TypeDef* TIMx, uint16_t Counter);
void TIM_SetAutoreload(TIM_TypeDef* TIMx, uint16_t Autoreload);
void TIM_SetCompare1(TIM_TypeDef* TIMx, uint16_t Compare1);
void TIM_SetCompare2(TIM_TypeDef* TIMx, uint16_t Compare2);
void TIM_SetCompare3(TIM_TypeDef* TIMx, uint16_t Compare3);
void TIM_SetCompare4(TIM_TypeDef* TIMx, uint16_t Compare4);
void TIM_SetIC1Prescaler(TIM_TypeDef* TIMx, uint16_t TIM_ICPSC);
void TIM_SetIC2Prescaler(TIM_TypeDef* TIMx, uint16_t TIM_ICPSC);
void TIM_SetIC3Prescaler(TIM_TypeDef* TIMx, uint16_t TIM_ICPSC);
void TIM_SetIC4Prescaler(TIM_TypeDef* TIMx, uint16_t TIM_ICPSC);
void TIM_SetClockDivision(TIM_TypeDef* TIMx, uint16_t TIM_CKD);
uint16_t TIM_GetCapture1(TIM_TypeDef* TIMx);
uint16_t TIM_GetCapture2(TIM_TypeDef* TIMx);
uint16_t TIM_GetCapture3(TIM_TypeDef* TIMx);
uint16_t TIM_GetCapture4(TIM_TypeDef* TIMx);
uint16_t TIM_GetCounter(TIM_TypeDef* TIMx);
uint16_t TIM_GetPrescaler(TIM_TypeDef* TIMx);
FlagStatus TIM_GetFlagStatus(TIM_TypeDef* TIMx, uint16_t TIM_FLAG);
void TIM_ClearFlag(TIM_TypeDef* TIMx, uint16_t TIM_FLAG);
ITStatus TIM_GetITStatus(TIM_TypeDef* TIMx, uint16_t TIM_IT);
void TIM_ClearITPendingBit(TIM_TypeDef* TIMx, uint16_t TIM_IT);








  



  



 

 
#line 32 ".\\Main\\comDef.h"
#line 1 ".\\System\\FWlib\\inc\\stm32f10x_usart.h"




















 

 







 
#line 33 ".\\System\\FWlib\\inc\\stm32f10x_usart.h"



 



  



  



  
  
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



  



  
  
















  
  


                                    




  



  
  
#line 146 ".\\System\\FWlib\\inc\\stm32f10x_usart.h"


  



  
  
#line 160 ".\\System\\FWlib\\inc\\stm32f10x_usart.h"


  



  
  





  



  
#line 187 ".\\System\\FWlib\\inc\\stm32f10x_usart.h"


  



  






  



 
  






  



 







 



 







  



 
  
#line 264 ".\\System\\FWlib\\inc\\stm32f10x_usart.h"


 



 







  



 







 



 
  







 



 







  



 

#line 336 ".\\System\\FWlib\\inc\\stm32f10x_usart.h"
                              
#line 344 ".\\System\\FWlib\\inc\\stm32f10x_usart.h"



  



  



  



  



 

void USART_DeInit(USART_TypeDef* USARTx);
void USART_Init(USART_TypeDef* USARTx, USART_InitTypeDef* USART_InitStruct);
void USART_StructInit(USART_InitTypeDef* USART_InitStruct);
void USART_ClockInit(USART_TypeDef* USARTx, USART_ClockInitTypeDef* USART_ClockInitStruct);
void USART_ClockStructInit(USART_ClockInitTypeDef* USART_ClockInitStruct);
void USART_Cmd(USART_TypeDef* USARTx, FunctionalState NewState);
void USART_ITConfig(USART_TypeDef* USARTx, uint16_t USART_IT, FunctionalState NewState);
void USART_DMACmd(USART_TypeDef* USARTx, uint16_t USART_DMAReq, FunctionalState NewState);
void USART_SetAddress(USART_TypeDef* USARTx, uint8_t USART_Address);
void USART_WakeUpConfig(USART_TypeDef* USARTx, uint16_t USART_WakeUp);
void USART_ReceiverWakeUpCmd(USART_TypeDef* USARTx, FunctionalState NewState);
void USART_LINBreakDetectLengthConfig(USART_TypeDef* USARTx, uint16_t USART_LINBreakDetectLength);
void USART_LINCmd(USART_TypeDef* USARTx, FunctionalState NewState);
void USART_SendData(USART_TypeDef* USARTx, uint16_t Data);
uint16_t USART_ReceiveData(USART_TypeDef* USARTx);
void USART_SendBreak(USART_TypeDef* USARTx);
void USART_SetGuardTime(USART_TypeDef* USARTx, uint8_t USART_GuardTime);
void USART_SetPrescaler(USART_TypeDef* USARTx, uint8_t USART_Prescaler);
void USART_SmartCardCmd(USART_TypeDef* USARTx, FunctionalState NewState);
void USART_SmartCardNACKCmd(USART_TypeDef* USARTx, FunctionalState NewState);
void USART_HalfDuplexCmd(USART_TypeDef* USARTx, FunctionalState NewState);
void USART_OverSampling8Cmd(USART_TypeDef* USARTx, FunctionalState NewState);
void USART_OneBitMethodCmd(USART_TypeDef* USARTx, FunctionalState NewState);
void USART_IrDAConfig(USART_TypeDef* USARTx, uint16_t USART_IrDAMode);
void USART_IrDACmd(USART_TypeDef* USARTx, FunctionalState NewState);
FlagStatus USART_GetFlagStatus(USART_TypeDef* USARTx, uint16_t USART_FLAG);
void USART_ClearFlag(USART_TypeDef* USARTx, uint16_t USART_FLAG);
ITStatus USART_GetITStatus(USART_TypeDef* USARTx, uint16_t USART_IT);
void USART_ClearITPendingBit(USART_TypeDef* USARTx, uint16_t USART_IT);








  



  



  

 
#line 33 ".\\Main\\comDef.h"
#line 1 ".\\System\\FWlib\\inc\\stm32f10x_wwdg.h"




















 

 







 
#line 33 ".\\System\\FWlib\\inc\\stm32f10x_wwdg.h"



 



  



  
  


  



  
  


  
  
#line 68 ".\\System\\FWlib\\inc\\stm32f10x_wwdg.h"



  



  



  


  



  
  
void WWDG_DeInit(void);
void WWDG_SetPrescaler(uint32_t WWDG_Prescaler);
void WWDG_SetWindowValue(uint8_t WindowValue);
void WWDG_EnableIT(void);
void WWDG_SetCounter(uint8_t Counter);
void WWDG_Enable(uint8_t Counter);
FlagStatus WWDG_GetFlagStatus(void);
void WWDG_ClearFlag(void);









  



  



  

 
#line 34 ".\\Main\\comDef.h"

 
 
typedef   signed          char int8;
typedef   signed short     int int16;
typedef   signed           int int32;
typedef   signed       __int64 int64;

 
typedef unsigned          char uint8;
typedef unsigned short     int uint16;
typedef unsigned           int uint32;
typedef unsigned       __int64 uint64;

 
enum NORMAL_ON_OFF{
	NORMAL_OFF = 0,
	NORMAL_ON = 1,
};

enum ABNORMAL_ON_OFF{
	ABNORMAL_ON = 0,
	ABNORMAL_OFF = 1,
};

#line 34 ".\\System\\FWlib\\inc\\misc.h"



 



 



 



 

typedef struct
{
  uint8_t NVIC_IRQChannel;                    


 

  uint8_t NVIC_IRQChannelPreemptionPriority;  

 

  uint8_t NVIC_IRQChannelSubPriority;         

 

  FunctionalState NVIC_IRQChannelCmd;         

    
} NVIC_InitTypeDef;
 


 



 
























 



 



 



 







 



 

#line 134 ".\\System\\FWlib\\inc\\misc.h"


 



 

#line 152 ".\\System\\FWlib\\inc\\misc.h"















 



 







 



 



 



 



 

void NVIC_PriorityGroupConfig(uint32_t NVIC_PriorityGroup);
void NVIC_Init(NVIC_InitTypeDef* NVIC_InitStruct);
void NVIC_SetVectorTable(uint32_t NVIC_VectTab, uint32_t Offset);
void NVIC_SystemLPConfig(uint8_t LowPowerMode, FunctionalState NewState);
void SysTick_CLKSourceConfig(uint32_t SysTick_CLKSource);









 



 



 

 
#line 49 ".\\System\\FWlib\\inc\\stm32f10x_conf.h"

 
 

 
 

 
#line 73 ".\\System\\FWlib\\inc\\stm32f10x_conf.h"



 
#line 8302 ".\\System\\CM3\\stm32f10x.h"




 

















 









 

  

 

 
#line 12 ".\\Management\\DisplayDriver\\Font.h"
#line 13 ".\\Management\\DisplayDriver\\Font.h"

 
void DisplayDriver_Text16(unsigned int x, unsigned int y, unsigned int Color,
		 u8 *s);
void DisplayDriver_Text16_B(uint16 x, uint16 y, uint16 fc,
		uint16 bc,uint8 *s);
void DisplayDriver_Clear(u16 xStart,u16 yStart,u16 xLong,u16 yLong,u16 Color);
void DisplayDriver_Clear(u16 xStart,u16 yStart,u16 xLong,u16 yLong,u16 Color);
void DisplayDriver_DrawLine(u16 start_x, u16 start_y, u16 end_x, u16 end_y, u16 color);

#line 10 "Management\\Interface\\Interface_main.c"
#line 1 "Management\\Interface\\Interface_main.h"





 




 
#line 13 "Management\\Interface\\Interface_main.h"
#line 1 ".\\Management\\DisplayDriver\\DisplayDriver.h"





 



 
#line 12 ".\\Management\\DisplayDriver\\DisplayDriver.h"
#line 13 ".\\Management\\DisplayDriver\\DisplayDriver.h"

 









 
extern void DisplayDriver_Init(void);

 

void Lcd_WR_Start(void);
void DataToWrite(u16 data);
void RCC_Configuration(void);
void Lcd_Display_Clear(void);
void DisplayDriver_Init(void);
void DisplayDriver_Init(void);
void LCD_WR_REG(u16 Index,u16 CongfigTemp);

 

u16 ssd1289_GetPoint(u16 x,u8 y);
char display_picture(char *filename);
void DrawPixel(u16 x, u16 y, u16 Color);
char Tiky_Button(char *filename,u16 x,u16 y);
void Lcd_ColorSpot(u16 xStart,u16 yStart,u16 Color);
void BlockWrite_Test(unsigned int Xstart,unsigned int Ystart);
void Lcd_ColorBox(u16 x,u16 y,u16 xLong,u16 yLong,u16 Color);
void DisplayDriver_DrawPic(u16 x, u16 y,u16 pic_H, u16 pic_V, const unsigned char* pic);
void BlockWrite(unsigned int Xstart,unsigned int Xend,unsigned int Ystart,unsigned int Yend);

 
 
#line 60 ".\\Management\\DisplayDriver\\DisplayDriver.h"




 



 


 






#line 14 "Management\\Interface\\Interface_main.h"
#line 15 "Management\\Interface\\Interface_main.h"

 


 
extern uint8 UI_state;
extern uint8 key_state;
extern uint8 Exti_lock;
extern uint8 Key_control;
extern uint8 Interface_Key;
extern uint8 key_state_confirm;
extern uint16 hours,minutes,seconds; 		
extern uint8 Display_Time;

 
 
typedef struct {
	uint16 startX; 			 
	uint16 startY; 			 
	uint16 width; 			 
	uint16 height; 			 
	uint16 color; 			 
} rect_attr;

 
 
typedef struct {
	uint16 startX; 			 
	uint16 startY; 			 
	uint16 endX; 			 
	uint16 endY; 			 
	uint16 color; 			 
} line_attr;

 
 
typedef struct {
	const uint8* src; 		 
	uint16 offsetX;			 
	uint16 offsetY; 		 
	uint16 width; 			 
	uint16 height;			 
} pic_attr;

 
 
typedef struct {
	uint8* str; 			 
	uint16 offsetX;			 
	uint16 offsetY; 		 
	uint16 color; 			 
	uint16 backColor; 		 
	uint16 faceColor; 		 
} char_attr;

 
typedef struct {
	uint8 rect_enabled; 				 
	rect_attr rect_attr;				 
	uint8 pic_enabled;     				 
	pic_attr pic_attr;     				 
	uint8 char_enabled;					 
	char_attr char_attr;				 
	uint8 line_enabled; 				 
	line_attr Parting_line_attr;		 
} block_attr;

 
typedef struct {
	uint8 char1_enabled;					 
	char_attr char1_attr;				 
	uint8 char2_enabled;					 
	char_attr char2_attr;				 
	uint8 char3_enabled;					 
	char_attr char3_attr;				 
	uint8 char4_enabled;					 
	char_attr char4_attr;				 
} block_font_attr;

 
typedef enum {
	UI_STATE_MAIN_WINDOW, 			 
	UI_STATE_KEY_STATE, 			 
	UI_STATE_MAIN_FONT, 			 
	UI_STATE_STANDARD, 				 
	UI_STATE_QUICK, 				 
	UI_STATE_RECORD, 				 
	UI_STATE_SETTING, 				 
	UI_STATE_START,					 
	UI_STATE_TESTING,				 
	UI_STATE_QUICK_FONT,			 
	UI_STATE_RESULT,				 
	UI_STATE_RESULT_2,				 
	UI_STATE_INSERT_CUP,			 

	UI_STATE_MAX_STATE_NUM,
} UI_STATE;

 
extern void Battery_Empty_ICO(void);
extern void UI_Draw_Status_Bar (void);
extern uint16 Get_Start_Postion(void);
extern void UI_Draw_Window(uint16 blockNum);
extern uint8 Interface_Main(uint16 KeyCode);
extern uint8 Interface_Quick(uint16 KeyCode);
extern uint8 Interface_Start(uint16 KeyCode);
extern uint8 Interface_Record(uint16 KeyCode);
extern uint8 Interface_Result(uint16 KeyCode);
extern uint8 Interface_Testing(uint16 KeyCode);
extern uint8 Interface_Setting(uint16 KeyCode);
extern uint8 Interface_Result_2(uint16 KeyCode);
extern uint8 Interface_Standard(uint16 KeyCode);
extern void UI_Draw_Window_font(uint16 blockNum);
extern uint8 Interface_Main_font(uint16 KeyCode);
extern uint8 Interface_Key_Event(uint16 KeyCode);
extern uint8 Interface_Insert_Cup(uint16 KeyCode);
extern uint8 Interface_Quick_font(uint16 blockNum);
extern uint8 Interface_Standard_font(uint16 KeyCode);
extern void UI_Draw_Window_Quick_font(uint16 blockNum);
extern void SignalSample_Moving_Average_Data(uint16 *Data,uint16 Length,uint16 Period);

#line 11 "Management\\Interface\\Interface_main.c"
#line 12 "Management\\Interface\\Interface_main.c"
#line 1 ".\\Management\\DisplayDriver\\PIC_Interface.h"





 




 
const unsigned char gImage_PIC_Standard[4050] = {  
0XFD,0X00,0XFC,0XE0,0XFC,0XE0,0XFC,0XE0,0XFC,0XE0,0XFC,0XE0,0XFC,0XC0,0XFC,0XE0,
0XFC,0XE0,0XFC,0XE0,0XFC,0XE0,0XFC,0XE0,0XFD,0X00,0XFD,0X00,0XFC,0XC0,0XFC,0XE0,
0XFC,0XE0,0XFC,0XE0,0XF4,0XC0,0XEC,0XC0,0XE4,0XA1,0XE4,0XA2,0XE4,0X82,0XEC,0XA2,
0XF4,0XA1,0XF4,0XC0,0XFC,0XC0,0XFC,0XC0,0XFC,0XC0,0XFC,0XC0,0XFC,0XC0,0XFC,0XC0,
0XFC,0XE0,0XFC,0XE0,0XFC,0XE0,0XFD,0X00,0XFC,0XE0,0XFC,0XE0,0XFC,0XE0,0XFC,0XC0,
0XFC,0XC0,0XFC,0XC0,0XFC,0XE0,0XFC,0XE0,0XFC,0XE0,0XFC,0XE0,0XFC,0XE0,0XFC,0XE0,
0XFC,0XE0,0XFC,0XE0,0XFC,0XC0,0XFC,0XE0,0XFC,0XC0,0XFC,0XE0,0XFD,0X00,0XFC,0XE0,
0XFC,0XC0,0XFC,0XC0,0XF4,0XA0,0XFC,0XE1,0XEC,0XA1,0XEC,0X81,0XEC,0XA1,0XF5,0X03,
0XFD,0X66,0XFD,0XA7,0XFD,0XC9,0XFD,0XC9,0XFD,0XA8,0XFD,0XA7,0XF5,0X24,0XEC,0XC2,
0XE4,0XA1,0XEC,0XA1,0XF4,0XA1,0XFC,0XC1,0XFC,0XC0,0XFC,0XC0,0XFD,0X00,0XFC,0XE0,
0XFC,0XC0,0XFD,0X00,0XFC,0XE0,0XFC,0XC0,0XFC,0XE0,0XFC,0XC0,0XFC,0XE0,0XFC,0XE0,
0XFC,0XE0,0XFC,0XE0,0XFC,0XC0,0XFC,0XC0,0XFC,0XE0,0XFC,0XE0,0XFC,0XE0,0XFC,0XE0,
0XFC,0XE0,0XFC,0XE0,0XFC,0XE0,0XF4,0XA0,0XFD,0X00,0XFC,0XE0,0XFC,0XA1,0XF4,0X81,
0XE4,0X83,0XFD,0X67,0XFE,0X2C,0XFE,0X8E,0XFE,0XAF,0XFE,0X8F,0XFE,0X90,0XFE,0X70,
0XFE,0X4F,0XFE,0X8F,0XFE,0X8F,0XFE,0XAE,0XFE,0XCF,0XFE,0X8D,0XFD,0XC9,0XEC,0XE4,
0XEC,0X81,0XEC,0XA0,0XFC,0XC0,0XFC,0XA0,0XFC,0XE0,0XFC,0XE0,0XFC,0XC0,0XFC,0XE0,
0XFC,0XE0,0XFC,0XE0,0XFC,0XE0,0XFC,0XE0,0XFC,0XE0,0XFC,0XE0,0XFC,0XE0,0XFC,0XC0,
0XFC,0XC0,0XFC,0XE0,0XFC,0XE0,0XFC,0XE0,0XFC,0XE0,0XFC,0XE0,0XFC,0XC0,0XFC,0XE0,
0XFC,0XE0,0XF4,0XA0,0XEC,0X80,0XEC,0XA2,0XFD,0XC9,0XFE,0X8E,0XFE,0X90,0XF6,0X30,
0XEE,0X31,0XF6,0X93,0XFE,0XD5,0XFE,0XF6,0XFF,0X37,0XFF,0X57,0XFF,0X16,0XFF,0X15,
0XF6,0XD3,0XEE,0X51,0XEE,0X30,0XFE,0X70,0XFE,0XCF,0XFE,0X4B,0XFD,0X66,0XE4,0X61,
0XF4,0XA1,0XFC,0XC1,0XFC,0XC0,0XFC,0XE0,0XFC,0XE0,0XFC,0XC0,0XFC,0XC0,0XFC,0XE0,
0XFC,0XE0,0XFC,0XE0,0XFC,0XE0,0XFC,0XE0,0XFC,0XE0,0XFC,0XE0,0XFC,0XE0,0XFC,0XE0,
0XFC,0XE0,0XFC,0XE0,0XFC,0XC0,0XFC,0XC0,0XFC,0XC0,0XF4,0XC0,0XE4,0XA2,0XF5,0X87,
0XFE,0XAE,0XFE,0X8F,0XEE,0X10,0XFE,0XB5,0XFF,0X79,0XFF,0XFC,0XFF,0XFD,0XFF,0XFE,
0XFF,0XFE,0XFF,0XFE,0XFF,0XFE,0XFF,0XFE,0XFF,0XFD,0XFF,0XFE,0XFF,0XFD,0XFF,0XDB,
0XFE,0XF7,0XEE,0X32,0XFE,0X4F,0XFE,0XAE,0XFE,0X4C,0XEC,0XC4,0XEC,0X81,0XFC,0XE1,
0XFC,0XE0,0XFC,0XC0,0XFC,0XC0,0XFC,0XE0,0XFC,0XE0,0XFC,0XE0,0XFC,0XE0,0XFC,0XE0,
0XFC,0XE0,0XFC,0XE0,0XFC,0XE0,0XFC,0XE0,0XFC,0XE0,0XFC,0XE0,0XFC,0XE0,0XFC,0XC0,
0XF4,0XC0,0XF4,0XC1,0XEC,0XA2,0XFE,0X4C,0XFE,0XB0,0XEE,0X2F,0XF6,0XB3,0XFF,0XFB,
0XFF,0XFD,0XFF,0XFE,0XFF,0XFE,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XF7,0XDF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFE,0XFF,0XFD,0XFF,0X58,
0XF6,0X52,0XFE,0X70,0XFE,0X8E,0XF5,0X67,0XE4,0X81,0XF4,0XC0,0XFC,0XE0,0XFC,0XC0,
0XFC,0XE0,0XFC,0XE0,0XFC,0XE0,0XFC,0XE0,0XFC,0XE0,0XFC,0XE0,0XFC,0XC0,0XFC,0XC0,
0XFC,0XC0,0XFC,0XE0,0XFC,0XE0,0XF4,0XE0,0XFC,0XE0,0XF4,0XC1,0XE4,0XA3,0XFE,0XAD,
0XFE,0X70,0XEE,0X32,0XFF,0XBA,0XFF,0XFD,0XFF,0XFE,0XFF,0XFE,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFE,0XFF,0XFE,0XFF,0XFE,0XFF,0XFD,0XFE,0XD5,0XEE,0X30,
0XFE,0XD0,0XFD,0XEA,0XE4,0X61,0XFC,0XC1,0XFC,0XC0,0XFC,0XC0,0XFC,0XE0,0XFC,0XE0,
0XFC,0XE0,0XFC,0XE0,0XFC,0XE0,0XFC,0XC0,0XFC,0XC0,0XFC,0XE0,0XFC,0XE0,0XF4,0XE0,
0XF4,0XC0,0XF4,0XE1,0XEC,0XC3,0XFE,0XCE,0XFE,0X50,0XF6,0XB4,0XFF,0XFC,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFD,0XFF,0XB9,0XEE,0X30,0XFE,0X8F,0XFE,0X0A,
0XEC,0X81,0XFC,0XA1,0XFC,0XC0,0XFC,0XE0,0XFC,0XE0,0XFC,0XE0,0XFC,0XE0,0XFC,0XE0,
0XFC,0XE0,0XFC,0XE0,0XFC,0XE0,0XFC,0XE0,0XFC,0XC0,0XF4,0X81,0XEC,0XC3,0XFE,0X8D,
0XFE,0X4F,0XFE,0XD5,0XFF,0XFD,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFE,0XFF,0XFF,0XFF,0XFF,0XFF,0XDF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFD,0XFF,0XFA,0XEE,0X31,0XFE,0X8F,0XFD,0XEA,0XE4,0X41,0XFC,0XC1,
0XFC,0XC0,0XFD,0X00,0XFC,0XC0,0XFD,0X00,0XFC,0XE0,0XFC,0XE0,0XFC,0XE0,0XFC,0XE0,
0XF4,0XC0,0XFC,0XC1,0XEC,0X82,0XFE,0X4C,0XFE,0X4F,0XFE,0XD4,0XFF,0XFD,0XFF,0XFF,
0XF7,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XF7,0XFF,0XF7,0XFF,0XFF,0XFF,0XFF,0XFD,
0XFF,0XDA,0XF6,0X51,0XFE,0X8F,0XF5,0X47,0XEC,0X81,0XFC,0XE1,0XFC,0XC0,0XFC,0XE0,
0XFC,0XE0,0XFC,0XE0,0XFC,0XE0,0XFC,0XE0,0XFC,0XE0,0XF4,0XE1,0XEC,0X81,0XFD,0XC9,
0XFE,0X8F,0XF6,0XB3,0XFF,0XFD,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XF7,0XDF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XDF,0XFF,0XFF,0XFF,0XFE,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XDF,0XFF,0XDF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFE,0XFF,0X99,0XEE,0X31,
0XFE,0XAF,0XE4,0XA3,0XF4,0XA0,0XFC,0XC0,0XFD,0X00,0XFC,0XE0,0XFC,0XE0,0XFD,0X00,
0XFC,0XE0,0XFC,0XE0,0XEC,0X80,0XEC,0XE4,0XFE,0XAF,0XF6,0X51,0XFF,0XDB,0XFF,0XFF,
0XF7,0XDF,0XFF,0XFF,0XFF,0XFF,0XE7,0XFD,0XE7,0XFD,0XEF,0XFD,0XEF,0XFC,0XF7,0XFD,
0XF7,0XFE,0XFF,0XFF,0XFF,0XFF,0XFF,0XDF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XF7,0XFF,0XF7,0XFF,0XF7,0XFE,0XF7,0XFE,0XEF,0XFD,0XEF,0XFC,0XEF,0XFC,0XF7,0XFE,
0XFF,0XFF,0XF7,0XFF,0XFF,0XFF,0XFF,0XFD,0XFF,0X37,0XF6,0X50,0XFE,0X2B,0XDC,0X61,
0XFC,0XE0,0XFC,0XE0,0XFC,0XE0,0XFC,0XE0,0XFC,0XC0,0XFC,0XC0,0XFC,0XC1,0XE4,0X82,
0XFE,0X6C,0XF6,0X70,0XFF,0X37,0XFF,0XFE,0XFF,0XDF,0XFF,0XFF,0XFF,0XFF,0XEF,0XFD,
0X6E,0X6E,0X25,0X05,0X35,0X27,0X3C,0XE7,0X4C,0X89,0XCF,0XD9,0XF7,0XFE,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XF7,0XFF,0XFF,0XFF,0XF7,0XFF,0XE7,0XFE,0X7D,0XB0,
0X3C,0XA8,0X35,0X26,0X35,0X45,0X3C,0XE7,0XD7,0XFA,0XF7,0XFF,0XF7,0XFF,0XF7,0XFF,
0XFF,0XFF,0XFF,0XFD,0XE6,0X52,0XFE,0XF0,0XE5,0X05,0XEC,0XA1,0XFC,0XC0,0XFC,0XE0,
0XFC,0XE0,0XFC,0XE0,0XFC,0XE0,0XEC,0X81,0XED,0X05,0XFE,0XAF,0XE6,0X31,0XFF,0XFD,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XEF,0XFC,0X4E,0X2A,0X36,0X67,0X77,0XCF,
0X87,0XB0,0X8F,0X71,0XCF,0XFA,0XE7,0XFC,0XEF,0XFD,0XEF,0XFD,0XF7,0XFE,0XEF,0XFD,
0XEF,0XFE,0XEF,0XFE,0XE7,0XFD,0XD7,0XFC,0XA7,0XF5,0X87,0XB0,0X77,0XCE,0X6F,0XED,
0X2D,0X05,0XC7,0X98,0XFF,0XFF,0XF7,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFE,0XFF,0X98,
0XEE,0X2F,0XFE,0X4C,0XDC,0X62,0XF4,0XC1,0XFC,0XC0,0XFC,0XC0,0XFC,0XE0,0XFC,0XC0,
0XE4,0X81,0XFE,0X0B,0XF6,0X50,0XFF,0X57,0XFF,0XFE,0XFF,0XDF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFE,0XEF,0XFC,0X4D,0XEA,0X56,0XEC,0XC7,0XF9,0XCF,0XFA,0XC7,0XF9,0XC7,0XF8,
0XC7,0XF9,0XC7,0XF8,0XCF,0XF9,0XCF,0XF9,0XC7,0XF8,0XCF,0XF9,0XC7,0XF9,0XBF,0XF7,
0XC7,0XF9,0XC7,0XF8,0XC7,0XF7,0XBF,0XF7,0XBF,0XF7,0X3C,0XA7,0XCF,0XB9,0XF7,0XFE,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFE,0XFF,0XFC,0XEE,0X51,0XFE,0XCF,0XE4,0XE5,
0XEC,0XA2,0XFC,0XC0,0XFC,0XC0,0XFC,0XE0,0XF4,0XC0,0XE4,0XA3,0XFE,0X6E,0XEE,0X11,
0XFF,0XFC,0XFF,0XFE,0XFF,0XFF,0XFF,0XFF,0XFF,0XFE,0XFF,0XFF,0XEF,0XFD,0X55,0XAB,
0X5E,0X8C,0XB7,0XF7,0X3C,0X87,0X2D,0X06,0X25,0X25,0X25,0X46,0X25,0X25,0X2D,0X45,
0X25,0X25,0X2D,0X45,0X25,0X45,0X25,0X45,0X25,0X65,0X25,0X45,0X25,0X25,0X2D,0X25,
0X35,0X06,0XC7,0XF9,0X44,0X69,0XCF,0X99,0XF7,0XFE,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFD,0XFF,0X37,0XFE,0X70,0XF5,0X89,0XE4,0X61,0XFC,0XE1,0XFC,0XC0,
0XFC,0XE0,0XEC,0XA0,0XED,0X26,0XFE,0X90,0XF6,0XB4,0XFF,0XFD,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XEF,0XFD,0X65,0X8D,0X6E,0X8E,0X87,0XB1,0X25,0X45,
0X15,0XE4,0X0E,0X04,0X0E,0X04,0X16,0X24,0X16,0X03,0X16,0X03,0X16,0X03,0X16,0X03,
0X16,0X24,0X16,0X03,0X0E,0X03,0X16,0X23,0X0D,0XE3,0X1D,0X64,0XCF,0XF9,0X54,0X2A,
0XDF,0X9B,0XF7,0XFE,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFE,0XFF,0XFC,
0XEE,0X10,0XFE,0X6E,0XE4,0X61,0XFC,0XC0,0XFC,0XC0,0XFC,0XE0,0XEC,0X80,0XFD,0XC9,
0XF6,0X70,0XFF,0X78,0XFF,0XFE,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XF7,0XFE,0XDF,0XFC,0XCF,0XFA,0X87,0XF1,0X15,0XA3,0X06,0X23,0X06,0X82,0X06,0X21,
0X06,0X42,0X06,0X42,0X06,0X22,0X0E,0X43,0X0E,0X23,0X06,0X22,0X06,0X62,0X06,0X62,
0X06,0X62,0X06,0X62,0X15,0XA3,0XCF,0XF9,0XE7,0XFC,0XF7,0XFE,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFD,0XF6,0X52,0XFE,0X6E,0XE4,0XC2,
0XF4,0XC0,0XFC,0XC0,0XFC,0XE0,0XE4,0XA0,0XFE,0X4B,0XEE,0X50,0XFF,0XFC,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XDF,0XFF,0XFF,0XFF,0XFF,0XF7,0XFF,0XDF,0XFC,
0X87,0XB0,0X1D,0XA3,0X0E,0X22,0X06,0X61,0X06,0X62,0X06,0X42,0X06,0X22,0X16,0X24,
0X57,0X8B,0X15,0XA3,0X06,0X22,0X06,0X62,0X06,0X62,0X06,0X62,0X06,0X41,0X1D,0X84,
0XDF,0XFB,0XFF,0XFE,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFD,0XFE,0XD5,0XFE,0X6F,0XED,0X25,0XEC,0XA1,0XFC,0XA0,0XFC,0XC0,
0XE4,0XA1,0XFE,0X6D,0XEE,0X51,0XFF,0XFD,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XEF,0XFD,0X87,0X70,0X1D,0XA3,0X0E,0X42,
0X06,0X41,0X06,0X61,0X0E,0X42,0X15,0XC4,0X57,0X2C,0XB7,0XF8,0X25,0X66,0X16,0X23,
0X06,0X41,0X06,0X62,0X06,0X62,0X06,0X62,0X15,0X63,0XDF,0XFC,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFD,0XFF,0X37,
0XFE,0X6F,0XFD,0X88,0XE4,0X81,0XFC,0XA0,0XFC,0XC0,0XEC,0X82,0XFE,0X8E,0XF6,0X53,
0XFF,0XFE,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XDF,0XFF,0XFF,0XFF,0XDF,0XFF,0XFF,
0XFF,0XDF,0XEF,0XFD,0X87,0X90,0X1D,0XC4,0X0E,0X23,0X0E,0X62,0X0E,0X62,0X15,0XE3,
0X25,0X05,0XC7,0XF8,0XD7,0XFB,0X76,0X6F,0X15,0XA3,0X06,0X41,0X06,0X62,0X06,0X62,
0X06,0X61,0X15,0XA4,0XDF,0XFC,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFE,0XFF,0X78,0XFE,0X50,0XFD,0XCA,0XE4,0X61,
0XFC,0XA1,0XFC,0XA0,0XEC,0X82,0XFE,0XB0,0XF6,0X73,0XFF,0XFE,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XDF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XEF,0XFD,0X87,0XB1,
0X0D,0X82,0X0E,0X23,0X06,0X42,0X0E,0X02,0X1D,0XA4,0X9F,0X93,0XE7,0XFC,0XEF,0XFD,
0XDF,0XFB,0X2D,0X86,0X0E,0X03,0X0E,0X42,0X06,0X41,0X06,0X81,0X15,0X83,0XDF,0XFC,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFE,0XFF,0X99,0XFE,0X50,0XFD,0XEB,0XDC,0X62,0XFC,0XC1,0XFC,0XA0,0XE4,0X82,
0XFE,0X8F,0XEE,0X53,0XFF,0XFE,0XFF,0XFF,0XF7,0XFF,0XF7,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XDF,0XFF,0XFF,0XEF,0XFD,0X7F,0X90,0X15,0XC3,0X06,0X43,0X06,0X43,
0X1D,0XC4,0X5E,0X4B,0XDF,0XFB,0XF7,0XFE,0XF7,0XFE,0XEF,0XFD,0XB7,0XF6,0X25,0XA4,
0X0E,0X22,0X06,0X61,0X06,0X61,0X1D,0X63,0XE7,0XFC,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFE,0XFF,0X79,0XFE,0X50,
0XFD,0XEB,0XDC,0X41,0XFC,0XC1,0XFC,0XA0,0XEC,0XA3,0XFE,0X8F,0XEE,0X52,0XFF,0XFD,
0XFF,0XFF,0XF7,0XFF,0XF7,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XEF,0XFD,0X87,0X90,0X0D,0XC3,0X06,0X42,0X06,0X23,0X1D,0X64,0XC7,0XF8,0XEF,0XFD,
0XFF,0XFF,0XFF,0XFF,0XF7,0XFE,0XD7,0XFA,0X56,0X8B,0X0E,0X02,0X06,0X61,0X06,0X41,
0X1D,0X84,0XE7,0XFC,0XFF,0XDF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0X79,0XFE,0X50,0XFD,0XEB,0XDC,0X41,0XFC,0XC1,
0XF4,0XA0,0XE4,0XA2,0XFE,0X8D,0XE6,0X50,0XFF,0XFD,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XDF,0XF7,0XFD,0X8F,0XB1,0X0D,0XA2,
0X06,0X22,0X0E,0X03,0X25,0X65,0XCF,0XF9,0XEF,0XFD,0XFF,0XFF,0XFF,0XDF,0XF7,0XFF,
0XDF,0XFC,0X77,0X0F,0X0D,0XC3,0X06,0X61,0X0E,0X82,0X1D,0X63,0XE7,0XFC,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFE,0XF6,0XFE,0X90,0XF5,0X88,0XE4,0X61,0XFC,0XC1,0XFC,0XE0,0XDC,0X40,0XFE,0X6C,
0XEE,0X50,0XFF,0XDB,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XDF,0XFF,0XFF,0XEF,0XFC,0X87,0X90,0X1D,0XC3,0X0E,0X23,0X0E,0X23,0X1D,0X84,
0XA7,0XF5,0XDF,0XFC,0XEF,0XFE,0XEF,0XFF,0XE7,0XFD,0XD7,0XFB,0X56,0X2B,0X16,0X24,
0X06,0X61,0X06,0X41,0X1D,0X83,0XDF,0XFC,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XDE,0XFE,0XB5,0XFE,0X6F,0XF5,0X46,
0XEC,0X81,0XF4,0XC0,0XFC,0XC0,0XDC,0X60,0XFE,0X0B,0XF6,0X50,0XFF,0X38,0XFF,0XFE,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XF7,0XFE,0XDF,0XFB,
0X87,0XB0,0X1D,0X83,0X16,0X23,0X0E,0X43,0X0E,0X02,0X1D,0XC4,0XB7,0XF7,0XCF,0XFA,
0XCF,0XFA,0XCF,0XFA,0X8F,0XF2,0X15,0X64,0X0E,0X22,0X06,0X61,0X06,0X82,0X1D,0XA4,
0XD7,0XFB,0XF7,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFE,0XF6,0X31,0XFE,0XAF,0XEC,0XC3,0XEC,0XA0,0XF4,0XC0,0XF4,0XC0,
0XEC,0X81,0XF5,0X68,0XFE,0X70,0XF6,0X74,0XFF,0XFD,0XFF,0XFE,0XFF,0XFF,0XF7,0XFF,
0XFF,0XFF,0XFF,0XDF,0XF7,0XFF,0XA6,0XF5,0X9F,0XD4,0X87,0XF1,0X1D,0X84,0X16,0X23,
0X06,0X21,0X06,0X62,0X0E,0X02,0X25,0XA5,0X4E,0X4A,0X66,0XED,0X3E,0X08,0X1D,0X84,
0X16,0X03,0X0E,0X62,0X06,0X61,0X06,0X61,0X0D,0X82,0XCF,0XFA,0X95,0XB2,0XE7,0XDD,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFC,0XED,0XCF,
0XFE,0XCE,0XDC,0X61,0XF4,0XC0,0XF4,0XC0,0XFC,0XE0,0XEC,0X81,0XE4,0X84,0XFE,0XB0,
0XED,0XF0,0XFF,0XFC,0XFF,0XFE,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XDF,0XEF,0XFE,
0X55,0XAC,0X56,0X8B,0X87,0XF1,0X1D,0X84,0X0E,0X22,0X06,0X61,0X06,0X41,0X0E,0X62,
0X0E,0X22,0X0E,0X02,0X0D,0XE2,0X16,0X03,0X0E,0X22,0X0E,0X42,0X06,0X41,0X06,0X82,
0X06,0X41,0X15,0XC2,0XBF,0XF8,0X44,0X29,0XCF,0X7A,0XF7,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFE,0XFE,0XF6,0XFE,0X4F,0XFE,0X2B,0XDC,0X40,0XFC,0XE0,
0XFC,0XC0,0XFC,0XE0,0XF4,0XA0,0XDC,0X22,0XFE,0X6E,0XFE,0X30,0XFF,0X16,0XFF,0XFE,
0XFF,0XFF,0XF7,0XDF,0XFF,0XFF,0XFF,0XFF,0XE7,0XFD,0X4E,0X0A,0X57,0X4B,0X97,0XF3,
0X25,0X45,0X15,0XA4,0X15,0XC3,0X1D,0X83,0X1D,0X63,0X15,0XA3,0X15,0XA3,0X1D,0X84,
0X1D,0X84,0X1D,0XA4,0X1D,0XA3,0X15,0XA3,0X0D,0XA2,0X15,0XA2,0X2E,0X25,0XB7,0XF6,
0X3C,0XA8,0XC7,0X99,0XF7,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFD,
0XE5,0XF1,0XFE,0XD1,0XE5,0X06,0XEC,0X80,0XFC,0XC0,0XFC,0XC0,0XFC,0XE0,0XF4,0XE0,
0XDC,0X61,0XF5,0X68,0XFE,0XB0,0XE5,0XF1,0XFF,0XFD,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XEF,0XFD,0X4E,0X29,0X4F,0X0A,0XB7,0XF6,0XBF,0XF8,0XC7,0XF9,0XC7,0XF8,
0XD7,0XFA,0XD7,0XFA,0XD7,0XFB,0XD7,0XFB,0XD7,0XFA,0XD7,0XFB,0XD7,0XFA,0XDF,0XFA,
0XCF,0XF9,0XC7,0XF9,0XBF,0XF8,0XB7,0XF6,0XAF,0XF6,0X2C,0XE6,0XC7,0XD9,0XF7,0XFE,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFE,0XFF,0X58,0XEE,0X10,0XFE,0X8E,0XCC,0X42,
0XEC,0XC1,0XFC,0XC0,0XFC,0XC0,0XFC,0XE0,0XFC,0XE0,0XEC,0X80,0XD4,0X21,0XFE,0XAF,
0XED,0XEF,0XFF,0X18,0XFF,0XFE,0XFF,0XFE,0XFF,0XFF,0XFF,0XFF,0XEF,0XFD,0X55,0XCB,
0X2D,0X25,0X3C,0XE7,0X44,0XA8,0X54,0X8B,0XDF,0XFB,0XF7,0XFE,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XEF,0XFE,0X9E,0X34,0X4C,0X69,
0X44,0XE8,0X2D,0X06,0X34,0XE7,0XC7,0X99,0XF7,0XFE,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFD,0XEE,0X11,0XFE,0X70,0XF5,0XA9,0XCC,0X41,0XF4,0XC1,0XFC,0XC0,0XFC,0XE0,
0XFC,0XC0,0XFC,0XC0,0XF4,0XC1,0XDC,0X21,0XED,0X68,0XFE,0XB0,0XDD,0X8F,0XFF,0XFC,
0XFF,0XFE,0XFF,0XFF,0XFF,0XFF,0XF7,0XFE,0XBF,0X98,0X9E,0XD3,0XA6,0XF3,0XA6,0XB3,
0XAE,0X95,0XE7,0XFC,0XF7,0XFE,0XFF,0XFF,0XFF,0XFF,0XFF,0XDF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XF7,0XFE,0XC7,0X59,0XA6,0X94,0X9E,0XB4,0X9F,0X13,0X9E,0XD4,
0XE7,0XFE,0XF7,0XFF,0XFF,0XFF,0XFF,0XFE,0XFF,0XFD,0XF6,0XD5,0XF5,0XEF,0XFE,0XCF,
0XCC,0X01,0XE4,0X81,0XFC,0XC0,0XFC,0XE0,0XFC,0XE0,0XFC,0XE0,0XFC,0XC0,0XFC,0XC0,
0XEC,0X81,0XCB,0XC0,0XFE,0X8E,0XFE,0X50,0XE6,0X12,0XFF,0XFD,0XFF,0XFE,0XFF,0XFF,
0XF7,0XFF,0XF7,0XFE,0XEF,0XFD,0XEF,0XFD,0XEF,0XFD,0XF7,0XFE,0XF7,0XFE,0XFF,0XFE,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XDF,0XFF,0XFF,0XFF,0XFF,0XF7,0XFF,
0XF7,0XFE,0XEF,0XFE,0XEF,0XFD,0XE7,0XFC,0XEF,0XFE,0XF7,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFE,0XFF,0XB9,0XDD,0XAF,0XFE,0XB0,0XED,0X26,0XE4,0X41,0XF4,0XA0,0XFC,0XE0,
0XFC,0XE0,0XFC,0XE0,0XFC,0XE0,0XFD,0X00,0XFC,0XE0,0XF4,0XC0,0XDC,0X40,0XDC,0XA4,
0XFE,0XD0,0XE5,0XCF,0XF6,0X94,0XFF,0XFD,0XFF,0XFF,0XF7,0XFF,0XF7,0XFF,0XF7,0XFF,
0XF7,0XFF,0XF7,0XFE,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XDF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XF7,0XFF,0XF7,0XFE,0XFF,0XFF,0XFF,0XFF,0XFF,0XFE,0XFF,0XFC,0XDD,0XB0,0XFE,0X4F,
0XFE,0X2B,0XD4,0X21,0XEC,0XA1,0XFC,0XC0,0XFC,0XE0,0XFC,0XE0,0XFC,0XE0,0XFD,0X00,
0XFC,0XE0,0XFD,0X00,0XFC,0XC0,0XFC,0XE2,0XD4,0X00,0XED,0X28,0XFE,0XD0,0XE5,0XAF,
0XEE,0XB5,0XFF,0XFD,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XDF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XDF,0XFF,0XFF,0XFF,0XFF,0XFF,0XDF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFE,0XFF,0XFC,0XD5,0X90,0XF6,0X2F,0XFE,0XAE,0XCC,0X22,0XDC,0X60,0XF4,0XC0,
0XFC,0XE0,0XFC,0XE0,0XFC,0XE0,0XFC,0XC0,0XFD,0X00,0XFC,0XC0,0XFC,0XE0,0XFC,0XC0,
0XFC,0XA0,0XF4,0XC1,0XD4,0X00,0XFD,0XA9,0XFE,0XB0,0XD5,0X8E,0XE6,0X73,0XFF,0XFC,
0XFF,0XFE,0XFF,0XFF,0XFF,0XFF,0XFF,0XDF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFD,0XFF,0XBA,0XD5,0X8F,0XED,0XEE,
0XFE,0XCF,0XD4,0X63,0XDC,0X40,0XF4,0XC1,0XFC,0XC0,0XFC,0XE0,0XFC,0XE0,0XFC,0XE0,
0XFC,0XC0,0XFC,0XE0,0XFC,0XE0,0XFC,0XE0,0XFC,0XC0,0XFC,0XE0,0XF4,0XA0,0XEC,0XA1,
0XD4,0X20,0XFD,0XCA,0XFE,0XD0,0XE5,0XEE,0XDD,0XD0,0XFF,0XFC,0XFF,0XFD,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XDF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFE,0XFF,0XFE,
0XFF,0XFE,0XF6,0XF6,0XD5,0X8E,0XFE,0X4F,0XFE,0XF0,0XDC,0X83,0XDC,0X61,0XF4,0XA0,
0XFC,0XC0,0XFC,0XC0,0XFC,0XE0,0XFC,0XE0,0XFC,0XE0,0XFC,0XE0,0XFC,0XC0,0XFC,0XC0,
0XFC,0XC0,0XFC,0XE0,0XFC,0XE0,0XF4,0XA0,0XFC,0XE1,0XE4,0X60,0XCB,0XE0,0XF5,0X87,
0XFE,0XF0,0XED,0XEF,0XD5,0X6E,0XF6,0XB4,0XFF,0XFC,0XFF,0XFE,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFE,0XFF,0XFD,0XFF,0XFC,0XFF,0X78,0XDD,0XD0,0XDD,0X8D,0XFE,0X8F,
0XFE,0XCE,0XCB,0XE1,0XDC,0X20,0XF4,0XA0,0XFC,0XC0,0XFC,0XE0,0XFC,0XC0,0XFC,0XC0,
0XFC,0XE0,0XFC,0XE0,0XFC,0XE0,0XFC,0XC0,0XFC,0XC0,0XFC,0XA0,0XFC,0XC0,0XFC,0XC0,
0XFC,0XE0,0XFC,0XC0,0XF4,0XC0,0XF4,0XA0,0XCB,0XE0,0XDC,0XA4,0XFE,0XAE,0XFE,0X8F,
0XED,0XCE,0XCD,0X4E,0XEE,0X74,0XFF,0X9A,0XFF,0XFD,0XFF,0XFE,0XFF,0XFE,0XFF,0XFE,
0XFF,0XFE,0XFF,0XFF,0XFF,0XFF,0XFF,0XFE,0XFF,0XFE,0XFF,0XFD,0XFF,0XFC,0XFF,0X36,
0XDD,0XD0,0XDD,0X8E,0XF6,0X2E,0XFE,0XCF,0XFD,0XEA,0XC3,0XC0,0XE4,0X61,0XF4,0XC1,
0XFC,0XA0,0XFC,0XE0,0XFC,0XC0,0XFC,0XC0,0XFC,0XC0,0XFC,0XC0,0XFC,0XE0,0XFC,0XE0,
0XFC,0XC0,0XFC,0XC0,0XFC,0XC0,0XFC,0XE0,0XFC,0XE0,0XFC,0XE0,0XFC,0XE0,0XFC,0XE0,
0XF4,0XC0,0XF4,0XA1,0XDC,0X00,0XD4,0X01,0XF5,0X68,0XFE,0XAF,0XFE,0X90,0XED,0XCE,
0XDD,0X6D,0XDD,0X8F,0XEE,0X32,0XFE,0XB4,0XFF,0X37,0XFF,0X58,0XFF,0X58,0XFF,0X57,
0XFE,0XF6,0XFE,0X93,0XE5,0XF0,0XDD,0X6D,0XE5,0X8D,0XFE,0X2F,0XFE,0X70,0XFE,0X6E,
0XDC,0XA4,0XCC,0X00,0XE4,0X40,0XF4,0XC1,0XFC,0XE0,0XFC,0XC0,0XFC,0XC0,0XFC,0XC0,
0XFC,0XC0,0XFC,0XC0,0XFC,0XE0,0XFC,0XE0,0XFC,0XE0,0XFC,0XE0,0XFC,0XE0,0XFC,0XE0,
0XFC,0XE0,0XFC,0XE0,0XFC,0XE0,0XFC,0XE0,0XFC,0XE0,0XFC,0XC0,0XFC,0XE1,0XFC,0XC1,
0XEC,0X60,0XD4,0X01,0XCC,0X23,0XED,0X48,0XFE,0X8D,0XFE,0XAF,0XFE,0X8F,0XFE,0X0E,
0XED,0XCD,0XE5,0XAD,0XDD,0XAE,0XDD,0XAE,0XDD,0X8D,0XED,0XAD,0XF5,0XED,0XFE,0X2E,
0XFE,0X8F,0XFE,0XAF,0XFE,0X0C,0XDC,0XC6,0XBB,0XA1,0XDC,0X41,0XE4,0X60,0XFC,0XE1,
0XFC,0XE0,0XFC,0XC0,0XFC,0XE0,0XFC,0XE0,0XFC,0XC0,0XFC,0XC0,0XFC,0XE0,0XFC,0XE0,
0XFC,0XE0,0XFC,0XE0,0XFC,0XE0,0XFC,0XE0,0XFC,0XE0,0XFC,0XE0,0XFC,0XE0,0XFC,0XE0,
0XFC,0XE0,0XFC,0XE0,0XFC,0XC0,0XFC,0XC0,0XFC,0XC0,0XFC,0XC0,0XEC,0XA0,0XE4,0X40,
0XD4,0X00,0XCB,0XE0,0XD4,0X64,0XED,0X27,0XFE,0X0B,0XFE,0X8D,0XFE,0XAE,0XFE,0XAE,
0XFE,0XAF,0XFE,0XCF,0XFE,0X8E,0XFE,0X4D,0XFD,0XCA,0XED,0X06,0XD4,0X22,0XD3,0XE0,
0XDC,0X20,0XEC,0X81,0XF4,0X80,0XF4,0XA0,0XFC,0XE0,0XFC,0XE0,0XFC,0XC0,0XFC,0XE0,
0XFC,0XE0,0XFC,0XE0,0XFC,0XE0,0XFC,0XE0,0XFC,0XE0,0XFC,0XE0,0XFC,0XE0,0XFC,0XE0,
0XFC,0XE0,0XFC,0XE0,0XFC,0XE0,0XFC,0XE0,0XFC,0XE0,0XFC,0XE0,0XFC,0XE0,0XFD,0X00,
0XFC,0XC0,0XFC,0XE0,0XFC,0XE0,0XFC,0XE0,0XFC,0XC0,0XF4,0XA0,0XEC,0X80,0XDC,0X40,
0XD4,0X21,0XCB,0XE0,0XCB,0XE0,0XCB,0XE1,0XCC,0X01,0XC4,0X01,0XC3,0XE1,0XC3,0XE1,
0XC3,0XC0,0XCB,0XE0,0XD4,0X20,0XE4,0X60,0XF4,0XA0,0XFC,0XC0,0XFC,0XC0,0XFC,0XE0,
0XFC,0XC0,0XFC,0XA0,0XFC,0XC0,0XFC,0XE0,0XFC,0XE0,0XFC,0XE0,0XFD,0X00,0XFC,0XE0,
0XFC,0XE0,0XFC,0XE0,0XFC,0XE0,0XFC,0XE0,0XFC,0XE0,0XFC,0XE0,0XFC,0XE0,0XFC,0XE0,
0XFC,0XE0,0XFC,0XE0,0XFC,0XC0,0XFC,0XE0,0XFD,0X00,0XFC,0XC0,0XFC,0XE0,0XFC,0XE0,
0XFC,0XC0,0XFC,0XC0,0XFC,0XC0,0XFC,0XC0,0XF4,0XC0,0XEC,0XA0,0XEC,0X80,0XEC,0X81,
0XEC,0X61,0XE4,0X61,0XE4,0X61,0XE4,0X61,0XE4,0X81,0XEC,0X81,0XEC,0X80,0XF4,0XA0,
0XFC,0XC0,0XFC,0XE0,0XFC,0XE0,0XFC,0XE0,0XFC,0XC0,0XFC,0XE0,0XFC,0XE0,0XFC,0XE0,
0XFC,0XE0,0XFC,0XE0,0XFC,0XE0,0XFC,0XE0,0XFC,0XE0,0XFC,0XE0,0XFC,0XE0,0XFC,0XE0,
0XFC,0XE0,};

 
const unsigned char gImage_PIC_Quick[4050] = {  
0XFC,0XE0,0XFD,0X00,0XFC,0XC0,0XFC,0XC0,0XFC,0XE0,0XFC,0XC0,0XFC,0XE0,0XFC,0XE0,
0XFC,0XC0,0XFC,0XC0,0XFC,0XC0,0XFC,0XE0,0XFC,0XE0,0XFC,0XE0,0XFD,0X00,0XF4,0XC0,
0XF4,0XC0,0XF4,0XA1,0XEC,0X82,0XE4,0X62,0XDC,0X63,0XDC,0X83,0XDC,0X84,0XDC,0X63,
0XE4,0X64,0XE4,0X63,0XEC,0X82,0XEC,0XA1,0XF4,0XA0,0XFC,0XE0,0XFC,0XE0,0XFC,0XE0,
0XFC,0XE0,0XFC,0XC0,0XFC,0XE0,0XFC,0XE0,0XFC,0XC0,0XFC,0XE0,0XFC,0XE0,0XFD,0X00,
0XFD,0X00,0XFD,0X00,0XFC,0XE0,0XFC,0XC0,0XFC,0XC0,0XFC,0XE0,0XFD,0X00,0XFC,0XC0,
0XFC,0XC0,0XFC,0XC0,0XFC,0XC0,0XFD,0X00,0XFC,0XE0,0XFC,0XC0,0XFC,0XC0,0XFC,0XE0,
0XFC,0XC0,0XFC,0XC0,0XFC,0XC0,0XEC,0X80,0XE4,0XA1,0XEC,0XC3,0XFD,0X66,0XFE,0X0A,
0XFE,0X4C,0XFE,0X8D,0XFE,0XAE,0XFE,0XAF,0XFE,0XCF,0XFE,0X6D,0XFE,0X4C,0XFE,0X0A,
0XFD,0X66,0XEC,0XC2,0XEC,0XA1,0XF4,0XC1,0XF4,0XA0,0XFC,0XE0,0XFC,0XE0,0XFC,0XA0,
0XFC,0XC0,0XFC,0XE0,0XFC,0XE0,0XFC,0XE0,0XFC,0XC0,0XFC,0XE0,0XFC,0XE0,0XFC,0XE0,
0XFC,0XC0,0XFC,0XC0,0XFC,0XE0,0XFC,0XE0,0XFC,0XC0,0XFC,0XC0,0XFC,0XE0,0XFC,0XE0,
0XFC,0XE0,0XFC,0XC0,0XFC,0XE0,0XFC,0XC0,0XFC,0XC0,0XFC,0XC0,0XF4,0XA1,0XE4,0X82,
0XF5,0X47,0XFE,0X6C,0XFE,0XCF,0XFE,0X8F,0XFE,0X4F,0XF6,0X30,0XF6,0X51,0XF6,0X52,
0XF6,0X52,0XEE,0X51,0XF6,0X70,0XF6,0X4F,0XF6,0X4E,0XFE,0X8F,0XFE,0XCF,0XFE,0X2B,
0XED,0X46,0XE4,0XA2,0XEC,0XC1,0XFC,0XE0,0XFD,0X00,0XFC,0XC0,0XFC,0XE0,0XFC,0XC0,
0XFC,0XC0,0XFC,0XE0,0XFC,0XE0,0XFC,0XE0,0XFC,0XE0,0XFC,0XE0,0XFC,0XE0,0XFC,0XE0,
0XFC,0XC0,0XFC,0XC0,0XFC,0XE0,0XFC,0XE0,0XFC,0XE0,0XFC,0XE0,0XFC,0XE0,0XFC,0XC0,
0XFC,0XE0,0XF4,0XA0,0XEC,0XA2,0XFD,0X46,0XFE,0X6D,0XFE,0XB0,0XEE,0X0F,0XEE,0X51,
0XF6,0XD4,0XFF,0X77,0XFF,0XBA,0XFF,0XDB,0XFF,0XFC,0XFF,0XFC,0XFF,0XFB,0XFF,0XFA,
0XFF,0XD9,0XFF,0X77,0XFE,0XD4,0XF6,0X31,0XFE,0X50,0XFE,0XAF,0XFE,0X8C,0XF5,0X65,
0XEC,0XA1,0XF4,0XC0,0XFC,0XC0,0XFC,0XC0,0XFC,0XC0,0XFC,0XC0,0XFC,0XA0,0XFC,0XC0,
0XFC,0XC0,0XFC,0XE0,0XFC,0XE0,0XFC,0XE0,0XFC,0XE0,0XFC,0XE0,0XFC,0XE0,0XFC,0XE0,
0XFC,0XC0,0XFC,0XC0,0XFC,0XE0,0XFD,0X00,0XFC,0XE0,0XE4,0X80,0XEC,0XE4,0XFE,0X2C,
0XFE,0XAF,0XF6,0X2F,0XF6,0X93,0XFF,0X79,0XFF,0XFC,0XFF,0XFD,0XFF,0XFE,0XFF,0XFE,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFE,0XFF,0XFE,0XFF,0XFE,0XFF,0XFE,0XFF,0XFE,0XFF,0XFE,
0XFF,0XFD,0XFF,0X38,0XEE,0X73,0XEE,0X2F,0XFE,0XAF,0XFE,0X4B,0XE4,0XC3,0XF4,0XC1,
0XF4,0XC0,0XFC,0XE0,0XFC,0XC0,0XFC,0XC0,0XFC,0XC0,0XFC,0XC0,0XFC,0XE0,0XFC,0XE0,
0XFC,0XE0,0XFC,0XE0,0XFC,0XE0,0XFC,0XE0,0XFC,0XC0,0XFC,0XC0,0XFC,0XE0,0XFC,0XC0,
0XF4,0XA0,0XEC,0XA1,0XF5,0X66,0XFE,0X8E,0XFE,0X50,0XF6,0X51,0XFF,0X98,0XFF,0XFC,
0XFF,0XFE,0XFF,0XFE,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFE,0XFF,0XFC,
0XFF,0X98,0XEE,0X51,0XFE,0X4F,0XFE,0X8E,0XED,0X25,0XEC,0XC1,0XF4,0XC0,0XFC,0XC0,
0XFC,0XE0,0XFC,0XC0,0XFC,0XC0,0XFC,0XC0,0XFC,0XE0,0XFC,0XE0,0XFC,0XC0,0XFC,0XC0,
0XFC,0XC0,0XFC,0XC0,0XFC,0XE0,0XFC,0XC1,0XFC,0X80,0XF4,0XA2,0XF5,0X67,0XFE,0XAE,
0XF6,0X30,0XFE,0XD5,0XFF,0XFC,0XFF,0XFE,0XFF,0XFE,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFE,0XFF,0XFE,0XFF,0XFD,0XFE,0XB6,
0XEE,0X10,0XFE,0XAF,0XF5,0XA7,0XE4,0X80,0XF4,0XE0,0XFC,0XC0,0XFC,0XC0,0XFC,0XC0,
0XFC,0XC0,0XFC,0XE0,0XFC,0XE0,0XFC,0XC0,0XFC,0XE0,0XFC,0XE0,0XFC,0XE0,0XFC,0XC0,
0XF4,0XA0,0XF4,0XA2,0XFD,0X67,0XFE,0XCF,0XF6,0X30,0XFF,0X37,0XFF,0XFD,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFD,0XFF,0X58,0XEE,0X10,0XFE,0XAF,
0XF5,0X87,0XE4,0XA1,0XF4,0XC0,0XFC,0XC0,0XFC,0XC0,0XFC,0XE0,0XFC,0XE0,0XFC,0XE0,
0XFC,0XE0,0XFD,0X00,0XFC,0XE0,0XFC,0XE0,0XFC,0XC0,0XF4,0XC1,0XED,0X05,0XFE,0XCF,
0XF6,0X10,0XFF,0X79,0XFF,0XFE,0XFF,0XFF,0XFF,0XDF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XDF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFE,0XFF,0X58,0XEE,0X30,0XFE,0XAF,0XED,0X47,0XE4,0X81,
0XFC,0XC0,0XFC,0XE0,0XFC,0XE0,0XFD,0X00,0XFC,0XE0,0XF4,0XE0,0XFD,0X20,0XFC,0XE0,
0XFC,0XC0,0XF4,0XC0,0XE4,0XA2,0XFE,0XCE,0XEE,0X0F,0XFF,0X38,0XFF,0XFE,0XFF,0XFF,
0XFF,0XDF,0XFF,0XFF,0XFF,0XFF,0XF7,0XDF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XDF,0XFF,0XFF,0XFF,0XBF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XF7,0XFF,0XFF,0XFF,
0XFF,0XFE,0XFF,0X38,0XEE,0X30,0XFE,0X8F,0XE4,0XA4,0XF4,0XA1,0XFC,0XE0,0XFC,0XE0,
0XFD,0X00,0XF4,0XC0,0XFD,0X00,0XFC,0XC0,0XFC,0XA0,0XFC,0XC1,0XE4,0X62,0XFE,0X4B,
0XFE,0X90,0XF6,0XF5,0XFF,0XFE,0XFF,0XFF,0XFF,0XDF,0XFF,0XFF,0XF7,0XDF,0XF7,0XFF,
0XF7,0XFF,0XFF,0XFF,0XFF,0XDF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XBF,0XFF,0XDF,
0XFF,0XDF,0XF7,0XDF,0XF7,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFE,0XFF,0XFE,0XFE,0XF6,
0XF6,0X2F,0XFE,0X4C,0XE4,0X81,0XFC,0XC0,0XFC,0XE0,0XFC,0XE0,0XFC,0XE0,0XFC,0XC0,
0XFC,0XC0,0XFC,0XC1,0XEC,0X81,0XF5,0X47,0XFE,0X8F,0XE6,0X51,0XFF,0XFC,0XFF,0XFF,
0XFF,0XFF,0XFF,0XDF,0XFF,0XFF,0XEF,0X7F,0XD6,0XDF,0XD6,0XDF,0XDE,0XFF,0XDE,0XDF,
0XFF,0XDF,0XFF,0XFF,0XF7,0XDE,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XDF,0XD6,0XBE,0XDE,0XBF,0XD6,0XDF,0XD6,0XDF,0XF7,0XBF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFE,0XFF,0XFD,0XF6,0X52,0XFE,0X6F,0XF5,0X26,
0XF4,0XA0,0XFC,0XE0,0XFC,0XE0,0XFD,0X00,0XFC,0XC0,0XFC,0XC1,0XF4,0XA1,0XE4,0X61,
0XFE,0X8D,0XF6,0X4F,0XFF,0X99,0XFF,0XFE,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XF7,0XBF,
0XAD,0X3E,0X63,0X18,0X6B,0X57,0X63,0X35,0X62,0XF3,0XC6,0X1D,0XF7,0XDF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XF7,0XDF,0XFF,0XFF,0XFF,0XFF,0XF7,0XFF,0XC6,0X3C,
0X63,0X13,0X6B,0X37,0X62,0XD7,0X6B,0X37,0XAD,0X5B,0XFF,0XDF,0XFF,0XDF,0XFF,0XFF,
0XFF,0XFE,0XFF,0XFD,0XFF,0X78,0XF6,0X51,0XFE,0X4C,0XEC,0XA2,0XFC,0XC0,0XFC,0XE0,
0XFC,0XC0,0XFC,0XA0,0XFC,0XC1,0XF4,0XC2,0XF5,0X26,0XFE,0X8F,0XF6,0X93,0XFF,0XFD,
0XFF,0XFF,0XFF,0XFF,0XF7,0XBE,0XFF,0XFF,0XF7,0XBF,0X9C,0XBE,0X83,0XDD,0XE6,0XFF,
0XEF,0X3F,0XEF,0X3F,0XEF,0X5F,0XEF,0X7F,0XEF,0XBF,0XF7,0XDF,0XF7,0XBF,0XF7,0XBF,
0XF7,0XBF,0XF7,0XDF,0XF7,0XDF,0XEF,0XBF,0XEF,0X7F,0XE7,0X3F,0XE6,0XFF,0XE6,0XDF,
0X83,0XFC,0XA5,0X1C,0XF7,0XBF,0XFF,0XDF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFE,0XFF,0XFD,
0XEE,0X53,0XFE,0XB0,0XED,0X05,0XEC,0XA1,0XFC,0XE0,0XFC,0XC0,0XFD,0X00,0XF4,0XC0,
0XE4,0X81,0XFE,0X0B,0XFE,0X71,0XFF,0X79,0XFF,0XFE,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XF7,0XBF,0X9D,0X1F,0X83,0XFD,0XEF,0X1F,0XC5,0XFF,0XB5,0X9F,0XAD,0X9F,
0XB5,0XBF,0XB5,0XBF,0XB5,0X9F,0XB5,0X9F,0XB5,0X9F,0XB5,0X7F,0XB5,0X9F,0XB5,0X9F,
0XAD,0X7F,0XB5,0X7F,0XB5,0X7F,0XB5,0X7F,0XE7,0X1F,0X84,0X3B,0X9D,0X3D,0XEF,0X7F,
0XF7,0XDF,0XFF,0XFF,0XFF,0XFE,0XFF,0XFF,0XFF,0XFE,0XFF,0X7A,0XFE,0X50,0XFE,0X0B,
0XE4,0XA1,0XF4,0XC0,0XFC,0XE0,0XFC,0XE0,0XF4,0XC0,0XE4,0XA3,0XFE,0XAF,0XE6,0X11,
0XFF,0XFD,0XFF,0XFE,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XF7,0XDF,0X9D,0X1D,
0X84,0X3B,0XDE,0XBF,0X6B,0X38,0X6B,0X5B,0X63,0X3C,0X63,0X3C,0X63,0X3B,0X6B,0X1C,
0X6B,0X3C,0X6B,0X1C,0X6B,0X3C,0X63,0X1C,0X6B,0X3C,0X63,0X3C,0X63,0X3C,0X6B,0X5C,
0X63,0X19,0XD6,0X9F,0X84,0X59,0X9D,0X3B,0XEF,0XBF,0XFF,0XDF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFD,0XEE,0X11,0XFE,0XCF,0XE4,0XA3,0XF4,0XE1,0XF4,0XC0,
0XFC,0XC0,0XEC,0XA1,0XF5,0X47,0XFE,0X90,0XF6,0XD5,0XFF,0XFE,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XF7,0XDF,0XBD,0XFD,0X9C,0XFB,0XD6,0X7F,0X6B,0X5A,
0X6B,0X5E,0X6B,0X5F,0X63,0X3E,0X6B,0X5E,0X6B,0X1F,0X6B,0X1E,0X6B,0X5E,0X6B,0X5E,
0X73,0X9F,0X63,0X5E,0X63,0X3F,0X63,0X3E,0X63,0X3E,0X6B,0X5B,0XD6,0XBF,0X9D,0X1A,
0XBE,0X1C,0XF7,0XBF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFD,
0XFE,0XD5,0XFE,0X6F,0XF5,0X47,0XEC,0X81,0XF4,0XC0,0XFC,0XC0,0XEC,0X81,0XFD,0XC9,
0XF6,0X50,0XFF,0X79,0XFF,0XFE,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XDF,0XFF,0XFF,
0XFF,0XDF,0XF7,0XDF,0XEF,0X7F,0XCE,0X5F,0X6B,0X3B,0X6B,0X3E,0X62,0XFF,0X6B,0X3F,
0X63,0X1F,0X6B,0X3F,0X6B,0X3F,0X5A,0XFB,0XA5,0X3F,0X73,0XBF,0X63,0X1F,0X63,0X1F,
0X63,0X3F,0X63,0X1F,0X5B,0X1B,0XCE,0X7F,0XF7,0XBF,0XF7,0XBF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XDF,0XFF,0XFF,0XFF,0XFE,0XFF,0X78,0XFE,0X70,0XFD,0XCA,
0XE4,0X82,0XF4,0XC1,0XFC,0XC0,0XEC,0X81,0XFE,0X4C,0XEE,0X30,0XFF,0XFC,0XFF,0XFE,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XF7,0XBF,
0XCE,0X3F,0X6B,0X1B,0X6B,0X3E,0X63,0X3F,0X63,0X3F,0X63,0X1F,0X5A,0XFF,0X63,0X1D,
0X94,0XBE,0XDE,0XFF,0X63,0X1D,0X62,0XFF,0X6B,0X3F,0X63,0X1F,0X5A,0XFF,0X5B,0X1C,
0XD6,0X9F,0XFF,0XBF,0XFF,0XDF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFE,0XFF,0XFB,0XF6,0X30,0XFE,0X2C,0XE4,0X81,0XF4,0XA1,0XFC,0XC0,
0XEC,0X81,0XFE,0X8D,0XEE,0X51,0XFF,0XFD,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XDF,0XFF,0XFF,0XF7,0XDF,0XEF,0X9F,0XD6,0X5F,0X63,0X1B,0X6B,0X5F,
0X63,0X1F,0X62,0XFF,0X63,0X3F,0X63,0X3E,0X6B,0X9C,0XE7,0X3F,0XB5,0XBF,0X63,0X1B,
0X6B,0X3F,0X63,0X1F,0X6B,0X3F,0X5B,0X1F,0X63,0X3D,0XD6,0X7F,0XFF,0XBF,0XFF,0XDF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XDF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFE,0XFF,0XFC,
0XEE,0X30,0XFE,0X8D,0XDC,0X62,0XF4,0XA1,0XFC,0XC0,0XE4,0X61,0XFE,0XAF,0XF6,0X52,
0XFF,0XFD,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XDF,0XCE,0X5F,0X6B,0X1B,0X63,0X3F,0X5B,0X1F,0X63,0X3F,0X63,0X5E,
0X6B,0X5A,0XD6,0XBF,0XE7,0X7F,0X8C,0X9A,0X6B,0X5A,0X6B,0X3D,0X6B,0X3E,0X63,0X3F,
0X63,0X1F,0X5A,0XFB,0XCE,0X9F,0XFF,0XDF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFE,0XFF,0XFD,0XF6,0X72,0XFE,0XCE,0XDC,0X62,
0XF4,0XA1,0XFC,0XC0,0XE4,0X61,0XFE,0XCF,0XEE,0X73,0XFF,0XFE,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XDF,0XCE,0X3F,
0X6B,0X3B,0X6B,0X5F,0X63,0X3F,0X63,0X3F,0X5A,0XFB,0XBD,0XDF,0XEF,0X7F,0XEF,0X9F,
0XA5,0X5B,0XAD,0X3F,0XAD,0X1F,0X84,0X1F,0X63,0X1F,0X63,0X1F,0X6B,0X3C,0XCE,0X9F,
0XF7,0XDF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XDF,
0XFF,0XFF,0XFF,0XFD,0XEE,0X93,0XFE,0XAF,0XDC,0X62,0XF4,0XA1,0XFC,0XC0,0XE4,0X61,
0XFE,0XB0,0XEE,0X53,0XFF,0XFE,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XF7,0XBF,0XD6,0X7F,0X6B,0X1B,0X63,0X3E,0X63,0X3F,
0X62,0XFE,0X9C,0XFF,0XE7,0X3F,0XEF,0X5F,0XF7,0X9F,0XF7,0X9F,0XEF,0X5F,0XDE,0XBF,
0X6B,0X3D,0X63,0X1F,0X63,0X1F,0X5A,0XFB,0XCE,0X9F,0XFF,0XDF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFD,0XEE,0X72,
0XFE,0XAF,0XDC,0X83,0XF4,0XC1,0XFC,0XC0,0XE4,0X82,0XFE,0X8F,0XE6,0X32,0XFF,0XFE,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XF7,0XBF,0XCE,0X5F,0X6B,0X3B,0X6B,0X5F,0X63,0X1F,0X63,0X3F,0X6B,0X3D,0X6B,0X39,
0X6B,0X35,0XD6,0X9F,0XF7,0X9F,0XE6,0XFF,0X73,0X5A,0X6B,0X3E,0X63,0X1F,0X5A,0XFF,
0X63,0X3C,0XD6,0X7F,0XFF,0XDF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFE,0XFF,0XFC,0XE6,0X31,0XFE,0XAF,0XDC,0X63,0XF4,0XA1,
0XFC,0XC0,0XE4,0X81,0XFE,0X8E,0XE5,0XF1,0XFF,0XFD,0XFF,0XFF,0XFF,0XDF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XF7,0XDF,0XCE,0X5F,0X63,0X1B,
0X6B,0X3E,0X63,0X1F,0X6B,0X1F,0X6B,0X1F,0X6B,0X5D,0X6B,0X7A,0XEF,0X7F,0XEF,0X5F,
0X73,0X7C,0X6B,0X3E,0X63,0X1F,0X63,0X3F,0X5B,0X1F,0X5B,0X1B,0XD6,0X9F,0XFF,0XDF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XDF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFB,0XEE,0X30,0XFE,0X6E,0XDC,0X62,0XF4,0XC1,0XFC,0XE0,0XE4,0X61,0XFE,0X2D,
0XEE,0X11,0XFF,0XDB,0XFF,0XFE,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XF7,0XFF,0XF7,0XBF,0XCE,0X5F,0X63,0X3B,0X63,0X3E,0X62,0XFF,0X63,0X1F,
0X62,0XFE,0X63,0X3E,0X9D,0X1F,0XDF,0X1F,0X7B,0XFA,0X63,0X1D,0X62,0XFF,0X63,0X1F,
0X63,0X1F,0X5A,0XFF,0X5B,0X1B,0XD6,0X9F,0XFF,0XDF,0XF7,0XDF,0XFF,0XFF,0XFF,0XDF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFE,0XFF,0XD9,0XF6,0X4F,0XFE,0X4C,
0XD4,0X41,0XF4,0XC0,0XFC,0XC0,0XE4,0X60,0XFD,0XCA,0XFE,0X70,0XF7,0X16,0XFF,0XFE,
0XFF,0XFE,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XF7,0XDF,0XF7,0XBF,0XEF,0X9F,
0XCE,0X5F,0X63,0X1B,0X6B,0X5F,0X63,0X3F,0X63,0X3F,0X6B,0X5F,0X5B,0X1B,0XC6,0X7F,
0X7B,0XFF,0X63,0X3D,0X63,0X5F,0X5B,0X1F,0X63,0X3F,0X63,0X1F,0X6B,0X3F,0X6B,0X1C,
0XD6,0X5F,0XFF,0XBF,0XF7,0XDF,0XF7,0XDF,0XFF,0XFF,0XFF,0XDF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFE,0XFF,0X35,0XFE,0X6F,0XFD,0XEA,0XDC,0X41,0XF4,0XC0,0XFC,0XC0,
0XEC,0XA1,0XED,0X27,0XFE,0X8F,0XEE,0X72,0XFF,0XFC,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XBE,0X1E,0X9C,0XFD,0XCE,0X7F,0X6B,0X5B,0X63,0X3E,
0X5A,0XFF,0X63,0X3F,0X63,0X3E,0X7B,0XFF,0X84,0X3F,0X5A,0XFE,0X63,0X3F,0X5A,0XFF,
0X63,0X5F,0X63,0X1F,0X63,0X1F,0X62,0XFF,0X6B,0X1C,0XDE,0XBF,0XAD,0X59,0XC6,0X3C,
0XFF,0XDF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFD,0XEE,0X51,
0XFE,0XAF,0XF5,0X46,0XEC,0X61,0XFC,0XC0,0XFC,0XC0,0XEC,0X80,0XDC,0X83,0XFE,0XAE,
0XE6,0X0F,0XFF,0XFB,0XFF,0XFE,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XF7,0XBF,
0XA5,0X1D,0X84,0X1C,0XCE,0X7F,0X63,0X3A,0X63,0X5E,0X63,0X1F,0X63,0X3F,0X63,0X3F,
0X63,0X1F,0X63,0X1F,0X62,0XFF,0X63,0X3F,0X63,0X1F,0X63,0X1F,0X62,0XFF,0X63,0X1F,
0X63,0X5F,0X5B,0X1C,0XCE,0X9F,0X8C,0X78,0XA5,0X19,0XF7,0XBF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFE,0XFF,0XFB,0XE6,0X0F,0XFE,0XAE,0XE4,0X82,0XF4,0X80,
0XFC,0XC0,0XFC,0XE0,0XF4,0X80,0XDC,0X41,0XFE,0X4B,0XF6,0X4E,0XFE,0XF5,0XFF,0XFE,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFE,0XFF,0XDF,0X9C,0XDE,0X7C,0X1E,0XDF,0X1F,
0X6B,0X9A,0X5B,0X1B,0X5A,0XFB,0X52,0XDA,0X63,0X3B,0X5A,0XDB,0X62,0XFC,0X62,0XFB,
0X62,0XFB,0X63,0X1B,0X63,0X1B,0X62,0XFB,0X5A,0XFB,0X52,0XDC,0X5B,0X1C,0XDF,0X1F,
0X84,0X5A,0XAD,0X5C,0XF7,0X9F,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFE,0XFF,0XFD,
0XF6,0XD5,0XF6,0X70,0XFE,0X4C,0XDC,0X20,0XFC,0XA0,0XFC,0XC0,0XFC,0XA0,0XFC,0XE1,
0XE4,0X60,0XED,0X06,0XFE,0XF0,0XD5,0X8F,0XFF,0XFD,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XBF,0X9C,0XDF,0X7B,0XFE,0XDE,0XFF,0XE7,0X3F,0XE7,0X1F,0XE7,0X5F,
0XEF,0X9F,0XF7,0XBF,0XF7,0X9F,0XF7,0X7F,0XF7,0X9F,0XF7,0X7F,0XEF,0X9F,0XEF,0X7F,
0XF7,0X9F,0XEF,0X5F,0XDE,0XFF,0XDE,0XDF,0XDE,0XDF,0X83,0XFC,0XA5,0X1C,0XF7,0X9F,
0XF7,0XDF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFE,0XFF,0XFD,0XCD,0X8F,0XFE,0XB0,0XE5,0X27,
0XE4,0X60,0XFC,0XC0,0XFC,0XC0,0XFC,0XC0,0XFC,0XC0,0XEC,0XA1,0XD4,0X21,0XFE,0X6E,
0XF6,0X30,0XEE,0X96,0XFF,0XFE,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XDF,0XA4,0XFD,
0X63,0X18,0X63,0X57,0X63,0X35,0X63,0X33,0XD6,0XBF,0XFF,0XDF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XDF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XD6,0X9D,0X63,0X34,
0X63,0X17,0X62,0XF8,0X6B,0X38,0XAD,0X3C,0XF7,0X9F,0XF7,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFE,0XEE,0X74,0XEE,0X50,0XFE,0X8E,0XCC,0X01,0XF4,0XA0,0XFC,0XC0,0XFC,0XC0,
0XFC,0XE0,0XFC,0XE0,0XF4,0XC0,0XDC,0X41,0XEC,0XE6,0XFE,0XD0,0XE5,0XAF,0XFF,0XDA,
0XFF,0XFE,0XFF,0XFE,0XFF,0XFF,0XFF,0XFF,0XE7,0X5F,0XCE,0X9F,0XD6,0X9F,0XD6,0X9E,
0XD6,0X9D,0XF7,0X9F,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XF7,0XBF,0XD6,0X9E,0XCE,0X7F,0XD6,0X9F,0XCE,0X5F,
0XE7,0X5F,0XFF,0XDF,0XF7,0XFF,0XFF,0XFF,0XFF,0XFE,0XFF,0X9A,0XDD,0XAF,0XFE,0XB0,
0XE4,0XE5,0XE4,0X82,0XF4,0XA0,0XFC,0XE0,0XFC,0XE0,0XFC,0XE0,0XFC,0XE0,0XFC,0XC0,
0XF4,0XA1,0XCB,0XE0,0XFE,0X4C,0XFE,0X4F,0XE5,0XF0,0XFF,0XFC,0XFF,0XFD,0XFF,0XFF,
0XFF,0XFF,0XF7,0XDF,0XF7,0XDF,0XFF,0XDF,0XFF,0XDF,0XFF,0XDF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XDF,0XF7,0XBF,0XFF,0XDF,0XFF,0XDF,0XF7,0XDF,0XFF,0XFF,
0XFF,0XFE,0XFF,0XFC,0XDE,0X11,0XFE,0X6F,0XFE,0X4C,0XD4,0X21,0XEC,0X80,0XFC,0XC0,
0XFC,0XE0,0XFC,0XC0,0XFC,0XC0,0XFD,0X00,0XFC,0XC0,0XF4,0XA0,0XE4,0X81,0XD4,0X42,
0XFE,0XAF,0XED,0XEF,0XE6,0X32,0XFF,0XFC,0XFF,0XFE,0XFF,0XDF,0XFF,0XFF,0XFF,0XFF,
0XF7,0XDF,0XFF,0XDF,0XFF,0XDF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XDF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFE,0XFF,0XFD,0XE6,0X32,0XED,0XEE,
0XFE,0XAF,0XCC,0X22,0XDC,0X60,0XFC,0XE1,0XFD,0X00,0XFC,0XC0,0XFC,0XE0,0XFC,0XE0,
0XFC,0XC0,0XFC,0XC0,0XFC,0XC0,0XF4,0XC1,0XDC,0X40,0XDC,0XC5,0XFE,0XD0,0XE5,0XCE,
0XE6,0X12,0XFF,0XFD,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFE,0XFF,0XFD,0XDE,0X12,0XE5,0XEF,0XFE,0XF0,0XE4,0XE5,0XD4,0X40,0XEC,0XC1,
0XFC,0XE0,0XFC,0XC0,0XFC,0XC0,0XFC,0XE0,0XFC,0XC0,0XFC,0XC0,0XFC,0XE0,0XFC,0XC0,
0XFD,0X00,0XEC,0XA0,0XD4,0X20,0XED,0X26,0XFE,0XD0,0XE5,0XCF,0XE5,0XD1,0XFF,0XFC,
0XFF,0XFE,0XFF,0XFE,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFE,0XFF,0XFC,0XDD,0XD0,0XED,0XEF,
0XFE,0XAF,0XED,0X27,0XD4,0X20,0XEC,0XA0,0XF4,0XC0,0XFC,0XC0,0XFC,0XE0,0XFC,0XE0,
0XFC,0XE0,0XFC,0XE0,0XFC,0XC0,0XFC,0XE0,0XFC,0XE0,0XFC,0XC0,0XFC,0XE0,0XEC,0X80,
0XD4,0X00,0XF5,0X48,0XFE,0XB0,0XF6,0X0F,0XDD,0X8F,0XFF,0X79,0XFF,0XFD,0XFF,0XFD,
0XFF,0XFE,0XFF,0XFE,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XDF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFE,0XFF,0XFE,
0XFF,0XFE,0XFF,0X58,0XD5,0X6E,0XEE,0X0F,0XFE,0XF0,0XF5,0X47,0XCB,0XE0,0XEC,0XA1,
0XF4,0XC0,0XFC,0XE0,0XFC,0XE0,0XFD,0X00,0XFD,0X00,0XFC,0XE0,0XFC,0XE0,0XFC,0XC0,
0XFD,0X00,0XFC,0XE0,0XFC,0XE0,0XFC,0XC0,0XFC,0XE1,0XEC,0XA1,0XD3,0XE0,0XE4,0XC5,
0XFE,0XF0,0XF6,0X0E,0XDD,0X8E,0XE6,0X12,0XFF,0XFA,0XFF,0XFD,0XFF,0XFE,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFE,0XFF,0XFD,0XFF,0XFD,0XFF,0XBA,0XE6,0X32,0XDD,0X8E,0XFE,0X6F,
0XFE,0XCF,0XE4,0XE5,0XD4,0X00,0XF4,0XA1,0XF4,0XA0,0XFC,0XC0,0XFC,0XC0,0XFC,0XC0,
0XFC,0XC0,0XFC,0XC0,0XFC,0XC0,0XFC,0XE0,0XFD,0X00,0XFC,0XC0,0XFC,0XE0,0XFC,0XE0,
0XFC,0XE0,0XFC,0XC0,0XFC,0XC0,0XF4,0X80,0XDC,0X40,0XCC,0X22,0XFE,0X6D,0XFE,0XB0,
0XEE,0X0E,0XD5,0X6E,0XDE,0X12,0XFF,0X38,0XFF,0XFC,0XFF,0XFD,0XFF,0XFE,0XFF,0XFE,
0XFF,0XFE,0XFF,0XFE,0XFF,0XFE,0XFF,0XFE,0XFF,0XFD,0XFF,0XFD,0XFF,0XFD,0XFF,0X38,
0XE5,0XF1,0XD5,0X6E,0XED,0XEF,0XFE,0XAF,0XFE,0X4C,0XD4,0X43,0XD4,0X00,0XF4,0XA1,
0XFC,0XC0,0XFC,0XE0,0XFC,0XE0,0XFC,0XC0,0XFC,0XC0,0XFC,0XE0,0XFC,0XE0,0XFC,0XE0,
0XFC,0XE0,0XFC,0XE0,0XFC,0XE0,0XFC,0XE0,0XFC,0XE0,0XFC,0XE0,0XFC,0XC0,0XFC,0XC0,
0XFC,0XC0,0XF4,0XC0,0XDC,0X40,0XD4,0X20,0XE5,0X06,0XFE,0X6E,0XFE,0XB0,0XF6,0X0E,
0XDD,0X8D,0XDD,0X8E,0XE5,0XF1,0XF6,0X73,0XFE,0XF6,0XFF,0X37,0XFF,0X37,0XFF,0X36,
0XFE,0XF5,0XFE,0X93,0XED,0XF0,0XDD,0X6E,0XE5,0X6E,0XF6,0X0F,0XFE,0X70,0XFE,0X8E,
0XE5,0X27,0XCC,0X00,0XDC,0X40,0XEC,0XA0,0XF4,0XA0,0XFC,0XE1,0XFC,0XC0,0XFC,0XE0,
0XFC,0XE0,0XFC,0XE0,0XFC,0XE0,0XFC,0XE0,0XFC,0XE0,0XFC,0XE0,0XFC,0XE0,0XFC,0XE0,
0XFC,0XC0,0XFC,0XE0,0XFC,0XE0,0XFC,0XC0,0XFC,0XC0,0XFC,0XE0,0XF4,0XC0,0XF4,0XA0,
0XEC,0X80,0XD4,0X20,0XCC,0X22,0XE5,0X07,0XFE,0X2C,0XFE,0XCF,0XFE,0X8F,0XFE,0X0E,
0XED,0XCD,0XE5,0XAD,0XE5,0XAE,0XE5,0XCD,0XE5,0XAD,0XED,0XAD,0XF5,0XCD,0XFE,0X2E,
0XFE,0X8F,0XFE,0XAF,0XFE,0X4C,0XE5,0X07,0XC4,0X02,0XD4,0X41,0XDC,0X40,0XFC,0XE1,
0XFC,0XE0,0XFC,0XA0,0XFC,0XE0,0XFC,0XC0,0XFC,0XC0,0XFC,0XE0,0XFC,0XE0,0XFC,0XE0,
0XFC,0XE0,0XFC,0XE0,0XFC,0XE0,0XFC,0XE0,0XFC,0XE0,0XFC,0XE0,0XFC,0XC0,0XFC,0XE0,
0XFC,0XE0,0XFC,0XE0,0XFD,0X00,0XFC,0XE0,0XFC,0XC0,0XF4,0XC0,0XF4,0XA1,0XE4,0X40,
0XD4,0X00,0XCB,0XE0,0XD4,0X43,0XE5,0X07,0XFE,0X0B,0XFE,0X6D,0XFE,0X8E,0XFE,0X8E,
0XFE,0XAF,0XFE,0XCF,0XFE,0X8E,0XFE,0X4D,0XFD,0XCB,0XED,0X07,0XD4,0X42,0XCB,0XE0,
0XDC,0X20,0XE4,0X80,0XEC,0X80,0XF4,0XA0,0XFC,0XC0,0XFC,0XA0,0XFC,0XE0,0XFD,0X00,
0XFC,0XC0,0XFC,0XE0,0XFC,0XE0,0XFC,0XE0,0XFC,0XE0,0XFC,0XE0,0XFC,0XE0,0XFC,0XE0,
0XFC,0XE0,0XFC,0XE0,0XFC,0XE0,0XFC,0XE0,0XFC,0XE0,0XFC,0XE0,0XFC,0XE0,0XFC,0XC0,
0XFC,0XE0,0XFC,0XE0,0XFC,0XC0,0XFC,0XC0,0XFC,0XC0,0XF4,0XA0,0XEC,0X81,0XD4,0X20,
0XD4,0X20,0XCB,0XE0,0XC3,0XC0,0XC3,0XE1,0XCC,0X01,0XC3,0XE1,0XC3,0XE1,0XC3,0XE1,
0XC3,0XE1,0XCB,0XE0,0XD4,0X20,0XE4,0X60,0XEC,0X80,0XF4,0XA0,0XFC,0XC0,0XFC,0XC0,
0XFC,0XC0,0XFC,0XC0,0XFC,0XE0,0XFD,0X00,0XFC,0XC0,0XFC,0XC0,0XFC,0XE0,0XFC,0XE0,
0XFC,0XE0,0XFC,0XE0,0XFC,0XE0,0XFC,0XE0,0XFC,0XC0,0XFC,0XC0,0XFC,0XE0,0XFC,0XE0,
0XFC,0XE0,0XFC,0XE0,0XFC,0XE0,0XFC,0XE0,0XFC,0XE0,0XFC,0XE0,0XFC,0XE0,0XFC,0XE0,
0XFC,0XE0,0XFC,0XC0,0XFC,0XC0,0XFC,0XC0,0XF4,0XC0,0XF4,0XC0,0XF4,0XA0,0XEC,0XA0,
0XEC,0X80,0XE4,0X60,0XE4,0X61,0XE4,0X81,0XE4,0X81,0XE4,0X80,0XEC,0XA0,0XEC,0XA0,
0XF4,0XC0,0XFC,0XE0,0XFC,0XE0,0XFC,0XE0,0XFC,0XC0,0XFC,0XE0,0XFC,0XE0,0XFC,0XE0,
0XFC,0XE0,0XFC,0XC0,0XFC,0XE0,0XFC,0XE0,0XFC,0XE0,0XFC,0XE0,0XFC,0XE0,0XFC,0XE0,
0XFC,0XE0,};


 
const unsigned char gImage_PIC_Record[4050] = {  
0XFC,0XC0,0XFC,0XC0,0XFC,0XE0,0XFC,0XE0,0XFC,0XE0,0XFC,0XE0,0XFC,0XE0,0XFC,0XE0,
0XFC,0XE0,0XFC,0XE0,0XFC,0XC0,0XFC,0XE0,0XFC,0XC0,0XFC,0XC0,0XFC,0XE0,0XFC,0XC0,
0XFC,0XC0,0XFC,0XC0,0XF4,0XC0,0XEC,0XA0,0XEC,0XA2,0XEC,0XA2,0XEC,0X81,0XEC,0XA1,
0XEC,0XA1,0XF4,0XA1,0XFC,0XC1,0XFC,0XE0,0XFC,0XC0,0XFC,0XE0,0XFC,0XC0,0XFC,0XC0,
0XFC,0XC0,0XFC,0XC0,0XFC,0XE0,0XFC,0XC0,0XFC,0XC0,0XFC,0XC0,0XFC,0XE0,0XFC,0XE0,
0XFC,0XE0,0XFC,0XE0,0XFC,0XE0,0XFC,0XE0,0XFC,0XE0,0XFC,0XC0,0XFC,0XE0,0XFC,0XE0,
0XFC,0XE0,0XFC,0XE0,0XFC,0XE0,0XFC,0XE0,0XFC,0XE0,0XFC,0XE0,0XFC,0XE0,0XFC,0XE0,
0XFC,0XE0,0XFC,0XC0,0XFC,0XC0,0XFC,0XC0,0XF4,0XA0,0XEC,0XA1,0XEC,0XA1,0XF5,0X03,
0XFD,0X66,0XFD,0X87,0XFD,0XA8,0XFD,0XE9,0XFD,0XE9,0XFD,0XA8,0XF5,0X45,0XEC,0XE3,
0XEC,0XA1,0XEC,0X80,0XFC,0XC1,0XFC,0XE1,0XFC,0XA0,0XFC,0XE0,0XFC,0XC0,0XFC,0XA0,
0XFD,0X01,0XFC,0XC0,0XFC,0XE0,0XFC,0XC0,0XFC,0XC0,0XFC,0XE0,0XFC,0XE0,0XFC,0XE0,
0XFC,0XE0,0XFC,0XE0,0XFC,0XE0,0XFC,0XE0,0XFC,0XE0,0XFC,0XE0,0XFC,0XE0,0XFC,0XE0,
0XFC,0XC0,0XFC,0XC0,0XFC,0XC0,0XFC,0XE0,0XFC,0XC0,0XFC,0XC0,0XFC,0XC0,0XEC,0XA1,
0XE4,0X83,0XED,0X26,0XFD,0XEA,0XFE,0X6D,0XFE,0XAE,0XFE,0X8F,0XFE,0X90,0XFE,0X70,
0XFE,0X4F,0XFE,0X4F,0XFE,0X70,0XFE,0X8F,0XFE,0XAE,0XFE,0XAD,0XFE,0X0A,0XF5,0X06,
0XE4,0X82,0XEC,0XA1,0XFC,0XE0,0XFC,0XC0,0XFC,0XE0,0XFC,0XC0,0XFC,0XE0,0XFC,0XC0,
0XFC,0XE0,0XFC,0XE0,0XFC,0XE0,0XFC,0XE0,0XFC,0XE0,0XFC,0XE0,0XFC,0XE0,0XFC,0XE0,
0XFC,0XE0,0XFC,0XE0,0XFC,0XE0,0XFC,0XE0,0XFC,0XE0,0XFC,0XE0,0XFC,0XE0,0XFD,0X00,
0XFC,0XA0,0XFC,0XC0,0XF4,0XA0,0XE4,0X81,0XFD,0X67,0XFE,0X6D,0XFE,0X8F,0XFE,0X70,
0XF6,0X30,0XF6,0X72,0XFE,0XD5,0XFE,0XF6,0XFF,0X37,0XFF,0X78,0XFF,0X58,0XFE,0XF6,
0XF6,0XD5,0XF6,0X92,0XEE,0X30,0XF6,0X2F,0XFE,0XB0,0XFE,0X6D,0XFD,0X87,0XE4,0X81,
0XF4,0XC1,0XFC,0XE0,0XFD,0X01,0XFC,0XE0,0XFC,0XC0,0XFC,0XC0,0XFC,0XE0,0XFC,0XE0,
0XFC,0XE0,0XFC,0XE0,0XFC,0XE0,0XFC,0XE0,0XFC,0XE0,0XFC,0XE0,0XFC,0XE0,0XFC,0XE0,
0XFC,0XE0,0XFC,0XE0,0XFC,0XE0,0XFC,0XE0,0XFC,0XC0,0XFC,0XE1,0XEC,0XA1,0XEC,0XE4,
0XFE,0X8D,0XFE,0XAF,0XEE,0X30,0XEE,0X73,0XFF,0X58,0XFF,0XFC,0XFF,0XFE,0XFF,0XFE,
0XFF,0XFE,0XFF,0XFE,0XFF,0XFE,0XFF,0XFE,0XFF,0XFE,0XFF,0XFE,0XFF,0XFD,0XFF,0XFC,
0XFF,0X58,0XEE,0X73,0XEE,0X30,0XFE,0X8F,0XFE,0X8C,0XED,0X24,0XE4,0X81,0XF4,0XC1,
0XF4,0XA0,0XFC,0XE0,0XFD,0X00,0XFC,0XC0,0XFC,0XE0,0XFC,0XE0,0XFC,0XE0,0XFC,0XE0,
0XFC,0XE0,0XFC,0XE0,0XFC,0XE0,0XFC,0XC0,0XFC,0XE0,0XFC,0XE0,0XFC,0XE0,0XFC,0XE0,
0XFC,0XC0,0XF4,0XC1,0XE4,0X82,0XFD,0XC9,0XFE,0XCF,0XFE,0X70,0XEE,0X51,0XFF,0X99,
0XFF,0XFD,0XFF,0XFE,0XFF,0XFE,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFE,0XFF,0XFD,0XFF,0XB9,
0XF6,0X93,0XF6,0X2F,0XFE,0XEF,0XFD,0XC9,0XE4,0X82,0XFC,0XE1,0XFC,0XA0,0XFC,0XC0,
0XFC,0XE0,0XFC,0XE0,0XFC,0XE0,0XFC,0XE0,0XFC,0XE0,0XFC,0XE0,0XFC,0XE0,0XFC,0XE0,
0XFC,0XC0,0XFC,0XC0,0XFC,0XE0,0XFC,0XE0,0XFC,0XE0,0XF4,0XC0,0XDC,0X62,0XFE,0X6D,
0XFE,0X6F,0XEE,0X30,0XFF,0X57,0XFF,0XFD,0XFF,0XFE,0XFF,0XFE,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XDF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFE,0XFF,0XFD,0XFF,0XFC,0XFF,0X37,0XEE,0X10,
0XFE,0X90,0XFE,0X2C,0XEC,0X82,0XFC,0XC1,0XFC,0XC0,0XFC,0XE0,0XFC,0XE0,0XFC,0XE0,
0XFC,0XE0,0XFC,0XE0,0XFC,0XE0,0XFC,0XE0,0XFC,0XE0,0XFC,0XC0,0XFC,0XC0,0XFC,0XC0,
0XFC,0XC0,0XF4,0XC0,0XE4,0XA2,0XFE,0X4C,0XFE,0X50,0XEE,0X73,0XFF,0XDB,0XFF,0XFE,
0XFF,0XFE,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFE,0XFF,0XFE,0XFF,0XDB,0XF6,0X52,0XFE,0X90,0XFE,0X2C,
0XE4,0X62,0XFC,0XA1,0XFC,0XC0,0XFC,0XE0,0XFC,0XE0,0XFC,0XE0,0XFC,0XE0,0XFC,0XE0,
0XFC,0XE0,0XFC,0XE0,0XFC,0XE0,0XFC,0XC0,0XFC,0XE0,0XF4,0XC0,0XE4,0X81,0XFE,0X2B,
0XFE,0X70,0XEE,0X53,0XFF,0XFD,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFE,0XFF,0XFD,0XF6,0X53,0XFE,0X70,0XFD,0XEB,0XEC,0X82,0XFC,0XC1,
0XFC,0XE0,0XFC,0XC0,0XFC,0XE0,0XFC,0XE0,0XFC,0XC0,0XFC,0XE0,0XFC,0XC0,0XFC,0XC0,
0XFC,0XE1,0XFC,0XA0,0XE4,0X60,0XFD,0XC9,0XFE,0X8F,0XEE,0X51,0XFF,0XFC,0XFF,0XFF,
0XF7,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XDF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XDF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XDF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFE,
0XFF,0XFC,0XF6,0X52,0XFE,0XB0,0XFD,0X88,0XEC,0X81,0XFC,0XE1,0XFC,0XC0,0XFD,0X00,
0XFD,0X00,0XFC,0XE0,0XFC,0XE0,0XFD,0X00,0XFC,0XC0,0XFC,0XC0,0XF4,0XC1,0XEC,0XE4,
0XFE,0XAF,0XEE,0X30,0XFF,0XFB,0XFF,0XFE,0XFF,0XFF,0XFF,0XFF,0XF7,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XDF,
0XFF,0XFF,0XF7,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XDF,0XFF,0XDF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFE,0XFF,0XFB,0XEE,0X31,
0XFE,0XCF,0XE4,0XE5,0XEC,0X81,0XFC,0XC0,0XFC,0XE0,0XFC,0XE0,0XFC,0XE0,0XFC,0XC0,
0XFC,0XE0,0XFC,0XE0,0XFC,0XA0,0XE4,0X62,0XFE,0X6D,0XF6,0X50,0XFF,0X57,0XFF,0XFE,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XDF,0XFF,0XFF,
0XFF,0XFF,0XF7,0XDF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XF7,0XFF,0XEF,0XFF,0XEF,0XFF,
0XE7,0XDF,0XF7,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XDF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFE,0XFF,0XFE,0XFF,0X78,0XEE,0X30,0XFE,0X6D,0XDC,0X82,
0XFC,0XC1,0XFC,0XC0,0XFC,0XE0,0XFC,0XE0,0XFC,0XC0,0XFC,0XC0,0XFC,0XE0,0XF4,0XA1,
0XF5,0X67,0XFE,0XB0,0XF6,0X94,0XFF,0XFD,0XFF,0XFE,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XDF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XE7,0XDF,0XB6,0XFD,0X9E,0XBD,0XCF,0XFF,0XEF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XF7,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFD,0XF6,0XB4,0XFE,0X90,0XF5,0X67,0XEC,0X81,0XFC,0XE0,0XFC,0XE0,
0XFC,0XE0,0XFC,0XE0,0XFC,0XE0,0XFC,0XC0,0XE4,0X81,0XFE,0XAE,0XEE,0X0F,0XFF,0XDB,
0XFF,0XFE,0XFF,0XFF,0XF7,0XDF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XDF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XEF,0XFF,0XC7,0XDF,
0X86,0X7E,0X4C,0XF8,0X6D,0XBA,0XD7,0XFF,0XEF,0XFF,0XF7,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFE,0XFF,0XBA,
0XEE,0X10,0XFE,0X6D,0XDC,0X62,0XFC,0XE1,0XFC,0XE0,0XFC,0XC0,0XFC,0XC0,0XFC,0XC0,
0XEC,0XA1,0XED,0X05,0XFE,0X6F,0XFE,0XB4,0XFF,0XFE,0XFF,0XFF,0XF7,0XDF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XDF,0XEF,0XFF,0XCF,0XFF,0X4D,0X5A,0X56,0X1F,0X8F,0X5F,0X4D,0X1A,
0X4D,0X1A,0XAF,0X7F,0XE7,0XFF,0XF7,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XDF,0XFF,0XFF,0XFF,0XFD,0XF6,0X94,0XFE,0XB0,0XED,0X06,
0XEC,0XA1,0XFC,0XC0,0XFC,0XE0,0XFC,0XC0,0XF4,0XC1,0XE4,0X81,0XFE,0X0A,0XFE,0X70,
0XFF,0X78,0XFF,0XFE,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XF7,0XDF,0XD7,0XFF,
0X6E,0X1C,0X24,0XFB,0X1C,0XFC,0X45,0X7C,0X8F,0X9F,0X55,0XBD,0X4C,0XB8,0XCF,0X9F,
0XF7,0XFF,0XF7,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFD,0XFF,0X99,0XFE,0X50,0XFE,0X0B,0XE4,0X61,0XFC,0XC0,0XFC,0XE0,
0XFC,0XE0,0XF4,0XC1,0XDC,0X82,0XFE,0X8E,0XEE,0X31,0XFF,0XFD,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XF7,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XE7,0XFF,0X86,0XDF,0X24,0XFB,0X15,0X1E,0X15,0X1D,
0X1D,0X1C,0X24,0XDA,0X87,0X1F,0X96,0X9E,0XDF,0XBF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFD,
0XF6,0X50,0XFE,0X8D,0XE4,0X81,0XFC,0XC0,0XFC,0XC0,0XFC,0XE0,0XEC,0XC1,0XE4,0XC4,
0XFE,0X8F,0XF6,0X94,0XFF,0XFD,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XF7,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XEF,0XFF,
0XBF,0X9F,0X2D,0X1B,0X14,0XFD,0X15,0X1E,0X0C,0XFC,0X46,0X7F,0X4E,0X1E,0X44,0XB7,
0XBF,0X7F,0XF7,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFD,0XF6,0X93,0XFE,0X8E,0XEC,0XC3,
0XFC,0XC0,0XFC,0XC0,0XFC,0XC0,0XEC,0XA1,0XF5,0X46,0XFE,0X8F,0XFF,0X16,0XFF,0XFE,
0XFF,0XFE,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XF7,0XFF,0XCF,0XFF,0X55,0X9B,0X1C,0XFB,0X15,0X5D,
0X1D,0X1C,0X3D,0XDE,0X7F,0XFF,0X45,0X7A,0X8E,0X1B,0XE7,0XDF,0XFF,0XFF,0XFF,0XFF,
0XF7,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFE,0XFF,0X16,0XFE,0X6F,0XFD,0X46,0XF4,0XA1,0XFC,0XC0,0XFC,0XC0,
0XEC,0X80,0XFD,0XA8,0XFE,0X6F,0XFF,0X78,0XFF,0XFE,0XFF,0XFE,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XDF,
0XDF,0XFF,0X6E,0X3E,0X25,0X1B,0X15,0X5D,0X15,0X1C,0X35,0X1A,0X97,0XFF,0X66,0X7E,
0X55,0X59,0XDF,0XFF,0XF7,0XFF,0XF7,0XBF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFE,0XFF,0X78,
0XFE,0X70,0XFD,0XA9,0XE4,0X61,0XFC,0XC0,0XFC,0XE0,0XE4,0X60,0XFD,0XEA,0XFE,0X4F,
0XFF,0XBA,0XFF,0XFE,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XEF,0XDF,0XE7,0XFF,0XAF,0X5F,0X24,0XFB,0X0C,0XFD,
0X15,0X3E,0X1C,0XDB,0X97,0X3F,0XA7,0X5F,0X55,0X17,0XAF,0X7F,0XE7,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XDF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFE,0XFF,0X99,0XF6,0X50,0XFD,0XEB,0XDC,0X41,
0XF4,0XC1,0XFC,0XC0,0XE4,0X60,0XFE,0X0A,0XFE,0X50,0XFF,0XBA,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XDF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XF7,0XFF,
0XE7,0XFF,0XC7,0XFF,0X3D,0X1A,0X15,0X3D,0X0D,0X1E,0X1C,0XFC,0X5E,0X1F,0XAF,0XDF,
0X65,0X18,0X9E,0XBD,0XDF,0XFF,0XF7,0XFF,0XFF,0XFF,0XFF,0XDF,0XFF,0XDF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFE,0XFF,0XBA,0XF6,0X50,0XFE,0X0C,0XDC,0X42,0XF4,0XC1,0XF4,0XE0,0XE4,0X81,
0XFE,0X0B,0XFE,0X50,0XFF,0XBA,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XF7,0XFF,0XF7,0XFF,0XCF,0XDF,0X55,0XDC,0X1D,0X3C,
0X0D,0X3D,0X15,0X1C,0X4D,0X7B,0XAF,0XBF,0X55,0X9C,0X65,0X9B,0XD7,0XDF,0XF7,0XFF,
0XF7,0XDF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XDF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFE,0XFF,0XB9,0XF6,0X50,
0XFE,0X0B,0XDC,0X42,0XF4,0XC1,0XF4,0XE0,0XE4,0X81,0XFD,0XEB,0XFE,0X30,0XFF,0X79,
0XFF,0XFE,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XF7,0XFF,0XEF,0XFF,0XA6,0XDE,0X25,0X1A,0X0D,0X1C,0X0D,0X1C,0X25,0X1B,0XA7,0X9F,
0X9E,0XBF,0X55,0X19,0XBF,0XDF,0XE7,0XFF,0XFF,0XFF,0XF7,0XFF,0XF7,0XFF,0XFF,0XFF,
0XFF,0XFF,0XF7,0XFF,0XF7,0XDF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFE,0XFF,0X98,0XF6,0X50,0XFD,0XEB,0XE4,0X61,0XF4,0XC1,
0XF4,0XE0,0XE4,0X61,0XFD,0X89,0XFE,0X50,0XFF,0X58,0XFF,0XFE,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XF7,0XFF,0XE7,0XFF,0X6D,0X78,
0X2D,0X1A,0X1D,0X5D,0X25,0X1C,0X66,0X9F,0X9F,0X9F,0X5C,0XF9,0XB7,0X1F,0XDF,0XFF,
0XEF,0XFF,0XEF,0XFF,0XE7,0XFF,0XE7,0XFF,0XE7,0XFF,0XE7,0XFF,0XEF,0XFF,0XEF,0XDF,
0XF7,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFE,
0XFF,0X57,0XFE,0X6F,0XFD,0XA8,0XEC,0X82,0XF4,0XA0,0XF4,0XC0,0XEC,0X81,0XF5,0X28,
0XFE,0X70,0XF6,0XB4,0XFF,0XFD,0XFF,0XFE,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XEF,0XFF,0XCF,0XBF,0XB7,0XFF,0X5E,0X1E,0X24,0XDA,0X2C,0XFB,
0X87,0X3F,0X5D,0X7C,0X8E,0X3D,0XE7,0XFF,0XF7,0XFF,0XF7,0XFF,0XDF,0X7F,0XB6,0XDD,
0XB7,0X1E,0XAE,0XDD,0XAE,0XDD,0XAE,0XDD,0XCF,0X5F,0XF7,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFE,0XF6,0XB4,0XFE,0XAF,0XF5,0X46,
0XEC,0X60,0XFC,0XC0,0XFC,0XC0,0XEC,0X81,0XE4,0XC5,0XFE,0X90,0XEE,0X51,0XFF,0XFC,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XDF,0XFF,0XFF,0XE7,0XFF,
0X75,0XFD,0X66,0X3F,0XAF,0XFF,0X86,0XFF,0X35,0X1B,0X3C,0XFA,0X65,0X7A,0XD7,0XFF,
0XEF,0XFF,0XF7,0XFF,0XFF,0XFF,0XC6,0X9C,0X75,0X37,0X65,0X38,0X5D,0X18,0X5D,0X39,
0X65,0X18,0X85,0XB9,0XEF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFE,0XE6,0X31,0XFE,0XAF,0XEC,0XC3,0XF4,0XA0,0XFC,0XE0,0XFC,0XC0,
0XF4,0XA1,0XD4,0X42,0XFE,0XAE,0XEE,0X0F,0XFF,0XFA,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XDF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XD7,0XBF,0X5D,0XBE,0X2C,0XDB,0X5D,0X9B,
0XAF,0XDF,0X9F,0X9F,0X55,0X39,0XBF,0X9F,0XDF,0XFF,0XE7,0XFF,0XEF,0XFF,0XEF,0XFF,
0XE7,0XFF,0XC7,0XDF,0XBF,0XFF,0XB7,0XDF,0XB7,0XFF,0XB7,0XFF,0XCF,0XFF,0XEF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFE,0XFF,0XDB,0XEE,0X10,
0XFE,0XAE,0XDC,0X20,0XFC,0XC0,0XFC,0XE0,0XFC,0XC0,0XFC,0XC1,0XDC,0X41,0XFE,0X0B,
0XFE,0X4F,0XFE,0XF5,0XFF,0XFE,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XDF,0XFF,0X4D,0X7D,0X1C,0XFD,0X2C,0XFB,0X44,0XF9,0XB7,0X7F,0XD7,0XFF,
0XDF,0XFF,0XDF,0XFF,0XD7,0XFF,0XD7,0XFF,0XD7,0XDF,0XD7,0XFF,0XCF,0XFF,0XC7,0XDF,
0XC7,0XFF,0XC7,0XFF,0XC7,0XFF,0XD7,0XFF,0XEF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFE,0XF6,0XD5,0XFE,0X2F,0XFE,0X0B,0XE4,0X60,0XFC,0XA0,
0XFC,0XE0,0XFC,0XE0,0XFC,0XC0,0XEC,0X81,0XEC,0XE5,0XFE,0XD0,0XDE,0X11,0XFF,0XFD,
0XFF,0XFF,0XFF,0XFF,0XFF,0XDF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XDF,0XFF,0X44,0XD9,
0X2C,0XFB,0X4D,0X9D,0X96,0XFF,0XDF,0XFF,0XEF,0XFF,0XCF,0X5F,0X9E,0X3B,0X96,0X5C,
0X96,0X5C,0X96,0X3C,0X96,0X5C,0X8E,0X3C,0X96,0X5C,0X8E,0X3C,0X96,0X5C,0X96,0X7C,
0XAE,0XBC,0XF7,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XDF,0XFF,0XFF,0XFF,0XFE,0XFF,0XFC,
0XE5,0XF0,0XFE,0XB0,0XE4,0XC5,0XF4,0XA0,0XFC,0XE0,0XFC,0XC0,0XFD,0X00,0XFC,0XC0,
0XFC,0XA1,0XDC,0X01,0XFE,0X6E,0XE6,0X0F,0XFF,0X37,0XFF,0XFE,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XEF,0XFF,0X8E,0X1A,0XAF,0X5F,0XDF,0XFF,0XE7,0XFF,
0XF7,0XFF,0XEF,0XFF,0XAE,0X9C,0X75,0X78,0X75,0X99,0X6D,0X99,0X6D,0X99,0X6D,0X99,
0X6D,0X79,0X75,0X9A,0X6D,0X7B,0X6D,0X7A,0X75,0X79,0X8D,0XD9,0XF7,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XDF,0XFF,0XFF,0XFF,0XFE,0XFF,0X37,0XEE,0X0F,0XFE,0X8E,0XCC,0X01,
0XEC,0XA0,0XFC,0XE0,0XFC,0XC0,0XFC,0XE0,0XFD,0X00,0XFC,0XA0,0XE4,0X41,0XED,0X48,
0XFE,0XB0,0XDD,0XCF,0XFF,0XFC,0XFF,0XFE,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFE,
0XF7,0XFF,0XE7,0XFF,0XE7,0XFF,0XF7,0XDF,0XFF,0XFF,0XFF,0XFF,0XEF,0XFF,0XDF,0XFF,
0XD7,0XFF,0XCF,0XFF,0XCF,0XFF,0XC7,0XFF,0XC7,0XFF,0XCF,0XFF,0XC7,0XDF,0XC7,0XDF,
0XC7,0XDF,0XCF,0XFF,0XDF,0XFF,0XF7,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFD,0XDD,0XCF,0XFE,0XB0,0XED,0X27,0XDC,0X61,0XFC,0XE0,0XFC,0XE0,0XFC,0XE0,
0XFD,0X00,0XFC,0XC0,0XFC,0XC0,0XF4,0XA1,0XCC,0X01,0XFE,0X6D,0XFE,0X4F,0XF6,0X93,
0XFF,0XFD,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XF7,0XDF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XF7,0XFF,0XEF,0XFF,0XEF,0XFF,0XEF,0XFF,
0XEF,0XFF,0XEF,0XFF,0XEF,0XFF,0XEF,0XFF,0XEF,0XFF,0XEF,0XFF,0XEF,0XFF,0XF7,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFE,0XFF,0XFD,0XEE,0X73,0XF6,0X2F,0XFE,0X8E,
0XD4,0X21,0XEC,0X81,0XFC,0XE0,0XFC,0XE0,0XFC,0XC0,0XFC,0XC0,0XFD,0X00,0XFC,0XC0,
0XF4,0XC0,0XDC,0X61,0XE4,0XE5,0XFE,0XD0,0XDD,0X8D,0XFF,0X37,0XFF,0XFD,0XFF,0XFE,
0XFF,0XFF,0XF7,0XDF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFE,
0XFF,0XFD,0XFF,0X37,0XE5,0XCF,0XFE,0XCF,0XDC,0X84,0XE4,0X61,0XF4,0XA0,0XFC,0XE0,
0XFC,0XE0,0XFC,0XE0,0XFC,0XE0,0XFC,0XC0,0XFC,0XC0,0XFC,0XC1,0XF4,0XA1,0XD4,0X00,
0XF5,0X89,0XFE,0XD1,0XD5,0X6D,0XFF,0X98,0XFF,0XFE,0XFF,0XDF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFE,0XFF,0XFD,0XFF,0X99,0XD5,0X6E,0XFE,0XB0,
0XF5,0X89,0XD4,0X21,0XEC,0XA1,0XFC,0XC0,0XFC,0XE0,0XFC,0XE0,0XFC,0XE0,0XFC,0XE0,
0XFC,0XE0,0XFC,0XE0,0XF4,0XC0,0XFC,0XC0,0XEC,0X81,0XCC,0X01,0XFE,0X0C,0XFE,0X90,
0XD5,0X6E,0XFF,0X58,0XFF,0XFE,0XFF,0XFE,0XFF,0XFE,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XDF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFE,
0XFF,0XFD,0XFF,0X58,0XD5,0X6E,0XFE,0X90,0XFD,0XCA,0XD4,0X41,0XE4,0X80,0XF4,0XC0,
0XFC,0XE0,0XFC,0XE0,0XFC,0XE0,0XFC,0XE0,0XF4,0XC0,0XFD,0X00,0XFC,0XE0,0XFC,0XE0,
0XFC,0XC0,0XFC,0XC1,0XE4,0X60,0XD4,0X01,0XFE,0X6D,0XFE,0X6F,0XDD,0X8D,0XFE,0XF5,
0XFF,0XFC,0XFF,0XFE,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFD,0XF6,0XD6,0XD5,0X8E,0XFE,0X70,
0XFE,0X4C,0XD4,0X21,0XE4,0X60,0XF4,0XA0,0XFC,0XC0,0XFC,0XE0,0XFC,0XE0,0XFC,0XE0,
0XFC,0XE0,0XFC,0XE0,0XFC,0XE0,0XFC,0XC0,0XFC,0XE0,0XFC,0XE0,0XFC,0XC0,0XFC,0XE1,
0XE4,0X40,0XCC,0X01,0XFE,0X4C,0XFE,0XCF,0XE5,0XAD,0XDD,0XF1,0XFF,0XFB,0XFF,0XFD,
0XFF,0XFD,0XFF,0XFE,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XF7,0XDF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFE,0XFF,0XFE,0XFF,0XFE,
0XFF,0XFC,0XDD,0XF1,0XDD,0XAE,0XFE,0XB0,0XFE,0X2C,0XCB,0XE0,0XE4,0X60,0XF4,0XC0,
0XFC,0XE0,0XFC,0XE0,0XFC,0XE0,0XFC,0XE0,0XFC,0XE0,0XFC,0XE0,0XFC,0XE0,0XFC,0XE0,
0XFC,0XE0,0XFC,0XC0,0XFC,0XE0,0XFD,0X20,0XFC,0XA0,0XFC,0XE0,0XEC,0X80,0XCB,0XC0,
0XFD,0XEA,0XFE,0XD0,0XED,0XEF,0XD5,0X6E,0XEE,0X73,0XFF,0XFB,0XFF,0XFD,0XFF,0XFE,
0XFF,0XFE,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XF7,0XFF,0XF7,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFE,0XFF,0XFE,0XFF,0XFC,0XFF,0XDA,0XF6,0X94,0XD5,0X6E,0XEE,0X0F,0XFE,0XD0,
0XFD,0XA9,0XD4,0X01,0XE4,0X60,0XFC,0XC0,0XFC,0XC0,0XFC,0XC0,0XFC,0XC0,0XFC,0XE0,
0XFC,0XE0,0XFC,0XE0,0XFC,0XE0,0XFC,0XC0,0XFC,0XE0,0XFC,0XE0,0XFC,0XE0,0XFC,0XE0,
0XFC,0XC0,0XFD,0X00,0XFC,0XC0,0XFC,0XC0,0XEC,0X60,0XCB,0XE0,0XDC,0XA5,0XFE,0XAF,
0XFE,0X90,0XE5,0XCE,0XD5,0X6E,0XDD,0XF2,0XF6,0XD6,0XFF,0XDB,0XFF,0XFD,0XFF,0XFE,
0XFF,0XFE,0XFF,0XFE,0XFF,0XFE,0XFF,0XFE,0XFF,0XFD,0XFF,0XDB,0XFE,0XF6,0XE5,0XF0,
0XDD,0X6D,0XED,0XCE,0XFE,0X8F,0XFE,0XAE,0XDC,0XA5,0XCB,0XE0,0XE4,0X60,0XFC,0XC1,
0XFC,0XC0,0XFC,0XC0,0XFC,0XE0,0XFC,0XC0,0XFC,0XC0,0XFC,0XC0,0XFC,0XC0,0XFC,0XE0,
0XFC,0XE0,0XFC,0XE0,0XFC,0XE0,0XFC,0XE0,0XFC,0XE0,0XFC,0XE0,0XFC,0XE0,0XFC,0XE0,
0XFC,0XE0,0XF4,0XA0,0XF4,0XC1,0XDC,0X40,0XC3,0XC0,0XF5,0X47,0XFE,0XCF,0XFE,0X8F,
0XF6,0X0F,0XE5,0XAE,0XDD,0X6D,0XD5,0X6E,0XDD,0XD0,0XE6,0X11,0XE6,0X32,0XE6,0X11,
0XE5,0XF0,0XD5,0X4D,0XDD,0X8D,0XED,0XCE,0XF5,0XED,0XFE,0XB0,0XFE,0X8E,0XED,0X48,
0XC3,0XE0,0XD4,0X20,0XEC,0X80,0XFC,0XC0,0XFC,0XE0,0XFC,0XE0,0XFC,0XC0,0XFC,0XE0,
0XFC,0XC0,0XFC,0XE0,0XFC,0XE0,0XFC,0XE0,0XFC,0XE0,0XFC,0XE0,0XFC,0XE0,0XFC,0XE0,
0XFC,0XE0,0XFC,0XE0,0XFC,0XE0,0XFC,0XE0,0XFC,0XE0,0XFC,0XE0,0XFC,0XE0,0XF4,0XA0,
0XEC,0XA0,0XEC,0X81,0XD3,0XE0,0XCB,0XE1,0XE4,0XE6,0XFE,0X2C,0XFE,0X6E,0XFE,0XAF,
0XFE,0X8F,0XFE,0X4E,0XFE,0X2D,0XFE,0X2E,0XFE,0X2E,0XFE,0X2E,0XFE,0X6E,0XFE,0XAE,
0XFE,0XAE,0XFE,0X0B,0XE4,0XE5,0XCC,0X01,0XD4,0X01,0XE4,0X61,0XF4,0XA0,0XFC,0XC0,
0XFC,0XE0,0XFC,0XE0,0XFC,0XE0,0XFC,0XC0,0XFC,0XE0,0XFC,0XE0,0XFC,0XE0,0XFC,0XE0,
0XFC,0XE0,0XFC,0XE0,0XFC,0XE0,0XFC,0XE0,0XFC,0XE0,0XFC,0XE0,0XFC,0XE0,0XFC,0XE0,
0XFC,0XE0,0XFC,0XE0,0XFC,0XE0,0XFC,0XE0,0XFC,0XC0,0XFC,0XC1,0XF4,0XC1,0XF4,0XA1,
0XEC,0X60,0XDC,0X20,0XCB,0XE0,0XCC,0X01,0XDC,0X83,0XED,0X06,0XFD,0X88,0XFD,0XCA,
0XFD,0XEA,0XFD,0XCA,0XFD,0X89,0XF5,0X47,0XDC,0X83,0XCB,0XE0,0XD4,0X00,0XDC,0X20,
0XE4,0X40,0XF4,0XA1,0XFC,0XC1,0XFC,0XC0,0XFC,0XE0,0XFC,0XC0,0XFC,0XE0,0XFC,0XE0,
0XFC,0XE0,0XFC,0XE0,0XFC,0XE0,0XFC,0XE0,0XFC,0XE0,0XFC,0XE0,0XFC,0XE0,0XFC,0XE0,
0XFC,0XE0,0XFC,0XE0,0XFC,0XE0,0XFC,0XE0,0XFC,0XE0,0XFC,0XE0,0XFC,0XE0,0XFC,0XE0,
0XFC,0XA0,0XFC,0XC0,0XFC,0XC0,0XFC,0XC0,0XFC,0XC0,0XFC,0XC0,0XF4,0XA0,0XEC,0X60,
0XE4,0X60,0XE4,0X61,0XDC,0X20,0XD4,0X00,0XD3,0XE0,0XCB,0XE0,0XCB,0XE0,0XD4,0X00,
0XD4,0X00,0XE4,0X40,0XEC,0X81,0XEC,0X80,0XF4,0XC1,0XFC,0XE1,0XFC,0XC0,0XFC,0XC0,
0XFC,0XC0,0XFC,0XC0,0XFC,0XE0,0XFC,0XE0,0XFC,0XE0,0XFC,0XE0,0XFC,0XE0,0XFC,0XE0,
0XFC,0XE0,0XFC,0XE0,0XFC,0XE0,0XFC,0XE0,0XFC,0XE0,0XFC,0XE0,0XFC,0XE0,0XFC,0XE0,
0XFC,0XE0,0XFC,0XC0,0XFC,0XC0,0XFC,0XE0,0XFC,0XC0,0XFD,0X20,0XFD,0X00,0XFC,0XE0,
0XFC,0XC0,0XFC,0XE0,0XFC,0XE0,0XFC,0XE0,0XFC,0XE0,0XFC,0XC0,0XF4,0XA0,0XF4,0XA0,
0XF4,0XC0,0XF4,0XC1,0XF4,0XC1,0XF4,0XA1,0XF4,0XC1,0XF4,0XA0,0XF4,0XA0,0XF4,0XC1,
0XFC,0XC1,0XFC,0XC0,0XFC,0XC0,0XFC,0XC0,0XFC,0XC0,0XFC,0XC0,0XFC,0XE0,0XFC,0XE0,
0XFC,0XE0,0XFC,0XE0,0XFC,0XE0,0XFC,0XE0,0XFC,0XE0,0XFC,0XE0,0XFC,0XE0,0XFC,0XE0,
0XFC,0XE0,};

 
const unsigned char gImage_PIC_Setting[4050] = {  
0XFC,0XC0,0XFC,0XE0,0XFC,0XE0,0XFC,0XE0,0XFC,0XE0,0XFC,0XE0,0XFC,0XE0,0XFC,0XE0,
0XFC,0XE0,0XFC,0XE0,0XFC,0XE0,0XFC,0XE0,0XFC,0XE0,0XFC,0XC0,0XFD,0X00,0XF4,0XC0,
0XFC,0XE0,0XFC,0XC0,0XFC,0XC0,0XFC,0XC0,0XFC,0XE0,0XF4,0XE0,0XFC,0XC0,0XFC,0XC0,
0XFC,0XC1,0XFC,0XA0,0XFC,0XC0,0XFC,0XE0,0XFC,0XC0,0XFC,0XC0,0XFC,0XE0,0XFC,0XC0,
0XFC,0XC0,0XFC,0XC0,0XFC,0XE0,0XFC,0XE0,0XFC,0XE0,0XFC,0XE0,0XFC,0XE0,0XFC,0XE0,
0XFC,0XE0,0XFC,0XC0,0XFC,0XC0,0XFC,0XC0,0XFC,0XE0,0XFC,0XC0,0XFC,0XE0,0XFC,0XE0,
0XFC,0XE0,0XFC,0XE0,0XFC,0XE0,0XFC,0XE0,0XFC,0XE0,0XFC,0XC0,0XFC,0XE0,0XFC,0XC0,
0XFC,0XC0,0XFC,0XE0,0XFC,0XE0,0XFC,0XE0,0XF4,0XC0,0XF4,0XA0,0XF4,0XC1,0XF4,0XC1,
0XEC,0XA1,0XE4,0XC2,0XEC,0XC2,0XEC,0XC3,0XF4,0XC3,0XF4,0XA2,0XF4,0XA2,0XF4,0X81,
0XF4,0XA0,0XF4,0XC0,0XFC,0XE0,0XFC,0XC0,0XFC,0XC0,0XFC,0XC0,0XFD,0X00,0XFC,0XC0,
0XFC,0XE0,0XFC,0XC0,0XFC,0XE0,0XFD,0X00,0XFC,0XE0,0XFC,0XE0,0XFC,0XE0,0XFC,0XC0,
0XFC,0XE0,0XFC,0XE0,0XFC,0XE0,0XFC,0XE0,0XFC,0XC0,0XFC,0XE0,0XFC,0XE0,0XFC,0XE0,
0XFC,0XE0,0XFC,0XE0,0XFC,0XE0,0XFC,0XA0,0XFC,0XE0,0XFC,0XC0,0XF4,0XA0,0XF4,0XC1,
0XEC,0X81,0XEC,0XA2,0XF5,0X46,0XFD,0XC9,0XFE,0X4C,0XFE,0X6D,0XFE,0XAE,0XFE,0XCF,
0XFE,0XCF,0XFE,0X8E,0XFE,0X8E,0XFE,0X6D,0XFE,0X2B,0XFD,0XA8,0XEC,0XE3,0XE4,0XA1,
0XF4,0XA1,0XFC,0XA0,0XFC,0XC0,0XF4,0XA0,0XFC,0XE0,0XFD,0X00,0XFC,0XE0,0XFC,0XE0,
0XFD,0X00,0XFC,0XC0,0XFC,0XE0,0XFC,0XE0,0XFC,0XE0,0XFC,0XE0,0XFC,0XE0,0XFC,0XE0,
0XFC,0XE0,0XFC,0XC0,0XFC,0XC0,0XFC,0XC0,0XFC,0XE0,0XFC,0XE0,0XFC,0XC0,0XFC,0X80,
0XFC,0XE1,0XFC,0XC1,0XF4,0XC1,0XE4,0X82,0XEC,0XC4,0XFE,0X0A,0XFE,0X6D,0XFE,0X6F,
0XFE,0X70,0XF6,0X50,0XEE,0X30,0XEE,0X51,0XEE,0X93,0XF6,0X93,0XEE,0X72,0XEE,0X31,
0XEE,0X51,0XEE,0X30,0XFE,0X6F,0XFE,0XCF,0XFE,0X6C,0XFD,0X66,0XEC,0X82,0XF4,0XA2,
0XF4,0XC1,0XFC,0XC1,0XFC,0XC0,0XFC,0XE0,0XFC,0XE0,0XFC,0XC0,0XFC,0XE0,0XFC,0XE0,
0XFC,0XE0,0XFC,0XE0,0XFC,0XE0,0XFC,0XE0,0XFC,0XE0,0XFC,0XE0,0XFC,0XC0,0XFC,0XC0,
0XFC,0XC0,0XFC,0XE0,0XFC,0XC0,0XFC,0XC0,0XFC,0XE0,0XFC,0XA0,0XEC,0XA1,0XE4,0X83,
0XFE,0X0A,0XFE,0XAF,0XFE,0X50,0XF6,0X11,0XF6,0XB4,0XFF,0X38,0XFF,0XFB,0XFF,0XFD,
0XFF,0XFE,0XFF,0XFE,0XFF,0XFE,0XFF,0XFE,0XFF,0XFD,0XFF,0XFC,0XFF,0XB9,0XEE,0XF6,
0XE6,0X51,0XEE,0X30,0XFE,0X90,0XFE,0X8E,0XF5,0X47,0XDC,0X62,0XEC,0XA1,0XFC,0XE1,
0XF4,0XC0,0XFC,0XA0,0XFD,0X00,0XFC,0XE0,0XFC,0XE0,0XFC,0XE0,0XFC,0XE0,0XFC,0XE0,
0XFC,0XE0,0XFD,0X00,0XFC,0XE0,0XFC,0XC0,0XFC,0XC0,0XFC,0XC0,0XFC,0XC0,0XFC,0XC0,
0XFC,0XC0,0XFC,0XA0,0XE4,0X61,0XFD,0XA8,0XFE,0XAE,0XFE,0X90,0XEE,0X31,0XFE,0XF6,
0XFF,0XFD,0XFF,0XFD,0XFF,0XFE,0XFF,0XFE,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFE,0XFF,0XFE,0XFF,0XFD,0XFF,0X99,0XF6,0XB4,
0XED,0XF0,0XFE,0X90,0XFE,0X6D,0XE4,0XA3,0XEC,0XA1,0XF4,0XA0,0XFD,0X01,0XFC,0XE0,
0XFC,0XC0,0XFC,0XE0,0XFC,0XE0,0XFC,0XE0,0XFC,0XE0,0XFC,0XE0,0XFD,0X00,0XFC,0XE0,
0XFC,0XE0,0XFC,0XC0,0XFC,0XC0,0XFC,0XC0,0XFC,0XE0,0XF4,0XC0,0XE4,0X82,0XFE,0X0A,
0XFE,0XAF,0XEE,0X50,0XF6,0XF5,0XFF,0XFC,0XFF,0XFD,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XDF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFE,0XFF,0XFE,0XFF,0XFD,0XFF,0XBA,0XEE,0X72,0XFE,0X4F,
0XFE,0XCF,0XE4,0XE5,0XF4,0XC1,0XFC,0XC0,0XFC,0XA0,0XFC,0XE0,0XFC,0XE0,0XFC,0XE0,
0XFC,0XE0,0XFC,0XE0,0XFC,0XE0,0XFC,0XE0,0XFC,0XE0,0XFC,0XE0,0XFC,0XC0,0XFC,0XC0,
0XFC,0XC0,0XF4,0XC0,0XEC,0XA2,0XFE,0X4B,0XFE,0X8F,0XEE,0X51,0XFF,0XD9,0XFF,0XFD,
0XFF,0XFE,0XFF,0XFF,0XFF,0XDF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFE,0XFF,0XFC,0XFE,0XF5,0XF6,0X30,0XFE,0XAF,0XFD,0X66,
0XEC,0X81,0XFC,0XC1,0XFC,0XE0,0XFC,0XE0,0XFC,0XE0,0XFC,0XE0,0XFC,0XE0,0XFC,0XE0,
0XFC,0XC0,0XFC,0XE0,0XFC,0XC0,0XFC,0XE0,0XFC,0XC0,0XFC,0XC1,0XE4,0X82,0XFE,0X4B,
0XFE,0X6F,0XEE,0X72,0XFF,0XFC,0XFF,0XFE,0XFF,0XFE,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XF7,0XDF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XDF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFE,0XFF,0XFD,0XFF,0X77,0XF6,0X30,0XFE,0X8F,0XF5,0X47,0XEC,0XA1,0XFC,0XC0,
0XFC,0XC0,0XFC,0XE0,0XFC,0XE0,0XFC,0XC0,0XFC,0XE0,0XFC,0XE0,0XFC,0XE0,0XFC,0XA0,
0XFC,0XC0,0XFC,0XC0,0XE4,0X61,0XFD,0XEA,0XFE,0X8F,0XEE,0X73,0XFF,0XFD,0XFF,0XFE,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XDF,0XFF,0XFF,0XFF,0XDF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFE,0XFF,0XFC,
0XFF,0X57,0XF6,0X51,0XFE,0XAF,0XEC,0XE4,0XF4,0XA1,0XFC,0XE0,0XFC,0XA0,0XFC,0XE0,
0XFC,0XC0,0XFC,0XE0,0XFC,0XC0,0XFC,0XE0,0XFC,0XE0,0XFC,0XC0,0XEC,0XA1,0XFD,0X67,
0XFE,0XAF,0XEE,0X51,0XFF,0XFC,0XFF,0XFE,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XDF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFE,0XFF,0XFE,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XDF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFD,0XFF,0X57,0XF6,0X50,
0XFE,0X6D,0XE4,0X82,0XF4,0XC1,0XFC,0XE0,0XFC,0XE0,0XFC,0XE0,0XFC,0XC0,0XFC,0XE0,
0XFC,0XA0,0XFC,0XC0,0XF4,0XA1,0XDC,0X62,0XFE,0XAE,0XEE,0X51,0XFF,0XDA,0XFF,0XFE,
0XFF,0XFF,0XFF,0XFF,0XF7,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFE,0XD5,0X94,0XC5,0X11,0XF6,0XD9,0XFF,0XFE,
0XFF,0XFF,0XFF,0XFF,0XFF,0XDF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFD,0XF6,0XB4,0XFE,0XB1,0XFD,0XCA,0XE4,0X61,
0XFC,0XE0,0XFC,0XC0,0XFD,0X00,0XFC,0XE0,0XFC,0XE0,0XFD,0X00,0XFC,0XC1,0XE4,0X82,
0XFE,0X0B,0XF6,0X4F,0XFF,0X36,0XFF,0XFD,0XFF,0XFE,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XDF,0XFF,0XBF,0XFF,0XFF,0XFF,0XFF,0XF7,0XFF,0XFF,0XDE,0XFF,0XFF,
0XFE,0XF9,0XAB,0X29,0XC3,0XAA,0XC4,0X4E,0XFF,0XFE,0XFF,0XFF,0XFF,0XDF,0XFF,0XDF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFC,0XEE,0X31,0XFE,0XCF,0XE4,0XE4,0XEC,0XA1,0XF4,0XC0,0XFC,0XE0,
0XFC,0XE0,0XFC,0XE0,0XF4,0XC0,0XF4,0XA1,0XE4,0X83,0XFE,0XD0,0XEE,0X52,0XFF,0XFD,
0XFF,0XFE,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XBD,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFE,0XFE,0X55,0XCB,0XAA,0XDB,0XAA,
0XCB,0XAA,0XFF,0XFE,0XFF,0XFF,0XFF,0XDF,0XFF,0XDF,0XFF,0XFE,0XFF,0XFE,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XDF,0XFF,0XFF,0XFF,0XFE,0XFF,0X57,
0XFE,0X4F,0XFE,0X0B,0XDC,0X81,0XF4,0XE1,0XFC,0XE0,0XFC,0XC0,0XFC,0XE0,0XFC,0XE1,
0XE4,0X40,0XFE,0X0A,0XFE,0X50,0XFF,0X16,0XFF,0XFD,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XDF,0XFF,0XFE,0XFE,0X76,0XB3,0X8A,0XD4,0XAF,0XFF,0XFD,
0XFF,0XFD,0XFF,0XDC,0XE4,0X8E,0XDB,0XCA,0XE3,0XCA,0XCB,0XA9,0XFD,0XF4,0XFF,0XDC,
0XFF,0XFD,0XFE,0XD8,0XC3,0XAB,0XC4,0X0D,0XFF,0XFE,0XFF,0XFE,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFE,0XFF,0XFC,0XEE,0X51,0XFE,0X8E,0XE4,0XC4,
0XEC,0XA1,0XFC,0XC0,0XFC,0XC0,0XFC,0XE0,0XF4,0XA0,0XEC,0XA3,0XFE,0X6D,0XEE,0X11,
0XFF,0XFC,0XFF,0XFE,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XDD,0XAB,0X6A,0XCB,0XCA,0XD3,0XCA,0XCB,0XCB,0XD4,0X8E,0XBB,0XCB,0XD3,0XAA,
0XE3,0XCA,0XE3,0XE9,0XDB,0XE9,0XC3,0X89,0XE4,0XCE,0XDC,0XEE,0XBB,0X89,0XDB,0XCA,
0XC3,0X69,0XC5,0X32,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFD,0XFF,0X16,0XFE,0X90,0XF5,0X88,0XE4,0X81,0XFC,0XE0,0XFC,0XE0,
0XFC,0XE0,0XF4,0X81,0XED,0X06,0XFE,0X90,0XEE,0X94,0XFF,0XFD,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFE,0XD4,0XCF,0XCB,0XAA,
0XDB,0XAA,0XDB,0XCA,0XD3,0XCA,0XD3,0XEA,0XDB,0XC9,0XE3,0XE9,0XE3,0XE9,0XE3,0XE9,
0XE3,0XE9,0XDB,0XC9,0XD3,0XAA,0XDB,0XCA,0XDB,0XCA,0XBB,0X8A,0XFE,0XFA,0XFF,0XFF,
0XFF,0XDF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFC,
0XE6,0X10,0XFE,0X6D,0XE4,0X61,0XFC,0XC0,0XFC,0XC0,0XFC,0XC0,0XEC,0X81,0XF5,0X89,
0XF6,0X50,0XFF,0X58,0XFF,0XFE,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFE,0XFF,0XFD,0XC3,0XCB,0XDB,0XCA,0XE3,0XEA,0XE3,0XEA,
0XD3,0XC9,0XC3,0X89,0XC3,0XCA,0XD4,0X2B,0XCB,0XA9,0XCB,0X89,0XDB,0XC9,0XE3,0XCA,
0XE3,0XCA,0XD3,0XAA,0XF5,0X92,0XFF,0XFE,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XDF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XF7,0XFF,0XFF,0XFE,0XEE,0X72,0XFE,0X6E,0XEC,0XC3,
0XFC,0XC0,0XFC,0XC0,0XFC,0XE0,0XEC,0XA1,0XFE,0X2B,0XEE,0X30,0XFF,0XFC,0XFF,0XFE,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XDF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFE,0XE5,0X52,0XCB,0XA9,0XDB,0XEA,0XD3,0XAA,0XC3,0XEB,0XFE,0XB8,0XFF,0XFE,
0XFF,0XFE,0XFF,0XBB,0XED,0X92,0XBB,0XAA,0XD3,0XEA,0XD3,0XE9,0XCB,0X89,0XFE,0XF8,
0XFF,0XFE,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XDF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFE,0XFE,0XD5,0XFE,0X8F,0XF5,0X25,0XF4,0XA1,0XFC,0XC0,0XFC,0XE0,
0XEC,0XA1,0XFE,0X6D,0XEE,0X31,0XFF,0XFD,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XDD,0XFF,0XFE,0XB3,0XAB,0XD3,0XCA,
0XD3,0XEA,0XC3,0XCB,0XFF,0XFD,0XFF,0XFE,0XFF,0XBD,0XFE,0XF9,0XFF,0XFE,0XFF,0XFD,
0XFE,0X14,0XBB,0X89,0XD4,0X0A,0XCB,0X89,0XFE,0X35,0XFF,0XDD,0XFF,0XFE,0XFF,0XFE,
0XFF,0XFF,0XFF,0XFF,0XF7,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XDF,0XFF,0XFE,0XFF,0X36,
0XFE,0X6F,0XFD,0X87,0XEC,0XA1,0XFC,0XC0,0XF4,0XC0,0XE4,0XA1,0XFE,0XAE,0XEE,0X52,
0XFF,0XFD,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XDE,0XFF,0XFF,0XFF,0XFF,
0XFE,0XF9,0XFE,0X96,0XD4,0X0C,0XD3,0XCA,0XDB,0XC9,0XCB,0X89,0XFE,0XB8,0XFF,0XFE,
0XF5,0X73,0XAB,0X08,0XBB,0X49,0XBB,0X8A,0XFF,0X7B,0XFF,0XFD,0XDC,0X6E,0XD3,0XCA,
0XDB,0XCA,0XCB,0X89,0XFD,0XB3,0XFE,0X97,0XFF,0XBC,0XFF,0XFE,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFE,0XFF,0X78,0XFE,0X6F,0XFD,0XC9,0XE4,0X61,
0XFC,0XC0,0XF4,0XC1,0XDC,0X82,0XFE,0XCF,0XEE,0X72,0XFF,0XFE,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XCD,0X74,0XA3,0X49,0XC3,0XA9,0XD3,0X89,
0XEB,0XEA,0XE3,0XEA,0XCB,0XCA,0XFF,0XFE,0XFF,0X5C,0XB3,0X28,0XEC,0X0B,0XEB,0XCA,
0XD3,0XAA,0XC4,0X6F,0XFF,0XFD,0XFD,0XF4,0XCB,0XAA,0XDB,0XE9,0XDB,0XE9,0XD3,0XAA,
0XB3,0X4A,0X9B,0X4A,0XFF,0XBC,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFE,0XFF,0XB8,0XF6,0X70,0XFD,0XEA,0XE4,0X61,0XFC,0XC1,0XEC,0XA1,0XE4,0X82,
0XFE,0XAF,0XF6,0X72,0XFF,0XFE,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XDF,0XCD,0X12,0XBB,0X89,0XDB,0XC9,0XE4,0X0A,0XE3,0XE9,0XE3,0XC8,0XD4,0X0B,
0XFF,0XFE,0XFE,0XF9,0XC3,0X8A,0XE3,0XC9,0XEB,0XEA,0XD3,0XAA,0XBC,0X2D,0XFF,0XDD,
0XFE,0X55,0XCB,0XAA,0XDB,0XE9,0XE3,0XE9,0XE3,0XCA,0XD3,0XAB,0XA3,0X6A,0XFF,0X7C,
0XFF,0XDE,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFE,0XFF,0X98,0XF6,0X70,
0XFE,0X0B,0XDC,0X41,0XFC,0XC1,0XF4,0XA1,0XE4,0XA2,0XFE,0X8E,0XF6,0X72,0XFF,0XFE,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFE,0XD9,0XC4,0X4D,
0XCB,0XCA,0XC3,0X68,0XDC,0X0A,0XE3,0XE9,0XC3,0XC9,0XFF,0XBB,0XFF,0XFD,0XB3,0X8A,
0XCB,0XA9,0XD3,0XCA,0XBB,0X49,0XE5,0X93,0XFF,0XFD,0XFD,0X92,0XCB,0XA9,0XDC,0X09,
0XD3,0XE9,0XCB,0X6A,0XCB,0XEC,0XCC,0XF1,0XFF,0XFE,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0X98,0XF6,0X4F,0XFE,0X0A,0XDC,0X41,0XFC,0XC1,
0XF4,0XC0,0XE4,0X81,0XFE,0X8E,0XEE,0X11,0XFF,0XFD,0XFF,0XFF,0XFF,0XDF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFE,0XFF,0XFD,0XFE,0XF8,0XBB,0X69,
0XDB,0XE9,0XD3,0XC9,0XED,0X92,0XFF,0XFD,0XFF,0X7B,0XC4,0XAF,0XB3,0XED,0XED,0XB4,
0XFF,0XFE,0XFF,0XFD,0XCB,0XAA,0XDB,0XEA,0XD3,0XE9,0XBB,0XCA,0XFF,0XFD,0XFF,0XFE,
0XFF,0XFE,0XFF,0XFE,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XDF,0XFF,0XFF,
0XFF,0X36,0XFE,0XB0,0XF5,0X88,0XEC,0X81,0XFC,0XC1,0XFC,0XE0,0XDC,0X40,0XFE,0X6D,
0XEE,0X10,0XFF,0XDC,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFD,0XD4,0X4D,0XDB,0XC9,0XDB,0XC9,0XBB,0X89,
0XFE,0X35,0XFF,0XFC,0XFF,0XFD,0XFF,0XFD,0XFF,0XFD,0XFF,0XFD,0XCC,0X4D,0XD3,0X89,
0XE3,0XE9,0XC3,0XA8,0XFF,0X58,0XFF,0XFE,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XDF,0XFF,0XDF,0XFF,0XFE,0XF6,0XB4,0XFE,0XAF,0XF5,0X46,
0XE4,0X60,0XFC,0XC1,0XFC,0XC0,0XE4,0X80,0XFD,0XEA,0XF6,0X71,0XF7,0X38,0XFF,0XFE,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XDF,0XFF,0XDF,
0XFF,0XFD,0XD4,0X4D,0XDB,0XAA,0XEB,0XEB,0XD3,0XE9,0XBB,0X88,0XDC,0X8E,0XFD,0XF4,
0XFE,0X55,0XFD,0X92,0XCB,0XCA,0XD3,0XCA,0XDC,0X09,0XDC,0X09,0XCB,0XA9,0XFF,0X39,
0XFF,0XFE,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFE,0XEE,0X31,0XFE,0X8E,0XE4,0XC3,0XF4,0XA1,0XFC,0XC0,0XFC,0XC0,
0XEC,0X80,0XF5,0X68,0XFE,0X70,0XE6,0X74,0XFF,0XFD,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XDF,0XFF,0XDF,0XFE,0XF8,0XC3,0X89,0XE3,0XCB,
0XDB,0XAA,0XDB,0XEA,0XD3,0XEA,0XD3,0XAA,0XD3,0XAA,0XCB,0XAA,0XC3,0X89,0XDB,0XE9,
0XDB,0XC9,0XDB,0XE9,0XDB,0XC9,0XD3,0XAA,0XD4,0X2E,0XFF,0XDE,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFE,0XFF,0XFB,0XE6,0X10,
0XFE,0XAE,0XDC,0X41,0XFC,0XE1,0XFC,0XC0,0XFC,0XE0,0XE4,0X80,0XDC,0XA3,0XFE,0XAF,
0XDD,0XF0,0XFF,0XFC,0XFF,0XFE,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFE,0XC3,0XEB,0XD3,0XA9,0XDB,0XEA,0XCB,0XA9,0XC3,0X89,0XCB,0XAA,
0XDB,0XEA,0XDC,0X09,0XDB,0XE9,0XDC,0X2A,0XCB,0XE9,0XCB,0XA9,0XCB,0X69,0XDB,0XCA,
0XE3,0XAA,0XC3,0X6A,0XE5,0XB4,0XFF,0XFE,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFD,0XF6,0XF6,0XF6,0X50,0XFE,0X4C,0XDC,0X40,0XFC,0XC0,
0XFC,0XE0,0XFC,0XE0,0XF4,0XC1,0XD4,0X21,0XFE,0X6C,0XF6,0X2F,0XFE,0XF6,0XFF,0XFE,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XDF,0XFF,0XFE,0XC4,0X0C,
0XCB,0XCA,0XBB,0X89,0XFD,0XD3,0XFF,0XFC,0XF5,0X72,0XC3,0X68,0XE4,0X0A,0XDB,0XC8,
0XD3,0XE9,0XD4,0X8D,0XFE,0XB6,0XFE,0XD8,0XCC,0X0D,0XCB,0X6A,0XB3,0X29,0XEE,0X36,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFE,0XFF,0XFC,
0XE5,0XF0,0XFE,0XD1,0XE5,0X06,0XEC,0X80,0XFC,0XC0,0XFC,0XC0,0XFC,0XC0,0XFC,0XC0,
0XE4,0X61,0XED,0X26,0XFE,0XD0,0XDD,0XB0,0XFF,0XFD,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFD,0XC5,0X11,0XFE,0XF9,0XFF,0XFE,
0XFF,0XFE,0XFF,0XFD,0XF5,0X72,0XCB,0X89,0XE3,0XEB,0XC3,0X69,0XFF,0XFD,0XFF,0XFE,
0XFF,0XFE,0XFF,0XFE,0XDD,0X94,0XEE,0X77,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFE,0XFF,0X37,0XEE,0X0F,0XFE,0X6E,0XCC,0X02,
0XF4,0XA0,0XFD,0X00,0XFC,0XC0,0XFC,0XC0,0XFC,0XE0,0XF4,0X80,0XD4,0X21,0XFE,0X8E,
0XF6,0X30,0XF6,0XB6,0XFF,0XFE,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XF7,0XFF,0XF7,0XFF,
0XF7,0XDF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFE,0XFE,0X97,
0XBB,0X6A,0XD3,0XAB,0XC3,0XCC,0XFF,0XFE,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XF7,0XFF,0XF7,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFE,
0XFF,0XFD,0XE5,0XD0,0XFE,0X90,0XF5,0X48,0XDC,0X41,0XFC,0XC0,0XFC,0XE0,0XFC,0XC0,
0XFC,0XC0,0XFC,0XC0,0XFC,0XC1,0XE4,0X41,0XE4,0XE6,0XFE,0XD1,0XE5,0XAF,0XFF,0XBA,
0XFF,0XFE,0XFF,0XFF,0XFF,0XFF,0XF7,0XFF,0XF7,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFE,0XFF,0X9C,0X9B,0X4A,0XA3,0X2A,0XD4,0XF1,
0XFF,0XFE,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XDF,0XFF,0XDF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFE,0XFF,0XFD,0XEE,0X74,0XFE,0X2F,0XFE,0X8E,
0XD4,0X01,0XEC,0X61,0XFC,0XC0,0XFC,0XE0,0XFC,0XC0,0XFC,0XC0,0XFC,0XC0,0XFC,0XC0,
0XF4,0XA1,0XCB,0XE0,0XFE,0X4C,0XFE,0X6F,0XE5,0XF1,0XFF,0XFC,0XFF,0XFE,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XDF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFE,0XFF,0X9C,0XFF,0X9C,0XFF,0XDE,0XFF,0XFE,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XDF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFD,0XFF,0X57,0XE5,0XAF,0XFE,0XCF,0XE4,0XA4,0XE4,0X61,0XF4,0XA0,0XFC,0XE0,
0XFC,0XE0,0XFC,0XC0,0XFC,0XC0,0XFC,0XE0,0XFC,0XC0,0XF4,0XA0,0XE4,0XA0,0XCC,0X41,
0XFE,0XAF,0XEE,0X0F,0XE6,0X11,0XFF,0XFC,0XFF,0XFF,0XFF,0XDF,0XFF,0XDF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFD,0XFF,0X79,0XD5,0X8E,0XFE,0X90,
0XFD,0XA9,0XD4,0X20,0XEC,0XA1,0XFC,0XC0,0XFC,0XE0,0XFC,0XE0,0XFC,0XE0,0XFC,0XE0,
0XFC,0XC0,0XFC,0XE0,0XFC,0XE0,0XF4,0XE0,0XDC,0X40,0XD4,0XA4,0XFE,0XCF,0XED,0XEE,
0XE5,0XF1,0XFF,0XFD,0XFF,0XDE,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XDF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFE,0XFF,0XFE,
0XFF,0XFD,0XFF,0X58,0XD5,0X6E,0XFE,0X90,0XFE,0X0B,0XD4,0X21,0XE4,0X60,0XF4,0XC0,
0XFC,0XC0,0XFC,0XE0,0XFC,0XE0,0XFC,0XE0,0XFC,0XE0,0XFC,0XE0,0XFC,0XE0,0XFC,0XE0,
0XFD,0X00,0XEC,0XA0,0XD4,0X00,0XED,0X06,0XFE,0XCF,0XED,0XEF,0XDD,0X8F,0XFF,0XFB,
0XFF,0XFE,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XDF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFE,0XFF,0XFD,0XFE,0XF6,0XD5,0X4E,0XFE,0X90,
0XFE,0X4C,0XCC,0X01,0XE4,0X60,0XF4,0XC0,0XFC,0XE0,0XFC,0XE0,0XFC,0XE0,0XFC,0XE0,
0XFC,0XE0,0XFC,0XE0,0XFC,0XE0,0XFD,0X00,0XFC,0XE0,0XFC,0XC0,0XFC,0XE1,0XF4,0XA0,
0XD4,0X00,0XE4,0XE6,0XFE,0XCF,0XFE,0X2E,0XDD,0X6E,0XF6,0XD6,0XFF,0XFD,0XFF,0XFE,
0XFF,0XFE,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XDF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFE,0XFF,0XFE,0XFF,0XFE,0XFF,0XFD,
0XFF,0XDB,0XDD,0XF2,0XDD,0X8E,0XFE,0XD1,0XFE,0X2C,0XCB,0XE1,0XE4,0X60,0XF4,0XA0,
0XFC,0XE0,0XFC,0XE0,0XFC,0XE0,0XFC,0XE0,0XFC,0XE0,0XFC,0XE0,0XFC,0XE0,0XFC,0XC0,
0XFC,0XE0,0XFC,0XC0,0XFC,0XE0,0XFC,0XC0,0XFD,0X01,0XF4,0XA0,0XDC,0X20,0XD4,0X42,
0XFE,0XCF,0XFE,0X90,0XDD,0XAE,0XDD,0XB0,0XFF,0X37,0XFF,0XFC,0XFF,0XFD,0XFF,0XFE,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XDF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFE,0XFF,0XFE,0XFF,0XFD,0XFF,0XDA,0XF6,0X74,0XD5,0X4E,0XEE,0X0F,0XFE,0XF0,
0XFD,0XA9,0XCB,0XE0,0XE4,0X61,0XFC,0XC1,0XFC,0XE0,0XFC,0XA0,0XFC,0XC0,0XFC,0XE0,
0XFC,0XE0,0XFC,0XE0,0XFC,0XE0,0XFC,0XC0,0XFC,0XE0,0XFC,0XC0,0XFC,0XE0,0XFC,0XE0,
0XFC,0XC0,0XFC,0XA0,0XFC,0XE0,0XF4,0XC0,0XDC,0X40,0XC3,0XC0,0XFD,0XAA,0XFE,0XEF,
0XFE,0X4F,0XD5,0X8D,0XD5,0X8F,0XE6,0X75,0XFF,0X59,0XFF,0XFC,0XFF,0XFC,0XFF,0XFD,
0XFF,0XFD,0XFF,0XFD,0XFF,0XFE,0XFF,0XFE,0XFF,0XFD,0XFF,0XDB,0XF6,0XF7,0XDD,0XF1,
0XD5,0X6E,0XED,0XCE,0XFE,0X90,0XFE,0XAE,0XDC,0XA4,0XCB,0XE0,0XE4,0X60,0XFC,0XC1,
0XFC,0XC0,0XFC,0XA0,0XFC,0XE0,0XFC,0XC0,0XFC,0XC0,0XFC,0XC0,0XFC,0XE0,0XFC,0XE0,
0XFC,0XE0,0XFC,0XE0,0XFC,0XC0,0XFC,0XC0,0XFC,0XC0,0XFC,0XC0,0XFC,0XC0,0XFC,0XC0,
0XF4,0XC0,0XF4,0XE0,0XE4,0X80,0XCB,0XE0,0XD4,0X63,0XFE,0X0B,0XFE,0XCF,0XFE,0X4F,
0XE5,0XCE,0XDD,0X8E,0XDD,0X6D,0XE5,0X8E,0XED,0XF0,0XEE,0X51,0XEE,0X52,0XE6,0X11,
0XE5,0XD0,0XD5,0X2D,0XDD,0X6E,0XED,0XCE,0XED,0XEE,0XFE,0XD0,0XFE,0XAE,0XED,0X27,
0XCB,0XE0,0XDC,0X20,0XEC,0X80,0XFC,0XC0,0XFC,0XC0,0XFC,0XE0,0XFC,0XC0,0XFC,0XE0,
0XFC,0XC0,0XFC,0XE0,0XFC,0XE0,0XFC,0XE0,0XFC,0XE0,0XFC,0XE0,0XFC,0XE0,0XFC,0XC0,
0XFC,0XC0,0XFC,0XC0,0XFC,0XC0,0XFC,0XC0,0XFC,0XC0,0XFC,0XE0,0XFC,0XE0,0XF4,0XE0,
0XEC,0XC1,0XDC,0X60,0XC3,0XE0,0XD4,0X42,0XF5,0X88,0XFE,0X8E,0XFE,0XAF,0XFE,0XAF,
0XFE,0X6E,0XFE,0X2E,0XF6,0X0D,0XF6,0X2E,0XF6,0X2E,0XFE,0X0E,0XFE,0X8F,0XFE,0X8F,
0XFE,0X8E,0XFE,0X2B,0XE4,0XE5,0XCB,0XE0,0XD4,0X00,0XEC,0X81,0XF4,0XA0,0XFC,0XC0,
0XFC,0XE0,0XFC,0XE0,0XFC,0XC0,0XFC,0XE0,0XFC,0XE0,0XFC,0XE0,0XFC,0XE0,0XFC,0XE0,
0XFC,0XE0,0XFC,0XE0,0XFC,0XC0,0XFC,0XC0,0XFC,0XE0,0XFC,0XE0,0XFC,0XE0,0XFC,0XE0,
0XFC,0XE0,0XFC,0XE0,0XFC,0XE0,0XFC,0XE0,0XFC,0XC0,0XF4,0XC0,0XF4,0XA0,0XEC,0XA1,
0XE4,0X60,0XD4,0X00,0XCB,0XE0,0XD4,0X22,0XE4,0XA4,0XF5,0X27,0XF5,0XA9,0XF5,0XCA,
0XF5,0XEB,0XF5,0XCA,0XFD,0X89,0XF5,0X07,0XE4,0X83,0XD4,0X00,0XCB,0XE0,0XDC,0X20,
0XEC,0X80,0XF4,0XA0,0XFC,0XC0,0XFC,0XC0,0XFC,0XE0,0XFC,0XC0,0XFC,0XE0,0XFC,0XE0,
0XFC,0XE0,0XFC,0XE0,0XFC,0XE0,0XFC,0XE0,0XFC,0XE0,0XFC,0XE0,0XFC,0XE0,0XFC,0XC0,
0XFC,0XC0,0XFC,0XE0,0XFC,0XE0,0XFC,0XE0,0XFC,0XE0,0XFC,0XE0,0XFC,0XE0,0XFC,0XC0,
0XFC,0XC0,0XFC,0XC0,0XFC,0XE1,0XFC,0XC0,0XFC,0XC0,0XFC,0XC0,0XF4,0XA0,0XEC,0XA1,
0XE4,0X61,0XDC,0X40,0XD4,0X00,0XCB,0XE0,0XCB,0XE1,0XC3,0XE1,0XC3,0XE0,0XCB,0XE0,
0XDC,0X00,0XE4,0X60,0XEC,0X80,0XEC,0XA0,0XFC,0XC0,0XFC,0XE0,0XFC,0XE0,0XFC,0XC0,
0XFC,0XC0,0XFC,0XC0,0XFC,0XE0,0XFC,0XE0,0XFC,0XE0,0XFC,0XE0,0XFC,0XE0,0XFC,0XE0,
0XFC,0XE0,0XFC,0XE0,0XFC,0XE0,0XFC,0XE0,0XFC,0XE0,0XFC,0XE0,0XFC,0XE0,0XFC,0XE0,
0XFC,0XE0,0XFC,0XE0,0XFC,0XE0,0XFC,0XE0,0XFC,0XE0,0XFD,0X00,0XFC,0XC0,0XFC,0XA0,
0XFC,0XC0,0XFC,0XC0,0XFC,0XC0,0XFC,0XE0,0XFC,0XC0,0XFC,0XC0,0XF4,0XC1,0XF4,0XA1,
0XF4,0X81,0XEC,0X81,0XF4,0X81,0XF4,0XA1,0XF4,0XC1,0XF4,0XC0,0XF4,0XA0,0XFC,0XC0,
0XFC,0XE0,0XFC,0XC0,0XFC,0XC0,0XFC,0XC0,0XFC,0XC0,0XFC,0XE0,0XFC,0XE0,0XFC,0XE0,
0XFC,0XE0,0XFC,0XE0,0XFC,0XE0,0XFC,0XE0,0XFC,0XE0,0XFC,0XE0,0XFC,0XE0,0XFC,0XE0,
0XFC,0XE0,};

const unsigned char gImage_PIC_System_Time[4050] = {  
0XFC,0XE0,0XFD,0X01,0XFD,0X22,0XFD,0X22,0XFD,0X22,0XFD,0X02,0XFD,0X22,0XFD,0X02,
0XFD,0X02,0XFD,0X22,0XFD,0X02,0XFD,0X02,0XFD,0X02,0XFD,0X02,0XFD,0X02,0XFD,0X02,
0XFD,0X02,0XFD,0X02,0XFD,0X02,0XFD,0X02,0XFD,0X22,0XFD,0X01,0XFD,0X02,0XFD,0X01,
0XFD,0X02,0XFD,0X01,0XFD,0X01,0XFD,0X01,0XFD,0X01,0XFD,0X01,0XFD,0X01,0XFD,0X01,
0XFD,0X01,0XFD,0X01,0XFD,0X01,0XFD,0X01,0XFD,0X01,0XFD,0X01,0XFD,0X01,0XFD,0X01,
0XFD,0X01,0XFD,0X01,0XFD,0X01,0XFD,0X01,0XFD,0X01,0XFC,0XE0,0XFD,0X01,0XFD,0X23,
0XFD,0X43,0XFD,0X43,0XFD,0X23,0XFD,0X23,0XFD,0X43,0XFD,0X23,0XFD,0X23,0XFD,0X43,
0XFD,0X63,0XFD,0X63,0XFD,0X63,0XFD,0X63,0XFD,0X63,0XFD,0X63,0XFD,0X63,0XFD,0X43,
0XFD,0X43,0XFD,0X63,0XFD,0X63,0XFD,0X43,0XFD,0X63,0XFD,0X43,0XFD,0X43,0XFD,0X43,
0XFD,0X62,0XFD,0X62,0XFD,0X43,0XFD,0X42,0XFD,0X42,0XFD,0X42,0XFD,0X42,0XFD,0X22,
0XFD,0X22,0XFD,0X22,0XFD,0X02,0XFD,0X23,0XFD,0X22,0XFD,0X22,0XFD,0X22,0XFD,0X22,
0XFD,0X22,0XFD,0X02,0XFC,0XE0,0XFD,0X01,0XFD,0X22,0XFD,0X44,0XFD,0X65,0XFD,0X45,
0XFD,0X44,0XFD,0X44,0XFD,0X64,0XFD,0X65,0XF5,0X45,0XED,0X05,0XE4,0XE5,0XDC,0XE5,
0XE4,0XE4,0XDC,0XE4,0XDC,0XC4,0XDC,0XC4,0XDC,0XE4,0XDC,0XE4,0XDC,0XC4,0XDC,0XC4,
0XDC,0XC4,0XDC,0XC4,0XDC,0XC4,0XDC,0XE4,0XDC,0XC4,0XDC,0XC4,0XDC,0XC4,0XDC,0XC4,
0XDC,0XC4,0XE4,0XE5,0XE5,0X04,0XE5,0X04,0XFD,0X44,0XFD,0X63,0XFD,0X63,0XFD,0X24,
0XFD,0X43,0XFD,0X23,0XFD,0X23,0XFD,0X43,0XFD,0X23,0XFD,0X23,0XFD,0X02,0XFC,0XE0,
0XFD,0X01,0XFD,0X22,0XFD,0X43,0XFD,0X65,0XFD,0X66,0XFD,0X65,0XFD,0XA6,0XF5,0X66,
0XD4,0XC5,0XBC,0X64,0XB4,0X45,0XB4,0X45,0XB4,0X44,0XB4,0X44,0XB4,0X24,0XB4,0X24,
0XB4,0X24,0XB4,0X24,0XB4,0X44,0XB4,0X44,0XB4,0X24,0XB4,0X44,0XB4,0X44,0XB4,0X44,
0XB4,0X44,0XB4,0X24,0XB4,0X44,0XB4,0X44,0XB4,0X44,0XB4,0X24,0XB4,0X44,0XB4,0X45,
0XB4,0X45,0XC4,0X65,0XD4,0XC5,0XFD,0X65,0XFD,0X85,0XFD,0X44,0XFD,0X64,0XFD,0X44,
0XFD,0X65,0XFD,0X44,0XFD,0X23,0XFD,0X02,0XFC,0XE0,0XFD,0X01,0XFD,0X22,0XFD,0X43,
0XFD,0X65,0XFD,0X86,0XFD,0XE8,0XED,0X67,0XBC,0X64,0XAC,0X24,0XBC,0X44,0XC4,0X64,
0XBC,0X64,0XC4,0X64,0XC4,0X64,0XC4,0X64,0XBC,0X64,0XC4,0X64,0XC4,0X64,0XBC,0X43,
0XBC,0X22,0XBC,0X02,0XBC,0X02,0XBC,0X02,0XB4,0X23,0XBC,0X44,0XBC,0X64,0XC4,0X64,
0XC4,0X64,0XBC,0X64,0XC4,0X64,0XC4,0X64,0XC4,0X64,0XBC,0X64,0XBC,0X44,0XB4,0X24,
0XBC,0X64,0XF5,0X66,0XFD,0XA6,0XFD,0X66,0XFD,0X86,0XFD,0X65,0XFD,0X44,0XFD,0X22,
0XFD,0X01,0XFC,0XE0,0XFD,0X01,0XFD,0X22,0XFD,0X44,0XFD,0X65,0XFD,0XC7,0XFD,0XA8,
0XBC,0X65,0XB4,0X43,0XC4,0X64,0XC4,0X64,0XC4,0X64,0XC4,0X64,0XBC,0X64,0XBC,0X64,
0XBC,0X64,0XBC,0X44,0XB3,0XE2,0XBC,0X02,0XBC,0X65,0XC4,0XA7,0XCC,0XE9,0XC5,0X09,
0XC4,0XE9,0XC4,0XA7,0XBC,0X44,0XB3,0XE1,0XB4,0X02,0XBC,0X64,0XBC,0X84,0XBC,0X64,
0XC4,0X64,0XC4,0X64,0XBC,0X64,0XBC,0X65,0XC4,0X64,0XB4,0X24,0XC4,0X85,0XFD,0XA7,
0XFD,0XA7,0XFD,0X87,0XFD,0X65,0XFD,0X44,0XFD,0X23,0XFD,0X02,0XFC,0XE0,0XFD,0X01,
0XFD,0X02,0XFD,0X44,0XFD,0X65,0XFD,0XA7,0XCC,0XA5,0XB4,0X24,0XC4,0X64,0XBC,0X44,
0XC4,0X64,0XC4,0X65,0XBC,0X44,0XC4,0X85,0XBC,0X64,0XB3,0XE1,0XBC,0X23,0XCD,0X0A,
0XDD,0XF1,0XE6,0X95,0XE6,0X96,0XE6,0X95,0XE6,0X95,0XE6,0X95,0XE6,0X95,0XE6,0X74,
0XDD,0XF0,0XC4,0XE8,0XB4,0X02,0XB4,0X01,0XC4,0X64,0XBC,0X65,0XBC,0X64,0XC4,0X64,
0XBC,0X64,0XC4,0X64,0XBC,0X64,0XB4,0X23,0XD4,0XE6,0XFD,0XE9,0XFD,0X87,0XFD,0X65,
0XFD,0X44,0XFD,0X23,0XFD,0X01,0XFC,0XE0,0XFC,0XE1,0XFD,0X22,0XFD,0X44,0XFD,0XA5,
0XF5,0X46,0XB4,0X24,0XBC,0X64,0XC4,0X44,0XC4,0X64,0XC4,0X64,0XC4,0X64,0XBC,0X65,
0XBC,0X23,0XB3,0XE1,0XCD,0X09,0XE6,0X94,0XEE,0XF7,0XD6,0X12,0XCD,0X4B,0XBC,0XA8,
0XBC,0X67,0XB4,0X66,0XBC,0X67,0XBC,0XA8,0XCD,0X4D,0XDE,0X32,0XE6,0XF7,0XE6,0X33,
0XC4,0XA8,0XB3,0XC1,0XBC,0X43,0XC4,0X65,0XBC,0X64,0XC4,0X64,0XBC,0X64,0XC4,0X64,
0XBC,0X44,0XBC,0X44,0XF5,0X88,0XFD,0XA7,0XFD,0X65,0XFD,0X44,0XFD,0X22,0XFD,0X01,
0XFC,0XE0,0XFD,0X01,0XFD,0X22,0XFD,0X43,0XFD,0X86,0XD4,0XA5,0XB4,0X24,0XC4,0X64,
0XC4,0X64,0XC4,0X64,0XBC,0X64,0XBC,0X65,0XBC,0X02,0XBC,0X22,0XDD,0XF0,0XF7,0X19,
0XD5,0XD0,0XB4,0X66,0XAB,0XC1,0XA3,0X81,0XA3,0XC1,0XAB,0XC2,0XB3,0XC2,0XAB,0XC2,
0XAB,0XC1,0XAB,0XA1,0XAB,0XC2,0XBC,0X67,0XDE,0X12,0XEF,0X18,0XD5,0X8D,0XB3,0XE1,
0XBC,0X23,0XBC,0X64,0XBC,0X64,0XC4,0X64,0XC4,0X64,0XBC,0X64,0XB4,0X24,0XDD,0X06,
0XFD,0XC7,0XFD,0X65,0XFD,0X44,0XFD,0X22,0XFD,0X01,0XFC,0XE0,0XFD,0X01,0XFD,0X23,
0XFD,0X43,0XFD,0X65,0XCC,0X85,0XB4,0X44,0XC4,0X64,0XC4,0X64,0XC4,0X64,0XC4,0X85,
0XB4,0X02,0XB4,0X02,0XE6,0X74,0XEF,0X18,0XBC,0XC9,0XA3,0X80,0XAB,0XA1,0XB4,0X04,
0XB4,0X24,0XBC,0X24,0XBC,0X24,0XBC,0X44,0XBC,0X44,0XBC,0X45,0XBC,0X24,0XB4,0X24,
0XAB,0XA1,0XA3,0X81,0XC5,0X0B,0XEF,0X39,0XDD,0XF0,0XB3,0XE1,0XBC,0X23,0XC4,0X85,
0XBC,0X64,0XC4,0X64,0XBC,0X64,0XB4,0X24,0XCC,0XA5,0XFD,0XC7,0XFD,0X65,0XFD,0X44,
0XFD,0X22,0XFD,0X01,0XFC,0XE0,0XFD,0X01,0XFD,0X23,0XFD,0X63,0XFD,0X65,0XC4,0X85,
0XBC,0X44,0XC4,0X64,0XBC,0X64,0XC4,0X84,0XBC,0X02,0XB4,0X02,0XE6,0XB5,0XEE,0XD7,
0XB4,0X24,0XA3,0X80,0XB4,0X04,0XB4,0X24,0XBC,0X24,0XBC,0X44,0XBC,0X64,0XBC,0X44,
0XBC,0X64,0XBC,0X64,0XBC,0X64,0XBC,0X64,0XBC,0X44,0XBC,0X44,0XB4,0X04,0XAB,0X80,
0XBC,0X67,0XEE,0XF8,0XD5,0XF1,0XB3,0XC1,0XBC,0X23,0XBC,0X65,0XBC,0X64,0XC4,0X64,
0XB4,0X44,0XC4,0XA5,0XFD,0XA7,0XFD,0X65,0XFD,0X44,0XFD,0X22,0XFD,0X01,0XFD,0X00,
0XFD,0X01,0XFD,0X22,0XFD,0X64,0XFD,0X66,0XC4,0X65,0XBC,0X64,0XC4,0X64,0XC4,0X65,
0XBC,0X43,0XB3,0XE0,0XE6,0X94,0XF7,0X5B,0XAC,0X04,0XA3,0X60,0XB4,0X25,0XBC,0X24,
0XBC,0X64,0XBC,0X64,0XBC,0X64,0XBC,0X44,0XBC,0X65,0XBC,0X64,0XBC,0X64,0XBC,0X64,
0XC4,0X64,0XBC,0X64,0XBC,0X65,0XBC,0X64,0XBC,0X44,0XAB,0X80,0XBC,0X46,0XEF,0X19,
0XCD,0X8E,0XAB,0XA0,0XBC,0X64,0XBC,0X64,0XBC,0X64,0XBC,0X44,0XC4,0X85,0XFD,0X87,
0XFD,0X65,0XFD,0X44,0XFD,0X22,0XFD,0X01,0XFD,0X00,0XFD,0X01,0XFD,0X23,0XFD,0X64,
0XF5,0X45,0XBC,0X65,0XBC,0X64,0XBC,0X64,0XBC,0X64,0XB3,0XC0,0XD5,0XAE,0XFF,0XFF,
0XB4,0X67,0XA3,0X40,0XB4,0X24,0XBC,0X44,0XBC,0X64,0XBC,0X64,0XC4,0X64,0XC4,0X64,
0XC4,0X64,0XC4,0X64,0XC4,0X64,0XC4,0X64,0XC4,0X64,0XC4,0X64,0XC4,0X64,0XBC,0X44,
0XBC,0X64,0XBC,0X44,0XBC,0X44,0XAB,0X80,0XBC,0XC9,0XEF,0X3A,0XC4,0XC9,0XB3,0XE1,
0XBC,0X64,0XBC,0X64,0XBC,0X44,0XC4,0X85,0XFD,0X87,0XFD,0X65,0XFD,0X44,0XFD,0X22,
0XFD,0X01,0XFD,0X00,0XFD,0X01,0XFD,0X23,0XFD,0X64,0XF5,0X45,0XBC,0X65,0XBC,0X44,
0XC4,0X64,0XBC,0X02,0XB4,0X44,0XFF,0XDE,0XCD,0XB0,0X9B,0X20,0XB4,0X04,0XBC,0X44,
0XBC,0X64,0XC4,0X64,0XC4,0X64,0XBC,0X64,0XBC,0X64,0XBC,0X64,0XBC,0X64,0XC4,0X64,
0XC4,0X64,0XC4,0X64,0XC4,0X64,0XC4,0X64,0XC4,0X64,0XBC,0X64,0XBC,0X64,0XBC,0X64,
0XBC,0X24,0XA3,0X80,0XD5,0XF1,0XE6,0XB6,0XAB,0XC2,0XBC,0X23,0XC4,0X64,0XBC,0X44,
0XC4,0X84,0XFD,0X86,0XFD,0X85,0XFD,0X44,0XFD,0X22,0XFD,0X01,0XFD,0X00,0XFD,0X02,
0XFD,0X23,0XFD,0X64,0XF5,0X45,0XBC,0X64,0XBC,0X44,0XC4,0X64,0XAB,0XC0,0XE6,0X32,
0XF7,0X7B,0XA3,0X82,0XAB,0XC2,0XBC,0X24,0XBC,0X64,0XBC,0X64,0XC4,0X64,0XC4,0X64,
0XBC,0X64,0XC4,0X64,0XBC,0X64,0XBC,0X64,0XC4,0X64,0XC4,0X64,0XC4,0X64,0XBC,0X64,
0XBC,0X64,0XC4,0X64,0XC4,0X64,0XBC,0X64,0XBC,0X64,0XBC,0X44,0XB3,0XE2,0XB4,0X25,
0XEE,0XF8,0XCD,0X2B,0XAB,0XC1,0XBC,0X64,0XBC,0X64,0XC4,0X64,0XFD,0X66,0XFD,0X65,
0XFD,0X44,0XFD,0X22,0XFD,0X01,0XFC,0XE0,0XFD,0X02,0XFD,0X23,0XFD,0X64,0XF5,0X25,
0XBC,0X64,0XBC,0X44,0XB4,0X23,0XBC,0X24,0XFF,0X9C,0XCD,0X4D,0X9B,0X20,0XB4,0X24,
0XBC,0X64,0XBC,0X44,0XC4,0X64,0XBC,0X64,0XC4,0X64,0XC4,0X64,0XC4,0X64,0XC4,0X64,
0XC4,0X64,0XBC,0X44,0XBC,0X64,0XC4,0X64,0XC4,0X45,0XBC,0X64,0XC4,0X64,0XBC,0X64,
0XC4,0X64,0XC4,0X64,0XBC,0X64,0XBC,0X44,0XAB,0XA0,0XCD,0X8E,0XE6,0X95,0XB3,0XE2,
0XBC,0X24,0XB4,0X44,0XC4,0X84,0XFD,0X66,0XFD,0X85,0XFD,0X44,0XFD,0X22,0XFD,0X01,
0XFD,0X00,0XFD,0X01,0XFD,0X23,0XFD,0X64,0XF5,0X45,0XBC,0X64,0XBC,0X64,0XB3,0XC0,
0XD5,0X8E,0XF7,0X7C,0XAB,0XE4,0XAB,0XC2,0XBC,0X44,0XBC,0X64,0XBC,0X64,0XBC,0X64,
0XC4,0X64,0XBC,0X64,0XC4,0X64,0XC4,0X64,0XBC,0X64,0XC4,0X64,0XC4,0X65,0XC4,0X64,
0XC4,0X64,0XBC,0X64,0XBC,0X64,0XBC,0X64,0XC4,0X64,0XC4,0X64,0XC4,0X64,0XBC,0X64,
0XBC,0X64,0XB3,0XE2,0XBC,0X66,0XEE,0XD7,0XBC,0XC8,0XB3,0XE1,0XBC,0X64,0XC4,0X65,
0XFD,0X66,0XFD,0X85,0XFD,0X44,0XFD,0X22,0XFD,0X01,0XFC,0XE0,0XFD,0X01,0XFD,0X23,
0XFD,0X64,0XF5,0X25,0XBC,0X64,0XBC,0X44,0XB3,0XE1,0XEE,0XF7,0XDE,0X54,0X9B,0X40,
0XB4,0X24,0XBC,0X44,0XBC,0X64,0XC4,0X64,0XC4,0X64,0XBC,0X23,0XBC,0X64,0XC4,0X64,
0XBC,0X64,0XBC,0X64,0XC4,0X64,0XBC,0X64,0XC4,0X64,0XC4,0X64,0XBC,0X64,0XC4,0X64,
0XC4,0X64,0XBC,0X64,0XC4,0X64,0XBC,0X43,0XBC,0X43,0XBC,0X84,0XBC,0X44,0XAB,0XE2,
0XDE,0X54,0XD5,0XD0,0XAB,0XC1,0XBC,0X44,0XBC,0X84,0XFD,0X66,0XFD,0X85,0XFD,0X44,
0XFD,0X22,0XFD,0X01,0XFC,0XE0,0XFD,0X01,0XFD,0X23,0XFD,0X64,0XF5,0X45,0XBC,0X64,
0XBC,0X43,0XBC,0X44,0XF7,0X39,0XC5,0X0C,0X9B,0X60,0XBC,0X24,0XBC,0X44,0XBC,0X64,
0XBC,0X64,0XBC,0X85,0XC4,0XA6,0XB4,0X01,0XBC,0X65,0XBC,0X84,0XBC,0X64,0XC4,0X64,
0XC4,0X64,0XBC,0X64,0XC4,0X64,0XC4,0X64,0XC4,0X64,0XC4,0X64,0XBC,0X64,0XB4,0X22,
0XC4,0X85,0XC4,0XC8,0XBC,0X44,0XBC,0X64,0XB3,0XC1,0XCD,0X6E,0XDE,0X54,0XB3,0XE3,
0XB4,0X24,0XC4,0X84,0XFD,0X66,0XFD,0X85,0XFD,0X44,0XFD,0X22,0XFD,0X01,0XFD,0X00,
0XFD,0X01,0XFD,0X23,0XFD,0X64,0XF5,0X45,0XBC,0X64,0XB4,0X02,0XC4,0XA8,0XEF,0X3A,
0XB4,0X88,0XAB,0XA1,0XBC,0X44,0XBC,0X64,0XBC,0X44,0XBC,0X43,0XBC,0X64,0XF7,0X39,
0XD5,0X8E,0XAB,0X80,0XC4,0X64,0XC4,0X65,0XC4,0X64,0XC4,0X44,0XBC,0X64,0XC4,0X64,
0XC4,0X64,0XC4,0X65,0XBC,0X22,0XB4,0X01,0XCD,0X0A,0XD5,0XCF,0XC4,0XE9,0XBC,0X43,
0XBC,0X64,0XB4,0X02,0XC4,0XE9,0XDE,0X54,0XB4,0X25,0XB4,0X03,0XC4,0X84,0XFD,0X66,
0XFD,0X85,0XFD,0X43,0XFD,0X22,0XFD,0X01,0XFD,0X00,0XFD,0X01,0XFD,0X23,0XFD,0X64,
0XF5,0X45,0XBC,0X64,0XB3,0XE1,0XCD,0X2B,0XF7,0X3A,0XAC,0X25,0XAB,0XE2,0XBC,0X64,
0XC4,0X64,0XC4,0X64,0XBC,0X64,0XB3,0XE2,0XBC,0XEA,0XF7,0XDD,0XD5,0X8E,0XAB,0X60,
0XBC,0X44,0XC4,0X84,0XBC,0X64,0XC4,0X64,0XC4,0X84,0XBC,0X64,0XB3,0XC0,0XC4,0X64,
0XDE,0X10,0XDE,0X12,0XBC,0X86,0XBC,0X02,0XBC,0X64,0XC4,0X64,0XB4,0X02,0XBC,0XA7,
0XE6,0X74,0XBC,0X87,0XAB,0XE3,0XBC,0X65,0XFD,0X66,0XFD,0X85,0XFD,0X43,0XFD,0X22,
0XFD,0X01,0XFD,0X00,0XFD,0X01,0XFD,0X23,0XFD,0X64,0XF5,0X45,0XBC,0X64,0XAB,0XC0,
0XD5,0X8E,0XEF,0X19,0XAB,0XE4,0XB3,0XE3,0XBC,0X65,0XC4,0X64,0XC4,0X64,0XBC,0X64,
0XBC,0X65,0XA3,0X60,0XB4,0X48,0XFF,0XDD,0XD5,0XAE,0XAB,0X80,0XBC,0X64,0XC4,0X64,
0XC4,0X65,0XBC,0X43,0XB3,0XC0,0XCD,0X2B,0XE6,0XF7,0XD5,0XD0,0XB3,0XE3,0XB3,0XE1,
0XBC,0X64,0XBC,0X64,0XBC,0X84,0XBC,0X23,0XBC,0X66,0XE6,0X74,0XBC,0XC8,0XAB,0XC1,
0XBC,0X65,0XFD,0X66,0XFD,0X85,0XFD,0X43,0XFD,0X22,0XFD,0X01,0XFD,0X00,0XFD,0X02,
0XFD,0X23,0XFD,0X64,0XF5,0X45,0XBC,0X64,0XAB,0XC0,0XD5,0XAF,0XEE,0XF9,0XA3,0XC3,
0XB4,0X03,0XBC,0X44,0XC4,0X64,0XC4,0X64,0XC4,0X64,0XBC,0X64,0XBC,0X64,0XA3,0X40,
0XAC,0X47,0XFF,0XFF,0XD5,0XAF,0XA3,0X60,0XBC,0X64,0XB4,0X01,0XB3,0XC1,0XE6,0X73,
0XF7,0X9C,0XC5,0X0A,0XAB,0X80,0XB4,0X03,0XBC,0X65,0XBC,0X44,0XBC,0X64,0XBC,0X64,
0XBC,0X23,0XBC,0X65,0XE6,0X75,0XBC,0XC9,0XAB,0XC1,0XC4,0X64,0XFD,0X66,0XFD,0X65,
0XFD,0X43,0XFD,0X22,0XFD,0X01,0XFD,0X00,0XFD,0X02,0XFD,0X23,0XFD,0X64,0XF5,0X45,
0XBC,0X65,0XAB,0XA0,0XD5,0XAF,0XEF,0X19,0XAB,0XE3,0XB4,0X03,0XBC,0X64,0XC4,0X64,
0XC4,0X64,0XC4,0X64,0XBC,0X64,0XBC,0X65,0XBC,0X65,0XA3,0X40,0XB4,0X68,0XFF,0XFF,
0XD5,0XD0,0XA3,0X00,0XC4,0XC7,0XF7,0X7B,0XEE,0XF9,0XB4,0X25,0XA3,0X60,0XB4,0X24,
0XBC,0X64,0XBC,0X64,0XC4,0X64,0XC4,0X64,0XC4,0X64,0XBC,0X23,0XBC,0X65,0XE6,0X74,
0XBC,0XC9,0XAB,0XC2,0XC4,0X65,0XFD,0X66,0XFD,0X65,0XFD,0X43,0XFD,0X22,0XFD,0X01,
0XFC,0XE1,0XFD,0X02,0XFD,0X23,0XFD,0X64,0XF5,0X45,0XBC,0X64,0XAB,0XC0,0XCD,0X6D,
0XEF,0X5A,0XAC,0X04,0XB4,0X03,0XBC,0X64,0XC4,0X64,0XC4,0X64,0XC4,0X64,0XBC,0X64,
0XC4,0X64,0XBC,0X64,0XBC,0X65,0XA3,0X20,0XB4,0X48,0XFF,0XFF,0XEE,0XF9,0XF7,0X7C,
0XD5,0XF1,0X9B,0X60,0XA3,0XA1,0XBC,0X45,0XC4,0X64,0XBC,0X64,0XC4,0X64,0XBC,0X64,
0XC4,0X64,0XC4,0X64,0XBC,0X23,0XBC,0X86,0XE6,0X74,0XBC,0XA8,0XAB,0XE2,0XBC,0X65,
0XFD,0X66,0XFD,0X65,0XFD,0X23,0XFD,0X22,0XFD,0X01,0XFC,0XE1,0XFD,0X02,0XFD,0X23,
0XFD,0X64,0XF5,0X45,0XBC,0X64,0XB3,0XE1,0XC4,0XEA,0XF7,0X5A,0XB4,0X47,0XAB,0XE2,
0XBC,0X44,0XC4,0X64,0XC4,0X64,0XC4,0X64,0XC4,0X64,0XBC,0X64,0XC4,0X64,0XBC,0X84,
0XC4,0X65,0XA3,0X20,0XB4,0X89,0XE7,0X1A,0XB4,0X69,0X93,0X00,0XB4,0X03,0XBC,0X65,
0XC4,0X64,0XC4,0X64,0XBC,0X44,0XC4,0X64,0XC4,0X64,0XC4,0X64,0XBC,0X64,0XBC,0X22,
0XC4,0XC8,0XDE,0X53,0XB4,0X45,0XAB,0XE3,0XC4,0X64,0XFD,0X66,0XFD,0X85,0XFD,0X23,
0XFD,0X22,0XFD,0X01,0XFC,0XE1,0XFD,0X02,0XFD,0X23,0XFD,0X64,0XF5,0X45,0XBC,0X64,
0XB4,0X02,0XBC,0X66,0XF7,0X3A,0XBC,0XC9,0XAB,0XA1,0XBC,0X64,0XC4,0X84,0XBC,0X64,
0XC4,0X65,0XC4,0X64,0XC4,0X64,0XBC,0X44,0XBC,0X64,0XC4,0X64,0XBC,0X85,0XAB,0XA1,
0X93,0X00,0X9B,0X61,0XB4,0X25,0XBC,0X44,0XC4,0X64,0XC4,0X84,0XBC,0X64,0XC4,0X64,
0XC4,0X64,0XC4,0X64,0XC4,0X64,0XC4,0X64,0XB4,0X01,0XCD,0X2A,0XDE,0X54,0XB4,0X03,
0XB4,0X03,0XC4,0X65,0XFD,0X66,0XFD,0X65,0XFD,0X23,0XFD,0X02,0XFD,0X01,0XFC,0XE1,
0XFD,0X22,0XFD,0X23,0XFD,0X64,0XF5,0X45,0XBC,0X64,0XBC,0X43,0XB4,0X02,0XF7,0X5A,
0XCD,0XB0,0XA3,0X40,0XBC,0X65,0XC4,0X64,0XC4,0X64,0XBC,0X64,0XC4,0X64,0XBC,0X64,
0XBC,0X64,0XC4,0X65,0XBC,0X64,0XC4,0X84,0XBC,0X44,0XB4,0X24,0XBC,0X44,0XBC,0X64,
0XC4,0X64,0XBC,0X64,0XBC,0X64,0XC4,0X64,0XC4,0X64,0XC4,0X44,0XC4,0X64,0XC4,0X64,
0XC4,0X64,0XB3,0XE1,0XD5,0XCF,0XDE,0X32,0XAB,0XC2,0XB4,0X24,0XBC,0X64,0XFD,0X66,
0XFD,0X65,0XFD,0X23,0XFD,0X02,0XFD,0X01,0XFD,0X01,0XFD,0X22,0XFD,0X23,0XFD,0X64,
0XF5,0X45,0XBC,0X64,0XBC,0X44,0XAB,0X80,0XE6,0X75,0XEF,0X3A,0XAB,0X81,0XB4,0X24,
0XC4,0X65,0XC4,0X64,0XBC,0X64,0XBC,0X64,0XC4,0X64,0XC4,0X64,0XBC,0X64,0XC4,0X64,
0XBC,0X64,0XC4,0X64,0XBC,0X64,0XBC,0X64,0XC4,0X64,0XBC,0X64,0XC4,0X64,0XC4,0X64,
0XC4,0X64,0XBC,0X64,0XC4,0X64,0XBC,0X64,0XC4,0X64,0XBC,0X44,0XBC,0X43,0XE6,0X95,
0XC5,0X2C,0XAB,0XA1,0XB4,0X24,0XC4,0X84,0XFD,0X66,0XFD,0X65,0XFD,0X23,0XFD,0X22,
0XFD,0X01,0XFD,0X01,0XFD,0X22,0XFD,0X23,0XFD,0X64,0XF5,0X45,0XBC,0X64,0XBC,0X64,
0XAB,0XC1,0XBC,0XC9,0XFF,0XFF,0XB4,0X67,0XAB,0X80,0XBC,0X64,0XBC,0X64,0XBC,0X64,
0XC4,0X64,0XBC,0X64,0XC4,0X64,0XC4,0X64,0XBC,0X64,0XBC,0X64,0XC4,0X64,0XC4,0X84,
0XBC,0X64,0XBC,0X64,0XBC,0X64,0XBC,0X44,0XBC,0X64,0XC4,0X64,0XC4,0X44,0XBC,0X64,
0XC4,0X64,0XBC,0X64,0XB3,0XE1,0XC4,0XE9,0XE6,0XD6,0XB4,0X45,0XB3,0XE3,0XBC,0X44,
0XC4,0X64,0XFD,0X66,0XFD,0X65,0XFD,0X23,0XFD,0X22,0XFD,0X01,0XFC,0XE1,0XFD,0X22,
0XFD,0X23,0XFD,0X64,0XF5,0X45,0XBC,0X64,0XBC,0X64,0XBC,0X43,0XAB,0XA1,0XF7,0X5A,
0XE6,0X96,0XA3,0X60,0XBC,0X44,0XBC,0X64,0XC4,0X64,0XBC,0X64,0XC4,0X64,0XBC,0X64,
0XC4,0X64,0XC4,0X64,0XBC,0X64,0XC4,0X64,0XBC,0X64,0XC4,0X64,0XC4,0X64,0XC4,0X64,
0XC4,0X64,0XC4,0X64,0XBC,0X64,0XC4,0X64,0XBC,0X64,0XC4,0X64,0XBC,0X44,0XB4,0X03,
0XE6,0X73,0XD6,0X11,0XA3,0XA0,0XBC,0X24,0XBC,0X44,0XC4,0X64,0XFD,0X66,0XFD,0X65,
0XFD,0X23,0XFD,0X22,0XFD,0X01,0XFC,0XE1,0XFD,0X02,0XFD,0X23,0XFD,0X64,0XF5,0X45,
0XC4,0X64,0XBC,0X44,0XBC,0X65,0XA3,0X60,0XC5,0X4D,0XFF,0XFF,0XB4,0X46,0XAB,0X80,
0XC4,0X85,0XBC,0X64,0XC4,0X64,0XC4,0X64,0XC4,0X64,0XBC,0X64,0XC4,0X64,0XBC,0X64,
0XC4,0X44,0XBC,0X64,0XC4,0X64,0XC4,0X64,0XC4,0X64,0XBC,0X64,0XC4,0X64,0XC4,0X64,
0XBC,0X64,0XC4,0X64,0XC4,0X64,0XB4,0X01,0XCD,0X2A,0XF7,0X5B,0XB4,0X46,0XAB,0XC2,
0XBC,0X44,0XB4,0X64,0XC4,0X64,0XFD,0X66,0XFD,0X65,0XFD,0X23,0XFD,0X22,0XFD,0X01,
0XFD,0X01,0XFD,0X02,0XFD,0X23,0XFD,0X64,0XF5,0X66,0XBC,0X65,0XBC,0X64,0XBC,0X64,
0XBC,0X23,0XA3,0X80,0XEF,0X5A,0XF7,0X3A,0XA3,0X60,0XB4,0X02,0XC4,0X65,0XC4,0X64,
0XC4,0X64,0XBC,0X64,0XBC,0X64,0XBC,0X64,0XBC,0X64,0XC4,0X64,0XBC,0X64,0XBC,0X64,
0XBC,0X64,0XBC,0X64,0XBC,0X64,0XBC,0X64,0XBC,0X64,0XBC,0X64,0XBC,0X64,0XBC,0X64,
0XBC,0X43,0XC4,0XC8,0XC5,0X2B,0XAB,0XE3,0XB4,0X24,0XBC,0X44,0XBC,0X44,0XC4,0X84,
0XFD,0X66,0XFD,0X64,0XFD,0X23,0XFD,0X02,0XFD,0X01,0XFC,0XE1,0XFD,0X02,0XFD,0X23,
0XFD,0X64,0XFD,0X66,0XBC,0X85,0XBC,0X44,0XBC,0X64,0XBC,0X65,0XA3,0XA0,0XB4,0X26,
0XFF,0XFF,0XD6,0X33,0XA3,0X20,0XB4,0X23,0XBC,0X84,0XBC,0X64,0XBC,0X65,0XBC,0X64,
0XBC,0X64,0XBC,0X64,0XBC,0X64,0XC4,0X64,0XBC,0X64,0XC4,0X65,0XBC,0X64,0XBC,0X64,
0XBC,0X64,0XBC,0X64,0XBC,0X65,0XBC,0X43,0XBC,0X43,0XBC,0X65,0XBC,0X23,0XB3,0XC1,
0XBC,0X24,0XBC,0X44,0XBC,0X64,0XBC,0X44,0XC4,0X85,0XFD,0X86,0XFD,0X64,0XFD,0X23,
0XFD,0X22,0XFD,0X01,0XFD,0X01,0XFD,0X02,0XFD,0X23,0XFD,0X64,0XFD,0X66,0XBC,0X65,
0XBC,0X44,0XBC,0X64,0XBC,0X44,0XBC,0X44,0X9B,0X40,0XC4,0XEC,0XFF,0XFF,0XD5,0XD0,
0XA3,0X20,0XB3,0XE2,0XBC,0X65,0XBC,0X44,0XBC,0X44,0XBC,0X64,0XBC,0X44,0XBC,0X64,
0XBC,0X44,0XC4,0X65,0XBC,0X64,0XBC,0X64,0XBC,0X44,0XBC,0X64,0XBC,0X65,0XB3,0XE2,
0XB4,0X02,0XBC,0X65,0XBC,0X44,0XBC,0X24,0XBC,0X24,0XBC,0X24,0XBC,0X44,0XBC,0X44,
0XB4,0X24,0XC4,0X85,0XFD,0X86,0XFD,0X64,0XFD,0X23,0XFD,0X02,0XFC,0XE1,0XFD,0X01,
0XFD,0X22,0XFD,0X43,0XFD,0X65,0XFD,0X86,0XC4,0X65,0XB4,0X24,0XBC,0X44,0XBC,0X44,
0XBC,0X24,0XB4,0X23,0X93,0X00,0XBD,0X4E,0XFF,0XFF,0XD6,0X13,0XA3,0X60,0XAB,0X80,
0XBC,0X64,0XBC,0X44,0XBC,0X44,0XBC,0X44,0XBC,0X44,0XBC,0X44,0XBC,0X44,0XBC,0X64,
0XBC,0X44,0XBC,0X44,0XBC,0X44,0XAB,0XA0,0XAB,0XC1,0XEF,0X18,0XEE,0XB6,0XAB,0XA1,
0XB4,0X24,0XB4,0X44,0XBC,0X44,0XBC,0X44,0XBC,0X44,0XB4,0X24,0XC4,0X85,0XFD,0X86,
0XFD,0X64,0XFD,0X23,0XFD,0X02,0XFD,0X01,0XFD,0X01,0XFD,0X02,0XFD,0X43,0XFD,0X44,
0XFD,0XA6,0XC4,0X85,0XB4,0X04,0XB4,0X44,0XB4,0X24,0XBC,0X45,0XBC,0X24,0XAB,0XE3,
0X8A,0XE0,0XC5,0X4E,0XFF,0XFF,0XEF,0X19,0XAC,0X05,0XA3,0X40,0XAB,0X80,0XB4,0X03,
0XBC,0X24,0XBC,0X24,0XBC,0X24,0XBC,0X24,0XBC,0X24,0XB4,0X03,0XAB,0X80,0XA3,0X60,
0XBC,0X67,0XF7,0X9C,0XFF,0XFF,0XC5,0X2C,0XAB,0XA1,0XB4,0X24,0XBC,0X24,0XBC,0X44,
0XBC,0X24,0XBC,0X44,0XAB,0XE3,0XCC,0XA6,0XFD,0XA7,0XFD,0X64,0XFD,0X23,0XFD,0X02,
0XFD,0X01,0XFD,0X01,0XFD,0X02,0XFD,0X23,0XFD,0X44,0XFD,0XC6,0XDC,0XC6,0XA3,0XE3,
0XB4,0X44,0XB4,0X24,0XBC,0X24,0XB4,0X24,0XB4,0X24,0XAB,0XE3,0X92,0XE0,0XAC,0X69,
0XF7,0X9D,0XFF,0XFF,0XDE,0X54,0XB4,0XA9,0XAB,0XE3,0XAB,0X81,0XA3,0X60,0XA3,0X80,
0XA3,0X80,0XA3,0XA1,0XAC,0X04,0XC4,0XEA,0XEE,0XB7,0XFF,0XFF,0XF7,0X3B,0XAC,0X06,
0X93,0X00,0XAC,0X04,0XB4,0X24,0XB4,0X24,0XB4,0X44,0XB4,0X24,0XBC,0X24,0XAB,0XE3,
0XE5,0X06,0XFD,0XA6,0XFD,0X64,0XFD,0X23,0XFD,0X22,0XFD,0X01,0XFC,0XE1,0XFD,0X02,
0XFD,0X23,0XFD,0X44,0XFD,0XA6,0XF5,0X67,0XAC,0X04,0XAC,0X03,0XB4,0X24,0XB4,0X24,
0XB4,0X24,0XB4,0X24,0XB4,0X24,0XAB,0XE4,0X93,0X00,0X93,0X42,0XC5,0X70,0XFF,0X9D,
0XFF,0XFF,0XFF,0XBC,0XE6,0XD7,0XE6,0X74,0XDE,0X54,0XE6,0X74,0XEE,0XD8,0XFF,0XDD,
0XFF,0XFF,0XF7,0X7C,0XBD,0X0E,0X8B,0X00,0X8A,0XE0,0XA3,0XE4,0XAC,0X04,0XB4,0X24,
0XB4,0X24,0XB4,0X24,0XB4,0X24,0XA3,0XE3,0XBC,0X24,0XFD,0XA7,0XFD,0X86,0XFD,0X44,
0XFD,0X23,0XFD,0X02,0XFD,0X01,0XFD,0X01,0XFD,0X22,0XFD,0X23,0XFD,0X44,0XFD,0X66,
0XFD,0XE8,0XDC,0XE7,0XA3,0XA3,0XAC,0X04,0XB4,0X24,0XB4,0X04,0XB4,0X04,0XB4,0X04,
0XAC,0X04,0XAC,0X04,0X9B,0X82,0X8A,0XC0,0X93,0X42,0XB4,0X8A,0XCD,0XD2,0XE6,0XB7,
0XEF,0X1A,0XEF,0X3A,0XEF,0X1A,0XDE,0X97,0XCD,0XB1,0XAC,0X49,0X8B,0X21,0X82,0XC0,
0X9B,0X62,0XA3,0XE4,0XAB,0XE4,0XAC,0X04,0XB4,0X04,0XB4,0X24,0XB4,0X24,0XAC,0X04,
0XA3,0XA3,0XE5,0X27,0XFD,0XE8,0XFD,0X66,0XFD,0X44,0XFD,0X23,0XFD,0X02,0XFD,0X01,
0XFD,0X01,0XFD,0X02,0XFD,0X23,0XFD,0X44,0XFD,0X66,0XFD,0XA8,0XFE,0X2A,0XC4,0X86,
0X9B,0X82,0XAC,0X04,0XB4,0X04,0XAC,0X24,0XAC,0X04,0XB4,0X04,0XAC,0X04,0XA3,0XE4,
0XA3,0XC4,0X9B,0X82,0X8A,0XE0,0X8A,0XC0,0X8A,0XE0,0X93,0X21,0X93,0X42,0X8B,0X22,
0X8B,0X00,0X82,0XC0,0X8A,0XE0,0X9B,0X62,0XA3,0XA4,0XA3,0XC4,0XAB,0XE4,0XAC,0X04,
0XAC,0X04,0XB4,0X04,0XB4,0X24,0XAB,0XE3,0X9B,0X82,0XD4,0XE7,0XFE,0X2A,0XFD,0X87,
0XFD,0X66,0XFD,0X44,0XFD,0X23,0XFD,0X02,0XFD,0X01,0XFD,0X01,0XFD,0X02,0XFD,0X43,
0XFD,0X64,0XFD,0X86,0XFD,0XA7,0XFE,0X0A,0XFE,0X2B,0XC4,0XA7,0X9B,0XA3,0X9B,0X83,
0XA3,0XA3,0XA3,0XC3,0XA3,0XC3,0XA3,0XE3,0XAB,0XE4,0XA3,0XC4,0XA3,0XC3,0XA3,0XA3,
0X9B,0XA3,0X93,0X83,0X93,0X63,0X93,0X62,0X93,0X62,0X93,0X83,0X9B,0X83,0X9B,0XA3,
0XA3,0XA3,0XA3,0XC3,0XA3,0XE3,0XAB,0XE3,0XA3,0XC3,0XA3,0XC3,0XA3,0XC3,0X9B,0X82,
0XA3,0XA3,0XD4,0XE8,0XFE,0X2C,0XFD,0XC9,0XFD,0X87,0XFD,0X66,0XFD,0X44,0XFD,0X23,
0XFD,0X02,0XFD,0X01,0XFD,0X01,0XFD,0X22,0XFD,0X43,0XFD,0X44,0XFD,0X86,0XFD,0XA8,
0XFD,0XC9,0XFE,0X0A,0XFE,0X4B,0XED,0X48,0XC4,0X86,0XB4,0X45,0XAC,0X24,0XAC,0X25,
0XAC,0X04,0XAB,0XE4,0XAB,0XE4,0XAB,0XE4,0XAB,0XE4,0XA3,0XC4,0XA3,0XC3,0XA3,0XC4,
0XA3,0XC4,0XA3,0XC4,0XA3,0XC4,0XA3,0XC4,0XA3,0XC4,0XA3,0XC4,0XAB,0XE4,0XAB,0XE4,
0XAC,0X04,0XAC,0X05,0XB4,0X25,0XBC,0X45,0XCC,0XC7,0XED,0X8A,0XFE,0X4C,0XFE,0X0B,
0XFD,0XA9,0XFD,0XA7,0XFD,0X66,0XFD,0X44,0XFD,0X23,0XFD,0X02,0XFC,0XE1,0XFD,0X01,
0XFD,0X22,0XFD,0X23,0XFD,0X45,0XFD,0X66,0XFD,0XA8,0XFD,0XA8,0XFD,0XA8,0XFD,0XC8,
0XFE,0X09,0XFE,0X09,0XFD,0XC9,0XFD,0XA8,0XFD,0XA9,0XF5,0X88,0XF5,0X68,0XF5,0X68,
0XF5,0X68,0XF5,0X68,0XF5,0X68,0XF5,0X68,0XF5,0X68,0XF5,0X68,0XF5,0X68,0XF5,0X68,
0XF5,0X88,0XF5,0X88,0XF5,0X68,0XF5,0X68,0XF5,0X68,0XF5,0XA9,0XFD,0XA9,0XFD,0XA9,
0XFD,0XEA,0XFE,0X0A,0XFE,0X0A,0XFD,0XC9,0XFD,0XC9,0XFD,0XC9,0XFD,0X87,0XFD,0X65,
0XFD,0X44,0XFD,0X23,0XFD,0X02,0XFC,0XE1,0XFD,0X01,0XFD,0X22,0XFD,0X43,0XFD,0X64,
0XFD,0X86,0XFD,0X86,0XFD,0X87,0XFD,0X86,0XFD,0X87,0XFD,0X87,0XFD,0X87,0XFD,0XA7,
0XFD,0XA7,0XFD,0XA7,0XFD,0XC7,0XFD,0XC7,0XFD,0XC7,0XFD,0XC8,0XFD,0XC7,0XFD,0XC7,
0XFD,0XC8,0XFD,0XC8,0XFD,0XC8,0XFD,0XC8,0XFD,0XE8,0XFD,0XC8,0XFD,0XC8,0XFD,0XC8,
0XFD,0XC8,0XFD,0XC8,0XFD,0XC8,0XFD,0XC8,0XFD,0XC8,0XFD,0XA7,0XFD,0XA7,0XFD,0XA8,
0XFD,0X87,0XFD,0X88,0XFD,0XA8,0XFD,0XA7,0XFD,0X65,0XFD,0X44,0XFD,0X23,0XFD,0X02,
0XFC,0XE1,};


const unsigned char gImage_PIC_About_Machine[4050] = {  
0XFF,0X33,0XFF,0X33,0XFF,0X32,0XFF,0X12,0XFF,0X12,0XFF,0X11,0XFF,0X11,0XFF,0X11,
0XFF,0X10,0XFF,0X10,0XFE,0XF0,0XFF,0X0F,0XFE,0XEF,0XF6,0XEF,0XFE,0XEF,0XFE,0XEF,
0XFE,0XEE,0XFE,0XEF,0XF6,0XCE,0XFE,0XEE,0XF6,0XEE,0XFE,0XEE,0XFE,0XEE,0XFE,0XEE,
0XF6,0XEE,0XFE,0XCE,0XFE,0XCE,0XFE,0XEF,0XF6,0XEF,0XFE,0XEF,0XFE,0XEF,0XFE,0XEF,
0XFE,0XEF,0XFE,0XF0,0XFE,0XF0,0XFF,0X10,0XFF,0X10,0XFF,0X11,0XFF,0X11,0XFF,0X12,
0XFF,0X12,0XFF,0X32,0XFF,0X33,0XFF,0X33,0XFF,0X33,0XFF,0X33,0XFF,0X32,0XFF,0X12,
0XFF,0X12,0XFF,0X11,0XFF,0X11,0XFF,0X11,0XFF,0X10,0XFE,0XF0,0XFF,0X10,0XFE,0XEF,
0XF6,0XEF,0XFE,0XEF,0XFE,0XEF,0XFE,0XCE,0XFE,0XEE,0XFE,0XEE,0XFE,0XEE,0XFE,0XCE,
0XFE,0XCE,0XFE,0XEE,0XFE,0XEE,0XFE,0XCE,0XFE,0XCE,0XFE,0XCE,0XFE,0XEE,0XF6,0XCE,
0XFE,0XCE,0XFE,0XCE,0XFE,0XEE,0XFE,0XEF,0XFE,0XEF,0XF6,0XEF,0XFE,0XEF,0XFE,0XF0,
0XFE,0XF0,0XFF,0X10,0XFF,0X10,0XFE,0XF1,0XFF,0X11,0XFF,0X12,0XFF,0X12,0XFF,0X12,
0XFF,0X32,0XFF,0X33,0XFF,0X32,0XFF,0X12,0XFF,0X11,0XFF,0X11,0XFF,0X11,0XFF,0X10,
0XFE,0XF0,0XFE,0XF0,0XFE,0XF0,0XFE,0XEF,0XFE,0XEF,0XFE,0XEF,0XFE,0XEE,0XFE,0XEE,
0XFE,0XCE,0XFE,0XCE,0XFE,0XCE,0XFE,0XED,0XFE,0XED,0XFE,0XEC,0XFE,0XAB,0XFE,0XAA,
0XFE,0XAA,0XFE,0XAA,0XFE,0XAB,0XFE,0XEC,0XFF,0X0D,0XFE,0XCE,0XFE,0XCE,0XFE,0XCE,
0XFE,0XEE,0XFE,0XEE,0XFE,0XCF,0XFE,0XEF,0XF6,0XEF,0XFE,0XEF,0XFE,0XEF,0XFE,0XF0,
0XFF,0X10,0XFF,0X10,0XFF,0X11,0XFF,0X11,0XFF,0X12,0XFF,0X32,0XFF,0X33,0XFF,0X12,
0XFF,0X12,0XFF,0X11,0XFF,0X11,0XFF,0X10,0XFE,0XF0,0XFE,0XF0,0XFE,0XEF,0XFE,0XEF,
0XFE,0XEF,0XFE,0XEE,0XF6,0XEE,0XFE,0XCE,0XFE,0XCD,0XFE,0XEE,0XFE,0XEC,0XFE,0XCA,
0XF6,0X89,0XE6,0X2B,0XE6,0X6F,0XE6,0X91,0XE6,0X92,0XE6,0X93,0XE6,0X93,0XE6,0X91,
0XE6,0X6F,0XDE,0X2C,0XEE,0X4A,0XFE,0XAA,0XFE,0XEC,0XFE,0XEE,0XFE,0XCE,0XFE,0XCE,
0XF6,0XCE,0XFE,0XEE,0XFE,0XEF,0XFE,0XEF,0XFE,0XF0,0XFE,0XF0,0XFF,0X10,0XFF,0X11,
0XFF,0X11,0XFF,0X11,0XFF,0X12,0XFF,0X32,0XFF,0X12,0XFF,0X11,0XFF,0X11,0XFF,0X10,
0XFE,0XF0,0XFE,0XF0,0XFE,0XEF,0XFE,0XEF,0XFE,0XEF,0XFE,0XCF,0XFE,0XEE,0XFE,0XCE,
0XFE,0XCE,0XFE,0XEB,0XF6,0X89,0XEE,0X8D,0XE6,0X92,0XE6,0XD6,0XE6,0XFB,0XEF,0X7F,
0XF7,0XBF,0XF7,0XDF,0XF7,0XBF,0XF7,0XDF,0XF7,0XBF,0XEF,0X7F,0XE7,0X1C,0XDE,0XB7,
0XE6,0XB3,0XEE,0X8E,0XF6,0X6A,0XFE,0XCB,0XFE,0XEE,0XFE,0XEE,0XFE,0XCE,0XFE,0XEE,
0XFE,0XEF,0XFE,0XEF,0XFE,0XEF,0XFF,0X0F,0XFF,0X10,0XFF,0X10,0XFF,0X11,0XFF,0X11,
0XFF,0X12,0XFF,0X11,0XFF,0X11,0XFF,0X10,0XFE,0XF0,0XFE,0XF0,0XFE,0XEF,0XFE,0XEF,
0XFE,0XEE,0XFE,0XEE,0XFE,0XEE,0XFE,0XCE,0XFE,0XEC,0XFE,0XA9,0XEE,0X6E,0XE6,0XD6,
0XE7,0X5C,0XEF,0X9F,0XF7,0XFF,0XFF,0XFF,0XF7,0XDF,0XF7,0XBE,0XF7,0X9E,0XF7,0X9E,
0XF7,0X9E,0XF7,0XBF,0XF7,0XBE,0XFF,0XFF,0XF7,0XFF,0XEF,0X9F,0XE7,0X3D,0XE6,0XD7,
0XE6,0X6F,0XF6,0X8A,0XFE,0XEC,0XFE,0XCE,0XF6,0XEE,0XFE,0XEE,0XFE,0XCF,0XFE,0XEF,
0XFE,0XF0,0XFE,0XF0,0XFE,0XF0,0XFF,0X11,0XFF,0X11,0XFF,0X12,0XFF,0X11,0XFF,0X10,
0XFE,0XF0,0XFE,0XEF,0XFE,0XEF,0XFE,0XEF,0XFE,0XEE,0XFE,0XEE,0XFE,0XCE,0XFE,0XEE,
0XFE,0XEB,0XF6,0X8B,0XE6,0X73,0XDF,0X1D,0XEF,0X9F,0XEF,0X7F,0XEF,0X5D,0XEF,0X5D,
0XEF,0X5C,0XEF,0X5C,0XEF,0X5D,0XE7,0X3D,0XE7,0X5D,0XEF,0X5D,0XEF,0X5D,0XEF,0X5D,
0XE7,0X3C,0XEF,0X5D,0XEF,0X5D,0XEF,0X7E,0XEF,0X9F,0XDF,0X3E,0XD6,0X95,0XEE,0X6C,
0XFE,0XEB,0XFE,0XEE,0XFE,0XCE,0XFE,0XEE,0XF6,0XEF,0XFE,0XEF,0XFE,0XEF,0XFF,0X10,
0XFF,0X10,0XFF,0X11,0XFF,0X11,0XFF,0X11,0XFF,0X10,0XFE,0XF0,0XFE,0XEF,0XFE,0XEF,
0XFE,0XEF,0XFE,0XEE,0XF6,0XCE,0XFE,0XEE,0XFE,0XCA,0XE6,0X4D,0XD6,0X99,0XDF,0X1F,
0XE7,0X3D,0XE6,0XFC,0XE7,0X1B,0XDF,0X1B,0XDE,0XFB,0XDE,0XFC,0XDE,0XFC,0XDE,0XFB,
0XE7,0X1C,0XDE,0XFB,0XE7,0X1B,0XDF,0X1B,0XDE,0XFB,0XE6,0XFC,0XDE,0XFB,0XDE,0XFB,
0XDE,0XFB,0XDE,0XFB,0XE7,0X3C,0XE7,0X3F,0XD6,0XBB,0XDE,0X2E,0XFE,0XCA,0XFF,0X0E,
0XF6,0XCE,0XFE,0XEE,0XFE,0XEF,0XFE,0XEF,0XFE,0XEF,0XFE,0XF0,0XFF,0X10,0XFF,0X11,
0XFF,0X10,0XFE,0XF0,0XFE,0XEF,0XFE,0XEF,0XF6,0XEF,0XFE,0XEE,0XF6,0XCE,0XFE,0XEE,
0XFF,0X0B,0XDE,0X0E,0XC6,0X5B,0XDE,0XFE,0XDE,0XDA,0XDE,0XBA,0XD6,0XBB,0XDE,0XDB,
0XDE,0XDB,0XDE,0XBA,0XDE,0XDB,0XDE,0XBA,0XDE,0XBA,0XDE,0XBA,0XDE,0XDB,0XDE,0XBA,
0XD6,0X9A,0XD6,0X9A,0XDE,0XDB,0XDE,0XDB,0XDE,0XBA,0XDE,0XBA,0XDE,0XBA,0XD6,0XBA,
0XDE,0XDA,0XDE,0XFD,0XCE,0X7D,0XC5,0XB0,0XFE,0XCA,0XFF,0X0E,0XF6,0XCE,0XFE,0XEE,
0XFE,0XEF,0XFE,0XEF,0XFE,0XF0,0XFF,0X10,0XFF,0X11,0XFE,0XF0,0XFE,0XF0,0XFE,0XEF,
0XFE,0XCF,0XFE,0XEE,0XFE,0XEE,0XFE,0XEE,0XFE,0XEB,0XD5,0XCD,0XC6,0X3B,0XD6,0XBC,
0XD6,0X79,0XD6,0X7A,0XD6,0X79,0XD6,0X9A,0XD6,0X79,0XD6,0X9A,0XD6,0X9A,0XD6,0X79,
0XD6,0X79,0XD6,0X9A,0XCE,0X79,0XC6,0X38,0XC6,0X38,0XD6,0XBA,0XDE,0XDB,0XD6,0X79,
0XD6,0X79,0XD6,0X9A,0XD6,0X79,0XD6,0X79,0XD6,0X9A,0XCE,0X99,0XCE,0X79,0XD6,0XDB,
0XCE,0X5C,0XBD,0X4F,0XFE,0XAA,0XFF,0X2E,0XF6,0XCE,0XFE,0XEE,0XFE,0XEF,0XFE,0XEF,
0XFF,0X10,0XFF,0X10,0XFE,0XF0,0XFE,0XEF,0XFE,0XEF,0XFE,0XEE,0XFE,0XCE,0XFE,0XCE,
0XFF,0X4C,0XCD,0XCC,0XAD,0X78,0XD6,0X7B,0XCE,0X38,0XC6,0X38,0XCE,0X38,0XCE,0X38,
0XCE,0X58,0XCE,0X38,0XC6,0X38,0XCE,0X38,0XCE,0X59,0XCE,0X38,0XBD,0XD7,0XC6,0X18,
0XD6,0XBA,0XEF,0X7D,0XFF,0XFF,0XF7,0XBE,0XBD,0XD7,0XC6,0X18,0XCE,0X59,0XCE,0X38,
0XCE,0X38,0XCE,0X38,0XCE,0X38,0XC6,0X38,0XC6,0X38,0XD6,0X9A,0XAD,0XBA,0XB5,0X0D,
0XFF,0X2B,0XFE,0XEE,0XFE,0XCE,0XFE,0XEE,0XFE,0XEF,0XFE,0XEF,0XFE,0XF0,0XF6,0XEF,
0XFE,0XEF,0XF6,0XEE,0XFE,0XEE,0XF6,0XAE,0XFF,0X6D,0XE6,0X0A,0X9C,0XF4,0XC6,0X5A,
0XC5,0XF7,0XBD,0XF7,0XC5,0XF8,0XBD,0XF7,0XBD,0XF7,0XBD,0XF7,0XBD,0XF7,0XBD,0XF7,
0XBD,0XF7,0XC5,0XF7,0XBD,0XB6,0XCE,0X59,0XF7,0X9E,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XDF,0XCE,0X59,0XBD,0XB6,0XB5,0X96,0XBD,0XF7,0XC5,0XF7,0XC5,0XF7,0XC5,0XF7,
0XC5,0XF7,0XC5,0XF7,0XBD,0XD7,0XCE,0X79,0X94,0XD5,0XCD,0X4A,0XFF,0X8E,0XFE,0XCE,
0XFE,0XEE,0XFE,0XEF,0XFE,0XEF,0XFE,0XF0,0XFE,0XEF,0XFE,0XEF,0XFE,0XCE,0XFE,0XEE,
0XFE,0XEE,0XFE,0XEB,0X94,0X2D,0XAD,0X79,0XBD,0XD6,0XB5,0XB6,0XBD,0XB6,0XB5,0XB6,
0XBD,0XB6,0XBD,0XB7,0XB5,0XB6,0XAD,0X55,0XB5,0X75,0XB5,0X75,0XAD,0X55,0XCE,0X59,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XEF,0X7D,
0XC6,0X18,0XA5,0X34,0XB5,0X96,0XBD,0XB6,0XB5,0XB6,0XB5,0XB6,0XBD,0XB6,0XBD,0XB7,
0XBD,0XB6,0XBE,0X19,0X73,0X6D,0XDE,0X0A,0XFF,0X8F,0XFE,0XCE,0XFE,0XEE,0XFE,0XEF,
0XFE,0XF0,0XFE,0XEF,0XF6,0XEF,0XFE,0XCE,0XFE,0XCD,0XFF,0XAE,0XAC,0XA9,0X73,0XD2,
0XC5,0XF7,0XAD,0X55,0XAD,0X75,0XAD,0X75,0XB5,0X75,0XB5,0X75,0XAD,0X55,0XA4,0XF3,
0XCE,0X59,0XEF,0X7D,0XD6,0X9A,0XA5,0X14,0XB5,0XB6,0XCE,0X59,0XE7,0X1C,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XDE,0XDB,0X9C,0XF3,
0XA5,0X34,0XAD,0X95,0XB5,0X75,0XAD,0X75,0XAD,0X75,0XAD,0X75,0XBD,0XD7,0X84,0X34,
0X83,0XA9,0XFF,0X8D,0XFE,0XCD,0XFE,0XCE,0XFE,0XEF,0XFE,0XEF,0XFE,0XEF,0XFE,0XEE,
0XFE,0XCE,0XFF,0X0D,0XF6,0XAB,0X73,0X6C,0X9C,0XF5,0XAD,0X34,0XA5,0X34,0XA5,0X14,
0XA5,0X34,0XAD,0X34,0XA5,0X34,0X94,0X72,0XC6,0X38,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XA4,0XF3,0X9C,0XF3,0X9C,0XB2,0X94,0X92,0XB5,0X75,0XFF,0XDF,0XFF,0XFF,0XAD,0X75,
0XDE,0XFB,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XDE,0XFB,0X94,0XB2,0XA4,0XF3,0XA5,0X34,
0XA5,0X14,0XA5,0X34,0XAD,0X14,0XA5,0X34,0XB5,0XB7,0X52,0XAB,0XD5,0X89,0XFF,0X6E,
0XFE,0XCE,0XFE,0XEE,0XFE,0XEF,0XFE,0XEF,0XFE,0XCE,0XF6,0XCD,0XFF,0XAE,0XB4,0XC9,
0X52,0XCD,0XAD,0X75,0X9C,0XF3,0XA4,0XD3,0X9C,0XF3,0XA4,0XF3,0XA4,0XF3,0X8C,0X51,
0XB5,0XB6,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XC6,0X18,0X94,0X92,0X9C,0XF3,0X9C,0XF3,
0X9C,0XF3,0X8C,0X51,0XAD,0X55,0XBD,0XD7,0X8C,0X51,0X8C,0X51,0XB5,0X96,0XFF,0XDF,
0XFF,0XFF,0XFF,0XFF,0XD6,0X9A,0X8C,0X31,0X9C,0XF3,0XA4,0XF3,0X9C,0XF3,0X9C,0XF3,
0XA4,0XD3,0XBD,0XB6,0X52,0X8C,0X83,0X66,0XFF,0XEF,0XF6,0XAE,0XF6,0XEE,0XFE,0XEF,
0XFE,0XEE,0XFE,0XCE,0XFE,0XCD,0XFF,0X8E,0X73,0X27,0X5A,0XEE,0XAD,0X34,0X94,0XB2,
0X94,0XB2,0X94,0XB2,0X9C,0XB2,0X8C,0X71,0X8C,0X71,0XF7,0XBE,0XFF,0XFF,0XFF,0XFF,
0XBD,0XD7,0X7B,0XEF,0X94,0XB2,0X9C,0XB2,0X94,0XB2,0X9C,0XB2,0X9C,0XD3,0X8C,0X71,
0X8C,0X51,0X9C,0XD3,0X94,0XB2,0X83,0XEF,0X9C,0XF3,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XAD,0X34,0X84,0X10,0X9C,0XD3,0X94,0XB2,0X94,0XB2,0X94,0XB2,0XA5,0X14,0X6B,0X8F,
0X29,0X44,0XF6,0XCD,0XFF,0X2E,0XFE,0XCF,0XFE,0XEF,0XFE,0XCF,0XF6,0XCE,0XFF,0X4E,
0XE6,0X2C,0X31,0XA6,0X73,0X8E,0X9C,0XB3,0X94,0X71,0X94,0X71,0X94,0X71,0X94,0X71,
0X7B,0XCF,0XC6,0X18,0XFF,0XFF,0XFF,0XFF,0XCE,0X59,0X73,0X8E,0X8C,0X71,0X94,0X71,
0X94,0X71,0X94,0X71,0X94,0X71,0X94,0X71,0X94,0X71,0X94,0X71,0X8C,0X71,0X94,0X71,
0X94,0X71,0X73,0XAE,0XAD,0X75,0XFF,0XFF,0XFF,0XFF,0XEF,0X5D,0X84,0X10,0X8C,0X51,
0X94,0X71,0X94,0X71,0X94,0X71,0X9C,0XB3,0X84,0X10,0X00,0X22,0XB4,0XC9,0XFF,0XEF,
0XFE,0XCE,0XFE,0XEF,0XFE,0XEE,0XFE,0XEE,0XFF,0XAE,0XBD,0X0B,0X18,0XE5,0X73,0XAE,
0X94,0X92,0X8C,0X30,0X8C,0X30,0X8C,0X30,0X83,0XEF,0X84,0X10,0XFF,0XDF,0XFF,0XFF,
0XFF,0XDF,0X84,0X30,0X7B,0XCF,0X8C,0X30,0X8C,0X30,0X8C,0X30,0X8C,0X30,0X8C,0X30,
0X8C,0X30,0X8C,0X30,0X8C,0X30,0X8C,0X30,0X8C,0X30,0X8C,0X30,0X8C,0X30,0X73,0X6D,
0XE6,0XFB,0XFF,0XFF,0XFF,0XFF,0XA5,0X14,0X73,0X8E,0X8C,0X30,0X8C,0X30,0X8C,0X30,
0X94,0X91,0X83,0XEF,0X00,0X02,0X83,0X67,0XFF,0XEF,0XFE,0XCE,0XFE,0XEF,0XFE,0XEE,
0XFE,0XCD,0XFF,0XAE,0X94,0X2A,0X10,0XC5,0X6B,0X2D,0X8C,0X51,0X83,0XEF,0X83,0XEF,
0X83,0XEF,0X6B,0X4D,0XA5,0X14,0XFF,0XFF,0XFF,0XFF,0XC6,0X38,0X6B,0X2C,0X83,0XEF,
0X83,0XEF,0X84,0X0F,0X83,0XEF,0X83,0XEF,0X83,0XEF,0X83,0XEF,0X83,0XEF,0X83,0XEF,
0X83,0XEF,0X83,0XEF,0X83,0XEF,0X83,0XEF,0X6B,0X4D,0X9C,0XD3,0XFF,0XFF,0XFF,0XFF,
0XC6,0X18,0X6B,0X2C,0X83,0XEF,0X83,0XEF,0X83,0XEF,0X94,0X71,0X6B,0X4D,0X00,0X01,
0X5A,0X86,0XFF,0X8E,0XFE,0XEE,0XFE,0XCE,0XFE,0XCE,0XFE,0XED,0XFF,0X6E,0X7B,0X69,
0X18,0XC5,0X5A,0X8A,0X84,0X10,0X7B,0XAE,0X7B,0XAE,0X7B,0XAE,0X62,0XEB,0XBD,0XD7,
0XFF,0XFF,0XFF,0XFF,0X94,0X92,0X62,0XEC,0X7B,0XAE,0X7B,0XAE,0X7B,0XAE,0X7B,0XAE,
0X7B,0XAE,0X7B,0XAE,0X7B,0XAE,0X7B,0XAE,0X7B,0XAE,0X7B,0XAE,0X7B,0XAE,0X7B,0XAE,
0X7B,0XAE,0X73,0X6D,0X73,0X8E,0XFF,0XDF,0XFF,0XFF,0XEF,0X1C,0X6B,0X2D,0X7B,0X8E,
0X7B,0XAE,0X7B,0XAE,0X94,0X71,0X52,0X8A,0X00,0X01,0X39,0XC5,0XFE,0XEE,0XFF,0X2E,
0XFE,0XEE,0XFE,0XCE,0XFF,0X0D,0XFF,0X2D,0X62,0XE8,0X21,0X26,0X39,0XC7,0X6B,0X2C,
0X7B,0XCF,0X73,0X6D,0X73,0X6D,0X5A,0XCA,0XC6,0X18,0XFF,0XFF,0XFF,0XDF,0X7B,0XCF,
0X6B,0X2C,0X73,0X6D,0X73,0X6D,0X73,0X8D,0X73,0X6D,0X73,0X8D,0X73,0X6D,0X73,0X6D,
0X73,0X6D,0X73,0X6D,0X73,0X6D,0X73,0X8D,0X73,0X6D,0X73,0X8D,0X73,0X6D,0X5A,0XCB,
0XCE,0X79,0XFF,0XFF,0XD6,0X9A,0X62,0XEB,0X73,0X6D,0X73,0X6D,0X7B,0XCF,0X73,0X6E,
0X29,0X24,0X08,0X62,0X29,0X44,0XEE,0X8D,0XFF,0X4E,0XF6,0XEE,0XFE,0XEE,0XFF,0X0E,
0XFE,0XED,0X5A,0X88,0X29,0X67,0X39,0XA7,0X42,0X07,0X6B,0X4D,0X73,0X8D,0X6B,0X2C,
0X52,0X89,0XC6,0X18,0XFF,0XFF,0XF7,0XBE,0X6B,0X4D,0X5A,0XAA,0X6B,0X2C,0X6B,0X4C,
0X6B,0X2C,0X6B,0X4C,0X6B,0X2C,0X6B,0X4C,0X6B,0X2C,0X6B,0X2C,0X6B,0X2C,0X6B,0X2C,
0X6B,0X2C,0X6B,0X2C,0X6B,0X2C,0X6B,0X4C,0X63,0X0C,0X6B,0X4D,0X94,0X92,0X73,0X8D,
0X63,0X0B,0X6B,0X2C,0X73,0X8E,0X7B,0X8E,0X31,0X65,0X18,0XA2,0X10,0XA3,0X29,0X24,
0XEE,0X6D,0XFF,0X4E,0XFE,0XEE,0XFE,0XCE,0XFF,0X0E,0XFE,0XED,0X5A,0X88,0X29,0X46,
0X42,0X07,0X39,0XC6,0X41,0XE8,0X5A,0XCB,0X6B,0X2C,0X4A,0X28,0XB5,0XB6,0XFF,0XFF,
0XFF,0XFF,0X63,0X0C,0X4A,0X28,0X63,0X0C,0X63,0X0B,0X63,0X0B,0X63,0X0B,0X63,0X0B,
0X63,0X0B,0X63,0X0B,0X63,0X0B,0X63,0X0B,0X63,0X0B,0X62,0XEB,0X63,0X0B,0X63,0X0B,
0X63,0X0C,0X6B,0X2C,0X4A,0X28,0X31,0X85,0X5A,0XCB,0X73,0X4D,0X6B,0X4D,0X62,0XEB,
0X31,0X86,0X18,0XC3,0X21,0X04,0X18,0XA3,0X29,0X44,0XEE,0X6C,0XFF,0X4E,0XFE,0XCF,
0XFE,0XCE,0XFE,0XEE,0XFF,0X2E,0X62,0XC8,0X21,0X26,0X41,0XC7,0X41,0XE7,0X39,0XC6,
0X39,0XC7,0X4A,0X49,0X29,0X24,0X94,0X92,0XFF,0XFF,0XFF,0XFF,0XDE,0XDB,0XC6,0X18,
0X5A,0XAA,0X52,0X8A,0X5A,0XCB,0X5A,0XCA,0X5A,0XCB,0X5A,0XCB,0X5A,0XCB,0X5A,0XCB,
0X5A,0XCB,0X5A,0XCB,0X5A,0XCB,0X5A,0XCB,0X62,0XEB,0X5A,0XCB,0X31,0X85,0X83,0XEF,
0XA5,0X34,0X52,0X8A,0X5A,0X8A,0X41,0XE7,0X21,0X04,0X18,0XC3,0X29,0X04,0X29,0X04,
0X10,0X83,0X29,0X44,0XEE,0X6D,0XFF,0X4F,0XFE,0XEE,0XFE,0XEE,0XFE,0XED,0XFF,0X6E,
0X73,0X29,0X21,0X06,0X39,0XC7,0X41,0XE7,0X41,0XE7,0X41,0XC7,0X20,0XE3,0X8C,0X30,
0XE7,0X3C,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XDE,0XFB,0X39,0XC7,0X52,0X69,0X52,0X8A,
0X52,0X8A,0X52,0X8A,0X52,0X8A,0X52,0X8A,0X52,0X8A,0X52,0X8A,0X52,0X8A,0X5A,0XAA,
0X52,0X69,0X31,0X86,0X21,0X04,0X6B,0X4D,0XFF,0XFF,0XFF,0XFF,0X29,0X45,0X20,0XC3,
0X18,0XE3,0X21,0X04,0X21,0X24,0X21,0X24,0X21,0X04,0X08,0X63,0X39,0XA5,0XFE,0XEE,
0XFF,0X2F,0XF6,0XEE,0XFE,0XEE,0XFE,0XCD,0XFF,0XAE,0X8B,0XE9,0X18,0XC5,0X39,0XC7,
0X39,0XC7,0X41,0XC7,0X41,0XE7,0X20,0XE3,0X8C,0X51,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XB5,0X96,0X20,0XE3,0X39,0XC7,0X39,0XC7,0X41,0XE7,0X39,0XC7,0X39,0XC7,
0X39,0XC7,0X39,0XC7,0X39,0XA6,0X39,0XA6,0X31,0X86,0X39,0XA6,0X4A,0X28,0XA5,0X14,
0XFF,0XFF,0XFF,0XFF,0XEF,0X5D,0X29,0X45,0X18,0XA2,0X21,0X24,0X21,0X24,0X21,0X24,
0X21,0X04,0X21,0X04,0X00,0X22,0X52,0X66,0XFF,0X6E,0XFE,0XEE,0XFE,0XCF,0XFE,0XEE,
0XFE,0XCE,0XFF,0XAE,0XB4,0XCA,0X18,0XE5,0X39,0XC7,0X39,0XC7,0X39,0XC7,0X41,0XE7,
0X39,0XC7,0X18,0XC3,0X8C,0X30,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XBD,0XD7,0X18,0XC3,
0X39,0XA6,0X39,0XA6,0X31,0X86,0X39,0X86,0X31,0X86,0X31,0X65,0X31,0X65,0X31,0X65,
0X31,0X65,0X18,0XA2,0X42,0X28,0XF7,0X9E,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XEF,0X5D,
0X31,0X65,0X18,0XC3,0X29,0X24,0X21,0X24,0X21,0X24,0X21,0X24,0X21,0X04,0X00,0X02,
0X7B,0X68,0XFF,0XEF,0XFE,0XEE,0XFE,0XEF,0XFE,0XEE,0XFE,0XCE,0XFF,0X6E,0XDE,0X0C,
0X29,0X66,0X31,0X86,0X39,0XA7,0X39,0XC7,0X39,0XC7,0X41,0XE7,0X39,0XA6,0X10,0X82,
0X5A,0XCB,0XE7,0X3C,0XFF,0XFF,0XFF,0XFF,0X39,0X86,0X31,0X45,0X31,0X66,0X31,0X86,
0X39,0XA6,0X39,0X86,0X39,0X86,0X31,0X86,0X31,0X86,0X31,0X86,0X18,0XA2,0X00,0X00,
0XBD,0XF7,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XDE,0XFB,0X18,0XC3,0X20,0XE3,0X29,0X24,
0X21,0X24,0X21,0X04,0X21,0X04,0X21,0X04,0X00,0X02,0XB4,0XC9,0XFF,0XEF,0XF6,0XCE,
0XFE,0XEF,0XFE,0XEF,0XFE,0XEE,0XFE,0XED,0XFF,0X8E,0X6B,0X08,0X10,0XC5,0X39,0XA6,
0X39,0XA7,0X39,0XC7,0X39,0XA6,0X41,0XC7,0X39,0XA7,0X18,0X82,0X39,0XA6,0X94,0XB2,
0XCE,0X59,0X52,0X69,0X20,0XC3,0X29,0X04,0X20,0XE4,0X10,0X82,0X20,0XE3,0X29,0X04,
0X20,0XE3,0X10,0X61,0X08,0X21,0X31,0X65,0X9C,0XF3,0XF7,0XBE,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XA5,0X14,0X00,0X00,0X21,0X24,0X21,0X24,0X21,0X04,0X21,0X24,0X21,0X04,
0X10,0X82,0X29,0X24,0XEE,0XAD,0XFF,0X4E,0XFE,0XEE,0XFE,0XEF,0XF6,0XEF,0XFE,0XEE,
0XFE,0XCE,0XFF,0XEE,0XBD,0X0A,0X10,0XA5,0X39,0XA7,0X39,0XA6,0X39,0XA6,0X39,0XA7,
0X39,0XA6,0X39,0XA6,0X41,0XC7,0X29,0X25,0X18,0XA2,0X21,0X04,0X29,0X24,0X52,0X49,
0XCE,0X79,0XCE,0X38,0X73,0X8E,0X42,0X28,0X39,0XE7,0X42,0X08,0X5A,0XEB,0XAD,0X34,
0XFF,0XBE,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XAD,0X75,0XFF,0XFF,0X5A,0XEB,0X00,0X20,
0X29,0X24,0X21,0X04,0X21,0X24,0X21,0X04,0X21,0X04,0X00,0X01,0X83,0XA8,0XFF,0XF0,
0XFE,0XCE,0XFE,0XEE,0XFE,0XEF,0XFE,0XEF,0XFE,0XEE,0XF6,0XCE,0XFF,0X2E,0XFE,0XED,
0X41,0XE7,0X20,0XE5,0X39,0XA6,0X39,0XA6,0X39,0XA6,0X39,0XA6,0X39,0XA6,0X39,0XA6,
0X39,0XA6,0X39,0XA6,0X31,0X66,0X08,0X20,0XA5,0X14,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XF7,0X9E,0XFF,0XDF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XDE,0XFB,
0X52,0XAA,0X00,0X00,0X52,0XAA,0X29,0X45,0X21,0X04,0X21,0X24,0X21,0X04,0X21,0X04,
0X21,0X04,0X18,0XA3,0X10,0X83,0XE6,0X0C,0XFF,0X8F,0XFE,0XCE,0XFE,0XEF,0XFE,0XEF,
0XFE,0XEF,0XFE,0XEF,0XF6,0XEE,0XFE,0XAE,0XFF,0XEF,0XA4,0X6A,0X00,0X24,0X39,0XA6,
0X39,0X86,0X39,0XA6,0X39,0X86,0X39,0X86,0X39,0X86,0X39,0XA6,0X31,0X86,0X39,0X86,
0X21,0X04,0X4A,0X28,0XCE,0X59,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XE7,0X1C,0X84,0X10,0X18,0XC3,0X08,0X20,0X21,0X24,0X08,0X41,
0X18,0XE3,0X29,0X45,0X29,0X04,0X21,0X24,0X21,0X04,0X29,0X24,0X00,0X01,0X6A,0XE7,
0XFF,0XEF,0XFE,0XCE,0XFE,0XEE,0XFE,0XEF,0XFE,0XEF,0XFF,0X0F,0XFE,0XCF,0XFE,0XEE,
0XFE,0XCE,0XFF,0X0E,0XFF,0X6E,0X52,0X47,0X10,0X85,0X39,0X86,0X31,0XA6,0X39,0X86,
0X31,0X86,0X31,0X86,0X31,0X86,0X39,0X86,0X31,0X86,0X39,0XA7,0X20,0XE3,0X20,0XC3,
0X52,0X8A,0X84,0X10,0X9C,0XF3,0XA5,0X34,0XA4,0XF3,0X84,0X30,0X5A,0XCB,0X21,0X24,
0X00,0X20,0X18,0XE3,0X29,0X65,0X29,0X45,0X29,0X24,0X29,0X24,0X21,0X24,0X21,0X24,
0X21,0X04,0X21,0X24,0X08,0X62,0X18,0XC3,0XEE,0X8D,0XFF,0X8F,0XFE,0XCE,0XFE,0XEF,
0XFE,0XEF,0XFE,0XF0,0XFE,0XF0,0XFE,0XEF,0XFE,0XEF,0XFE,0XEE,0XFE,0XAE,0XFF,0XAF,
0XEE,0X8C,0X20,0XE5,0X18,0XE5,0X39,0X86,0X31,0X86,0X31,0X86,0X31,0X86,0X31,0X86,
0X31,0X86,0X31,0X86,0X31,0X66,0X39,0X86,0X31,0X45,0X18,0XA2,0X08,0X41,0X08,0X41,
0X08,0X61,0X08,0X41,0X08,0X20,0X08,0X61,0X20,0XE3,0X29,0X45,0X29,0X45,0X29,0X45,
0X29,0X24,0X29,0X24,0X21,0X24,0X21,0X04,0X21,0X04,0X21,0X04,0X18,0XA3,0X00,0X01,
0XC5,0X4B,0XFF,0XEF,0XF6,0XCD,0XFE,0XEE,0XFE,0XEF,0XF6,0XEF,0XFE,0XF0,0XFE,0XF0,
0XFE,0XEF,0XFE,0XEF,0XFE,0XEE,0XFE,0XCE,0XFE,0XAD,0XFF,0XEF,0XB4,0XEA,0X08,0X44,
0X21,0X25,0X31,0X86,0X31,0X66,0X31,0X65,0X31,0X66,0X31,0X66,0X31,0X66,0X31,0X66,
0X31,0X86,0X31,0X65,0X31,0X65,0X31,0X65,0X31,0X65,0X29,0X45,0X29,0X45,0X29,0X45,
0X29,0X45,0X29,0X45,0X29,0X45,0X29,0X24,0X29,0X24,0X29,0X24,0X21,0X24,0X21,0X04,
0X21,0X24,0X21,0X04,0X18,0XE3,0X00,0X01,0X83,0X87,0XFF,0XEF,0XFE,0XEE,0XFE,0XCE,
0XFE,0XEF,0XFE,0XEF,0XFF,0X10,0XFF,0X10,0XFF,0X10,0XFE,0XF0,0XF6,0XEF,0XFE,0XEF,
0XF6,0XEE,0XFE,0XCE,0XFE,0XEE,0XFF,0XEF,0XA4,0X8A,0X00,0X24,0X21,0X05,0X39,0X86,
0X31,0X66,0X31,0X65,0X31,0X65,0X31,0X65,0X31,0X65,0X31,0X65,0X29,0X65,0X29,0X65,
0X31,0X65,0X31,0X45,0X31,0X45,0X29,0X45,0X29,0X45,0X29,0X45,0X29,0X45,0X29,0X45,
0X29,0X24,0X29,0X24,0X21,0X24,0X21,0X04,0X21,0X04,0X29,0X24,0X18,0XE3,0X00,0X01,
0X73,0X27,0XFF,0XEF,0XFF,0X2E,0XFE,0XCE,0XF6,0XEF,0XFE,0XEF,0XFE,0XF0,0XFE,0XF0,
0XFF,0X10,0XFF,0X10,0XFF,0X10,0XFE,0XEF,0XFE,0XEF,0XFE,0XEF,0XF6,0XEE,0XF6,0XCE,
0XFF,0X0E,0XFF,0XEF,0X9C,0X6A,0X08,0X44,0X18,0XA4,0X31,0X85,0X31,0X65,0X31,0X65,
0X29,0X65,0X31,0X65,0X29,0X65,0X29,0X65,0X29,0X65,0X29,0X45,0X29,0X45,0X29,0X45,
0X29,0X45,0X29,0X45,0X29,0X24,0X29,0X24,0X29,0X24,0X29,0X24,0X21,0X24,0X21,0X24,
0X21,0X04,0X29,0X24,0X10,0XA3,0X00,0X01,0X6A,0XE7,0XFF,0XEF,0XFF,0X6F,0XFE,0XCE,
0XF6,0XEF,0XFE,0XEF,0XFE,0XEF,0XFE,0XF0,0XFF,0X10,0XFF,0X11,0XFF,0X11,0XFF,0X10,
0XF7,0X10,0XFE,0XEF,0XFE,0XEF,0XFE,0XEF,0XF6,0XCE,0XF6,0XCE,0XFF,0X0E,0XFF,0XEF,
0XCD,0XAB,0X29,0X05,0X00,0X24,0X21,0X45,0X31,0X65,0X31,0X45,0X29,0X45,0X29,0X65,
0X29,0X65,0X29,0X45,0X29,0X45,0X29,0X45,0X29,0X45,0X29,0X45,0X29,0X24,0X29,0X24,
0X29,0X24,0X21,0X24,0X21,0X24,0X21,0X04,0X21,0X24,0X19,0X04,0X00,0X02,0X00,0X02,
0X9C,0X69,0XFF,0XEF,0XFF,0X4E,0XFE,0XCD,0XFE,0XEE,0XFE,0XEF,0XFE,0XEF,0XFE,0XF0,
0XFF,0X10,0XFF,0X10,0XFF,0X11,0XFF,0X11,0XFF,0X10,0XFE,0XF0,0XFE,0XF0,0XFE,0XF0,
0XFE,0XEF,0XF6,0XEF,0XFE,0XEE,0XFE,0XCE,0XFE,0XCE,0XFF,0XEF,0XE6,0X8D,0X5A,0XA7,
0X10,0X64,0X08,0X43,0X29,0X45,0X29,0X65,0X29,0X45,0X29,0X45,0X29,0X45,0X29,0X45,
0X29,0X45,0X29,0X45,0X29,0X24,0X29,0X24,0X29,0X24,0X21,0X24,0X29,0X24,0X21,0X24,
0X21,0X05,0X00,0X22,0X00,0X02,0X39,0XC5,0XC5,0X8B,0XFF,0XF0,0XFF,0X0E,0XFE,0XCE,
0XFE,0XEF,0XFE,0XEF,0XFE,0XEF,0XFF,0X10,0XFE,0XF0,0XFF,0X10,0XF7,0X11,0XFF,0X11,
0XFF,0X11,0XFF,0X11,0XFF,0X11,0XFF,0X10,0XFE,0XF0,0XFE,0XEF,0XFE,0XEF,0XFE,0XEF,
0XFE,0XEE,0XFE,0XCE,0XF6,0XAE,0XFF,0XAF,0XFF,0XCF,0XBD,0X6B,0X52,0X87,0X08,0X83,
0X00,0X03,0X10,0X84,0X19,0X04,0X21,0X24,0X29,0X45,0X29,0X24,0X29,0X24,0X21,0X24,
0X21,0X24,0X21,0X04,0X18,0XE4,0X08,0X63,0X00,0X02,0X00,0X02,0X41,0XC5,0XA4,0XCA,
0XFF,0X8E,0XFF,0XEF,0XFE,0XCE,0XF6,0XCE,0XFE,0XEE,0XFE,0XEF,0XFE,0XEF,0XFF,0X10,
0XFE,0XF0,0XFF,0X10,0XFF,0X11,0XFF,0X11,0XFF,0X12,0XFF,0X12,0XFF,0X11,0XFF,0X11,
0XFF,0X10,0XFE,0XF0,0XFE,0XF0,0XFE,0XEF,0XFE,0XEF,0XFE,0XEF,0XFE,0XEE,0XFE,0XCE,
0XFE,0XCE,0XFE,0XEE,0XFF,0XEF,0XFF,0XAE,0XBD,0X4B,0X73,0X68,0X42,0X05,0X19,0X04,
0X08,0X83,0X00,0X43,0X00,0X03,0X00,0X22,0X00,0X22,0X00,0X42,0X08,0X43,0X18,0XC3,
0X31,0XA4,0X62,0XE7,0XAC,0XEA,0XFF,0X4E,0XFF,0XEF,0XFF,0X0E,0XFE,0XAE,0XF6,0XEE,
0XFE,0XEF,0XFE,0XCF,0XFE,0XEF,0XFF,0X10,0XFE,0XF0,0XFF,0X10,0XFF,0X11,0XFF,0X11,
0XFF,0X11,0XFF,0X12,0XFF,0X32,0XFF,0X12,0XFF,0X11,0XFF,0X11,0XFF,0X11,0XFF,0X10,
0XFE,0XF0,0XFE,0XF0,0XFE,0XEF,0XF6,0XEF,0XFE,0XEF,0XFE,0XCE,0XF6,0XCE,0XF6,0XCE,
0XFE,0XEE,0XFF,0XCF,0XFF,0XEF,0XFF,0X0E,0XDD,0XEC,0XBD,0X0B,0XA4,0X4A,0X93,0XE9,
0X93,0XE8,0X94,0X09,0XA4,0X69,0XB5,0X0A,0XD5,0XEC,0XFE,0XCD,0XFF,0XCF,0XFF,0XEF,
0XFF,0X2E,0XF6,0XAE,0XFE,0XEE,0XFE,0XEF,0XFE,0XEF,0XFE,0XEF,0XFE,0XEF,0XFE,0XF0,
0XFE,0XF0,0XFF,0X10,0XFF,0X11,0XFF,0X11,0XFF,0X12,0XFF,0X32,0XFF,0X32,0XFF,0X32,
0XFF,0X12,0XFF,0X12,0XFF,0X11,0XFF,0X11,0XFF,0X10,0XFF,0X10,0XFF,0X10,0XFE,0XF0,
0XFE,0XEF,0XFE,0XEF,0XFE,0XEF,0XFE,0XEF,0XFE,0XEE,0XFE,0XEE,0XF6,0XEE,0XFE,0XCE,
0XFF,0X0E,0XFF,0X6E,0XFF,0XCF,0XFF,0XEF,0XFF,0XEF,0XFF,0XEF,0XFF,0XCF,0XFF,0XCF,
0XFF,0XCF,0XFF,0X8E,0XFF,0X2E,0XFE,0XEE,0XFE,0XEE,0XFE,0XEE,0XFE,0XEF,0XFE,0XEF,
0XF6,0XEF,0XFE,0XEF,0XF6,0XEF,0XFE,0XF0,0XFF,0X10,0XFF,0X10,0XFF,0X11,0XFF,0X11,
0XFF,0X31,0XFF,0X12,0XFF,0X32,0XFF,0X33,0XFF,0X33,0XFF,0X33,0XFF,0X12,0XFF,0X12,
0XFF,0X12,0XFF,0X11,0XFF,0X11,0XFE,0XF1,0XFF,0X10,0XFF,0X10,0XFE,0XF0,0XFE,0XEF,
0XFE,0XEF,0XFE,0XEF,0XFE,0XEF,0XFE,0XEF,0XFE,0XEE,0XFE,0XCE,0XFE,0XCE,0XFE,0XCE,
0XF6,0XEE,0XFE,0XCE,0XFE,0XEE,0XFE,0XEE,0XFE,0XEE,0XFE,0XEE,0XFE,0XEE,0XFE,0XCF,
0XFE,0XCF,0XFE,0XEF,0XFE,0XEF,0XFE,0XEF,0XFE,0XEF,0XFE,0XF0,0XFF,0X10,0XFE,0XF0,
0XFF,0X10,0XFF,0X11,0XFF,0X11,0XFF,0X11,0XFF,0X12,0XFF,0X12,0XFF,0X12,0XFF,0X33,
0XFF,0X33,};

const unsigned char gImage_Right_arrow[1050] = {  
0XFC,0XE0,0XFC,0XE0,0XFC,0XE0,0XFC,0XE0,0XFC,0XE0,0XFC,0XC0,0XFC,0XE0,0XFC,0XE0,
0XFC,0XE0,0XFC,0XE0,0XFC,0XE0,0XFC,0XE0,0XFC,0XC0,0XFC,0XC0,0XFC,0XE0,0XFC,0XE0,
0XFC,0XE0,0XFC,0XE0,0XFC,0XC0,0XFC,0XE0,0XFC,0XC0,0XFC,0XE0,0XFC,0XC0,0XFC,0XE0,
0XFC,0XE0,0XFC,0XE0,0XFC,0XE0,0XFC,0XE0,0XFC,0XE0,0XFC,0XC0,0XFC,0XC0,0XFC,0XE0,
0XFC,0XC0,0XFC,0XE0,0XFC,0XE0,0XFC,0XC0,0XFC,0XE0,0XFC,0XE0,0XFC,0XE0,0XFC,0XE0,
0XFC,0XC0,0XFC,0XE0,0XFC,0XE0,0XFC,0XE0,0XFC,0XC0,0XFC,0XE0,0XFC,0XC0,0XFC,0XE0,
0XFC,0XE0,0XFC,0XE0,0XFC,0XC0,0XFC,0XC0,0XFC,0XE0,0XFC,0XE0,0XFC,0XC0,0XFC,0XE0,
0XFC,0XC0,0XFC,0XE0,0XFC,0XE0,0XFC,0XE0,0XFC,0XC0,0XFC,0XE0,0XFC,0XE0,0XFC,0XE0,
0XFC,0XE0,0XFC,0XE0,0XFC,0XC0,0XFC,0XE0,0XFC,0XC0,0XFC,0XC0,0XFC,0XE0,0XFC,0XE0,
0XE5,0X4A,0XDD,0XD2,0XD5,0XF3,0XDD,0XF3,0XD5,0XF1,0XDE,0X13,0XDE,0X32,0XE6,0X53,
0XEE,0X94,0XF6,0XB5,0XF6,0XF6,0XFF,0X57,0XFF,0X57,0XFF,0X58,0XFF,0X37,0XFF,0X58,
0XFF,0X77,0XFF,0X58,0XFF,0X78,0XFF,0X99,0XF7,0X79,0XFF,0X9A,0XFF,0X9B,0XFF,0XBB,
0XFF,0XBC,0XFF,0X16,0XFC,0XE0,0XFC,0XE0,0XFC,0XE0,0XFC,0XE0,0XFC,0XE0,0XFC,0XE0,
0XFC,0XE0,0XFC,0XE0,0XE5,0X6B,0X9C,0XD5,0X2A,0X90,0X3B,0X11,0X43,0X52,0X4B,0X32,
0X4B,0X33,0X4B,0X54,0X4B,0X34,0X53,0X55,0X53,0X56,0X53,0X57,0X53,0X58,0X53,0X38,
0X53,0X39,0X53,0X19,0X5B,0X1B,0X5B,0X1B,0X5A,0XDC,0X5A,0XDC,0XAD,0X5D,0XFF,0XFE,
0X73,0XBD,0X52,0XFD,0X6B,0X9D,0X6B,0XDD,0XA5,0X7E,0XFF,0XFF,0XFE,0X0B,0XFC,0XC0,
0XFC,0XE0,0XFC,0XE0,0XFC,0XE0,0XFC,0XE0,0XFC,0XC0,0XFC,0XE0,0XD5,0X4D,0X43,0X52,
0X00,0XED,0X01,0X8F,0X09,0XD1,0X09,0XB2,0X01,0X72,0X09,0X73,0X09,0X54,0X09,0X35,
0X01,0X15,0X08,0XF6,0X00,0XD7,0X08,0XD8,0X00,0XB9,0X08,0X9A,0X08,0X7B,0X08,0X7C,
0X00,0X5C,0X00,0X3C,0X6B,0XBD,0X94,0XDD,0X00,0X7D,0X00,0X5C,0X09,0X1D,0X00,0XBD,
0X63,0XBD,0XFF,0XFF,0XFF,0X78,0XFC,0XE0,0XFC,0XE0,0XFC,0XE0,0XFC,0XE0,0XFC,0XE0,
0XFC,0XC0,0XFC,0XE0,0XF5,0X47,0X84,0X75,0X01,0X70,0X11,0XD2,0X22,0X34,0X22,0X14,
0X22,0X15,0X21,0XF6,0X21,0XD7,0X29,0XD8,0X21,0XB8,0X21,0X99,0X21,0X9A,0X29,0X9B,
0X29,0X5C,0X29,0X7D,0X29,0X7D,0X29,0X9C,0X29,0X9D,0X00,0XBC,0X6B,0XDD,0XC6,0X5E,
0X32,0X5D,0X09,0X7D,0X2A,0X5D,0X01,0X5D,0X43,0X1D,0XF7,0XBF,0XFF,0XBC,0XFC,0XE0,
0XFC,0XC0,0XFC,0XC0,0XFC,0XE0,0XFC,0XE0,0XFC,0XE0,0XFC,0XC0,0XFC,0XE0,0X9C,0XD5,
0X11,0XD2,0X19,0XF4,0X21,0XF6,0X21,0XF6,0X21,0XB7,0X21,0XD8,0X21,0XB9,0X21,0X99,
0X29,0X7A,0X29,0X7B,0X29,0X5C,0X29,0X5D,0X29,0X9D,0X29,0X9C,0X29,0XBD,0X29,0XFC,
0X29,0XFD,0X00,0XFC,0X6B,0XDE,0XBE,0X1E,0X32,0X5D,0X22,0X3D,0X2A,0X9D,0X01,0X9D,
0X33,0X1D,0XE7,0X7F,0XFF,0X9C,0XFC,0XE0,0XFC,0XE0,0XFC,0XE0,0XFC,0XE0,0XFC,0XE0,
0XFC,0XE0,0XFC,0XE0,0XFC,0XC0,0XA5,0X37,0X00,0XD4,0X21,0XD6,0X21,0XD7,0X21,0XB8,
0X21,0XB9,0X29,0X9A,0X29,0X7B,0X29,0X7C,0X29,0X5D,0X31,0X7D,0X29,0X9C,0X29,0XBC,
0X29,0XBD,0X29,0XFC,0X21,0XFD,0X2A,0X1D,0X2A,0X5D,0X00,0X9C,0X7C,0X5E,0XD6,0XDF,
0X01,0XBD,0X2A,0XBD,0X32,0XFD,0X01,0X7D,0X3B,0X5E,0XFF,0XFF,0XFC,0XC0,0XFC,0XC0,
0XFC,0XC0,0XFC,0XE0,0XFC,0XE0,0XFC,0XE0,0XFC,0XE0,0XFC,0XE0,0X63,0X56,0X09,0X36,
0X21,0XB8,0X21,0XB8,0X21,0X99,0X29,0X9A,0X29,0X7B,0X29,0X5C,0X29,0X5C,0X29,0X9C,
0X29,0X9C,0X29,0XBD,0X29,0XDC,0X29,0XFC,0X29,0XFD,0X2A,0X1D,0X2A,0X3D,0X19,0XDC,
0X11,0XDD,0XAD,0XDE,0X84,0XDE,0X12,0X3D,0X2A,0XFD,0X1A,0X9D,0X02,0X1D,0X8D,0X5E,
0XFF,0XDE,0XFC,0XE0,0XFC,0XE0,0XFC,0XE0,0XFC,0XC0,0XFC,0XE0,0XFD,0X01,0XFC,0XE0,
0X52,0XB4,0X00,0XB6,0X29,0XD8,0X21,0XB9,0X29,0X99,0X29,0X9A,0X29,0X7B,0X29,0X5C,
0X29,0X7D,0X29,0X7D,0X29,0X9D,0X29,0XBD,0X29,0XDD,0X21,0XDC,0X2A,0X1D,0X2A,0X1D,
0X2A,0X5D,0X11,0XBD,0X1A,0X3D,0XC6,0X5F,0X84,0XFE,0X01,0XDD,0X2A,0XFD,0X1A,0X9D,
0X02,0X1D,0X9D,0XBE,0XFF,0XDE,0XFC,0XE0,0XFC,0XC0,0XFC,0XC0,0XFC,0XC0,0XFC,0XE0,
0XFD,0X00,0XFC,0XC0,0X4A,0X73,0X00,0X96,0X21,0XB8,0X21,0X99,0X21,0X7A,0X21,0X7B,
0X21,0X5C,0X21,0X3C,0X29,0X5C,0X29,0X7C,0X21,0X9C,0X21,0X9D,0X29,0XBC,0X29,0XDD,
0X21,0XFD,0X22,0X1D,0X2A,0X3D,0X09,0X7D,0X19,0XFD,0XB6,0X1E,0X74,0X9E,0X01,0XBD,
0X22,0XBD,0X0A,0X3D,0X02,0X5D,0XAE,0X3F,0XFF,0XDD,0XFC,0XE0,0XFC,0XE0,0XFC,0XE0,
0XFC,0XE0,0XFC,0XE0,0XFD,0X01,0XFC,0XE0,0X11,0X74,0X08,0X55,0X11,0X18,0X10,0XF9,
0X10,0XDB,0X10,0XDB,0X10,0XBC,0X10,0XBD,0X10,0XBD,0X10,0XFC,0X11,0X1C,0X11,0X1C,
0X11,0X3D,0X11,0X5C,0X11,0X7C,0X11,0X7D,0X11,0X9D,0X09,0X9C,0X43,0X1D,0XCE,0X9F,
0X64,0X1E,0X01,0X5D,0X12,0X7D,0X1A,0X9D,0X3B,0X7D,0XCE,0XDF,0XFF,0XDD,0XFC,0XE0,
0XFC,0XC0,0XFC,0XE0,0XFC,0XE0,0XFC,0XE0,0XFC,0XE0,0XFE,0X70,0XCE,0X7C,0XA5,0X5C,
0XC6,0X1D,0XBD,0XFD,0XBE,0X1E,0XBD,0XFD,0XC6,0X1E,0XC5,0XFE,0XC6,0X1F,0XC6,0X3E,
0XC6,0X3F,0XC6,0X3F,0XC6,0X5F,0XC6,0X5F,0XC6,0X7F,0XC6,0X7F,0XC6,0X9F,0XC6,0X7F,
0XCE,0XBF,0XFF,0XFF,0XFF,0XFF,0XC6,0X9F,0XD7,0X1F,0XD7,0X1F,0XDF,0X5F,0XFF,0XFF,
0XFF,0X58,0XFC,0XE0,0XFC,0XC0,0XFC,0XE0,0XFC,0XE0,0XFC,0XE0,0XFC,0XE0,0XFC,0XE0,
0XFC,0XC0,0XFC,0XE0,0XFD,0X64,0XFD,0X86,0XFD,0XA6,0XFD,0X65,0XFD,0X65,0XFD,0X86,
0XFD,0X86,0XFD,0X86,0XFD,0XA6,0XFD,0XA6,0XFD,0XA6,0XFD,0XA6,0XFD,0XA7,0XFD,0XA7,
0XFD,0XE8,0XFD,0XE9,0XFD,0XA7,0XFD,0XC8,0XFD,0XC8,0XFD,0XCA,0XFD,0XE9,0XFD,0XEA,
0XFE,0X2B,0XFD,0X65,0XFC,0XC0,0XFC,0XE0,0XFC,0XE0,0XFC,0XE0,0XFC,0XE0,0XFC,0XE0,
0XFC,0XE0,0XFC,0XE0,0XFC,0XE0,0XFC,0XE0,0XFC,0XE0,0XFC,0XE0,0XFC,0XE0,0XFC,0XE0,
0XFC,0XE0,0XFC,0XC0,0XFC,0XE0,0XFC,0XE0,0XFC,0XC0,0XFC,0XE0,0XFC,0XE0,0XFC,0XE0,
0XFC,0XE0,0XFC,0XE0,0XFC,0XE0,0XFC,0XC0,0XFC,0XE0,0XFC,0XE0,0XFC,0XC0,0XFC,0XE0,
0XFC,0XC0,0XFC,0XC0,0XFC,0XE0,0XFC,0XE0,0XFC,0XE0,0XFC,0XC0,0XFC,0XE0,0XFC,0XE0,
0XFC,0XE0,0XFC,0XE0,0XFC,0XE0,0XFC,0XC0,0XFC,0XC0,};

const unsigned char gImage_Left_arrow[1050] = {  
0XFC,0XE0,0XFC,0XC0,0XFC,0XC0,0XFC,0XE0,0XFC,0XE0,0XFC,0XC0,0XFC,0XE0,0XFC,0XE0,
0XFC,0XE0,0XFC,0XC0,0XFC,0XE0,0XFC,0XC0,0XFC,0XE0,0XFC,0XC0,0XFC,0XE0,0XFC,0XC0,
0XFC,0XE0,0XFC,0XE0,0XFC,0XC0,0XFC,0XE0,0XFC,0XE0,0XFC,0XE0,0XFC,0XE0,0XFC,0XC0,
0XFC,0XE0,0XFC,0XE0,0XFC,0XE0,0XFC,0XE0,0XFC,0XC0,0XFC,0XE0,0XFC,0XE0,0XFC,0XE0,
0XFC,0XE0,0XFC,0XE0,0XFC,0XE1,0XFC,0XE0,0XFC,0XE0,0XFC,0XE0,0XFC,0XE0,0XFC,0XE0,
0XFC,0XC0,0XFC,0XE0,0XFC,0XE0,0XFD,0X85,0XFE,0X2C,0XFE,0X09,0XFD,0XE9,0XFD,0XCA,
0XFD,0XC8,0XFD,0XC8,0XFD,0XA7,0XFD,0XE9,0XFD,0XC8,0XFD,0XA7,0XFD,0XA7,0XFD,0XA6,
0XFD,0XA7,0XFD,0XA6,0XFD,0X86,0XFD,0X86,0XFD,0X86,0XFD,0X86,0XFD,0X65,0XFD,0XA5,
0XFD,0X86,0XFD,0X86,0XFD,0X63,0XFC,0XE0,0XFC,0XE0,0XFD,0X00,0XFC,0XE0,0XFC,0XE0,
0XFC,0XC0,0XFC,0XC0,0XFC,0XE0,0XFC,0XE0,0XFF,0X79,0XFF,0XFF,0XDF,0X5F,0XD6,0XFF,
0XD7,0X1F,0XC6,0XBF,0XFF,0XFF,0XFF,0XFF,0XC6,0X9F,0XC6,0X7F,0XC6,0X7F,0XCE,0X7F,
0XC6,0X7F,0XC6,0X7F,0XC6,0X3F,0XC6,0X3F,0XC6,0X3E,0XC6,0X3E,0XC5,0XFE,0XC6,0X1E,
0XC5,0XFE,0XBD,0XFE,0XBD,0XFD,0XBD,0XFD,0XBE,0X1D,0X9D,0X5C,0XD6,0X9B,0XF6,0X4F,
0XFC,0XE0,0XFC,0XE0,0XFC,0XE0,0XFC,0XE0,0XFC,0XE0,0XFC,0XE0,0XFF,0XDD,0XCE,0XBF,
0X33,0X7D,0X1A,0X9D,0X12,0X7D,0X01,0X5D,0X6C,0X3E,0XCE,0XBF,0X3A,0XFD,0X09,0X9D,
0X09,0X9D,0X11,0X9D,0X11,0X7C,0X11,0X5C,0X11,0X3D,0X11,0X1C,0X10,0XFC,0X10,0XFC,
0X10,0XBC,0X10,0XBD,0X10,0XBC,0X08,0XDB,0X10,0XDA,0X10,0XF9,0X11,0X38,0X08,0X55,
0X19,0X74,0XFC,0XE0,0XFD,0X01,0XFC,0XE0,0XFC,0XC0,0XFC,0XE0,0XFC,0XE0,0XFC,0XE0,
0XFF,0XDD,0XAE,0X1F,0X02,0X5D,0X0A,0X3D,0X22,0XBD,0X01,0XBD,0X74,0X9E,0XB6,0X1F,
0X11,0XFC,0X09,0X5D,0X2A,0X3D,0X2A,0X1D,0X21,0XFD,0X29,0XDD,0X29,0XDD,0X21,0XBC,
0X21,0X9C,0X21,0X7C,0X21,0X5D,0X21,0X3D,0X21,0X5C,0X21,0X7B,0X21,0X9A,0X21,0X99,
0X21,0XB8,0X00,0XB6,0X4A,0X93,0XFC,0XE0,0XFD,0X00,0XFC,0XC0,0XFC,0XE0,0XFC,0XE0,
0XFC,0XE0,0XFC,0XE0,0XFF,0XDE,0X9D,0XDF,0X02,0X1D,0X1A,0X7D,0X2A,0XFD,0X01,0XDD,
0X84,0XFE,0XBE,0X5F,0X1A,0X1D,0X11,0XBD,0X2A,0X7C,0X2A,0X1D,0X2A,0X1D,0X29,0XDD,
0X29,0XDD,0X29,0XBD,0X29,0XBC,0X29,0X7D,0X29,0X7D,0X29,0X5C,0X29,0X7B,0X29,0X9A,
0X29,0X99,0X21,0XB9,0X29,0XD8,0X00,0XD5,0X4A,0XD5,0XFC,0XC0,0XFD,0X01,0XFC,0XE0,
0XFC,0XE0,0XFC,0XE0,0XFC,0XE0,0XFC,0XE0,0XFF,0XDE,0X8D,0X5F,0X02,0X1D,0X1A,0X9D,
0X2B,0X1D,0X12,0X3D,0X84,0XDE,0XAD,0XDE,0X11,0XDC,0X11,0XDC,0X2A,0X5D,0X2A,0X1D,
0X2A,0X1D,0X29,0XFC,0X29,0XDC,0X29,0XBD,0X29,0X9C,0X29,0X7D,0X29,0X7C,0X29,0X5C,
0X21,0X7B,0X29,0X9A,0X29,0X99,0X21,0XB9,0X21,0XB8,0X09,0X36,0X63,0X75,0XFC,0XC0,
0XFC,0XE0,0XFC,0XE0,0XFC,0XE0,0XFC,0XC0,0XFC,0XE0,0XFC,0XE0,0XFC,0XE0,0XFF,0XFF,
0X33,0X5D,0X01,0X7D,0X33,0X1D,0X2A,0XBD,0X09,0XBD,0XD6,0XDE,0X7C,0X5E,0X00,0X9C,
0X2A,0X5D,0X2A,0X1D,0X29,0XFD,0X29,0XDD,0X29,0XBD,0X29,0XBC,0X29,0X9C,0X29,0X7D,
0X29,0X5D,0X29,0X5C,0X21,0X7A,0X21,0XBA,0X21,0XB9,0X21,0XB8,0X21,0XD7,0X21,0XD6,
0X00,0XF4,0XA5,0X37,0XFC,0XE0,0XFC,0XE0,0XFC,0XE0,0XFC,0XE0,0XFC,0XE0,0XFC,0XE0,
0XFC,0XC0,0XFC,0XC0,0XFF,0X9B,0XE7,0X7F,0X3B,0X3D,0X01,0X7D,0X2A,0X9D,0X22,0X3D,
0X2A,0X5D,0XBE,0X1E,0X6B,0XFD,0X00,0XFD,0X29,0XDD,0X29,0XFC,0X29,0XBD,0X29,0X9D,
0X29,0X9D,0X29,0X7D,0X29,0X7D,0X29,0X7B,0X29,0X7B,0X21,0X99,0X29,0XB9,0X29,0XB8,
0X21,0XD7,0X21,0XD6,0X21,0XF5,0X19,0XD4,0X11,0XB3,0X94,0XD5,0XFC,0XE0,0XFC,0XC0,
0XFC,0XE0,0XFC,0XE0,0XFC,0XE0,0XFC,0XE0,0XFC,0XE0,0XFC,0XE0,0XFF,0X9B,0XF7,0XDF,
0X4B,0X3D,0X01,0X5C,0X2A,0X5D,0X09,0X7D,0X32,0X3D,0XC6,0X5E,0X73,0XFD,0X00,0XBC,
0X29,0X9D,0X29,0XBC,0X29,0X7D,0X29,0X7C,0X29,0X5C,0X29,0X7B,0X29,0X9A,0X21,0X9A,
0X21,0X98,0X21,0XD7,0X21,0XD7,0X21,0XF6,0X21,0XF5,0X22,0X14,0X22,0X53,0X11,0XB2,
0X01,0X71,0X74,0X75,0XED,0X47,0XFC,0XE0,0XFC,0XE0,0XFC,0XC0,0XFC,0XE0,0XFC,0XE0,
0XFC,0XE0,0XFC,0XC0,0XFF,0X58,0XFF,0XFF,0X63,0XDD,0X00,0XDD,0X08,0XFD,0X00,0X5C,
0X00,0X5C,0X94,0XBD,0X73,0XDC,0X00,0X3D,0X00,0X5C,0X10,0X9C,0X08,0X7B,0X08,0X9A,
0X08,0XB9,0X08,0XD8,0X00,0XD7,0X08,0XF7,0X01,0X15,0X09,0X35,0X09,0X54,0X09,0X73,
0X09,0X73,0X01,0XB1,0X09,0XD1,0X01,0X8F,0X00,0XED,0X43,0X32,0XD5,0X4D,0XFC,0XE0,
0XFC,0XE0,0XFC,0XE1,0XFC,0XE0,0XFC,0XC0,0XFC,0XE0,0XFC,0XC0,0XFE,0X0A,0XFF,0XFE,
0XA5,0X7E,0X6B,0XDD,0X6B,0XBE,0X52,0XFD,0X6B,0X9D,0XFF,0XFD,0XAD,0X7D,0X5A,0XFC,
0X52,0XDC,0X5B,0X1B,0X5B,0X1B,0X5B,0X3A,0X53,0X39,0X53,0X38,0X53,0X58,0X53,0X57,
0X53,0X57,0X53,0X55,0X4B,0X54,0X4B,0X34,0X4B,0X32,0X43,0X32,0X4B,0X32,0X3B,0X11,
0X22,0X70,0X94,0XD5,0XE5,0X6B,0XFC,0XE0,0XFC,0XE0,0XFC,0XE0,0XFC,0XC0,0XFC,0XE0,
0XFC,0XE0,0XFC,0XE0,0XFC,0XC0,0XFE,0XF6,0XFF,0XBC,0XFF,0XBB,0XFF,0XBB,0XFF,0X9A,
0XF7,0X79,0XFF,0X79,0XFF,0X78,0XFF,0X58,0XFF,0X77,0XFF,0X58,0XFF,0X37,0XFF,0X37,
0XFF,0X58,0XFF,0X57,0XF6,0XF5,0XF6,0XB5,0XEE,0X94,0XE6,0X53,0XE6,0X33,0XDE,0X13,
0XD5,0XF2,0XDD,0XF2,0XDD,0XF3,0XD5,0XD2,0XE5,0X4A,0XFC,0XE0,0XFC,0XC0,0XFC,0XE1,
0XFC,0XE0,0XFC,0XE0,0XFC,0XE0,0XFC,0XE0,0XFC,0XC0,0XFC,0XC0,0XFC,0XE0,0XFC,0XE0,
0XFC,0XE0,0XFC,0XE0,0XFC,0XE0,0XFC,0XE0,0XFC,0XE0,0XFC,0XC0,0XFC,0XE0,0XFC,0XE0,
0XFC,0XC0,0XFC,0XE0,0XFC,0XE0,0XFC,0XE0,0XFC,0XC0,0XFC,0XE0,0XFC,0XE0,0XFC,0XE0,
0XFC,0XE0,0XFC,0XE0,0XFC,0XE0,0XFC,0XE0,0XFC,0XC0,0XFC,0XC0,0XFC,0XE0,0XFC,0XE0,
0XFC,0XC0,0XFC,0XE0,0XFC,0XE1,0XFC,0XE0,0XFC,0XC0,0XFC,0XE0,0XFC,0XE0,0XFC,0XE0,
0XFC,0XE0,0XFC,0XE0,0XFC,0XE0,0XFC,0XC0,0XFC,0XE0,0XFC,0XE0,0XFC,0XE0,0XFC,0XE0,
0XFC,0XE0,0XFC,0XE0,0XFC,0XC0,0XFC,0XE0,0XFC,0XC0,0XFC,0XE0,0XFC,0XE0,0XFC,0XE0,
0XFC,0XC0,0XFC,0XE0,0XFC,0XE0,0XFC,0XE0,0XFC,0XE0,0XFC,0XE0,0XFC,0XC0,0XFC,0XE0,
0XFC,0XE0,0XFC,0XC0,0XFC,0XE0,0XFC,0XE0,0XFC,0XE0,};

const unsigned char gImage_Power_on[40960] = {  
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFD,0X72,0XFC,0XE0,0XFC,0XE0,
0XFC,0XE0,0XFC,0XE0,0XFF,0XF6,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFC,0XEC,0XFC,0XE0,0XFF,0X72,0XFF,0XFF,
0XFC,0XEC,0XFC,0XE0,0XFF,0XF6,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFD,0XF6,0XFC,0XE0,0XFC,0XE0,0XFC,0XE0,0XFC,0XE0,0XFF,0X72,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFD,0X72,0XFC,0XE0,
0XFC,0XE0,0XFC,0XE0,0XFC,0XE0,0XFC,0XE0,0XFF,0XF6,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0X7F,0XFC,0XE7,0XFC,0XE0,0XFF,0XF6,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFC,0XEC,0XFC,0XE0,0XFC,0XE0,0XFC,0XE0,
0XFC,0XE0,0XFC,0XE0,0XFF,0XF6,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFC,0XEC,0XFC,0XE0,0XFF,0X72,0XFF,0XFF,
0XFC,0XEC,0XFC,0XE0,0XFF,0XF6,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFC,0XEC,0XFC,0XE0,0XFC,0XE0,0XFC,0XE0,0XFC,0XE0,0XFC,0XE0,0XFF,0X72,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFD,0X72,0XFC,0XE0,
0XFC,0XE0,0XFC,0XE0,0XFC,0XE0,0XFC,0XE0,0XFC,0XE0,0XFF,0XF6,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0X7F,0XFC,0XE7,0XFC,0XE0,0XFF,0XF6,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFE,0XFF,0XFC,0XE0,0XFC,0XE0,0XFF,0XF6,0XFF,0XFF,
0XFF,0XFF,0XFD,0XF6,0XFF,0XF6,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFC,0XEC,0XFC,0XE0,0XFF,0X72,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFD,0X72,0XFC,0XE0,0XFC,0XE0,0XFF,0XF6,0XFF,0XFF,0XFF,0XFF,0XFE,0X7B,0XFF,0X72,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFD,0X72,0XFC,0XE0,
0XFF,0X72,0XFF,0XFF,0XFF,0X7F,0XFC,0XE7,0XFC,0XE0,0XFF,0X72,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0X7F,0XFC,0XE7,0XFC,0XE0,0XFF,0XF6,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFE,0XFF,0XFC,0XE0,0XFC,0XE0,0XFE,0X67,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFD,0XF6,0XFC,0XE0,0XFC,0XE0,0XFC,0XE0,
0XFC,0XE0,0XFF,0XF6,0XFF,0XFF,0XFF,0XFF,0XFC,0XEC,0XFC,0XE0,0XFF,0X72,0XFF,0XFF,
0XFC,0XEC,0XFC,0XE0,0XFF,0XF6,0XFD,0XF6,0XFC,0XE0,0XFD,0XE0,0XFF,0XFF,0XFF,0XFF,
0XFE,0X7B,0XFC,0XE0,0XFE,0X67,0XFF,0XFF,0XFD,0XF6,0XFC,0XE0,0XFC,0XE0,0XFC,0XE0,
0XFC,0XE0,0XFF,0XF6,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0X7F,
0XFC,0XE7,0XFC,0XE0,0XFF,0X72,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFE,0XFF,0XFC,0XE0,0XFD,0X60,0XFF,0XFB,0XFF,0XFF,0XFD,0X72,0XFC,0XE0,0XFE,0XEC,
0XFF,0XFF,0XFC,0XEC,0XFC,0XE0,0XFF,0XF6,0XFC,0XEC,0XFC,0XE0,0XFC,0XE0,0XFE,0XEC,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFD,0X72,0XFC,0XE0,
0XFF,0X72,0XFF,0XFF,0XFE,0XFF,0XFC,0XE0,0XFC,0XE0,0XFF,0XF6,0XFF,0XFF,0XFF,0XFF,
0XFD,0XF6,0XFC,0XE0,0XFC,0XE0,0XFC,0XE0,0XFF,0X72,0XFF,0XFF,0XFF,0XFF,0XFD,0XF6,
0XFC,0XE0,0XFC,0XE0,0XFC,0XE0,0XFC,0XE0,0XFF,0XF6,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFC,0XEC,0XFC,0XE0,0XFC,0XE0,0XFE,0XF2,0XFC,0XE7,0XFC,0XE0,0XFF,0XF6,0XFF,0XFF,
0XFF,0XFF,0XFD,0XF6,0XFC,0XE0,0XFC,0XE0,0XFC,0XE0,0XFF,0X72,0XFF,0XFF,0XFF,0XFF,
0XFD,0X72,0XFC,0XE0,0XFE,0XF2,0XFC,0XE7,0XFD,0XE0,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFD,0XF6,0XFC,0XE0,0XFC,0XE0,0XFC,0XE0,
0XFD,0X60,0XFF,0XFB,0XFF,0XFF,0XFF,0XFF,0XFD,0X72,0XFC,0XE0,0XFC,0XE0,0XFC,0XE0,
0XFC,0XE0,0XFD,0X60,0XFF,0XFB,0XFF,0XFF,0XFC,0XEC,0XFC,0XE0,0XFF,0X72,0XFF,0XFF,
0XFC,0XEC,0XFC,0XE0,0XFF,0XF6,0XFF,0X7F,0XFC,0XE7,0XFD,0X60,0XFF,0XFB,0XFF,0XFF,
0XFD,0X72,0XFC,0XE0,0XFF,0X72,0XFF,0XFF,0XFD,0X72,0XFC,0XE0,0XFC,0XE0,0XFC,0XE0,
0XFC,0XE0,0XFD,0X60,0XFF,0XFB,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFE,0XFF,
0XFC,0XE0,0XFD,0X60,0XFF,0XFB,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFE,0XFF,0XFC,0XE0,0XFD,0X60,0XFF,0XFB,0XFF,0XFF,0XFD,0X72,0XFC,0XE0,0XFE,0XEC,
0XFF,0XFF,0XFC,0XEC,0XFC,0XE0,0XFD,0X67,0XFC,0XE0,0XFC,0XE0,0XFC,0XE0,0XFC,0XE0,
0XFF,0X72,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFD,0X72,0XFC,0XE0,
0XFC,0XE0,0XFC,0XE0,0XFC,0XE0,0XFC,0XE0,0XFE,0XEC,0XFF,0XFF,0XFF,0XFF,0XFD,0X72,
0XFC,0XE0,0XFC,0XE0,0XFC,0XE0,0XFC,0XE0,0XFC,0XE0,0XFF,0X72,0XFF,0XFF,0XFD,0X72,
0XFC,0XE0,0XFC,0XE0,0XFC,0XE0,0XFC,0XE0,0XFD,0X60,0XFF,0XFB,0XFF,0XFF,0XFC,0XEC,
0XFC,0XE0,0XFC,0XE0,0XFC,0XE0,0XFC,0XE0,0XFC,0XE0,0XFC,0XE0,0XFF,0XF6,0XFF,0XFF,
0XFD,0X72,0XFC,0XE0,0XFC,0XE0,0XFC,0XE0,0XFC,0XE0,0XFC,0XE0,0XFF,0X72,0XFF,0XFF,
0XFD,0X72,0XFC,0XE0,0XFD,0X67,0XFC,0XE0,0XFD,0XE0,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFE,0XFF,0XFC,0XE0,0XFC,0XE0,
0XFC,0XE0,0XFC,0XE0,0XFF,0XF6,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFC,0XEC,0XFC,0XE0,0XFF,0XF6,0XFF,0XFF,0XFC,0XEC,0XFC,0XE0,0XFF,0X72,0XFF,0XFF,
0XFC,0XEC,0XFC,0XE0,0XFF,0XF6,0XFF,0XFF,0XFD,0X72,0XFC,0XE0,0XFF,0X72,0XFF,0X7F,
0XFC,0XE7,0XFD,0X60,0XFF,0XFB,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFC,0XEC,0XFC,0XE0,0XFF,0XF6,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFE,0X7B,
0XFC,0XE0,0XFD,0X60,0XFF,0XFB,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFE,0XFF,0XFC,0XE0,0XFD,0X60,0XFF,0XFB,0XFF,0XFF,0XFD,0X72,0XFC,0XE0,0XFE,0XEC,
0XFF,0XFF,0XFC,0XEC,0XFC,0XE0,0XFE,0XEC,0XFF,0XFF,0XFF,0XFF,0XFC,0XEC,0XFC,0XE0,
0XFE,0X67,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFD,0X72,0XFC,0XE0,
0XFC,0XE0,0XFC,0XE0,0XFC,0XE0,0XFE,0XEC,0XFF,0XFF,0XFF,0XFF,0XFE,0XFF,0XFC,0XE0,
0XFC,0XE0,0XFF,0XF6,0XFF,0XFF,0XFE,0X7B,0XFC,0XE0,0XFE,0X67,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFC,0XEC,0XFC,0XE0,0XFF,0XF6,0XFE,0XFF,0XFC,0XE0,
0XFC,0XE0,0XFF,0XF6,0XFF,0XFF,0XFE,0XFF,0XFC,0XE0,0XFC,0XE0,0XFF,0XF6,0XFE,0XFF,
0XFC,0XE0,0XFC,0XE0,0XFF,0XF6,0XFF,0XFF,0XFE,0X7B,0XFC,0XE0,0XFE,0X67,0XFF,0XFF,
0XFD,0X72,0XFC,0XE0,0XFE,0X67,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFC,0XEC,0XFC,0XE0,0XFE,0XEC,0XFF,0XFF,0XFD,0XF6,0XFC,0XE0,0XFC,0XE0,0XFC,0XE0,
0XFC,0XE0,0XFC,0XE0,0XFF,0XF6,0XFF,0XFF,0XFC,0XEC,0XFC,0XE0,0XFF,0X72,0XFF,0XFF,
0XFC,0XEC,0XFC,0XE0,0XFF,0XF6,0XFF,0XFF,0XFE,0X7B,0XFC,0XE0,0XFE,0X67,0XFE,0XFF,
0XFC,0XE0,0XFE,0XEC,0XFF,0XFF,0XFF,0XFF,0XFD,0XF6,0XFC,0XE0,0XFC,0XE0,0XFC,0XE0,
0XFC,0XE0,0XFC,0XE0,0XFF,0XF6,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFE,0XFF,
0XFC,0XE0,0XFC,0XE0,0XFF,0XF6,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFE,0XFF,0XFC,0XE0,0XFD,0X60,0XFF,0XFB,0XFF,0XFF,0XFD,0X72,0XFC,0XE0,0XFE,0XEC,
0XFF,0XFF,0XFC,0XEC,0XFC,0XE0,0XFF,0XF6,0XFF,0XFF,0XFF,0XFF,0XFD,0XF6,0XFC,0XE0,
0XFE,0X67,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFD,0X72,0XFC,0XE0,
0XFE,0XEC,0XFF,0X7F,0XFC,0XE7,0XFC,0XE0,0XFF,0X72,0XFF,0XFF,0XFE,0XFF,0XFC,0XE0,
0XFC,0XE0,0XFC,0XE0,0XFC,0XE0,0XFC,0XE0,0XFC,0XE0,0XFE,0X67,0XFF,0XFF,0XFD,0XF6,
0XFC,0XE0,0XFC,0XE0,0XFC,0XE0,0XFC,0XE0,0XFC,0XE0,0XFF,0XF6,0XFE,0X7B,0XFC,0XE0,
0XFD,0XE0,0XFF,0XFF,0XFF,0XFF,0XFF,0X7F,0XFC,0XE7,0XFC,0XE0,0XFF,0XF6,0XFE,0XFF,
0XFC,0XE0,0XFC,0XE0,0XFC,0XE0,0XFC,0XE0,0XFC,0XE0,0XFC,0XE0,0XFE,0X67,0XFF,0XFF,
0XFD,0X72,0XFC,0XE0,0XFF,0X72,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFE,0XFF,0XFE,0X67,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFD,0X72,0XFC,0XE0,0XFE,0XEC,0XFE,0XFF,0XFC,0XE0,0XFE,0X67,0XFF,0XFF,0XFF,0XFF,
0XFC,0XEC,0XFC,0XE0,0XFF,0XF6,0XFF,0XFF,0XFC,0XEC,0XFC,0XE0,0XFF,0X72,0XFF,0XFF,
0XFC,0XEC,0XFC,0XE0,0XFF,0XF6,0XFF,0XFF,0XFF,0X7F,0XFC,0XE7,0XFD,0XE0,0XFD,0XF6,
0XFC,0XE0,0XFF,0XF6,0XFF,0XFF,0XFE,0XFF,0XFC,0XE0,0XFE,0X67,0XFF,0XFF,0XFF,0XFF,
0XFC,0XEC,0XFC,0XE0,0XFF,0XF6,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFC,0XEC,0XFC,0XE0,0XFC,0XE0,0XFF,0XF6,0XFF,0XFF,0XFF,0XFF,0XFE,0X7B,0XFF,0X72,
0XFE,0XFF,0XFC,0XE0,0XFC,0XE0,0XFF,0XF6,0XFF,0XFF,0XFC,0XEC,0XFC,0XE0,0XFE,0XEC,
0XFF,0XFF,0XFC,0XEC,0XFC,0XE0,0XFE,0XEC,0XFF,0XFF,0XFF,0X7F,0XFC,0XE7,0XFC,0XE0,
0XFE,0XEC,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFD,0X72,0XFC,0XE0,
0XFE,0XEC,0XFF,0XFF,0XFD,0XF6,0XFC,0XE0,0XFD,0X60,0XFF,0XFB,0XFE,0XFF,0XFC,0XE0,
0XFD,0X60,0XFF,0XFB,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFE,0XFF,0XFC,0XE0,
0XFE,0X67,0XFF,0XFF,0XFF,0XFF,0XFC,0XEC,0XFC,0XE0,0XFF,0XF6,0XFE,0X7B,0XFC,0XE0,
0XFC,0XE0,0XFF,0XF6,0XFF,0XFF,0XFE,0X7B,0XFC,0XE0,0XFC,0XE0,0XFF,0XF6,0XFE,0XFF,
0XFC,0XE0,0XFD,0X60,0XFF,0XFB,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFD,0X72,0XFC,0XE0,0XFF,0X72,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFE,0XFF,0XFC,0XE0,0XFC,0XE0,0XFC,0XE0,0XFC,0XE0,
0XFC,0XE0,0XFC,0XE0,0XFF,0XF6,0XFE,0X7B,0XFC,0XE0,0XFC,0XE0,0XFC,0XE0,0XFC,0XE0,
0XFC,0XE0,0XFC,0XE0,0XFF,0XF6,0XFF,0XFF,0XFC,0XEC,0XFC,0XE0,0XFF,0X72,0XFF,0XFF,
0XFC,0XEC,0XFC,0XE0,0XFF,0XF6,0XFF,0XFF,0XFF,0XFF,0XFD,0X72,0XFC,0XE0,0XFD,0X67,
0XFD,0XE0,0XFF,0XFF,0XFF,0XFF,0XFE,0X7B,0XFC,0XE0,0XFC,0XE0,0XFC,0XE0,0XFC,0XE0,
0XFC,0XE0,0XFC,0XE0,0XFF,0XF6,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFE,0XFF,0XFC,0XE0,0XFC,0XE0,0XFC,0XE0,0XFC,0XE0,0XFC,0XE0,0XFC,0XE0,0XFF,0X72,
0XFF,0X7F,0XFC,0XE7,0XFC,0XE0,0XFC,0XE0,0XFC,0XE0,0XFC,0XE0,0XFC,0XE0,0XFE,0XEC,
0XFF,0XFF,0XFC,0XEC,0XFC,0XE0,0XFC,0XE0,0XFC,0XE0,0XFC,0XE0,0XFC,0XE0,0XFC,0XE0,
0XFF,0XF6,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFD,0X72,0XFC,0XE0,
0XFE,0XEC,0XFF,0XFF,0XFF,0XFF,0XFC,0XEC,0XFC,0XE0,0XFE,0XEC,0XFF,0XFF,0XFC,0XEC,
0XFC,0XE0,0XFC,0XE0,0XFC,0XE0,0XFC,0XE0,0XFD,0X60,0XFF,0XFB,0XFE,0X7B,0XFC,0XE0,
0XFC,0XE0,0XFC,0XE0,0XFC,0XE0,0XFC,0XE0,0XFC,0XE0,0XFF,0XF6,0XFF,0X7F,0XFC,0XE7,
0XFC,0XE0,0XFC,0XE0,0XFC,0XE0,0XFC,0XE0,0XFC,0XE0,0XFC,0XE0,0XFF,0XF6,0XFF,0XFF,
0XFC,0XEC,0XFC,0XE0,0XFC,0XE0,0XFC,0XE0,0XFC,0XE0,0XFD,0X60,0XFF,0XFB,0XFF,0XFF,
0XFD,0X72,0XFC,0XE0,0XFF,0X72,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFC,0XEC,0XFC,0XE0,0XFC,0XE0,0XFC,0XE0,
0XFC,0XE0,0XFF,0XF6,0XFF,0XFF,0XFF,0XFF,0XFD,0X72,0XFC,0XE0,0XFC,0XE0,0XFE,0X67,
0XFC,0XEC,0XFC,0XE0,0XFF,0XF6,0XFF,0XFF,0XFC,0XEC,0XFC,0XE0,0XFF,0X72,0XFF,0XFF,
0XFC,0XEC,0XFC,0XE0,0XFF,0XF6,0XFF,0XFF,0XFF,0XFF,0XFE,0X7B,0XFC,0XE0,0XFC,0XE0,
0XFE,0XEC,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFD,0X72,0XFC,0XE0,0XFC,0XE0,0XFE,0X67,
0XFC,0XEC,0XFC,0XE0,0XFF,0XF6,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFC,0XEC,0XFC,0XE0,0XFC,0XE0,0XFC,0XE0,0XFD,0X60,0XFF,0XFB,
0XFF,0XFF,0XFE,0X7B,0XFC,0XE0,0XFC,0XE0,0XFE,0XEC,0XFD,0X72,0XFC,0XE0,0XFE,0XEC,
0XFF,0XFF,0XFC,0XEC,0XFC,0XE0,0XFF,0X76,0XFC,0XE7,0XFC,0XE0,0XFC,0XE0,0XFF,0XF6,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFD,0X72,0XFC,0XE0,
0XFE,0XEC,0XFF,0XFF,0XFF,0XFF,0XFE,0X7B,0XFC,0XE0,0XFC,0XE0,0XFF,0X72,0XFF,0XFF,
0XFD,0X72,0XFC,0XE0,0XFC,0XE0,0XFC,0XE0,0XFD,0X60,0XFF,0XFB,0XFF,0XFF,0XFD,0X72,
0XFC,0XE0,0XFC,0XE0,0XFE,0X67,0XFC,0XEC,0XFC,0XE0,0XFF,0XF6,0XFF,0XFF,0XFF,0X7F,
0XFC,0XE7,0XFC,0XE0,0XFC,0XE0,0XFF,0X76,0XFC,0XE7,0XFC,0XE0,0XFF,0XF6,0XFF,0XFF,
0XFF,0XFF,0XFD,0X72,0XFC,0XE0,0XFC,0XE0,0XFC,0XE0,0XFD,0X60,0XFF,0XFB,0XFF,0XFF,
0XFD,0X72,0XFC,0XE0,0XFF,0X72,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFC,0XEC,0XFC,0XE0,0XFF,0XF6,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFC,0XEC,0XFC,0XE0,0XFF,0XF6,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFC,0XEC,0XFC,0XE0,0XFF,0XF6,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFD,0X72,0XFC,0XE0,0XFC,0XE0,0XFC,0XE0,
0XFC,0XE0,0XFC,0XE0,0XFF,0XF6,0XFF,0XFF,0XFF,0XFF,0XFD,0X72,0XFC,0XE0,0XFF,0X72,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFD,0X72,0XFC,0XE0,0XFC,0XE0,0XFC,0XE0,0XFC,0XE0,
0XFF,0XF6,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFD,0XF6,0XFC,0XE0,0XFE,0X67,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFD,0XF6,0XFC,0XE0,0XFC,0XE0,0XFC,0XE0,0XFF,0XF6,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFD,0XF6,0XFC,0XE0,0XFC,0XE0,0XFC,0XE0,0XFF,0XF6,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFD,0X72,0XFC,0XE0,0XFC,0XE0,0XFC,0XE0,
0XFC,0XE0,0XFC,0XE0,0XFC,0XE0,0XFF,0XF6,0XFF,0XFF,0XFD,0X72,0XFC,0XE0,0XFF,0X72,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFC,0XEC,0XFC,0XE0,0XFC,0XE0,0XFC,0XE0,0XFC,0XE0,0XFC,0XE0,
0XFF,0XF6,0XFF,0XFF,0XFD,0X72,0XFC,0XE0,0XFC,0XE0,0XFC,0XE0,0XFE,0X67,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFD,0XF6,0XFC,0XE0,0XFC,0XE0,0XFC,0XE0,0XFC,0XE0,0XFD,0X60,
0XFF,0XFB,0XFF,0XFF,0XFD,0XF6,0XFC,0XE0,0XFC,0XE0,0XFC,0XE0,0XFC,0XE0,0XFD,0X60,
0XFF,0XFB,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFD,0X72,0XFC,0XE0,0XFF,0X72,0XFF,0XFF,
0XFF,0X7F,0XFC,0XE7,0XFC,0XE0,0XFF,0X72,0XFF,0XFF,0XFD,0X72,0XFC,0XE0,0XFF,0X72,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFE,0XFF,0XFC,0XE0,0XFC,0XE0,0XFF,0XF6,0XFF,0XFF,0XFF,0XFF,0XFD,0XF6,
0XFF,0XF6,0XFF,0XFF,0XFD,0X72,0XFD,0X60,0XFE,0X76,0XFC,0XE0,0XFE,0X67,0XFF,0XFF,
0XFF,0XFF,0XFF,0X7F,0XFC,0XE7,0XFC,0XE0,0XFF,0X72,0XFF,0XFF,0XFC,0XEC,0XFC,0XE0,
0XFF,0X72,0XFF,0X7F,0XFC,0XE7,0XFC,0XE0,0XFF,0X72,0XFF,0XFF,0XFC,0XEC,0XFC,0XE0,
0XFF,0X72,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFD,0X72,0XFC,0XE0,0XFF,0X72,0XFF,0XFF,
0XFE,0XFF,0XFC,0XE0,0XFC,0XE0,0XFF,0XF6,0XFF,0XFF,0XFD,0X72,0XFC,0XE0,0XFF,0X72,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFE,0XFF,0XFC,0XE0,0XFC,0XE0,0XFE,0X67,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFE,0X7B,0XFC,0XE0,0XFE,0X67,0XFF,0XFF,
0XFF,0XFF,0XFE,0XFF,0XFC,0XE0,0XFD,0XE0,0XFF,0XFF,0XFF,0XFF,0XFD,0XF6,0XFC,0XE0,
0XFE,0XEC,0XFE,0XFF,0XFC,0XE0,0XFD,0XE0,0XFF,0XFF,0XFF,0XFF,0XFD,0XF6,0XFC,0XE0,
0XFE,0XEC,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFD,0X72,0XFC,0XE0,0XFC,0XE0,0XFC,0XE0,
0XFC,0XE0,0XFC,0XE0,0XFE,0XEC,0XFF,0XFF,0XFF,0XFF,0XFD,0X72,0XFC,0XE0,0XFF,0X72,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFD,0XF6,0XFC,0XE0,0XFC,0XE0,0XFC,0XE0,0XFD,0X60,0XFF,0XFB,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFE,0X7B,0XFC,0XE0,0XFE,0X67,0XFF,0XFF,
0XFF,0XFF,0XFE,0X7B,0XFC,0XE0,0XFE,0X67,0XFF,0XFF,0XFF,0XFF,0XFE,0X7B,0XFC,0XE0,
0XFE,0X67,0XFE,0X7B,0XFC,0XE0,0XFE,0X67,0XFF,0XFF,0XFF,0XFF,0XFE,0X7B,0XFC,0XE0,
0XFE,0X67,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFD,0X72,0XFC,0XE0,0XFC,0XE0,0XFC,0XE0,
0XFC,0XE0,0XFE,0XEC,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFD,0X72,0XFC,0XE0,0XFF,0X72,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFE,0XFF,0XFC,0XE0,0XFC,0XE0,0XFC,0XE0,0XFC,0XE0,
0XFF,0XF6,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFE,0X7B,0XFC,0XE0,0XFE,0X67,0XFF,0XFF,
0XFF,0XFF,0XFE,0X7B,0XFC,0XE0,0XFE,0X67,0XFF,0XFF,0XFF,0XFF,0XFE,0X7B,0XFC,0XE0,
0XFE,0X67,0XFE,0X7B,0XFC,0XE0,0XFE,0X67,0XFF,0XFF,0XFF,0XFF,0XFE,0X7B,0XFC,0XE0,
0XFE,0X67,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFD,0X72,0XFC,0XE0,0XFE,0XEC,0XFF,0X7F,
0XFC,0XE7,0XFC,0XE0,0XFF,0X72,0XFF,0XFF,0XFF,0XFF,0XFD,0X72,0XFC,0XE0,0XFF,0X72,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFC,0XEC,0XFC,0XE0,0XFC,0XE0,0XFC,0XE0,
0XFF,0XF6,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFC,0XEC,0XFC,0XE0,
0XFE,0XEC,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFE,0X7B,0XFC,0XE0,0XFE,0X67,0XFF,0XFF,
0XFF,0XFF,0XFE,0X7B,0XFC,0XE0,0XFD,0XE0,0XFF,0XFF,0XFF,0XFF,0XFD,0XF6,0XFC,0XE0,
0XFE,0XEC,0XFE,0X7B,0XFC,0XE0,0XFD,0XE0,0XFF,0XFF,0XFF,0XFF,0XFD,0XF6,0XFC,0XE0,
0XFE,0XEC,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFD,0X72,0XFC,0XE0,0XFE,0XEC,0XFF,0XFF,
0XFD,0XF6,0XFC,0XE0,0XFD,0X60,0XFF,0XFB,0XFF,0XFF,0XFD,0X72,0XFC,0XE0,0XFF,0X72,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFE,0XFF,0XFE,0X67,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFD,0X72,0XFC,0XE0,
0XFE,0XEC,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFE,0X7B,0XFC,0XE0,0XFE,0X67,0XFF,0XFF,
0XFF,0XFF,0XFF,0X7F,0XFC,0XE7,0XFC,0XE0,0XFF,0XF6,0XFF,0XFF,0XFC,0XEC,0XFC,0XE0,
0XFF,0X72,0XFF,0X7F,0XFC,0XE7,0XFC,0XE0,0XFF,0XF6,0XFF,0XFF,0XFC,0XEC,0XFC,0XE0,
0XFF,0X72,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFD,0X72,0XFC,0XE0,0XFE,0XEC,0XFF,0XFF,
0XFF,0XFF,0XFC,0XEC,0XFC,0XE0,0XFE,0XEC,0XFF,0XFF,0XFD,0X72,0XFC,0XE0,0XFC,0XE0,
0XFC,0XE0,0XFC,0XE0,0XFD,0X60,0XFF,0XFB,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFE,0XFF,0XFC,0XE0,0XFC,0XE0,0XFC,0XE0,0XFC,0XE0,0XFC,0XE0,0XFC,0XE0,
0XFF,0XF6,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFE,0X7B,0XFC,0XE0,0XFE,0X67,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFD,0X72,0XFC,0XE0,0XFC,0XE0,0XFC,0XE0,0XFC,0XE0,0XFD,0XE0,
0XFF,0XFF,0XFF,0XFF,0XFD,0X72,0XFC,0XE0,0XFC,0XE0,0XFC,0XE0,0XFC,0XE0,0XFD,0XE0,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFD,0X72,0XFC,0XE0,0XFE,0XEC,0XFF,0XFF,
0XFF,0XFF,0XFE,0X7B,0XFC,0XE0,0XFC,0XE0,0XFF,0X72,0XFD,0X72,0XFC,0XE0,0XFC,0XE0,
0XFC,0XE0,0XFC,0XE0,0XFD,0X60,0XFF,0XFB,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFC,0XEC,0XFC,0XE0,0XFC,0XE0,0XFC,0XE0,0XFC,0XE0,0XFF,0XF6,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFE,0X7B,0XFC,0XE0,0XFE,0X67,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFD,0X72,0XFC,0XE0,0XFC,0XE0,0XFD,0X60,0XFF,0XFB,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFD,0X72,0XFC,0XE0,0XFC,0XE0,0XFD,0X60,0XFF,0XFB,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFD,0X72,0XFC,0XE0,0XFC,0XE0,0XFC,0XE0,0XFC,0XE0,0XFC,0XE0,0XFF,0XF6,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFC,0XEC,0XFC,0XE0,0XFF,0X72,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFD,0X72,
0XFC,0XE0,0XFC,0XE0,0XFC,0XE0,0XFC,0XE0,0XFC,0XE0,0XFC,0XE0,0XFD,0X60,0XFF,0XFB,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFC,0XEC,
0XFD,0X60,0XFF,0XFB,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFD,0X72,0XFC,0XE0,0XFC,0XE0,0XFC,0XE0,0XFC,0XE0,0XFC,0XE0,0XFC,0XE0,0XFF,0XF6,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFC,0XEC,0XFC,0XE0,0XFF,0X72,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFD,0X72,
0XFC,0XE0,0XFC,0XE0,0XFC,0XE0,0XFC,0XE0,0XFC,0XE0,0XFC,0XE0,0XFD,0X60,0XFF,0XFB,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFC,0XEC,
0XFD,0X60,0XFF,0XFB,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFD,0X72,0XFC,0XE0,0XFF,0X72,0XFF,0XFF,0XFF,0X7F,0XFC,0XE7,0XFC,0XE0,0XFF,0X72,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFC,0XEC,0XFC,0XE0,0XFF,0X72,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFC,0XEC,0XFC,0XE0,0XFF,0XF6,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFC,0XEC,
0XFD,0X60,0XFF,0XFB,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFD,0X72,0XFC,0XE0,0XFF,0X72,0XFF,0XFF,0XFE,0XFF,0XFC,0XE0,0XFC,0XE0,0XFF,0XF6,
0XFF,0XFF,0XFF,0XFF,0XFD,0XF6,0XFC,0XE0,0XFC,0XE0,0XFC,0XE0,0XFF,0X72,0XFF,0XFF,
0XFF,0XFF,0XFD,0XF6,0XFC,0XE0,0XFC,0XE0,0XFC,0XE0,0XFC,0XE0,0XFF,0XF6,0XFF,0XFF,
0XFF,0XFF,0XFC,0XEC,0XFC,0XE0,0XFF,0X72,0XFD,0X72,0XFC,0XE0,0XFE,0XEC,0XFF,0XFF,
0XFF,0XFF,0XFD,0X72,0XFC,0XE0,0XFF,0X72,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFC,0XEC,0XFC,0XE0,0XFF,0XF6,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFD,0XF6,0XFC,0XE0,0XFC,0XE0,0XFC,0XE0,0XFF,0X72,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFD,0XF6,0XFC,0XE0,0XFC,0XE0,0XFC,0XE0,0XFE,0XEC,0XFF,0XFF,0XFC,0XEC,
0XFD,0X60,0XFE,0XFB,0XFC,0XE0,0XFC,0XE0,0XFE,0X67,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFD,0X72,0XFC,0XE0,0XFC,0XE0,0XFC,0XE0,0XFC,0XE0,0XFC,0XE0,0XFE,0XEC,0XFF,0XFF,
0XFF,0XFF,0XFD,0X72,0XFC,0XE0,0XFC,0XE0,0XFC,0XE0,0XFC,0XE0,0XFC,0XE0,0XFF,0X72,
0XFF,0XFF,0XFD,0X72,0XFC,0XE0,0XFC,0XE0,0XFC,0XE0,0XFC,0XE0,0XFD,0X60,0XFF,0XFB,
0XFF,0XFF,0XFC,0XEC,0XFC,0XE0,0XFF,0X72,0XFE,0X7B,0XFC,0XE0,0XFD,0XE0,0XFF,0XFF,
0XFF,0X7F,0XFC,0XE7,0XFD,0X60,0XFF,0XFB,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFC,0XEC,0XFC,0XE0,0XFF,0XF6,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFD,0X72,0XFC,0XE0,0XFC,0XE0,0XFC,0XE0,0XFC,0XE0,0XFC,0XE0,0XFF,0X72,0XFF,0XFF,
0XFD,0X72,0XFC,0XE0,0XFC,0XE0,0XFC,0XE0,0XFC,0XE0,0XFE,0XEC,0XFF,0XFF,0XFC,0XEC,
0XFD,0X60,0XFC,0XE7,0XFC,0XE0,0XFC,0XE0,0XFC,0XE0,0XFF,0XF6,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFD,0X72,0XFC,0XE0,0XFC,0XE0,0XFC,0XE0,0XFC,0XE0,0XFE,0XEC,0XFF,0XFF,0XFF,0XFF,
0XFE,0XFF,0XFC,0XE0,0XFC,0XE0,0XFF,0XF6,0XFF,0XFF,0XFE,0X7B,0XFC,0XE0,0XFE,0X67,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFC,0XEC,0XFC,0XE0,0XFF,0XF6,
0XFF,0XFF,0XFC,0XEC,0XFC,0XE0,0XFF,0X72,0XFF,0XFF,0XFC,0XEC,0XFC,0XE0,0XFF,0XF6,
0XFE,0XFF,0XFC,0XE0,0XFE,0X67,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFC,0XEC,0XFC,0XE0,0XFF,0XF6,0XFF,0XFF,0XFF,0XFF,0XFE,0XFF,
0XFC,0XE0,0XFC,0XE0,0XFF,0XF6,0XFF,0XFF,0XFE,0X7B,0XFC,0XE0,0XFE,0X67,0XFE,0XFF,
0XFC,0XE0,0XFC,0XE0,0XFF,0XF6,0XFF,0XFF,0XFF,0X7F,0XFE,0XF2,0XFF,0XFF,0XFC,0XEC,
0XFC,0XE0,0XFF,0X72,0XFF,0XFF,0XFC,0XEC,0XFC,0XE0,0XFF,0X72,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFD,0X72,0XFC,0XE0,0XFE,0XEC,0XFF,0X7F,0XFC,0XE7,0XFC,0XE0,0XFF,0X72,0XFF,0XFF,
0XFE,0XFF,0XFC,0XE0,0XFC,0XE0,0XFC,0XE0,0XFC,0XE0,0XFC,0XE0,0XFC,0XE0,0XFE,0X67,
0XFF,0XFF,0XFD,0XF6,0XFC,0XE0,0XFC,0XE0,0XFC,0XE0,0XFC,0XE0,0XFC,0XE0,0XFF,0XF6,
0XFF,0XFF,0XFC,0XEC,0XFC,0XE0,0XFF,0X72,0XFF,0XFF,0XFD,0XF6,0XFC,0XE0,0XFE,0XEC,
0XFD,0XF6,0XFC,0XE0,0XFF,0X72,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFC,0XEC,0XFC,0XE0,0XFF,0XF6,0XFF,0XFF,0XFF,0XFF,0XFE,0XFF,
0XFC,0XE0,0XFC,0XE0,0XFC,0XE0,0XFC,0XE0,0XFC,0XE0,0XFC,0XE0,0XFE,0X67,0XFE,0X7B,
0XFC,0XE0,0XFD,0XE0,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFC,0XEC,
0XFD,0X60,0XFF,0XFB,0XFF,0XFF,0XFD,0XF6,0XFC,0XE0,0XFF,0X72,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFD,0X72,0XFC,0XE0,0XFE,0XEC,0XFF,0XFF,0XFD,0XF6,0XFC,0XE0,0XFD,0X60,0XFF,0XFB,
0XFE,0XFF,0XFC,0XE0,0XFD,0X60,0XFF,0XFB,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFE,0XFF,0XFC,0XE0,0XFE,0X67,0XFF,0XFF,0XFF,0XFF,0XFC,0XEC,0XFC,0XE0,0XFF,0XF6,
0XFF,0XFF,0XFC,0XEC,0XFC,0XE0,0XFF,0X72,0XFF,0XFF,0XFE,0XFF,0XFC,0XE0,0XFD,0XE0,
0XFC,0XEC,0XFD,0X60,0XFF,0XFB,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFC,0XEC,0XFC,0XE0,0XFF,0XF6,0XFF,0XFF,0XFF,0XFF,0XFE,0XFF,
0XFC,0XE0,0XFD,0X60,0XFF,0XFB,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFE,0XFF,
0XFC,0XE0,0XFC,0XE0,0XFF,0XF6,0XFF,0XFF,0XFF,0X7F,0XFE,0XF2,0XFF,0XFF,0XFC,0XEC,
0XFD,0X60,0XFF,0XFB,0XFF,0XFF,0XFD,0XF6,0XFC,0XE0,0XFF,0X72,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFD,0X72,0XFC,0XE0,0XFE,0XEC,0XFF,0XFF,0XFF,0XFF,0XFC,0XEC,0XFC,0XE0,0XFE,0XEC,
0XFF,0XFF,0XFC,0XEC,0XFC,0XE0,0XFC,0XE0,0XFC,0XE0,0XFC,0XE0,0XFD,0X60,0XFF,0XFB,
0XFE,0X7B,0XFC,0XE0,0XFC,0XE0,0XFC,0XE0,0XFC,0XE0,0XFC,0XE0,0XFC,0XE0,0XFF,0XF6,
0XFF,0XFF,0XFC,0XEC,0XFC,0XE0,0XFF,0X72,0XFF,0XFF,0XFF,0XFF,0XFD,0X72,0XFC,0XE0,
0XFC,0XE0,0XFE,0X67,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFC,0XEC,0XFC,0XE0,0XFF,0XF6,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFC,0XEC,0XFC,0XE0,0XFC,0XE0,0XFC,0XE0,0XFC,0XE0,0XFD,0X60,0XFF,0XFB,0XFF,0XFF,
0XFC,0XEC,0XFC,0XE0,0XFC,0XE0,0XFC,0XE0,0XFC,0XE0,0XFE,0XEC,0XFF,0XFF,0XFC,0XEC,
0XFD,0X60,0XFF,0XFB,0XFF,0XFF,0XFD,0XF6,0XFC,0XE0,0XFF,0X72,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFD,0X72,0XFC,0XE0,0XFE,0XEC,0XFF,0XFF,0XFF,0XFF,0XFE,0X7B,0XFC,0XE0,0XFC,0XE0,
0XFF,0X72,0XFF,0XFF,0XFD,0X72,0XFC,0XE0,0XFC,0XE0,0XFC,0XE0,0XFD,0X60,0XFF,0XFB,
0XFF,0XFF,0XFD,0X72,0XFC,0XE0,0XFC,0XE0,0XFE,0X67,0XFC,0XEC,0XFC,0XE0,0XFF,0XF6,
0XFF,0XFF,0XFC,0XEC,0XFC,0XE0,0XFF,0X72,0XFF,0XFF,0XFF,0XFF,0XFE,0X7B,0XFC,0XE0,
0XFC,0XE0,0XFF,0X72,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFC,0XEC,0XFC,0XE0,0XFF,0XF6,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFD,0X72,0XFC,0XE0,0XFC,0XE0,0XFC,0XE0,0XFD,0X60,0XFF,0XFB,0XFF,0XFF,
0XFF,0XFF,0XFD,0X72,0XFC,0XE0,0XFC,0XE0,0XFC,0XE0,0XFF,0X72,0XFF,0XFF,0XFC,0XEC,
0XFD,0X60,0XFF,0XFB,0XFF,0XFF,0XFD,0XF6,0XFC,0XE0,0XFF,0X72,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFE,0XFF,0XFC,0XE0,
0XFD,0X60,0XFF,0XFB,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFD,0X72,0XFC,0XE0,
0XFE,0X67,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFD,0XF6,0XFC,0XE0,0XFC,0XE0,0XFC,0XE0,
0XFF,0XF6,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFD,0XF6,0XFC,0XE0,0XFC,0XE0,0XFF,0X72,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
};

#line 13 "Management\\Interface\\Interface_main.c"
#line 1 ".\\Management\\SystemManage_RTC\\SystemManage_RTC.h"





 




 
#line 13 ".\\Management\\SystemManage_RTC\\SystemManage_RTC.h"
#line 14 ".\\Management\\SystemManage_RTC\\SystemManage_RTC.h"

 




 









 
typedef struct
{
	uint16 year;
	uint8 month;
	uint8 day;
	uint8 hour;
	uint8 min;
	uint8 sec;
} RTC_DATA;

 
 
extern RTC_DATA SystemManage_CurrentTime;
 
extern RTC_DATA SystemManage_RecordTime;
 
extern RTC_DATA SystemManage_UserSetTime;

#line 14 "Management\\Interface\\Interface_main.c"

 
uint8 UI_state = UI_STATE_MAIN_WINDOW;
uint16 UI_WindowBlocks = 0;
extern uint8 Page_Flag;

 
block_attr block_standard = {
		ENABLE,									 
		{
			0,   20,
			128, 160,
			0xFCE0
		},

		ENABLE,									 
		{
			gImage_PIC_Standard,
			12, 24,
			45, 45
		},

		ENABLE,									 
		{
			"Standard",
			0,  71,
			0xFFFF,0xF81F,
			0xFCE0
		},

		ENABLE,									 
		{
			63,  20,
			63,  160,
			0xDC40
		},
};

 
block_attr block_quick = {
		DISABLE,
		{0},

		ENABLE,									 
		{
			gImage_PIC_Quick,
			71,  24,
			45,  45
		},

		ENABLE,									 
		{
			"Quick",
			74,  71,
			0xFFFF,0xF81F,
			0xFCE0
		},

		ENABLE,									 
		{
			64,  20,
			64,  160,
			0xFD45
		},
};

 
block_attr block_record = {
		DISABLE,
		{0},

		ENABLE,									 
		{
			gImage_PIC_Record,
			12,  94,
			45, 45
		},

		ENABLE,									 
		{
			"Record",
			12,  141,
			0xFFFF,0xF81F,
			0xFCE0
		},

		ENABLE,									 
		{
			0,    90,
			128,  90,
			0xDC40
		},
};

 
block_attr block_settings = {
		DISABLE,
		{0},

		ENABLE,									 
		{
			gImage_PIC_Setting,
			71, 94,
			45, 45
		},

		ENABLE,									 
		{
			"Setting",
			68,  141,
			0xFFFF,0xF81F,
			0xFCE0
		},

		ENABLE,									 
		{
			0,    91,
			128,  91,
			0xFD45
		},
};

 
block_font_attr block_font = {
		ENABLE,									 
		{
			"Standard",
			0,  71,
			0xFFFF,0xF81F,
			0xFCE0
		},

		ENABLE,									 
		{
			"Quick",
			74,  71,
			0xFFFF,0xF81F,
			0xFCE0
		},

		ENABLE,									 
		{
			"Record",
			12,  141,
			0xFFFF,0xF81F,
			0xFCE0
		},

		ENABLE,									 
		{
			"Setting",
			68,  141,
			0xFFFF,0xF81F,
			0xFCE0
		},
};

 
block_attr* UI_WindowBlocksAttrArray_Main[] = {		 
		&block_standard,
		&block_quick,
		&block_record,
		&block_settings,
};
block_font_attr* UI_WindowBlocksAttrArray_Main_font[] = {
		&block_font,
};

 
void UI_Draw_Block(block_attr* block);
void UI_Draw_Block_font(block_font_attr* block);

 
uint8 Interface_Process(uint16* KeyCode)
{
	 
	static uint8 (* const UI_stateMachine[UI_STATE_MAX_STATE_NUM])(uint16) =
	{
			Interface_Main,					 
			Interface_Key_Event,			 
			Interface_Main_font,			 
			Interface_Standard,				 
			Interface_Quick,				 
			Interface_Record,				 
			Interface_Setting,				 
			Interface_Start,				 
			Interface_Testing,				 
			Interface_Quick_font,			 
			Interface_Result,				 
			Interface_Result_2,				 
			Interface_Insert_Cup,			 
	};
	uint8 state;
	do										 
	{
		if (UI_state < UI_STATE_MAX_STATE_NUM)
		{
			state = UI_stateMachine[UI_state](*KeyCode);
		}
		*KeyCode = 0;	 
	} while(state & (1u));

	return (1u);
}

 
uint8 Interface_Main(uint16 KeyCode)
{
	Exti_lock = DISABLE;
	key_state_confirm = 0;
	QRCode_Trigger_Disabled();
	UI_WindowBlocks = sizeof(UI_WindowBlocksAttrArray_Main) >> 2;
	UI_Draw_Window(UI_WindowBlocks);
	UI_state = UI_STATE_MAIN_FONT;
	return (1u);
}

 
void UI_Draw_Window(uint16 blockNum)
{
	uint8 blockIndex = 0;						 
	for (blockIndex = 0; blockIndex < blockNum; blockIndex++)
	{
		UI_Draw_Block(UI_WindowBlocksAttrArray_Main[blockIndex]);
	}
}

 
void UI_Draw_Block(block_attr* block)
{
	Display_Time = 0;
	if (block->rect_enabled)					 
	{
		Lcd_ColorBox(block->rect_attr.startX, block->rect_attr.startY,
				block->rect_attr.width, block->rect_attr.height,
				block->rect_attr.color);
	}

	if (block->pic_enabled)						 
	{
		DisplayDriver_DrawPic(block->pic_attr.offsetX,
				block->pic_attr.offsetY, block->pic_attr.width,
				block->pic_attr.height,block->pic_attr.src);
	}

	if (block->line_enabled)					 
	{
		DisplayDriver_DrawLine(block->Parting_line_attr.startX,
				block->Parting_line_attr.startY,
				block->Parting_line_attr.endX,
				block->Parting_line_attr.endY,
				block->Parting_line_attr.color);
	}

	if (block->char_enabled)					 
	{
			DisplayDriver_Text16_B(
					block->char_attr.offsetX,block->char_attr.offsetY,
					block->char_attr.color,block->char_attr.faceColor,
					block->char_attr.str);
	}
	Display_Time = 1;
}

 
uint8 Interface_Key_Event(uint16 KeyCode)
{
	switch(Interface_Key)
	{
		case 0:
			switch(key_state_confirm)
			{
				case 0:
					UI_state = UI_STATE_MAIN_FONT;
				break;

				case 1:
					UI_state = Key_control + 2;
					key_state_confirm = DISABLE;
					key_state = 1;
					Key_control = 1;
					Interface_Key = ENABLE;
				break;

				default:
				break;
			}
		break;

		case 1:
			switch(key_state_confirm)
			{
				case 1:
					UI_state = UI_STATE_TESTING;
					Key_control = 1;
				break;

				default:
				break;
			}
		break;

		case 2:
			switch(key_state_confirm)
			{
				case 0:
					if(Key_control == 1 && Page_Flag)
					{
						UI_state = UI_STATE_RESULT;
					}
					if(Key_control == 2 && (!Page_Flag))
					{
						UI_state = UI_STATE_RESULT_2;
					}
				break;

				case 1:
					UI_state = UI_STATE_MAIN_WINDOW;
					Interface_Key = DISABLE;
					Key_control = 1;
					key_state = 1;
					key_state_confirm = DISABLE;
				break;

				default:
				break;
			}
		break;
		case 3:
			switch(key_state_confirm)
			{
			case 0:
				if(Key_control == 1)
				{
					UI_state = UI_STATE_MAIN_WINDOW;
					Interface_Key = DISABLE;
					Key_control = 1;
					key_state = 1;
					key_state_confirm = DISABLE;
				}
			break;
			}
		break;

		default:
		break;
	}
}

 
uint8 Interface_Main_font(uint16 KeyCode)
{
	Exti_lock = DISABLE;
	UI_WindowBlocks = 0;
	UI_WindowBlocks = sizeof(UI_WindowBlocksAttrArray_Main_font) >> 2;
	UI_Draw_Window_font(UI_WindowBlocks);
	Exti_lock = ENABLE;
	while(!key_state);
	UI_state = UI_STATE_KEY_STATE;
	return (1u);
}

 
void UI_Draw_Window_font(uint16 blockNum)
{
	uint8 blockIndex = 0;
	if(key_state)								 
	{
		for (blockIndex = 0; blockIndex < blockNum; blockIndex++)
		{
			UI_Draw_Block_font(UI_WindowBlocksAttrArray_Main_font[blockIndex]);
		}
	}
}

 
void UI_Draw_Block_font(block_font_attr* block)
{
	Display_Time = 0;
	if (block->char1_enabled)				 
	{
		if(Key_control == 1)
		{
			DisplayDriver_Text16_B(
					block->char1_attr.offsetX,block->char1_attr.offsetY,
					block->char1_attr.color,block->char1_attr.backColor,
					block->char1_attr.str);
		}
		else
		{
			DisplayDriver_Text16_B(
					block->char1_attr.offsetX,block->char1_attr.offsetY,
					block->char1_attr.color,block->char1_attr.faceColor,
					block->char1_attr.str);
		}
	}

	if (block->char2_enabled)				 
	{
		if(Key_control == 2)
		{
			DisplayDriver_Text16_B(
					block->char2_attr.offsetX,block->char2_attr.offsetY,
					block->char2_attr.color,block->char2_attr.backColor,
					block->char2_attr.str);
		}
		else
		{
			DisplayDriver_Text16_B(
					block->char2_attr.offsetX,block->char2_attr.offsetY,
					block->char2_attr.color,block->char2_attr.faceColor,
					block->char2_attr.str);
		}
	}

	if (block->char3_enabled)				 
	{
		if(Key_control == 3)
		{
			DisplayDriver_Text16_B(
					block->char3_attr.offsetX,block->char3_attr.offsetY,
					block->char3_attr.color,block->char3_attr.backColor,
					block->char3_attr.str);
		}
		else
		{
			DisplayDriver_Text16_B(
					block->char3_attr.offsetX,block->char3_attr.offsetY,
					block->char3_attr.color,block->char3_attr.faceColor,
					block->char3_attr.str);
		}
	}

	if (block->char4_enabled)				 
	{
		if(Key_control == 4)
		{
			DisplayDriver_Text16_B(
					block->char4_attr.offsetX,block->char4_attr.offsetY,
					block->char4_attr.color,block->char4_attr.backColor,
					block->char4_attr.str);
		}
		else
		{
			DisplayDriver_Text16_B(
					block->char4_attr.offsetX,block->char4_attr.offsetY,
					block->char4_attr.color,block->char4_attr.faceColor,
					block->char4_attr.str);
		}
	}
	key_state = DISABLE;
	Display_Time = 1;
}

 
void UI_Draw_Status_Bar(void)					 
{
	char tbuf[10] = {0};
	PCF8563_Read(&SystemManage_CurrentTime);
	sprintf((char*)tbuf,"%02d:%02d:%02d",SystemManage_CurrentTime.hour,
			SystemManage_CurrentTime.min,SystemManage_CurrentTime.sec);
	DisplayDriver_Text16_B(4,2,0xFFFF,0xC2E0,tbuf);
}

 
void Battery_Empty_ICO(void)
{
	int i = 0;
	Display_Time = 0;
	DisplayDriver_DrawLine(101,5,120,5,0xFFFF);
	DisplayDriver_DrawLine(101,6,120,6,0xFFFF);

	DisplayDriver_DrawLine(101,5,101,16,0xFFFF);
	DisplayDriver_DrawLine(102,5,102,16,0xFFFF);

	DisplayDriver_DrawLine(101,16,120,16,0xFFFF);
	DisplayDriver_DrawLine(101,15,120,15,0xFFFF);

	DisplayDriver_DrawLine(120,16,120,12,0xFFFF);
	DisplayDriver_DrawLine(121,15,121,12,0xFFFF);
	DisplayDriver_DrawLine(120,6,120,9,0xFFFF);
	DisplayDriver_DrawLine(121,6,121,9,0xFFFF);

	DisplayDriver_DrawLine(122,8,123,8,0xFFFF);
	DisplayDriver_DrawLine(122,9,123,9,0xFFFF);
	DisplayDriver_DrawLine(122,10,123,10,0xFFFF);
	DisplayDriver_DrawLine(122,11,123,11,0xFFFF);
	DisplayDriver_DrawLine(122,12,123,12,0xFFFF);
	DisplayDriver_DrawLine(122,13,123,13,0xFFFF);
	for(i= 104;i<120;)
	{
		Lcd_ColorBox(i,8,3,6,0xFFFF);
		i += 4;
	}
	Display_Time = 1;
}
