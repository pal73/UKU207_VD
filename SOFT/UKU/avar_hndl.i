#line 1 "avar_hndl.c"
#line 1 "avar_hndl.h"




extern unsigned avar_stat;	 	
extern unsigned avar_ind_stat; 	
extern unsigned avar_stat_old;
extern unsigned avar_stat_new,avar_stat_offed;









void avar_hndl(void);
void avar_unet_hndl(char in);
void avar_uout_hndl(char in);
void reload_hndl(void);
void avar_bps_hndl(char bps, char v, char in);
void avar_bat_hndl(char bat, char in);
void avar_bat_as_hndl(char b, char in);
void ke_mem_hndl(char b,unsigned short in);
void vz_mem_hndl(unsigned short in);
void wrk_mem_hndl(char b);
void avar_bat_ips_hndl(char in);



#line 2 "avar_hndl.c"
#line 1 "eeprom_map.h"






#line 35 "eeprom_map.h"



#line 161 "eeprom_map.h"

#line 172 "eeprom_map.h"






 









 

#line 204 "eeprom_map.h"



#line 216 "eeprom_map.h"


#line 227 "eeprom_map.h"



#line 238 "eeprom_map.h"



#line 294 "eeprom_map.h"


#line 337 "eeprom_map.h"








#line 359 "eeprom_map.h"







































































































extern const unsigned short ADR_EE_BAT_ZAR_CNT[2];
extern const unsigned short ADR_EE_BAT_ZAR_CNT_KE[2];
extern const unsigned short ADR_EE_BAT_C_NOM[2];
extern const unsigned short ADR_EE_BAT_YEAR_OF_ON[2];
extern const unsigned short ADR_EE_BAT_IS_ON[2];
extern const unsigned short ADR_EE_BAT_DAY_OF_ON[2];
extern const unsigned short ADR_EE_BAT_MONTH_OF_ON[2];
extern const unsigned short ADR_EE_BAT_RESURS[2];
extern const unsigned short ADR_EE_BAT_C_REAL[2];
extern const unsigned short ADR_EE_BAT_TYPE[2];
extern const unsigned short ADR_KUBAT[2];
extern const unsigned short ADR_KUBATM[2];
extern const unsigned short ADR_KI0BAT[2];
extern const unsigned short ADR_KI1BAT[2];
extern const unsigned short ADR_KTBAT[2];
extern const unsigned short ADR_EE_BAT_TYPE[2];


extern const unsigned short ADR_TMAX_EXT_EN[3];
extern const unsigned short ADR_TMAX_EXT[3];
extern const unsigned short ADR_TMIN_EXT_EN[3];
extern const unsigned short ADR_TMIN_EXT[3];
extern const unsigned short ADR_T_EXT_REL_EN[3];
extern const unsigned short ADR_T_EXT_ZVUK_EN[3];
extern const unsigned short ADR_T_EXT_LCD_EN[3];
extern const unsigned short ADR_T_EXT_RS_EN[3];

extern const unsigned short ADR_SK_SIGN[4];
extern const unsigned short ADR_SK_REL_EN[4];
extern const unsigned short ADR_SK_ZVUK_EN[4];
extern const unsigned short ADR_SK_LCD_EN[4];
extern const unsigned short ADR_SK_RS_EN[4];

extern const unsigned short ADR_EE_RELE_SET_MASK[4];

#line 3 "avar_hndl.c"
#line 1 "full_can.h"


  








#line 19 "full_can.h"



  
















extern char ptr_can1_tx_wr,ptr_can1_tx_rd;
extern long can1_info[8];
extern long can1_id[8];
extern long can1_data[8];
extern long can1_datb[8];
																							 
extern char ptr_can2_tx_wr,ptr_can2_tx_rd;

extern long can2_info[8];
extern long can2_id[8];
extern long can2_data[8];
extern long can2_datb[8];

extern unsigned short rotor_can[6];



extern char bR;
extern char RXBUFF[40],TXBUFF[40];
extern char bIN,bIN2;
extern char bd_dumm[25];
extern char bd[25];
extern char TX_len;

extern char RXBUFF2[40],TXBUFF2[40];
extern char can_tx_cnt;
extern char can_tx_cnt2;


extern char rotor_rotor_rotor[2];
extern char can_tx_cnt;

extern const char Table87[];
extern const char Table95[];

extern char can_debug_plazma[2][10];
extern char bOUT_FREE;
extern char can_rotor[10];
extern char plazma_can;
extern char plazma_can1,plazma_can2,plazma_can3,plazma_can4;



typedef struct
{
  unsigned int Dat1; 
                     
                     
                     
  unsigned int DatA; 
  unsigned int DatB; 
} FULLCAN_MSG; 
extern short volatile gCANFilter;
extern FULLCAN_MSG volatile gFullCANList[2];
extern char can_reset_cnt;



char CRC1_in(void);
char CRC2_in(void);
char CRC1_out(void);
char CRC2_out(void);
void can1_out_adr(char* ptr,char num);
__irq void can_isr_err (void);
void can1_out(char data0,char data1,char data2,char data3,char data4,char data5,char data6,char data7);
void can_adr_hndl(void);
void can_in_an(void);
void can_in_an2(void);
__irq void can_isr_rx (void); 
__irq void can_isr_tx (void); 
short can1_init ( unsigned int can_btr);
short can2_init ( unsigned int can_btr);
short FullCAN_SetFilter (
  unsigned short can_port, 
  unsigned int CANID 
  );

void CAN_IRQHandler(void);
void CAN_ISR_Rx1( void );

extern char can_debug_plazma[2][10];
extern char ccc_plazma[20];

#line 4 "avar_hndl.c"
#line 1 "25lc640.h"












char spi1(char in);
void spi1_config(void);
void spi1_config_mcp2515(void);
void spi1_unconfig(void);
void lc640_wren(void);
char lc640_rdsr(void);
int lc640_read(int ADR);
int lc640_read_int(int ADR);
long lc640_read_long(int ADR);
void lc640_read_long_ptr(int ADR,char* out_ptr);
void lc640_read_str(int ADR, char* ram_ptr, char num);
char lc640_write(int ADR,char in);
char lc640_write_int(short ADR,short in);
char lc640_write_long(int ADR,long in);
char lc640_write_long_ptr(int ADR,char* in);
#line 5 "avar_hndl.c"
#line 1 "C:\\Keil\\ARM\\INC\\NXP\\LPC17xx\\LPC17xx.H"




















 









 

typedef enum IRQn
{
 
  NonMaskableInt_IRQn           = -14,       
  MemoryManagement_IRQn         = -12,       
  BusFault_IRQn                 = -11,       
  UsageFault_IRQn               = -10,       
  SVCall_IRQn                   = -5,        
  DebugMonitor_IRQn             = -4,        
  PendSV_IRQn                   = -2,        
  SysTick_IRQn                  = -1,        

 
  WDT_IRQn                      = 0,         
  TIMER0_IRQn                   = 1,         
  TIMER1_IRQn                   = 2,         
  TIMER2_IRQn                   = 3,         
  TIMER3_IRQn                   = 4,         
  UART0_IRQn                    = 5,         
  UART1_IRQn                    = 6,         
  UART2_IRQn                    = 7,         
  UART3_IRQn                    = 8,         
  PWM1_IRQn                     = 9,         
  I2C0_IRQn                     = 10,        
  I2C1_IRQn                     = 11,        
  I2C2_IRQn                     = 12,        
  SPI_IRQn                      = 13,        
  SSP0_IRQn                     = 14,        
  SSP1_IRQn                     = 15,        
  PLL0_IRQn                     = 16,        
  RTC_IRQn                      = 17,        
  EINT0_IRQn                    = 18,        
  EINT1_IRQn                    = 19,        
  EINT2_IRQn                    = 20,        
  EINT3_IRQn                    = 21,        
  ADC_IRQn                      = 22,        
  BOD_IRQn                      = 23,        
  USB_IRQn                      = 24,        
  CAN_IRQn                      = 25,        
  DMA_IRQn                      = 26,        
  I2S_IRQn                      = 27,        
  ENET_IRQn                     = 28,        
  RIT_IRQn                      = 29,        
  MCPWM_IRQn                    = 30,        
  QEI_IRQn                      = 31,        
  PLL1_IRQn                     = 32,        
  USBActivity_IRQn              = 33,        
  CANActivity_IRQn              = 34,        
} IRQn_Type;






 

 





#line 1 "C:\\Keil\\ARM\\RV31\\INC\\core_cm3.h"



















 




















































 

 
 
 
 
 
 
 
 


#line 1 "C:\\Keil\\ARM\\RV31\\INC\\stdint.h"
 
 





 









#line 25 "C:\\Keil\\ARM\\RV31\\INC\\stdint.h"







 

     

     
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




     

     





     





     





     

     





     





     





     

     





     





     





     

     


     


     


     

     


     


     


     

     



     



     


     
    
 



#line 196 "C:\\Keil\\ARM\\RV31\\INC\\stdint.h"

     







     










     











#line 260 "C:\\Keil\\ARM\\RV31\\INC\\stdint.h"



 


#line 86 "C:\\Keil\\ARM\\RV31\\INC\\core_cm3.h"

















 

#line 112 "C:\\Keil\\ARM\\RV31\\INC\\core_cm3.h"





 


 





 






 
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


 
#line 274 "C:\\Keil\\ARM\\RV31\\INC\\core_cm3.h"

#line 281 "C:\\Keil\\ARM\\RV31\\INC\\core_cm3.h"










 






#line 311 "C:\\Keil\\ARM\\RV31\\INC\\core_cm3.h"


 


 




#line 336 "C:\\Keil\\ARM\\RV31\\INC\\core_cm3.h"


 
 
 
 









 
extern uint32_t __get_PSP(void);









 
extern void __set_PSP(uint32_t topOfProcStack);









 
extern uint32_t __get_MSP(void);









 
extern void __set_MSP(uint32_t topOfMainStack);








 
extern uint32_t __REV16(uint16_t value);








 
extern int32_t __REVSH(int16_t value);


#line 502 "C:\\Keil\\ARM\\RV31\\INC\\core_cm3.h"









 









 
static __inline uint32_t  __get_BASEPRI(void)
{
  register uint32_t __regBasePri         __asm("basepri");
  return(__regBasePri);
}








 
static __inline void __set_BASEPRI(uint32_t basePri)
{
  register uint32_t __regBasePri         __asm("basepri");
  __regBasePri = (basePri & 0x1ff);
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





#line 1044 "C:\\Keil\\ARM\\RV31\\INC\\core_cm3.h"



 













 
static __inline void NVIC_SetPriorityGrouping(uint32_t PriorityGroup)
{
  uint32_t reg_value;
  uint32_t PriorityGroupTmp = (PriorityGroup & 0x07);                          
  
  reg_value  = ((SCB_Type *) ((0xE000E000) + 0x0D00))->AIRCR;                                                     
  reg_value &= ~((0xFFFFU << 16) | (0x0F << 8));                               
  reg_value  = ((reg_value | (0x5FA << 16) | (PriorityGroupTmp << 8)));   
  ((SCB_Type *) ((0xE000E000) + 0x0D00))->AIRCR = reg_value;
}









 
static __inline uint32_t NVIC_GetPriorityGrouping(void)
{
  return ((((SCB_Type *) ((0xE000E000) + 0x0D00))->AIRCR >> 8) & 0x07);                                           
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
    ((SCB_Type *) ((0xE000E000) + 0x0D00))->SHP[((uint32_t)(IRQn) & 0xF)-4] = ((priority << (8 - 5)) & 0xff); }  
  else {
    ((NVIC_Type *) ((0xE000E000) + 0x0100))->IP[(uint32_t)(IRQn)] = ((priority << (8 - 5)) & 0xff);    }         
}















 
static __inline uint32_t NVIC_GetPriority(IRQn_Type IRQn)
{

  if(IRQn < 0) {
    return((uint32_t)(((SCB_Type *) ((0xE000E000) + 0x0D00))->SHP[((uint32_t)(IRQn) & 0xF)-4] >> (8 - 5)));  }  
  else {
    return((uint32_t)(((NVIC_Type *) ((0xE000E000) + 0x0100))->IP[(uint32_t)(IRQn)]           >> (8 - 5)));  }  
}
















 
static __inline uint32_t NVIC_EncodePriority (uint32_t PriorityGroup, uint32_t PreemptPriority, uint32_t SubPriority)
{
  uint32_t PriorityGroupTmp = (PriorityGroup & 0x07);                          
  uint32_t PreemptPriorityBits;
  uint32_t SubPriorityBits;

  PreemptPriorityBits = ((7 - PriorityGroupTmp) > 5) ? 5 : 7 - PriorityGroupTmp;
  SubPriorityBits     = ((PriorityGroupTmp + 5) < 7) ? 0 : PriorityGroupTmp - 7 + 5;
 
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

  PreemptPriorityBits = ((7 - PriorityGroupTmp) > 5) ? 5 : 7 - PriorityGroupTmp;
  SubPriorityBits     = ((PriorityGroupTmp + 5) < 7) ? 0 : PriorityGroupTmp - 7 + 5;
  
  *pPreemptPriority = (Priority >> SubPriorityBits) & ((1 << (PreemptPriorityBits)) - 1);
  *pSubPriority     = (Priority                   ) & ((1 << (SubPriorityBits    )) - 1);
}



 



 














 
static __inline uint32_t SysTick_Config(uint32_t ticks)
{ 
  if (ticks > ((1<<24) -1))  return (1);                                              

  ((SysTick_Type *) ((0xE000E000) + 0x0010))->LOAD  =  (ticks & ((1<<24) -1)) - 1;                                       
  NVIC_SetPriority (SysTick_IRQn, (1<<5) - 1);                             
  ((SysTick_Type *) ((0xE000E000) + 0x0010))->VAL   =  (0x00);                                                               
  ((SysTick_Type *) ((0xE000E000) + 0x0010))->CTRL = (1 << 2) | (1<<0) | (1<<1);  
  return (0);                                                                             
}







 








 
static __inline void NVIC_SystemReset(void)
{
  ((SCB_Type *) ((0xE000E000) + 0x0D00))->AIRCR  = ((0x5FA << 16) | (((SCB_Type *) ((0xE000E000) + 0x0D00))->AIRCR & (0x700)) | (1<<2));  
  __dsb(0);                                                                                            
  while(1);                                                                             
}


 











 
static __inline uint32_t ITM_SendChar (uint32_t ch)
{
  if (ch == '\n') ITM_SendChar('\r');
  
  if ((((CoreDebug_Type *) (0xE000EDF0))->DEMCR & (1 << 24))  &&
      (((ITM_Type *) (0xE0000000))->TCR & 1)                  &&
      (((ITM_Type *) (0xE0000000))->TER & (1UL << 0))  ) 
  {
    while (((ITM_Type *) (0xE0000000))->PORT[0].u32 == 0);
    ((ITM_Type *) (0xE0000000))->PORT[0].u8 = (uint8_t) ch;
  }  
  return (ch);
}







 
#line 97 "C:\\Keil\\ARM\\INC\\NXP\\LPC17xx\\LPC17xx.H"
#line 1 "C:\\Keil\\ARM\\INC\\NXP\\LPC17xx\\system_LPC17xx.h"




















 









extern uint32_t SystemFrequency;     










 
extern void SystemInit (void);





#line 98 "C:\\Keil\\ARM\\INC\\NXP\\LPC17xx\\LPC17xx.H"


 
 
 


#pragma anon_unions


 
typedef struct
{
  volatile uint32_t FLASHCFG;                
       uint32_t RESERVED0[31];
  volatile uint32_t PLL0CON;                 
  volatile uint32_t PLL0CFG;
  volatile const  uint32_t PLL0STAT;
  volatile  uint32_t PLL0FEED;
       uint32_t RESERVED1[4];
  volatile uint32_t PLL1CON;
  volatile uint32_t PLL1CFG;
  volatile const  uint32_t PLL1STAT;
  volatile  uint32_t PLL1FEED;
       uint32_t RESERVED2[4];
  volatile uint32_t PCON;
  volatile uint32_t PCONP;
       uint32_t RESERVED3[15];
  volatile uint32_t CCLKCFG;
  volatile uint32_t USBCLKCFG;
  volatile uint32_t CLKSRCSEL;
  volatile uint32_t	CANSLEEPCLR;
  volatile uint32_t	CANWAKEFLAGS;
       uint32_t RESERVED4[10];
  volatile uint32_t EXTINT;                  
       uint32_t RESERVED5;
  volatile uint32_t EXTMODE;
  volatile uint32_t EXTPOLAR;
       uint32_t RESERVED6[12];
  volatile uint32_t RSID;                    
       uint32_t RESERVED7[7];
  volatile uint32_t SCS;                     
  volatile uint32_t IRCTRIM;                 
  volatile uint32_t PCLKSEL0;
  volatile uint32_t PCLKSEL1;
       uint32_t RESERVED8[4];
  volatile uint32_t USBIntSt;                
  volatile uint32_t DMAREQSEL;
  volatile uint32_t CLKOUTCFG;               
 } LPC_SC_TypeDef;

 
typedef struct
{
  volatile uint32_t PINSEL0;
  volatile uint32_t PINSEL1;
  volatile uint32_t PINSEL2;
  volatile uint32_t PINSEL3;
  volatile uint32_t PINSEL4;
  volatile uint32_t PINSEL5;
  volatile uint32_t PINSEL6;
  volatile uint32_t PINSEL7;
  volatile uint32_t PINSEL8;
  volatile uint32_t PINSEL9;
  volatile uint32_t PINSEL10;
       uint32_t RESERVED0[5];
  volatile uint32_t PINMODE0;
  volatile uint32_t PINMODE1;
  volatile uint32_t PINMODE2;
  volatile uint32_t PINMODE3;
  volatile uint32_t PINMODE4;
  volatile uint32_t PINMODE5;
  volatile uint32_t PINMODE6;
  volatile uint32_t PINMODE7;
  volatile uint32_t PINMODE8;
  volatile uint32_t PINMODE9;
  volatile uint32_t PINMODE_OD0;
  volatile uint32_t PINMODE_OD1;
  volatile uint32_t PINMODE_OD2;
  volatile uint32_t PINMODE_OD3;
  volatile uint32_t PINMODE_OD4;
  volatile uint32_t I2CPADCFG;
} LPC_PINCON_TypeDef;

 
typedef struct
{
  union {
    volatile uint32_t FIODIR;
    struct {
      volatile uint16_t FIODIRL;
      volatile uint16_t FIODIRH;
    };
    struct {
      volatile uint8_t  FIODIR0;
      volatile uint8_t  FIODIR1;
      volatile uint8_t  FIODIR2;
      volatile uint8_t  FIODIR3;
    };
  };
  uint32_t RESERVED0[3];
  union {
    volatile uint32_t FIOMASK;
    struct {
      volatile uint16_t FIOMASKL;
      volatile uint16_t FIOMASKH;
    };
    struct {
      volatile uint8_t  FIOMASK0;
      volatile uint8_t  FIOMASK1;
      volatile uint8_t  FIOMASK2;
      volatile uint8_t  FIOMASK3;
    };
  };
  union {
    volatile uint32_t FIOPIN;
    struct {
      volatile uint16_t FIOPINL;
      volatile uint16_t FIOPINH;
    };
    struct {
      volatile uint8_t  FIOPIN0;
      volatile uint8_t  FIOPIN1;
      volatile uint8_t  FIOPIN2;
      volatile uint8_t  FIOPIN3;
    };
  };
  union {
    volatile uint32_t FIOSET;
    struct {
      volatile uint16_t FIOSETL;
      volatile uint16_t FIOSETH;
    };
    struct {
      volatile uint8_t  FIOSET0;
      volatile uint8_t  FIOSET1;
      volatile uint8_t  FIOSET2;
      volatile uint8_t  FIOSET3;
    };
  };
  union {
    volatile  uint32_t FIOCLR;
    struct {
      volatile  uint16_t FIOCLRL;
      volatile  uint16_t FIOCLRH;
    };
    struct {
      volatile  uint8_t  FIOCLR0;
      volatile  uint8_t  FIOCLR1;
      volatile  uint8_t  FIOCLR2;
      volatile  uint8_t  FIOCLR3;
    };
  };
} LPC_GPIO_TypeDef;

typedef struct
{
  volatile const  uint32_t IntStatus;
  volatile const  uint32_t IO0IntStatR;
  volatile const  uint32_t IO0IntStatF;
  volatile  uint32_t IO0IntClr;
  volatile uint32_t IO0IntEnR;
  volatile uint32_t IO0IntEnF;
       uint32_t RESERVED0[3];
  volatile const  uint32_t IO2IntStatR;
  volatile const  uint32_t IO2IntStatF;
  volatile  uint32_t IO2IntClr;
  volatile uint32_t IO2IntEnR;
  volatile uint32_t IO2IntEnF;
} LPC_GPIOINT_TypeDef;

 
typedef struct
{
  volatile uint32_t IR;
  volatile uint32_t TCR;
  volatile uint32_t TC;
  volatile uint32_t PR;
  volatile uint32_t PC;
  volatile uint32_t MCR;
  volatile uint32_t MR0;
  volatile uint32_t MR1;
  volatile uint32_t MR2;
  volatile uint32_t MR3;
  volatile uint32_t CCR;
  volatile const  uint32_t CR0;
  volatile const  uint32_t CR1;
       uint32_t RESERVED0[2];
  volatile uint32_t EMR;
       uint32_t RESERVED1[12];
  volatile uint32_t CTCR;
} LPC_TIM_TypeDef;

 
typedef struct
{
  volatile uint32_t IR;
  volatile uint32_t TCR;
  volatile uint32_t TC;
  volatile uint32_t PR;
  volatile uint32_t PC;
  volatile uint32_t MCR;
  volatile uint32_t MR0;
  volatile uint32_t MR1;
  volatile uint32_t MR2;
  volatile uint32_t MR3;
  volatile uint32_t CCR;
  volatile const  uint32_t CR0;
  volatile const  uint32_t CR1;
  volatile const  uint32_t CR2;
  volatile const  uint32_t CR3;
       uint32_t RESERVED0;
  volatile uint32_t MR4;
  volatile uint32_t MR5;
  volatile uint32_t MR6;
  volatile uint32_t PCR;
  volatile uint32_t LER;
       uint32_t RESERVED1[7];
  volatile uint32_t CTCR;
} LPC_PWM_TypeDef;

 
typedef struct
{
  union {
  volatile const  uint8_t  RBR;
  volatile  uint8_t  THR;
  volatile uint8_t  DLL;
       uint32_t RESERVED0;
  };
  union {
  volatile uint8_t  DLM;
  volatile uint32_t IER;
  };
  union {
  volatile const  uint32_t IIR;
  volatile  uint8_t  FCR;
  };
  volatile uint8_t  LCR;
       uint8_t  RESERVED1[7];
  volatile const  uint8_t  LSR;
       uint8_t  RESERVED2[7];
  volatile uint8_t  SCR;
       uint8_t  RESERVED3[3];
  volatile uint32_t ACR;
  volatile uint8_t  ICR;
       uint8_t  RESERVED4[3];
  volatile uint8_t  FDR;
       uint8_t  RESERVED5[7];
  volatile uint8_t  TER;
       uint8_t  RESERVED6[39];
  volatile uint32_t FIFOLVL;
} LPC_UART_TypeDef;

typedef struct
{
  union {
  volatile const  uint8_t  RBR;
  volatile  uint8_t  THR;
  volatile uint8_t  DLL;
       uint32_t RESERVED0;
  };
  union {
  volatile uint8_t  DLM;
  volatile uint32_t IER;
  };
  union {
  volatile const  uint32_t IIR;
  volatile  uint8_t  FCR;
  };
  volatile uint8_t  LCR;
       uint8_t  RESERVED1[7];
  volatile const  uint8_t  LSR;
       uint8_t  RESERVED2[7];
  volatile uint8_t  SCR;
       uint8_t  RESERVED3[3];
  volatile uint32_t ACR;
  volatile uint8_t  ICR;
       uint8_t  RESERVED4[3];
  volatile uint8_t  FDR;
       uint8_t  RESERVED5[7];
  volatile uint8_t  TER;
       uint8_t  RESERVED6[39];
  volatile uint32_t FIFOLVL;
} LPC_UART0_TypeDef;

typedef struct
{
  union {
  volatile const  uint8_t  RBR;
  volatile  uint8_t  THR;
  volatile uint8_t  DLL;
       uint32_t RESERVED0;
  };
  union {
  volatile uint8_t  DLM;
  volatile uint32_t IER;
  };
  union {
  volatile const  uint32_t IIR;
  volatile  uint8_t  FCR;
  };
  volatile uint8_t  LCR;
       uint8_t  RESERVED1[3];
  volatile uint8_t  MCR;
       uint8_t  RESERVED2[3];
  volatile const  uint8_t  LSR;
       uint8_t  RESERVED3[3];
  volatile const  uint8_t  MSR;
       uint8_t  RESERVED4[3];
  volatile uint8_t  SCR;
       uint8_t  RESERVED5[3];
  volatile uint32_t ACR;
       uint32_t RESERVED6;
  volatile uint32_t FDR;
       uint32_t RESERVED7;
  volatile uint8_t  TER;
       uint8_t  RESERVED8[27];
  volatile uint8_t  RS485CTRL;
       uint8_t  RESERVED9[3];
  volatile uint8_t  ADRMATCH;
       uint8_t  RESERVED10[3];
  volatile uint8_t  RS485DLY;
       uint8_t  RESERVED11[3];
  volatile uint32_t FIFOLVL;
} LPC_UART1_TypeDef;

 
typedef struct
{
  volatile uint32_t SPCR;
  volatile const  uint32_t SPSR;
  volatile uint32_t SPDR;
  volatile uint32_t SPCCR;
       uint32_t RESERVED0[3];
  volatile uint32_t SPINT;
} LPC_SPI_TypeDef;

 
typedef struct
{
  volatile uint32_t CR0;
  volatile uint32_t CR1;
  volatile uint32_t DR;
  volatile const  uint32_t SR;
  volatile uint32_t CPSR;
  volatile uint32_t IMSC;
  volatile uint32_t RIS;
  volatile uint32_t MIS;
  volatile uint32_t ICR;
  volatile uint32_t DMACR;
} LPC_SSP_TypeDef;

 
typedef struct
{
  volatile uint32_t I2CONSET;
  volatile const  uint32_t I2STAT;
  volatile uint32_t I2DAT;
  volatile uint32_t I2ADR0;
  volatile uint32_t I2SCLH;
  volatile uint32_t I2SCLL;
  volatile  uint32_t I2CONCLR;
  volatile uint32_t MMCTRL;
  volatile uint32_t I2ADR1;
  volatile uint32_t I2ADR2;
  volatile uint32_t I2ADR3;
  volatile const  uint32_t I2DATA_BUFFER;
  volatile uint32_t I2MASK0;
  volatile uint32_t I2MASK1;
  volatile uint32_t I2MASK2;
  volatile uint32_t I2MASK3;
} LPC_I2C_TypeDef;

 
typedef struct
{
  volatile uint32_t I2SDAO;
  volatile uint32_t I2SDAI;
  volatile  uint32_t I2STXFIFO;
  volatile const  uint32_t I2SRXFIFO;
  volatile const  uint32_t I2SSTATE;
  volatile uint32_t I2SDMA1;
  volatile uint32_t I2SDMA2;
  volatile uint32_t I2SIRQ;
  volatile uint32_t I2STXRATE;
  volatile uint32_t I2SRXRATE;
  volatile uint32_t I2STXBITRATE;
  volatile uint32_t I2SRXBITRATE;
  volatile uint32_t I2STXMODE;
  volatile uint32_t I2SRXMODE;
} LPC_I2S_TypeDef;

 
typedef struct
{
  volatile uint32_t RICOMPVAL;
  volatile uint32_t RIMASK;
  volatile uint8_t  RICTRL;
       uint8_t  RESERVED0[3];
  volatile uint32_t RICOUNTER;
} LPC_RIT_TypeDef;

 
typedef struct
{
  volatile uint8_t  ILR;
       uint8_t  RESERVED0[7];
  volatile uint8_t  CCR;
       uint8_t  RESERVED1[3];
  volatile uint8_t  CIIR;
       uint8_t  RESERVED2[3];
  volatile uint8_t  AMR;
       uint8_t  RESERVED3[3];
  volatile const  uint32_t CTIME0;
  volatile const  uint32_t CTIME1;
  volatile const  uint32_t CTIME2;
  volatile uint8_t  SEC;
       uint8_t  RESERVED4[3];
  volatile uint8_t  MIN;
       uint8_t  RESERVED5[3];
  volatile uint8_t  HOUR;
       uint8_t  RESERVED6[3];
  volatile uint8_t  DOM;
       uint8_t  RESERVED7[3];
  volatile uint8_t  DOW;
       uint8_t  RESERVED8[3];
  volatile uint16_t DOY;
       uint16_t RESERVED9;
  volatile uint8_t  MONTH;
       uint8_t  RESERVED10[3];
  volatile uint16_t YEAR;
       uint16_t RESERVED11;
  volatile uint32_t CALIBRATION;
  volatile uint32_t GPREG0;
  volatile uint32_t GPREG1;
  volatile uint32_t GPREG2;
  volatile uint32_t GPREG3;
  volatile uint32_t GPREG4;
  volatile uint8_t  RTC_AUXEN;
       uint8_t  RESERVED12[3];
  volatile uint8_t  RTC_AUX;
       uint8_t  RESERVED13[3];
  volatile uint8_t  ALSEC;
       uint8_t  RESERVED14[3];
  volatile uint8_t  ALMIN;
       uint8_t  RESERVED15[3];
  volatile uint8_t  ALHOUR;
       uint8_t  RESERVED16[3];
  volatile uint8_t  ALDOM;
       uint8_t  RESERVED17[3];
  volatile uint8_t  ALDOW;
       uint8_t  RESERVED18[3];
  volatile uint16_t ALDOY;
       uint16_t RESERVED19;
  volatile uint8_t  ALMON;
       uint8_t  RESERVED20[3];
  volatile uint16_t ALYEAR;
       uint16_t RESERVED21;
} LPC_RTC_TypeDef;

 
typedef struct
{
  volatile uint8_t  WDMOD;
       uint8_t  RESERVED0[3];
  volatile uint32_t WDTC;
  volatile  uint8_t  WDFEED;
       uint8_t  RESERVED1[3];
  volatile const  uint32_t WDTV;
  volatile uint32_t WDCLKSEL;
} LPC_WDT_TypeDef;

 
typedef struct
{
  volatile uint32_t ADCR;
  volatile uint32_t ADGDR;
       uint32_t RESERVED0;
  volatile uint32_t ADINTEN;
  volatile const  uint32_t ADDR0;
  volatile const  uint32_t ADDR1;
  volatile const  uint32_t ADDR2;
  volatile const  uint32_t ADDR3;
  volatile const  uint32_t ADDR4;
  volatile const  uint32_t ADDR5;
  volatile const  uint32_t ADDR6;
  volatile const  uint32_t ADDR7;
  volatile const  uint32_t ADSTAT;
  volatile uint32_t ADTRM;
} LPC_ADC_TypeDef;

 
typedef struct
{
  volatile uint32_t DACR;
  volatile uint32_t DACCTRL;
  volatile uint16_t DACCNTVAL;
} LPC_DAC_TypeDef;

 
typedef struct
{
  volatile const  uint32_t MCCON;
  volatile  uint32_t MCCON_SET;
  volatile  uint32_t MCCON_CLR;
  volatile const  uint32_t MCCAPCON;
  volatile  uint32_t MCCAPCON_SET;
  volatile  uint32_t MCCAPCON_CLR;
  volatile uint32_t MCTIM0;
  volatile uint32_t MCTIM1;
  volatile uint32_t MCTIM2;
  volatile uint32_t MCPER0;
  volatile uint32_t MCPER1;
  volatile uint32_t MCPER2;
  volatile uint32_t MCPW0;
  volatile uint32_t MCPW1;
  volatile uint32_t MCPW2;
  volatile uint32_t MCDEADTIME;
  volatile uint32_t MCCCP;
  volatile uint32_t MCCR0;
  volatile uint32_t MCCR1;
  volatile uint32_t MCCR2;
  volatile const  uint32_t MCINTEN;
  volatile  uint32_t MCINTEN_SET;
  volatile  uint32_t MCINTEN_CLR;
  volatile const  uint32_t MCCNTCON;
  volatile  uint32_t MCCNTCON_SET;
  volatile  uint32_t MCCNTCON_CLR;
  volatile const  uint32_t MCINTFLAG;
  volatile  uint32_t MCINTFLAG_SET;
  volatile  uint32_t MCINTFLAG_CLR;
  volatile  uint32_t MCCAP_CLR;
} LPC_MCPWM_TypeDef;

 
typedef struct
{
  volatile  uint32_t QEICON;
  volatile const  uint32_t QEISTAT;
  volatile uint32_t QEICONF;
  volatile const  uint32_t QEIPOS;
  volatile uint32_t QEIMAXPOS;
  volatile uint32_t CMPOS0;
  volatile uint32_t CMPOS1;
  volatile uint32_t CMPOS2;
  volatile const  uint32_t INXCNT;
  volatile uint32_t INXCMP;
  volatile uint32_t QEILOAD;
  volatile const  uint32_t QEITIME;
  volatile const  uint32_t QEIVEL;
  volatile const  uint32_t QEICAP;
  volatile uint32_t VELCOMP;
  volatile uint32_t FILTER;
       uint32_t RESERVED0[998];
  volatile  uint32_t QEIIEC;
  volatile  uint32_t QEIIES;
  volatile const  uint32_t QEIINTSTAT;
  volatile const  uint32_t QEIIE;
  volatile  uint32_t QEICLR;
  volatile  uint32_t QEISET;
} LPC_QEI_TypeDef;

 
typedef struct
{
  volatile uint32_t mask[512];               
} LPC_CANAF_RAM_TypeDef;

typedef struct                           
{
  volatile uint32_t AFMR;
  volatile uint32_t SFF_sa;
  volatile uint32_t SFF_GRP_sa;
  volatile uint32_t EFF_sa;
  volatile uint32_t EFF_GRP_sa;
  volatile uint32_t ENDofTable;
  volatile const  uint32_t LUTerrAd;
  volatile const  uint32_t LUTerr;
  volatile uint32_t FCANIE;
  volatile uint32_t FCANIC0;
  volatile uint32_t FCANIC1;
} LPC_CANAF_TypeDef;

typedef struct                           
{
  volatile const  uint32_t CANTxSR;
  volatile const  uint32_t CANRxSR;
  volatile const  uint32_t CANMSR;
} LPC_CANCR_TypeDef;

typedef struct                           
{
  volatile uint32_t MOD;
  volatile  uint32_t CMR;
  volatile uint32_t GSR;
  volatile const  uint32_t ICR;
  volatile uint32_t IER;
  volatile uint32_t BTR;
  volatile uint32_t EWL;
  volatile const  uint32_t SR;
  volatile uint32_t RFS;
  volatile uint32_t RID;
  volatile uint32_t RDA;
  volatile uint32_t RDB;
  volatile uint32_t TFI1;
  volatile uint32_t TID1;
  volatile uint32_t TDA1;
  volatile uint32_t TDB1;
  volatile uint32_t TFI2;
  volatile uint32_t TID2;
  volatile uint32_t TDA2;
  volatile uint32_t TDB2;
  volatile uint32_t TFI3;
  volatile uint32_t TID3;
  volatile uint32_t TDA3;
  volatile uint32_t TDB3;
} LPC_CAN_TypeDef;

 
typedef struct                           
{
  volatile const  uint32_t DMACIntStat;
  volatile const  uint32_t DMACIntTCStat;
  volatile  uint32_t DMACIntTCClear;
  volatile const  uint32_t DMACIntErrStat;
  volatile  uint32_t DMACIntErrClr;
  volatile const  uint32_t DMACRawIntTCStat;
  volatile const  uint32_t DMACRawIntErrStat;
  volatile const  uint32_t DMACEnbldChns;
  volatile uint32_t DMACSoftBReq;
  volatile uint32_t DMACSoftSReq;
  volatile uint32_t DMACSoftLBReq;
  volatile uint32_t DMACSoftLSReq;
  volatile uint32_t DMACConfig;
  volatile uint32_t DMACSync;
} LPC_GPDMA_TypeDef;

typedef struct                           
{
  volatile uint32_t DMACCSrcAddr;
  volatile uint32_t DMACCDestAddr;
  volatile uint32_t DMACCLLI;
  volatile uint32_t DMACCControl;
  volatile uint32_t DMACCConfig;
} LPC_GPDMACH_TypeDef;

 
typedef struct
{
  volatile const  uint32_t HcRevision;              
  volatile uint32_t HcControl;
  volatile uint32_t HcCommandStatus;
  volatile uint32_t HcInterruptStatus;
  volatile uint32_t HcInterruptEnable;
  volatile uint32_t HcInterruptDisable;
  volatile uint32_t HcHCCA;
  volatile const  uint32_t HcPeriodCurrentED;
  volatile uint32_t HcControlHeadED;
  volatile uint32_t HcControlCurrentED;
  volatile uint32_t HcBulkHeadED;
  volatile uint32_t HcBulkCurrentED;
  volatile const  uint32_t HcDoneHead;
  volatile uint32_t HcFmInterval;
  volatile const  uint32_t HcFmRemaining;
  volatile const  uint32_t HcFmNumber;
  volatile uint32_t HcPeriodicStart;
  volatile uint32_t HcLSTreshold;
  volatile uint32_t HcRhDescriptorA;
  volatile uint32_t HcRhDescriptorB;
  volatile uint32_t HcRhStatus;
  volatile uint32_t HcRhPortStatus1;
  volatile uint32_t HcRhPortStatus2;
       uint32_t RESERVED0[40];
  volatile const  uint32_t Module_ID;

  volatile const  uint32_t OTGIntSt;                
  volatile uint32_t OTGIntEn;
  volatile  uint32_t OTGIntSet;
  volatile  uint32_t OTGIntClr;
  volatile uint32_t OTGStCtrl;
  volatile uint32_t OTGTmr;
       uint32_t RESERVED1[58];

  volatile const  uint32_t USBDevIntSt;             
  volatile uint32_t USBDevIntEn;
  volatile  uint32_t USBDevIntClr;
  volatile  uint32_t USBDevIntSet;

  volatile  uint32_t USBCmdCode;              
  volatile const  uint32_t USBCmdData;

  volatile const  uint32_t USBRxData;               
  volatile  uint32_t USBTxData;
  volatile const  uint32_t USBRxPLen;
  volatile  uint32_t USBTxPLen;
  volatile uint32_t USBCtrl;
  volatile  uint32_t USBDevIntPri;

  volatile const  uint32_t USBEpIntSt;              
  volatile uint32_t USBEpIntEn;
  volatile  uint32_t USBEpIntClr;
  volatile  uint32_t USBEpIntSet;
  volatile  uint32_t USBEpIntPri;

  volatile uint32_t USBReEp;                 
  volatile  uint32_t USBEpInd;
  volatile uint32_t USBMaxPSize;

  volatile const  uint32_t USBDMARSt;               
  volatile  uint32_t USBDMARClr;
  volatile  uint32_t USBDMARSet;
       uint32_t RESERVED2[9];
  volatile uint32_t USBUDCAH;
  volatile const  uint32_t USBEpDMASt;
  volatile  uint32_t USBEpDMAEn;
  volatile  uint32_t USBEpDMADis;
  volatile const  uint32_t USBDMAIntSt;
  volatile uint32_t USBDMAIntEn;
       uint32_t RESERVED3[2];
  volatile const  uint32_t USBEoTIntSt;
  volatile  uint32_t USBEoTIntClr;
  volatile  uint32_t USBEoTIntSet;
  volatile const  uint32_t USBNDDRIntSt;
  volatile  uint32_t USBNDDRIntClr;
  volatile  uint32_t USBNDDRIntSet;
  volatile const  uint32_t USBSysErrIntSt;
  volatile  uint32_t USBSysErrIntClr;
  volatile  uint32_t USBSysErrIntSet;
       uint32_t RESERVED4[15];

  union {
  volatile const  uint32_t I2C_RX;                  
  volatile  uint32_t I2C_TX;
  };
  volatile const  uint32_t I2C_STS;
  volatile uint32_t I2C_CTL;
  volatile uint32_t I2C_CLKHI;
  volatile  uint32_t I2C_CLKLO;
       uint32_t RESERVED5[824];

  union {
  volatile uint32_t USBClkCtrl;              
  volatile uint32_t OTGClkCtrl;
  };
  union {
  volatile const  uint32_t USBClkSt;
  volatile const  uint32_t OTGClkSt;
  };
} LPC_USB_TypeDef;

 
typedef struct
{
  volatile uint32_t MAC1;                    
  volatile uint32_t MAC2;
  volatile uint32_t IPGT;
  volatile uint32_t IPGR;
  volatile uint32_t CLRT;
  volatile uint32_t MAXF;
  volatile uint32_t SUPP;
  volatile uint32_t TEST;
  volatile uint32_t MCFG;
  volatile uint32_t MCMD;
  volatile uint32_t MADR;
  volatile  uint32_t MWTD;
  volatile const  uint32_t MRDD;
  volatile const  uint32_t MIND;
       uint32_t RESERVED0[2];
  volatile uint32_t SA0;
  volatile uint32_t SA1;
  volatile uint32_t SA2;
       uint32_t RESERVED1[45];
  volatile uint32_t Command;                 
  volatile const  uint32_t Status;
  volatile uint32_t RxDescriptor;
  volatile uint32_t RxStatus;
  volatile uint32_t RxDescriptorNumber;
  volatile const  uint32_t RxProduceIndex;
  volatile uint32_t RxConsumeIndex;
  volatile uint32_t TxDescriptor;
  volatile uint32_t TxStatus;
  volatile uint32_t TxDescriptorNumber;
  volatile uint32_t TxProduceIndex;
  volatile const  uint32_t TxConsumeIndex;
       uint32_t RESERVED2[10];
  volatile const  uint32_t TSV0;
  volatile const  uint32_t TSV1;
  volatile const  uint32_t RSV;
       uint32_t RESERVED3[3];
  volatile uint32_t FlowControlCounter;
  volatile const  uint32_t FlowControlStatus;
       uint32_t RESERVED4[34];
  volatile uint32_t RxFilterCtrl;            
  volatile uint32_t RxFilterWoLStatus;
  volatile uint32_t RxFilterWoLClear;
       uint32_t RESERVED5;
  volatile uint32_t HashFilterL;
  volatile uint32_t HashFilterH;
       uint32_t RESERVED6[882];
  volatile const  uint32_t IntStatus;               
  volatile uint32_t IntEnable;
  volatile  uint32_t IntClear;
  volatile  uint32_t IntSet;
       uint32_t RESERVED7;
  volatile uint32_t PowerDown;
       uint32_t RESERVED8;
  volatile uint32_t Module_ID;
} LPC_EMAC_TypeDef;


#pragma no_anon_unions



 
 
 
 
#line 924 "C:\\Keil\\ARM\\INC\\NXP\\LPC17xx\\LPC17xx.H"

 
#line 945 "C:\\Keil\\ARM\\INC\\NXP\\LPC17xx\\LPC17xx.H"

 
#line 959 "C:\\Keil\\ARM\\INC\\NXP\\LPC17xx\\LPC17xx.H"

 
#line 972 "C:\\Keil\\ARM\\INC\\NXP\\LPC17xx\\LPC17xx.H"

 







 
 
 
#line 1031 "C:\\Keil\\ARM\\INC\\NXP\\LPC17xx\\LPC17xx.H"

#line 6 "avar_hndl.c"
#line 1 "main.h"
#line 1 "C:\\Keil\\ARM\\RV31\\INC\\rtl.h"









 




 

 


#line 27 "C:\\Keil\\ARM\\RV31\\INC\\rtl.h"







 typedef unsigned int   size_t;


typedef signed char     S8;
typedef unsigned char   U8;
typedef short           S16;
typedef unsigned short  U16;
typedef int             S32;
typedef unsigned int    U32;
typedef long long       S64;
typedef unsigned long long U64;
typedef unsigned char   BIT;
typedef unsigned int    BOOL;

#line 54 "C:\\Keil\\ARM\\RV31\\INC\\rtl.h"

#line 66 "C:\\Keil\\ARM\\RV31\\INC\\rtl.h"



 





 
typedef U32 OS_SEM[2];

 

typedef U32 OS_MBX[];

 
typedef U32 OS_MUT[3];

 
typedef U32 OS_TID;

 
typedef void *OS_ID;

 
typedef U32 OS_RESULT;

 












 




#line 194 "C:\\Keil\\ARM\\RV31\\INC\\rtl.h"



 



 
extern void      os_set_env    (void);
extern void      rt_sys_init   (void (*task)(void), U8 priority, void *stk);
extern void      rt_tsk_pass   (void);
extern OS_TID    rt_tsk_self   (void);
extern OS_RESULT rt_tsk_prio   (OS_TID task_id, U8 new_prio);
extern OS_TID    rt_tsk_create (void (*task)(void), U8 priority, void *stk, void *argv);
extern OS_RESULT rt_tsk_delete (OS_TID task_id);

#line 230 "C:\\Keil\\ARM\\RV31\\INC\\rtl.h"

extern void      _os_sys_init(U32 p, void (*task)(void), U32 prio_stksz,
                                     void *stk)                        __svc_indirect(0);
extern OS_TID    _os_tsk_create (U32 p, void (*task)(void), U32 prio_stksz,
                                        void *stk, void *argv)         __svc_indirect(0);
extern OS_TID    _os_tsk_create_ex (U32 p, void (*task)(void *), U32 prio_stksz,
                                           void *stk, void *argv)      __svc_indirect(0);
extern OS_TID    _os_tsk_self (U32 p)                                  __svc_indirect(0);
extern void      _os_tsk_pass (U32 p)                                  __svc_indirect(0);
extern OS_RESULT _os_tsk_prio (U32 p, OS_TID task_id, U8 new_prio)     __svc_indirect(0);
extern OS_RESULT _os_tsk_delete (U32 p, OS_TID task_id)                __svc_indirect(0);

 
extern OS_RESULT rt_evt_wait (U16 wait_flags,  U16 timeout, BOOL and_wait);
extern void      rt_evt_set  (U16 event_flags, OS_TID task_id);
extern void      rt_evt_clr  (U16 clear_flags, OS_TID task_id);
extern U16       rt_evt_get  (void);







extern OS_RESULT _os_evt_wait(U32 p, U16 wait_flags, U16 timeout,
                                     BOOL and_wait)                    __svc_indirect(0);
extern void      _os_evt_set (U32 p, U16 event_flags, OS_TID task_id)  __svc_indirect(0);
extern void      _os_evt_clr (U32 p, U16 clear_flags, OS_TID task_id)  __svc_indirect(0);
extern U16       _os_evt_get (U32 p)                                   __svc_indirect(0);
extern void      isr_evt_set (U16 event_flags, OS_TID task_id);

 
extern void      rt_sem_init (OS_ID semaphore, U16 token_count);
extern OS_RESULT rt_sem_send (OS_ID semaphore);
extern OS_RESULT rt_sem_wait (OS_ID semaphore, U16 timeout);





extern void      _os_sem_init (U32 p, OS_ID semaphore, 
                                      U16 token_count)                 __svc_indirect(0);
extern OS_RESULT _os_sem_send (U32 p, OS_ID semaphore)                 __svc_indirect(0);
extern OS_RESULT _os_sem_wait (U32 p, OS_ID semaphore, U16 timeout)    __svc_indirect(0);
extern void      isr_sem_send (OS_ID semaphore);

 
extern void      rt_mbx_init  (OS_ID mailbox, U16 mbx_size);
extern OS_RESULT rt_mbx_send  (OS_ID mailbox, void *p_msg,    U16 timeout);
extern OS_RESULT rt_mbx_wait  (OS_ID mailbox, void **message, U16 timeout);
extern OS_RESULT rt_mbx_check (OS_ID mailbox);







extern void      _os_mbx_init (U32 p, OS_ID mailbox, U16 mbx_size)     __svc_indirect(0);
extern OS_RESULT _os_mbx_send (U32 p, OS_ID mailbox, void *message_ptr,
                                      U16 timeout)                     __svc_indirect(0);
extern OS_RESULT _os_mbx_wait (U32 p, OS_ID mailbox, void  **message,
                                      U16 timeout)                     __svc_indirect(0);
extern OS_RESULT _os_mbx_check (U32 p, OS_ID mailbox)                  __svc_indirect(0);
extern void      isr_mbx_send (OS_ID mailbox, void *message_ptr);
extern OS_RESULT isr_mbx_receive (OS_ID mailbox, void **message);

 
extern void      rt_mut_init    (OS_ID mutex);
extern OS_RESULT rt_mut_release (OS_ID mutex);
extern OS_RESULT rt_mut_wait    (OS_ID mutex, U16 timeout);





extern void      _os_mut_init (U32 p, OS_ID mutex)                     __svc_indirect(0);
extern OS_RESULT _os_mut_release (U32 p, OS_ID mutex)                  __svc_indirect(0);
extern OS_RESULT _os_mut_wait (U32 p, OS_ID mutex, U16 timeout)        __svc_indirect(0);

 
extern void      rt_dly_wait (U16 delay_time);
extern void      rt_itv_set  (U16 interval_time);
extern void      rt_itv_wait (void);





extern void      _os_dly_wait (U32 p, U16 delay_time)                  __svc_indirect(0);
extern void      _os_itv_set (U32 p, U16 interval_time)                __svc_indirect(0);
extern void      _os_itv_wait (U32 p)                                  __svc_indirect(0);

 
extern OS_ID     rt_tmr_create (U16 tcnt, U16 info);
extern OS_ID     rt_tmr_kill   (OS_ID timer);




extern OS_ID     _os_tmr_create (U32 p, U16 tcnt, U16 info)            __svc_indirect(0);
extern OS_ID     _os_tmr_kill (U32 p, OS_ID timer)                     __svc_indirect(0);

 
extern void      rt_tsk_lock   (void);
extern void      rt_tsk_unlock (void);




extern void      _os_tsk_lock (U32 p)                                  __svc_indirect(0);
extern void      _os_tsk_unlock (U32 p)                                __svc_indirect(0);

 
extern int       _init_box (void *box_mem, U32 box_size, U32 blk_size);
extern void     *_alloc_box (void *box_mem);
extern void     *_calloc_box (void *box_mem);
extern int       _free_box (void *box_mem, void *box);








 




 

typedef struct {                         
  U8  hr;                                
  U8  min;                               
  U8  sec;                               
  U8  day;                               
  U8  mon;                               
  U16 year;                              
} RL_TIME;

typedef struct {                         
  S8  name[256];                         
  U32 size;                              
  U16 fileID;                            
  U8  attrib;                            
  RL_TIME time;                          
} FINFO;

extern int finit (void);
extern int fdelete (const char *filename);
extern int frename (const char *oldname, const char *newname);
extern int ffind (const char *pattern, FINFO *info);
extern U64 ffree (const char *drive);
extern int fformat (const char *drive);
extern int fanalyse (const char *drive);
extern int fcheck (const char *drive);
extern int fdefrag (const char *drive);

 




 

 



 






 
#line 415 "C:\\Keil\\ARM\\RV31\\INC\\rtl.h"

 
#line 428 "C:\\Keil\\ARM\\RV31\\INC\\rtl.h"

 





 
#line 442 "C:\\Keil\\ARM\\RV31\\INC\\rtl.h"

 




 



extern void init_TcpNet (void);
extern void main_TcpNet (void);
extern void timer_tick (void);
extern U8   udp_get_socket (U8 tos, U8 opt, 
                            U16 (*listener)(U8 socket, U8 *remip, U16 port, U8 *buf, U16 len));
extern BOOL udp_release_socket (U8 socket);
extern BOOL udp_open (U8 socket, U16 locport);
extern BOOL udp_close (U8 socket);
extern BOOL udp_mcast_ttl (U8 socket, U8 ttl);
extern U8  *udp_get_buf (U16 size);
extern BOOL udp_send (U8 socket, U8 *remip, U16 remport, U8 *buf, U16 dlen);
extern U8   tcp_get_socket (U8 type, U8 tos, U16 tout,
                            U16 (*listener)(U8 socket, U8 event, U8 *buf, U16 len));
extern BOOL tcp_release_socket (U8 socket);
extern BOOL tcp_listen (U8 socket, U16 locport);
extern BOOL tcp_connect (U8 socket, U8 *remip, U16 remport, U16 locport);
extern U8  *tcp_get_buf (U16 size);
extern U16  tcp_max_dsize (U8 socket);
extern BOOL tcp_check_send (U8 socket);
extern U8   tcp_get_state (U8 socket);
extern BOOL tcp_send (U8 socket, U8 *buf, U16 dlen);
extern BOOL tcp_close (U8 socket);
extern BOOL tcp_abort (U8 socket);
extern void tcp_reset_window (U8 socket);
extern BOOL arp_cache_ip (U8 *ipadr, U8 type);
extern void ppp_listen (char const *user, char const *passw);
extern void ppp_connect (char const *dialnum, char const *user, char const *passw);
extern void ppp_close (void);
extern BOOL ppp_is_up (void);
extern void slip_listen (void);
extern void slip_connect (char const *dialnum);
extern void slip_close (void);
extern BOOL slip_is_up (void);
extern U8   get_host_by_name (U8 *hostn, void (*cbfunc)(U8 event, U8 *host_ip));
extern BOOL smtp_connect (U8 *ipadr, U16 port, void (*cbfunc)(U8 event));
extern void dhcp_disable (void);
extern BOOL igmp_join (U8 *group_ip);
extern BOOL igmp_leave (U8 *group_ip);
extern BOOL snmp_trap (U8 *manager_ip, U8 gen_trap, U8 spec_trap, U16 *obj_list);
extern BOOL snmp_set_community (const char *community);






 
  

 
#line 2 "main.h"

































#line 41 "main.h"





#line 58 "main.h"

#line 67 "main.h"






#line 80 "main.h"

#line 89 "main.h"











#line 106 "main.h"







#line 136 "main.h"





#line 151 "main.h"













#line 182 "main.h"

#line 220 "main.h"

#line 240 "main.h"























#line 410 "main.h"








































#line 478 "main.h"







		










#line 510 "main.h"

#line 524 "main.h"

#line 540 "main.h"





















#line 574 "main.h"

#line 588 "main.h"









 


#line 609 "main.h"

#line 619 "main.h"

#line 628 "main.h"

#line 637 "main.h"

#line 649 "main.h"

#line 659 "main.h"

#line 668 "main.h"

#line 676 "main.h"

#line 685 "main.h"

#line 697 "main.h"

#line 709 "main.h"



extern char b1000Hz,b100Hz,b50Hz,b10Hz,b5Hz,b2Hz,b1Hz;
extern short t0cnt,t0cnt0,t0cnt1,t0cnt2,t0cnt3,t0cnt4,t0cnt5,t0cnt6,t0_cnt7;
extern char bFL5,bFL2,bFL,bFL_;
extern signed short main_10Hz_cnt;
extern signed short main_1Hz_cnt;



extern char cnt_of_slave;







typedef enum {
	iMn_VD,
	iSet_VD,

	iMn_TELECORE2017,

	iSrv_sl,iNet,iNet3,iNetEM,
	iSet,iSet_220_IPS_TERMOKOMPENSAT,iSet_220_V2,
	iInv_set,
	iMakb,
	iBps,iBps_elteh,iS2,iSet_prl,iK_prl,iDnd,
	iK, iK_220_IPS_TERMOKOMPENSAT,iK_220_IPS_TERMOKOMPENSAT_IB,iK_TELECORE,iK_VD,
	iSpcprl,iSpc,k,Crash_0,Crash_1,iKednd,iAv_view_avt,iAKE,iSpc_termocompensat,
	iLoad,iAVAR,
	iStr,iStr_VD,
	iVrs,iPrltst,iApv,
	iK_bps,iK_bps_sel,iK_bat_ips_termokompensat_ib,iK_bat_TELECORE,iK_bat_sel,iK_bat_sel_TELECORE,iK_load,iK_net,iK_net3,
	iK_makb_sel,iK_makb,iK_out,
	iTst_220_IPS_TERMOKOMPENSAT,iTst_VD,
	iTst_klbr,iTst_BPS1,iTst_BPS2,iTst_BPS12,iDebug,
	iDef_220_IPS_TERMOKOMPENSAT,
	iSet_st_prl,iK_pdp,iSet_T,
	iDeb,
	iPrl_bat_in_out,iPrl_bat_in_sel,iPdp1,iJAv_sel,iJAv_net_sel,iJAv_net,iJAv_src1,
	iTst_bps, iAusw,iAusw_prl,iAusw_set,
	iK_t_ext,iK_t_3U,iK_t_ext_6U,
	iAv_view,
	iBatLogKe,iJ_bat_ke,iBatLogVz,iJ_bat_vz,iBatLogWrk,
	iExtern,
	iK_power_net,
	iExt_set,iExt_set_3U,
	iExt_dt,
	iExt_sk,iExt_sk_3U,
	iExt_ddv,iExt_ddi,iExt_dud,iExt_dp,iSM,iLog,iLog_,iBatLog,iKlimat,iKlimat_kontur,iKlimat_TELECORE,
	iEnerg3,iEnerg,
	iVent,
	iK_power_net3,
	iAvt,iLan_set,
	iNpn_set,
	iBps_list,
	iRele_set,iRele_set_,
	iFWabout,
	iAvt_set_sel,iAvt_set,iSet_li_bat,
	iOut_volt_contr,iDop_rele_set,iBlok_ips_set,iIps_Curr_Avg_Set}i_enum;

typedef struct  
{

i_enum i;
signed char s_i;
signed char s_i1;
signed char s_i2;
signed char i_s;
} stuct_ind;







extern stuct_ind a_ind,b_ind[10],c_ind;
extern signed short ptr_ind;
extern char lcd_buffer[200+100];
extern signed char parol[3];
extern char phase;
extern char lcd_bitmap[1024];
extern char dig[5];
extern signed short ind_pointer;
extern char zero_on;
extern char mnemo_cnt;
extern char simax;
extern short av_j_si_max;
extern const char ABCDEF[];
extern const char sm_mont[13][4]; 




extern signed short Ktsrc[2];
extern signed short Kusrc[2];
extern signed short Kisrc[2];
extern signed short Ki0src[2];
extern signed short Kubat[2];
extern signed short Kubatm[2];
extern unsigned short Kibat0[2];
extern signed short Kibat1[2];
extern signed short Ktbat[2];
extern signed short Kunet;
extern signed short Ktext[3];
extern signed short Kuload;
extern signed short Kunet_ext[3];
extern signed short KunetA;
extern signed short KunetB;
extern signed short KunetC;
extern signed short Kubps;
extern signed short Kuout;
extern signed short Kuvd;

extern signed short MAIN_IST;
extern signed short UMAX;
extern signed short UOUT;
extern signed short UOUT_;
extern signed short UAVT;
extern signed short TMAX;
extern signed short TSIGN;
extern signed short TSYSMAX;
extern signed short AV_OFF_AVT;
extern signed short USIGN;
extern signed short UMN;
extern signed short ZV_ON;
extern signed short UINMAX;
extern signed short UINMIN;
extern signed short UOUTMAX;
extern signed short UOUTMIN;
extern signed short IKB;

extern signed short IMAX;
extern signed short IMIN;
extern signed short APV_ON;
extern signed short IZMAX;
extern signed short U0B;
extern signed short TZAS;
extern signed short VZ_HR;
extern signed short TBAT;
extern signed short U_AVT;
extern signed short DU;

extern signed short RELE_SET_MASK[4];

extern signed short TBATMAX;
extern signed short TBATSIGN;
extern signed short UBM_AV;
extern signed short RELE_LOG;
extern signed short TBOXMAX;
extern signed short TBOXREG;
extern signed short TBOXVENTMAX;
extern signed short TLOADDISABLE;
extern signed short TLOADENABLE;
extern signed short TBATDISABLE;
extern signed short TBATENABLE;
extern signed short TBOXMAX;
extern signed short TBOXREG;
extern signed short TBOXVENTMAX;
extern signed short TLOADDISABLE;
extern signed short TLOADENABLE;
extern signed short TBATDISABLE;
extern signed short TBATENABLE;
extern signed short TVENTON;
extern signed short TVENTOFF;
extern signed short TWARMON;
extern signed short TWARMOFF;
typedef enum {rvsAKB=0,rvsEXT,rvsBPS} enum_releventsign;
extern enum_releventsign RELEVENTSIGN;
extern signed short TZNPN;
extern signed short UONPN;
extern signed short UVNPN;
typedef enum {npnoOFF=0,npnoRELEVENT,npnoRELEAVBAT2} enum_npn_out;
extern enum_npn_out NPN_OUT;
typedef enum {npnsULOAD=0,npnsAVNET} enum_npn_sign;
extern enum_npn_sign NPN_SIGN;
extern signed short TERMOKOMPENS;
extern signed short TBOXVENTON; 
extern signed short TBOXVENTOFF;
extern signed short TBOXWARMON; 
extern signed short TBOXWARMOFF;
extern signed short BAT_TYPE;	
extern signed short DU_LI_BAT;	
extern signed short FORVARDBPSCHHOUR;	
extern signed short NUMBAT;
extern signed short NUMBAT_TELECORE;
extern signed short NUMIST;
extern signed short NUMINV;
extern signed short NUMDT;
extern signed short NUMSK;
extern signed short NUMEXT;
extern signed short NUMAVT;
extern signed short NUMMAKB;
extern signed short NUMBYPASS;
extern signed short U_OUT_KONTR_MAX;
extern signed short U_OUT_KONTR_MIN;
extern signed short U_OUT_KONTR_DELAY;
extern signed short DOP_RELE_FUNC;
extern signed short CNTRL_HNDL_TIME;	
extern signed short USODERG_LI_BAT;		
extern signed short QSODERG_LI_BAT;		
extern signed short TVENTMAX;			
extern signed short ICA_EN;				
extern signed short ICA_CH;				
extern signed short ICA_MODBUS_ADDRESS;
extern signed short ICA_MODBUS_TCP_IP1,ICA_MODBUS_TCP_IP2,ICA_MODBUS_TCP_IP3,ICA_MODBUS_TCP_IP4;	
extern signed short ICA_MODBUS_TCP_UNIT_ID;	
extern signed short PWM_START;			
extern signed short KB_ALGORITM;		
extern signed short REG_SPEED;			

typedef enum {apvON=0x01,apvOFF=0x00}enum_apv_on;
extern enum_apv_on APV_ON1,APV_ON2;

extern signed short APV_ON2_TIME;

typedef enum {bisON=0x0055,bisOFF=0x00aa}enum_bat_is_on;
extern enum_bat_is_on BAT_IS_ON[2];

extern signed short BAT_DAY_OF_ON[2];
extern signed short BAT_MONTH_OF_ON[2];
extern signed short BAT_YEAR_OF_ON[2];
extern signed short BAT_C_NOM[2];
extern signed short BAT_RESURS[2];
extern signed short BAT_C_REAL[2];


extern unsigned short AUSW_MAIN;
extern unsigned long 	AUSW_MAIN_NUMBER;
extern unsigned short AUSW_DAY;
extern unsigned short AUSW_MONTH;
extern unsigned short AUSW_YEAR;
extern unsigned short AUSW_UKU;
extern unsigned short AUSW_UKU_SUB;
extern unsigned long AUSW_UKU_NUMBER;
extern unsigned long	AUSW_BPS1_NUMBER;
extern unsigned long  AUSW_BPS2_NUMBER;
extern unsigned short AUSW_RS232;
extern unsigned short AUSW_PDH;
extern unsigned short AUSW_SDH;
extern unsigned short AUSW_ETH;

extern signed short TMAX_EXT_EN[3];
extern signed short TMAX_EXT[3];
extern signed short TMIN_EXT_EN[3];
extern signed short TMIN_EXT[3];
extern signed short T_EXT_REL_EN[3];
extern signed short T_EXT_ZVUK_EN[3];
extern signed short T_EXT_LCD_EN[3];
extern signed short T_EXT_RS_EN[3];

extern signed short SK_SIGN[4];
extern signed short SK_REL_EN[4];
extern signed short SK_ZVUK_EN[4];
extern signed short SK_LCD_EN[4];
extern signed short SK_RS_EN[4];

typedef enum {AVZ_1=1,AVZ_2=2,AVZ_3=3,AVZ_6=6,AVZ_12=12,AVZ_OFF=0}enum_avz;
extern enum_avz AVZ;

extern unsigned short HOUR_AVZ;
extern unsigned short MIN_AVZ;
extern unsigned short SEC_AVZ;
extern unsigned short DATE_AVZ;
extern unsigned short MONTH_AVZ;
extern unsigned short YEAR_AVZ;
extern unsigned short AVZ_TIME;
typedef enum {mnON=0x55,mnOFF=0xAA}enum_mnemo_on;
extern enum_mnemo_on MNEMO_ON;
extern unsigned short MNEMO_TIME;
extern signed short POWER_CNT_ADRESS;

extern signed short ETH_IS_ON;
extern signed short ETH_DHCP_ON;
extern signed short ETH_IP_1;
extern signed short ETH_IP_2;
extern signed short ETH_IP_3;
extern signed short ETH_IP_4;
extern signed short ETH_MASK_1;
extern signed short ETH_MASK_2;
extern signed short ETH_MASK_3;
extern signed short ETH_MASK_4;
extern signed short ETH_TRAP1_IP_1;
extern signed short ETH_TRAP1_IP_2;
extern signed short ETH_TRAP1_IP_3;
extern signed short ETH_TRAP1_IP_4;
extern signed short ETH_TRAP2_IP_1;
extern signed short ETH_TRAP2_IP_2;
extern signed short ETH_TRAP2_IP_3;
extern signed short ETH_TRAP2_IP_4;
extern signed short ETH_TRAP3_IP_1;
extern signed short ETH_TRAP3_IP_2;
extern signed short ETH_TRAP3_IP_3;
extern signed short ETH_TRAP3_IP_4;
extern signed short ETH_TRAP4_IP_1;
extern signed short ETH_TRAP4_IP_2;
extern signed short ETH_TRAP4_IP_3;
extern signed short ETH_TRAP4_IP_4;
extern signed short ETH_TRAP5_IP_1;
extern signed short ETH_TRAP5_IP_2;
extern signed short ETH_TRAP5_IP_3;
extern signed short ETH_TRAP5_IP_4;
extern signed short ETH_SNMP_PORT_READ;
extern signed short ETH_SNMP_PORT_WRITE;
extern signed short ETH_GW_1;
extern signed short ETH_GW_2;
extern signed short ETH_GW_3;
extern signed short ETH_GW_4;

extern signed short RELE_VENT_LOGIC;

extern signed short MODBUS_ADRESS;
extern signed short MODBUS_BAUDRATE;
extern signed short BAT_LINK;
extern signed short I_LOAD_MODE;		




typedef struct
     {
	char 		_cnt_to_block;
	signed short	_Ub;
     signed short	_Ubm;
     signed short	_dUbm;
	signed short	_Ib;
	signed short	_Tb;
	char 		_nd;
	char 		_cnt_wrk;
	char 		_wrk;
	unsigned short _zar;
	char 		_full_ver;
	signed long 	_zar_cnt;
	signed long 	_zar_cnt_ke;
	unsigned short _Iintegr,_Iintegr_; 
	signed short 	_u_old[8];
	signed short	_u_old_cnt;
	unsigned long 	_wrk_date[2];
	char 		_rel_stat;
	char			_av;
	char			_time_cnt;
	char 		_temper_stat;
	
	
	signed short 	_sign_temper_cnt;
	signed short 	_max_temper_cnt;
	signed long 	_resurs_cnt;
	signed short 	_cnt_as; 	
     
	
	} BAT_STAT; 
extern BAT_STAT bat[2],bat_ips;
extern signed short		bat_u_old_cnt;
extern signed short 	Ib_ips_termokompensat;


typedef enum {bsOFF=0,bsCOMM_ON,bsOK} enum_batStat;


typedef struct
     {
	
	signed short	_Ub;
     
     
	signed short	_Ib;
	signed short	_Tb;
	char 		_nd;
	char   		_soh;
	char 		_soc;
	signed short   _ratCap;
	char 		_comErrStat;	
	enum_batStat	_batStat;
	signed short 	_cclv;
	char 		_rbt;
	short 		_canErrorCnt;
	char			_canError;
	char 		_485Error;
	short 		_485ErrorCnt;
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
     
	
	} LI_BAT_STAT; 
extern LI_BAT_STAT li_bat;



typedef struct
     {
	signed short 	_Iout;
	signed short 	_Uout;
	signed short 	_Pout;
	signed short 	_Unet; 	
	signed short 	_Uin;
	char			_T;
	char 		_flags;
	char			_cnt;
	char 		_adress;
	} BYPS_STAT; 
extern BYPS_STAT byps;



typedef struct
     {
	signed short	_U[5];
	signed short	_Ub[5];
	signed short	_T[5];
	signed short	_T_nd[5];
	signed short 	_cnt; 	
	} MAKB_STAT; 
extern MAKB_STAT makb[4];



typedef struct
     {
	signed short	_max_cell_volt;
	signed short	_min_cell_volt;
	signed short	_max_cell_temp;
	signed short	_min_cell_temp;
	signed short	_tot_bat_volt;
	signed short	_ch_curr;
	signed short	_dsch_curr;
	signed short	_rat_cap;
	signed short	_s_o_h;
	signed short	_s_o_c;
	signed short	_c_c_l_v;
	signed short	_r_b_t;
	signed short	_b_p_ser_num;
	signed short   _flags1;
	signed short 	_flags2;
	signed short 	_communication2lvlErrorStat; 	
	signed short	_communication2lvlErrorCnt;  	
	signed short 	_cnt;
	signed short 	_communicationFullErrorStat;	
	signed short   _battIsOn;		
	char 		_plazma[8];		
	signed short 	_isOnCnt;
	signed short	_s_o_c_abs;		
	signed short 	_s_o_c_percent; 
	signed short	_plazma_ss;
	signed short	_zar_percent;	
	signed char		_cell_temp_1;	
	signed char		_cell_temp_2;	
	signed char		_cell_temp_3;	
	signed char		_cell_temp_4;	
	signed char		_cell_temp_ambient;	
	signed char		_cell_temp_power;	
	
	
	
	signed char		_charge_and_discharge_current_alarm_status;	 	
	signed char 	_battery_total_voltage_alarm_status;			
	signed char		_custom_alarm_quantity;							
	signed char		_balanced_event_code;							
	signed char 	_voltage_event_code;							
	signed char 	_temperature_event_code;						
	signed char		_current_event_code;							
	signed char		_fet_status_code;								
	signed short	_balanced_status_code;							
	signed char 	_system_status_code;							

	} LAKB_STAT; 
extern LAKB_STAT lakb[3];
extern char lakb_damp[1][42];
extern char bLAKB_KONF_CH;
extern char bLAKB_KONF_CH_old;
extern char lakb_ison_mass[7];
extern short lakb_mn_ind_cnt;
extern char bLAKB_KONF_CH_EN;
extern char bRS485ERR;
extern short LBAT_STRUKT;
extern char lakb_error_cnt;	
extern short numOfPacks,numOfPacks_;
extern short numOfCells, numOfTemperCells, baseOfData;
extern short lakb_stat_comm_error;	
extern short lakbNotErrorNum;		
extern short lakbKanErrorCnt;		
extern short lakbKanErrorStat;		





extern char can_slot[12][16];
extern char plazma_can_inv[3];





typedef struct
    {
    enum {dSRC=3,dINV=5,dNET_METR=7,dIBAT_METR=9,dMAKB=11}_device;
	char _av;
	
	
	
	
	
 	enum {bsOFF_AV_NET,bsAPV,bsWRK,bsRDY,bsBL,bsAV}_state;
    char _cnt;
     char _cnt_old;
     char _cnt_more2;
     char _buff[20]; 
     
     
     
     
     signed _Uii; 
     signed _Uin;
     signed _Ii;
     signed _Ti; 
     char _flags_tu;
	 signed _Uisum;
     
     
     
     
     
     signed _vol_u;
     signed _vol_i;
     char _is_on_cnt;
     
     int _ist_blok_host_cnt;
     short _blok_cnt; 
     char _flags_tm;
	signed short _overload_av_cnt;     
     signed short _temp_av_cnt;
     signed short _umax_av_cnt;
     signed short _umin_av_cnt;
     signed _rotor;
     signed  short _x_; 
     char _adr_ee;
	char _last_avar;
	char _vent_resurs_temp[4];
	unsigned short _vent_resurs;
	signed short debug_info_to_uku0;
	signed short debug_info_to_uku1;
	signed short _avg;
	signed short _cntrl_stat;
     } BPS_STAT; 
extern BPS_STAT bps[29];



typedef struct
     {
	char _av;
	
	
	
	
     enum {isAPV,isWRK,isRDY,isBL,isAV,isOFF_AV_NET}_state;
     char _cnt;
     char _cnt_old;
     char _cnt_more2;
     char _buff[16]; 
     signed _Uio; 
     signed _Ii;
     signed _Ti; 
	signed _Uil;
	signed _Uin;
	signed _Pio;
     char _flags_tu;
     signed _vol_u;
     signed _vol_i;
     char _is_on_cnt;
     
     int _ist_blok_host_cnt;
     short _blok_cnt; 
     char _flags_tm;
	char _flags_tm_old;
	signed short _overload_av_cnt;     
     signed short _temp_av_cnt;
     signed short _umax_av_cnt;
     signed short _umin_av_cnt;
     signed _rotor;
     signed  short _x_; 
     char _adr_ee;
	char _last_avar;
	char _Pnom;
	char _Uoutmin;
	char _Uoutmax;
	char _net_contr_en;
	char _pwm_en;
	char _phase_mode;

     } INV_STAT; 




extern INV_STAT inv[20];

extern char first_inv_slot;



extern signed short load_U;
extern signed short load_I;



extern signed short bps_U;
extern signed short out_U;
extern signed short in_U;
extern signed short out_I;
extern signed short vd_U;
extern signed short bps_I;
extern signed short sys_T;



extern signed short net_U,net_Ustore,net_Ua,net_Ub,net_Uc;
extern char bFF,bFF_;
extern signed short net_F,hz_out,hz_out_cnt,net_F3;
extern signed char unet_drv_cnt;
extern char net_av;


extern char plazma_plazma_plazma;

void bitmap_hndl(void);
void ind_hndl(void);
__irq void timer1_interrupt(void);
__irq void timer0_interrupt(void); 





extern char tout_max_cnt[4],tout_min_cnt[4];
typedef enum {tNORM,tMAX,tMIN}enum_tout_stat;
extern enum_tout_stat tout_stat[4];
extern signed short t_ext[3];
extern char ND_EXT[3];
extern signed char sk_cnt[4],sk_av_cnt[4];
typedef enum  {ssOFF,ssON} enum_sk_stat;
extern enum_sk_stat sk_stat[4];
typedef enum  {sasOFF,sasON} enum_sk_av_stat;
extern enum_sk_av_stat sk_av_stat[4],sk_av_stat_old[4];
extern signed short t_box,t_box_warm,t_box_vent;
extern char TELECORE2017_EXT_VENT_PWM,TELECORE2017_INT_VENT_PWM;



extern BOOL bSILENT;



typedef enum {tstOFF,tst1,tst2} enum_tst_state;
extern enum_tst_state tst_state[15];



extern char sign_U[2],sign_I[2];
extern char superviser_cnt;



extern unsigned short adc_buff_ext_[3];
extern unsigned short Uvv[3];
extern unsigned short Uvv0;
extern short pos_vent;
extern short t_ext_can;
extern char t_ext_can_nd;



extern char eb2_data[30];
extern short eb2_data_short[10];
extern short Uvv_eb2[3],Upes_eb2[3];
extern short Kvv_eb2[3],Kpes_eb2[3];



extern signed short vvod_pos;



extern signed long power_summary;
extern signed short power_current;
extern signed long power_summary_tempo,power_summary_tempo_old;
extern signed short power_current_tempo,power_current_tempo_old;
extern char powerSummaryCnt;
extern char powerCurrentCnt;


extern char bRESET;
extern char bRESET_EXT;
extern char bRESET_INT_WDT;
extern char bRESET_EXT_WDT;



extern signed short main_vent_pos;
extern signed char t_box_cnt;
typedef enum  {mvsOFF,mvsON} enum_mixer_vent_stat;
extern enum_mixer_vent_stat mixer_vent_stat;
typedef struct
     {
	signed short _T[4];
	char _nd[4];
	signed short _T_dispers[4];
	char _max_dispers_num;
	signed short _max_dispers;
    	signed short _avg1;
	signed short _avg2;
	char _avg_cnt;
     } INT_BOX_TEMPER;
extern INT_BOX_TEMPER ibt;
typedef enum {tbdsON,tbdsOFF,tbdsMNL} enum_tbatdisable_stat;
extern enum_tbatdisable_stat tbatdisable_stat;
typedef enum {tldsON,tldsOFF,tldsMNL} enum_tloaddisable_stat;
extern enum_tloaddisable_stat tloaddisable_stat;
typedef enum {atsOFF,atsON} enum_av_tbox_stat;
extern enum_av_tbox_stat av_tbox_stat;
extern signed short av_tbox_cnt;
extern char tbatdisable_cmnd,tloaddisable_cmnd;
extern short tbatdisable_cnt,tloaddisable_cnt;
#line 1459 "main.h"

#line 1470 "main.h"

#line 1486 "main.h"

extern char ext_can_cnt;


signed short abs_pal(signed short in);
void ADC_IRQHandler(void);




typedef enum  {avtOFF,avtON} enum_avt_stat;
extern enum_avt_stat avt_stat[12],avt_stat_old[12]; 



extern signed long ibat_metr_buff_[2];
extern short bIBAT_SMKLBR;
extern char ibat_metr_cnt;



extern signed short npn_tz_cnt;
typedef enum {npnsOFF=0,npnsON} enum_npn_stat;
extern enum_npn_stat npn_stat;

extern char snmp_plazma;


extern char ips_bat_av_vzvod;
extern char ips_bat_av_stat;

extern char rel_warm_plazma;
extern char can_byps_plazma0,can_byps_plazma1;

extern short plazma_bat_drv0,plazma_bat_drv1,bat_drv_cnt_cnt;
extern unsigned short bat_drv_rx_cnt;
extern char bat_drv_rx_buff[512];
extern char bat_drv_rx_in;

extern short can_plazma;



#line 1540 "main.h"












extern signed short speedChrgCurr;			
extern signed short speedChrgVolt;			
extern signed short speedChrgTimeInHour; 	
extern signed short speedChrgAvtEn;	 		
extern signed short speedChrgDU;	    	
extern signed short speedChIsOn;			
extern signed long  speedChTimeCnt;			
extern signed short speedChrgBlckSrc;		
extern signed short speedChrgBlckLog;		
extern signed short speedChrgBlckStat;		
extern char  		speedChrgShowCnt;		



extern signed short ipsBlckSrc;
extern signed short ipsBlckLog;
extern signed short ipsBlckStat;



extern signed short outVoltContrHndlCnt;		
extern signed short outVoltContrHndlCnt_;		
extern char uout_av;



extern char bVDISWORK;
extern char vd_is_work_cnt;

extern short plazma_numOfCells;
extern short plazma_numOfTemperCells;
extern short plazma_numOfPacks;

extern char plazma_ztt[2];
extern char plazma1809;

extern U8 socket_tcp;



extern char ica_plazma[10];
extern char ica_timer_cnt;
extern signed short ica_my_current;
extern signed short ica_your_current;
extern signed short ica_u_necc;
extern U8 tcp_soc_avg;
extern U8 tcp_connect_stat;



extern short overloadHndlCnt;
extern char overloadAvar;



extern short uAvarHndlOutUMaxCnt,uAvarHndlOutUMinCnt;
extern char uOutAvar;



extern short uAvarHndlInUMaxCnt,uAvarHndlInUMinCnt;
extern char uInAvar;



extern short sysTAvarHndlCnt;
extern char sysTAvar;



extern short rele_av_flags;
extern short avar_vd_stat;

extern short pvlk;






 
#line 7 "avar_hndl.c"
#line 1 "control.h"







extern char num_of_wrks_bps;
extern char bps_all_off_cnt,bps_mask_off_cnt,bps_mask_on_off_cnt;
extern char bps_hndl_2sec_cnt;
extern unsigned short bps_on_mask,bps_off_mask;
extern char num_necc_up,num_necc_down;
extern unsigned char sh_cnt0,b1Hz_sh;


extern short cntrl_stat_blok_cnt,cntrl_stat_blok_cnt_,cntrl_stat_blok_cnt_plus[2],cntrl_stat_blok_cnt_minus[2];



extern long adc_buff[16][16];
extern signed short adc_buff_max[12],adc_buff_min[12],unet_buff_max,unet_buff_min;
extern short adc_buff_[16];
extern char adc_self_ch_cnt,adc_ch_net;
extern char adc_cnt,adc_cnt1,adc_ch,adc_ch_cnt;
extern short zero_cnt;
typedef enum {asCH=1,asNET_WAIT=2,asNET_RDY=3,asNET=4} enum_adc_stat;
extern enum_adc_stat adc_stat;
extern unsigned short net_buff[32],net_buff_,net_metr_buff_[3];
extern char net_buff_cnt;
extern short ADWR,period_cnt,non_zero_cnt;
extern char rele_stat;
extern char bRELE_OUT;
extern short plazma_adc_cnt;
extern signed short adc_self_ch_buff[3],adc_self_ch_disp[3];
extern long main_power_buffer[8],main_power_buffer_;
extern short main_power_buffer_cnt;
extern short adc_gorb_cnt,adc_zero_cnt;
extern char adc_window_flag;
extern short adc_window_cnt;
extern short adc_net_buff_cnt;


char vz_start(char hour);
void vz_stop(void);

void samokalibr_init(void);
void samokalibr_hndl(void);
void kb_init(void);
void kb_hndl(void);
void ubat_old_drv(void);
void unet_drv(void);
void matemat(void);
void adc_init(void);
void adc_drv5(void);
void adc_drv_(void);
void avg_hndl(void);


void rele_hndl(void);
void bps_hndl(void);
void bps_drv(char in);
void bat_hndl(void);
void bat_drv(char in);
void u_necc_hndl(void);
void cntrl_hndl(void);
void zar_drv(void);
void num_necc_hndl(void);
void ke_start(char in);
void zar_drv(void);
void vent_hndl(void);
void avz_next_date_hndl(void);
void klimat_hndl(void);
void ext_drv(void);
void adc_drv7(void);
void avt_hndl(void);
void vent_resurs_hndl(void);
void ips_current_average_hndl(void);




typedef enum {spcOFF=0,spcKE, spcVZ}enum_spc_stat;
typedef enum {kssNOT=0,kssNOT_VZ,kssYES=100,kssNOT_BAT,kssNOT_BAT_AV,kssNOT_BAT_AV_T,kssNOT_BAT_AV_ASS,kssNOT_BAT_ZAR,kssNOT_BAT_RAZR,kssNOT_KE1,kssNOT_KE2}enum_ke_start_stat;
extern enum_spc_stat spc_stat;
extern enum_ke_start_stat ke_start_stat;
extern char spc_bat;
extern char spc_phase;
extern unsigned short vz_cnt_s,vz_cnt_s_,vz_cnt_h,vz_cnt_h_;
extern short cnt_end_ke;
extern unsigned long ke_date[2];
extern short __ee_vz_cnt;
extern short __ee_spc_stat;
extern short __ee_spc_bat;
extern short __ee_spc_phase;



extern signed short cntrl_stat;
extern signed short cntrl_stat_old;
extern signed short cntrl_stat_new;
extern signed short Ibmax;
extern unsigned char unh_cnt0,unh_cnt1,b1Hz_unh;
extern unsigned char	ch_cnt0,b1Hz_ch,i,iiii;
extern unsigned char	ch_cnt1,b1_30Hz_ch;
extern unsigned char	ch_cnt2,b1_10Hz_ch;
extern unsigned short IZMAX_;
extern unsigned short IZMAX_70;
extern unsigned short IZMAX_130;
extern unsigned short Ubpsmax;
extern unsigned short cntrl_stat_blck_cnt;

extern short plazma_sk;
extern char	plazma_inv[4];
extern char plazma_bat;
extern char plazma_cntrl_stat;


extern signed int i_avg_max,i_avg_min,i_avg_summ,i_avg; 
extern signed int avg;
extern char bAVG;
extern const char sk_buff_TELECORE2015[4];



extern signed short 	main_kb_cnt;
extern signed short 	kb_cnt_1lev;
extern signed short 	kb_cnt_2lev;
extern char 		kb_full_ver;
extern char kb_start[2],kb_start_ips;
extern signed short ibat_ips,ibat_ips_;



extern char numOfForvardBps,numOfForvardBps_old;
extern char numOfForvardBps_minCnt;
extern short numOfForvardBps_hourCnt;



extern char bPARALLEL_NOT_ENOUG;
extern char bPARALLEL_ENOUG;
extern char bPARALLEL;

extern char bAVG_BLOCK;

extern char cntrl_hndl_plazma;

void zar_superviser_drv(void);
void zar_superviser_start(void);
void vent_hndl(void);
void speedChargeHndl(void);
void speedChargeStartStop(void);
void numOfForvardBps_init(void);
void outVoltContrHndl(void);


#line 8 "avar_hndl.c"
#line 1 "snmp_data_file.h"
extern char snmp_community[10];


extern signed short snmp_device_code;
extern signed 	   snmp_sernum;
extern signed short snmp_sernum_lsb;
extern signed short snmp_sernum_msb;
extern char 	   snmp_location[100];
extern signed short snmp_numofbat;
extern signed short snmp_numofbps;
extern signed short snmp_numofinv;
extern signed short snmp_numofavt;
extern signed short snmp_numofdt;
extern signed short snmp_numofsk;
extern signed short snmp_numofevents;


extern signed short snmp_mains_power_voltage;
extern signed short snmp_mains_power_frequency;
extern signed short snmp_mains_power_status;
extern signed short snmp_mains_power_alarm;
extern signed short snmp_mains_power_voltage_phaseA;
extern signed short snmp_mains_power_voltage_phaseB;
extern signed short snmp_mains_power_voltage_phaseC;


extern signed short snmp_load_voltage;
extern signed short snmp_load_current;


extern signed short snmp_bps_number[8];
extern signed short snmp_bps_voltage[8];
extern signed short snmp_bps_current[8];
extern signed short snmp_bps_temperature[8];
extern signed short snmp_bps_stat[8];


extern signed short snmp_inv_number[3];
extern signed short snmp_inv_voltage[3];
extern signed short snmp_inv_current[3];
extern signed short snmp_inv_temperature[3];
extern signed short snmp_inv_stat[3];


extern signed short snmp_bat_number[2];
extern signed short snmp_bat_voltage[2];
extern signed short snmp_bat_part_voltage[2];
extern signed short snmp_bat_current[2];
extern signed short snmp_bat_temperature[2];
extern signed short snmp_bat_capacity[2];
extern signed short snmp_bat_charge[2];
extern signed short snmp_bat_status[2];


extern signed short snmp_makb_number[4];
extern signed short snmp_makb_connect_status[4];
extern signed short snmp_makb_voltage0[4];
extern signed short snmp_makb_voltage1[4];
extern signed short snmp_makb_voltage2[4];
extern signed short snmp_makb_voltage3[4];
extern signed short snmp_makb_voltage4[4];
extern signed short snmp_makb_temper0[4];
extern signed short snmp_makb_temper1[4];
extern signed short snmp_makb_temper2[4];
extern signed short snmp_makb_temper3[4];
extern signed short snmp_makb_temper4[4];
extern signed short snmp_makb_temper0_stat[4];
extern signed short snmp_makb_temper1_stat[4];
extern signed short snmp_makb_temper2_stat[4];
extern signed short snmp_makb_temper3_stat[4];
extern signed short snmp_makb_temper4_stat[4];
extern signed short snmp_bat_voltage[2];
extern signed short snmp_bat_current[2];
extern signed short snmp_bat_temperature[2];
extern signed short snmp_bat_capacity[2];
extern signed short snmp_bat_charge[2];
extern signed short snmp_bat_status[2]; 



extern signed short snmp_spc_stat;
extern char snmp_spc_trap_message[100];
extern signed short snmp_spc_trap_value_0,snmp_spc_trap_value_1,snmp_spc_trap_value_2;


extern signed short snmp_energy_vvod_phase_a;
extern signed short snmp_energy_vvod_phase_b;
extern signed short snmp_energy_vvod_phase_c;
extern signed short snmp_energy_pes_phase_a;
extern signed short snmp_energy_pes_phase_b;
extern signed short snmp_energy_pes_phase_c;
extern signed short snmp_energy_input_voltage;


extern signed long snmp_energy_total_energy;
extern signed short snmp_energy_current_energy;


extern signed char snmp_sk_number[4];
extern signed char snmp_sk_aktiv[4];
extern signed char snmp_sk_alarm_aktiv[4];
extern signed char snmp_sk_alarm[4];
extern char snmp_sk_name[4][20];


extern signed char snmp_dt_number[3];
extern signed short snmp_dt_temper[3];
extern signed char snmp_dt_error[3];


extern signed char snmp_avt_number[12];
extern signed char snmp_avt_stat[12];


extern signed short snmp_command;
extern signed short snmp_command_parametr;


extern char snmp_log[64][128];


extern signed short snmp_main_bps;
extern signed short snmp_zv_en;
extern signed short snmp_alarm_auto_disable;
extern signed short snmp_bat_test_time;
extern signed short snmp_u_max;
extern signed short snmp_u_min;
extern signed short snmp_u_0_grad;
extern signed short snmp_u_20_grad;
extern signed short snmp_u_sign;
extern signed short snmp_u_min_power;
extern signed short snmp_u_withouth_bat;
extern signed short snmp_control_current;
extern signed short snmp_max_charge_current;
extern signed short snmp_max_current;
extern signed short snmp_min_current;
extern signed short snmp_uvz;
extern signed short snmp_max_current_koef;
extern signed short snmp_max_current_koef;
extern signed short snmp_up_charge_koef;
extern signed short snmp_powerup_psu_timeout;
extern signed short snmp_max_temperature;
extern signed short snmp_tsign_bat; 
extern signed short snmp_tmax_bat;
extern signed short snmp_tsign_bps;
extern signed short snmp_tmax_bps;
extern signed short snmp_bat_part_alarm;
extern signed short snmp_power_cnt_adress;


extern signed short snmp_klimat_box_temper;
extern signed short snmp_klimat_settings_box_alarm;
extern signed short snmp_klimat_settings_vent_on;
extern signed short snmp_klimat_settings_vent_off;
extern signed short snmp_klimat_settings_warm_on;
extern signed short snmp_klimat_settings_warm_off;
extern signed short snmp_klimat_settings_load_on;
extern signed short snmp_klimat_settings_load_off;
extern signed short snmp_klimat_settings_batt_on;
extern signed short snmp_klimat_settings_batt_off;


extern signed short snmp_dt_ext;
extern signed short snmp_dt_msan;
extern signed short snmp_dt_epu;


extern short snmp_lakb_number[7];				
extern short snmp_lakb_voltage[7];				
extern short snmp_lakb_max_cell_voltage[7];		
extern short snmp_lakb_min_cell_voltage[7];		
extern short snmp_lakb_max_cell_temperature[7];	
extern short snmp_lakb_min_cell_temperature[7];	
extern short snmp_lakb_ch_curr[7];				
extern short snmp_lakb_dsch_curr[7];			
extern short snmp_lakb_rat_cap[7];				
extern short snmp_lakb_soh[7];				
extern short snmp_lakb_soc[7];				
extern short snmp_lakb_cclv[7];  				
extern short snmp_lakb_rbt[7];				
extern short snmp_lakb_flags1[7];				
extern short snmp_lakb_flags2[7];				
extern char snmp_lakb_damp1[3][150];				
extern char snmp_lakb_damp2[100];				
extern signed char	snmp_lakb_cell_temperature_1[3];		
extern signed char	snmp_lakb_cell_temperature_2[3];		
extern signed char	snmp_lakb_cell_temperature_3[3];		
extern signed char	snmp_lakb_cell_temperature_4[3];		
extern signed char	snmp_lakb_cell_temperature_ambient[3];	
extern signed char	snmp_lakb_cell_temperature_power[3];	


extern signed char	snmp_warm_sign;				
extern signed char	snmp_cool_sign;				
extern signed char	snmp_warm_on_temper;		
extern signed char	snmp_warm_off_temper;		
extern signed char	snmp_warm_q;				
extern signed char	snmp_cool_100_temper;		
extern signed char	snmp_cool_80_temper;		
extern signed char	snmp_cool_60_temper;		
extern signed char	snmp_cool_40_temper;		
extern signed char	snmp_cool_20_temper;		
extern signed char	snmp_cool_100_dtemper;		
extern signed char	snmp_cool_80_dtemper;		
extern signed char	snmp_cool_60_dtemper;		
extern signed char	snmp_cool_40_dtemper;		
extern signed char	snmp_cool_20_dtemper;		
extern signed char 	snmp_warm_stat;				
  

void snmp_data (void);
void snmp_sernum_write (int mode); 
void snmp_location_write (int mode);
void snmp_command_execute (int mode);
void event2snmp(char num);
void snmp_main_bps_write (int mode);
void snmp_zv_on_write (int mode);
void snmp_alarm_auto_disable_write (int mode);
void snmp_bat_test_time_write (int mode);
void snmp_u_max_write (int mode);
void snmp_u_min_write (int mode);
void snmp_u_0_grad_write (int mode);
void snmp_u_20_grad_write (int mode);
void snmp_u_sign_write (int mode);
void snmp_u_min_power_write (int mode);
void snmp_u_withouth_bat_write (int mode);
void snmp_control_current_write (int mode);
void snmp_max_charge_current_write (int mode);
void snmp_max_current_write (int mode);
void snmp_min_current_write (int mode);
void snmp_up_charge_koef_write (int mode);
void snmp_powerup_psu_timeout_write (int mode);
void snmp_max_temperature_write (int mode);
void event2snmp(char num);
void snmp_trap_send(char* str, signed short in0, signed short in1, signed short in2);
void snmp_alarm_aktiv_write1(int mode);
void snmp_alarm_aktiv_write2(int mode);
void snmp_alarm_aktiv_write3(int mode);
void snmp_alarm_aktiv_write4(int mode);
void snmp_klimat_settings_box_alarm_write(int mode);
void snmp_klimat_settings_vent_on_write(int mode);
void snmp_klimat_settings_vent_off_write(int mode);
void snmp_klimat_settings_warm_on_write(int mode);
void snmp_klimat_settings_warm_off_write(int mode);
void snmp_klimat_settings_load_on_write(int mode);
void snmp_klimat_settings_load_off_write(int mode);
void snmp_klimat_settings_batt_on_write(int mode);
void snmp_klimat_settings_batt_off_write(int mode);
void snmp_tsign_bat_write(int mode);
void snmp_tmax_bat_write(int mode);
void snmp_tsign_bps_write(int mode);
void snmp_tmax_bps_write(int mode);
void snmp_bat_part_alarm_write(int mode);
void snmp_power_cnt_adress_write(int mode);
void snmp_uvz_write(int mode);
void snmp_warm_sign_write(int mode);
void snmp_cool_sign_write(int mode);
void snmp_warm_on_temper_write(int mode);
void snmp_warm_off_temper_write(int mode);
void snmp_warm_q_write(int mode);
void snmp_cool_100_temper_write(int mode);
void snmp_cool_80_temper_write(int mode);
void snmp_cool_60_temper_write(int mode);
void snmp_cool_40_temper_write(int mode);
void snmp_cool_20_temper_write(int mode);
void snmp_cool_100_dtemper_write(int mode);
void snmp_cool_80_dtemper_write(int mode);
void snmp_cool_60_dtemper_write(int mode);
void snmp_cool_40_dtemper_write(int mode);
void snmp_cool_20_dtemper_write(int mode);





 
#line 9 "avar_hndl.c"

     


unsigned avar_stat;	 	
unsigned avar_ind_stat; 	
unsigned avar_stat_old;
unsigned avar_stat_new,avar_stat_offed;








char    av_inv[6];

extern char bOUT_FREE2;	





















extern signed short net_U,net_Ustore;
extern char bFF,bFF_;
extern signed short net_F,hz_out,hz_out_cnt;
extern signed char unet_drv_cnt;
extern char net_av;




void avar_hndl(void)
{

char i;


if(net_av)		avar_stat = ( (avar_stat & ~((0xffffffff>>(32-1))<<0)) | (1 << 0) );
else	   			avar_stat = ( (avar_stat & ~((0xffffffff>>(32-1))<<0)) | (0 << 0) );

for(i=0;i<2;i++)
	{
	if(bat[i]._av&1)	avar_stat = ( (avar_stat & ~((0xffffffff>>(32-1))<<1+i)) | (1 << 1+i) );
	else	   		avar_stat = ( (avar_stat & ~((0xffffffff>>(32-1))<<1+i)) | (0 << 1+i) );
	}

for(i=0;i<12;i++)
	{
	if(bps[i]._av&0xef)	avar_stat = ( (avar_stat & ~((0xffffffff>>(32-1))<<3+i)) | (1 << 3+i) );
	else	   		avar_stat = ( (avar_stat & ~((0xffffffff>>(32-1))<<3+i)) | (0 << 3+i) );
	}

for(i=0;i<6;i++)
	{
	if(av_inv[i])	avar_stat = ( (avar_stat & ~((0xffffffff>>(32-1))<<15+i)) | (1 << 15+i) );
	else	   		avar_stat = ( (avar_stat & ~((0xffffffff>>(32-1))<<15+i)) | (0 << 15+i) );
	}





 
for(i=0;i<4;i++)
	{
	if(sk_av_stat[i]==sasON)	avar_stat = ( (avar_stat & ~((0xffffffff>>(32-1))<<24+i)) | (1 << 24+i) );
	else	   		avar_stat = ( (avar_stat & ~((0xffffffff>>(32-1))<<24+i)) | (0 << 24+i) );
	}

if(uout_av)			avar_stat = ( (avar_stat & ~((0xffffffff>>(32-1))<<28)) | (1 << 28) );
else	   			avar_stat = ( (avar_stat & ~((0xffffffff>>(32-1))<<28)) | (0 << 28) );


avar_stat_new=(avar_stat^avar_stat_old)&avar_stat;

avar_ind_stat|=avar_stat_new;

if((SK_ZVUK_EN[0])) avar_ind_stat&=(~(1UL<<24));
if((SK_ZVUK_EN[1])) avar_ind_stat&=(~(1UL<<25));
if((SK_ZVUK_EN[2])) avar_ind_stat&=(~(1UL<<26));
if((SK_ZVUK_EN[3])) avar_ind_stat&=(~(1UL<<27));	


avar_stat_offed=~((avar_stat^avar_stat_old)&avar_stat_old);

if(!AV_OFF_AVT)avar_stat_offed|=0xeffffffe;

avar_ind_stat&=avar_stat_offed; 

avar_stat_old=avar_stat;
}


void reload_hndl(void)
{
char data[4];
unsigned int event_ptr,lc640_adr ,event_cnt;

event_ptr=lc640_read_int(1024+1024+512+1024);
event_ptr++;	
if(event_ptr>63)event_ptr=0;	
lc640_write_int(1024+1024+512+1024,event_ptr);	
	
event_cnt=lc640_read_int(1024+1024+512+1024+2);
if(event_cnt!=63)event_cnt=event_ptr;
lc640_write_int(1024+1024+512+1024+2,event_cnt); 
	
lc640_adr=1024+(lc640_read_int(1024+1024+512+1024)*32);
	
data[0]='U';
data[1]=0;
data[2]='R';
data[3]=0;
lc640_write_long_ptr(lc640_adr,data);

data[0]=0;
data[1]=0;
data[2]=0;
data[3]=0;
lc640_write_long_ptr(lc640_adr+4,data);

data[0]=((LPC_RTC_TypeDef *) ((0x40000000UL) + 0x24000) )->YEAR;
data[1]=((LPC_RTC_TypeDef *) ((0x40000000UL) + 0x24000) )->MONTH;
data[2]=((LPC_RTC_TypeDef *) ((0x40000000UL) + 0x24000) )->DOM;
data[3]=0;
lc640_write_long_ptr(lc640_adr+8,data);

data[0]=((LPC_RTC_TypeDef *) ((0x40000000UL) + 0x24000) )->HOUR;
data[1]=((LPC_RTC_TypeDef *) ((0x40000000UL) + 0x24000) )->MIN;
data[2]=((LPC_RTC_TypeDef *) ((0x40000000UL) + 0x24000) )->SEC;
data[3]=((LPC_SC_TypeDef *) ((0x40080000UL) + 0x7C000) )->RSID;

lc640_write_long_ptr(lc640_adr+12,data);
	
data[0]='A';
data[1]='A';
data[2]='A';
data[3]='A';
lc640_write_long_ptr(lc640_adr+16,data);
	
data[0]='A';
data[1]='A';
data[2]='A';
data[3]='A';
lc640_write_long_ptr(lc640_adr+20,data);
	
data[0]='A';
data[1]='A';
data[2]='A';
data[3]='A';
lc640_write_long_ptr(lc640_adr+24,data);
	
data[0]='A';
data[1]='A';
data[2]='A';
data[3]='A';
lc640_write_long_ptr(lc640_adr+28,data);				
	
	
}



















































































































































 


void avar_bps_hndl(char dev, char v, char in)
{
char data[4];
unsigned short event_ptr,lc640_adr,event_ptr_find,event_cnt;
char avar_simbol;

avar_simbol='T';
if(v==0)avar_simbol='T';
else if(v==1)avar_simbol='U';
else if(v==2)avar_simbol='u';
else if(v==3)avar_simbol='L';

if(in==1)
	{
	



	
	if(v==0)bps[dev]._av|=(1<<0);
	else if(v==1) bps[dev]._av|=(1<<1);
	else if(v==2) bps[dev]._av|=(1<<2);
	else if(v==3) bps[dev]._av|=(1<<3);

	bps[dev]._last_avar=v;

	

	event_ptr=lc640_read_int(1024+1024+512+1024);
	event_ptr++;	
	if(event_ptr>63)event_ptr=0;	
	lc640_write_int(1024+1024+512+1024,event_ptr);	
	
     event_cnt=lc640_read_int(1024+1024+512+1024+2);
	if(event_cnt!=63)event_cnt=event_ptr;
	lc640_write_int(1024+1024+512+1024+2,event_cnt); 
	
	lc640_adr=1024+(lc640_read_int(1024+1024+512+1024)*32);
	
	data[0]='S';
	data[1]=dev; 
	data[2]=avar_simbol;
	data[3]=0;
	lc640_write_long_ptr(lc640_adr,data);

	data[0]=0;
	data[1]=0;
	data[2]=0;
	data[3]=0;
	lc640_write_long_ptr(lc640_adr+4,data);

	data[0]=((LPC_RTC_TypeDef *) ((0x40000000UL) + 0x24000) )->YEAR;
	data[1]=((LPC_RTC_TypeDef *) ((0x40000000UL) + 0x24000) )->MONTH;
	data[2]=((LPC_RTC_TypeDef *) ((0x40000000UL) + 0x24000) )->DOM;
	data[3]=0;
	lc640_write_long_ptr(lc640_adr+8,data);

	data[0]=((LPC_RTC_TypeDef *) ((0x40000000UL) + 0x24000) )->HOUR;
	data[1]=((LPC_RTC_TypeDef *) ((0x40000000UL) + 0x24000) )->MIN;
	data[2]=((LPC_RTC_TypeDef *) ((0x40000000UL) + 0x24000) )->SEC;
	data[3]=0;
	lc640_write_long_ptr(lc640_adr+12,data);
	
	data[0]='A';
	data[1]='A';
	data[2]='A';
	data[3]='A';
	lc640_write_long_ptr(lc640_adr+16,data);
	
	data[0]='A';
	data[1]='A';
	data[2]='A';
	data[3]='A';
	lc640_write_long_ptr(lc640_adr+20,data);
	
	data[0]='A';
	data[1]='A';
	data[2]='A';
	data[3]='A';
	lc640_write_long_ptr(lc640_adr+24,data);
	
	data[0]='A';
	data[1]='A';
	data[2]='A';
	data[3]='A';
	lc640_write_long_ptr(lc640_adr+28,data);		


	if(dev==0)
		{
		if(v==0)
		snmp_trap_send("BPS #1 Alarm, overheat",4,1,0);
		else if(v==1)
		snmp_trap_send("BPS #1 Alarm, voltage is up",4,1,1);
		else if(v==2)
		snmp_trap_send("BPS #1 Alarm, voltage is down",4,1,2);
		else if(v==3)
		snmp_trap_send("BPS #1 Alarm, connect is lost",4,1,3);
		}
	else if(dev==1)
		{
		if(v==0)
		snmp_trap_send("BPS #2 Alarm, overheat",4,2,0);
		else if(v==1)
		snmp_trap_send("BPS #2 Alarm, voltage is up",4,2,1);
		else if(v==2)
		snmp_trap_send("BPS #2 Alarm, voltage is down",4,2,2);
		else if(v==3)
		snmp_trap_send("BPS #2 Alarm, connect is lost",4,2,3);
		} 
	else if(dev==2)
		{
		if(v==0)
		snmp_trap_send("BPS #3 Alarm, overheat",4,3,0);
		else if(v==1)
		snmp_trap_send("BPS #3 Alarm, voltage is up",4,3,1);
		else if(v==2)
		snmp_trap_send("BPS #3 Alarm, voltage is down",4,3,2);
		else if(v==3)
		snmp_trap_send("BPS #3 Alarm, connect is lost",4,3,3);
		} 	
			
	}

else if(in==0)
	{      


	
	

     if(v==0) bps[dev]._av&=(~(1<<0));
     else if(v==1) bps[dev]._av&=(~(1<<1));
	else if(v==2) bps[dev]._av&=(~(1<<2));
	else if(v==3) bps[dev]._av&=(~(1<<3));
     


		
		

 
	event_ptr=lc640_read_int(1024+1024+512+1024);
	event_ptr_find=event_ptr;
	
avar_src_hndl_lbl1: 

	lc640_adr=1024+(event_ptr_find*32);

     lc640_read_long_ptr(lc640_adr,data);
     
     if(!((data[0]=='S')&&(data[1]==dev)&&(data[2]==avar_simbol)))
     	{        
     	if(event_ptr_find)event_ptr_find--;
     	else event_ptr_find=63;
     	if(event_ptr_find==event_ptr)goto avar_src_hndl_end;
     	else goto avar_src_hndl_lbl1;
     	}
     else 
     	{
     	lc640_read_long_ptr(lc640_adr+16,data);
     	if(!((data[0]=='A')&&(data[1]=='A')&&(data[2]=='A')&&(data[3]=='A')))
     		{        
     		if(event_ptr_find)event_ptr_find--;
         		else event_ptr_find=63;
         		if(event_ptr_find==event_ptr)goto avar_src_hndl_end;
     		else goto avar_src_hndl_lbl1;
     		}

     	}	


	
	data[0]=((LPC_RTC_TypeDef *) ((0x40000000UL) + 0x24000) )->YEAR;
	data[1]=((LPC_RTC_TypeDef *) ((0x40000000UL) + 0x24000) )->MONTH;
	data[2]=((LPC_RTC_TypeDef *) ((0x40000000UL) + 0x24000) )->DOM;
	data[3]=0;
	lc640_write_long_ptr(lc640_adr+16,data);

	data[0]=((LPC_RTC_TypeDef *) ((0x40000000UL) + 0x24000) )->HOUR;
	data[1]=((LPC_RTC_TypeDef *) ((0x40000000UL) + 0x24000) )->MIN;
	data[2]=((LPC_RTC_TypeDef *) ((0x40000000UL) + 0x24000) )->SEC;
	data[3]=0;
	lc640_write_long_ptr(lc640_adr+20,data);
	
	data[0]='B';
	data[1]='B';
	data[2]='B';
	data[3]='B';
	lc640_write_long_ptr(lc640_adr+24,data);
	
	data[0]='B';
	data[1]='B';
	data[2]='B';
	data[3]='B';
	lc640_write_long_ptr(lc640_adr+28,data);
	
	}
	
avar_src_hndl_end:
__nop();		
}


void wrk_mem_hndl(char b)
{
char data[4];
unsigned short event_ptr,lc640_adr ,event_cnt;

signed short temp_temp;

if(bat[b]._Iintegr_<5) goto wrk_mem_hndl_end;

temp_temp=bat[b]._u_old[((bat_u_old_cnt+6)&0x07)]; 


event_ptr=lc640_read_int(1024+1024+512+1024);
event_ptr++;	
if(event_ptr>63)event_ptr=0;	
lc640_write_int(1024+1024+512+1024,event_ptr);	
	
event_cnt=lc640_read_int(1024+1024+512+1024+2);
if(event_cnt!=63)event_cnt=event_ptr;
lc640_write_int(1024+1024+512+1024+2,event_cnt); 
	
lc640_adr=1024+(lc640_read_int(1024+1024+512+1024)*32);
	
data[0]='B';
data[1]=b; 
data[2]='W';
data[3]=0;

lc640_write_long_ptr(lc640_adr,data);

data[0]=*((char*)&bat[b]._Iintegr_);
data[1]=*(((char*)(&bat[b]._Iintegr_))+1);
data[2]=*((char*)&temp_temp);
data[3]=*(((char*)(&temp_temp))+1);
lc640_write_long_ptr(lc640_adr+4,data);

lc640_write_long_ptr(lc640_adr+8,(char*)&bat[b]._wrk_date[0]);
	
lc640_write_long_ptr(lc640_adr+12,(char*)&bat[b]._wrk_date[1]);

data[0]=((LPC_RTC_TypeDef *) ((0x40000000UL) + 0x24000) )->YEAR;
data[1]=((LPC_RTC_TypeDef *) ((0x40000000UL) + 0x24000) )->MONTH;
data[2]=((LPC_RTC_TypeDef *) ((0x40000000UL) + 0x24000) )->DOM;
data[3]=0;
lc640_write_long_ptr(lc640_adr+16,data);

data[0]=((LPC_RTC_TypeDef *) ((0x40000000UL) + 0x24000) )->HOUR;
data[1]=((LPC_RTC_TypeDef *) ((0x40000000UL) + 0x24000) )->MIN;
data[2]=((LPC_RTC_TypeDef *) ((0x40000000UL) + 0x24000) )->SEC;
data[3]=0;
lc640_write_long_ptr(lc640_adr+20,data);
	


	
wrk_mem_hndl_end:	
__nop();
}  



void avar_bat_ips_hndl(char in)
{
char data[4];
unsigned short event_ptr,lc640_adr,event_ptr_find,event_cnt;

if(in==1)
	{
	bat_ips._av|=1;
    ips_bat_av_stat=1;
	ips_bat_av_vzvod=1;

	event_ptr=lc640_read_int(1024+1024+512+1024);
	event_ptr++;	
	if(event_ptr>63)event_ptr=0;	
	lc640_write_int(1024+1024+512+1024,event_ptr);	
	
     event_cnt=lc640_read_int(1024+1024+512+1024+2);
	if(event_cnt!=63)event_cnt=event_ptr;
	lc640_write_int(1024+1024+512+1024+2,event_cnt); 
	
	lc640_adr=1024+(lc640_read_int(1024+1024+512+1024)*32);
	
	data[0]='B';
	data[1]=0;
	data[2]='C';
	data[3]=0;
	lc640_write_long_ptr(lc640_adr,data);

	data[0]=0;
	data[1]=0;
	data[2]=0;
	data[3]=0;
	lc640_write_long_ptr(lc640_adr+4,data);

	data[0]=((LPC_RTC_TypeDef *) ((0x40000000UL) + 0x24000) )->YEAR;
	data[1]=((LPC_RTC_TypeDef *) ((0x40000000UL) + 0x24000) )->MONTH;
	data[2]=((LPC_RTC_TypeDef *) ((0x40000000UL) + 0x24000) )->DOM;
	data[3]=0;
	lc640_write_long_ptr(lc640_adr+8,data);

	data[0]=((LPC_RTC_TypeDef *) ((0x40000000UL) + 0x24000) )->HOUR;
	data[1]=((LPC_RTC_TypeDef *) ((0x40000000UL) + 0x24000) )->MIN;
	data[2]=((LPC_RTC_TypeDef *) ((0x40000000UL) + 0x24000) )->SEC;
	data[3]=0;
	lc640_write_long_ptr(lc640_adr+12,data);
	
	data[0]='A';
	data[1]='A';
	data[2]='A';
	data[3]='A';
	lc640_write_long_ptr(lc640_adr+16,data);
	
	data[0]='A';
	data[1]='A';
	data[2]='A';
	data[3]='A';
	lc640_write_long_ptr(lc640_adr+20,data);
	
	data[0]='A';
	data[1]='A';
	data[2]='A';
	data[3]='A';
	lc640_write_long_ptr(lc640_adr+24,data);
	
	data[0]='A';
	data[1]='A';
	data[2]='A';
	data[3]='A';
	lc640_write_long_ptr(lc640_adr+28,data);
					
	snmp_trap_send("BAT �1 Alarm, lost",5,1,1);
		
	}

else if(in==0)
	{
	bat_ips._av&=~1;
	ips_bat_av_stat=0;
	ips_bat_av_vzvod=0;
     
	event_ptr=lc640_read_int(1024+1024+512+1024);
	event_ptr_find=event_ptr;
	
avar_bat_ips_hndl_lbl1: 

	lc640_adr=1024+(event_ptr_find*32);

     lc640_read_long_ptr(lc640_adr,data);
     
     if(!((data[0]=='B')&&(data[1]==0)&&(data[2]=='C')))
     	{        
     	if(event_ptr_find)event_ptr_find--;
     	else event_ptr_find=63;
     	if(event_ptr_find==event_ptr)goto avar_bat_ips_hndl_end;
     	else goto avar_bat_ips_hndl_lbl1;
     	}
     else 
     	{
     	lc640_read_long_ptr(lc640_adr+16,data);
     	if(!((data[0]=='A')&&(data[1]=='A')&&(data[2]=='A')&&(data[3]=='A')))
     		{        
     		if(event_ptr_find)event_ptr_find--;
         		else event_ptr_find=63;
         	    	if(event_ptr_find==event_ptr)goto avar_bat_ips_hndl_end;
     		else goto avar_bat_ips_hndl_lbl1;
     		}

     	}
     		
	data[0]=((LPC_RTC_TypeDef *) ((0x40000000UL) + 0x24000) )->YEAR;
	data[1]=((LPC_RTC_TypeDef *) ((0x40000000UL) + 0x24000) )->MONTH;
	data[2]=((LPC_RTC_TypeDef *) ((0x40000000UL) + 0x24000) )->DOM;
	data[3]=0;
	lc640_write_long_ptr(lc640_adr+16,data);

	data[0]=((LPC_RTC_TypeDef *) ((0x40000000UL) + 0x24000) )->HOUR;
	data[1]=((LPC_RTC_TypeDef *) ((0x40000000UL) + 0x24000) )->MIN;
	data[2]=((LPC_RTC_TypeDef *) ((0x40000000UL) + 0x24000) )->SEC;
	data[3]=0;
	lc640_write_long_ptr(lc640_adr+20,data);
	
	data[0]='B';
	data[1]='B';
	data[2]='B';
	data[3]='B';
	lc640_write_long_ptr(lc640_adr+24,data);
	
	data[0]='B';
	data[1]='B';
	data[2]='B';
	data[3]='B';
	lc640_write_long_ptr(lc640_adr+28,data);
	

	
	}
	
avar_bat_ips_hndl_end:
__nop();		
}





void avar_overload_hndl(char in)
{

char data[4];
unsigned short event_ptr,lc640_adr,event_ptr_find,event_cnt;

if(in==1)
	{
	if(overloadAvar==1)return;
    
	overloadAvar=1;

	event_ptr=lc640_read_int(1024+1024+512+1024);
	event_ptr++;	
	if(event_ptr>63)event_ptr=0;	
	lc640_write_int(1024+1024+512+1024,event_ptr);	
	
    event_cnt=lc640_read_int(1024+1024+512+1024+2);
	if(event_cnt!=63)event_cnt=event_ptr;
	lc640_write_int(1024+1024+512+1024+2,event_cnt); 
	
	lc640_adr=1024+(lc640_read_int(1024+1024+512+1024)*32);
	
	data[0]='O';
	data[1]='V';
	data[2]='L';
	data[3]=0;
	lc640_write_long_ptr(lc640_adr,data);

	data[0]=0;
	data[1]=0;
	data[2]=0;
	data[3]=0;
	lc640_write_long_ptr(lc640_adr+4,data);

	data[0]=((LPC_RTC_TypeDef *) ((0x40000000UL) + 0x24000) )->YEAR;
	data[1]=((LPC_RTC_TypeDef *) ((0x40000000UL) + 0x24000) )->MONTH;
	data[2]=((LPC_RTC_TypeDef *) ((0x40000000UL) + 0x24000) )->DOM;
	data[3]=0;
	lc640_write_long_ptr(lc640_adr+8,data);

	data[0]=((LPC_RTC_TypeDef *) ((0x40000000UL) + 0x24000) )->HOUR;
	data[1]=((LPC_RTC_TypeDef *) ((0x40000000UL) + 0x24000) )->MIN;
	data[2]=((LPC_RTC_TypeDef *) ((0x40000000UL) + 0x24000) )->SEC;
	data[3]=0;
	lc640_write_long_ptr(lc640_adr+12,data);
	
	data[0]='A';
	data[1]='A';
	data[2]='A';
	data[3]='A';
	lc640_write_long_ptr(lc640_adr+16,data);
	
	data[0]='A';
	data[1]='A';
	data[2]='A';
	data[3]='A';
	lc640_write_long_ptr(lc640_adr+20,data);
	
	data[0]='A';
	data[1]='A';
	data[2]='A';
	data[3]='A';
	lc640_write_long_ptr(lc640_adr+24,data);
	
	data[0]='A';
	data[1]='A';
	data[2]='A';
	data[3]='A';
	lc640_write_long_ptr(lc640_adr+28,data);				

	snmp_trap_send("System overload",5,1,2);

	
	}

else if(in==0)
	{
	if(overloadAvar==0)return;

	overloadAvar=0;

    event_ptr=lc640_read_int(1024+1024+512+1024);
	event_ptr_find=event_ptr;
	
avar_overload_hndl_lbl1: 

	lc640_adr=1024+(event_ptr_find*32);

	lc640_read_long_ptr(lc640_adr,data);
     
    if(!((data[0]=='O')&&(data[1]=='V')&&(data[2]=='L')))
     	{        
     	if(event_ptr_find)event_ptr_find--;
     	else event_ptr_find=63;
     	if(event_ptr_find==event_ptr)return;
     	else goto avar_overload_hndl_lbl1;
     	}
     else 
     	{
     	lc640_read_long_ptr(lc640_adr+16,data);
     	if(!((data[0]=='A')&&(data[1]=='A')&&(data[2]=='A')&&(data[3]=='A')))
     		{        
     		if(event_ptr_find)event_ptr_find--;
         	else event_ptr_find=63;
         	if(event_ptr_find==event_ptr)return;
     		else goto avar_overload_hndl_lbl1;
     		}

     	}
     		
	data[0]=((LPC_RTC_TypeDef *) ((0x40000000UL) + 0x24000) )->YEAR;
	data[1]=((LPC_RTC_TypeDef *) ((0x40000000UL) + 0x24000) )->MONTH;
	data[2]=((LPC_RTC_TypeDef *) ((0x40000000UL) + 0x24000) )->DOM;
	data[3]=0;
	lc640_write_long_ptr(lc640_adr+16,data);

	data[0]=((LPC_RTC_TypeDef *) ((0x40000000UL) + 0x24000) )->HOUR;
	data[1]=((LPC_RTC_TypeDef *) ((0x40000000UL) + 0x24000) )->MIN;
	data[2]=((LPC_RTC_TypeDef *) ((0x40000000UL) + 0x24000) )->SEC;
	data[3]=0;
	lc640_write_long_ptr(lc640_adr+20,data);
	
	data[0]='B';
	data[1]='B';
	data[2]='B';
	data[3]='B';
	lc640_write_long_ptr(lc640_adr+24,data);
	
	data[0]='B';
	data[1]='B';
	data[2]='B';
	data[3]='B';
	lc640_write_long_ptr(lc640_adr+28,data);
	

	
	}
	
}


void avar_u_out_hndl(char in,short volt)
{

char data[4];
unsigned short event_ptr,lc640_adr,event_ptr_find,event_cnt;
if(uOutAvar==in)return;
if((in==1)||(in==2))
	{
	if(uOutAvar==in)return;
    
	uOutAvar=in;

	event_ptr=lc640_read_int(1024+1024+512+1024);
	event_ptr++;	
	if(event_ptr>63)event_ptr=0;	
	lc640_write_int(1024+1024+512+1024,event_ptr);	
	
    event_cnt=lc640_read_int(1024+1024+512+1024+2);
	if(event_cnt!=63)event_cnt=event_ptr;
	lc640_write_int(1024+1024+512+1024+2,event_cnt); 
	
	lc640_adr=1024+(lc640_read_int(1024+1024+512+1024)*32);
	
	data[0]='V';
	data[1]='O';
	data[2]='U';
	data[3]='T';
	lc640_write_long_ptr(lc640_adr,data);

	data[0]=*((char*)&volt);
	data[1]=*(((char*)&volt)+1);
	data[2]=in;
	data[3]=0;
	lc640_write_long_ptr(lc640_adr+4,data);

	data[0]=((LPC_RTC_TypeDef *) ((0x40000000UL) + 0x24000) )->YEAR;
	data[1]=((LPC_RTC_TypeDef *) ((0x40000000UL) + 0x24000) )->MONTH;
	data[2]=((LPC_RTC_TypeDef *) ((0x40000000UL) + 0x24000) )->DOM;
	data[3]=0;
	lc640_write_long_ptr(lc640_adr+8,data);

	data[0]=((LPC_RTC_TypeDef *) ((0x40000000UL) + 0x24000) )->HOUR;
	data[1]=((LPC_RTC_TypeDef *) ((0x40000000UL) + 0x24000) )->MIN;
	data[2]=((LPC_RTC_TypeDef *) ((0x40000000UL) + 0x24000) )->SEC;
	data[3]=0;
	lc640_write_long_ptr(lc640_adr+12,data);
	
	data[0]='A';
	data[1]='A';
	data[2]='A';
	data[3]='A';
	lc640_write_long_ptr(lc640_adr+16,data);
	
	data[0]='A';
	data[1]='A';
	data[2]='A';
	data[3]='A';
	lc640_write_long_ptr(lc640_adr+20,data);
	
	data[0]='A';
	data[1]='A';
	data[2]='A';
	data[3]='A';
	lc640_write_long_ptr(lc640_adr+24,data);
	
	data[0]='A';
	data[1]='A';
	data[2]='A';
	data[3]='A';
	lc640_write_long_ptr(lc640_adr+28,data);				

	

	
	}

else if(in==0)
	{
	if(uOutAvar==0)return;

	uOutAvar=0;

    event_ptr=lc640_read_int(1024+1024+512+1024);
	event_ptr_find=event_ptr;
	
avar_u_out_hndl_lbl1: 

	lc640_adr=1024+(event_ptr_find*32);

	lc640_read_long_ptr(lc640_adr,data);
     
    if(!((data[0]=='V')&&(data[1]=='O')&&(data[2]=='U')&&(data[3]=='T')))
     	{        
     	if(event_ptr_find)event_ptr_find--;
     	else event_ptr_find=63;
     	if(event_ptr_find==event_ptr)return;
     	else goto avar_u_out_hndl_lbl1;
     	}
     else 
     	{
     	lc640_read_long_ptr(lc640_adr+16,data);
     	if(!((data[0]=='A')&&(data[1]=='A')&&(data[2]=='A')&&(data[3]=='A')))
     		{        
     		if(event_ptr_find)event_ptr_find--;
         	else event_ptr_find=63;
         	if(event_ptr_find==event_ptr)return;
     		else goto avar_u_out_hndl_lbl1;
     		}

     	}
     		
	data[0]=((LPC_RTC_TypeDef *) ((0x40000000UL) + 0x24000) )->YEAR;
	data[1]=((LPC_RTC_TypeDef *) ((0x40000000UL) + 0x24000) )->MONTH;
	data[2]=((LPC_RTC_TypeDef *) ((0x40000000UL) + 0x24000) )->DOM;
	data[3]=0;
	lc640_write_long_ptr(lc640_adr+16,data);

	data[0]=((LPC_RTC_TypeDef *) ((0x40000000UL) + 0x24000) )->HOUR;
	data[1]=((LPC_RTC_TypeDef *) ((0x40000000UL) + 0x24000) )->MIN;
	data[2]=((LPC_RTC_TypeDef *) ((0x40000000UL) + 0x24000) )->SEC;
	data[3]=0;
	lc640_write_long_ptr(lc640_adr+20,data);
	
	data[0]=*((char*)&volt);
	data[1]=*(((char*)&volt)+1);
	data[2]='B';
	data[3]='B';
	lc640_write_long_ptr(lc640_adr+24,data);
	
	data[0]='B';
	data[1]='B';
	data[2]='B';
	data[3]='B';
	lc640_write_long_ptr(lc640_adr+28,data);
	}
	
}


void avar_u_in_hndl(char in,short volt)
{

char data[4];
unsigned short event_ptr,lc640_adr,event_ptr_find,event_cnt;
if(uInAvar==in)return;
if((in==1)||(in==2))
	{
	
    
	uInAvar=in;

	event_ptr=lc640_read_int(1024+1024+512+1024);
	event_ptr++;	
	if(event_ptr>63)event_ptr=0;	
	lc640_write_int(1024+1024+512+1024,event_ptr);	
	
    event_cnt=lc640_read_int(1024+1024+512+1024+2);
	if(event_cnt!=63)event_cnt=event_ptr;
	lc640_write_int(1024+1024+512+1024+2,event_cnt); 
	
	lc640_adr=1024+(lc640_read_int(1024+1024+512+1024)*32);
	
	data[0]='V';
	data[1]='I';
	data[2]='N';
	data[3]=0;
	lc640_write_long_ptr(lc640_adr,data);

	data[0]=*((char*)&volt);
	data[1]=*(((char*)&volt)+1);
	data[2]=in;
	data[3]=0;
	lc640_write_long_ptr(lc640_adr+4,data);

	data[0]=((LPC_RTC_TypeDef *) ((0x40000000UL) + 0x24000) )->YEAR;
	data[1]=((LPC_RTC_TypeDef *) ((0x40000000UL) + 0x24000) )->MONTH;
	data[2]=((LPC_RTC_TypeDef *) ((0x40000000UL) + 0x24000) )->DOM;
	data[3]=0;
	lc640_write_long_ptr(lc640_adr+8,data);

	data[0]=((LPC_RTC_TypeDef *) ((0x40000000UL) + 0x24000) )->HOUR;
	data[1]=((LPC_RTC_TypeDef *) ((0x40000000UL) + 0x24000) )->MIN;
	data[2]=((LPC_RTC_TypeDef *) ((0x40000000UL) + 0x24000) )->SEC;
	data[3]=0;
	lc640_write_long_ptr(lc640_adr+12,data);
	
	data[0]='A';
	data[1]='A';
	data[2]='A';
	data[3]='A';
	lc640_write_long_ptr(lc640_adr+16,data);
	
	data[0]='A';
	data[1]='A';
	data[2]='A';
	data[3]='A';
	lc640_write_long_ptr(lc640_adr+20,data);
	
	data[0]='A';
	data[1]='A';
	data[2]='A';
	data[3]='A';
	lc640_write_long_ptr(lc640_adr+24,data);
	
	data[0]='A';
	data[1]='A';
	data[2]='A';
	data[3]='A';
	lc640_write_long_ptr(lc640_adr+28,data);				

	

	
	}

else if(in==0)
	{
	

	uInAvar=0;

    event_ptr=lc640_read_int(1024+1024+512+1024);
	event_ptr_find=event_ptr;
	
avar_u_in_hndl_lbl1: 

	lc640_adr=1024+(event_ptr_find*32);

	lc640_read_long_ptr(lc640_adr,data);
     
    if(!((data[0]=='V')&&(data[1]=='I')&&(data[2]=='N')&&(data[3]==0)))
     	{        
     	if(event_ptr_find)event_ptr_find--;
     	else event_ptr_find=63;
     	if(event_ptr_find==event_ptr)return;
     	else goto avar_u_in_hndl_lbl1;
     	}
     else 
     	{
     	lc640_read_long_ptr(lc640_adr+16,data);
     	if(!((data[0]=='A')&&(data[1]=='A')&&(data[2]=='A')&&(data[3]=='A')))
     		{        
     		if(event_ptr_find)event_ptr_find--;
         	else event_ptr_find=63;
         	if(event_ptr_find==event_ptr)return;
     		else goto avar_u_in_hndl_lbl1;
     		}

     	}
     		
	data[0]=((LPC_RTC_TypeDef *) ((0x40000000UL) + 0x24000) )->YEAR;
	data[1]=((LPC_RTC_TypeDef *) ((0x40000000UL) + 0x24000) )->MONTH;
	data[2]=((LPC_RTC_TypeDef *) ((0x40000000UL) + 0x24000) )->DOM;
	data[3]=0;
	lc640_write_long_ptr(lc640_adr+16,data);

	data[0]=((LPC_RTC_TypeDef *) ((0x40000000UL) + 0x24000) )->HOUR;
	data[1]=((LPC_RTC_TypeDef *) ((0x40000000UL) + 0x24000) )->MIN;
	data[2]=((LPC_RTC_TypeDef *) ((0x40000000UL) + 0x24000) )->SEC;
	data[3]=0;
	lc640_write_long_ptr(lc640_adr+20,data);
	
	data[0]=*((char*)&volt);
	data[1]=*(((char*)&volt)+1);
	data[2]='B';
	data[3]='B';
	lc640_write_long_ptr(lc640_adr+24,data);
	
	data[0]='B';
	data[1]='B';
	data[2]='B';
	data[3]='B';
	lc640_write_long_ptr(lc640_adr+28,data);
	}
	
}



void avar_sys_t_hndl(char in,short temper)
{

char data[4];
unsigned short event_ptr,lc640_adr,event_ptr_find,event_cnt;
if(sysTAvar==in)return;
if(in==1)
	{
	sysTAvar=in;

	event_ptr=lc640_read_int(1024+1024+512+1024);
	event_ptr++;	
	if(event_ptr>63)event_ptr=0;	
	lc640_write_int(1024+1024+512+1024,event_ptr);	
	
    event_cnt=lc640_read_int(1024+1024+512+1024+2);
	if(event_cnt!=63)event_cnt=event_ptr;
	lc640_write_int(1024+1024+512+1024+2,event_cnt); 
	
	lc640_adr=1024+(lc640_read_int(1024+1024+512+1024)*32);
	
	data[0]='T';
	data[1]='S';
	data[2]='Y';
	data[3]='S';
	lc640_write_long_ptr(lc640_adr,data);

	data[0]=*((char*)&temper);
	data[1]=*(((char*)&temper)+1);
	data[2]=0;
	data[3]=0;
	lc640_write_long_ptr(lc640_adr+4,data);

	data[0]=((LPC_RTC_TypeDef *) ((0x40000000UL) + 0x24000) )->YEAR;
	data[1]=((LPC_RTC_TypeDef *) ((0x40000000UL) + 0x24000) )->MONTH;
	data[2]=((LPC_RTC_TypeDef *) ((0x40000000UL) + 0x24000) )->DOM;
	data[3]=0;
	lc640_write_long_ptr(lc640_adr+8,data);

	data[0]=((LPC_RTC_TypeDef *) ((0x40000000UL) + 0x24000) )->HOUR;
	data[1]=((LPC_RTC_TypeDef *) ((0x40000000UL) + 0x24000) )->MIN;
	data[2]=((LPC_RTC_TypeDef *) ((0x40000000UL) + 0x24000) )->SEC;
	data[3]=0;
	lc640_write_long_ptr(lc640_adr+12,data);
	
	data[0]='A';
	data[1]='A';
	data[2]='A';
	data[3]='A';
	lc640_write_long_ptr(lc640_adr+16,data);
	
	data[0]='A';
	data[1]='A';
	data[2]='A';
	data[3]='A';
	lc640_write_long_ptr(lc640_adr+20,data);
	
	data[0]='A';
	data[1]='A';
	data[2]='A';
	data[3]='A';
	lc640_write_long_ptr(lc640_adr+24,data);
	
	data[0]='A';
	data[1]='A';
	data[2]='A';
	data[3]='A';
	lc640_write_long_ptr(lc640_adr+28,data);				

	

	
	}

else if(in==0)
	{
	

	sysTAvar=0;

    event_ptr=lc640_read_int(1024+1024+512+1024);
	event_ptr_find=event_ptr;
	
avar_sys_t_hndl_lbl1: 

	lc640_adr=1024+(event_ptr_find*32);

	lc640_read_long_ptr(lc640_adr,data);
     
    if(!((data[0]=='T')&&(data[1]=='S')&&(data[2]=='Y')&&(data[3]=='S')))
     	{        
     	if(event_ptr_find)event_ptr_find--;
     	else event_ptr_find=63;
     	if(event_ptr_find==event_ptr)return;
     	else goto avar_sys_t_hndl_lbl1;
     	}
     else 
     	{
     	lc640_read_long_ptr(lc640_adr+16,data);
     	if(!((data[0]=='A')&&(data[1]=='A')&&(data[2]=='A')&&(data[3]=='A')))
     		{        
     		if(event_ptr_find)event_ptr_find--;
         	else event_ptr_find=63;
         	if(event_ptr_find==event_ptr)return;
     		else goto avar_sys_t_hndl_lbl1;
     		}

     	}
     		
	data[0]=((LPC_RTC_TypeDef *) ((0x40000000UL) + 0x24000) )->YEAR;
	data[1]=((LPC_RTC_TypeDef *) ((0x40000000UL) + 0x24000) )->MONTH;
	data[2]=((LPC_RTC_TypeDef *) ((0x40000000UL) + 0x24000) )->DOM;
	data[3]=0;
	lc640_write_long_ptr(lc640_adr+16,data);

	data[0]=((LPC_RTC_TypeDef *) ((0x40000000UL) + 0x24000) )->HOUR;
	data[1]=((LPC_RTC_TypeDef *) ((0x40000000UL) + 0x24000) )->MIN;
	data[2]=((LPC_RTC_TypeDef *) ((0x40000000UL) + 0x24000) )->SEC;
	data[3]=0;
	lc640_write_long_ptr(lc640_adr+20,data);
	
	data[0]=*((char*)&temper);
	data[1]=*(((char*)&temper)+1);
	data[2]='B';
	data[3]='B';
	lc640_write_long_ptr(lc640_adr+24,data);
	
	data[0]='B';
	data[1]='B';
	data[2]='B';
	data[3]='B';
	lc640_write_long_ptr(lc640_adr+28,data);
	}
	
}

