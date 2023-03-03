#include <lpc17xx.h>
#include "MODBUS_func6.h"
#include "curr_version.h"
#include "main.h"
#include "MODBUS_RTU.h"
#include "eeprom_map.h"
#include "control.h"

void analiz_func6(unsigned short mbadr, unsigned short mbdat){
			if(mbadr==11)		//Установка времени 
				{
				LPC_RTC->YEAR=(uint16_t)mbdat;
				}
			else if(mbadr==12)		//Установка времени 
				{
				LPC_RTC->MONTH=(uint16_t)mbdat;
				}
			else if(mbadr==13)		//Установка времени 
				{
				LPC_RTC->DOM=(uint16_t)mbdat;
				}
			else if(mbadr==14)		//Установка времени 
				{
				LPC_RTC->HOUR=(uint16_t)mbdat;
				}
			else if(mbadr==15)		//Установка времени 
				{
				LPC_RTC->MIN=(uint16_t)mbdat;
				}
			else if(mbadr==16)		//Установка времени 
				{
				LPC_RTC->SEC=(uint16_t)mbdat;
				}
			else if(mbadr==20)		//количество БПС
				{
				if((mbdat>0)&&(mbdat<=32))
				lc640_write_int(EE_NUMIST,mbdat);  
				}
			else if(mbadr==22)		//звуковая сигнализация аварий
				{
				if((mbdat==0)||(mbdat==1))
				lc640_write_int(EE_ZV_ON,mbdat);  
				}
			else if(mbadr==31)		//Максимальное (аварийное) напряжение выпрямителей, 0.1В
				{
				lc640_write_int(EE_UMAX,mbdat);
	     		}
			else if(mbadr==43)		//Время задержки включения выпрямителей, сек
				{
				if(mbdat>=3 && mbdat<=60) lc640_write_int(EE_TZAS,mbdat);
	     		}
			else if(mbadr==44)		//Температура выпрямителей аварийная, 1гЦ
				{
				lc640_write_int(EE_TMAX,mbdat);
	     		}
			else if(mbadr==45)		//Температура выпрямителей сигнальная, 1гЦ
				{
				lc640_write_int(EE_TSIGN,mbdat);
	     		}
			else if(mbadr==60)//60	Выходное напряжение ВДК при работе с УКУ
				{
				lc640_write_int(EE_UOUT,mbdat);
				}
			else if(mbadr==61)//61  Автономное выходное напряжение ВДК при работе без УКУ 
				{
				lc640_write_int(EE_UAVT,mbdat);
				}
			else if(mbadr==62)//62 порог Перегрев системы 
				{
				lc640_write_int(EE_TSYSMAX,mbdat);
				}
			else if(mbadr==63)//63 	Уставка защиты от недопустимого понижения напряжения
				{
				lc640_write_int(EE_DU,mbdat);
				}
			else if(mbadr==64)//64	Уставка максимального входного напряжения 
				{
				lc640_write_int(EE_UINMAX,mbdat);
				}
			else if(mbadr==65)//65	Уставка минимального входного напряжения 
				{
				lc640_write_int(EE_UINMIN,mbdat);
				}
			else if(mbadr==66)//66	Уставка максимального выходного напряжения
				{
				lc640_write_int(EE_UOUTMAX,mbdat);
				}
			else if(mbadr==67)//67	Уставка минимального выходного напряжения 
				{
				lc640_write_int(EE_UOUTMIN,mbdat);
				}
			else if(mbadr==68)//68 порог ресурса вентилятора
				{
				if(mbdat<=6000) lc640_write_int(EE_TVENTMAX,mbdat);
				}
			else if(mbadr==69)//69 адрес модбас
				{
				lc640_write_int(EE_MODBUS_ADRESS,mbdat);
				}
			else if(mbadr==70)//70 скорость модбас
				{
				lc640_write_int(EE_MODBUS_BAUDRATE,mbdat);
				}
			else if(mbadr==71)//71 способ измерения выходного тока - по шунту=1 или как сумму токов источников=0
				{
				if(mbdat==0 || mbdat==1) lc640_write_int(EE_I_LOAD_MODE,mbdat);
				}
			else if(mbadr==72)//72 назначение реле 1
				{
				lc640_write_int(EE_RELE_SET_MASK0,mbdat);
				}
			else if(mbadr==73)//73 назначение реле 2
				{
				lc640_write_int(EE_RELE_SET_MASK1,mbdat);
				}
			else if(mbadr==74)//74 назначение реле 3
				{
				lc640_write_int(EE_RELE_SET_MASK2,mbdat);
				}
			else if(mbadr==75)//75 назначение реле 4
				{
				lc640_write_int(EE_RELE_SET_MASK3,mbdat);
				}
			else if(mbadr==76)//76 отключения аварийного сигнала, 1-автоматическое, 0-ручное
				{
				if(mbdat==0 || mbdat==1) lc640_write_int(EE_AV_OFF_AVT,mbdat);
				}
			else if(mbadr==77)//77 перегрузка по току, 1А
				{
				if(mbdat>0 && mbdat<=2000) lc640_write_int(EE_OVERLOAD_CURR,mbdat);
				}
			else if(mbadr==78)//78 время выдержки перегрузки по току, 1сек
				{
				if(mbdat>0 && mbdat<=120) lc640_write_int(EE_OVERLOAD_TIME,mbdat);
				}





}
//-----------



