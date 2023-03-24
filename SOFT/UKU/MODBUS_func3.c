#include "MODBUS_func3.h"
#include "eeprom_map.h"
#include "main.h"
#include "MODBUS_RTU.h"
#include <LPC17xx.H>
signed short tmp_oleg3=1478;

//----------- Таблица адресов функц 3
//адреса распологаются по порядку, если нет регистра, то &NULL_0, &NULL_0,
unsigned char *const reg_func3 []={
&NULL_0,
&NULL_0,
&NULL_0,
&NULL_0,
&NULL_0,
&NULL_0,
&NULL_0,
&NULL_0,
&NULL_0,
&NULL_0,
&NULL_0,
&NULL_0,
&NULL_0,
&NULL_0,
&NULL_0,
&NULL_0,
&NULL_0,
&NULL_0,
&NULL_0,
&NULL_0,
&NULL_0,
&NULL_0,
(unsigned char*)&LPC_RTC->YEAR+1,			//Рег11  Время, год
(unsigned char*)&LPC_RTC->YEAR,
(unsigned char*)&LPC_RTC->MONTH+1,		    //Рег12  Время, месяц
(unsigned char*)&LPC_RTC->MONTH,
(unsigned char*)&LPC_RTC->DOM+1,			//Рег13  Время, день месяца
(unsigned char*)&LPC_RTC->DOM,
(unsigned char*)&LPC_RTC->HOUR+1,			//Рег14  Время, час
(unsigned char*)&LPC_RTC->HOUR,
(unsigned char*)&LPC_RTC->MIN+1,			//Рег15  Время, минуты
(unsigned char*)&LPC_RTC->MIN,
(unsigned char*)&LPC_RTC->SEC+1,			//Рег16  Время, секунды
(unsigned char*)&LPC_RTC->SEC,
&NULL_0,
&NULL_0,
&NULL_0,
&NULL_0,
&NULL_0,
&NULL_0,
(unsigned char*)&NUMIST+1,				//Рег20  Количество выпрямителей в структуре
(unsigned char*)&NUMIST,
&NULL_0,					
&NULL_0,
(unsigned char*)&ZV_ON+1,				//Рег22  Звуковая аварийная сигнализация вкл./выкл.
(unsigned char*)&ZV_ON,
&NULL_0,
&NULL_0,
(unsigned char*)&UBM_AV+1,	//не используется	//Рег24  Аварийный уровень отклонения напряжения средней точки батареи, %
(unsigned char*)&UBM_AV,
&NULL_0,
&NULL_0,
&NULL_0,
&NULL_0,
&NULL_0,
&NULL_0,
&NULL_0,
&NULL_0,
&NULL_0,
&NULL_0,
(unsigned char*)&TBAT+1,//не используется	//Рег30  Период проверки цепи батареи, минут.
(unsigned char*)&TBAT,
(unsigned char*)&UMAX+1,					//Рег31  Максимальное (аварийное) напряжение выпрямителей, 0.1В
(unsigned char*)&UMAX,
&NULL_0,				     //Рег32  
&NULL_0,
&NULL_0,					//Рег33  
&NULL_0,
&NULL_0,					//Рег34  
&NULL_0,
(unsigned char*)&USIGN+1,	//не используется	//Рег35  Минимальное (сигнальное) напряжение батареи, 1В
(unsigned char*)&USIGN,
(unsigned char*)&UMN+1,		//не используется	//Рег36  Минимальное (аварийное) напряжение питающей сети, 1В
(unsigned char*)&UMN,
(unsigned char*)&U0B+1,		//не используется	//Рег37  Рабочее напряжение при невведенных батареях, 0.1В
(unsigned char*)&U0B,
(unsigned char*)&IKB+1,		//не используется	//Рег38  Ток контроля наличия батареи, 0.1а
(unsigned char*)&IKB,
(unsigned char*)&IZMAX+1,	//не используется	//Рег39  Ток заряда батареи максимальный, 0.1А
(unsigned char*)&IZMAX,
(unsigned char*)&IMAX+1,	//не используется	//Рег40  Ток переключения на большее кол-во выпрямителей, 0.1А
(unsigned char*)&IMAX,
(unsigned char*)&IMIN+1,	//не используется	//Рег41  Ток переключения на меньшее кол-во выпрямителей, 0.1А
(unsigned char*)&IMIN,
&NULL_0,					//Рег42  
&NULL_0,
(unsigned char*)&TZAS+1,					//Рег43  Время задержки включения выпрямителей, сек
(unsigned char*)&TZAS,
(unsigned char*)&TMAX+1,					//Рег44  Температура выпрямителей аварийная, 1гЦ
(unsigned char*)&TMAX,
(unsigned char*)&TSIGN+1,					//Рег45  Температура выпрямителей сигнальная, 1гЦ
(unsigned char*)&TSIGN,
(unsigned char*)&TBATMAX+1,	//не используется			    //Рег46  Температура батареи аварийная, 1гЦ
(unsigned char*)&TBATMAX,
(unsigned char*)&TBATSIGN+1,//не используется				//Рег47  Температура батареи сигнальная, 1гЦ
(unsigned char*)&TBATSIGN,
(unsigned char*)&speedChrgCurr+1,//не используется			//Рег48  Ток ускоренного заряда, 0.1А
(unsigned char*)&speedChrgCurr,
(unsigned char*)&speedChrgVolt+1,//не используется			//Рег49	 Напряжение ускоренного заряда, 0.1В 
(unsigned char*)&speedChrgVolt,
(unsigned char*)&speedChrgTimeInHour+1,//не используется	//Рег50	 Время ускоренного заряда, 1ч
(unsigned char*)&speedChrgTimeInHour,
(unsigned char*)&U_OUT_KONTR_MAX+1,	//не используется				//Рег51	 Контроль выходного напряжения, Umax, 0.1В
(unsigned char*)&U_OUT_KONTR_MAX,
(unsigned char*)&U_OUT_KONTR_MIN+1,	//не используется				//Рег52	 Контроль выходного напряжения, Umin, 0.1В
(unsigned char*)&U_OUT_KONTR_MIN,
(unsigned char*)&U_OUT_KONTR_DELAY+1,//не используется				//Рег53	 Контроль выходного напряжения, Tзадержки, 1сек.
(unsigned char*)&U_OUT_KONTR_DELAY,
&NULL_0,
&NULL_0,
&NULL_0,
&NULL_0,
&NULL_0,
&NULL_0,
&NULL_0,
&NULL_0,
&NULL_0,
&NULL_0,
&NULL_0,
&NULL_0,
(unsigned char*)&UOUT+1,//60	Выходное напряжение ВДК при работе с УКУ
(unsigned char*)&UOUT,  //60
(unsigned char*)&UAVT+1,//61  	Автономное выходное напряжение ВДК при работе без УКУ 
(unsigned char*)&UAVT,//61
(unsigned char*)&TSYSMAX+1,//62 порог Перегрев системы
(unsigned char*)&TSYSMAX,//62
(unsigned char*)&DU+1,//63 	Уставка защиты от недопустимого понижения напряжения 
(unsigned char*)&DU,  //63 
(unsigned char*)&UINMAX+1,//64	Уставка максимального входного напряжения 
(unsigned char*)&UINMAX,//64
(unsigned char*)&UINMIN+1,//65	Уставка минимального входного напряжения 
(unsigned char*)&UINMIN,//65
(unsigned char*)&UOUTMAX+1,//66	Уставка максимального выходного напряжения 
(unsigned char*)&UOUTMAX,//66
(unsigned char*)&UOUTMIN+1,//67	Уставка минимального выходного напряжения 
(unsigned char*)&UOUTMIN,//67
(unsigned char*)&TVENTMAX+1,//68 порог ресурса вентилятора 
(unsigned char*)&TVENTMAX,//68
(unsigned char*)&MODBUS_ADRESS+1,//69 адрес модбас
(unsigned char*)&MODBUS_ADRESS,//69
(unsigned char*)&MODBUS_BAUDRATE+1,//70 скорость модбас
(unsigned char*)&MODBUS_BAUDRATE,//70
(unsigned char*)&I_LOAD_MODE+1,	//71 способ измерения выходного тока - по шунту=1 или как сумму токов источников=0
(unsigned char*)&I_LOAD_MODE,	//71
//назначение срабатывания реле. Задается битами. Если бит=1:
//0-авария БПС
//1-Перегрузка системы по току
//2-перегрев системы
//3-Uвых занижено
//4-Uвых завышено
//5-Uвх занижено
//6-Uвх завышено
//7-ресурс вентилятора исчерпан
//8-Uв.д. >2В
(unsigned char*)&RELE_SET_MASK[0]+1,//72 назначение реле 1
(unsigned char*)&RELE_SET_MASK[0],//72
(unsigned char*)&RELE_SET_MASK[1]+1,//73 назначение реле 2
(unsigned char*)&RELE_SET_MASK[1],//73
(unsigned char*)&RELE_SET_MASK[2]+1,//74 назначение реле 3
(unsigned char*)&RELE_SET_MASK[2],//74
(unsigned char*)&RELE_SET_MASK[3]+1,//75 назначение реле 4
(unsigned char*)&RELE_SET_MASK[3],//75
(unsigned char*)&AV_OFF_AVT+1,//76 отключения аварийного сигнала, 1-автоматическое, 0-ручное
(unsigned char*)&AV_OFF_AVT,//76
(unsigned char*)&OVERLOAD_CURR+1,//77 перегрузка по току,1А
(unsigned char*)&OVERLOAD_CURR,//77
(unsigned char*)&OVERLOAD_TIME+1,//78 время выдержки перегрузки по току, 1 сек
(unsigned char*)&OVERLOAD_TIME,//78
&NULL_0,
&NULL_0,
&NULL_0,
&NULL_0,
&NULL_0,
&NULL_0,
&NULL_0,
&NULL_0,
&NULL_0,
&NULL_0,
&NULL_0,
&NULL_0,
};

//количество байт в массиве, включая нулевой байт:= (максимальный номер регистра+1)*2
// задать в .h #define MODBUS_FUNC_3_LENGTH 


