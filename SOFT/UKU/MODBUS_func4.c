#include "MODBUS_func4.h"
//#include "akb_f.h"
#include "curr_version.h"
#include "main.h"
#include "MODBUS_RTU.h"
signed short tmp_oleg4=0x2569;

//----------- Таблица адресов функц 3
//адреса распологаются по порядку, если нет регистра, то &NULL_0, &NULL_0,
unsigned char *const reg_func4 []={
&NULL_0,  //0
&NULL_0,  //0
(unsigned char*)&bps[0]._Uii+1,			//Рег1	Выходное напряжение выпрямителя №1, 0.1В
(unsigned char*)&bps[0]._Uii,
(unsigned char*)&bps[0]._Ii+1,			//Рег2	Выходной ток выпрямителя №1, 0.1А
(unsigned char*)&bps[0]._Ii,
(unsigned char*)&bps[0]._Ti+1,			//Рег3	Температура радиатора выпрямителя №1, 1гЦ
(unsigned char*)&bps[0]._Ti,
(unsigned char*)&bps[0]._av+1,			//Рег4	Байт флагов выпрямителя №1, 0x01 - перегрев, 0x02 завышено Uвых, 0x04 занижено Uвых, 0x08 - отсутствует связь с выпрямителем
(unsigned char*)&bps[0]._av,
(unsigned char*)&bps[1]._Uii+1,			//Рег5	Выходное напряжение выпрямителя №2, 0.1В
(unsigned char*)&bps[1]._Uii,
(unsigned char*)&bps[1]._Ii+1,			//Рег6	Выходной ток выпрямителя №2, 0.1А
(unsigned char*)&bps[1]._Ii,
(unsigned char*)&bps[1]._Ti+1,			//Рег7	Температура радиатора выпрямителя №2, 1гЦ
(unsigned char*)&bps[1]._Ti,
(unsigned char*)&bps[1]._av+1,			//Рег8	Байт флагов выпрямителя №2, 0x01 - перегрев, 0x02 завышено Uвых, 0x04 занижено Uвых, 0x08 - отсутствует связь с выпрямителем
(unsigned char*)&bps[1]._av,
(unsigned char*)&bps[2]._Uii+1,			//Рег9	Выходное напряжение выпрямителя №3, 0.1В
(unsigned char*)&bps[2]._Uii,
(unsigned char*)&bps[2]._Ii+1,			//Рег10	Выходной ток выпрямителя №3, 0.1А
(unsigned char*)&bps[2]._Ii,
(unsigned char*)&bps[2]._Ti+1,			//Рег11	Температура радиатора выпрямителя №3, 1гЦ
(unsigned char*)&bps[2]._Ti,
(unsigned char*)&bps[2]._av+1,			//Рег12	Байт флагов выпрямителя №3, 0x01 - перегрев, 0x02 завышено Uвых, 0x04 занижено Uвых, 0x08 - отсутствует связь с выпрямителем
(unsigned char*)&bps[2]._av,
(unsigned char*)&bps[3]._Uii+1,			//Рег13	Выходное напряжение выпрямителя №4, 0.1В
(unsigned char*)&bps[3]._Uii,
(unsigned char*)&bps[3]._Ii+1,			//Рег14	Выходной ток выпрямителя №4, 0.1А
(unsigned char*)&bps[3]._Ii,
(unsigned char*)&bps[3]._Ti+1,			//Рег15	Температура радиатора выпрямителя №4, 1гЦ
(unsigned char*)&bps[3]._Ti,
(unsigned char*)&bps[3]._av+1,			//Рег16	Байт флагов выпрямителя №4, 0x01 - перегрев, 0x02 завышено Uвых, 0x04 занижено Uвых, 0x08 - отсутствует связь с выпрямителем
(unsigned char*)&bps[3]._av,
(unsigned char*)&bps[4]._Uii+1,			//Рег17	Выходное напряжение выпрямителя №5, 0.1В
(unsigned char*)&bps[4]._Uii,
(unsigned char*)&bps[4]._Ii+1,			//Рег18	Выходной ток выпрямителя №5, 0.1А
(unsigned char*)&bps[4]._Ii,
(unsigned char*)&bps[4]._Ti+1,			//Рег19	Температура радиатора выпрямителя №5, 1гЦ
(unsigned char*)&bps[4]._Ti,
(unsigned char*)&bps[4]._av+1,			//Рег20	Байт флагов выпрямителя №5, 0x01 - перегрев, 0x02 завышено Uвых, 0x04 занижено Uвых, 0x08 - отсутствует связь с выпрямителем
(unsigned char*)&bps[4]._av,
(unsigned char*)&bps[5]._Uii+1,			//Рег21	Выходное напряжение выпрямителя №6, 0.1В
(unsigned char*)&bps[5]._Uii,
(unsigned char*)&bps[5]._Ii+1,			//Рег22	Выходной ток выпрямителя №6, 0.1А
(unsigned char*)&bps[5]._Ii,
(unsigned char*)&bps[5]._Ti+1,			//Рег23	Температура радиатора выпрямителя №6, 1гЦ
(unsigned char*)&bps[5]._Ti,
(unsigned char*)&bps[5]._av+1,			//Рег24	Байт флагов выпрямителя №6, 0x01 - перегрев, 0x02 завышено Uвых, 0x04 занижено Uвых, 0x08 - отсутствует связь с выпрямителем
(unsigned char*)&bps[5]._av,
(unsigned char*)&bps[6]._Uii+1,			//Рег25	Выходное напряжение выпрямителя №7, 0.1В
(unsigned char*)&bps[6]._Uii,
(unsigned char*)&bps[6]._Ii+1,			//Рег26	Выходной ток выпрямителя №7, 0.1А
(unsigned char*)&bps[6]._Ii,
(unsigned char*)&bps[6]._Ti+1,			//Рег27	Температура радиатора выпрямителя №7, 1гЦ
(unsigned char*)&bps[6]._Ti,
(unsigned char*)&bps[6]._av+1,			//Рег28	Байт флагов выпрямителя №7, 0x01 - перегрев, 0x02 завышено Uвых, 0x04 занижено Uвых, 0x08 - отсутствует связь с выпрямителем
(unsigned char*)&bps[6]._av,
(unsigned char*)&bps[7]._Uii+1,			//Рег29	Выходное напряжение выпрямителя №8, 0.1В
(unsigned char*)&bps[7]._Uii,
(unsigned char*)&bps[7]._Ii+1,			//Рег30	Выходной ток выпрямителя №8, 0.1А
(unsigned char*)&bps[7]._Ii,
(unsigned char*)&bps[7]._Ti+1,			//Рег31	Температура радиатора выпрямителя №8, 1гЦ
(unsigned char*)&bps[7]._Ti,
(unsigned char*)&bps[7]._av+1,			//Рег32	Байт флагов выпрямителя №8, 0x01 - перегрев, 0x02 завышено Uвых, 0x04 занижено Uвых, 0x08 - отсутствует связь с выпрямителем
(unsigned char*)&bps[7]._av,
(unsigned char*)&bps[8]._Uii+1,			//Рег33	Выходное напряжение выпрямителя №9, 0.1В
(unsigned char*)&bps[8]._Uii,
(unsigned char*)&bps[8]._Ii+1,			//Рег34	Выходной ток выпрямителя №9, 0.1А
(unsigned char*)&bps[8]._Ii,
(unsigned char*)&bps[8]._Ti+1,			//Рег35	Температура радиатора выпрямителя №9, 1гЦ
(unsigned char*)&bps[8]._Ti,
(unsigned char*)&bps[8]._av+1,			//Рег36	Байт флагов выпрямителя №9, 0x01 - перегрев, 0x02 завышено Uвых, 0x04 занижено Uвых, 0x08 - отсутствует связь с выпрямителем
(unsigned char*)&bps[8]._av,
(unsigned char*)&bps[9]._Uii+1,			//Рег37	Выходное напряжение выпрямителя №10, 0.1В
(unsigned char*)&bps[9]._Uii,
(unsigned char*)&bps[9]._Ii+1,			//Рег38	Выходной ток выпрямителя №10, 0.1А
(unsigned char*)&bps[9]._Ii,
(unsigned char*)&bps[9]._Ti+1,			//Рег39	Температура радиатора выпрямителя №10, 1гЦ
(unsigned char*)&bps[9]._Ti,
(unsigned char*)&bps[9]._av+1,			//Рег40	Байт флагов выпрямителя №10, 0x01 - перегрев, 0x02 завышено Uвых, 0x04 занижено Uвых, 0x08 - отсутствует связь с выпрямителем
(unsigned char*)&bps[9]._av,
(unsigned char*)&bps[10]._Uii+1,		//Рег41	Выходное напряжение выпрямителя №11, 0.1В
(unsigned char*)&bps[10]._Uii,
(unsigned char*)&bps[10]._Ii+1,			//Рег42	Выходной ток выпрямителя №11, 0.1А
(unsigned char*)&bps[10]._Ii,
(unsigned char*)&bps[10]._Ti+1,			//Рег43	Температура радиатора выпрямителя №11, 1гЦ
(unsigned char*)&bps[10]._Ti,
(unsigned char*)&bps[10]._av+1,			//Рег44	Байт флагов выпрямителя №11, 0x01 - перегрев, 0x02 завышено Uвых, 0x04 занижено Uвых, 0x08 - отсутствует связь с выпрямителем
(unsigned char*)&bps[10]._av,
(unsigned char*)&bps[11]._Uii+1,		//Рег45	Выходное напряжение выпрямителя №12, 0.1В
(unsigned char*)&bps[11]._Uii,
(unsigned char*)&bps[11]._Ii+1,			//Рег46	Выходной ток выпрямителя №12, 0.1А
(unsigned char*)&bps[11]._Ii,
(unsigned char*)&bps[11]._Ti+1,			//Рег47	Температура радиатора выпрямителя №12, 1гЦ
(unsigned char*)&bps[11]._Ti,
(unsigned char*)&bps[11]._av+1,			//Рег48	Байт флагов выпрямителя №12, 0x01 - перегрев, 0x02 завышено Uвых, 0x04 занижено Uвых, 0x08 - отсутствует связь с выпрямителем
(unsigned char*)&bps[11]._av,
&NULL_0,  //49
&NULL_0,															
(unsigned char*)&out_U+1,				//Рег50 Выходное напряжение системы 0.1v
(unsigned char*)&out_U,
(unsigned char*)&in_U+1,				//Рег51	Входное напряжение системы 0.1v
(unsigned char*)&in_U,
(unsigned char*)&vd_U+1,				//Рег52	Напряжение вольтдобавки 0.1v
(unsigned char*)&vd_U,
(unsigned char*)&Ib_ips_termokompensat+1,		//Рег53	Выходной ток 1A
(unsigned char*)&Ib_ips_termokompensat,

(unsigned char*)&t_ext[0]+1,				//Рег54	 Температура системы  1C
(unsigned char*)&t_ext[0],
(unsigned char*)&avar_vd_stat+1,		//Рег55	 Флаг аварий системы
(unsigned char*)&avar_vd_stat,			// Бит 0 - авария одного из БПС
															// Бит 1 - перегрузка системы по току
															// Бит 2 - перегрев системы
															// Бит 3 - выходное напряжение занижено
															// Бит 4 - выходное напряжение завышено
															// Бит 5 - входное напряжение занижено
															// Бит 6 - входное напряжение завышено
															// Бит 7 - ресурс вентилятора превышен
															// Бит 8 - вольтдобавка в работе

&NULL_0,  
&NULL_0,
&NULL_0,  
&NULL_0,
&NULL_0,  
&NULL_0,
&NULL_0,  
&NULL_0,
(unsigned char*)&bps[12]._Uii+1,			//Рег60	Выходное напряжение выпрямителя №13, 0.1В
(unsigned char*)&bps[12]._Uii,
(unsigned char*)&bps[12]._Ii+1,			//Рег61	Выходной ток выпрямителя №13, 0.1А
(unsigned char*)&bps[12]._Ii,
(unsigned char*)&bps[12]._Ti+1,			//Рег62	Температура радиатора выпрямителя №13, 1гЦ
(unsigned char*)&bps[12]._Ti,
(unsigned char*)&bps[12]._av+1,			//Рег63	Байт флагов выпрямителя №13, 0x01 - перегрев, 0x02 завышено Uвых, 0x04 занижено Uвых, 0x08 - отсутствует связь с выпрямителем
(unsigned char*)&bps[12]._av,
(unsigned char*)&bps[13]._Uii+1,			//Рег64	Выходное напряжение выпрямителя №14, 0.1В
(unsigned char*)&bps[13]._Uii,
(unsigned char*)&bps[13]._Ii+1,			//Рег65	Выходной ток выпрямителя №14, 0.1А
(unsigned char*)&bps[13]._Ii,
(unsigned char*)&bps[13]._Ti+1,			//Рег66	Температура радиатора выпрямителя №14, 1гЦ
(unsigned char*)&bps[13]._Ti,
(unsigned char*)&bps[13]._av+1,			//Рег67	Байт флагов выпрямителя №14, 0x01 - перегрев, 0x02 завышено Uвых, 0x04 занижено Uвых, 0x08 - отсутствует связь с выпрямителем
(unsigned char*)&bps[13]._av,
(unsigned char*)&bps[14]._Uii+1,			//Рег68	Выходное напряжение выпрямителя №15, 0.1В
(unsigned char*)&bps[14]._Uii,
(unsigned char*)&bps[14]._Ii+1,			//Рег69	Выходной ток выпрямителя №15, 0.1А
(unsigned char*)&bps[14]._Ii,
(unsigned char*)&bps[14]._Ti+1,			//Рег70	Температура радиатора выпрямителя №15, 1гЦ
(unsigned char*)&bps[14]._Ti,
(unsigned char*)&bps[14]._av+1,			//Рег71	Байт флагов выпрямителя №15, 0x01 - перегрев, 0x02 завышено Uвых, 0x04 занижено Uвых, 0x08 - отсутствует связь с выпрямителем
(unsigned char*)&bps[14]._av,
(unsigned char*)&bps[15]._Uii+1,			//Рег72	Выходное напряжение выпрямителя №16, 0.1В
(unsigned char*)&bps[15]._Uii,
(unsigned char*)&bps[15]._Ii+1,			//Рег73	Выходной ток выпрямителя №16, 0.1А
(unsigned char*)&bps[15]._Ii,
(unsigned char*)&bps[15]._Ti+1,			//Рег74	Температура радиатора выпрямителя №16, 1гЦ
(unsigned char*)&bps[15]._Ti,
(unsigned char*)&bps[15]._av+1,			//Рег75	Байт флагов выпрямителя №16, 0x01 - перегрев, 0x02 завышено Uвых, 0x04 занижено Uвых, 0x08 - отсутствует связь с выпрямителем
(unsigned char*)&bps[15]._av,
(unsigned char*)&bps[16]._Uii+1,			//Рег76	Выходное напряжение выпрямителя №17, 0.1В
(unsigned char*)&bps[16]._Uii,
(unsigned char*)&bps[16]._Ii+1,			//Рег77	Выходной ток выпрямителя №17, 0.1А
(unsigned char*)&bps[16]._Ii,
(unsigned char*)&bps[16]._Ti+1,			//Рег78	Температура радиатора выпрямителя №17, 1гЦ
(unsigned char*)&bps[16]._Ti,
(unsigned char*)&bps[16]._av+1,			//Рег79	Байт флагов выпрямителя №17, 0x01 - перегрев, 0x02 завышено Uвых, 0x04 занижено Uвых, 0x08 - отсутствует связь с выпрямителем
(unsigned char*)&bps[16]._av,
(unsigned char*)&bps[17]._Uii+1,			//Рег80	Выходное напряжение выпрямителя №18, 0.1В
(unsigned char*)&bps[17]._Uii,
(unsigned char*)&bps[17]._Ii+1,			//Рег81	Выходной ток выпрямителя №18, 0.1А
(unsigned char*)&bps[17]._Ii,
(unsigned char*)&bps[17]._Ti+1,			//Рег82	Температура радиатора выпрямителя №18, 1гЦ
(unsigned char*)&bps[17]._Ti,
(unsigned char*)&bps[17]._av+1,			//Рег83	Байт флагов выпрямителя №18, 0x01 - перегрев, 0x02 завышено Uвых, 0x04 занижено Uвых, 0x08 - отсутствует связь с выпрямителем
(unsigned char*)&bps[17]._av,
(unsigned char*)&bps[18]._Uii+1,			//Рег84	Выходное напряжение выпрямителя №19, 0.1В
(unsigned char*)&bps[18]._Uii,
(unsigned char*)&bps[18]._Ii+1,			//Рег85	Выходной ток выпрямителя №19, 0.1А
(unsigned char*)&bps[18]._Ii,
(unsigned char*)&bps[18]._Ti+1,			//Рег86	Температура радиатора выпрямителя №19, 1гЦ
(unsigned char*)&bps[18]._Ti,
(unsigned char*)&bps[18]._av+1,			//Рег87	Байт флагов выпрямителя №19, 0x01 - перегрев, 0x02 завышено Uвых, 0x04 занижено Uвых, 0x08 - отсутствует связь с выпрямителем
(unsigned char*)&bps[18]._av,
(unsigned char*)&bps[19]._Uii+1,			//Рег88	Выходное напряжение выпрямителя №20, 0.1В
(unsigned char*)&bps[19]._Uii,
(unsigned char*)&bps[19]._Ii+1,			//Рег89	Выходной ток выпрямителя №20, 0.1А
(unsigned char*)&bps[19]._Ii,
(unsigned char*)&bps[19]._Ti+1,			//Рег90	Температура радиатора выпрямителя №20, 1гЦ
(unsigned char*)&bps[19]._Ti,
(unsigned char*)&bps[19]._av+1,			//Рег91	Байт флагов выпрямителя №20, 0x01 - перегрев, 0x02 завышено Uвых, 0x04 занижено Uвых, 0x08 - отсутствует связь с выпрямителем
(unsigned char*)&bps[19]._av,
(unsigned char*)&bps[20]._Uii+1,			//Рег92	Выходное напряжение выпрямителя №21, 0.1В
(unsigned char*)&bps[20]._Uii,
(unsigned char*)&bps[20]._Ii+1,			//Рег93	Выходной ток выпрямителя №21, 0.1А
(unsigned char*)&bps[20]._Ii,
(unsigned char*)&bps[20]._Ti+1,			//Рег94	Температура радиатора выпрямителя №21, 1гЦ
(unsigned char*)&bps[20]._Ti,
(unsigned char*)&bps[20]._av+1,			//Рег95	Байт флагов выпрямителя №21, 0x01 - перегрев, 0x02 завышено Uвых, 0x04 занижено Uвых, 0x08 - отсутствует связь с выпрямителем
(unsigned char*)&bps[20]._av,
(unsigned char*)&bps[21]._Uii+1,			//Рег96	Выходное напряжение выпрямителя №22, 0.1В
(unsigned char*)&bps[21]._Uii,
(unsigned char*)&bps[21]._Ii+1,			//Рег97	Выходной ток выпрямителя №22, 0.1А
(unsigned char*)&bps[21]._Ii,
(unsigned char*)&bps[21]._Ti+1,			//Рег98	Температура радиатора выпрямителя №22, 1гЦ
(unsigned char*)&bps[21]._Ti,
(unsigned char*)&bps[21]._av+1,			//Рег99	Байт флагов выпрямителя №22, 0x01 - перегрев, 0x02 завышено Uвых, 0x04 занижено Uвых, 0x08 - отсутствует связь с выпрямителем
(unsigned char*)&bps[21]._av,
(unsigned char*)&bps[22]._Uii+1,			//Рег100	Выходное напряжение выпрямителя №23, 0.1В
(unsigned char*)&bps[22]._Uii,
(unsigned char*)&bps[22]._Ii+1,			//Рег101	Выходной ток выпрямителя №23, 0.1А
(unsigned char*)&bps[22]._Ii,
(unsigned char*)&bps[22]._Ti+1,			//Рег102	Температура радиатора выпрямителя №23, 1гЦ
(unsigned char*)&bps[22]._Ti,
(unsigned char*)&bps[22]._av+1,			//Рег103	Байт флагов выпрямителя №23, 0x01 - перегрев, 0x02 завышено Uвых, 0x04 занижено Uвых, 0x08 - отсутствует связь с выпрямителем
(unsigned char*)&bps[22]._av,
(unsigned char*)&bps[23]._Uii+1,			//Рег104	Выходное напряжение выпрямителя №24, 0.1В
(unsigned char*)&bps[23]._Uii,
(unsigned char*)&bps[23]._Ii+1,			//Рег105	Выходной ток выпрямителя №24, 0.1А
(unsigned char*)&bps[23]._Ii,
(unsigned char*)&bps[23]._Ti+1,			//Рег106	Температура радиатора выпрямителя №24, 1гЦ
(unsigned char*)&bps[23]._Ti,
(unsigned char*)&bps[23]._av+1,			//Рег107	Байт флагов выпрямителя №24, 0x01 - перегрев, 0x02 завышено Uвых, 0x04 занижено Uвых, 0x08 - отсутствует связь с выпрямителем
(unsigned char*)&bps[23]._av,
(unsigned char*)&bps[24]._Uii+1,			//Рег108	Выходное напряжение выпрямителя №25, 0.1В
(unsigned char*)&bps[24]._Uii,
(unsigned char*)&bps[24]._Ii+1,			//Рег109	Выходной ток выпрямителя №25, 0.1А
(unsigned char*)&bps[24]._Ii,
(unsigned char*)&bps[24]._Ti+1,			//Рег110	Температура радиатора выпрямителя №25, 1гЦ
(unsigned char*)&bps[24]._Ti,
(unsigned char*)&bps[24]._av+1,			//Рег111	Байт флагов выпрямителя №25, 0x01 - перегрев, 0x02 завышено Uвых, 0x04 занижено Uвых, 0x08 - отсутствует связь с выпрямителем
(unsigned char*)&bps[24]._av,
(unsigned char*)&bps[25]._Uii+1,			//Рег112	Выходное напряжение выпрямителя №26, 0.1В
(unsigned char*)&bps[25]._Uii,
(unsigned char*)&bps[25]._Ii+1,			//Рег113	Выходной ток выпрямителя №26, 0.1А
(unsigned char*)&bps[25]._Ii,
(unsigned char*)&bps[25]._Ti+1,			//Рег114	Температура радиатора выпрямителя №26, 1гЦ
(unsigned char*)&bps[25]._Ti,
(unsigned char*)&bps[25]._av+1,			//Рег115	Байт флагов выпрямителя №26, 0x01 - перегрев, 0x02 завышено Uвых, 0x04 занижено Uвых, 0x08 - отсутствует связь с выпрямителем
(unsigned char*)&bps[25]._av,
(unsigned char*)&bps[26]._Uii+1,			//Рег116	Выходное напряжение выпрямителя №27, 0.1В
(unsigned char*)&bps[26]._Uii,
(unsigned char*)&bps[26]._Ii+1,			//Рег117	Выходной ток выпрямителя №27, 0.1А
(unsigned char*)&bps[26]._Ii,
(unsigned char*)&bps[26]._Ti+1,			//Рег118	Температура радиатора выпрямителя №27, 1гЦ
(unsigned char*)&bps[26]._Ti,
(unsigned char*)&bps[26]._av+1,			//Рег119	Байт флагов выпрямителя №27, 0x01 - перегрев, 0x02 завышено Uвых, 0x04 занижено Uвых, 0x08 - отсутствует связь с выпрямителем
(unsigned char*)&bps[26]._av,
(unsigned char*)&bps[27]._Uii+1,			//Рег120	Выходное напряжение выпрямителя №28, 0.1В
(unsigned char*)&bps[27]._Uii,
(unsigned char*)&bps[27]._Ii+1,			//Рег121	Выходной ток выпрямителя №28, 0.1А
(unsigned char*)&bps[27]._Ii,
(unsigned char*)&bps[27]._Ti+1,			//Рег122	Температура радиатора выпрямителя №28, 1гЦ
(unsigned char*)&bps[27]._Ti,
(unsigned char*)&bps[27]._av+1,			//Рег123	Байт флагов выпрямителя №28, 0x01 - перегрев, 0x02 завышено Uвых, 0x04 занижено Uвых, 0x08 - отсутствует связь с выпрямителем
(unsigned char*)&bps[27]._av,
(unsigned char*)&bps[28]._Uii+1,			//Рег124	Выходное напряжение выпрямителя №29, 0.1В
(unsigned char*)&bps[28]._Uii,
(unsigned char*)&bps[28]._Ii+1,			//Рег125	Выходной ток выпрямителя №29, 0.1А
(unsigned char*)&bps[28]._Ii,
(unsigned char*)&bps[28]._Ti+1,			//Рег126	Температура радиатора выпрямителя №29, 1гЦ
(unsigned char*)&bps[28]._Ti,
(unsigned char*)&bps[28]._av+1,			//Рег127	Байт флагов выпрямителя №29, 0x01 - перегрев, 0x02 завышено Uвых, 0x04 занижено Uвых, 0x08 - отсутствует связь с выпрямителем
(unsigned char*)&bps[28]._av,
(unsigned char*)&bps[29]._Uii+1,			//Рег128	Выходное напряжение выпрямителя №30, 0.1В
(unsigned char*)&bps[29]._Uii,
(unsigned char*)&bps[29]._Ii+1,			//Рег129	Выходной ток выпрямителя №30, 0.1А
(unsigned char*)&bps[29]._Ii,
(unsigned char*)&bps[29]._Ti+1,			//Рег130	Температура радиатора выпрямителя №30, 1гЦ
(unsigned char*)&bps[29]._Ti,
(unsigned char*)&bps[29]._av+1,			//Рег131	Байт флагов выпрямителя №30, 0x01 - перегрев, 0x02 завышено Uвых, 0x04 занижено Uвых, 0x08 - отсутствует связь с выпрямителем
(unsigned char*)&bps[29]._av,
(unsigned char*)&bps[30]._Uii+1,			//Рег132	Выходное напряжение выпрямителя №31, 0.1В
(unsigned char*)&bps[30]._Uii,
(unsigned char*)&bps[30]._Ii+1,			//Рег133	Выходной ток выпрямителя №31, 0.1А
(unsigned char*)&bps[30]._Ii,
(unsigned char*)&bps[30]._Ti+1,			//Рег134	Температура радиатора выпрямителя №31, 1гЦ
(unsigned char*)&bps[30]._Ti,
(unsigned char*)&bps[30]._av+1,			//Рег135	Байт флагов выпрямителя №31, 0x01 - перегрев, 0x02 завышено Uвых, 0x04 занижено Uвых, 0x08 - отсутствует связь с выпрямителем
(unsigned char*)&bps[30]._av,
(unsigned char*)&bps[31]._Uii+1,			//Рег136	Выходное напряжение выпрямителя №32, 0.1В
(unsigned char*)&bps[31]._Uii,
(unsigned char*)&bps[31]._Ii+1,			//Рег137	Выходной ток выпрямителя №32, 0.1А
(unsigned char*)&bps[31]._Ii,
(unsigned char*)&bps[31]._Ti+1,			//Рег138	Температура радиатора выпрямителя №32, 1гЦ
(unsigned char*)&bps[31]._Ti,
(unsigned char*)&bps[31]._av+1,			//Рег139	Байт флагов выпрямителя №32, 0x01 - перегрев, 0x02 завышено Uвых, 0x04 занижено Uвых, 0x08 - отсутствует связь с выпрямителем
(unsigned char*)&bps[31]._av,
(unsigned char*)&HARDVARE_VERSION+1,  //Рег 140  	аппаратная версия
(unsigned char*)&HARDVARE_VERSION,	  //Рег 140  	аппаратная версия
(unsigned char*)&SOFT_VERSION+1,	  //Рег 141  	Версия ПО
(unsigned char*)&SOFT_VERSION,		  //Рег 141  	Версия ПО
(unsigned char*)&BUILD+1,			  //Рег 142  	Номер компиляции ПО
(unsigned char*)&BUILD,				  //Рег 142  	Номер компиляции ПО
(unsigned char*)&BUILD_YEAR+1,		  //Рег 143  	Год компиляции ПО
(unsigned char*)&BUILD_YEAR,		  //Рег 143  	Год компиляции ПО
(unsigned char*)&BUILD_MONTH+1,		  //Рег 144  	Месяц компиляции ПО
(unsigned char*)&BUILD_MONTH,		  //Рег 144  	Месяц компиляции ПО
(unsigned char*)&BUILD_DAY+1,		  //Рег 145  	День компиляции ПО
(unsigned char*)&BUILD_DAY,			  //Рег 145  	День компиляции ПО
&NULL_0,//
&NULL_0,//
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
};

//количество байт в массиве, включая нулевой байт:= (максимальный номер регистра+1)*2
// задать в .h #define MODBUS_FUNC_4_LENGTH  		   





