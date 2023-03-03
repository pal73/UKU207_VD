#include "MODBUS_func3.h"
#include "eeprom_map.h"
#include "main.h"
#include "MODBUS_RTU.h"
#include <LPC17xx.H>
signed short tmp_oleg3=1478;

//----------- ������� ������� ����� 3
//������ ������������� �� �������, ���� ��� ��������, �� &NULL_0, &NULL_0,
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
(unsigned char*)&LPC_RTC->YEAR+1,			//���11  �����, ���
(unsigned char*)&LPC_RTC->YEAR,
(unsigned char*)&LPC_RTC->MONTH+1,		    //���12  �����, �����
(unsigned char*)&LPC_RTC->MONTH,
(unsigned char*)&LPC_RTC->DOM+1,			//���13  �����, ���� ������
(unsigned char*)&LPC_RTC->DOM,
(unsigned char*)&LPC_RTC->HOUR+1,			//���14  �����, ���
(unsigned char*)&LPC_RTC->HOUR,
(unsigned char*)&LPC_RTC->MIN+1,			//���15  �����, ������
(unsigned char*)&LPC_RTC->MIN,
(unsigned char*)&LPC_RTC->SEC+1,			//���16  �����, �������
(unsigned char*)&LPC_RTC->SEC,
&NULL_0,
&NULL_0,
&NULL_0,
&NULL_0,
&NULL_0,
&NULL_0,
(unsigned char*)&NUMIST+1,				//���20  ���������� ������������ � ���������
(unsigned char*)&NUMIST,
&NULL_0,					
&NULL_0,
(unsigned char*)&ZV_ON+1,				//���22  �������� ��������� ������������ ���./����.
(unsigned char*)&ZV_ON,
&NULL_0,
&NULL_0,
(unsigned char*)&UBM_AV+1,	//�� ������������	//���24  ��������� ������� ���������� ���������� ������� ����� �������, %
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
(unsigned char*)&TBAT+1,//�� ������������	//���30  ������ �������� ���� �������, �����.
(unsigned char*)&TBAT,
(unsigned char*)&UMAX+1,					//���31  ������������ (���������) ���������� ������������, 0.1�
(unsigned char*)&UMAX,
&NULL_0,				     //���32  
&NULL_0,
&NULL_0,					//���33  
&NULL_0,
&NULL_0,					//���34  
&NULL_0,
(unsigned char*)&USIGN+1,	//�� ������������	//���35  ����������� (����������) ���������� �������, 1�
(unsigned char*)&USIGN,
(unsigned char*)&UMN+1,		//�� ������������	//���36  ����������� (���������) ���������� �������� ����, 1�
(unsigned char*)&UMN,
(unsigned char*)&U0B+1,		//�� ������������	//���37  ������� ���������� ��� ����������� ��������, 0.1�
(unsigned char*)&U0B,
(unsigned char*)&IKB+1,		//�� ������������	//���38  ��� �������� ������� �������, 0.1�
(unsigned char*)&IKB,
(unsigned char*)&IZMAX+1,	//�� ������������	//���39  ��� ������ ������� ������������, 0.1�
(unsigned char*)&IZMAX,
(unsigned char*)&IMAX+1,	//�� ������������	//���40  ��� ������������ �� ������� ���-�� ������������, 0.1�
(unsigned char*)&IMAX,
(unsigned char*)&IMIN+1,	//�� ������������	//���41  ��� ������������ �� ������� ���-�� ������������, 0.1�
(unsigned char*)&IMIN,
&NULL_0,					//���42  
&NULL_0,
(unsigned char*)&TZAS+1,					//���43  ����� �������� ��������� ������������, ���
(unsigned char*)&TZAS,
(unsigned char*)&TMAX+1,					//���44  ����������� ������������ ���������, 1��
(unsigned char*)&TMAX,
(unsigned char*)&TSIGN+1,					//���45  ����������� ������������ ����������, 1��
(unsigned char*)&TSIGN,
(unsigned char*)&TBATMAX+1,	//�� ������������			    //���46  ����������� ������� ���������, 1��
(unsigned char*)&TBATMAX,
(unsigned char*)&TBATSIGN+1,//�� ������������				//���47  ����������� ������� ����������, 1��
(unsigned char*)&TBATSIGN,
(unsigned char*)&speedChrgCurr+1,//�� ������������			//���48  ��� ����������� ������, 0.1�
(unsigned char*)&speedChrgCurr,
(unsigned char*)&speedChrgVolt+1,//�� ������������			//���49	 ���������� ����������� ������, 0.1� 
(unsigned char*)&speedChrgVolt,
(unsigned char*)&speedChrgTimeInHour+1,//�� ������������	//���50	 ����� ����������� ������, 1�
(unsigned char*)&speedChrgTimeInHour,
(unsigned char*)&U_OUT_KONTR_MAX+1,	//�� ������������				//���51	 �������� ��������� ����������, Umax, 0.1�
(unsigned char*)&U_OUT_KONTR_MAX,
(unsigned char*)&U_OUT_KONTR_MIN+1,	//�� ������������				//���52	 �������� ��������� ����������, Umin, 0.1�
(unsigned char*)&U_OUT_KONTR_MIN,
(unsigned char*)&U_OUT_KONTR_DELAY+1,//�� ������������				//���53	 �������� ��������� ����������, T��������, 1���.
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
(unsigned char*)&UOUT+1,//60	�������� ���������� ��� ��� ������ � ���
(unsigned char*)&UOUT,  //60
(unsigned char*)&UAVT+1,//61  	���������� �������� ���������� ��� ��� ������ ��� ��� 
(unsigned char*)&UAVT,//61
(unsigned char*)&TSYSMAX+1,//62 ����� �������� �������
(unsigned char*)&TSYSMAX,//62
(unsigned char*)&DU+1,//63 	������� ������ �� ������������� ��������� ���������� 
(unsigned char*)&DU,  //63 
(unsigned char*)&UINMAX+1,//64	������� ������������� �������� ���������� 
(unsigned char*)&UINMAX,//64
(unsigned char*)&UINMIN+1,//65	������� ������������ �������� ���������� 
(unsigned char*)&UINMIN,//65
(unsigned char*)&UOUTMAX+1,//66	������� ������������� ��������� ���������� 
(unsigned char*)&UOUTMAX,//66
(unsigned char*)&UOUTMIN+1,//67	������� ������������ ��������� ���������� 
(unsigned char*)&UOUTMIN,//67
(unsigned char*)&TVENTMAX+1,//68 ����� ������� ����������� 
(unsigned char*)&TVENTMAX,//68
(unsigned char*)&MODBUS_ADRESS+1,//69 ����� ������
(unsigned char*)&MODBUS_ADRESS,//69
(unsigned char*)&MODBUS_BAUDRATE+1,//70 �������� ������
(unsigned char*)&MODBUS_BAUDRATE,//70
(unsigned char*)&I_LOAD_MODE+1,	//71 ������ ��������� ��������� ���� - �� �����=1 ��� ��� ����� ����� ����������=0
(unsigned char*)&I_LOAD_MODE,	//71
//���������� ������������ ����. �������� ������. ���� ���=1:
//0-������ ���
//1-���������� ������� �� ����
//2-�������� �������
//3-U��� ��������
//4-U��� ��������
//5-U�� ��������
//6-U�� ��������
//7-������ ����������� ��������
//8-U�.�. >2�
(unsigned char*)&RELE_SET_MASK[0]+1,//72 ���������� ���� 1
(unsigned char*)&RELE_SET_MASK[0],//72
(unsigned char*)&RELE_SET_MASK[1]+1,//73 ���������� ���� 2
(unsigned char*)&RELE_SET_MASK[1],//73
(unsigned char*)&RELE_SET_MASK[2]+1,//74 ���������� ���� 3
(unsigned char*)&RELE_SET_MASK[2],//74
(unsigned char*)&RELE_SET_MASK[3]+1,//75 ���������� ���� 4
(unsigned char*)&RELE_SET_MASK[3],//75
(unsigned char*)&AV_OFF_AVT+1,//76 ���������� ���������� �������, 1-��������������, 0-������
(unsigned char*)&AV_OFF_AVT,//76
(unsigned char*)&OVERLOAD_CURR+1,//77 ���������� �� ����,1�
(unsigned char*)&OVERLOAD_CURR,//77
(unsigned char*)&OVERLOAD_TIME+1,//78 ����� �������� ���������� �� ����, 1 ���
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

//���������� ���� � �������, ������� ������� ����:= (������������ ����� ��������+1)*2
// ������ � .h #define MODBUS_FUNC_3_LENGTH 


