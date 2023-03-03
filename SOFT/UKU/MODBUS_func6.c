#include <lpc17xx.h>
#include "MODBUS_func6.h"
#include "curr_version.h"
#include "main.h"
#include "MODBUS_RTU.h"
#include "eeprom_map.h"
#include "control.h"

void analiz_func6(unsigned short mbadr, unsigned short mbdat){
			if(mbadr==11)		//��������� ������� 
				{
				LPC_RTC->YEAR=(uint16_t)mbdat;
				}
			else if(mbadr==12)		//��������� ������� 
				{
				LPC_RTC->MONTH=(uint16_t)mbdat;
				}
			else if(mbadr==13)		//��������� ������� 
				{
				LPC_RTC->DOM=(uint16_t)mbdat;
				}
			else if(mbadr==14)		//��������� ������� 
				{
				LPC_RTC->HOUR=(uint16_t)mbdat;
				}
			else if(mbadr==15)		//��������� ������� 
				{
				LPC_RTC->MIN=(uint16_t)mbdat;
				}
			else if(mbadr==16)		//��������� ������� 
				{
				LPC_RTC->SEC=(uint16_t)mbdat;
				}
			else if(mbadr==20)		//���������� ���
				{
				if((mbdat>0)&&(mbdat<=32))
				lc640_write_int(EE_NUMIST,mbdat);  
				}
			else if(mbadr==22)		//�������� ������������ ������
				{
				if((mbdat==0)||(mbdat==1))
				lc640_write_int(EE_ZV_ON,mbdat);  
				}
			else if(mbadr==31)		//������������ (���������) ���������� ������������, 0.1�
				{
				lc640_write_int(EE_UMAX,mbdat);
	     		}
			else if(mbadr==43)		//����� �������� ��������� ������������, ���
				{
				if(mbdat>=3 && mbdat<=60) lc640_write_int(EE_TZAS,mbdat);
	     		}
			else if(mbadr==44)		//����������� ������������ ���������, 1��
				{
				lc640_write_int(EE_TMAX,mbdat);
	     		}
			else if(mbadr==45)		//����������� ������������ ����������, 1��
				{
				lc640_write_int(EE_TSIGN,mbdat);
	     		}
			else if(mbadr==60)//60	�������� ���������� ��� ��� ������ � ���
				{
				lc640_write_int(EE_UOUT,mbdat);
				}
			else if(mbadr==61)//61  ���������� �������� ���������� ��� ��� ������ ��� ��� 
				{
				lc640_write_int(EE_UAVT,mbdat);
				}
			else if(mbadr==62)//62 ����� �������� ������� 
				{
				lc640_write_int(EE_TSYSMAX,mbdat);
				}
			else if(mbadr==63)//63 	������� ������ �� ������������� ��������� ����������
				{
				lc640_write_int(EE_DU,mbdat);
				}
			else if(mbadr==64)//64	������� ������������� �������� ���������� 
				{
				lc640_write_int(EE_UINMAX,mbdat);
				}
			else if(mbadr==65)//65	������� ������������ �������� ���������� 
				{
				lc640_write_int(EE_UINMIN,mbdat);
				}
			else if(mbadr==66)//66	������� ������������� ��������� ����������
				{
				lc640_write_int(EE_UOUTMAX,mbdat);
				}
			else if(mbadr==67)//67	������� ������������ ��������� ���������� 
				{
				lc640_write_int(EE_UOUTMIN,mbdat);
				}
			else if(mbadr==68)//68 ����� ������� �����������
				{
				if(mbdat<=6000) lc640_write_int(EE_TVENTMAX,mbdat);
				}
			else if(mbadr==69)//69 ����� ������
				{
				lc640_write_int(EE_MODBUS_ADRESS,mbdat);
				}
			else if(mbadr==70)//70 �������� ������
				{
				lc640_write_int(EE_MODBUS_BAUDRATE,mbdat);
				}
			else if(mbadr==71)//71 ������ ��������� ��������� ���� - �� �����=1 ��� ��� ����� ����� ����������=0
				{
				if(mbdat==0 || mbdat==1) lc640_write_int(EE_I_LOAD_MODE,mbdat);
				}
			else if(mbadr==72)//72 ���������� ���� 1
				{
				lc640_write_int(EE_RELE_SET_MASK0,mbdat);
				}
			else if(mbadr==73)//73 ���������� ���� 2
				{
				lc640_write_int(EE_RELE_SET_MASK1,mbdat);
				}
			else if(mbadr==74)//74 ���������� ���� 3
				{
				lc640_write_int(EE_RELE_SET_MASK2,mbdat);
				}
			else if(mbadr==75)//75 ���������� ���� 4
				{
				lc640_write_int(EE_RELE_SET_MASK3,mbdat);
				}
			else if(mbadr==76)//76 ���������� ���������� �������, 1-��������������, 0-������
				{
				if(mbdat==0 || mbdat==1) lc640_write_int(EE_AV_OFF_AVT,mbdat);
				}
			else if(mbadr==77)//77 ���������� �� ����, 1�
				{
				if(mbdat>0 && mbdat<=2000) lc640_write_int(EE_OVERLOAD_CURR,mbdat);
				}
			else if(mbadr==78)//78 ����� �������� ���������� �� ����, 1���
				{
				if(mbdat>0 && mbdat<=120) lc640_write_int(EE_OVERLOAD_TIME,mbdat);
				}





}
//-----------



