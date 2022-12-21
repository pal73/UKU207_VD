
#include <lpc17xx.h>
#include "modbus.h"
//#include "LPC17xx.H"
#include "main.h"
#include "control.h"
#include <string.h>

#include "eeprom_map.h"
#include "rtl.h"
#include "modbus_tcp.h"
#include "25lc640.h"
#include "sc16is7xx.h"
#include "uart0.h"
#include "avar_hndl.h"

#define MODBUS_RTU_PROT	0

extern int  mem_copy (void *dp, void *sp, int len);

unsigned char modbus_buf[20];
short modbus_crc16;
char modbus_timeout_cnt;
char bMODBUS_TIMEOUT;
unsigned char modbus_rx_buffer[30];	//�����, ���� ���������� ����������� ������� ���������� ���������� �� ������ ����� 
unsigned char modbus_an_buffer[30];    	//�����, ���� ��� ����� ���������� ��� �������
unsigned char modbus_rx_buffer_ptr;	//��������� �� ������� ������� ������������ ������
unsigned char modbus_rx_counter;		//���������� �������� ����, ������������ ��� ������� ����������� ������� � ��� �����������

short modbus_plazma;				//�������
short modbus_plazma1;				//�������
short modbus_plazma2;				//�������
short modbus_plazma3;				//�������

unsigned short modbus_rx_arg0;		//���������� � ������� ������ ��������
unsigned short modbus_rx_arg1;		//���������� � ������� ������ ��������
unsigned short modbus_rx_arg2;		//���������� � ������� ������ ��������
unsigned short modbus_rx_arg3;		//���������� � ������� ��������� ��������

char modbus_tx_buff[100];

//char modbus_registers[200];

//static const char foo[] = "I wish I'd read K&R, and other tomes more diligently";



/*modbus_registers[3]=(char)(bps_I%256);
modbus_registers[4]=(char)(net_U/256);					//���3   	���������� ���� �������, 1�
modbus_registers[5]=(char)(net_U%256);
modbus_registers[6]=(char)(net_F/256);					//���4   	������� ���� �������, 0.1��
modbus_registers[7]=(char)(net_F%256);
modbus_registers[8]=(char)(net_Ua/256);					//���5	���������� ���� ������� ���� A, 1�	
modbus_registers[9]=(char)(net_Ua%256);		 	
modbus_registers[10]=(char)(net_Ub/256);				//���6	���������� ���� ������� ���� B, 1�
modbus_registers[11]=(char)(net_Ub%256);
modbus_registers[12]=(char)(net_Uc/256);				//���7	���������� ���� ������� ���� C, 1�
modbus_registers[13]=(char)(net_Uc%256);
modbus_registers[14]=(char)(bat[0]._Ub/256);				//���8	���������� ������� �1, 0.1�
modbus_registers[15]=(char)(bat[0]._Ub%256);
modbus_registers[16]=(char)(bat[0]._Ib/256);				//���9   	��� ������� �1, 0.01�
modbus_registers[17]=(char)(bat[0]._Ib%256);
modbus_registers[18]=(char)(bat[0]._Tb/256);				//���10	����������� ������� �1, 1��
modbus_registers[19]=(char)(bat[0]._Tb%256);
modbus_registers[20]=(char)(bat[0]._zar/256);			//���11	����� ������� �1, %
modbus_registers[21]=(char)(bat[0]._zar%256);
modbus_registers[22]=(char)(bat[0]._Ubm/256);			//���12	���������� ������� ����� ������� �1, 0.1�
modbus_registers[23]=(char)(bat[0]._Ubm%256);
modbus_registers[24]=(char)(bat[0]._dUbm/256);			//���13	������ ������� ����� ������� �1, %
modbus_registers[25]=(char)(bat[0]._dUbm%256);
modbus_registers[26]=(char)(BAT_C_REAL[0]/256);			//���14	�������� ������� ������� �1, 0.1�*�, ���� 0x5555 �� �� ����������
modbus_registers[27]=(char)(BAT_C_REAL[0]%256);
modbus_registers[28]=(char)(bat[1]._Ub/256);				//���15	���������� ������� �1, 0.1�
modbus_registers[29]=(char)(bat[1]._Ub%256);
modbus_registers[30]=(char)(bat[1]._Ib/256);				//���16   	��� ������� �1, 0.01�
modbus_registers[31]=(char)(bat[1]._Ib%256);
modbus_registers[32]=(char)(bat[1]._Tb/256);				//���17	����������� ������� �1, 1��
modbus_registers[33]=(char)(bat[1]._Tb%256);
modbus_registers[34]=(char)(bat[1]._zar/256);			//���18	����� ������� �1, %
modbus_registers[35]=(char)(bat[1]._zar%256);
modbus_registers[36]=(char)(bat[1]._Ubm/256);			//���19	���������� ������� ����� ������� �1, 0.1�
modbus_registers[37]=(char)(bat[1]._Ubm%256);
modbus_registers[38]=(char)(bat[1]._dUbm/256);			//���20	������ ������� ����� ������� �1, %
modbus_registers[39]=(char)(bat[1]._dUbm%256);
modbus_registers[40]=(char)(BAT_C_REAL[1]/256);			//���21	�������� ������� ������� �1, 0.1�*�, ���� 0x5555 �� �� ����������
modbus_registers[41]=(char)(BAT_C_REAL[1]%256);
modbus_registers[42]=(char)(bps[0]._Uii/256);			//���22	�������� ���������� ����������� �1, 0.1�
modbus_registers[43]=(char)(bps[0]._Uii%256);
modbus_registers[44]=(char)(bps[0]._Ii/256);				//���23	�������� ��� ����������� �1, 0.1�
modbus_registers[45]=(char)(bps[0]._Ii%256);
modbus_registers[46]=(char)(bps[0]._Ti/256);				//���24	����������� ��������� ����������� �1, 1��
modbus_registers[47]=(char)(bps[0]._Ti%256);
modbus_registers[48]=(char)(bps[0]._av/256);				//���25	���� ������ ����������� �1, 0x01 - ��������, 0x02 �������� U���, 0x04 �������� U���, 0x08 - ����������� ����� � ������������
modbus_registers[49]=(char)(bps[0]._av%256);
modbus_registers[50]=(char)(bps[1]._Uii/256);			//���26	�������� ���������� ����������� �2, 0.1�
modbus_registers[51]=(char)(bps[1]._Uii%256);
modbus_registers[52]=(char)(bps[1]._Ii/256);				//���27	�������� ��� ����������� �2, 0.1�
modbus_registers[53]=(char)(bps[1]._Ii%256);
modbus_registers[54]=(char)(bps[1]._Ti/256);				//���28	����������� ��������� ����������� �2, 1��
modbus_registers[55]=(char)(bps[1]._Ti%256);
modbus_registers[56]=(char)(bps[1]._av/256);				//���29	���� ������ ����������� �2, 0x01 - ��������, 0x02 �������� U���, 0x04 �������� U���, 0x08 - ����������� ����� � ������������
modbus_registers[57]=(char)(bps[1]._av%256);
modbus_registers[58]=(char)(bps[2]._Uii/256);			//���30	�������� ���������� ����������� �3, 0.1�
modbus_registers[59]=(char)(bps[2]._Uii%256);
modbus_registers[60]=(char)(bps[2]._Ii/256);				//���31	�������� ��� ����������� �3, 0.1�
modbus_registers[61]=(char)(bps[2]._Ii%256);
modbus_registers[62]=(char)(bps[2]._Ti/256);				//���32	����������� ��������� ����������� �3, 1��
modbus_registers[63]=(char)(bps[2]._Ti%256);
modbus_registers[64]=(char)(bps[2]._av/256);				//���33	���� ������ ����������� �3, 0x01 - ��������, 0x02 �������� U���, 0x04 �������� U���, 0x08 - ����������� ����� � ������������
modbus_registers[65]=(char)(bps[2]._av%256);
modbus_registers[66]=(char)(bps[3]._Uii/256);			//���34	�������� ���������� ����������� �4, 0.1�
modbus_registers[67]=(char)(bps[3]._Uii%256);
modbus_registers[68]=(char)(bps[3]._Ii/256);				//���35	�������� ��� ����������� �4, 0.1�
modbus_registers[69]=(char)(bps[3]._Ii%256);
modbus_registers[70]=(char)(bps[3]._Ti/256);				//���36	����������� ��������� ����������� �4, 1��
modbus_registers[71]=(char)(bps[3]._Ti%256);
modbus_registers[72]=(char)(bps[3]._av/256);				//���37	���� ������ ����������� �4, 0x01 - ��������, 0x02 �������� U���, 0x04 �������� U���, 0x08 - ����������� ����� � ������������
modbus_registers[73]=(char)(bps[3]._av%256);
modbus_registers[74]=(char)(bps[4]._Uii/256);			//���38	�������� ���������� ����������� �5, 0.1�
modbus_registers[75]=(char)(bps[4]._Uii%256);
modbus_registers[76]=(char)(bps[4]._Ii/256);				//���39	�������� ��� ����������� �5, 0.1�
modbus_registers[77]=(char)(bps[4]._Ii%256);
modbus_registers[78]=(char)(bps[4]._Ti/256);				//���40	����������� ��������� ����������� �5, 1��
modbus_registers[79]=(char)(bps[4]._Ti%256);
modbus_registers[80]=(char)(bps[4]._av/256);				//���41	���� ������ ����������� �5, 0x01 - ��������, 0x02 �������� U���, 0x04 �������� U���, 0x08 - ����������� ����� � ������������
modbus_registers[81]=(char)(bps[4]._av%256);
modbus_registers[82]=(char)(bps[5]._Uii/256);			//���42	�������� ���������� ����������� �6, 0.1�
modbus_registers[83]=(char)(bps[5]._Uii%256);
modbus_registers[84]=(char)(bps[5]._Ii/256);				//���43	�������� ��� ����������� �6, 0.1�
modbus_registers[85]=(char)(bps[5]._Ii%256);
modbus_registers[86]=(char)(bps[5]._Ti/256);				//���44	����������� ��������� ����������� �6, 1��
modbus_registers[87]=(char)(bps[5]._Ti%256);
modbus_registers[88]=(char)(bps[5]._av/256);				//���45	���� ������ ����������� �6, 0x01 - ��������, 0x02 �������� U���, 0x04 �������� U���, 0x08 - ����������� ����� � ������������
modbus_registers[89]=(char)(bps[5]._av%256);
modbus_registers[90]=(char)(bps[6]._Uii/256);			//���46	�������� ���������� ����������� �7, 0.1�
modbus_registers[91]=(char)(bps[6]._Uii%256);
modbus_registers[92]=(char)(bps[6]._Ii/256);				//���47	�������� ��� ����������� �7, 0.1�
modbus_registers[93]=(char)(bps[6]._Ii%256);
modbus_registers[94]=(char)(bps[6]._Ti/256);				//���48	����������� ��������� ����������� �7, 1��
modbus_registers[95]=(char)(bps[6]._Ti%256);
modbus_registers[96]=(char)(bps[6]._av/256);				//���49	���� ������ ����������� �7, 0x01 - ��������, 0x02 �������� U���, 0x04 �������� U���, 0x08 - ����������� ����� � ������������
modbus_registers[97]=(char)(bps[6]._av%256);
modbus_registers[98]=(char)(bps[7]._Uii/256);			//���50	�������� ���������� ����������� �8, 0.1�
modbus_registers[99]=(char)(bps[7]._Uii%256);
modbus_registers[100]=(char)(bps[7]._Ii/256);			//���51	�������� ��� ����������� �8, 0.1�
modbus_registers[101]=(char)(bps[7]._Ii%256);
modbus_registers[102]=(char)(bps[7]._Ti/256);			//���52	����������� ��������� ����������� �8, 1��
modbus_registers[103]=(char)(bps[7]._Ti%256);
modbus_registers[104]=(char)(bps[7]._av/256);			//���53	���� ������ ����������� �8, 0x01 - ��������, 0x02 �������� U���, 0x04 �������� U���, 0x08 - ����������� ����� � ������������
modbus_registers[105]=(char)(bps[7]._av%256);
modbus_registers[106]=(char)(bps_U/256);					//���54   	���������� ������������, 0.1�
modbus_registers[107]=(char)(bps_U%256);
tempS=0;
if(speedChIsOn) tempS=1;
modbus_registers[108]=(char)(tempS/256);					//���55   	���������� ����� ������������, (1 - ���, 0 - ����)
modbus_registers[109]=(char)(tempS%256);
tempS=0;
if(spc_stat==spcVZ) tempS=1;
modbus_registers[110]=(char)(tempS/256);					//���56   	������������� ����� ������������, (1 - ���, 0 - ����)
modbus_registers[111]=(char)(tempS%256);
modbus_registers[112]=(char)(uout_av/256);					//���57   �������� ��������� ����������, (0 - �����, 1 - ��������, 2 - ��������)
modbus_registers[113]=(char)(uout_av%256);

tempS=t_ext[0];
if(ND_EXT[0])tempS=-1000;
modbus_registers[400]=(char)(tempS/256);				//���201	������� ������ ����������� �1
modbus_registers[401]=(char)(tempS%256);
tempS=t_ext[1];
if(ND_EXT[1])tempS=-1000;
modbus_registers[402]=(char)(tempS/256);				//���202	������� ������ ����������� �2
modbus_registers[403]=(char)(tempS%256);
tempS=t_ext[2];
if(ND_EXT[2])tempS=-1000;
modbus_registers[404]=(char)(tempS/256);				//���203	������� ������ ����������� �3
modbus_registers[405]=(char)(tempS%256);
tempS=t_ext[3];
if(ND_EXT[3])tempS=-1000;
modbus_registers[406]=(char)(tempS/256);				//���204	������� ������ ����������� �4
modbus_registers[407]=(char)(tempS%256);

tempS=0;
if(sk_stat[0]==ssON) tempS|=0x0001;
if(sk_av_stat[0]==sasON) tempS|=0x0002;
modbus_registers[420]=(char)(tempS/256);				//���211	���������  ������ �������� �1, (������� ��� - ���������� ���������, 1 - �������, 0 - ���������, ������ ��� - �����������, 1 - ������, 0 - �����)
modbus_registers[421]=(char)(tempS%256);
tempS=0;
if(sk_stat[1]==ssON) tempS|=0x0001;
if(sk_av_stat[1]==sasON) tempS|=0x0002;
modbus_registers[422]=(char)(tempS/256);				//���212	���������  ������ �������� �1, (������� ��� - ���������� ���������, 1 - �������, 0 - ���������, ������ ��� - �����������, 1 - ������, 0 - �����)
modbus_registers[423]=(char)(tempS%256);
tempS=0;
if(sk_stat[2]==ssON) tempS|=0x0001;
if(sk_av_stat[2]==sasON) tempS|=0x0002;
modbus_registers[424]=(char)(tempS/256);				//���213	���������  ������ �������� �1, (������� ��� - ���������� ���������, 1 - �������, 0 - ���������, ������ ��� - �����������, 1 - ������, 0 - �����)
modbus_registers[425]=(char)(tempS%256);
tempS=0;
if(sk_stat[3]==ssON) tempS|=0x0001;
if(sk_av_stat[3]==sasON) tempS|=0x0002;
modbus_registers[426]=(char)(tempS/256);				//���214	���������  ������ �������� �1, (������� ��� - ���������� ���������, 1 - �������, 0 - ���������, ������ ��� - �����������, 1 - ������, 0 - �����)
modbus_registers[427]=(char)(tempS%256);

if(prot==MODBUS_RTU_PROT)
	{
	modbus_tx_buff[0]=adr;
	modbus_tx_buff[1]=func;

	modbus_tx_buff[2]=(char)(reg_quantity*2);

	mem_copy((char*)&modbus_tx_buff[3],(char*)&modbus_registers[(reg_adr-1)*2],reg_quantity*2);

	crc_temp=CRC16_2(modbus_tx_buff,(reg_quantity*2)+3);

	modbus_tx_buff[3+(reg_quantity*2)]=crc_temp%256;
	modbus_tx_buff[4+(reg_quantity*2)]=crc_temp/256;

	for (i=0;i<(5+(reg_quantity*2));i++)
		{
		putchar0(modbus_tx_buff[i]);
		}
	for (i=0;i<(5+(reg_quantity*2));i++)
		{
		putchar_sc16is700(modbus_tx_buff[i]);
		}
	}
else if(prot==MODBUS_TCP_PROT)
	{
	modbus_tcp_out_ptr=(char*)&modbus_registers[(reg_adr-1)*2];
	} */

	//   	};


//-----------------------------------------------
unsigned short CRC16_2(char* buf, short len)
{
unsigned short crc = 0xFFFF;
short pos;
short i;

for (pos = 0; pos < len; pos++)
  	{
    	crc ^= (unsigned short)buf[pos];          // XOR byte into least sig. byte of crc

    	for ( i = 8; i != 0; i--) 
		{    // Loop over each bit
      	if ((crc & 0x0001) != 0) 
			{      // If the LSB is set
        		crc >>= 1;                    // Shift right and XOR 0xA001
        		crc ^= 0xA001;
      		}
      	else  crc >>= 1;                    // Just shift right
    		}
  	}
  // Note, this number has low and high bytes swapped, so use it accordingly (or swap bytes)
return crc;
}

//-----------------------------------------------
void modbus_in(void)
{
short crc16_calculated;		//����������� �� �������� ������ CRC
short crc16_incapsulated;	//����������� � ������� CRC
unsigned short modbus_rx_arg0;		//���������� � ������� ������ ��������
unsigned short modbus_rx_arg1;		//���������� � ������� ������ ��������
//unsigned short modbus_rx_arg2;		//���������� � ������� ������ ��������
//unsigned short modbus_rx_arg3;		//���������� � ������� ��������� ��������
unsigned char modbus_func;			//���������� � ������� ��� �������



mem_copy(modbus_an_buffer,modbus_rx_buffer,modbus_rx_buffer_ptr);
modbus_rx_counter=modbus_rx_buffer_ptr;
modbus_rx_buffer_ptr=0;
bMODBUS_TIMEOUT=0;
	
crc16_calculated  = CRC16_2((char*)modbus_an_buffer, modbus_rx_counter-2);
crc16_incapsulated = *((short*)&modbus_an_buffer[modbus_rx_counter-2]);

modbus_plazma1=modbus_rx_counter;
modbus_plazma2=crc16_calculated;
modbus_plazma3=crc16_incapsulated;

modbus_func=modbus_an_buffer[1];
modbus_rx_arg0=(((unsigned short)modbus_an_buffer[2])*((unsigned short)256))+((unsigned short)modbus_an_buffer[3]);
modbus_rx_arg1=(((unsigned short)modbus_an_buffer[4])*((unsigned short)256))+((unsigned short)modbus_an_buffer[5]);
//modbus_rx_arg2=(((unsigned short)modbus_an_buffer[6])*((unsigned short)256))+((unsigned short)modbus_an_buffer[7]);
//modbus_rx_arg3=(((unsigned short)modbus_an_buffer[8])*((unsigned short)256))+((unsigned short)modbus_an_buffer[9]);

//#define IPS_CURR_AVG_MODBUS_ADRESS	1


if(modbus_an_buffer[0]=='r')
	{
	pvlk=1;
	if(modbus_an_buffer[1]=='e')
		{
		pvlk=2;
		if(modbus_an_buffer[2]=='a')
			{
			pvlk=3;
			if(modbus_an_buffer[3]=='d')
				{
				pvlk=4;
				if(modbus_an_buffer[6]==crc_87(modbus_an_buffer,6))
					{
					pvlk=5;
					if(modbus_an_buffer[7]==crc_95(modbus_an_buffer,6))
						{
						pvlk=6;	

							{
							unsigned short ptr;
							unsigned long data1,data2;
							char temp_out[20];
							pvlk++;
							ptr=modbus_an_buffer[4]+(modbus_an_buffer[5]*256U);
							data1=lc640_read_long(ptr);
							data2=lc640_read_long(ptr+4);
							temp_out[0]='r';
							temp_out[1]='e';
							temp_out[2]='a';
							temp_out[3]='d';
							temp_out[4]=*((char*)&ptr);
							temp_out[5]=*(((char*)&ptr)+1);	
							temp_out[6]=*((char*)&data1);
							temp_out[7]=*(((char*)&data1)+1);		
							temp_out[8]=*(((char*)&data1)+2);	
							temp_out[9]=*(((char*)&data1)+3);		
							temp_out[10]=*((char*)&data2);
							temp_out[11]=*(((char*)&data2)+1);		
							temp_out[12]=*(((char*)&data2)+2);	
							temp_out[13]=*(((char*)&data2)+3);	
							temp_out[14]=crc_87(temp_out,14);	
							temp_out[15]=crc_95(temp_out,14);			
							
							temp_out[17]=0;
							for (i=0;i<16;i++)
								{
								putchar_sc16is700(temp_out[i]);
								temp_out[17]^=temp_out[i];
								}
							putchar_sc16is700(16);
							putchar_sc16is700(temp_out[17]^16);
							putchar_sc16is700(0x0a);
							}
						}
					}
				}
			} 
		}	 
	} 

if(modbus_an_buffer[0]=='w')
	{
//	pvlk=1;
	if(modbus_an_buffer[1]=='r')
		{
//		pvlk=2;
		if(modbus_an_buffer[2]=='i')
			{
//			pvlk=3;
			if(modbus_an_buffer[3]=='t')
				{
//				pvlk=4;
				if(modbus_an_buffer[4]=='e')
					{
//					pvlk=5;
					if(modbus_an_buffer[15]==crc_87(modbus_an_buffer,15))
						{
//						pvlk=6;
						if(modbus_an_buffer[16]==crc_95(modbus_an_buffer,15))

							{
							unsigned short ptr;
							unsigned long data1,data2;
							char temp_out[20];
//							pvlk=7;
							ptr=modbus_an_buffer[5]+(modbus_an_buffer[6]*256U);
							*((char*)&data1)=modbus_an_buffer[7];
							*(((char*)&data1)+1)=modbus_an_buffer[8];
							*(((char*)&data1)+2)=modbus_an_buffer[9];
							*(((char*)&data1)+3)=modbus_an_buffer[10];
							*((char*)&data2)=modbus_an_buffer[11];
							*(((char*)&data2)+1)=modbus_an_buffer[12];
							*(((char*)&data2)+2)=modbus_an_buffer[13];
							*(((char*)&data2)+3)=modbus_an_buffer[14];	
							lc640_write_long(ptr,data1);
							lc640_write_long(ptr+4,data2);
							
							//data1=lc640_read_long(ptr);
							//data2=lc640_read_long(ptr+4);
							temp_out[0]='w';
							temp_out[1]='r';
							temp_out[2]='i';
							temp_out[3]='t';
							temp_out[4]='e';
							temp_out[5]=*((char*)&ptr);
							temp_out[6]=*(((char*)&ptr)+1);	
						
							temp_out[7]=crc_87(temp_out,7);	
							temp_out[8]=crc_95(temp_out,7);			
							
							temp_out[10]=0;
							for (i=0;i<9;i++)
								{
								putchar_sc16is700(temp_out[i]);
								temp_out[10]^=temp_out[i];
								}
							putchar_sc16is700(9);
							putchar_sc16is700(temp_out[10]^9);
							putchar_sc16is700(0x0a);
							}
						}
					}
				}
		   	}
		}
	}

if(crc16_calculated==crc16_incapsulated)
	{
	ica_plazma[4]++;
 	if(modbus_an_buffer[0]==MODBUS_ADRESS)
		{
		if(modbus_func==3)		//������ ������������� ���-�� ��������� ��������
			{
			modbus_plazma++;
			modbus_hold_registers_transmit(MODBUS_ADRESS,modbus_func,modbus_rx_arg0,modbus_rx_arg1,MODBUS_RTU_PROT);
			}

		if(modbus_func==4)		//������ ������������� ���-�� ���������	������
			{
			modbus_input_registers_transmit(MODBUS_ADRESS,modbus_func,modbus_rx_arg0,modbus_rx_arg1,MODBUS_RTU_PROT);
			}

		else if(modbus_func==6) 	//������ ��������� ��������
			{
			if(modbus_rx_arg0==11)		//��������� ������� 
				{
				LPC_RTC->YEAR=(uint16_t)modbus_rx_arg1;
				}
			if(modbus_rx_arg0==12)		//��������� ������� 
				{
				LPC_RTC->MONTH=(uint16_t)modbus_rx_arg1;
				}
			if(modbus_rx_arg0==13)		//��������� ������� 
				{
				LPC_RTC->DOM=(uint16_t)modbus_rx_arg1;
				}
			if(modbus_rx_arg0==14)		//��������� ������� 
				{
				LPC_RTC->HOUR=(uint16_t)modbus_rx_arg1;
				}
			if(modbus_rx_arg0==15)		//��������� ������� 
				{
				LPC_RTC->MIN=(uint16_t)modbus_rx_arg1;
				}
			if(modbus_rx_arg0==16)		//��������� ������� 
				{
				LPC_RTC->SEC=(uint16_t)modbus_rx_arg1;
				}
			if(modbus_rx_arg0==20)		//��� ������������ ��� ������ ������������ ����
				{
				if((modbus_rx_arg1>0)&&(modbus_rx_arg1<=18))
				lc640_write_int(EE_NUMIST,modbus_rx_arg1);  
				}
			if(modbus_rx_arg0==21)		//��� ������������ ��� ������ ������������ ����
				{
				if((modbus_rx_arg1==0)||(modbus_rx_arg1==1))
				lc640_write_int(EE_PAR,modbus_rx_arg1);  
				}
			if(modbus_rx_arg0==22)		//��� ������������ ��� ������ ������������ ����
				{
				if((modbus_rx_arg1==0)||(modbus_rx_arg1==1))
				lc640_write_int(EE_ZV_ON,modbus_rx_arg1);  
				}
			if(modbus_rx_arg0==23)		//��� ������������ ��� ������ ������������ ����
				{
				if((modbus_rx_arg1==0)||(modbus_rx_arg1==1))
				lc640_write_int(EE_TERMOKOMP,modbus_rx_arg1);  
				}
			if(modbus_rx_arg0==24)		//��� ������������ ��� ������ ������������ ����
				{
				if(/*(modbus_rx_arg1>=0)||*/(modbus_rx_arg1<=20))
				lc640_write_int(EE_UBM_AV,modbus_rx_arg1);  
				}


			if(modbus_rx_arg0==30)		//���������� ������������ ��� ������ ������������ ����������
				{
				/*if(modbus_rx_arg1<0)TBAT=0;
				else */if((modbus_rx_arg1>0)&&(modbus_rx_arg1<=5))modbus_rx_arg1=0;
				else if(modbus_rx_arg1>=60)TBAT=60;
				else TBAT=modbus_rx_arg1;
				lc640_write_int(EE_TBAT,TBAT);

				main_kb_cnt=(TBAT*60)-20;
	     		}
			if(modbus_rx_arg0==31)		//
				{
				lc640_write_int(EE_UMAX,modbus_rx_arg1);
	     		}
			if(modbus_rx_arg0==32)		//
				{
				//lc640_write_int(EE_DU,UB20-modbus_rx_arg1);
	     		}
			if(modbus_rx_arg0==33)		//
				{
				//lc640_write_int(EE_UB0,modbus_rx_arg1);
	     		}
			if(modbus_rx_arg0==34)		//
				{
				//lc640_write_int(EE_UB20,modbus_rx_arg1);
	     		}
			if(modbus_rx_arg0==35)		//
				{
				lc640_write_int(EE_USIGN,modbus_rx_arg1);
	     		}
			if(modbus_rx_arg0==36)		//
				{
				lc640_write_int(EE_UMN,modbus_rx_arg1);
	     		}
			if(modbus_rx_arg0==37)		//
				{
				lc640_write_int(EE_U0B,modbus_rx_arg1);
	     		}
			if(modbus_rx_arg0==38)		//
				{
				lc640_write_int(EE_IKB,modbus_rx_arg1);
	     		}
			if(modbus_rx_arg0==39)		//
				{
				lc640_write_int(EE_IZMAX,modbus_rx_arg1);
	     		}
			if(modbus_rx_arg0==40)		//
				{
				lc640_write_int(EE_IMAX,modbus_rx_arg1);
	     		}
			if(modbus_rx_arg0==41)		//
				{
				lc640_write_int(EE_IMIN,modbus_rx_arg1);
	     		}
			if(modbus_rx_arg0==42)		//
				{
				//lc640_write_int(EE_UVZ,modbus_rx_arg1);
	     		}
			if(modbus_rx_arg0==43)		//
				{
				lc640_write_int(EE_TZAS,modbus_rx_arg1);
	     		}
			if(modbus_rx_arg0==44)		//
				{
				lc640_write_int(EE_TMAX,modbus_rx_arg1);
	     		}
			if(modbus_rx_arg0==45)		//
				{
				lc640_write_int(EE_TSIGN,modbus_rx_arg1);
	     		}
			if(modbus_rx_arg0==46)		//
				{
				lc640_write_int(EE_TBATMAX,modbus_rx_arg1);
	     		}
			if(modbus_rx_arg0==47)		//
				{
				lc640_write_int(EE_TBATSIGN,modbus_rx_arg1);
	     		}
			if(modbus_rx_arg0==48)		//
				{
				lc640_write_int(EE_SPEED_CHRG_CURR,modbus_rx_arg1);
	     		}
			if(modbus_rx_arg0==49)		//
				{
				lc640_write_int(EE_SPEED_CHRG_VOLT,modbus_rx_arg1);
	     		}
			if(modbus_rx_arg0==50)		//
				{
				lc640_write_int(EE_SPEED_CHRG_TIME,modbus_rx_arg1);
	     		}
			if(modbus_rx_arg0==51)		//
				{
				lc640_write_int(EE_U_OUT_KONTR_MAX,modbus_rx_arg1);
	     		}
			if(modbus_rx_arg0==52)		//
				{
				lc640_write_int(EE_U_OUT_KONTR_MIN,modbus_rx_arg1);
	     		}
			if(modbus_rx_arg0==53)		//
				{
				lc640_write_int(EE_U_OUT_KONTR_DELAY,modbus_rx_arg1);
	     		}

			if(modbus_rx_arg0==19)		//���/���� ��������� ����.
				{
	/*			if(modbus_rx_arg1==1)
					{
					if(work_stat!=wsPS)
						{
						work_stat=wsPS;
						time_proc=0;
						time_proc_remain=T_PROC_PS;
						restart_on_PS();
						lc640_write_int(EE_MAIN_MENU_MODE,mmmIN);
						}
					}
				if(modbus_rx_arg1==0)
					{
					if(work_stat==wsPS)
						{
						work_stat=wsOFF;
						restart_off();
						}
					} */
				}
			if(modbus_rx_arg0==20)		//���/���� ��������� ����
				{
/*				if(modbus_rx_arg1==1)
					{
					if(work_stat!=wsGS)
						{
						work_stat=wsGS;
						time_proc=0;
						time_proc_remain=T_PROC_GS;
						lc640_write_int(EE_MAIN_MENU_MODE,mmmIT);
						}
					}
				if(modbus_rx_arg1==0)
					{
					if(work_stat==wsGS)
						{
						work_stat=wsOFF;
						restart_off();
						}
					}*/
				}
			//modbus_hold_register_transmit(MODBUS_ADRESS,modbus_func,modbus_rx_arg0);
			modbus_hold_registers_transmit(MODBUS_ADRESS,modbus_func,modbus_rx_arg0,1,MODBUS_RTU_PROT);
			}
		} 
	else if(modbus_an_buffer[0]==ICA_MODBUS_ADDRESS)
		{
		ica_plazma[3]++;
		if(modbus_func==4)		//������ ������������� ���-�� ���������	������
			{
			ica_plazma[2]++;
			if(modbus_an_buffer[2]==2)
				{
				ica_your_current=(((unsigned short)modbus_an_buffer[3])*((unsigned short)256))+((unsigned short)modbus_an_buffer[4]);
				}
			}
		}
	
	}


}
/*
//-----------------------------------------------
void modbus_register_transmit(unsigned char adr,unsigned char func,unsigned short reg_adr)
{
char modbus_registers[150];
//char modbus_tx_buff[50];
unsigned short crc_temp;
char i;


modbus_registers[0]=(char)(load_U/256);					//���1
modbus_registers[1]=(char)(load_U%256);
modbus_registers[2]=(char)(load_I/256);					//���2
modbus_registers[3]=(char)(load_I%256);
modbus_registers[4]=(char)(Ib_ips_termokompensat/256);		//���3
modbus_registers[5]=(char)(Ib_ips_termokompensat%256);
modbus_registers[6]=(char)(t_ext[0]/256);				//���4
modbus_registers[7]=(char)(t_ext[0]%256);
modbus_registers[8]=(char)(net_Ua/256);					//���5
modbus_registers[9]=(char)(net_Ua%256);		 	
modbus_registers[10]=(char)(net_Ub/256);				//���6
modbus_registers[11]=(char)(net_Ub%256);
modbus_registers[12]=(char)(net_Uc/256);				//���7
modbus_registers[13]=(char)(net_Uc%256);
modbus_registers[14]=(char)(net_F3/256);				//���8
modbus_registers[15]=(char)(net_F3%256);
modbus_registers[16]=(char)(load_I/256);				//���9
modbus_registers[17]=(char)(load_I%256);
modbus_registers[18]=(char)(load_I/256);				//���10
modbus_registers[19]=(char)(load_I%256);
modbus_registers[20]=(char)((LPC_RTC->YEAR)/256);			//���11
modbus_registers[21]=(char)((LPC_RTC->YEAR)%256);
modbus_registers[22]=(char)((LPC_RTC->MONTH)/256);		//���12
modbus_registers[23]=(char)((LPC_RTC->MONTH)%256);
modbus_registers[24]=(char)((LPC_RTC->DOM)/256);			//���13
modbus_registers[25]=(char)((LPC_RTC->DOM)%256);
modbus_registers[26]=(char)((LPC_RTC->HOUR)/256);			//���14
modbus_registers[27]=(char)((LPC_RTC->HOUR)%256);
modbus_registers[28]=(char)((LPC_RTC->MIN)/256);			//���15
modbus_registers[29]=(char)((LPC_RTC->MIN)%256);
modbus_registers[30]=(char)((LPC_RTC->SEC)/256);			//���16
modbus_registers[31]=(char)((LPC_RTC->SEC)%256);
modbus_registers[32]=(char)(load_I/256);				//���17
modbus_registers[33]=(char)(load_I%256);
modbus_registers[34]=(char)(load_I/256);				//���18
modbus_registers[35]=(char)(load_I%256);
modbus_registers[36]=(char)(load_I/256);		//���19
modbus_registers[37]=(char)(load_I%256);
modbus_registers[38]=(char)(NUMIST/256);		//���20
modbus_registers[39]=(char)(NUMIST%256);
modbus_registers[40]=(char)(PAR/256);		//���21
modbus_registers[41]=(char)(PAR%256);
modbus_registers[42]=(char)(ZV_ON/256);		//���22
modbus_registers[43]=(char)(ZV_ON%256);
modbus_registers[44]=(char)(TERMOKOMPENS/256);		//���23
modbus_registers[45]=(char)(TERMOKOMPENS%256);
modbus_registers[46]=(char)(UBM_AV/256);		//���24
modbus_registers[47]=(char)(UBM_AV%256);
modbus_registers[48]=(char)(load_I/256);		//���25
modbus_registers[49]=(char)(load_I%256);
modbus_registers[50]=(char)(load_I/256);		//���26
modbus_registers[51]=(char)(load_I%256);
modbus_registers[52]=(char)(load_I/256);		//���27
modbus_registers[53]=(char)(load_I%256);
modbus_registers[54]=(char)(load_I/256);		//���28
modbus_registers[55]=(char)(load_I%256);
modbus_registers[56]=(char)(load_I/256);		//���29
modbus_registers[57]=(char)(load_I%256);
modbus_registers[58]=(char)(TBAT/256);			//���30
modbus_registers[59]=(char)(TBAT%256);
modbus_registers[60]=(char)(UMAX/256);			//���31
modbus_registers[61]=(char)(UMAX%256);
modbus_registers[62]=(char)((UB20-DU)/256);		//���32
modbus_registers[63]=(char)((UB20-DU)%256);
modbus_registers[64]=(char)(UB0/256);			//���33
modbus_registers[65]=(char)(UB0%256);
modbus_registers[66]=(char)(UB20/256);			//���34
modbus_registers[67]=(char)(UB20%256);
modbus_registers[68]=(char)(USIGN/256);		//���35
modbus_registers[69]=(char)(USIGN%256);
modbus_registers[70]=(char)(UMN/256);		//���36
modbus_registers[71]=(char)(UMN%256);
modbus_registers[72]=(char)(U0B/256);		//���37
modbus_registers[73]=(char)(U0B%256);
modbus_registers[74]=(char)(IKB/256);		//���38
modbus_registers[75]=(char)(IKB%256);
modbus_registers[76]=(char)(IZMAX/256);		//���39
modbus_registers[77]=(char)(IZMAX%256);
modbus_registers[78]=(char)(IMAX/256);		//���40
modbus_registers[79]=(char)(IMAX%256);
modbus_registers[80]=(char)(IMIN/256);		//���41
modbus_registers[81]=(char)(IMIN%256);
modbus_registers[82]=(char)(UVZ/256);		//���42
modbus_registers[83]=(char)(UVZ%256);
modbus_registers[84]=(char)(TZAS/256);		//���43
modbus_registers[85]=(char)(TZAS%256);
modbus_registers[86]=(char)(TMAX/256);		//���44
modbus_registers[87]=(char)(TMAX%256);
modbus_registers[88]=(char)(TSIGN/256);		//���45
modbus_registers[89]=(char)(TSIGN%256);
modbus_registers[90]=(char)(TBATMAX/256);		//���46
modbus_registers[91]=(char)(TBATMAX%256);
modbus_registers[92]=(char)(TBATSIGN/256);		//���47
modbus_registers[93]=(char)(TBATSIGN%256);
modbus_registers[94]=(char)(load_I/256);		//���48
modbus_registers[95]=(char)(load_I%256);
modbus_registers[96]=(char)(load_I/256);		//���49
modbus_registers[97]=(char)(load_I%256);
modbus_registers[98]=(char)(load_I/256);		//���50
modbus_registers[99]=(char)(load_I%256);

modbus_tx_buff[0]=adr;
modbus_tx_buff[1]=func;
modbus_tx_buff[2]=(char)(reg_adr/256);
modbus_tx_buff[3]=(char)(reg_adr%256);
//modbus_tx_buff[4]=(char)(reg_quantity/256);
//modbus_tx_buff[5]=(char)(reg_quantity%256);


mem_copy((char*)&modbus_tx_buff[4],(char*)&modbus_registers[(reg_adr-1)*2],2);

crc_temp=CRC16_2(modbus_tx_buff,6);

modbus_tx_buff[6]=crc_temp%256;
modbus_tx_buff[7]=crc_temp/256;

for (i=0;i<8;i++)
	{
	putchar0(modbus_tx_buff[i]);
	}
for (i=0;i<8;i++)
	{
	putchar_sc16is700(modbus_tx_buff[i]);
	}
}*/
/*//-----------------------------------------------
void modbus_registers_transmit(unsigned char adr,unsigned char func,unsigned short reg_adr,unsigned short reg_quantity)
{
char modbus_registers[100];
//char modbus_tx_buff[100];
unsigned short crc_temp;
char i;


modbus_registers[0]=(char)(load_U/256);					//���1
modbus_registers[1]=(char)(load_U%256);
modbus_registers[2]=(char)(load_I/256);					//���2
modbus_registers[3]=(char)(load_I%256);
modbus_registers[4]=(char)(Ib_ips_termokompensat/256);		//���3
modbus_registers[5]=(char)(Ib_ips_termokompensat%256);
modbus_registers[6]=(char)(t_ext[0]/256);				//���4
modbus_registers[7]=(char)(t_ext[0]%256);
modbus_registers[8]=(char)(net_Ua/256);					//���5
modbus_registers[9]=(char)(net_Ua%256);		 	
modbus_registers[10]=(char)(net_Ub/256);				//���6
modbus_registers[11]=(char)(net_Ub%256);
modbus_registers[12]=(char)(net_Uc/256);				//���7
modbus_registers[13]=(char)(net_Uc%256);
modbus_registers[14]=(char)(net_F3/256);				//���8
modbus_registers[15]=(char)(net_F3%256);
modbus_registers[16]=(char)(load_I/256);				//���9
modbus_registers[17]=(char)(load_I%256);
modbus_registers[18]=(char)(load_I/256);				//���10
modbus_registers[19]=(char)(load_I%256);
modbus_registers[20]=(char)((LPC_RTC->YEAR)/256);			//���11
modbus_registers[21]=(char)((LPC_RTC->YEAR)%256);
modbus_registers[22]=(char)((LPC_RTC->MONTH)/256);		//���12
modbus_registers[23]=(char)((LPC_RTC->MONTH)%256);
modbus_registers[24]=(char)((LPC_RTC->DOM)/256);			//���13
modbus_registers[25]=(char)((LPC_RTC->DOM)%256);
modbus_registers[26]=(char)((LPC_RTC->HOUR)/256);			//���14
modbus_registers[27]=(char)((LPC_RTC->HOUR)%256);
modbus_registers[28]=(char)((LPC_RTC->MIN)/256);			//���15
modbus_registers[29]=(char)((LPC_RTC->MIN)%256);
modbus_registers[30]=(char)((LPC_RTC->SEC)/256);			//���16
modbus_registers[31]=(char)((LPC_RTC->SEC)%256);
modbus_registers[32]=(char)(load_I/256);				//���17
modbus_registers[33]=(char)(load_I%256);
modbus_registers[34]=(char)(load_I/256);				//���18
modbus_registers[35]=(char)(load_I%256);
modbus_registers[36]=(char)(load_I/256);		//���19
modbus_registers[37]=(char)(load_I%256);
modbus_registers[38]=(char)(NUMIST/256);		//���20
modbus_registers[39]=(char)(NUMIST%256);
modbus_registers[40]=(char)(PAR/256);		//���21
modbus_registers[41]=(char)(PAR%256);
modbus_registers[42]=(char)(ZV_ON/256);		//���22
modbus_registers[43]=(char)(ZV_ON%256);
modbus_registers[44]=(char)(TERMOKOMPENS/256);		//���23
modbus_registers[45]=(char)(TERMOKOMPENS%256);
modbus_registers[46]=(char)(UBM_AV/256);		//���24
modbus_registers[47]=(char)(UBM_AV%256);
modbus_registers[48]=(char)(load_I/256);		//���25
modbus_registers[49]=(char)(load_I%256);
modbus_registers[50]=(char)(load_I/256);		//���26
modbus_registers[51]=(char)(load_I%256);
modbus_registers[52]=(char)(load_I/256);		//���27
modbus_registers[53]=(char)(load_I%256);
modbus_registers[54]=(char)(load_I/256);		//���28
modbus_registers[55]=(char)(load_I%256);
modbus_registers[56]=(char)(load_I/256);		//���29
modbus_registers[57]=(char)(load_I%256);
modbus_registers[58]=(char)(TBAT/256);			//���30
modbus_registers[59]=(char)(TBAT%256);
modbus_registers[60]=(char)(UMAX/256);			//���31
modbus_registers[61]=(char)(UMAX%256);
modbus_registers[62]=(char)((UB20-DU)/256);		//���32
modbus_registers[63]=(char)((UB20-DU)%256);
modbus_registers[64]=(char)(UB0/256);			//���33
modbus_registers[65]=(char)(UB0%256);
modbus_registers[66]=(char)(UB20/256);			//���34
modbus_registers[67]=(char)(UB20%256);
modbus_registers[68]=(char)(USIGN/256);		//���35
modbus_registers[69]=(char)(USIGN%256);
modbus_registers[70]=(char)(UMN/256);		//���36
modbus_registers[71]=(char)(UMN%256);
modbus_registers[72]=(char)(U0B/256);		//���37
modbus_registers[73]=(char)(U0B%256);
modbus_registers[74]=(char)(IKB/256);		//���38
modbus_registers[75]=(char)(IKB%256);
modbus_registers[76]=(char)(IZMAX/256);		//���39
modbus_registers[77]=(char)(IZMAX%256);
modbus_registers[78]=(char)(IMAX/256);		//���40
modbus_registers[79]=(char)(IMAX%256);
modbus_registers[80]=(char)(IMIN/256);		//���41
modbus_registers[81]=(char)(IMIN%256);
modbus_registers[82]=(char)(UVZ/256);		//���42
modbus_registers[83]=(char)(UVZ%256);
modbus_registers[84]=(char)(TZAS/256);		//���43
modbus_registers[85]=(char)(TZAS%256);
modbus_registers[86]=(char)(TMAX/256);		//���44
modbus_registers[87]=(char)(TMAX%256);
modbus_registers[88]=(char)(TSIGN/256);		//���45
modbus_registers[89]=(char)(TSIGN%256);
modbus_registers[90]=(char)(TBATMAX/256);		//���46
modbus_registers[91]=(char)(TBATMAX%256);
modbus_registers[92]=(char)(TBATSIGN/256);		//���47
modbus_registers[93]=(char)(TBATSIGN%256);
modbus_registers[94]=(char)(load_I/256);		//���48
modbus_registers[95]=(char)(load_I%256);
modbus_registers[96]=(char)(load_I/256);		//���49
modbus_registers[97]=(char)(load_I%256);
modbus_registers[98]=(char)(load_I/256);		//���50
modbus_registers[99]=(char)(load_I%256);




modbus_tx_buff[0]=adr;
modbus_tx_buff[1]=func;
//modbus_tx_buff[2]=(char)(reg_adr/256);
//modbus_tx_buff[3]=(char)(reg_adr%256);
modbus_tx_buff[2]=(char)(reg_quantity*2);
//modbus_tx_buff[5]=(char)(reg_quantity%256);


mem_copy((char*)&modbus_tx_buff[3],(char*)&modbus_registers[(reg_adr-1)*2],reg_quantity*2);

crc_temp=CRC16_2(modbus_tx_buff,(reg_quantity*2)+3);

modbus_tx_buff[3+(reg_quantity*2)]=crc_temp%256;
modbus_tx_buff[4+(reg_quantity*2)]=crc_temp/256;

//int2lcdyx(reg_quantity,0,10,0);

for (i=0;i<15;i++)
	{
	putchar0(modbus_tx_buff[i]);
	} 
for (i=0;i<15;i++)
	{
	putchar_sc16is700(modbus_tx_buff[i]);
	}
}*/
/*
//-----------------------------------------------
void modbus_hold_register_transmit(unsigned char adr,unsigned char func,unsigned short reg_adr)
{
char modbus_registers[150];
//char modbus_tx_buff[150];
unsigned short crc_temp;
char i;

modbus_registers[20]=(char)((LPC_RTC->YEAR)/256);			//���11  �����, ���
modbus_registers[21]=(char)((LPC_RTC->YEAR)%256);
modbus_registers[22]=(char)((LPC_RTC->MONTH)/256);		//���12  �����, �����
modbus_registers[23]=(char)((LPC_RTC->MONTH)%256);
modbus_registers[24]=(char)((LPC_RTC->DOM)/256);			//���13  �����, ���� ������
modbus_registers[25]=(char)((LPC_RTC->DOM)%256);
modbus_registers[26]=(char)((LPC_RTC->HOUR)/256);			//���14  �����, ���
modbus_registers[27]=(char)((LPC_RTC->HOUR)%256);
modbus_registers[28]=(char)((LPC_RTC->MIN)/256);			//���15  �����, ������
modbus_registers[29]=(char)((LPC_RTC->MIN)%256);
modbus_registers[30]=(char)((LPC_RTC->SEC)/256);			//���16  �����, �������
modbus_registers[31]=(char)((LPC_RTC->SEC)%256);
modbus_registers[38]=(char)(NUMIST/256);				//���20  ���������� ������������ � ���������
modbus_registers[39]=(char)(NUMIST%256);
modbus_registers[40]=(char)(PAR/256);					//���21  ������������ ������ ������������ ���./����.
modbus_registers[41]=(char)(PAR%256);
modbus_registers[42]=(char)(ZV_ON/256);					//���22  �������� ��������� ������������ ���./����.
modbus_registers[43]=(char)(ZV_ON%256);
modbus_registers[46]=(char)(UBM_AV/256);				//���24  ��������� ������� ���������� ���������� ������� ����� �������, %
modbus_registers[47]=(char)(UBM_AV%256);
modbus_registers[58]=(char)(TBAT/256);					//���30  ������ �������� ���� �������, �����.
modbus_registers[59]=(char)(TBAT%256);
modbus_registers[60]=(char)(UMAX/256);					//���31  ������������ (���������) ���������� ������������, 0.1�
modbus_registers[61]=(char)(UMAX%256);
modbus_registers[62]=(char)((UB20-DU)/256);				//���32  ����������� (���������) ���������� ������������, 0.1�
modbus_registers[63]=(char)((UB20-DU)%256);
modbus_registers[64]=(char)(UB0/256);					//���33  ���������� ���������� ������� ��� 0��, 0.1�
modbus_registers[65]=(char)(UB0%256);
modbus_registers[66]=(char)(UB20/256);					//���34  ���������� ���������� ������� ��� 20��, 0.1�
modbus_registers[67]=(char)(UB20%256);
modbus_registers[68]=(char)(USIGN/256);					//���35  ����������� (����������) ���������� �������, 1�
modbus_registers[69]=(char)(USIGN%256);
modbus_registers[70]=(char)(UMN/256);					//���36  ����������� (���������) ���������� �������� ����, 1�
modbus_registers[71]=(char)(UMN%256);
modbus_registers[72]=(char)(U0B/256);					//���37  ������� ���������� ��� ����������� ��������, 0.1�
modbus_registers[73]=(char)(U0B%256);
modbus_registers[74]=(char)(IKB/256);					//���38  ��� �������� ������� �������, 0.1�
modbus_registers[75]=(char)(IKB%256);
modbus_registers[76]=(char)(IZMAX/256);					//���39  ��� ������ ������� ������������, 0.1�
modbus_registers[77]=(char)(IZMAX%256);
modbus_registers[78]=(char)(IMAX/256);					//���40  ��� ������������ �� ������� ���-�� ������������, 0.1�
modbus_registers[79]=(char)(IMAX%256);
modbus_registers[80]=(char)(IMIN/256);					//���41  ��� ������������ �� ������� ���-�� ������������, 0.1�
modbus_registers[81]=(char)(IMIN%256);
modbus_registers[82]=(char)(UVZ/256);					//���42  ���������� �������������� ������, 0.1�
modbus_registers[83]=(char)(UVZ%256);
modbus_registers[84]=(char)(TZAS/256);					//���43  ����� �������� ��������� ������������, ���
modbus_registers[85]=(char)(TZAS%256);
modbus_registers[86]=(char)(TMAX/256);					//���44  ����������� ������������ ���������, 1��
modbus_registers[87]=(char)(TMAX%256);
modbus_registers[88]=(char)(TSIGN/256);					//���45  ����������� ������������ ����������, 1��
modbus_registers[89]=(char)(TSIGN%256);
modbus_registers[90]=(char)(TBATMAX/256);				//���46  ����������� ������� ���������, 1��
modbus_registers[91]=(char)(TBATMAX%256);
modbus_registers[92]=(char)(TBATSIGN/256);				//���47  ����������� ������� ����������, 1��
modbus_registers[93]=(char)(TBATSIGN%256);
modbus_registers[94]=(char)(speedChrgCurr/256);					//���48  ��� ����������� ������, 0.1�
modbus_registers[95]=(char)(speedChrgCurr%256);
modbus_registers[96]=(char)(speedChrgVolt/256);				//���49	 ���������� ����������� ������, 0.1� 
modbus_registers[97]=(char)(speedChrgVolt%256);
modbus_registers[98]=(char)(speedChrgTimeInHour/256);				//���50	 ����� ����������� ������, 1�
modbus_registers[99]=(char)(speedChrgTimeInHour%256);


modbus_tx_buff[0]=adr;
modbus_tx_buff[1]=func;
modbus_tx_buff[2]=(char)(reg_adr/256);
modbus_tx_buff[3]=(char)(reg_adr%256);

mem_copy((char*)&modbus_tx_buff[4],(char*)&modbus_registers[(reg_adr-1)*2],2);

crc_temp=CRC16_2(modbus_tx_buff,6);

modbus_tx_buff[6]=crc_temp%256;
modbus_tx_buff[7]=crc_temp/256;

for (i=0;i<8;i++)
	{
	putchar0(modbus_tx_buff[i]);
	}
for (i=0;i<8;i++)
	{
	putchar_sc16is700(modbus_tx_buff[i]);
	}
}
*/

//-----------------------------------------------
void modbus_hold_registers_transmit(unsigned char adr,unsigned char func,unsigned short reg_adr,unsigned short reg_quantity, char prot)
{
signed char modbus_registers[150];
//char modbus_tx_buff[150];
unsigned short crc_temp;
char i;

modbus_registers[20]=(char)((LPC_RTC->YEAR)>>8);			//���11  �����, ���
modbus_registers[21]=(char)((LPC_RTC->YEAR));
modbus_registers[22]=(char)((LPC_RTC->MONTH)>>8);		//���12  �����, �����
modbus_registers[23]=(char)((LPC_RTC->MONTH));
modbus_registers[24]=(char)((LPC_RTC->DOM)>>8);			//���13  �����, ���� ������
modbus_registers[25]=(char)((LPC_RTC->DOM));
modbus_registers[26]=(char)((LPC_RTC->HOUR)>>8);			//���14  �����, ���
modbus_registers[27]=(char)((LPC_RTC->HOUR));
modbus_registers[28]=(char)((LPC_RTC->MIN)>>8);			//���15  �����, ������
modbus_registers[29]=(char)((LPC_RTC->MIN));
modbus_registers[30]=(char)((LPC_RTC->SEC)>>8);			//���16  �����, �������
modbus_registers[31]=(char)((LPC_RTC->SEC));
modbus_registers[38]=(char)(NUMIST>>8);				//���20  ���������� ������������ � ���������
modbus_registers[39]=(char)(NUMIST);
//modbus_registers[40]=(char)(PAR>>8);					//���21  ������������ ������ ������������ ���./����.
//modbus_registers[41]=(char)(PAR);
modbus_registers[42]=(char)(ZV_ON>>8);					//���22  �������� ��������� ������������ ���./����.
modbus_registers[43]=(char)(ZV_ON);
modbus_registers[46]=(char)(UBM_AV>>8);				//���24  ��������� ������� ���������� ���������� ������� ����� �������, %
modbus_registers[47]=(char)(UBM_AV);
modbus_registers[58]=(char)(TBAT>>8);					//���30  ������ �������� ���� �������, �����.
modbus_registers[59]=(char)(TBAT);
modbus_registers[60]=(char)(UMAX>>8);					//���31  ������������ (���������) ���������� ������������, 0.1�
modbus_registers[61]=(char)(UMAX);
//modbus_registers[62]=(char)((UB20-DU)>>8);				//���32  ����������� (���������) ���������� ������������, 0.1�
//modbus_registers[63]=(char)((UB20-DU));
//modbus_registers[64]=(char)(UB0>>8);					//���33  ���������� ���������� ������� ��� 0��, 0.1�
//modbus_registers[65]=(char)(UB0);
//modbus_registers[66]=(char)(UB20>>8);					//���34  ���������� ���������� ������� ��� 20��, 0.1�
//modbus_registers[67]=(char)(UB20);
modbus_registers[68]=(char)(USIGN>>8);					//���35  ����������� (����������) ���������� �������, 1�
modbus_registers[69]=(char)(USIGN);
modbus_registers[70]=(char)(UMN>>8);					//���36  ����������� (���������) ���������� �������� ����, 1�
modbus_registers[71]=(char)(UMN);
modbus_registers[72]=(char)(U0B>>8);					//���37  ������� ���������� ��� ����������� ��������, 0.1�
modbus_registers[73]=(char)(U0B);
modbus_registers[74]=(char)(IKB>>8);					//���38  ��� �������� ������� �������, 0.1�
modbus_registers[75]=(char)(IKB);
modbus_registers[76]=(char)(IZMAX>>8);					//���39  ��� ������ ������� ������������, 0.1�
modbus_registers[77]=(char)(IZMAX);
modbus_registers[78]=(char)(IMAX>>8);					//���40  ��� ������������ �� ������� ���-�� ������������, 0.1�
modbus_registers[79]=(char)(IMAX);
modbus_registers[80]=(char)(IMIN>>8);					//���41  ��� ������������ �� ������� ���-�� ������������, 0.1�
modbus_registers[81]=(char)(IMIN);
//modbus_registers[82]=(char)(UVZ>>8);					//���42  ���������� �������������� ������, 0.1�
//modbus_registers[83]=(char)(UVZ);
modbus_registers[84]=(char)(TZAS>>8);					//���43  ����� �������� ��������� ������������, ���
modbus_registers[85]=(char)(TZAS);
modbus_registers[86]=(char)(TMAX>>8);					//���44  ����������� ������������ ���������, 1��
modbus_registers[87]=(char)(TMAX);
modbus_registers[88]=(char)(TSIGN>>8);					//���45  ����������� ������������ ����������, 1��
modbus_registers[89]=(char)(TSIGN);
modbus_registers[90]=(char)(TBATMAX>>8);				//���46  ����������� ������� ���������, 1��
modbus_registers[91]=(char)(TBATMAX);
modbus_registers[92]=(char)(TBATSIGN>>8);				//���47  ����������� ������� ����������, 1��
modbus_registers[93]=(char)(TBATSIGN);
modbus_registers[94]=(char)(speedChrgCurr>>8);					//���48  ��� ����������� ������, 0.1�
modbus_registers[95]=(char)(speedChrgCurr);
modbus_registers[96]=(char)(speedChrgVolt>>8);				//���49	 ���������� ����������� ������, 0.1� 
modbus_registers[97]=(char)(speedChrgVolt);
modbus_registers[98]=(char)(speedChrgTimeInHour>>8);				//���50	 ����� ����������� ������, 1�
modbus_registers[99]=(char)(speedChrgTimeInHour);
modbus_registers[100]=(char)(U_OUT_KONTR_MAX>>8);					//���51	 �������� ��������� ����������, Umax, 0.1�
modbus_registers[101]=(char)(U_OUT_KONTR_MAX);
modbus_registers[102]=(char)(U_OUT_KONTR_MIN>>8);					//���52	 �������� ��������� ����������, Umin, 0.1�
modbus_registers[103]=(char)(U_OUT_KONTR_MIN);
modbus_registers[104]=(char)(U_OUT_KONTR_DELAY>>8);				//���53	 �������� ��������� ����������, T��������, 1���.
modbus_registers[105]=(char)(U_OUT_KONTR_DELAY);




if(prot==MODBUS_RTU_PROT)
	{
	modbus_tx_buff[0]=adr;
	modbus_tx_buff[1]=func;
	modbus_tx_buff[2]=(char)(reg_quantity*2);
	mem_copy((char*)&modbus_tx_buff[3],(char*)&modbus_registers[(reg_adr-1)*2],reg_quantity*2);
	
	crc_temp=CRC16_2(modbus_tx_buff,(reg_quantity*2)+3);

	modbus_tx_buff[3+(reg_quantity*2)]=(char)crc_temp;
	modbus_tx_buff[4+(reg_quantity*2)]=crc_temp>>8;

	for (i=0;i<(5+(reg_quantity*2));i++)
		{
		putchar0(modbus_tx_buff[i]);
		}

	for (i=0;i<(5+(reg_quantity*2));i++)
		{
		putchar_sc16is700(modbus_tx_buff[i]);
		}
	}
else if(prot==MODBUS_TCP_PROT)
	{
	modbus_tcp_out_ptr=(char*)&modbus_registers[(reg_adr-1)*2];
	}
}

//-----------------------------------------------
void modbus_input_registers_transmit(unsigned char adr,unsigned char func,unsigned short reg_adr,unsigned short reg_quantity, char prot)
{
signed char modbus_registers[500];
//char modbus_tx_buff[200];
unsigned short crc_temp;
char i;
short tempS;

//tempS=(MODBUS_INPUT_REGS[0]);
modbus_registers[0]=(signed char)(bps[0]._Uii>>8);			//���1	�������� ���������� ����������� �1, 0.1�
modbus_registers[1]=(signed char)(bps[0]._Uii);
modbus_registers[2]=(signed char)(bps[0]._Ii>>8);			//���2	�������� ��� ����������� �1, 0.1�
modbus_registers[3]=(signed char)(bps[0]._Ii);
modbus_registers[4]=(signed char)(bps[0]._Ti>>8);			//���3	����������� ��������� ����������� �1, 1��
modbus_registers[5]=(signed char)(bps[0]._Ti);
modbus_registers[6]=(signed char)(bps[0]._av>>8);			//���4	���� ������ ����������� �1, 0x01 - ��������, 0x02 �������� U���, 0x04 �������� U���, 0x08 - ����������� ����� � ������������
modbus_registers[7]=(signed char)(bps[0]._av);
modbus_registers[8]=(signed char)(bps[1]._Uii>>8);			//���5	�������� ���������� ����������� �2, 0.1�
modbus_registers[9]=(signed char)(bps[1]._Uii);
modbus_registers[10]=(signed char)(bps[1]._Ii>>8);			//���6	�������� ��� ����������� �2, 0.1�
modbus_registers[11]=(signed char)(bps[1]._Ii);
modbus_registers[12]=(signed char)(bps[1]._Ti>>8);			//���7	����������� ��������� ����������� �2, 1��
modbus_registers[13]=(signed char)(bps[1]._Ti);
modbus_registers[14]=(signed char)(bps[1]._av>>8);			//���8	���� ������ ����������� �2, 0x01 - ��������, 0x02 �������� U���, 0x04 �������� U���, 0x08 - ����������� ����� � ������������
modbus_registers[15]=(signed char)(bps[1]._av);
modbus_registers[16]=(signed char)(bps[2]._Uii>>8);			//���9	�������� ���������� ����������� �3, 0.1�
modbus_registers[17]=(signed char)(bps[2]._Uii);
modbus_registers[18]=(signed char)(bps[2]._Ii>>8);			//���10	�������� ��� ����������� �3, 0.1�
modbus_registers[19]=(signed char)(bps[2]._Ii);
modbus_registers[20]=(signed char)(bps[2]._Ti>>8);			//���11	����������� ��������� ����������� �3, 1��
modbus_registers[21]=(signed char)(bps[2]._Ti);
modbus_registers[22]=(signed char)(bps[2]._av>>8);			//���12	���� ������ ����������� �3, 0x01 - ��������, 0x02 �������� U���, 0x04 �������� U���, 0x08 - ����������� ����� � ������������
modbus_registers[23]=(signed char)(bps[2]._av);
modbus_registers[24]=(signed char)(bps[3]._Uii>>8);			//���13	�������� ���������� ����������� �4, 0.1�
modbus_registers[25]=(signed char)(bps[3]._Uii);
modbus_registers[26]=(signed char)(bps[3]._Ii>>8);			//���14	�������� ��� ����������� �4, 0.1�
modbus_registers[27]=(signed char)(bps[3]._Ii);
modbus_registers[28]=(signed char)(bps[3]._Ti>>8);			//���15	����������� ��������� ����������� �4, 1��
modbus_registers[29]=(signed char)(bps[3]._Ti);
modbus_registers[30]=(signed char)(bps[3]._av>>8);			//���16	���� ������ ����������� �4, 0x01 - ��������, 0x02 �������� U���, 0x04 �������� U���, 0x08 - ����������� ����� � ������������
modbus_registers[31]=(signed char)(bps[3]._av);
modbus_registers[32]=(signed char)(bps[4]._Uii>>8);			//���17	�������� ���������� ����������� �5, 0.1�
modbus_registers[33]=(signed char)(bps[4]._Uii);
modbus_registers[34]=(signed char)(bps[4]._Ii>>8);			//���18	�������� ��� ����������� �5, 0.1�
modbus_registers[35]=(signed char)(bps[4]._Ii);
modbus_registers[36]=(signed char)(bps[4]._Ti>>8);			//���19	����������� ��������� ����������� �5, 1��
modbus_registers[37]=(signed char)(bps[4]._Ti);
modbus_registers[38]=(signed char)(bps[4]._av>>8);			//���20	���� ������ ����������� �5, 0x01 - ��������, 0x02 �������� U���, 0x04 �������� U���, 0x08 - ����������� ����� � ������������
modbus_registers[39]=(signed char)(bps[4]._av);
modbus_registers[40]=(signed char)(bps[5]._Uii>>8);			//���21	�������� ���������� ����������� �6, 0.1�
modbus_registers[41]=(signed char)(bps[5]._Uii);
modbus_registers[42]=(signed char)(bps[5]._Ii>>8);			//���22	�������� ��� ����������� �6, 0.1�
modbus_registers[43]=(signed char)(bps[5]._Ii);
modbus_registers[44]=(signed char)(bps[5]._Ti>>8);			//���23	����������� ��������� ����������� �6, 1��
modbus_registers[45]=(signed char)(bps[5]._Ti);
modbus_registers[46]=(signed char)(bps[5]._av>>8);			//���24	���� ������ ����������� �6, 0x01 - ��������, 0x02 �������� U���, 0x04 �������� U���, 0x08 - ����������� ����� � ������������
modbus_registers[47]=(signed char)(bps[5]._av);
modbus_registers[48]=(signed char)(bps[6]._Uii>>8);			//���25	�������� ���������� ����������� �7, 0.1�
modbus_registers[49]=(signed char)(bps[6]._Uii);
modbus_registers[50]=(signed char)(bps[6]._Ii>>8);			//���26	�������� ��� ����������� �7, 0.1�
modbus_registers[51]=(signed char)(bps[6]._Ii);
modbus_registers[52]=(signed char)(bps[6]._Ti>>8);			//���27	����������� ��������� ����������� �7, 1��
modbus_registers[53]=(signed char)(bps[6]._Ti);
modbus_registers[54]=(signed char)(bps[6]._av>>8);			//���28	���� ������ ����������� �7, 0x01 - ��������, 0x02 �������� U���, 0x04 �������� U���, 0x08 - ����������� ����� � ������������
modbus_registers[55]=(signed char)(bps[6]._av);
modbus_registers[56]=(signed char)(bps[7]._Uii>>8);			//���29	�������� ���������� ����������� �8, 0.1�
modbus_registers[57]=(signed char)(bps[7]._Uii);
modbus_registers[58]=(signed char)(bps[7]._Ii>>8);			//���30	�������� ��� ����������� �8, 0.1�
modbus_registers[59]=(signed char)(bps[7]._Ii);
modbus_registers[60]=(signed char)(bps[7]._Ti>>8);			//���31	����������� ��������� ����������� �8, 1��
modbus_registers[61]=(signed char)(bps[7]._Ti);
modbus_registers[62]=(signed char)(bps[7]._av>>8);			//���32	���� ������ ����������� �8, 0x01 - ��������, 0x02 �������� U���, 0x04 �������� U���, 0x08 - ����������� ����� � ������������
modbus_registers[63]=(signed char)(bps[7]._av);
modbus_registers[64]=(signed char)(bps[8]._Uii>>8);			//���33	�������� ���������� ����������� �9, 0.1�
modbus_registers[65]=(signed char)(bps[8]._Uii);
modbus_registers[66]=(signed char)(bps[8]._Ii>>8);			//���34	�������� ��� ����������� �9, 0.1�
modbus_registers[67]=(signed char)(bps[8]._Ii);
modbus_registers[68]=(signed char)(bps[8]._Ti>>8);			//���35	����������� ��������� ����������� �9, 1��
modbus_registers[69]=(signed char)(bps[8]._Ti);
modbus_registers[70]=(signed char)(bps[8]._av>>8);			//���36	���� ������ ����������� �9, 0x01 - ��������, 0x02 �������� U���, 0x04 �������� U���, 0x08 - ����������� ����� � ������������
modbus_registers[71]=(signed char)(bps[8]._av);
modbus_registers[72]=(signed char)(bps[9]._Uii>>8);			//���37	�������� ���������� ����������� �10, 0.1�
modbus_registers[73]=(signed char)(bps[9]._Uii);
modbus_registers[74]=(signed char)(bps[9]._Ii>>8);			//���38	�������� ��� ����������� �10, 0.1�
modbus_registers[75]=(signed char)(bps[9]._Ii);
modbus_registers[76]=(signed char)(bps[9]._Ti>>8);			//���39	����������� ��������� ����������� �10, 1��
modbus_registers[77]=(signed char)(bps[9]._Ti);
modbus_registers[78]=(signed char)(bps[9]._av>>8);			//���40	���� ������ ����������� �10, 0x01 - ��������, 0x02 �������� U���, 0x04 �������� U���, 0x08 - ����������� ����� � ������������
modbus_registers[79]=(signed char)(bps[9]._av);
modbus_registers[80]=(signed char)(bps[10]._Uii>>8);		//���41	�������� ���������� ����������� �11, 0.1�
modbus_registers[81]=(signed char)(bps[10]._Uii);
modbus_registers[82]=(signed char)(bps[10]._Ii>>8);			//���42	�������� ��� ����������� �11, 0.1�
modbus_registers[83]=(signed char)(bps[10]._Ii);
modbus_registers[84]=(signed char)(bps[10]._Ti>>8);			//���43	����������� ��������� ����������� �11, 1��
modbus_registers[85]=(signed char)(bps[10]._Ti);
modbus_registers[86]=(signed char)(bps[10]._av>>8);			//���44	���� ������ ����������� �11, 0x01 - ��������, 0x02 �������� U���, 0x04 �������� U���, 0x08 - ����������� ����� � ������������
modbus_registers[87]=(signed char)(bps[10]._av);
modbus_registers[88]=(signed char)(bps[11]._Uii>>8);		//���45	�������� ���������� ����������� �12, 0.1�
modbus_registers[89]=(signed char)(bps[11]._Uii);
modbus_registers[90]=(signed char)(bps[11]._Ii>>8);			//���46	�������� ��� ����������� �12, 0.1�
modbus_registers[91]=(signed char)(bps[11]._Ii);
modbus_registers[92]=(signed char)(bps[11]._Ti>>8);			//���47	����������� ��������� ����������� �12, 1��
modbus_registers[93]=(signed char)(bps[11]._Ti);
modbus_registers[94]=(signed char)(bps[11]._av>>8);			//���48	���� ������ ����������� �12, 0x01 - ��������, 0x02 �������� U���, 0x04 �������� U���, 0x08 - ����������� ����� � ������������
modbus_registers[95]=(signed char)(bps[11]._av);

															
modbus_registers[98]=(signed char)	(out_U>>8);				//���50 �������� ���������� �������
modbus_registers[99]=(signed char)	(out_U);
modbus_registers[100]=(signed char)	(in_U>>8);				//���51	������� ���������� �������
modbus_registers[101]=(signed char)	(in_U);
modbus_registers[102]=(signed char)	(vd_U>>8);				//���52	���������� ������������
modbus_registers[103]=(signed char)	(vd_U);
modbus_registers[104]=(signed char)	(Ib_ips_termokompensat>>8);		//���53	�������� ���
modbus_registers[105]=(signed char)	(Ib_ips_termokompensat);

modbus_registers[106]=(signed char)	(t_ext[0]>>8);				//���54	 ����������� �������
modbus_registers[107]=(signed char)	(t_ext[0]);
modbus_registers[108]=(signed char)	(avar_vd_stat>>8);		//���55	 ���� ������ �������
modbus_registers[109]=(signed char)	(avar_vd_stat);			// ��� 0 - ������ ������ �� ���
															// ��� 1 - ���������� ������� �� ����
															// ��� 2 - �������� �������
															// ��� 3 - �������� ���������� ��������
															// ��� 4 - �������� ���������� ��������
															// ��� 5 - ������� ���������� ��������
															// ��� 6 - ������� ���������� ��������
															// ��� 7 - ������ ����� �� ���� KAN
															// ��� 8 - ������������ � ������


modbus_registers[118]=(signed char)(bps[12]._Uii>>8);			//���60	�������� ���������� ����������� �13, 0.1�
modbus_registers[119]=(signed char)(bps[12]._Uii);
modbus_registers[120]=(signed char)(bps[12]._Ii>>8);			//���61	�������� ��� ����������� �13, 0.1�
modbus_registers[121]=(signed char)(bps[12]._Ii);
modbus_registers[122]=(signed char)(bps[12]._Ti>>8);			//���62	����������� ��������� ����������� �13, 1��
modbus_registers[123]=(signed char)(bps[12]._Ti);
modbus_registers[124]=(signed char)(bps[12]._av>>8);			//���63	���� ������ ����������� �13, 0x01 - ��������, 0x02 �������� U���, 0x04 �������� U���, 0x08 - ����������� ����� � ������������
modbus_registers[125]=(signed char)(bps[12]._av);
modbus_registers[126]=(signed char)(bps[13]._Uii>>8);			//���64	�������� ���������� ����������� �14, 0.1�
modbus_registers[127]=(signed char)(bps[13]._Uii);
modbus_registers[128]=(signed char)(bps[13]._Ii>>8);			//���65	�������� ��� ����������� �14, 0.1�
modbus_registers[129]=(signed char)(bps[13]._Ii);
modbus_registers[130]=(signed char)(bps[13]._Ti>>8);			//���66	����������� ��������� ����������� �14, 1��
modbus_registers[131]=(signed char)(bps[13]._Ti);
modbus_registers[132]=(signed char)(bps[13]._av>>8);			//���67	���� ������ ����������� �14, 0x01 - ��������, 0x02 �������� U���, 0x04 �������� U���, 0x08 - ����������� ����� � ������������
modbus_registers[133]=(signed char)(bps[13]._av);
modbus_registers[134]=(signed char)(bps[14]._Uii>>8);			//���68	�������� ���������� ����������� �15, 0.1�
modbus_registers[135]=(signed char)(bps[14]._Uii);
modbus_registers[136]=(signed char)(bps[14]._Ii>>8);			//���69	�������� ��� ����������� �15, 0.1�
modbus_registers[137]=(signed char)(bps[14]._Ii);
modbus_registers[138]=(signed char)(bps[14]._Ti>>8);			//���70	����������� ��������� ����������� �15, 1��
modbus_registers[139]=(signed char)(bps[14]._Ti);
modbus_registers[140]=(signed char)(bps[14]._av>>8);			//���71	���� ������ ����������� �15, 0x01 - ��������, 0x02 �������� U���, 0x04 �������� U���, 0x08 - ����������� ����� � ������������
modbus_registers[141]=(signed char)(bps[14]._av);
modbus_registers[142]=(signed char)(bps[15]._Uii>>8);			//���72	�������� ���������� ����������� �16, 0.1�
modbus_registers[143]=(signed char)(bps[15]._Uii);
modbus_registers[144]=(signed char)(bps[15]._Ii>>8);			//���73	�������� ��� ����������� �16, 0.1�
modbus_registers[145]=(signed char)(bps[15]._Ii);
modbus_registers[146]=(signed char)(bps[15]._Ti>>8);			//���74	����������� ��������� ����������� �16, 1��
modbus_registers[147]=(signed char)(bps[15]._Ti);
modbus_registers[148]=(signed char)(bps[15]._av>>8);			//���75	���� ������ ����������� �16, 0x01 - ��������, 0x02 �������� U���, 0x04 �������� U���, 0x08 - ����������� ����� � ������������
modbus_registers[149]=(signed char)(bps[15]._av);
modbus_registers[150]=(signed char)(bps[16]._Uii>>8);			//���76	�������� ���������� ����������� �17, 0.1�
modbus_registers[151]=(signed char)(bps[16]._Uii);
modbus_registers[152]=(signed char)(bps[16]._Ii>>8);			//���77	�������� ��� ����������� �17, 0.1�
modbus_registers[153]=(signed char)(bps[16]._Ii);
modbus_registers[154]=(signed char)(bps[16]._Ti>>8);			//���78	����������� ��������� ����������� �17, 1��
modbus_registers[155]=(signed char)(bps[16]._Ti);
modbus_registers[156]=(signed char)(bps[16]._av>>8);			//���79	���� ������ ����������� �17, 0x01 - ��������, 0x02 �������� U���, 0x04 �������� U���, 0x08 - ����������� ����� � ������������
modbus_registers[157]=(signed char)(bps[16]._av);
modbus_registers[158]=(signed char)(bps[17]._Uii>>8);			//���80	�������� ���������� ����������� �18, 0.1�
modbus_registers[159]=(signed char)(bps[17]._Uii);
modbus_registers[160]=(signed char)(bps[17]._Ii>>8);			//���81	�������� ��� ����������� �18, 0.1�
modbus_registers[161]=(signed char)(bps[17]._Ii);
modbus_registers[162]=(signed char)(bps[17]._Ti>>8);			//���82	����������� ��������� ����������� �18, 1��
modbus_registers[163]=(signed char)(bps[17]._Ti);
modbus_registers[164]=(signed char)(bps[17]._av>>8);			//���83	���� ������ ����������� �18, 0x01 - ��������, 0x02 �������� U���, 0x04 �������� U���, 0x08 - ����������� ����� � ������������
modbus_registers[165]=(signed char)(bps[17]._av);
modbus_registers[166]=(signed char)(bps[18]._Uii>>8);			//���84	�������� ���������� ����������� �19, 0.1�
modbus_registers[167]=(signed char)(bps[18]._Uii);
modbus_registers[168]=(signed char)(bps[18]._Ii>>8);			//���85	�������� ��� ����������� �19, 0.1�
modbus_registers[169]=(signed char)(bps[18]._Ii);
modbus_registers[170]=(signed char)(bps[18]._Ti>>8);			//���86	����������� ��������� ����������� �19, 1��
modbus_registers[171]=(signed char)(bps[18]._Ti);
modbus_registers[172]=(signed char)(bps[18]._av>>8);			//���87	���� ������ ����������� �19, 0x01 - ��������, 0x02 �������� U���, 0x04 �������� U���, 0x08 - ����������� ����� � ������������
modbus_registers[173]=(signed char)(bps[18]._av);
modbus_registers[174]=(signed char)(bps[19]._Uii>>8);			//���88	�������� ���������� ����������� �20, 0.1�
modbus_registers[175]=(signed char)(bps[19]._Uii);
modbus_registers[176]=(signed char)(bps[19]._Ii>>8);			//���89	�������� ��� ����������� �20, 0.1�
modbus_registers[177]=(signed char)(bps[19]._Ii);
modbus_registers[178]=(signed char)(bps[19]._Ti>>8);			//���90	����������� ��������� ����������� �20, 1��
modbus_registers[179]=(signed char)(bps[19]._Ti);
modbus_registers[180]=(signed char)(bps[19]._av>>8);			//���91	���� ������ ����������� �20, 0x01 - ��������, 0x02 �������� U���, 0x04 �������� U���, 0x08 - ����������� ����� � ������������
modbus_registers[181]=(signed char)(bps[19]._av);
modbus_registers[182]=(signed char)(bps[20]._Uii>>8);			//���92	�������� ���������� ����������� �21, 0.1�
modbus_registers[183]=(signed char)(bps[20]._Uii);
modbus_registers[184]=(signed char)(bps[20]._Ii>>8);			//���93	�������� ��� ����������� �21, 0.1�
modbus_registers[185]=(signed char)(bps[20]._Ii);
modbus_registers[186]=(signed char)(bps[20]._Ti>>8);			//���94	����������� ��������� ����������� �21, 1��
modbus_registers[187]=(signed char)(bps[20]._Ti);
modbus_registers[188]=(signed char)(bps[20]._av>>8);			//���95	���� ������ ����������� �21, 0x01 - ��������, 0x02 �������� U���, 0x04 �������� U���, 0x08 - ����������� ����� � ������������
modbus_registers[189]=(signed char)(bps[20]._av);
modbus_registers[190]=(signed char)(bps[21]._Uii>>8);			//���96	�������� ���������� ����������� �22, 0.1�
modbus_registers[191]=(signed char)(bps[21]._Uii);
modbus_registers[192]=(signed char)(bps[21]._Ii>>8);			//���97	�������� ��� ����������� �22, 0.1�
modbus_registers[193]=(signed char)(bps[21]._Ii);
modbus_registers[194]=(signed char)(bps[21]._Ti>>8);			//���98	����������� ��������� ����������� �22, 1��
modbus_registers[195]=(signed char)(bps[21]._Ti);
modbus_registers[196]=(signed char)(bps[21]._av>>8);			//���99	���� ������ ����������� �22, 0x01 - ��������, 0x02 �������� U���, 0x04 �������� U���, 0x08 - ����������� ����� � ������������
modbus_registers[197]=(signed char)(bps[21]._av);
modbus_registers[198]=(signed char)(bps[22]._Uii>>8);			//���100	�������� ���������� ����������� �23, 0.1�
modbus_registers[199]=(signed char)(bps[22]._Uii);
modbus_registers[200]=(signed char)(bps[22]._Ii>>8);			//���101	�������� ��� ����������� �23, 0.1�
modbus_registers[201]=(signed char)(bps[22]._Ii);
modbus_registers[202]=(signed char)(bps[22]._Ti>>8);			//���102	����������� ��������� ����������� �23, 1��
modbus_registers[203]=(signed char)(bps[22]._Ti);
modbus_registers[204]=(signed char)(bps[22]._av>>8);			//���103	���� ������ ����������� �23, 0x01 - ��������, 0x02 �������� U���, 0x04 �������� U���, 0x08 - ����������� ����� � ������������
modbus_registers[205]=(signed char)(bps[22]._av);
modbus_registers[206]=(signed char)(bps[23]._Uii>>8);			//���104	�������� ���������� ����������� �24, 0.1�
modbus_registers[207]=(signed char)(bps[23]._Uii);
modbus_registers[208]=(signed char)(bps[23]._Ii>>8);			//���105	�������� ��� ����������� �24, 0.1�
modbus_registers[209]=(signed char)(bps[23]._Ii);
modbus_registers[210]=(signed char)(bps[23]._Ti>>8);			//���106	����������� ��������� ����������� �24, 1��
modbus_registers[211]=(signed char)(bps[23]._Ti);
modbus_registers[212]=(signed char)(bps[23]._av>>8);			//���107	���� ������ ����������� �24, 0x01 - ��������, 0x02 �������� U���, 0x04 �������� U���, 0x08 - ����������� ����� � ������������
modbus_registers[213]=(signed char)(bps[23]._av);
modbus_registers[214]=(signed char)(bps[24]._Uii>>8);			//���108	�������� ���������� ����������� �25, 0.1�
modbus_registers[215]=(signed char)(bps[24]._Uii);
modbus_registers[216]=(signed char)(bps[24]._Ii>>8);			//���109	�������� ��� ����������� �25, 0.1�
modbus_registers[217]=(signed char)(bps[24]._Ii);
modbus_registers[218]=(signed char)(bps[24]._Ti>>8);			//���110	����������� ��������� ����������� �25, 1��
modbus_registers[219]=(signed char)(bps[24]._Ti);
modbus_registers[220]=(signed char)(bps[24]._av>>8);			//���111	���� ������ ����������� �25, 0x01 - ��������, 0x02 �������� U���, 0x04 �������� U���, 0x08 - ����������� ����� � ������������
modbus_registers[221]=(signed char)(bps[24]._av);
modbus_registers[222]=(signed char)(bps[25]._Uii>>8);			//���112	�������� ���������� ����������� �26, 0.1�
modbus_registers[223]=(signed char)(bps[25]._Uii);
modbus_registers[224]=(signed char)(bps[25]._Ii>>8);			//���113	�������� ��� ����������� �26, 0.1�
modbus_registers[225]=(signed char)(bps[25]._Ii);
modbus_registers[226]=(signed char)(bps[25]._Ti>>8);			//���114	����������� ��������� ����������� �26, 1��
modbus_registers[227]=(signed char)(bps[25]._Ti);
modbus_registers[228]=(signed char)(bps[25]._av>>8);			//���115	���� ������ ����������� �26, 0x01 - ��������, 0x02 �������� U���, 0x04 �������� U���, 0x08 - ����������� ����� � ������������
modbus_registers[229]=(signed char)(bps[25]._av);
modbus_registers[230]=(signed char)(bps[26]._Uii>>8);			//���116	�������� ���������� ����������� �27, 0.1�
modbus_registers[231]=(signed char)(bps[26]._Uii);
modbus_registers[232]=(signed char)(bps[26]._Ii>>8);			//���117	�������� ��� ����������� �27, 0.1�
modbus_registers[233]=(signed char)(bps[26]._Ii);
modbus_registers[234]=(signed char)(bps[26]._Ti>>8);			//���118	����������� ��������� ����������� �27, 1��
modbus_registers[235]=(signed char)(bps[26]._Ti);
modbus_registers[236]=(signed char)(bps[26]._av>>8);			//���119	���� ������ ����������� �27, 0x01 - ��������, 0x02 �������� U���, 0x04 �������� U���, 0x08 - ����������� ����� � ������������
modbus_registers[237]=(signed char)(bps[26]._av);
modbus_registers[238]=(signed char)(bps[27]._Uii>>8);			//���120	�������� ���������� ����������� �28, 0.1�
modbus_registers[239]=(signed char)(bps[27]._Uii);
modbus_registers[240]=(signed char)(bps[27]._Ii>>8);			//���121	�������� ��� ����������� �28, 0.1�
modbus_registers[241]=(signed char)(bps[27]._Ii);
modbus_registers[242]=(signed char)(bps[27]._Ti>>8);			//���122	����������� ��������� ����������� �28, 1��
modbus_registers[243]=(signed char)(bps[27]._Ti);
modbus_registers[244]=(signed char)(bps[27]._av>>8);			//���123	���� ������ ����������� �28, 0x01 - ��������, 0x02 �������� U���, 0x04 �������� U���, 0x08 - ����������� ����� � ������������
modbus_registers[245]=(signed char)(bps[27]._av);
modbus_registers[246]=(signed char)(bps[28]._Uii>>8);			//���124	�������� ���������� ����������� �29, 0.1�
modbus_registers[247]=(signed char)(bps[28]._Uii);
modbus_registers[248]=(signed char)(bps[28]._Ii>>8);			//���125	�������� ��� ����������� �29, 0.1�
modbus_registers[249]=(signed char)(bps[28]._Ii);
modbus_registers[250]=(signed char)(bps[28]._Ti>>8);			//���126	����������� ��������� ����������� �29, 1��
modbus_registers[251]=(signed char)(bps[28]._Ti);
modbus_registers[252]=(signed char)(bps[28]._av>>8);			//���127	���� ������ ����������� �29, 0x01 - ��������, 0x02 �������� U���, 0x04 �������� U���, 0x08 - ����������� ����� � ������������
modbus_registers[253]=(signed char)(bps[28]._av);
modbus_registers[254]=(signed char)(bps[29]._Uii>>8);			//���128	�������� ���������� ����������� �30, 0.1�
modbus_registers[255]=(signed char)(bps[29]._Uii);
modbus_registers[256]=(signed char)(bps[29]._Ii>>8);			//���129	�������� ��� ����������� �30, 0.1�
modbus_registers[257]=(signed char)(bps[29]._Ii);
modbus_registers[258]=(signed char)(bps[29]._Ti>>8);			//���130	����������� ��������� ����������� �30, 1��
modbus_registers[259]=(signed char)(bps[29]._Ti);
modbus_registers[260]=(signed char)(bps[29]._av>>8);			//���131	���� ������ ����������� �30, 0x01 - ��������, 0x02 �������� U���, 0x04 �������� U���, 0x08 - ����������� ����� � ������������
modbus_registers[261]=(signed char)(bps[29]._av);
modbus_registers[262]=(signed char)(bps[30]._Uii>>8);			//���132	�������� ���������� ����������� �31, 0.1�
modbus_registers[263]=(signed char)(bps[30]._Uii);
modbus_registers[264]=(signed char)(bps[30]._Ii>>8);			//���133	�������� ��� ����������� �31, 0.1�
modbus_registers[265]=(signed char)(bps[30]._Ii);
modbus_registers[266]=(signed char)(bps[30]._Ti>>8);			//���134	����������� ��������� ����������� �31, 1��
modbus_registers[267]=(signed char)(bps[30]._Ti);
modbus_registers[268]=(signed char)(bps[30]._av>>8);			//���135	���� ������ ����������� �31, 0x01 - ��������, 0x02 �������� U���, 0x04 �������� U���, 0x08 - ����������� ����� � ������������
modbus_registers[269]=(signed char)(bps[30]._av);
modbus_registers[270]=(signed char)(bps[31]._Uii>>8);			//���136	�������� ���������� ����������� �32, 0.1�
modbus_registers[271]=(signed char)(bps[31]._Uii);
modbus_registers[272]=(signed char)(bps[31]._Ii>>8);			//���137	�������� ��� ����������� �32, 0.1�
modbus_registers[273]=(signed char)(bps[31]._Ii);
modbus_registers[274]=(signed char)(bps[31]._Ti>>8);			//���138	����������� ��������� ����������� �32, 1��
modbus_registers[275]=(signed char)(bps[31]._Ti);
modbus_registers[276]=(signed char)(bps[31]._av>>8);			//���139	���� ������ ����������� �32, 0x01 - ��������, 0x02 �������� U���, 0x04 �������� U���, 0x08 - ����������� ����� � ������������
modbus_registers[277]=(signed char)(bps[31]._av);



 /*
modbus_registers[106]=(signed char)(bps_U>>8);				//���54   	���������� ������������, 0.1�
modbus_registers[107]=(signed char)(bps_U);
tempS=0;
if(speedChIsOn) tempS=1;
modbus_registers[108]=(signed char)(tempS>>8);					//���55   	���������� ����� ������������, (1 - ���, 0 - ����)
modbus_registers[109]=(signed char)(tempS);
tempS=0;
if(spc_stat==spcVZ) tempS=1;
modbus_registers[110]=(signed char)(tempS>>8);					//���56   	������������� ����� ������������, (1 - ���, 0 - ����)
modbus_registers[111]=(signed char)(tempS);
modbus_registers[112]=(signed char)(uout_av>>8);					//���57   �������� ��������� ����������, (0 - �����, 1 - ��������, 2 - ��������)
modbus_registers[113]=(signed char)(uout_av);

tempS=0;													 //���60	������� ������ ��������� �������
if(bat_ips._av)			tempS|=(1<<0);						 // ��� 0	������ �������
if(avar_stat&0x0001)   	tempS|=(1<<1);						 //	��� 1	������ �������� ���� 
if(avar_stat&(1<<(3+0)))tempS|=(1<<2);						 //	��� 2	������ ����������� �1
if(avar_stat&(1<<(3+1)))tempS|=(1<<3);						 //	��� 3	������ ����������� �2
if(avar_stat&(1<<(3+2)))tempS|=(1<<4);						 //	��� 4	������ ����������� �2
modbus_registers[118]=(signed char)(tempS>>8);
modbus_registers[119]=(signed char)(tempS);

tempS=t_ext[0];
if(ND_EXT[0])tempS=-1000;
modbus_registers[400]=(signed char)(tempS>>8);				//���201	������� ������ ����������� �1
modbus_registers[401]=(signed char)(tempS);
tempS=t_ext[1];
if(ND_EXT[1])tempS=-1000;
modbus_registers[402]=(signed char)(tempS>>8);				//���202	������� ������ ����������� �2
modbus_registers[403]=(signed char)(tempS);
tempS=t_ext[2];
if(ND_EXT[2])tempS=-1000;
modbus_registers[404]=(signed char)(tempS>>8);				//���203	������� ������ ����������� �3
modbus_registers[405]=(signed char)(tempS);
/*tempS=t_ext[3];
if(ND_EXT[3])tempS=-1000;
modbus_registers[406]=(signed char)(tempS>>8);				//���204	������� ������ ����������� �4
modbus_registers[407]=(signed char)(tempS);   */
/*
tempS=0;
if(sk_stat[0]==ssON) tempS|=0x0001;
if(sk_av_stat[0]==sasON) tempS|=0x0002;
modbus_registers[420]=(signed char)(tempS>>8);				//���211	���������  ������ �������� �1, (������� ��� - ���������� ���������, 1 - �������, 0 - ���������, ������ ��� - �����������, 1 - ������, 0 - �����)
modbus_registers[421]=(signed char)(tempS);
tempS=0;
if(sk_stat[1]==ssON) tempS|=0x0001;
if(sk_av_stat[1]==sasON) tempS|=0x0002;
modbus_registers[422]=(signed char)(tempS>>8);				//���212	���������  ������ �������� �1, (������� ��� - ���������� ���������, 1 - �������, 0 - ���������, ������ ��� - �����������, 1 - ������, 0 - �����)
modbus_registers[423]=(signed char)(tempS);
tempS=0;
if(sk_stat[2]==ssON) tempS|=0x0001;
if(sk_av_stat[2]==sasON) tempS|=0x0002;
modbus_registers[424]=(signed char)(tempS>>8);				//���213	���������  ������ �������� �1, (������� ��� - ���������� ���������, 1 - �������, 0 - ���������, ������ ��� - �����������, 1 - ������, 0 - �����)
modbus_registers[425]=(signed char)(tempS);
tempS=0;
if(sk_stat[3]==ssON) tempS|=0x0001;
if(sk_av_stat[3]==sasON) tempS|=0x0002;
modbus_registers[426]=(signed char)(tempS>>8);				//���214	���������  ������ �������� �1, (������� ��� - ���������� ���������, 1 - �������, 0 - ���������, ������ ��� - �����������, 1 - ������, 0 - �����)
modbus_registers[427]=(signed char)(tempS);

//modbus_registers[
  */

if(prot==MODBUS_RTU_PROT)
	{
	modbus_tx_buff[0]=adr;
	modbus_tx_buff[1]=func;

	modbus_tx_buff[2]=(char)(reg_quantity*2);

	mem_copy((signed char*)&modbus_tx_buff[3],(signed char*)&modbus_registers[(reg_adr-1)*2],reg_quantity*2);

	crc_temp=CRC16_2(modbus_tx_buff,(reg_quantity*2)+3);

	modbus_tx_buff[3+(reg_quantity*2)]=(char)crc_temp;
	modbus_tx_buff[4+(reg_quantity*2)]=crc_temp>>8;

	for (i=0;i<(5+(reg_quantity*2));i++)
		{
		putchar0(modbus_tx_buff[i]);
		}
	for (i=0;i<(5+(reg_quantity*2));i++)
		{
		putchar_sc16is700(modbus_tx_buff[i]);
		}
	}
else if(prot==MODBUS_TCP_PROT)
	{
	mem_copy((signed char*)modbus_tx_buff,(signed char*)&modbus_registers[(reg_adr-1)*2],reg_quantity*2);
	modbus_tcp_out_ptr=(signed char*)modbus_tx_buff;
	}
}

