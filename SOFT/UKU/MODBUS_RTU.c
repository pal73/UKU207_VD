unsigned char NULL_0=0;

#include "MODBUS_RTU.h"
#include "main.h"

#include <lpc17xx.h>
#include "eeprom_map.h"

#include "MODBUS_func3.h"
#include "MODBUS_func4.h"
#include "MODBUS_func6.h"
#include "sc16is7xx.h"
#include "25lc640.h"

unsigned char mb_rtu_func;
unsigned long mb_rtu_start_adr;
unsigned char mb_rtu_num, mb_rtu_num_send;
unsigned short mb_data_1, mb_data_2, crc_f;
char modbus_timeout_cnt;
//char bMODBUS_TIMEOUT;
unsigned char modbus_rx_buff[30];	//Буфер, куда складывает принимаемые даннные обработчик прерывания по приему УАРТа
unsigned char modbus_rx_buff_ptr;	//Указатель на текущую позицию принимающего буфера
char sc16is700RecieveDisableFlag;
signed short modbusTimeoutInMills;
//-----------------------------------------------
// вычисление CRC побайтно, результат в unsigned short crc_f
//перед вычислением сделать crc_f=0xFFFF;
void crc_calc_f( unsigned short data)
{
  short i;
  crc_f ^= data;
  for ( i = 8; i != 0; i--){ 
      	if ((crc_f & 0x0001) != 0) { crc_f >>= 1; crc_f ^= 0xA001;}
      	else  crc_f >>= 1;
  }
  
}
//-----------------------------------------------
//-----------------------------------------------
unsigned short CRC16_MB(char* buf, short len)
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
//----------------
void modbus_puts (void) {
	mb_data_1=((unsigned short)modbus_rx_buff[2]<<8)+modbus_rx_buff[3];
	mb_data_2=((unsigned short)modbus_rx_buff[4]<<8)+modbus_rx_buff[5];
	mb_rtu_func=modbus_rx_buff[1];
	if(mb_rtu_func==6){
		sc16is700RecieveDisableFlag=1; //запретить прием во время отправки
	 	sc16is700_wr_buff_ptr(CS16IS7xx_THR, modbus_rx_buff, 8);//отправить подтверждение получения
	 	analiz_func6(mb_data_1, mb_data_2);				
	}
	else if(mb_rtu_func==3 || mb_rtu_func==4){
		if(mb_data_2<=127 && mb_data_2>0){
			mb_rtu_start_adr=mb_data_1;
			mb_rtu_start_adr=mb_rtu_start_adr*2;  //адрес в таблице			
			mb_rtu_num=(unsigned char)(mb_data_2<<1);
			if((mb_rtu_start_adr+mb_rtu_num) <=0x20000UL){
				mb_rtu_num_send=0;
			}
			else mb_rtu_num=0;
		} 
	}
}

//--------------------------------------------------------------
//Обработчик sc16is700 
void sc16is700_uart_hndl_mb(void)
{

sc16is700ByteAvailable=sc16is700_rd_byte(CS16IS7xx_RXLVL); //Читаем состояние ФИФО приема микросхемы

if(sc16is700ByteAvailable) //Если в приемном ФИФО	микросхемы есть данные
	{	  
	char i;
	for(i=0;(i<sc16is700ByteAvailable)&&(i<5);i++) //Читаем их пачками не больше 5 в программный буфер модбас
		{  
		if(!sc16is700RecieveDisableFlag) //если не идет передача данных
			{
				char zi;
				for(zi=1;zi<8;zi++) modbus_rx_buff[zi-1]=modbus_rx_buff[zi];
				modbus_rx_buff[7]=sc16is700_rd_byte(CS16IS7xx_RHR);
				if(modbus_rx_buff_ptr==8) {modbus_rx_buff_ptr=0;} //если после запроса идут данные, то сброс и не высылать ответ
				if(modbus_rx_buff[0]==MODBUS_ADRESS && (modbus_rx_buff[1]==3 || modbus_rx_buff[1]==4 || modbus_rx_buff[1]==6) &&
					CRC16_MB((char*)modbus_rx_buff,6)==(((unsigned short)modbus_rx_buff[7])<<8) + modbus_rx_buff[6] ){    //*((short*)&modbus_rx_buff[6]) ){
					modbus_timeout_cnt=0;   //Запускаем таймер задержки отправки посылки 
					modbus_rx_buff_ptr=8;					
				}
		}
		else sc16is700_rd_byte(CS16IS7xx_RHR); //считываем данные, которые передали, попавшие в буфер приема.
		}
	}

if(mb_rtu_num!=0){
	sc16is700TxFifoLevel=sc16is700_rd_byte(CS16IS7xx_TXLVL);//Читаем сколько свободно в ФИФО передачи 
 	if(sc16is700TxFifoLevel>0){ //если есть свободное место в ФИФО передачи
	 unsigned char z=0;
	 if(mb_rtu_num_send==0){ //если начало передачи
	 	if(sc16is700TxFifoLevel==64){ //начинать, если буфер пустой
			sc16is700RecieveDisableFlag=1;
			crc_f=0xFFFF;
			sc16is700_spi_init();
			delay_us(2);
			sc16is700_CS_ON
			spi1((CS16IS7xx_THR&0x0f)<<3);
			spi1(MODBUS_ADRESS);
			crc_calc_f(MODBUS_ADRESS);
			spi1(mb_rtu_func);
			crc_calc_f(mb_rtu_func);
			spi1(mb_rtu_num);
			crc_calc_f(mb_rtu_num);
			z=0;
			while (mb_rtu_num_send<mb_rtu_num && z<61){
				unsigned char data_reg=0;
				unsigned long adrr_reg;
				adrr_reg=mb_rtu_start_adr+mb_rtu_num_send;
				if(mb_rtu_func==4){
					if(adrr_reg<MODBUS_FUNC_4_LENGTH) data_reg=*reg_func4[adrr_reg];
					else data_reg=0; //если запрос регистров за пределами таблицы регистров, то 0
				}
				else if(mb_rtu_func==3){
					if(adrr_reg<MODBUS_FUNC_3_LENGTH) data_reg=*reg_func3[adrr_reg];
					else data_reg=0; //если запрос регистров за пределами таблицы регистров, то 0				
				}
				spi1(data_reg);
				crc_calc_f(data_reg);
				++mb_rtu_num_send;
				++z;
			}	
			if(mb_rtu_num_send==mb_rtu_num && z<61){
				spi1((unsigned char)crc_f);
				++mb_rtu_num_send;
				++z;
			}
			if(mb_rtu_num_send>mb_rtu_num && z<61) {
				spi1((unsigned char)(crc_f>>8));					
				mb_rtu_num=0;//закончить передачу
			}	
 			sc16is700_CS_OFF 			
		}
	 }
	 else{ //отправка остальных регистров
	 	sc16is700RecieveDisableFlag=1;
	 	sc16is700_spi_init();
		delay_us(2);
		sc16is700_CS_ON
		spi1((CS16IS7xx_THR&0x0f)<<3);
		if(mb_rtu_num_send<mb_rtu_num){//если отправлены не все регистры
			z=0;
			while (mb_rtu_num_send<mb_rtu_num && z<sc16is700TxFifoLevel){
				unsigned char data_reg=0;
				unsigned long adrr_reg;
				adrr_reg=mb_rtu_start_adr+mb_rtu_num_send;
				if(mb_rtu_func==4){
					if(adrr_reg<MODBUS_FUNC_4_LENGTH)	data_reg=*reg_func4[adrr_reg];
					else data_reg=0; //если запрос регистров за пределами таблицы регистров, то 0
				}
				else if(mb_rtu_func==3){
					if(adrr_reg<MODBUS_FUNC_3_LENGTH)	data_reg=*reg_func3[adrr_reg];
					else data_reg=0; //если запрос регистров за пределами таблицы регистров, то 0				
				}
				spi1(data_reg);
				crc_calc_f(data_reg);
				++mb_rtu_num_send;
				++z;
			}
		}

		if(mb_rtu_num_send==mb_rtu_num && z<sc16is700TxFifoLevel){
				spi1((unsigned char)crc_f);
				++mb_rtu_num_send;
				++z;
		}
		if(mb_rtu_num_send>mb_rtu_num && z<sc16is700TxFifoLevel) {
					spi1((unsigned char)(crc_f>>8));					
					mb_rtu_num=0;//закончить передачу
		}
 		sc16is700_CS_OFF

	 } 
	}
}
//Ожидаем, когда освободятся буферы передачи и приема:
if((sc16is700_rd_byte(CS16IS7xx_LSR))&0x40)	sc16is700RecieveDisableFlag=0;


}
//----------------------------------------------- 
//Запись с массива
void sc16is700_wr_buff_ptr(char reg_num, unsigned char *buff, char num)
{
short i;
sc16is700_spi_init();
delay_us(2);
sc16is700_CS_ON 
spi1((reg_num&0x0f)<<3);
for (i=0;i<num;i++)spi1(*(buff+i));
sc16is700_CS_OFF
}
//-------------------














