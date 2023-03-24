extern unsigned char NULL_0;
extern unsigned char mb_rtu_func;
extern unsigned long mb_rtu_start_adr;
extern unsigned char mb_rtu_num, mb_rtu_num_send;
extern unsigned short mb_data_1, mb_data_2, crc_f;
extern char modbus_timeout_cnt;
//extern char bMODBUS_TIMEOUT;
//extern unsigned char modbus_rx_buffer[30];	//Буфер, куда складывает принимаемые даннные обработчик прерывания по приему УАРТа
//extern unsigned char modbus_rx_buffer_ptr;	//Указатель на текущую позицию принимающего буфера
extern char sc16is700RecieveDisableFlag;
extern signed short modbusTimeoutInMills;

void analiz_func6(unsigned short mbadr, unsigned short mbdat);
char lc640_write_int(short ADR,short in);
void putchar_sc16is700(char out_byte);
void crc_calc_f( unsigned short data);
void modbus_puts (void);
unsigned short CRC16_MB(char* buf, short len);
void sc16is700_uart_hndl_mb(void);
void sc16is700_wr_buff_ptr(char reg_num, unsigned char *buff, char num);
