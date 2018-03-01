//������� �����
//#define SC16IS740_UART

#ifdef UKU2071x
#define can1_out mcp2515_transmit
#endif


#include "lcd_AGM1232_uku207_3.h"
#include "rtl.h"
#include "type.h"
#include "main.h"
#include "simbol.h"
#include "25lc640.h"
#include "Timer.h"
#include "gran.h"
#include "uart0.h"
#include "uart1.h"
#include "uart2.h"
#include "cmd.h"
#include "ret.h"
#include "eeprom_map.h"
#include "common_func.h"
#include "control.h"
#include "mess.h"
#include "full_can.h"
#include "watchdog.h"
#include "ad7705.h"
#include "beep.h"
#include "avar_hndl.h"
#include "memo.h"
#include "simbols.h"
#include "graphic.h"
#include "snmp_data_file.h" 
#include "net_config.h"
#include "uart0.h"
#include <rtl.h>
#include "modbus.h"
#include "sacred_sun.h"
#include "ztt.h"
#include "mcp2515.h"
//#include "sc16is7xx.h"
#include "modbus_tcp.h"

extern U8 own_hw_adr[];
extern U8  snmp_Community[];
BOOL tick;
extern LOCALM localm[];
#define MY_IP localm[NETIF_ETH].IpAdr
#define DHCP_TOUT   50

//***********************************************
//������
char b10000Hz,b1000Hz,b2000Hz,b100Hz,b50Hz,b10Hz,b5Hz,b2Hz,b1Hz,b1min;
short t0cnt,t0cnt0,t0cnt1,t0cnt2,t0cnt3,t0cnt4,t0cnt5,t0cnt6,t0_cnt7,t0_cnt_min,t0cntMin;
char bFL5,bFL2,bFL,bFL_,bTPS;
signed short main_10Hz_cnt=0;
signed short main_1Hz_cnt=0;

 
//***********************************************
//��������� �����
char cnt_of_slave=3;
//char cnt_of_wrks;   //����������� ���������� ���������� , ��� ���������



//**********************************************
//������������, ������������ �� EEPROM
signed short Ktsrc[2];
signed short Kusrc[2];
signed short Kisrc[2];
signed short Ki0src[2];
signed short Kubat[2];
signed short Kubatm[2];
unsigned short Kibat0[2];
signed short Kibat1[2];
signed short Ktbat[2];
signed short Kunet;
signed short Kunet_ext[3];
signed short Ktext[3];
signed short Kuload;
signed short KunetA;
signed short KunetB;
signed short KunetC;
signed short Kubps;
signed short Kuout;
signed short Kuvd;

signed short MAIN_IST;
signed short UMAX;
signed short UOUT;
signed short UOUT_;
signed short UAVT;
signed short TMAX;
signed short TSIGN;
signed short TSYSMAX;
signed short AV_OFF_AVT;
signed short USIGN;
signed short UMN;
signed short ZV_ON;
signed short UINMAX;
signed short UINMIN;
signed short UOUTMAX;
signed short UOUTMIN;

signed short IKB;
//signed short KVZ;

signed short IMAX;
signed short IMIN;
signed short APV_ON;
signed short IZMAX;
signed short U0B;
signed short TZAS;
signed short VZ_HR;
signed short TBAT;
signed short U_AVT;
signed short DU;
//signed short PAR;

signed short RELE_SET_MASK[4]={1,2,3,4};

signed short TBATMAX;
signed short TBATSIGN;
signed short UBM_AV;
signed short RELE_LOG;
signed short TBOXMAX;
signed short TBOXREG;
signed short TBOXVENTMAX;
signed short TLOADDISABLE;
signed short TLOADENABLE;
signed short TBATDISABLE;
signed short TBATENABLE;
signed short TVENTON;
signed short TVENTOFF;
signed short TWARMON;
signed short TWARMOFF;
enum_releventsign RELEVENTSIGN;
signed short TZNPN;
signed short UONPN;
signed short UVNPN;
enum_npn_out NPN_OUT;
enum_npn_sign NPN_SIGN;
signed short TERMOKOMPENS;
signed short TBOXVENTON; 
signed short TBOXVENTOFF;
signed short TBOXWARMON; 
signed short TBOXWARMOFF;
signed short BAT_TYPE;	//��� �������. 0 - ������� ���������, 1-�������� COSLIGHT, 2-�������� SACRED SUN , 3-�������� ZTT
signed short DU_LI_BAT;	//��������, ������������ ���������� ���������� �������� �������
signed short FORVARDBPSCHHOUR;	//������������������ �������� ��������� � �����. ���� 0 - ������� ��������� � ������� ������ ��������
signed short NUMBAT;
signed short NUMBAT_TELECORE;
signed short NUMIST;
signed short NUMINV;
signed short NUMDT;
signed short NUMSK;
signed short NUMEXT;
signed short NUMAVT;
signed short NUMMAKB;
signed short NUMBYPASS;
signed short U_OUT_KONTR_MAX;
signed short U_OUT_KONTR_MIN;
signed short U_OUT_KONTR_DELAY;
signed short DOP_RELE_FUNC;
signed short CNTRL_HNDL_TIME;	//���������� ������� ������������� ���������� ��� ��������
signed short USODERG_LI_BAT;	//���������� ���������� �������� �������
signed short QSODERG_LI_BAT;	//����� ��� ������� �������� ����������� ���������� ���������� �������� �������
signed short TVENTMAX;			//������������ ������ �����������
signed short ICA_EN;			//������������ ������ ������������ ����� ���
signed short ICA_CH;			//����� ����� ��� ������������ �����, 0 - MODBUS, 1 - MODBUS-TCP
signed short ICA_MODBUS_ADDRESS;//����� �������� ��� ������������ ����� �� ���� MODBUS-RTU
signed short ICA_MODBUS_TCP_IP1,ICA_MODBUS_TCP_IP2,ICA_MODBUS_TCP_IP3,ICA_MODBUS_TCP_IP4;	//IP �������� ��� ������������ ����� �� ���� MODBUS-TCP
signed short ICA_MODBUS_TCP_UNIT_ID;	//UNIT ID �������� ��� ������������ ����� �� ���� MODBUS-TCP
signed short PWM_START;			//��������� ��� ��� ������
signed short KB_ALGORITM;		//2-� ��� 3-� ������������ �������� �������� ���� �������
signed short REG_SPEED;			//�������� �������������, 1- �����������, 2,3,4,5- ����������� � 2,3,4,5 ���
enum_apv_on APV_ON1,APV_ON2;
signed short APV_ON2_TIME;

enum_bat_is_on BAT_IS_ON[2];
signed short BAT_DAY_OF_ON[2];
signed short BAT_MONTH_OF_ON[2];
signed short BAT_YEAR_OF_ON[2];
signed short BAT_C_NOM[2];
signed short BAT_RESURS[2];
signed short BAT_C_REAL[2];
//signed short BAT_TYPE[2];

unsigned short AUSW_MAIN;
unsigned long AUSW_MAIN_NUMBER;
unsigned short AUSW_DAY;
unsigned short AUSW_MONTH;
unsigned short AUSW_YEAR;
unsigned short AUSW_UKU;
unsigned short AUSW_UKU_SUB;
unsigned long AUSW_UKU_NUMBER;
unsigned long	AUSW_BPS1_NUMBER;
unsigned long  AUSW_BPS2_NUMBER;
unsigned short AUSW_RS232;
unsigned short AUSW_PDH;
unsigned short AUSW_SDH;
unsigned short AUSW_ETH;

signed short TMAX_EXT_EN[3];
signed short TMAX_EXT[3];
signed short TMIN_EXT_EN[3];
signed short TMIN_EXT[3];
signed short T_EXT_REL_EN[3];
signed short T_EXT_ZVUK_EN[3];
signed short T_EXT_LCD_EN[3];
signed short T_EXT_RS_EN[3];

signed short SK_SIGN[4];
signed short SK_REL_EN[4];
signed short SK_ZVUK_EN[4];
signed short SK_LCD_EN[4];
signed short SK_RS_EN[4];

enum_avz AVZ;

unsigned short HOUR_AVZ;
unsigned short MIN_AVZ;
unsigned short SEC_AVZ;
unsigned short DATE_AVZ;
unsigned short MONTH_AVZ;
unsigned short YEAR_AVZ;
unsigned short AVZ_TIME;

enum_mnemo_on MNEMO_ON;
unsigned short MNEMO_TIME;

signed short POWER_CNT_ADRESS;

signed short ETH_IS_ON;
signed short ETH_DHCP_ON;
signed short ETH_IP_1;
signed short ETH_IP_2;
signed short ETH_IP_3;
signed short ETH_IP_4;
signed short ETH_MASK_1;
signed short ETH_MASK_2;
signed short ETH_MASK_3;
signed short ETH_MASK_4;
signed short ETH_TRAP1_IP_1;
signed short ETH_TRAP1_IP_2;
signed short ETH_TRAP1_IP_3;
signed short ETH_TRAP1_IP_4;
signed short ETH_TRAP2_IP_1;
signed short ETH_TRAP2_IP_2;
signed short ETH_TRAP2_IP_3;
signed short ETH_TRAP2_IP_4;
signed short ETH_TRAP3_IP_1;
signed short ETH_TRAP3_IP_2;
signed short ETH_TRAP3_IP_3;
signed short ETH_TRAP3_IP_4;
signed short ETH_TRAP4_IP_1;
signed short ETH_TRAP4_IP_2;
signed short ETH_TRAP4_IP_3;
signed short ETH_TRAP4_IP_4;
signed short ETH_TRAP5_IP_1;
signed short ETH_TRAP5_IP_2;
signed short ETH_TRAP5_IP_3;
signed short ETH_TRAP5_IP_4;

signed short ETH_SNMP_PORT_READ;
signed short ETH_SNMP_PORT_WRITE;

signed short ETH_GW_1;
signed short ETH_GW_2;
signed short ETH_GW_3;
signed short ETH_GW_4;

signed short RELE_VENT_LOGIC;

signed short MODBUS_ADRESS;
signed short MODBUS_BAUDRATE;
signed short BAT_LINK;
signed short I_LOAD_MODE;	//������ ��������� ��������� ���� - �� ����� ��� ��� ����� ����� ����������



//***********************************************
//��������� �������
BAT_STAT bat[2],bat_ips;
signed short		bat_u_old_cnt;
signed short 		Ib_ips_termokompensat;

//***********************************************
//�������� ���
MAKB_STAT makb[4];

//***********************************************
//�������� ���
LAKB_STAT lakb[3];
char lakb_damp[1][42];
char bLAKB_KONF_CH=0;
char bLAKB_KONF_CH_old=0;
char lakb_ison_mass[7];
short lakb_mn_ind_cnt;
char bLAKB_KONF_CH_EN;
//char bRS485ERR;
short LBAT_STRUKT;
char lakb_error_cnt;		//������� ������������� ��������� ����������� �������
short numOfPacks,numOfPacks_;
short numOfCells, numOfTemperCells, baseOfData;
short lakb_stat_comm_error;	//����������� ������ ����� � ��������� ���������. 0 �������� ����������� ����� ���������� � ������� ����� �� ����� ��������� ���������
short lakbNotErrorNum;		//����������� �������� ������� � ��������� ������
short lakbKanErrorCnt;		//������� ����������� ������ ����� � ������ ����������
short lakbKanErrorStat;		//��������� ����������� ������ ����� � ������ ����������

//#ifdef UKU_TELECORE2015
//***********************************************
//��������� �������� �������  
LI_BAT_STAT li_bat;
//#endif

//***********************************************
//���������� �� ���������� ����
char can_slot[12][16];


//***********************************************
//��������� ����������
BPS_STAT bps[29];

//***********************************************
//��������� ����������
#ifdef UKU_220_V2
INV_STAT inv[3];
#endif
#ifndef UKU_220_V2
INV_STAT inv[20];
#endif
char first_inv_slot=MINIM_INV_ADRESS;

//***********************************************
//��������� �������
BYPS_STAT byps;

//***********************************************
//��������� ��������
signed short load_U;
signed short load_I;

//***********************************************
//��������� ������
signed short bps_U;
signed short out_U;
signed short in_U;
signed short out_I;
signed short vd_U;
signed short bps_I;
signed short sys_T;


//***********************************************
//���������

char lcd_buffer[LCD_SIZE+100]={"Hello World"};
signed char parol[3];
char phase;
char lcd_bitmap[1024];
char dig[5];
char dumm_ind[20];
stuct_ind a_ind,b_ind[10],c_ind;
char dumm_ind_[20];
char zero_on;
char mnemo_cnt=50;
char simax;
short av_j_si_max;
const char ABCDEF[]={"0123456789ABCDEF"};
const char sm_mont[13][4]={"   ","���","���","���","���","���","���","���","���","���","���","���","���"}; //
signed short ptr_ind=0;

signed short ind_pointer=0;

//***********************************************
//��������� ��������� ����
signed short net_U,net_Ustore,net_Ua,net_Ub,net_Uc;
char bFF,bFF_;
signed short net_F,hz_out,hz_out_cnt,net_F3;
signed char unet_drv_cnt;
char net_av;

//***********************************************
//��������� ������� ��������
//signed short tout[4];
char tout_max_cnt[4],tout_min_cnt[4];
enum_tout_stat tout_stat[4];
signed short t_ext[3];

signed char sk_cnt_dumm[4],sk_cnt[4],sk_av_cnt[4];
enum_sk_stat sk_stat[4]={ssOFF,ssOFF,ssOFF,ssOFF};
enum_sk_av_stat sk_av_stat[4]={sasOFF,sasOFF,sasOFF,sasOFF},sk_av_stat_old[4];
signed short t_box,t_box_warm,t_box_vent;
char ND_EXT[3];
//***********************************************
//�����
extern char beep_cnt;
BOOL bSILENT;








signed short u_necc,u_necc_,u_necc_up,u_necc_dn;
signed short main_cnt_5Hz;
signed short num_necc;
signed short num_necc_Imax;
signed short num_necc_Imin;
signed short cnt_num_necc;
//char bSAME_IST_ON;
signed mat_temper;

//***********************************************
//���
unsigned main_apv_cnt,hour_apv_cnt[2],reset_apv_cnt[2];
unsigned short apv_cnt_sec[2],apv_cnt[2];

//***********************************************
//��������� ���������
const char sm_[]	={"                    "};
const char sm_exit[]={" �����              "};
const char sm_time[]={" 0%:0^:0& 0</>  /0{ "};





//**********************************************
//������ � �������� 
char but;                            
unsigned long but_n,but_s;
char but0_cnt;
char but1_cnt;
char but_onL_temp;

//***********************************************
//����������� �����
signed char cnt_net_drv;

//***********************************************
//��� 
extern char ptr_can1_tx_wr,ptr_can1_tx_rd;
extern char ptr_can2_tx_wr,ptr_can2_tx_rd;
extern unsigned short rotor_can[6];
extern char RXBUFF[40],TXBUFF[40];





//***********************************************
//������ � ��������
char speed,l_but,n_but;

//***********************************************
//�������������
enum {wrkON=0x55,wrkOFF=0xAA}wrk;
char cnt_wrk;
signed short ibat_integr;
unsigned short av_beep,av_rele,av_stat;
char default_temp;
char ND_out[3];

//***********************************************
//����
enum_tst_state tst_state[15];

//***********************************************
//���
//extern short adc_buff[16][16],adc_buff_[16];
extern char adc_cnt,adc_cnt1,adc_ch;

//***********************************************

char flag=0;


extern signed short bat_ver_cnt;
signed short Isumm;
signed short Isumm_;

#include <LPC17xx.H>                        /* LPC21xx definitions */



/*
extern void lcd_init(void);
extern void lcd_on(void);
extern void lcd_clear(void);
*/

extern short plazma_adc_cnt;
extern char net_buff_cnt;
extern unsigned short net_buff[32],net_buff_;
extern char rele_stat/*,rele_stat_*/;
extern char bRXIN0;


char cntrl_plazma;
extern char bOUT_FREE2;
extern char /*av_net,*//*av_bat[2],*/av_bps[12],av_inv[6],av_dt[4],av_sk[4];

char content[63];

//const short ptr_bat_zar_cnt[2]={EE_ZAR1_CNT,EE_ZAR2_CNT};


//unsigned short YEAR_AVZ,MONTH_AVZ,DATE_AVZ,HOUR_AVZ,MIN_AVZ,SEC_AVZ;


//**********************************************
//���������������
extern signed short samokalibr_cnt;

//**********************************************
//���������
extern char mess[MESS_DEEP],mess_old[MESS_DEEP],mess_cnt[MESS_DEEP];
extern short mess_par0[MESS_DEEP],mess_par1[MESS_DEEP],mess_data[2];


//**********************************************
//�������� ������� �������
extern signed short 	main_kb_cnt;
extern signed short 	kb_cnt_1lev;
extern signed short 	kb_cnt_2lev;
extern char 			kb_full_ver;
extern char 			kb_start[2],kb_start_ips;

//***********************************************
//���������� �����
extern signed short cntrl_stat;
extern signed short cntrl_stat_old;
extern signed short cntrl_stat_new;
extern signed short Ibmax;


//-----------------------------------------------
//�������� ������
char sign_U[2],sign_I[2];
char superviser_cnt;


char plazma_plazma_plazma;

char bRESET=0;
char bRESET_EXT=0;
char ext_can_cnt;
char bRESET_INT_WDT=0;
char bRESET_EXT_WDT=0;
//-----------------------------------------------
//��������� ������
signed short vvod_pos;

//-----------------------------------------------
//����� ����������
unsigned short adc_buff_ext_[3];
unsigned short Uvv[3];
unsigned short Uvv0;
short pos_vent;
short t_ext_can;
char t_ext_can_nd;


//-----------------------------------------------
//����� ���������� 2
char eb2_data[30];
short eb2_data_short[10];
short Uvv_eb2[3],Upes_eb2[3];
short Kvv_eb2[3],Kpes_eb2[3];
//-----------------------------------------------
//������ �� ��������
signed long power_summary;
signed short power_current;
signed long power_summary_tempo,power_summary_tempo_old;
signed short power_current_tempo,power_current_tempo_old;
char powerSummaryCnt;
char powerCurrentCnt;

//-----------------------------------------------
//�������������� � �����������
signed short main_vent_pos;
signed char t_box_cnt=0;
enum_mixer_vent_stat mixer_vent_stat=mvsOFF;
INT_BOX_TEMPER ibt;
enum_tbatdisable_stat tbatdisable_stat=tbdsON;
enum_tloaddisable_stat tloaddisable_stat=tldsON;
enum_av_tbox_stat av_tbox_stat=atsOFF;
signed short av_tbox_cnt;
char tbatdisable_cmnd=20,tloaddisable_cmnd=22;
short tbatdisable_cnt,tloaddisable_cnt;
#ifdef UKU_KONTUR
short t_box_vent_on_cnt;
short t_box_warm_on_cnt;
enum_vent_stat vent_stat_k=vsON;
enum_warm_stat warm_stat_k=wsON;
#endif

#ifdef UKU_TELECORE2015 
short t_box_vent_on_cnt;
short t_box_warm_on_cnt;
short t_box_vvent_on_cnt;
enum_vent_stat vent_stat_k=vsON,vvent_stat_k=vsON;
enum_warm_stat warm_stat_k=wsON;
signed short TELECORE2015_KLIMAT_WARM_ON_temp;
#endif

#ifdef UKU_TELECORE2017 
short t_box_vent_on_cnt;
short t_box_warm_on_cnt;
short t_box_vvent_on_cnt;
enum_vent_stat vent_stat_k=vsON,vvent_stat_k=vsON;
enum_warm_stat warm_stat_k=wsON;
signed short TELECORE2017_KLIMAT_WARM_ON_temp;

signed char t_box_warm_minus20_cnt;
signed char t_box_warm_plus65_cnt;
signed char t_box_cool_plus70_cnt;
#endif

//-----------------------------------------------
//��������� �������������� ��������� �������� 
enum_avt_stat avt_stat[12],avt_stat_old[12];

//short sys_plazma,sys_plazma1;

char snmp_plazma;


short plazma_but_an;

char bCAN_OFF;


char max_net_slot;

//-----------------------------------------------
//��������� ��� �� ����� ��������� ���� �������
signed long ibat_metr_buff_[2];
short bIBAT_SMKLBR;
char ibat_metr_cnt=0;


//-----------------------------------------------
//�������������� TELECORE2015	
#ifdef UKU_TELECORE2015
signed short TELECORE2015_KLIMAT_WARM_SIGNAL;
signed short TELECORE2015_KLIMAT_VENT_SIGNAL;
signed short TELECORE2015_KLIMAT_WARM_ON;
signed short TELECORE2015_KLIMAT_WARM_OFF;
signed short TELECORE2015_KLIMAT_CAP;
signed short TELECORE2015_KLIMAT_VENT_ON;
signed short TELECORE2015_KLIMAT_VENT_OFF;
signed short TELECORE2015_KLIMAT_VVENT_ON;
signed short TELECORE2015_KLIMAT_VVENT_OFF;
#endif  

//-----------------------------------------------
//���������� ����������������� ���������
signed short npn_tz_cnt;
enum_npn_stat npn_stat=npnsON;


char ips_bat_av_vzvod=0;
char ips_bat_av_stat=0;

char rel_warm_plazma;
char can_byps_plazma0,can_byps_plazma1;

char bCAN_INV;
char plazma_can_inv[3];

unsigned short bat_drv_rx_cnt;
char bat_drv_rx_buff[512];
char bat_drv_rx_in;

short plazma_bat_drv0,plazma_bat_drv1,bat_drv_cnt_cnt;
short can_plazma;

//-----------------------------------------------
//���������� �����
signed short speedChrgCurr;			//������������ ��� ����������� ������, ����������� �� ������
signed short speedChrgVolt;			//������������ ���������� ����������� ������, ����������� �� ������
signed short speedChrgTimeInHour; 		//������������ ����� ����������� ������ � �����, ����������� �� ������
signed short speedChrgAvtEn;	    		//�������������� ��������� ����������� ������ ��������/���������
signed short speedChrgDU;	    		//�������� ���������� ����������� ��� ��������� ����������� ������
signed short speedChIsOn;			//������� ��������� ����������� ������ ���/����
signed long  speedChTimeCnt;			//������� ������� ������ ����������� ������
signed short speedChrgBlckSrc;		//�������� ������� ����������, 0-����., 1-��1, 2-��2
signed short speedChrgBlckLog;		//������ ������� ����������, 1 - ���������� �� ���������� ��, 0 - �� ������������
signed short speedChrgBlckStat;		//������ ���������� ��� �������������� � ����������� ������.
char  	   speedChrgShowCnt;		//������� ������ ��������������� ���������

//-----------------------------------------------
//���������� ���
signed short ipsBlckSrc;
signed short ipsBlckLog;
signed short ipsBlckStat;


//-----------------------------------------------
//�������� ��������� ����������
signed short outVoltContrHndlCnt;		//�������, ������� � ���� � ������ ���������� ������� ������
signed short outVoltContrHndlCnt_;		//�������, ������� � ���� � ������ ���������� ���������� ������� ������
char uout_av;

//-----------------------------------------------
//�������� ������ ������������
char bVDISWORK;
char vd_is_work_cnt;

short plazma_numOfCells;
short plazma_numOfTemperCells;
short plazma_numOfPacks;

#ifdef MCP2515_CAN
#define can1_out	mcp2515_transmit
#endif


char plazma_ztt[2];
char plazma1809;

U8 socket_tcp;

//-----------------------------------------------
//������ ������������
//char vent_resurs_temp[4];

//-----------------------------------------------
//������������ ����� ���
char ica_plazma[10];
char ica_timer_cnt;
signed short ica_my_current;
signed short ica_your_current;
signed short ica_u_necc;
U8 tcp_soc_avg;
U8 tcp_connect_stat;

//-----------------------------------------------
//������ �� ����������
short overloadHndlCnt;
char overloadAvar;

//-----------------------------------------------
//������ �� ��������� ����������
short uAvarHndlOutUMaxCnt,uAvarHndlOutUMinCnt;
char uOutAvar;

//-----------------------------------------------
//������ �� �������� ����������
short uAvarHndlInUMaxCnt,uAvarHndlInUMinCnt;
char uInAvar;

//-----------------------------------------------
//������ �� ����������� �������
short sysTAvarHndlCnt;
char sysTAvar;

short pvlk;
char klbr_en;

//***********************************************
//��������� ������ ��� ����
short rele_av_flags;
short avar_vd_stat;
// ��� 0 - ������ ������ �� ���
// ��� 1 - ���������� ������� �� ����
// ��� 2 - �������� �������
// ��� 3 - �������� ���������� ��������
// ��� 4 - �������� ���������� ��������
// ��� 5 - ������� ���������� ��������
// ��� 6 - ������� ���������� ��������
// ��� 7 - ������ ����� �� ���� KAN
// ��� 8 - ������������ � ������

//-----------------------------------------------
void rtc_init (void) 
{
LPC_RTC->CCR=0x11;
}

//-----------------------------------------------
static void timer_poll () 
{
if (SysTick->CTRL & 0x10000) 
     {
     timer_tick ();
     tick = __TRUE;
     }
}

//-----------------------------------------------
void inv_search(void)
{
char i;

first_inv_slot=8;
for(i=0;i<12;i++)
	{
	if(bps[i]._device==dINV)
		{
		first_inv_slot=i;
		break;

		}
	}
}

//-----------------------------------------------
signed short abs_pal(signed short in)
{
if(in<0)return -in;
else return in;
}

//-----------------------------------------------
void init_ETH(void)
{
localm[NETIF_ETH].IpAdr[0]=lc640_read_int(EE_ETH_IP_1);
localm[NETIF_ETH].IpAdr[1]=lc640_read_int(EE_ETH_IP_2);
localm[NETIF_ETH].IpAdr[2]=lc640_read_int(EE_ETH_IP_3);
localm[NETIF_ETH].IpAdr[3]=lc640_read_int(EE_ETH_IP_4);

localm[NETIF_ETH].NetMask[0]=lc640_read_int(EE_ETH_MASK_1);
localm[NETIF_ETH].NetMask[1]=lc640_read_int(EE_ETH_MASK_2);
localm[NETIF_ETH].NetMask[2]=lc640_read_int(EE_ETH_MASK_3);
localm[NETIF_ETH].NetMask[3]=lc640_read_int(EE_ETH_MASK_4);

localm[NETIF_ETH].DefGW[0]=lc640_read_int(EE_ETH_GW_1);
localm[NETIF_ETH].DefGW[1]=lc640_read_int(EE_ETH_GW_2);
localm[NETIF_ETH].DefGW[2]=lc640_read_int(EE_ETH_GW_3);
localm[NETIF_ETH].DefGW[3]=lc640_read_int(EE_ETH_GW_4);

}


//-----------------------------------------------
void ADC_IRQHandler(void) {
LPC_ADC->ADCR &=  ~(7<<24);



adc_self_ch_buff[adc_self_ch_cnt]=(LPC_ADC->ADGDR>>4) & 0xFFF;/* Read Conversion Result             */
adc_self_ch_cnt++;
if(adc_self_ch_cnt<3)
	{
	LPC_ADC->ADCR |=  (1<<24);
	}
else
	{

 
	//SET_REG(LPC_ADC->ADCR,1,24,3);
	}

/*			adc_buff_[0]=AD_last;
			if(AD_last<adc_buff_min[adc_ch])adc_buff_min[adc_ch]=AD_last;
			if(AD_last>adc_buff_max[adc_ch])adc_buff_max[adc_ch]=AD_last;*/
}

//-----------------------------------------------
void def_set(int umax__,int ub0__,int ub20__,int usign__,int imax__,int uob__,int numi,int _uvz)
{
;
lc640_write_int(EE_NUMIST,numi);
lc640_write_int(EE_NUMINV,0);
//lc640_write_int(EE_NUMDT,0);
//lc640_write_int(EE_NUMSK,0);
lc640_write_int(EE_MAIN_IST,0);
lc640_write_int(EE_PAR,1);
lc640_write_int(EE_TBAT,60);
lc640_write_int(EE_UMAX,umax__);
lc640_write_int(EE_DU,ub20__/2);
//lc640_write_int(EE_UB0,ub0__);
//lc640_write_int(EE_UB20,ub20__);
lc640_write_int(EE_TSIGN,70);
lc640_write_int(EE_TMAX,80);
//lc640_write_int(EE_C_BAT,180);
lc640_write_int(EE_USIGN,usign__);
lc640_write_int(EE_UMN,187);
lc640_write_int(EE_ZV_ON,0);
lc640_write_int(EE_IKB,10);
//lc640_write_int(EE_KVZ,1030);
//lc640_write_int(EE_UVZ,_uvz);
lc640_write_int(EE_IMAX,imax__);
lc640_write_int(EE_IMIN,(imax__*8)/10);
//lc640_write_int(EE_APV_ON,apvON);
lc640_write_int(EE_APV_ON1,apvON);
lc640_write_int(EE_APV_ON2,apvON);
lc640_write_int(EE_APV_ON2_TIME,1);
lc640_write_int(EE_IZMAX,160);
lc640_write_int(EE_U0B,uob__);
lc640_write_int(EE_TZAS,3);
lc640_write_int(EE_TBATMAX,50);  
lc640_write_int(EE_TBATSIGN,40);
lc640_write_int(EE_MNEMO_ON,mnON);
lc640_write_int(EE_MNEMO_TIME,30);	
lc640_write_int(EE_AV_OFF_AVT,1);
//lc640_write_int(EE_APV_ON1,apvOFF);



lc640_write_int(EE_TBOXMAX,70);
lc640_write_int(EE_TBOXVENTMAX,60);
lc640_write_int(EE_TBOXREG,25);
lc640_write_int(EE_TLOADDISABLE,80);
lc640_write_int(EE_TLOADENABLE,70);
lc640_write_int(EE_TBATDISABLE,91);
lc640_write_int(EE_TBATENABLE,80);

lc640_write_int(ADR_SK_SIGN[0],0);
lc640_write_int(ADR_SK_REL_EN[0],0);
lc640_write_int(ADR_SK_LCD_EN[0],0xffff);

lc640_write_int(ADR_SK_SIGN[1],0);
lc640_write_int(ADR_SK_REL_EN[1],0);
lc640_write_int(ADR_SK_LCD_EN[1],0xffff);

lc640_write_int(ADR_SK_SIGN[2],0);
lc640_write_int(ADR_SK_REL_EN[2],0);
lc640_write_int(ADR_SK_LCD_EN[2],0xffff);

lc640_write_int(ADR_SK_SIGN[3],0);
lc640_write_int(ADR_SK_REL_EN[3],0);
lc640_write_int(ADR_SK_LCD_EN[3],0xffff);

lc640_write_int(EE_UBM_AV,10);

lc640_write_int(EE_POS_VENT,11);
}


//-----------------------------------------------
void def_ips_set(short voltage)
{
if(voltage==24)
	{
	def_set(300,voltage,voltage,22,150,240,7,0);
	}
if(voltage==48)
	{
	def_set(600,voltage,voltage,44,100,480,7,0);
	}
if(voltage==60)
	{
	def_set(750,voltage,voltage,55,100,600,7,0);
	}

if(voltage==220)
	{
	def_set(2450,2366,2315,187,100,2200,2,2346);

	lc640_write_int(EE_NUMIST,2);
	lc640_write_int(EE_NUMINV,0);
//lc640_write_int(EE_NUMDT,0);
//lc640_write_int(EE_NUMSK,0);
	lc640_write_int(EE_MAIN_IST,0);
	lc640_write_int(EE_PAR,1);
	lc640_write_int(EE_TBAT,60);
	lc640_write_int(EE_UMAX,2450);
	lc640_write_int(EE_DU,2315/2);
	//lc640_write_int(EE_UB0,2366);
	//lc640_write_int(EE_UB20,2315);
	lc640_write_int(EE_TSIGN,70);
	lc640_write_int(EE_TMAX,80);
//lc640_write_int(EE_C_BAT,180);
	lc640_write_int(EE_USIGN,187);
	lc640_write_int(EE_UMN,187);
	lc640_write_int(EE_ZV_ON,0);
	lc640_write_int(EE_IKB,20);
//lc640_write_int(EE_KVZ,1030);
	//lc640_write_int(EE_UVZ,2346);
	lc640_write_int(EE_IMAX,80);
	lc640_write_int(EE_IMIN,50);
//lc640_write_int(EE_APV_ON,apvON);
	lc640_write_int(EE_APV_ON1,apvON);
	lc640_write_int(EE_APV_ON2,apvON);
	lc640_write_int(EE_APV_ON2_TIME,1);
	lc640_write_int(EE_IZMAX,160);
	lc640_write_int(EE_U0B,2200);
	lc640_write_int(EE_TZAS,3);
	lc640_write_int(EE_TBATMAX,50);  
	lc640_write_int(EE_TBATSIGN,40);
	lc640_write_int(EE_MNEMO_ON,mnON);
	lc640_write_int(EE_MNEMO_TIME,30);	
	lc640_write_int(EE_AV_OFF_AVT,1);
//lc640_write_int(EE_APV_ON1,apvOFF);



	lc640_write_int(EE_TBOXMAX,70);
	lc640_write_int(EE_TBOXVENTMAX,60);
	lc640_write_int(EE_TBOXREG,25);
	lc640_write_int(EE_TLOADDISABLE,80);
	lc640_write_int(EE_TLOADENABLE,70);
	lc640_write_int(EE_TBATDISABLE,91);
	lc640_write_int(EE_TBATENABLE,80);

	lc640_write_int(ADR_SK_SIGN[0],0);
	lc640_write_int(ADR_SK_REL_EN[0],0);
	lc640_write_int(ADR_SK_LCD_EN[0],0xffff);

	lc640_write_int(ADR_SK_SIGN[1],0);
	lc640_write_int(ADR_SK_REL_EN[1],0);
	lc640_write_int(ADR_SK_LCD_EN[1],0xffff);

	lc640_write_int(ADR_SK_SIGN[2],0);
	lc640_write_int(ADR_SK_REL_EN[2],0);
	lc640_write_int(ADR_SK_LCD_EN[2],0xffff);

	lc640_write_int(ADR_SK_SIGN[3],0);
	lc640_write_int(ADR_SK_REL_EN[3],0);
	lc640_write_int(ADR_SK_LCD_EN[3],0xffff);

	lc640_write_int(EE_UBM_AV,10);

	lc640_write_int(EE_POS_VENT,11);


	lc640_write_int(EE_DU,2315-1870);
	lc640_write_int(EE_U_AVT,2200);
	lc640_write_int(EE_IZMAX,20);
	lc640_write_int(EE_AUSW_MAIN,22033);
	lc640_write_int(EE_PAR,1);
	lc640_write_int(EE_MNEMO_ON,mnOFF);
	}

if(voltage==110)
	{
	def_set(1350,1270,1225,99,20,1220,2,1290);

	lc640_write_int(EE_NUMIST,2);
	lc640_write_int(EE_NUMINV,0);
//lc640_write_int(EE_NUMDT,0);
//lc640_write_int(EE_NUMSK,0);
	lc640_write_int(EE_MAIN_IST,0);
	lc640_write_int(EE_PAR,1);
	lc640_write_int(EE_TBAT,60);
	lc640_write_int(EE_UMAX,1350);
	lc640_write_int(EE_DU,1350/2);
	//lc640_write_int(EE_UB0,1270);
	//lc640_write_int(EE_UB20,1225);
	lc640_write_int(EE_TSIGN,70);
	lc640_write_int(EE_TMAX,80);
//lc640_write_int(EE_C_BAT,180);
	lc640_write_int(EE_USIGN,99);
	lc640_write_int(EE_UMN,187);
	lc640_write_int(EE_ZV_ON,0);
	lc640_write_int(EE_IKB,20);
//lc640_write_int(EE_KVZ,1030);
	//lc640_write_int(EE_UVZ,1290);
	lc640_write_int(EE_IMAX,80);
	lc640_write_int(EE_IMIN,50);
//lc640_write_int(EE_APV_ON,apvON);
	lc640_write_int(EE_APV_ON1,apvON);
	lc640_write_int(EE_APV_ON2,apvON);
	lc640_write_int(EE_APV_ON2_TIME,1);
	lc640_write_int(EE_IZMAX,160);
	lc640_write_int(EE_U0B,1220);
	lc640_write_int(EE_TZAS,3);
	lc640_write_int(EE_TBATMAX,50);  
	lc640_write_int(EE_TBATSIGN,40);
	lc640_write_int(EE_MNEMO_ON,mnON);
	lc640_write_int(EE_MNEMO_TIME,30);	
	lc640_write_int(EE_AV_OFF_AVT,1);
//lc640_write_int(EE_APV_ON1,apvOFF);



	lc640_write_int(EE_TBOXMAX,70);
	lc640_write_int(EE_TBOXVENTMAX,60);
	lc640_write_int(EE_TBOXREG,25);
	lc640_write_int(EE_TLOADDISABLE,80);
	lc640_write_int(EE_TLOADENABLE,70);
	lc640_write_int(EE_TBATDISABLE,91);
	lc640_write_int(EE_TBATENABLE,80);

	lc640_write_int(ADR_SK_SIGN[0],0);
	lc640_write_int(ADR_SK_REL_EN[0],0);
	lc640_write_int(ADR_SK_LCD_EN[0],0xffff);

	lc640_write_int(ADR_SK_SIGN[1],0);
	lc640_write_int(ADR_SK_REL_EN[1],0);
	lc640_write_int(ADR_SK_LCD_EN[1],0xffff);

	lc640_write_int(ADR_SK_SIGN[2],0);
	lc640_write_int(ADR_SK_REL_EN[2],0);
	lc640_write_int(ADR_SK_LCD_EN[2],0xffff);

	lc640_write_int(ADR_SK_SIGN[3],0);
	lc640_write_int(ADR_SK_REL_EN[3],0);
	lc640_write_int(ADR_SK_LCD_EN[3],0xffff);

	lc640_write_int(EE_UBM_AV,10);

	lc640_write_int(EE_POS_VENT,11);


	lc640_write_int(EE_DU,1220-600);
	lc640_write_int(EE_U_AVT,1220);
	lc640_write_int(EE_IZMAX,20);
	lc640_write_int(EE_AUSW_MAIN,22033);
	lc640_write_int(EE_PAR,1);
	lc640_write_int(EE_MNEMO_ON,mnOFF);
	}

lc640_write_int(ADR_EE_BAT_IS_ON[0],bisOFF);
lc640_write_int(ADR_EE_BAT_DAY_OF_ON[0],LPC_RTC->DOM);
lc640_write_int(ADR_EE_BAT_MONTH_OF_ON[0],LPC_RTC->MONTH);
lc640_write_int(ADR_EE_BAT_YEAR_OF_ON[0],LPC_RTC->YEAR);
lc640_write_int(ADR_EE_BAT_C_NOM[0],0);
lc640_write_int(ADR_EE_BAT_RESURS[0],0);

lc640_write_int(ADR_EE_BAT_IS_ON[1],bisOFF);
lc640_write_int(ADR_EE_BAT_DAY_OF_ON[1],LPC_RTC->DOM);
lc640_write_int(ADR_EE_BAT_MONTH_OF_ON[1],LPC_RTC->MONTH);
lc640_write_int(ADR_EE_BAT_YEAR_OF_ON[1],LPC_RTC->YEAR);
lc640_write_int(ADR_EE_BAT_C_NOM[1],0);
lc640_write_int(ADR_EE_BAT_RESURS[1],0);


lc640_write_int(EE_SPEED_CHRG_VOLT,2400);
lc640_write_int(EE_SPEED_CHRG_CURR,20);
lc640_write_int(EE_SPEED_CHRG_TIME,1);
lc640_write_int(EE_SPEED_CHRG_AVT_EN,0);
lc640_write_int(EE_SPEED_CHRG_BLOCK_SRC,0);
lc640_write_int(EE_SPEED_CHRG_BLOCK_LOG,0);
lc640_write_int(EE_SPEED_CHRG_D_U,40);
lc640_write_int(EE_U_OUT_KONTR_MAX,1310);
lc640_write_int(EE_U_OUT_KONTR_MIN,1100);


}

//-----------------------------------------------
void can_reset_hndl(void)
{
if((lc640_read_int(EE_CAN_RESET_CNT)<0)||(lc640_read_int(EE_CAN_RESET_CNT)>2))	lc640_write_int(EE_CAN_RESET_CNT,0);

can_reset_cnt++;

if((can_reset_cnt>=10)&&(!(avar_stat&0x0001))&&(!bRESET))
	{
	if(lc640_read_int(EE_CAN_RESET_CNT)<2)
		{
		lc640_write_int(EE_CAN_RESET_CNT,lc640_read_int(EE_CAN_RESET_CNT)+1);
		bRESET=1;
		}
	}

if((main_1Hz_cnt>=3600UL)&&(lc640_read_int(EE_CAN_RESET_CNT)!=0))
	{
	lc640_write_int(EE_CAN_RESET_CNT,0);
	}

if(((LPC_CAN1->GSR)>>24)==127)bRESET=1;
if((((LPC_CAN1->GSR)>>16)&0x00ff)==127)bRESET=1;

}

//-----------------------------------------------
void net_drv(void)
{ 

max_net_slot=24;


if(++cnt_net_drv>max_net_slot) 
	{
	cnt_net_drv=-5;
	} 


if((cnt_net_drv>=0)&&(cnt_net_drv<=max_net_slot)) // � 1 �� 12 ������� ��������


	{
	//cnt_net_drv=2; 
	if(mess_find_unvol(MESS2NET_DRV))
		{
		if(mess_data[0]==PARAM_BPS_NET_OFF)
			{
			//mess_data[1]=1;
			if(sub_ind1==cnt_net_drv)
				{
				return;
				}
			}
		}
	
	//bps[cnt_net_drv]._vol_u=2400;
	//bps[cnt_net_drv]._x_=0;
	//if(cntrl_stat==1000) 		   
	if(!bCAN_OFF)
		{
//		if(cntrl_stat==1000)	can1_out(cnt_net_drv,cnt_net_drv,GETTM,bps[cnt_net_drv]._flags_tu,*((char*)(&UOUT)),*((char*)((&UOUT))+1),0xe8,0x03);
//		else 					can1_out(cnt_net_drv,cnt_net_drv,GETTM,bps[cnt_net_drv]._flags_tu,*((char*)(&UOUT)),*((char*)((&UOUT))+1),*((char*)(&bps[cnt_net_drv]._x_)),*((char*)((&bps[cnt_net_drv]._x_))+1));

		if(cntrl_stat==2000) can1_out(cnt_net_drv,cnt_net_drv,GETTM,bps[cnt_net_drv]._flags_tu,*((char*)(&UOUT)),*((char*)((&UOUT))+1),(char)(cntrl_stat),(char)((cntrl_stat)>>8)/*,*((char*)(&cntrl_stat)),*((char*)((&cntrl_stat))+1)*/);

		else can1_out(cnt_net_drv,cnt_net_drv,GETTM,bps[cnt_net_drv]._flags_tu,*((char*)(&UOUT)),*((char*)((&UOUT))+1),(char)(cntrl_stat+bps[cnt_net_drv]._x_),(char)((cntrl_stat+bps[cnt_net_drv]._x_)>>8)/*,*((char*)(&cntrl_stat)),*((char*)((&cntrl_stat))+1)*/);

     	}
	if(cnt_net_drv<=max_net_slot)
	     {
	     if(bps[cnt_net_drv]._cnt<CNT_SRC_MAX)
   	 		{    
   	 		bps[cnt_net_drv]._cnt++;
   	 		if( (bps[cnt_net_drv]._cnt>=CNT_SRC_MAX) && (!net_av) && (!(bps[cnt_net_drv]._av&0x08)) && (cnt_net_drv<NUMIST) ) 
   	 			{
   	 			avar_bps_hndl(cnt_net_drv,3,1);
   	 			}
   	 		}
		else bps[cnt_net_drv]._cnt=CNT_SRC_MAX;
						
		if((bps[cnt_net_drv]._cnt>=3)&&(bps[cnt_net_drv]._cnt_old<3))bps[cnt_net_drv]._cnt_more2++;
		bps[cnt_net_drv]._cnt_old=bps[cnt_net_drv]._cnt;
	     }
	}

else if(cnt_net_drv==-1)
	{
     if(!bCAN_OFF)can1_out(0xff,0xff,MEM_KF,*((char*)(&UMAX)),*((char*)((&UMAX))+1),*((char*)(&DU)),*((char*)((&DU))+1),0);
     } 
     
else if(cnt_net_drv==-2)
	{
	//UAVT=4567;
     if(!bCAN_OFF)can1_out(0xff,0xff,MEM_KF1,*((char*)(&UAVT)),*((char*)((&UAVT))+1),(char)(TMAX),(char)(TSIGN),(char)TZAS);
     byps._cnt++;
	} 
else if(cnt_net_drv==-3)
	{                 
	if(!bCAN_OFF) can1_out(GETTM_IBATMETER,GETTM_IBATMETER,0,0,0,0,0,0);
	ibat_metr_cnt++;
	}
	
	
//else if(cnt_net_drv==15)
	//{
     //if(!bCAN_OFF)can1_out(0xff,0xff,MEM_KF1,*((char*)(&TMAX)),*((char*)((&TMAX))+1),*((char*)(&TSIGN)),*((char*)((&TSIGN))+1),(char)TZAS);
     //}


}

#ifdef MCP2515_CAN

//-----------------------------------------------
void net_drv_mcp2515(void)
{ 
//char temp_;    



max_net_slot=MINIM_INV_ADRESS+20;
//if(NUMINV) max_net_slot=MINIM_INV_ADRESS+NUMINV;
//gran_char(&max_net_slot,0,MAX_NET_ADRESS);

if(++cnt_net_drv>max_net_slot) 
	{
	cnt_net_drv=0;
	//LPC_GPIO2->FIODIR|=(1UL<<7);
	//LPC_GPIO2->FIOPIN^=(1UL<<7);
	if(bCAN_INV)bCAN_INV=0;
	else bCAN_INV=1;

	} 



if(cnt_net_drv<=17) // � 1 �� 17 ������� ��������

	{ 
	if(mess_find_unvol(MESS2NET_DRV))
		{
		if(mess_data[0]==PARAM_BPS_NET_OFF)
			{
			//mess_data[1]=1;
			if(sub_ind1==cnt_net_drv)
				{
				return;
				}
			}
		}
	
	bps[cnt_net_drv]._vol_u=2400;
			   
	if(!bCAN_OFF)mcp2515_transmit(cnt_net_drv,cnt_net_drv,GETTM,bps[cnt_net_drv]._flags_tu,0x60/**((char*)(&bps[cnt_net_drv]._vol_u))*/,0x09/**((char*)((&bps[cnt_net_drv]._vol_u))+1)*/,*((char*)(&bps[cnt_net_drv]._vol_i)),*((char*)((&bps[cnt_net_drv]._vol_i))+1));
     
	if(cnt_net_drv<=17)
	     {
	     if(bps[cnt_net_drv]._cnt<CNT_SRC_MAX)
   	 		{    
   	 		bps[cnt_net_drv]._cnt++;
   	 		if( (bps[cnt_net_drv]._cnt>=CNT_SRC_MAX) && (!net_av) && (!(bps[cnt_net_drv]._av&0x08)) && (cnt_net_drv<NUMIST) ) 
   	 			{
   	 			avar_bps_hndl(cnt_net_drv,3,1);
   	 			}
   	 		}
		else bps[cnt_net_drv]._cnt=CNT_SRC_MAX;
						
		if((bps[cnt_net_drv]._cnt>=3)&&(bps[cnt_net_drv]._cnt_old<3))bps[cnt_net_drv]._cnt_more2++;
		bps[cnt_net_drv]._cnt_old=bps[cnt_net_drv]._cnt;
	     }
	}



/*
else if((cnt_net_drv>=MINIM_INV_ADRESS)&&(cnt_net_drv<(MINIM_INV_ADRESS+NUMINV))&&(NUMINV))
	{
    if(!bCAN_OFF) can1_out(cnt_net_drv,cnt_net_drv,GETTM,bps[cnt_net_drv]._flags_tu,*((char*)(&bps[cnt_net_drv]._vol_u)),*((char*)((&bps[cnt_net_drv]._vol_u))+1),*((char*)(&bps[cnt_net_drv]._vol_i)),*((char*)((&bps[cnt_net_drv]._vol_i))+1));

	if(bps[cnt_net_drv]._cnt<CNT_SRC_MAX)
   	 		{    
   	 		bps[cnt_net_drv]._cnt++;
   	 		if( (bps[cnt_net_drv]._cnt>=CNT_SRC_MAX) && (!net_av) && (!(bps[cnt_net_drv]._av&0x08)) && (cnt_net_drv<NUMIST) ) 
   	 			{
   	 			avar_bps_hndl(cnt_net_drv,3,1);
   	 			}
   	 		}
		else bps[cnt_net_drv]._cnt=CNT_SRC_MAX;
						
		if((bps[cnt_net_drv]._cnt>=3)&&(bps[cnt_net_drv]._cnt_old<3))bps[cnt_net_drv]._cnt_more2++;
		bps[cnt_net_drv]._cnt_old=bps[cnt_net_drv]._cnt;
	} 
*/	
	

else if(cnt_net_drv==19)
	{
     if(!bCAN_OFF)
		{
		mcp2515_transmit(cnt_net_drv,cnt_net_drv,GETTM,BAT_TYPE,NUMBAT,0,0,0);
		lakb[0]._cnt++;
		if(lakb[0]._cnt>20)lakb[0]._cnt=20;
		lakb[1]._cnt++;
		if(lakb[1]._cnt>20)lakb[1]._cnt=20;
		}
     }
	
	

}
#endif

//-----------------------------------------------
void parol_init(void)
{
parol[0]=0;
parol[1]=0;
parol[2]=0;
sub_ind=0;
}

//-----------------------------------------------
void bitmap_hndl(void)
{
short x,ii,i;
unsigned int ptr_bitmap;
static char ptr_cnt,ptr_cnt1,ptr_cnt2,ptr_cnt3,ptr_cnt4;

for(ii=0;ii<488;ii++)
	{
	lcd_bitmap[ii]=0x00;
	}


	{
	for(i=0;i<4;i++)
		{
		ptr_bitmap=122*(unsigned)i;
		for(x=(20*i);x<((20*i)+20);x++)
	 		{
			lcd_bitmap[ptr_bitmap++]=caracter[(unsigned)lcd_buffer[x]*6];
			lcd_bitmap[ptr_bitmap++]=caracter[((unsigned)lcd_buffer[x]*6)+1];
			lcd_bitmap[ptr_bitmap++]=caracter[((unsigned)lcd_buffer[x]*6)+2];
			lcd_bitmap[ptr_bitmap++]=caracter[((unsigned)lcd_buffer[x]*6)+3];
			lcd_bitmap[ptr_bitmap++]=caracter[((unsigned)lcd_buffer[x]*6)+4];
			lcd_bitmap[ptr_bitmap++]=caracter[((unsigned)lcd_buffer[x]*6)+5];
			} 
		}
	}	
}

//-----------------------------------------------
void ind_hndl(void)
{			  
//const char* ptr;
const char* ptrs[60];
const char* sub_ptrs[50];
static char sub_cnt,sub_cnt1;
char i,sub_cnt_max;
char ii_;				  
static char ii_cnt,cnt_ind_bat;


	   
sub_cnt_max=5;
i=0;
/*	      
if(spc_stat==spcVZ)
	{
	sub_ptrs[i++]=		" ������.�����  X:0x ";
	sub_cnt_max++;
	}
if(spc_stat==spcKE)
	{
	if(spc_bat==0)		sub_ptrs[i++]=		"�������� ���. ��� �1";
	else if(spc_bat==1)	sub_ptrs[i++]=		"�������� ���. ��� �2";
	sub_cnt_max++;
	}*/	
if(avar_stat&0x0001)
	{
	sub_ptrs[i++]=		"   ������ ����!!!   ";
	sub_cnt_max++;	
	}

/*
if(avar_stat&0x0002)
	{
	sub_ptrs[i++]=	" ������ ������� �1  ";
	sub_cnt_max++;	
	}

if(avar_stat&0x0004)
	{
	sub_ptrs[i++]=	" ������ ������� �2  ";
	sub_cnt_max++;	
	}*/

if(ips_bat_av_stat)
	{
	sub_ptrs[i++]=	"  ������ �������    ";
	sub_cnt_max++;	
	}
if(avar_stat&(1<<(3+0)))
	{
	sub_ptrs[i++]=	"   ������ ��� �1    ";
	sub_cnt_max++;	
	}
if(avar_stat&(1<<(3+1)))
	{
	sub_ptrs[i++]=	"   ������ ��� �2    ";
	sub_cnt_max++;	
	}
if(avar_stat&(1<<(3+2)))
	{
	sub_ptrs[i++]=	"   ������ ��� �3    ";
	sub_cnt_max++;	
	}
if(avar_stat&(1<<(3+3)))
	{
	sub_ptrs[i++]=	"   ������ ��� �4    ";
	sub_cnt_max++;	
	}
if(avar_stat&(1<<(3+4)))
	{
	sub_ptrs[i++]=	"   ������ ��� �5    ";
	sub_cnt_max++;	
	}
if(avar_stat&(1<<(3+5)))
	{
	sub_ptrs[i++]=	"   ������ ��� �6    ";
	sub_cnt_max++;	
	}
if(avar_stat&(1<<(3+6)))
	{
	sub_ptrs[i++]=	"   ������ ��� �7    ";
	sub_cnt_max++;	
	}
if(avar_stat&(1<<(3+7)))
	{
	sub_ptrs[i++]=	"   ������ ��� �8    ";
	sub_cnt_max++;	
	}
if(avar_stat&(1<<(3+8)))
	{
	sub_ptrs[i++]=	"   ������ ��� �9    ";
	sub_cnt_max++;	
	}
if(avar_stat&(1<<(3+9)))
	{
	sub_ptrs[i++]=	"   ������ ��� �10   ";
	sub_cnt_max++;	
	}
if(avar_stat&(1<<(3+10)))
	{
	sub_ptrs[i++]=	"   ������ ��� �11   ";
	sub_cnt_max++;	
	}
if(avar_stat&(1<<(3+11)))
	{
	sub_ptrs[i++]=	"   ������ ��� �12   ";
	sub_cnt_max++;	
	}

if(uout_av)
	{
	sub_ptrs[i++]=	"   ������ U���!!!   ";
	sub_cnt_max++;	
	}


if(bps[0]._av&(1<<4))
	{
	sub_ptrs[i++]=	"������ ����. ���1   ";
	sub_cnt_max++;
	sub_ptrs[i++]=	"     ��������       ";
	sub_cnt_max++;		
	}

if(overloadAvar)
	{
	sub_ptrs[i++]=	" ���������� �� ����!";
	sub_cnt_max++;	
	}

if(sysTAvar)
	{
	sub_ptrs[i++]=	" �������� �������!  ";
	sub_cnt_max++;	
	}

if(uOutAvar==1)
	{
	sub_ptrs[i++]=	"   �������� U���.!  ";
	sub_cnt_max++;	
	}

if(uOutAvar==2)
	{
	sub_ptrs[i++]=	"   �������� U���.!  ";
	sub_cnt_max++;	
	}

if(uInAvar==1)
	{
	sub_ptrs[i++]=	"   �������� U��.!   ";
	sub_cnt_max++;	
	}

if(uInAvar==2)
	{
	sub_ptrs[i++]=	"   �������� U��.!   ";
	sub_cnt_max++;	
	}

cnt_of_slave=NUMIST+NUMINV;


//cnt_of_wrks=0;
//for(i=0;i<NUMIST;i++)
 //    {
//     if(bps[i]._state==bsWRK)cnt_of_wrks++;
  //   }


sub_cnt1++;	
if(sub_cnt1>=20)
	{
	sub_cnt1=0;
	sub_cnt++;
	if(sub_cnt>=sub_cnt_max)
		{
		sub_cnt=0;
		}
	}






else if(ind==iMn_VD)
	{
	ptrs[0]	=	"                    ";

	ptrs[0]	=	"  � ������    r���. ";


	 
    i=0;
 	
	ptrs[1]=			"U��.    ]� U�.�.  @�";	
	ptrs[2]=			"U���.   #� I���.  $�";
    ptrs[3]=			" 0%:0^:0& 0</>  /0{ ";
	ptrs[4]=			"   T�������    ?�C  ";
								
	ptrs[5]=  			" ��� N1             ";
    ptrs[6]=  			" ��� N2             ";
    ptrs[7]=  			" ��� N3             ";
    ptrs[8]=  			" ��� N4             ";
    ptrs[9]= 			" ��� N5             ";
    ptrs[10]= 			" ��� N6             ";
    ptrs[11]= 			" ��� N7             ";
    ptrs[12]= 			" ��� N8             ";
    ptrs[13]= 			" ��� N9             ";
    ptrs[14]= 			" ��� N10            ";
	ptrs[15]=  			" ��� N11            ";
    ptrs[16]=  			" ��� N12            ";
    ptrs[17]=  			" ��� N13            ";
    ptrs[18]=  			" ��� N14            ";
    ptrs[19]= 			" ��� N15            ";
    ptrs[20]= 			" ��� N16            ";
    ptrs[21]= 			" ��� N17            ";
    ptrs[22]= 			" ��� N18            ";
    ptrs[23]= 			" ��� N19            ";
    ptrs[24]= 			" ��� N20            ";
 	ptrs[25]=  			" ��� N21            ";
    ptrs[26]=  			" ��� N22            ";
    ptrs[27]=  			" ��� N23            ";
    ptrs[28]=  			" ��� N24            ";
    ptrs[29]= 			" ��� N25            ";
    ptrs[30]= 			" ��� N26            ";
    ptrs[31]= 			" ��� N27            ";
    ptrs[32]= 			" ��� N28            ";
    ptrs[33]= 			" ��� N29            ";
    ptrs[34]= 			" ��� N30            ";
    ptrs[35]= 			" ��� N31            ";
    ptrs[36]= 			" ��� N32            ";
	
	ptrs[5+NUMIST]=		" ������� ���������� ";
	ptrs[6+NUMIST]= 	" ���������          "; 
    ptrs[7+NUMIST]= 	" ������ �������     ";
	ptrs[8+NUMIST]= 	" �����              ";
	ptrs[9+NUMIST]=	" ����               ";
	

    if(sub_ind==0)index_set=0;
	else if((index_set-sub_ind)>2)index_set=sub_ind+2;
	else if(sub_ind>index_set)index_set=sub_ind;
	
	if(sub_cnt<5)bgnd_par(ptrs[0],ptrs[index_set+1],ptrs[index_set+2],ptrs[index_set+3]);
	else bgnd_par(sub_ptrs[sub_cnt-5],ptrs[index_set+1],ptrs[index_set+2],ptrs[index_set+3]);
	
	if((ii_cnt<=80)||((spc_stat!=spcVZ)&&(spc_stat!=spcKE))) 
		{
	//if((ii_!=139)&&(/*(src_state[0]==ssWRK)||(src_state[1]==ssWRK)||(src_state[2]==ssWRK))*/num_of_wrks_bps!=0))
//		{
		
		if((sub_cnt<5)/*&&(num_of_wrks_bps!=0)*/)int2lcdyx(num_of_wrks_bps,0,14,0);

          //}
     	}

//	if((AUSW_MAIN==22063)||(AUSW_MAIN==22023)||(AUSW_MAIN==22043))
		//{
		//int2lcd(load_U/10,'#',0);
 		//int2lcd(load_I/10,'$',0);
	//	}
	//else 
		//{
		//int2lcd(load_U/10,'#',0);
 		//int2lcd(load_I,'$',1);
		//}

/*	if(bps_U<1000)	int2lcd(bps_U,']',1);
	else  			int2lcd(bps_U/10,']',0);
	if(bps_I<100)	int2lcd(bps_I,'@',1);
	else  			int2lcd(bps_I/10,'@',0);
	if(out_U<1000)	int2lcd(out_U,'#',1);
	else  			int2lcd(out_U/10,'#',0); */
 	
	int2lcd(LPC_RTC->HOUR,'%',0);
	int2lcd(LPC_RTC->MIN,'^',0);
	int2lcd(LPC_RTC->SEC,'&',0);
	int2lcd(LPC_RTC->DOM,'<',0);
	int2lcd(LPC_RTC->YEAR,'{',0); 
	sub_bgnd(sm_mont[LPC_RTC->MONTH],'>',0);

 	if(bFL2)
 	     {
 	     lcd_buffer[find(':')]=' ';
 	     lcd_buffer[find(':')]=' ';
 	     lcd_buffer[find(':')]=' ';
 	     }

	if((index_set)&&(sub_ind>1))
	     {
	     if(index_set==sub_ind)lcd_buffer[60]=1;
	     else if((index_set-sub_ind)==1)lcd_buffer[40]=1;
	     else if((index_set-sub_ind)==2)lcd_buffer[20]=1;
	     }	
		
	//cnt_ind_bat++;
	//if(cnt_ind_bat>=(NUMBAT*20)) cnt_ind_bat=0;
	   
	//if((AUSW_MAIN==22063)||(AUSW_MAIN==22023)||(AUSW_MAIN==22043)||(AUSW_MAIN==22011))
		//{
		//sub_bgnd("I�      @� T�    ?�C",'z',-2);
		if(bIBAT_SMKLBR)sub_bgnd("����. ",'$',-3);
		else /*if(Ib_ips_termokompensat<100)*/int2lcd_mmm(Ib_ips_termokompensat,'$',0);
		//else int2lcd_mmm(Ib_ips_termokompensat/10,'$',1);
		int2lcd_mmm(t_ext[0],'?',0);
		//}
	//if(NUMBAT==0)sub_bgnd(" ������ ��� ������� ",'z',-2);
	//else
		//{
		//int2lcd((cnt_ind_bat/20)+1+(!(BAT_IS_ON[0]==bisON)),'z',0);
		//int2lcd((cnt_ind_bat/20)+1+(!(BAT_IS_ON[0]==bisON)),'z',0);
		//if((NUMBAT==1)&&(!(BAT_IS_ON[0]==bisON)))
			//{
			//int2lcd(bat[1]._Ub/10,']',0);
			//if((mess_find_unvol(MESS2IND_HNDL))&&(mess_data[0]==PARAM_SAMOKALIBR)) sub_bgnd("����. ",'@',-4);
			//else int2lcd_mmm(bat[1]._Ib,'@',2);
			//}
		//else
			//{
			//int2lcd(bat[cnt_ind_bat/20]._Ub/10,']',0);
			//if((mess_find_unvol(MESS2IND_HNDL))&&(mess_data[0]==PARAM_SAMOKALIBR)) sub_bgnd("����. ",'@',-4);
		///	else int2lcd_mmm(bat[cnt_ind_bat/20]._Ib,'@',2);
		//	}		}

	int2lcd(out_U/10,'#',0);
	int2lcd(sys_T,'#',0);
	int2lcd(vd_U/10,'@',0);
	int2lcd(in_U/10,']',0);
	//int2lcd(vz_cnt_s_/60,'x',0);
	//int2lcd(vz_cnt_h_,'X',0); 

	//int2lcdyx(UOUT,0,3,0);

	//int2lcdyx(UOUT_,0,8,0);

	//int2lcdyx(cnt_net_drv,0,19,0);
	//int2lcdyx((short)ibat_metr_buff_[0],0,4,0);
	//int2lcdyx((short)ibat_metr_buff_[1],0,9,0);
	//int2lcdyx((short)ibat_metr_buff_[2],0,19,0);
//	int2lcdyx(bat[0]._sign_temper_cnt,0,19,0);
	//int2lcdyx(uAvarHndlOutUMaxCnt,0,16,0);
//		int2lcdyx(UOUT_,0,9,0);
	}

#ifndef _DEBUG_

else if(ind==iMakb)
	{
	const char* ptr[12];
 
	simax=10;

	ptr[0]=			" U�1    =     @�    ";
	ptr[1]=			" U�2    =     #�    ";
	ptr[2]=			" U�3    =     $�    ";
	ptr[3]=			" U�4    =     %�    ";
	ptr[4]=			" U�5    =     ^�    ";
	ptr[5]=			" t�1    =     &��   ";
	ptr[6]=			" t�2    =     *��   ";
	ptr[7]=			" t�3    =     (��   ";
	ptr[8]=			" t�4    =     )��   ";
	ptr[9]=			" t�5    =     +��   ";
	ptr[10]=			sm_exit;

 	if(makb[sub_ind1]._cnt>=5)
	 	{
		bgnd_par(		"   ������� ��� N<   ",
					"   �� ���������!!!  ",
					"                    ",
					"                    ");
		}


	else 
		{
		bgnd_par(		"   ������� ��� N<   ",
					ptr[index_set],
					ptr[index_set+1],
					ptr[index_set+2]);

		if(sub_ind-index_set>2)index_set=sub_ind-2;
		else if (sub_ind<index_set)index_set=sub_ind;

		if(sub_ind>=simax)	pointer_set(1);
		
		int2lcd(makb[sub_ind1]._Ub[0],'@',1);
		int2lcd(makb[sub_ind1]._Ub[1],'#',1);
		int2lcd(makb[sub_ind1]._Ub[2],'$',1);
		int2lcd(makb[sub_ind1]._Ub[3],'%',1);
		int2lcd(makb[sub_ind1]._Ub[4],'^',1);
		if(makb[sub_ind1]._T_nd[0])sub_bgnd("�����������",'&',-5);
		else int2lcd_mmm(makb[sub_ind1]._T[0],'&',0); 
 		if(makb[sub_ind1]._T_nd[1])sub_bgnd("�����������",'*',-5);
		else int2lcd_mmm(makb[sub_ind1]._T[1],'*',0); 
		if(makb[sub_ind1]._T_nd[2])sub_bgnd("�����������",'(',-5);
		else int2lcd_mmm(makb[sub_ind1]._T[2],'(',0); 
		if(makb[sub_ind1]._T_nd[3])sub_bgnd("�����������",')',-5);
		else int2lcd_mmm(makb[sub_ind1]._T[3],')',0); 
		if(makb[sub_ind1]._T_nd[4])sub_bgnd("�����������",'+',-5);
		else int2lcd_mmm(makb[sub_ind1]._T[4],'+',0); 
		}
	int2lcd(sub_ind1+1,'<',0);
    }

 else if(ind==iBps)
	{
	const char* ptr[8];
 
	simax=5;

	ptr[1]=			" U��� =        (�   ";
	ptr[2]=			" I��� =        [A   ";
	ptr[3]=			" t��� =        ]��  ";
	ptr[4]=			" ����� ������       ";
	ptr[5]=			sm_exit;

	if(bps[sub_ind1]._state==bsWRK)
		{
		ptr[0]=		"      � ������      ";
		if((bps[sub_ind1]._flags_tm&0x08)&&(bFL2))
		ptr[0]=		"  ������� ������!!! ";	      
		}
 	 else if(bps[sub_ind1]._state==bsRDY)
	 	{
		ptr[0]=		"      � �������     ";	
		}

 	 else if(bps[sub_ind1]._state==bsBL)
	 	{
		ptr[0]=		" ������������ ����� ";	
		}

	 else if(bps[sub_ind1]._state==bsAPV)
	 	{
		ptr[0]=		"    �������� ���    ";
		}
	 
	 else if(bps[sub_ind1]._state==bsAV)
	 	{
		if(bps[sub_ind1]._av&(1<<0))
		ptr[0]=		" ������ - ��������! ";
		else if(bps[sub_ind1]._av&(1<<1))
		ptr[0]=		"������ - �����.U���!";
		else if(bps[sub_ind1]._av&(1<<2))	 
		ptr[0]=		"������ - �����.U���!";
		else if(bps[sub_ind1]._av&(1<<3))
			{
			ptr[0]=	"  ������ - �������� ";
			ptr[1]=	"      �����!!!      ";
			ptr[2]=	"                    ";
			simax=0;
			}
		}

	 else if(bps[sub_ind1]._state==bsOFF_AV_NET)
	 	{
		ptr[0]=		"      ��������      ";
		ptr[1]=		"     �����������    ";
		ptr[2]=		" ��������� �������! ";
		simax=0;
		}

	bgnd_par(			"       ��� N&       ",
					ptr[index_set],
					ptr[index_set+1],
					ptr[index_set+2]);

	if(sub_ind-index_set>2)index_set=sub_ind-2;
	else if (sub_ind<index_set)index_set=sub_ind;

	if(sub_ind>=4)	pointer_set(1);


		

	int2lcd(sub_ind1+1,'&',0);
	int2lcd(bps[sub_ind1]._Uii,'(',1);
     int2lcd(bps[sub_ind1]._Ii,'[',1);  
   	int2lcd_mmm(bps[sub_ind1]._Ti,']',0); 
   			 
    // char2lcdhxy(bps[sub_ind1]._state,0x32);
    
    //	int2lcdyx(sub_ind,0,2,0);
//	int2lcdyx(index_set,0,4,0);
	//int2lcdyx(u_necc,0,4,0);
	//int2lcdyx(cntrl_stat,0,8,0);	
     }
 else if(ind==iBps_elteh)
	{
	const char* ptr[11];
 
	simax=9;

	ptr[1]=			" U���.=        (�   ";
	ptr[2]=			" U���.=        )�   ";
	ptr[3]=			" I���.=        [A   ";
	ptr[4]=			" t���.=        ]��  ";
	ptr[5]=			" ���������          ";
	ptr[6]=			" �����������      >�";
	ptr[7]=			" ����� ������       ";
	ptr[8]=			sm_exit;
 	ptr[9]=			" ����� ���������    ";

	if(bps[sub_ind1]._state==bsWRK)
		{
		ptr[0]=		"      � ������      ";
		if((bps[sub_ind1]._flags_tm&0x08)&&(bFL2))
		ptr[0]=		"  ������� ������!!! ";	      
		}
 	 else if(bps[sub_ind1]._state==bsRDY)
	 	{
		ptr[0]=		"      � �������     ";	
		}

 	 else if(bps[sub_ind1]._state==bsBL)
	 	{
		ptr[0]=		" ������������ ����� ";	
		}

	 else if(bps[sub_ind1]._state==bsAPV)
	 	{
		ptr[0]=		"    �������� ���    ";
		}
	 
	 else if(bps[sub_ind1]._state==bsAV)
	 	{
		if(bps[sub_ind1]._av&(1<<0))
		ptr[0]=		" ������ - ��������! ";
		else if(bps[sub_ind1]._av&(1<<1))
		ptr[0]=		"������ - �����.U���!";
		else if(bps[sub_ind1]._av&(1<<2))	 
		ptr[0]=		"������ - �����.U���!";
		else if(bps[sub_ind1]._av&(1<<3))
			{
			ptr[0]=	"  ������ - �������� ";
			ptr[1]=	"      �����!!!      ";
			ptr[2]=	"                    ";
			simax=0;
			}
		}

	 else if(bps[sub_ind1]._state==bsOFF_AV_NET)
	 	{
		ptr[0]=		"      ��������      ";
		ptr[1]=		"     �����������    ";
		ptr[2]=		" ��������� �������! ";
		simax=0;
		}

	bgnd_par(			" \       ��� N&       ",
					ptr[index_set],
					ptr[index_set+1],
					ptr[index_set+2]);

	if(sub_ind-index_set>2)index_set=sub_ind-2;
	else if (sub_ind<index_set)index_set=sub_ind;

	if(sub_ind>=7)	pointer_set(1);


		

	int2lcd(sub_ind1+1,'&',0);
	int2lcd(bps[sub_ind1]._Uii,'(',1);
	int2lcd(bps[sub_ind1]._Uisum,')',1);
    int2lcd(bps[sub_ind1]._Ii,'[',1);  
   	int2lcd_mmm(bps[sub_ind1]._Ti,']',0); 
	int2lcd(bps[sub_ind1]._vent_resurs,'>',0);
   			 
    // char2lcdhxy(bps[sub_ind1].Uin,0x04);
    
   	//int2lcdyx(sub_ind,0,2,0);
	//int2lcdyx(bps[sub_ind1]._av,0,4,0);
	//int2lcdyx(u_necc,0,4,0);
	//int2lcdyx(avg,0,19,0);
	
	//int2lcdyx(bps[sub_ind1]._flags_tu,1,19,0);	
	//int2lcdyx(bps[sub_ind1]._Uin,1,19,0);
	//int2lcdyx(bps[sub_ind1].debug_info_to_uku0,2,19,0);
	//int2lcdyx(bps[sub_ind1].debug_info_to_uku1,3,19,0);
    }  	   
	 	  
else if(ind==iNet)
	{
	bgnd_par(		"        ����        ",
				" U   =     [�       ",
				" f   =     ]��      ",
				sm_exit);
	lcd_buffer[60]=1;
	     	
     int2lcd(net_U,'[',0);
     int2lcd(net_F,']',1);

 //temp_SL=(signed long)net_buff_;
//temp_SL*=Kunet;    
//int2lcdyx(net_buff_,0,4,0);
//int2lcdyx(Kunet,0,9,0);
                  	      	   	    		
     }

else if(ind==iNet3)
	{


	ptrs[0]=  		" U�A           !�   ";
    ptrs[1]=  		" U�B           @�   ";
    ptrs[2]=  	    " U�C           #�   ";
	ptrs[3]=  	    " f   =     ]��      ";           
	ptrs[4]=  		" �����              ";


	bgnd_par(		"        ����        ",
					ptrs[index_set],
					ptrs[index_set+1],
					ptrs[index_set+2]);

	if(sub_ind-index_set>2)index_set=sub_ind-2;
	else if (sub_ind<index_set)index_set=sub_ind;
	pointer_set(1);

    int2lcd(net_Ua,'!',0);
	int2lcd(net_Ub,'@',0);
	int2lcd(net_Uc,'#',0);
	#ifdef UKU_220_IPS_TERMOKOMPENSAT
	int2lcd(net_F3,']',1);
	#else
    int2lcd(net_F,']',1);
    #endif

 //temp_SL=(signed long)net_buff_;
//temp_SL*=Kunet;    
//int2lcdyx(net_buff_,0,4,0);
//int2lcdyx(Kunet,0,9,0);
                  	      	   	    		
     }

else if(ind==iNetEM)
	{


	ptrs[0]=  		" U             [�   ";
    	ptrs[1]=  		" f             ]��  ";
    	ptrs[2]=  	    	" P�����.       #��  ";
	ptrs[3]=  	    	" P����.        $����";           
	ptrs[4]=  		" �����              ";


	bgnd_par(		"        ����        ",
					ptrs[index_set],
					ptrs[index_set+1],
					ptrs[index_set+2]);

	if(sub_ind-index_set>2)index_set=sub_ind-2;
	else if (sub_ind<index_set)index_set=sub_ind;
	pointer_set(1);

     int2lcd(net_U,'[',0);
     int2lcd(net_F,']',1);
     long2lcd_mmm(power_summary/10,'$',1);
     int2lcd(power_current,'#',0);

 //temp_SL=(signed long)net_buff_;
//temp_SL*=Kunet;    
//int2lcdyx(net_buff_,0,4,0);


                  	      	   	    		
     }


else if(ind==iLoad)
	{
	bgnd_par(		"      ��������      ",
				" U���� =     [�     ",
				" I���� =     ]�     ",
				sm_exit);
	lcd_buffer[60]=1;
	     	
     int2lcd(load_U,'[',1);
     int2lcd(load_I,']',1);

     
                   	      	   	    		
     }

else if(ind==iVent)
	{

	ptrs[0]=  		" F����.�����.     !%";
     ptrs[1]=  		" F����.max. (  @%) #";
	ptrs[2]=  	     " �����              ";

	bgnd_par(			"     ����������     ",
					ptrs[index_set],
					ptrs[index_set+1],
					ptrs[index_set+2]);

	pointer_set(1);

     int2lcd(main_vent_pos*5,'!',0);
	int2lcd(pos_vent,'#',0);
	int2lcd(pos_vent*5+45,'@',0);     
	}

else if(ind==iAvt)
	{
     ptrs[0]=  		"  �������� �������� ";
	ptrs[1]=  		" ������� �1       ! ";
	ptrs[2]=  		" ������� �2       @ ";
	ptrs[3]=  		" ������� �3       # ";
	ptrs[4]=  		" ������� �4       $ ";
	ptrs[5]=  		" ������� �5       % ";
	ptrs[6]=  		" ������� �6       ^ ";
	ptrs[7]=  		" ������� �7       & ";
	ptrs[8]=  		" ������� �8       * ";
	ptrs[9]=  		" ������� �9       ( ";
	ptrs[10]=  		" ������� �10      ) ";
	ptrs[11]=  		" ������� �11      + ";
	ptrs[12]=  		" ������� �12      = ";

	ptrs[1+NUMAVT]=  	" �����              ";
	ptrs[2+NUMAVT]=  	"                    ";
	ptrs[3+NUMAVT]=  	"                    ";

	bgnd_par(		ptrs[0],
				ptrs[index_set+1],
				ptrs[index_set+2],
				ptrs[index_set+3]);

	if(sub_ind-index_set>2)index_set=sub_ind-2;
	else if (sub_ind<index_set)index_set=sub_ind;
	pointer_set(1);

	//int2lcdyx(eb2_data_short[6],0,6,0);

	if(avt_stat[0]==avtON)	sub_bgnd("���.",'!',-3);
	else 				sub_bgnd("����.",'!',-4);
	if(avt_stat[1]==avtON)	sub_bgnd("���.",'@',-3);
	else 				sub_bgnd("����.",'@',-4);
	if(avt_stat[2]==avtON)	sub_bgnd("���.",'#',-3);
	else 				sub_bgnd("����.",'#',-4);
	if(avt_stat[3]==avtON)	sub_bgnd("���.",'$',-3);
	else 				sub_bgnd("����.",'$',-4);
	if(avt_stat[4]==avtON)	sub_bgnd("���.",'%',-3);
	else 				sub_bgnd("����.",'%',-4);
	if(avt_stat[5]==avtON)	sub_bgnd("���.",'^',-3);
	else 				sub_bgnd("����.",'^',-4);
	if(avt_stat[6]==avtON)	sub_bgnd("���.",'&',-3);
	else 				sub_bgnd("����.",'&',-4);
	if(avt_stat[7]==avtON)	sub_bgnd("���.",'*',-3);
	else 				sub_bgnd("����.",'*',-4);
	if(avt_stat[8]==avtON)	sub_bgnd("���.",'(',-3);
	else 				sub_bgnd("����.",'(',-4);
	if(avt_stat[9]==avtON)	sub_bgnd("���.",')',-3);
	else 				sub_bgnd("����.",')',-4);
	if(avt_stat[10]==avtON)	sub_bgnd("���.",'+',-3);
	else 				sub_bgnd("����.",'+',-4); 
	if(avt_stat[11]==avtON)	sub_bgnd("���.",'=',-3);
	else 				sub_bgnd("����.",'=',-4);
     //int2lcd(Uvv[1],'$',0);
     //int2lcd(Uvv[2],'$',0);

     //long2lcd_mmm(power_summary,'%',2);
     //int2lcd(power_current,'^',0);

     //int2lcdyx(adc_buff_ext_[0],0,4,0);
     //int2lcdyx(adc_buff_ext_[1],0,10,0);
     //int2lcdyx(adc_buff_ext_[2],0,16,0);
     }

else if(ind==iEnerg)
	{
     ptrs[0]=  		"  ����������������  ";

     ptrs[1]=  		" ����       #�      ";
     ptrs[2]=  	     " ���        $�      ";            
     ptrs[3]=  	     " P����.       %���*�";
	ptrs[4]=  	     " P�����.      ^��   ";
	ptrs[5]=  	     " �����              ";
	ptrs[6]=  	     "                    ";
	ptrs[7]=  	     "                    ";

	bgnd_par(		ptrs[0],
				ptrs[index_set+1],
				ptrs[index_set+2],
				ptrs[index_set+3]);

	if(sub_ind-index_set>2)index_set=sub_ind-2;
	else if (sub_ind<index_set)index_set=sub_ind;
	pointer_set(1);

	int2lcd(Uvv0,'#',0);
     int2lcd(Uvv[1],'$',0);
     //int2lcd(Uvv[2],'$',0);

     long2lcd_mmm(power_summary,'%',2);
     int2lcd(power_current,'^',0);

     //int2lcdyx(adc_buff_ext_[0],0,4,0);
     //int2lcdyx(adc_buff_ext_[1],0,10,0);
     //int2lcdyx(adc_buff_ext_[2],0,16,0);
     }

else if(ind==iEnerg3)
	{
     ptrs[0]=  		"  ����������������  ";

     ptrs[1]=  		" ���� �.A    !�     ";
	ptrs[2]=  		" ���� �.B    @�     ";
	ptrs[3]=  		" ���� �.C    #�     ";
     ptrs[4]=  	     " ���  �.A    &�     ";
     ptrs[5]=  	     " ���  �.B    )�     ";
     ptrs[6]=  	     " ���  �.C    (�     ";		            
     ptrs[7]=  	     " P����.       %���*�";
	ptrs[8]=  	     " P�����.      ^��   ";
	ptrs[9]=  	     " �����              ";
	ptrs[10]=  	     "                    ";
	ptrs[11]=  	     "                    ";

	bgnd_par(		ptrs[0],
				ptrs[index_set+1],
				ptrs[index_set+2],
				ptrs[index_set+3]);

	if(sub_ind-index_set>2)index_set=sub_ind-2;
	else if (sub_ind<index_set)index_set=sub_ind;
	pointer_set(1);

	int2lcd(Uvv_eb2[0],'!',0);
	int2lcd(Uvv_eb2[1],'@',0);
	int2lcd(Uvv_eb2[2],'#',0);
	int2lcd(Upes_eb2[0],'&',0);
	int2lcd(Upes_eb2[1],')',0);
	int2lcd(Upes_eb2[2],'(',0);
     long2lcd_mmm(power_summary,'%',3);
     int2lcd(power_current,'^',0);

     }

else if(ind==iSpc)
	{

 	ptrs[0]=	" ���.�����          ";
 	ptrs[1]=	" ���.���.�����      ";
 	ptrs[2]=	" �.�. ������� N1    ";
 	ptrs[3]=	" �.�. ������� N2    ";
 //	ptrs[4]=	" �.�.�.  ���. N1    ";
 //	ptrs[5]=	" �.�.�.  ���. N2    ";
 	ptrs[4]=	" �����              ";
	
	if(sub_ind<index_set) index_set=sub_ind;
	else if((sub_ind-index_set)>2) index_set=sub_ind-2;
    	bgnd_par( "     �����������    ",
    	          ptrs[index_set],
    	          ptrs[index_set+1],
    	          ptrs[index_set+2]);
	pointer_set(1);
	}    		

else if(ind==iSpc_termocompensat)
	{
 	ptrs[0]=	" ���.�����          ";
 	ptrs[1]=	" ���.���.�����      ";
 	ptrs[2]=	" �.�. �������       ";
 	ptrs[3]=	" �����              ";
	
	if(sub_ind<index_set) index_set=sub_ind;
	else if((sub_ind-index_set)>2) index_set=sub_ind-2;
    	bgnd_par( "     �����������    ",
    	          ptrs[index_set],
    	          ptrs[index_set+1],
    	          ptrs[index_set+2]);
	pointer_set(1);
	}    		

else if(ind==iLog)
	{
	//char dt[4],dt_[4],dt__[4];
//	char iii;

	av_j_si_max=lc640_read_int(CNT_EVENT_LOG);
	if(av_j_si_max>64)av_j_si_max=0;

	if(av_j_si_max==0)
		{
		bgnd_par("   ������ �������   "," ������ ����        ",sm_exit,sm_);
		//lcd_buffer[33]=1;
		sub_ind=1;
		index_set=0;
		}       
		
	else if(av_j_si_max==1)
		{
		bgnd_par("   ������ �������   "," (                  ",sm_exit," �������� ������    ");
		//if(sub_ind==0)lcd_buffer[16]=1;
		//else if(sub_ind==1)lcd_buffer[33]=1;
		//else if(sub_ind==2)lcd_buffer[50]=1;		
		index_set=0;
		}

	else if(av_j_si_max==2)
		{
		if(sub_ind<index_set) index_set=sub_ind;
		else if((sub_ind-index_set)>2) index_set=sub_ind-2;		
		if(index_set==0) bgnd_par("   ������ �������   "," (                  "," [                  ",sm_exit);
		else if(index_set==1) bgnd_par("   ������ �������   "," (                  ",sm_exit," �������� ������    ");
		
		//if((sub_ind-index_set)==0) lcd_buffer[16]=1; 
		//else if((sub_ind-index_set)==1) lcd_buffer[33]=1;
		//else if((sub_ind-index_set)==2) lcd_buffer[50]=1;
		}
		
	else if(av_j_si_max>2)
		{
		if(sub_ind<index_set) index_set=sub_ind;
		else if((sub_ind-index_set)>2) index_set=sub_ind-2;  
		if(index_set==(av_j_si_max-1)) bgnd_par("   ������ �������   "," (                  ",sm_exit," �������� ������    ");
		else if(index_set==(av_j_si_max-2)) bgnd_par("   ������ �������   "," (                  "," [                  ",sm_exit);
		else bgnd_par("   ������ �������   "," (                  "," [                  "," {                  ");
		
		//if((sub_ind-index_set)==0) lcd_buffer[16]=1; 
		//else if((sub_ind-index_set)==1) lcd_buffer[33]=1;
		//else if((sub_ind-index_set)==2) lcd_buffer[50]=1;

		}
	pointer_set(1);
     event2ind(index_set,'(');
     event2ind(index_set+1,'[');	
     event2ind(index_set+2,'{');	  
    
	}



else if(ind==iLog_)
	{	
	unsigned short tempUI/*,tempUI_*/;
//	unsigned long tempUL;
	char av_head[4],av_data_on[8],av_data_off[8],av_data[4];
	short av_head_int[2];
	
	bgnd_par(sm_,sm_,sm_,sm_);
	tempUI=lc640_read_int(PTR_EVENT_LOG);
	tempUI=ptr_carry(tempUI,64,-1*((signed)sub_ind1));
	tempUI*=32;
	tempUI+=EVENT_LOG;
     
     lc640_read_long_ptr(tempUI,av_head);
     lc640_read_long_ptr(tempUI+4,(char*)av_head_int);
     lc640_read_long_ptr(tempUI+8,av_data_on);
     lc640_read_long_ptr(tempUI+12,&(av_data_on[4])); 
     lc640_read_long_ptr(tempUI+16,av_data_off);
     lc640_read_long_ptr(tempUI+20,&(av_data_off[4]));      
	lc640_read_long_ptr(tempUI+24,av_data);
	
	//av_head_int[0]=123;  
//av_head_int[1]=456;	

	if((av_head[0]=='U')&&(av_head[2]=='R'))
		{
		if(index_set==0) {
		
		bgnd_par(	"    ������������    ",
				"   ��� ���������    ",
				"       �������      ",
				"  0%(  0^ 0@:0#:0$  ");
		} else if(index_set==1) {

		bgnd_par(	"    ������������    ",
				"   ��� ���������    ",
				"       �������      ",
				"  ��� ���������  [  ");		
		
		} else if(index_set==2) {

		bgnd_par(	"    ������������    ",
				"   ��� ���������    ",
				"       �������      ",
				"                 ]  ");		
		
		}					
				  	
		int2lcd(av_data_on[4],'@',0);
		int2lcd(av_data_on[5],'#',0);
		int2lcd(av_data_on[6],'$',0);
		int2lcd(av_data_on[2],'%',0);
		int2lcd(av_data_on[0],'^',0); 
		int2lcd(av_data_on[7],'[',0);
		if(!((av_data_on[1]>=1)&&(av_data_on[1]<=12)))av_data_on[1]=1;
		sub_bgnd(sm_mont[av_data_on[1]],'(',0);
		//int2lcd(av_data_on[1],'(',0);
		//int2lcdyx(av_data_on[1],2,1,0);
		if(av_data_on[7]&0x08)sub_bgnd("superwiser  ",']',-12);
		else if(av_data_on[7]&0x04)sub_bgnd("watchdog      ",']',-12);
		else if(av_data_on[7]&0x02)sub_bgnd("ext.reset    ",']',-12);
		else if(av_data_on[7]&0x01)sub_bgnd("power on     ",']',-12);
		av_j_si_max=2;

		
		}

	else if((av_head[0]=='P')&&(av_head[2]=='A'))
		{  
		ptrs[0]="   ������ ����!!!   ";
		ptrs[1]="  0%(  0^ 0@:0#:0$  ";
		if((av_data_off[0]=='A')&&(av_data_off[1]=='A'))
			{
			ptrs[2]="    �� ���������    ";
			ptrs[3]="     U����=  +�     ";
			bgnd_par(ptrs[0],ptrs[1],ptrs[2],ptrs[3]);
			int2lcd(net_U,'+',0);
			}
		else 
			{
			gran_char(&index_set,0,1);
			ptrs[2]="      ���������     ";
			ptrs[3]="  0[]  0< 0>:0=:0)  ";
			ptrs[4]="     U���=  +�      ";
			bgnd_par(ptrs[index_set],ptrs[1+index_set],ptrs[2+index_set],ptrs[3+index_set]);
			int2lcd(av_data_off[4],'>',0);
			int2lcd(av_data_off[5],'=',0);
			int2lcd(av_data_off[6],')',0);
			int2lcd(av_data_off[2],'[',0);
			int2lcd(av_data_off[0],'<',0); 
			if(!((av_data_off[1]>=1)&&(av_data_off[1]<=12)))av_data_off[1]=1;
			sub_bgnd(sm_mont[av_data_off[1]],']',0);
			
			int2lcd(av_data[0]+(av_data[1]*256),'+',0);			
			}	
		
		int2lcd(av_data_on[4],'@',0);
		int2lcd(av_data_on[5],'#',0);
		int2lcd(av_data_on[6],'$',0);
		int2lcd(av_data_on[2],'%',0);
		int2lcd(av_data_on[0],'^',0); 
		if(!((av_data_on[1]>=1)&&(av_data_on[1]<=12)))av_data_on[1]=1;
		sub_bgnd(sm_mont[av_data_on[1]],'(',0);
		
		av_j_si_max=1;
		}

	else if((av_head[0]=='B')&&(av_head[2]=='C'))
		{  
		ptrs[0]="       ������       ";
		ptrs[1]="     ������� N+     ";
		ptrs[2]="  0%(  0^ 0@:0#:0$  ";
		if((av_data_off[0]=='A')&&(av_data_off[1]=='A'))
			{
			ptrs[3]="    �� ���������    ";
			bgnd_par(ptrs[0],ptrs[1],ptrs[2],ptrs[3]);
			}
		else 
			{
			gran_char(&index_set,0,1);
			ptrs[3]="      ���������     ";
			ptrs[4]="  0[]  0< 0>:0=:0)  ";
			bgnd_par(ptrs[index_set],ptrs[1+index_set],ptrs[2+index_set],ptrs[3+index_set]);
			int2lcd(av_data_off[4],'>',0);
			int2lcd(av_data_off[5],'=',0);
			int2lcd(av_data_off[6],')',0);
			int2lcd(av_data_off[2],'[',0);
			int2lcd(av_data_off[0],'<',0); 
			if(!((av_data_off[1]>=1)&&(av_data_off[1]<=12)))av_data_off[1]=1;
			sub_bgnd(sm_mont[av_data_off[1]],']',0);
			
						
			}	
		int2lcd(av_head[1]+1,'+',0);
		int2lcd(av_data_on[4],'@',0);
		int2lcd(av_data_on[5],'#',0);
		int2lcd(av_data_on[6],'$',0);
		int2lcd(av_data_on[2],'%',0);
		int2lcd(av_data_on[0],'^',0); 
		if(!((av_data_on[1]>=1)&&(av_data_on[1]<=12)))av_data_on[1]=1;
		sub_bgnd(sm_mont[av_data_on[1]],'(',0);
		
		av_j_si_max=1;
		}

	else if((av_head[0]=='B')&&(av_head[2]=='S'))
		{  
		ptrs[0]="       ������       ";
		ptrs[1]="    �����������     ";
		ptrs[2]="     ������� N+     ";
		ptrs[3]="  0%(  0^ 0@:0#:0$  ";
		bgnd_par(ptrs[0],ptrs[1],ptrs[2],ptrs[3]);
		int2lcd(av_head[1]+1,'+',0);
		int2lcd(av_data_on[4],'@',0);
		int2lcd(av_data_on[5],'#',0);
		int2lcd(av_data_on[6],'$',0);
		int2lcd(av_data_on[2],'%',0);
		int2lcd(av_data_on[0],'^',0); 
		if(!((av_data_on[1]>=1)&&(av_data_on[1]<=12)))av_data_on[1]=1;
		sub_bgnd(sm_mont[av_data_on[1]],'(',0);
		
		av_j_si_max=0;
		}

	else if((av_head[0]=='B')&&(av_head[2]=='Z'))
		{  
		ptrs[0]="   �������������    ";
		ptrs[1]="       �����        ";
		ptrs[2]="  0%(  0^ 0@:0#:0$  ";
		if((av_data_off[0]=='A')&&(av_data_off[1]=='A'))
			{
			ptrs[3]="    �� ��������     ";
			bgnd_par(ptrs[0],ptrs[1],ptrs[2],ptrs[3]);
			}
		else 
			{
			gran_char(&index_set,0,1);
			ptrs[3]="      ��������      ";
			ptrs[4]="  0[]  0< 0>:0=:0)  ";
			bgnd_par(ptrs[index_set],ptrs[1+index_set],ptrs[2+index_set],ptrs[3+index_set]);
			int2lcd(av_data_off[4],'>',0);
			int2lcd(av_data_off[5],'=',0);
			int2lcd(av_data_off[6],')',0);
			int2lcd(av_data_off[2],'[',0);
			int2lcd(av_data_off[0],'<',0); 
			if(!((av_data_off[1]>=1)&&(av_data_off[1]<=12)))av_data_off[1]=1;
			sub_bgnd(sm_mont[av_data_off[1]],']',0);
			
						
			}	
		int2lcd(av_head[1]+1,'+',0);
		int2lcd(av_data_on[4],'@',0);
		int2lcd(av_data_on[5],'#',0);
		int2lcd(av_data_on[6],'$',0);
		int2lcd(av_data_on[2],'%',0);
		int2lcd(av_data_on[0],'^',0); 
		if(!((av_data_on[1]>=1)&&(av_data_on[1]<=12)))av_data_on[1]=1;
		sub_bgnd(sm_mont[av_data_on[1]],'(',0);
		
		av_j_si_max=1;
		}



	else if((av_head[0]=='B')&&(av_head[2]=='W'))
		{  
		ptrs[0]="       ������       ";
		ptrs[1]="     ������� N!     ";
		ptrs[2]="   ������           ";
		ptrs[3]="  0%(  0^ 0@:0#:0$  ";
		ptrs[4]="       U���=  <�";
		ptrs[5]="   �����            ";
		ptrs[6]="  0qw  0r 0i:0l:0s  ";
		ptrs[7]="         U���=  >�  ";
		ptrs[8]="   ������    /�*�.  ";
		
		bgnd_par(ptrs[index_set],ptrs[1+index_set],ptrs[2+index_set],ptrs[3+index_set]);
		
		int2lcd(av_head[1]+1,'!',0);

		int2lcd(av_data_on[4],'@',0);
		int2lcd(av_data_on[5],'#',0);
		int2lcd(av_data_on[6],'$',0);
		int2lcd(av_data_on[2],'%',0);
		int2lcd(av_data_on[0],'^',0); 
		if(!((av_data_on[1]>=1)&&(av_data_on[1]<=12)))av_data_on[1]=1;
		sub_bgnd(sm_mont[av_data_on[1]],'(',0);
				
		int2lcd(av_data_off[4],'i',0);
		int2lcd(av_data_off[5],'l',0);
		int2lcd(av_data_off[6],'s',0);
		int2lcd(av_data_off[2],'q',0);
		int2lcd(av_data_off[0],'r',0); 
		if(!((av_data_off[1]>=1)&&(av_data_off[1]<=12)))av_data_off[1]=1;
		sub_bgnd(sm_mont[av_data_off[1]],'w',0);
		
		
		int2lcd(av_head_int[0]/10,'/',1);
		int2lcd(av_data_on[3]+(av_data_on[7]*256),'<',1);
		int2lcd(av_head_int[1],'>',1);	
		av_j_si_max=5;				

		
		}

	else if((av_head[0]=='B')&&(av_head[2]=='K'))
		{  
		ptrs[0]="  �������� �������  ";
		ptrs[1]="       �������      ";
		ptrs[2]="   ������           ";
		ptrs[3]="  0%(  0^ 0@:0#:0$  ";
		ptrs[4]="         U���=  <�  ";
		ptrs[5]="   �����            ";
		ptrs[6]="  0qw  0r 0i:0l:0s  ";
		ptrs[7]="         U���=  >�  ";
		ptrs[8]="   �������   /�*�.  ";
		
		bgnd_par(ptrs[index_set],ptrs[1+index_set],ptrs[2+index_set],ptrs[3+index_set]);
		
		int2lcd(av_head[1]+1,'!',0);

		int2lcd(av_data_on[4],'@',0);
		int2lcd(av_data_on[5],'#',0);
		int2lcd(av_data_on[6],'$',0);
		int2lcd(av_data_on[2],'%',0);
		int2lcd(av_data_on[0],'^',0); 
		if(!((av_data_on[1]>=1)&&(av_data_on[1]<=12)))av_data_on[1]=1;
		sub_bgnd(sm_mont[av_data_on[1]],'(',0);
				
		int2lcd(av_data_off[4],'i',0);
		int2lcd(av_data_off[5],'l',0);
		int2lcd(av_data_off[6],'s',0);
		int2lcd(av_data_off[2],'q',0);
		int2lcd(av_data_off[0],'r',0); 
		if(!((av_data_off[1]>=1)&&(av_data_off[1]<=12)))av_data_off[1]=1;
		sub_bgnd(sm_mont[av_data_off[1]],'w',0);
		
		
		int2lcd(av_head_int[0],'/',1);
		int2lcd(av_data_on[3]+(av_data_on[7]*256),'<',1);
		int2lcd(av_head_int[1],'>',1);	
		av_j_si_max=5;				

		
		}



	else if((av_head[0]=='S')||(av_head[0]=='I'))
		{  
		ptrs[0]="   ������ ��� N+    ";
		
		if(av_head[2]=='L')
			{
			ptrs[1]="     ����������     ";
			}
		else if(av_head[2]=='T')
			{
			ptrs[1]="      ��������      ";
			}		
		else if(av_head[2]=='U')
			{
			ptrs[1]="   �������� U���.   ";
			}		
		else if(av_head[2]=='u')
			{
			ptrs[1]="   �������� U���.   ";
			}								
		else if(av_head[2]=='O')
			{
			ptrs[1]="    �������� I���   ";
			}		
		
		ptrs[2]="  0%(  0^ 0@:0#:0$  ";
		if((av_data_off[0]=='A')&&(av_data_off[1]=='A'))
			{
			ptrs[3]="    �� ���������    ";
			bgnd_par(ptrs[0],ptrs[1],ptrs[2],ptrs[3]);
			}
		else 
			{
			gran_char(&index_set,0,1);
			ptrs[3]="      ���������     ";
			ptrs[4]="  0[]  0< 0>:0=:0)  ";
			bgnd_par(ptrs[index_set],ptrs[1+index_set],ptrs[2+index_set],ptrs[3+index_set]);
			int2lcd(av_data_off[4],'>',0);
			int2lcd(av_data_off[5],'=',0);
			int2lcd(av_data_off[6],')',0);
			int2lcd(av_data_off[2],'[',0);
			int2lcd(av_data_off[0],'<',0); 
			if(!((av_data_off[1]>=1)&&(av_data_off[1]<=12)))av_data_off[1]=1;
			sub_bgnd(sm_mont[av_data_off[1]],']',0);
			
						
			}	
		int2lcd(av_head[1]+1,'+',0);
		int2lcd(av_data_on[4],'@',0);
		int2lcd(av_data_on[5],'#',0);
		int2lcd(av_data_on[6],'$',0);
		int2lcd(av_data_on[2],'%',0);
		int2lcd(av_data_on[0],'^',0); 
		if(!((av_data_on[1]>=1)&&(av_data_on[1]<=12)))av_data_on[1]=1;
		sub_bgnd(sm_mont[av_data_on[1]],'(',0);
		
		}

	else if((av_head[0]=='V')&&(av_head[1]=='I')&&(av_head[2]=='N'))
		{
		av_j_si_max=4;  
		ptrs[0]=" ������ ��������    ";
		ptrs[1]=" ���������� (�����.)";
		if(av_head_int[1]==1)ptrs[1]=" ���������� (�����.)";
		ptrs[2]="   ������           ";
		ptrs[3]="  0%[  0^ 0@:0#:0$  ";
		ptrs[4]="        U��=    <�  ";
		ptrs[5]="   �����            ";
		ptrs[6]="  0qw  0r 0i:0l:0s  ";
		ptrs[7]="        U��=    >�  ";
		if((av_data_off[0]=='A')&&(av_data_off[1]=='A'))
			{
			ptrs[5]="  �� ���������      ";
			ptrs[6]="                    ";
			ptrs[7]="                    ";
			av_j_si_max=2;
			}
			
		bgnd_par(ptrs[index_set],ptrs[1+index_set],ptrs[2+index_set],ptrs[3+index_set]);
		
		int2lcd(av_head[1]+1,'!',0);

		int2lcd(av_data_on[4],'@',0);
		int2lcd(av_data_on[5],'#',0);
		int2lcd(av_data_on[6],'$',0);
		int2lcd(av_data_on[2],'%',0);
		int2lcd(av_data_on[0],'^',0); 
		if(!((av_data_on[1]>=1)&&(av_data_on[1]<=12)))av_data_on[1]=1;
		sub_bgnd(sm_mont[av_data_on[1]],'[',0);
				
		int2lcd(av_data_off[4],'i',0);
		int2lcd(av_data_off[5],'l',0);
		int2lcd(av_data_off[6],'s',0);
		int2lcd(av_data_off[2],'q',0);
		int2lcd(av_data_off[0],'r',0); 
		if(!((av_data_off[1]>=1)&&(av_data_off[1]<=12)))av_data_off[1]=1;
		sub_bgnd(sm_mont[av_data_off[1]],'w',0);
		
		int2lcd(av_head_int[0],'<',1);
		int2lcd(av_data[0]+(av_data[1]*256),'>',1);
			
		
		//int2lcdyx(av_head_int[0],0,3,0);
		//int2lcdyx(av_head_int[1],0,7,0);

						
		}

	else if((av_head[0]=='V')&&(av_head[1]=='O')&&(av_head[2]=='U')&&(av_head[3]=='T'))
		{
		av_j_si_max=4;  
		ptrs[0]=" ������ ���������   ";
		ptrs[1]=" ���������� (�����.)";
		if(av_head_int[2]==1)ptrs[1]=" ���������� (�����.)";
		ptrs[2]="   ������           ";
		ptrs[3]="  0%[  0^ 0@:0#:0$  ";
		ptrs[4]="       U���=    <�  ";
		ptrs[5]="   �����            ";
		ptrs[6]="  0qw  0r 0i:0l:0s  ";
		ptrs[7]="       U���=    >�  ";
		if((av_data_off[0]=='A')&&(av_data_off[1]=='A'))
			{
			ptrs[5]="  �� ���������      ";
			ptrs[6]="                    ";
			ptrs[7]="                    ";
			av_j_si_max=2;
			}
			
		bgnd_par(ptrs[index_set],ptrs[1+index_set],ptrs[2+index_set],ptrs[3+index_set]);
		
		int2lcd(av_head[1]+1,'!',0);

		int2lcd(av_data_on[4],'@',0);
		int2lcd(av_data_on[5],'#',0);
		int2lcd(av_data_on[6],'$',0);
		int2lcd(av_data_on[2],'%',0);
		int2lcd(av_data_on[0],'^',0); 
		if(!((av_data_on[1]>=1)&&(av_data_on[1]<=12)))av_data_on[1]=1;
		sub_bgnd(sm_mont[av_data_on[1]],'[',0);
				
		int2lcd(av_data_off[4],'i',0);
		int2lcd(av_data_off[5],'l',0);
		int2lcd(av_data_off[6],'s',0);
		int2lcd(av_data_off[2],'q',0);
		int2lcd(av_data_off[0],'r',0); 
		if(!((av_data_off[1]>=1)&&(av_data_off[1]<=12)))av_data_off[1]=1;
		sub_bgnd(sm_mont[av_data_off[1]],'w',0);
		
		int2lcd(av_head_int[0],'<',1);
		int2lcd(av_data[0]+(av_data[1]*256),'>',1);

						
		}


	else if((av_head[0]=='O')&&(av_head[1]=='V')&&(av_head[2]=='L'))
		{  
		ptrs[0]=" ���������� ������� ";
		ptrs[1]="       �� ����      ";
		
		ptrs[2]="  0%(  0^ 0@:0#:0$  ";
		if((av_data_off[0]=='A')&&(av_data_off[1]=='A'))
			{
			ptrs[3]="    �� ���������    ";
			bgnd_par(ptrs[0],ptrs[1],ptrs[2],ptrs[3]);
			}
		else 
			{
			gran_char(&index_set,0,1);
			ptrs[3]="      ���������     ";
			ptrs[4]="  0[]  0< 0>:0=:0)  ";
			bgnd_par(ptrs[index_set],ptrs[1+index_set],ptrs[2+index_set],ptrs[3+index_set]);
			int2lcd(av_data_off[4],'>',0);
			int2lcd(av_data_off[5],'=',0);
			int2lcd(av_data_off[6],')',0);
			int2lcd(av_data_off[2],'[',0);
			int2lcd(av_data_off[0],'<',0); 
			if(!((av_data_off[1]>=1)&&(av_data_off[1]<=12)))av_data_off[1]=1;
			sub_bgnd(sm_mont[av_data_off[1]],']',0);
			
						
			}	
		int2lcd(av_head[1]+1,'+',0);
		int2lcd(av_data_on[4],'@',0);
		int2lcd(av_data_on[5],'#',0);
		int2lcd(av_data_on[6],'$',0);
		int2lcd(av_data_on[2],'%',0);
		int2lcd(av_data_on[0],'^',0); 
		if(!((av_data_on[1]>=1)&&(av_data_on[1]<=12)))av_data_on[1]=1;
		sub_bgnd(sm_mont[av_data_on[1]],'(',0);
		
		}

	else if((av_head[0]=='T')&&(av_head[1]=='S')&&(av_head[2]=='Y')&&(av_head[3]=='S'))
		{
		av_j_si_max=4;  
		ptrs[0]="  ������ ��������   ";
		ptrs[1]="      �������       ";
		ptrs[2]="   ������           ";
		ptrs[3]="  0%[  0^ 0@:0#:0$  ";
		ptrs[4]="  T������� =   <��  ";
		ptrs[5]="   �����            ";
		ptrs[6]="  0qw  0r 0i:0l:0s  ";
		ptrs[7]="  T������� =   >��  ";
		if((av_data_off[0]=='A')&&(av_data_off[1]=='A'))
			{
			ptrs[5]="  �� ���������      ";
			ptrs[6]="                    ";
			ptrs[7]="                    ";
			av_j_si_max=2;
			}
			
		bgnd_par(ptrs[index_set],ptrs[1+index_set],ptrs[2+index_set],ptrs[3+index_set]);
		
		int2lcd(av_head[1]+1,'!',0);

		int2lcd(av_data_on[4],'@',0);
		int2lcd(av_data_on[5],'#',0);
		int2lcd(av_data_on[6],'$',0);
		int2lcd(av_data_on[2],'%',0);
		int2lcd(av_data_on[0],'^',0); 
		if(!((av_data_on[1]>=1)&&(av_data_on[1]<=12)))av_data_on[1]=1;
		sub_bgnd(sm_mont[av_data_on[1]],'[',0);
				
		int2lcd(av_data_off[4],'i',0);
		int2lcd(av_data_off[5],'l',0);
		int2lcd(av_data_off[6],'s',0);
		int2lcd(av_data_off[2],'q',0);
		int2lcd(av_data_off[0],'r',0); 
		if(!((av_data_off[1]>=1)&&(av_data_off[1]<=12)))av_data_off[1]=1;
		sub_bgnd(sm_mont[av_data_off[1]],'w',0);
		
		int2lcd(av_head_int[0],'<',0);
		int2lcd(av_data[0]+(av_data[1]*256),'>',0);

						
		}	
	
	}
		 
else if(ind==iBatLog)
	{
	if(BAT_IS_ON[sub_ind1]==bisON)ptrs[0]=" �������  0!/@  /0# ";
	else ptrs[0]=" �������� 0!/@  /0# ";
     ptrs[1]=" �����.���.     $A*�";
     ptrs[2]=" ���������      %�. ";
     ptrs[3]=" �������� �������   ";
     ptrs[4]=" ������������� �����";
     ptrs[5]=" �������            ";
     ptrs[6]=sm_exit;	
	if((sub_ind-index_set)>1)index_set=sub_ind-1;
	else if(sub_ind<index_set)index_set=sub_ind;
	bgnd_par(	" ���������� ������  ",
			"     ������� N^     ",
			ptrs[index_set],
			ptrs[index_set+1]);
	pointer_set(2);	

	int2lcd(sub_ind1+1,'^',0); 
	int2lcd(BAT_DAY_OF_ON[sub_ind1],'!',0);
	sub_bgnd(sm_mont[BAT_MONTH_OF_ON[sub_ind1]],'@',0);
	int2lcd(BAT_YEAR_OF_ON[sub_ind1],'#',0); 
	int2lcd(BAT_C_NOM[sub_ind1],'$',0);
	int2lcd(BAT_RESURS[sub_ind1],'%',0);

	/*int2lcdyx(BAT_IS_ON[0],0,2,0);
	int2lcdyx(BAT_IS_ON[1],0,6,0); 
	int2lcdyx(lc640_read_int(EE_BAT1_IS_ON),0,10,0);
	int2lcdyx(lc640_read_int(EE_BAT2_IS_ON),0,14,0);*/
	}

else if(ind==iBatLogKe)
	{             
	if(av_j_si_max==0)
		{
		bgnd_par(	"  �������� �������  ",
				"     ������� N!     ",
				" ������ ����        ",
				sm_exit);
		pointer_set(3);
		sub_ind=0;
		index_set=0;
		} 
	else if(av_j_si_max==1)
		{
		bgnd_par(	"  �������� �������  ",
				"     ������� N!     ",
				" (                  ",
				sm_exit);
		index_set=0;
		pointer_set(2);
		}	
	else
		{
		if(sub_ind<index_set) index_set=sub_ind;
		else if((sub_ind-index_set)>1) index_set=sub_ind-1;
		if(index_set==(av_j_si_max-1)) 
			{
			bgnd_par( "  �������� �������  ",
					"     ������� N!     ",
					" (                  ",
					sm_exit);
			}
		else
			{
			bgnd_par(	"  �������� �������  ",
					"     ������� N!     ",
					" (                  ",
					" [                  ");
			}
		pointer_set(2);			 
		}
		
   	int2lcd(sub_ind1+1,'!',0);
 	event_data2ind(content[index_set],'(');
 	event_data2ind(content[index_set+1],'[');
	}

else if(ind==iBatLogVz)
	{
	if(av_j_si_max==0)
		{
		bgnd_par(	"������������� ������",
				"     ������� N!     ",
				" ������ ����        ",
				sm_exit);
		sub_ind=0;
		index_set=0;
		pointer_set(3);
		} 
	else if(av_j_si_max==1)
		{
		bgnd_par(	"������������� ������",
				"     ������� N!     ",
				" (                  ",
				sm_exit);
		index_set=0;
		pointer_set(2);
		}	
	else
		{
		if(sub_ind<index_set) index_set=sub_ind;
		else if((sub_ind-index_set)>1) index_set=sub_ind-1;
		if(index_set==(av_j_si_max-1)) 
			{
			bgnd_par(	"������������� ������",
					"     ������� N!     ",
					" (                  ",
					sm_exit);
			}

		else bgnd_par(	"������������� ������",
					"     ������� N!     ",
					" (                  ",
					" [                  "); 
		pointer_set(2);			        
		}
   	int2lcd(sub_ind1+1,'!',0);
 	event_data2ind(content[index_set],'(');
 	event_data2ind(content[index_set+1],'[');
	
	}
   
else if(ind==iBatLogWrk)
	{
	if(av_j_si_max==0)
		{
		bgnd_par(	"      �������       ",
				"     ������� N!     ",
				" ������ ����        ",
				sm_exit);
		sub_ind=0;
		index_set=0;
		pointer_set(3);
		} 
	else if(av_j_si_max==1)
		{
		bgnd_par(	"      �������       ",
				"     ������� N!     ",
				" (                  ",
				sm_exit);
		index_set=0;
		pointer_set(2);
		}	

	else
		{
		if(sub_ind<index_set) index_set=sub_ind;
		else if((sub_ind-index_set)>1) index_set=sub_ind-1;
		if(index_set==(av_j_si_max-1))
			{
			bgnd_par(	"      �������       ",
					"     ������� N!     ",
					" (                  ",
					sm_exit);
			}
		else bgnd_par(	"      �������       ",
					"     ������� N!     ",
					" (                  ",
					" [                  ");

		pointer_set(2);
		}

   	int2lcd(sub_ind1+1,'!',0);
 	event_data2ind(content[index_set],'(');
 	event_data2ind(content[index_set+1],'[');

	

	} 
	
else if((ind==iSet_prl)||(ind==iK_prl)
	||(ind==iAusw_prl)||(ind==iPrltst))
	{
	bgnd_par("  �������  ������   ",sm_,sm_,sm_);
	int2lcdyx(parol[0],1,8,0);
     int2lcdyx(parol[1],1,9,0);
     int2lcdyx(parol[2],1,10,0);
     lcd_buffer[48+sub_ind]='�';
	}	
		
else if(ind==iPrl_bat_in_out)
	{
	if(BAT_IS_ON[sub_ind1]==bisON)ptrs[0]="��� ��������� ���.-�";
	else  ptrs[0]="��� �������� �������";
	bgnd_par(ptrs[0],"  �������� ������   ",sm_,sm_);
	
     int2lcdyx(parol[0],2,8,0);
     int2lcdyx(parol[1],2,9,0);
     int2lcdyx(parol[2],2,10,0);
     lcd_buffer[68+sub_ind]='�';	
	}

else if(ind==iPrl_bat_in_sel)
	{
	
	bgnd_par(	"��� �������� �������",
			"   ������� �� ���   ",
			" ��������-��������� ",
			" GYFP4875T          ");
	
	pointer_set(2);
	}


else if(ind==iSet)
	{
     ptrs[0]=		" �����������        ";
	ptrs[1]=		" ����� � ����       ";
     ptrs[2]=		" ���������          ";
	ptrs[3]=		" ��������������     ";
	ptrs[4]=		" �����              ";
     ptrs[5]=		" ���������         y";
	ptrs[6]=		" ��.����.   (       ";
	ptrs[7]=		" ���������� ������� ";
	ptrs[8]=		"  ������    )       ";
	ptrs[9]=		" ��� ����������     ";
	ptrs[10]=		" ������.������ z    ";
	ptrs[11]=		" T ��������   ����  ";
     ptrs[12]=		" �������     q���.  ";
     ptrs[13]=		" Umax=       !�     ";
     ptrs[14]=		" Umin=       Z�     ";
     ptrs[15]=		" U�0�=       @�     ";
     ptrs[16]=		" U�20�=      #�     ";
     ptrs[17]=		" U����=      ^�     ";
     ptrs[18]=		" Umin.����=  &�     ";
	ptrs[19]=		" U0�=        >�     ";
	ptrs[20]=		" I��.=       j�     ";
     ptrs[21]=		" I�.���.=    J�     ";
     ptrs[22]=		" Imax =      ]A     ";
     ptrs[23]=		" Imin =      {A     ";
     ptrs[24]=		" U���.���.=   [�    ";
     ptrs[25]=		" T�.���.�.�. !�     ";
	ptrs[26]=		" t�.max=     $�C    ";
	ptrs[27]=		" t�.����=    z�C    ";
	ptrs[28]=		" t���.max=   b�C    ";
	ptrs[29]=		" t���.����=  X�C    ";
     ptrs[30]=		" ������� �������    ";
	ptrs[31]=		" Ethernet           ";
     ptrs[32]=      " ����� ��������    +";
     ptrs[33]=      " �������� ��.�����  ";
     ptrs[34]=      " �������         Q% ";
	ptrs[35]=      " �������� N        w";
     ptrs[36]=		" �����              ";
     ptrs[37]=		" ����������         "; 
     ptrs[38]=		"                    ";        
	
	if((sub_ind-index_set)>2)index_set=sub_ind-2;
	else if(sub_ind<index_set)index_set=sub_ind;
	bgnd_par("     ���������      ",
			ptrs[index_set],ptrs[index_set+1],ptrs[index_set+2]);

	pointer_set(1);	
	
	if(index_set<19)
	     {
	     if(ZV_ON)sub_bgnd("���.",'(',0);
	     else sub_bgnd("���.",'(',0);
	     if(AV_OFF_AVT)sub_bgnd("�����.",')',0);
	     else sub_bgnd("����.",')',0);
//		if(PAR)sub_bgnd("���.",'z',0);
//	     else sub_bgnd("���.",'z',0);
	     if(MNEMO_ON==mnON)
	     	{
	     	sub_bgnd("����� y�.",'y',-8);
	     	int2lcd(MNEMO_TIME,'y',0);
	     	}
	     else sub_bgnd("����.",'y',-4);
	     int2lcd(UMAX,'!',1);
	     //int2lcd((UB20-DU),'Z',1);
	     //int2lcd(UB0,'@',1);
	     //int2lcd(UB20,'#',1);
	     int2lcd(USIGN,'^',0);
	     int2lcd(UMN,'&',0);
	     int2lcd(U0B,'>',1);
	     } 
	int2lcd(TMAX,'$',0);
	int2lcd(IKB,'j',2);
//	int2lcd(UVZ,'[',1);
	int2lcd(IMAX,']',1);
	int2lcd(IMIN,'{',1);
	int2lcd(IZMAX,'J',1); 
	int2lcd(TZAS,'!',0);
	int2lcd(TBAT,'q',0);
	int2lcd(TSIGN,'z',0); 
	int2lcd(TBATMAX,'b',0); 
	int2lcd(TBATSIGN,'X',0);
     int2lcd(POWER_CNT_ADRESS,'+',0);
     if(UBM_AV)
          {
          int2lcd(UBM_AV,'Q',0);
          } 
     else sub_bgnd("����.",'Q',-2);


	long2lcd_mmm(AUSW_MAIN_NUMBER,'w',0);
	}


else if((ind==iSet_VD))
	{
    ptrs[0]=		" �����������        ";
	ptrs[1]=		" ����� � ����       ";
    ptrs[2]=		" ���������          ";
	ptrs[3]=		" �����              ";
	ptrs[4]=		" ��.����.   (       ";
	ptrs[5]=		" ���������� ������� ";
	ptrs[6]=		"  ������    )       ";
	ptrs[7]=		" ��� ����������     ";
	ptrs[8]=		" U���. =        @�  ";
    ptrs[9]=		" U�����. =      #�  ";
	ptrs[10]=		" t���.���� =    z�C ";
	ptrs[11]=		" t���.max =     $�C ";
	ptrs[12]=		" t����.max =    [�C ";
	ptrs[13]=		" dU���.���� =   Z�  ";
    ptrs[14]=		" U���.���� =    !�  ";
    ptrs[15]=		" U��.����.max =   ^�";
    ptrs[16]=		" U��.����.min =   &�";
    ptrs[17]=		" U���.����.max =  j�";
    ptrs[18]=		" U���.����.min =  J�";
	ptrs[19]=		" T�.��.����. =    }�";
    ptrs[20]=		" ����� �������      ";
    ptrs[21]=		" �����������      ]�";
	ptrs[22]=		" ����               ";
	ptrs[23]=		" Ethernet           ";
	ptrs[24]=		" MODBUS ADRESS     <";
	ptrs[25]=		" MODBUS BAUDRATE    ";
	ptrs[26]=		"                  >0";
	ptrs[27]=		" ��������� ����     ";
	ptrs[28]=		" ��������          {";
    ptrs[29]=		" �����              ";
    ptrs[30]=		" ����������         "; 
    ptrs[31]=		"                    ";        
	
	if((sub_ind-index_set)>2)index_set=sub_ind-2;
	else if(sub_ind<index_set)index_set=sub_ind;
	bgnd_par("     ���������      ",
			ptrs[index_set],ptrs[index_set+1],ptrs[index_set+2]);

	pointer_set(1);	
	
//	if(index_set<19)
		{
	    if(ZV_ON)sub_bgnd("���.",'(',0);
	    else sub_bgnd("���.",'(',0);
	    if(AV_OFF_AVT)sub_bgnd("�����.",')',0);
	    else sub_bgnd("����.",')',0);
		int2lcd(UOUT,'@',1);
	    int2lcd(UAVT,'#',1);
		int2lcd(TMAX,'$',0);	
		int2lcd(TSIGN,'z',0);
		int2lcd(TSYSMAX,'[',0);
		int2lcd(UMAX,'!',1);
		int2lcd(DU,'Z',1);
	    int2lcd(UINMAX/10,'^',0);
	    int2lcd(UINMIN/10,'&',0);
		int2lcd(UOUTMAX/10,'j',0);	
		int2lcd(UOUTMIN/10,'J',0);
		int2lcd(TZAS,'}',0);
		int2lcd(TVENTMAX*10,']',0);

		if(TBAT==0)sub_bgnd("����.",'q',0);	
		else int2lcd(TBAT,'q',0);

		if(I_LOAD_MODE)
     		{
     		sub_bgnd("����",'{',-3);
     		}
		else sub_bgnd("����I���",'{',-7);
	    } 
	
	
//	int2lcd(UVZ,'[',1);
	//int2lcd(IMAX,']',1);
	//int2lcd(IMIN,'{',1);
 
	
	
 
	int2lcd(TBATMAX,'b',0); 
	int2lcd(TBATSIGN,'X',0);
     int2lcd(POWER_CNT_ADRESS,'+',0);
     if(UBM_AV)
          {
          int2lcd(UBM_AV,'Q',0);
          } 
     else sub_bgnd("����.",'Q',-2);


	long2lcd_mmm(AUSW_MAIN_NUMBER,'w',0);
	if(index_set>19)
		{
		if(TERMOKOMPENS)sub_bgnd("���.",'q',-3);
		else sub_bgnd("����.",'q',-4);
		}
	int2lcd(MODBUS_ADRESS,'<',0);
	int2lcd(MODBUS_BAUDRATE,'>',0);

	if((FORVARDBPSCHHOUR<=0)||(FORVARDBPSCHHOUR>500)) {
		sub_bgnd("����.",'l',0);
	} else {
		int2lcd(FORVARDBPSCHHOUR,'l',0);	
	}

	//int2lcdyx(sub_ind,0,3,0);
	//int2lcdyx(index_set,0,1,0);PWM_START
	int2lcd(PWM_START,'(',0);
	
	if(KB_ALGORITM==1)	sub_bgnd("1-� ����.",')',0);
	else if(KB_ALGORITM==2)	sub_bgnd("2-� ����.",')',0);
	else 				sub_bgnd("3-� ����.",')',0);
	if(REG_SPEED==2)	sub_bgnd("��������/2",'&',0);
	else if(REG_SPEED==3)	sub_bgnd("��������/3",'&',0);
	else if(REG_SPEED==4)	sub_bgnd("��������/4",'&',0);
	else if(REG_SPEED==5)	sub_bgnd("��������/5",'&',0);
	else 				sub_bgnd("��������",'&',0);

	int2lcd(MODBUS_ADRESS,'<',0);
	int2lcd(MODBUS_BAUDRATE,'>',0);

	}




else if (ind==iDef_220_IPS_TERMOKOMPENSAT)
	{ 
	ptrs[0]=" ���380/220-45����17";
	ptrs[1]=" ���220/220-10����17";
	ptrs[2]=" ���380/110-90����9 ";
	ptrs[3]=" ���380/220-���18   ";
	ptrs[4]=sm_exit;
	ptrs[5]="                    ";
	ptrs[6]="                    ";
	if(bFL5)ptrs[default_temp]=sm_;
	
	if(sub_ind<index_set) index_set=sub_ind;
	else if((sub_ind-index_set)>2) index_set=sub_ind-2;

    	bgnd_par("����������� ���.-�� ",ptrs[index_set],ptrs[index_set+1],ptrs[index_set+2]);
	
	pointer_set(1);
	} 	

else if(ind==iSet_T)
	{
	static char phase_cnt;
	if(++phase_cnt>=15)
	     {
	     phase_cnt=0;
	     if(++phase>=3)phase=0;
	     }
	ptrs[0]=sm_time;
	ptrs[1]=sm_;
	if(phase==0)ptrs[2]="     <> - �����     ";
     if(phase==1)ptrs[2]="   ^v - ���������   ";
     if(phase==2)ptrs[2]="     �  - �����     ";
	
	bgnd_par(" ���������  ������� ",ptrs[0],ptrs[1],ptrs[2]);
     if(sub_ind==0)lcd_buffer[42]='^';
     else if(sub_ind==1)lcd_buffer[45]='^';
     else if(sub_ind==2)lcd_buffer[48]='^';
     else if(sub_ind==3)lcd_buffer[51]='^';
     else if(sub_ind==4)lcd_buffer[54]='^';
     else if(sub_ind==5)lcd_buffer[58]='^';
  
 	int2lcd(LPC_RTC->SEC,'&',0);
 	int2lcd(LPC_RTC->MIN,'^',0);
 	int2lcd(LPC_RTC->HOUR,'%',0);
 	
 	int2lcd(LPC_RTC->DOM,'<',0);
 	sub_bgnd(sm_mont[LPC_RTC->MONTH],'>',0);
 	int2lcd(LPC_RTC->YEAR,'{',0);
 	if(bFL2)
 	     {
 	     lcd_buffer[find(':')]=' ';
 	     lcd_buffer[find(':')]=' ';
 	     }  
	}  

else if(ind==iStr)
	{
	ptrs[0]=" �������           @";
	ptrs[1]=" ����������        !";
	ptrs[2]=" ����������        ^";	
	ptrs[3]=" ��������������     ";
	ptrs[4]=" ���������         $";
	ptrs[5]=" �����              ";

	if(sub_ind<index_set) index_set=sub_ind;
	else if((sub_ind-index_set)>2) index_set=sub_ind-2;
	bgnd_par("      ���������     ",ptrs[index_set],ptrs[index_set+1],ptrs[index_set+2]);
	
	pointer_set(1);

	int2lcd(NUMBAT,'@',0);		
	int2lcd(NUMIST,'!',0);
	int2lcd(NUMAVT,'$',0);	 
	int2lcd(NUMINV,'^',0);
	int2lcd(NUMDT,'#',0);
	int2lcd(NUMSK,'$',0);
	}    

else if(ind==iStr_VD)
	{
	ptrs[0]=" ����������        !";
	ptrs[1]=" �������� ������.  #";
//	ptrs[2]=" ��������� ���     %";
//	ptrs[3]=" ����� ���������   $";
	ptrs[2]=" �����              ";
	
	if(sub_ind<index_set) index_set=sub_ind;
	else if((sub_ind-index_set)>2) index_set=sub_ind-2;
	bgnd_par("      ���������     ",ptrs[index_set],ptrs[index_set+1],ptrs[index_set+2]);
	
	pointer_set(1);

	int2lcd(NUMIST,'!',0); 
	int2lcd(NUMDT,'#',0);
	int2lcd(NUMMAKB,'%',0);
	int2lcd(NUMSK,'$',0);
	}    

else if (ind==iLan_set)
	{
	char sss[10]="abcdef";
	char i/*,i_flag*/;
	 
	ptrs[0]=	" Ethernet         ! ";
	ptrs[1]=	" DHCP������       @ ";
	ptrs[2]=	" IP�����            ";
	ptrs[3]=	"  000.000.000.00#   ";
	ptrs[4]=	" ����� �������      ";
	ptrs[5]=	"  000.000.000.00$   ";
	ptrs[6]=	" ����               ";
	ptrs[7]=	"  000.000.000.00)   ";
	ptrs[8]=	" ����.������       [";
	ptrs[9]=	" ����.������       ]";
	ptrs[10]=	" Community <        ";
	ptrs[11]=	" ������� ��� TRAP N1";
	ptrs[12]=	"  000.000.000.00%   ";
	ptrs[13]=	" ������� ��� TRAP N2";
	ptrs[14]=	"  000.000.000.00^   ";
	ptrs[15]=	" ������� ��� TRAP N3";
	ptrs[16]=	"  000.000.000.00&   ";
	ptrs[17]=	" ������� ��� TRAP N4";
	ptrs[18]=	"  000.000.000.00*   ";
	ptrs[19]=	" ������� ��� TRAP N5";
	ptrs[20]=	"  000.000.000.00(   ";
	ptrs[21]=	" �����              ";

	
	if(!ETH_IS_ON)
		{
		ptrs[1]=" �����              ";
		ptrs[2]="                    ";
		ptrs[3]="                    ";
		}

	
	if(sub_ind<index_set) index_set=sub_ind;
	else if((sub_ind-index_set)>2) index_set=sub_ind-2;

     bgnd_par(	" ��������� Ethernet ",
			ptrs[index_set],
			ptrs[index_set+1],
			ptrs[index_set+2]);
	
	pointer_set(1);
     if(ETH_IS_ON)
     	{
     	sub_bgnd("���.",'!',-3);   
     	}
     else 
     	{
     	sub_bgnd("����.",'!',-4);   
     	}

     if(ETH_DHCP_ON)
     	{
     	sub_bgnd("���.",'@',-3);   
     	}
     else 
     	{
     	sub_bgnd("����.",'@',-4);   
     	}
		  
	if(sub_ind==2)	ip2lcd(ETH_IP_1,ETH_IP_2,ETH_IP_3,ETH_IP_4,'#',(sub_ind1+1));
	else ip2lcd(ETH_IP_1,ETH_IP_2,ETH_IP_3,ETH_IP_4,'#',0);
	if(sub_ind==4)	ip2lcd(ETH_MASK_1,ETH_MASK_2,ETH_MASK_3,ETH_MASK_4,'$',(sub_ind1+1));
	else ip2lcd(ETH_MASK_1,ETH_MASK_2,ETH_MASK_3,ETH_MASK_4,'$',0);
	if(sub_ind==6)	ip2lcd(ETH_GW_1,ETH_GW_2,ETH_GW_3,ETH_GW_4,')',(sub_ind1+1));
	else ip2lcd(ETH_GW_1,ETH_GW_2,ETH_GW_3,ETH_GW_4,')',0);

	int2lcd(ETH_SNMP_PORT_READ,'[',0);
	int2lcd(ETH_SNMP_PORT_WRITE,']',0);

	if( (ETH_TRAP1_IP_1==255) && (ETH_TRAP1_IP_2==255) && (ETH_TRAP1_IP_3==255) && (ETH_TRAP1_IP_4==255) ) sub_bgnd("    ���������    ",'%',-14);
	else
		{
		if(sub_ind==11)	ip2lcd(ETH_TRAP1_IP_1,ETH_TRAP1_IP_2,ETH_TRAP1_IP_3,ETH_TRAP1_IP_4,'%',(sub_ind1+1));
		else ip2lcd(ETH_TRAP1_IP_1,ETH_TRAP1_IP_2,ETH_TRAP1_IP_3,ETH_TRAP1_IP_4,'%',0);
		}

	if( (ETH_TRAP2_IP_1==255) && (ETH_TRAP2_IP_2==255) && (ETH_TRAP2_IP_3==255) && (ETH_TRAP2_IP_4==255) ) sub_bgnd("    ���������    ",'^',-14);
	else
		{
		if(sub_ind==13)	ip2lcd(ETH_TRAP2_IP_1,ETH_TRAP2_IP_2,ETH_TRAP2_IP_3,ETH_TRAP2_IP_4,'^',(sub_ind1+1));
		else ip2lcd(ETH_TRAP2_IP_1,ETH_TRAP2_IP_2,ETH_TRAP2_IP_3,ETH_TRAP2_IP_4,'^',0);
		}

	if( (ETH_TRAP3_IP_1==255) && (ETH_TRAP3_IP_2==255) && (ETH_TRAP3_IP_3==255) && (ETH_TRAP3_IP_4==255) ) sub_bgnd("    ���������    ",'&',-14);
	else
		{
		if(sub_ind==15)	ip2lcd(ETH_TRAP3_IP_1,ETH_TRAP3_IP_2,ETH_TRAP3_IP_3,ETH_TRAP3_IP_4,'&',(sub_ind1+1));
		else ip2lcd(ETH_TRAP3_IP_1,ETH_TRAP3_IP_2,ETH_TRAP3_IP_3,ETH_TRAP3_IP_4,'&',0);
		}

	if( (ETH_TRAP4_IP_1==255) && (ETH_TRAP4_IP_2==255) && (ETH_TRAP4_IP_3==255) && (ETH_TRAP4_IP_4==255) ) sub_bgnd("    ���������    ",'*',-14);
	else
		{
		if(sub_ind==17)	ip2lcd(ETH_TRAP4_IP_1,ETH_TRAP4_IP_2,ETH_TRAP4_IP_3,ETH_TRAP4_IP_4,'*',(sub_ind1+1));
		else ip2lcd(ETH_TRAP4_IP_1,ETH_TRAP4_IP_2,ETH_TRAP4_IP_3,ETH_TRAP4_IP_4,'*',0);
		}

	if( (ETH_TRAP5_IP_1==255) && (ETH_TRAP5_IP_2==255) && (ETH_TRAP5_IP_3==255) && (ETH_TRAP5_IP_4==255) ) sub_bgnd("    ���������    ",'(',-14);
	else
		{
		if(sub_ind==19)	ip2lcd(ETH_TRAP5_IP_1,ETH_TRAP5_IP_2,ETH_TRAP5_IP_3,ETH_TRAP5_IP_4,'(',(sub_ind1+1));
		else ip2lcd(ETH_TRAP5_IP_1,ETH_TRAP5_IP_2,ETH_TRAP5_IP_3,ETH_TRAP5_IP_4,'(',0);
		}

/*	if((sub_ind==2)&&(sub_ind1==0)&&(bFL2))
		{
		sub_bgnd("   ",'#',-2);
		}
	else int2lcd(ETH_IP_1,'#',0);

	if((sub_ind==2)&&(sub_ind1==1)&&(bFL2))
		{
		sub_bgnd("   ",'$',-2);
		}
	else int2lcd(ETH_IP_2,'$',0);

	if((sub_ind==2)&&(sub_ind1==2)&&(bFL2))
		{
		sub_bgnd("   ",'%',-2);
		}
	else int2lcd(ETH_IP_3,'%',0);

	if((sub_ind==2)&&(sub_ind1==3)&&(bFL2))
		{
		sub_bgnd("   ",'^',-2);
		}
	else int2lcd(ETH_IP_4,'^',0);*/


	//int2lcdyx(sub_ind,0,1,0);	
	//int2lcdyx(index_set,0,3,0);
	//int2lcdyx(sub_ind1,0,5,0);
	//for(i=0;(i<9)&&(snmp_community[i]))

	for(i=0;i<9;i++)
		{
		sss[i]=snmp_community[i];
		}
	sss[9]=0;		

	if(sub_ind==10)community2lcd(sss,'<',sub_ind1,1);
	else community2lcd(sss,'<',sub_ind1,0);
	
	//int2lcdyx(snmp_community[0],0,4,0);
	//int2lcdyx(snmp_community[11],0,9,0);
	//int2lcdyx(snmp_community[2],0,14,0);
	//int2lcdyx(snmp_community[sub_ind1],0,19,0);	
	}



else if (ind==iBlok_ips_set)
	{
	char sss[10]="abcdef";
///	char i/*,i_flag*/;
	 
	ptrs[0]=	" ������������      ^";
	ptrs[1]=	" ������ ������������";
	ptrs[2]=	"                   &";
	ptrs[3]=	" �����              ";

	
	if(sub_ind<index_set) index_set=sub_ind;
	else if((sub_ind-index_set)>2) index_set=sub_ind-2;

     bgnd_par(	"   ���������� ���   ",
			ptrs[index_set],
			ptrs[index_set+1],
			ptrs[index_set+2]);
	
	pointer_set(1);

	if(ipsBlckSrc==1)sub_bgnd("��1",'^',-2);
	else if(ipsBlckSrc==2)sub_bgnd("��2",'^',-2);   
    else if(ipsBlckSrc==0)sub_bgnd("����.",'^',-4);
	else sub_bgnd("������..",'^',-6); 
	if(ipsBlckLog==0)sub_bgnd("�������.",'&',-7);
	else if(ipsBlckLog==1) sub_bgnd("�����.",'&',-5);  
    else sub_bgnd("������.",'&',-6); 	  

	
	}


else if (ind==iApv)
	{ 
	ptrs[0]=			" ��� 1� ������� !   ";
	if(APV_ON1!=apvON)
	     {
	     ptrs[1]=		" �����              ";
	     ptrs[2]=sm_;
	     ptrs[3]=sm_;
	     ptrs[4]=sm_;
	     simax=1;
	     }
	else
	     {
	     if(APV_ON2!=apvON)
	          {
	          ptrs[1]=" ��� 2� ������� @   ";
	          ptrs[2]=" �����              ";
	          ptrs[3]=sm_;
	          ptrs[4]=sm_;
	          simax=2;
	          }
	     else 
	          {
               ptrs[1]=" ��� 2� ������� @   ";
	          ptrs[2]=" ������ ���2     #�.";
	          ptrs[3]=" �����              ";
	          ptrs[4]=sm_;
	          simax=3;	          
	          }     
	     }     
	
	if(sub_ind<index_set) index_set=sub_ind;
	else if((sub_ind-index_set)>2) index_set=sub_ind-2;	
     bgnd_par("   ��� ����������   ",ptrs[index_set],ptrs[index_set+1],ptrs[index_set+2]);
	
	pointer_set(1);
	if(APV_ON1==apvON)sub_bgnd("���.",'!',0);
	else sub_bgnd("����.",'!',-1);
	
	if(APV_ON2==apvON)
	     {
	     sub_bgnd("���.",'@',0);
	     int2lcd(APV_ON2_TIME,'#',0);
	     }
	else sub_bgnd("����.",'@',-1);	
     
 	} 
/*
     ptrs[0+NUMDT]=  	" ��1        $       ";            
     ptrs[1+NUMDT]=  	" ��2        %       ";
	ptrs[2+NUMDT]=  	" ��3        ^       ";
	ptrs[3+NUMDT]=  	" ��4        &       ";
	ptrs[0+NUMEXT]=  	" �����              ";
	ptrs[1+NUMEXT]=  	"                    ";
	ptrs[2+NUMEXT]=  	"                    ";

	bgnd_par(		"  ������� �������   ",
				ptrs[index_set],
				ptrs[index_set+1],
				ptrs[index_set+2]);

*/

else if (ind==iExt_set)
	{ 
//	ptrs[0]=			" ������ ������. N1  ";
//	ptrs[1]=			" ������ ������. N2  ";
//	ptrs[2]=			" ������ ������. N3  ";
	ptrs[0]=		" ������ �����       ";
	ptrs[1]=		" ������ ����        ";
	ptrs[2]=		" ������ �����       ";
//	ptrs[3]=		" ������ �����������.";
	ptrs[3]=  	" �����              ";
	ptrs[4]=  	"                    ";
	ptrs[5]=  	"                    ";
	
	if(sub_ind<index_set) index_set=sub_ind;
	else if((sub_ind-index_set)>2) index_set=sub_ind-2;
     bgnd_par("     ���������      ",ptrs[index_set],ptrs[index_set+1],ptrs[index_set+2]);
	
	pointer_set(1);
	}

else if (ind==iExt_set_3U)
	{ 
	ptrs[0]=		" ����� ������� �1   ";
	ptrs[1]=		" ����� ������� �2   ";
	ptrs[2]=		" ����� ������� �3   ";
	ptrs[3]=		" ����� ������� �4   ";
	ptrs[NUMSK]=  	" �����              ";
	ptrs[NUMSK+1]= "                    ";
	ptrs[NUMSK+2]=	"                    ";
	ptrs[NUMSK+3]=	"                    ";
		
	if(sub_ind<index_set) index_set=sub_ind;
	else if((sub_ind-index_set)>2) index_set=sub_ind-2;
     bgnd_par("     ���������      ",ptrs[index_set],ptrs[index_set+1],ptrs[index_set+2]);
	
	pointer_set(1);
	}
	
else if (ind==iExt_dt)
	{ 
	ptrs[0]=" �����������     @�C";
	ptrs[1]=" tmax            #�C";
	ptrs[2]=" tmin            $�C";
	ptrs[3]=" ����            [  ";
	ptrs[4]=" ����            ]  ";
	ptrs[5]=" �������         (  ";
	ptrs[6]=" RS232           )  ";
	ptrs[7]=sm_exit;
	
	if(sub_ind<index_set) index_set=sub_ind;
	else if((sub_ind-index_set)>1) index_set=sub_ind-1;
     bgnd_par("  ������� ������    ","   ����������� N!   ",ptrs[index_set],ptrs[index_set+1]);
	
	pointer_set(2);
	int2lcd(sub_ind1+1,'!',0);
	int2lcd_mmm(t_ext[sub_ind1],'@',0);
	if(!TMAX_EXT_EN[sub_ind1])int2lcd_mmm(TMAX_EXT[sub_ind1],'#',0);
	else sub_bgnd("����.",'#',-2);
	if(!TMIN_EXT_EN[sub_ind1])int2lcd_mmm(TMIN_EXT[sub_ind1],'$',0);
	else sub_bgnd("����.",'$',-2);
	if(!T_EXT_REL_EN[sub_ind1])sub_bgnd("���.",'[',-2);
	else sub_bgnd("����.",'[',-2);
	if(!T_EXT_ZVUK_EN[sub_ind1])sub_bgnd("���.",']',-2);
	else sub_bgnd("����.",']',-2);
	if(!T_EXT_LCD_EN[sub_ind1])sub_bgnd("���.",'(',-2);
	else sub_bgnd("����.",'(',-2);
	if(!T_EXT_RS_EN[sub_ind1])sub_bgnd("���.",')',-2);
	else sub_bgnd("����.",')',-2);	
	
	//int2lcdyx(sub_ind,0,1,0);	
	//int2lcdyx(index_set,0,3,0);
	}	
else if (ind==iExt_sk)
	{ 
	ptrs[0]=" ��������� - @      ";
	ptrs[1]=" ���������          ";
	ptrs[2]=" ��������� - #      ";
	ptrs[3]=" ����            [  ";
	ptrs[4]=" ����            ]  ";
	ptrs[5]=" �������         (  ";
	ptrs[6]=" RS232           )  ";
	ptrs[7]=sm_exit;
	
	if(sub_ind<index_set) index_set=sub_ind;
	else if((sub_ind-index_set)>2) index_set=sub_ind-2;
     bgnd_par("  ����� ������� N!  ",ptrs[index_set],ptrs[index_set+1],ptrs[index_set+2]);
	
	pointer_set(1);
	int2lcd(sub_ind1+1,'!',0);
	if(sk_stat[sub_ind1]==ssON)sub_bgnd("�������",'@',0);
	else sub_bgnd("������.",'@',0);
	if(!SK_SIGN[sub_ind1])sub_bgnd("�������",'#',0);
	else sub_bgnd("������.",'#',0);
	if(!TMIN_EXT_EN[sub_ind1])int2lcd_mmm(TMIN_EXT[sub_ind1],'$',0);
	else sub_bgnd("����.",'$',-6);
	if(!SK_REL_EN[sub_ind1])sub_bgnd("���.",'[',-2);
	else sub_bgnd("����.",'[',-2);
	if(!SK_ZVUK_EN[sub_ind1])sub_bgnd("���.",']',-2);
	else sub_bgnd("����.",']',-2);
	if(!SK_LCD_EN[sub_ind1])sub_bgnd("���.",'(',-2);
	else sub_bgnd("����.",'(',-2);
	if(!SK_RS_EN[sub_ind1])sub_bgnd("���.",')',-2);
	else sub_bgnd("����.",')',-2);	
	
	//int2lcdyx(sub_ind,0,1,0);	
	//int2lcdyx(index_set,0,3,0);
	}		

else if (ind==iExt_sk_3U)
	{ 
	ptrs[0]=" ��������� - @      ";
	ptrs[1]=" ���������          ";
	ptrs[2]=" ��������� - #      ";
	ptrs[3]=" ����            ]  ";
	ptrs[4]=" �������         (  ";
	ptrs[5]=sm_exit;
	
	if(sub_ind<index_set) index_set=sub_ind;
	else if((sub_ind-index_set)>2) index_set=sub_ind-2;
     bgnd_par("  ����� ������� N!  ",ptrs[index_set],ptrs[index_set+1],ptrs[index_set+2]);
	
	pointer_set(1);
	int2lcd(sub_ind1+1,'!',0);
	if(sk_stat[sub_ind1]==ssON)sub_bgnd("�������",'@',0);
	else sub_bgnd("������.",'@',0);
	if(!SK_SIGN[sub_ind1])sub_bgnd("�������",'#',0);
	else sub_bgnd("������.",'#',0);
	if(!SK_ZVUK_EN[sub_ind1])sub_bgnd("���.",']',-2);
	else sub_bgnd("����.",']',-2);
	if(!SK_LCD_EN[sub_ind1])sub_bgnd("���.",'(',-2);
	else sub_bgnd("����.",'(',-2);
	}		

else if (ind==iExt_ddv)
	{ 
	ptrs[0]=" ��������� - @      ";
	ptrs[1]=" �������� ��������� ";
	ptrs[2]=" �����     - #      ";
	ptrs[3]=" ����            [  ";
	ptrs[4]=" �������         (  ";
	ptrs[5]=sm_exit;
	
	if(sub_ind<index_set) index_set=sub_ind;
	else if((sub_ind-index_set)>2) index_set=sub_ind-2;
     bgnd_par("   ������ �����     ",ptrs[index_set],ptrs[index_set+1],ptrs[index_set+2]);
	
	pointer_set(1);
	
	if(sk_stat[0]==ssON)sub_bgnd("�������",'@',0);
	else sub_bgnd("������.",'@',0);
	if(!SK_SIGN[0])sub_bgnd("�������",'#',0);
	else sub_bgnd("������.",'#',0);
	if(SK_REL_EN[0])sub_bgnd("���.",'[',-2);
	else sub_bgnd("����.",'[',-2);
     if(SK_LCD_EN[0])sub_bgnd("���.",'(',-2);
	else sub_bgnd("����.",'(',-2);
	
	}	

else if (ind==iExt_ddi)
	{ 
	ptrs[0]=" ��������� - @      ";
	ptrs[1]=" ���������          ";
	ptrs[2]=" ��������� - #      ";
	ptrs[3]=" ����            [  ";
	ptrs[4]=" �������         (  ";
	ptrs[5]=sm_exit;
	
	if(sub_ind<index_set) index_set=sub_ind;
	else if((sub_ind-index_set)>2) index_set=sub_ind-2;
     bgnd_par("   ������ ����      ",ptrs[index_set],ptrs[index_set+1],ptrs[index_set+2]);
	
	pointer_set(1);
	
	if(sk_stat[1]==ssON)sub_bgnd("�������",'@',0);
	else sub_bgnd("������.",'@',0);
	if(!SK_SIGN[1])sub_bgnd("�������",'#',0);
	else sub_bgnd("������.",'#',0);
	if(SK_REL_EN[1])sub_bgnd("���.",'[',-2);
	else sub_bgnd("����.",'[',-2);
	if(SK_LCD_EN[1])sub_bgnd("���.",'(',-2);
	else sub_bgnd("����.",'(',-2);
	}	

else if (ind==iExt_dud)
	{ 
	ptrs[0]=" ��������� - @      ";
	ptrs[1]=" ���������          ";
	ptrs[2]=" ��������� - #      ";
	ptrs[3]=" ����            [  ";
	ptrs[4]=" �������         (  ";
	ptrs[5]=sm_exit;
	
	if(sub_ind<index_set) index_set=sub_ind;
	else if((sub_ind-index_set)>2) index_set=sub_ind-2;
     bgnd_par("   ������ �����     ",ptrs[index_set],ptrs[index_set+1],ptrs[index_set+2]);
	
	pointer_set(1);
	
	if(sk_stat[2]==ssON)sub_bgnd("�������",'@',0);
	else sub_bgnd("������.",'@',0);
	if(!SK_SIGN[2])sub_bgnd("�������",'#',0);
	else sub_bgnd("������.",'#',0);
	if(SK_REL_EN[2])sub_bgnd("���.",'[',-2);
	else sub_bgnd("����.",'[',-2);
	if(SK_LCD_EN[2])sub_bgnd("���.",'(',-2);
	else sub_bgnd("����.",'(',-2);
	}


else if (ind==iExt_dp)
	{ 
	ptrs[0]=" ��������� - @      ";
	ptrs[1]=" ���������          ";
	ptrs[2]=" ��������� - #      ";
	ptrs[3]=" ����            [  ";
     ptrs[4]=" �������         (  ";
	ptrs[5]=sm_exit;
	
	if(sub_ind<index_set) index_set=sub_ind;
	else if((sub_ind-index_set)>2) index_set=sub_ind-2;
     bgnd_par("������ �����������. ",ptrs[index_set],ptrs[index_set+1],ptrs[index_set+2]);
	
	pointer_set(1);

	if(sk_stat[3]==ssON)sub_bgnd("�������",'@',0);
	else sub_bgnd("������.",'@',0);
	if(!SK_SIGN[3])sub_bgnd("�������",'#',0);
	else sub_bgnd("������.",'#',0);
	if(!SK_REL_EN[3])sub_bgnd("���.",'[',-2);
	else sub_bgnd("����.",'[',-2);
	if(!SK_LCD_EN[3])sub_bgnd("���.",'(',-2);
	else sub_bgnd("����.",'(',-2);


    /* int2lcdyx(sk_stat[0],0,2,0);
     int2lcdyx(sk_stat[1],0,5,0);
     int2lcdyx(sk_stat[2],0,8,0);
     int2lcdyx(sk_stat[3],0,11,0);*/
	}

else if(ind==iK)
	{
	char i;
	i=0;
	
	ptrs[i++]=" ����               ";
	if(NUMBAT)
     ptrs[i++]=" �������            ";
	if(NUMIST)
	ptrs[i++]=" ���                ";
	if(NUMINV)
     ptrs[i++]=" ���������          ";
	ptrs[i++]=" ��������           ";
     ptrs[i++]=" ������� �������    ";
     ptrs[i++]=" ������� �����      ";
     ptrs[i++]=" �����              ";
     ptrs[i++]="                    ";
     ptrs[i++]="                    ";

	if((sub_ind-index_set)>2)index_set=sub_ind-2;
	else if(sub_ind<index_set)index_set=sub_ind;
	bgnd_par("     ����������     ",
			ptrs[index_set],
			ptrs[index_set+1],
			ptrs[index_set+2]);

	pointer_set(1);	 
	}    	


else if(ind==iK_220_IPS_TERMOKOMPENSAT)
	{
	char i;
	i=0;
	
	ptrs[i++]=" ����               ";
	if(NUMBAT)
     ptrs[i++]=" �������            ";
	if(NUMIST)
	ptrs[i++]=" ���                ";
	ptrs[i++]=" �������� ��������� ";
     if(NUMDT)
     ptrs[i++]=" ������� �������    ";
     ptrs[i++]=" �����              ";
     ptrs[i++]="                    ";
     ptrs[i++]="                    ";

	if((sub_ind-index_set)>2)index_set=sub_ind-2;
	else if(sub_ind<index_set)index_set=sub_ind;
	bgnd_par("     ����������     ",
			ptrs[index_set],
			ptrs[index_set+1],
			ptrs[index_set+2]);

	pointer_set(1);	 
	}   

else if(ind==iK_VD)
	{
	char i;
	i=0;
	
	ptrs[i++]=" U���.         !�   ";
	ptrs[i++]=" I���.         @�   ";
	if(NUMIST)
	ptrs[i++]=" ���                ";
	ptrs[i++]=" U�.�.         #�   ";
    ptrs[i++]=" T�������  =   ^�C  ";;
    ptrs[i++]=" �����              ";
    ptrs[i++]="                    ";
    ptrs[i++]="                    ";

	if((sub_ind-index_set)>2)index_set=sub_ind-2;
	else if(sub_ind<index_set)index_set=sub_ind;
	bgnd_par("     ����������     ",
			ptrs[index_set],
			ptrs[index_set+1],
			ptrs[index_set+2]);

	pointer_set(1);

	int2lcd(out_U,'!',1);
	int2lcd(out_I,'@',0);
	int2lcd(vd_U,'#',1);
	int2lcd(sys_T,'^',0);
	
	//int2lcdyx(adc_buff_[0],0,4,0);
    //int2lcdyx(adc_buff_[1],0,9,0);	
	
	if((sub_ind==0)||(sub_ind==3))mess_send(MESS2CNTRL_HNDL,PARAM_CNTRL_STAT_SET,1000,10);
	
	//int2lcdyx(adc_buff_[6],0,4,0);
	//int2lcdyx(Ktext[0],0,9,0);	 
	}   


else if(ind==iK_220_IPS_TERMOKOMPENSAT_IB)
	{
	char i;
	i=0;
	
	ptrs[i++]=" ����               ";
    ptrs[i++]=" �������            ";
	if(NUMIST)
	ptrs[i++]=" ���                ";
	ptrs[i++]=" �������� ��������� ";
    if(NUMDT)
    ptrs[i++]=" ������� �������    ";
    ptrs[i++]=" �����              ";
    ptrs[i++]="                    ";
    ptrs[i++]="                    ";

	if((sub_ind-index_set)>2)index_set=sub_ind-2;
	else if(sub_ind<index_set)index_set=sub_ind;
	bgnd_par("     ����������     ",
			ptrs[index_set],
			ptrs[index_set+1],
			ptrs[index_set+2]);

	pointer_set(1);	 
	}   


else if(ind==iK_TELECORE)
	{
	char i;
	i=0;
	
	ptrs[i++]=	" ����               ";
	if(NUMBAT_TELECORE)
    ptrs[i++]=	" �������            ";
	if(NUMIST)
	ptrs[i++]=	" ���                ";
	ptrs[i++]=	" ��������           ";
    if(NUMDT)
    ptrs[i++]=	" ������� �������    ";
    ptrs[i++]=" �����              ";
    ptrs[i++]="                    ";
    ptrs[i++]="                    ";

	if((sub_ind-index_set)>2)index_set=sub_ind-2;
	else if(sub_ind<index_set)index_set=sub_ind;
	bgnd_par("     ����������     ",
			ptrs[index_set],
			ptrs[index_set+1],
			ptrs[index_set+2]);

	pointer_set(1);	 
	}    	

else if(ind==iK_net)
	{
	ptrs[0]=" U =     @�         ";
     ptrs[1]=" �����              ";
	ptrs[2]="                    ";
	
	if((sub_ind-index_set)>2)index_set=sub_ind-2;
	else if(sub_ind<index_set)index_set=sub_ind;
	bgnd_par("   ���������� ����  ",ptrs[index_set],ptrs[index_set+1],ptrs[index_set+2]);

	pointer_set(1);	
	int2lcd(net_U,'@',0);
	//int2lcdyx(net_buff_,3,10,0);
	
	//int2lcdyx(Kunet,3,16,0);
     }


else if(ind==iK_net3)
	{

	ptrs[0]=  		" U�A           !�   ";
    ptrs[1]=  		" U�B           @�   ";
    ptrs[2]=  	    " U�C           #�   ";
	ptrs[3]=  	    " �����              ";


	bgnd_par(		"   ���������� ����  ",
					ptrs[index_set],
					ptrs[index_set+1],
					ptrs[index_set+2]);

	if(sub_ind-index_set>2)index_set=sub_ind-2;
	else if (sub_ind<index_set)index_set=sub_ind;
	pointer_set(1);

    int2lcd(net_Ua,'!',0);
	int2lcd(net_Ub,'@',0);
	int2lcd(net_Uc,'#',0);

	/*int2lcdyx(KunetC,0,19,0);
	int2lcdyx(adc_buff_[10],0,13,0);
	int2lcdyx(KunetB,0,8,0);
	int2lcdyx(adc_buff_[3],0,4,0);*/

    }


else if(ind==iK_load)
	{
	ptrs[0]=" U =     @�         ";
     ptrs[1]=" �����              ";
	ptrs[2]="                    ";
	
	if((sub_ind-index_set)>2)index_set=sub_ind-2;
	else if(sub_ind<index_set)index_set=sub_ind;
	bgnd_par(		" ���������� ��������",
				ptrs[index_set],
				ptrs[index_set+1],
				ptrs[index_set+2]);

	pointer_set(1);
	if((load_U)>1000)int2lcd(load_U/10,'@',0);	
	else int2lcd(load_U,'@',1);
     }

else if(ind==iK_out)
	{
	ptrs[0]=" U����. =     @�    ";
    ptrs[1]=" U����  =     #�    ";
	ptrs[2]=" �����              ";
	ptrs[3]="                    ";
	
	if((sub_ind-index_set)>1)index_set=sub_ind-1;
	else if(sub_ind<index_set)index_set=sub_ind;
	bgnd_par(	"���������� �������� ",
				"     ����������     ",
				ptrs[index_set],
				ptrs[index_set+1]);

	pointer_set(2);
	
	if((bps_U)>1000)int2lcd(bps_U/10,'@',0);	
	else int2lcd(bps_U,'@',1);

	if((out_U)>1000)int2lcd(out_U/10,'#',0);	
	else int2lcd(out_U,'#',1);
    }

else if(ind==iK_t_ext)
	{
	ptrs[0]=  	" t����.����.    !�� ";
     ptrs[1]=  	" t����� ���     @�� ";
     ptrs[2]=  	" t����� MSAN    #�� ";
     ptrs[3]=	     " �����              ";
	ptrs[4]=	     "                    ";
	ptrs[5]=	     "                    ";
	
	if((sub_ind-index_set)>1)index_set=sub_ind-1;
	else if(sub_ind<index_set)index_set=sub_ind;
	bgnd_par(		" ���������� ������� ",
				" �������� ����������",
				ptrs[index_set],
				ptrs[index_set+1]);

	pointer_set(2);	
	if(ND_EXT[0])sub_bgnd("������.",'!',-3);
     else int2lcd_mmm(t_ext[0],'!',0);

	if(ND_EXT[1])sub_bgnd("������.",'@',-3);
     else int2lcd_mmm(t_ext[1],'@',0);

	if(ND_EXT[2])sub_bgnd("������.",'#',-3);
     else int2lcd_mmm(t_ext[2],'#',0);
     }

else if(ind==iK_t_ext_6U)
	{
	ptrs[0]=  		" t1             !�� ";
    ptrs[1]=  		" t2             @�� ";
    ptrs[2]=  		" t3             #�� ";
    ptrs[NUMDT]=	" �����              ";
	ptrs[NUMDT+1]=  "                    ";
	ptrs[NUMDT+2]=  "                    ";
	
	if((sub_ind-index_set)>1)index_set=sub_ind-1;
	else if(sub_ind<index_set)index_set=sub_ind;
	bgnd_par(		" ���������� ������� ",
				" �������� ����������",
				ptrs[index_set],
				ptrs[index_set+1]);

	pointer_set(2);	
	if(ND_EXT[0])sub_bgnd("������.",'!',-3);
     else int2lcd_mmm(t_ext[0],'!',0);

	if(ND_EXT[1])sub_bgnd("������.",'@',-3);
     else int2lcd_mmm(t_ext[1],'@',0);

	if(ND_EXT[2])sub_bgnd("������.",'#',-3);
     else int2lcd_mmm(t_ext[2],'#',0);
	//int2lcdyx(u_necc,3,18,0);
     }
     
else if(ind==iK_bat_sel)
	{
	ptrs[0]=						" ������� N1         ";
     ptrs[1]=						" ������� N2         ";
     if(BAT_IS_ON[0]!=bisON)ptrs[0]=	" ������� N2         ";
	ptrs[0+NUMBAT]=				" �����              ";
	ptrs[1+NUMBAT]=				"                    ";
	ptrs[2+NUMBAT]=				"                    ";

	if((sub_ind-index_set)>2)index_set=sub_ind-2;
	else if(sub_ind<index_set)index_set=sub_ind;
	bgnd_par(" ���������� ������� ",ptrs[index_set],ptrs[index_set+1],ptrs[index_set+2]);
	pointer_set(1);
	
     }     

else if(ind==iK_bat_sel_TELECORE)
	{
	ptrs[0]=						" ������� N1         ";
    ptrs[1]=						" ������� N2         ";
    ptrs[0+NUMBAT_TELECORE]=		" �����              ";
	ptrs[1+NUMBAT_TELECORE]=		"                    ";
	ptrs[2+NUMBAT_TELECORE]=		"                    ";

	if((sub_ind-index_set)>2)index_set=sub_ind-2;
	else if(sub_ind<index_set)index_set=sub_ind;
	bgnd_par(" ���������� ������� ",ptrs[index_set],ptrs[index_set+1],ptrs[index_set+2]);
	pointer_set(1);
	
     }  

else if(ind==iK_bat_TELECORE)
	{
    ptrs[0]=		" I��� =     #�      ";
    if(phase==0)
      	{
        ptrs[1]=	"   ������� � ���    ";
        ptrs[2]=	"���������� ���� I���";
        }
    else          
       	{
        ptrs[1]=	" ������������ I���  ";
        ptrs[2]=	"  �������� � ��� �  ";
        }
	ptrs[3]=		" �����              ";
    ptrs[4]=		"                    ";
    ptrs[5]=		"                    ";

	bgnd_par(		" ���������� ���. N! ",
				ptrs[index_set],
				ptrs[index_set+1],
				ptrs[index_set+2]);
     
   	if(sub_ind==0)
     	{
     	if(phase==0)
     		{
			mess_send(MESS2BPS_HNDL,PARAM_BPS_MASK_ON_OFF_AFTER_2SEC,0xffff,10);
     		mess_send(MESS2RELE_HNDL,PARAM_RELE_SAMOKALIBR,1,10);
			mess_send(MESS2BAT_HNDL,PARAM_BAT_MASK_OFF_AFTER_2SEC,(1<<sub_ind1),10);
     		//mess_send(MESS2BAT_HNDL1,PARAM_BAT_ON,(1<<(1-sub_ind1)),10);
     		}
    	else if(phase==1)
     		{
			mess_send(MESS2BPS_HNDL,PARAM_BPS_ALL_OFF_AFTER_2SEC,0xffff,10);
			mess_send(MESS2BAT_HNDL,PARAM_BAT_MASK_OFF_AFTER_2SEC,(1<<(1-sub_ind1)),10);
     		//mess_send(MESS2BAT_HNDL1,PARAM_BAT_ON,(1<<sub_ind1),10);
   			}
     		
     	}

	
	if((sub_ind==0)||(sub_ind==1)||(sub_ind==2))index_set=0;
	else index_set=3;

	pointer_set(1);	
	int2lcd(sub_ind1+1,'!',0);
	int2lcd_mmm(bat[sub_ind1]._Ib,'#',2);

	}  	

else if(ind==iK_bat_ips_termokompensat_ib)
	{
     ptrs[0]=		" I��� =     #�      ";
     ptrs[1]=		" ������������ I���  ";
     ptrs[2]=		"  �������� � ��� �  ";
     ptrs[3]=		" �����              ";
     ptrs[4]=		"                    ";
     ptrs[5]=		"                    ";

	bgnd_par(		" ���������� ���. N1 ",
				ptrs[index_set],
				ptrs[index_set+1],
				ptrs[index_set+2]);
     

	if((sub_ind==0)||(sub_ind==1)||(sub_ind==2))index_set=0;
	else if((sub_ind==3)||(sub_ind==4)||(sub_ind==5))index_set=3;
	


	pointer_set(1);	
	
	if(bIBAT_SMKLBR)sub_bgnd("����. ",'#',-4);
	else int2lcd_mmm(Ib_ips_termokompensat,'#',2);

	
	}  	

  

else if(ind==iInv_set)
	{
	ptrs[0]=						" ����������� ����.  ";
     ptrs[1]=						" ������          <� ";
     ptrs[2]=						" ������������ ����. ";
	ptrs[3]=						" ������          >� ";
	ptrs[4]=					  	" �����              ";
	ptrs[5]=						"                    ";
	ptrs[6]=						"                    ";

	if((sub_ind-index_set)>2)index_set=sub_ind-2;
	else if(sub_ind<index_set)index_set=sub_ind;
	bgnd_par("    �������� N!     ",ptrs[index_set],ptrs[index_set+1],ptrs[index_set+2]);
	pointer_set(1);
	
	int2lcd(sub_ind1+1,'!',0);
	int2lcd(inv[sub_ind1]._Uoutmin,'<',0);
	int2lcd(inv[sub_ind1]._Uoutmax,'>',0);
	}     

else if(ind==iK_makb_sel)
	{
	ptrs[0]=						" ������� ��� N1     ";
     ptrs[1]=						" ������� ��� N2     ";
     ptrs[2]=						" ������� ��� N3     ";
	ptrs[3]=						" ������� ��� N4     ";
	ptrs[NUMMAKB]=					" �����              ";
	ptrs[1+NUMMAKB]=				"                    ";
	ptrs[2+NUMMAKB]=				"                    ";

	if((sub_ind-index_set)>2)index_set=sub_ind-2;
	else if(sub_ind<index_set)index_set=sub_ind;
	bgnd_par("����������� ��������",ptrs[index_set],ptrs[index_set+1],ptrs[index_set+2]);
	pointer_set(1);
	
     }     

else if(ind==iK_makb)
	{
	ptrs[0]=						" U1  =    !�        ";
	ptrs[1]=						" U2  =    @�        ";
	ptrs[2]=						" U3  =    #�        ";
	ptrs[3]=						" U4  =    $�        ";
	ptrs[4]=						" U5  =    %�        ";
	ptrs[5]=						" t1  =    ^�C       ";
	ptrs[6]=						" t2  =    &�C       ";
	ptrs[7]=						" t3  =    *�C       ";
	ptrs[8]=						" t4  =    (�C       ";
	ptrs[9]=						" t5  =    )�C       ";
	ptrs[10]=						" �����              ";
	ptrs[11]=						"                    ";

	if((sub_ind-index_set)>1)index_set=sub_ind-1;
	else if(sub_ind<index_set)index_set=sub_ind;
	bgnd_par("     ����������     ","   ������� ��� N<   ",ptrs[index_set],ptrs[index_set+1]);
	pointer_set(2);
	simax=10;

	int2lcd(sub_ind1+1,'<',0);
	int2lcd(makb[sub_ind1]._U[0],'!',1);
	int2lcd(makb[sub_ind1]._U[1],'@',1);
	int2lcd(makb[sub_ind1]._U[2],'#',1);
	int2lcd(makb[sub_ind1]._U[3],'$',1);
	int2lcd(makb[sub_ind1]._U[4],'%',1);

	if(makb[sub_ind1]._T_nd[0])sub_bgnd("�����������",'^',-5);
	else int2lcd_mmm(makb[sub_ind1]._T[0],'^',0); 
	if(makb[sub_ind1]._T_nd[1])sub_bgnd("�����������",'&',-5);
	else int2lcd_mmm(makb[sub_ind1]._T[1],'&',0); 
	if(makb[sub_ind1]._T_nd[2])sub_bgnd("�����������",'*',-5);
	else int2lcd_mmm(makb[sub_ind1]._T[2],'*',0); 
	if(makb[sub_ind1]._T_nd[3])sub_bgnd("�����������",'(',-5);
	else int2lcd_mmm(makb[sub_ind1]._T[3],'(',0); 
	if(makb[sub_ind1]._T_nd[4])sub_bgnd("�����������",')',-5);
	else int2lcd_mmm(makb[sub_ind1]._T[4],')',0); 


/*	int2lcd(makb[sub_ind1]._T[0],'^',0);
	int2lcd(makb[sub_ind1]._T[1],'&',0);
	int2lcd(makb[sub_ind1]._T[2],'*',0);
	int2lcd(makb[sub_ind1]._T[3],'(',0);
	int2lcd(makb[sub_ind1]._T[4],')',0);*/
	
     }   

else if(ind==iK_bps_sel)
	{
	ptrs[0]=						" ��� N1             ";
    ptrs[1]=						" ��� N2             ";
    ptrs[2]=						" ��� N3             ";
	ptrs[3]=						" ��� N4             ";
    ptrs[4]=						" ��� N5             ";
    ptrs[5]=						" ��� N6             ";
	ptrs[6]=						" ��� N7             ";
    ptrs[7]=						" ��� N8             ";
    ptrs[8]=						" ��� N9             ";
	ptrs[9]=						" ��� N10            ";
    ptrs[10]=						" ��� N11            ";
    ptrs[11]=						" ��� N12            ";
    ptrs[12]=						" ��� N13            ";
	ptrs[13]=						" ��� N14            ";
    ptrs[14]=						" ��� N15            ";
    ptrs[15]=						" ��� N16            ";
	ptrs[16]=						" ��� N17            ";
    ptrs[17]=						" ��� N18            ";
    ptrs[18]=						" ��� N19            ";
	ptrs[19]=						" ��� N20            ";
    ptrs[20]=						" ��� N21            ";
    ptrs[21]=						" ��� N22            ";	               
	ptrs[NUMIST]=					" �����              ";
	ptrs[1+NUMIST]=				"                    ";
	ptrs[2+NUMIST]=				"                    ";

	if((sub_ind-index_set)>2)index_set=sub_ind-2;
	else if(sub_ind<index_set)index_set=sub_ind;
	bgnd_par("  ���������� �����  ",ptrs[index_set],ptrs[index_set+1],ptrs[index_set+2]);
	pointer_set(1);
	
     }     

else if(ind==iK_bps)
	{
	
	ptrs[0]=" U��� =    @�       ";
    ptrs[1]=" ������������ U���  ";
	ptrs[2]="  �������� � ��� �  "; 
	ptrs[3]=" U���� =   #�       ";
    ptrs[4]=" ������������ U���� ";
    ptrs[5]="  �������� � ��� �  ";
	ptrs[6]=" U����� =   $�      ";
	if(bFL_)
		{
		ptrs[7]=" ���������� U�����  ";
     	ptrs[8]="  �������� � ��� �  ";
     	}
    else 
     	{
		ptrs[7]=" ����������� � ���  ";
     	ptrs[8]="    �����������     ";     	
     	}	
	ptrs[9]=" I��� =     %�      ";
	if(phase==0)
    	{
        ptrs[10]=	"   ������� � ���    ";
        ptrs[11]=	"���������� ���� I���";
        }
    else
     	{
        ptrs[10]=" ������������ I���  ";
        ptrs[11]="  �������� � ��� �  ";     	
     	} 
     	
    ptrs[12]=" t��� =   ^�C       ";    
	ptrs[13]=" ������������ t���  ";
    ptrs[14]="  �������� � ��� �  ";
	ptrs[15]=" U���  =    &�      ";
    ptrs[16]=" ������������ U���  ";
	ptrs[17]="  �������� � ��� �  ";
    ptrs[18]=sm_exit;
    ptrs[19]=sm_;
    ptrs[20]=sm_;     	     	    
	

    if((sub_ind==0)||(sub_ind==1)||(sub_ind==2))index_set=0;
	else if((sub_ind==3)||(sub_ind==4)||(sub_ind==5))index_set=3;
	else if((sub_ind==6)||(sub_ind==7)||(sub_ind==8))index_set=6;
	else if((sub_ind==9)||(sub_ind==10)||(sub_ind==11))index_set=9;
	else if((sub_ind==12)||(sub_ind==13)||(sub_ind==14))index_set=12;
	else if((sub_ind==15)||(sub_ind==16)||(sub_ind==17))index_set=15;	
	else index_set=18;
	
	bgnd_par(" ���������� ��� N! ",ptrs[index_set],ptrs[index_set+1],ptrs[index_set+2]);

	pointer_set(1);	
	int2lcd(sub_ind1+1,'!',0);
	int2lcd(bps[sub_ind1]._Uii,'@',1);
	int2lcd(bps[sub_ind1]._Uin,'#',1);
	int2lcd(U_AVT,'$',1);
	int2lcd(bps[sub_ind1]._Ii,'%',1);
	int2lcd(bps[sub_ind1]._Ti,'^',0); 
	int2lcd(bps[sub_ind1]._Uisum,'&',1);
	 
	
    if((sub_ind==0)||(sub_ind==3))
		{
		mess_send(MESS2BPS_HNDL,PARAM_BPS_MASK_ON_OFF_AFTER_2SEC,(1<<sub_ind1),10);
		mess_send(MESS2BAT_HNDL,PARAM_BAT_ALL_OFF_AFTER_2SEC,0,10);
	    mess_send(MESS2CNTRL_HNDL,PARAM_CNTRL_STAT_SET,2000,10);
        }
  	if(sub_ind==6)
		{
        mess_send(MESS2BPS_HNDL,PARAM_BPS_MASK_ON_OFF_AFTER_2SEC,(1<<sub_ind1),10);
        mess_send(MESS2BAT_HNDL,PARAM_BAT_ALL_OFF_AFTER_2SEC,0,40);
        mess_send(MESS2UNECC_HNDL,PARAM_UNECC_SET,U_AVT,10);
	   	mess_send(MESS2CNTRL_HNDL,PARAM_CNTRL_STAT_FAST_REG,0,10);
		}

    if(sub_ind==9)
		{
		if(phase==0)
			{
          	mess_send(MESS2BPS_HNDL,PARAM_BPS_MASK_ON_OFF_AFTER_2SEC,~(1<<sub_ind1),10);
          	}
      	else if(phase==1)
			{
          	mess_send(MESS2BPS_HNDL,PARAM_BPS_MASK_ON_OFF_AFTER_2SEC,(1<<sub_ind1),10);
			mess_send(MESS2BAT_HNDL,PARAM_BAT_ALL_OFF_AFTER_2SEC,0,10);
          	}
       	mess_send(MESS2CNTRL_HNDL,PARAM_CNTRL_STAT_SET,2000,10);
        }
	
    if(sub_ind==12)
		{
        }	

    if(sub_ind==15)
		{
		mess_send(MESS2BPS_HNDL,PARAM_BPS_MASK_ON_OFF_AFTER_2SEC,(1<<sub_ind1),10);
		mess_send(MESS2BAT_HNDL,PARAM_BAT_ALL_OFF_AFTER_2SEC,0,10);
	    mess_send(MESS2CNTRL_HNDL,PARAM_CNTRL_STAT_SET,2000,10);
        }          
          
	if(mess_find( (MESS2IND_HNDL)) && (mess_data[0]==PARAM_U_AVT_GOOD) )
		{
		show_mess("     ���������      ",
	          	"    ����������      ",
	          	" ���������� ������  ",
	          	"    �����������     ",3000);
		}	     
	     
	//MSG_IND2PWM_SRC1=900;
	//MSG_IND2PWM_SRC2=900;         
/*int2lcdyx(sub_ind1,0,0,0);
int2lcdyx(sub_ind,0,1,0);
int2lcdyx(phase,0,2,0);
int2lcdyx(MSG_IND2OUT_DIS_SRC1,0,3,0);
int2lcdyx(MSG_IND2OUT_DIS_SRC2,0,4,0);  
int2lcdyx(MSG_IND2OUT_EN_SRC1,0,5,0);
int2lcdyx(MSG_IND2OUT_EN_SRC2,0,6,0); */

//int2lcdyx(cntrl_stat1,0,19,0); 
//int2lcdyx(load_U,0,5,0); 
//int2lcdyx(cntrl_stat,0,10,0); 
//int2lcdyx(bps[sub_ind1]._rotor,0,19,0); 
//int2lcdyx(u_necc,0,19,0);  
	 }


else if(ind==iRele_set)
	{
	ptrs[0]=				" ���� N1            ";
    ptrs[1]=				" ���� N2            ";
    ptrs[2]=				" ���� N3            ";
	ptrs[3]=				" ���� N4            ";
	ptrs[4]=				" �����              ";
	ptrs[5]=				"                    ";
	ptrs[6]=				"                    ";

	if((sub_ind-index_set)>2)index_set=sub_ind-2;
	else if(sub_ind<index_set)index_set=sub_ind;
	bgnd_par(	"   ��������� ����   ",
				ptrs[index_set],
				ptrs[index_set+1],
				ptrs[index_set+2]);
	pointer_set(1);
	
     }     

else if(ind==iRele_set_)
	{
	
	ptrs[0]=	" ������ ������     @";
    ptrs[1]=	" ��������� �������  ";
	ptrs[2]=	"  �� ����          #"; 
	ptrs[3]=	" �������� �������  $";
    ptrs[4]=	" U���. ��������    %";
    ptrs[5]=	" U���. ��������    ^";
	ptrs[6]=	" U��. ��������     [";
	ptrs[7]=	" U��. ��������     ]";
	ptrs[8]=	" ������ ����������� ";
	ptrs[9]=	"  ������������     {";
	ptrs[10]=	" U�.�. > 2�        }";
    ptrs[11]=	sm_exit;
    ptrs[12]=	sm_;
    ptrs[13]=	sm_;     	     	    
	

	if((sub_ind-index_set)>2)index_set=sub_ind-2;
	else if(sub_ind<index_set)index_set=sub_ind;
	
	bgnd_par(	" ���� N! ��������. ",
				ptrs[index_set],
				ptrs[index_set+1],
				ptrs[index_set+2]);

	pointer_set(1);	

	int2lcd(sub_ind1+1,'!',0); 
 	checkboxing('@',RELE_SET_MASK[sub_ind1]&(1<<0));
	checkboxing('#',RELE_SET_MASK[sub_ind1]&(1<<1));
 	checkboxing('$',RELE_SET_MASK[sub_ind1]&(1<<2));
	checkboxing('%',RELE_SET_MASK[sub_ind1]&(1<<3));
 	checkboxing('^',RELE_SET_MASK[sub_ind1]&(1<<4));
	checkboxing('[',RELE_SET_MASK[sub_ind1]&(1<<5));
 	checkboxing(']',RELE_SET_MASK[sub_ind1]&(1<<6));
	checkboxing('{',RELE_SET_MASK[sub_ind1]&(1<<7));
	checkboxing('}',RELE_SET_MASK[sub_ind1]&(1<<8));
	
	//int2lcdyx(lc640_read_int(ADR_EE_RELE_SET_MASK[sub_ind1]),0,19,0);
	//int2lcdyx(RELE_SET_MASK[sub_ind1],0,12,0);
	//int2lcdyx(sub_ind1,0,2,0);  
	}


else if(ind==iK_power_net)
	{
	ptrs[0]=" U����=    @�       ";
	ptrs[1]=" U��� =    #�       ";
     ptrs[2]=" �����              ";
	ptrs[3]="                    ";
	
	if((sub_ind-index_set)>1)index_set=sub_ind-1;
	else if(sub_ind<index_set)index_set=sub_ind;
	bgnd_par( "      ����������    ",
               "   ������� ������   ",
               ptrs[index_set],
               ptrs[index_set+1]);

	pointer_set(2);	
	int2lcd(Uvv[0],'@',0);
     int2lcd(Uvv[1],'#',0);
	//int2lcdyx(net_buff_,3,10,0);
	
	//int2lcdyx(Kunet,3,16,0);
     }


else if(ind==iK_power_net3)
	{
     ptrs[0]=  		" ���� �.A    !�     ";
	ptrs[1]=  		" ���� �.B    @�     ";
	ptrs[2]=  		" ���� �.C    #�     ";
     ptrs[3]=  	     " ���  �.A    &�     ";
     ptrs[4]=  	     " ���  �.B    *�     ";
     ptrs[5]=  	     " ���  �.C    (�     ";		            
     ptrs[6]=" �����              ";
	ptrs[7]="                    ";
	
	if((sub_ind-index_set)>1)index_set=sub_ind-1;
	else if(sub_ind<index_set)index_set=sub_ind;
	bgnd_par( "      ����������    ",
               "   ������� ������   ",
               ptrs[index_set],
               ptrs[index_set+1]);

	pointer_set(2);	
	int2lcd(Uvv_eb2[0],'!',0);
	int2lcd(Uvv_eb2[1],'@',0);
	int2lcd(Uvv_eb2[2],'#',0);
	int2lcd(Upes_eb2[0],'&',0);
	int2lcd(Upes_eb2[1],'*',0);
	int2lcd(Upes_eb2[2],'(',0);
	//int2lcdyx(net_buff_,3,10,0);
	
	//int2lcdyx(Kunet,3,16,0);
     }


#endif	 
			
if(ind==iDeb)
     {
     if(sub_ind==0)
     	{


        bgnd_par("*0000*000000*       ",
     	         "                    ",
     	         "                    ",
     	         "      ********      ");

		int2lcdyx(SOFT_NUM,0,4,0);
		long2lcdyx_mmm(SOFT_DATE,0,11,0);
	
		int2lcdyx(numOfForvardBps_minCnt,1,5,0);
		int2lcdyx(numOfForvardBps_hourCnt,1,10,0);
		int2lcdyx(numOfForvardBps,1,15,0);
	
		int2lcdyx(numOfForvardBps_minCnt,2,4,0);
		int2lcdyx(numOfForvardBps_hourCnt,2,9,0);
		int2lcdyx(cntrl_stat,2,19,0);

      

     	}     

    	else if(sub_ind==1) 
     	{
     	bgnd_par("�                   ",
     	         "                    ",
     	         "                    ",
     	         "                    ");

		//int2lcdyx(bAVG,0,0,0);
		//int2lcdyx(LPC_CAN1->GSR,0,6,0);
		//int2lcdyx((LPC_CAN1->GSR)>>16,0,16,0);
		//int2lcdyx(avg,0,19,0);

				//int2lcdyx((((LPC_CAN1->GSR)&(0xff000000))>>24),0,19,0);
		//int2lcdyx((((LPC_CAN1->GSR)&(0x00ff0000))>>16),0,15,0);

 /*         int2lcdyx(bat[0]._Ubm,1,7,0); 	int2lcdyx(bat[0]._av,1,10,0);
		int2lcdyx(bat[0]._dUbm,2,7,0);
		int2lcdyx(bat[0]._cnt_as,3,7,0);
		
 
		int2lcdyx(bat[1]._Ub,0,14,0);
          int2lcdyx(bat[1]._Ubm,1,14,0);	int2lcdyx(bat[1]._av,1,17,0);
		int2lcdyx(bat[1]._dUbm,2,14,0);
		int2lcdyx(bat[1]._cnt_as,3,14,0);*/

		int2lcdyx(sub_ind1+0,1,0,0);
		int2lcdyx(sub_ind1+1,2,0,0);
		int2lcdyx(sub_ind1+2,3,0,0);
		
		
		int2lcdyx(bps[sub_ind1  ]._cnt,1,2,0);
		int2lcdyx(bps[sub_ind1+1]._cnt,2,2,0);
		int2lcdyx(bps[sub_ind1+2]._cnt,3,2,0);		
		
	/*	int2lcdyx(bps[sub_ind1  ]._ist_blok_cnt,1,5,0);
		int2lcdyx(bps[sub_ind1+1]._ist_blok_cnt,2,5,0);
		int2lcdyx(bps[sub_ind1+2]._ist_blok_cnt,3,5,0);*/			
		
	/*	char2lcdhyx(bps[sub_ind1  ]._flags_tu,1,8);
		char2lcdhyx(bps[sub_ind1+1]._flags_tu,2,8);
		char2lcdhyx(bps[sub_ind1+2]._flags_tu,3,8);

		int2lcdyx(bps[sub_ind1  ]._vol_u,1,12,0);
		int2lcdyx(bps[sub_ind1+1]._vol_u,2,12,0);
		int2lcdyx(bps[sub_ind1+2]._vol_u,3,12,0);		


		char2lcdhyx(bps[sub_ind1]._flags_tm,1,15);
		char2lcdhyx(bps[sub_ind1+1]._flags_tm,2,15);
		char2lcdhyx(bps[sub_ind1+2]._flags_tm,3,15);	
		*/
		int2lcdyx(bps[sub_ind1]._Ii,1,15,0);
		int2lcdyx(bps[sub_ind1+1]._Ii,2,15,0);
		int2lcdyx(bps[sub_ind1+2]._Ii,3,15,0);
	/*
		char2lcdhyx(bps[sub_ind1]._rotor>>8,1,15);
		char2lcdhyx(bps[sub_ind1+1]._rotor>>8,2,15);
		char2lcdhyx(bps[sub_ind1+2]._rotor>>8,3,15);		
		*/
		
		int2lcdyx(bps[sub_ind1]._rotor,1,19,0);
		int2lcdyx(bps[sub_ind1+1]._rotor,2,19,0);
		int2lcdyx(bps[sub_ind1+2]._rotor,3,19,0);


 		}

 

    else if(sub_ind==2)
     	{
     	bgnd_par(	"F                   ",
     		    	"                    ",
     		    	"                    ",
     		    	"                    ");


		int2lcdyx(uout_av,1,5,0);
		int2lcdyx(USIGN,2,5,0); 

		int2lcdyx(bSILENT,3,5,0);

		
		
		int2lcdyx(U_OUT_KONTR_MAX,0,19,0);
		int2lcdyx(load_U,1,19,0);
		int2lcdyx(U_OUT_KONTR_MIN,2,19,0);
		//int2lcdyx(U_OUT_KONTR_DELAY,3,19,0);
		int2lcdyx(outVoltContrHndlCnt,3,19,0);

		long2lcdhyx(0x12345678UL,1,14);
		long2lcdhyx(avar_stat,2,14);
		long2lcdhyx(avar_ind_stat,3,14);
		}  

	else if(sub_ind==3)
     	{
     	bgnd_par("��                  ",
     	         "                    ",
     	         "                   ^",
     	         "                   &");

/*	int2lcdyx(spc_stat,0,5,0);
	int2lcdyx(__ee_spc_stat,0,9,0);
	int2lcdyx(lc640_read_int(EE_SPC_STAT),0,13,0);

	int2lcdyx(spc_bat,1,5,0);
	int2lcdyx(__ee_spc_bat,1,9,0);
	int2lcdyx(lc640_read_int(EE_SPC_BAT),1,13,0);

	int2lcdyx(bat_u_old_cnt,0,19,0);
	
	
	int2lcdyx(bat[0]._zar_cnt_ke,2,5,0);
	int2lcdyx(lc640_read_int(ADR_EE_BAT_ZAR_CNT_KE[0]),2,10,0);	
	int2lcdyx(bat[0]._u_old[0],2,14,0);
	int2lcd_mmm(bat[0]._Ib,'^',2);

	int2lcdyx(bat[1]._zar_cnt_ke,3,5,0);
	int2lcdyx(lc640_read_int(ADR_EE_BAT_ZAR_CNT_KE[1]),3,10,0);
	int2lcdyx(bat[1]._Ub,3,14,0);
	int2lcd_mmm(bat[1]._Ib,'&',2);	

	int2lcdyx(spc_phase,1,15,0);
	int2lcdyx(__ee_spc_phase,1,17,0);
	int2lcdyx(lc640_read_int(EE_SPC_PHASE),1,19,0);	*/

/*	int2lcdyx(speedChIsOn,0,4,0);
	int2lcdyx(speedChTimeCnt,1,4,0);
	int2lcdyx(speedChrgBlckStat,2,4,0);
	int2lcdyx(speedChrgBlckSrc,3,4,0);

	int2lcdyx(speedChrgBlckSrc,0,8,0);
	int2lcdyx(speedChrgBlckLog,1,8,0); */
	
	
		

/*	    		int2lcdyx(adc_net_buff_cnt,0,4,0);

		    	int2lcdyx((short)(main_power_buffer[0]>>12),0,19,0);
			int2lcdyx((short)(main_power_buffer[1]>>12),1,19,0);
			int2lcdyx((short)(main_power_buffer[2]>>12),2,19,0);
			int2lcdyx((short)(main_power_buffer[3]>>12),3,19,0);

		    	int2lcdyx((net_buff_),2,5,0); */


		   
		    


/*		int2lcdyx(load_U,0,4,0);
		int2lcdyx(load_I,1,4,0);
		lcd_buffer[44]='a';
		int2lcd_mmm((bat[0]._Ib)/10,'a',1);
		lcd_buffer[64]='a';
		int2lcd_mmm((bat[1]._Ib)/10,'a',1);

 		int2lcdyx(u_necc,0,8,0);

		
		
		lcd_buffer[14]='.';
		lcd_buffer[34]='.';
		int2lcdyx(Isumm,0,15,1);		
		int2lcdyx(Isumm_,1,15,1);


		int2lcdyx(cntrl_stat,0,19,0);
		int2lcdyx(num_necc,1,19,0);
		
		
		  
//		int2lcdyx(cntrl_stat,0,15,0);
		 
		//int2lcdyx(cntrl_plazma,1,3,0);
		//lcd_buffer[30]='a';
		int2lcd_mmm(Ibmax,'a',0);
		int2lcdyx(IZMAX,1,14,0);

		lcd_buffer[65]='a';
		int2lcd_mmm(bat[0]._Ib,'a',0);

		lcd_buffer[70]='a';
		int2lcd_mmm(bat[1]._Ib,'a',0); 

		lcd_buffer[75]='a';
		int2lcd_mmm(Ibmax,'a',0); 

	//	int2lcdyx(IMAX,2,3,0);
		
		

	//	int2lcdyx(IZMAX,3,19,0);

		//int2lcdyx(num_necc_Imax,3,6,0);
		//int2lcdyx(num_necc_Imin,3,12,0);


 //    	lcd_buffer[4]='a';            
 //    	int2lcd_mmm(Ibat,'a',1);   int2lcdyx(cntrl_stat,0,9,0);          int2lcdyx(hour_apv_cnt,0,13,0);                             char2lcdhyx(St_[0],0,19);  
 //    	int2lcdyx(Ubat,1,4,0);     int2lcdyx(main_apv_cnt,1,9,0);        int2lcdyx(lc640_read_int(bps1_AVAR_PTR),1,13,0);            char2lcdhyx(St_[1],1,19);
 //    	int2lcdyx(Us[0],2,4,0);  int2lcdyx(apv_cnt_1,2,9,0);           int2lcdyx(lc640_read_int(SRC1_AVAR_CNT),2,13,0);                                     int2lcdhyx(av_stat,2,19);
 //    	int2lcdyx(Us[1],3,4,0);  int2lcdyx(reset_apv_cnt,3,9,0);                                            int2lcdyx(plazma,3,19,0);
     	//int2lcd(plazma,'(',0);

     	//int2lcd(Us[0],'#',1);
     	//int2lcd(Us[1],'$',1);
     	//int2lcd(Is[0],'%',1);
     	//int2lcd(Is[1],'^',1);
    // 	int2lcd(bat[0]._Ub,'<',1);
    // 	int2lcd_mmm(bat[0]._Ib,'>',2);
 //    	char2lcdhyx(St_[0],3,13);
 //    	char2lcdhyx(St_[1],3,19);
 //    	char2lcdhyx(St,3,5);  */
		}

   else if(sub_ind==4)
     	{
     	bgnd_par(	"LB                  ",
     		    	"                    ",
     		    	"      !   #         ",
     		    	"      @   $         ");


     	int2lcdyx(NUMBAT_TELECORE,0,1,0);
		
		int2lcdyx(bps[0]._Ii,0,5,0);
		int2lcdyx(bps[1]._Ii,0,9,0);
		
		int2lcdyx(plazma_cntrl_stat,0,19,0);
		
		int2lcdyx(Ubpsmax,1,3,0);
		int2lcdyx(Ibmax,1,7,0);
		int2lcdyx(cntrl_stat,1,11,0);
		int2lcdyx(load_U,1,19,0);
			
		int2lcdyx(lakb[0]._balanced_event_code/*_tot_bat_volt*/,2,2,0);
	   	int2lcdyx(lakb[1]._balanced_event_code/*_tot_bat_volt*/,3,2,0);
		
	 	int2lcd_mmm(lakb[0]._ch_curr/10,'!',0);
		int2lcd_mmm(lakb[1]._ch_curr/10,'@',0);		
		
	 	int2lcd_mmm(bat[0]._Ib/10,'#',0);
		int2lcd_mmm(bat[1]._Ib/10,'$',0);

//		int2lcdyx(TELECORE2017_ULINECC,2,19,0);
//	   	int2lcdyx(TELECORE2017_ULINECC_,3,19,0);

		//int2lcdyx(lakb[0]._communicationFullErrorStat,2,19,0);
	   	//int2lcdyx(lakb[1]._communicationFullErrorStat,3,19,0);

		int2lcdyx(IMAX,0,13,0);
		
		int2lcdyx(lakb[0]._voltage_event_code,2,15,0); 
		int2lcdyx(lakb[1]._voltage_event_code,3,15,0);



		//int2lcdyx(load_I,2,17,0); 
		//int2lcdyx(Isumm,3,17,0);

		//int2lcdyx(lakb[0]._s_o_c_percent,2,17,0); 
		//int2lcdyx(lakb[1]._s_o_c_percent,3,17,0);


		




		
		//int2lcdyx(u_necc,1,15,0);
		

		//int2lcdyx(plazma_ztt[0],1,13,0);lakb[i]._communicationFullErrorStat
		//int2lcdyx(plazma_ztt[1],1,17,0);
 		//int2lcdyx(zTTBatteryHndlCmnd/*zTTBatteryHndlPhase*/,0,5,0);

		//int2lcdyx(zTTButteryCnter,0,7,0);

		//int2lcdyx(numOfPacks,0,9,0);

		//int2lcdyx(numOfTemperCells,0,12,0);
		//t2lcdyx(cntrl_stat_blck_cnt,0,6,0);
		 
		//int2lcdyx(ch_cnt0,0,13,0);
		//int2lcdyx(ch_cnt1,0,16,0);
		//int2lcdyx(bat[0]._Ib,0,9,0);
		//int2lcdyx(bat[1]._Ib,0,15,0);
		}
/*	else if(sub_ind==4)
     	{
     	bgnd_par(" ������             ",
     	         "                    ",
     	         "                    ",
     	         "                    ");

		int2lcdyx(main_10Hz_cnt,0,7,0);
		int2lcdyx(bat[0]._av,0,10,0);
		int2lcdyx(bat[1]._av,0,12,0);
		char2lcdhyx(rele_stat,0,19);

 		long2lcdhyx(avar_stat,1,7);
		long2lcdhyx(avar_stat_old,2,7);
		long2lcdhyx(avar_ind_stat,3,7);

		long2lcdhyx(avar_stat_new,2,19);
		long2lcdhyx(avar_stat_offed,3,19);



		}*/
 
    else if(sub_ind==5)
     	{
     	bgnd_par(	"*                   ",
     		    	"                    ",
     		    	"                    ",
     		    	"                    ");

		int2lcdyx(adc_buff_[0],0,4,0);
		int2lcdyx(adc_buff_[1],1,4,0);
		int2lcdyx(adc_buff_[2],2,4,0);
		int2lcdyx(adc_buff_[3],3,4,0);
		int2lcdyx(adc_buff_[4],0,9,0);
		int2lcdyx(adc_buff_[5],1,9,0);
		int2lcdyx(adc_buff_[6],2,9,0);
		int2lcdyx(adc_buff_[7],3,9,0);
		int2lcdyx(adc_buff_[8],0,14,0);
		int2lcdyx(adc_buff_[9],1,14,0);
		int2lcdyx(adc_buff_[10],2,14,0);
		int2lcdyx(adc_buff_[11],3,14,0);
		int2lcdyx(adc_buff_[12],0,19,0);
		int2lcdyx(adc_buff_[13],1,19,0);
		int2lcdyx(adc_buff_[14],2,19,0);
		int2lcdyx(adc_buff_[15],3,19,0);



	/*	int2lcdyx(main_kb_cnt,0,3,0);
		int2lcdyx(cntrl_stat,1,3,0);
		int2lcdyx(ibat_ips,2,3,0);
		int2lcdyx(ibat_ips_,3,3,0);

		int2lcdyx(kb_cnt_1lev,0,19,0);
		int2lcdyx(kb_cnt_2lev,1,19,0);
		int2lcdyx(kb_full_ver,2,19,0);

		int2lcdyx(Ibmax,0,16,0);
		int2lcdyx(IZMAX_,1,16,0);
		int2lcdyx(cntrl_hndl_plazma,2,16,0); */

		
    	}  		  		


  else if(sub_ind==6)
     	{
     	bgnd_par(	"6                   ",
     		    	"    !     $         ",
     		    	"    @     %         ",
     		    	"            ^       ");
#ifdef UKU_TELECORE2017     		    
/*		int2lcd_mmm(bat[0]._Ib,'!',2);
		int2lcd_mmm(bat[1]._Ib,'@',2);
		int2lcd_mmm(bps[0]._Ii,'$',1);
		int2lcd_mmm(bps[1]._Ii,'%',1);
		int2lcd_mmm(bps[2]._Ii,'^',1); */

		int2lcd_mmm(t_ext[0],'!',0);
		int2lcd_mmm(t_ext[1],'@',0);
		int2lcd_mmm(t_box_warm,'$',0);
		int2lcd_mmm(t_box_vent,'%',0);
		//int2lcdyx(load_I,3,7,0);
		//int2lcdyx(t_box_vent,2,7,0);

		int2lcdyx(warm_stat_k,1,19,0);
		int2lcdyx(vent_stat_k,2,19,0);
		

		int2lcdyx(TELECORE2017_KLIMAT_WARM_ON_temp,3,3,0);
		int2lcdyx(TELECORE2017_KLIMAT_WARM_ON,3,7,0);
		int2lcdyx(TELECORE2017_KLIMAT_WARM_OFF,3,11,0);
		int2lcdyx(TELECORE2017_INT_VENT_PWM,3,15,0);
		int2lcdyx(TELECORE2017_EXT_VENT_PWM,3,19,0);
		
		
		
		//int2lcdyx(li_bat._Tb,3,3,0);
		
		


		int2lcdyx(t_box_warm_on_cnt,1,17,0);
		int2lcdyx(t_box_vent_on_cnt,2,17,0);
//			//int2lcdyx(t_box_vent_on_cnt,0,3,0);
	//int2lcdyx(t_box_warm_on_cnt,0,7,0);
//		int2lcdyx(vent_stat_k,2,15,0);

		
		int2lcdyx(ND_EXT[0],0,9,0);
		int2lcdyx(ND_EXT[1],0,13,0);
		//int2lcdyx(TELECORE2017_KLIMAT_VENT_ON20,0,16,0);	 
		int2lcdyx(lakb[0]._zar_percent,0,19,0);	 
#endif
    	}


   else if(sub_ind==7)
     	{
     	bgnd_par("7                   ",
     		    "                    ",
     		    "                    ",
     		    "                    ");
     		    
		int2lcdyx(adc_buff_[sk_buff_TELECORE2015[0]],0,19,0);
		int2lcdyx(adc_buff_[sk_buff_TELECORE2015[1]],1,19,0);
		int2lcdyx(adc_buff_[sk_buff_TELECORE2015[2]],2,19,0);
		int2lcdyx(adc_buff_[sk_buff_TELECORE2015[3]],3,19,0);
		int2lcdyx(sk_cnt[0],0,14,0);
		int2lcdyx(sk_cnt[1],1,14,0);
		int2lcdyx(sk_cnt[2],2,14,0);
		int2lcdyx(sk_cnt[3],3,14,0);
		int2lcdyx(sk_stat[0],0,10,0);
		int2lcdyx(sk_stat[1],1,10,0);
		int2lcdyx(sk_stat[2],2,10,0);
		int2lcdyx(sk_stat[3],3,10,0);

    		}
    else if(sub_ind==8)
     	{
     	bgnd_par("                    ",
     		    "                    ",
     		    "                    ",
     		    "                    ");
     	int2lcdyx(ibt._T[0],0,2,0);
		int2lcdyx(ibt._T[1],1,2,0);
     	int2lcdyx(ibt._T[2],2,2,0);
		int2lcdyx(ibt._T[3],3,2,0);
		
     	int2lcdyx(ibt._nd[0],0,4,0);
		int2lcdyx(ibt._nd[1],1,4,0);
     	int2lcdyx(ibt._nd[2],2,4,0);
		int2lcdyx(ibt._nd[3],3,4,0);	    

     	int2lcdyx(ibt._T_dispers[0],0,7,0);
		int2lcdyx(ibt._T_dispers[1],1,7,0);
     	int2lcdyx(ibt._T_dispers[2],2,7,0);
		int2lcdyx(ibt._T_dispers[3],3,7,0);
			    
		int2lcdyx(ibt._avg1,0,19,0);
		int2lcdyx(ibt._max_dispers_num,1,19,0);
		int2lcdyx(t_box,3,19,0);
     	}		     	

    else if(sub_ind==10)
     	{
     	bgnd_par("LB                  ",
     		    "                    ",
     		    "                    ",
     		    "                    ");

     	int2lcdyx(sub_ind1+1,0,3,0);
		int2lcdyx(lakb[sub_ind1]._cnt,0,6,0);

		int2lcdyx(lakb[sub_ind1]._max_cell_temp,0,14,0);
		int2lcdyx(lakb[sub_ind1]._min_cell_temp,0,19,0);

		int2lcdyx(lakb[sub_ind1]._max_cell_volt,1,4,0);
		int2lcdyx(lakb[sub_ind1]._min_cell_volt,1,9,0);
		int2lcdyx(lakb[sub_ind1]._tot_bat_volt,1,14,0);
		int2lcdyx(lakb[sub_ind1]._s_o_h,1,19,0);

		int2lcdyx(lakb[sub_ind1]._ch_curr,2,4,0);
		int2lcdyx(lakb[sub_ind1]._dsch_curr,2,9,0);
		int2lcdyx(lakb[sub_ind1]._rat_cap,2,14,0);
		int2lcdyx(lakb[sub_ind1]._s_o_c,2,19,0);

		int2lcdyx(lakb[sub_ind1]._c_c_l_v,3,4,0);
		int2lcdyx(lakb[sub_ind1]._r_b_t,3,9,0);
		int2lcdyx(lakb[sub_ind1]._b_p_ser_num,3,14,0);
		//int2lcdyx(lakb[sub_ind1]._bRS485ERR,3,16,0);
		//int2lcdyx(lakb[sub_ind1]._rs485_cnt,3,19,0); 
		
     	}	

    else if(sub_ind==11)
     	{
     	bgnd_par("LB                  ",
     		    "                    ",
     		    "                    ",
     		    "                    ");

     	int2lcdyx(sub_ind,0,1,0);
		
		int2lcdyx(u_necc,0,5,0);

		int2lcdyx(load_U,0,11,0);
		int2lcdyx(bps[0]._Uin,0,15,0);
		int2lcdyx(bps[1]._Uii,0,19,0);

		int2lcdyx(li_bat._Ub,1,3,0);


		int2lcdyx(lakb[0]._tot_bat_volt,1,8,0);   

		int2lcdyx(cntrl_stat,1,19,0);
		
		int2lcdyx(lakb_error_cnt,3,19,0);



		//int2lcdyx(ccc_plazma[2],3,12,0);
		//int2lcdyx(ccc_plazma[3],1,16,0);  
		//int2lcdyx(ccc_plazma[4],2,16,0);  
		//int2lcdyx(ccc_plazma[5],3,16,0); 

		int2lcdyx(li_bat._canErrorCnt,2,5,0);
		int2lcdyx(li_bat._canError,2,8,0);
		int2lcdyx(li_bat._485ErrorCnt,3,5,0);
		int2lcdyx(li_bat._485Error,3,8,0);
			
		/*int2lcdyx(lakb[sub_ind1]._cnt,0,6,0);

		int2lcdyx(lakb[sub_ind1]._max_cell_temp,0,14,0);
		int2lcdyx(lakb[sub_ind1]._min_cell_temp,0,19,0);

		int2lcdyx(lakb[sub_ind1]._max_cell_volt,1,4,0);
		int2lcdyx(lakb[sub_ind1]._min_cell_volt,1,9,0);
		int2lcdyx(lakb[sub_ind1]._tot_bat_volt,1,14,0);
		int2lcdyx(lakb[sub_ind1]._s_o_h,1,19,0);

		int2lcdyx(lakb[sub_ind1]._ch_curr,2,4,0);
		int2lcdyx(lakb[sub_ind1]._dsch_curr,2,9,0);
		int2lcdyx(lakb[sub_ind1]._rat_cap,2,14,0);
		int2lcdyx(lakb[sub_ind1]._s_o_c,2,19,0);

		int2lcdyx(lakb[sub_ind1]._c_c_l_v,3,4,0);
		int2lcdyx(lakb[sub_ind1]._r_b_t,3,9,0);
		int2lcdyx(lakb[sub_ind1]._b_p_ser_num,3,14,0);
		int2lcdyx(lakb[sub_ind1]._bRS485ERR,3,16,0);
		 */
		
     	}	
    else if(sub_ind==12)
     	{
     	bgnd_par(	"ica                 ",
     		    	"                    ",
     		    	"                    ",
     		    	"                    ");
		int2lcdyx(ica_my_current,1,4,0);
     	int2lcdyx(ica_your_current,2,4,0);
     	int2lcdyx(ica_timer_cnt,1,14,0);


		int2lcdyx(ica_plazma[0],0,15,0);
     	int2lcdyx(ica_plazma[1],1,15,0);
     	int2lcdyx(ica_plazma[2],2,15,0);
     	int2lcdyx(ica_plazma[3],3,15,0);
     	int2lcdyx(ica_plazma[4],0,19,0);
		int2lcdyx(ica_plazma[5],1,19,0);
     	int2lcdyx(ica_plazma[6],2,19,0);
     	int2lcdyx(ica_plazma[7],3,19,0);

 
		int2lcdyx(ica_u_necc+50,0,10,0);
		int2lcdyx(u_necc,1,10,0);

		int2lcdyx(bps_U,2,10,0);


     	//int2lcdyx(ica_plazma[8],3,15,0);
     	//int2lcdyx(ica_plazma[9],3,19,0); */
 
		
     	}	     			
     }

else if((ind==iAv_view)||(ind==iAv_view_avt))
	{
//	unsigned short tempUI,tempUI_;
//    	unsigned long tempUL;
	
	bgnd_par(sm_,sm_,sm_,sm_);
	if(sub_ind==0)
		{	
		if(avar_stat&0x00000001)
			{
			bgnd_par(	"    ������  ����    ",
				    	"    �� ���������    ",
				    	sm_,sm_); 
			int2lcd(net_U,']',0);
			}
    		else 
			{
	    		bgnd_par(	"    ������  ����    ",
	    				"     ���������      ",
					sm_,sm_); 
			}
		}
	else if((sub_ind==1)||(sub_ind==2))
		{
		if(avar_stat&(1<<sub_ind))
			{
			bgnd_par(	"   ������ ���. N!   ",
				    	"    �� ���������    ",
				    	sm_,sm_); 
			}
    		else 
			{
	    		bgnd_par(	"   ������ ���. N!   ",
	    				"     ���������      ",
					sm_,sm_); 
		
		     }
		int2lcd(sub_ind,'!',0);
		} 
     
	else if((sub_ind>=3)&&(sub_ind<=14))
		{
		if((sub_ind-2)<=9)					ptrs[0]=	"   ������ ��� N+    ";
		else 							ptrs[0]=	"   ������ ��� N +   ";
		if(bps[sub_ind-3]._last_avar==0)		ptrs[1]=	"     ��������!!!    ";
		else if(bps[sub_ind-3]._last_avar==1)	ptrs[1]=	"  �������� U���!!!  ";	
		else if(bps[sub_ind-3]._last_avar==2)	ptrs[1]=	"  �������� U���!!!  ";	
		else if(bps[sub_ind-3]._last_avar==3)	ptrs[1]=	"    ����������!!!   ";
		if(avar_stat&(1<<sub_ind)) 			ptrs[2]=	"    �� ���������    ";
		else								ptrs[2]=	"     ���������      ";	
										ptrs[3]=	"                    ";

		bgnd_par(ptrs[0],ptrs[1],ptrs[2],ptrs[3]);
		int2lcd((sub_ind-2),'+',0);
          
		//int2lcdxy(sub_ind,0x20,0);

		} 
		
	else if(sub_ind==24)
		{ 

		if(sk_av_stat[0]==sasON)
			{
			bgnd_par(	"   ������ ��. N1    ",
				    	"    �� ���������    ",
				    	sm_,sm_); 
			}
    		else 
			{
	    		bgnd_par(	"   ������ ��. N1    ",
	    				"     ���������      ",
					sm_,sm_); 
		
		     }

		}

	else if(sub_ind==25)
		{ 

		if(sk_av_stat[1]==sasON)
			{
			bgnd_par(	"   ������ ��. N2    ",
				    	"    �� ���������    ",
				    	sm_,sm_); 
			}
    		else 
			{
	    		bgnd_par(	"   ������ ��. N2    ",
	    				"     ���������      ",
					sm_,sm_); 
		
		     }

		}

	else if(sub_ind==26)
		{ 

		if(sk_av_stat[2]==sasON)
			{
			bgnd_par(	"   ������ ��. N3    ",
				    	"    �� ���������    ",
				    	sm_,sm_); 
			}
    		else 
			{
	    		bgnd_par(	"   ������ ��. N3    ",
	    				"     ���������      ",
					sm_,sm_); 
		
		     }

		}

	else if(sub_ind==27)
		{ 

		if(sk_av_stat[3]==sasON)
			{
			bgnd_par(	"   ������ ��. N4    ",
				    	"    �� ���������    ",
				    	sm_,sm_); 
			}
    		else 
			{
	    		bgnd_par(	"   ������ ��. N4    ",
	    				"     ���������      ",
					sm_,sm_); 
		
		     }

		}

	else if(sub_ind==28)
		{ 
		if(avar_stat&(1<<sub_ind))
			{
			bgnd_par(	"  ������ ���������  ",
				    	"    ����������!!!   ",
				    	sm_,sm_); 
			}

		}

	else if(sub_ind==5)
		{

		}

	else if(sub_ind==6)
		{

		}

	else if(sub_ind==7)
		{

		} 
		
	else if(sub_ind==8)
		{

		}

	else if(sub_ind==9)
		{

		}

	else if(sub_ind==10)
		{

		}
	    		     
	else if(sub_ind==11)
		{

		} 
		
	else if(sub_ind==12)
		{

		}

	else if(sub_ind==13)
		{

		}

	else if(sub_ind==14)
		{

		}

	else if(sub_ind==15)
		{

		} 
					
	} 
#ifndef _DEBUG_	


else if(ind==iTst_VD)
	{
	ptrs[0]=						" ���� ������        ";
    ptrs[1]=						" 1                 !";
    ptrs[2]=						" ���� ������        ";
    ptrs[3]=						" 2                 @";
    ptrs[4]=						" ���� ������        ";
    ptrs[5]=						" 3                 #";
	ptrs[6]=						" ���� ������        ";
    ptrs[7]=						" 4                 $";
	ptrs[8]=						" ��� N1             ";
    ptrs[9]=						" ��� N2             ";
    ptrs[10]=						" ��� N3             ";
	ptrs[11]=						" ��� N4             ";
    ptrs[12]=						" ��� N5             ";
    ptrs[13]=						" ��� N6             ";
	ptrs[14]=						" ��� N7             ";
    ptrs[15]=						" ��� N8             ";
    ptrs[16]=						" ��� N9             ";               
	ptrs[17]=						" ��� N10            ";
    ptrs[18]=						" ��� N11            ";
    ptrs[19]=						" ��� N12            ";               
	ptrs[8+NUMIST]=					" �����              ";
	ptrs[9+NUMIST]=					" �������� WDT(�����)";
	ptrs[10+NUMIST]=				" �������� WDT(�����)";

	if((sub_ind-index_set)>2)index_set=sub_ind-2;
	else if(sub_ind<index_set)index_set=sub_ind;
	bgnd_par("        ����        ",ptrs[index_set],ptrs[index_set+1],ptrs[index_set+2]);
	pointer_set(1);
/*	int2lcdyx(sub_ind,0,19,0);
	int2lcdyx(index_set,0,17,0);
	int2lcdyx(tst_state[0],0,15,0); */

	//int2lcd_mmm(bat[0]._Ib,'<',2);
	//int2lcd_mmm(bat[1]._Ib,'>',2);

	if(tst_state[0]==tst1) sub_bgnd("���.",'!',-3);
	if(tst_state[0]==tst2) sub_bgnd("����.",'!',-4);
	else sub_bgnd("�����.",'!',-5);

	if(tst_state[1]==tst1) sub_bgnd("���.",'@',-3);
	if(tst_state[1]==tst2) sub_bgnd("����.",'@',-4);
	else sub_bgnd("�����.",'@',-5);

	if(tst_state[2]==tst1) sub_bgnd("���.",'#',-3);
	if(tst_state[2]==tst2) sub_bgnd("����.",'#',-4);
	else sub_bgnd("�����.",'#',-5);

	if(tst_state[3]==tst1) sub_bgnd("���.",'$',-3);
	if(tst_state[3]==tst2) sub_bgnd("����.",'$',-4);
	else sub_bgnd("�����.",'$',-5);
	
	if(sub_ind==0)
		{
		if(tst_state[0]==tst1)mess_send(MESS2RELE_HNDL,PARAM_RELE_AV_1,1,5);
		else if(tst_state[0]==tst2)mess_send(MESS2RELE_HNDL,PARAM_RELE_AV_1,0,5);
		}
	else if(sub_ind==2)
		{
		if(tst_state[1]==tst1)mess_send(MESS2RELE_HNDL,PARAM_RELE_AV_2,1,5);
		else if(tst_state[1]==tst2)mess_send(MESS2RELE_HNDL,PARAM_RELE_AV_2,0,5);
		}	
	else if(sub_ind==4)
		{
		if(tst_state[2]==tst1)mess_send(MESS2RELE_HNDL,PARAM_RELE_AV_3,1,5);
		else if(tst_state[2]==tst2)mess_send(MESS2RELE_HNDL,PARAM_RELE_AV_3,0,5);
		}
	else if(sub_ind==6)
		{
		if(tst_state[3]==tst1)mess_send(MESS2RELE_HNDL,PARAM_RELE_AV_4,1,5);
		else if(tst_state[3]==tst2)mess_send(MESS2RELE_HNDL,PARAM_RELE_AV_4,0,5);
		}
	}

else if(ind==iTst_bps)
	{
	if(tst_state[5]==tstOFF)ptrs[0]=		" ��������           ";
	else if(tst_state[5]==tst1)ptrs[0]=		" �������            ";
//	else ptrs[0]=							" ���������          ";
    ptrs[1]=								" ���              @ ";
    ptrs[2]=								" U =  .$�  I =  .#A ";
	ptrs[3]=								" �����              ";


	if((sub_ind-index_set)>2)index_set=sub_ind-2;
	else if(sub_ind<index_set)index_set=sub_ind;
	bgnd_par("     ���� ��� N!    ",ptrs[index_set],ptrs[index_set+1],ptrs[index_set+2]);
	pointer_set(1);

	if(tst_state[6]==tst1) sub_bgnd("Umax",'@',-3); 
	else sub_bgnd("Umin",'@',-3);




	/*int2lcdyx(sub_ind,0,19,0);
	int2lcdyx(index_set,0,17,0);
	int2lcdyx(tst_state[5],0,3,0); */
	//int2lcdyx(tst_state[6],0,1,0); 
	//int2lcdyx(cntrl_stat,0,5,0); 

	int2lcd(sub_ind1+1,'!',0);
	int2lcd(bps[sub_ind1]._Ii,'#',1);
	int2lcd(bps[sub_ind1]._Uii,'$',1);


	if(tst_state[5]==tstOFF) mess_send(MESS2BPS_HNDL,PARAM_BPS_MASK_ON_OFF_AFTER_2SEC,~(1<<sub_ind1),10);
	else mess_send(MESS2BPS_HNDL,PARAM_BPS_MASK_ON_OFF_AFTER_2SEC,(1<<sub_ind1),10);
	
	//if(tst_state[5]==tst2) mess_send(MESS2NET_DRV,PARAM_BPS_NET_OFF,1,10);
	
	if(sub_ind==0)mess_send(MESS2CNTRL_HNDL,PARAM_CNTRL_STAT_SET,1020,10);	
	else if(tst_state[6]==tstOFF) mess_send(MESS2CNTRL_HNDL,PARAM_CNTRL_STAT_SET,30,10);
	else if(tst_state[6]==tst1) mess_send(MESS2CNTRL_HNDL,PARAM_CNTRL_STAT_SET,1020,10);
/*		mess_send(MESS2BAT_HNDL,PARAM_BAT_ALL_OFF_AFTER_2SEC,0,10);
	    	mess_send(MESS2CNTRL_HNDL,PARAM_CNTRL_STAT_SET,1000,10);
          }
     if(sub_ind==6)
		{
          mess_send(MESS2BPS_HNDL,PARAM_BPS_MASK_ON_OFF_AFTER_2SEC,(1<<sub_ind1),10);
          mess_send(MESS2BAT_HNDL,PARAM_BAT_ALL_OFF_AFTER_2SEC,0,40);
          mess_send(MESS2UNECC_HNDL,PARAM_UNECC_SET,U_AVT,10);
	    	mess_send(MESS2CNTRL_HNDL,PARAM_CNTRL_STAT_FAST_REG,0,10);

          }

     if(sub_ind==9)
		{
		if(phase==0)
			{
          	
          	}
      	else if(phase==1)
			{
          	mess_send(MESS2BPS_HNDL,PARAM_BPS_MASK_ON_OFF_AFTER_2SEC,(1<<sub_ind1),10);
			mess_send(MESS2BAT_HNDL,PARAM_BAT_ALL_OFF_AFTER_2SEC,0,10);
          	}
          mess_send(MESS2CNTRL_HNDL,PARAM_CNTRL_STAT_SET,1000,10);
          }
	*/




	}	 
else if(ind==iKlimat)
	{
	ptrs[0]=				" t��.max=       !�C ";
	ptrs[1]=				" t����.max=     @�C ";
	ptrs[2]=				" t��.���.=      #�C ";
	ptrs[3]=				" t����.����.    $�C ";
	ptrs[4]=				" t���.����.     %�C ";
	ptrs[5]=				" t����.���.     ^�C ";
	ptrs[6]=				" t���.���.      &�C ";
	ptrs[7]=				" �����              ";



	if((sub_ind-index_set)>2)index_set=sub_ind-2;
	else if(sub_ind<index_set)index_set=sub_ind;
	bgnd_par(				"   ��������������   ",
						ptrs[index_set],
						ptrs[index_set+1],
						ptrs[index_set+2]);
	pointer_set(1);
	
	int2lcd(TBOXMAX,'!',0); 
	if((TBOXVENTMAX>=50)&&(TBOXVENTMAX<=80))int2lcd(TBOXVENTMAX,'@',0); 
	else sub_bgnd("����.",'@',-2);
	int2lcd(TBOXREG,'#',0);
	if((TLOADDISABLE>=50)&&(TLOADDISABLE<=80))int2lcd(TLOADDISABLE,'$',0);
	else sub_bgnd("����.",'$',-2);
	if((TLOADDISABLE>=50)&&(TLOADDISABLE<=80)/*&&(TLOADENABLE>=50)&&(TLOADENABLE<=80)*/)int2lcd(TLOADENABLE,'%',0);
	else sub_bgnd("����.",'%',-2);
	if((TBATDISABLE>=50)&&(TBATDISABLE<=90))int2lcd(TBATDISABLE,'^',0);
	else sub_bgnd("����.",'^',-2);
	if((TBATDISABLE>=50)&&(TBATDISABLE<=90)/*&&(TBATENABLE>=50)&&(TBATENABLE<=80)*/)int2lcd(TBATENABLE,'&',0);
	else sub_bgnd("����.",'&',-2);
	}

else if(ind==iKlimat_kontur)
	{
	ptrs[0]=				" t��.max=       !�C ";
	ptrs[1]=				" t����.max=     @�C ";
	ptrs[2]=				" t��.���.=      #�C ";
	ptrs[3]=				" t���.������.   $�C ";
	ptrs[4]=				" t����.������.  %�C ";
	ptrs[5]=				" t����.������.  ^�C ";
	ptrs[6]=				" t���.������.   &�C ";
	ptrs[7]=				" t����.���.     *�C ";
	ptrs[8]=				" t���.���.      (�C ";
	ptrs[9]=				" �����              ";



	if((sub_ind-index_set)>2)index_set=sub_ind-2;
	else if(sub_ind<index_set)index_set=sub_ind;
	bgnd_par(				"   ��������������   ",
						ptrs[index_set],
						ptrs[index_set+1],
						ptrs[index_set+2]);
	pointer_set(1);
	
	int2lcd(TBOXMAX,'!',0); 
/*	int2lcd(TBOXVENTON,'@',0); 
	int2lcd(TBOXVENTOFF,'#',0);*/
	if((TBOXVENTMAX>=50)&&(TBOXVENTMAX<=80))int2lcd(TBOXVENTMAX,'@',0); 
	else sub_bgnd("����.",'@',-2);
	int2lcd(TBOXREG,'#',0);
	int2lcd_mmm(TBOXWARMON,'$',0); 
	int2lcd_mmm(TBOXWARMOFF,'%',0);
	if((TLOADDISABLE>=50)&&(TLOADDISABLE<=80))int2lcd(TLOADDISABLE,'^',0);
	else sub_bgnd("����.",'^',-2);
	if((TLOADDISABLE>=50)&&(TLOADDISABLE<=80))int2lcd(TLOADENABLE,'&',0);
	else sub_bgnd("����.",'&',-2);
	if((TBATDISABLE>=50)&&(TBATDISABLE<=90))int2lcd(TBATDISABLE,'*',0);
	else sub_bgnd("����.",'*',-2);
	if((TBATDISABLE>=50)&&(TBATDISABLE<=90))int2lcd(TBATENABLE,'(',0);
	else sub_bgnd("����.",'(',-2);

//		int2lcdyx(t_box_vent_on_cnt,0,3,0);
//	int2lcdyx(t_box_warm_on_cnt,0,7,0);
//	int2lcdyx(vent_stat_k,0,10,0);
//	int2lcdyx(warm_stat_k,0,13,0);
	
	int2lcdyx(t_box,0,19,0);	 
	}
#ifdef UKU_TELECORE2015
else if(ind==iKlimat_TELECORE)
	{
	ptrs[0]=				" ������ ����������� ";
	ptrs[1]=				" ��������         ! ";
	ptrs[2]=				" ������ ����������� ";
	ptrs[3]=				" �����������      @ ";
	ptrs[4]=				" t���.������.   $�C ";
	ptrs[5]=				" t����.������.  <�C ";
	ptrs[6]=				" Q�����.        *%  ";
	ptrs[7]=				" t���.����.     ^�C ";
	ptrs[8]=				" t����.����.    &�C ";
	ptrs[9]=				" t���.��.����.  [�C ";
	ptrs[10]=				" t����.��.����. ]�C ";
	ptrs[11]=				" �����              ";



	if((sub_ind-index_set)>2)index_set=sub_ind-2;
	else if(sub_ind<index_set)index_set=sub_ind;
	bgnd_par(				"   ��������������   ",
						ptrs[index_set],
						ptrs[index_set+1],
						ptrs[index_set+2]);
	pointer_set(1);

	if(TELECORE2015_KLIMAT_WARM_SIGNAL==0) sub_bgnd("�� 2",'!',-3);
	else if(TELECORE2015_KLIMAT_WARM_SIGNAL==1) sub_bgnd("�� 1",'!',-3);
	//else sub_bgnd("�� BMS",'!',-5);
	if(TELECORE2015_KLIMAT_VENT_SIGNAL==0) sub_bgnd("�� 2",'@',-3);
	else if(TELECORE2015_KLIMAT_VENT_SIGNAL==1) sub_bgnd("�� 1",'@',-3);
	//else sub_bgnd("�� BMS",'@',-5);
	int2lcd_mmm(TELECORE2015_KLIMAT_WARM_ON,'$',0); 
	int2lcd_mmm(TELECORE2015_KLIMAT_WARM_OFF,'<',0);
	int2lcd_mmm(TELECORE2015_KLIMAT_CAP,'*',0);	
	int2lcd_mmm(TELECORE2015_KLIMAT_VENT_ON,'^',0);
	int2lcd_mmm(TELECORE2015_KLIMAT_VENT_OFF,'&',0);
	int2lcd_mmm(TELECORE2015_KLIMAT_VVENT_ON,'[',0);
	int2lcd_mmm(TELECORE2015_KLIMAT_VVENT_OFF,']',0);

	//int2lcd(TBOXMAX,'!',0); 
	
/*	int2lcd(TBOXVENTON,'@',0); 
	int2lcd(TBOXVENTOFF,'#',0);*/
	//if((TBOXVENTMAX>=50)&&(TBOXVENTMAX<=80))int2lcd(TBOXVENTMAX,'@',0); 
	//else sub_bgnd("����.",'@',-2);
	//int2lcd(TBOXREG,'#',0);




//	int2lcdyx(t_ext[0],0,3,0);
//	int2lcdyx(t_ext[1],0,7,0);
/*//	int2lcdyx(vent_stat_k,0,10,0);
	int2lcdyx(TELECORE2015_KLIMAT_WARM_ON_temp,0,13,0);
	
	int2lcdyx(t_box,0,19,0);	 */
	}
#endif
#ifdef UKU_TELECORE2017
else if(ind==iKlimat_TELECORE)
	{
	ptrs[0]=				" ������ ����������� ";
	ptrs[1]=				" ��������         ! ";
	ptrs[2]=				" ������ ����������� ";
	ptrs[3]=				" �����������      @ ";
	ptrs[4]=				" t���.������.   $�C ";
	ptrs[5]=				" t����.������.  <�C ";
	ptrs[6]=				" Q�����.        >%  ";
	ptrs[7]=				" t����.100%     y�C ";	
	ptrs[8]=				" t����.80%      f�C ";
	ptrs[9]=				" t����.60%      r�C ";
	ptrs[10]=				" t����.40%      e�C ";
	ptrs[11]=				" t����.20%      w�C ";
	ptrs[12]=				" dt����.100%    s�C ";
	ptrs[13]=				" dt����.80%     a�C ";
	ptrs[14]=				" dt����.60%     p�C ";
	ptrs[15]=				" dt����.40%     o�C ";
	ptrs[16]=				" dt����.20%     i�C ";
	ptrs[17]=				" �����              ";

//	ptrs[7]=				" t����.0%       q�C ";
//	ptrs[13]=				" dt����.0%      u�C ";

	if((sub_ind-index_set)>2)index_set=sub_ind-2;
	else if(sub_ind<index_set)index_set=sub_ind;
	bgnd_par(				"   ��������������   ",
						ptrs[index_set],
						ptrs[index_set+1],
						ptrs[index_set+2]);
	pointer_set(1);

	if(TELECORE2017_KLIMAT_WARM_SIGNAL==0) sub_bgnd("�� 2",'!',-3);
	else if(TELECORE2017_KLIMAT_WARM_SIGNAL==1) sub_bgnd("�� 1",'!',-3);
	//else sub_bgnd("�� BMS",'!',-5);
	if(TELECORE2017_KLIMAT_VENT_SIGNAL==0) sub_bgnd("�� 2",'@',-3);
	else if(TELECORE2017_KLIMAT_VENT_SIGNAL==1) sub_bgnd("�� 1",'@',-3);
	//else sub_bgnd("�� BMS",'@',-5);
	int2lcd_mmm(TELECORE2017_KLIMAT_WARM_ON,'$',0); 
	int2lcd_mmm(TELECORE2017_KLIMAT_WARM_OFF,'<',0);
	int2lcd_mmm(TELECORE2017_KLIMAT_CAP,'>',0);	
//	int2lcd_mmm(TELECORE2017_KLIMAT_VENT_ON0,'q',0);
	int2lcd_mmm(TELECORE2017_KLIMAT_VENT_ON20,'w',0);
	int2lcd_mmm(TELECORE2017_KLIMAT_VENT_ON40,'e',0);
	int2lcd_mmm(TELECORE2017_KLIMAT_VENT_ON60,'r',0);
	int2lcd_mmm(TELECORE2017_KLIMAT_VENT_ON80,'f',0);
	int2lcd_mmm(TELECORE2017_KLIMAT_VENT_ON100,'y',0);
//	int2lcd_mmm(TELECORE2017_KLIMAT_DVENT_ON0,'u',0);
	int2lcd_mmm(TELECORE2017_KLIMAT_DVENT_ON20,'i',0);
	int2lcd_mmm(TELECORE2017_KLIMAT_DVENT_ON40,'o',0);
	int2lcd_mmm(TELECORE2017_KLIMAT_DVENT_ON60,'p',0);
	int2lcd_mmm(TELECORE2017_KLIMAT_DVENT_ON80,'a',0);
	int2lcd_mmm(TELECORE2017_KLIMAT_DVENT_ON100,'s',0);

	//int2lcd(TBOXMAX,'!',0); 
	
/*	int2lcd(TBOXVENTON,'@',0); 
	int2lcd(TBOXVENTOFF,'#',0);*/
	//if((TBOXVENTMAX>=50)&&(TBOXVENTMAX<=80))int2lcd(TBOXVENTMAX,'@',0); 
	//else sub_bgnd("����.",'@',-2);
	//int2lcd(TBOXREG,'#',0);




//	int2lcdyx(t_ext[0],0,3,0);
//	int2lcdyx(t_ext[1],0,7,0);
/*//	int2lcdyx(vent_stat_k,0,10,0);
	int2lcdyx(TELECORE2015_KLIMAT_WARM_ON_temp,0,13,0);
	
	int2lcdyx(t_box,0,19,0);	 */
	}
#endif

else if(ind==iNpn_set)
	{
	ptrs[0]=				" �����          !   ";
	if(NPN_OUT==npnoOFF)
		{
		ptrs[1]=			" �����              ";
		ptrs[2]=			"                    ";
		simax=1;
		}
	else 
		{
/*		ptrs[1]=			" ������         @   ";
 		if(NPN_SIGN==npnsAVNET)
			{
			ptrs[2]=		" T�.�.�.�.       #�.";
			ptrs[3]=		" �����              ";
			simax=3;
			}
		else if(NPN_SIGN==npnsULOAD)
			{
			ptrs[2]=		" U����.�.�.�.    $� ";
			ptrs[3]=		" U���.�.�.�.     %� ";
			ptrs[4]=		" T�.�.�.�.       #�.";
			ptrs[5]=		" �����              ";
			simax=5;
			}*/

			ptrs[1]=		" U����.�.�.�.    $� ";
			ptrs[2]=		" U���.�.�.�.     %� ";
			ptrs[3]=		" T�.�.�.�.       #�.";
			ptrs[4]=		" �����              ";
			simax=4;
		}


	if((sub_ind-index_set)>2)index_set=sub_ind-2;
	else if(sub_ind<index_set)index_set=sub_ind;
	bgnd_par(				" ���������� �.�.�.  ",
							ptrs[index_set],
							ptrs[index_set+1],
							ptrs[index_set+2]);
	pointer_set(1);
	
	if(NPN_OUT==npnoRELEVENT) sub_bgnd("���� ����-��",'!',-8);
	else if(NPN_OUT==npnoRELEAVBAT2) sub_bgnd("���� ��.���2",'!',-8);
	else sub_bgnd("����.",'!',-1);
//	if(NPN_SIGN==npnsAVNET) sub_bgnd("��.����",'@',-3);
//	else sub_bgnd("U����.",'@',-2);
	int2lcd(TZNPN,'#',0);
	int2lcd(UONPN,'$',1);
	int2lcd(UVNPN,'%',1);

	}

#endif

else if(ind==iBps_list)
     {
     if(sub_ind==0)
     	{
     	bgnd_par(" N  L   U    I    t ",
     	         " !  @    ^    $    #",
     	         " !  @    ^    $    #",
     	         " !  @    ^    $    #");
      

     	}     

    else if(sub_ind==1) 
     	{
      	bgnd_par(" N  L   U    I   U� ",
     	         " !  @    ^    $    %",
     	         " !  @    ^    $    %",
     	         " !  @    ^    $    %");

		} 

    else if(sub_ind==2) 
     	{
      	bgnd_par(" N   U    I   pwm ��",
     	         " !    ^    $   &   (",
     	         " !    ^    $   &   (",
     	         " !    ^    $   &   (");

		}

 	else if(sub_ind==3) 
     	{
      	bgnd_par(" N   U    I   pwm ��",
     	         " !    ^    $   &   )",
     	         " !    ^    $   &   )",
     	         " !    ^    $   &   )");

		}


	int2lcd(sub_ind1+1,'!',0);
	int2lcd(sub_ind1+2,'!',0);
	if(sub_ind1==NUMIST-2) sub_bgnd("�",'!',0);
	else int2lcd(sub_ind1+3,'!',0);

	int2lcd(bps[sub_ind1]._cnt,'@',0);
	int2lcd(bps[sub_ind1+1]._cnt,'@',0);
	if(sub_ind1==NUMIST-2)int2lcd(ibat_metr_cnt,'@',0);
	else int2lcd(bps[sub_ind1+2]._cnt,'@',0);		
		
	int2lcd(bps[sub_ind1]._Uii/10,'^',0);
	int2lcd(bps[sub_ind1+1]._Uii/10,'^',0);
	if(sub_ind1<NUMIST-2) int2lcd(bps[sub_ind1+2]._Uii/10,'^',0);
	else sub_bgnd(" ",'^',0);

     int2lcd(bps[sub_ind1]._Ii,'$',1); 
	int2lcd(bps[sub_ind1+1]._Ii,'$',1); 
	if(sub_ind1<NUMIST-2) int2lcd(bps[sub_ind1+2]._Ii,'$',1); 
	else int2lcd_mmm(Ib_ips_termokompensat,'$',0);

	int2lcd(bps[sub_ind1]._Uin/10,'%',0);
	int2lcd(bps[sub_ind1+1]._Uin/10,'%',0);
	if(sub_ind1<NUMIST-2) int2lcd(bps[sub_ind1+2]._Uin/10,'%',0);
	else sub_bgnd(" ",'%',0);

	int2lcd(bps[sub_ind1]._Ti,'#',0);
	int2lcd(bps[sub_ind1+1]._Ti,'#',0); 
   	if(sub_ind1<NUMIST-2) int2lcd(bps[sub_ind1+2]._Ti,'#',0);
	else sub_bgnd(" ",'#',0);

	int2lcd(bps[sub_ind1].debug_info_to_uku0,'&',0);
	int2lcd(bps[sub_ind1+1].debug_info_to_uku0,'&',0);
	if(sub_ind1<NUMIST-2) int2lcd(bps[sub_ind1+2].debug_info_to_uku0,'&',0);
	else sub_bgnd(" ",'&',0);

	int2lcd(bps[sub_ind1]._x_+50,'(',0);
	int2lcd(bps[sub_ind1+1]._x_+50,'(',0);
	if(sub_ind1<NUMIST-2) int2lcd(bps[sub_ind1+2]._x_+50,'(',0);
	else sub_bgnd(" ",'(',0);

	int2lcd(bps[sub_ind1]._avg,')',0);
	int2lcd(bps[sub_ind1+1]._avg,')',0);
	if(sub_ind1<NUMIST-2) int2lcd(bps[sub_ind1+2]._avg,')',0);
	else sub_bgnd(" ",')',0);

//	int2lcdyx(cntrl_stat,0,3,0);
//	int2lcdyx(UOUT_,0,9,0);
//	int2lcdyx(out_U,0,14,0); 	
	}
	
else if(ind==iAvt_set_sel)
	{
	ptrs[0]=						" ��� N1             ";
     ptrs[1]=						" ��� N2             ";
     ptrs[2]=						" ��� N3             ";
	ptrs[3]=						" ��� N4             ";
     ptrs[4]=						" ��� N5             ";
     ptrs[5]=						" ��� N6             ";
	ptrs[6]=						" ��� N7             ";
     ptrs[7]=						" ��� N8             ";
     ptrs[8]=						" ��� N9             ";
	ptrs[9]=						" ��� N10            ";
     ptrs[10]=						" ��� N11            ";
     ptrs[11]=						" ��� N12            ";               
	ptrs[NUMIST]=					" �����              ";
	ptrs[1+NUMIST]=				"                    ";
	ptrs[2+NUMIST]=				"                    ";


	if((sub_ind-index_set)>1)index_set=sub_ind-1;
	else if(sub_ind<index_set)index_set=sub_ind;
	bgnd_par(				"��������� ����������",
						" ���������� ������  ",
						ptrs[index_set],
						ptrs[index_set+1]);
	pointer_set(2);

	}		 

else if(ind==iAvt_set)
	{
	ptrs[0]=" U����� =   $�      ";
	if(bFL_)
		{
		ptrs[1]=" ���������� U�����  ";
     	ptrs[2]="  �������� � ��� �  ";
     	}
     else 
     	{
		ptrs[1]=" ����������� � ���  ";
     	ptrs[2]="    �����������     ";     	
     	}	

     ptrs[3]=sm_exit;
     ptrs[4]=sm_;
     ptrs[5]=sm_;     	     	    
	

     if((sub_ind==0)||(sub_ind==1)||(sub_ind==2))index_set=0;
	else index_set=3;
	
	bgnd_par("       ��� N!      ",ptrs[index_set],ptrs[index_set+1],ptrs[index_set+2]);

	pointer_set(1);	
	int2lcd(sub_ind1+1,'!',0);
	int2lcd(U_AVT,'$',1);
	 
	
     if(sub_ind==0)
		{
        mess_send(MESS2BPS_HNDL,PARAM_BPS_MASK_ON_OFF_AFTER_2SEC,(1<<sub_ind1),10);
        mess_send(MESS2BAT_HNDL,PARAM_BAT_ALL_OFF_AFTER_2SEC,0,40);
        mess_send(MESS2UNECC_HNDL,PARAM_UNECC_SET,U_AVT,10);
	    mess_send(MESS2CNTRL_HNDL,PARAM_CNTRL_STAT_FAST_REG,0,10);

        }

 	if(mess_find( (MESS2IND_HNDL)) && (mess_data[0]==PARAM_U_AVT_GOOD) )
		{
		sub_ind=3;
		show_mess(	"     ���������      ",
	          		"    ����������      ",
	          		" ���������� ������  ",
	          		"    �����������     ",3000);
		
		}
	//int2lcdyx(sub_ind,0,4,0);		      
	 }
else if(ind==iOut_volt_contr)
	{
	ptrs[0]=" U���max         !� ";
    ptrs[1]=" U���min         @� ";
    ptrs[2]=" T����.��.       #�.";     	
    ptrs[3]=sm_exit;
    ptrs[4]=sm_;
    ptrs[5]=sm_;     	     	    
	

	if((sub_ind-index_set)>1)index_set=sub_ind-1;
	else if(sub_ind<index_set)index_set=sub_ind;
	
	bgnd_par(	"�������� ��������� ",
				"    ����������     ",
				ptrs[index_set],
				ptrs[index_set+1]);

	pointer_set(2);	
	int2lcd(U_OUT_KONTR_MAX,'!',1);
	int2lcd(U_OUT_KONTR_MIN,'@',1);
	int2lcd(U_OUT_KONTR_DELAY,'#',0);
	}
else if(ind==iDop_rele_set)
	{
	if(DOP_RELE_FUNC==0)
		{
		ptrs[0]=	" ���������          ";
		ptrs[1]=	" ����������� ������ ";
		}
	else /*if(DOP_RELE_FUNC==0)*/
		{
		ptrs[0]=	" ���������          ";
		ptrs[1]=	" ����������� �������";
		}
	ptrs[2]=		" �����              ";
	ptrs[3]=		"                    ";


	if((sub_ind-index_set)>1)index_set=sub_ind;
	else if(sub_ind<index_set)index_set=sub_ind;
	bgnd_par(			"��������������� ����",
						"  ����������������  ",
						ptrs[index_set],
						ptrs[index_set+1]);
	pointer_set(2);

	//int2lcdyx(sub_ind,0,6,0);
	//int2lcdyx(index_set,0,9,0);
	
	}

else if (ind==iIps_Curr_Avg_Set)
	{ 
	if(ICA_EN==0)
		{
		ptrs[0]=		" ���������          ";
		simax=1;
		}
	else 
		{
		ptrs[0]=		" ��������           ";
		if(ICA_CH==0)
			{
			ptrs[1]=	" �����  MODBUS-RTU  ";
			ptrs[2]=	" ����� ��������   ! ";
			simax=3;
			}
		else
			{
			ptrs[1]=	" �����   MODBUS-TCP ";
			ptrs[2]=	" IP 00@.00#.00$.00% ";
			ptrs[3]=	" ����� ��������   ^ ";
			simax=4;
			}
		} 
	ptrs[simax]=		" �����              ";
	
	if(sub_ind<index_set) index_set=sub_ind;
	else if((sub_ind-index_set)>1) index_set=sub_ind-1;	
	bgnd_par(	" ������������ ����� ",
				"        ���         ",
				ptrs[index_set],
				ptrs[index_set+1]);
	
	pointer_set(2);
	int2lcd(ICA_MODBUS_ADDRESS,'!',0);
	if((sub_ind==2)&&(sub_ind1==0)&&bFL2)sub_bgnd("   ",'@',-2);
	else int2lcd(ICA_MODBUS_TCP_IP1,'@',0);
	if((sub_ind==2)&&(sub_ind1==1)&&bFL2)sub_bgnd("   ",'#',-2);
	else int2lcd(ICA_MODBUS_TCP_IP2,'#',0);
	if((sub_ind==2)&&(sub_ind1==2)&&bFL2)sub_bgnd("   ",'$',-2);
	else int2lcd(ICA_MODBUS_TCP_IP3,'$',0);
	if((sub_ind==2)&&(sub_ind1==3)&&bFL2)sub_bgnd("   ",'%',-2);
	else int2lcd(ICA_MODBUS_TCP_IP4,'%',0);
	int2lcd(ICA_MODBUS_TCP_UNIT_ID,'^',0);	
     
 	} 

/*
const char sm7[]	={" �������� N2        "}; //
const char sm8[]	={" ��������           "}; //
const char sm9[]	={" ����               "}; //
const char sm10[]	={" �����������        "}; // 
const char sm11[]	={" ������ ������      "}; //
const char sm12[]	=" ���������� ������  "}; //
const cha		=" �������            "}; //
*/


//char2lcdhyx(bat_rel_stat[0],0,10);
//char2lcdhyx(bat_rel_stat[1],0,15);
//int2lcdyx(u_necc,0,19,0);
//int2lcdyx(cntrl_stat,0,5,0); 	   mess_cnt[i]

//char2lcdhyx(bat_rel_stat[0],0,5);
//char2lcdhyx(bat_rel_stat[1],0,10);
//int2lcdyx(mess_cnt[1],0,2,0);
//int2lcdyx(GET_REG(IOPIN1,21,1),0,5,0); 
//int2lcdyx(samokalibr_cnt,0,10,0);
//char2lcdhyx(rele_stat,0,19);
//char2lcdhyx(mess_cnt[1],0,16); 

//int2lcdyx(ad7705_res1,0,8,0);
//int2lcdyx(ad7705_res2,0,16,0); 
//	int2lcdyx(bat[0]._cnt_to_block,0,1,0);
//	int2lcdyx(bat[1]._cnt_to_block,0,3,0);
//	int2lcdyx(bat[0]._rel_stat,0,5,0);
/*	int2lcdyx(ind,0,3,0); 
	int2lcdyx(sub_ind,0,6,0);
	int2lcdyx(index_set,0,9,0);
	int2lcdyx(ptr_ind,0,14,0);
	;*/
/*int2lcdyx(ind,0,19,0);
int2lcdyx(retindsec,0,15,0);
int2lcdyx(retcnt,0,11,0);
int2lcdyx(retcntsec,0,7,0);	*/
//int2lcdyx(bps[0]._vol_i,0,15,0);
//int2lcdyx(cntrl_stat,0,19,0); 
//int2lcdyx(bps[0]._Uin,0,4,0);
}							    


#define BUT0	16
#define BUT1	17
#define BUT2	18
#define BUT3	19
#define BUT4	20   
#define BUT_MASK (1UL<<BUT0)|(1UL<<BUT1)|(1UL<<BUT2)|(1UL<<BUT3)|(1UL<<BUT4)

#define BUT_ON 4
#define BUT_ONL 20 

#define butLUR_  101
#define butU   253
#define butU_  125
#define butD   251
#define butD_  123
#define butL   247
#define butL_  119
#define butR   239
#define butR_  111
#define butE   254
#define butE_  126
#define butEL_  118
#define butUD  249
#define butUD_  121
#define butLR   231
#define butLR_   103
#define butED_  122
#define butDR_  107
#define butDL_  115

#define BUT_ON 4
#define BUT_ONL 20 
//-----------------------------------------------
void but_drv(void)
{
char i;
LPC_GPIO1->FIODIR|=(1<<21);
LPC_GPIO1->FIOPIN&=~(1<<21);
LPC_GPIO1->FIODIR&=~((1<<22)|(1<<23)|(1<<24)|(1<<25)|(1<<26));
LPC_PINCON->PINMODE3&=~((1<<12)|(1<<13)|(1<<14)|(1<<15)|(1<<16)|(1<<17)|(1<<18)|(1<<19)|(1<<20)|(1<<21));

LPC_GPIO2->FIODIR|=(1<<8);
LPC_GPIO2->FIOPIN&=~(1<<8);
for(i=0;i<200;i++)
{
__nop();
__nop();
__nop();
__nop();
__nop();
__nop();
__nop();
__nop();
}

			LPC_GPIO2->FIODIR|=(1<<8);
			LPC_GPIO2->FIOPIN|=(1<<8);

but_n=((LPC_GPIO1->FIOPIN|(~((1<<22)|(1<<23)|(1<<24)|(1<<25)|(1<<26))))>>22)/*&0x0000001f*/;



if((but_n==1023UL)||(but_n!=but_s))
 	{
	speed=0;
 
   	if (((but0_cnt>=BUT_ON)||(but1_cnt!=0))&&(!l_but))
  		{
   	     n_but=1;
          but=but_s;

          }
   	if (but1_cnt>=but_onL_temp)
  		{
   	     n_but=1;
 
          but=but_s&0x7f;
          }
    	l_but=0;
   	but_onL_temp=BUT_ONL;
    	but0_cnt=0;
  	but1_cnt=0;          
     goto but_drv_out;
  	}
else if(but_n==but_s)
 	{
  	but0_cnt++;
  	if(but0_cnt>=BUT_ON)
  		{
   		but0_cnt=0;
   		but1_cnt++;
   		if(but1_cnt>=but_onL_temp)
   			{              
    			but=but_s&0x7f;
    			but1_cnt=0;
    			n_but=1;
    			     
    			l_but=1;
			if(speed)
				{
    				but_onL_temp=but_onL_temp>>1;
        			if(but_onL_temp<=2) but_onL_temp=2;
				}    
   			}
  		}
 	}
but_drv_out: 
but_s=but_n; 
   
}

//-----------------------------------------------
void but_an(void)
{
signed short temp_SS;
signed short deep,i,cap,ptr;
char av_head[4];
if(!n_but)return;
/*else  					
	{
	plazma_but_an++;
	goto but_an_end;
	}*/
av_beep=0x0000;
av_rele=0x0000;
mnemo_cnt=MNEMO_TIME;
ips_bat_av_stat=0;
//bat_ips._av&=~1;

if((main_1Hz_cnt<10)&&((but==butU)||(but==butU_)||(but==butD)||(but==butD_)||(but==butL)||(but==butL_)||(but==butR)||(but==butR_)||(but==butE)||(but==butE_)))
	{
	__ee_spc_stat=spcOFF;
	spc_stat=spcOFF;
	}
if(but==butUD)
     {
     if(ind!=iDeb)
          {
		c_ind=a_ind;
		tree_up(iDeb,5,0,0);
		
          }
     else 
          {
		tree_down(0,0);
          }
		
		     
     }
else if(but==butLR)
	{
	bSILENT=1;
	beep_init(0x00000000,'S');
	}
else if(but==butUD_)
     {
	//avar_bat_as_hndl(0,1);
	}

else if(but==butED_)
     {
	if(!bCAN_OFF)bCAN_OFF=1;
	else bCAN_OFF=0;
	speed=0;
	}

else if(ind==iDeb)
	{
	if(but==butR)
		{
		sub_ind++;
		index_set=0;
		gran_ring_char(&sub_ind,0,12);
		}
	else if(but==butL)
		{
		sub_ind--;
		index_set=0;
		gran_ring_char(&sub_ind,0,12);
		}
		
	else if(sub_ind==1)
		{
		if(but==butU)
	     	{
	     	sub_ind1--;
	     	gran_char(&sub_ind1,0,30);
	     	}
		if(but==butD)
	     	{
	     	sub_ind1++;
	     	gran_char(&sub_ind1,0,30);
	     	}
	     
		if(but==butE)
	     	{
	     	/*SET_REG(C2GSR,3,24,8);
			C2MOD=0;
			 bOUT_FREE2=1;*/

			 // CAN interface 1, use IRQVec7, at 125kbit
//can2_init(7,8,CANBitrate250k_60MHz);

// Receive message with ID 102h on CAN 1
//FullCAN_SetFilter(2,0x18e);
			 }

		if(but==butE)
	     	{
			//lc640_write_int(EE_BAT1_ZAR_CNT,10);
			ind_pointer=0;
			ind=(i_enum)0;
			sub_ind=0;
			sub_ind1=0;
			sub_ind2=0;
			index_set=0;
			}
	     
			
		} 

	 else if(sub_ind==5)
	 	{
		if(but==butE_)	numOfForvardBps_init();
		}
				
	 else if(sub_ind==5)
	 	{
		if(but==butE_)
		{
		//can1_init(BITRATE62_5K6_25MHZ);
		//FullCAN_SetFilter(0,0x18e);
		LPC_CAN1->MOD&=~(1<<0);
		}
		}

	else if(sub_ind==1)
		{
		if(but==butU)
	     	{
	     	sub_ind1--;
	     	gran_char(&sub_ind1,0,1);
	     	}
		if(but==butD)
	     	{
	     	sub_ind1++;
	     	gran_char(&sub_ind1,0,1);
	     	}
		}		
		
		
			
     else if(but==butU)
	     {
	     index_set--;
	     gran_char(&index_set,0,4);
	     //lc640_write_int(ptr_ki_src[0],lc640_read_int(ptr_ki_src[0])+10);
	     }	
     else if(but==butD)
	     {
	     index_set++;
	     gran_char(&index_set,0,4); 
	     //lc640_write_int(ptr_ki_src[0],lc640_read_int(ptr_ki_src[0])-10);
	     }	
     else if(but==butE)
         	{
          //a=b[--ptr_ind];
          can1_out(1,2,3,4,5,6,7,8);
          }   
          
     else if(but==butE_)
         	{
          //a=b[--ptr_ind];
          can1_out_adr(TXBUFF,3);
          }                      				
	}

else if(ind==iMn_VD)
	{
	if(but==butD)
		{
		sub_ind++;
		gran_char(&sub_ind,0,6+NUMIST);
		}
		
	else if(but==butU)
		{
		sub_ind--;
		gran_char(&sub_ind,0,6+NUMIST);
		}	

	else if(but==butR)
		{
		//ind=iMn;
		sub_ind=0;

		}
	else if(but==butL)
		{
		//ind=iMn;
		sub_ind=0;

		}
	else if(but==butD_)
		{
		sub_ind=0;
		}

	else if(but==butLR_)
		{
		if(klbr_en)klbr_en=0;
		else klbr_en=1;
		}
				
	else if(but==butE)
		{
		if(sub_ind==0)
			{
			if(avar_ind_stat)
				{
				//ind=iAv_view;
				//sub_ind=0;
				tree_up(iAv_view,0,0,0);
				while(!(avar_ind_stat&(1<<sub_ind)))
					{
					sub_ind++;
					if(sub_ind>=32)
						{
						tree_down(0,0);
						avar_ind_stat=0;
						}
					}
				}																							
			}

		else if((sub_ind>1)&&(sub_ind<=(1+NUMIST)))
		    	{
		    	tree_up(iBps_elteh,0,0,sub_ind-2);
		    	} 


		else if(sub_ind==(2+NUMIST))
			{
	     	tree_up(iBps_list,0,0,0);
		    }		
		else if(sub_ind==(3+NUMIST))
			{
			tree_up(iSet_prl,0,0,0);
		    ret(50);
		    parol_init();
			}
		else if(sub_ind==(4+NUMIST))
			{
			if(but==butE_)avar_uout_hndl(1);
			tree_up(iLog,0,0,0);
		    ret(1000);
			}
		else if(sub_ind==(5+NUMIST))
			{
			sub_ind=0;
			}
		else if(sub_ind==(6+NUMIST))
			{
	     	tree_up(iPrltst,0,0,0);
		    parol_init();
			//tree_up(iRele_set,0,0,0);
			}
 		}
	else if(but==butU_)
		{
		if(sysTAvar==0)avar_sys_t_hndl(1,456);
		else avar_sys_t_hndl(0,789);
		}
    }



else if(ind==iMakb)
	{
	ret(1000);
	if (but==butU)
		{      
		sub_ind--;
		if(sub_ind>7)sub_ind=7;
		//else if(sub_ind==1)sub_ind=0;
		gran_char(&sub_ind,0,simax);
		}
		
	else if (but==butD)
		{
		sub_ind++;
		if(sub_ind<3)sub_ind=3;
		gran_char(&sub_ind,0,simax);
		}
	else if((but==butL)||((sub_ind==simax)&&(but==butE)))
	     {
	     tree_down(0,0);
	     ret(0);
	     }
	else if(but==butD_)
		{
		sub_ind=simax;
		}		    
	}

#ifndef _DEBUG_
else if(ind==iBps)
	{
	ret_ind(0,0,0);
	if (but==butU)
		{      
		sub_ind--;
		if(sub_ind==3)sub_ind=1;
		else if(sub_ind==1)sub_ind=0;
		gran_char(&sub_ind,0,simax);
		}
		
	else if (but==butD)
		{
		sub_ind++;
		if(sub_ind<3)sub_ind=3;
		gran_char(&sub_ind,0,simax);
		}
		
	else if((but==butE)&&(sub_ind==4))
		{
		can1_out(sub_ind1,sub_ind1,CMND,ALRM_RES,0,0,0,0);
		}
				
	else if(((but==butE)&&(sub_ind==5))||(but))
		{
	     tree_down(0,0);
	     ret(0);
		}	
		
	}	
else if(ind==iBps_elteh)
	{
	ret_ind(0,0,0);
	if (but==butU)
		{      
		sub_ind--;
		if(sub_ind==4)sub_ind=2;
		else if(sub_ind==1)sub_ind=0;
		gran_char(&sub_ind,0,simax);
		}
		
	else if (but==butD)
		{
		sub_ind++;
		if(sub_ind<3)sub_ind=3;
		if(sub_ind==4)index_set=4;
		if(sub_ind==5)sub_ind=7;
		gran_char(&sub_ind,0,simax);
		}
	else if (but==butR)
		{      
		//bps[sub_ind1]._x_++;
		//gran(&bps[sub_ind1]._x_,-50,50);
		}
		
/*	else if (but==butL)
		{      
		bps[sub_ind1]._x_--;
		gran(&bps[sub_ind1]._x_,-50,50);
		}*/
	
	else if((but==butE)&&(sub_ind==7))
		{
		can1_out(sub_ind1,sub_ind1,CMND,ALRM_RES,0,0,0,0);
		}
				
	else if(((but==butE)&&(sub_ind==8))||(but==butL))
		{
	     tree_down(0,0);
	     ret(0);
		}	
	else if((but==butE)&&(sub_ind==9))
		{
		can1_out(sub_ind1,sub_ind1,CMND,VENT_RES,0,0,0,0);
		}			
	}			
else if(ind==iNet)
	{
	ret(1000);
	if((but==butL)||(but==butE))
	     {
	     tree_down(0,0);
	     ret(0);
	     }
	}

else if(ind==iNet3)
	{
	ret(1000);
	ret_ind(0,0,0);
	if (but==butU)
		{      
		sub_ind--;
		gran_char(&sub_ind,0,4);
		}
		
	else if (but==butD)
		{
		sub_ind++;
		gran_char(&sub_ind,0,4);
		}
	else if((but==butL)||((but==butE)&&(sub_ind==4)))
	     {
	     tree_down(0,0);
	     ret(0);
	     }
	}
else if(ind==iNetEM)
	{
	ret(1000);
	ret_ind(0,0,0);
	if (but==butU)
		{      
		sub_ind--;
		gran_char(&sub_ind,0,4);
		}
		
	else if (but==butD)
		{
		sub_ind++;
		gran_char(&sub_ind,0,4);
		}
	else if((but==butL)||((but==butE)&&(sub_ind==4)))
	     {
	     tree_down(0,0);
	     ret(0);
	     }
	}

else if(ind==iLoad)
	{
	ret(1000);
	if((but==butL)||(but==butE))
	     {
	     tree_down(0,0);
	     ret(0);
	     }
	}	

else if(ind==iExtern)
	{
	ret(1000);
	if (but==butU)
		{      
		sub_ind--;
		gran_char(&sub_ind,0,8);
		}
		
	else if (but==butD)
		{
		sub_ind++;
		gran_char(&sub_ind,0,8);		
		}

	else if((but==butE)&&(sub_ind==8))
		{
	     tree_down(0,0);
	     ret(0);
		}	
	}

else if(ind==iVent)
	{
	ret(1000);
	if (but==butU)
		{      
		sub_ind--;
		gran_char(&sub_ind,1,2);
		}
		
	else if (but==butD)
		{
		sub_ind++;
		gran_char(&sub_ind,1,2);		
		}

	else if (sub_ind==1)
		{
          if((but==butR)||(but==butR_))
               {
               pos_vent++;
               }
          else if((but==butL)||(but==butL_))
               {
               pos_vent--;
               }

		gran(&pos_vent,1,11);
          lc640_write_int(EE_POS_VENT,pos_vent);		
		}
		
	else if((but==butE)&&(sub_ind==2))
		{
	     tree_down(0,0);
	     ret(0);
		}	
	}
else if(ind==iAvt)
	{
	ret(1000);
	if (but==butU)
		{      
		sub_ind--;
		gran_char(&sub_ind,0,NUMAVT);
		}
		
	else if (but==butD)
		{
		sub_ind++;
		gran_char(&sub_ind,0,NUMAVT);		
		}

	else if((but==butE)&&(sub_ind==NUMAVT))
		{
	     tree_down(0,0);
	     ret(0);
		}
	}
else if(ind==iEnerg)
	{
	ret(1000);
	if (but==butU)
		{      
		sub_ind--;
		gran_char(&sub_ind,0,4);
		}
		
	else if (but==butD)
		{
		sub_ind++;
		gran_char(&sub_ind,0,4);		
		}

	else if((but==butE)&&(sub_ind==4))
		{
	     tree_down(0,0);
	     ret(0);
		}
     }

else if(ind==iEnerg3)
	{
	ret(1000);
	if (but==butU)
		{      
		sub_ind--;
		gran_char(&sub_ind,0,8);
		}
		
	else if (but==butD)
		{
		sub_ind++;
		gran_char(&sub_ind,0,8);		
		}

	else if((but==butE)&&(sub_ind==8))
		{
	     tree_down(0,0);
	     ret(0);
		}
     }

else if((ind==iPrl_bat_in_out)||(ind==iSet_prl)||(ind==iK_prl)
	||(ind==iAusw_prl)
	||(ind==iPrltst))
	{
	ret(50);
	if(but==butR)
		{
		sub_ind++;
		gran_ring_char(&sub_ind,0,2);
		}
	else if(but==butL)
		{
		sub_ind--;
		gran_ring_char(&sub_ind,0,2);
		}	
	else if(but==butU)
		{
		parol[sub_ind]++;
		gran_ring_char(&parol[sub_ind],0,9);
		}	
	else if(but==butD)
		{
		parol[sub_ind]--;
		gran_ring_char(&parol[sub_ind],0,9);
		}	
	else if(but==butE)
		{
		unsigned short tempU;
		tempU=parol[2]+(parol[1]*10U)+(parol[0]*100U);
		
		if(ind==iPrl_bat_in_out)
		     {
		     if(BAT_IS_ON[sub_ind1]!=bisON)
		          {
		          if(tempU==PAROL_BAT_IN)
		               {
					//tree_up(iPrl_bat_in_sel,0,0,sub_ind1);
					
		               lc640_write_int(ADR_EE_BAT_IS_ON[sub_ind1],bisON);
					lc640_write_int(EE_BAT_TYPE,0);
		               lc640_write_int(ADR_EE_BAT_DAY_OF_ON[sub_ind1],LPC_RTC->DOM);
		               lc640_write_int(ADR_EE_BAT_MONTH_OF_ON[sub_ind1],LPC_RTC->MONTH);
		               lc640_write_int(ADR_EE_BAT_YEAR_OF_ON[sub_ind1],LPC_RTC->YEAR);
		               lc640_write_int(ADR_EE_BAT_C_REAL[sub_ind1],0x5555);
		               lc640_write_int(ADR_EE_BAT_RESURS[sub_ind1],0);
					lc640_write_int(ADR_EE_BAT_ZAR_CNT[sub_ind1],0);
		               
		               lc640_write(KE_PTR,0);
					lc640_write(VZ_PTR,0);
					lc640_write(WRK_PTR,0);
					lc640_write(KE_CNT,0);
					lc640_write(VZ_CNT,0);
					lc640_write(WRK_CNT,0);
					lc640_write(BAT_AVAR_CNT,0);
					lc640_write(BAT_AVAR_PTR,0);					
		               
                         tree_down(0,0);
                         ret(0); 
		               }
		          else
		               {
		               tree_down(0,0);
	    	               show_mess("                    ",
	          				"       ������       ",
	          				"     ��������!!!    ",
	          				"                    ",1000);
     	               }
		          }      
               else		          
		          {
		          if(tempU==PAROL_BAT_OUT)
		               {
		               lc640_write_int(ADR_EE_BAT_IS_ON[sub_ind1],bisOFF);
		               lc640_write_int(ADR_EE_BAT_DAY_OF_ON[sub_ind1],LPC_RTC->DOM);
		               lc640_write_int(ADR_EE_BAT_MONTH_OF_ON[sub_ind1],LPC_RTC->MONTH);
		               lc640_write_int(ADR_EE_BAT_YEAR_OF_ON[sub_ind1],LPC_RTC->YEAR);

		               tree_down(0,0);
		               ret(0);
		               
		               }
	               else
		               {
		               tree_down(0,0);
	    	               show_mess("                    ",
	          				"       ������       ",
	          				"     ��������!!!    ",
	          				"                    ",1000);
		               }		               
		          }     
               }
		
		else if(ind==iSet_prl)
			{
	     	if(tempU==PAROL_SET) 
				{
				tree_down(0,0);

				#ifdef UKU_VD
				tree_up(iSet_VD,0,0,0);
				#endif

				ret(1000);
				}
			else 
				{
		          tree_down(0,0);
	    	          show_mess("                    ",
	          			"       ������       ",
	          			"     ��������!!!    ",
	          			"                    ",1000);
				}
			}
		else	if(ind==iK_prl)
			{
	     	if(tempU==PAROL_KALIBR) 
				{
				tree_down(0,0);
				tree_up(iK_VD,0,0,0);
				ret(100);				
				}
			else 
				{
		          tree_down(0,0);
	    	          show_mess("                    ",
	          			"       ������       ",
	          			"     ��������!!!    ",
	          			"                    ",1000);
				}
			} 
	
		else	if(ind==iAusw_prl)
			{
	     	if(tempU==PAROL_AUSW) 
				{
				tree_down(0,0);
				tree_up(iAusw_set,1,0,0);
				default_temp=10;
				ret(0);
				}
			else 
				{
		          tree_down(0,0);
	    	          show_mess("                    ",
	          			"       ������       ",
	          			"     ��������!!!    ",
	          			"                    ",1000);
				}
			} 	
			
		else	if(ind==iSet_st_prl)
			{
	     	if(tempU==PAROL_DEFAULT) 
				{
	//			ind=iDefault;
				sub_ind=1;
				index_set=0;
				default_temp=10;
				}
			else 
				{
		          tree_down(0,0);
	    	          show_mess("                    ",
	          			"       ������       ",
	          			"     ��������!!!    ",
	          			"                    ",1000);
				}
			} 
						
		else if(ind==iPrltst)
			{
			if(tempU==PAROL_TST) 
				{
				tree_down(0,0);

				tree_up(iTst_VD,0,0,0);
				
				tst_state[0]=tstOFF;
				tst_state[1]=tstOFF;
				tst_state[2]=tstOFF;
				tst_state[3]=tstOFF;
				tst_state[4]=tstOFF;
				tst_state[5]=tstOFF;
				tst_state[6]=tstOFF;
				tst_state[7]=tstOFF;
				tst_state[9]=tstOFF;
				tst_state[10]=(enum_tst_state)0;
				ret(10000);				


				}
	  		else 
				{
		          tree_down(0,0);
	    	          show_mess("                    ",
	          			"       ������       ",
	          			"     ��������!!!    ",
	          			"                    ",1000);
				}
			}
		}
	}


#endif
else if(ind==iPrl_bat_in_sel)
	{
	ret(1000);
	if (but==butU)
		{
		sub_ind--;
		gran_char(&sub_ind,0,1);
		}
	else if (but==butD)
		{
		sub_ind++;
		gran_char(&sub_ind,0,1);
		}
	else if(but==butE)
		{
		if(sub_ind==0)
			{
			lc640_write_int(ADR_EE_BAT_IS_ON[sub_ind1],bisON);
		     lc640_write_int(ADR_EE_BAT_DAY_OF_ON[sub_ind1],LPC_RTC->DOM);
		     lc640_write_int(ADR_EE_BAT_MONTH_OF_ON[sub_ind1],LPC_RTC->MONTH);
		     lc640_write_int(ADR_EE_BAT_YEAR_OF_ON[sub_ind1],LPC_RTC->YEAR);
		     lc640_write_int(ADR_EE_BAT_C_REAL[sub_ind1],0x5555);
		     lc640_write_int(ADR_EE_BAT_RESURS[sub_ind1],0);
			lc640_write_int(ADR_EE_BAT_ZAR_CNT[sub_ind1],0);
			//lc640_write_int(ADR_EE_BAT_TYPE[sub_ind1],0);
		               
		     lc640_write(KE_PTR,0);
			lc640_write(VZ_PTR,0);
			lc640_write(WRK_PTR,0);
			lc640_write(KE_CNT,0);
			lc640_write(VZ_CNT,0);
			lc640_write(WRK_CNT,0);
			lc640_write(BAT_AVAR_CNT,0);
			lc640_write(BAT_AVAR_PTR,0);					
		               
               tree_down(-1,0);
               ret(0);
 
			}
		else if (sub_ind==1)
			{
			lc640_write_int(ADR_EE_BAT_IS_ON[sub_ind1],bisON);
		     lc640_write_int(ADR_EE_BAT_DAY_OF_ON[sub_ind1],LPC_RTC->DOM);
		     lc640_write_int(ADR_EE_BAT_MONTH_OF_ON[sub_ind1],LPC_RTC->MONTH);
		     lc640_write_int(ADR_EE_BAT_YEAR_OF_ON[sub_ind1],LPC_RTC->YEAR);
		     lc640_write_int(ADR_EE_BAT_C_REAL[sub_ind1],0x5555);
		     lc640_write_int(ADR_EE_BAT_RESURS[sub_ind1],0);
			lc640_write_int(ADR_EE_BAT_ZAR_CNT[sub_ind1],0);
			//lc640_write_int(ADR_EE_BAT_TYPE[sub_ind1],1);
		               
		     lc640_write(KE_PTR,0);
			lc640_write(VZ_PTR,0);
			lc640_write(WRK_PTR,0);
			lc640_write(KE_CNT,0);
			lc640_write(VZ_CNT,0);
			lc640_write(WRK_CNT,0);
			lc640_write(BAT_AVAR_CNT,0);
			lc640_write(BAT_AVAR_PTR,0);					
		               
               tree_down(-1,0);
               ret(0);
	    	     show_mess("    �� ��������     ",
	          		"  ��������� �����   ",
	          		"       �����        ",
	          		"     � ��������.    ",3000);			 
			}
		}
	}

else if(ind==iSpc)
	{
	ret(1000);
	if (but==butU)
		{
		sub_ind--;
		gran_char(&sub_ind,0,4);
		}
	else if (but==butD)
		{
		sub_ind++;
		gran_char(&sub_ind,0,4);
		}
	else if(but==butE)
		{
		if(sub_ind==0)
			{   
            //   tree_up(iSpc_prl_vz,0,0,0);
			parol_init();
			}
		else if(sub_ind==1)
			{
            //tree_up(iAvz,0,0,0);
            parol_init();
			}			
		else if((sub_ind==2)||(sub_ind==3))
			{
            //tree_up(iSpc_prl_ke,0,0,sub_ind-2);
            parol_init();
			} 
		else if(sub_ind==4)
			{
			tree_down(0,0);
			ret(0);
			}	
		}
	else if(but==butL)
		{
		tree_down(0,0);
		ret(0);
		}			
	}
else if(ind==iSpc_termocompensat)
	{
	ret(1000);
	if (but==butU)
		{
		sub_ind--;
		gran_char(&sub_ind,0,3);
		}
	else if (but==butD)
		{
		sub_ind++;
		gran_char(&sub_ind,0,3);
		}
	else if(but==butE)
		{
		if(sub_ind==0)
			{   
            //tree_up(iSpc_prl_vz,0,0,0);
			parol_init();
			}
		else if(sub_ind==1)
			{
            //tree_up(iAvz,0,0,0);
            parol_init();
			}			
		else if(sub_ind==2)
			{
            //tree_up(iSpc_prl_ke,0,0,0);
            parol_init();
			} 
		else if(sub_ind==3)
			{
			tree_down(0,0);
			ret(0);
			}	
		}
	else if(but==butL)
		{
		tree_down(0,0);
		ret(0);
		}
	}



else if(ind==iLog)
	{
	ret_ind_sec(0,0);
	ret_ind(0,0,0);
	if (but==butU)
		{
		sub_ind--;
		gran_char(&sub_ind,0,av_j_si_max+1);
		}
	else if (but==butD)
		{
		sub_ind++;
		gran_char(&sub_ind,0,av_j_si_max+1);
          
		}  

	else if (but==butD_)
		{
		sub_ind=av_j_si_max;
		} 
		 
	else if (but==butL)
		{
		tree_down(0,0);
		ret(0);
		}  
		
	else if(but==butE)
		{  
		if(sub_ind==av_j_si_max+1)
			{
			lc640_write(CNT_EVENT_LOG,0);
			lc640_write(PTR_EVENT_LOG,0);
			tree_down(0,0);
			avar_ind_stat=0;
			avar_stat=0;
			avar_stat_old=0;				
			}
					
		else if(sub_ind==av_j_si_max)
			{
			tree_down(0,0);
			ret(0);
			}
			
		else 
			{
			/*ind=iLog_;
			sub_ind1=sub_ind;
			index_set=0;
			sub_ind=0;*/
			tree_up(iLog_,0,0,sub_ind);
			}	
			
		} 

	else if(but==butR)
		{
	    //	avar_bat_hndl(0,1);	
		}
	else if(but==butR_)
		{
	    	//avar_bat_hndl(0,0);	
		}		
	else if(but==butL)
		{
	    	//avar_s_hndl(1,0,1);	
		}
				
	else if(but==butL_)
		{           
		/*lc640_write(CNT_EVENT_LOG,0);
		lc640_write(PTR_EVENT_LOG,0);
		ind=iMn;
		sub_ind=cnt_of_slave+10;
		index_set=0;*/				
	
		}	 		
	}

else if(ind==iLog_)
	{          
	if(but==butU)
		{
		index_set--;
		gran_char(&index_set,0,av_j_si_max);
		}
	else if(but==butD)
		{
		index_set++;
		gran_char(&index_set,0,av_j_si_max);
		}
	else 
		{
		/*ind=iLog;
		sub_ind=sub_ind1;*/
		tree_down(0,0/*sub_ind1-sub_ind*/);
		}		
	}	

else if(ind==iSet)
	{
	ret(1000);
	if(but==butD)
		{
		sub_ind++;
		if(sub_ind==7)index_set=6;
		if(sub_ind==8)sub_ind=9;
		if(sub_ind==11)index_set=10;
		if(sub_ind==12)sub_ind=13;
          if(sub_ind==32)
               {
               index_set=31;
               }
          if(sub_ind==34)
               {
               sub_ind=35;
               //index_set=31;
               }
		
		gran_char(&sub_ind,0,37);
		}
	else if(but==butU)
		{
		sub_ind--;
		if(sub_ind==8)sub_ind=7;
		if(sub_ind==12)sub_ind=9;
          if(sub_ind==33)
               {
               sub_ind=32;
		     //index_set=29;
               }
		gran_char(&sub_ind,0,37);
		}
	else if(but==butD_)
		{
		sub_ind=35;
		}
		
	else if(sub_ind==0)
	     {
	     if(but==butE)
	          {
			#ifdef UKU_GLONASS
	          tree_up(iDef_GLONASS,0,0,0);
			#endif
			#ifdef UKU_3U
	          tree_up(iDef_3U,0,0,0);
			#endif
			#ifdef UKU
	          tree_up(iDef,0,0,0);
			#endif
			#ifdef UKU_6U
	          tree_up(iDef_6U,0,0,0);
			#endif
	          ret(1000);
	          default_temp=10;
	          }


	     }	
	
     else if(sub_ind==1)
		{
		if(but==butE)
		     {
		     tree_up(iSet_T,0,0,0);
		     ret(1000);
		     phase=0;
		     }
		}	
					 
     else if(sub_ind==2)
		{
		if(but==butE)
		     {
			#ifdef UKU_3U
		     tree_up(iStr_3U,0,0,0);
		     ret(1000);
		     index_set=0;
			#endif
			#ifdef UKU_GLONASS
		     tree_up(iStr_GLONASS,0,0,0);
		     ret(1000);
		     index_set=0;
			#endif
			#ifdef UKU_RSTKM
		     tree_up(iStr_RSTKM,0,0,0);
		     ret(1000);
		     index_set=0;
			#endif
			#ifdef UKU_KONTUR
		     tree_up(iStr_KONTUR,0,0,0);
		     ret(1000);
		     index_set=0;
			#endif
			#ifdef UKU_6U
		     tree_up(iStr_6U,0,0,0);
		     ret(1000);
		     index_set=0;
			#endif

			#ifdef UKU
		     tree_up(iStr,0,0,0);
		     ret(1000);
		     index_set=0;
			#endif



		     }
		}	
	
	else if(sub_ind==3)
	     {
		if(but==butE)
		     {		
			tree_up(iKlimat,0,0,0);
			}
	     }

	else if(sub_ind==5)
	     {
	     if(but==butR)MNEMO_TIME++;
	     else if(but==butR_)MNEMO_TIME+=10;
	     else if(but==butL)MNEMO_TIME--;
	     else if(but==butL_)MNEMO_TIME-=10;

	     if(((MNEMO_TIME<5)||(MNEMO_TIME>60))&&(MNEMO_ON!=mnOFF))lc640_write_int(EE_MNEMO_ON,mnOFF);	
	     if(((MNEMO_TIME>=5)&&(MNEMO_TIME<=60))&&(MNEMO_ON!=mnON))lc640_write_int(EE_MNEMO_ON,mnON);
	     gran((signed short*)&MNEMO_TIME,4,61);
	     lc640_write_int(EE_MNEMO_TIME,MNEMO_TIME);
	     speed=1;
	     }
				     		
	else if(sub_ind==6)
	     {
		if(ZV_ON)ZV_ON=0;
		else ZV_ON=1;
	     lc640_write_int(EE_ZV_ON,ZV_ON);
	     speed=1;
	     }	
	
	else if(sub_ind==7)
	     {
		if(AV_OFF_AVT)AV_OFF_AVT=0;
		else AV_OFF_AVT=1;
	     lc640_write_int(EE_AV_OFF_AVT,AV_OFF_AVT);
	     speed=1;
	     }	

	else if(sub_ind==9)
	     {
	     if(but==butE)
	          {
	          tree_up(iApv,0,0,0);
	          ret(1000);
	          }
	     }	

	else if(sub_ind==10)
	     {
//		if(PAR)PAR=0;
//		else PAR=1;
//	     lc640_write_int(EE_PAR,PAR);
	     speed=1;
	     }

	else if(sub_ind==11)
	     {
	     if(but==butR)TBAT++;
	     else if(but==butR_)TBAT+=10;
	     else if(but==butL)TBAT--;
	     else if(but==butL_)TBAT-=10;
	     gran(&TBAT,5,60);
	     lc640_write_int(EE_TBAT,TBAT);
	     speed=1;
	     }	
	                    	     	
	else if(sub_ind==13)
	     {
	     if(but==butR)UMAX++;
	     else if(but==butR_)UMAX+=10;
	     else if(but==butL)UMAX--;
	     else if(but==butL_)UMAX-=10;
		#ifdef UKU206_220
		gran(&UMAX,2000,3000);
		#else
	     gran(&UMAX,10,1000);
		#endif
	     lc640_write_int(EE_UMAX,UMAX);
	     speed=1;
	     }
	else if(sub_ind==14)
	     {
/*	     if(but==butR)DU++;
	     else if(but==butR_)DU+=10;
	     else if(but==butL)DU--;
	     else if(but==butL_)DU-=10;
	     gran(&DU,10,1000);*/
/*	     if(but==butL)DU++;
	     else if(but==butL_)DU+=10;
	     else if(but==butR)DU--;
	     else if(but==butR_)DU-=10;
	     gran(&DU,50,UB20-250);
	     lc640_write_int(EE_DU,DU);
	     speed=1;*/
	     }	     
	else if(sub_ind==15)
	     {
	     /*if(but==butR)UB0++;
	     else if(but==butR_)UB0+=10;
	     else if(but==butL)UB0--;
	     else if(but==butL_)UB0-=10;
		gran(&UB0,400,800);
          lc640_write_int(EE_UB0,UB0);
	     speed=1; */
	     }
	     
	else if(sub_ind==16)
	     {
/*	     if(but==butR)UB20++;
	     else if(but==butR_)UB20+=10;
	     else if(but==butL)UB20--;
	     else if(but==butL_)UB20-=10;
		gran(&UB20,400,800);
	     lc640_write_int(EE_UB20,UB20);
	     speed=1; */
	     }	

	else if(sub_ind==17)
	     {
	     if(but==butR)USIGN++;
	     else if(but==butR_)USIGN+=10;
	     else if(but==butL)USIGN--;
	     else if(but==butL_)USIGN-=10;
		gran(&USIGN,40,100);
	     lc640_write_int(EE_USIGN,USIGN);
	     speed=1;
	     }	
	else if(sub_ind==18)
	     {
	     if(but==butR)UMN++;
	     else if(but==butR_)UMN+=10;
	     else if(but==butL)UMN--;
	     else if(but==butL_)UMN-=10;
	     gran(&UMN,1,220);
	     lc640_write_int(EE_UMN,UMN);
	     speed=1;
	     }	

	else if(sub_ind==19)
	     {
	     if(but==butR)U0B++;
	     else if(but==butR_)U0B+=10;
	     else if(but==butL)U0B--;
	     else if(but==butL_)U0B-=10;
		gran(&U0B,400,800);
	     lc640_write_int(EE_U0B,U0B);
	     speed=1;
	     }	
	     
	else if(sub_ind==20)
	     {
	     if(but==butR)IKB++;
	     else if(but==butR_)IKB+=10;
	     else if(but==butL)IKB--;
	     else if(but==butL_)IKB-=10;
	     gran(&IKB,1,500);
	     lc640_write_int(EE_IKB,IKB);
	     speed=1;
	     }		
            
	else if(sub_ind==21)
	     {
	     if(but==butR)IZMAX++;
	     else if(but==butR_)IZMAX+=10;
	     else if(but==butL)IZMAX--;
	     else if(but==butL_)IZMAX-=10;
		gran(&IZMAX,10,1000);
	     lc640_write_int(EE_IZMAX,IZMAX);
	     speed=1;
	     }   

	else if(sub_ind==22)
	     {
	     if(but==butR)IMAX++;
	     else if(but==butR_)IMAX+=10;
	     else if(but==butL)IMAX--;
	     else if(but==butL_)IMAX-=10;
		gran(&IMAX,10,1000);
	     lc640_write_int(EE_IMAX,IMAX);
	     speed=1;
	     }		
	     
	else if(sub_ind==23)
	     {
	     if(but==butR)IMIN++;
	     else if(but==butR_)IMIN+=10;
	     else if(but==butL)IMIN--;
	     else if(but==butL_)IMIN-=10;
	     gran(&IMIN,40,IMAX-10);
	     lc640_write_int(EE_IMIN,IMIN);
	     speed=1;
	     }
	
	else if(sub_ind==24)
	     {
/*	     if ((but==butR)||(but==butR_))UVZ+=1;
		if ((but==butL)||(but==butL_))UVZ-=1;
		gran(&UVZ,UB20,UMAX); 	          
		lc640_write_int(EE_UVZ,UVZ);
	     speed=1;					 */
	     }
	     
	else if(sub_ind==25)
		{
		if ((but==butR)||(but==butR_))TZAS++;
		if ((but==butL)||(but==butL_))TZAS--;
		gran(&TZAS,0,3);
		lc640_write_int(EE_TZAS,TZAS);
		speed=1; 
		}	
			       	        
	else if(sub_ind==26)
	     {
	     if(but==butR)TMAX++;
	     else if(but==butR_)TMAX+=2;
	     else if(but==butL)TMAX--;
	     else if(but==butL_)TMAX-=2;
	     gran(&TMAX,10,100);
	     lc640_write_int(EE_TMAX,TMAX);
	     speed=1;
	     }	
	
	else if(sub_ind==27)
	     {
	     if(but==butR)TSIGN++;
	     else if(but==butR_)TSIGN+=2;
	     else if(but==butL)TSIGN--;
	     else if(but==butL_)TSIGN-=2;
	     gran(&TSIGN,10,100);
	     lc640_write_int(EE_TSIGN,TSIGN);
	     speed=1;
	     }	     
	else if(sub_ind==28)
	     {
	     if(but==butR)TBATMAX++;
	     else if(but==butR_)TBATMAX+=2;
	     else if(but==butL)TBATMAX--;
	     else if(but==butL_)TBATMAX-=2;
	     gran(&TBATMAX,10,100);
	     lc640_write_int(EE_TBATMAX,TBATMAX);
	     speed=1;
	     }	
	
	else if(sub_ind==29)
	     {
	     if(but==butR)TBATSIGN++;
	     else if(but==butR_)TBATSIGN+=2;
	     else if(but==butL)TBATSIGN--;
	     else if(but==butL_)TBATSIGN-=2;
	     gran(&TBATSIGN,10,100);
	     lc640_write_int(EE_TBATSIGN,TBATSIGN);
	     speed=1;
	     }	
     	     	     		     	     
    else if(sub_ind==30)
		{
		if(but==butE)
		     {
		     tree_up(iExt_set,0,0,0);
		     ret(1000);
		     }
		}

    else if(sub_ind==31)
		{
		if(but==butE)
		     {
		     tree_up(iLan_set,0,0,0);
		     ret(1000);
		     }
		}
	else if(sub_ind==32)
	     {
	     if(but==butR)POWER_CNT_ADRESS++;
	     else if(but==butR_)POWER_CNT_ADRESS+=10;
	     else if(but==butL)POWER_CNT_ADRESS--;
	     else if(but==butL_)POWER_CNT_ADRESS-=10;
	     gran(&POWER_CNT_ADRESS,0,10000);
	     lc640_write_int(EE_POWER_CNT_ADRESS,POWER_CNT_ADRESS);
	     speed=1;
	     } 
	else if(sub_ind==33)
	     {
	     if(but==butR)UBM_AV++;
	     else if(but==butR_)UBM_AV++;
	     else if(but==butL)UBM_AV--;
	     else if(but==butL_)UBM_AV--;
	     gran(&UBM_AV,0,50);
	     lc640_write_int(EE_UBM_AV,UBM_AV);
	     speed=1;
	     }

	else if(sub_ind==34)
	     {
	     if(but==butR)AUSW_MAIN_NUMBER++;
	     else if(but==butR_)AUSW_MAIN_NUMBER+=20;
	     else if(but==butL)AUSW_MAIN_NUMBER--;
	     else if(but==butL_)AUSW_MAIN_NUMBER-=20;
		else if(but==butEL_)AUSW_MAIN_NUMBER=15000;
		if(AUSW_MAIN_NUMBER<13000)AUSW_MAIN_NUMBER=100000;
		if(AUSW_MAIN_NUMBER>100000)AUSW_MAIN_NUMBER=13000;
	     lc640_write_int(EE_AUSW_MAIN_NUMBER,(short)(AUSW_MAIN_NUMBER&0x0000ffffUL));
		lc640_write_int(EE_AUSW_MAIN_NUMBER+2,(short)((AUSW_MAIN_NUMBER&0xffff0000UL)>>16UL));
	     speed=1;
	     }                    		
 
     else if((sub_ind==36) || (sub_ind==4))
		{
		if(but==butE)
		     {
		     tree_down(0,0);
		     ret(0);
		     }
		}
				
	else if(sub_ind==37)
		{
		if(but==butE)
		     {		
			tree_up(iK_prl,0,0,0);
			parol_init();
			ret(50);
			}						
		}
		/*			
	else if(sub_ind==37)
		{
		if(but==butE)
		     {
		     tree_up(iPrltst,0,0,0);
		     parol_init();
		     }
		}*/	
     }




else if(ind==iSet_VD)
	{
	ret(1000);
	if(but==butD)
		{
		sub_ind++;
		if(sub_ind==5)index_set=6;
		if(sub_ind==6)sub_ind=7;
		if(sub_ind==20)
        	{
			index_set=18;
			}
		if(sub_ind==21)
        	{
			sub_ind=22;
            index_set=19;
            }
		if(sub_ind==25)
        	{
			//sub_ind=27;
            index_set=24;
            }
		if(sub_ind==26)
        	{
			sub_ind=27;
            //index_set=19;
            }
		if(sub_ind==28)
        	{
			sub_ind=29;
            //index_set=19;
            }																			
		gran_char(&sub_ind,0,30);
		}
	else if(but==butU)
		{
		sub_ind--;
		if(sub_ind==6)
			{
			sub_ind=5;
			}
		if(sub_ind==21)
			{
			sub_ind=20;
			}
		if(sub_ind==26)
        	{
			sub_ind=25;
            //index_set=19;
            }
		if(sub_ind==28)
        	{
			sub_ind=27;
            //index_set=19;
            }
		gran_char(&sub_ind,0,30);
		}
	else if(but==butD_)
		{
		sub_ind=29;
		}

	else if(but==butLR_)
		{
		lc640_write_int(EE_SPEED_CHRG_CURR,20);
		lc640_write_int(EE_SPEED_CHRG_VOLT,2400);
		lc640_write_int(EE_SPEED_CHRG_TIME,1);
		lc640_write_int(EE_SPEED_CHRG_D_U,50);
		lc640_write_int(EE_SPEED_CHRG_AVT_EN,0);
		lc640_write_int(EE_SPEED_CHRG_BLOCK_SRC,1);
		lc640_write_int(EE_SPEED_CHRG_BLOCK_LOG,1);
		lc640_write_int(EE_AUSW_MAIN_NUMBER,20000);
		}
		
	else if(sub_ind==0)
	     {
	     if(but==butE)
	        {
	          //tree_up(iDef_220_IPS_TERMOKOMPENSAT,0,0,0);
	          //ret(1000);
	          //default_temp=10;

			lc640_write_int(EE_ZV_ON,1);
			lc640_write_int(EE_AV_OFF_AVT,1);
			lc640_write_int(EE_APV_ON1,apvON);
			lc640_write_int(EE_APV_ON2,apvON);
			lc640_write_int(EE_APV_ON2_TIME,1);
			lc640_write_int(EE_UOUT,2300);
			lc640_write_int(EE_UAVT,2300);
			lc640_write_int(EE_TSIGN,70);
			lc640_write_int(EE_TMAX,80);
			lc640_write_int(EE_TSYSMAX,50);
			lc640_write_int(EE_DU,200);
			lc640_write_int(EE_UMAX,700);
			lc640_write_int(EE_UINMAX,2500);
			lc640_write_int(EE_UINMIN,1700);
			lc640_write_int(EE_UOUTMAX,2500);
			lc640_write_int(EE_UOUTMIN,2200);
			lc640_write_int(EE_TZAS,5);
			lc640_write_int(EE_TVENTMAX,6000);
			lc640_write_int(ADR_EE_RELE_SET_MASK[0],0x01);
			lc640_write_int(ADR_EE_RELE_SET_MASK[1],0x78);
			lc640_write_int(ADR_EE_RELE_SET_MASK[2],0x02);
			lc640_write_int(ADR_EE_RELE_SET_MASK[3],0x84);

	        }
		}	
	
     else if(sub_ind==1)
		{
		if(but==butE)
		     {
		     tree_up(iSet_T,0,0,0);
		     ret(1000);
		     phase=0;
		     }
		}	
					 
     else if(sub_ind==2)
		{
		if(but==butE)
		     {
		     tree_up(iStr_VD,0,0,0);
		     ret(1000);
		     index_set=0;
 		     }
		}	
	
	else if(sub_ind==4)
	     {
		if(ZV_ON)ZV_ON=0;
		else ZV_ON=1;
	     lc640_write_int(EE_ZV_ON,ZV_ON);
	     speed=1;
	     }	
	
	else if(sub_ind==5)
	     {
		if(AV_OFF_AVT)AV_OFF_AVT=0;
		else AV_OFF_AVT=1;
	     lc640_write_int(EE_AV_OFF_AVT,AV_OFF_AVT);
	     speed=1;
	     }	

	else if(sub_ind==7)
	     {
	     if(but==butE)
	          {
	          tree_up(iApv,0,0,0);
	          ret(1000);
	          }
	     }	
 
	else if(sub_ind==8)
	     {
	     if(but==butR)UOUT++;
	     else if(but==butR_)UOUT+=10;
	     else if(but==butL)UOUT--;
	     else if(but==butL_)UOUT-=10;

	     gran(&UOUT,2250,2350);

	     lc640_write_int(EE_UOUT,UOUT);
	     speed=1;
	     }
	else if(sub_ind==9)
	     {
	     if(but==butR)UAVT++;
	     else if(but==butR_)UAVT+=10;
	     else if(but==butL)UAVT--;
	     else if(but==butL_)UAVT-=10;

	     gran(&UAVT,2250,2350);

	     lc640_write_int(EE_UAVT,UAVT);
	     speed=1;
	     }
		 	     
	else if(sub_ind==10)
	     {
	     if(but==butR)TSIGN++;
	     else if(but==butR_)TSIGN+=2;
	     else if(but==butL)TSIGN--;
	     else if(but==butL_)TSIGN-=2;
	     gran(&TSIGN,10,100);
	     lc640_write_int(EE_TSIGN,TSIGN);
	     speed=1;
	     }

 	else if(sub_ind==11)
	     {
	     if(but==butR)TMAX++;
	     else if(but==butR_)TMAX+=2;
	     else if(but==butL)TMAX--;
	     else if(but==butL_)TMAX-=2;
	     gran(&TMAX,10,100);
	     lc640_write_int(EE_TMAX,TMAX);
	     speed=1;
	     }

 	else if(sub_ind==12)
	     {
	     if(but==butR)TSYSMAX++;
	     else if(but==butR_)TSYSMAX+=2;
	     else if(but==butL)TSYSMAX--;
	     else if(but==butL_)TSYSMAX-=2;
	     gran(&TSYSMAX,10,100);
	     lc640_write_int(EE_TSYSMAX,TSYSMAX);
	     speed=1;
	     }		 	
	else if(sub_ind==13)
	    {
		if(but==butR)DU++;
	    else if(but==butR_)DU+=10;
	    else if(but==butL)DU--;
	    else if(but==butL_)DU-=10;
	    gran(&DU,50,1000);
	    lc640_write_int(EE_DU,DU);
	    speed=1;
	    }
		 	
	else if(sub_ind==14)
		{
	    if(but==butR)UMAX++;
	    else if(but==butR_)UMAX+=10;
	    else if(but==butL)UMAX--;
	    else if(but==butL_)UMAX-=10;
		gran(&UMAX,10,3000);
		lc640_write_int(EE_UMAX,UMAX);
	    speed=1;
	    }

	else if(sub_ind==15)
		{
	    if(but==butR)UINMAX=(UINMAX*10+100)/10;
	    else if(but==butR_)UINMAX=(UINMAX*10+1000)/10;
	    else if(but==butL)UINMAX=(UINMAX*10-100)/10;
	    else if(but==butL_)UINMAX=(UINMAX*10-1000)/10;
		gran(&UINMAX,10,3000);
		lc640_write_int(EE_UINMAX,UINMAX);
	    speed=1;
	    }

	else if(sub_ind==16)
		{
	    if(but==butR)UINMIN=(UINMIN*10+100)/10;
	    else if(but==butR_)UINMIN=(UINMIN*10+1000)/10;
	    else if(but==butL)UINMIN=(UINMIN*10-100)/10;
	    else if(but==butL_)UINMIN=(UINMIN*10-1000)/10;
		gran(&UINMIN,10,3000);
		lc640_write_int(EE_UINMIN,UINMIN);
	    speed=1;
	    }

	else if(sub_ind==17)
		{
	    if(but==butR)UOUTMAX=(UOUTMAX*10+100)/10;
	    else if(but==butR_)UOUTMAX=(UOUTMAX*10+1000)/10;
	    else if(but==butL)UOUTMAX=(UOUTMAX*10-100)/10;
	    else if(but==butL_)UOUTMAX=(UOUTMAX*10-1000)/10;
		gran(&UOUTMAX,10,3000);
		lc640_write_int(EE_UOUTMAX,UOUTMAX);
	    speed=1;
	    }

	else if(sub_ind==18)
		{
	    if(but==butR)UOUTMIN=(UOUTMIN*10+100)/10;
	    else if(but==butR_)UOUTMIN=(UOUTMIN*10+1000)/10;
	    else if(but==butL)UOUTMIN=(UOUTMIN*10-100)/10;
	    else if(but==butL_)UOUTMIN=(UOUTMIN*10-1000)/10;
		gran(&UOUTMIN,10,3000);
		lc640_write_int(EE_UOUTMIN,UOUTMIN);
	    speed=1;
	    }

	else if(sub_ind==19)
		{
		if ((but==butR)||(but==butR_))TZAS++;
		if ((but==butL)||(but==butL_))TZAS--;
		gran(&TZAS,3,60);
		lc640_write_int(EE_TZAS,TZAS);
		speed=1; 
		}

	else if(sub_ind==20)
		{
	    if(but==butR)		TVENTMAX=((TVENTMAX/100)+1)*100;
	    else if(but==butR_)	TVENTMAX=((TVENTMAX/100)+1)*100;
	    else if(but==butL)	TVENTMAX=((TVENTMAX/100)-1)*100;
	    else if(but==butL_)	TVENTMAX=((TVENTMAX/100)-1)*100;
		else if(but==butE_)	TVENTMAX=1500;
		gran(&TVENTMAX,1,6000);
		lc640_write_int(EE_TVENTMAX,TVENTMAX);
	    speed=1;
	    }

	else if(sub_ind==22)
		{
		if(but==butE)
		    {		
			tree_up(iRele_set,0,0,0);
			ret(500);
			}						
		}
  	else if(sub_ind==23)
		{
		if(but==butE) 
		     {
		     tree_up(iLan_set,0,0,0);
		     ret(1000);
		     }
		}
  	else if(sub_ind==24)
	    {
	     if((but==butR)||(but==butR_))
	     	{
	     	MODBUS_ADRESS++;
	     	gran(&MODBUS_ADRESS,1,100);
	     	lc640_write_int(EE_MODBUS_ADRESS,MODBUS_ADRESS);
			speed=1;
	     	}
	     
	     else if((but==butL)||(but==butL_))
	     	{
	     	MODBUS_ADRESS--;
	     	gran(&MODBUS_ADRESS,1,100);
	     	lc640_write_int(EE_MODBUS_ADRESS,MODBUS_ADRESS);
			speed=1;
	     	}
          }

     else if(sub_ind==25)
	     {
	     if((but==butR)||(but==butR_))
	     	{
			if(MODBUS_BAUDRATE==120)MODBUS_BAUDRATE=240;
			else if(MODBUS_BAUDRATE==240)MODBUS_BAUDRATE=480;
	     	else if(MODBUS_BAUDRATE==480)MODBUS_BAUDRATE=960;
			else if(MODBUS_BAUDRATE==960)MODBUS_BAUDRATE=1920;
			else if(MODBUS_BAUDRATE==1920)MODBUS_BAUDRATE=3840;
			else if(MODBUS_BAUDRATE==3840)MODBUS_BAUDRATE=5760;
			//else if(MODBUS_BAUDRATE==3840)MODBUS_BAUDRATE=5760;
			else if(MODBUS_BAUDRATE==5760)MODBUS_BAUDRATE=11520;
			else MODBUS_BAUDRATE=120;
	     	gran(&MODBUS_BAUDRATE,120,11520);
	     	lc640_write_int(EE_MODBUS_BAUDRATE,MODBUS_BAUDRATE);
	     	}
	     
	     else if((but==butL)||(but==butL_))
	     	{
			if(MODBUS_BAUDRATE==120)MODBUS_BAUDRATE=11520;
			else if(MODBUS_BAUDRATE==240)MODBUS_BAUDRATE=120;
	     	else if(MODBUS_BAUDRATE==480)MODBUS_BAUDRATE=240;
			else if(MODBUS_BAUDRATE==960)MODBUS_BAUDRATE=480;
			else if(MODBUS_BAUDRATE==1920)MODBUS_BAUDRATE=960;
			else if(MODBUS_BAUDRATE==3840)MODBUS_BAUDRATE=1920;
			//else if(MODBUS_BAUDRATE==3840)MODBUS_BAUDRATE=3840;
			else if(MODBUS_BAUDRATE==5760)MODBUS_BAUDRATE=3840;
			else MODBUS_BAUDRATE=11520;
	     	gran(&MODBUS_BAUDRATE,120,11520);
	     	lc640_write_int(EE_MODBUS_BAUDRATE,MODBUS_BAUDRATE);
	     	}
          }

     else if(sub_ind==27)
	     {
	     if((but==butR)||(but==butR_))
	     	{
	     	I_LOAD_MODE=1;
	     	lc640_write_int(EE_I_LOAD_MODE,I_LOAD_MODE);
	     	}
	     
	     else if((but==butL)||(but==butL_))
	     	{
	     	I_LOAD_MODE=0;
	     	lc640_write_int(EE_I_LOAD_MODE,I_LOAD_MODE);
	     	}
          }

    else if((sub_ind==29) || (sub_ind==3))
		{
		if(but==butE)
		     {
		     tree_down(0,0);
		     ret(0);
		     }
		}
				
	else if(sub_ind==30)
		{
		if(but==butE)
		     {		
			tree_up(iK_prl,0,0,0);
			parol_init();
			ret(50);
			}						
		}
     }


else if (ind==iDef_220_IPS_TERMOKOMPENSAT)
	{
	simax=4;
	ret(1000);
	if(but==butD)
		{
		sub_ind++;
		gran_char(&sub_ind,0,simax);
		}
	else if(but==butU)
		{
		sub_ind--;
		gran_char(&sub_ind,0,simax);
		}
	else if(but==butD_)
		{
		sub_ind=simax;
		}
	
	else if(but==butE)
		{
		if(sub_ind==0)
			{
			def_ips_set(220);
			//lc640_write_int(EE_DU,2315-1850);
			lc640_write_int(EE_U_AVT,2200);
			lc640_write_int(EE_IZMAX,50);
			//lc640_write_int(EE_IMAX,12);
			//lc640_write_int(EE_IMIN,8);
			lc640_write_int(EE_AUSW_MAIN,22043);
			lc640_write_int(EE_NUMIST,3);

			lc640_write_int(EE_PAR,0);
			//lc640_write_int(EE_UMAX,2550);
			lc640_write_int(EE_DU,2315-1110);
//			lc640_write_int(EE_UB0,2397);
//			lc640_write_int(EE_UB20,2314);
			//lc640_write_int(EE_IZMAX,20);
			//lc640_write_int(EE_UVZ,2346);
			lc640_write_int(EE_UBM_AV,0);
			lc640_write_int(EE_U_OUT_KONTR_MAX,2420);
			lc640_write_int(EE_U_OUT_KONTR_MIN,2200);

			}
		else if(sub_ind==1)
			{
			def_ips_set(220);
			lc640_write_int(EE_DU,2315-1850);
			lc640_write_int(EE_U_AVT,2200);
			//lc640_write_int(EE_IZMAX,50);
			lc640_write_int(EE_AUSW_MAIN,22011);
			lc640_write_int(EE_NUMIST,2);
			lc640_write_int(EE_TERMOKOMP,1);
			//lc640_write_int(EE_IMAX,30);
		     //lc640_write_int(EE_IMIN,24);
			//lc640_write_int(EE_UMAX,2450);
	/*		}
		else if(sub_ind==3)
			{ */
		//	def_ips_set(220);
			//lc640_write_int(EE_DU,2315-1850);
		//	lc640_write_int(EE_U_AVT,2200);
			//lc640_write_int(EE_IZMAX,10);
			//lc640_write_int(EE_IMAX,30);
		//	lc640_write_int(EE_IMIN,24);
			//lc640_write_int(EE_AUSW_MAIN,22023);
			lc640_write_int(EE_NUMIST,2);

			lc640_write_int(EE_PAR,0);
			lc640_write_int(EE_UMAX,2550);
			lc640_write_int(EE_DU,2315-1110);
			//lc640_write_int(EE_UB0,2397);
			//lc640_write_int(EE_UB20,2314);
			lc640_write_int(EE_IZMAX,50);
			//lc640_write_int(EE_UVZ,2440);
			lc640_write_int(EE_UBM_AV,0);
			lc640_write_int(EE_UMAX,2450);
			lc640_write_int(EE_U_OUT_KONTR_MAX,2420);
			lc640_write_int(EE_U_OUT_KONTR_MIN,2200);

			}
		else if(sub_ind==2)
			{
			def_ips_set(110);
			//lc640_write_int(EE_DU,2315-1850);
			//lc640_write_int(EE_U_AVT,2200);
			//lc640_write_int(EE_IZMAX,50);
			//lc640_write_int(EE_IMAX,12);
			//lc640_write_int(EE_IMIN,8);
			lc640_write_int(EE_AUSW_MAIN,22043);
			//lc640_write_int(EE_NUMIST,3);

			//lc640_write_int(EE_PAR,0);
			//lc640_write_int(EE_UMAX,2550);
			//lc640_write_int(EE_DU,2315-1110);
			//lc640_write_int(EE_UB0,2397);
			//lc640_write_int(EE_UB20,2314);
			//lc640_write_int(EE_IZMAX,20);
			//lc640_write_int(EE_UVZ,2346);
			//lc640_write_int(EE_UBM_AV,0);
			//lc640_write_int(EE_U_OUT_KONTR_MAX,2420);
			//lc640_write_int(EE_U_OUT_KONTR_MIN,2200);

			}
		else if(sub_ind==3)
			{
			def_ips_set(220);
			//lc640_write_int(EE_DU,2315-1850);
			lc640_write_int(EE_U_AVT,2200);
			//lc640_write_int(EE_IZMAX,50);
			lc640_write_int(EE_AUSW_MAIN,22018);
			lc640_write_int(EE_NUMIST,2);
			lc640_write_int(EE_TERMOKOMP,1);
			//lc640_write_int(EE_IMAX,30);
		     //lc640_write_int(EE_IMIN,24);
			//lc640_write_int(EE_UMAX,2450);
	/*		}
		else if(sub_ind==3)
			{ */
		//	def_ips_set(220);
			//lc640_write_int(EE_DU,2315-1850);
		//	lc640_write_int(EE_U_AVT,2200);
			//lc640_write_int(EE_IZMAX,10);
			//lc640_write_int(EE_IMAX,30);
		//	lc640_write_int(EE_IMIN,24);
			//lc640_write_int(EE_AUSW_MAIN,22023);
			lc640_write_int(EE_NUMIST,2);

			lc640_write_int(EE_PAR,0);
			lc640_write_int(EE_UMAX,2700);
			lc640_write_int(EE_DU,2450-1200);
//			lc640_write_int(EE_UB0,2590);
//			lc640_write_int(EE_UB20,2450);
			lc640_write_int(EE_IZMAX,50);
			//lc640_write_int(EE_UVZ,2590);
			lc640_write_int(EE_UBM_AV,0);
			//lc640_write_int(EE_UMAX,2450);
			lc640_write_int(EE_U_OUT_KONTR_MAX,2420);
			lc640_write_int(EE_U_OUT_KONTR_MIN,2200);

			}
		else if(sub_ind==simax)
			{
			tree_down(0,0);
			}
		default_temp=sub_ind;	
		}
     }

else if(ind==iSet_T)
	{
	signed char temp;
	if(but==butR)
		{
		sub_ind++;
		gran_char(&sub_ind,0,5);
		}
	else if(but==butL)
		{
		sub_ind--;
		gran_char(&sub_ind,0,5);
		}
	else if(but==butE)
		{
		tree_down(0,0);
		}	
	else if(sub_ind==0)
	     {			    
	     temp=LPC_RTC->HOUR;
	     if((but==butU)||(but==butU_))
	          {
	          temp++;
	          gran_ring_char(&temp,0,23);
	          LPC_RTC->HOUR=temp;
	          }
          else if((but==butD)||(but==butD_))
	          {
	          temp--;
	          gran_ring_char(&temp,0,23);
	          LPC_RTC->HOUR=temp;
	          }	
	     speed=1;               
	     }
     else if(sub_ind==1)
	     {
	     temp=LPC_RTC->MIN;
	     if((but==butU)||(but==butU_))
	          {
	          temp++;
	          gran_ring_char(&temp,0,59);
	          LPC_RTC->MIN=temp;
	          }
          else if((but==butD)||(but==butD_))
	          {
	          temp--;
	          gran_ring_char(&temp,0,59);
	          LPC_RTC->MIN=temp;
	          }	
	     speed=1;               
	     }
     else if(sub_ind==2)
	     {				  
	     temp=LPC_RTC->SEC;
	     if((but==butU)||(but==butU_))
	          {
	          temp=0;
	          gran_ring_char(&temp,0,59);
	          LPC_RTC->SEC=temp;
	          }
          else if((but==butD)||(but==butD_))
	          {
	          temp=0;
	          gran_ring_char(&temp,0,59);
	          LPC_RTC->SEC=temp;
	          }	
	     speed=1;               
	     }

     else if(sub_ind==3)
	     {
	     temp=LPC_RTC->DOM;
	     if((but==butU)||(but==butU_))
	          {
	          temp++;
	          gran_ring_char(&temp,1,31);
	          LPC_RTC->DOM=temp;
	          }
          else if((but==butD)||(but==butD_))
	          {
	          temp--;
	          gran_ring_char(&temp,1,31);
	          LPC_RTC->DOM=temp;
	          }	
	     speed=1;               
	     }
     else if(sub_ind==4)
	     {
	     temp=LPC_RTC->MONTH;
	     if((but==butU)||(but==butU_))
	          {
	          temp++;
	          gran_ring_char(&temp,1,12);
	          LPC_RTC->MONTH=temp;
	          }
          else if((but==butD)||(but==butD_))
	          {
	          temp--;
	          gran_ring_char(&temp,1,12);
	          LPC_RTC->MONTH=temp;
	          }	
	     speed=1;               
	     }	  
     else if(sub_ind==5)
	     {
	     temp=LPC_RTC->YEAR;
	     if((but==butU)||(but==butU_))
	          {
	          temp++;
	          gran_ring_char(&temp,0,99);
	          LPC_RTC->YEAR=temp;
	          }
          else if((but==butD)||(but==butD_))
	          {
	          temp--;
	          gran_ring_char(&temp,0,99);
	          LPC_RTC->YEAR=temp;
	          }	
	     speed=1;               
	     }		        
	}  

else if(ind==iStr)
	{
	ret(1000);
	if(but==butD)
		{
		sub_ind++;
		if(sub_ind==2)index_set=1;
		if(sub_ind==3)sub_ind++;
		gran_char(&sub_ind,1,5);	

		}
	else if(but==butU)
		{
		sub_ind--;
		if(sub_ind==3)sub_ind--;
		gran_char(&sub_ind,1,5);	 
		}
	else if(but==butD_)
		{
/**/		sub_ind=5;		 /**/
		}				
     else if(sub_ind==1)
	     {
	     if((but==butR)||(but==butR_))
	     	{
	     	NUMIST++;
	     	gran(&NUMIST,0,12);
	     	lc640_write_int(EE_NUMIST,NUMIST);
			numOfForvardBps_init();
	     	}
	     
	     else if((but==butL)||(but==butL_))
	     	{
	     	NUMIST--;
	     	gran(&NUMIST,0,12);
	     	lc640_write_int(EE_NUMIST,NUMIST);
			numOfForvardBps_init();
	     	}
          }	
/**/
	  else if(sub_ind==2)
	     {
	     if((but==butR)||(but==butR_))
	     	{
	     	NUMINV++;
	     	gran(&NUMINV,0,12-NUMIST);
	     	lc640_write_int(EE_NUMINV,NUMINV);
	     	}
	     
	     else if((but==butL)||(but==butL_))
	     	{
	     	NUMINV--;
	     	gran(&NUMINV,0,12-NUMIST);
	     	lc640_write_int(EE_NUMINV,NUMINV);
	     	}
          }	
/**/
/**/     else if(sub_ind==3)  /**/
	     {
	     if((but==butR)||(but==butR_))
	     	{
	     	NUMAVT++;
	     	gran(&NUMAVT,0,12);
	     	lc640_write_int(EE_NUMAVT,NUMAVT);
	     	}
	     
	     else if((but==butL)||(but==butL_))
	     	{
	     	NUMAVT--;
	     	gran(&NUMAVT,0,12);
	     	lc640_write_int(EE_NUMAVT,NUMAVT);
	     	}
          }	
          
/**/    else if(sub_ind==5)	  /**/
	     {
	     if(but==butE)
	          {
			tree_down(0,0);
	          }
          }	          
	}

else if(ind==iStr_VD)
	{
	ret(1000);
	if(but==butD)
		{
		sub_ind++;
		gran_char(&sub_ind,0,2);
		}
	else if(but==butU)
		{
		sub_ind--;
		gran_char(&sub_ind,0,2);
		}
	else if(but==butD_)
		{
		sub_ind=4;
		}				
     else if(sub_ind==0)
	     {
	     if((but==butR)||(but==butR_))
	     	{
	     	NUMIST++;
	     	gran(&NUMIST,0,18);
	     	lc640_write_int(EE_NUMIST,NUMIST);
			numOfForvardBps_init();
	     	}
	     
	     else if((but==butL)||(but==butL_))
	     	{
	     	NUMIST--;
	     	gran(&NUMIST,0,18);
	     	lc640_write_int(EE_NUMIST,NUMIST);
			numOfForvardBps_init();
	     	}
          }	
          
     else if(sub_ind==1)
	     {
	     if((but==butR)||(but==butR_))
	     	{
	     	NUMDT++;
	     	gran(&NUMDT,0,3);
	     	lc640_write_int(EE_NUMDT,NUMDT);
	     	}
	     
	     else if((but==butL)||(but==butL_))
	     	{
	     	NUMDT--;
	     	gran(&NUMDT,0,3);
	     	lc640_write_int(EE_NUMDT,NUMDT);
	     	}
          }	

 /*    else if(sub_ind==2)
	     {
	     if((but==butR)||(but==butR_))
	     	{
	     	NUMMAKB++;
			if(NUMMAKB==1)NUMMAKB++;
			if(NUMMAKB==3)NUMMAKB++;
	     	gran(&NUMMAKB,0,4);
	     	lc640_write_int(EE_NUMMAKB,NUMMAKB);
	     	}
	     
	     else if((but==butL)||(but==butL_))
	     	{
	     	NUMMAKB--;
			if(NUMMAKB==1)NUMMAKB--;
			if(NUMMAKB==3)NUMMAKB--;
	     	gran(&NUMMAKB,0,4);
	     	lc640_write_int(EE_NUMMAKB,NUMMAKB);
	     	}
          }
    else if(sub_ind==3)
	     {
	     if((but==butR)||(but==butR_))
	     	{
	     	NUMSK++;
	     	gran(&NUMSK,0,4);
	     	lc640_write_int(EE_NUMSK,NUMSK);
	     	}
	     
	     else if((but==butL)||(but==butL_))
	     	{
	     	NUMSK--;
	     	gran(&NUMSK,0,4);
	     	lc640_write_int(EE_NUMSK,NUMSK);
	     	}
		}*/			  			                 
    else if(sub_ind==2)
	     {
	     if(but==butE)
	          {
			tree_down(0,0);
	          }
          }
	}
 
else if (ind==iLan_set)
	{
	char si_max;
	ret(1000);

	si_max=1;
	if(ETH_IS_ON!=0)si_max=21;

	if(but==butD)
		{
		sub_ind++;

		if((sub_ind==2)&&(index_set==0))
			{
			index_set=1;
			sub_ind1=0;
			}
		if(sub_ind==3) 
			{
			sub_ind=4;
			index_set=3;
			sub_ind1=0;
			}
		if(sub_ind==5) 
			{
			sub_ind=6;
			index_set=5;
			sub_ind1=0;
			}
		if(sub_ind==7) 
			{
			sub_ind=8;
			//index_set=3;
			sub_ind1=0;
			}
		if(sub_ind==10) 
			{
			//sub_ind=6;
			//index_set=9;
			sub_ind1=0;
			}
		if(sub_ind==11) 
			{
			//sub_ind=6;
			index_set=10;
			sub_ind1=0;
			}
		if(sub_ind==12) 
			{
			sub_ind++;
			}
		if(sub_ind==13) 
			{
			//sub_ind=6;
			index_set=12;
			sub_ind1=0;
			}
		if(sub_ind==14) 
			{
			sub_ind++;
			}
		if(sub_ind==15) 
			{
			//sub_ind=6;
			index_set=14;
			sub_ind1=0;
			}
		if(sub_ind==16) 
			{
			sub_ind++;
			}
		if(sub_ind==17) 
			{
			//sub_ind=6;
			index_set=16;
			sub_ind1=0;
			}
		if(sub_ind==18) 
			{
			sub_ind++;
			}
		if(sub_ind==19) 
			{
			//sub_ind=6;
			index_set=18;
			sub_ind1=0;
			}
		if(sub_ind==20) 
			{
			sub_ind++;
			}
	/*	if((sub_ind==4)&&(index_set==2))
			{
			index_set=3;
			sub_ind1=0;
			}*/
		
		gran_char(&sub_ind,0,si_max);
		}
	else if(but==butU)
		{
		sub_ind--;
		gran_char(&sub_ind,0,si_max);
		if(sub_ind==20) 
			{
			sub_ind--;
			}		
		if(sub_ind==18) 
			{
			sub_ind--;
			}		
		if(sub_ind==16) 
			{
			sub_ind--;
			}
		if(sub_ind==14) 
			{
			sub_ind--;
			}
		if(sub_ind==12) 
			{
			sub_ind--;
			}
		if(sub_ind==7) 
			{
			sub_ind--;
			}
		if(sub_ind==5) 
			{
			sub_ind--;
			}
		if(sub_ind==3) 
			{
			sub_ind--;
			}
		}
	else if(but==butD_)
		{
		sub_ind=si_max;
		}
	else if(but==butLR_)
		{
		lc640_write_int(EE_ETH_IS_ON,1);
		lc640_write_int(EE_ETH_DHCP_ON,1);
		lc640_write_int(EE_ETH_IP_1,192);
		lc640_write_int(EE_ETH_IP_2,168);
		lc640_write_int(EE_ETH_IP_3,1);
		lc640_write_int(EE_ETH_IP_4,251);
		#ifdef UKU_KONTUR
		lc640_write_int(EE_ETH_IP_4,230);
		#endif
		lc640_write_int(EE_ETH_MASK_1,255);
		lc640_write_int(EE_ETH_MASK_2,255);
		lc640_write_int(EE_ETH_MASK_3,255);
		lc640_write_int(EE_ETH_MASK_4,0);
		lc640_write_int(EE_ETH_GW_1,192);
		lc640_write_int(EE_ETH_GW_2,168);
		lc640_write_int(EE_ETH_GW_3,1);
		lc640_write_int(EE_ETH_GW_4,254);
		lc640_write_int(EE_ETH_SNMP_PORT_READ,161);
		lc640_write_int(EE_ETH_SNMP_PORT_WRITE,162);
		lc640_write_int(EE_COMMUNITY,'1');
		lc640_write_int(EE_COMMUNITY+2,'2');
		lc640_write_int(EE_COMMUNITY+4,'3');
		lc640_write_int(EE_COMMUNITY+6,0);
		lc640_write_int(EE_COMMUNITY+8,0);
		lc640_write_int(EE_ETH_TRAP1_IP_1,255);
		lc640_write_int(EE_ETH_TRAP1_IP_2,255);
		lc640_write_int(EE_ETH_TRAP1_IP_3,255);
		lc640_write_int(EE_ETH_TRAP1_IP_4,255);
		lc640_write_int(EE_ETH_TRAP2_IP_1,255);
		lc640_write_int(EE_ETH_TRAP2_IP_2,255);
		lc640_write_int(EE_ETH_TRAP2_IP_3,255);
		lc640_write_int(EE_ETH_TRAP2_IP_4,255);
		lc640_write_int(EE_ETH_TRAP3_IP_1,255);
		lc640_write_int(EE_ETH_TRAP3_IP_2,255);
		lc640_write_int(EE_ETH_TRAP3_IP_3,255);
		lc640_write_int(EE_ETH_TRAP3_IP_4,255);
		lc640_write_int(EE_ETH_TRAP4_IP_1,255);
		lc640_write_int(EE_ETH_TRAP4_IP_2,255);
		lc640_write_int(EE_ETH_TRAP4_IP_3,255);
		lc640_write_int(EE_ETH_TRAP4_IP_4,255);
		lc640_write_int(EE_ETH_TRAP5_IP_1,255);
		lc640_write_int(EE_ETH_TRAP5_IP_2,255);
		lc640_write_int(EE_ETH_TRAP5_IP_3,255);
		lc640_write_int(EE_ETH_TRAP5_IP_4,255);
		}					
	else if(sub_ind==0)
	     {
	     if((but==butE)||(but==butL)||(but==butR))
	     	{
	     	if(ETH_IS_ON)lc640_write_int(EE_ETH_IS_ON,0);
			else lc640_write_int(EE_ETH_IS_ON,1);
	     	}
	     }	
     else if((sub_ind==1)&&(ETH_IS_ON))
	     {
		if((but==butE)||(but==butL)||(but==butR))
	     	{
	     	if(ETH_DHCP_ON)lc640_write_int(EE_ETH_DHCP_ON,0);
			else lc640_write_int(EE_ETH_DHCP_ON,1);
	     	}
		}	
     else if(sub_ind==2)
	     {
		if(but==butE_)
	     	{
	     	sub_ind1++;
			gran_ring_char(&sub_ind1,0,3);
	     	}
		else if(sub_ind1==0)
			{
			if((but==butR)||(but==butR_))
				{
				ETH_IP_1++;
				gran_ring(&ETH_IP_1,0,255);
				lc640_write_int(EE_ETH_IP_1,ETH_IP_1);
				}
			else if((but==butL)||(but==butL_))
				{
				ETH_IP_1--;
				gran_ring(&ETH_IP_1,0,255);
				lc640_write_int(EE_ETH_IP_1,ETH_IP_1);
				}
			speed=1;
			}
		else if(sub_ind1==1)
			{
			if((but==butR)||(but==butR_))
				{
				ETH_IP_2++;
				gran_ring(&ETH_IP_2,0,255);
				lc640_write_int(EE_ETH_IP_2,ETH_IP_2);
				}
			else if((but==butL)||(but==butL_))
				{
				ETH_IP_2--;
				gran_ring(&ETH_IP_2,0,255);
				lc640_write_int(EE_ETH_IP_2,ETH_IP_2);
				}
			speed=1;
			}
		else if(sub_ind1==2)
			{
			if((but==butR)||(but==butR_))
				{
				ETH_IP_3++;
				gran_ring(&ETH_IP_3,0,255);
				lc640_write_int(EE_ETH_IP_3,ETH_IP_3);
				}
			else if((but==butL)||(but==butL_))
				{
				ETH_IP_3--;
				gran_ring(&ETH_IP_3,0,255);
				lc640_write_int(EE_ETH_IP_3,ETH_IP_3);
				}
			speed=1;
			}
		else if(sub_ind1==3)
			{
			if((but==butR)||(but==butR_))
				{
				ETH_IP_4++;
				gran_ring(&ETH_IP_4,0,255);
				lc640_write_int(EE_ETH_IP_4,ETH_IP_4);
				}
			else if((but==butL)||(but==butL_))
				{
				ETH_IP_4--;
				gran_ring(&ETH_IP_4,0,255);
				lc640_write_int(EE_ETH_IP_4,ETH_IP_4);
				}
			speed=1;
			}

          }
     else if(sub_ind==4)
	     {
		if(but==butE_)
	     	{
	     	sub_ind1++;
			gran_ring_char(&sub_ind1,0,3);
	     	}
		else if(sub_ind1==0)
			{
			if((but==butR)||(but==butR_))
				{
				ETH_MASK_1++;
				gran_ring(&ETH_MASK_1,0,255);
				lc640_write_int(EE_ETH_MASK_1,ETH_MASK_1);
				}
			else if((but==butL)||(but==butL_))
				{
				ETH_MASK_1--;
				gran_ring(&ETH_MASK_1,0,255);
				lc640_write_int(EE_ETH_MASK_1,ETH_MASK_1);
				}
			speed=1;
			}
		else if(sub_ind1==1)
			{
			if((but==butR)||(but==butR_))
				{
				ETH_MASK_2++;
				gran_ring(&ETH_MASK_2,0,255);
				lc640_write_int(EE_ETH_MASK_2,ETH_MASK_2);
				}
			else if((but==butL)||(but==butL_))
				{
				ETH_MASK_2--;
				gran_ring(&ETH_MASK_2,0,255);
				lc640_write_int(EE_ETH_MASK_2,ETH_MASK_2);
				}
			speed=1;
			}
		else if(sub_ind1==2)
			{
			if((but==butR)||(but==butR_))
				{
				ETH_MASK_3++;
				gran_ring(&ETH_MASK_3,0,255);
				lc640_write_int(EE_ETH_MASK_3,ETH_MASK_3);
				}
			else if((but==butL)||(but==butL_))
				{
				ETH_MASK_3--;
				gran_ring(&ETH_MASK_3,0,255);
				lc640_write_int(EE_ETH_MASK_3,ETH_MASK_3);
				}
			speed=1;
			}
		else if(sub_ind1==3)
			{
			if((but==butR)||(but==butR_))
				{
				ETH_MASK_4++;
				gran_ring(&ETH_MASK_4,0,255);
				lc640_write_int(EE_ETH_MASK_4,ETH_MASK_4);
				}
			else if((but==butL)||(but==butL_))
				{
				ETH_MASK_4--;
				gran_ring(&ETH_MASK_4,0,255);
				lc640_write_int(EE_ETH_MASK_4,ETH_MASK_4);
				}
			speed=1;
			}
		}
     else if(sub_ind==6)
	     {
		if(but==butE_)
	     	{
	     	sub_ind1++;
			gran_ring_char(&sub_ind1,0,3);
	     	}
		else if(sub_ind1==0)
			{
			if((but==butR)||(but==butR_))
				{
				ETH_GW_1++;
				gran_ring(&ETH_GW_1,0,255);
				lc640_write_int(EE_ETH_GW_1,ETH_GW_1);
				}
			else if((but==butL)||(but==butL_))
				{
				ETH_GW_1--;
				gran_ring(&ETH_GW_1,0,255);
				lc640_write_int(EE_ETH_GW_1,ETH_GW_1);
				}
			speed=1;
			}
		else if(sub_ind1==1)
			{
			if((but==butR)||(but==butR_))
				{
				ETH_GW_2++;
				gran_ring(&ETH_GW_2,0,255);
				lc640_write_int(EE_ETH_GW_2,ETH_GW_2);
				}
			else if((but==butL)||(but==butL_))
				{
				ETH_GW_2--;
				gran_ring(&ETH_GW_2,0,255);
				lc640_write_int(EE_ETH_GW_2,ETH_GW_2);
				}
			speed=1;
			}
		else if(sub_ind1==2)
			{
			if((but==butR)||(but==butR_))
				{
				ETH_GW_3++;
				gran_ring(&ETH_GW_3,0,255);
				lc640_write_int(EE_ETH_GW_3,ETH_GW_3);
				}
			else if((but==butL)||(but==butL_))
				{
				ETH_GW_3--;
				gran_ring(&ETH_GW_3,0,255);
				lc640_write_int(EE_ETH_GW_3,ETH_GW_3);
				}
			speed=1;
			}
		else if(sub_ind1==3)
			{
			if((but==butR)||(but==butR_))
				{
				ETH_GW_4++;
				gran_ring(&ETH_GW_4,0,255);
				lc640_write_int(EE_ETH_GW_4,ETH_GW_4);
				}
			else if((but==butL)||(but==butL_))
				{
				ETH_GW_4--;
				gran_ring(&ETH_GW_4,0,255);
				lc640_write_int(EE_ETH_GW_4,ETH_GW_4);
				}
			speed=1;
			}
		}
      else if(sub_ind==8)
	     {
		if(but==butR)ETH_SNMP_PORT_READ++;
		else if(but==butR_)ETH_SNMP_PORT_READ+=2;
		else if(but==butL)ETH_SNMP_PORT_READ--;
		else if(but==butL_)ETH_SNMP_PORT_READ-=2;
		speed=1;
		lc640_write_int(EE_ETH_SNMP_PORT_READ,ETH_SNMP_PORT_READ);
		}

     else if(sub_ind==9)
	     {
		if(but==butR)ETH_SNMP_PORT_WRITE++;
		else if(but==butR_)ETH_SNMP_PORT_WRITE+=2;
		else if(but==butL)ETH_SNMP_PORT_WRITE--;
		else if(but==butL_)ETH_SNMP_PORT_WRITE-=2;
		speed=1;
		lc640_write_int(EE_ETH_SNMP_PORT_WRITE,ETH_SNMP_PORT_WRITE);
		}					
     else if(sub_ind==10)
	     {
		if(but==butE_)
	     	{
	     	sub_ind1++;
			gran_ring_char(&sub_ind1,0,8);
	     	}
		if((but==butR)||(but==butR_))
			{
			snmp_community[sub_ind1]++;
			if(snmp_community[sub_ind1]<32) snmp_community[sub_ind1]=32;
			else if ((snmp_community[sub_ind1]>32)&&(snmp_community[sub_ind1]<48)) snmp_community[sub_ind1]=48;
			else if ((snmp_community[sub_ind1]>57)&&(snmp_community[sub_ind1]<65)) snmp_community[sub_ind1]=65;
			else if ((snmp_community[sub_ind1]>90)&&(snmp_community[sub_ind1]<97)) snmp_community[sub_ind1]=97;
			else if (snmp_community[sub_ind1]>122) snmp_community[sub_ind1]=32;
				//gran_ring(&ETH_GW_1,0,255);
			lc640_write_int(EE_COMMUNITY+(sub_ind1*2),snmp_community[sub_ind1]);
			speed=1;
			}
		if((but==butL)||(but==butL_))
			{
			snmp_community[sub_ind1]--;
			if(snmp_community[sub_ind1]<32) snmp_community[sub_ind1]=122;
			else if ((snmp_community[sub_ind1]>32)&&(snmp_community[sub_ind1]<48)) snmp_community[sub_ind1]=32;
			else if ((snmp_community[sub_ind1]>57)&&(snmp_community[sub_ind1]<65)) snmp_community[sub_ind1]=57;
			else if ((snmp_community[sub_ind1]>90)&&(snmp_community[sub_ind1]<97)) snmp_community[sub_ind1]=90;
			else if (snmp_community[sub_ind1]>122) snmp_community[sub_ind1]=122;
			//gran_ring(&ETH_GW_1,0,255);
			lc640_write_int(EE_COMMUNITY+(sub_ind1*2),snmp_community[sub_ind1]);
			speed=1;
			}
		}
 
     else if(sub_ind==11)
	     {
		if(but==butE_)
	     	{
	     	sub_ind1++;
			gran_ring_char(&sub_ind1,0,3);
	     	}
		else if(sub_ind1==0)
			{
			if((but==butR)||(but==butR_))
				{
				ETH_TRAP1_IP_1++;
				gran_ring(&ETH_TRAP1_IP_1,0,255);
				lc640_write_int(EE_ETH_TRAP1_IP_1,ETH_TRAP1_IP_1);
				}
			else if((but==butL)||(but==butL_))
				{
				ETH_TRAP1_IP_1--;
				gran_ring(&ETH_TRAP1_IP_1,0,255);
				lc640_write_int(EE_ETH_TRAP1_IP_1,ETH_TRAP1_IP_1);
				}
			speed=1;
			}
		else if(sub_ind1==1)
			{
			if((but==butR)||(but==butR_))
				{
				ETH_TRAP1_IP_2++;
				gran_ring(&ETH_TRAP1_IP_2,0,255);
				lc640_write_int(EE_ETH_TRAP1_IP_2,ETH_TRAP1_IP_2);
				}
			else if((but==butL)||(but==butL_))
				{
				ETH_TRAP1_IP_2--;
				gran_ring(&ETH_TRAP1_IP_2,0,255);
				lc640_write_int(EE_ETH_TRAP1_IP_2,ETH_TRAP1_IP_2);
				}
			speed=1;
			}
		else if(sub_ind1==2)
			{
			if((but==butR)||(but==butR_))
				{
				ETH_TRAP1_IP_3++;
				gran_ring(&ETH_TRAP1_IP_3,0,255);
				lc640_write_int(EE_ETH_TRAP1_IP_3,ETH_TRAP1_IP_3);
				}
			else if((but==butL)||(but==butL_))
				{
				ETH_TRAP1_IP_3--;
				gran_ring(&ETH_TRAP1_IP_3,0,255);
				lc640_write_int(EE_ETH_TRAP1_IP_3,ETH_TRAP1_IP_3);
				}
			speed=1;
			}
		else if(sub_ind1==3)
			{
			if((but==butR)||(but==butR_))
				{
				ETH_TRAP1_IP_4++;
				gran_ring(&ETH_TRAP1_IP_4,0,255);
				lc640_write_int(EE_ETH_TRAP1_IP_4,ETH_TRAP1_IP_4);
				}
			else if((but==butL)||(but==butL_))
				{
				ETH_TRAP1_IP_4--;
				gran_ring(&ETH_TRAP1_IP_4,0,255);
				lc640_write_int(EE_ETH_TRAP1_IP_4,ETH_TRAP1_IP_4);
				}
			speed=1;
			}
		}	
     else if(sub_ind==13)
	     {
		if(but==butE_)
	     	{
	     	sub_ind1++;
			gran_ring_char(&sub_ind1,0,3);
	     	}
		else if(sub_ind1==0)
			{
			if((but==butR)||(but==butR_))
				{
				ETH_TRAP2_IP_1++;
				gran_ring(&ETH_TRAP2_IP_1,0,255);
				lc640_write_int(EE_ETH_TRAP2_IP_1,ETH_TRAP2_IP_1);
				}
			else if((but==butL)||(but==butL_))
				{
				ETH_TRAP2_IP_1--;
				gran_ring(&ETH_TRAP2_IP_1,0,255);
				lc640_write_int(EE_ETH_TRAP2_IP_1,ETH_TRAP2_IP_1);
				}
			speed=1;
			}
		else if(sub_ind1==1)
			{
			if((but==butR)||(but==butR_))
				{
				ETH_TRAP2_IP_2++;
				gran_ring(&ETH_TRAP2_IP_2,0,255);
				lc640_write_int(EE_ETH_TRAP2_IP_2,ETH_TRAP2_IP_2);
				}
			else if((but==butL)||(but==butL_))
				{
				ETH_TRAP2_IP_2--;
				gran_ring(&ETH_TRAP2_IP_2,0,255);
				lc640_write_int(EE_ETH_TRAP2_IP_2,ETH_TRAP2_IP_2);
				}
			speed=1;
			}
		else if(sub_ind1==2)
			{
			if((but==butR)||(but==butR_))
				{
				ETH_TRAP2_IP_3++;
				gran_ring(&ETH_TRAP2_IP_3,0,255);
				lc640_write_int(EE_ETH_TRAP2_IP_3,ETH_TRAP2_IP_3);
				}
			else if((but==butL)||(but==butL_))
				{
				ETH_TRAP2_IP_3--;
				gran_ring(&ETH_TRAP2_IP_3,0,255);
				lc640_write_int(EE_ETH_TRAP2_IP_3,ETH_TRAP2_IP_3);
				}
			speed=1;
			}
		else if(sub_ind1==3)
			{
			if((but==butR)||(but==butR_))
				{
				ETH_TRAP2_IP_4++;
				gran_ring(&ETH_TRAP2_IP_4,0,255);
				lc640_write_int(EE_ETH_TRAP2_IP_4,ETH_TRAP2_IP_4);
				}
			else if((but==butL)||(but==butL_))
				{
				ETH_TRAP2_IP_4--;
				gran_ring(&ETH_TRAP2_IP_4,0,255);
				lc640_write_int(EE_ETH_TRAP2_IP_4,ETH_TRAP2_IP_4);
				}
			speed=1;
			}
		}	
     else if(sub_ind==15)
	     {
		if(but==butE_)
	     	{
	     	sub_ind1++;
			gran_ring_char(&sub_ind1,0,3);
	     	}
		else if(sub_ind1==0)
			{
			if((but==butR)||(but==butR_))
				{
				ETH_TRAP3_IP_1++;
				gran_ring(&ETH_TRAP3_IP_1,0,255);
				lc640_write_int(EE_ETH_TRAP3_IP_1,ETH_TRAP3_IP_1);
				}
			else if((but==butL)||(but==butL_))
				{
				ETH_TRAP3_IP_1--;
				gran_ring(&ETH_TRAP3_IP_1,0,255);
				lc640_write_int(EE_ETH_TRAP3_IP_1,ETH_TRAP3_IP_1);
				}
			speed=1;
			}
		else if(sub_ind1==1)
			{
			if((but==butR)||(but==butR_))
				{
				ETH_TRAP3_IP_2++;
				gran_ring(&ETH_TRAP3_IP_2,0,255);
				lc640_write_int(EE_ETH_TRAP3_IP_2,ETH_TRAP3_IP_2);
				}
			else if((but==butL)||(but==butL_))
				{
				ETH_TRAP3_IP_2--;
				gran_ring(&ETH_TRAP3_IP_2,0,255);
				lc640_write_int(EE_ETH_TRAP3_IP_2,ETH_TRAP3_IP_2);
				}
			speed=1;
			}
		else if(sub_ind1==2)
			{
			if((but==butR)||(but==butR_))
				{
				ETH_TRAP3_IP_3++;
				gran_ring(&ETH_TRAP3_IP_3,0,255);
				lc640_write_int(EE_ETH_TRAP3_IP_3,ETH_TRAP3_IP_3);
				}
			else if((but==butL)||(but==butL_))
				{
				ETH_TRAP3_IP_3--;
				gran_ring(&ETH_TRAP3_IP_3,0,255);
				lc640_write_int(EE_ETH_TRAP3_IP_3,ETH_TRAP3_IP_3);
				}
			speed=1;
			}
		else if(sub_ind1==3)
			{
			if((but==butR)||(but==butR_))
				{
				ETH_TRAP3_IP_4++;
				gran_ring(&ETH_TRAP3_IP_4,0,255);
				lc640_write_int(EE_ETH_TRAP3_IP_4,ETH_TRAP3_IP_4);
				}
			else if((but==butL)||(but==butL_))
				{
				ETH_TRAP3_IP_4--;
				gran_ring(&ETH_TRAP3_IP_4,0,255);
				lc640_write_int(EE_ETH_TRAP3_IP_4,ETH_TRAP3_IP_4);
				}
			speed=1;
			}
		}	
     else if(sub_ind==17)
	     {
		if(but==butE_)
	     	{
	     	sub_ind1++;
			gran_ring_char(&sub_ind1,0,3);
	     	}
		else if(sub_ind1==0)
			{
			if((but==butR)||(but==butR_))
				{
				ETH_TRAP4_IP_1++;
				gran_ring(&ETH_TRAP4_IP_1,0,255);
				lc640_write_int(EE_ETH_TRAP4_IP_1,ETH_TRAP4_IP_1);
				}
			else if((but==butL)||(but==butL_))
				{
				ETH_TRAP4_IP_1--;
				gran_ring(&ETH_TRAP4_IP_1,0,255);
				lc640_write_int(EE_ETH_TRAP4_IP_1,ETH_TRAP4_IP_1);
				}
			speed=1;
			}
		else if(sub_ind1==1)
			{
			if((but==butR)||(but==butR_))
				{
				ETH_TRAP4_IP_2++;
				gran_ring(&ETH_TRAP4_IP_2,0,255);
				lc640_write_int(EE_ETH_TRAP4_IP_2,ETH_TRAP4_IP_2);
				}
			else if((but==butL)||(but==butL_))
				{
				ETH_TRAP4_IP_2--;
				gran_ring(&ETH_TRAP4_IP_2,0,255);
				lc640_write_int(EE_ETH_TRAP4_IP_2,ETH_TRAP4_IP_2);
				}
			speed=1;
			}
		else if(sub_ind1==2)
			{
			if((but==butR)||(but==butR_))
				{
				ETH_TRAP4_IP_3++;
				gran_ring(&ETH_TRAP4_IP_3,0,255);
				lc640_write_int(EE_ETH_TRAP4_IP_3,ETH_TRAP4_IP_3);
				}
			else if((but==butL)||(but==butL_))
				{
				ETH_TRAP4_IP_3--;
				gran_ring(&ETH_TRAP4_IP_3,0,255);
				lc640_write_int(EE_ETH_TRAP4_IP_3,ETH_TRAP4_IP_3);
				}
			speed=1;
			}
		else if(sub_ind1==3)
			{
			if((but==butR)||(but==butR_))
				{
				ETH_TRAP4_IP_4++;
				gran_ring(&ETH_TRAP4_IP_4,0,255);
				lc640_write_int(EE_ETH_TRAP4_IP_4,ETH_TRAP4_IP_4);
				}
			else if((but==butL)||(but==butL_))
				{
				ETH_TRAP4_IP_4--;
				gran_ring(&ETH_TRAP4_IP_4,0,255);
				lc640_write_int(EE_ETH_TRAP4_IP_4,ETH_TRAP4_IP_4);
				}
			speed=1;
			}
		}	
     else if(sub_ind==19)
	     {
		if(but==butE_)
	     	{
	     	sub_ind1++;
			gran_ring_char(&sub_ind1,0,3);
	     	}
		else if(sub_ind1==0)
			{
			if((but==butR)||(but==butR_))
				{
				ETH_TRAP5_IP_1++;
				gran_ring(&ETH_TRAP5_IP_1,0,255);
				lc640_write_int(EE_ETH_TRAP5_IP_1,ETH_TRAP5_IP_1);
				}
			else if((but==butL)||(but==butL_))
				{
				ETH_TRAP5_IP_1--;
				gran_ring(&ETH_TRAP5_IP_1,0,255);
				lc640_write_int(EE_ETH_TRAP5_IP_1,ETH_TRAP5_IP_1);
				}
			speed=1;
			}
		else if(sub_ind1==1)
			{
			if((but==butR)||(but==butR_))
				{
				ETH_TRAP5_IP_2++;
				gran_ring(&ETH_TRAP5_IP_2,0,255);
				lc640_write_int(EE_ETH_TRAP5_IP_2,ETH_TRAP5_IP_2);
				}
			else if((but==butL)||(but==butL_))
				{
				ETH_TRAP5_IP_2--;
				gran_ring(&ETH_TRAP5_IP_2,0,255);
				lc640_write_int(EE_ETH_TRAP5_IP_2,ETH_TRAP5_IP_2);
				}
			speed=1;
			}
		else if(sub_ind1==2)
			{
			if((but==butR)||(but==butR_))
				{
				ETH_TRAP5_IP_3++;
				gran_ring(&ETH_TRAP5_IP_3,0,255);
				lc640_write_int(EE_ETH_TRAP5_IP_3,ETH_TRAP5_IP_3);
				}
			else if((but==butL)||(but==butL_))
				{
				ETH_TRAP5_IP_3--;
				gran_ring(&ETH_TRAP5_IP_3,0,255);
				lc640_write_int(EE_ETH_TRAP5_IP_3,ETH_TRAP5_IP_3);
				}
			speed=1;
			}
		else if(sub_ind1==3)
			{
			if((but==butR)||(but==butR_))
				{
				ETH_TRAP5_IP_4++;
				gran_ring(&ETH_TRAP5_IP_4,0,255);
				lc640_write_int(EE_ETH_TRAP5_IP_4,ETH_TRAP5_IP_4);
				}
			else if((but==butL)||(but==butL_))
				{
				ETH_TRAP5_IP_4--;
				gran_ring(&ETH_TRAP5_IP_4,0,255);
				lc640_write_int(EE_ETH_TRAP5_IP_4,ETH_TRAP5_IP_4);
				}
			speed=1;
			}
		}													          
    else if(sub_ind==si_max)
	     {
	     if(but==butE)
	          {
	          tree_down(0,0);
	          }
          }	          	
	}


else if (ind==iBlok_ips_set)
	{
    ret(1000);
	if(but==butD)
		{
		sub_ind++;
		if(sub_ind==2)sub_ind=3;
		gran_char(&sub_ind,0,3);
		}
	else if(but==butU)
		{
		sub_ind--;
		if(sub_ind==2)sub_ind=1;
		gran_char(&sub_ind,0,3);
		}
	else if(but==butD_)
		{
		sub_ind=3;
		}			
     
	else if(sub_ind==0)
		{
		if((but==butR)||(but==butR_))
			{
			ipsBlckSrc++;
			gran(&ipsBlckSrc,0,2);
			lc640_write_int(EE_IPS_BLOCK_SRC,ipsBlckSrc);
			}
		else if((but==butL)||(but==butL_))
			{
			ipsBlckSrc--;
			gran(&ipsBlckSrc,0,2);
			lc640_write_int(EE_IPS_BLOCK_SRC,ipsBlckSrc);
			}
		speed=1;
		}

	else if(sub_ind==1)
		{
		if((but==butR)||(but==butR_))
			{
			ipsBlckLog=1;
			lc640_write_int(EE_IPS_BLOCK_LOG,ipsBlckLog);
			}
		else if((but==butL)||(but==butL_))
			{
			ipsBlckLog=0;
			lc640_write_int(EE_IPS_BLOCK_LOG,ipsBlckLog);
			}
		speed=1;
		}

	else if((sub_ind==3)&&(but==butE))
		{
	     tree_down(0,0);
	     ret(0);
		} 												
	}

else if (ind==iApv)
	{
     ret(1000);
	if(but==butD)
		{
		sub_ind++;
		gran_char(&sub_ind,0,simax);
		}
	else if(but==butU)
		{
		sub_ind--;
		gran_char(&sub_ind,0,simax);
		}
	else if(but==butD_)
		{
		sub_ind=simax;
		}			
	else if(but==butE)
	     {
	     if(sub_ind==simax)
	          {
	          //a=b[--ptr_ind];
	          tree_down(0,0);
	          }
	     else if(sub_ind==0)   
	          {
	          if(APV_ON1==apvON)lc640_write_int(EE_APV_ON1,apvOFF);
	          else lc640_write_int(EE_APV_ON1,apvON);
	          }
          else if((sub_ind==1)&&(APV_ON1==apvON))   
	          {
	          if(APV_ON2==apvON)lc640_write_int(EE_APV_ON2,apvOFF);
	          else lc640_write_int(EE_APV_ON2,apvON);
	          }	 
          }
     
     else if((sub_ind==2)&&(APV_ON2==apvON))   
          {
	     if((but==butR)||(but==butR_))
	          {
	          signed short tempSS;
	          tempSS=APV_ON2_TIME;
	          tempSS++;
	          gran(&tempSS,1,24);
	          lc640_write_int(EE_APV_ON2_TIME,tempSS);
	          }
          else if((but==butL)||(but==butL_))
	          {
	          signed short tempSS;
	          tempSS=APV_ON2_TIME;
	          tempSS--;
	          gran(&tempSS,1,24);
	          lc640_write_int(EE_APV_ON2_TIME,tempSS);
	          }	          
	     speed=1;
	     }	 
  	} 

else if (ind==iExt_set)
	{
	ret(1000);
	if (but==butU)
		{      
		sub_ind--;
		gran_char(&sub_ind,0,3);
		}
		
	else if (but==butD)
		{
		sub_ind++;
		gran_char(&sub_ind,0,3);		
		}

	else if((but==butE)&&(sub_ind==0))
		{
	     tree_up(iExt_ddv,0,0,0);
	     ret(0);
		}

	else if((but==butE)&&(sub_ind==1))
		{
	     tree_up(iExt_ddi,0,0,0);
	     ret(0);
		}
	else if((but==butE)&&(sub_ind==2))
		{
	     tree_up(iExt_dud,0,0,0);
	     ret(0);
		}
/*	else if((but==butE)&&(sub_ind==3))
		{
	     tree_up(iExt_dp,0,0,0);
	     ret(0);
		} */
		
	else if((but==butE)&&(sub_ind==3))
		{
	     tree_down(0,0);
	     ret(0);
		}        	
	}

else if (ind==iExt_set_3U)
	{
	ret(1000);
	if (but==butU)
		{      
		sub_ind--;
		gran_char(&sub_ind,0,NUMSK);
		}
		
	else if (but==butD)
		{
		sub_ind++;
		gran_char(&sub_ind,0,NUMSK);		
		}
 	else if((but==butE)&&(sub_ind==NUMSK))
		{
	     tree_down(0,0);
	     ret(0);
		}
	else if((but==butE)&&(sub_ind==0))
		{
	     tree_up(iExt_sk_3U,0,0,0);
	     ret(0);
		}
	else if((but==butE)&&(sub_ind==1))
		{
	     tree_up(iExt_sk_3U,0,0,1);
	     ret(0);
		}
	else if((but==butE)&&(sub_ind==2))
		{
	     tree_up(iExt_sk_3U,0,0,2);
	     ret(0);
		}
	else if((but==butE)&&(sub_ind==3))
		{
	     tree_up(iExt_sk_3U,0,0,3);
	     ret(0);
		} 
	}

else if (ind==iExt_dt)
	{
	ret(1000);
	if(but==butD)
		{
		sub_ind++;
		gran_char(&sub_ind,1,7);
		}
	else if(but==butU)
		{
		if(sub_ind==1)index_set=0;
		else sub_ind--;
		gran_char(&sub_ind,1,7);
		}	
	else if(but==butD_)
		{
		sub_ind=7;
		}			
		
	else if(sub_ind==1) 
		{
		if(but==butE)
			{
			if(!TMAX_EXT_EN[sub_ind1])lc640_write_int(ADR_TMAX_EXT_EN[sub_ind1],0xffff);
			else lc640_write_int(ADR_TMAX_EXT_EN[sub_ind1],0);
			}
		else if((but==butR)||(but==butR_))
			{
			TMAX_EXT[sub_ind1]++;
			}	
		else if((but==butL)||(but==butL_))
			{
			TMAX_EXT[sub_ind1]--;
			}	
		gran(&TMAX_EXT[sub_ind1],-50,100);
		if(lc640_read_int(ADR_TMAX_EXT[sub_ind1])!=TMAX_EXT[sub_ind1]) lc640_write_int(ADR_TMAX_EXT[sub_ind1],TMAX_EXT[sub_ind1]);			
		speed=1;
		}
	else if(sub_ind==2) 
		{
		if(but==butE)
			{
			if(!TMIN_EXT_EN[sub_ind1])lc640_write_int(ADR_TMIN_EXT_EN[sub_ind1],0xffff);
			else lc640_write_int(ADR_TMIN_EXT_EN[sub_ind1],0);
			}
		else if((but==butR)||(but==butR_))
			{
			TMIN_EXT[sub_ind1]++;
			}	
		else if((but==butL)||(but==butL_))
			{
			TMIN_EXT[sub_ind1]--;
			}	
		gran(&TMIN_EXT[sub_ind1],-50,100);
		if(lc640_read_int(ADR_TMIN_EXT[sub_ind1])!=TMIN_EXT[sub_ind1]) lc640_write_int(ADR_TMIN_EXT[sub_ind1],TMIN_EXT[sub_ind1]);			
		speed=1;
		}		
	else if(sub_ind==3) 
		{
		if(but==butE)
			{
			if(!T_EXT_REL_EN[sub_ind1])lc640_write_int(ADR_T_EXT_REL_EN[sub_ind1],0xffff);
			else lc640_write_int(ADR_T_EXT_REL_EN[sub_ind1],0);
			}
		}	
	else if(sub_ind==4) 
		{
		if(but==butE)
			{
			if(!T_EXT_ZVUK_EN[sub_ind1])lc640_write_int(ADR_T_EXT_ZVUK_EN[sub_ind1],0xffff);
			else lc640_write_int(ADR_T_EXT_ZVUK_EN[sub_ind1],0);
			}
		}	
	else if(sub_ind==5) 
		{
		if(but==butE)
			{
			if(!T_EXT_LCD_EN[sub_ind1])lc640_write_int(ADR_T_EXT_LCD_EN[sub_ind1],0xffff);
			else lc640_write_int(ADR_T_EXT_LCD_EN[sub_ind1],0);
			}
		}	
	else if(sub_ind==6) 
		{
		if(but==butE)
			{
			if(!T_EXT_RS_EN[sub_ind1])lc640_write_int(ADR_T_EXT_RS_EN[sub_ind1],0xffff);
			else lc640_write_int(ADR_T_EXT_RS_EN[sub_ind1],0);
			}
		}	
	else if(sub_ind==7) 
		{
		if(but==butE)
			{
			tree_down(0,0);
			//a=b[--ptr_ind];
			}
		}												
	}	

else if (ind==iExt_sk)
	{
	ret(1000);
	if(but==butD)
		{
		sub_ind++;
		if(sub_ind==2)sub_ind=3;
		gran_char(&sub_ind,1,7);
		
		}
	else if(but==butU)
		{
		if(sub_ind==1)index_set=0;
		else sub_ind--;
		if(sub_ind==2)sub_ind=1;
		gran_char(&sub_ind,1,7);
		}	
	else if(but==butD_)
		{
		sub_ind=7;
		}			
	else if(sub_ind==1) 
		{
		if(but==butE)
			{
			if(!SK_SIGN[sub_ind1])lc640_write_int(ADR_SK_SIGN[sub_ind1],0xffff);
			else lc640_write_int(ADR_SK_SIGN[sub_ind1],0);
			}
		}
	else if(sub_ind==3) 
		{
	//	if(but==butE)
		//	{
	//		if(!SK_REL_EN[sub_ind1])lc640_write_int(ADR_SK_REL_EN[sub_ind1],0xffff);
	//		else lc640_write_int(ADR_SK_REL_EN[sub_ind1],0);
	//		}
		}	
	else if(sub_ind==4) 
		{
		if(but==butE)
			{
			if(!SK_ZVUK_EN[sub_ind1])lc640_write_int(ADR_SK_ZVUK_EN[sub_ind1],0xffff);
			else lc640_write_int(ADR_SK_ZVUK_EN[sub_ind1],0);
			}
		}	
	else if(sub_ind==5) 
		{
		if(but==butE)
			{
			if(!SK_LCD_EN[sub_ind1])lc640_write_int(ADR_SK_LCD_EN[sub_ind1],0xffff);
			else lc640_write_int(ADR_SK_LCD_EN[sub_ind1],0);
			}
		}	
	else if(sub_ind==6) 
		{
		if(but==butE)
			{
			if(!SK_RS_EN[sub_ind1])lc640_write_int(ADR_SK_RS_EN[sub_ind1],0xffff);
			else lc640_write_int(ADR_SK_RS_EN[sub_ind1],0);
			}
		}	
	else if(sub_ind==7) 
		{
		if(but==butE)
			{
			//a=b[--ptr_ind];
			tree_down(0,0);
			}
		}												
	}	

else if (ind==iExt_sk_3U)
	{
	ret(1000);
	if(but==butD)
		{
		sub_ind++;
		if(sub_ind==2)sub_ind=3;
		gran_char(&sub_ind,1,5);
		
		}
	else if(but==butU)
		{
		if(sub_ind==1)index_set=0;
		else sub_ind--;
		if(sub_ind==2)sub_ind=1;
		gran_char(&sub_ind,1,5);
		}	
	else if(but==butD_)
		{
		sub_ind=5;
		}			
	else if(sub_ind==1) 
		{
		if(but==butE)
			{
			if(!SK_SIGN[sub_ind1])lc640_write_int(ADR_SK_SIGN[sub_ind1],0xffff);
			else lc640_write_int(ADR_SK_SIGN[sub_ind1],0);
			}
		}
	else if(sub_ind==3) 
		{
		if(but==butE)
			{
			if(!SK_ZVUK_EN[sub_ind1])lc640_write_int(ADR_SK_ZVUK_EN[sub_ind1],0xffff);
			else lc640_write_int(ADR_SK_ZVUK_EN[sub_ind1],0);
			}
		}	
	else if(sub_ind==4) 
		{
		if(but==butE)
			{
			if(!SK_LCD_EN[sub_ind1])lc640_write_int(ADR_SK_LCD_EN[sub_ind1],0xffff);
			else lc640_write_int(ADR_SK_LCD_EN[sub_ind1],0);
			}
		}	
	else if(sub_ind==5) 
		{
		if(but==butE)
			{
			tree_down(0,0);
			}
		}												
	}	

else if (ind==iExt_ddv)
	{
	ret(1000);
	if(but==butD)
		{
		sub_ind++;
		if(sub_ind==2)sub_ind=3;
		gran_char(&sub_ind,1,5);
		
		}
	else if(but==butU)
		{
		if(sub_ind==1)index_set=0;
		else sub_ind--;
		if(sub_ind==2)sub_ind=1;
		gran_char(&sub_ind,1,5);
		}	
	else if(but==butD_)
		{
		sub_ind=7;
		}			
	else if(sub_ind==1) 
		{
		if(but==butE)
			{
			if(!SK_SIGN[0])lc640_write_int(ADR_SK_SIGN[0],0xffff);
			else lc640_write_int(ADR_SK_SIGN[0],0);
			}
		}
	else if(sub_ind==3) 
		{
		if(but==butE)
			{
			if(SK_REL_EN[0])lc640_write_int(ADR_SK_REL_EN[0],0);
			else lc640_write_int(ADR_SK_REL_EN[0],0xffff);
			}
		}	

	else if(sub_ind==4) 
		{
		if(but==butE)
			{
			if(SK_LCD_EN[0])lc640_write_int(ADR_SK_LCD_EN[0],0);
			else lc640_write_int(ADR_SK_LCD_EN[0],0xffff);
			}
		}	
	else if(sub_ind==5) 
		{
		if(but==butE)
			{
			tree_down(0,0);
			}
		}												
	}	

else if (ind==iExt_ddi)
	{
	ret(1000);
	if(but==butD)
		{
		sub_ind++;
		if(sub_ind==2)sub_ind=3;
		gran_char(&sub_ind,1,5);
		
		}
	else if(but==butU)
		{
		if(sub_ind==1)index_set=0;
		else sub_ind--;
		if(sub_ind==2)sub_ind=1;
		gran_char(&sub_ind,1,5);
		}	
	else if(but==butD_)
		{
		sub_ind=7;
		}			
	else if(sub_ind==1) 
		{
		if(but==butE)
			{
			if(!SK_SIGN[1])lc640_write_int(ADR_SK_SIGN[1],0xffff);
			else lc640_write_int(ADR_SK_SIGN[1],0);
			}
		}
	else if(sub_ind==3) 
		{
		if(but==butE)
			{
			if(SK_REL_EN[1])lc640_write_int(ADR_SK_REL_EN[1],0);
			else lc640_write_int(ADR_SK_REL_EN[1],0xffff);
			}
		}	
	else if(sub_ind==4) 
		{
		if(but==butE)
			{
			if(SK_LCD_EN[1])lc640_write_int(ADR_SK_LCD_EN[1],0);
			else lc640_write_int(ADR_SK_LCD_EN[1],0xffff);
			}
		}	
	else if(sub_ind==5) 
		{
		if(but==butE)
			{
			tree_down(0,0);
			}
		}												
	}
 
 else if (ind==iExt_dud)
	{
	ret(1000);
	if(but==butD)
		{
		sub_ind++;
		if(sub_ind==2)sub_ind=3;
		gran_char(&sub_ind,1,5);
		
		}
	else if(but==butU)
		{
		if(sub_ind==1)index_set=0;
		else sub_ind--;
		if(sub_ind==2)sub_ind=1;
		gran_char(&sub_ind,1,5);
		}	
	else if(but==butD_)
		{
		sub_ind=7;
		}			
	else if(sub_ind==1) 
		{
		if(but==butE)
			{
			if(!SK_SIGN[2])lc640_write_int(ADR_SK_SIGN[2],0xffff);
			else lc640_write_int(ADR_SK_SIGN[2],0);
			}
		}
	else if(sub_ind==3) 
		{
		if(but==butE)
			{
			if(SK_REL_EN[2])lc640_write_int(ADR_SK_REL_EN[2],0);
			else lc640_write_int(ADR_SK_REL_EN[2],0xffff);
			}
		}	
	else if(sub_ind==4) 
		{
		if(but==butE)
			{
			if(SK_LCD_EN[2])lc640_write_int(ADR_SK_LCD_EN[2],0);
			else lc640_write_int(ADR_SK_LCD_EN[2],0xffff);
			}
		}	
	else if(sub_ind==5) 
		{
		if(but==butE)
			{
			tree_down(0,0);
			}
		}												
	}
/*     
else if (ind==iExt_dp)
	{
	ret(1000);
	if(but==butD)
		{
		sub_ind++;
		if(sub_ind==2)sub_ind=3;
		gran_char(&sub_ind,1,5);
		
		}
	else if(but==butU)
		{
		if(sub_ind==1)index_set=0;
		else sub_ind--;
		if(sub_ind==2)sub_ind=1;
		gran_char(&sub_ind,1,5);
		}	
	else if(but==butD_)
		{
		sub_ind=7;
		}			
	else if(sub_ind==1) 
		{
		if(but==butE)
			{
			if(!SK_SIGN[3])lc640_write_int(ADR_SK_SIGN[3],0xffff);
			else lc640_write_int(ADR_SK_SIGN[3],0);
			}
		}
	else if(sub_ind==3) 
		{
		if(but==butE)
			{
			if(!SK_REL_EN[3])lc640_write_int(ADR_SK_REL_EN[3],0xffff);
			else lc640_write_int(ADR_SK_REL_EN[3],0);
			}
		}	
	else if(sub_ind==4) 
		{
		if(but==butE)
			{
			if(!SK_LCD_EN[3])lc640_write_int(ADR_SK_LCD_EN[3],0xffff);
			else lc640_write_int(ADR_SK_LCD_EN[3],0);
			}
		}	
	else if(sub_ind==5) 
		{
		if(but==butE)
			{
			tree_down(0,0);
			}
		}												
	}         	
*/		     
else if(ind==iK)
	{
	ret(1000);
	if(but==butD)
		{
		sub_ind++;
		gran_char(&sub_ind,0,2+(NUMBAT!=0)+(NUMIST!=0)+(NUMINV!=0)+2);
		}
	else if(but==butU)
		{
		sub_ind--;
		gran_char(&sub_ind,0,2+(NUMBAT!=0)+(NUMIST!=0)+(NUMINV!=0)+2);
		}
	else if(but==butD_)
		{
		sub_ind=2+(NUMBAT!=0)+(NUMIST!=0)+(NUMINV!=0)+2;
		}				
	else if(but==butE)
		{
		if(sub_ind==0)
			{
			tree_up(iK_net,0,0,0);	
			ret(1000);		
			}
		else if((NUMBAT)&&(sub_ind==1))
			{
			tree_up(iK_bat_sel,0,0,0);	
			ret(1000);
			}
		else if((NUMIST)&&(sub_ind==(1+(NUMBAT!=0))))
			{
			tree_up(iK_bps_sel,0,0,0);	
			ret(1000);
			}
/*		else if((sub_ind==(1+(NUMBAT!=0)+(NUMIST!=0))))
			{
			tree_up(iK_load,0,0,0);	
			ret(1000);
			}
          
         	else if(sub_ind==(2+(NUMBAT!=0)+(NUMIST!=0)))
			{
			tree_up(iK_t_ext,0,0,0);	
			ret(1000);			
			}
          else if(sub_ind==(2+(NUMBAT!=0)+(NUMIST!=0)+1))
			{
			tree_up(iK_power_net3,0,0,0);	
			ret(1000);
               }               				
          else if(sub_ind==(2+(NUMBAT!=0)+(NUMIST!=0)+2))
			{
	          tree_down(0,0);
	          ret(0);
               }	               			
		}*/
/**/		else if((NUMINV)&&(sub_ind==(1+(NUMBAT!=0)+(NUMIST!=0))))
			{
			//tree_up(iK_inv_sel,0,0,0);	
			ret(1000);
			}
								
			else if((sub_ind==(1+(NUMBAT!=0)+(NUMIST!=0)+(NUMINV!=0))))		 /**/
			{
			tree_up(iK_load,0,0,0);	
			ret(1000);
			}
          
/**/     	else if(sub_ind==(2+(NUMBAT!=0)+(NUMIST!=0)+(NUMINV!=0)))	   /**/
			{
			tree_up(iK_t_ext,0,0,0);	
			ret(1000);			
			}
/**/		else if(sub_ind==(2+(NUMBAT!=0)+(NUMIST!=0)+(NUMINV!=0)+1))		/**/
			{
			tree_up(iK_power_net3,0,0,0);	
			ret(1000);
               }               				
/**/	   	else if(sub_ind==(2+(NUMBAT!=0)+(NUMIST!=0)+(NUMINV!=0)+2))	   /**/
			{
	          tree_down(0,0);
	          ret(0);
               }	               			
		}					
	}

else if(ind==iK_VD)
	{
	ret(1000);
	if(but==butD)
		{
		sub_ind++;
		gran_char(&sub_ind,0,4+(NUMIST!=0));
		}
	else if(but==butU)
		{
		sub_ind--;
		gran_char(&sub_ind,0,4+(NUMIST!=0));
		}
	else if(but==butD_)
		{
		sub_ind=4+(NUMIST!=0);
		}
	else if(sub_ind==0)
		{
		temp_SS=lc640_read_int(EE_KUOUT);
	     if(but==butR)
	     	{
		     temp_SS++;
	     	}
	     else if(but==butR_)
	     	{
	     	temp_SS+=2;
	     	}	
	     else if(but==butL)
	     	{
	     	temp_SS--;
	     	}
	     else if(but==butL_)
	     	{
	     	temp_SS-=2;
	     	}
	    gran(&temp_SS,50,2000);

		lc640_write_int(EE_KUOUT,temp_SS);					
		speed=1;	
					
		}

	else if(sub_ind==1)
		{
		temp_SS=lc640_read_int(ADR_KI1BAT[0]);
		if(but==butR)temp_SS++;
		else if(but==butR_)temp_SS+=2;
		else if(but==butL)temp_SS--;
		else if(but==butL_)temp_SS-=2;
						
		gran(&temp_SS,200,30000);
		lc640_write_int(ADR_KI1BAT[0],temp_SS);
		phase=1;
		speed=1;
		}

	else if((NUMIST)&&(sub_ind==2))
		{
		tree_up(iK_bps_sel,0,0,0);	
		ret(1000);
		}
	else if(sub_ind==(2+(NUMIST!=0)))
		{
		temp_SS=lc640_read_int(EE_KUVD);
	     if(but==butR)
	     	{
		     temp_SS++;
	     	}
	     else if(but==butR_)
	     	{
	     	temp_SS+=2;
	     	}	
	     else if(but==butL)
	     	{
	     	temp_SS--;
	     	}
	     else if(but==butL_)
	     	{
	     	temp_SS-=2;
	     	}
	    gran(&temp_SS,30,2000);

		lc640_write_int(EE_KUVD,temp_SS);					
		speed=1;	
					
		}
	else if(sub_ind==(3+(NUMIST!=0)))
		{
		temp_SS=lc640_read_int(KT_EXT0);
	    if(but==butR)
	    	{
		    temp_SS++;
	     	}
	    else if(but==butR_)
	     	{
	     	temp_SS+=2;
	     	}	
	    else if(but==butL)
	     	{
	     	temp_SS--;
	     	}
	    else if(but==butL_)
	     	{
	     	temp_SS-=2;
	     	}
	    gran(&temp_SS,1900,3000);
		lc640_write_int(KT_EXT0,temp_SS);					
		speed=1;	
		}			
   	else if(sub_ind==(4+(NUMIST!=0)))	  
		{
	    tree_down(0,0);
	    ret(0);
        }
	}


else if(ind==iK_220_IPS_TERMOKOMPENSAT)
	{
	ret(1000);
	if(but==butD)
		{
		sub_ind++;
		gran_char(&sub_ind,0,2+(NUMBAT!=0)+(NUMIST!=0)+(NUMINV!=0)+(NUMDT!=0));
		}
	else if(but==butU)
		{
		sub_ind--;
		gran_char(&sub_ind,0,2+(NUMBAT!=0)+(NUMIST!=0)+(NUMINV!=0)+(NUMDT!=0));
		}
	else if(but==butD_)
		{
		sub_ind=2+(NUMBAT!=0)+(NUMIST!=0)+(NUMINV!=0)+(NUMDT!=0);
		}
	else if(but==butE)
		{
		if(sub_ind==0)
			{
			if((AUSW_MAIN==22033)||(AUSW_MAIN==22018))
				{
				tree_up(iK_net3,0,0,0);
		     	ret(1000);
				}
			else
				{
				tree_up(iK_net,0,0,0);	
				ret(1000);		
				}
			}
		else if((NUMBAT)&&(sub_ind==1))
			{
			tree_up(iK_bat_sel,0,0,0);	
			ret(1000);
			}
		else if((NUMIST)&&(sub_ind==(1+(NUMBAT!=0))))
			{
			tree_up(iK_bps_sel,0,0,0);	
			ret(1000);
			}

		else if((sub_ind==(1+(NUMBAT!=0)+(NUMIST!=0))))
			{
			tree_up(iK_out,0,0,0);	
			ret(1000);
			}
          
         	else if((NUMDT)&&(sub_ind==(2+(NUMBAT!=0)+(NUMIST!=0))))
			{
			tree_up(iK_t_ext_6U,0,0,0);	
			ret(1000);			
			}
 						
          else if(sub_ind==(3+(NUMBAT!=0)+(NUMIST!=0)/*+(NUMINV!=0)*/))
			{
	          tree_down(0,0);
	          ret(0);
               }	               			
		}			
	}

else if(ind==iK_220_IPS_TERMOKOMPENSAT_IB)
	{
	ret(1000);
	if(but==butD)
		{
		sub_ind++;
		gran_char(&sub_ind,0,3+(NUMIST!=0)+(NUMDT!=0));
		}
	else if(but==butU)
		{
		sub_ind--;
		gran_char(&sub_ind,0,3+(NUMIST!=0)+(NUMDT!=0));
		}
	else if(but==butD_)
		{
		sub_ind=3+(NUMIST!=0)+(NUMDT!=0);
		}
	else if(but==butE)
		{
		if(sub_ind==0)
			{
			if((AUSW_MAIN==22063)||(AUSW_MAIN==22023)||(AUSW_MAIN==22043)||(AUSW_MAIN==22018))
				{
				tree_up(iK_net3,0,0,0);
		     	ret(1000);
				}
			else
				{
				tree_up(iK_net,0,0,0);	
				ret(1000);		
				}
			}
		else if(sub_ind==1)
			{
			tree_up(iK_bat_ips_termokompensat_ib,0,0,0);	
			ret(1000);
			}
		else if((NUMIST)&&(sub_ind==2))
			{
			tree_up(iK_bps_sel,0,0,0);	
			ret(1000);
			}

		else if((sub_ind==(2+(NUMIST!=0))))
			{
			tree_up(iK_out,0,0,0);	
			ret(1000);
			}
          
         	else if((NUMDT)&&(sub_ind==(3+(NUMIST!=0))))
			{
			tree_up(iK_t_ext_6U,0,0,0);	
			ret(1000);			
			}
 						
          else if(sub_ind==(3+(NUMIST!=0)+(NUMDT!=0)))
			{
	          tree_down(0,0);
	          ret(0);
               }	               			
		}			
	}

else if(ind==iK_TELECORE)
	{
	ret(1000);
	if(but==butD)
		{
		sub_ind++;
		gran_char(&sub_ind,0,2+(NUMBAT_TELECORE!=0)+(NUMIST!=0)+(NUMDT!=0));
		}
	else if(but==butU)
		{
		sub_ind--;
		gran_char(&sub_ind,0,2+(NUMBAT_TELECORE!=0)+(NUMIST!=0)+(NUMDT!=0));
		}
	else if(but==butD_)
		{
		sub_ind=2+(NUMBAT_TELECORE!=0)+(NUMIST!=0)+(NUMDT!=0);
		}				
	else if(but==butE)
		{
		if(sub_ind==0)
			{
			tree_up(iK_net,0,0,0);
		    ret(1000);
			}
		else if((NUMBAT_TELECORE)&&(sub_ind==1))
			{
			tree_up(iK_bat_sel_TELECORE,0,0,0);	
			ret(1000);
			}
		else if((NUMIST)&&(sub_ind==(1+(NUMBAT_TELECORE!=0))))
			{
			tree_up(iK_bps_sel,0,0,0);	
			ret(1000);
			}

		else if((sub_ind==(1+(NUMBAT_TELECORE!=0)+(NUMIST!=0))))
			{
			tree_up(iK_load,0,0,0);	
			ret(1000);
			}
          
    	else if((NUMDT)&&(sub_ind==(2+(NUMBAT_TELECORE!=0)+(NUMIST!=0))))
			{
			tree_up(iK_t_ext_6U,0,0,0);	
			ret(1000);			
			}
    	else if(sub_ind==(2+(NUMBAT_TELECORE!=0)+(NUMIST!=0)+(NUMDT!=0)))
			{
	     	tree_down(0,0);
	        ret(0);
            }	               			
		}			
	}

else if(ind==iK_net)
	{
	ret(1000);
	if(but==butD)
		{
		sub_ind++;
		gran_char(&sub_ind,0,1);
		phase=0;
		}
	else if(but==butU)
		{
		sub_ind--;
		gran_char(&sub_ind,0,1);
		phase=0;
		}
	else if(but==butD_)
		{
		sub_ind=1;
		}				
	else if(sub_ind==0)
		{
		temp_SS=lc640_read_int(EE_KUNET);
		if(but==butR)
			{
			temp_SS++;
			}
		else if(but==butR_)
			{
			//temp_SS=lc640_read_int(EE_KUNET);
			temp_SS+=10;
			//lc640_write_int(EE_KUNET,temp_SS);
			}	
		else if(but==butL)
			{
			//temp_SS=lc640_read_int(EE_KUNET);
			temp_SS--;
			//lc640_write_int(EE_KUNET,temp_SS);
			}
		else if(but==butL_)
			{
			//temp_SS=lc640_read_int(EE_KUNET);
			temp_SS-=10;
			//lc640_write_int(EE_KUNET,temp_SS);
			}				
		speed=1;
		gran(&temp_SS,10,12000);
		lc640_write_int(EE_KUNET,temp_SS);
					
		}
	else if(sub_ind==1)
		{
		if(but==butE)
			{
			tree_down(0,1);
			ret(0);
			}
		}			
	}

else if(ind==iK_net3)
	{
	ret(1000);
	if(but==butD)
		{
		sub_ind++;
		gran_char(&sub_ind,0,3);
		phase=0;
		}
	else if(but==butU)
		{
		sub_ind--;
		gran_char(&sub_ind,0,3);
		phase=0;
		}
	else if(but==butD_)
		{
		sub_ind=3;
		}				
	else if(sub_ind==0)
		{
		temp_SS=lc640_read_int(EE_KUNETA);
		if(but==butR)
			{
			temp_SS++;
			}
		else if(but==butR_)
			{
			temp_SS+=10;
			}	
		else if(but==butL)
			{
			temp_SS--;
			}
		else if(but==butL_)
			{
			temp_SS-=10;
			}				
		speed=1;
		gran(&temp_SS,150,4000);
		lc640_write_int(EE_KUNETA,temp_SS);
		}

	else if(sub_ind==1)
		{
		temp_SS=lc640_read_int(EE_KUNETB);
		if(but==butR)
			{
			temp_SS++;
			}
		else if(but==butR_)
			{
			temp_SS+=10;
			}	
		else if(but==butL)
			{
			temp_SS--;
			}
		else if(but==butL_)
			{
			temp_SS-=10;
			}				
		speed=1;
		gran(&temp_SS,150,4000);
		lc640_write_int(EE_KUNETB,temp_SS);
		}

	else if(sub_ind==2)
		{
		temp_SS=lc640_read_int(EE_KUNETC);
		if(but==butR)
			{
			temp_SS++;
			}
		else if(but==butR_)
			{
			temp_SS+=10;
			}	
		else if(but==butL)
			{
			temp_SS--;
			}
		else if(but==butL_)
			{
			temp_SS-=10;
			}				
		speed=1;
		gran(&temp_SS,150,4000);
		lc640_write_int(EE_KUNETC,temp_SS);
		}

	else if(sub_ind==3)
		{
		if(but==butE)
			{
			tree_down(0,1);
			ret(0);
			}
		}			
	}

else if(ind==iK_power_net)
	{
	ret(1000);
	if(but==butD)
		{
		sub_ind++;
		gran_char(&sub_ind,0,2);
		phase=0;
		}
	else if(but==butU)
		{
		sub_ind--;
		gran_char(&sub_ind,0,2);
		phase=0;
		}
	else if(but==butD_)
		{
		sub_ind=3;
		}				
	else if(sub_ind==0)
		{
		temp_SS=lc640_read_int(EE_KUNET_EXT0);
		if(but==butR)
			{
			temp_SS++;
			}
		else if(but==butR_)
			{
			temp_SS+=10;
			}	
		else if(but==butL)
			{
			temp_SS--;
			}
		else if(but==butL_)
			{
			temp_SS-=10;
			}				
		speed=1;
		gran(&temp_SS,200,550);
		lc640_write_int(EE_KUNET_EXT0,temp_SS);
					
		}
	else if(sub_ind==1)
		{
		temp_SS=lc640_read_int(EE_KUNET_EXT1);
		if(but==butR)
			{
			temp_SS++;
			}
		else if(but==butR_)
			{
			temp_SS+=10;
			}	
		else if(but==butL)
			{
			temp_SS--;
			}
		else if(but==butL_)
			{
			temp_SS-=10;
			}				
		speed=1;
		gran(&temp_SS,200,550);
		lc640_write_int(EE_KUNET_EXT1,temp_SS);
					
		}



	else if(sub_ind==2)
		{
		if(but==butE)
			{
			tree_down(0,1);
			ret(0);
			}
		}			
	}

else if(ind==iK_power_net3)
	{
	ret(1000);
	if(but==butD)
		{
		sub_ind++;
		gran_char(&sub_ind,0,6);
		phase=0;
		}
	else if(but==butU)
		{
		sub_ind--;
		gran_char(&sub_ind,0,6);
		phase=0;
		}
	else if(but==butD_)
		{
		sub_ind=6;
		}
						
	else if(sub_ind==0)
		{
		temp_SS=lc640_read_int(EE_KVV0_EB2);
		if(but==butR)
			{
			temp_SS++;
			}
		else if(but==butR_)
			{
			temp_SS+=10;
			}	
		else if(but==butL)
			{
			temp_SS--;
			}
		else if(but==butL_)
			{
			temp_SS-=10;
			}				
		speed=1;
		gran(&temp_SS,200,550);
		lc640_write_int(EE_KVV0_EB2,temp_SS);
		}

	else if(sub_ind==1)
		{
		temp_SS=lc640_read_int(EE_KVV1_EB2);
		if(but==butR)
			{
			temp_SS++;
			}
		else if(but==butR_)
			{
			temp_SS+=10;
			}	
		else if(but==butL)
			{
			temp_SS--;
			}
		else if(but==butL_)
			{
			temp_SS-=10;
			}				
		speed=1;
		gran(&temp_SS,200,550);
		lc640_write_int(EE_KVV1_EB2,temp_SS);
		}

	else if(sub_ind==2)
		{
		temp_SS=lc640_read_int(EE_KVV2_EB2);
		if(but==butR)
			{
			temp_SS++;
			}
		else if(but==butR_)
			{
			temp_SS+=10;
			}	
		else if(but==butL)
			{
			temp_SS--;
			}
		else if(but==butL_)
			{
			temp_SS-=10;
			}				
		speed=1;
		gran(&temp_SS,200,550);
		lc640_write_int(EE_KVV2_EB2,temp_SS);
		}

	else if(sub_ind==3)
		{
		temp_SS=lc640_read_int(EE_KPES0_EB2);
		if(but==butR)
			{
			temp_SS++;
			}
		else if(but==butR_)
			{
			temp_SS+=10;
			}	
		else if(but==butL)
			{
			temp_SS--;
			}
		else if(but==butL_)
			{
			temp_SS-=10;
			}				
		speed=1;
		gran(&temp_SS,200,550);
		lc640_write_int(EE_KPES0_EB2,temp_SS);
		}

	else if(sub_ind==4)
		{
		temp_SS=lc640_read_int(EE_KPES1_EB2);
		if(but==butR)
			{
			temp_SS++;
			}
		else if(but==butR_)
			{
			temp_SS+=10;
			}	
		else if(but==butL)
			{
			temp_SS--;
			}
		else if(but==butL_)
			{
			temp_SS-=10;
			}				
		speed=1;
		gran(&temp_SS,200,550);
		lc640_write_int(EE_KPES1_EB2,temp_SS);
		}

	else if(sub_ind==5)
		{
		temp_SS=lc640_read_int(EE_KPES2_EB2);
		if(but==butR)
			{
			temp_SS++;
			}
		else if(but==butR_)
			{
			temp_SS+=10;
			}	
		else if(but==butL)
			{
			temp_SS--;
			}
		else if(but==butL_)
			{
			temp_SS-=10;
			}				
		speed=1;
		gran(&temp_SS,200,550);
		lc640_write_int(EE_KPES2_EB2,temp_SS);
		}




	else if(sub_ind==6)
		{
		if(but==butE)
			{
			tree_down(0,1);
			ret(0);
			}
		}			
	}

else if(ind==iK_bat_sel)
	{
	ret(1000);
	if(but==butD)
		{
		sub_ind++;
		gran_char(&sub_ind,0,NUMBAT);
		phase=0;
		}
	else if(but==butU)
		{
		sub_ind--;
		gran_char(&sub_ind,0,NUMBAT);
		phase=0;
		}
	else if(but==butD_)
		{
		sub_ind=1+NUMBAT;
		}	
	else if((but==butE)&&(NUMBAT)&&(BAT_IS_ON[0]==bisON)&&(sub_ind==0))
		{
		#ifdef UKU_6U
		tree_up(iK_bat_simple,0,0,0);
		#else

		#ifdef UKU_220_V2
		tree_up(iK_bat_simple,0,0,0);
		#else
		
	/*	#ifdef UKU_220 
		tree_up(iK_bat_simple,0,0,0);
		#else*/
		//tree_up(iK_bat,0,0,0);	
		#endif
		#endif
		//mess_send(MESS_SRC_CONTROL,0xFFFF,0,10);
     	//mess_send(MESS_BAT_CONTROL,0xFFFF&(~(1<<sub_ind1)),1<<(sub_ind1),10);

		ret(1000);
		}	
	else if((but==butE)&&(NUMBAT)&&(BAT_IS_ON[1]==bisON)&&(sub_ind==((BAT_IS_ON[0]==bisON))))
		{

		
		//mess_send(MESS_SRC_CONTROL,0xFFFF,0,10);
     	//mess_send(MESS_BAT_CONTROL,0xFFFF&(~(1<<sub_ind1)),1<<(sub_ind1),10);
     		
		ret(1000);
		}	
	else if(sub_ind==(NUMBAT))
		{
		if(but==butE)
			{
			tree_down(0,0);
			ret(0);
			}
		}				
	}

else if(ind==iK_bat_sel_TELECORE)
	{
	ret(1000);
	if(but==butD)
		{
		sub_ind++;
		gran_char(&sub_ind,0,NUMBAT_TELECORE);
		phase=0;
		}
	else if(but==butU)
		{
		sub_ind--;
		gran_char(&sub_ind,0,NUMBAT_TELECORE);
		phase=0;
		}
	else if(but==butD_)
		{
		sub_ind=1+NUMBAT_TELECORE;
		}
	else if(sub_ind==0)
		{
		if((but==butE)&&(NUMBAT_TELECORE>0))
			{
			tree_up(iK_bat_TELECORE,0,0,0);	
			ret(1000);
			}
		}	
	else if((sub_ind==1)&&(NUMBAT_TELECORE>1))
		{
		if((but==butE)&&(NUMBAT_TELECORE>1))
			{
			tree_up(iK_bat_TELECORE,0,0,1);	
			ret(1000);
			}
		}
	else if(sub_ind==(NUMBAT_TELECORE))
		{
		if(but==butE)
			{
			tree_down(0,0);
			ret(0);
			}
		}				
	}

else if(ind==iK_bat_TELECORE)
	{
	ret(1000);
	if(but==butD)
		{
		sub_ind++;
		if((sub_ind==1)||(sub_ind==2))sub_ind=3;
		gran_char(&sub_ind,0,3);
		phase=0;
		}
	else if(but==butU)
		{
		sub_ind--;
		if((sub_ind==1)||(sub_ind==2)) sub_ind=0;
        gran_char(&sub_ind,0,3);
		phase=0;
		}
	else if(but==butD_)
		{
		sub_ind=3;
		}			
					
	else if(sub_ind==0)
		{
		if(but==butE)
		     {
		     if(phase==0)
		          {
		          lc640_write_int(ADR_KI0BAT[sub_ind1],ad7705_buff_[sub_ind1]);
		          phase=1;
		          }
		     }	
		else
			{
			temp_SS=lc640_read_int(ADR_KI1BAT[sub_ind1]);
			if(but==butR)temp_SS++;
			else if(but==butR_)temp_SS+=2;
			else if(but==butL)temp_SS--;
			else if(but==butL_)temp_SS-=2;
						
			gran(&temp_SS,20,4000);
			lc640_write_int(ADR_KI1BAT[sub_ind1],temp_SS);
			phase=1;
			}
				
		speed=1;			
		}					
	else if(sub_ind==3)
		{
		if(but==butE)
			{
			tree_down(0,1);
			ret(0);
			}
		}			
	}	

else if(ind==iK_bat_ips_termokompensat_ib)
	{
	ret(1000);
	if(but==butD)
		{
		sub_ind++;
		if((sub_ind==1)||(sub_ind==2))sub_ind=3;
		gran_char(&sub_ind,0,3);
		phase=0;
		}
	else if(but==butU)
		{
		sub_ind--;
		if((sub_ind==1)||(sub_ind==2)) sub_ind=0;
          gran_char(&sub_ind,0,3);
		phase=0;
		}
	else if(but==butD_)
		{
		sub_ind=3;
		}			
					
	else if(sub_ind==0)
		{
		temp_SS=lc640_read_int(ADR_KI1BAT[0]);
		if(but==butR)temp_SS++;
		else if(but==butR_)temp_SS+=2;
		else if(but==butL)temp_SS--;
		else if(but==butL_)temp_SS-=2;
						
		gran(&temp_SS,200,30000);
		lc640_write_int(ADR_KI1BAT[0],temp_SS);
		phase=1;
		speed=1;
		}
									 	
	else if(sub_ind==3)
		{
		if(but==butE)
			{
			tree_down(0,1);
			ret(0);
			}
		}			
	}		

else if(ind==iK_bps_sel)
	{
	ret(1000);
	if(but==butD)
		{
		sub_ind++;
		gran_char(&sub_ind,0,NUMIST);
		phase=0;
		can1_out(sub_ind,sub_ind,CMND,ALRM_RES,0,0,0,0);
		}
	else if(but==butU)
		{
		sub_ind--;
		gran_char(&sub_ind,0,NUMIST);
		phase=0;
		can1_out(sub_ind,sub_ind,CMND,ALRM_RES,0,0,0,0);
		}
	else if(but==butD_)
		{
		sub_ind=1+NUMIST;
		}	
	else if((but==butE)&&(NUMIST)&&(sub_ind<NUMIST))
		{
		tree_up(iK_bps,0,0,sub_ind);	
		
		can1_out(sub_ind1,sub_ind1,CMND,ALRM_RES,0,0,0,0);
		//mess_send(MESS_SRC_CONTROL,0xFFFF,0,10);
     	//mess_send(MESS_BAT_CONTROL,0xFFFF&(~(1<<sub_ind1)),1<<(sub_ind1),10);

		ret(1000);
		}	
	else if(sub_ind==(NUMIST))
		{
		if(but==butE)
			{
			tree_down(0,0);
			ret(0);
			}
		}				
	}

else if(ind==iK_bps)
	{
	ret(1000);
	if(but==butD)
		{
		sub_ind++;
		if((sub_ind==1)||(sub_ind==2))sub_ind=3;
		else if((sub_ind==4)||(sub_ind==5))sub_ind=9;
		else if((sub_ind==7)||(sub_ind==8))sub_ind=9;
		else if((sub_ind==10)||(sub_ind==11))sub_ind=12;
		else if((sub_ind==13)||(sub_ind==14))sub_ind=15;
		else if((sub_ind==16)||(sub_ind==17))sub_ind=18;
		gran_char(&sub_ind,0,18);
		phase=0;
		}
	else if(but==butU)
		{
		sub_ind--;
		if((sub_ind==1)||(sub_ind==2))sub_ind=0;
		else if((sub_ind==4)||(sub_ind==5))sub_ind=3;
		else if((sub_ind==7)||(sub_ind==8))sub_ind=3;
		else if((sub_ind==10)||(sub_ind==11))sub_ind=9;
		else if((sub_ind==13)||(sub_ind==14))sub_ind=12;
		else if((sub_ind==16)||(sub_ind==17))sub_ind=15;		
		gran_char(&sub_ind,0,18);
		phase=0;
		}
	else if(but==butD_)
		{
		sub_ind=18;
		}
	else if (sub_ind == 0)
		{
		if(but==butLR) can1_out(sub_ind1,sub_ind1,KLBR,(0*16)+1,(0*16)+1,0,0,0);
	    else if(but==butR) can1_out(sub_ind1,sub_ind1,KLBR,(0*16)+2,(0*16)+2,0,0,0);
		else if(but==butR_)	can1_out(sub_ind1,sub_ind1,KLBR,(0*16)+3,(0*16)+3,0,0,0);
    	else if(but==butL) can1_out(sub_ind1,sub_ind1,KLBR,(0*16)+4,(0*16)+4,0,0,0); 
		else if(but==butL_) can1_out(sub_ind1,sub_ind1,KLBR,(0*16)+5,(0*16)+5,0,0,0);
		speed=1;
		}	
		
	else if (sub_ind == 3)
		{
		if(but==butLR) can1_out(sub_ind1,sub_ind1,KLBR,(1*16)+1,(1*16)+1,0,0,0);
	    else if(but==butR) can1_out(sub_ind1,sub_ind1,KLBR,(1*16)+2,(1*16)+2,0,0,0);
		else if(but==butR_)	can1_out(sub_ind1,sub_ind1,KLBR,(1*16)+3,(1*16)+3,0,0,0);
    	else if(but==butL) can1_out(sub_ind1,sub_ind1,KLBR,(1*16)+4,(1*16)+4,0,0,0); 
		else if(but==butL_) can1_out(sub_ind1,sub_ind1,KLBR,(1*16)+5,(1*16)+5,0,0,0);
		speed=1;
		}		
		
/*	else if (sub_ind == 6)
		{
		temp_SS=lc640_read_int(EE_U_AVT);
		if(but==butR)temp_SS++;
		else if(but==butR_)temp_SS+=2;
		else if(but==butL)temp_SS--;
		else if(but==butL_)temp_SS-=2;
		else if(but==butE_)can1_out(sub_ind1,sub_ind1,CMND,0xee,0xee,0,0,0);   
		

		gran(&temp_SS,200,1800);
	
		lc640_write_int(EE_U_AVT,temp_SS);
		
		speed=1;
		}*/	
		
	else if (sub_ind == 9)
		{
		if(but==butE)
			{
			can1_out(sub_ind1,sub_ind1,KLBR,(2*16)+1,(2*16)+1,0,0,0);
			phase=1;
			}
	    else if(but==butR) can1_out(sub_ind1,sub_ind1,KLBR,(2*16)+2,(2*16)+2,0,0,0);
		else if(but==butR_)	can1_out(sub_ind1,sub_ind1,KLBR,(2*16)+3,(2*16)+3,0,0,0);
    	else if(but==butL) can1_out(sub_ind1,sub_ind1,KLBR,(2*16)+4,(2*16)+4,0,0,0); 
		else if(but==butL_) can1_out(sub_ind1,sub_ind1,KLBR,(2*16)+5,(2*16)+5,0,0,0);
		speed=1;
		}		
		
	else if (sub_ind == 12)
		{
		if(but==butR) can1_out(sub_ind1,sub_ind1,KLBR,(3*16)+2,(3*16)+2,0,0,0);
		else if(but==butR_)	can1_out(sub_ind1,sub_ind1,KLBR,(3*16)+3,(3*16)+3,0,0,0);
    	else if(but==butL) can1_out(sub_ind1,sub_ind1,KLBR,(3*16)+4,(3*16)+4,0,0,0); 
		else if(but==butL_) can1_out(sub_ind1,sub_ind1,KLBR,(3*16)+5,(3*16)+5,0,0,0);
		speed=1;
		}								

	else if (sub_ind == 15)
		{
		if(but==butLR) can1_out(sub_ind1,sub_ind1,KLBR,(5*16)+1,(5*16)+1,0,0,0);
	    else if(but==butR) can1_out(sub_ind1,sub_ind1,KLBR,(5*16)+2,(5*16)+2,0,0,0);
		else if(but==butR_)	can1_out(sub_ind1,sub_ind1,KLBR,(5*16)+3,(5*16)+3,0,0,0);
    	else if(but==butL) can1_out(sub_ind1,sub_ind1,KLBR,(5*16)+4,(5*16)+4,0,0,0); 
		else if(but==butL_) can1_out(sub_ind1,sub_ind1,KLBR,(5*16)+5,(5*16)+5,0,0,0);
		speed=1;
		}
	else if(sub_ind==18)
		{
		if(but==butE)
			{
			tree_down(0,1);
			ret(0);
			}
		}
	}		

else if(ind==iRele_set)
	{
	ret(1000);
	if(but==butD)
		{
		sub_ind++;
		gran_char(&sub_ind,0,4);
		}
	else if(but==butU)
		{
		sub_ind--;
		gran_char(&sub_ind,0,4);
		}
	else if(but==butD_)
		{
		sub_ind=4;
		}	
	else if((but==butE)&&(sub_ind>=0)&&(sub_ind<=3))
		{
		tree_up(iRele_set_,0,0,sub_ind);	
		
		//can1_out(sub_ind1,sub_ind1,CMND,ALRM_RES,0,0,0,0);
		//mess_send(MESS_SRC_CONTROL,0xFFFF,0,10);
     	//mess_send(MESS_BAT_CONTROL,0xFFFF&(~(1<<sub_ind1)),1<<(sub_ind1),10);

		ret(1000);
		}	
	else if(sub_ind==4)
		{
		if(but==butE)
			{
			tree_down(0,0);
			ret(0);
			}
		}				
	}

else if(ind==iRele_set_)
	{
	ret(1000);
	if(but==butD)
		{
		sub_ind++;
		if(sub_ind==2)
			{
			sub_ind=3;
			index_set=2;
			}
		else if(sub_ind==8)
			{
			index_set=7;
			}
		else if(sub_ind==9)
			{
			sub_ind=10;
			}
		gran_char(&sub_ind,0,11);
		}
	else if(but==butU)
		{
		sub_ind--;
		if(sub_ind==2)sub_ind=1;
		else if(sub_ind==9)sub_ind=8;
		gran_char(&sub_ind,0,11);		
		}
	else if(but==butD_)
		{
		sub_ind=11;
		}
	else if (sub_ind == 0)
		{
		if(but==butE) RELE_SET_MASK[sub_ind1]^=(1<<0);
	    else if(but==butR) RELE_SET_MASK[sub_ind1]|=(1<<0);
    	else if(but==butL) RELE_SET_MASK[sub_ind1]&=~(1<<0);
		}	
		
	else if (sub_ind == 1)
		{
		if(but==butE) RELE_SET_MASK[sub_ind1]^=(1<<1);
	    else if(but==butR) RELE_SET_MASK[sub_ind1]|=(1<<1);
    	else if(but==butL) RELE_SET_MASK[sub_ind1]&=~(1<<1);
		}	
		
	else if (sub_ind == 3)
		{
		if(but==butE) RELE_SET_MASK[sub_ind1]^=(1<<2);
	    else if(but==butR) RELE_SET_MASK[sub_ind1]|=(1<<2);
    	else if(but==butL) RELE_SET_MASK[sub_ind1]&=~(1<<2);
		}
		
	else if (sub_ind == 4)
		{
		if(but==butE) RELE_SET_MASK[sub_ind1]^=(1<<3);
	    else if(but==butR) RELE_SET_MASK[sub_ind1]|=(1<<3);
    	else if(but==butL) RELE_SET_MASK[sub_ind1]&=~(1<<3);
		}	
		
	else if (sub_ind == 5)
		{
		if(but==butE) RELE_SET_MASK[sub_ind1]^=(1<<4);
	    else if(but==butR) RELE_SET_MASK[sub_ind1]|=(1<<4);
    	else if(but==butL) RELE_SET_MASK[sub_ind1]&=~(1<<4);
		}							

	else if (sub_ind == 6)
		{
		if(but==butE) RELE_SET_MASK[sub_ind1]^=(1<<5);
	    else if(but==butR) RELE_SET_MASK[sub_ind1]|=(1<<5);
    	else if(but==butL) RELE_SET_MASK[sub_ind1]&=~(1<<5);
		}
	else if (sub_ind == 7)
		{
		if(but==butE) RELE_SET_MASK[sub_ind1]^=(1<<6);
	    else if(but==butR) RELE_SET_MASK[sub_ind1]|=(1<<6);
    	else if(but==butL) RELE_SET_MASK[sub_ind1]&=~(1<<6);
		}
	else if (sub_ind == 8)
		{
		if(but==butE) RELE_SET_MASK[sub_ind1]^=(1<<7);
	    else if(but==butR) RELE_SET_MASK[sub_ind1]|=(1<<7);
    	else if(but==butL) RELE_SET_MASK[sub_ind1]&=~(1<<7);
		}

	else if (sub_ind == 10)
		{
		if(but==butE) RELE_SET_MASK[sub_ind1]^=(1<<8);
	    else if(but==butR) RELE_SET_MASK[sub_ind1]|=(1<<8);
    	else if(but==butL) RELE_SET_MASK[sub_ind1]&=~(1<<8);
		}

	else if(sub_ind==11)
		{
		if(but==butE)
			{
			tree_down(0,0);
			ret(0);
			}
		}
	//if(RELE_SET_MASK[sub_ind1]!=lc640_read_int(ADR_EE_RELE_SET_MASK[sub_ind1]))lc640_write_int(ADR_EE_RELE_SET_MASK[sub_ind1],RELE_SET_MASK[sub_ind1]);
	if(RELE_SET_MASK[sub_ind1]!=lc640_read_int(ADR_EE_RELE_SET_MASK[sub_ind1]))lc640_write_int(ADR_EE_RELE_SET_MASK[sub_ind1],RELE_SET_MASK[sub_ind1]);

	}

else if(ind==iInv_set)
	{
	ret(1000);
	if(but==butD)
		{
		sub_ind++;
		gran_char(&sub_ind,0,4);
		if(sub_ind==1)
			{
			sub_ind=2;
			index_set=1;
			}
		if(sub_ind==3)
			{
			sub_ind=4;
			index_set=2;
			}
		}
	else if(but==butU)
		{
		sub_ind--;
		gran_char(&sub_ind,0,4);
		if(sub_ind==1)sub_ind=0;
		if(sub_ind==3)
			{
			sub_ind=2;
			index_set=2;
			}
		}
	else if (sub_ind == 0)
		{
		if(but==butR) 		can1_out(sub_ind1+first_inv_slot,sub_ind1+first_inv_slot,KLBR,0xa2/*(12*16)+2*/,0xa2/*(12*16)+2*/,0,0,0);
		else if(but==butR_)	can1_out(sub_ind1+first_inv_slot,sub_ind1+first_inv_slot,KLBR,0xa3/*(12*16)+3*/,0xa3/*(12*16)+3*/,0,0,0);
    		else if(but==butL) 	can1_out(sub_ind1+first_inv_slot,sub_ind1+first_inv_slot,KLBR,0xa4/*(12*16)+4*/,0xa4/*(12*16)+4*/,0,0,0); 
		else if(but==butL_) can1_out(sub_ind1+first_inv_slot,sub_ind1+first_inv_slot,KLBR,0xa5/*(12*16)+5*/,0xa5/*(12*16)+5*/,0,0,0);
		speed=1;
		}
	else if (sub_ind == 2)
		{
		if(but==butR) 		can1_out(sub_ind1+first_inv_slot,sub_ind1+first_inv_slot,KLBR,0xb2/*(12*16)+2*/,0xb2/*(12*16)+2*/,0,0,0);
		else if(but==butR_)	can1_out(sub_ind1+first_inv_slot,sub_ind1+first_inv_slot,KLBR,0xb3/*(12*16)+3*/,0xb3/*(12*16)+3*/,0,0,0);
    		else if(but==butL) 	can1_out(sub_ind1+first_inv_slot,sub_ind1+first_inv_slot,KLBR,0xb4/*(12*16)+4*/,0xb4/*(12*16)+4*/,0,0,0); 
		else if(but==butL_) can1_out(sub_ind1+first_inv_slot,sub_ind1+first_inv_slot,KLBR,0xb5/*(12*16)+5*/,0xb5/*(12*16)+5*/,0,0,0);
		speed=1;
		}
	else if(sub_ind==4)
		{
		if(but==butE)
			{
			tree_down(0,0);
			ret(0);
			}
		}
	}
	
else if(ind==iK_makb_sel)
	{
	ret(1000);
	if(but==butD)
		{
		sub_ind++;
		gran_char(&sub_ind,0,NUMMAKB);
		}
	else if(but==butU)
		{
		sub_ind--;
		gran_char(&sub_ind,0,NUMMAKB);
		}
	else if(but==butD_)
		{
		sub_ind=NUMMAKB;
		}	
	else if((but==butE)&&(NUMMAKB)&&(sub_ind<NUMMAKB))
		{
		if(makb[sub_ind]._cnt<5)
			{
			tree_up(iK_makb,0,0,sub_ind);
			ret(1000);
			}
		else show_mess(
					"                    ",
	          		"   �� ���������!!!  ",
	          		"                    ",
	          		"                    ",1000);	
		}	
	else if(sub_ind==(NUMMAKB))
		{
		if(but==butE)
			{
			tree_down(0,0);
			ret(0);
			}
		}				
	}

else if(ind==iK_makb)
	{
	ret(1000);
	if (but==butU)
		{      
		sub_ind--;
		//if(sub_ind>7)sub_ind=7;
		//else if(sub_ind==1)sub_ind=0;
		gran_char(&sub_ind,0,simax);
		}
		
	else if (but==butD)
		{
		sub_ind++;
		//if(sub_ind<3)sub_ind=3;
		gran_char(&sub_ind,0,simax);
		}
	else if(but==butD_)
		{
		sub_ind=10;
		}
	else if ((sub_ind >= 0) && (sub_ind <= 9))
		{
		if(but==butLR) can1_out(sub_ind1,sub_ind1,KLBR_MAKB,		(sub_ind*16)+1,(sub_ind*16)+1,0,0,0);
	     else if(but==butR) can1_out(sub_ind1,sub_ind1,KLBR_MAKB,	(sub_ind*16)+2,(sub_ind*16)+2,0,0,0);
		else if(but==butR_)	can1_out(sub_ind1,sub_ind1,KLBR_MAKB,	(sub_ind*16)+3,(sub_ind*16)+3,0,0,0);
    		else if(but==butL) can1_out(sub_ind1,sub_ind1,KLBR_MAKB,	(sub_ind*16)+4,(sub_ind*16)+4,0,0,0); 
		else if(but==butL_) can1_out(sub_ind1,sub_ind1,KLBR_MAKB,	(sub_ind*16)+5,(sub_ind*16)+5,0,0,0);
		speed=1;
		}	
		
	else if(sub_ind==10)
		{
		if(but==butE)
			{
			tree_down(0,1);
			ret(0);
			}
		}			
	}




else if(ind==iK_load)
	{
	ret(1000);
	if(but==butD)
		{
		sub_ind=1;
		}
	else if(but==butU)
		{
		sub_ind=0;
		}
	else if(sub_ind==0)
		{
		temp_SS=lc640_read_int(EE_KULOAD);
	     if(but==butR)
	     	{
		     temp_SS++;
	     	}
	     else if(but==butR_)
	     	{
	     	temp_SS+=2;
	     	}	
	     else if(but==butL)
	     	{
	     	temp_SS--;
	     	}
	     else if(but==butL_)
	     	{
	     	temp_SS-=2;
	     	}
		#ifdef UKU_220
	    gran(&temp_SS,300,2000);
		#else 
		#ifdef UKU_220_IPS_TERMOKOMPENSAT
	    gran(&temp_SS,50,2000);
	    #else 
		gran(&temp_SS,100,5000);
		#endif
		#endif
		lc640_write_int(EE_KULOAD,temp_SS);					
		speed=1;	
					
		}
	else if(sub_ind==1)
		{
		if(but==butE)
			{
			tree_down(0,1);
			ret(0);
			}
		}			
	}

else if(ind==iK_out)
	{
	ret(1000);
	if(but==butD)
		{
		sub_ind++;
		gran_char(&sub_ind,0,2);
		}
	else if(but==butU)
		{
		sub_ind--;
		gran_char(&sub_ind,0,2);
		}
	else if(sub_ind==0)
		{
		temp_SS=lc640_read_int(EE_KUBPS);
	     if(but==butR)
	     	{
		     temp_SS++;
	     	}
	     else if(but==butR_)
	     	{
	     	temp_SS+=2;
	     	}	
	     else if(but==butL)
	     	{
	     	temp_SS--;
	     	}
	     else if(but==butL_)
	     	{
	     	temp_SS-=2;
	     	}
	    gran(&temp_SS,50,2000);

		lc640_write_int(EE_KUBPS,temp_SS);					
		speed=1;	
					
		}
	else if(sub_ind==1)
		{
		temp_SS=lc640_read_int(EE_KUOUT);
	     if(but==butR)
	     	{
		     temp_SS++;
	     	}
	     else if(but==butR_)
	     	{
	     	temp_SS+=2;
	     	}	
	     else if(but==butL)
	     	{
	     	temp_SS--;
	     	}
	     else if(but==butL_)
	     	{
	     	temp_SS-=2;
	     	}
	    gran(&temp_SS,50,2000);

		lc640_write_int(EE_KUOUT,temp_SS);					
		speed=1;	
					
		}
	else if(sub_ind==2)
		{
		if(but==butE)
			{
			tree_down(0,1);
			ret(0);
			}
		}			
	}

else if(ind==iK_t_ext)
	{
	ret(1000);
	if(but==butD)
		{
		sub_ind++;
		gran_char(&sub_ind,0,3);
		}
	else if(but==butU)
		{
		sub_ind--;
		gran_char(&sub_ind,0,3);
		}
	else if(sub_ind==0)
		{
		temp_SS=lc640_read_int(KT_EXT0);
	     if(but==butR)
	     	{
		     temp_SS++;
	     	}
	     else if(but==butR_)
	     	{
	     	temp_SS+=2;
	     	}	
	     else if(but==butL)
	     	{
	     	temp_SS--;
	     	}
	     else if(but==butL_)
	     	{
	     	temp_SS-=2;
	     	}
	     gran(&temp_SS,1900,3000);
		lc640_write_int(KT_EXT0,temp_SS);					
		speed=1;	
					
		}

	else if(sub_ind==1)
		{
		temp_SS=lc640_read_int(KT_EXT1);
	     if(but==butR)
	     	{
		     temp_SS++;
	     	}
	     else if(but==butR_)
	     	{
	     	temp_SS+=2;
	     	}	
	     else if(but==butL)
	     	{
	     	temp_SS--;
	     	}
	     else if(but==butL_)
	     	{
	     	temp_SS-=2;
	     	}
	     gran(&temp_SS,1900,3000);
		lc640_write_int(KT_EXT1,temp_SS);					
		speed=1;	
					
		}
	else if(sub_ind==2)
		{
		temp_SS=lc640_read_int(KT_EXT2);
	     if(but==butR)
	     	{
		     temp_SS++;
	     	}
	     else if(but==butR_)
	     	{
	     	temp_SS+=2;
	     	}	
	     else if(but==butL)
	     	{
	     	temp_SS--;
	     	}
	     else if(but==butL_)
	     	{
	     	temp_SS-=2;
	     	}
	     gran(&temp_SS,1900,3000);
		lc640_write_int(KT_EXT2,temp_SS);					
		speed=1;	
					
		}
	else if(sub_ind==3)
		{
		if(but==butE)
			{
			tree_down(0,1);
			ret(0);
			}
		}			
	}

else if(ind==iK_t_ext_6U)
	{
	ret(1000);
	if(but==butD)
		{
		sub_ind++;
		gran_char(&sub_ind,0,NUMDT);
		}
	else if(but==butU)
		{
		sub_ind--;
		gran_char(&sub_ind,0,NUMDT);
		}
	else if(sub_ind==0)
		{
		temp_SS=lc640_read_int(KT_EXT0);
	     if(but==butR)
	     	{
		     temp_SS++;
	     	}
	     else if(but==butR_)
	     	{
	     	temp_SS+=2;
	     	}	
	     else if(but==butL)
	     	{
	     	temp_SS--;
	     	}
	     else if(but==butL_)
	     	{
	     	temp_SS-=2;
	     	}
	     gran(&temp_SS,1900,3000);
		lc640_write_int(KT_EXT0,temp_SS);					
		speed=1;	
					
		}

	else if(sub_ind==1)
		{
		temp_SS=lc640_read_int(KT_EXT1);
	     if(but==butR)
	     	{
		     temp_SS++;
	     	}
	     else if(but==butR_)
	     	{
	     	temp_SS+=2;
	     	}	
	     else if(but==butL)
	     	{
	     	temp_SS--;
	     	}
	     else if(but==butL_)
	     	{
	     	temp_SS-=2;
	     	}
	     gran(&temp_SS,1900,3000);
		lc640_write_int(KT_EXT1,temp_SS);					
		speed=1;	
					
		}
	else if(sub_ind==2)
		{
		temp_SS=lc640_read_int(KT_EXT2);
	     if(but==butR)
	     	{
		     temp_SS++;
	     	}
	     else if(but==butR_)
	     	{
	     	temp_SS+=2;
	     	}	
	     else if(but==butL)
	     	{
	     	temp_SS--;
	     	}
	     else if(but==butL_)
	     	{
	     	temp_SS-=2;
	     	}
	     gran(&temp_SS,1900,3000);
		lc640_write_int(KT_EXT2,temp_SS);					
		speed=1;	
					
		}
 	if(sub_ind==NUMDT)
		{
		if(but==butE)
			{
			tree_down(0,1);
			ret(0);
			}
		}			
	}
			
else if(ind==iBatLog)
	{
	if(but==butD)
		{
		sub_ind++;
		gran_char(&sub_ind,0,6);
		}
	else if(but==butU)
		{
		sub_ind--;
		gran_char(&sub_ind,0,6);
		}
	else if(but==butD_)
		{
		sub_ind=6;
		}				
	else if((but==butL)&&((sub_ind==0)||(sub_ind==3)||(sub_ind==4)))
		{
		tree_down(0,0);
		}		
	else if(sub_ind==0)
	     {
	     if(but==butE)
	          {
	          //b[ptr_ind++]=a;
	          //if(BAT_IS_ON[sub_ind1]==bisON) ind=iPrl_bat_in_out;
	          //else 
	               //{
	               //ind=iPdp1;
	               //ret_ind(iPrl_bat_in_out,0,10);
	               //}
	          tree_up(iPrl_bat_in_out,0,0,sub_ind1);
	          if(BAT_IS_ON[sub_ind1]!=bisON) show_mess("  �������� �������  ",
	          								 "    ��������� ���   ",
	          								 "   ��������������   ",
	          								 "      ������!!!     ",4000);     
	          parol_init();
	          }
	     }
	else if(sub_ind==1)
	     {
	     if(but==butR)BAT_C_NOM[sub_ind1]++;
	     else if(but==butR_)BAT_C_NOM[sub_ind1]+=10;
	     else if(but==butL)BAT_C_NOM[sub_ind1]--;
	     else if(but==butL_)BAT_C_NOM[sub_ind1]-=10;
	     gran(&BAT_C_NOM[sub_ind1],0,2000);
	     lc640_write_int(ADR_EE_BAT_C_NOM[sub_ind1],BAT_C_NOM[sub_ind1]);
	     speed=1;
	     }		     
		
	else if(sub_ind==3)
		{
		if(but==butE)
			{ 
               cap=0;
			deep=lc640_read_int(CNT_EVENT_LOG);
			ptr=lc640_read_int(PTR_EVENT_LOG);
			
			if(deep>63)
				{
				deep=0;
			     ptr=0;
			     }
			     			
			for(i=0;i<deep;i++)
				{
				lc640_read_long_ptr(EVENT_LOG+(32*ptr),av_head);
				
				if((av_head[0]=='B')&&(av_head[1]==sub_ind1)&&(av_head[2]=='K')) 	//���� ������ ���������� ������� 'K'(�������� �������)
					{
					cap++;
					content[cap-1]=ptr;
					}
					
				ptr=ptr_carry(ptr,64,-1); 
	
				} 
				
			tree_up(iBatLogKe,0,0,sub_ind1);
			
			av_j_si_max=cap;
			if(av_j_si_max>63)av_j_si_max=0;
			} 
		}




	else if(sub_ind==4)
		{
		if(but==butE)
			{ 
               cap=0;
			deep=lc640_read_int(CNT_EVENT_LOG);
			ptr=lc640_read_int(PTR_EVENT_LOG);
			
			if(deep>63)
				{
				deep=0;
			     ptr=0;
			     }
			     
			for(i=0;i<deep;i++)
				{
				lc640_read_long_ptr(EVENT_LOG+(32*ptr),av_head);
				
				if((av_head[0]=='B')/*&&(av_head[1]==sub_ind1)*/&&(av_head[2]=='Z')) 	//���� ������ ���������� ������� 'z'(������������� �����)
					{
					cap++;
					content[cap-1]=ptr;
					}
					
				ptr=ptr_carry(ptr,64,-1); 
				} 
				
			tree_up(iBatLogVz,0,0,sub_ind1);   
			
			av_j_si_max=cap;
			if(av_j_si_max>63)av_j_si_max=0;
			} 
		else if(but==butR)
			{
			//vz_mem_hndl(0,5);
			//vz_mem_hndl(1,6);	       
			} 
		}

	else if(sub_ind==5)
		{
		if(but==butE)
			{ 
               cap=0;
			deep=lc640_read_int(CNT_EVENT_LOG);
			ptr=lc640_read_int(PTR_EVENT_LOG);

			if(deep>63)
				{
				deep=0;
			     ptr=0;
			     }
			
			//out_usart0 (8,0x11,*((char*)&deep),*(((char*)&deep)+1),*((char*)&ptr),*(((char*)&ptr)+1),cap,content[cap-1],i,0);
			
			for(i=0;i<deep;i++)
				{
				lc640_read_long_ptr(EVENT_LOG+(32*ptr),av_head);
				
				if((av_head[0]=='B')&&(av_head[1]==sub_ind1)&&(av_head[2]=='W')) 	//���� ������ ���������� ������� 'W'(�������)
					{
					cap++;
					content[cap-1]=ptr;
					}
					
		   	/*   	out_usart0 (8,0x22,*((char*)&deep),*(((char*)&deep)+1),*((char*)&ptr),*(((char*)&ptr)+1),cap,content[cap-1],i,0); 
				delay_ms(100);
				PORTC.7=!PORTC.7;
				#asm("wdr"); 	*/
				ptr=ptr_carry(ptr,64,-1); 
				} 
				
			/*ind=iJ_bat_wrk_sel;
			sub_ind=0;*/

			tree_up(iBatLogWrk,0,0,sub_ind1);
			
			av_j_si_max=cap;
			if(av_j_si_max>63)av_j_si_max=0;
			} 
		else if(but==butR)
			{
			//vz_mem_hndl(0,5);
			//vz_mem_hndl(1,6);	       
			} 
		}		
		 	         	
     else if(sub_ind==6)
	     {
	     if(but==butE)
	          {
			if(BAT_IS_ON[sub_ind1]!=bisON)tree_down(0,-4);
	          else tree_down(0,0);
	          }
	     }		     
		
	} 

else if(ind==iBatLogVz)
	{
	if(but==butD)
		{
		sub_ind++;
		gran_char(&sub_ind,0,av_j_si_max);
		}
	else if(but==butU)
		{
		sub_ind--;
		gran_char(&sub_ind,0,av_j_si_max);
		}
	else if(but==butE)
		{
		if(sub_ind==av_j_si_max)
			{
			tree_down(0,0);
			}
		} 
	else if(but==butL)
		{
		tree_down(0,0);
		}		
    //	else if(but==butR) vz_mem_hndl(sub_ind1,_sec);
	
		
	}

else if(ind==iBatLogKe)
	{
	if(but==butD)
		{
		sub_ind++;
		gran_char(&sub_ind,0,av_j_si_max);
		}
	else if(but==butU)
		{
		sub_ind--;
		gran_char(&sub_ind,0,av_j_si_max);
		}
	else if(but==butE)
		{
		if(sub_ind==av_j_si_max)
			{
			tree_down(0,0);
			}
		} 
	else if(but==butL)
		{
		tree_down(0,0);
		}		
    //	else if(but==butR) ke_mem_hndl(sub_ind1,_sec);		
	}

else if(ind==iBatLogWrk)
	{
	if(but==butD)
		{
		sub_ind++;
		gran_char(&sub_ind,0,av_j_si_max);
		}
	else if(but==butU)
		{
		sub_ind--;
		gran_char(&sub_ind,0,av_j_si_max);
		}
	else if(but==butE)
		{
		if(sub_ind==av_j_si_max)
			{
			tree_down(0,0);
			}
		else if(sub_ind<=av_j_si_max)
			{
			//ind=iWrk;
			//sub_ind2=content[sub_ind];
			index_set=0;
			//sub_ind=0;
			}	
		} 
	else if(but==butL)
		{
		tree_down(0,0);
		}		
	else if(but==butR)
		{
	    //	wrk_mem_hndl(sub_ind1);

		} 
	//int2lcdyx(BAT_TYPE[sub_ind1],0,2,0);
	}

else if(ind==iAv_view)
	{
	if(but==butE)
		{
		avar_ind_stat&=~(1L<<sub_ind);
		if(avar_ind_stat)
			{
			while(!(avar_ind_stat&(1<<sub_ind)))
				{
				sub_ind++;
				if(sub_ind>=32)
					{
					tree_down(0,0);
					avar_ind_stat=0;
					}
				}
		 	}
	 	else 
			{
			tree_down(0,0);
			avar_ind_stat=0;
			}
		}
 	}


		
else if(ind==iTst_VD)
	{
	ret(1000);
	if(but==butD)
		{
		sub_ind++;
		gran_char(&sub_ind,0,10+NUMIST);
		phase=0;
		tst_state[0]=tstOFF;
		tst_state[1]=tstOFF;
		tst_state[2]=tstOFF;
		tst_state[3]=tstOFF;
		tst_state[4]=tstOFF;
		tst_state[5]=tstOFF;
		tst_state[6]=tstOFF;
		tst_state[7]=tstOFF;
		tst_state[8]=tstOFF;

		if(sub_ind==1)
			{
			sub_ind=2;
			index_set=1;
			}
		if(sub_ind==3)
			{
			sub_ind=4;
			index_set=3;
			}
		if(sub_ind==5)
			{
			sub_ind=6;
			index_set=5;
			}
		if(sub_ind==7)
			{
			sub_ind=8;
			index_set=7;
			}
		}

	else if(but==butU)
		{
		sub_ind--;
		gran_char(&sub_ind,0,10+NUMIST);
		phase=0;
		tst_state[0]=tstOFF;
		tst_state[1]=tstOFF;
		tst_state[2]=tstOFF;
		tst_state[3]=tstOFF;
		tst_state[4]=tstOFF;
		tst_state[5]=tstOFF;
		tst_state[6]=tstOFF;
		tst_state[7]=tstOFF;
		tst_state[8]=tstOFF;
		
		if(sub_ind==7)
			{
			sub_ind=6;
			index_set=4;
			} 
		if(sub_ind==5)
			{
			sub_ind=4;
			index_set=5;
			} 
		if(sub_ind==3)
			{
			sub_ind=2;
			//index_set=5;
			}
		if(sub_ind==1)
			{
			sub_ind=0;
			//index_set=5;
			}
		}

	else if(sub_ind==0)
		{
		if((but==butE)||(but==butR))
			{
			if(tst_state[0]==tstOFF) tst_state[0]=tst1;
			else if(tst_state[0]==tst1) tst_state[0]=tst2;
			else tst_state[0]=tstOFF; 
			}
		else if(but==butL)
			{
			if(tst_state[0]==tst2) tst_state[0]=tst1;
			else if(tst_state[0]==tstOFF) tst_state[0]=tst2;
			else tst_state[0]=tstOFF; 
			}
		}
		
	else if(sub_ind==2)
		{
		if((but==butE)||(but==butR))
			{
			if(tst_state[1]==tstOFF) tst_state[1]=tst1;
			else if(tst_state[1]==tst1) tst_state[1]=tst2;
			else tst_state[1]=tstOFF;
			}
		else if(but==butL)
			{
			if(tst_state[1]==tst2) tst_state[1]=tst1;
			else if(tst_state[1]==tstOFF) tst_state[1]=tst2;
			else tst_state[1]=tstOFF;
			}
		}
			
	else if(sub_ind==4)
		{
		if((but==butE)||(but==butR))
			{
			if(tst_state[2]==tstOFF) tst_state[2]=tst1;
			else if(tst_state[2]==tst1) tst_state[2]=tst2;
			else tst_state[2]=tstOFF;
			}
		else if(but==butL)
			{
			if(tst_state[2]==tst2) tst_state[2]=tst1;
			else if(tst_state[2]==tstOFF) tst_state[2]=tst2;
			else tst_state[2]=tstOFF;
			}
		}

	else if(sub_ind==6)
		{
		if((but==butE)||(but==butR))
			{
			if(tst_state[3]==tstOFF) tst_state[3]=tst1;
			else if(tst_state[3]==tst1) tst_state[3]=tst2;
			else tst_state[3]=tstOFF;
			}
		else if(but==butL)
			{
			if(tst_state[3]==tst2) tst_state[3]=tst1;
			else if(tst_state[3]==tstOFF) tst_state[3]=tst2;
			else tst_state[3]=tstOFF;
			}
		}
	else if((sub_ind>=8)&&(sub_ind<(8+NUMIST))&&(NUMIST)&&((but==butE)))	
		{
		tree_up(iTst_bps,0,0,sub_ind-8);
		can1_out(sub_ind1,sub_ind1,CMND,ALRM_RES,0,0,0,0);
		tst_state[5]=tst1;
		tst_state[6]=tstOFF;
		}											

	else if(sub_ind==(8+NUMIST))
		{
		if(but==butE)
			{
			tree_down(0,0);
			ret(0);
			}
		}
	else if(sub_ind==(9+NUMIST))
		{
		if(but==butE)
			{
			bRESET_INT_WDT=1;
			}
		}
	else if(sub_ind==(10+NUMIST))
		{
		if(but==butE)
			{
			bRESET_EXT_WDT=1;
			}
		}					
	}
#ifdef UKU_TELECORE2015
else if(ind==iTst_TELECORE)
	{
	ret(1000);
	if(but==butD)
		{
		sub_ind++;
		gran_char(&sub_ind,0,14+NUMIST);
		phase=0;
		tst_state[0]=tstOFF;
		tst_state[1]=tstOFF;
		tst_state[2]=tstOFF;
		tst_state[3]=tstOFF;
		tst_state[4]=tstOFF;
		tst_state[5]=tstOFF;
		tst_state[6]=tstOFF;
		tst_state[7]=tstOFF;
		tst_state[8]=tstOFF;
		tst_state[9]=tstOFF;
		tst_state[10]=(enum_tst_state)0;

		if(sub_ind==1)
			{
			sub_ind=2;
			index_set=1;
			}
		if(sub_ind==3)
			{
			sub_ind=4;
			index_set=3;
			}
		if(sub_ind==5)
			{
			sub_ind=6;
			index_set=5;
			}
		if(sub_ind==7)
			{
			sub_ind=8;
			index_set=7;
			}
		if(sub_ind==9)
			{
			sub_ind=10;
			index_set=9;
			}
		if(sub_ind==11)
			{
			sub_ind=12;
			index_set=11;
			}
	/*	if(sub_ind==13)
			{
               sub_ind=14;
			//index_set=9;
			}*/
/*		if(sub_ind==11)
			{
			sub_ind=12;
			index_set=11;
			}
		if(sub_ind==13)
			{
			sub_ind=14;
			index_set=13;
			}   */
		}

	else if(but==butU)
		{
		sub_ind--;
		gran_char(&sub_ind,0,14+NUMIST);
		phase=0;
		tst_state[0]=tstOFF;
		tst_state[1]=tstOFF;
		tst_state[2]=tstOFF;
		tst_state[3]=tstOFF;
		tst_state[4]=tstOFF;
		tst_state[5]=tstOFF;
		tst_state[6]=tstOFF;
		tst_state[7]=tstOFF;
		tst_state[8]=tstOFF;
		tst_state[9]=tstOFF;
		tst_state[10]=(enum_tst_state)0;

/*		if(sub_ind==13)
			{
			sub_ind=12;
			}		
		if(sub_ind==11)
			{
			sub_ind=10;
			}*/
	/*	if(sub_ind==13)
			{
			sub_ind=12;
			}*/
		if(sub_ind==11)
			{
			sub_ind=10;
			}
		if(sub_ind==9)
			{
			sub_ind=8;
			}
		if(sub_ind==7)
			{
			sub_ind=6;
			}
		if(sub_ind==5)
			{
			sub_ind=4;
			}
		if(sub_ind==3)
			{
			sub_ind=2;
			}
		if(sub_ind==1)
			{
			sub_ind=0;
			}
		}

	else if(sub_ind==0)
		{
		if((but==butE)||(but==butR))
			{
			if(tst_state[0]==tstOFF) tst_state[0]=tst1;
			else if(tst_state[0]==tst1) tst_state[0]=tst2;
			else tst_state[0]=tstOFF; 
			}
		else if(but==butL)
			{
			if(tst_state[0]==tst2) tst_state[0]=tst1;
			else if(tst_state[0]==tstOFF) tst_state[0]=tst2;
			else tst_state[0]=tstOFF; 
			}
		}

	else if(sub_ind==2)
		{
		if((but==butE)||(but==butR))
			{
			if(tst_state[1]==tstOFF) tst_state[1]=tst1;
			else if(tst_state[1]==tst1) tst_state[1]=tst2;
			else tst_state[1]=tstOFF;
			}
		else if(but==butL)
			{
			if(tst_state[1]==tst2) tst_state[1]=tst1;
			else if(tst_state[1]==tstOFF) tst_state[1]=tst2;
			else tst_state[1]=tstOFF;
			}
		}

	else if(sub_ind==4)
		{
		if((but==butE)||(but==butR))
			{
			if(tst_state[3]==tstOFF) tst_state[3]=tst1;
			else if(tst_state[3]==tst1) tst_state[3]=tst2;
			else tst_state[3]=tstOFF;
			}
		else if(but==butL)
			{
			if(tst_state[3]==tst2) tst_state[3]=tst1;
			else if(tst_state[3]==tstOFF) tst_state[3]=tst2;
			else tst_state[3]=tstOFF;
			}
		}

	else if(sub_ind==6)
		{
		if((but==butE)||(but==butR))
			{
			if(tst_state[4]==tstOFF) tst_state[4]=tst1;
			else if(tst_state[4]==tst1) tst_state[4]=tst2;
			else tst_state[4]=tstOFF;
			}
		else if(but==butL)
			{
			if(tst_state[4]==tst2) tst_state[4]=tst1;
			else if(tst_state[4]==tstOFF) tst_state[4]=tst2;
			else tst_state[4]=tstOFF;
			}
		}

	else if(sub_ind==8)
		{
		if((but==butE)||(but==butR))
			{
			if(tst_state[5]==tstOFF) tst_state[5]=tst1;
			else if(tst_state[5]==tst1) tst_state[5]=tst2;
			else tst_state[5]=tstOFF;
			}
		else if(but==butL)
			{
			if(tst_state[5]==tst2) tst_state[5]=tst1;
			else if(tst_state[5]==tstOFF) tst_state[5]=tst2;
			else tst_state[5]=tstOFF;
			}
		}

	else if(sub_ind==10)
		{
		if((but==butE)||(but==butR)||(but==butL))
			{
			if(tst_state[2]==tstOFF) tst_state[2]=tst1;
			else tst_state[2]=tstOFF;
			}
		}

	else if((sub_ind>=12)&&(sub_ind<(12+NUMIST))&&(NUMIST)&&((but==butE)))	
		{
		tree_up(iTst_bps,0,0,sub_ind-12);
		can2_out(sub_ind1,sub_ind1,CMND,ALRM_RES,0,0,0,0);
		tst_state[5]=tst1;
		tst_state[6]=tstOFF;
		}											
	else if(sub_ind==(12+NUMIST))
		{
		if(but==butE)
			{
			bRESET=1;
			}
	
		}
	else if(sub_ind==(13+NUMIST))
		{
		if(but==butE)
			{
			bRESET_EXT=1;
			}
	
		}			
	else if(sub_ind==(14+NUMIST))
		{
		if(but==butE)
			{
			tree_down(0,0);
			ret(0);
			}
	
		}	
	}
#endif

#ifdef UKU_TELECORE2017
else if(ind==iTst_TELECORE)
	{
	ret(1000);
	if(but==butD)
		{
		sub_ind++;
		gran_char(&sub_ind,0,13+NUMIST);
		phase=0;
		tst_state[0]=tstOFF;
		tst_state[1]=tstOFF;
		tst_state[2]=tstOFF;
		tst_state[3]=tstOFF;
		tst_state[4]=tstOFF;
		tst_state[5]=tstOFF;
		tst_state[6]=tstOFF;
		tst_state[7]=tstOFF;
		tst_state[8]=tstOFF;
		tst_state[9]=tstOFF;
		tst_state[10]=(enum_tst_state)0;

		if(sub_ind==1)
			{
			sub_ind=2;
			index_set=1;
			}
		if(sub_ind==3)
			{
			sub_ind=4;
			index_set=3;
			}
		if(sub_ind==5)
			{
			sub_ind=6;
			index_set=5;
			}
		if(sub_ind==7)
			{
			sub_ind=8;
			index_set=7;
			}
		if(sub_ind==9)
			{
			sub_ind=10;
			index_set=9;
			}
		if(sub_ind==11)
			{
			sub_ind=12;
			index_set=11;
			}
		}

	else if(but==butU)
		{
		sub_ind--;
		gran_char(&sub_ind,0,13+NUMIST);
		phase=0;
		tst_state[0]=tstOFF;
		tst_state[1]=tstOFF;
		tst_state[2]=tstOFF;
		tst_state[3]=tstOFF;
		tst_state[4]=tstOFF;
		tst_state[5]=tstOFF;
		tst_state[6]=tstOFF;
		tst_state[7]=tstOFF;
		tst_state[8]=tstOFF;
		tst_state[9]=tstOFF;
		tst_state[10]=(enum_tst_state)0;

		if(sub_ind==11)
			{
			sub_ind=10;
			}
		if(sub_ind==9)
			{
			sub_ind=8;
			}
		if(sub_ind==7)
			{
			sub_ind=6;
			}
		if(sub_ind==5)
			{
			sub_ind=4;
			}
		if(sub_ind==3)
			{
			sub_ind=2;
			}
		if(sub_ind==1)
			{
			sub_ind=0;
			}
		}

	else if(sub_ind==0)
		{
		if((but==butE)||(but==butR))
			{
			if(tst_state[0]==tstOFF) tst_state[0]=tst1;
			else if(tst_state[0]==tst1) tst_state[0]=tst2;
			else tst_state[0]=tstOFF; 
			}
		else if(but==butL)
			{
			if(tst_state[0]==tst2) tst_state[0]=tst1;
			else if(tst_state[0]==tstOFF) tst_state[0]=tst2;
			else tst_state[0]=tstOFF; 
			}
		}

	else if(sub_ind==2)
		{
		if((but==butE)||(but==butR))
			{
			if(tst_state[1]==tstOFF) tst_state[1]=tst1;
			else if(tst_state[1]==tst1) tst_state[1]=tst2;
			else tst_state[1]=tstOFF;
			}
		else if(but==butL)
			{
			if(tst_state[1]==tst2) tst_state[1]=tst1;
			else if(tst_state[1]==tstOFF) tst_state[1]=tst2;
			else tst_state[1]=tstOFF;
			}
		}

	else if(sub_ind==4)
		{
		if((but==butE)||(but==butR))
			{
			if(tst_state[2]==tstOFF) tst_state[2]=tst1;
			else if(tst_state[2]==tst1) tst_state[2]=tst2;
			else tst_state[2]=tstOFF;
			}
		else if(but==butL)
			{
			if(tst_state[2]==tst2) tst_state[2]=tst1;
			else if(tst_state[2]==tstOFF) tst_state[2]=tst2;
			else tst_state[2]=tstOFF;
			}
		}

	else if(sub_ind==6)
		{
		if((but==butR))
			{
			if(tst_state[3]<5)tst_state[3]++;
			else tst_state[3]=5;
			}
		else if(but==butL)
			{
			if(tst_state[3]>0)tst_state[3]--;
			else tst_state[3]=0;
			}
		}

	else if(sub_ind==8)
		{
		if((but==butR))
			{
			if(tst_state[4]<5)tst_state[4]++;
			else tst_state[4]=5;
			}
		else if(but==butL)
			{
			if(tst_state[4]>0)tst_state[4]--;
			else tst_state[4]=0;
			}
		}

	else if(sub_ind==10)
		{
		if((but==butE)||(but==butR)||(but==butL))
			{
			if(tst_state[5]==tstOFF) tst_state[5]=tst1;
			else tst_state[5]=tstOFF;
			}
		}

	else if((sub_ind>=12)&&(sub_ind<(12+NUMIST))&&(NUMIST)&&((but==butE)))	
		{
		tree_up(iTst_bps,0,0,sub_ind-12);
		can2_out(sub_ind1,sub_ind1,CMND,ALRM_RES,0,0,0,0);
		tst_state[5]=tst1;
		tst_state[6]=tstOFF;
		}											
	else if(sub_ind==(12+NUMIST))
		{
		if(but==butE)
			{
			bRESET=1;
			bRESET_INT_WDT=1;
			bRESET_EXT_WDT=1;
			}
	
		}
	else if(sub_ind==(13+NUMIST))
		{
		if(but==butE)
			{
			tree_down(0,0);
			ret(0);
			}
	
		}	
	}
#endif


else if(ind==iTst_bps)
	{
	ret(1000);
	if(but==butD)
		{
		sub_ind++;
		gran_char(&sub_ind,0,3);
		tst_state[5]=tst1;
		tst_state[6]=tstOFF;
		
		if(sub_ind==2)
			{
			sub_ind=3;
			//index_set=2;
			}

		}

	else if(but==butU)
		{
		sub_ind--;
		gran_char(&sub_ind,0,3);
		tst_state[5]=tst1;
		tst_state[6]=tstOFF;
		
		if(sub_ind==2)
			{
			sub_ind=1;
			//index_set=2;
			}
		}

	else if(sub_ind==0)
		{
		if(but==butR)
			{
			if(tst_state[5]==tstOFF)tst_state[5]=tst1;
			//else if(tst_state[5]==tst1)tst_state[5]=tst2;
			else tst_state[5]=tstOFF;
			}
		else if(but==butL)
			{
			if(tst_state[5]==tstOFF)tst_state[5]=tst1;
			//else if(tst_state[5]==tst1)tst_state[5]=tstOFF;
			else tst_state[5]=tstOFF;
			}
		}
	else if(sub_ind==1)
		{
		if((but==butE)||(but==butR))
			{
			if(tst_state[6]==tstOFF) tst_state[6]=tst1;
			//else if(tst_state[6]==tst1) tst_state[6]=tst2;
			else tst_state[6]=tstOFF;
			}
		else if(but==butL)
			{
			if(tst_state[6]==tstOFF) tst_state[6]=tst1;
			//else if(tst_state[6]==tstOFF) tst_state[6]=tst2;
			else tst_state[6]=tstOFF;
			}
		}		
		
	else if(sub_ind==3)
		{
		if(but==butE)
			{
			tree_down(0,0);
			ret(0);
			}
		}	
	}

else if(ind==iKlimat)
	{
	ret(1000);
	if(but==butD)
		{
		sub_ind++;
		gran_char(&sub_ind,0,7);
	
		}

	else if(but==butU)
		{
		sub_ind--;
		gran_char(&sub_ind,0,7);
		
		}
	else if(sub_ind==0)
	     {
	     if(but==butR)TBOXMAX++;
	     else if(but==butR_)TBOXMAX+=2;
	     else if(but==butL)TBOXMAX--;
	     else if(but==butL_)TBOXMAX-=2;
	     gran(&TBOXMAX,50,80);
	     lc640_write_int(EE_TBOXMAX,TBOXMAX);
	     speed=1;
	     }

	else if(sub_ind==1)
	     {
	     if(but==butR)TBOXVENTMAX++;
	     else if(but==butR_)TBOXVENTMAX+=2;
	     else if(but==butL)TBOXVENTMAX--;
	     else if(but==butL_)TBOXVENTMAX-=2;
	     gran(&TBOXVENTMAX,49,81);
	     lc640_write_int(EE_TBOXVENTMAX,TBOXVENTMAX);
	     speed=1;
	     }

	else if(sub_ind==2)
	     {
	     if(but==butR)TBOXREG++;
	     else if(but==butR_)TBOXREG+=2;
	     else if(but==butL)TBOXREG--;
	     else if(but==butL_)TBOXREG-=2;
	     gran(&TBOXREG,5,30);
	     lc640_write_int(EE_TBOXREG,TBOXREG);
	     speed=1;
	     }

	else if(sub_ind==3)
	     {
	     if(but==butR)TLOADDISABLE++;
	     else if(but==butR_)TLOADDISABLE+=2;
	     else if(but==butL)TLOADDISABLE--;
	     else if(but==butL_)TLOADDISABLE-=2;
	     gran(&TLOADDISABLE,49,81);
	     lc640_write_int(EE_TLOADDISABLE,TLOADDISABLE);
	     speed=1;
	     }

	else if(sub_ind==4)
	     {
	     if(but==butR)TLOADENABLE++;
	     else if(but==butR_)TLOADENABLE+=2;
	     else if(but==butL)TLOADENABLE--;
	     else if(but==butL_)TLOADENABLE-=2;
	     gran(&TLOADENABLE,44,TLOADDISABLE-5);
	     lc640_write_int(EE_TLOADENABLE,TLOADENABLE);
	     speed=1;
	     }

	else if(sub_ind==5)
	     {
	     if(but==butR)TBATDISABLE++;
	     else if(but==butR_)TBATDISABLE+=2;
	     else if(but==butL)TBATDISABLE--;
	     else if(but==butL_)TBATDISABLE-=2;
	     gran(&TBATDISABLE,49,91);
	     lc640_write_int(EE_TBATDISABLE,TBATDISABLE);
	     speed=1;
	     }

	else if(sub_ind==6)
	     {
	     if(but==butR)TBATENABLE++;
	     else if(but==butR_)TBATENABLE+=2;
	     else if(but==butL)TBATENABLE--;
	     else if(but==butL_)TBATENABLE-=2;
	     gran(&TBATENABLE,44,TBATDISABLE-5);
	     lc640_write_int(EE_TBATENABLE,TBATENABLE);
	     speed=1;
	     }
	else if(sub_ind==7)
		{
		if(but==butE)
			{
			tree_down(0,0);
			ret(0);
			}
		}
	}

else if(ind==iKlimat_kontur)
	{
	ret(1000);
	if(but==butD)
		{
		sub_ind++;
		gran_char(&sub_ind,0,9);
	
		}

	else if(but==butU)
		{
		sub_ind--;
		gran_char(&sub_ind,0,9);
		
		}
	else if(sub_ind==0)
	     {
	     if(but==butR)TBOXMAX++;
	     else if(but==butR_)TBOXMAX+=2;
	     else if(but==butL)TBOXMAX--;
	     else if(but==butL_)TBOXMAX-=2;
	     gran(&TBOXMAX,50,80);
	     lc640_write_int(EE_TBOXMAX,TBOXMAX);
	     speed=1;
	     }

/*	else if(sub_ind==1)
	     {
	     if(but==butR)TBOXVENTON++;
	     else if(but==butR_)TBOXVENTON+=2;
	     else if(but==butL)TBOXVENTON--;
	     else if(but==butL_)TBOXVENTON-=2;
	     gran(&TBOXVENTON,TBOXVENTOFF+2,150);
	     lc640_write_int(EE_TBOXVENTON,TBOXVENTON);
	     speed=1;
	     }

	else if(sub_ind==2)
	     {
	     if(but==butR)TBOXVENTOFF++;
	     else if(but==butR_)TBOXVENTOFF+=2;
	     else if(but==butL)TBOXVENTOFF--;
	     else if(but==butL_)TBOXVENTOFF-=2;
	     gran(&TBOXVENTOFF,TBOXWARMOFF+2,TBOXVENTON-2);
	     lc640_write_int(EE_TBOXVENTOFF,TBOXVENTOFF);
	     speed=1;
	     }  */
	else if(sub_ind==1)
	     {
	     if(but==butR)TBOXVENTMAX++;
	     else if(but==butR_)TBOXVENTMAX+=2;
	     else if(but==butL)TBOXVENTMAX--;
	     else if(but==butL_)TBOXVENTMAX-=2;
	     gran(&TBOXVENTMAX,49,81);
	     lc640_write_int(EE_TBOXVENTMAX,TBOXVENTMAX);
	     speed=1;
	     }

	else if(sub_ind==2)
	     {
	     if(but==butR)TBOXREG++;
	     else if(but==butR_)TBOXREG+=2;
	     else if(but==butL)TBOXREG--;
	     else if(but==butL_)TBOXREG-=2;
	     //gran(&TBOXREG,5,30);
		gran(&TBOXREG,0,50);
	     lc640_write_int(EE_TBOXREG,TBOXREG);
	     speed=1;
	     }


	else if(sub_ind==3)
	     {
	     if(but==butR)TBOXWARMON++;
	     else if(but==butR_)TBOXWARMON+=2;
	     else if(but==butL)TBOXWARMON--;
	     else if(but==butL_)TBOXWARMON-=2;
	     //gran(&TBOXWARMON,-20,20);
		gran(&TBOXWARMON,-50,50);
	     lc640_write_int(EE_TBOXWARMON,TBOXWARMON);
	     speed=1;
	     }

	else if(sub_ind==4)
	     {
	     if(but==butR)TBOXWARMOFF++;
	     else if(but==butR_)TBOXWARMOFF+=2;
	     else if(but==butL)TBOXWARMOFF--;
	     else if(but==butL_)TBOXWARMOFF-=2;
	     //gran(&TBOXWARMOFF,-20,20);
		gran(&TBOXWARMOFF,-50,50);
	     lc640_write_int(EE_TBOXWARMOFF,TBOXWARMOFF);
	     speed=1;
	     }

	else if(sub_ind==5)
	     {
	     if(but==butR)TLOADDISABLE++;
	     else if(but==butR_)TLOADDISABLE+=2;
	     else if(but==butL)TLOADDISABLE--;
	     else if(but==butL_)TLOADDISABLE-=2;
	     gran(&TLOADDISABLE,49,81);
	     lc640_write_int(EE_TLOADDISABLE,TLOADDISABLE);
	     speed=1;
	     }

	else if(sub_ind==6)
	     {
	     if(but==butR)TLOADENABLE++;
	     else if(but==butR_)TLOADENABLE+=2;
	     else if(but==butL)TLOADENABLE--;
	     else if(but==butL_)TLOADENABLE-=2;
	     gran(&TLOADENABLE,44,TLOADDISABLE-5);
	     lc640_write_int(EE_TLOADENABLE,TLOADENABLE);
	     speed=1;
	     }

	else if(sub_ind==7)
	     {
	     if(but==butR)TBATDISABLE++;
	     else if(but==butR_)TBATDISABLE+=2;
	     else if(but==butL)TBATDISABLE--;
	     else if(but==butL_)TBATDISABLE-=2;
	     gran(&TBATDISABLE,49,91);
	     lc640_write_int(EE_TBATDISABLE,TBATDISABLE);
	     speed=1;
	     }

	else if(sub_ind==8)
	     {
	     if(but==butR)TBATENABLE++;
	     else if(but==butR_)TBATENABLE+=2;
	     else if(but==butL)TBATENABLE--;
	     else if(but==butL_)TBATENABLE-=2;
	     gran(&TBATENABLE,44,TBATDISABLE-5);
	     lc640_write_int(EE_TBATENABLE,TBATENABLE);
	     speed=1;
	     }
	else if(sub_ind==9)
		{
		if(but==butE)
			{
			tree_down(0,0);
			ret(0);
			}
		}
	}
#ifdef UKU_TELECORE2015
 else if(ind==iKlimat_TELECORE)
	{
	ret(1000);
	if(but==butD)
		{
		sub_ind++;
		if(sub_ind==1)
			{
			sub_ind++;
			index_set=1;
			}
		gran_char(&sub_ind,0,11);
		if(sub_ind==3)sub_ind++;
		gran_char(&sub_ind,0,11);	
		}

	else if(but==butU)
		{
		sub_ind--;
		if(sub_ind==1)sub_ind--;
		gran_char(&sub_ind,0,11);
		if(sub_ind==3)sub_ind--;
		gran_char(&sub_ind,0,11);
		
		}
	else if(sub_ind==0)
	     {
	     if(but==butR)TELECORE2015_KLIMAT_WARM_SIGNAL++;
	     else if(but==butR_)TELECORE2015_KLIMAT_WARM_SIGNAL++;
	     else if(but==butL)TELECORE2015_KLIMAT_WARM_SIGNAL--;
	     else if(but==butL_)TELECORE2015_KLIMAT_WARM_SIGNAL--;
	     gran_ring(&TELECORE2015_KLIMAT_WARM_SIGNAL,0,1);
	     lc640_write_int(EE_TELECORE2015_KLIMAT_WARM_SIGNAL,TELECORE2015_KLIMAT_WARM_SIGNAL);
	     speed=1;
	     }
	else if(sub_ind==2)
	     {
	     if(but==butR)TELECORE2015_KLIMAT_VENT_SIGNAL++;
	     else if(but==butR_)TELECORE2015_KLIMAT_VENT_SIGNAL++;
	     else if(but==butL)TELECORE2015_KLIMAT_VENT_SIGNAL--;
	     else if(but==butL_)TELECORE2015_KLIMAT_VENT_SIGNAL--;
	     gran_ring(&TELECORE2015_KLIMAT_VENT_SIGNAL,0,1);
	     lc640_write_int(EE_TELECORE2015_KLIMAT_VENT_SIGNAL,TELECORE2015_KLIMAT_VENT_SIGNAL);
	     speed=1;
	     }
	else if(sub_ind==4)
	     {
	     if(but==butR)TELECORE2015_KLIMAT_WARM_ON++;
	     else if(but==butR_)TELECORE2015_KLIMAT_WARM_ON+=2;
	     else if(but==butL)TELECORE2015_KLIMAT_WARM_ON--;
	     else if(but==butL_)TELECORE2015_KLIMAT_WARM_ON-=2;
	     gran(&TELECORE2015_KLIMAT_WARM_ON,-20,50);
	     lc640_write_int(EE_TELECORE2015_KLIMAT_WARM_ON,TELECORE2015_KLIMAT_WARM_ON);
	     speed=1;
	     }

	else if(sub_ind==5)
	     {
	     if(but==butR)TELECORE2015_KLIMAT_WARM_OFF++;
	     else if(but==butR_)TELECORE2015_KLIMAT_WARM_OFF+=2;
	     else if(but==butL)TELECORE2015_KLIMAT_WARM_OFF--;
	     else if(but==butL_)TELECORE2015_KLIMAT_WARM_OFF-=2;
	     gran(&TELECORE2015_KLIMAT_WARM_OFF,-20,50);
	     lc640_write_int(EE_TELECORE2015_KLIMAT_WARM_OFF,TELECORE2015_KLIMAT_WARM_OFF);
	     speed=1;
	     }


	else if(sub_ind==6)
	     {
	     if(but==butR)TELECORE2015_KLIMAT_CAP++;
	     else if(but==butR_)TELECORE2015_KLIMAT_CAP+=2;
	     else if(but==butL)TELECORE2015_KLIMAT_CAP--;
	     else if(but==butL_)TELECORE2015_KLIMAT_CAP-=2;
	     //gran(&TBOXWARMON,-20,20);
		gran(&TELECORE2015_KLIMAT_CAP,5,95);
	     lc640_write_int(EE_TELECORE2015_KLIMAT_CAP,TELECORE2015_KLIMAT_CAP);
	     speed=1;
	     }

	else if(sub_ind==7)
	     {
	     if(but==butR)TELECORE2015_KLIMAT_VENT_ON++;
	     else if(but==butR_)TELECORE2015_KLIMAT_VENT_ON+=2;
	     else if(but==butL)TELECORE2015_KLIMAT_VENT_ON--;
	     else if(but==butL_)TELECORE2015_KLIMAT_VENT_ON-=2;
	     gran(&TELECORE2015_KLIMAT_VENT_ON,0,80);
		gran(&TELECORE2015_KLIMAT_VENT_ON,TELECORE2015_KLIMAT_VENT_OFF,80);
	     lc640_write_int(EE_TELECORE2015_KLIMAT_VENT_ON,TELECORE2015_KLIMAT_VENT_ON);
	     speed=1;
	     }

	else if(sub_ind==8)
	     {
	     if(but==butR)TELECORE2015_KLIMAT_VENT_OFF++;
	     else if(but==butR_)TELECORE2015_KLIMAT_VENT_OFF+=2;
	     else if(but==butL)TELECORE2015_KLIMAT_VENT_OFF--;
	     else if(but==butL_)TELECORE2015_KLIMAT_VENT_OFF-=2;
	     gran(&TELECORE2015_KLIMAT_VENT_OFF,0,80);
		gran(&TELECORE2015_KLIMAT_VENT_OFF,0,TELECORE2015_KLIMAT_VENT_OFF);
	     lc640_write_int(EE_TELECORE2015_KLIMAT_VENT_OFF,TELECORE2015_KLIMAT_VENT_OFF);
	     speed=1;
	     }
	else if(sub_ind==9)
	     {
	     if(but==butR)TELECORE2015_KLIMAT_VVENT_ON++;
	     else if(but==butR_)TELECORE2015_KLIMAT_VVENT_ON+=2;
	     else if(but==butL)TELECORE2015_KLIMAT_VVENT_ON--;
	     else if(but==butL_)TELECORE2015_KLIMAT_VVENT_ON-=2;
	     gran(&TELECORE2015_KLIMAT_VVENT_ON,0,80);
		gran(&TELECORE2015_KLIMAT_VVENT_ON,TELECORE2015_KLIMAT_VVENT_OFF,80);
	     lc640_write_int(EE_TELECORE2015_KLIMAT_VVENT_ON,TELECORE2015_KLIMAT_VVENT_ON);
	     speed=1;
	     }

	else if(sub_ind==10)
	     {
	     if(but==butR)TELECORE2015_KLIMAT_VVENT_OFF++;
	     else if(but==butR_)TELECORE2015_KLIMAT_VVENT_OFF+=2;
	     else if(but==butL)TELECORE2015_KLIMAT_VVENT_OFF--;
	     else if(but==butL_)TELECORE2015_KLIMAT_VVENT_OFF-=2;
	     gran(&TELECORE2015_KLIMAT_VVENT_OFF,0,80);
		gran(&TELECORE2015_KLIMAT_VVENT_OFF,0,TELECORE2015_KLIMAT_VVENT_OFF);
	     lc640_write_int(EE_TELECORE2015_KLIMAT_VVENT_OFF,TELECORE2015_KLIMAT_VVENT_OFF);
	     speed=1;
	     }


	else if(sub_ind==11)
		{
		if(but==butE)
			{
			tree_down(0,0);
			ret(0);
			}
		}
	}
#endif
#ifdef UKU_TELECORE2017
 else if(ind==iKlimat_TELECORE)
	{
	ret(1000);
	if(but==butD)
		{
		sub_ind++;
		if(sub_ind==1)
			{
			sub_ind++;
			index_set=1;
			}
		gran_char(&sub_ind,0,17);
		if(sub_ind==3)sub_ind++;
		gran_char(&sub_ind,0,17);	
		}

	else if(but==butU)
		{
		sub_ind--;
		if(sub_ind==1)sub_ind--;
		gran_char(&sub_ind,0,17);
 		}
	else if(but==butLR_)
		{
		lc640_write_int(EE_TELECORE2017_KLIMAT_WARM_SIGNAL,0);
		lc640_write_int(EE_TELECORE2017_KLIMAT_VENT_SIGNAL,1);
		lc640_write_int(EE_TELECORE2017_KLIMAT_WARM_ON,10);
		lc640_write_int(EE_TELECORE2017_KLIMAT_WARM_OFF,30);
		lc640_write_int(EE_TELECORE2017_KLIMAT_CAP,30);
		//lc640_write_int(EE_TELECORE2017_KLIMAT_VENT_ON0,30);
	    lc640_write_int(EE_TELECORE2017_KLIMAT_VENT_ON20,40);
	    lc640_write_int(EE_TELECORE2017_KLIMAT_VENT_ON40,50);
	    lc640_write_int(EE_TELECORE2017_KLIMAT_VENT_ON60,60);
	    lc640_write_int(EE_TELECORE2017_KLIMAT_VENT_ON80,70);
	    lc640_write_int(EE_TELECORE2017_KLIMAT_VENT_ON100,80);
	    //lc640_write_int(EE_TELECORE2017_KLIMAT_DVENT_ON0,10);
	    lc640_write_int(EE_TELECORE2017_KLIMAT_DVENT_ON20,20);
	    lc640_write_int(EE_TELECORE2017_KLIMAT_DVENT_ON40,30);
	    lc640_write_int(EE_TELECORE2017_KLIMAT_DVENT_ON60,40);
	    lc640_write_int(EE_TELECORE2017_KLIMAT_DVENT_ON80,50);
	    lc640_write_int(EE_TELECORE2017_KLIMAT_DVENT_ON100,60);
 		}

	else if(sub_ind==0)
	     {
	     if(but==butR)TELECORE2017_KLIMAT_WARM_SIGNAL++;
	     else if(but==butR_)TELECORE2017_KLIMAT_WARM_SIGNAL++;
	     else if(but==butL)TELECORE2017_KLIMAT_WARM_SIGNAL--;
	     else if(but==butL_)TELECORE2017_KLIMAT_WARM_SIGNAL--;
	     gran_ring(&TELECORE2017_KLIMAT_WARM_SIGNAL,0,1);
	     lc640_write_int(EE_TELECORE2017_KLIMAT_WARM_SIGNAL,TELECORE2017_KLIMAT_WARM_SIGNAL);
	     speed=1;
	     }
	else if(sub_ind==2)
	     {
	     if(but==butR)TELECORE2017_KLIMAT_VENT_SIGNAL++;
	     else if(but==butR_)TELECORE2017_KLIMAT_VENT_SIGNAL++;
	     else if(but==butL)TELECORE2017_KLIMAT_VENT_SIGNAL--;
	     else if(but==butL_)TELECORE2017_KLIMAT_VENT_SIGNAL--;
	     gran_ring(&TELECORE2017_KLIMAT_VENT_SIGNAL,0,1);
	     lc640_write_int(EE_TELECORE2017_KLIMAT_VENT_SIGNAL,TELECORE2017_KLIMAT_VENT_SIGNAL);
	     speed=1;
	     }
	else if(sub_ind==4)
	     {
	     if(but==butR)TELECORE2017_KLIMAT_WARM_ON++;
	     else if(but==butR_)TELECORE2017_KLIMAT_WARM_ON+=2;
	     else if(but==butL)TELECORE2017_KLIMAT_WARM_ON--;
	     else if(but==butL_)TELECORE2017_KLIMAT_WARM_ON-=2;
	     gran(&TELECORE2017_KLIMAT_WARM_ON,-20,50);
	     lc640_write_int(EE_TELECORE2017_KLIMAT_WARM_ON,TELECORE2017_KLIMAT_WARM_ON);
	     speed=1;
	     }

	else if(sub_ind==5)
	     {
	     if(but==butR)TELECORE2017_KLIMAT_WARM_OFF++;
	     else if(but==butR_)TELECORE2017_KLIMAT_WARM_OFF+=2;
	     else if(but==butL)TELECORE2017_KLIMAT_WARM_OFF--;
	     else if(but==butL_)TELECORE2017_KLIMAT_WARM_OFF-=2;
	     gran(&TELECORE2017_KLIMAT_WARM_OFF,-20,50);
	     lc640_write_int(EE_TELECORE2017_KLIMAT_WARM_OFF,TELECORE2017_KLIMAT_WARM_OFF);
	     speed=1;
	     }


	else if(sub_ind==6)
	     {
	     if(but==butR)TELECORE2017_KLIMAT_CAP++;
	     else if(but==butR_)TELECORE2017_KLIMAT_CAP+=2;
	     else if(but==butL)TELECORE2017_KLIMAT_CAP--;
	     else if(but==butL_)TELECORE2017_KLIMAT_CAP-=2;
	     gran(&TELECORE2017_KLIMAT_CAP,5,95);
	     lc640_write_int(EE_TELECORE2017_KLIMAT_CAP,TELECORE2017_KLIMAT_CAP);
	     speed=1;
	     }

/*	else if(sub_ind==7)
	     {
	     if(but==butR)TELECORE2017_KLIMAT_VENT_ON0++;
	     else if(but==butR_)TELECORE2017_KLIMAT_VENT_ON0+=2;
	     else if(but==butL)TELECORE2017_KLIMAT_VENT_ON0--;
	     else if(but==butL_)TELECORE2017_KLIMAT_VENT_ON0-=2;
		 gran(&TELECORE2017_KLIMAT_VENT_ON0,0,TELECORE2017_KLIMAT_VENT_ON20);
	     lc640_write_int(EE_TELECORE2017_KLIMAT_VENT_ON0,TELECORE2017_KLIMAT_VENT_ON0);
	     speed=1;
	     }*/


	else if(sub_ind==7)
	     {
	     if(but==butR)TELECORE2017_KLIMAT_VENT_ON100++;
	     else if(but==butR_)TELECORE2017_KLIMAT_VENT_ON100+=2;
	     else if(but==butL)TELECORE2017_KLIMAT_VENT_ON100--;
	     else if(but==butL_)TELECORE2017_KLIMAT_VENT_ON100-=2;
		 gran(&TELECORE2017_KLIMAT_VENT_ON100,TELECORE2017_KLIMAT_VENT_ON80,100);
	     lc640_write_int(EE_TELECORE2017_KLIMAT_VENT_ON100,TELECORE2017_KLIMAT_VENT_ON100);
	     speed=1;
	     }
	else if(sub_ind==8)
	     {
	     if(but==butR)TELECORE2017_KLIMAT_VENT_ON80++;
	     else if(but==butR_)TELECORE2017_KLIMAT_VENT_ON80+=2;
	     else if(but==butL)TELECORE2017_KLIMAT_VENT_ON80--;
	     else if(but==butL_)TELECORE2017_KLIMAT_VENT_ON80-=2;
		 gran(&TELECORE2017_KLIMAT_VENT_ON80,TELECORE2017_KLIMAT_VENT_ON60,TELECORE2017_KLIMAT_VENT_ON100);
	     lc640_write_int(EE_TELECORE2017_KLIMAT_VENT_ON80,TELECORE2017_KLIMAT_VENT_ON80);
	     speed=1;
	     }


	else if(sub_ind==9)
	     {
	     if(but==butR)TELECORE2017_KLIMAT_VENT_ON60++;
	     else if(but==butR_)TELECORE2017_KLIMAT_VENT_ON60+=2;
	     else if(but==butL)TELECORE2017_KLIMAT_VENT_ON60--;
	     else if(but==butL_)TELECORE2017_KLIMAT_VENT_ON60-=2;
		 gran(&TELECORE2017_KLIMAT_VENT_ON60,TELECORE2017_KLIMAT_VENT_ON40,TELECORE2017_KLIMAT_VENT_ON80);
	     lc640_write_int(EE_TELECORE2017_KLIMAT_VENT_ON60,TELECORE2017_KLIMAT_VENT_ON60);
	     speed=1;
	     }

	else if(sub_ind==10)
	     {
	     if(but==butR)TELECORE2017_KLIMAT_VENT_ON40++;
	     else if(but==butR_)TELECORE2017_KLIMAT_VENT_ON40+=2;
	     else if(but==butL)TELECORE2017_KLIMAT_VENT_ON40--;
	     else if(but==butL_)TELECORE2017_KLIMAT_VENT_ON40-=2;
		 gran(&TELECORE2017_KLIMAT_VENT_ON40,TELECORE2017_KLIMAT_VENT_ON20,TELECORE2017_KLIMAT_VENT_ON60);
	     lc640_write_int(EE_TELECORE2017_KLIMAT_VENT_ON40,TELECORE2017_KLIMAT_VENT_ON40);
	     speed=1;
	     }

	else if(sub_ind==11)
	     {
	     if(but==butR)TELECORE2017_KLIMAT_VENT_ON20++;
	     else if(but==butR_)TELECORE2017_KLIMAT_VENT_ON20+=2;
	     else if(but==butL)TELECORE2017_KLIMAT_VENT_ON20--;
	     else if(but==butL_)TELECORE2017_KLIMAT_VENT_ON20-=2;
		 gran(&TELECORE2017_KLIMAT_VENT_ON20,0,TELECORE2017_KLIMAT_VENT_ON40);
	     lc640_write_int(EE_TELECORE2017_KLIMAT_VENT_ON20,TELECORE2017_KLIMAT_VENT_ON20);
	     speed=1;
	     }

/*	else if(sub_ind==12)
	     {
	     if(but==butR)TELECORE2017_KLIMAT_DVENT_ON0++;
	     else if(but==butR_)TELECORE2017_KLIMAT_DVENT_ON0+=2;
	     else if(but==butL)TELECORE2017_KLIMAT_DVENT_ON0--;
	     else if(but==butL_)TELECORE2017_KLIMAT_DVENT_ON0-=2;
		 gran(&TELECORE2017_KLIMAT_DVENT_ON0,0,TELECORE2017_KLIMAT_DVENT_ON20);
	     lc640_write_int(EE_TELECORE2017_KLIMAT_DVENT_ON0,TELECORE2017_KLIMAT_DVENT_ON0);
	     speed=1;
	     }*/

	else if(sub_ind==12)
	     {
	     if(but==butR)TELECORE2017_KLIMAT_DVENT_ON100++;
	     else if(but==butR_)TELECORE2017_KLIMAT_DVENT_ON100+=2;
	     else if(but==butL)TELECORE2017_KLIMAT_DVENT_ON100--;
	     else if(but==butL_)TELECORE2017_KLIMAT_DVENT_ON100-=2;
		 gran(&TELECORE2017_KLIMAT_DVENT_ON100,TELECORE2017_KLIMAT_DVENT_ON80,100);
	     lc640_write_int(EE_TELECORE2017_KLIMAT_DVENT_ON100,TELECORE2017_KLIMAT_DVENT_ON100);
	     speed=1;
	     }

	else if(sub_ind==13)
	     {
	     if(but==butR)TELECORE2017_KLIMAT_DVENT_ON80++;
	     else if(but==butR_)TELECORE2017_KLIMAT_DVENT_ON80+=2;
	     else if(but==butL)TELECORE2017_KLIMAT_DVENT_ON80--;
	     else if(but==butL_)TELECORE2017_KLIMAT_DVENT_ON80-=2;
		 gran(&TELECORE2017_KLIMAT_DVENT_ON80,TELECORE2017_KLIMAT_DVENT_ON60,TELECORE2017_KLIMAT_DVENT_ON100);
	     lc640_write_int(EE_TELECORE2017_KLIMAT_DVENT_ON80,TELECORE2017_KLIMAT_DVENT_ON80);
	     speed=1;
	     }

	else if(sub_ind==14)
	     {
	     if(but==butR)TELECORE2017_KLIMAT_DVENT_ON60++;
	     else if(but==butR_)TELECORE2017_KLIMAT_DVENT_ON60+=2;
	     else if(but==butL)TELECORE2017_KLIMAT_DVENT_ON60--;
	     else if(but==butL_)TELECORE2017_KLIMAT_DVENT_ON60-=2;
		 gran(&TELECORE2017_KLIMAT_DVENT_ON60,TELECORE2017_KLIMAT_DVENT_ON40,TELECORE2017_KLIMAT_DVENT_ON80);
	     lc640_write_int(EE_TELECORE2017_KLIMAT_DVENT_ON60,TELECORE2017_KLIMAT_DVENT_ON60);
	     speed=1;
	     }

	else if(sub_ind==15)
	     {
	     if(but==butR)TELECORE2017_KLIMAT_DVENT_ON40++;
	     else if(but==butR_)TELECORE2017_KLIMAT_DVENT_ON40+=2;
	     else if(but==butL)TELECORE2017_KLIMAT_DVENT_ON40--;
	     else if(but==butL_)TELECORE2017_KLIMAT_DVENT_ON40-=2;
		 gran(&TELECORE2017_KLIMAT_DVENT_ON40,TELECORE2017_KLIMAT_DVENT_ON20,TELECORE2017_KLIMAT_DVENT_ON60);
	     lc640_write_int(EE_TELECORE2017_KLIMAT_DVENT_ON40,TELECORE2017_KLIMAT_DVENT_ON40);
	     speed=1;
	     }

	else if(sub_ind==16)
	     {
	     if(but==butR)TELECORE2017_KLIMAT_DVENT_ON20++;
	     else if(but==butR_)TELECORE2017_KLIMAT_DVENT_ON20+=2;
	     else if(but==butL)TELECORE2017_KLIMAT_DVENT_ON20--;
	     else if(but==butL_)TELECORE2017_KLIMAT_DVENT_ON20-=2;
		 gran(&TELECORE2017_KLIMAT_DVENT_ON20,0,TELECORE2017_KLIMAT_DVENT_ON40);
	     lc640_write_int(EE_TELECORE2017_KLIMAT_DVENT_ON20,TELECORE2017_KLIMAT_DVENT_ON20);
	     speed=1;
	     }

	else if(sub_ind==17)
		{
		if(but==butE)
			{
			tree_down(0,0);
			ret(0);
			}
		}
	}
#endif
else if(ind==iNpn_set)
	{
	ret(1000);
	if(but==butD)
		{
		sub_ind++;
		gran_char(&sub_ind,0,simax);
	
		}

	else if(but==butU)
		{
		sub_ind--;
		gran_char(&sub_ind,0,simax);
		
		}
	else if(sub_ind==0)
	    {
	    if(NPN_OUT==npnoRELEVENT)NPN_OUT=npnoRELEAVBAT2;
		else if(NPN_OUT==npnoRELEAVBAT2)NPN_OUT=npnoOFF;
		else NPN_OUT=npnoRELEVENT;
	    lc640_write_int(EE_NPN_OUT,NPN_OUT);
	    
	    }
	else if(sub_ind==1)
	    {
		if(NPN_OUT==npnoOFF)
			{
			if(but==butE)			
				{
				tree_down(0,0);
				ret(0);
				}
			}
		else
			{
			/*if(NPN_SIGN==npnsAVNET)NPN_SIGN=npnsULOAD;
			else NPN_SIGN=npnsAVNET;
			lc640_write_int(EE_NPN_SIGN,NPN_SIGN);*/

			if(but==butR)UONPN++;
	     	else if(but==butR_)UONPN+=2;
	     	else if(but==butL)UONPN--;
	     	else if(but==butL_)UONPN-=2;
	     	gran(&UONPN,100,2500);
	     	lc640_write_int(EE_UONPN,UONPN);
	     	speed=1;

			}
		}
	else if(sub_ind==2)
		{
/*		if(NPN_SIGN==npnsULOAD)
			{
			if(but==butR)UONPN++;
	     	else if(but==butR_)UONPN+=2;
	     	else if(but==butL)UONPN--;
	     	else if(but==butL_)UONPN-=2;
	     	gran(&UONPN,100,2500);
	     	lc640_write_int(EE_UONPN,UONPN);
	     	speed=1;
			}
		else 
			{
			if(but==butR)TZNPN++;
	     	else if(but==butR_)TZNPN+=2;
	     	else if(but==butL)TZNPN--;
	     	else if(but==butL_)TZNPN-=2;
	     	gran(&TZNPN,10,60);
	     	lc640_write_int(EE_TZNPN,TZNPN);
	     	speed=1;
			}*/

			if(but==butR)UVNPN++;
	     	else if(but==butR_)UVNPN+=2;
	     	else if(but==butL)UVNPN--;
	     	else if(but==butL_)UVNPN-=2;
	     	gran(&UVNPN,100,2500);
	     	lc640_write_int(EE_UVNPN,UVNPN);
	     	speed=1;
		}
	else if(sub_ind==3)
		{
/*		if(NPN_SIGN==npnsULOAD)
			{
			if(but==butR)UVNPN++;
	     	else if(but==butR_)UVNPN+=2;
	     	else if(but==butL)UVNPN--;
	     	else if(but==butL_)UVNPN-=2;
	     	gran(&UVNPN,100,2500);
	     	lc640_write_int(EE_UVNPN,UVNPN);
	     	speed=1;
			}
		else 
			{
			if(but==butE)			
				{
				tree_down(0,0);
				ret(0);
				}
			}*/
			if(but==butR)TZNPN++;
	     	else if(but==butR_)TZNPN+=2;
	     	else if(but==butL)TZNPN--;
	     	else if(but==butL_)TZNPN-=2;
	     	gran(&TZNPN,10,60);
	     	lc640_write_int(EE_TZNPN,TZNPN);
	     	speed=1;
		}
	else if(sub_ind==4)
		{
/*		if(NPN_SIGN==npnsULOAD)
			{
			if(but==butR)TZNPN++;
	     	else if(but==butR_)TZNPN+=2;
	     	else if(but==butL)TZNPN--;
	     	else if(but==butL_)TZNPN-=2;
	     	gran(&TZNPN,10,60);
	     	lc640_write_int(EE_TZNPN,TZNPN);
	     	speed=1;
			}*/
			if(but==butE)			
				{
				tree_down(0,0);
				ret(0);
				}
		}
	else if(sub_ind==5)
		{
		if(NPN_SIGN==npnsULOAD)
			{
			if(but==butE)			
				{
				tree_down(0,0);
				ret(0);
				}
			}
		}


	}
else if(ind==iBps_list)
	{
	ret_ind(0,0,0);
	if (but==butU)
		{      
		sub_ind1--;
		gran_char(&sub_ind1,0,NUMIST-2);
		}
		
	else if (but==butD)
		{
		sub_ind1++;
		gran_char(&sub_ind1,0,NUMIST-2);
		}

	else if (but==butD_)
		{
		sub_ind1=NUMIST-2;
		}
				
	else if(but==butR)
		{
		sub_ind++;
		gran_char(&sub_ind,0,3);
		}
				
	else if(but==butL)
		{
		sub_ind--;
		gran_char(&sub_ind,0,3);
		}
	else if(but==butE)
		{
		tree_down(0,0);
		}
		
	else if(but==butE_)
		{
		if(bAVG_BLOCK) bAVG_BLOCK=0;
		else bAVG_BLOCK=1;
		}						
	}
else if(ind==iAvt_set_sel)
	{
	ret(1000);
	if (but==butU)
		{      
		sub_ind--;
		gran_char(&sub_ind,0,NUMIST);
		}
		
	else if (but==butD)
		{
		sub_ind++;
		gran_char(&sub_ind,0,NUMIST);
		}
	else if((sub_ind>=0)&&(sub_ind<NUMIST))
		{
		if(but==butE)
			{
			tree_up(iAvt_set,0,0,sub_ind);
			}
		}
	else if(sub_ind==NUMIST)
		{
		if(but==butE)
			{
			tree_down(0,0);
			}	
		}
	}
else if(ind==iAvt_set)
	{
	ret(1000);
	if(but==butD)
		{
		sub_ind++;
		if((sub_ind==1)||(sub_ind==2))sub_ind=3;
		gran_char(&sub_ind,0,3);
		phase=0;
		}
	else if(but==butU)
		{
		sub_ind--;
		if((sub_ind==1)||(sub_ind==2))sub_ind=0;
		gran_char(&sub_ind,0,3);
		phase=0;
		}
	else if(but==butD_)
		{
		sub_ind=3;
		}
 	else if (sub_ind == 0)
		{
		temp_SS=lc640_read_int(EE_U_AVT);
		if(but==butR)temp_SS++;
		else if(but==butR_)temp_SS+=2;
		else if(but==butL)temp_SS--;
		else if(but==butL_)temp_SS-=2;
		else if(but==butE_)can1_out(sub_ind1,sub_ind1,CMND,0xee,0xee,0,0,0);   
		
		#ifdef UKU206_220				
		gran(&temp_SS,1000,3000);
		#endif

		#ifdef UKU206_24
		gran(&temp_SS,200,300);
		#endif

		#ifdef UKU320
		gran(&temp_SS,400,800);
		#endif

		#ifdef UKU320_24
		gran(&temp_SS,200,300);
		#endif

		#ifdef UKU320_F
		gran(&temp_SS,400,800);
		#endif		
		lc640_write_int(EE_U_AVT,temp_SS);
		
		speed=1;	
					
		}	
	else if(sub_ind==3)
		{
		if(but==butE)
			{
			//a=b[--ptr_ind];
			//sub_ind++;
			tree_down(0,1);
			ret(0);
			}
		}			

	}
else if(ind==iOut_volt_contr)
	{
	ret(1000);
	if(but==butD)
		{
		sub_ind++;
		gran_char(&sub_ind,0,3);
		}
	else if(but==butU)
		{
		sub_ind--;
		gran_char(&sub_ind,0,3);
		}
	else if(but==butD_)
		{
		sub_ind=3;
		}
	else if(sub_ind==3)
		{
		if(but==butE)
			{
			tree_down(0,0);
			ret(0);
			}
		}
	else if(sub_ind==0)
		{
		if(but==butR)U_OUT_KONTR_MAX++;
		else if(but==butR_)U_OUT_KONTR_MAX=(U_OUT_KONTR_MAX/5+1)*5;
		else if(but==butL)U_OUT_KONTR_MAX--;
		else if(but==butL_)U_OUT_KONTR_MAX=(U_OUT_KONTR_MAX/5-1)*5;
		gran(&U_OUT_KONTR_MAX,10,3000);
		lc640_write_int(EE_U_OUT_KONTR_MAX,U_OUT_KONTR_MAX);
		speed=1;
		}				

	else if(sub_ind==1)
		{
		if(but==butR)U_OUT_KONTR_MIN++;
		else if(but==butR_)U_OUT_KONTR_MIN=(U_OUT_KONTR_MIN/5+1)*5;
		else if(but==butL)U_OUT_KONTR_MIN--;
		else if(but==butL_)U_OUT_KONTR_MIN=(U_OUT_KONTR_MIN/5-1)*5;
		gran(&U_OUT_KONTR_MIN,10,3000);
		lc640_write_int(EE_U_OUT_KONTR_MIN,U_OUT_KONTR_MIN);
		speed=1;
		}				


	else if(sub_ind==2)
		{
		if(but==butR)U_OUT_KONTR_DELAY++;
	    else if(but==butR_)U_OUT_KONTR_DELAY+=2;
	    else if(but==butL)U_OUT_KONTR_DELAY--;
	    else if(but==butL_)U_OUT_KONTR_DELAY-=2;
	    gran(&U_OUT_KONTR_DELAY,5,100);
	    lc640_write_int(EE_U_OUT_KONTR_DELAY,U_OUT_KONTR_DELAY);
	    speed=1;
		}				


/*	
	int2lcd(U_BAT_KONTR_MAX,'!',1);
	int2lcd(U_BAT_KONTR_MIN,'@',1);
	int2lcd(U_BAT_KONTR_DELAY,'#',0);
	*/

	}

else if(ind==iDop_rele_set)
	{
	ret(1000);
	if(but==butD)
		{
		sub_ind+=2;
		gran_char(&sub_ind,0,2);
		}
	else if(but==butU)
		{
		sub_ind-=2;
		gran_char(&sub_ind,0,2);
		}
	else if(but==butD_)
		{
		sub_ind=2;
		}
	else if(sub_ind==2)
		{
		if(but==butE)
			{
			tree_down(0,0);
			ret(0);
			}
		}
	else if(sub_ind==0)
		{
		if((but==butR)||(but==butR_))DOP_RELE_FUNC++;
		if((but==butL)||(but==butL_))DOP_RELE_FUNC--;
		gran(&DOP_RELE_FUNC,0,1);
		lc640_write_int(EE_DOP_RELE_FUNC,DOP_RELE_FUNC);
		speed=1;
		}				
	}

else if (ind==iIps_Curr_Avg_Set)
	{
     ret(1000);
	if(but==butD)
		{
		sub_ind++;
		sub_ind1=0;
		gran_char(&sub_ind,0,simax);
		}
	else if(but==butU)
		{
		sub_ind--;
		sub_ind1=0;
		gran_char(&sub_ind,0,simax);
		}
	else if(but==butD_)
		{
		sub_ind=simax;
		}			
	else if(sub_ind==simax)
		{
		if(but==butE)tree_down(0,0);
		}

	else if(sub_ind==0)
		{
		if(but==butE)
			{
			if(ICA_EN)ICA_EN=0;
			else ICA_EN=1;
			lc640_write_int(EE_ICA_EN,ICA_EN);
			}
		}
	else if(ICA_EN)
		{
		if(sub_ind==1)
			{
			if(but==butE)
				{
				if(ICA_CH)ICA_CH=0;
				else ICA_CH=1;
				lc640_write_int(EE_ICA_CH,ICA_CH);
				}
			}
		else if(ICA_CH==0)
			{
			if(sub_ind==2)
				{
				if((but==butR)||(but==butR_))
					{
					ICA_MODBUS_ADDRESS++;
					gran(&ICA_MODBUS_ADDRESS,1,254);
					lc640_write_int(EE_ICA_MODBUS_ADDRESS,ICA_MODBUS_ADDRESS);
					speed=1;
					}
				if((but==butL)||(but==butL_))
					{
					ICA_MODBUS_ADDRESS--;
					gran(&ICA_MODBUS_ADDRESS,1,254);
					lc640_write_int(EE_ICA_MODBUS_ADDRESS,ICA_MODBUS_ADDRESS);
					speed=1;
					}
				}
			}

		else if(ICA_CH==1)
			{
			if(sub_ind==2)
				{
				if((but==butE)||(but==butE_))
					{
					sub_ind1++;
					gran_ring_char(&sub_ind1,0,3);
					}
				else if(sub_ind1==0)
					{
					if((but==butR)||(but==butR_))
						{
						ICA_MODBUS_TCP_IP1++;
						gran_ring(&ICA_MODBUS_TCP_IP1,0,255);
						lc640_write_int(EE_ICA_MODBUS_TCP_IP1,ICA_MODBUS_TCP_IP1);
						speed=1;
						}
					if((but==butL)||(but==butL_))
						{
						ICA_MODBUS_TCP_IP1--;
						gran(&ICA_MODBUS_TCP_IP1,0,255);
						lc640_write_int(EE_ICA_MODBUS_TCP_IP1,ICA_MODBUS_TCP_IP1);
						speed=1;
						}
					}
				else if(sub_ind1==1)
					{
					if((but==butR)||(but==butR_))
						{
						ICA_MODBUS_TCP_IP2++;
						gran_ring(&ICA_MODBUS_TCP_IP2,0,255);
						lc640_write_int(EE_ICA_MODBUS_TCP_IP2,ICA_MODBUS_TCP_IP2);
						speed=1;
						}
					if((but==butL)||(but==butL_))
						{
						ICA_MODBUS_TCP_IP2--;
						gran(&ICA_MODBUS_TCP_IP2,0,255);
						lc640_write_int(EE_ICA_MODBUS_TCP_IP2,ICA_MODBUS_TCP_IP2);
						speed=1;
						}
					}
				else if(sub_ind1==2)
					{
					if((but==butR)||(but==butR_))
						{
						ICA_MODBUS_TCP_IP3++;
						gran_ring(&ICA_MODBUS_TCP_IP3,0,255);
						lc640_write_int(EE_ICA_MODBUS_TCP_IP3,ICA_MODBUS_TCP_IP3);
						speed=1;
						}
					if((but==butL)||(but==butL_))
						{
						ICA_MODBUS_TCP_IP3--;
						gran(&ICA_MODBUS_TCP_IP3,0,255);
						lc640_write_int(EE_ICA_MODBUS_TCP_IP3,ICA_MODBUS_TCP_IP3);
						speed=1;
						}
					}
				else if(sub_ind1==3)
					{
					if((but==butR)||(but==butR_))
						{
						ICA_MODBUS_TCP_IP4++;
						gran_ring(&ICA_MODBUS_TCP_IP4,0,255);
						lc640_write_int(EE_ICA_MODBUS_TCP_IP4,ICA_MODBUS_TCP_IP4);
						speed=1;
						}
					if((but==butL)||(but==butL_))
						{
						ICA_MODBUS_TCP_IP4--;
						gran(&ICA_MODBUS_TCP_IP4,0,255);
						lc640_write_int(EE_ICA_MODBUS_TCP_IP4,ICA_MODBUS_TCP_IP4);
						speed=1;
						}
					}
				}
			if(sub_ind==3)
				{
				if((but==butR)||(but==butR_))
					{
					ICA_MODBUS_TCP_UNIT_ID++;
					gran(&ICA_MODBUS_TCP_UNIT_ID,1,254);
					lc640_write_int(EE_ICA_MODBUS_TCP_UNIT_ID,ICA_MODBUS_TCP_UNIT_ID);
					speed=1;
					}
				if((but==butL)||(but==butL_))
					{
					ICA_MODBUS_TCP_UNIT_ID--;
					gran(&ICA_MODBUS_TCP_UNIT_ID,1,254);
					lc640_write_int(EE_ICA_MODBUS_TCP_UNIT_ID,ICA_MODBUS_TCP_UNIT_ID);
					speed=1;
					}
				}
			}
		}


  	} 

		
but_an_end:
n_but=0;
}

//-----------------------------------------------
void watchdog_enable (void) 
{
LPC_WDT->WDTC=2000000;
LPC_WDT->WDCLKSEL=0;
LPC_WDT->WDMOD=3;
LPC_WDT->WDFEED=0xaa;
LPC_WDT->WDFEED=0x55;
}

//-----------------------------------------------
void watchdog_reset (void) 
{
LPC_WDT->WDFEED=0xaa;
LPC_WDT->WDFEED=0x55;
}


//***********************************************
//***********************************************
//***********************************************
//***********************************************
//***********************************************
void SysTick_Handler (void) 	 /* SysTick Interrupt Handler (1ms)    */
{
//sys_plazma++;
b2000Hz=1;

if(bTPS)
	{
	LPC_GPIO1->FIODIR|=(1UL<<26);
	LPC_GPIO1->FIOPIN^=(1UL<<26);
	}

if(++t0cnt4>=2)
	{
t0cnt4=0;
b1000Hz=1;

	bFF=(char)(GET_REG(LPC_GPIO0->FIOPIN, 27, 1));
	if(bFF!=bFF_) hz_out++;
	bFF_=bFF;


if(++t0cnt5>=40)
     {
     t0cnt5=0;
     b50Hz=1;
     }
     
if(++t0cnt>=10)
     {
     t0cnt=0;
     b100Hz=1;

     hz_out_cnt++;
     if(hz_out_cnt>=500)
	     {	
	     hz_out_cnt=0;
	     net_F=hz_out;
	     hz_out=0;
	     }

     if(++t0cnt0>=10)
	     {
	     t0cnt0=0;
	     b10Hz=1;
		beep_drv();
		if(main_10Hz_cnt<10000) main_10Hz_cnt++;
	     }

     if(t0cnt0==5)
	     {
		//beep_drv();
	     }

     if(++t0cnt1>=20)
	     {
	     t0cnt1=0;
	     b5Hz=1;
		if(bFL5)bFL5=0;
		else bFL5=1;     
	     }

     if(++t0cnt2>=50)
	     {
	     t0cnt2=0;
	     b2Hz=1;
		if(bFL2)bFL2=0;
		else bFL2=1;

	     }         

     if(++t0cnt3>=100)
	     {
	     t0cnt3=0;
	     b1Hz=1;
		if(main_1Hz_cnt<10000) main_1Hz_cnt++;
		if(bFL)bFL=0;
		else bFL=1;

		t0cntMin++;
		if(t0cntMin>=60)
			{
			t0cntMin=0;
			b1min=1;
			}
	     }
     }

	}


if(modbus_timeout_cnt<6)
	{
	modbus_timeout_cnt++;
	if(modbus_timeout_cnt>=6)
		{
		bMODBUS_TIMEOUT=1;
		}
	}
else if (modbus_timeout_cnt>6)
	{
	modbus_timeout_cnt=0;
	bMODBUS_TIMEOUT=0;
	}

//LPC_GPIO0->FIOCLR|=0x00000001;
  return;          



//LPC_GPIO0->FIOCLR|=0x00000001;
}


//***********************************************
__irq void timer0_interrupt(void) 
{	
/*if(BPS1_spa_leave)T0EMR_bit.EM1=0; 
else T0EMR_bit.EM1=1;
if(BPS2_spa_leave)T0EMR_bit.EM3=0; 
else T0EMR_bit.EM3=1;
T0IR = 0xff;*/
}

//===============================================
//===============================================
//===============================================
//===============================================
int main (void) 
{
char ind_reset_cnt=0;
//long i;
char mac_adr[6] = { 0x00,0x73,0x04,50,60,70 };

//i=200000;
//while(--i){};

SystemInit();

bTPS=1;

SysTick->LOAD = (SystemFrequency / 2000) - 1;
SysTick->CTRL = 0x07;

//init_timer( 0,SystemFrequency/2000/4 - 1 ); // 1ms	
//enable_timer( 0 );

//rs232_data_out_1();

bps[0]._state=bsOFF_AV_NET;
bps[1]._state=bsOFF_AV_NET;
bps[2]._state=bsOFF_AV_NET;
bps[3]._state=bsOFF_AV_NET;
bps[4]._state=bsOFF_AV_NET;
bps[5]._state=bsOFF_AV_NET;
bps[6]._state=bsOFF_AV_NET;

SET_REG(LPC_GPIO0->FIODIR, 0, 27, 1);
SET_REG(LPC_GPIO2->FIODIR, 1, 7, 1);
SET_REG(LPC_GPIO2->FIODIR, 1, 8, 1);
//LPC_GPIO1->FIODIR  |= 1<<27;                
	;
//FIO1MASK = 0x00000000;	 
//LPC_GPIO0->FIODIR  |= 1<<27;
//LPC_GPIO0->FIOSET  |= 1<<27;

///SET_REG(LPC_GPIO0->FIODIR,0,10,1); //���� ������� 
#ifdef UKU2071x
SET_REG(LPC_GPIO3->FIODIR,1,SHIFT_REL_AV_NET,1);
SET_REG(LPC_GPIO3->FIOCLR,1,SHIFT_REL_AV_NET,1);  // ���� ������ ���� ��� ���
#else 
SET_REG(LPC_GPIO3->FIODIR,1,SHIFT_REL_AV_NET,1);
SET_REG(LPC_GPIO3->FIOSET,1,SHIFT_REL_AV_NET,1);  // ���� ������ ���� ��� ���
#endif



ad7705_reset();
delay_ms(20);

ad7705_write(0x21);
ad7705_write(BIN8(1101)); 
ad7705_write(0x11);
ad7705_write(0x44);


ad7705_buff[0][1]=0x7fff;
ad7705_buff[0][2]=0x7fff;
ad7705_buff[0][3]=0x7fff;
ad7705_buff[0][4]=0x7fff;
ad7705_buff[0][5]=0x7fff;
ad7705_buff[0][6]=0x7fff;
ad7705_buff[0][7]=0x7fff;
ad7705_buff[0][8]=0x7fff;
ad7705_buff[0][9]=0x7fff;
ad7705_buff[0][10]=0x7fff;
ad7705_buff[0][11]=0x7fff;
ad7705_buff[0][12]=0x7fff;
ad7705_buff[0][13]=0x7fff;
ad7705_buff[0][14]=0x7fff;
ad7705_buff[0][15]=0x7fff;
ad7705_buff[1][1]=0x7fff;
ad7705_buff[1][2]=0x7fff;
ad7705_buff[1][3]=0x7fff;
ad7705_buff[1][4]=0x7fff;
ad7705_buff[1][5]=0x7fff;
ad7705_buff[1][6]=0x7fff;
ad7705_buff[1][7]=0x7fff;
ad7705_buff[1][8]=0x7fff;
ad7705_buff[1][9]=0x7fff;
ad7705_buff[1][10]=0x7fff;
ad7705_buff[1][11]=0x7fff;
ad7705_buff[1][12]=0x7fff;
ad7705_buff[1][13]=0x7fff;
ad7705_buff[1][14]=0x7fff;
ad7705_buff[1][15]=0x7fff;

ad7705_buff_[0]=0x7fff;
ad7705_buff_[1]=0x7fff;

/*
ad7705_reset();
delay_ms(20);

ad7705_write(0x20);
ad7705_write(BIN8(1101)); 
ad7705_write(0x10);
ad7705_write(0x44);

ad7705_reset();
delay_ms(20);  

ad7705_write(0x20);
ad7705_write(BIN8(1101)); 
ad7705_write(0x10);
ad7705_write(0x44); 

delay_ms(20); */




lcd_init();  
lcd_on();
lcd_clear();
		
///LPC_GPIO4->FIODIR |= (1<<29);           /* LEDs on PORT2 defined as Output    */
rtc_init();
///pwm_init();
ind=iMn_VD;

//snmp_plazma=15;


//#ifdef ETHISON
//mac_adr[5]=*((char*)&AUSW_MAIN_NUMBER);
//mac_adr[4]=*(((char*)&AUSW_MAIN_NUMBER)+1);
//mac_adr[3]=*(((char*)&AUSW_MAIN_NUMBER)+2);
//mem_copy (own_hw_adr, mac_adr, 6);


//if(lc640_read_int(EE_ETH_IS_ON)==1)
	//{
	bgnd_par(		"                    ",
     		"    �������������   ",
     		"      Ethernet      ",
     		"                    ");
	//bitmap_hndl();
	//lcd_out(lcd_bitmap);
	//init_TcpNet ();

	//init_ETH();
	//mem_copy (&localm[NETIF_ETH], &ip_config, sizeof(ip_config));

//	}
//#endif
//event2snmp(2);


//LPC_GPIO0->FIODIR |= (0x60000000);

adc_init();

LPC_GPIO0->FIODIR|=(1<<11);
LPC_GPIO0->FIOSET|=(1<<11);


lc640_write_int(100,134);

#ifndef MCP2515_CAN
can1_init(BITRATE62_5K25MHZ); 
can2_init(BITRATE125K25MHZ);
FullCAN_SetFilter(1,0x0e9);
FullCAN_SetFilter(0,0x18e);
#endif



memo_read();

#ifndef UKU_220 
UARTInit(0, (uint32_t)MODBUS_BAUDRATE*10UL);	/* baud rate setting */
#endif

#ifdef UKU_220 
UART_2_Init((uint32_t)MODBUS_BAUDRATE*10UL);	/* baud rate setting */
UARTInit(0, (uint32_t)MODBUS_BAUDRATE*10UL);	/* baud rate setting */
#endif


mac_adr[5]=*((char*)&AUSW_MAIN_NUMBER);
mac_adr[4]=*(((char*)&AUSW_MAIN_NUMBER)+1);
mac_adr[3]=*(((char*)&AUSW_MAIN_NUMBER)+2);
mem_copy (own_hw_adr, mac_adr, 6);

snmp_Community[0]=(char)lc640_read_int(EE_COMMUNITY); 
//if((snmp_Community[0]==0)||(snmp_Community[0]==' '))snmp_Community[0]=0;
snmp_Community[1]=(char)lc640_read_int(EE_COMMUNITY+2);
if((snmp_Community[1]==0)||(snmp_Community[1]==' '))snmp_Community[1]=0;
snmp_Community[2]=(char)lc640_read_int(EE_COMMUNITY+4);
if((snmp_Community[2]==0)||(snmp_Community[2]==' '))snmp_Community[2]=0;
snmp_Community[3]=(char)lc640_read_int(EE_COMMUNITY+6);
if((snmp_Community[3]==0)||(snmp_Community[3]==' '))snmp_Community[3]=0;
snmp_Community[4]=(char)lc640_read_int(EE_COMMUNITY+8);
if((snmp_Community[4]==0)||(snmp_Community[4]==' '))snmp_Community[4]=0;
snmp_Community[5]=(char)lc640_read_int(EE_COMMUNITY+10);
if((snmp_Community[5]==0)||(snmp_Community[5]==' '))snmp_Community[5]=0;
snmp_Community[6]=(char)lc640_read_int(EE_COMMUNITY+12);
if((snmp_Community[6]==0)||(snmp_Community[6]==' '))snmp_Community[6]=0;
snmp_Community[7]=(char)lc640_read_int(EE_COMMUNITY+14);
if((snmp_Community[7]==0)||(snmp_Community[7]==' '))snmp_Community[7]=0;
snmp_Community[8]=(char)lc640_read_int(EE_COMMUNITY+16);
if((snmp_Community[8]==0)||(snmp_Community[8]==' '))snmp_Community[8]=0;
snmp_Community[9]=0; /**/

if(lc640_read_int(EE_ETH_IS_ON)==1)
	{
	bgnd_par(		"                    ",
     		"    �������������   ",
     		"      Ethernet      ",
     		"                    ");
	bitmap_hndl();
	lcd_out(lcd_bitmap);
	init_TcpNet ();
	lcd_out(lcd_bitmap);
	init_ETH();
	//mem_copy (&localm[NETIF_ETH], &ip_config, sizeof(ip_config));
//	lcd_out(lcd_bitmap);
	}
//sys_plazma1=sys_plazma;
ind_reset_cnt=58;

if(__ee_spc_stat==spcVZ)
	{
	if(__ee_vz_cnt)
		{
		spc_stat=spcVZ;  
		vz_cnt_h=__ee_vz_cnt/60;
		vz_cnt_h_=(lc640_read_int(EE_SPC_VZ_LENGT)-__ee_vz_cnt)/60;
		if(vz_cnt_h_<0)vz_cnt_h_=0;
		vz_cnt_s_=(short)(((lc640_read_int(EE_SPC_VZ_LENGT)-__ee_vz_cnt)*60)%3600UL);

		vz_cnt_s=0;
		}
	}
else if(__ee_spc_stat==spcKE)
	{
	spc_stat=spcKE;
	spc_bat=__ee_spc_bat;
	bat[spc_bat]._zar_cnt_ke=0;
	spc_phase=__ee_spc_phase;
	}
watchdog_enable();
if((AUSW_MAIN==2400)||(AUSW_MAIN==4800)||(AUSW_MAIN==6000)||(BAT_TYPE==1))
	{
	cntrl_stat=350;
	cntrl_stat_old=350;
	}

#ifdef UKU_TELECORE2015
cntrl_stat=0;
cntrl_stat_old=0;
#endif


#ifdef UKU_TELECORE2017
{
signed short tempSS,tempSS1;
tempSS=0;
tempSS1=0;
tempSS=ULAUNCH;
if(ULAUNCH<420)tempSS=420;
if(ULAUNCH>540)tempSS=540;
tempSS-=420;
tempSS*=100;
tempSS/=12;

cntrl_stat=tempSS;
gran(&cntrl_stat,100,1000);
cntrl_stat_old=0;
//cntrl_stat=500;
}
#endif

#ifdef UKU_220_IPS_TERMOKOMPENSAT
cntrl_stat=10*PWM_START;
cntrl_stat_old=10*PWM_START;
#endif


//ind=iDeb;
//sub_ind=6;
#ifdef MCP2515_CAN
can_mcp2515_init();
#endif

#ifdef SC16IS740_UART
sc16is700_init((uint32_t)(MODBUS_BAUDRATE*10UL));
#endif
//sc16is700_init();

reload_hndl();

socket_tcp = tcp_get_socket (TCP_TYPE_SERVER, 0, 10, tcp_callback);
if (socket_tcp != 0) 
	{
    tcp_listen (socket_tcp, 502);
  	}

UOUT_=UOUT;

//lc640_write_int(EE_RELE_SET_MASK0,25);
  		
while (1)  
	{
	bTPS=0; 
     //timer_poll ();
     main_TcpNet ();

	//watchdog_reset();
#ifdef MCP2515_CAN
	if(bMCP2515_IN)
		{
		bMCP2515_IN=0;
		can_in_an1();
		}
#endif
	if(bMODBUS_TIMEOUT)
		{
		bMODBUS_TIMEOUT=0;
		//modbus_plazma++;;
		modbus_in();
		}

	if(bRXIN0) 
		{
		bRXIN0=0;
	
		uart_in0();
		} 

/*	if(bRXIN_SC16IS700) 
		{
		bRXIN_SC16IS700=0;
	
		uart_in_SC16IS700();
		}*/

	/*
	if(bRXIN1) 
		{
		bRXIN1=0;
	
		uart_in1();
		}*/ 
     if(b10000Hz)
		{
		b10000Hz=0; 
		

		}

     if(b2000Hz)
		{

		if(adc_window_cnt<200)adc_window_cnt++;

		b2000Hz=0; 
		adc_drv7();
		
		}

	if(b1000Hz)
		{
		b1000Hz=0;
		#ifdef MCP2515_CAN
		can_mcp2515_hndl();
		//mcp2515_read_status();
		#endif

		#ifdef SC16IS740_UART
		sc16is700_uart_hndl();
		#endif		
		}
	
	if(b100Hz)
		{
		b100Hz=0;

		//LPC_GPIO2->FIODIR|=(1<<7);
		//LPC_GPIO2->FIOPIN^=(1<<7);		

		if((!bRESET_INT_WDT)&&(!bRESET_EXT_WDT))but_drv();
		but_an();
		}
		 
	if(b50Hz)
		{
		b50Hz=0;
		//#ifdef MCP2515_CAN
		//net_drv_mcp2515();
		//#endif
		//#ifndef MCP2515_CAN
		net_drv();
		//#endif
		}

	if(b10Hz)
		{
		char i;

     timer_tick ();
     tick = __TRUE;

		b10Hz=0;
				
		u_necc_hndl();
		
		for(i=0;i<NUMIST;i++)bps_drv(i);
		bps_hndl();

		//inv_search();
		
		//if(NUMINV) {for(i=0;i<NUMINV;i++)inv_drv(i);}		  
		
		//nv[0]._Uii=123;


		
		ind_hndl(); 
		#ifndef SIMULATOR
		bitmap_hndl();
		if(!bRESET_EXT_WDT)
			{
			lcd_out(lcd_bitmap);
			}
		#endif
		//ad7705_drv();
		//ad7705_write(0x20);

		adc_window_cnt=0;  

		ret_hndl();  
		mess_hndl();

		cntrl_hndl();
			  

		//ret_hndl();
		ext_drv();
		avt_hndl();
		
		u_avar_hndl();
		t_sys_avar_hndl();
		overload_hndl();
		}

	if(b5Hz)
		{
		b5Hz=0;

		if(!bRESET_EXT_WDT)
			{
			ad7705_drv();
			}
		if(!bRESET_EXT_WDT)
			{
			
			}  memo_read();
		LPC_GPIO1->FIODIR|=(1UL<<26);
		matemat();
		
		rele_av_hndl();
		rele_hndl();
		if(!bRESET_EXT_WDT)avar_hndl();
		zar_superviser_drv();
		snmp_data();
		//LPC_GPIO1->FIODIR|=(1UL<<31);
		//LPC_GPIO1->FIOPIN^=(1UL<<31);

 		cntrl_hndl();
  		}

	if(b2Hz)
		{
		b2Hz=0;

				//uart_out_adr1(dig,150);
		//sc16is700_wr_buff(CS16IS7xx_THR, 20);

		//sc16is700_wr_byte(CS16IS7xx_LCR, 0x80);
		//can1_out(cnt_net_drv,cnt_net_drv,GETTM,bps[cnt_net_drv]._flags_tu,*((char*)(&bps[cnt_net_drv]._vol_u)),*((char*)((&bps[cnt_net_drv]._vol_u))+1),*((char*)(&bps[cnt_net_drv]._vol_i)),*((char*)((&bps[cnt_net_drv]._vol_i))+1));
  		}

	if(b1Hz)
		{
		b1Hz=0;
		if(!bRESET_INT_WDT)
			{
			watchdog_reset();
			}
		//can1_out_adr((char*)&net_U,21);

		samokalibr_hndl();
		num_necc_hndl();
		//zar_drv();
		ubat_old_drv();
		beep_hndl();
		avg_hndl();
		u_out_reg_hndl();
		//vz_drv();	 
		//avz_drv();
		//ke_drv();
	
		vent_hndl();

		plazma_plazma_plazma++;

		if(++ind_reset_cnt>=60)
			{
			ind_reset_cnt=0;
			lcd_init();
			lcd_on();
			lcd_clear();
			}
               
          vent_hndl();


		  
		if(t_ext_can_nd<10) t_ext_can_nd++;
		
		//if(main_1Hz_cnt<200)main_1Hz_cnt++;


		can_reset_hndl();
		npn_hndl();
/*		#ifdef UKU_220_IPS_TERMOKOMPENSAT
		if((AUSW_MAIN==22063)||(AUSW_MAIN==22023)||(AUSW_MAIN==22043))rs232_data_out_tki();
		else if(AUSW_MAIN==22010)rs232_data_out_1();
		else rs232_data_out();
		#endif */	
 
		//modbus_registers_transmit(MODBUS_ADRESS,4,0,5);
		
	/*	putchar2(0x56);
		putchar2(0x57);
		putchar2(0x58);
		putchar2(0x59);
		putchar2(0x5a);*/

		powerAntiAliasingHndl();

		//outVoltContrHndl();

		
		vent_resurs_hndl();
		

		ips_current_average_hndl();
		vd_is_work_hndl();
		}
	if(b1min)
		{
		b1min=0;

		if((tloaddisable_cmnd)&&(tloaddisable_cmnd<=10))
			{
			tloaddisable_cmnd--;
			if(!tloaddisable_cmnd)tloaddisable_cmnd=20;
			}
		if((tbatdisable_cmnd)&&(tbatdisable_cmnd<=10))
			{
			if(!tbatdisable_cmnd)tbatdisable_cmnd=20;
			}
		
		numOfForvardBps_hndl();			//��������� ����� ������� ��������� ��� ������������ ������ �����
		}

	}
}
