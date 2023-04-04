#include "25lc640.h"
#include "control.h"
#include "mess.h"
#include "gran.h"
#include "common_func.h"
#include "eeprom_map.h"
#include "avar_hndl.h"
#include "main.h"
#include "beep.h"
#include "snmp_data_file.h" 
#include "sacred_sun.h"
#include "sc16is7xx.h"
#include "modbus.h"
#include "modbus_tcp.h"
#include <LPC17xx.h>

#define KOEFPOT  105L







extern signed short u_necc,u_necc_,u_necc_up,u_necc_dn;
extern signed short main_cnt_5Hz;
extern signed short num_necc;
extern signed short num_necc_Imax;
extern signed short num_necc_Imin;
//extern char bSAME_IST_ON;
//extern signed short Unet,unet_store;
//extern char bat_cnt_to_block[2];
//extern enum  {bisON=0x0055,bisOFF=0x00aa}BAT_IS_ON[2];
extern signed mat_temper;




//***********************************************
//������
typedef struct  
	{
     unsigned int bAN:1; 
     unsigned int bAB1:1; 
     unsigned int bAB2:1;
     unsigned int bAS1:1;
     unsigned int bAS2:1;
     unsigned int bAS3:1;
     unsigned int bAS4:1;
     unsigned int bAS5:1;
     unsigned int bAS6:1;
     unsigned int bAS7:1;
     unsigned int bAS8:1;
     unsigned int bAS9:1;
     unsigned int bAS10:1;
     unsigned int bAS11:1;
     unsigned int bAS12:1;
     }avar_struct;
     
extern union 
{
avar_struct av;
int avar_stat;
}a__,a_;

//***********************************************
//���
long adc_buff[16][16];
signed short adc_buff_max[12],adc_buff_min[12]={5000,5000,5000,5000,5000,5000,5000,5000,5000,5000},unet_buff_max,unet_buff_min=5000;
char adc_self_ch_cnt,adc_ch_net;
short adc_buff_[16];
char adc_cnt,adc_cnt1,adc_ch,adc_ch_cnt;
short zero_cnt;
enum_adc_stat adc_stat=asCH;
unsigned short net_buff[32],net_buff_,net_metr_buff_[3];
char net_buff_cnt;
short ADWR,period_cnt,non_zero_cnt;
char rele_stat;
char bRELE_OUT;
signed short adc_self_ch_buff[3],adc_self_ch_disp[3];
long main_power_buffer[8],main_power_buffer_;
short adc_result;
short main_power_buffer_cnt;
short adc_gorb_cnt,adc_zero_cnt;
char adc_window_flag;
short adc_window_cnt;
short adc_net_buff_cnt;
short rele_on_cnt;


extern int mess_par0[MESS_DEEP],mess_par1[MESS_DEEP],mess_data[2];

extern signed short TBAT;
extern signed short Kunet;
extern signed short Kubat[2];
extern unsigned short ad7705_buff[2][16],ad7705_buff_[2];
extern unsigned short Kibat0[2];
extern signed short Kibat1[2];
extern signed short Ktbat[2];
//extern signed short bat_Ib[2];
short adc_buff_out_[3];
extern char kb_full_ver;
extern signed short Kuload;

signed short bat_ver_cnt=150;
extern signed short Isumm;
extern signed short Isumm_;
extern char ND_out[3];
//extern signed short tout[4];


short plazma_adc_cnt;
short plazma_sk;
extern char cntrl_plazma;

//extern const short ptr_bat_zar_cnt[2];

//***********************************************
//���������� ������������
signed char vent_stat=0;

//***********************************************
//���������� �����
signed short cntrl_stat=1200;
signed short cntrl_stat_old=1200;
signed short cntrl_stat_new;
signed short Ibmax;
unsigned char unh_cnt0,unh_cnt1,b1Hz_unh;
unsigned char	ch_cnt0,b1Hz_ch,i,iiii;
unsigned char	ch_cnt1,b1_30Hz_ch;
unsigned char	ch_cnt2,b1_10Hz_ch;
unsigned short IZMAX_;
unsigned short IZMAX_70;
unsigned short IZMAX_130;
unsigned short Ubpsmax;
unsigned short cntrl_stat_blck_cnt;
signed short cntrl_stat_buff[32];
signed short cntrl_stat_buff_;
char cntrl_stat_buff_ptr;


//***********************************************
//���������������
signed short samokalibr_cnt;



//***********************************************
//������������ �����
short avg_main_cnt=20;
signed int i_avg_max,i_avg_min,i_avg_summ,i_avg; 
signed int avg;
char bAVG;
char avg_cnt_;  
char avg_num; 
char bAVG_BLOCK;
char bAVG_DIR;

short u_out_reg_main_cnt=10;

//**********************************************
//�������� ������� �������
signed short 	main_kb_cnt;
signed short 	kb_cnt_1lev;
signed short 	kb_cnt_2lev;
char 		kb_full_ver;
char kb_start[2],kb_start_ips;
signed short ibat_ips,ibat_ips_;

//**********************************************
//������ � ������
char num_of_wrks_bps;
char bps_all_off_cnt,bps_mask_off_cnt,bps_mask_on_off_cnt;
char bps_hndl_2sec_cnt;
unsigned int bps_on_mask,bps_off_mask;
char num_necc_up,num_necc_down;
unsigned char sh_cnt0,b1Hz_sh;

//***********************************************
//�����������
enum_spc_stat spc_stat;
char spc_bat;
char spc_phase;
unsigned short vz_cnt_s,vz_cnt_s_,vz_cnt_h,vz_cnt_h_;
char bAVZ;
enum_ke_start_stat ke_start_stat;
short cnt_end_ke;
unsigned long ke_date[2];
short __ee_vz_cnt;
short __ee_spc_stat;
short __ee_spc_bat;
short __ee_spc_phase;

//***********************************************
//������
extern unsigned avar_stat;	 	//"�����������" ���� ��������� � ������ ������ ��������� � ����� �����
extern unsigned avar_ind_stat; 	//"�����������" ���� �� ������������� ��������� ��������� � ����� �����
extern unsigned avar_stat_old;
extern unsigned avar_stat_new,avar_stat_offed;
//��������� ����������
//1���  - �������� ����
//2���� - �������
//12��� - ����
//5���  - ���������
//4���� - ������� ������� �����������
//4���� - ������� ����� ��������
//1���	- �������� ��������� ����������


short cntrl_stat_blok_cnt,cntrl_stat_blok_cnt_,cntrl_stat_blok_cnt_plus[2],cntrl_stat_blok_cnt_minus[2];

//***********************************************
//����� ��������
const char sk_buff_KONTUR[4]={13,11,15,14};
const char sk_buff_RSTKM[4]={13,11,15,14};
const char sk_buff_GLONASS[4]={11,13,15,14};
const char sk_buff_3U[4]={11,13,15,14};
const char sk_buff_6U[4]={11,13,15,14};
const char sk_buff_220[4]={11,13,15,14};
const char sk_buff_TELECORE2015[4]={11,13,15,14};

char	plazma_inv[4];
char plazma_bat;
char plazma_cntrl_stat;

//***********************************************
//������� �������� ���������
char numOfForvardBps,numOfForvardBps_old;
char numOfForvardBps_minCnt;
short numOfForvardBps_hourCnt;

//***********************************************
// ������������ ������ � ������ ��������� ���������
//char bPARALLEL_NOT_ENOUG;
//char bPARALLEL_ENOUG;
//char bPARALLEL;

char cntrl_hndl_plazma;


//signed short u_avar_hndl_outu_max_cnt;
//signed short u_avar_hndl_outu_min_cnt;

//-----------------------------------------------
void ke_start(char in)
{          
ke_start_stat=(enum_ke_start_stat)0;		 

if(spc_stat==spcVZ)ke_start_stat=kssNOT_VZ;
#ifndef UKU_220_IPS_TERMOKOMPENSAT
else if(BAT_IS_ON[in]!=bisON)ke_start_stat=kssNOT_BAT;
#endif
else if(bat[in]._av&(1<<0))ke_start_stat=kssNOT_BAT_AV;
else if(bat[in]._temper_stat&(1<<1))ke_start_stat=kssNOT_BAT_AV_T;
else if(bat[in]._av&(1<<1))ke_start_stat=kssNOT_BAT_AV_ASS;
else if(bat[in]._Ib>IKB)ke_start_stat=kssNOT_BAT_ZAR;
else if(bat[in]._Ib<-IKB)ke_start_stat=kssNOT_BAT_RAZR;
else if((spc_stat==spcKE)&&(spc_bat==0))ke_start_stat=kssNOT_KE1;
else if((spc_stat==spcKE)&&(spc_bat==1))ke_start_stat=kssNOT_KE2;
else 
	{

	ke_start_stat=kssYES;

	spc_stat=spcKE;
	__ee_spc_stat=spcKE;
	lc640_write_int(EE_SPC_STAT,__ee_spc_stat);
	
	spc_bat=in;
	__ee_spc_bat=in;
	lc640_write_int(EE_SPC_BAT,__ee_spc_bat);

	bat[in]._zar_cnt_ke=0;
	lc640_write_int(ADR_EE_BAT_ZAR_CNT_KE[in],0);
	
	spc_phase=0;
	__ee_spc_phase=0;
	lc640_write_int(EE_SPC_PHASE,__ee_spc_phase);

	//ke_mem_hndl(in,lc640_read_int(ADR_EE_BAT_ZAR_CNT_KE[spc_bat]));

		{					
		signed short temp_temp;
		signed char temp;
		temp_temp=bat[in]._u_old[((bat_u_old_cnt+1)&0x07)]; 
		    
		temp=LPC_RTC->YEAR;
		gran_char(&temp,1,99);
		*((char*)(&(ke_date[0])))=temp;
			
		temp=LPC_RTC->MONTH;
		gran_char(&temp,1,12);
		*(((char*)(&(ke_date[0])))+1)=temp;
		
		temp=LPC_RTC->DOM;
		gran_char(&temp,1,31);
		*(((char*)(&(ke_date[0])))+2)=temp;			
				
		*(((char*)(&(ke_date[0])))+3)=*((char*)&temp_temp);
		lc640_write_long(EE_SPC_KE_DATE0,ke_date[0]);

		temp=LPC_RTC->HOUR;
		gran_char(&temp,0,23);
		*((char*)(&(ke_date[1])))=temp;
               
		temp=LPC_RTC->MIN;
		gran_char(&temp,0,59);
		*(((char*)(&(ke_date[1])))+1)=temp;
	          
		temp=LPC_RTC->SEC;
		gran_char(&temp,0,59);
		*(((char*)(&(ke_date[1])))+2)=temp;
			
		*(((char*)(&(ke_date[1])))+3)=*(((char*)&temp_temp)+1);
		lc640_write_long(EE_SPC_KE_DATE1,ke_date[1]);
		}

	}
}


//-----------------------------------------------
void samokalibr_init(void)
{
samokalibr_cnt=1785;
}
//-----------------------------------------------
void samokalibr_hndl(void)
{
if(++samokalibr_cnt>=1800)samokalibr_cnt=0;

if(samokalibr_cnt>=1785U)
	{
	mess_send(MESS2RELE_HNDL,PARAM_RELE_SAMOKALIBR,1,15);
	mess_send(MESS2IND_HNDL,PARAM_SAMOKALIBR,0,15);
	mess_send(MESS2MATEMAT,PARAM_SAMOKALIBR,0,15);
	} 

if(samokalibr_cnt==1799U)
	{
	if((Kibat0[0]!=ad7705_buff_[0])&&(abs(bat[0]._Ib/10)<IZMAX)) lc640_write_int(ADR_KI0BAT[0],ad7705_buff_[0]);
	if((Kibat0[1]!=ad7705_buff_[1])&&(abs(bat[0]._Ib/10)<IZMAX)) lc640_write_int(ADR_KI0BAT[1],ad7705_buff_[1]);
	
	}	 	
}



//-----------------------------------------------
void ubat_old_drv(void)
{        
bat_u_old_cnt++;
gran_ring(&bat_u_old_cnt,0,8);

bat[0]._u_old[bat_u_old_cnt]=bat[0]._Ub;
bat[1]._u_old[bat_u_old_cnt]=bat[1]._Ub;
}
/*
//-----------------------------------------------
void unet_drv(void)
{
if(net_U<UMN)
	{
	if((unet_drv_cnt<10)&&(main_1Hz_cnt>15))
		{
		unet_drv_cnt++;
		if(unet_drv_cnt>=10)
			{
			net_Ustore=net_U;
		 	avar_unet_hndl(1);
			
			}
		}
	else if(unet_drv_cnt>=10)unet_drv_cnt=10;

	if(net_U<net_Ustore) net_Ustore=net_U;	
	}

else if(net_U>UMN)
	{                 
	if(unet_drv_cnt)
		{
		unet_drv_cnt--;
		if(unet_drv_cnt<=0)
			{
			avar_unet_hndl(0);
			}
		}
	else if(unet_drv_cnt<0)unet_drv_cnt=0;
	
	}

}
*/
//-----------------------------------------------
void matemat(void)
{
//signed short temp_SS;
signed long temp_SL/*,temp_SL_*/;
char /*temp,*/i;
//signed short temp_SS;
/*
#ifdef UKU_220_IPS_TERMOKOMPENSAT
//���������� ����


	if(bps[11]._device==dNET_METR)
		{
		net_metr_buff_[0]=((signed short)bps[11]._buff[0])+(((signed short)bps[11]._buff[1])<<8);
		net_metr_buff_[1]=((signed short)bps[11]._buff[2])+(((signed short)bps[11]._buff[3])<<8);
		net_metr_buff_[2]=((signed short)bps[11]._buff[4])+(((signed short)bps[11]._buff[5])<<8);

		temp_SL=(signed long)net_metr_buff_[2];
		temp_SL*=KunetA;
		temp_SL/=6000L;
		net_Ua=(signed short)temp_SL;
	
		temp_SL=(signed long)net_metr_buff_[1];
		temp_SL*=KunetB;
		temp_SL/=6000L;
		net_Ub=(signed short)temp_SL;
	
		temp_SL=(signed long)net_metr_buff_[0];
		temp_SL*=KunetC;
		temp_SL/=6000L;
		net_Uc=(signed short)temp_SL;

		net_F3=((signed short)bps[11]._buff[6])+(((signed short)bps[11]._buff[7])<<8);

		net_U=net_Ua;
		if(net_Ub<net_U)net_U=net_Ub;
		if(net_Uc<net_U)net_U=net_Uc;
		}
	  else if(AUSW_MAIN==22033)
	{
	temp_SL=(signed long)net_buff_;
	temp_SL*=KunetA;
	temp_SL/=4000L;
	net_Ua=(signed short)temp_SL;

	temp_SL=(signed long)adc_buff_[3];
	temp_SL*=KunetB;
	temp_SL/=6000L;
	net_Ub=(signed short)temp_SL;

	temp_SL=(signed long)adc_buff_[10];
	temp_SL*=KunetC;
	temp_SL/=6000L;
	net_Uc=(signed short)temp_SL;

	net_U=net_Ua;
	if(net_Ub<net_U)net_U=net_Ub;
	if(net_Uc<net_U)net_U=net_Uc;
	}
else if((AUSW_MAIN==22063)||(AUSW_MAIN==22023)||(AUSW_MAIN==22043)||(AUSW_MAIN==22018))
	{
	temp_SL=(signed long)net_buff_;
	temp_SL*=KunetA;
	temp_SL/=40000L;
	net_Ua=(signed short)temp_SL;

	temp_SL=(signed long)adc_buff_[3];
	temp_SL*=KunetB;
	temp_SL/=6000L;
	net_Ub=(signed short)temp_SL;

	temp_SL=(signed long)adc_buff_[10];
	temp_SL*=KunetC;
	temp_SL/=6000L;
	net_Uc=(signed short)temp_SL;

	net_U=net_Ua;
	if(net_Ub<net_U)net_U=net_Ub;
	if(net_Uc<net_U)net_U=net_Uc;
	}
else	if((AUSW_MAIN==22010)||(AUSW_MAIN==22011) )
	{
	temp_SL=(signed long)net_buff_;
	temp_SL*=Kunet;
	temp_SL/=35000L;
	net_U=(signed short)temp_SL;
	
	}
else
	{
	temp_SL=(signed long)net_buff_;
	temp_SL*=Kunet;
	#ifdef _ACDC_
	temp_SL/=500L;
	#else
	temp_SL/=5000L;
	#endif
	net_U=(signed short)temp_SL;
	
	}
if(bps[11]._device!=dNET_METR) net_F3=net_F;
#endif */


//���������� �������
temp_SL=(signed long)adc_buff_[0];
temp_SL*=Kubat[0];
temp_SL/=2000L;
bat[0]._Ub=(signed short)temp_SL;
/*
#ifdef UKU_220
//���������� �������
temp_SL=(signed long)adc_buff_[0];
temp_SL*=Kubat[0];
temp_SL/=400L;
bat[0]._Ub=(signed short)temp_SL;
#endif

#ifdef UKU_220_V2
//���������� �������
temp_SL=(signed long)adc_buff_[0];
temp_SL*=Kubat[0];
temp_SL/=400L;
bat[0]._Ub=(signed short)temp_SL;
#endif */

temp_SL=(signed long)adc_buff_[4];
temp_SL*=Kubatm[0];
temp_SL/=700L;
bat[0]._Ubm=(signed short)temp_SL;
/*
#ifdef UKU_KONTUR
temp_SL=(signed long)adc_buff_[4];
temp_SL*=Kubatm[0];
temp_SL/=2000L;
bat[0]._Ubm=(signed short)temp_SL;
#endif */

temp_SL=(signed long)adc_buff_[12];
temp_SL*=Kubat[1];
temp_SL/=2000L;
bat[1]._Ub=(signed short)temp_SL;
/*
#ifdef UKU_220
temp_SL=(signed long)adc_buff_[12];
temp_SL*=Kubat[1];
temp_SL/=400L;
bat[1]._Ub=(signed short)temp_SL;
#endif

#ifdef UKU_220_V2
temp_SL=(signed long)adc_buff_[12];
temp_SL*=Kubat[1];
temp_SL/=400L;
bat[1]._Ub=(signed short)temp_SL;
#endif
*/
temp_SL=(signed long)adc_buff_[1];
temp_SL*=Kubatm[1];
temp_SL/=700L;
bat[1]._Ubm=(signed short)temp_SL;
#ifdef UKU_KONTUR
temp_SL=(signed long)adc_buff_[1];
temp_SL*=Kubatm[1];
temp_SL/=2000L;
bat[1]._Ubm=(signed short)temp_SL;
#endif

#ifdef UKU_TELECORE2015
temp_SL=(signed long)adc_buff_[2];
temp_SL*=Kubat[0];
temp_SL/=2000L;
bat[0]._Ub=(signed short)temp_SL;
#endif

/*
//���� �������
if(!mess_find_unvol(MESS2MATEMAT))
	{
	temp_SL=(signed long)ad7705_buff_[0];
	temp_SL-=(signed long)Kibat0[0];
	temp_SL*=(signed long)Kibat1[0];
	if((AUSW_MAIN==24120)||(AUSW_MAIN==24210))temp_SL/=300L;
	else temp_SL/=1000L;
	bat[0]._Ib=(signed short)temp_SL;

	temp_SL=(signed long)ad7705_buff_[1];
	temp_SL-=(signed long)Kibat0[1];
	temp_SL*=(signed long)Kibat1[1];
	if((AUSW_MAIN==24120)||(AUSW_MAIN==24210))temp_SL/=300L;
	else temp_SL/=1000L;
	bat[1]._Ib=(signed short)temp_SL;
	}
*/


//���� �������
if(!mess_find_unvol(MESS2MATEMAT))
	{
	temp_SL=(signed long)ad7705_buff_[0];
	temp_SL-=(signed long)Kibat0[0];
	temp_SL*=(signed long)Kibat1[0];
	if((AUSW_MAIN==24120)||(AUSW_MAIN==24210))temp_SL/=300L;
	else if((AUSW_MAIN==22010)||(AUSW_MAIN==22035)||(AUSW_MAIN==22033))temp_SL/=2000L;
	else temp_SL/=1000L;
	#ifdef UKU_TELECORE2015
	temp_SL/=2L;
	//temp_SL=-temp_SL;
	#endif
	//#ifdef UKU_TELECORE2017
	//temp_SL/=-2L;
	//temp_SL=-temp_SL;
	//#endif
	bat[0]._Ib=(signed short)temp_SL;

	temp_SL=(signed long)ad7705_buff_[1];
	temp_SL-=(signed long)Kibat0[1];
	temp_SL*=(signed long)Kibat1[1];
	if((AUSW_MAIN==24120)||(AUSW_MAIN==24210))temp_SL/=300L;
	else if((AUSW_MAIN==22010)||(AUSW_MAIN==22035)||(AUSW_MAIN==22033))temp_SL/=2000L;
	else temp_SL/=1000L;
	bat[1]._Ib=(signed short)temp_SL;
	}





//����������� �������
/*
#ifdef UKU_KONTUR
if((adc_buff_[6]>800)&&(adc_buff_[6]<3800))bat[0]._nd=0;
else bat[0]._nd=1;
temp_SL=(signed long)adc_buff_[6];
temp_SL*=Ktbat[0];
temp_SL/=20000L;
temp_SL-=273L;
bat[0]._Tb=(signed short)temp_SL;
#else
if((adc_buff_[6]>800)&&(adc_buff_[6]<3800))bat[0]._nd=0;
else bat[0]._nd=1;
temp_SL=(signed long)adc_buff_[6];
temp_SL*=Ktbat[0];
temp_SL/=20000L;
temp_SL-=273L;
bat[0]._Tb=(signed short)temp_SL;
#endif	*/
/*
#ifdef UKU_KONTUR
if((adc_buff_[7]>800)&&(adc_buff_[7]<3800))bat[1]._nd=0;
else bat[1]._nd=1;
temp_SL=(signed long)adc_buff_[7];
temp_SL*=Ktbat[1];
temp_SL/=20000L;
temp_SL-=273L;
bat[1]._Tb=(signed short)temp_SL;
#else
if((adc_buff_[7]>800)&&(adc_buff_[7]<3800))bat[1]._nd=0;
else bat[1]._nd=1;
temp_SL=(signed long)adc_buff_[7];
temp_SL*=Ktbat[1];
temp_SL/=20000L;
temp_SL-=273L;
bat[1]._Tb=(signed short)temp_SL;
#endif */
/*
#ifdef UKU_6U

if(NUMMAKB==2)
	{
	if(makb[0]._cnt<5)
		{
		if(makb[0]._T_nd[0]==0)
			{
			bat[0]._Tb=makb[0]._T[0];
			bat[0]._nd=0;
			}
		}

	if(makb[1]._cnt<5)
		{
		if(makb[1]._T_nd[0]==0)
			{
			bat[1]._Tb=makb[1]._T[0];
			bat[1]._nd=0;
			}
		}

	}
else if(NUMMAKB==4)
	{
	signed short temp_t;
	temp_t=-20;
	if(makb[0]._cnt<5)
		{
		if(makb[0]._T_nd[0]==0)
			{
			temp_t=makb[0]._T[0];
			bat[0]._nd=0;
			}
		}
	if(makb[1]._cnt<5)
		{
		if(makb[1]._T_nd[0]==0)
			{
			if(temp_t<makb[1]._T[0])
				{
				bat[0]._nd=0;
				temp_t=makb[1]._T[0];
				}
			}
		}
	if(temp_t!=-20)bat[0]._Tb = temp_t;

 	temp_t=-20;
	if(makb[2]._cnt<5)
		{
		if(makb[2]._T_nd[0]==0)
			{
			temp_t=makb[2]._T[0];
			bat[1]._nd=0;
			}
		}
	if(makb[3]._cnt<5)
		{
		if(makb[3]._T_nd[0]==0)
			{
			if(temp_t<makb[3]._T[0])
				{
				bat[1]._nd=0;
				temp_t=makb[3]._T[0];
				}
			}
		}
	if(temp_t!=-20)bat[1]._Tb = temp_t;
	}

#endif	*/


//���������� ��������
temp_SL=(signed long)adc_buff_[2];
temp_SL*=Kuload;
temp_SL/=2000L;
load_U=(signed short)temp_SL;
/*
#ifdef UKU_220 
//���������� ��������
temp_SL=(signed long)adc_buff_[2];
temp_SL*=Kuload;
temp_SL/=350L;
load_U=(signed short)temp_SL;
#endif

#ifdef UKU_220_V2 
//���������� ��������
temp_SL=(signed long)adc_buff_[2];
temp_SL*=Kuload;
temp_SL/=350L;
load_U=(signed short)temp_SL;
#endif
*/
#ifdef UKU_VD
//���������� ������
temp_SL=(signed long)adc_buff_[1];
temp_SL*=Kuout;
temp_SL/=500L;
out_U=(signed short)temp_SL;


//���������� ������������
temp_SL=(signed long)adc_buff_[2];
temp_SL*=Kuvd;
temp_SL/=500L;
vd_U=(signed short)temp_SL;

//���������� �����
in_U=out_U-vd_U;

//���������� ������������
temp_SL=(signed long)adc_buff_[2];
temp_SL*=Kubps;
if(AUSW_MAIN==22010)temp_SL/=400L;
else temp_SL/=500L;
bps_U=(signed short)temp_SL;

if(bps_U<100)
	{
	char i;
	for(i=0;i<NUMIST;i++)
		{
		if(bps[i]._Uin>bps_U)bps_U=bps[i]._Uin;
		}
	}

//��������� ��� ������������
temp_SL=0;
for (i=0;i<NUMIST;i++)
	{
	temp_SL+=((signed long)bps[i]._Ii);
	}
bps_I=(signed short)temp_SL;


#endif

/*
#ifdef UKU_KONTUR
//������� ������ ����������� �1(����������� �������� �������)
if((adc_buff_[5]>800)&&(adc_buff_[5]<3800))ND_EXT[0]=0;
else ND_EXT[0]=1;
temp_SL=(signed long)adc_buff_[5];
temp_SL*=Ktext[0];
temp_SL/=20000L;
temp_SL-=273L;
t_ext[0]=(signed short)temp_SL;
#else 
//������� ������ ����������� �1(����������� �������� �������)
if((adc_buff_[5]>800)&&(adc_buff_[5]<3800))ND_EXT[0]=0;
else ND_EXT[0]=1;
temp_SL=(signed long)adc_buff_[5];
temp_SL*=Ktext[0];
temp_SL/=20000L;
temp_SL-=273L;
t_ext[0]=(signed short)temp_SL;	

#endif	  */
/*
#ifdef UKU_220

//������� ������ ����������� �2(����������� ������ ���)
if((adc_buff_[3]>800)&&(adc_buff_[3]<3800))ND_EXT[1]=0;
else ND_EXT[1]=1;
temp_SL=(signed long)adc_buff_[3];
temp_SL*=Ktext[1];
temp_SL/=20000L;
temp_SL-=273L;
t_ext[1]=(signed short)temp_SL;

//������� ������ ����������� �3(����������� ������ MSAN)
if((adc_buff_[10]>800)&&(adc_buff_[10]<3800))ND_EXT[2]=0;
else ND_EXT[2]=1;
temp_SL=(signed long)adc_buff_[10];
temp_SL*=Ktext[2];
temp_SL/=20000L;
temp_SL-=273L;
t_ext[2]=(signed short)temp_SL;

#else
*/

#ifdef UKU_VD

//������� ������ ����������� 
if((adc_buff_[6]>800)&&(adc_buff_[6]<3800))ND_EXT[0]=0;
else ND_EXT[0]=1;
temp_SL=(signed long)adc_buff_[6];
temp_SL*=Ktext[0];
temp_SL/=20000L;
temp_SL-=273L;
t_ext[0]=(signed short)temp_SL;
if(rele_on_cnt<0)rele_on_cnt=0;
if(rele_on_cnt>3)rele_on_cnt=3;
t_ext[0]+=3*rele_on_cnt;
if(ND_EXT[0]==0)sys_T=t_ext[0];
else sys_T=20;

#endif

/*
//������� ������ ����������� �2(����������� ������ ���)
if((adc_buff_[6]>800)&&(adc_buff_[6]<3800))ND_EXT[1]=0;
else ND_EXT[1]=1;
temp_SL=(signed long)adc_buff_[6];
temp_SL*=Ktext[1];
temp_SL/=20000L;
temp_SL-=273L;
t_ext[1]=(signed short)temp_SL;

//������� ������ ����������� �3(����������� ������ MSAN)
if((adc_buff_[3]>800)&&(adc_buff_[3]<3800))ND_EXT[2]=0;
else ND_EXT[2]=1;
temp_SL=(signed long)adc_buff_[3];
temp_SL*=Ktext[2];
temp_SL/=20000L;
temp_SL-=273L;
t_ext[2]=(signed short)temp_SL;

#endif */


if(!bIBAT_SMKLBR)
	{
	signed long temp_SL;
	temp_SL=(signed long)ibat_metr_buff_[0];
	temp_SL-=(signed long)ibat_metr_buff_[1];
	temp_SL*=(signed long)Kibat1[0];
	temp_SL/=2000L;
	
	if(bIBAT_SMKLBR_CNT) bIBAT_SMKLBR_CNT--;

	if(ibat_metr_cnt>30)temp_SL=0;
	
	if(!bIBAT_SMKLBR_CNT)
		{
		Ib_ips_termokompensat =(signed short)temp_SL;
		out_I=Ib_ips_termokompensat;
		}
	}
else
	{
	bIBAT_SMKLBR_CNT=50;
	}

if(I_LOAD_MODE==0)
	{
	temp_SL=0;
	for(i=0;i<NUMIST;i++)
		{
		temp_SL+=(signed long)bps[i]._Ii;
		}
	Ib_ips_termokompensat=(signed short)temp_SL/10;
	
	if(vd_U==0) Ib_ips_termokompensat=0;	// �� �������� �.���������� �� 04.04.23
	out_I=Ib_ips_termokompensat;

	
	}


bat[0]._Ub=load_U;



/*
#ifdef UKU_TELECORE2015

//������� ������ ����������� �1
if((adc_buff_[7]>800)&&(adc_buff_[7]<3800))ND_EXT[0]=0;
else ND_EXT[0]=1;
temp_SL=(signed long)adc_buff_[7];
temp_SL*=Ktext[0];
temp_SL/=20000L;
temp_SL-=273L;
t_ext[0]=(signed short)temp_SL;


//������� ������ ����������� �2
if((adc_buff_[6]>800)&&(adc_buff_[6]<3800))ND_EXT[1]=0;
else ND_EXT[1]=1;
temp_SL=(signed long)adc_buff_[6];
temp_SL*=Ktext[1];
temp_SL/=20000L;
temp_SL-=273L;
t_ext[1]=(signed short)temp_SL;

#endif
*/
/*
#ifdef UKU_TELECORE2017

//������� ������ ����������� �1
if((adc_buff_[7]>800)&&(adc_buff_[7]<3800))ND_EXT[0]=0;
else ND_EXT[0]=1;
temp_SL=(signed long)adc_buff_[7];
temp_SL*=Ktext[0];
temp_SL/=20000L;
temp_SL-=273L;
t_ext[0]=(signed short)temp_SL;


//������� ������ ����������� �2
if((adc_buff_[6]>800)&&(adc_buff_[6]<3800))ND_EXT[1]=0;
else ND_EXT[1]=1;
temp_SL=(signed long)adc_buff_[6];
temp_SL*=Ktext[1];
temp_SL/=20000L;
temp_SL-=273L;
t_ext[1]=(signed short)temp_SL;

#endif
*/
//���������� �����
temp_SL=(signed long)adc_buff_ext_[0];
temp_SL*=Kunet_ext[0];
temp_SL/=4000L;
Uvv[0]=(signed short)temp_SL;
if(Uvv[0]<100) Uvv0=Uvv[0];
else Uvv0=net_U;

//���������� ���
temp_SL=(signed long)adc_buff_ext_[1];
temp_SL*=Kunet_ext[1];
temp_SL/=4000L;
Uvv[1]=(signed short)temp_SL;


//���������� ����� ����������
temp_SL=(signed long)eb2_data_short[0];
temp_SL*=Kvv_eb2[0];
temp_SL/=6000L;
Uvv_eb2[0]=(signed short)temp_SL;

temp_SL=(signed long)eb2_data_short[1];
temp_SL*=Kvv_eb2[1];
temp_SL/=6000L;
Uvv_eb2[1]=(signed short)temp_SL;

temp_SL=(signed long)eb2_data_short[2];
temp_SL*=Kvv_eb2[2];
temp_SL/=6000L;
Uvv_eb2[2]=(signed short)temp_SL;

//���������� ��� ����������
temp_SL=(signed long)eb2_data_short[3];
temp_SL*=Kpes_eb2[0];
temp_SL/=6000L;
Upes_eb2[0]=(signed short)temp_SL;

temp_SL=(signed long)eb2_data_short[4];
temp_SL*=Kpes_eb2[1];
temp_SL/=6000L;
Upes_eb2[1]=(signed short)temp_SL;

temp_SL=(signed long)eb2_data_short[5];
temp_SL*=Kpes_eb2[2];
temp_SL/=6000L;
Upes_eb2[2]=(signed short)temp_SL;

//���������� ����������� �����

ibt._T[0]=t_ext[1]+273;
ibt._T[1]=t_ext[2]+273;

ibt._nd[0]=ND_EXT[1];
ibt._nd[1]=ND_EXT[2];

#ifndef UKU_TELECORE2015
if((ibt._nd[0]==0) &&  (ibt._nd[1]==0))
	{
	t_box=((ibt._T[0]+ibt._T[1])/2)-273;
	}
else if((ibt._nd[0]==1) &&  (ibt._nd[1]==0))
	{
	t_box=ibt._T[1]-273;
	}
else if((ibt._nd[0]==0) &&  (ibt._nd[1]==1))
	{
	t_box=ibt._T[0]-273;
	}
else if((ibt._nd[0]==1) &&  (ibt._nd[1]==1))
	{
	if(t_ext_can_nd<5)t_box= t_ext_can;
	else t_box=20;
	}
#endif


#ifndef TELECORE
if((BAT_IS_ON[0]==bisON)&&(bat[0]._Ub>200)) Ibmax=bat[0]._Ib;
if((BAT_IS_ON[1]==bisON)&&(bat[1]._Ub>200)&&(bat[1]._Ib>bat[0]._Ib)) Ibmax=bat[1]._Ib;
#endif

#ifdef TELECORE
Ibmax=0;
/*
if((NUMBAT_TELECORE>0)&&(lakb[0]._communicationFullErrorStat==0)&&(lakb[0]._ch_curr/10>Ibmax))Ibmax=lakb[0]._ch_curr/10;
if((NUMBAT_TELECORE>1)&&(lakb[1]._communicationFullErrorStat==0)&&(lakb[1]._ch_curr/10>Ibmax))Ibmax=lakb[1]._ch_curr/10;
if((NUMBAT_TELECORE>2)&&(lakb[2]._communicationFullErrorStat==0)&&(lakb[2]._ch_curr/10>Ibmax))Ibmax=lakb[2]._ch_curr/10;
*/
if((NUMBAT_TELECORE>0)&&(bat[0]._Ib/10>Ibmax))Ibmax=bat[0]._Ib/10;
if((NUMBAT_TELECORE>1)&&(bat[1]._Ib/10>Ibmax))Ibmax=bat[1]._Ib/10;
//if((BAT_IS_ON[0]==bisON)&&(bat[0]._Ub>200)) Ibmax=bat[0]._Ib/1;
//if((BAT_IS_ON[1]==bisON)&&(bat[1]._Ub>200)&&(bat[1]._Ib>bat[0]._Ib)) Ibmax=bat[1]._Ib;
#endif
//Ibmax=bat[0]._Ib;
//if((AUSW_MAIN==22063)||(AUSW_MAIN==22023)||(AUSW_MAIN==22043))Ibmax=Ib_ips_termokompensat;

#ifdef UKU_TELECORE2017
Ibmax=0;
if((NUMBAT_TELECORE>0)&&(bat[0]._Ib/10>Ibmax))Ibmax=bat[0]._Ib/10;
if((NUMBAT_TELECORE>1)&&(bat[1]._Ib/10>Ibmax))Ibmax=bat[1]._Ib/10;
#endif

#ifdef UKU_220_IPS_TERMOKOMPENSAT
Ibmax=Ib_ips_termokompensat;
#endif
for(i=0;i<NUMIST;i++)
	{
	if(bps[i]._cnt<25)
     	{
     	bps[i]._Ii=bps[i]._buff[0]+(bps[i]._buff[1]*256);
     	bps[i]._Uin=bps[i]._buff[2]+(bps[i]._buff[3]*256);
     	bps[i]._Uii=bps[i]._buff[4]+(bps[i]._buff[5]*256);
     	bps[i]._Ti=(signed)(bps[i]._buff[6]);
     	bps[i]._adr_ee=bps[i]._buff[7];
     	bps[i]._flags_tm=bps[i]._buff[8];
	    //bps[i]._rotor=bps[i]._buff[10]+(bps[i]._buff[11]*256); 
		bps[i]._Uisum=bps[i]._buff[10]+(bps[i]._buff[11]*256); 
		bps[i].debug_info_to_uku0=bps[i]._buff[12]+(bps[i]._buff[13]*256); 
		bps[i].debug_info_to_uku1=bps[i]._buff[14]+(bps[i]._buff[15]*256); 
		bps[i].debug_info_to_uku2=bps[i]._buff[16]+(bps[i]._buff[17]*256);   
     	} 
	else 
     	{
     	bps[i]._Uii=0; 
     	bps[i]._Ii=0;
     	bps[i]._Uin=0;
     	bps[i]._Ti=0;
     	bps[i]._flags_tm=0; 
	     //bps[i]._rotor=0;
		bps[i]._Uisum=0; 
		bps[i].debug_info_to_uku0=0; 
		bps[i].debug_info_to_uku1=0; 
		bps[i].debug_info_to_uku2=0;   
     	}
     
     }

load_I=0;
#ifdef TELECORE

/*for(i=0;i<NUMBAT_TELECORE;i++)
	{
	load_I-=lakb[i]._ch_curr/10;
	}*/
load_I=-(bat[0]._Ib/10)-(bat[1]._Ib/10);
#elif UKU_TELECORE2017
load_I=-(bat[0]._Ib/10)-(bat[1]._Ib/10);
#else
load_I=-(bat[0]._Ib/10)-(bat[1]._Ib/10);
#endif
Isumm=0;

for(i=0;i<NUMIST;i++)
     {
     if(bps[i]._cnt<5)Isumm+=bps[i]._Ii;
     }  
     
load_I=load_I+Isumm;
if(load_I<0)load_I=0;

#ifdef UKU_220_IPS_TERMOKOMPENSAT
load_I=0;

Isumm=0;

for(i=0;i<NUMIST;i++)
     {
     if(bps[i]._cnt<5)Isumm+=bps[i]._Ii;
     }  
     
load_I=load_I+Isumm;
if(load_I<0)load_I=0;

#endif



#ifdef UKU_GLONASS
inv[0]._Uio=6;
if (NUMINV)
	{
	for(i=0;i<NUMINV;i++)
		{
		if(bps[i+first_inv_slot]._cnt<25)
     		{
     		inv[i]._Ii=bps[i+first_inv_slot]._buff[0]+(bps[i+first_inv_slot]._buff[1]*256);
     		inv[i]._Uin=bps[i+first_inv_slot]._buff[2]+(bps[i+first_inv_slot]._buff[3]*256);
     		inv[i]._Uio=bps[i+first_inv_slot]._buff[4]+(bps[i+first_inv_slot]._buff[5]*256);
     		inv[i]._Ti=(signed)(bps[i+first_inv_slot]._buff[6]);
     		inv[i]._flags_tm=bps[i+first_inv_slot]._buff[8];
	    	//	inv[i]._rotor=bps[i+first_inv_slot]._buff[10]+(bps[i+first_inv_slot]._buff[11]*256);
			inv[i]._cnt=0;    
     		} 
		else 
     		{
     		inv[i]._Uio=0; 
     		inv[i]._Ii=0;
     		inv[i]._Uin=0;
     		inv[i]._Ti=0;
     		inv[i]._flags_tm=0; 
//	     	inv[i]._rotor0;
			inv[i]._cnt=25;    
     		}
     	}
   	}
#endif

#ifndef UKU_GLONASS
if (NUMINV)
	{
	for(i=0;i<NUMINV;i++)
		{
		if(bps[i+20]._cnt<25)
     		{
     		inv[i]._Ii=bps[i+20]._buff[0]+(bps[i+20]._buff[1]*256);
     		inv[i]._Pio=bps[i+20]._buff[2]+(bps[i+20]._buff[3]*256);
     		inv[i]._Uio=bps[i+20]._buff[4]+(bps[i+20]._buff[5]*256);
     		inv[i]._Ti=(signed)(bps[i+20]._buff[6]);
     		inv[i]._flags_tm=bps[i+20]._buff[7];
     		inv[i]._Uin=bps[i+20]._buff[8]+(bps[i+20]._buff[9]*256);
     		inv[i]._Uil=bps[i+20]._buff[10]+(bps[i+20]._buff[11]*256);
			inv[i]._cnt=0;
			inv[i]._Uoutmin=bps[i+20]._buff[12]; 
			inv[i]._Uoutmax=bps[i+20]._buff[13]; 
			inv[i]._Pnom=bps[i+20]._buff[14]; 
			inv[i]._net_contr_en=bps[i+20]._buff[15];
			inv[i]._pwm_en=bps[i+20]._buff[16];  
			inv[i]._phase_mode=bps[i+20]._buff[17];  
     		} 
		else 
     		{
      		inv[i]._Ii=0;
			inv[i]._Pio=0;
			inv[i]._Uio=0;
     		inv[i]._Ti=0;
     		inv[i]._flags_tm=0; 
     		inv[i]._Uil=0;
     		inv[i]._Uin=0;
			inv[i]._cnt=25; 
			inv[i]._Uoutmin=0; 
			inv[i]._Uoutmax=0; 
			inv[i]._Pnom=0; 
			inv[i]._net_contr_en=0;
			inv[i]._pwm_en=0;   
			   
     		}
     	}
   	}
#endif

#ifdef GLADKOV
inv[0]._Ii=bps[4]._buff[0]+(bps[4]._buff[1]*256);
inv[0]._Pio=bps[4]._buff[2]+(bps[4]._buff[3]*256);
inv[0]._Uio=bps[4]._buff[4]+(bps[4]._buff[5]*256);
inv[0]._Ti=(signed)(bps[4]._buff[6]);
inv[0]._flags_tm=bps[4]._buff[7];
inv[0]._Uin=bps[4]._buff[8]+(bps[4]._buff[9]*256);
inv[0]._Uil=bps[4]._buff[10]+(bps[4]._buff[11]*256);
inv[0]._cnt=0;    

inv[1]._Ii=bps[21]._buff[0]+(bps[21]._buff[1]*256);
inv[1]._Pio=bps[21]._buff[2]+(bps[21]._buff[3]*256);
inv[1]._Uio=bps[21]._buff[4]+(bps[21]._buff[5]*256);
inv[1]._Ti=(signed)(bps[21]._buff[6]);
inv[1]._flags_tm=bps[21]._buff[7];
inv[1]._Uin=bps[21]._buff[8]+(bps[21]._buff[9]*256);
inv[1]._Uil=bps[21]._buff[10]+(bps[21]._buff[11]*256);
inv[1]._cnt=0;    
#endif

/*
if((BAT_IS_ON[0]==bisON)&&(BAT_TYPE==1))
	{
	lakb[0]._battCommState=0;
	if(lakb[0]._cnt>10)lakb[0]._battCommState=2;
	else if(lakb[0]._bRS485ERR==1)lakb[0]._battCommState=1;
	
	if(lakb[0]._battCommState==0)
		{	
		bat[0]._Ub=(signed short)((lakb[0]._tot_bat_volt+5)/10);
		bat[0]._Ib=(signed short)lakb[0]._ch_curr;
		if(lakb[0]._dsch_curr) bat[0]._Ib=(signed short) (-lakb[0]._dsch_curr);
		bat[0]._Tb=(signed short)lakb[0]._max_cell_temp;
		}
	}
*/

#ifdef UKU_TELECORE2015

	if(BAT_TYPE==2)
		{
		lakb[0]._ch_curr/*temp_SS*/=((ascii2halFhex(liBatteryInBuff[113]))<<12)+
					 		((ascii2halFhex(liBatteryInBuff[114]))<<8)+
							((ascii2halFhex(liBatteryInBuff[115]))<<4)+
							((ascii2halFhex(liBatteryInBuff[116])));
		
		/*if(temp_SS&0x8000)		lakb[0]._ch_curr=~temp_SS;
		else 				lakb[0]._ch_curr=temp_SS;*/
	
		lakb[0]._tot_bat_volt=	(unsigned short)(((ascii2halFhex(liBatteryInBuff[117]))<<12)+
							((ascii2halFhex(liBatteryInBuff[118]))<<8)+
							((ascii2halFhex(liBatteryInBuff[119]))<<4)+
							((ascii2halFhex(liBatteryInBuff[120]))))/100;
	
		lakb[0]._max_cell_temp= 	(((ascii2halFhex(liBatteryInBuff[93]))<<12)+
							((ascii2halFhex(liBatteryInBuff[94]))<<8)+
							((ascii2halFhex(liBatteryInBuff[95]))<<4)+
							((ascii2halFhex(liBatteryInBuff[96]))))/10-273;
	
		lakb[0]._s_o_c_abs=		(unsigned short)((ascii2halFhex(liBatteryInBuff[121]))<<12)+
							((ascii2halFhex(liBatteryInBuff[122]))<<8)+
							((ascii2halFhex(liBatteryInBuff[123]))<<4)+
							((ascii2halFhex(liBatteryInBuff[124])));
	
		lakb[0]._rat_cap=		(unsigned short)((ascii2halFhex(liBatteryInBuff[127]))<<12)+
							((ascii2halFhex(liBatteryInBuff[128]))<<8)+
							((ascii2halFhex(liBatteryInBuff[129]))<<4)+
							((ascii2halFhex(liBatteryInBuff[130])));
	
		lakb[0]._s_o_c=		lakb[0]._s_o_c_abs/(lakb[0]._rat_cap/100);
	
	
	/*	lakb[0]._rat_cap= (lakb_damp[i][13]*256)+ lakb_damp[i][14];
		lakb[0]._max_cell_volt= (lakb_damp[i][0]*256)+ lakb_damp[i][1];
		lakb[0]._min_cell_volt= (lakb_damp[i][2]*256)+ lakb_damp[i][3];
		lakb[0]._max_cell_temp= lakb_damp[i][4];
		lakb[0]._min_cell_temp= lakb_damp[i][5];
		lakb[0]._tot_bat_volt= (lakb_damp[i][6]*256)+ lakb_damp[i][7];
		lakb[0]._ch_curr= (lakb_damp[i][8]*256)+ lakb_damp[i][8];
		lakb[0]._dsch_curr= (lakb_damp[i][10]*256)+ lakb_damp[i][11];
		lakb[0]._s_o_c= lakb_damp[i][12];
		lakb[0]._r_b_t= lakb_damp[i][15];
		lakb[0]._c_c_l_v= (lakb_damp[i][16]*256)+ lakb_damp[i][17];
		lakb[0]._s_o_h= lakb_damp[i][18];
	
		if(lakb[i]._rat_cap==0)
			{
			if(lakb[i]._isOnCnt)
				{
				lakb[i]._isOnCnt--;
				if(lakb[i]._isOnCnt==0)
					{
					if(lakb[i]._battIsOn!=0) bLAKB_KONF_CH=1;
					}
				}
			}
		else 
			{
			if(lakb[i]._isOnCnt<50)
				{
				lakb[i]._isOnCnt++;
				if(lakb[i]._isOnCnt==50)
					{
					if(lakb[i]._battIsOn!=1) bLAKB_KONF_CH=1;
					}
				}
			}
		gran(&lakb[i]._isOnCnt,0,50);*/
		}
	else if(BAT_TYPE==3)
		{
		//short numOfPacks;
		//short numOfCells, numOfTemperCells, baseOfData;
		
		#ifndef UKU_TELECORE2016
		numOfCells=((ascii2halFhex(liBatteryInBuff[17]))<<4)+((ascii2halFhex(liBatteryInBuff[18])));
		numOfTemperCells=((ascii2halFhex(liBatteryInBuff[17+(numOfCells*4)+2]))<<4)+((ascii2halFhex(liBatteryInBuff[18+(numOfCells*4)+2])));
		numOfPacks=((ascii2halFhex(liBatteryInBuff[15]))<<4)+((ascii2halFhex(liBatteryInBuff[16])));
		if(numOfPacks)numOfPacks-=1;
		if((numOfPacks<0)||(numOfPacks>NUMBAT_TELECORE))numOfPacks=0;
		plazma_numOfCells=numOfCells;
		plazma_numOfTemperCells=numOfTemperCells;
		plazma_numOfPacks=numOfPacks;


		baseOfData=16+(numOfCells*4)+2+(numOfTemperCells*4)+2;

		lakb[numOfPacks]._ch_curr=(signed short)(
							((ascii2halFhex(liBatteryInBuff[1+baseOfData]))<<12)+
							((ascii2halFhex(liBatteryInBuff[2+baseOfData]))<<8)+
							((ascii2halFhex(liBatteryInBuff[3+baseOfData]))<<4)+
							((ascii2halFhex(liBatteryInBuff[4+baseOfData])))
							);	  

		lakb[numOfPacks]._tot_bat_volt=(signed short)(
							((ascii2halFhex(liBatteryInBuff[1+baseOfData+4]))<<12)+
							((ascii2halFhex(liBatteryInBuff[2+baseOfData+4]))<<8)+
							((ascii2halFhex(liBatteryInBuff[3+baseOfData+4]))<<4)+
							((ascii2halFhex(liBatteryInBuff[4+baseOfData+4])))
							)/10;

		lakb[numOfPacks]._max_cell_temp=(signed short)(
							((ascii2halFhex(liBatteryInBuff[1+baseOfData-4]))<<12)+
							((ascii2halFhex(liBatteryInBuff[2+baseOfData-4]))<<8)+
							((ascii2halFhex(liBatteryInBuff[3+baseOfData-4]))<<4)+
							((ascii2halFhex(liBatteryInBuff[4+baseOfData-4])))
							)-2730;

		lakb[numOfPacks]._s_o_c=(signed short)(
							((ascii2halFhex(liBatteryInBuff[1+baseOfData+8]))<<12)+
							((ascii2halFhex(liBatteryInBuff[2+baseOfData+8]))<<8)+
							((ascii2halFhex(liBatteryInBuff[3+baseOfData+8]))<<4)+
							((ascii2halFhex(liBatteryInBuff[4+baseOfData+8])))
							)/10;

		lakb[numOfPacks]._s_o_h=(signed short)(
							((ascii2halFhex(liBatteryInBuff[1+baseOfData+14]))<<12)+
							((ascii2halFhex(liBatteryInBuff[2+baseOfData+14]))<<8)+
							((ascii2halFhex(liBatteryInBuff[3+baseOfData+14]))<<4)+
							((ascii2halFhex(liBatteryInBuff[4+baseOfData+14])))
							)/10;
		#endif
		
		#ifdef UKU_TELECORE2016
		{
		char i;
		
		for(i=0;i<NUMBAT_TELECORE;i++)
			{
			lakb[i]._s_o_c_percent= (signed short)(((unsigned long)lakb[i]._s_o_c*100UL)/(unsigned long)lakb[i]._s_o_h);
			}
		}
		#endif
		
										  
		}
	
if(sacredSunSilentCnt<3) 
	{
    	bat[0]._Ub=lakb[0]._tot_bat_volt;
    	bat[0]._Tb=lakb[0]._max_cell_temp;
   	//bat[0]._Ib=lakb[0]._ch_curr/10;
	}
else 
	{
    	//bat[0]._Ub=0;
    	//bat[0]._Tb=0;
   	//bat[0]._Ib=0;
	}

if(BAT_TYPE==1)
	{
	char i;
	for(i=0;i<1;i++)
		{
		lakb[i]._rat_cap= (lakb_damp[i][13]*256)+ lakb_damp[i][14];
		lakb[i]._max_cell_volt= (lakb_damp[i][0]*256)+ lakb_damp[i][1];
		lakb[i]._min_cell_volt= (lakb_damp[i][2]*256)+ lakb_damp[i][3];
		lakb[i]._max_cell_temp= lakb_damp[i][4];
		lakb[i]._min_cell_temp= lakb_damp[i][5];
		lakb[i]._tot_bat_volt= (lakb_damp[i][6]*256)+ lakb_damp[i][7];
		lakb[i]._ch_curr= (lakb_damp[i][8]*256)+ lakb_damp[i][9];
		lakb[i]._dsch_curr= (lakb_damp[i][10]*256)+ lakb_damp[i][11];
		lakb[i]._s_o_c= lakb_damp[i][12];
		lakb[i]._r_b_t= lakb_damp[i][15];
		lakb[i]._c_c_l_v= (lakb_damp[i][16]*256)+ lakb_damp[i][17];
		lakb[i]._s_o_h= lakb_damp[i][18];
		lakb[i]._flags1= lakb_damp[i][34];
		lakb[i]._flags2= lakb_damp[i][35];
		lakb[i]._b_p_ser_num= lakb_damp[i][36];

/*		if(lakb[i]._rat_cap==0)
			{
			if(lakb[i]._isOnCnt)
				{
				lakb[i]._isOnCnt--;
				if(lakb[i]._isOnCnt==0)
					{
					if(lakb[i]._battIsOn!=0) bLAKB_KONF_CH=1;
					}
				}
			}
		else 
			{
			if(lakb[i]._isOnCnt<50)
				{
				lakb[i]._isOnCnt++;
				if(lakb[i]._isOnCnt==50)
					{
					if(lakb[i]._battIsOn!=1) bLAKB_KONF_CH=1;
					}
				}
			} */
		gran(&lakb[i]._isOnCnt,0,50);
	 	}

	if(lakb_damp[0][41]==100)
		{
		li_bat._485Error=1;
		}
	if(lakb_damp[0][41]==0)
		{
		//if(bRS485ERR)bLAKB_KONF_CH=1;
		li_bat._485Error=0;
		}
	li_bat._485ErrorCnt=lakb_damp[0][41];


	}


#endif

		#ifdef UKU_TELECORE2017
		{
		char i;
		
		for(i=0;i<NUMBAT_TELECORE;i++)
			{
			lakb[i]._s_o_c_percent= (signed short)(((unsigned long)lakb[i]._s_o_c*100UL)/(unsigned long)lakb[i]._s_o_h);
			}
		}
		#endif
#ifdef UKU_TELECORE2015
//���������� ���������� ������ �������
//TODO �������� ��� ���� ������� ��� ��������� � ��� ������� �������
li_bat._batStat=bsOK;
if(BAT_TYPE==1) //COSLIGHT
	{
	if(li_bat._batStat==bsOK)
		{
		li_bat._Ub=lakb[0]._tot_bat_volt/10;

		if(lakb[0]._ch_curr)li_bat._Ib=lakb[0]._ch_curr/10;
		else if(lakb[0]._dsch_curr) li_bat._Ib=bat[0]._Ib/10;//lakb[0]._dsch_curr/10;
	
		li_bat._ratCap=lakb[0]._rat_cap/100;
		li_bat._soc=lakb[0]._s_o_c;
		li_bat._soh=lakb[0]._s_o_h;
		li_bat._cclv=lakb[0]._c_c_l_v/10;
		li_bat._Tb=lakb[0]._max_cell_temp;
		li_bat._rbt=lakb[0]._r_b_t;
		}
	else 
		{
		li_bat._Ub=bat[0]._Ub;
		li_bat._Ib=bat[0]._Ib/10;
		li_bat._Tb=bat[0]._Tb;
		}

	if((li_bat._485Error)||(li_bat._canError))
		{
		li_bat._batStat=bsOFF;
		}
	else li_bat._batStat=bsOK;
	}
else if(BAT_TYPE==2) //SACRED SUN
	{
	}
else if(BAT_TYPE==3) //ZTT
	{
	if(li_bat._batStat==bsOK)
		{

		}
	}
#endif


/*
if((BAT_IS_ON[0]==bisON)&&(BAT_TYPE[0]==1)&&(BAT_LINK==0))
	{


	if(bat_drv_rx_buff[13]<=0x39)bbb[0]=bat_drv_rx_buff[13]-0x30;
	else bbb[0]=bat_drv_rx_buff[13]-55;
	if(bat_drv_rx_buff[14]<=0x39)bbb[1]=bat_drv_rx_buff[14]-0x30;
	else bbb[1]=bat_drv_rx_buff[14]-55;
	if(bat_drv_rx_buff[15]<=0x39)bbb[2]=bat_drv_rx_buff[15]-0x30;
	else bbb[2]=bat_drv_rx_buff[15]-55;
	if(bat_drv_rx_buff[16]<=0x39)bbb[3]=bat_drv_rx_buff[16]-0x30;
	else bbb[3]=bat_drv_rx_buff[16]-55;

	tempSS=0;
	tempSS=((bbb[0]*4096)+(bbb[1]*256)+(bbb[2]*16)+bbb[3]);

	bat[0]._max_cell_volt=(tempSS+5)/10;

	if(bat_drv_rx_buff[17]<=0x39)bbb[0]=bat_drv_rx_buff[17]-0x30;
	else bbb[0]=bat_drv_rx_buff[17]-55;
	if(bat_drv_rx_buff[18]<=0x39)bbb[1]=bat_drv_rx_buff[18]-0x30;
	else bbb[1]=bat_drv_rx_buff[18]-55;
	if(bat_drv_rx_buff[19]<=0x39)bbb[2]=bat_drv_rx_buff[19]-0x30;
	else bbb[2]=bat_drv_rx_buff[19]-55;
	if(bat_drv_rx_buff[20]<=0x39)bbb[3]=bat_drv_rx_buff[20]-0x30;
	else bbb[3]=bat_drv_rx_buff[20]-55;

	tempSS=0;
	tempSS=((bbb[0]*4096)+(bbb[1]*256)+(bbb[2]*16)+bbb[3]);

	bat[0]._min_cell_volt=(tempSS+5)/10;


	}*/



}



//-----------------------------------------------
void adc_init(void)
{

SET_REG(LPC_PINCON->PINSEL1,1,(25-16)*2,2);
SET_REG(LPC_PINCON->PINSEL1,1,(24-16)*2,2);
SET_REG(LPC_PINCON->PINSEL1,1,(23-16)*2,2);


SET_REG(LPC_PINCON->PINMODE1,2,(25-16)*2,2);
SET_REG(LPC_PINCON->PINMODE1,2,(24-16)*2,2);
SET_REG(LPC_PINCON->PINMODE1,2,(23-16)*2,2);

SET_REG(LPC_ADC->ADCR,0,24,3);

SET_REG(LPC_ADC->ADCR,1,21,1);
SET_REG(LPC_ADC->ADCR,0,16,1);
SET_REG(LPC_ADC->ADCR,1,8,8);

//SET_REG(LPC_GPIO0->FIODIR,7,5,3);
//SET_REG(LPC_GPIO0->FIOPIN,4,5,3);
	
	/*if(adc_ch<=7)*///SET_REG(LPC_ADC->ADCR,1<<5,0,8);
     /*else if(adc_ch==8) SET_REG(LPC_ADC->ADCR,1<<2,0,8);
     else SET_REG(LPC_ADC->ADCR,1<<4,0,8);*/

LPC_ADC->ADINTEN     =  (1<< 8);      /* global enable interrupt            */

NVIC_EnableIRQ(ADC_IRQn);             /* enable ADC Interrupt               */


}

//-----------------------------------------------
void adc_drv7(void) //(U���� - ���������)
{
//int temp_S;
//char i;
//signed short temp_SS;

adc_self_ch_disp[0]=abs_pal(adc_self_ch_buff[1]-adc_self_ch_buff[0]);//adc_self_ch_buff[0]&0x0f80;
adc_self_ch_disp[1]=abs_pal(adc_self_ch_buff[2]-adc_self_ch_buff[1]);//adc_self_ch_buff[1]&0x0f80;
adc_self_ch_disp[2]=abs_pal(adc_self_ch_buff[2]-adc_self_ch_buff[0]);//adc_self_ch_buff[2]&0x0f80;

//adc_self_ch_disp[0]=adc_self_ch_buff[0]&0x0ff0;
//adc_self_ch_disp[1]=adc_self_ch_buff[1]&0x0ff0;
//adc_self_ch_disp[2]=adc_self_ch_buff[2]&0x0ff0;


if(adc_self_ch_disp[2]<300)//==adc_self_ch_disp[2])
	{
	adc_result=adc_self_ch_buff[2];
	} 
else if(adc_self_ch_disp[1]<300)//==adc_self_ch_disp[2])
	{
	adc_result=adc_self_ch_buff[1];
	}
else if(adc_self_ch_disp[0]<300)//==adc_self_ch_disp[1])
	{
	adc_result=adc_self_ch_buff[0];
	}
    //adc_result=92;

if(adc_ch_net)
	{

	main_power_buffer[0]+=(long)(adc_result);
	main_power_buffer[1]+=(long)(adc_result);
	main_power_buffer[2]+=(long)(adc_result);
	main_power_buffer[3]+=(long)(adc_result);

	adc_net_buff_cnt++;
	if(adc_net_buff_cnt>=0x1000)
		{
		adc_net_buff_cnt=0;
		}
	if((adc_net_buff_cnt&0x03ff)==0)
		{
		#ifdef UKU_220
		net_buff_=(short)((main_power_buffer[adc_net_buff_cnt>>10])>>11);
		#else
		#ifdef UKU_220_V2
		net_buff_=(short)((main_power_buffer[adc_net_buff_cnt>>10])>>11);
		#else
		net_buff_=(short)((main_power_buffer[adc_net_buff_cnt>>10])>>8);
		#endif
		#endif
		main_power_buffer[adc_net_buff_cnt>>10]=0;
		}


	} 
else if(!adc_ch_net)
	{
	adc_buff[adc_ch][adc_ch_cnt]=(long)adc_result;
	
	if((adc_ch_cnt&0x03)==0)
		{
		long temp_L;
		char i;
		temp_L=0;
		for(i=0;i<16;i++)
			{
			temp_L+=adc_buff[adc_ch][i];
			}
		adc_buff_[adc_ch]= (short)(temp_L>>4);

		//adc_buff_[3]=346;
		}
	if(++adc_ch>=16) 
		{
		adc_ch=0;
		adc_ch_cnt++;
		if(adc_ch_cnt>=16)adc_ch_cnt=0;
		}
	}

//adc_buff[adc_ch][adc_cnt1]=(adc_self_ch_buff[2]+adc_self_ch_buff[1])/2;

//if(adc_buff[adc_ch][adc_cnt1]<adc_buff_min[adc_ch])adc_buff_min[adc_ch]=adc_buff[adc_ch][adc_cnt1];
//if(adc_buff[adc_ch][adc_cnt1]>adc_buff_max[adc_ch])adc_buff_max[adc_ch]=adc_buff[adc_ch][adc_cnt1];
/*
	{
	if((adc_cnt1&0x03)==0)
		{
		temp_S=0;
		for(i=0;i<16;i++)
			{
			temp_S+=adc_buff[adc_ch][i];
			} 
         	adc_buff_[adc_ch]=temp_S>>4;
          }
	}*/


		  

adc_self_ch_cnt=0;

adc_ch_net++;
adc_ch_net&=1;

//SET_REG(LPC_GPIO0->FIODIR,7,5,3);
//SET_REG(LPC_GPIO0->FIOPIN,adc_ch,5,3);

if(adc_ch_net)
	{
	//LPC_GPIO2->FIODIR|=(1<<7);
	//LPC_GPIO2->FIOPIN|=(1<<7);
	SET_REG(LPC_ADC->ADCR,1<<2,0,8);
	}
else
	{
	//LPC_GPIO2->FIODIR|=(1<<7);
	//LPC_GPIO2->FIOPIN&=~(1<<7);
	if(!(adc_ch&(1<<3)))SET_REG(LPC_ADC->ADCR,1<<0,0,8);
	else 			SET_REG(LPC_ADC->ADCR,1<<1,0,8);


	SET_REG(LPC_GPIO0->FIODIR,1,28,1);
	SET_REG(LPC_GPIO1->FIODIR,1,30,1);
	SET_REG(LPC_GPIO3->FIODIR,1,26,1);

	if(!(adc_ch&(1<<0)))SET_REG(LPC_GPIO0->FIOPIN,0,28,1);
	else 			SET_REG(LPC_GPIO0->FIOPIN,1,28,1);

	if(!(adc_ch&(1<<1)))SET_REG(LPC_GPIO1->FIOPIN,0,30,1);
	else 			SET_REG(LPC_GPIO1->FIOPIN,1,30,1);

	if(!(adc_ch&(1<<2)))SET_REG(LPC_GPIO3->FIOPIN,0,26,1);
	else 			SET_REG(LPC_GPIO3->FIOPIN,1,26,1);
	}
	



LPC_ADC->ADCR |=  (1<<24);

}

//-----------------------------------------------
void adc_drv6(void) //(� �������� �������� �����)
{
//int temp_S;
//char i;
//signed short temp_SS;

adc_self_ch_disp[0]=abs_pal(adc_self_ch_buff[1]-adc_self_ch_buff[0]);//adc_self_ch_buff[0]&0x0f80;
adc_self_ch_disp[1]=abs_pal(adc_self_ch_buff[2]-adc_self_ch_buff[1]);//adc_self_ch_buff[1]&0x0f80;
adc_self_ch_disp[2]=abs_pal(adc_self_ch_buff[2]-adc_self_ch_buff[0]);//adc_self_ch_buff[2]&0x0f80;

//adc_self_ch_disp[0]=adc_self_ch_buff[0]&0x0ff0;
//adc_self_ch_disp[1]=adc_self_ch_buff[1]&0x0ff0;
//adc_self_ch_disp[2]=adc_self_ch_buff[2]&0x0ff0;


if(adc_self_ch_disp[2]<300)//==adc_self_ch_disp[2])
	{
	adc_result=adc_self_ch_buff[2];
	} 
else if(adc_self_ch_disp[1]<300)//==adc_self_ch_disp[2])
	{
	adc_result=adc_self_ch_buff[1];
	}
else if(adc_self_ch_disp[0]<300)//==adc_self_ch_disp[1])
	{
	adc_result=adc_self_ch_buff[0];
	}
    //adc_result=92;

if(adc_ch_net)
	{

	if(adc_window_flag)
		{
		main_power_buffer[0]+=(long)(adc_result>>2);
		main_power_buffer[1]+=(long)(adc_result>>2);
		main_power_buffer[2]+=(long)(adc_result>>2);
		main_power_buffer[3]+=(long)(adc_result>>2);
		}
//	main_power_buffer[4]+=(long)(adc_result>>2);
//	main_power_buffer[5]+=(long)(adc_result>>2);
//	main_power_buffer[6]+=(long)(adc_result>>2);
//	main_power_buffer[7]+=(long)(adc_result>>2);
//	main_power_buffer_cnt++;


	if(adc_result<100)
		{
		adc_zero_cnt++;
		}
	else adc_zero_cnt=0;

	if(adc_zero_cnt>=2000)
		{
		adc_zero_cnt=2000;
		main_power_buffer[0]=0;
		main_power_buffer[1]=0;
		main_power_buffer[2]=0;
		main_power_buffer[3]=0;
		net_buff_=0;
		}

	if(adc_zero_cnt==5)
		{
		
		if(adc_window_flag)
			{
			adc_gorb_cnt++;
			if(adc_gorb_cnt>=512)
				{
				adc_gorb_cnt=0;
				//net_buff_=main_power_buffer[0]>>8;
				//main_power_buffer[0]=0;
			   	}

			if((adc_gorb_cnt&0x007f)==0)
				{
				net_buff_=main_power_buffer[adc_gorb_cnt>>7]>>8;
				main_power_buffer[adc_gorb_cnt>>7]=0;
				}
			}

		//LPC_GPIO2->FIODIR|=(1<<8);
		//LPC_GPIO2->FIOPIN^=(1<<8);

		if((adc_window_cnt>150)&&(adc_window_flag))
			{
			adc_window_flag=0;

			
			}
		if((adc_window_cnt>30)&&(adc_window_cnt<70)&&(!adc_window_flag))
			{
			adc_window_flag=1;

			//LPC_GPIO2->FIODIR|=(1<<8);
			//LPC_GPIO2->FIOPIN|=(1<<8);
			}
		}
	} 
else if(!adc_ch_net)
	{
	adc_buff[adc_ch][adc_ch_cnt]=(long)adc_result;
	
	if((adc_ch_cnt&0x03)==0)
		{
		long temp_L;
		char i;
		temp_L=0;
		for(i=0;i<16;i++)
			{
			temp_L+=adc_buff[adc_ch][i];
			}
		adc_buff_[adc_ch]= (short)(temp_L>>4);

		//adc_buff_[3]=346;
		}
	if(++adc_ch>=16) 
		{
		adc_ch=0;
		adc_ch_cnt++;
		if(adc_ch_cnt>=16)adc_ch_cnt=0;
		}
	}

//adc_buff[adc_ch][adc_cnt1]=(adc_self_ch_buff[2]+adc_self_ch_buff[1])/2;

//if(adc_buff[adc_ch][adc_cnt1]<adc_buff_min[adc_ch])adc_buff_min[adc_ch]=adc_buff[adc_ch][adc_cnt1];
//if(adc_buff[adc_ch][adc_cnt1]>adc_buff_max[adc_ch])adc_buff_max[adc_ch]=adc_buff[adc_ch][adc_cnt1];
/*
	{
	if((adc_cnt1&0x03)==0)
		{
		temp_S=0;
		for(i=0;i<16;i++)
			{
			temp_S+=adc_buff[adc_ch][i];
			} 
         	adc_buff_[adc_ch]=temp_S>>4;
          }
	}*/


		  

adc_self_ch_cnt=0;

adc_ch_net++;
adc_ch_net&=1;

//SET_REG(LPC_GPIO0->FIODIR,7,5,3);
//SET_REG(LPC_GPIO0->FIOPIN,adc_ch,5,3);

if(adc_ch_net)
	{
	//LPC_GPIO2->FIODIR|=(1<<7);
	//LPC_GPIO2->FIOPIN|=(1<<7);
	SET_REG(LPC_ADC->ADCR,1<<2,0,8);
	}
else
	{
	//LPC_GPIO2->FIODIR|=(1<<7);
	//LPC_GPIO2->FIOPIN&=~(1<<7);
	if(!(adc_ch&(1<<3)))SET_REG(LPC_ADC->ADCR,1<<0,0,8);
	else 			SET_REG(LPC_ADC->ADCR,1<<1,0,8);


	SET_REG(LPC_GPIO0->FIODIR,1,28,1);
	SET_REG(LPC_GPIO1->FIODIR,1,30,1);
	SET_REG(LPC_GPIO3->FIODIR,1,26,1);

	if(!(adc_ch&(1<<0)))SET_REG(LPC_GPIO0->FIOPIN,0,28,1);
	else 			SET_REG(LPC_GPIO0->FIOPIN,1,28,1);

	if(!(adc_ch&(1<<1)))SET_REG(LPC_GPIO1->FIOPIN,0,30,1);
	else 			SET_REG(LPC_GPIO1->FIOPIN,1,30,1);

	if(!(adc_ch&(1<<2)))SET_REG(LPC_GPIO3->FIOPIN,0,26,1);
	else 			SET_REG(LPC_GPIO3->FIOPIN,1,26,1);
	}
	



LPC_ADC->ADCR |=  (1<<24);

}
 /*
//-----------------------------------------------
void adc_drv_()
{
short temp_S;
char i;
adc_ch=4;
if(ADDR&0x00000001)
	{
	ADWR=ADDR_bit.VVDDA;
	
	if(++period_cnt>=200)
		{
		period_cnt=0;
		adc_buff[adc_ch][adc_cnt]=ADWR;
		
		if((adc_cnt&0x03)==0)
			{
			temp_S=0;
			for(i=0;i<16;i++)
				{
				temp_S+=adc_buff[adc_ch][i];
				}
			adc_buff_[adc_ch]=temp_S>>4;
			uart_out0(2,*((char*)&adc_buff_[adc_ch]),*(((char*)&adc_buff_[adc_ch])+1),0,0,0,0);

			}
		adc_cnt++;
		if(adc_cnt>=16)adc_cnt=0;
		
		}
	}

PINSEL1_bit.P0_28=1;	
PINSEL1_bit.P0_29=1;	
PINSEL1_bit.P0_30=1;	

PINSEL0_bit.P0_4=0;
PINSEL0_bit.P0_5=0;
PINSEL0_bit.P0_6=0;

IO0DIR_bit.P0_4=1;
IO0DIR_bit.P0_5=1;
IO0DIR_bit.P0_6=1;


if(adc_ch&0x02)IO0SET|=((long)1UL<<5);
else IO0CLR|=((long)1UL<<5);
if(adc_ch&0x04)IO0SET|=((long)1UL<<6);
else IO0CLR|=((long)1UL<<6);
if(adc_ch&0x08)IO0SET|=((long)1UL<<4);
else IO0CLR|=((long)1UL<<4);

ADCR_bit.PDN=1;
ADCR_bit.CLKDIV=14;
ADCR_bit.BURST=0;
ADCR_bit.CLKS=0;
ADCR_bit.TEST=0;

ADCR_bit.SEL=4;
ADCR_bit.START=1;
	

}
*/



//-----------------------------------------------
void avg_hndl(void)
{ 
char i;

if(bAVG_BLOCK)return;
//#define AVGCNTMAX	5
if(avg_main_cnt)
	{
	avg_main_cnt--;
	return;
	}                 

if(avg_cnt_<10)
	{
	avg_cnt_++;
	return;
	}   
avg_cnt_=0;
avg_num=0;

if(bAVG_DIR)bAVG_DIR=0;
else bAVG_DIR=1;

for(i=0;i<NUMIST;i++)
	{
	if((bps[i]._state==bsWRK)&&(bps[i]._cnt<20))avg_num++;
	}

/*if((K[NUMI]>=1)&&(bps_state[0]==ssWRK))	avg_num++;
if((K[NUMI]>=2)&&(bps_state[1]==ssWRK))	avg_num++;
if((K[NUMI]>=3)&&(bps_state[2]==ssWRK))	avg_num++;*/

	
if(avg_num<2)
	{
	return;
	}
	
else
	{
	i_avg_min=5000;
	i_avg_max=0;
	i_avg_summ=0;
	for(i=0;i<NUMIST;i++)
		{
		if(bps[i]._state==bsWRK)
			{
			if(bps[i]._Ii>i_avg_max)i_avg_max=bps[i]._Ii;
			if(bps[i]._Ii<i_avg_min)i_avg_min=bps[i]._Ii;
			
			i_avg_summ+=bps[i]._Ii;
			}
		}
	i_avg=i_avg_summ/avg_num;	
	
	if(i_avg_min==0)i_avg_min=1;

	avg=i_avg_max;
	avg*=100;
	avg/=i_avg_min;

	if(avg>160) bAVG=1;
	if((avg<120)||(vd_U<50)) bAVG=0;

	if(bAVG==1)
		{
		for(i=0;i<NUMIST;i++)
			{
			if(bps[i]._state==bsWRK)
				{
				signed long tempSL;
				tempSL=0;
				tempSL=(signed long)bps[i]._Ii;
				tempSL*=100L;
				if(i_avg==0) i_avg=1;
				tempSL/=(signed long)i_avg;
				bps[i]._avg=(signed short)tempSL;

				if((bps[i]._Ii>i_avg)&&(bps[i]._avg>120)&&(!bAVG_DIR))bps[i]._x_avg--;
				if((bps[i]._Ii<i_avg)&&(bps[i]._avg<80)&&(bAVG_DIR))bps[i]._x_avg++;
			
				if(bps[i]._x_avg<-50)bps[i]._x_avg=-50;
				if(bps[i]._x_avg>50)bps[i]._x_avg=50;	
				}
			else bps[i]._avg=0;
			}		
		}			
	}   	 

/*if(((signed long)(UOUT_-in_U))<20L)
	{
	for(i=0;i<NUMIST;i++) bps[i]._x_=0;
	}*/

avg_hndl_end:
__nop();  
}

//-----------------------------------------------
void u_out_reg_hndl(void)
{ 
char i;

if(u_out_reg_main_cnt)
	{
	u_out_reg_main_cnt--;
	return;
	}                 
u_out_reg_main_cnt=5;

if(out_U<(UOUT-5))
	{
	//if(UOUT_<UOUT)UOUT_=UOUT;
	UOUT_++;
	if(UOUT_>(UOUT+50))UOUT_=UOUT+50;
	}
else if(out_U>(UOUT+5))
	{
	//if(UOUT_>UOUT)UOUT_=UOUT;
	UOUT_--;
	if(UOUT_<(UOUT-50))UOUT_=UOUT-50;
	}
UOUT_=UOUT;
}

/*//-----------------------------------------------
void bp_on_(char in)
{
bp_tumbler[in-1]=1;
}

//-----------------------------------------------
void bp_off_(char in)
{
bp_tumbler[in-1]=0;
}
 */
//-----------------------------------------------
void rele_av_hndl(void)
{
char i,i1,i2;

i1=0;
i2=0;

for(i=0;i<NUMIST;i++)
	{
	if(bps[i]._av&0x0f)i1=1;
	if(bps[i]._av&0x10)i2=1;
	}

if(i1)					avar_vd_stat|=(1<<0);
else 					avar_vd_stat&=~(1<<0);
if(overloadAvar==0) 	avar_vd_stat&=~(1<<1);
else if(overloadAvar==1)avar_vd_stat|=(1<<1);

if(sysTAvar==0) 		avar_vd_stat&=~(1<<2);
else if(sysTAvar==1)	avar_vd_stat|=(1<<2);

if(uOutAvar==0) 		avar_vd_stat&=~((1<<3)|(1<<4));
else if(uOutAvar==1)	avar_vd_stat|=(1<<4);
else if(uOutAvar==2)	avar_vd_stat|=(1<<3);
if(uInAvar==0) 			avar_vd_stat&=~((1<<5)|(1<<6));
else if(uInAvar==1)		avar_vd_stat|=(1<<6);
else if(uInAvar==2)		avar_vd_stat|=(1<<5);

if(i2)					avar_vd_stat|=(1<<7);
else 					avar_vd_stat&=~(1<<7);

if(bVDISWORK)			avar_vd_stat|=(1<<8);
else 					avar_vd_stat&=~(1<<8);

for (i=0;i<4;i++)
	{
	if(avar_vd_stat&RELE_SET_MASK[i])rele_av_flags|=(1<<i);
	else rele_av_flags&=~(1<<i);
	} 
}

//*************-----------------------------------------------
void rele_hndl(void)
{
//static char cnt_rel_sam;
//char temp;

//temp=0;


SET_REG(LPC_PINCON->PINSEL0,0,4*2,6*2);
SET_REG(LPC_GPIO0->FIODIR,63,4,6);
SET_REG(LPC_PINCON->PINSEL7,0,(25-16)*2,2);
SET_REG(LPC_GPIO3->FIODIR,1,25,1);
SET_REG(LPC_PINCON->PINSEL1,0,(29-16)*2,2);
SET_REG(LPC_GPIO0->FIODIR,1,29,1);





if((mess_find_unvol(MESS2RELE_HNDL))&&	(mess_data[0]==PARAM_RELE_AV_1))
	{
	if(mess_data[1]==0) SET_REG(LPC_GPIO3->FIOCLR,1,25,1);
	else SET_REG(LPC_GPIO3->FIOSET,1,25,1);
	}
else if(!(rele_av_flags&(1<<0))) SET_REG(LPC_GPIO3->FIOCLR,1,25,1);
else SET_REG(LPC_GPIO3->FIOSET,1,25,1);

if((mess_find_unvol(MESS2RELE_HNDL))&&	(mess_data[0]==PARAM_RELE_AV_2))
	{
	if(mess_data[1]==0) SET_REG(LPC_GPIO0->FIOCLR,1,7,1);
	else if(mess_data[1]==1) SET_REG(LPC_GPIO0->FIOSET,1,7,1);
	}
else if(!(rele_av_flags&(1<<1))) SET_REG(LPC_GPIO0->FIOCLR,1,7,1);
else SET_REG(LPC_GPIO0->FIOSET,1,7,1);

if((mess_find_unvol(MESS2RELE_HNDL))&&	(mess_data[0]==PARAM_RELE_AV_3))
	{
	if(mess_data[1]==0) SET_REG(LPC_GPIO0->FIOCLR,1,4,1);
	else if(mess_data[1]==1) SET_REG(LPC_GPIO0->FIOSET,1,4,1);
    }
else if(!(rele_av_flags&(1<<2))) SET_REG(LPC_GPIO0->FIOCLR,1,4,1);
else SET_REG(LPC_GPIO0->FIOSET,1,4,1);

if((mess_find_unvol(MESS2RELE_HNDL))&&	(mess_data[0]==PARAM_RELE_AV_4))	 
	{
	if(mess_data[1]==0) SET_REG(LPC_GPIO0->FIOCLR,1,9,1);
	else if(mess_data[1]==1) SET_REG(LPC_GPIO0->FIOSET,1,9,1);
	}
else if(!(rele_av_flags&(1<<3))) SET_REG(LPC_GPIO0->FIOCLR,1,9,1);
else SET_REG(LPC_GPIO0->FIOSET,1,9,1);

rele_on_cnt=0;
if(GET_REG( LPC_GPIO3->FIOPIN, 25, 1))	rele_on_cnt++;
if(GET_REG( LPC_GPIO0->FIOPIN, 7, 1))  rele_on_cnt++;
if(GET_REG( LPC_GPIO0->FIOPIN, 4, 1))  rele_on_cnt++;

}

//-----------------------------------------------
void bps_hndl(void)
{
char ptr__,i;
unsigned short tempUS;

if(sh_cnt0<10)
	{
	sh_cnt0++;
	if(sh_cnt0>=10)
		{
		sh_cnt0=0;
		b1Hz_sh=1;
		}
	}

/*if(sh_cnt1<5)
	{
	sh_cnt1++;
	if(sh_cnt1==5)
		{
		sh_cnt1=0;
		b2Hz_sh=1;
		}
	} */


/*
if(mess_find(MESS_SRC_ON_OFF))
	{
	if(mess_data[0]==_MESS_SRC_MASK_BLOK_2SEC)
		{
		char i;
		for(i=0;i<NUMIST;i++)
			{
			if(mess_data[1]&(1<<i))src[i]._ist_blok_cnt=20;
			}
		
		}
	else if(mess_data[0]==_MESS_SRC_MASK_UNBLOK)
		{
		char i;
		for(i=0;i<NUMIST;i++)
			{
			if(mess_data[1]&(1<<i))src[i]._ist_blok_cnt=0;
			}
		
		}
	}
	
else if(mess_find(_MESS_SRC_MASK_ON))
	{				
	if(mess_data[0]==_MESS_SRC_MASK_ON)
		{
		char i;
		for(i=0;i<NUMIST;i++)
			{
			if(mess_data[1]&(1<<i))
				{
				src[i]._ist_blok_cnt=0;
				src[i]._flags_tu=2;
				}
			}
		
		}				
	}*/



/*else*/ 
bps_on_mask=0;
bps_off_mask=0;

if(mess_find_unvol(MESS2BPS_HNDL))
	{
	if(mess_data[0]==PARAM_BPS_ALL_OFF_AFTER_2SEC)
		{
		bps_off_mask=0xffff;
		}

	if(mess_data[0]==PARAM_BPS_MASK_OFF_AFTER_2SEC)
		{
		bps_off_mask=mess_data[1];
		}

	if(mess_data[0]==PARAM_BPS_MASK_ON)
		{
		bps_on_mask=mess_data[1];
		}

	if(mess_data[0]==PARAM_BPS_ALL_ON)
		{
		bps_on_mask=0xffffffff;
		}

	if(mess_data[0]==PARAM_BPS_MASK_ON_OFF_AFTER_2SEC)
		{
		bps_on_mask=mess_data[1];
		bps_off_mask=~(mess_data[1]);
		}


	for(i=0;i<=NUMIST;i++)
		{
		if(bps_off_mask&(1<<i)) bps[i]._blok_cnt++;
		else bps[i]._blok_cnt=0;
		gran(&bps[i]._blok_cnt,0,50);
		if(bps[i]._blok_cnt>20) bps[i]._flags_tu=1;
		if(bps_on_mask&(1<<i)) bps[i]._flags_tu=0;
	     }

	
/*

	if(bps_all_off_cnt>20)
		{
		for(i=0;i<=NUMIST;i++)
			{
	     	bps[i]._flags_tu=1;
	     	}
		}
	else if(bps_mask_off_cnt>20)
		{
		for(i=0;i<=NUMIST;i++)
			{
			if(mess_data[1]&(1<<i)) bps[i]._flags_tu=1;
	     	}
		}	
		
	else if(bps_mask_on_off_cnt>20)
		{
		for(i=0;i<=NUMIST;i++)
			{
			if(mess_data[1]&(1<<i)) bps[i]._flags_tu=1;
			else bps[i]._flags_tu=0;
	     	}
		}
		
	if(mess_data[0]==PARAM_BPS_MASK_ON)
		{
		for(i=0;i<=NUMIST;i++)
			{
			if(mess_data[1]&(1<<i)) bps[i]._flags_tu=0;
	     	}
		}
*/										
	}


else if(b1Hz_sh)
	{
	ptr__=0;
     for(i=0;i<=NUMIST;i++)
		{
	    bps[i]._flags_tu=1;
	    }	
  	     
  	for(i=0;(i<NUMIST)&&(ptr__<num_necc);i++)
  		{
		char ii,iii;

		ii=(char)NUMIST;
		//if(ii<0)ii=0;
		if(ii>32)ii=32;
		iii=numOfForvardBps;
		//if(iii<0)iii=0;
		if(iii>=NUMIST)iii=0;
		iii+=i;
		iii=iii%ii;
		
  	     if((bps[iii]._state==bsRDY)||(bps[iii]._state==bsWRK))
  	         	{
  	         	bps[iii]._flags_tu=0;
  	         	ptr__++;
  	         	}
			
  	     }
	bps[numOfForvardBps_old]._flags_tu=0;

	if(main_1Hz_cnt<60)
		{
     	for(i=0;i<=NUMIST;i++)
			{
	     	bps[i]._flags_tu=0;
	     	}	
		}
	if(ipsBlckStat)
		{
     	for(i=0;i<=NUMIST;i++)
			{
	     	bps[i]._flags_tu=1;
	     	}
		}

     for(i=0;i<=NUMIST;i++)
		{
	    //if(bps[i]._flags_tu==1) 	bps[i]._x_=-50;
	   	}	
		 
  	}


for(i=0;i<NUMIST;i++)
     {
     if(bps[i]._ist_blok_host_cnt!=0)
        {
        bps[i]._flags_tu=99;
		bps[i]._ist_blok_host_cnt--;
        }
     }




b1Hz_sh=0;


num_of_wrks_bps=0;
tempUS=0;
for(i=0;i<NUMIST;i++)
	{
	if(bps[i]._state==bsWRK)
		{
		num_of_wrks_bps++;
		if(bps[i]._Uii>tempUS)tempUS=bps[i]._Uii;
		}
	}
Ubpsmax=tempUS;

//bPARALLEL_ENOUG=0;
//bPARALLEL_NOT_ENOUG=1;
/*
for(i=0;i<NUMIST;i++)
	{
	if(bps[i]._Ti>=TSIGN)
		{
		bPARALLEL_ENOUG=1;
		}
	if(bps[i]._Ti>=(TSIGN-5))
		{
		bPARALLEL_NOT_ENOUG=0;
		}
	}

if(bPARALLEL_ENOUG==1)
	{
	bPARALLEL=1;
	}
else if(bPARALLEL && bPARALLEL_NOT_ENOUG)
	{
	bPARALLEL=0;
	}*/
}

//���� ������ � ���������� ���������� �� ���������� � ����������
#define AV_OVERLOAD	0
#define AV_T	1
#define AVUMAX	3
#define AVUMIN	4

//-----------------------------------------------
void powerAntiAliasingHndl(void)
{
if((power_summary_tempo/10UL)==(power_summary_tempo_old/10UL))
	{
	if(powerSummaryCnt<15)powerSummaryCnt++;
	if(powerSummaryCnt>=10)
		{
		power_summary=power_summary_tempo;
		}
	}
else powerSummaryCnt=0;
power_summary_tempo_old=power_summary_tempo;

if((power_current_tempo/10UL)==(power_current_tempo_old/10UL))
	{
	if(powerCurrentCnt<15)powerCurrentCnt++;
	if(powerCurrentCnt>=10)
		{
		power_current=power_current_tempo;
		}
	}
else powerCurrentCnt=0;
power_current_tempo_old=power_current_tempo;
}

//-----------------------------------------------
void ips_current_average_hndl(void)
{
if(++ica_timer_cnt>=10)
	{
	ica_timer_cnt=0;
	ica_plazma[0]++;

	ica_my_current=bps_I;

	if((ica_my_current>ica_your_current)&&((ica_my_current-ica_your_current)>=10)&&(ICA_EN==1))
		{
		ica_plazma[1]++;
		ica_u_necc--;
		}
	else if((ica_my_current<ica_your_current)&&((ica_your_current-ica_my_current)>=10)&&(ICA_EN==1))
		{
		ica_plazma[1]--;
		ica_u_necc++;
		}
	gran(&ica_u_necc,-20,20);
	}

if((ica_timer_cnt==8)&&(ICA_EN==1))
	{
	char modbus_buff[20],i;
	short crc_temp;

	modbus_buff[0] = ICA_MODBUS_ADDRESS;
	modbus_buff[1] = 4;
	modbus_buff[2] = 0;
	modbus_buff[3] = 2;
	modbus_buff[4] = 0;	
	modbus_buff[5] = 1;

	crc_temp= CRC16_2(modbus_buff,6);

	modbus_buff[6]= (char)crc_temp;
	modbus_buff[7]= (char)(crc_temp>>8);

	if(ICA_CH==0)
		{
		for (i=0;i<8;i++)
			{
			putchar_sc16is700(modbus_buff[i]);
			}
		}
	else if(ICA_CH==1)
		{
	/*	static U8 rem_IP[4];
		rem_IP[0]=ICA_MODBUS_TCP_IP1;
		rem_IP[1]=ICA_MODBUS_TCP_IP2;
		rem_IP[2]=ICA_MODBUS_TCP_IP3;
		rem_IP[3]=ICA_MODBUS_TCP_IP4;*/
  		//tcp_soc_avg = tcp_get_socket (TCP_TYPE_CLIENT, 0, 30, tcp_callback);
  		if (tcp_soc_avg != 0) 
			{
    		
			//tcp_connect_stat=0;
    		//tcp_connect (tcp_soc_avg, rem_IP, 502, 1000);
			/*while(!tcp_connect_stat)
				{
				}*/
			//delay_us(500);
			//tcp_close(tcp_soc_avg);

			}
		}
	}

if((ica_timer_cnt==3)&&(ICA_EN==1))
	{
	//if(tcp_connect_stat)
		{
		//tcp_close(tcp_soc_avg);
		//tcp_connect_stat=3;
		}
	}

if((main_kb_cnt==(TBAT*60)-21)&&(ICA_EN==1))
	{
	char modbus_buff[20],i;
	short crc_temp;

	modbus_buff[0] = ICA_MODBUS_ADDRESS;
	modbus_buff[1] = 6;
	modbus_buff[2] = 0;
	modbus_buff[3] = 30;
	modbus_buff[4] = (char)(TBAT/256);	
	modbus_buff[5] = (char)(TBAT%256);

	crc_temp= CRC16_2(modbus_buff,6);

	modbus_buff[6]= (char)crc_temp;
	modbus_buff[7]= (char)(crc_temp>>8);

	if(ICA_CH==0)
		{
		for (i=0;i<8;i++)
			{
			putchar_sc16is700(modbus_buff[i]);
			}
		}
	}

}

//-----------------------------------------------
void inv_drv(char in)
{
char temp,temp_;
//if (bps[in]._device!=dINV) return;
//plazma_inv[4];

gran_char((signed char*)&first_inv_slot,1,7);


temp=inv[in]._flags_tm_old^inv[in]._flags_tm;
if(temp)plazma_inv[1] =temp;

temp_=inv[in]._flags_tm&temp;
if(temp_)plazma_inv[2] =temp_;

if( (temp&(1<<0)) && (temp_&(1<<0)) ) 
	{
	plazma_inv[3]++;
	if(in==0)snmp_trap_send("Invertor #1 Alarm, overload",14,1,1);
	else if(in==1)snmp_trap_send("Invertor #2 Alarm, overload",14,2,1);
	else if(in==2)snmp_trap_send("Invertor #3 Alarm, overload",14,3,1);
	else if(in==3)snmp_trap_send("Invertor #4 Alarm, overload",14,4,1);
	}
else if( (temp&(1<<1)) && (temp_&(1<<1)) )
	{
		plazma_inv[3]++;
	if(in==0)snmp_trap_send("Invertor #1 Alarm, overheat",14,1,2);
	else if(in==1)snmp_trap_send("Invertor #2 Alarm, overheat",14,2,2);
	else if(in==2)snmp_trap_send("Invertor #3 Alarm, overheat",14,3,2);
	else if(in==3)snmp_trap_send("Invertor #4 Alarm, overheat",14,4,2);
	}

else if( (temp&(1<<2)) && (temp_&(1<<2)) )
	{
		plazma_inv[3]++;
	if(in==0)snmp_trap_send("Invertor #1 Alarm, is warm",14,1,3);
	else if(in==1)snmp_trap_send("Invertor #2 Alarm, is warm",14,2,3);
	else if(in==2)snmp_trap_send("Invertor #3 Alarm, is warm",14,3,3);
	else if(in==3)snmp_trap_send("Invertor #4 Alarm, is warm",14,4,3);
	}

else if( (temp&(1<<3)) && (temp_&(1<<3)) ) 
	{
		plazma_inv[3]++;
	if(in==0)snmp_trap_send("Invertor #1 Alarm, voltage is up",14,1,4);
	else if(in==1)snmp_trap_send("Invertor #2 Alarm, voltage is up",14,2,4);
	else if(in==2)snmp_trap_send("Invertor #3 Alarm, voltage is up",14,3,4);
	else if(in==3)snmp_trap_send("Invertor #4 Alarm, voltage is up",14,4,4);
	}

else if( (temp&(1<<4)) && (temp_&(1<<4)) ) 
	{
		plazma_inv[3]++;
	if(in==0)snmp_trap_send("Invertor #1 Alarm, voltage is down",14,1,5);
	else if(in==1)snmp_trap_send("Invertor #2 Alarm, voltage is down",14,2,5);
	else if(in==2)snmp_trap_send("Invertor #3 Alarm, voltage is down",14,3,5);
	else if(in==3)snmp_trap_send("Invertor #4 Alarm, voltage is down",14,4,5);
	}

else if( (temp&(1<<5)) && (temp_&(1<<5)) )
	{
		plazma_inv[3]++;
	if(in==0)snmp_trap_send("Invertor #1 Alarm, output is offed",14,1,6);
	else if(in==1)snmp_trap_send("Invertor #2 Alarm, output is offed",14,2,6);
	else if(in==2)snmp_trap_send("Invertor #3 Alarm, output is offed",14,3,6);
	else if(in==3)snmp_trap_send("Invertor #4 Alarm, output is offed",14,4,6);
	}

else if((temp)&&(!temp_)) 
	{
		plazma_inv[3]++;
	if(in==0)snmp_trap_send("Invertor #1 is norm",14,1,0);
	else if(in==1)snmp_trap_send("Invertor #2 is norm",14,2,0);
	else if(in==2)snmp_trap_send("Invertor #3 is norm",14,3,0);
	else if(in==3)snmp_trap_send("Invertor #4 is norm",14,4,0);
	}

inv[in]._flags_tm_old=inv[in]._flags_tm;

}	

//-----------------------------------------------
void ipsBlckHndl(char in)
{

ipsBlckStat=0;
if(ipsBlckSrc==1)
	{
	if(((ipsBlckLog==0)&&(adc_buff_[11]>2000)) || ((ipsBlckLog==1)&&(adc_buff_[11]<2000))) ipsBlckStat=1;
	}
else if(ipsBlckSrc==2)
	{
	if(((ipsBlckLog==0)&&(adc_buff_[13]>2000)) || ((ipsBlckLog==1)&&(adc_buff_[13]<2000))) ipsBlckStat=1;
	}
}

//-----------------------------------------------
void bps_drv(char in)
{
char temp;

if (bps[in]._device!=dSRC) return;
temp=bps[in]._flags_tm;
if(temp&(1<<AV_T))
	{
	if(bps[in]._temp_av_cnt<10/*1200*/) 
		{
		bps[in]._temp_av_cnt++;
		if(bps[in]._temp_av_cnt>=10/*1200*/)
			{
			bps[in]._temp_av_cnt=10/*1200*/;
		   	if(!(bps[in]._av&(1<<0)))avar_bps_hndl(in,0,1);
			}
		}
	}

else if(!(temp&(1<<AV_T)))
	{
	if(bps[in]._temp_av_cnt) 
		{
		bps[in]._temp_av_cnt--;
		if(!bps[in]._temp_av_cnt)
			{
			if(bps[in]._av&(1<<0))avar_bps_hndl(in,0,0);
			}
		} 	

	}

if((temp&(1<<AVUMAX)))
	{
	if(bps[in]._umax_av_cnt<10) 
		{
		bps[in]._umax_av_cnt++;
		if(bps[in]._umax_av_cnt>=10)
			{ 
			bps[in]._umax_av_cnt=10;
			if(!(bps[in]._av&(1<<1)))avar_bps_hndl(in,1,1);
		  	/*if((K[APV]!=ON)||((apv_cnt[in,0]==0)&&(apv_cnt[in,1]==0)&&(apv_cnt[in,2]==0)&&(apv_flags[in]==afOFF)))avar_s_hndl(in,1,1);
			if((apv_cnt[in,0]==0)&&(apv_cnt[in,1]==0)&&(apv_cnt[in,2]==0)&&(apv_flags[in]==afON))
				{
				apv_cnt[in,0]=APV_INIT;
				apv_cnt[in,1]=APV_INIT;
				apv_cnt[in,2]=APV_INIT;
				apv_flags[in]=afOFF;
				}				*/
						
			}
		} 
	}		
else if(!(temp&(1<<AVUMAX)))
	{
	if(bps[in]._umax_av_cnt>0) 
		{
		bps[in]._umax_av_cnt--;
		if(bps[in]._umax_av_cnt==0)
			{
			bps[in]._umax_av_cnt=0;
			avar_bps_hndl(in,1,0);
	 //		apv_cnt[in,0]=0;
	//		apv_cnt[in,1]=0;
	 //		apv_cnt[in,2]=0;			
			}
		}
	else if(bps[in]._umax_av_cnt<0) bps[in]._umax_av_cnt=0;		 
	}

if(temp&(1<<AVUMIN))
	{
	if(bps[in]._umin_av_cnt<10) 
		{
		bps[in]._umin_av_cnt++;
		if(bps[in]._umin_av_cnt>=10)
			{ 
			bps[in]._umin_av_cnt=10;
			if(!(bps[in]._av&(1<<2)))avar_bps_hndl(in,2,1);
		  	/*	if((K[APV]!=ON)||((apv_cnt[in,0]==0)&&(apv_cnt[in,1]==0)&&(apv_cnt[in,2]==0)&&(apv_flags[in]==afOFF)))avar_s_hndl(in,2,1);
			if((apv_cnt[in,0]==0)&&(apv_cnt[in,1]==0)&&(apv_cnt[in,2]==0)&&(apv_flags[in]==afON))
				{
				apv_cnt[in,0]=APV_INIT;
				apv_cnt[in,1]=APV_INIT;
				apv_cnt[in,2]=APV_INIT;
				apv_flags[in]=afOFF;
				}*/				
			}
		} 
	}	
	
else if(!(temp&(1<<AVUMIN)))
	{
	if(bps[in]._umin_av_cnt) 
		{
		bps[in]._umin_av_cnt--;
		if(bps[in]._umin_av_cnt==0)
			{
			bps[in]._umin_av_cnt=0;
			avar_bps_hndl(in,2,0);
		//	apv_cnt[in,0]=0;
		//	apv_cnt[in,1]=0;
		//	apv_cnt[in,2]=0;
			}
		}
	else if(bps[in]._umin_av_cnt>10)bps[in]._umin_av_cnt--;	 
	}

//bps[in]._state=bsOFF;

if (bps[in]._av&0x0f)					bps[in]._state=bsAV;
else if ( (net_av) && (bps[in]._cnt>20)/*&& 
		(bps[in]._Uii<200)*/)				bps[in]._state=bsOFF_AV_NET;
else if (bps[in]._flags_tm&BIN8(100000))	bps[in]._state=bsRDY;
else if (bps[in]._cnt<20)				bps[in]._state=bsWRK;



//else if(bps[in]._flags_tm&BIN8(100000)) bps[in]._state=ssBL;
//else if((!(bps[in]._flags_tm&BIN8(100000)))&&(net_U>100))bps[in]._state=ssWRK;
//else bps[0]._state=ssNOT;

//bps[in]._is_ready=0;
//bps[in]._is_wrk=0;
//if(bps[in]._av_net) bps[in]._flags_bp='N';// �� ���������
//else if(bps[in]._av_u_max) bps[in]._flags_bp='P';// �������� ����������(u_.av_.bAS1T)) bps_state[0]=ssAV;
//else if(bps[in]._av_u_min) bps[in]._flags_bp='M';// �������� ����������
//else if(bps[in]._av_temper) bps[in]._flags_bp='T';// �����������
//else if(bps[in]._flags_tm&BIN8(100000)) 
//	{
//	bps[in]._flags_bp='B';// ������������
//	bps[in]._is_ready=1;
//	}
//else if((!(bps[in]._flags_tm&BIN8(100000)))&&(net_U>100))
//     {
//     bps[in]._flags_bp='W';// ��������
//     bps[in]._is_ready=1;
//     bps[in]._is_wrk=1;
     
//     }
//else bps[in]._is_ready=1;     





/*
bps[in]._flags_tu&=BIN8(11111110);
if(bps[in]._ist_blok_cnt)
	{
	bps[in]._ist_blok_cnt--;
	bps[in]._flags_tu|=BIN8(1);
	}

	   */ 

//��������� ���� ��� ������ �����
if(bps[in]._cnt>=10) bps[in]._flags_tu|=BIN8(10000000);
else bps[in]._flags_tu&=BIN8(1111111);
	
bps[in]._vol_u=cntrl_stat+bps[in]._x_;	
bps[in]._vol_i=2000; 
}

//-----------------------------------------------
void avt_hndl(void)
{
char i;
for(i=0;i<12;i++)
	{
	if(eb2_data_short[6]&(1<<i))
		{
		avt_stat[i]=avtON;
		}
	else avt_stat[i]=avtOFF;
	}

if((avt_stat_old[0]!=avt_stat[0])&&(NUMAVT>=1))
	{
	if(avt_stat[0]==avtON) 	snmp_trap_send("Avtomat #1 is ON ",11,1,1);
	else 				snmp_trap_send("Avtomat #1 is OFF",11,1,0);
	}
if((avt_stat_old[1]!=avt_stat[1])&&(NUMAVT>=2))
	{
	if(avt_stat[1]==avtON) 	snmp_trap_send("Avtomat #2 is ON ",11,2,1);
	else 				snmp_trap_send("Avtomat #2 is OFF",11,2,0);
	}
if((avt_stat_old[2]!=avt_stat[2])&&(NUMAVT>=3))
	{
	if(avt_stat[2]==avtON) 	snmp_trap_send("Avtomat #3 is ON ",11,3,1);
	else 				snmp_trap_send("Avtomat #3 is OFF",11,3,0);
	}
if((avt_stat_old[3]!=avt_stat[3])&&(NUMAVT>=4))
	{
	if(avt_stat[3]==avtON) 	snmp_trap_send("Avtomat #4 is ON ",11,4,1);
	else 				snmp_trap_send("Avtomat #4 is OFF",11,4,0);
	}
if((avt_stat_old[4]!=avt_stat[4])&&(NUMAVT>=5))
	{
	if(avt_stat[4]==avtON) 	snmp_trap_send("Avtomat #5 is ON ",11,5,1);
	else 				snmp_trap_send("Avtomat #5 is OFF",11,5,0);
	}
if((avt_stat_old[5]!=avt_stat[5])&&(NUMAVT>=6))
	{
	if(avt_stat[5]==avtON) 	snmp_trap_send("Avtomat #6 is ON ",11,6,1);
	else 				snmp_trap_send("Avtomat #6 is OFF",11,6,0);
	}
if((avt_stat_old[6]!=avt_stat[6])&&(NUMAVT>=7))
	{
	if(avt_stat[6]==avtON) 	snmp_trap_send("Avtomat #7 is ON ",11,7,1);
	else 				snmp_trap_send("Avtomat #7 is OFF",11,7,0);
	}
if((avt_stat_old[7]!=avt_stat[7])&&(NUMAVT>=8))
	{
	if(avt_stat[7]==avtON) 	snmp_trap_send("Avtomat #8 is ON ",11,8,1);
	else 				snmp_trap_send("Avtomat #8 is OFF",11,8,0);
	}
if((avt_stat_old[8]!=avt_stat[8])&&(NUMAVT>=9))
	{
	if(avt_stat[8]==avtON) 	snmp_trap_send("Avtomat #9 is ON ",11,9,1);
	else 				snmp_trap_send("Avtomat #9 is OFF",11,9,0);
	}

for(i=0;i<12;i++)
	{
	avt_stat_old[i]=avt_stat[i];
	}
}

//-----------------------------------------------
void bat_hndl(void)
{
/*if(mess_find(_MESS_BAT_MASK_ON))
	{
	if(mess_data[0]==_MESS_BAT_MASK_ON)
		{
		char i;
		for(i=0;i<2;i++)
			{
			if(mess_data[1]&(1<<i))
				{
				bat[i]._cnt_to_block=0;
     			bat[i]._rel_stat=0;
     			}
			}
		}
	}
if(mess_find(_MESS_BAT_MASK_OFF))
	{		
	if(mess_data[0]==_MESS_BAT_MASK_OFF)
		{
		char i;
		for(i=0;i<2;i++)
			{
			if((mess_data[1]&(1<<i)) && (bat[i]._cnt_to_block==0) && (bat[i]._rel_stat==0))
				{
				bat[i]._cnt_to_block=20;
				bat[i]._rel_stat=1;
     			}
			}
		
		}		
	}*/

if(mess_find_unvol(MESS2BAT_HNDL))
	{ 
	char i;
	
	if(mess_data[0]==PARAM_BAT_ALL_OFF_AFTER_2SEC)
		{
		for(i=0;i<MAX_NUM_OF_BAT;i++)
			{
			if(bat[i]._cnt_to_block<50)bat[i]._cnt_to_block++;
			}
		}

	else if(mess_data[0]==PARAM_BAT_MASK_OFF_AFTER_2SEC)
		{
		for(i=0;i<MAX_NUM_OF_BAT;i++)
			{
			if(mess_data[1]&(1<<i))
				{
				if(bat[i]._cnt_to_block<50) bat[i]._cnt_to_block++;
				}
			else bat[i]._cnt_to_block=0;
			}
		}
	else 
	 	{
		for(i=0;i<MAX_NUM_OF_BAT;i++)
			{
			bat[i]._cnt_to_block=0;
			}

		}
	for(i=0;i<MAX_NUM_OF_BAT;i++)
		{
		if(bat[i]._cnt_to_block>20)bat[i]._rel_stat=1;
		else bat[i]._rel_stat=0;
		}

	}

else 
	{
	char i;
	for(i=0;i<MAX_NUM_OF_BAT;i++)
		{
		bat[i]._cnt_to_block=0;
		bat[i]._rel_stat=0;
		}

	}

/*if(mess_find_unvol(MESS2BAT_HNDL1))
	{
	if(PARAM_BAT_ON)
		{
		char i;
		for(i=0;i<MAX_NUM_OF_BAT;i++)
			{
			if(mess_data[1]&(1<<i))
				{
				bat[i]._cnt_to_block=0;
				bat[i]._rel_stat=0;
				}
			}
		}
	} */
}

#ifdef UKU_TELECORE2015
//-----------------------------------------------
void lakb_hndl(void)
{
char i;
char temp;

//if()
temp=0;
for(i=0;i<3;i++)
	{
	if(i>=NUMBAT_TELECORE)lakb[i]._communicationFullErrorStat=0;
	else
		{
		if(lakbKanErrorStat)					lakb[i]._communicationFullErrorStat=1;
		if(lakb[i]._communication2lvlErrorStat)	lakb[i]._communicationFullErrorStat=2;
		else
			{
			 									lakb[i]._communicationFullErrorStat=0;
			temp++;
			}
		}
	}
lakbNotErrorNum=temp;





for(i=0;i<3;i++)
	{
	if((NUMBAT_TELECORE>i)&&(lakb[i]._communicationFullErrorStat==0))
		{
		signed short tempSS;
		tempSS=lakb[i]._s_o_c;
		tempSS*=10;
		tempSS/=(lakb[i]._s_o_h/10);
		gran(&tempSS,0,100);
		lakb[i]._zar_percent=tempSS;
		}
	else 
		{
		lakb[i]._zar_percent=0;
		}
	}


for(i=0;i<3;i++)
	{
	if((i>NUMBAT_TELECORE)||(lakb[i]._communicationFullErrorStat))
		{
		lakb[i]._ch_curr=0;	  
		lakb[i]._tot_bat_volt=0;
		lakb[i]._max_cell_temp=0;
		lakb[i]._s_o_c=0;
		lakb[i]._s_o_h=0;
		}
	}
}
#endif
#ifdef UKU_TELECORE2017
//-----------------------------------------------
void lakb_hndl(void)
{
char i;
char temp;

//if()
temp=0;
for(i=0;i<3;i++)
	{
	if(i>=NUMBAT_TELECORE)lakb[i]._communicationFullErrorStat=0;
	else
		{
		if(lakbKanErrorStat)					lakb[i]._communicationFullErrorStat=1;
		if(lakb[i]._communication2lvlErrorStat)	lakb[i]._communicationFullErrorStat=2;
		else
			{
			 									lakb[i]._communicationFullErrorStat=0;
			temp++;
			}
		}
	}
lakbNotErrorNum=temp;





for(i=0;i<3;i++)
	{
	if((NUMBAT_TELECORE>i)&&(lakb[i]._communicationFullErrorStat==0))
		{
		signed short tempSS;
		tempSS=lakb[i]._s_o_c;
		tempSS*=10;
		tempSS/=(lakb[i]._s_o_h/10);
		gran(&tempSS,0,100);
		lakb[i]._zar_percent=tempSS;
		}
	else 
		{
		lakb[i]._zar_percent=0;
		}
	}


for(i=0;i<3;i++)
	{
	if((i>NUMBAT_TELECORE)||(lakb[i]._communicationFullErrorStat))
		{
		lakb[i]._ch_curr=0;	  
		lakb[i]._tot_bat_volt=0;
		lakb[i]._max_cell_temp=0;
		lakb[i]._s_o_c=0;
		lakb[i]._s_o_h=0;
		}
	}
}
#endif
/*
#ifdef UKU_TELECORE2015
//-----------------------------------------------
void klimat_hndl_telecore2015(void)
{
char i;
char t_bps=20;
//t_box=25; 

if(TELECORE2015_KLIMAT_WARM_SIGNAL==0)
	{
	t_box_warm=t_ext[1];
	if(ND_EXT[1])t_box_warm=20;
	}
else if(TELECORE2015_KLIMAT_WARM_SIGNAL==1) 
	{
	t_box_warm=t_ext[0];
	if(ND_EXT[0])t_box_warm=20;
	}

if(TELECORE2015_KLIMAT_VENT_SIGNAL==0)
	{
	t_box_vent=t_ext[1];
	if(ND_EXT[1])t_box_vent=20;
	}
else if(TELECORE2015_KLIMAT_VENT_SIGNAL==1) 
	{
	t_box_vent=t_ext[0];
	if(ND_EXT[0])t_box_vent=20;
	}

TELECORE2015_KLIMAT_WARM_ON_temp=TELECORE2015_KLIMAT_WARM_ON;
if(
	(lakb_stat_comm_error)		//���� �� � ����� �� �������� ������� ���� �������� �� ������ 
	|| ((NUMBAT_TELECORE>0)&&(lakb[0]._zar_percent<TELECORE2015_KLIMAT_CAP)) //���� 1 ������� � �� ����� �������� ���� �������������� ������
	|| ((NUMBAT_TELECORE>1)&&(lakb[1]._zar_percent<TELECORE2015_KLIMAT_CAP)) //���� 2-� ������� .....
	|| ((NUMBAT_TELECORE>2)&&(lakb[2]._zar_percent<TELECORE2015_KLIMAT_CAP)) //���� 3-� �������	.....
  )	TELECORE2015_KLIMAT_WARM_ON_temp=0;

if(t_box_warm<TELECORE2015_KLIMAT_WARM_ON_temp) t_box_warm_on_cnt++;
else if(t_box_warm>TELECORE2015_KLIMAT_WARM_OFF) t_box_warm_on_cnt--;
gran(&t_box_warm_on_cnt,0,10);

if(t_box_warm_on_cnt>9) warm_stat_k=wsON;
else if(t_box_warm_on_cnt<1) warm_stat_k=wsOFF;


if(t_box_vent>TELECORE2015_KLIMAT_VENT_ON) t_box_vent_on_cnt++;
else if(t_box_vent<TELECORE2015_KLIMAT_VENT_OFF) t_box_vent_on_cnt--;
gran(&t_box_vent_on_cnt,0,10);

if(t_box_vent_on_cnt>9) vent_stat_k=vsON;
else if(t_box_vent_on_cnt<1) vent_stat_k=vsOFF;

for(i=0;i<NUMIST;i++)
	{
	if((bps[i]._Ti>t_bps)&&(bps[i]._cnt<10))t_bps=bps[i]._Ti;
	}

if(t_bps>TELECORE2015_KLIMAT_VVENT_ON) t_box_vvent_on_cnt++;
else if(t_bps<TELECORE2015_KLIMAT_VVENT_OFF) t_box_vvent_on_cnt--;
gran(&t_box_vvent_on_cnt,0,10);

if(t_box_vvent_on_cnt>9) vvent_stat_k=vsON;
else if(t_box_vvent_on_cnt<1) vvent_stat_k=vsOFF;

}
#endif
*/

/*
#ifdef UKU_TELECORE2017
//-----------------------------------------------
void klimat_hndl_telecore2017(void)
{
//char i;
//char t_bps=20;
short delta_t;
 

if(TELECORE2017_KLIMAT_WARM_SIGNAL==0)
	{
	t_box_warm=t_ext[1];
	if(ND_EXT[1])t_box_warm=20;
	}
else if(TELECORE2017_KLIMAT_WARM_SIGNAL==1) 
	{
	t_box_warm=t_ext[0];
	if(ND_EXT[0])t_box_warm=20;
	}

if(TELECORE2017_KLIMAT_VENT_SIGNAL==0)
	{
	t_box_vent=t_ext[1];
	if(ND_EXT[1])t_box_vent=20;
	}
else if(TELECORE2017_KLIMAT_VENT_SIGNAL==1) 
	{
	t_box_vent=t_ext[0];
	if(ND_EXT[0])t_box_vent=20;
	}

TELECORE2017_KLIMAT_WARM_ON_temp=TELECORE2017_KLIMAT_WARM_ON;
if(
	(lakb_stat_comm_error)		//���� �� � ����� �� �������� ������� ���� �������� �� ������ 
	|| ((NUMBAT_TELECORE>0)&&(lakb[0]._zar_percent<TELECORE2017_KLIMAT_CAP)) //���� 1 ������� � �� ����� �������� ���� �������������� ������
	|| ((NUMBAT_TELECORE>1)&&(lakb[1]._zar_percent<TELECORE2017_KLIMAT_CAP)) //���� 2-� ������� .....
	|| ((NUMBAT_TELECORE>2)&&(lakb[2]._zar_percent<TELECORE2017_KLIMAT_CAP)) //���� 3-� �������	.....
  )	TELECORE2017_KLIMAT_WARM_ON_temp=0;

if(t_box_warm<TELECORE2017_KLIMAT_WARM_ON_temp) t_box_warm_on_cnt++;
else if(t_box_warm>TELECORE2017_KLIMAT_WARM_OFF) t_box_warm_on_cnt--;
gran(&t_box_warm_on_cnt,0,10);

if(t_box_warm_on_cnt>9) warm_stat_k=wsON;
else if(t_box_warm_on_cnt<1) warm_stat_k=wsOFF;


if((t_box_vent<TELECORE2017_KLIMAT_VENT_ON20)) 	TELECORE2017_EXT_VENT_PWM=0;
if((t_box_vent>TELECORE2017_KLIMAT_VENT_ON20)&&(t_box_vent<TELECORE2017_KLIMAT_VENT_ON40)) 	TELECORE2017_EXT_VENT_PWM=1;
if((t_box_vent>TELECORE2017_KLIMAT_VENT_ON40)&&(t_box_vent<TELECORE2017_KLIMAT_VENT_ON60)) 	TELECORE2017_EXT_VENT_PWM=2;
if((t_box_vent>TELECORE2017_KLIMAT_VENT_ON60)&&(t_box_vent<TELECORE2017_KLIMAT_VENT_ON80)) 	TELECORE2017_EXT_VENT_PWM=3;
if((t_box_vent>TELECORE2017_KLIMAT_VENT_ON80)&&(t_box_vent<TELECORE2017_KLIMAT_VENT_ON100)) TELECORE2017_EXT_VENT_PWM=4;
if((t_box_vent>TELECORE2017_KLIMAT_VENT_ON100) ) 											TELECORE2017_EXT_VENT_PWM=5;
if(warm_stat_k==wsON) TELECORE2017_EXT_VENT_PWM=0;

delta_t= abs(t_ext[0]-t_ext[1]);
if((delta_t<TELECORE2017_KLIMAT_DVENT_ON20)) 		TELECORE2017_INT_VENT_PWM=0;
if((delta_t>TELECORE2017_KLIMAT_DVENT_ON20)&&(delta_t<TELECORE2017_KLIMAT_DVENT_ON40)) 		TELECORE2017_INT_VENT_PWM=1;
if((delta_t>TELECORE2017_KLIMAT_DVENT_ON40)&&(delta_t<TELECORE2017_KLIMAT_DVENT_ON60)) 		TELECORE2017_INT_VENT_PWM=2;
if((delta_t>TELECORE2017_KLIMAT_DVENT_ON60)&&(delta_t<TELECORE2017_KLIMAT_DVENT_ON80)) 		TELECORE2017_INT_VENT_PWM=3;
if((delta_t>TELECORE2017_KLIMAT_DVENT_ON80)&&(delta_t<TELECORE2017_KLIMAT_DVENT_ON100)) 	TELECORE2017_INT_VENT_PWM=4;
if((delta_t>TELECORE2017_KLIMAT_DVENT_ON100) ) 												TELECORE2017_INT_VENT_PWM=5;

gran_char(&TELECORE2017_EXT_VENT_PWM,0,5);
gran_char(&TELECORE2017_INT_VENT_PWM,0,5);

if(TELECORE2017_EXT_VENT_PWM)TELECORE2017_INT_VENT_PWM=TELECORE2017_EXT_VENT_PWM;

//ND_EXT[0]=0;
//ND_EXT[1]=0;




if((mess_find_unvol(MESS2KLIMAT_CNTRL))&&(mess_data[0]==PARAM_KLIMAT_CNTRL_VENT_INT))
	{
	TELECORE2017_INT_VENT_PWM=mess_data[1];
	}


if((mess_find_unvol(MESS2KLIMAT_CNTRL))&&(mess_data[0]==PARAM_KLIMAT_CNTRL_VENT_EXT))
	{
	TELECORE2017_EXT_VENT_PWM=mess_data[1];
	}

	
if(TELECORE2017_INT_VENT_PWM||TELECORE2017_EXT_VENT_PWM) 	vent_stat_k=vsON;
else 														vent_stat_k=vsOFF;


if(t_box_warm<-20)
	{
	if(t_box_warm_minus20_cnt<60)
		{
		t_box_warm_minus20_cnt++;
		if(t_box_warm_minus20_cnt==60)
			{
			snmp_trap_send("Temperature at the bottom of box is below -20",20,1,1);
			}
		}
	}
else if(t_box_warm>-20)
	{
	if(t_box_warm_minus20_cnt>0)
		{
		t_box_warm_minus20_cnt--;
		if(t_box_warm_minus20_cnt==0)
			{
			snmp_trap_send("Temperature at the bottom of box is below -20  clear",20,1,0);
			}
		}
	} 

if(t_box_warm>65)
	{
	if(t_box_warm_plus65_cnt<60)
		{
		t_box_warm_plus65_cnt++;
		if(t_box_warm_plus65_cnt==60)
			{
			snmp_trap_send("Temperature at the bottom of box is above 65",20,2,1);
			}
		}
	}
else if(t_box_warm<65)
	{
	if(t_box_warm_plus65_cnt>0)
		{
		t_box_warm_plus65_cnt--;
		if(t_box_warm_plus65_cnt==0)
			{
			snmp_trap_send("Temperature at the bottom of box is above 65 clear",20,2,0);
			}
		}
	}
	
if(t_box_vent>70)
	{
	if(t_box_cool_plus70_cnt<60)
		{
		t_box_cool_plus70_cnt++;
		if(t_box_cool_plus70_cnt==60)
			{
			snmp_trap_send("Temperature at the top of box is above 70",20,3,1);
			}
		}
	}
else if(t_box_vent<70)
	{
	if(t_box_cool_plus70_cnt>0)
		{
		t_box_cool_plus70_cnt--;
		if(t_box_cool_plus70_cnt==0)
			{
			snmp_trap_send("Temperature at the top of box is above 70 clear",20,3,0);
			}
		}
	}	 
}

														
#endif
*/
#ifndef UKU_KONTUR
//-----------------------------------------------
void klimat_hndl(void)
{


if(t_box>TBOXMAX)
	{
	av_tbox_cnt++;
	} 
else if(t_box<TBOXMAX)
	{
	av_tbox_cnt--;
	}
gran(&av_tbox_cnt,0,6);

if(av_tbox_cnt>5)
	{
	av_tbox_stat=atsON;
	}
if(av_tbox_cnt<1)
	{
	av_tbox_stat=atsOFF;
	}

if(t_box<(TBOXREG-2))
	{
	if(t_box_cnt<30)
		{
		t_box_cnt++;
		if(t_box_cnt>=30)
			{
			main_vent_pos--;
			t_box_cnt=0;
			}
		}
	}
else if(t_box>(TBOXREG))
	{
	if(t_box_cnt<30)
		{
		t_box_cnt++;
		if(t_box_cnt>=30)
			{
			main_vent_pos++;
			t_box_cnt=0;
			}
		}
	}
else
	{
	t_box_cnt=0;
	}

#ifndef UKU_KONTUR
if(t_box>TBOXVENTMAX)gran(&main_vent_pos,0,20); 
else gran(&main_vent_pos,0,pos_vent+9);

if((mess_find_unvol(MESS2VENT_HNDL))&&(mess_data[0]==PARAM_VENT_CB))
	{
	main_vent_pos=mess_data[1];
	}


if(main_vent_pos<=1)mixer_vent_stat=mvsON;
else mixer_vent_stat=mvsOFF;
#endif

#ifdef UKU_KONTUR

if(t_box>TBOXVENTON) t_box_vent_on_cnt++;
else if(t_box<TBOXVENTOFF) t_box_vent_on_cnt--;
gran(&t_box_vent_on_cnt,0,10);

if(t_box_vent_on_cnt>9) vent_stat_k=vsON;
else if(t_box_vent_on_cnt<1) vent_stat_k=vsOFF;

if(t_box<TBOXWARMON) t_box_warm_on_cnt++;
else if(t_box>TBOXWARMOFF) t_box_warm_on_cnt--;
gran(&t_box_warm_on_cnt,0,10);

if(t_box_warm_on_cnt>9) warm_stat_k=wsON;
else if(t_box_warm_on_cnt<1) warm_stat_k=wsOFF;

#endif

if((TBATDISABLE>=50) && (TBATDISABLE<=90))
	{
	if(t_box>TBATDISABLE)
		{
		tbatdisable_cnt++;
		}
	if(t_box<TBATENABLE)
		{
		tbatdisable_cnt--;
		}
	gran(&tbatdisable_cnt,0,6);

	if(tbatdisable_cnt>5)
		{
		tbatdisable_stat=tbdsOFF;
		}
	if(tbatdisable_cnt<1)
		{
		tbatdisable_stat=tbdsON;
		}
	}
else 
	{
	tbatdisable_stat=tbdsON;
	}

if((TLOADDISABLE>=50) && (TLOADDISABLE<=80))
	{
	if(t_box>TLOADDISABLE)
		{
		tloaddisable_cnt++;
		}
	if(t_box<TLOADENABLE)
		{
		tloaddisable_cnt--;
		}
	gran(&tloaddisable_cnt,0,6);

	if(tloaddisable_cnt>5)
		{
		tloaddisable_stat=tldsOFF;
		}
	if(tloaddisable_cnt<1)
		{
		tloaddisable_stat=tldsON;
		}
	}
else 
	{
	tloaddisable_stat=tldsON;
	}

}
#endif

#ifdef UKU_KONTUR
//-----------------------------------------------
void klimat_hndl(void)
{


if(t_box>TBOXMAX)
	{
	av_tbox_cnt++;
	} 
else if(t_box<TBOXMAX)
	{
	av_tbox_cnt--;
	}
gran(&av_tbox_cnt,0,6);

if(av_tbox_cnt>5)
	{
	av_tbox_stat=atsON;
	}
if(av_tbox_cnt<1)
	{
	av_tbox_stat=atsOFF;
	}

if(t_box<(TBOXREG-2))
	{
	if(t_box_cnt<30)
		{
		t_box_cnt++;
		if(t_box_cnt>=30)
			{
			main_vent_pos--;
			t_box_cnt=0;
			}
		}
	}
else if(t_box>(TBOXREG))
	{
	if(t_box_cnt<30)
		{
		t_box_cnt++;
		if(t_box_cnt>=30)
			{
			main_vent_pos++;
			t_box_cnt=0;
			}
		}
	}
else
	{
	t_box_cnt=0;
	}


if(t_box>TBOXVENTMAX)gran(&main_vent_pos,0,20); 
else gran(&main_vent_pos,0,pos_vent+9);

if((mess_find_unvol(MESS2VENT_HNDL))&&(mess_data[0]==PARAM_VENT_CB))
	{
	main_vent_pos=mess_data[1];
	}


if(main_vent_pos<=1)vent_stat_k=vsON;
else vent_stat_k=vsOFF;


if(t_box<TBOXWARMON) t_box_warm_on_cnt++;
else if(t_box>TBOXWARMOFF) t_box_warm_on_cnt--;
gran(&t_box_warm_on_cnt,0,10);

if(t_box_warm_on_cnt>9) warm_stat_k=wsON;
else if(t_box_warm_on_cnt<1) warm_stat_k=wsOFF;



if((TBATDISABLE>=50) && (TBATDISABLE<=90))
	{


	if(t_box>TBATDISABLE)
		{
		tbatdisable_cnt++;
		}
	if(t_box<TBATENABLE)
		{
		tbatdisable_cnt--;
		}
	gran(&tbatdisable_cnt,0,6);

	if(tbatdisable_cnt>5)
		{
		tbatdisable_stat=tbdsOFF;
		}
	if(tbatdisable_cnt<1)
		{
		tbatdisable_stat=tbdsON;
		}
	}
else 
	{
	tbatdisable_stat=tbdsON;
	}

if((TLOADDISABLE>=50) && (TLOADDISABLE<=80))
	{
	if(t_box>TLOADDISABLE)
		{
		tloaddisable_cnt++;
		}
	if(t_box<TLOADENABLE)
		{
		tloaddisable_cnt--;
		}
	gran(&tloaddisable_cnt,0,6);

	if(tloaddisable_cnt>5)
		{
		tloaddisable_stat=tldsOFF;
		}
	if(tloaddisable_cnt<1)
		{
		tloaddisable_stat=tldsON;
		}
	}
else 
	{
	tloaddisable_stat=tldsON;
	}

}
#endif



//-----------------------------------------------
void overload_hndl(void)
{

if(main_1Hz_cnt<30)return;

if((out_U<UOUT)&&((UOUT-out_U)>50)&&(!(avar_vd_stat&0x0081)))
	{
	if(overloadHndlCnt<(OVERLOAD_TIME*10/*TZAS*10*/))
		{
		overloadHndlCnt++;
		if(overloadHndlCnt==(OVERLOAD_TIME*10/*TZAS*10*/))
			{
			avar_overload_hndl(1);
			}
		}
	}
else if(out_U>(UOUT-50))
	{
	if(overloadHndlCnt)
		{
		overloadHndlCnt--;
		if(overloadHndlCnt==0)
			{
			avar_overload_hndl(0);
			}
		}
	}

if((Ib_ips_termokompensat>OVERLOAD_CURR))
	{
	if(overloadHndlCnt1<(OVERLOAD_TIME*10))
		{
		overloadHndlCnt1++;
		if(overloadHndlCnt1==(OVERLOAD_TIME*10))
			{
			avar_overload_hndl(1);
			}
		}
	}
else if((Ib_ips_termokompensat<OVERLOAD_CURR)  && (!bIBAT_SMKLBR))
	{
	if(overloadHndlCnt1)
		{
		overloadHndlCnt1--;
		if(overloadHndlCnt1==0)
			{
			avar_overload_hndl(0);
			}
		}
	}


}

//-----------------------------------------------
void u_avar_hndl(void)
{

if(main_1Hz_cnt<10)return;

if(out_U>UOUTMAX)
	{
	if(uAvarHndlOutUMaxCnt<(10*TZAS))
		{
		uAvarHndlOutUMaxCnt++;
		if(uAvarHndlOutUMaxCnt>=(10*TZAS))
			{
			avar_u_out_hndl(1,out_U);
			}
		}
	else uAvarHndlOutUMaxCnt=(10*TZAS);
	}
else 
	{
	if(uAvarHndlOutUMaxCnt>0)
		{
		uAvarHndlOutUMaxCnt--;
		if(uAvarHndlOutUMaxCnt==0)
			{
			avar_u_out_hndl(0,out_U);
			}
		}
	else uAvarHndlOutUMaxCnt=0;
	}

if(out_U<UOUTMIN)
	{
	if(uAvarHndlOutUMinCnt<(10*TZAS))
		{
		uAvarHndlOutUMinCnt++;
		if(uAvarHndlOutUMinCnt>=(10*TZAS))
			{
			avar_u_out_hndl(2,out_U);
			}
		}
	else uAvarHndlOutUMinCnt=(10*TZAS);
	}
else 
	{
	if(uAvarHndlOutUMinCnt>0)
		{
		uAvarHndlOutUMinCnt--;
		if(uAvarHndlOutUMinCnt==0)
			{
			avar_u_out_hndl(0,out_U);
			}
		}
	else uAvarHndlOutUMinCnt=0;
	}

if(in_U>UINMAX)
	{
	if(uAvarHndlInUMaxCnt<(10*TZAS))
		{
		uAvarHndlInUMaxCnt++;
		if(uAvarHndlInUMaxCnt>=(10*TZAS))
			{
			avar_u_in_hndl(1,in_U);
			}
		}
	else uAvarHndlInUMaxCnt=(10*TZAS);
	}
else 
	{
	if(uAvarHndlInUMaxCnt>0)
		{
		uAvarHndlInUMaxCnt--;
		if(uAvarHndlInUMaxCnt==0)
			{
			avar_u_in_hndl(0,in_U);
			}
		}
	else uAvarHndlInUMaxCnt=0;
	}

if(in_U<UINMIN)
	{
	if(uAvarHndlInUMinCnt<(10*TZAS))
		{
		uAvarHndlInUMinCnt++;
		if(uAvarHndlInUMinCnt>=(10*TZAS))
			{
			avar_u_in_hndl(2,in_U);
			}
		}
	else uAvarHndlInUMinCnt=(10*TZAS);
	}
else 
	{
	if(uAvarHndlInUMinCnt>0)
		{
		uAvarHndlInUMinCnt--;
		if(uAvarHndlInUMinCnt==0)
			{
			avar_u_in_hndl(0,in_U);
			}
		}
	else uAvarHndlInUMinCnt=0;
	}
}

//-----------------------------------------------
void t_sys_avar_hndl(void)
{

if(main_1Hz_cnt<10)return;

if(sys_T>TSYSMAX)
	{
	if(sysTAvarHndlCnt<(10*TZAS))
		{
		sysTAvarHndlCnt++;
		if(sysTAvarHndlCnt>=(10*TZAS))
			{
			avar_sys_t_hndl(1,sys_T);
			}
		}
	else sysTAvarHndlCnt=(10*TZAS);
	}
else 
	{
	if(sysTAvarHndlCnt>0)
		{
		sysTAvarHndlCnt--;
		if(sysTAvarHndlCnt==0)
			{
			avar_sys_t_hndl(0,sys_T);
			}
		}
	else sysTAvarHndlCnt=0;
	}
}


//-----------------------------------------------
void u_necc_hndl(void)
{
signed long temp_L;
signed long temp_SL;
//signed short temp_SS;

//char i;

//temp_SS=0;
signed short t[2];
#ifdef UKU_220_IPS_TERMOKOMPENSAT

if(!TERMOKOMPENS)
	{
	//u_necc=U0B;
	//u_necc=UB20;
	}
else
	{
	if(ND_EXT[0])t[0]=20;
	else t[0]=t_ext[0];

	mat_temper=t[0];
			
/*	if(mat_temper<0)temp_SL=UB0; 
	else 
		{
		if(mat_temper>40)mat_temper=40; 
		temp_SL=(UB20-UB0)*10;
		temp_SL*=mat_temper;
		temp_SL/=200;
		temp_SL+=UB0;
		}			  */
	if(spc_stat==spcVZ)
		{
		//temp_SL=UVZ;
		}
	u_necc=(unsigned int)temp_SL;
	///u_necc=3456;
	}  

//u_necc=2355;

if(speedChIsOn)
	{
	u_necc=speedChrgVolt;
	}

if(mess_find_unvol(MESS2UNECC_HNDL))
	{		
	if(mess_data[0]==PARAM_UNECC_SET)
		{
		u_necc=mess_data[1];
		}		
	}
if(ICA_EN)u_necc+=ica_u_necc;
#endif


//#ifndef UKU_220_IPS_TERMOKOMPENSAT
/*
#ifndef UKU_TELECORE2015
#ifndef UKU_TELECORE2017
if(unh_cnt0<10)
	{
	unh_cnt0++;
	if(unh_cnt0>=10)
		{
		unh_cnt0=0;
		b1Hz_unh=1;
		}
	}

if(unh_cnt1<5)
	{
	unh_cnt1++;
	if(unh_cnt1==5)
		{
		unh_cnt1=0;
//		b2Hz_unh=1;
		}
	} 



if(mess_find_unvol(MESS2UNECC_HNDL))
	{		
	if(mess_data[0]==PARAM_UNECC_SET)
		{
		u_necc=mess_data[1];
		}		
	}


else if(b1Hz_unh)
	{
	
	if((BAT_IS_ON[0]!=bisON) && (BAT_IS_ON[1]!=bisON))
		{
		
		u_necc=U0B;
		}
	else 
		{
		if(BAT_TYPE==0) //���� ������� �������
			{
			for(i=0;i<2;i++)
				{
				if(BAT_IS_ON[i]==bisON)
					{
					if(bat[i]._nd)t[i]=20;
					else t[i]=bat[i]._Tb;
					}
				else
					{
					t[i]=-20;
					}
				}
			if(t[0]>t[1])mat_temper=t[0];
			else mat_temper=t[1];
			
		
			if(mat_temper<0)temp_SL=UB0; 
			else 
				{
				if(mat_temper>40)mat_temper=40; 
				temp_SL=(UB20-UB0)*10;
				temp_SL*=mat_temper;
				temp_SL/=200;
				temp_SL+=UB0;
				}
			if(spc_stat==spcVZ)
				{
				temp_SL=UVZ;
				}
			u_necc=(unsigned int)temp_SL;
			}
		else if(BAT_TYPE==1) //���� ������� ���������
			{
			u_necc=U0B;
					
			u_necc=bat[0]._Ub+10;

			
			if(spc_stat==spcVZ)
				{
				u_necc=UVZ;
				}
			if(u_necc>=UB0) u_necc=UB0;
			if(u_necc>=UB20) u_necc=UB20;
			}
		}  
	}
#endif
#endif*/
/*
#ifdef UKU_TELECORE2015

if(unh_cnt0<10)
	{
	unh_cnt0++;
	if(unh_cnt0>=10)
		{
		unh_cnt0=0;
		b1Hz_unh=1;
		}
	}

if(unh_cnt1<5)
	{
	unh_cnt1++;
	if(unh_cnt1==5)
		{
		unh_cnt1=0;
//		b2Hz_unh=1;
		}
	} 



if(mess_find_unvol(MESS2UNECC_HNDL))
	{		
	if(mess_data[0]==PARAM_UNECC_SET)
		{
		u_necc=mess_data[1];
		}		
	}


else if(b1Hz_unh)
	{
	b1Hz_unh=0;

	if(BAT_TYPE==0)
		{
		if(bat[0]._nd)mat_temper=20;
		else mat_temper=bat[0]._Tb;

			
		if(mat_temper<0)temp_SL=UB0; 
		else 
			{
			if(mat_temper>40)mat_temper=40; 
			temp_SL=(UB20-UB0)*10;
			temp_SL*=mat_temper;
			temp_SL/=200;
			temp_SL+=UB0;
			}
		if(spc_stat==spcVZ)
			{
			temp_SL=UVZ;
			}
		u_necc=(unsigned int)temp_SL;
	///u_necc=3456;
		}
	else if(BAT_TYPE==1)
		{
		
		gran(&DU_LI_BAT,1,30);
		u_necc=li_bat._Ub+DU_LI_BAT;
		gran(&u_necc,0,UB0);
		gran(&u_necc,0,UB20);
		gran(&u_necc,0,540);		


		if(li_bat._batStat!=bsOK)
			{
			u_necc=U0B;
			}
		if(spc_stat==spcVZ)
			{
			u_necc=UVZ;
			}

		}
	else if(BAT_TYPE==2)
		{
		u_necc=U0B;
		
		if(spc_stat==spcVZ)
			{
			u_necc=UVZ;
			}
	
		u_necc=UB0;
		}

	else if(BAT_TYPE==3)
		{
		u_necc=U0B;
		
		if(spc_stat==spcVZ)
			{
			u_necc=UVZ;
			}

		gran(&DU_LI_BAT,1,30);


		if(lakbNotErrorNum==0)
			{
			u_necc=U0B;
			}
		else 
			{
			signed short i;
			//signed short u_necc_max;
			//u_necc_max=0;
			char soc_flag=0;

			for(i=(NUMBAT_TELECORE-1);i>=0;i--)
				{
				if(lakb[i]._communicationFullErrorStat==0)u_necc=lakb[i]._tot_bat_volt+DU_LI_BAT;
				if(lakb[i]._s_o_c_percent<QSODERG_LI_BAT)soc_flag=1;
				}

			if(soc_flag==0)u_necc=USODERG_LI_BAT;
			}
		gran(&u_necc,0,UB0);
		//gran(&u_necc,0,UB20);
		gran(&u_necc,0,540);
		}
	}

#endif */
/*
#ifdef UKU_TELECORE2017

if(unh_cnt0<10)
	{
	unh_cnt0++;
	if(unh_cnt0>=10)
		{
		unh_cnt0=0;
		b1Hz_unh=1;
		}
	}

if(unh_cnt1<5)
	{
	unh_cnt1++;
	if(unh_cnt1==5)
		{
		unh_cnt1=0;
//		b2Hz_unh=1;
		}
	} 



if(mess_find_unvol(MESS2UNECC_HNDL))
	{		
	if(mess_data[0]==PARAM_UNECC_SET)
		{
		u_necc=mess_data[1];
		}		
	}


else if(b1Hz_unh)
	{
	b1Hz_unh=0;

	if(BAT_TYPE==0)
		{
		if(bat[0]._nd)mat_temper=20;
		else mat_temper=bat[0]._Tb;

			
		if(mat_temper<0)temp_SL=UB0; 
		else 
			{
			if(mat_temper>40)mat_temper=40; 
			temp_SL=(UB20-UB0)*10;
			temp_SL*=mat_temper;
			temp_SL/=200;
			temp_SL+=UB0;
			}
		if(spc_stat==spcVZ)
			{
			temp_SL=UVZ;
			}
		u_necc=(unsigned int)temp_SL;
	///u_necc=3456;
		}
	else if(BAT_TYPE==1)
		{
		
		gran(&DU_LI_BAT,1,30);
		u_necc=li_bat._Ub+DU_LI_BAT;
		gran(&u_necc,0,UB0);
		gran(&u_necc,0,UB20);
		gran(&u_necc,0,540);		


		if(li_bat._batStat!=bsOK)
			{
			u_necc=U0B;
			}
		if(spc_stat==spcVZ)
			{
			u_necc=UVZ;
			}
		}
	else if(BAT_TYPE==2)
		{
		u_necc=U0B;
		
		if(spc_stat==spcVZ)
			{
			u_necc=UVZ;
			}
	
		u_necc=UB0;
		}

	else if(BAT_TYPE==3)
		{
		u_necc=U0B;
		
		if(spc_stat==spcVZ)
			{
			u_necc=UVZ;
			}

		gran(&DU_LI_BAT,1,30);


		if(lakbNotErrorNum==0)
			{
			u_necc=U0B;
			}
		else 
			{
			signed short i;
			//signed short u_necc_max;
			//u_necc_max=0;
			char soc_flag=0;

			for(i=(NUMBAT_TELECORE-1);i>=0;i--)
				{
				if(lakb[i]._communicationFullErrorStat==0)u_necc=lakb[i]._tot_bat_volt+DU_LI_BAT;
				if(lakb[i]._s_o_c_percent<QSODERG_LI_BAT)soc_flag=1;
				}

			if(soc_flag==0)u_necc=USODERG_LI_BAT;
			}
		gran(&u_necc,0,UB0);
		//gran(&u_necc,0,UB20);
		gran(&u_necc,0,540);
		}
	}

#endif 
*/
//u_necc=2356;
//#endif//gran(&u_necc,400,UMAX);

temp_L=(signed long) u_necc;
temp_L*=98L;
temp_L/=100L;
u_necc_dn=(signed short)temp_L;

temp_L=(signed long) u_necc;
temp_L*=102L;
temp_L/=100L;
u_necc_up=(signed short)temp_L;


}


//-----------------------------------------------
void num_necc_hndl(void)
{

static short num_necc_block_cnt;
if(num_necc_block_cnt) num_necc_block_cnt--;

Isumm_=Isumm;

if(bat[0]._Ib<0) Isumm_+=(abs(bat[0]._Ib))/10;
if(bat[1]._Ib<0) Isumm_+=(abs(bat[1]._Ib))/10;

num_necc_up=(Isumm_/((signed short)IMAX))+1;
////Isumm_+=(signed short)((IMAX*(10-KIMAX))/10);
////Isumm_+=(signed short)(IMAX-IMIN);

num_necc_down=(Isumm_/((signed short)IMIN))+1;

if(num_necc_up>num_necc)
	{
	num_necc=num_necc_up;
	num_necc_block_cnt=60;
	}
else if(num_necc_down<num_necc)
	{
	if(!num_necc_block_cnt)
		{
		num_necc=num_necc_down;
		num_necc_block_cnt=60;
		}
	}

/*if(PAR)*/ num_necc=NUMIST;

#ifdef UKU_220_IPS_TERMOKOMPENSAT
//if(bPARALLEL) num_necc=NUMIST;
#endif

num_necc=NUMIST;

gran(&num_necc,1,NUMIST);

}


//-----------------------------------------------
void cntrl_hndl(void)
{
#define UBPSMAXPWM	2350
#define UBPSMINPWM	2100

IZMAX_=IZMAX;

//cntrl_hndl_plazma=10;

if(speedChIsOn)IZMAX_=speedChrgCurr;

if(cntrl_stat_blok_cnt)cntrl_stat_blok_cnt--;
if(cntrl_stat_blok_cnt_)cntrl_stat_blok_cnt_--;

if((bat[0]._temper_stat&0x03)||(bat[1]._temper_stat&0x03))IZMAX_=IZMAX/10;


#ifdef UKU_220_IPS_TERMOKOMPENSAT
if((REG_SPEED<1)||(REG_SPEED>5)) REG_SPEED=1;
if(ch_cnt0<(10*REG_SPEED))
	{
	ch_cnt0++;
	if(ch_cnt0>=10*REG_SPEED)
		{
		ch_cnt0=0;
		b1Hz_ch=1;
		}
	}
#endif


if(mess_find_unvol(MESS2CNTRL_HNDL))
	{
	if(mess_data[0]==PARAM_CNTRL_STAT_PLUS)
		{
		cntrl_stat=cntrl_stat_old+mess_data[1];
		}
	else if(mess_data[0]==PARAM_CNTRL_STAT_MINUS)
		{
		cntrl_stat=cntrl_stat_old-mess_data[1];
		}
	else if(mess_data[0]==PARAM_CNTRL_STAT_STEP_DOWN)
		{
		static char cntrlStatIsDownCnt;
		cntrl_stat--;

		if((cntrl_stat<=30)||(load_U<USIGN))
			{
			if(++cntrlStatIsDownCnt==250)mess_send(MESS2KB_HNDL,PARAM_CNTRL_IS_DOWN,0,10);
			}
		else 
			{
			cntrlStatIsDownCnt=0;
			}

		}
	else if(mess_data[0]==PARAM_CNTRL_STAT_SET)
		{
		cntrl_stat=mess_data[1];
		}

	else if(mess_data[0]==PARAM_CNTRL_STAT_FAST_REG)
		{
		#ifdef UKU_220_IPS_TERMOKOMPENSAT
		if(load_U>u_necc)
			{
			cntrl_hndl_plazma=11;
			if(((bps_U-u_necc)>40)&&(cntrl_stat>0))cntrl_stat-=5;
			else if((cntrl_stat)&&b1Hz_ch)cntrl_stat--;
			}
		else if(bps_U<u_necc)
			{
			cntrl_hndl_plazma=12;	
			if(((u_necc-bps_U)>40)&&(cntrl_stat<2015))cntrl_stat+=5;
			else	if((cntrl_stat<2020)&&b1Hz_ch)cntrl_stat++;
			}
		#endif	
	 	}
	}

#ifdef UKU_VD
else if((b1Hz_ch)&&((!bIBAT_SMKLBR)||(bps[8]._cnt>40)))
	{
	cntrl_stat_new=cntrl_stat_old;
	cntrl_hndl_plazma=19;
	if((Ibmax/10)>(2*IZMAX_))
		{
		cntrl_hndl_plazma=20;
          if(cntrl_stat_blok_cnt)cntrl_stat_new--;
		else	cntrl_stat_new-=10;
		}		
	else if(((Ibmax/10)<(IZMAX_*2))&&(Ibmax>(IZMAX_*15)))
		{
		cntrl_hndl_plazma=21;
          if(cntrl_stat_blok_cnt)cntrl_stat_new--;
          else	cntrl_stat_new-=3;
		}   
	else if((Ibmax<(IZMAX_*15))&&((Ibmax/10)>IZMAX_))
		{
		cntrl_hndl_plazma=22;
		cntrl_stat_new--;
		}
		
	else if(bps_U<u_necc)
		{
		cntrl_hndl_plazma=23;
/*		if(bps_U<(u_necc-(UB0-UB20)))
			{
			cntrl_hndl_plazma=24;
			if(Ibmax<0)
				{
				cntrl_hndl_plazma=25;
                    if(cntrl_stat_blok_cnt)cntrl_stat_new++;
				else cntrl_stat_new+=10;
				}
			else if(Ibmax<(IZMAX_*5))
				{
				cntrl_hndl_plazma=26;
                    if(cntrl_stat_blok_cnt)cntrl_stat_new++;
				else	cntrl_stat_new+=2;
				}
			else if(Ibmax<((IZMAX_*10)))//(Ibmax<((IZMAX_*95)/10))
				{
				cntrl_hndl_plazma=27;
				cntrl_stat_new++;
				}					
			}	*/
/*		else if(bps_U<(u_necc-((UB0-UB20)/4)))
			{
			cntrl_hndl_plazma=28;
			if(Ibmax<(IZMAX_*5))
				{
				cntrl_hndl_plazma=29;
                    if(cntrl_stat_blok_cnt)cntrl_stat_new++;
				else	cntrl_stat_new+=2;
				}
			else if(Ibmax<((IZMAX_*10)))//(Ibmax<((IZMAX_*95)/10))
				{
				cntrl_hndl_plazma=30;
				cntrl_stat_new++;
				}					
			}
		else if(bps_U<(u_necc-1))
			{
			cntrl_hndl_plazma=31;
			if(Ibmax<((IZMAX_*10)))//(Ibmax<((IZMAX_*95)/10))
				{
				cntrl_hndl_plazma=32;
				cntrl_stat_new++;
				}					
			}	*/					
		}	
	else if((bps_U>u_necc)/*&&(!cntrl_blok)*/)
		{ 	
		cntrl_hndl_plazma=33;
/*		if(bps_U>(u_necc+(UB0-UB20)))
			{
			cntrl_hndl_plazma=34;
               if(cntrl_stat_blok_cnt)cntrl_stat_new--;
			else	cntrl_stat_new-=10;
			}
		else if(bps_U>(u_necc+((UB0-UB20)/4)))
			{
			cntrl_hndl_plazma=35;
               if(cntrl_stat_blok_cnt)cntrl_stat_new--;
			else cntrl_stat_new-=2;
			}	
		else if(bps_U>(u_necc+1))
			{
			cntrl_hndl_plazma=36;
			cntrl_stat_new--;
			}*/					
		}

	cntrl_stat_new=(short)((2000L*((long)(UOUT_-in_U+30)))/650L);
	if(((signed long)(UOUT_-in_U+30))<0L)cntrl_stat_new=0;
	gran(&cntrl_stat_new,10,2010);			
	cntrl_stat_old=cntrl_stat_new;
	cntrl_stat=cntrl_stat_new;
	cntrl_stat_buff[cntrl_stat_buff_ptr]=cntrl_stat_new;  //pwm_u_buff[pwm_u_buff_ptr]=pwm_u_;
	cntrl_stat_buff_ptr++;								   //pwm_u_buff_ptr++;
	
	if(cntrl_stat_buff_ptr>=16)cntrl_stat_buff_ptr=0;	//if(pwm_u_buff_ptr>=16)pwm_u_buff_ptr=0;
		{
		char i;
		signed long tempSL;
		tempSL=0;
		for(i=0;i<16;i++)
			{
			tempSL+=(signed long)cntrl_stat_buff[i];			//tempSL+=(signed long)pwm_u_buff[i];
			}
		tempSL>>=4;
		cntrl_stat_buff_=(signed short)tempSL;
		}
	//if(NUMSK)cntrl_stat=cntrl_stat_buff_;

	}

#endif

iiii=0;
for(i=0;i<NUMIST;i++)
     {

	 //bps[i]._x_avg=0;

     /*if(bps[i]._cnt<30)iiii=1;*/

	 bps[i]._x_=_x_reg+bps[i]._x_avg;
	 //if(bps[i]._flags_tu&0x01) bps[i]._cntrl_stat=0;
     }

/*if(iiii==0)
     {
     cntrl_stat=1200;	
     cntrl_stat_old=1200;
     cntrl_stat_new=1200;
     }
gran(&cntrl_stat,10,2010); */

//if(main_1Hz_cnt<10)	cntrl_stat=2000;

if(++_x_reg_cnt>=100)
	{
	_x_reg_cnt=0;

	if((out_U<UOUT)&&(vd_U>20))
		{
		_x_reg++;
		}
	else if((out_U>UOUT)&&(vd_U>20))
		{
		_x_reg--;
		}
	}
gran(&_x_reg,-50,50);
gran(&_x_reg,UBPSMINPWM-UOUT,UBPSMAXPWM-UOUT);

if(mess_data[0]==PARAM_CNTRL_STAT_SET)
		{
		if(cntrl_stat==1020)_x_reg=5000;
		if(cntrl_stat==30)_x_reg=-5000;
		}

b1Hz_ch=0;
}


//-----------------------------------------------
void ext_drv(void)
{
char i;


for(i=0;i<NUMSK;i++)
	{
	#ifdef UKU_MGTS
	if(adc_buff_[sk_buff_RSTKM[i]]<2000)
	#endif
	#ifdef UKU_RSTKM
	if(adc_buff_[sk_buff_RSTKM[i]]<2000)
	#endif
	#ifdef UKU_3U
	if(adc_buff_[sk_buff_3U[i]]<2000)
	#endif
	#ifdef UKU_GLONASS
	if(adc_buff_[sk_buff_GLONASS[i]]<2000)
	#endif
	#ifdef UKU_KONTUR
	if(adc_buff_[sk_buff_KONTUR[i]]<2000)
	#endif
	#ifdef UKU_6U
	if(adc_buff_[sk_buff_6U[i]]<2000)
	#endif
	#ifdef UKU_220
	if(adc_buff_[sk_buff_220[i]]<2000)
	#endif
	#ifdef UKU_220_V2
	if(adc_buff_[sk_buff_220[i]]<2000)
	#endif
	#ifdef UKU_220_IPS_TERMOKOMPENSAT
	if(adc_buff_[sk_buff_220[i]]<2000)
	#endif
	#ifdef UKU_TELECORE2015	
	if(adc_buff_[sk_buff_TELECORE2015[i]]<2000)	 //TODO
	#endif
	#ifdef UKU_TELECORE2017
	if(adc_buff_[sk_buff_TELECORE2015[i]]<2000)	 //TODO
	#endif		
		{
		if(sk_cnt[i]<10)
			{
			sk_cnt[i]++;
			if(sk_cnt[i]>=10)
				{
				sk_stat[i]=ssON;
				}
			}
		else 
			{
			sk_cnt[i]=10;
			}
               
		}
	else
		{
		if(sk_cnt[i]>0)
			{
			sk_cnt[i]--;
			if(sk_cnt[i]<=0)
				{
				sk_stat[i]=ssOFF;
				}
			}
		else 
			{
			sk_cnt[i]=0;
			}
		}
	}

for(i=0;i<NUMSK;i++)
	{
	if(((SK_SIGN[i]==0)&&(sk_stat[i]==ssON))||((SK_SIGN[i])&&(sk_stat[i]==ssOFF)) )
		{
		if(sk_av_cnt[i]<10)
			{
			sk_av_cnt[i]++;
			if(sk_av_cnt[i]>=10)
				{
				sk_av_stat[i]=sasON;
				}
			}
		else 
			{
			sk_av_cnt[i]=10;
			}
		}
	else
		{
		if(sk_av_cnt[i]>=0)
			{
			sk_av_cnt[i]--;
			if(sk_av_cnt[i]<=0)
				{
				sk_av_stat[i]=sasOFF;
				}
			}
		else 
			{
			sk_av_cnt[i]=0;
			}
		}

#ifndef UKU_KONTUR
	if(sk_av_stat_old[i]!=sk_av_stat[i])
		{
		plazma_sk++;
		if(sk_av_stat[i]==sasON)
			{
			if(i==0)snmp_trap_send("SK #1 Alarm",15,1,1);
			else if(i==1)
				{
				#ifndef UKU_TELEKORE2017
				snmp_trap_send("SK #2 Alarm",15,2,1);
				#endif
				#ifdef UKU_TELEKORE2017
				snmp_trap_send("Door open",15,2,1);
				#endif
				}
			else if(i==2)snmp_trap_send("SK #3 Alarm",15,3,1);
			else if(i==3)snmp_trap_send("SK #4 Alarm",15,4,1);
			}
		else 
			{
			if(i==0)snmp_trap_send("SK #1 Alarm is off",15,1,0);
			else if(i==1)
				{
				#ifndef UKU_TELEKORE2017
				snmp_trap_send("SK #2 Alarm is off",15,2,0);
				#endif
				#ifdef UKU_TELEKORE2017
				snmp_trap_send("Door open clear",15,2,0);
				#endif
				}
			else if(i==2)snmp_trap_send("SK #3 Alarm is off",15,3,0);
			else if(i==3)snmp_trap_send("SK #4 Alarm is off",15,4,0);
			}
	 	}
#endif

#ifdef UKU_KONTUR
	if(sk_av_stat_old[i]!=sk_av_stat[i])
		{
		plazma_sk++;
		if(sk_av_stat[i]==sasON)
			{
			if(i==0)snmp_trap_send("Door is opened",15,1,1);
			else if(i==1)snmp_trap_send("Smoke Alarm",15,2,1);
			else if(i==2)snmp_trap_send("Shock Sensor Alarm",15,3,1);
			else if(i==3)snmp_trap_send("SK #4 Alarm",15,4,1);
			}
		else 
			{
			if(i==0)snmp_trap_send("Door is closed",15,1,0);
			else if(i==1)snmp_trap_send("Smoke Alarm is off",15,2,0);
			else if(i==2)snmp_trap_send("Shock Sensor Alarm is off",15,3,0);
			else if(i==3)snmp_trap_send("SK #4 Alarm is off",15,4,0);
			}
	 	}
#endif
	sk_av_stat_old[i]=sk_av_stat[i];
	}
}


//-----------------------------------------------
void zar_superviser_drv(void)
{

if(((bat[0]._Ub>u_necc_up) || (bat[0]._Ub<u_necc_dn))&&(main_kb_cnt<((TBAT*60)-30))&&(main_kb_cnt>((TBAT*60)-150))) sign_U[0]=0;

if(((bat[0]._Ib>(2*IKB)) || (bat[0]._Ib<(-IKB*2))) &&(main_kb_cnt<((TBAT*60)-30))&&(main_kb_cnt>((TBAT*60)-150))) sign_I[0]=0;
																 
if((main_kb_cnt==((TBAT*60)-10)) &&(spc_stat==spcOFF))
	{
	if((sign_U[0]==1) && (sign_I[0]==1) && (lc640_read_int(EE_BAT1_ZAR_CNT)!=BAT_C_REAL[0]) && (NUMBAT) && (!(bat[0]._av&1)))
		{
		lc640_write_int(EE_BAT1_ZAR_CNT,BAT_C_REAL[0]);
		superviser_cnt++;
		}
	
	}

if(((bat[0]._Ub>u_necc_up) || (bat[1]._Ub<u_necc_dn))&&(main_kb_cnt<((TBAT*60)-30))&&(main_kb_cnt>((TBAT*60)-150))) sign_U[1]=0;

if(((bat[1]._Ib>(2*IKB)) || (bat[1]._Ib<(-IKB*2))) &&(main_kb_cnt<((TBAT*60)-30))&&(main_kb_cnt>((TBAT*60)-150))) sign_I[1]=0;
																 
if((main_kb_cnt==((TBAT*60)-10)) &&(spc_stat==spcOFF))
	{
	if((sign_U[1]==1) && (sign_I[1]==1) && (lc640_read_int(EE_BAT2_ZAR_CNT)!=BAT_C_REAL[1]) && (NUMBAT==2) && (!(bat[1]._av&1)))
		{
		lc640_write_int(EE_BAT2_ZAR_CNT,BAT_C_REAL[1]);
		superviser_cnt++;
		}
	
	}

if(main_kb_cnt==((TBAT*60)-2)) zar_superviser_start();
}

//-----------------------------------------------
void zar_superviser_start(void)
{
sign_U[0]=1;
sign_I[0]=1;
sign_U[1]=1;
sign_I[1]=1;

}

//-----------------------------------------------
void npn_hndl(void)
{
if(NPN_OUT!=npnoOFF)
	{
/*	if(NPN_SIGN==npnsAVNET)
		{
		if(net_av==1)
			{
			if(npn_tz_cnt<TZNPN)
				{
				npn_tz_cnt++;
				if(npn_tz_cnt==TZNPN)
					{
					npn_stat=npnsOFF;
					}
				}
			}
		else
			{
			if(npn_tz_cnt)
				{
				npn_tz_cnt--;
				if(npn_tz_cnt==0)
					{
					npn_stat=npnsON;
					}
				}
			}
		}*/
/*	if(NPN_SIGN==npnsULOAD)
		{
		if(load_U<UONPN)
			{
			if(npn_tz_cnt<TZNPN)
				{
				npn_tz_cnt++;
				if(npn_tz_cnt==TZNPN)
					{
					npn_stat=npnsOFF;
					}
				}
			}
		else if(load_U>UVNPN)
			{
			if(npn_tz_cnt)
				{
				npn_tz_cnt--;
				if(npn_tz_cnt==0)
					{
					npn_stat=npnsON;
					}
				}
			}
		}*/

	if((load_U<UONPN)&&((net_Ua<UMN)||(net_Ub<UMN)||(net_Uc<UMN)))
		{
		if(npn_tz_cnt<TZNPN)
			{
			npn_tz_cnt++;
			if(npn_tz_cnt==TZNPN)
				{
				npn_stat=npnsOFF;
				}
			}
		}
	else if((load_U>UVNPN)&&(net_Ua>UMN)&&(net_Ub>UMN)&&(net_Uc>UMN))
		{
		if(npn_tz_cnt)
			{
			npn_tz_cnt--;
			if(npn_tz_cnt==0)
				{
				npn_stat=npnsON;
				}
			}
		}
	}
else
	{
	npn_tz_cnt=0;
	npn_stat=npnsON;
	}

if(npn_stat==npnsOFF) mess_send(MESS2RELE_HNDL,PARAM_RELE_NPN,1,15);


}

//-----------------------------------------------
void speedChargeHndl(void)
{
if(speedChIsOn)
	{
	speedChTimeCnt++;
	if(speedChTimeCnt>=((signed long)speedChrgTimeInHour*3600L))
		{
		speedChIsOn=0;
		}
	if(speedChrgBlckStat)
		{
		speedChIsOn=0;
		speedChTimeCnt=0;
		}
	}



if(speedChrgAvtEn)
	{
	if(!speedChIsOn)
		{
		if((load_U<u_necc)&&((u_necc-load_U)>speedChrgDU)&&(abs(Ib_ips_termokompensat/10-IZMAX)<5)&&(!speedChrgBlckStat))
			{
			speedChIsOn=1;
			}
		}
	}



if((speedChrgBlckSrc!=1)&&(speedChrgBlckSrc!=2)) speedChrgBlckStat=0;
else
	{
	speedChrgBlckStat=0;
	if(speedChrgBlckSrc==1)
		{
		if(((speedChrgBlckLog==0)&&(adc_buff_[11]>2000)) || ((speedChrgBlckLog==1)&&(adc_buff_[11]<2000))) speedChrgBlckStat=1;
		}
	else if(speedChrgBlckSrc==2)
		{
		if(((speedChrgBlckLog==0)&&(adc_buff_[13]>2000)) || ((speedChrgBlckLog==1)&&(adc_buff_[13]<2000))) speedChrgBlckStat=1;
		}
	}


if(speedChrgBlckStat==1)
	{

	//speedChargeStartStop();

	speedChrgShowCnt++;
	if(speedChrgShowCnt>=30)	
		{
		speedChrgShowCnt=0;
		show_mess(	"     ����������     ",
					"       �����        ",
					"     ��������!!!    ",
					"                    ",
					5000);
		}
	}
else speedChrgShowCnt=0;


}

//-----------------------------------------------
void speedChargeStartStop(void)
{
if(speedChIsOn)
	{
	speedChIsOn=0;
	}

else
	{
	if(speedChrgBlckStat==0)
		{
		speedChIsOn=1;
		speedChTimeCnt=0;
		}
	else
		{
		show_mess(	"     ����������     ",
	          		"       �����        ",
	          		"    ������������!   ",
	          		"                    ",2000);	 
		}
	}
}

//-----------------------------------------------
void	numOfForvardBps_hndl(void)			//��������� ����� ������� ��������� ��� ������������ ������ �����
{

numOfForvardBps_old=numOfForvardBps;

numOfForvardBps=0;

//FORVARDBPSCHHOUR=10;

if((FORVARDBPSCHHOUR<=0)||(FORVARDBPSCHHOUR>500))
	{
	FORVARDBPSCHHOUR=0;
	return;
	}

numOfForvardBps_minCnt++;


if(numOfForvardBps_minCnt>=60)
	{
	numOfForvardBps_minCnt=0;
	numOfForvardBps_hourCnt=lc640_read_int(EE_FORVBPSHOURCNT);
	numOfForvardBps_hourCnt++;
	if(numOfForvardBps_hourCnt>=(FORVARDBPSCHHOUR*NUMIST))
		{
		numOfForvardBps_hourCnt=0;
		}
	lc640_write_int(EE_FORVBPSHOURCNT,numOfForvardBps_hourCnt);
	}

numOfForvardBps=numOfForvardBps_hourCnt/FORVARDBPSCHHOUR;

//if(numOfForvardBps)
//numOfForvardBps_old=numOfForvardBps; 
}

//-----------------------------------------------
void	numOfForvardBps_init(void)			//��������� ������ ������� ����� ������� ��������� ��� ������������ ������ �����
{									//������ ���������� ��� ��������� ���-�� ���������� � ���������
lc640_write_int(EE_FORVBPSHOURCNT,0);
numOfForvardBps_minCnt=58;
}

/*
//-----------------------------------------------
void outVoltContrHndl(void)
{ 
if((load_U>U_OUT_KONTR_MAX)||(load_U<U_OUT_KONTR_MIN))
	{
	outVoltContrHndlCnt_=0;
	if(outVoltContrHndlCnt<U_OUT_KONTR_DELAY)
		{
		outVoltContrHndlCnt++;
		if(outVoltContrHndlCnt==U_OUT_KONTR_DELAY)
			{
			avar_uout_hndl(1);
			}
		}
	}
else
	{
	if(outVoltContrHndlCnt)
		{
		if(outVoltContrHndlCnt_<5)
			{
			outVoltContrHndlCnt_++;
			if(outVoltContrHndlCnt_>=5)
				{
				outVoltContrHndlCnt=0;
				if(uout_av)avar_uout_hndl(0);
				}
			}
		}
	}

if (load_U<(USIGN*10)) 
	{
	if(!bSILENT)
		{
		mess_send(MESS2RELE_HNDL,PARAM_RELE_BAT_IS_DISCHARGED,1,20);
		}

	//bU_BAT2REL_AV_BAT=1;
	}


}
*/
//-----------------------------------------------
void vent_resurs_hndl(void)
{
char i;
char crc_in,crc_eval;

for(i=0;i<NUMIST;i++)
	{
	if((bps[i]._buff[7]&0xc0)==0x00)
		{
		bps[i]._vent_resurs_temp[0]=bps[i]._buff[7];
		}
	else if((bps[i]._buff[7]&0xc0)==0x40)
		{
		bps[i]._vent_resurs_temp[1]=bps[i]._buff[7];
		}
	else if((bps[i]._buff[7]&0xc0)==0x80)
		{
		bps[i]._vent_resurs_temp[2]=bps[i]._buff[7];
		}
	else if((bps[i]._buff[7]&0xc0)==0xc0)
		{
		bps[i]._vent_resurs_temp[3]=bps[i]._buff[7];
		}
	crc_in=0;
	crc_in|=(bps[i]._vent_resurs_temp[0]&0x30)>>4;
	crc_in|=(bps[i]._vent_resurs_temp[1]&0x30)>>2;
	crc_in|=(bps[i]._vent_resurs_temp[2]&0x30);
	crc_in|=(bps[i]._vent_resurs_temp[3]&0x30)<<2;

	crc_eval =bps[i]._vent_resurs_temp[0]&0x0f;
	crc_eval^=bps[i]._vent_resurs_temp[1]&0x0f;
	crc_eval^=bps[i]._vent_resurs_temp[2]&0x0f;
	crc_eval^=bps[i]._vent_resurs_temp[3]&0x0f;

	if(crc_eval==crc_in)
		{
		unsigned short temp_US;
		temp_US=0;

		temp_US|=(bps[i]._vent_resurs_temp[3]&0x0f);
		temp_US<<=4;
		temp_US|=(bps[i]._vent_resurs_temp[2]&0x0f);
		temp_US<<=4;
		temp_US|=(bps[i]._vent_resurs_temp[1]&0x0f);
		temp_US<<=4;
		temp_US|=(bps[i]._vent_resurs_temp[0]&0x0f);

		if(bps[i]._vent_resurs!=temp_US)bps[i]._vent_resurs=temp_US;
		}

	if(bps[i]._vent_resurs>TVENTMAX*10)
		{
		bps[i]._av|=(1<<4);
		}
	else bps[i]._av&=~(1<<4);
	}
}

//-----------------------------------------------
void vent_hndl(void)
{
if(RELEVENTSIGN==rvsAKB)
	{
	if(vent_stat==0)
		{
		if	(
			(BAT_IS_ON[0]==bisON)&&((bat[0]._Tb>TVENTON)||(bat[0]._nd))
			||
			(BAT_IS_ON[1]==bisON)&&((bat[1]._Tb>TVENTON)||(bat[1]._nd))
			)
			{
			vent_stat=1;
			}
		}
	else if(vent_stat==1)
		{
		if	(
			((BAT_IS_ON[0]!=bisON)||((BAT_IS_ON[0]==bisON)&&(bat[0]._Tb<TVENTOFF)&&(!bat[0]._nd)))
			&&
			((BAT_IS_ON[1]!=bisON)||((BAT_IS_ON[1]==bisON)&&(bat[1]._Tb<TVENTOFF)&&(!bat[1]._nd)))
			)
			{
			vent_stat=0;
			}
		}
	}
else if(RELEVENTSIGN==rvsBPS)
	{
/*	if	(
		(((bps[0]._flags_tm&0x06)||(bps[0]._cnt>=30)))||
		(((bps[1]._flags_tm&0x06)||(bps[1]._cnt>=30))&&(NUMIST>1))||
		(((bps[2]._flags_tm&0x06)||(bps[2]._cnt>=30))&&(NUMIST>2))||
		(((bps[3]._flags_tm&0x06)||(bps[3]._cnt>=30))&&(NUMIST>3))||
		(((bps[4]._flags_tm&0x06)||(bps[4]._cnt>=30))&&(NUMIST>4))||
		(((bps[5]._flags_tm&0x06)||(bps[5]._cnt>=30))&&(NUMIST>5))||
		(((bps[6]._flags_tm&0x06)||(bps[6]._cnt>=30))&&(NUMIST>6))
		)
		{
		vent_stat=1;
		}
	else vent_stat=0;
	*/

	if	(
		((NUMIST)&&((bps[0]._Ti>TVENTON)||(bps[0]._cnt>=30)))
		||
		((NUMIST>1)&&((bps[1]._Ti>TVENTON)||(bps[1]._cnt>=30)))
		||
		((NUMIST>2)&&((bps[2]._Ti>TVENTON)||(bps[2]._cnt>=30)))
		||
		((NUMIST>3)&&((bps[3]._Ti>TVENTON)||(bps[3]._cnt>=30)))
		||
		((NUMIST>4)&&((bps[4]._Ti>TVENTON)||(bps[4]._cnt>=30)))
		||
		((NUMIST>5)&&((bps[5]._Ti>TVENTON)||(bps[5]._cnt>=30)))
		||
		((NUMIST>6)&&((bps[6]._Ti>TVENTON)||(bps[6]._cnt>=30)))
		||
		((NUMIST>7)&&((bps[7]._Ti>TVENTON)||(bps[7]._cnt>=30)))
		)
		{
		vent_stat=1;
		}
	else if(vent_stat==1)
		{
		if	(
			((!NUMIST)||((NUMIST)&&(bps[0]._Ti<TVENTOFF)&&(bps[0]._cnt<10)))
			&&
			((NUMIST<2)||((NUMIST>=2)&&(bps[1]._Ti<TVENTOFF)&&(bps[1]._cnt<10)))
			&&
			((NUMIST<3)||((NUMIST>=3)&&(bps[2]._Ti<TVENTOFF)&&(bps[2]._cnt<10)))
			&&
			((NUMIST<4)||((NUMIST>=4)&&(bps[3]._Ti<TVENTOFF)&&(bps[3]._cnt<10)))
			&&
			((NUMIST<5)||((NUMIST>=5)&&(bps[4]._Ti<TVENTOFF)&&(bps[4]._cnt<10)))
			&&
			((NUMIST<6)||((NUMIST>=6)&&(bps[5]._Ti<TVENTOFF)&&(bps[5]._cnt<10)))
			&&
			((NUMIST<7)||((NUMIST>=7)&&(bps[6]._Ti<TVENTOFF)&&(bps[6]._cnt<10)))
			&&
			((NUMIST<8)||((NUMIST>=8)&&(bps[7]._Ti<TVENTOFF)&&(bps[7]._cnt<10)))
			)
			{
			vent_stat=0;
			}
		}
	}
else if(RELEVENTSIGN==rvsEXT)
	{
	if	(
		((NUMDT)&&((t_ext[0]>TVENTON)||(ND_EXT[0])))
		||
		((NUMDT>1)&&((t_ext[1]>TVENTON)||(ND_EXT[1])))
		||
		((NUMDT>2)&&((t_ext[2]>TVENTON)||(ND_EXT[2])))
		)
		{
		vent_stat=1;
		}
	else if(vent_stat==1)
		{
		if	(
			((!NUMDT)||((NUMDT)&&(t_ext[0]<TVENTOFF)&&(!ND_EXT[0])))
			&&
			((NUMDT<2)||((NUMDT>=2)&&(t_ext[1]<TVENTOFF)&&(!ND_EXT[1])))
			&&
			((NUMDT<3)||((NUMDT>=3)&&(t_ext[2]<TVENTOFF)&&(!ND_EXT[2])))
			)
			{
			vent_stat=0;
			}
		}
	}
else vent_stat=1;
}

//-----------------------------------------------
void vd_is_work_hndl(void)
{
if(vd_U>20)
	{
	if(vd_is_work_cnt<5)vd_is_work_cnt++;
	else vd_is_work_cnt=5;
	}
else
	{
	if(vd_is_work_cnt>0)vd_is_work_cnt--;
	else vd_is_work_cnt=0;
	}

if(vd_is_work_cnt==5) bVDISWORK=1;
else if(vd_is_work_cnt==0) bVDISWORK=0;
}

