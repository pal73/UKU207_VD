#include "MODBUS_func4.h"
//#include "akb_f.h"
#include "curr_version.h"
#include "main.h"
#include "MODBUS_RTU.h"
signed short tmp_oleg4=0x2569;

//----------- ������� ������� ����� 3
//������ ������������� �� �������, ���� ��� ��������, �� &NULL_0, &NULL_0,
unsigned char *const reg_func4 []={
&NULL_0,  //0
&NULL_0,  //0
(unsigned char*)&bps[0]._Uii+1,			//���1	�������� ���������� ����������� �1, 0.1�
(unsigned char*)&bps[0]._Uii,
(unsigned char*)&bps[0]._Ii+1,			//���2	�������� ��� ����������� �1, 0.1�
(unsigned char*)&bps[0]._Ii,
(unsigned char*)&bps[0]._Ti+1,			//���3	����������� ��������� ����������� �1, 1��
(unsigned char*)&bps[0]._Ti,
(unsigned char*)&bps[0]._av+1,			//���4	���� ������ ����������� �1, 0x01 - ��������, 0x02 �������� U���, 0x04 �������� U���, 0x08 - ����������� ����� � ������������
(unsigned char*)&bps[0]._av,
(unsigned char*)&bps[1]._Uii+1,			//���5	�������� ���������� ����������� �2, 0.1�
(unsigned char*)&bps[1]._Uii,
(unsigned char*)&bps[1]._Ii+1,			//���6	�������� ��� ����������� �2, 0.1�
(unsigned char*)&bps[1]._Ii,
(unsigned char*)&bps[1]._Ti+1,			//���7	����������� ��������� ����������� �2, 1��
(unsigned char*)&bps[1]._Ti,
(unsigned char*)&bps[1]._av+1,			//���8	���� ������ ����������� �2, 0x01 - ��������, 0x02 �������� U���, 0x04 �������� U���, 0x08 - ����������� ����� � ������������
(unsigned char*)&bps[1]._av,
(unsigned char*)&bps[2]._Uii+1,			//���9	�������� ���������� ����������� �3, 0.1�
(unsigned char*)&bps[2]._Uii,
(unsigned char*)&bps[2]._Ii+1,			//���10	�������� ��� ����������� �3, 0.1�
(unsigned char*)&bps[2]._Ii,
(unsigned char*)&bps[2]._Ti+1,			//���11	����������� ��������� ����������� �3, 1��
(unsigned char*)&bps[2]._Ti,
(unsigned char*)&bps[2]._av+1,			//���12	���� ������ ����������� �3, 0x01 - ��������, 0x02 �������� U���, 0x04 �������� U���, 0x08 - ����������� ����� � ������������
(unsigned char*)&bps[2]._av,
(unsigned char*)&bps[3]._Uii+1,			//���13	�������� ���������� ����������� �4, 0.1�
(unsigned char*)&bps[3]._Uii,
(unsigned char*)&bps[3]._Ii+1,			//���14	�������� ��� ����������� �4, 0.1�
(unsigned char*)&bps[3]._Ii,
(unsigned char*)&bps[3]._Ti+1,			//���15	����������� ��������� ����������� �4, 1��
(unsigned char*)&bps[3]._Ti,
(unsigned char*)&bps[3]._av+1,			//���16	���� ������ ����������� �4, 0x01 - ��������, 0x02 �������� U���, 0x04 �������� U���, 0x08 - ����������� ����� � ������������
(unsigned char*)&bps[3]._av,
(unsigned char*)&bps[4]._Uii+1,			//���17	�������� ���������� ����������� �5, 0.1�
(unsigned char*)&bps[4]._Uii,
(unsigned char*)&bps[4]._Ii+1,			//���18	�������� ��� ����������� �5, 0.1�
(unsigned char*)&bps[4]._Ii,
(unsigned char*)&bps[4]._Ti+1,			//���19	����������� ��������� ����������� �5, 1��
(unsigned char*)&bps[4]._Ti,
(unsigned char*)&bps[4]._av+1,			//���20	���� ������ ����������� �5, 0x01 - ��������, 0x02 �������� U���, 0x04 �������� U���, 0x08 - ����������� ����� � ������������
(unsigned char*)&bps[4]._av,
(unsigned char*)&bps[5]._Uii+1,			//���21	�������� ���������� ����������� �6, 0.1�
(unsigned char*)&bps[5]._Uii,
(unsigned char*)&bps[5]._Ii+1,			//���22	�������� ��� ����������� �6, 0.1�
(unsigned char*)&bps[5]._Ii,
(unsigned char*)&bps[5]._Ti+1,			//���23	����������� ��������� ����������� �6, 1��
(unsigned char*)&bps[5]._Ti,
(unsigned char*)&bps[5]._av+1,			//���24	���� ������ ����������� �6, 0x01 - ��������, 0x02 �������� U���, 0x04 �������� U���, 0x08 - ����������� ����� � ������������
(unsigned char*)&bps[5]._av,
(unsigned char*)&bps[6]._Uii+1,			//���25	�������� ���������� ����������� �7, 0.1�
(unsigned char*)&bps[6]._Uii,
(unsigned char*)&bps[6]._Ii+1,			//���26	�������� ��� ����������� �7, 0.1�
(unsigned char*)&bps[6]._Ii,
(unsigned char*)&bps[6]._Ti+1,			//���27	����������� ��������� ����������� �7, 1��
(unsigned char*)&bps[6]._Ti,
(unsigned char*)&bps[6]._av+1,			//���28	���� ������ ����������� �7, 0x01 - ��������, 0x02 �������� U���, 0x04 �������� U���, 0x08 - ����������� ����� � ������������
(unsigned char*)&bps[6]._av,
(unsigned char*)&bps[7]._Uii+1,			//���29	�������� ���������� ����������� �8, 0.1�
(unsigned char*)&bps[7]._Uii,
(unsigned char*)&bps[7]._Ii+1,			//���30	�������� ��� ����������� �8, 0.1�
(unsigned char*)&bps[7]._Ii,
(unsigned char*)&bps[7]._Ti+1,			//���31	����������� ��������� ����������� �8, 1��
(unsigned char*)&bps[7]._Ti,
(unsigned char*)&bps[7]._av+1,			//���32	���� ������ ����������� �8, 0x01 - ��������, 0x02 �������� U���, 0x04 �������� U���, 0x08 - ����������� ����� � ������������
(unsigned char*)&bps[7]._av,
(unsigned char*)&bps[8]._Uii+1,			//���33	�������� ���������� ����������� �9, 0.1�
(unsigned char*)&bps[8]._Uii,
(unsigned char*)&bps[8]._Ii+1,			//���34	�������� ��� ����������� �9, 0.1�
(unsigned char*)&bps[8]._Ii,
(unsigned char*)&bps[8]._Ti+1,			//���35	����������� ��������� ����������� �9, 1��
(unsigned char*)&bps[8]._Ti,
(unsigned char*)&bps[8]._av+1,			//���36	���� ������ ����������� �9, 0x01 - ��������, 0x02 �������� U���, 0x04 �������� U���, 0x08 - ����������� ����� � ������������
(unsigned char*)&bps[8]._av,
(unsigned char*)&bps[9]._Uii+1,			//���37	�������� ���������� ����������� �10, 0.1�
(unsigned char*)&bps[9]._Uii,
(unsigned char*)&bps[9]._Ii+1,			//���38	�������� ��� ����������� �10, 0.1�
(unsigned char*)&bps[9]._Ii,
(unsigned char*)&bps[9]._Ti+1,			//���39	����������� ��������� ����������� �10, 1��
(unsigned char*)&bps[9]._Ti,
(unsigned char*)&bps[9]._av+1,			//���40	���� ������ ����������� �10, 0x01 - ��������, 0x02 �������� U���, 0x04 �������� U���, 0x08 - ����������� ����� � ������������
(unsigned char*)&bps[9]._av,
(unsigned char*)&bps[10]._Uii+1,		//���41	�������� ���������� ����������� �11, 0.1�
(unsigned char*)&bps[10]._Uii,
(unsigned char*)&bps[10]._Ii+1,			//���42	�������� ��� ����������� �11, 0.1�
(unsigned char*)&bps[10]._Ii,
(unsigned char*)&bps[10]._Ti+1,			//���43	����������� ��������� ����������� �11, 1��
(unsigned char*)&bps[10]._Ti,
(unsigned char*)&bps[10]._av+1,			//���44	���� ������ ����������� �11, 0x01 - ��������, 0x02 �������� U���, 0x04 �������� U���, 0x08 - ����������� ����� � ������������
(unsigned char*)&bps[10]._av,
(unsigned char*)&bps[11]._Uii+1,		//���45	�������� ���������� ����������� �12, 0.1�
(unsigned char*)&bps[11]._Uii,
(unsigned char*)&bps[11]._Ii+1,			//���46	�������� ��� ����������� �12, 0.1�
(unsigned char*)&bps[11]._Ii,
(unsigned char*)&bps[11]._Ti+1,			//���47	����������� ��������� ����������� �12, 1��
(unsigned char*)&bps[11]._Ti,
(unsigned char*)&bps[11]._av+1,			//���48	���� ������ ����������� �12, 0x01 - ��������, 0x02 �������� U���, 0x04 �������� U���, 0x08 - ����������� ����� � ������������
(unsigned char*)&bps[11]._av,
&NULL_0,  //49
&NULL_0,															
(unsigned char*)&out_U+1,				//���50 �������� ���������� ������� 0.1v
(unsigned char*)&out_U,
(unsigned char*)&in_U+1,				//���51	������� ���������� ������� 0.1v
(unsigned char*)&in_U,
(unsigned char*)&vd_U+1,				//���52	���������� ������������ 0.1v
(unsigned char*)&vd_U,
(unsigned char*)&Ib_ips_termokompensat+1,		//���53	�������� ��� 1A
(unsigned char*)&Ib_ips_termokompensat,

(unsigned char*)&t_ext[0]+1,				//���54	 ����������� �������  1C
(unsigned char*)&t_ext[0],
(unsigned char*)&avar_vd_stat+1,		//���55	 ���� ������ �������
(unsigned char*)&avar_vd_stat,			// ��� 0 - ������ ������ �� ���
															// ��� 1 - ���������� ������� �� ����
															// ��� 2 - �������� �������
															// ��� 3 - �������� ���������� ��������
															// ��� 4 - �������� ���������� ��������
															// ��� 5 - ������� ���������� ��������
															// ��� 6 - ������� ���������� ��������
															// ��� 7 - ������ ����������� ��������
															// ��� 8 - ������������ � ������

&NULL_0,  
&NULL_0,
&NULL_0,  
&NULL_0,
&NULL_0,  
&NULL_0,
&NULL_0,  
&NULL_0,
(unsigned char*)&bps[12]._Uii+1,			//���60	�������� ���������� ����������� �13, 0.1�
(unsigned char*)&bps[12]._Uii,
(unsigned char*)&bps[12]._Ii+1,			//���61	�������� ��� ����������� �13, 0.1�
(unsigned char*)&bps[12]._Ii,
(unsigned char*)&bps[12]._Ti+1,			//���62	����������� ��������� ����������� �13, 1��
(unsigned char*)&bps[12]._Ti,
(unsigned char*)&bps[12]._av+1,			//���63	���� ������ ����������� �13, 0x01 - ��������, 0x02 �������� U���, 0x04 �������� U���, 0x08 - ����������� ����� � ������������
(unsigned char*)&bps[12]._av,
(unsigned char*)&bps[13]._Uii+1,			//���64	�������� ���������� ����������� �14, 0.1�
(unsigned char*)&bps[13]._Uii,
(unsigned char*)&bps[13]._Ii+1,			//���65	�������� ��� ����������� �14, 0.1�
(unsigned char*)&bps[13]._Ii,
(unsigned char*)&bps[13]._Ti+1,			//���66	����������� ��������� ����������� �14, 1��
(unsigned char*)&bps[13]._Ti,
(unsigned char*)&bps[13]._av+1,			//���67	���� ������ ����������� �14, 0x01 - ��������, 0x02 �������� U���, 0x04 �������� U���, 0x08 - ����������� ����� � ������������
(unsigned char*)&bps[13]._av,
(unsigned char*)&bps[14]._Uii+1,			//���68	�������� ���������� ����������� �15, 0.1�
(unsigned char*)&bps[14]._Uii,
(unsigned char*)&bps[14]._Ii+1,			//���69	�������� ��� ����������� �15, 0.1�
(unsigned char*)&bps[14]._Ii,
(unsigned char*)&bps[14]._Ti+1,			//���70	����������� ��������� ����������� �15, 1��
(unsigned char*)&bps[14]._Ti,
(unsigned char*)&bps[14]._av+1,			//���71	���� ������ ����������� �15, 0x01 - ��������, 0x02 �������� U���, 0x04 �������� U���, 0x08 - ����������� ����� � ������������
(unsigned char*)&bps[14]._av,
(unsigned char*)&bps[15]._Uii+1,			//���72	�������� ���������� ����������� �16, 0.1�
(unsigned char*)&bps[15]._Uii,
(unsigned char*)&bps[15]._Ii+1,			//���73	�������� ��� ����������� �16, 0.1�
(unsigned char*)&bps[15]._Ii,
(unsigned char*)&bps[15]._Ti+1,			//���74	����������� ��������� ����������� �16, 1��
(unsigned char*)&bps[15]._Ti,
(unsigned char*)&bps[15]._av+1,			//���75	���� ������ ����������� �16, 0x01 - ��������, 0x02 �������� U���, 0x04 �������� U���, 0x08 - ����������� ����� � ������������
(unsigned char*)&bps[15]._av,
(unsigned char*)&bps[16]._Uii+1,			//���76	�������� ���������� ����������� �17, 0.1�
(unsigned char*)&bps[16]._Uii,
(unsigned char*)&bps[16]._Ii+1,			//���77	�������� ��� ����������� �17, 0.1�
(unsigned char*)&bps[16]._Ii,
(unsigned char*)&bps[16]._Ti+1,			//���78	����������� ��������� ����������� �17, 1��
(unsigned char*)&bps[16]._Ti,
(unsigned char*)&bps[16]._av+1,			//���79	���� ������ ����������� �17, 0x01 - ��������, 0x02 �������� U���, 0x04 �������� U���, 0x08 - ����������� ����� � ������������
(unsigned char*)&bps[16]._av,
(unsigned char*)&bps[17]._Uii+1,			//���80	�������� ���������� ����������� �18, 0.1�
(unsigned char*)&bps[17]._Uii,
(unsigned char*)&bps[17]._Ii+1,			//���81	�������� ��� ����������� �18, 0.1�
(unsigned char*)&bps[17]._Ii,
(unsigned char*)&bps[17]._Ti+1,			//���82	����������� ��������� ����������� �18, 1��
(unsigned char*)&bps[17]._Ti,
(unsigned char*)&bps[17]._av+1,			//���83	���� ������ ����������� �18, 0x01 - ��������, 0x02 �������� U���, 0x04 �������� U���, 0x08 - ����������� ����� � ������������
(unsigned char*)&bps[17]._av,
(unsigned char*)&bps[18]._Uii+1,			//���84	�������� ���������� ����������� �19, 0.1�
(unsigned char*)&bps[18]._Uii,
(unsigned char*)&bps[18]._Ii+1,			//���85	�������� ��� ����������� �19, 0.1�
(unsigned char*)&bps[18]._Ii,
(unsigned char*)&bps[18]._Ti+1,			//���86	����������� ��������� ����������� �19, 1��
(unsigned char*)&bps[18]._Ti,
(unsigned char*)&bps[18]._av+1,			//���87	���� ������ ����������� �19, 0x01 - ��������, 0x02 �������� U���, 0x04 �������� U���, 0x08 - ����������� ����� � ������������
(unsigned char*)&bps[18]._av,
(unsigned char*)&bps[19]._Uii+1,			//���88	�������� ���������� ����������� �20, 0.1�
(unsigned char*)&bps[19]._Uii,
(unsigned char*)&bps[19]._Ii+1,			//���89	�������� ��� ����������� �20, 0.1�
(unsigned char*)&bps[19]._Ii,
(unsigned char*)&bps[19]._Ti+1,			//���90	����������� ��������� ����������� �20, 1��
(unsigned char*)&bps[19]._Ti,
(unsigned char*)&bps[19]._av+1,			//���91	���� ������ ����������� �20, 0x01 - ��������, 0x02 �������� U���, 0x04 �������� U���, 0x08 - ����������� ����� � ������������
(unsigned char*)&bps[19]._av,
(unsigned char*)&bps[20]._Uii+1,			//���92	�������� ���������� ����������� �21, 0.1�
(unsigned char*)&bps[20]._Uii,
(unsigned char*)&bps[20]._Ii+1,			//���93	�������� ��� ����������� �21, 0.1�
(unsigned char*)&bps[20]._Ii,
(unsigned char*)&bps[20]._Ti+1,			//���94	����������� ��������� ����������� �21, 1��
(unsigned char*)&bps[20]._Ti,
(unsigned char*)&bps[20]._av+1,			//���95	���� ������ ����������� �21, 0x01 - ��������, 0x02 �������� U���, 0x04 �������� U���, 0x08 - ����������� ����� � ������������
(unsigned char*)&bps[20]._av,
(unsigned char*)&bps[21]._Uii+1,			//���96	�������� ���������� ����������� �22, 0.1�
(unsigned char*)&bps[21]._Uii,
(unsigned char*)&bps[21]._Ii+1,			//���97	�������� ��� ����������� �22, 0.1�
(unsigned char*)&bps[21]._Ii,
(unsigned char*)&bps[21]._Ti+1,			//���98	����������� ��������� ����������� �22, 1��
(unsigned char*)&bps[21]._Ti,
(unsigned char*)&bps[21]._av+1,			//���99	���� ������ ����������� �22, 0x01 - ��������, 0x02 �������� U���, 0x04 �������� U���, 0x08 - ����������� ����� � ������������
(unsigned char*)&bps[21]._av,
(unsigned char*)&bps[22]._Uii+1,			//���100	�������� ���������� ����������� �23, 0.1�
(unsigned char*)&bps[22]._Uii,
(unsigned char*)&bps[22]._Ii+1,			//���101	�������� ��� ����������� �23, 0.1�
(unsigned char*)&bps[22]._Ii,
(unsigned char*)&bps[22]._Ti+1,			//���102	����������� ��������� ����������� �23, 1��
(unsigned char*)&bps[22]._Ti,
(unsigned char*)&bps[22]._av+1,			//���103	���� ������ ����������� �23, 0x01 - ��������, 0x02 �������� U���, 0x04 �������� U���, 0x08 - ����������� ����� � ������������
(unsigned char*)&bps[22]._av,
(unsigned char*)&bps[23]._Uii+1,			//���104	�������� ���������� ����������� �24, 0.1�
(unsigned char*)&bps[23]._Uii,
(unsigned char*)&bps[23]._Ii+1,			//���105	�������� ��� ����������� �24, 0.1�
(unsigned char*)&bps[23]._Ii,
(unsigned char*)&bps[23]._Ti+1,			//���106	����������� ��������� ����������� �24, 1��
(unsigned char*)&bps[23]._Ti,
(unsigned char*)&bps[23]._av+1,			//���107	���� ������ ����������� �24, 0x01 - ��������, 0x02 �������� U���, 0x04 �������� U���, 0x08 - ����������� ����� � ������������
(unsigned char*)&bps[23]._av,
(unsigned char*)&bps[24]._Uii+1,			//���108	�������� ���������� ����������� �25, 0.1�
(unsigned char*)&bps[24]._Uii,
(unsigned char*)&bps[24]._Ii+1,			//���109	�������� ��� ����������� �25, 0.1�
(unsigned char*)&bps[24]._Ii,
(unsigned char*)&bps[24]._Ti+1,			//���110	����������� ��������� ����������� �25, 1��
(unsigned char*)&bps[24]._Ti,
(unsigned char*)&bps[24]._av+1,			//���111	���� ������ ����������� �25, 0x01 - ��������, 0x02 �������� U���, 0x04 �������� U���, 0x08 - ����������� ����� � ������������
(unsigned char*)&bps[24]._av,
(unsigned char*)&bps[25]._Uii+1,			//���112	�������� ���������� ����������� �26, 0.1�
(unsigned char*)&bps[25]._Uii,
(unsigned char*)&bps[25]._Ii+1,			//���113	�������� ��� ����������� �26, 0.1�
(unsigned char*)&bps[25]._Ii,
(unsigned char*)&bps[25]._Ti+1,			//���114	����������� ��������� ����������� �26, 1��
(unsigned char*)&bps[25]._Ti,
(unsigned char*)&bps[25]._av+1,			//���115	���� ������ ����������� �26, 0x01 - ��������, 0x02 �������� U���, 0x04 �������� U���, 0x08 - ����������� ����� � ������������
(unsigned char*)&bps[25]._av,
(unsigned char*)&bps[26]._Uii+1,			//���116	�������� ���������� ����������� �27, 0.1�
(unsigned char*)&bps[26]._Uii,
(unsigned char*)&bps[26]._Ii+1,			//���117	�������� ��� ����������� �27, 0.1�
(unsigned char*)&bps[26]._Ii,
(unsigned char*)&bps[26]._Ti+1,			//���118	����������� ��������� ����������� �27, 1��
(unsigned char*)&bps[26]._Ti,
(unsigned char*)&bps[26]._av+1,			//���119	���� ������ ����������� �27, 0x01 - ��������, 0x02 �������� U���, 0x04 �������� U���, 0x08 - ����������� ����� � ������������
(unsigned char*)&bps[26]._av,
(unsigned char*)&bps[27]._Uii+1,			//���120	�������� ���������� ����������� �28, 0.1�
(unsigned char*)&bps[27]._Uii,
(unsigned char*)&bps[27]._Ii+1,			//���121	�������� ��� ����������� �28, 0.1�
(unsigned char*)&bps[27]._Ii,
(unsigned char*)&bps[27]._Ti+1,			//���122	����������� ��������� ����������� �28, 1��
(unsigned char*)&bps[27]._Ti,
(unsigned char*)&bps[27]._av+1,			//���123	���� ������ ����������� �28, 0x01 - ��������, 0x02 �������� U���, 0x04 �������� U���, 0x08 - ����������� ����� � ������������
(unsigned char*)&bps[27]._av,
(unsigned char*)&bps[28]._Uii+1,			//���124	�������� ���������� ����������� �29, 0.1�
(unsigned char*)&bps[28]._Uii,
(unsigned char*)&bps[28]._Ii+1,			//���125	�������� ��� ����������� �29, 0.1�
(unsigned char*)&bps[28]._Ii,
(unsigned char*)&bps[28]._Ti+1,			//���126	����������� ��������� ����������� �29, 1��
(unsigned char*)&bps[28]._Ti,
(unsigned char*)&bps[28]._av+1,			//���127	���� ������ ����������� �29, 0x01 - ��������, 0x02 �������� U���, 0x04 �������� U���, 0x08 - ����������� ����� � ������������
(unsigned char*)&bps[28]._av,
(unsigned char*)&bps[29]._Uii+1,			//���128	�������� ���������� ����������� �30, 0.1�
(unsigned char*)&bps[29]._Uii,
(unsigned char*)&bps[29]._Ii+1,			//���129	�������� ��� ����������� �30, 0.1�
(unsigned char*)&bps[29]._Ii,
(unsigned char*)&bps[29]._Ti+1,			//���130	����������� ��������� ����������� �30, 1��
(unsigned char*)&bps[29]._Ti,
(unsigned char*)&bps[29]._av+1,			//���131	���� ������ ����������� �30, 0x01 - ��������, 0x02 �������� U���, 0x04 �������� U���, 0x08 - ����������� ����� � ������������
(unsigned char*)&bps[29]._av,
(unsigned char*)&bps[30]._Uii+1,			//���132	�������� ���������� ����������� �31, 0.1�
(unsigned char*)&bps[30]._Uii,
(unsigned char*)&bps[30]._Ii+1,			//���133	�������� ��� ����������� �31, 0.1�
(unsigned char*)&bps[30]._Ii,
(unsigned char*)&bps[30]._Ti+1,			//���134	����������� ��������� ����������� �31, 1��
(unsigned char*)&bps[30]._Ti,
(unsigned char*)&bps[30]._av+1,			//���135	���� ������ ����������� �31, 0x01 - ��������, 0x02 �������� U���, 0x04 �������� U���, 0x08 - ����������� ����� � ������������
(unsigned char*)&bps[30]._av,
(unsigned char*)&bps[31]._Uii+1,			//���136	�������� ���������� ����������� �32, 0.1�
(unsigned char*)&bps[31]._Uii,
(unsigned char*)&bps[31]._Ii+1,			//���137	�������� ��� ����������� �32, 0.1�
(unsigned char*)&bps[31]._Ii,
(unsigned char*)&bps[31]._Ti+1,			//���138	����������� ��������� ����������� �32, 1��
(unsigned char*)&bps[31]._Ti,
(unsigned char*)&bps[31]._av+1,			//���139	���� ������ ����������� �32, 0x01 - ��������, 0x02 �������� U���, 0x04 �������� U���, 0x08 - ����������� ����� � ������������
(unsigned char*)&bps[31]._av,
(unsigned char*)&HARDVARE_VERSION+1,  //��� 140  	���������� ������
(unsigned char*)&HARDVARE_VERSION,	  //��� 140  	���������� ������
(unsigned char*)&SOFT_VERSION+1,	  //��� 141  	������ ��
(unsigned char*)&SOFT_VERSION,		  //��� 141  	������ ��
(unsigned char*)&BUILD+1,			  //��� 142  	����� ���������� ��
(unsigned char*)&BUILD,				  //��� 142  	����� ���������� ��
(unsigned char*)&BUILD_YEAR+1,		  //��� 143  	��� ���������� ��
(unsigned char*)&BUILD_YEAR,		  //��� 143  	��� ���������� ��
(unsigned char*)&BUILD_MONTH+1,		  //��� 144  	����� ���������� ��
(unsigned char*)&BUILD_MONTH,		  //��� 144  	����� ���������� ��
(unsigned char*)&BUILD_DAY+1,		  //��� 145  	���� ���������� ��
(unsigned char*)&BUILD_DAY,			  //��� 145  	���� ���������� ��
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

//���������� ���� � �������, ������� ������� ����:= (������������ ����� ��������+1)*2
// ������ � .h #define MODBUS_FUNC_4_LENGTH  		   





