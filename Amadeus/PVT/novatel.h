/*
 * novatel.h
 *
 *  Created on: 2016-8-3
 *      Author: dell
 */

#ifndef NOVATEL_H_
#define NOVATEL_H_

extern void Uart_Send_BESTPOSA(byte comid);
extern void SenddataRS232_RangeA(byte comid);
extern void SenddataRS232_Satvis2A(byte comid);
extern void PeriodSendNovPro(void);
extern void Uart_Send_HEADINGA(byte comid);
extern void Uart_Send_BESTPOSB(byte comid);
extern void Uart_Send_RangeB(byte comid);
extern void Uart_Send_PsrdopB(byte comid);
extern unsigned long CRC32Value(int i);

extern int32 RangeANotOutCnt;
#endif /* NOVATEL_H_ */

typedef struct _tagBINHEAD
{
  INT16    type;
  UINT8   comflag;		//���
  UINT16 Reserved;
	UINT8  IdleTime;
	UINT8  TStatus;
	UINT16  mWeeks;		//�ܼ���
	UINT32 nSeconds;		//���������
	UINT32 Reserved1;
	UINT16  BD2LeapSec;
	UINT16 Reserved2;
}BINHEAD;

typedef struct _tag_STAR__POS    //Bestpos
{
	UINT32  sol_stat;
	UINT32	pos_type;
	DOUBLE	lat;
	DOUBLE	lon;
	DOUBLE	hgt;
	float	undulation;
	UINT32	datum;
	float	lat_delta;
	float	lon_delta;
	float	hgt_delta;
	char	base_id[4];
	float	diff_age;
	float	sol_age;
	unsigned char	trac_num;
	unsigned char	used_num;
	unsigned char	L1_num;
	unsigned char	muti_num;
	unsigned char	reserved;
	unsigned char	ext_sol_stat;
	unsigned char	Ga_BD_sin;
	unsigned char	GP_GL_sin;
}STAR_POS;

typedef struct _tag_STAR__PR    //range
{
    INT32     obs;
	UINT16    PRN;
    UINT16    glofreq;              // ͨ���ţ����Ǻţ�Ƶ��
	DOUBLE    psr;                   //α��
	float     psr_std;               //α����
	DOUBLE    CarrierPhase;       //�ز���λ����
	float     CarrierPhase_std;       //�ز���λ
	float     Dopple;                //������С��
	float     dSnr;                  //�����
	float     Locktime;
	UINT32    Channel_State;         // ͨ��״̬
	
}STAR_PR;

typedef struct _tag_STAR__DOP    //psrdop
{
   float     gdop;
   float     pdop;
   float     hdop;
   float     htdop;
   float     tdop;
   float     cutoff;
   INT32     obs;
	
}STAR_DOP;




