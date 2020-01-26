#ifndef _FRAME_DECODE_H
#define _FRAME_DECODE_H

#include "typedefine.h"
#include "define.h"
#include "global.h"
#include "FrameSync.h"

#if SUPPORT_GLONASS
#define MJD_GLO_GPS 5839
#endif
#define GetTeleParamValueU4(pWord, uWordId, uBitIdStart, uTotalBit)	( (UINT32)(((UINT32)(pWord[uWordId-1]<<(1+uBitIdStart))) >> (32-uTotalBit)) )
#define GetTeleParamValueI4(pWord, uWordId, uBitIdStart, uTotalBit)	( (INT32)(((INT32)(pWord[uWordId-1]<<(1+uBitIdStart))) >> (32-uTotalBit)) )
#define GetTeleParamValueF4(pWord, uWordId, uBitIdStart, uTotalBit)	( (float)(((INT32)(pWord[uWordId-1]<<(1+uBitIdStart))) >> (32-uTotalBit)) )
#define GetTeleParamValueF8(pWord, uWordId, uBitIdStart, uTotalBit)	( (double)(((INT32)(pWord[uWordId-1]<<(1+uBitIdStart))) >> (32-uTotalBit)) )



typedef struct
{
	  UINT16 svid;  //���Ǳ��
      UINT16 vflg;         // ������Ч��־
      UINT16 s1hlth;       // ��֡1������
      int16 wkn;         // ��֡1����ʱ��GPS��
      UINT16 toewk;       // GPSweek corresponding to toc
      UINT16 ura;          //��ྫ��URA
      UINT16 iodc;        // Issue of data, clock
      UINT16 iode;         // Issue of data, ephemeris
      UINT32 toc;          // ��һ���ݿ�ο�ʱ��
      UINT32 toe;        // ��ÿ������ʼ��Ԫ��������/��������ҹ�������������Ĳο�ʱ��
      //����BD2
      double af0;               // ����ʱ�Ӹ�������
      double af1;               // ����ʱ�Ӹ�������
      double af2;               // ����ʱ�Ӹ�������
      //����GPS
      double tgd;               // ������ӳٸ���
      double crs;               // �����������ҵ�������������
      double deltan;            // ƽ�����ٶȸ���
      double m0;                // �ο�ʱ��toe��ƽ�����
      double cuc;               // �����Ǿ�����ҵ�������������
      double ecc;               // ������Բ���ƫ����
      double cus;               // �����Ǿ�����ҵ�������������
      double sqrta;             // ������Բ����İ볤��ƽ����
      double cic;               // �����ǵ����ҵ�������������
      double omega0;            // �ο�ʱ��toe�Ĺ��������׼����
      double cis;               // �����ǵ����ҵ�������������
      double i0;                // �ο�ʱ��toe�Ĺ�����
      double crc;               // �����������ҵ�������������
      double w;                 // ������ص�Ǿ�
      double omegadot;          // ������ྭ�仯��
      double idot;              // �����Ǳ仯��
      UINT32 TofXmission;  // ��֡1����ʱ��(���������)
      unsigned char u1Flags;
}GPSBD2EphFrameStruct;

#if SUPPORT_GLONASS
typedef struct
{
	word16 svid;
	UINT16 vflg;		 // ������Ч��־
	word16 Bn;	//health flag  MBS = 0:health;  MBS = 1:unhealth
	word16 P1;	 //flag of the immediate data updating: 00(0);01(30);10(45);11(60)

	word16 P2;	 //flag of oddness("1") or evenness("0")
	word16 P3;	//flag indicating a number of satellites for which almanac is transmitted within given frame:1(five satellites);0(four satellites)
	word16 P4;	//flag to show that ephemeris parameters are present. 1(have new ephemeris );0 (not...)
	word16 tb;	 //an index of a time inerval within current day according to UTC(SU) + 03hours00min
	
	word32 tk;	 //the time referenced to the beginning of the frame within the current day.
	word16 M;	 //string No.
	word16 NT; //current date(within four-year)
	
	word16 FT; //user range accuracy
	word16 En; //age of eph data (day)
	
	double gaman;	//�ز�Ƶ��ƫ����	
	double DeltaTn;  //equipment delays of  (G2 - G1)
	double Tn;//corretion to the satellite time tn relative toGLO time tc
	double x;//the sv pos of x axis
	double y;
	double z;
	double vx;
	double vy;
	double vz;
	double vvx;
	double vvy;
	double vvz;

	//UTC time
	word32 NA;  //the day of Alm
	word32 N4;
	double DecDifGlo2UTCSU;  // decimal(TGlo-TUTCSU)(s)
	double DecDifGlo2Gps; //(TGlo-TGps) (day)
}GLOEphFrameStruct;
#endif

// ����ṹ��
typedef struct
{
      UINT32 vflg;         // ��Ч��־
      UINT32 almhlth;     // ������Ч״̬
      int32 refweek;     // �ο���
      int32 toa;          // Reference time, seconds of refweek
      double ecc;               // ������Բ���ƫ����
      double i0;                // �ο�ʱ��toe�Ĺ�����
      double omegadot;          // ������ྭ�仯��
      double sqrta;             // ������Բ����İ볤��ƽ����
      double omega0;            // �ο�ʱ��toe�Ĺ��������׼����
      double w;                 // ������ص�Ǿ�
      double m0;                // �ο�ʱ��toe��ƽ�����
      double af0;               // Clock correction at ref time (seconds)
      double af1;               // Clock correction rate (seconds/second)
}GPSBD2AlmanaceStruct;


#if SUPPORT_GLONASS
typedef struct
{
	word32 vflg;
	word16 svid;
	int16 HnFreq;       //carrier frequency number of RF signal
	word16 HealthSvflag; // 0:non-operability;  1:operability
	double TNode;     //time of the first ascending node passage of NA day
	double LonNode;   // longitude of the first ascending node passage of NA day
	double DelTclk;  // coarse value of sv clk time correction to GLONASS time
	double DelI0;   // rad correction of 63
	double ecc;   // eccentricity
	double wn;  //argument
	double DelTn;  // correction to the mean value of Draconian period T is 43200s
	double Del2Tn;  // rate of change of Draconianperiod of sv
	word16 Mn;        //00:GLONASS;01:GLONASS-M
	word32 Day;

}GLOAlmanaceStruct;
#endif

// �����������ṹ��
typedef struct
{
    // 32�����ǵ��������״̬����ӦλΪ1��ʾ�Ѿ����յ������ǵ����飬
    // ����û�н��յ�(��������齫���ٲ���)��
    UINT32 AlmanaceReceiveStatus;
    UINT16 Get_Subframe_56;      // ����������֡�Ľ���״̬��Ϊ1��ʾ�ѽ��յ���֡������û�н��ܵ���
    UINT16 WeekOfSubframe56;    // ���ո�����������֡ʱ��Ӧ��GPS��
}AlmanaceCollectStatusStruct;

// ��������֡GPS�ܽṹ��
typedef struct
{
    UINT16 week;                // ���ո�����֡ʱ��GPSʱ��
}AlmanaceFrameBufferStruct;

// ����2UTCʱ�ṹ��
typedef struct
{
    UINT16 vflg;
    short  dtls;
    UINT16 dn;
    short  dtlsf;
    UINT16 wnlsf;
    double A0;
    double A1;
}BD2UtcTimeStruct;

// ����2����ʱ������ṹ��
typedef struct
{
    UINT16 vflg;
    double A0MAT;
    double A1MAT;
    double A0GPS;
    double A1GPS;
    double A0Gal;
    double A1Gal;
    double A0Glo;
    double A1Glo;
}BD2OtherTimeRelationStruct;

// GPS UTCʱ�ṹ��
typedef struct
{
	double	A0;		//!< Seconds
	double	A1;		//!< Seconds/second
	int	dtls;	//!< Seconds
	UINT32	tot;	//!< Seconds
	UINT32	wnt;	//!< Weeks
	UINT32	wnlsf;	//!< Weeks
	UINT32	dn;		//!< Days
	int	dtlsf;	//!< Seconds
	UINT16 vflg;
}GpsUtcTimeStruct;

//������UTCʱ�����(��JAVAD���������������ͬ) ���������ṹ��
typedef struct
{
    UINT16 vflg;                   // �������Ч��־
    double a0,a1,a2,a3;                     // ������������
    double b0,b1,b2,b3;                     // ������������
    double gama0,gama1,gama2,gama3;
    double A,B;
    UINT16 v14ParaFlg;
}IonoStruct;

#if SUPPORT_GLONASS
typedef struct
{
	bool vflag;
	double B1;
	double B2;
	word16 KP;

}GLO_KP;
#endif

typedef struct
{
	GPSBD2EphFrameStruct GpsBd2_EphStruct[SV_NUM_GPSBD];    //Eph
#if SUPPORT_GLONASS
	GLOEphFrameStruct Glo_EphStruct[FRE_NUM_GLO];
#endif
	GPSBD2AlmanaceStruct GpsBd2_AlmStruct[SV_NUM_GPSBD];   //Alm
#if SUPPORT_GLONASS
	GLOAlmanaceStruct Glo_AlmStruct[SV_NUM_GLO_TRUE];

	GLO_KP GLO_kp;
#endif
	GpsUtcTimeStruct GpsUtcTime;
	BD2UtcTimeStruct BD2UtcTime;

	IonoStruct GeoD2Iono;                                   // GEO ����2��������
	IonoStruct MeoD1Iono;                                   // MEO ����2��������
	IonoStruct Gpsionoutc;

	BD2OtherTimeRelationStruct OtherTimeRelation;
	AlmanaceCollectStatusStruct AlmCollectStruct;
	}AlmEphUTCInfo;
// �����ṹ��

#ifdef __cplusplus
extern "C" {
#endif
extern AlmEphUTCInfo Sys3AlmEphUTCInfo;

#ifndef _SIMULATE
extern void TaskFrameDecode();
#endif
extern void InitFrameDecode();
extern int sbit(unsigned int val, int nbits);


#ifdef __cplusplus
}
#endif


#endif

