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
	  UINT16 svid;  //卫星编号
      UINT16 vflg;         // 星历有效标志
      UINT16 s1hlth;       // 子帧1健康码
      int16 wkn;         // 子帧1接收时刻GPS周
      UINT16 toewk;       // GPSweek corresponding to toc
      UINT16 ura;          //测距精度URA
      UINT16 iodc;        // Issue of data, clock
      UINT16 iode;         // Issue of data, ephemeris
      UINT32 toc;          // 第一数据块参考时刻
      UINT32 toe;        // 从每星期起始历元，星期六/星期日子夜零点起计算星历的参考时刻
      //考虑BD2
      double af0;               // 卫星时钟改正参数
      double af1;               // 卫星时钟改正参数
      double af2;               // 卫星时钟改正参数
      //考虑GPS
      double tgd;               // 电离层延迟改正
      double crs;               // 轨道半轴的正弦调和项改正的振幅
      double deltan;            // 平均角速度改正
      double m0;                // 参考时刻toe的平近点角
      double cuc;               // 升交角距的余弦调和项改正的振幅
      double ecc;               // 卫星椭圆轨道偏心率
      double cus;               // 升交角距的正弦调和项改正的振幅
      double sqrta;             // 卫星椭圆轨道的半长轴平方根
      double cic;               // 轨道倾角的余弦调和项改正的振幅
      double omega0;            // 参考时刻toe的轨道升交点准经度
      double cis;               // 轨道倾角的正弦调和项改正的振幅
      double i0;                // 参考时刻toe的轨道倾角
      double crc;               // 轨道半轴的余弦调和项改正的振幅
      double w;                 // 轨道近地点角距
      double omegadot;          // 升交点赤经变化率
      double idot;              // 轨道倾角变化率
      UINT32 TofXmission;  // 子帧1发射时刻(周内秒计数)
      unsigned char u1Flags;
}GPSBD2EphFrameStruct;

#if SUPPORT_GLONASS
typedef struct
{
	word16 svid;
	UINT16 vflg;		 // 星历有效标志
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
	
	double gaman;	//载波频率偏差率	
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

// 历书结构体
typedef struct
{
      UINT32 vflg;         // 有效标志
      UINT32 almhlth;     // 历书有效状态
      int32 refweek;     // 参考周
      int32 toa;          // Reference time, seconds of refweek
      double ecc;               // 卫星椭圆轨道偏心率
      double i0;                // 参考时刻toe的轨道倾角
      double omegadot;          // 升交点赤经变化率
      double sqrta;             // 卫星椭圆轨道的半长轴平方根
      double omega0;            // 参考时刻toe的轨道升交点准经度
      double w;                 // 轨道近地点角距
      double m0;                // 参考时刻toe的平近点角
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

// 历书接收情况结构体
typedef struct
{
    // 32颗卫星的历书接收状态，对应位为1表示已经接收到该卫星的历书，
    // 否则没有接收到(或该星历书将不再播发)。
    UINT32 AlmanaceReceiveStatus;
    UINT16 Get_Subframe_56;      // 三个历书子帧的接收状态，为1表示已接收到该帧，否则没有接受到。
    UINT16 WeekOfSubframe56;    // 接收该三个历书子帧时对应的GPS周
}AlmanaceCollectStatusStruct;

// 接收历书帧GPS周结构体
typedef struct
{
    UINT16 week;                // 接收该历书帧时的GPS时间
}AlmanaceFrameBufferStruct;

// 北斗2UTC时结构体
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

// 北斗2其他时间参数结构体
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

// GPS UTC时结构体
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

//电离层和UTC时间参数(与JAVAD定义的数据类型相同) 电离层参数结构体
typedef struct
{
    UINT16 vflg;                   // 电离层有效标志
    double a0,a1,a2,a3;                     // 电离层改正参数
    double b0,b1,b2,b3;                     // 电离层改正参数
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

	IonoStruct GeoD2Iono;                                   // GEO 北斗2电离层改正
	IonoStruct MeoD1Iono;                                   // MEO 北斗2电离层改正
	IonoStruct Gpsionoutc;

	BD2OtherTimeRelationStruct OtherTimeRelation;
	AlmanaceCollectStatusStruct AlmCollectStruct;
	}AlmEphUTCInfo;
// 星历结构体

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

