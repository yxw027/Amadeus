#ifndef _HEAD_TIME_PROC_H_
#define _HEAD_TIME_PROC_H_

#include "define.h"
#ifndef _POSTPROC
#ifndef _SIMULATE
#include "global.h"
#endif
#endif




#define MJD_JAN61980		44244
#define MJD_JAN11901		15385


#define GPSTIME_RTCCNT					0x1
#define GPSTIME_RFCNT54					0x4
#define GPSTIME_ALL_CNT					0xF

#define GPSTIME_RFCNT54_VALIDITY			(24*SECONDS_IN_HOUR)

#define RTC_32K_FREQ					(32768)

#define MAX_RTCCNT						(4.294967296e9)

#define GPSTIME_SYNC_SRC_NONE					0
#define GPSTIME_SYNC_SRC_RTC_UTCTIME			1
#define GPSTIME_SYNC_SRC_GLO_ONLY				2
#define GPSTIME_SYNC_SRC_SFSYNC_TIME			3
#define GPSTIME_SYNC_SRC_RTCPULSE_ERRGT05MS		4
#define GPSTIME_SYNC_SRC_RTCPULSE_ERR05MS		5
#define GPSTIME_SYNC_SRC_RTCPULSE_ERR30US		6
#define GPSTIME_SYNC_SRC_REFBIT_RFCNT			7
#define GPSTIME_SYNC_SRC_FIXRFCNT				8


#define RTCLOCKINFO_ANSYNC_NONE		0
#define RTCLOCKINFO_ANSYNC_PRE_WR	1
#define RTCLOCKINFO_ANSYNC_WR		2
#define RTCLOCKINFO_ANSYNC_RD		3
#define RTCLOCKINFO_ANSYNC_BACKUP	4



#define RTC_SYNC_NONE				0
#define RTC_SYNC_LEVEL_1			1
#define RTC_SYNC_LEVEL_2			2
#define RTC_SYNC_LEVEL_3			3


#define RTC_20MIN_SECOND			1200
#define RTC_15MIN_SECOND			900
#define RTC_12MIN_SECOND			720
#define RTC_10MIN_SECOND			600
#define RTC_2_5MIN_SECOND			150
#define RTC_2MIN_SECOND				120
#define RTC_1MIN_SECOND				60



typedef struct{
	UINT32 reg_0;
	UINT32 reg_1;
	UINT32 reg_2;
	UINT32 reg_3;
}TIC_TIME;


typedef struct{
	int16 year;
	int16 mon;
	int16 day;
	int16 hour;
	int16 min;
	int16 sec;
	int16 ms;
}UTC_TIME;


typedef struct{
	boolean  valid;	
	UTC_TIME rtctime;	
	double 	 rfcnt;	
}SYS_SWRTC_TIME;


typedef struct{
	byte syncsrc;
	int32 wn;
	double tow;
	double rfcnt;
	double rfcntEx;
}SYNC_NAVSYS_TIME;

typedef struct{
	bool  syncsrc;
	bool  bRTCFreValid;
	int32  wn;
	double tow;
	double rtcfreq;
	double rtccnt;
	double rfcnt;
}SYNC_NAVSYS_TIME_RTC;

#ifdef __cplusplus
extern "C" {
#endif

extern boolean bLastFirstRTCLock;
extern UINT32  backuprtcDate;
extern UINT32  backuprtcTime; 
extern int32   backupWN; 
extern double  backupTow; 
extern boolean bBackupRtcLockValid;

extern int32  g_RTCRdPulseCnt;


extern boolean 	bFirstRTCLock;
extern int  	g_RTCPulseCnt;

extern boolean bRTCFreqCorrected;

extern SYS_SWRTC_TIME swRtcTime;


extern SYNC_NAVSYS_TIME SyncGPSTime;
extern SYNC_NAVSYS_TIME SyncBDTime;
#if SUPPORT_GLONASS
extern SYNC_NAVSYS_TIME SyncGLOTime;
#endif


extern SYNC_NAVSYS_TIME_RTC SyncRTCTime;

#if SUPPORT_GLONASS
extern int32 Global_TleapsGLO;
extern int32 Global_DayN4GLO;
extern int32 Global_DayNTGLO;
#endif


extern word64_rtc RFSampleCnt;
extern word64_rtc RFNxSampleCnt;
extern void UpdateRFSampleCnt(UINT32 low);
extern float64 GetRFSampleCntByLow(UINT32 low);
extern void UpdateRFNxCnt(UINT32 low);
extern void H32L32ToF64(float64 *ret, word32 high, word32 low);

extern void SetSysTimeSyncInfo(int8 navsys,SYNC_NAVSYS_TIME *pSyncTime);
extern void setRTC32Freq(double rtcfreq);
extern double getRTC32Freq(void);
extern double GetCurRFSampleCnt(void);
//extern double GetCur100MRFSampleCnt(void);
extern word64_rtc GetCurRFSampleCntWord64(void);
extern double GetCurRFSampleFrq(void);
extern double getSysOnTime(void);
extern void GetTimeOfFixUTC (UTC_TIME *pDateTime, F8 fRound);
extern bool GetTimeOfCurUTC (UTC_TIME *pDateTime);

extern bool GetNavSysTimeByRFCnt(int8 navsys,int8 syncLevel, double curRFCnt, double *ptow, int32 *pWN);
extern bool GetRFCntByNavSysTime(int8 navsys,int8 syncLevel,float64 tow, int32 wn, float64* pRFcnt);
extern bool GetCurNavSysTime(int8 navsys,int8 syncLevel, float64 *ptow, int32 *pWN);
extern void GetSyncNavSysTime(int8 navsys, SYNC_NAVSYS_TIME* pSyncTime);
extern byte GetSyncNavSysTimeSrc(int8 navsys);

extern bool GetCurNavSysTimeRFcnt(int8 navsys,int8 syncLevel,float64 *ptow, int32 *pWN,double *pRFcnt);

extern void SyncNavSysTime(void);

extern void YMDHMSToGPSTime(UTC_TIME *pUTC, int32 *pWN, double *pSOW);
extern void GPSTimeToYMDHMS(int32 gpsWeek, float64 SecOfWeek, UTC_TIME* pTime);
extern float64 GetCurRFSampleFrq(void);
extern double GetCurRFSampleFrq100M(void);

extern boolean CheckSyncTime2HotAcqThres(void);

extern void RestSyncSynTime(byte sysflag, byte level);

#ifndef _POSTPROC
#ifndef _SIMULATE
extern void SetRTC(UTC_TIME* pTime,double rfcnt);
extern int32 GetRTC(UTC_TIME* pTime);
extern void ResetRTC(void);
extern byte getSWGPSTimeSrc(void);


extern void InitRTC32(void);
extern boolean CheckGPSTimeValidity(void);


extern void FormatRTCTime(UTC_TIME* pTime,UINT32 *rtcDate,UINT32 *rtcTime);
extern void SyncTICInterTime(void);
extern bool RestoreRTCTime2GPSTime(void);
extern void SetTICInterval(int32 interval);

extern int32 getRTCFrmTow(float64 tow, double *pRTC);
extern void Correct32KRTCFreq(void);


//extern word64_rtc gRtcIsrRFCntSample;
#else
extern int32 gTaskOpCnt;
#endif
#endif

#ifdef __cplusplus
}
#endif

#endif














