

#include <math.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "cfgpara.h"
#include "constdef.h"
#include "global.h"

#ifndef _POSTPROC
#ifdef _SIMULATE
#include "simulatorGlobal.h"
#include "dataprocess.h"
#else
#include "FrameDecode.h"
#include "HDefine.h"
#include "FpgaReg.h"
#endif
#endif

#include "TimeProc.h"
#include "PVT.h"
#include "PVTRcvrInfo.h"
#include "PostProc.h"
#include "lspvt.h"
#include "cfgpara.h"


int32 ClkErrGT10usCnt_gps = 0;
int32 ClkErrGT10usCnt_bd = 0;

//static unsigned int MJD( unsigned short year, unsigned short month, unsigned short day);
//static void MJDtoYMD( int MJD, unsigned short *y, unsigned short *m, unsigned short *d );
//static void GpsTimeToUtcTime(GpsUtcTimeStruct *ionoutc,unsigned short in_gwk,  double in_gsec, unsigned short *year, unsigned short *month, unsigned short *day, unsigned short *hour, unsigned short *minute, double *second );
//static void BD2TimeToUtcTime(BD2UtcTimeStruct *ionoutc,unsigned short in_gwk,  double in_gsec, unsigned short *year, unsigned short *month, unsigned short *day, unsigned short *hour, unsigned short *minute, double *second );

static void GPSTimeToUTC(int32 wn, float64 tow, int32 *pUTC_wn, float64 *pUTC_tow);\
static void BDTimeToUTC(int32 wn, float64 tow, int32 *pUTC_wn, float64 *pUTC_tow);


boolean  bUtcTime = FALSE;
UTC_TIME UtcTime;
SYS_SWRTC_TIME swRtcTime = {FALSE,{2015,1,1,0,0,0,0},0.0};


word64_rtc RFSampleCnt = {0,0};
word64_rtc RFNxSampleCnt = {0,0};

static UTC_TIME OuputTimeBase={2015,1,1,0,0,0,0};


SYNC_NAVSYS_TIME SyncGPSTime={GPSTIME_SYNC_SRC_NONE, 0, 0.0, 0.0};
SYNC_NAVSYS_TIME SyncBDTime={GPSTIME_SYNC_SRC_NONE, 0, 0.0, 0.0};
#if SUPPORT_GLONASS
SYNC_NAVSYS_TIME SyncGLOTime={GPSTIME_SYNC_SRC_NONE, 0, 0.0, 0.0};
#endif


SYNC_NAVSYS_TIME_RTC SyncRTCTime={FALSE, 0, 0, 0, 0.0, RTC_32K_FREQ, 0.0};
boolean bRTCFreqCorrected = FALSE;


static int16 MonthDay[2][13] = 
{
	{0, 31, 59, 90, 120, 151, 181, 212, 243, 273, 304, 334, 365},
	{0, 31, 60, 91, 121, 152, 182, 213, 244, 274, 305, 335, 366}
};

#ifndef _POSTPROC
#ifdef _SIMULATE
int32 gTaskOpCnt = 0;
#endif
#endif

boolean bRTC32KValid = TRUE;
boolean bFirstRTCLock = FALSE;
int32  g_RTCPulseCnt = 0;
int32  g_RTCRdPulseCnt = 0;
int32  g_RTCRdIRQPulseCnt = 0;
int  g_BackupRTCPulseCnt = 0;


//Glonass time info
#if SUPPORT_GLONASS
int32 Global_TleapsGLO = 0;
int32 Global_DayN4GLO = 0;
int32 Global_DayNTGLO = 0;
#endif

//Update system sample frequency. 
void UpdateRFSampleCnt(UINT32 low)
{
	if((RFSampleCnt.high!=0) || (RFSampleCnt.low!=0))
	{
		if(low < RFSampleCnt.low)
			RFSampleCnt.high++;
	}
	RFSampleCnt.low = low;
}

float64 GetRFSampleCntByLow(UINT32 low)
{
	float64 rfcnt=0.0;
	float64 difrfcnt = 0.0;
	UINT32 high = RFSampleCnt.high;

	difrfcnt = (float64)low - (float64)(RFSampleCnt.low);
	
	if(difrfcnt > LOW_32BIT_INVERT*RF_SAMP_FREQ)
		high--;
	else if(difrfcnt < (-LOW_32BIT_INVERT*RF_SAMP_FREQ))
		high++;

	rfcnt = high*TWO_P32 + low;

	return rfcnt;
}

//Update system Nx frequency. 
void UpdateRFNxCnt(UINT32 low)
{
	if((RFNxSampleCnt.high!=0) || (RFNxSampleCnt.low!=0))
	{
		if(low < RFNxSampleCnt.low)
			RFNxSampleCnt.high++;
	}
	RFNxSampleCnt.low = low;
}

/*********************************
	retrun the system on time: s
*********************************/
double getSysOnTime(void)
{
#ifndef _POSTPROC
#ifndef _SIMULATE
	double curRFCnt = GetCurRFSampleCnt();
	//double curSampFreq = GetCurRFSampleFrq();

	return (curRFCnt*RECIP_RF_SAMP_FREQ);	
#else
	return gTaskOpCnt*1;
#endif
#endif
}


//get current RF sample count.
double GetCurRFSampleCnt(void)
{
	double curRFcnt = 0.0;
#ifndef _POSTPROC
#ifndef _SIMULATE

	UINT32 high=0,low=0;
	UINT32 curRFCnt_L = getGlobalCnt();
	high = RFSampleCnt.high;
	low = RFSampleCnt.low;
	if((high!=0) || (low!=0))
	{
		if(curRFCnt_L < low)
			high++;
	}
	
	low = curRFCnt_L;	
	curRFcnt = ((FLOAT64)high)*TWO_P32 + low;
#else
	curRFcnt = Debug_MiscInfo.TicLockRFcnt;
#endif
#endif
	return curRFcnt;
}

//get current RF sample count.
word64_rtc GetCurRFSampleCntWord64(void)
{
	word64_rtc curRFcnt = {0,0};
#ifndef _POSTPROC
#ifndef _SIMULATE
	UINT32 high=0,low=0;
	UINT32 curRFCnt_L = getGlobalCnt();
	high = RFSampleCnt.high;
	low = RFSampleCnt.low;
	if((high!=0) || (low!=0))
	{
		if(curRFCnt_L < low)
			high++;
	}
	
	low = curRFCnt_L;
	curRFcnt.high = high;
	curRFcnt.low = low;
#endif
#endif
	return curRFcnt;
}



double GetCurRFSampleFrq(void)
{
	double tcxo = 0.0, freq=RF_SAMP_FREQ;

	if(GetTCXOOffset(&tcxo))
		freq = RF_SAMP_FREQ - tcxo;
	
	return freq;
}

double GetCurRFSampleFrq100M(void)
{
	double tcxo = 0.0, freq=RF_SAMP_FREQ100M;

	if(GetTCXOOffset(&tcxo))
		freq = RF_SAMP_FREQ100M - 2.0*tcxo;
	
	return freq;
}


bool GetNavSysTimeByRFCnt(int8 navsys,int8 syncLevel, double curRFCnt, double *ptow, int32 *pWN)
{
	double difRFcnt=0.0, difTime=0.0;
	int32 wn=0;
	double tow=0.0;
	SYNC_NAVSYS_TIME curSyncNavTime;

	GetSyncNavSysTime(navsys, &curSyncNavTime);

	if(curSyncNavTime.syncsrc < syncLevel)
		return FALSE;

	difRFcnt = curRFCnt - curSyncNavTime.rfcnt;
	difTime = difRFcnt/GetCurRFSampleFrq();

	tow = curSyncNavTime.tow + difTime;
	wn = curSyncNavTime.wn;

	if(tow > SECONDS_IN_WEEK)
	{
		wn++;
		tow -= SECONDS_IN_WEEK;
	}
	else if(tow < MICRO_NUM)
	{
		wn--;
		tow += SECONDS_IN_WEEK;
	}

	if(ptow != NULL)
		*ptow = tow;

	if(pWN != NULL)
		*pWN = wn;
	
	return TRUE;
}

bool GetNavSysTimeBy100MRFCnt(int8 navsys,int8 syncLevel, double curRFCnt, double *ptow, int32 *pWN)
{
	double difRFcnt=0.0, difTime=0.0;
	int32 wn=0;
	double tow=0.0;
	SYNC_NAVSYS_TIME curSyncNavTime;

	GetSyncNavSysTime(navsys, &curSyncNavTime);

	if(curSyncNavTime.syncsrc < syncLevel)
		return FALSE;

	difRFcnt = curRFCnt - curSyncNavTime.rfcntEx;
	difTime = difRFcnt/GetCurRFSampleFrq100M();

	tow = curSyncNavTime.tow + difTime;
	wn = curSyncNavTime.wn;

	if(tow > SECONDS_IN_WEEK)
	{
		wn++;
		tow -= SECONDS_IN_WEEK;
	}
	else if(tow < MICRO_NUM)
	{
		wn--;
		tow += SECONDS_IN_WEEK;
	}

	if(ptow != NULL)
		*ptow = tow;

	if(pWN != NULL)
		*pWN = wn;
	
	return TRUE;
}



void GetSyncNavSysTime(int8 navsys, SYNC_NAVSYS_TIME* pSyncTime)
{
	pSyncTime->syncsrc = 0;
	if(navsys & NAV_SYS_BD)
		*pSyncTime = SyncBDTime;

	if(navsys & NAV_SYS_GPS)
	{
		if(SyncGPSTime.syncsrc > pSyncTime->syncsrc)
			*pSyncTime = SyncGPSTime;
	}
	
#if SUPPORT_GLONASS
	if(navsys & NAV_SYS_GLO)
	{
		if(SyncGLOTime.syncsrc > pSyncTime->syncsrc)
			*pSyncTime = SyncGLOTime;
	}
#endif

	if(navsys == NAV_SYS_NONE)
	{
		*pSyncTime = SyncBDTime;
		if(SyncGPSTime.syncsrc > pSyncTime->syncsrc)
			*pSyncTime = SyncGPSTime;
		
#if SUPPORT_GLONASS		
		if(SyncGLOTime.syncsrc > pSyncTime->syncsrc)
			*pSyncTime = SyncGLOTime;
#endif
	}

	return;
}



byte GetSyncNavSysTimeSrc(int8 navsys)
{
	byte syncsrc;
	switch(navsys)
	{
	case NAV_SYS_GPS:
		syncsrc = SyncGPSTime.syncsrc;
		break;
	case NAV_SYS_BD:
		syncsrc = SyncBDTime.syncsrc;
		break;
	#if SUPPORT_GLONASS
	case NAV_SYS_GLO:
		syncsrc = SyncGLOTime.syncsrc;
		break;
	#endif
	default:
		if(SyncBDTime.syncsrc >= SyncGPSTime.syncsrc)
			syncsrc = SyncBDTime.syncsrc;
#if SUPPORT_GLONASS
		else if(SyncGPSTime.syncsrc >= SyncGLOTime.syncsrc)
			syncsrc = SyncGPSTime.syncsrc;
		else
			syncsrc = SyncGLOTime.syncsrc;	
#else
		else
			syncsrc = SyncGPSTime.syncsrc;
#endif
		break;
	}

	return syncsrc;
}

bool GetCurNavSysTime(int8 navsys,int8 syncLevel, float64 *ptow, int32 *pWN)
{
	double curRFcnt = GetCurRFSampleCnt();

	if(GetNavSysTimeByRFCnt(navsys,syncLevel,curRFcnt, ptow, pWN))
		return TRUE;
	else
		return FALSE;
}

bool GetCurNavSysTimeRFcnt(int8 navsys,int8 syncLevel,float64 *ptow, int32 *pWN,double *pRFcnt)
{
	double curRFcnt = GetCurRFSampleCnt();

	*pRFcnt = curRFcnt;
	if(GetNavSysTimeByRFCnt(navsys,syncLevel,curRFcnt, ptow, pWN))
		return TRUE;
	else
		return FALSE;
}

bool GetRFCntByNavSysTime(int8 navsys,int8 syncLevel,float64 tow, int32 wn, float64* pRFcnt)
{
	float64 diftime=0.0;
	SYNC_NAVSYS_TIME curSyncNavTime;

	GetSyncNavSysTime(navsys, &curSyncNavTime);

	if(curSyncNavTime.syncsrc < syncLevel)
		return FALSE;

	diftime = (wn-curSyncNavTime.wn)*SECONDS_IN_WEEK + tow-curSyncNavTime.tow;

	*pRFcnt = curSyncNavTime.rfcnt + diftime*RF_SAMP_FREQ;

	return TRUE;
}

void GetTimeOfFixUTC (UTC_TIME *pDateTime, F8 fRound)
{
#ifndef _POSTPROC
#ifndef _SIMULATE
	double tow=0.0, utc_tow=0.0;
	int32 wn=0, utc_wn=0;
	double curFixRFcnt = 0.0;

	curFixRFcnt = getTICLockRFcnt();
	if(GetNavSysTimeByRFCnt(NAV_SYS_NONE,GPSTIME_SYNC_SRC_GLO_ONLY,curFixRFcnt,&tow,&wn))
	{
		if(Sys3AlmEphUTCInfo.GpsUtcTime.vflg == 1)
		{
			GPSTimeToUTC(wn, tow, &utc_wn, &utc_tow);
		}
		else if(Sys3AlmEphUTCInfo.BD2UtcTime.vflg == 1)
		{
			BDTimeToUTC(wn, tow, &utc_wn, &utc_tow);
		}
		else
		{
			utc_wn = wn;
			utc_tow = tow + pActiveCPT->SysmCptWorkConfig.LeapSec; // leap seconds Units:S
			if(utc_tow > SECONDS_IN_WEEK)
				utc_tow -=SECONDS_IN_WEEK;
		}	
	}
	else
	{
		YMDHMSToGPSTime(&OuputTimeBase,&utc_wn,&utc_tow);
		curFixRFcnt = getTICLockRFcnt();	//no valid UTC time, output initial time
		utc_tow += curFixRFcnt / RF_SAMP_FREQ;
	}

	GPSTimeToYMDHMS(utc_wn, utc_tow, pDateTime);
#else	//simulator
	memcpy(pDateTime, &CurDebugUTCTime, sizeof(CurDebugUTCTime));
#endif
#endif 
	return;
}

bool GetTimeOfCurUTC (UTC_TIME *pDateTime)
{
#ifndef _POSTPROC
#ifndef _SIMULATE
	double tow=0.0, utc_tow=0.0;
	int32 wn=0, utc_wn=0;

	if(GetCurNavSysTime(NAV_SYS_NONE,GPSTIME_SYNC_SRC_FIXRFCNT,&tow,&wn))
	{
		if(Sys3AlmEphUTCInfo.GpsUtcTime.vflg == 1)
		{
			GPSTimeToUTC(wn, tow, &utc_wn, &utc_tow);
		}
		else if(Sys3AlmEphUTCInfo.BD2UtcTime.vflg == 1)
		{
			BDTimeToUTC(wn, tow, &utc_wn, &utc_tow);
		}
		else
		{
			utc_wn = wn;
			utc_tow = tow;
		}	

		GPSTimeToYMDHMS(utc_wn, utc_tow, pDateTime);

		return TRUE;
	}
#endif
#endif
	return FALSE;
}


void SetSysTimeSyncInfo(int8 navsys,SYNC_NAVSYS_TIME *pSyncTime)
{
	switch(navsys)
	{
	case NAV_SYS_GPS:
		if(SyncGPSTime.syncsrc <= pSyncTime->syncsrc)
			memcpy(&SyncGPSTime,pSyncTime,sizeof(SYNC_NAVSYS_TIME));
		break;
	case NAV_SYS_BD:
		if(SyncBDTime.syncsrc <= pSyncTime->syncsrc)		
			memcpy(&SyncBDTime,pSyncTime,sizeof(SYNC_NAVSYS_TIME));
		break;
#if SUPPORT_GLONASS
	case NAV_SYS_GLO:
		if(SyncGLOTime.syncsrc <= pSyncTime->syncsrc)
			memcpy(&SyncGLOTime,pSyncTime,sizeof(SYNC_NAVSYS_TIME));
		break;
#endif
	 default:
		if((SyncGPSTime.syncsrc <= pSyncTime->syncsrc) || (SyncBDTime.syncsrc <= pSyncTime->syncsrc))
		{
			memcpy(&SyncGPSTime,pSyncTime,sizeof(SYNC_NAVSYS_TIME));
			memcpy(&SyncBDTime,pSyncTime,sizeof(SYNC_NAVSYS_TIME));
#if SUPPORT_GLONASS
			memcpy(&SyncGLOTime,pSyncTime,sizeof(SYNC_NAVSYS_TIME));
#endif
		}
		 break;
	}
}

void SyncNavSysTime(void)
{
	const double ClkErrTh=0.5e-6; //?¡ìa?2??¡ìo?¡§¡è??
	static int32 ClkErrCnt_gps=0, ClkErrCnt_bd=0;
	int32 gpssvcnt=0, bdsvcnt=0;
	int32 gpsavrcn0=0, bdavrcn0=0;
	word256 chnmap={{0,}};
	int32 trkch=0, svid=0;
	int32 wn=0;
	double tow=0.0, rfcnt=0.0 ,rfcntEx=0.0;
	FIX_DOP dop;
	double clkerr=0.0;
#if SUPPORT_GLONASS
	int32 glosvcnt=0, gloavrcn0=0;
	static int32 ClkErrCnt_glo=0;
#endif

	if((!bFixAtSecBoundary) && GetCurNavSysTime(NAV_SYS_NONE,GPSTIME_SYNC_SRC_FIXRFCNT,&tow, &wn))
		return;

	if(getCurFixPosInfo(NULL,&dop,&chnmap) == FIX_NOT)
		return;

	//if(getRcvrInfo(NULL,NULL,NULL,NULL) == FIX_NOT)
	//	return;
		
	for(trkch=0; trkch<MAXCHANNELS; trkch++)
	{
		if(GetBitWord256(&chnmap,trkch)==0)
			continue;

		svid = PVTTrkchInfo[trkch].svid;

		if(SV_IsGps(svid))
		{
			gpssvcnt++;
			gpsavrcn0 += PVTTrkchInfo[trkch].cn1s;
		}
		else if(SV_IsBd2(svid))
		{
			bdsvcnt++;
			bdavrcn0 += PVTTrkchInfo[trkch].cn1s;
		}
	#if SUPPORT_GLONASS
		else if(SV_IsGlo(svid))
		{
			glosvcnt++;
			gloavrcn0 += PVTTrkchInfo[trkch].cn1s;
		}
	#endif
	}

	if(gpssvcnt > 0)
		gpsavrcn0 = gpsavrcn0/gpssvcnt;

	if(bdsvcnt > 0)
		bdavrcn0 = bdavrcn0/bdsvcnt;

#if SUPPORT_GLONASS
	if(glosvcnt > 0)
		gloavrcn0 = gloavrcn0/glosvcnt;
#endif	

	if(getCurFixGPSTime((int16*)(&wn),&tow, &rfcnt, &rfcntEx, &clkerr)>0)
	{
		if(fabs(clkerr)>ClkErrTh)
			ClkErrCnt_gps++;
		else
			ClkErrCnt_gps=0;

		ClkErrGT10usCnt_gps = ClkErrCnt_gps;

		//if((SyncGPSTime.syncsrc < GPSTIME_SYNC_SRC_FIXRFCNT)		//Syn sys time
		//	|| (ClkErrCnt_gps > ClkErrCntTh)
		//	|| ((SysEnvSta<SYS_HALF_OPEN_SKY) && (gpssvcnt >= 1) && (gpsavrcn0>=40) && (dop.gpstdop<2.5))
		//	|| ((SysEnvSta<SYS_HALF_OPEN_SKY) && (gpssvcnt >= 2) && (gpsavrcn0>=36) && (dop.gpstdop<2.5))
		//	|| ((gpssvcnt >= 4) && (gpsavrcn0>=37) && (dop.gpstdop<2.5))
		//	|| ((gpssvcnt >= 7) && (gpsavrcn0>=28) && (dop.gpstdop<2.5)))
		{
			SyncGPSTime.syncsrc = GPSTIME_SYNC_SRC_FIXRFCNT;
			SyncGPSTime.rfcnt = rfcnt;
			SyncGPSTime.rfcntEx= rfcntEx;
			SyncGPSTime.tow = tow;
			SyncGPSTime.wn = wn;
		}
	}
	else
		ClkErrCnt_gps=0;

	if(getCurFixBDTime((int16*)&wn,&tow, &rfcnt, &rfcntEx, &clkerr)>0)
	{
		if(fabs(clkerr)>ClkErrTh)
			ClkErrCnt_bd++;
		else
			ClkErrCnt_bd=0;

		ClkErrGT10usCnt_bd = ClkErrCnt_bd;
		
		//if((SyncBDTime.syncsrc < GPSTIME_SYNC_SRC_FIXRFCNT)	//Syn sys time
		//	|| (ClkErrCnt_bd > ClkErrCntTh)
		//	|| ((SysEnvSta<SYS_HALF_OPEN_SKY) && (bdsvcnt >= 1) && (bdavrcn0>=40) && (dop.bdtdop<2.5))
		//	|| ((SysEnvSta<SYS_HALF_OPEN_SKY) && (bdsvcnt >= 2) && (bdavrcn0>=36) && (dop.bdtdop<2.5))
		//	|| ((bdsvcnt >= 5) && (bdavrcn0>=37) && (dop.bdtdop<2.5))
		//	|| ((bdsvcnt >= 7) && (bdavrcn0>=28) && (dop.bdtdop<2.5)))
		{
			SyncBDTime.syncsrc = GPSTIME_SYNC_SRC_FIXRFCNT;
			SyncBDTime.rfcnt = rfcnt;
			SyncBDTime.rfcntEx= rfcntEx;
			SyncBDTime.tow = tow;
			SyncBDTime.wn = wn;
		}
	}
	else
		ClkErrCnt_bd=0;
		
#if SUPPORT_GLONASS
	if(getCurFixGloTime((int16*)&wn,&tow, &rfcnt, &rfcntEx, &clkerr)>0)
	{
		if(fabs(clkerr)>ClkErrTh)
			ClkErrCnt_glo++;
		else
			ClkErrCnt_glo=0;

		//if((!SyncGLOTime.syncsrc)
		//	|| (ClkErrCnt_glo > ClkErrCntTh)
		//	|| ((glosvcnt >= 5) && (gloavrcn0>=35) && (dop.glotdop<2.5))
		//	|| ((glosvcnt >= 7) && (gloavrcn0>=28) && (dop.glotdop<2.5)))
		{
			if((gpssvcnt>0) || (bdsvcnt>0))	
				SyncGLOTime.syncsrc = GPSTIME_SYNC_SRC_FIXRFCNT;
			else
				SyncGLOTime.syncsrc = GPSTIME_SYNC_SRC_GLO_ONLY;
			SyncGLOTime.rfcnt = rfcnt;
			SyncGLOTime.rfcntEx= rfcntEx;
			SyncGLOTime.tow = tow;
			SyncGLOTime.wn = wn;
		}
	}
	else
		ClkErrCnt_glo=0;
#endif

	return;
}

void RestSyncSynTime(byte sysflag, byte level)
{
	if((sysflag==NAV_SYS_GPS) || (sysflag==NAV_SYS_NONE))
		SyncGPSTime.syncsrc = level;
	
	if((sysflag==NAV_SYS_BD) || (sysflag==NAV_SYS_NONE))
		SyncBDTime.syncsrc = level;

#if SUPPORT_GLONASS
	if((sysflag==NAV_SYS_GLO) || (sysflag==NAV_SYS_NONE))
		SyncGLOTime.syncsrc = level;
#endif


	return;
}


/**
*	@brief	convert GPS time Year/Month/Day/Hour/Minute/Second(UTC) format
*	@param	gps week, second of week
*	@author	Haiquan Huang
*	@date	Apr 19, 2006
*
*/
float64 YMDHMSToMJD(UTC_TIME *pTime)
{
	int32 year, month, mday, hour, minute,second;	
	int32 yday, leap;
	double mjd, fmjd;

	year = pTime->year;
	month = pTime->mon;
	mday = pTime->day;
	hour = pTime->hour;
	minute = pTime->min;
	second = pTime->sec;

	leap = (year%4 == 0);
	yday = (MonthDay[leap][month-1] + mday);
	mjd = ((year - 1901)/4)*1461 + ((year - 1901)%4)*365 + yday - 1 + MJD_JAN11901;
	fmjd = ((second/60.0 + minute)/60.0 + hour)/24.0;

	return mjd+fmjd;
}


/**
*	@brief	convert UTC time(Year/Month/Day/Hour/Minute/Second, but this is 
*			GPS YMDHMS time in fact,not UTC time) 
*	@param	pUTC, time, pWN, week number; pSOW, second of week
*	@return pWN, return GPS week number, range 0~, not successive GPS week number
*
*/
void YMDHMSToGPSTime(UTC_TIME *pUTC, int32 *pWN, double *pSOW)
{
	float64 mjd, dayFrmGPSEpoc;
	int wn;

	mjd = YMDHMSToMJD(pUTC);
	dayFrmGPSEpoc = mjd - MJD_JAN61980;

	wn = (int32)(dayFrmGPSEpoc/7.0);
	*pWN = wn;
	*pSOW = (dayFrmGPSEpoc - wn*7.0)*SECONDS_IN_DAY;

}


/**
*	@brief	convert GPS Time to year/mon/day/hour/min/sec time format (none leaf seconds)
*	@para	gpsWeek:
*					0~1023
*			SecOfWeek:
*					0~604800
*			pTime:
*					return parameter pointer.
*	@return
*			none
*/
void GPSTimeToYMDHMS(int32 gpsWeek, float64 SecOfWeek, UTC_TIME* pTime)
{
	int32 leap, guess, more;
	int32 yday, mjd, days_fr_jan1_1901;
	int32 delta_yrs, num_four_yrs, years_so_far, days_left;
	float64 fmjd;
	
	SecOfWeek = SecOfWeek + 0.0005;	//round for NMEA
	if(SecOfWeek > SECONDS_IN_WEEK)
	{
		gpsWeek += 1;
		SecOfWeek -= SECONDS_IN_WEEK;
	}

	mjd = gpsWeek*7 + (int32)(SecOfWeek/SECONDS_IN_DAY) + MJD_JAN61980;
	fmjd=SecOfWeek/SECONDS_IN_DAY-(int32)(SecOfWeek/SECONDS_IN_DAY);

	days_fr_jan1_1901 = mjd - MJD_JAN11901;
	num_four_yrs = days_fr_jan1_1901/1461;
	years_so_far = 1901 + 4*num_four_yrs;
	days_left = days_fr_jan1_1901 - 1461*num_four_yrs;
	delta_yrs = days_left/365 - days_left/1460;

	pTime->year = years_so_far + delta_yrs;
	yday = days_left - 365*delta_yrs + 1;

	//hour,min, sec come from fmjd only
	pTime->hour = (int32)(fmjd*24.0);
	pTime->min = (int32)(fmjd*1440.0 - pTime->hour*60.0);
	pTime->sec = (int32)(fmjd*86400.0 - pTime->hour*3600.0 - pTime->min*60.0);
	pTime->ms = ((int32)((fmjd*86400.0 - pTime->hour*3600.0 - pTime->min*60.0-pTime->sec)*100.0))*10;
	
	if(pTime->year %4 == 0 )
	{
		leap = 1;
		if((pTime->year %100 == 0) && (pTime->year %400 != 0))
			leap = 0;
	}
	else
		leap = 0;

	guess = (int32)(yday * 0.032);//0.032: day per month
	
	if((yday - MonthDay[leap][guess+1]) <= 0)
		more = 0;
	else
		more = 1;
	pTime->mon = guess + more + 1;
	pTime->day = yday - MonthDay[leap][guess+more];

	return;
}


static void GPSTimeToUTC(int32 wn, float64 tow, int32 *pUTC_wn, float64 *pUTC_tow)
{
	float64 delta_UTC, delta_t, W;
	float64 UTC_TOD;
	float64 FmodX, FmodY;
	float64 secondsofday;
	int32 leapseconds = 0;
#ifndef _POSTPROC
#ifndef _SIMULATE
	int32 WN_8bit = wn & 0xff; // WNt & WNlsf are 8 bits, so GPS_WN & 0xff

	/*calculate delta_t(effectiveity date to user's current GPS time)*/
	int32 delta_wn=WN_8bit-(int32)Sys3AlmEphUTCInfo.GpsUtcTime.wnlsf;
	if(delta_wn>127)
		delta_wn-=256;

	if(delta_wn<-127)
		delta_wn+=256;

	delta_t=SECONDS_IN_WEEK*delta_wn + tow - SECONDS_IN_DAY*Sys3AlmEphUTCInfo.GpsUtcTime.dn;

	/*calculate delta_UTC*/
	delta_wn=WN_8bit-(int32)Sys3AlmEphUTCInfo.GpsUtcTime.wnt;
	if(delta_wn>127)
		delta_wn-=256;
	
	if(delta_wn<-127)
		delta_wn+=256;

	delta_UTC= Sys3AlmEphUTCInfo.GpsUtcTime.dtls +Sys3AlmEphUTCInfo.GpsUtcTime.A0 +Sys3AlmEphUTCInfo.GpsUtcTime.A1*(tow-Sys3AlmEphUTCInfo.GpsUtcTime.tot+SECONDS_IN_WEEK*delta_wn);

	/* case 1: 
	the effectivity time indicated by the WNLSF and the DN values is not in the past (relative to
	the user's present time), and the user's present time does not fall in the time span which 
	starts at six hours prior to the effectivity time and ends at six hours after the effectivity time 
	*/
	if (delta_t < -21600.0) 
	{
		FmodX = tow-delta_UTC;
		FmodY = SECONDS_IN_DAY;
		UTC_TOD = FmodX - ((int32)(FmodX/FmodY))*FmodY;
	}
	
	/* case 2: 
	the user's current time falls within the time span of six hours prior to the effectivity time to six
	hours after the effectivity time 
	*/
	else if (delta_t>=-21600.0 && delta_t<=21600.0) {
		FmodX = tow-delta_UTC-43200;
		FmodY = SECONDS_IN_DAY;
		W = (FmodX - ((int32)(FmodX/FmodY))*FmodY)  + 43200;

		FmodX = W;
		FmodY = SECONDS_IN_DAY+Sys3AlmEphUTCInfo.GpsUtcTime.dtlsf-Sys3AlmEphUTCInfo.GpsUtcTime.dtls;
		UTC_TOD  =FmodX - ((int32)(FmodX/FmodY))*FmodY;		

	}
	
	/* case 3: 
	the effectivity time of the leap second event, as indicated by the WNLSF and DN values, is in
	the "past" (relative to the user's current time), and the user's current time does not fall in the 
	time span as given above 
	*/
	else {
		delta_UTC = delta_UTC - Sys3AlmEphUTCInfo.GpsUtcTime.dtls + Sys3AlmEphUTCInfo.GpsUtcTime.dtlsf;
		FmodX = tow-delta_UTC;
		FmodY = SECONDS_IN_DAY;
		UTC_TOD  = FmodX - ((int32)(FmodX/FmodY))*FmodY;	
	}

	secondsofday= tow-((int32)(tow/SECONDS_IN_DAY))*SECONDS_IN_DAY;
	if(secondsofday >= UTC_TOD)
		leapseconds = (int32)(secondsofday - UTC_TOD + 0.5);		//round op
	else
		leapseconds = (int32)(secondsofday + SECONDS_IN_DAY -UTC_TOD + 0.5);	//round op

	if(leapseconds > 2*SECONDS_IN_MINUTE)
		leapseconds = 0;
#else
	leapseconds = 17;
#endif
#endif
	//week roll over handle
	if(tow < leapseconds)
	{
		(*pUTC_wn) =wn-1;
		(*pUTC_tow) = tow + SECONDS_IN_WEEK-leapseconds;
	}
	else
	{
		(*pUTC_wn) =wn;
		(*pUTC_tow) = tow-leapseconds;

	}
	return;
}


static void BDTimeToUTC(int32 wn, float64 tow, int32 *pUTC_wn, float64 *pUTC_tow)
{
	float64 delta_UTC, delta_t, W;
	float64 UTC_TOD;
	float64 FmodX, FmodY;
	int32 WN_8bit;
	int32 delta_wn;
	float64 secondsofday;
	int32 leapseconds = 0;
#ifndef _POSTPROC
#ifndef _SIMULATE
	wn -= GPS_BD_WN_OFFSET;
	tow -= GPS_BD_SYSTIME_OFFSET;
	if(tow < MICRO_NUM)
	{
		tow += SECONDS_IN_WEEK;
		wn--;
	}
	
	WN_8bit = wn & 0xff; // WNt & WNlsf are 8 bits, so GPS_WN & 0xff

	/*calculate delta_t(effectiveity date to user's current BDT)*/
	delta_wn=WN_8bit-(int32)Sys3AlmEphUTCInfo.BD2UtcTime.wnlsf;
	if(delta_wn>127)
		delta_wn-=256;

	if(delta_wn<-127)
		delta_wn+=256;

	delta_t=SECONDS_IN_WEEK*delta_wn + tow - SECONDS_IN_DAY*(Sys3AlmEphUTCInfo.BD2UtcTime.dn+1);

	/*calculate delta_UTC*/
	delta_UTC= Sys3AlmEphUTCInfo.BD2UtcTime.dtls +Sys3AlmEphUTCInfo.BD2UtcTime.A0 +Sys3AlmEphUTCInfo.BD2UtcTime.A1*tow;

	/* case 1: 
	the effectivity time indicated by the WNLSF and the DN values is not in the past (relative to
	the user's present time), and the user's present time does not fall in the time span which 
	starts at 16 hours prior to the effectivity time and ends  
	*/
	if (delta_t < 57600.0) 
	{
		FmodX = tow-delta_UTC;
		FmodY = SECONDS_IN_DAY;
		UTC_TOD = FmodX - ((int32)(FmodX/FmodY))*FmodY;
	}
	
	/* case 2: 
	the user's current time falls within the time span of 16 hours prior to the effectivity time to 30
	hours after the effectivity time 
	*/
	else if (delta_t>= 57600.0 && delta_t<=108000.0) {
		FmodX = tow-delta_UTC-43200;
		FmodY = SECONDS_IN_DAY;
		W = (FmodX - ((int32)(FmodX/FmodY))*FmodY)  + 43200;

		FmodX = W;
		FmodY = SECONDS_IN_DAY+Sys3AlmEphUTCInfo.BD2UtcTime.dtlsf-Sys3AlmEphUTCInfo.BD2UtcTime.dtls;
		UTC_TOD  =FmodX - ((int32)(FmodX/FmodY))*FmodY;		

	}
	
	/* case 3: 
	the effectivity time of the leap second event, as indicated by the WNLSF and DN values, is in
	the "past" (relative to the user's current time), and the user's current time does not fall in the 
	time span as given above 
	*/
	else {
		delta_UTC = delta_UTC - Sys3AlmEphUTCInfo.BD2UtcTime.dtls + Sys3AlmEphUTCInfo.BD2UtcTime.dtlsf;
		FmodX = tow-delta_UTC;
		FmodY = SECONDS_IN_DAY;
		UTC_TOD  = FmodX - ((int32)(FmodX/FmodY))*FmodY;	
	}

	secondsofday= tow-((int32)(tow/SECONDS_IN_DAY))*SECONDS_IN_DAY;
	if(secondsofday >= UTC_TOD)
		leapseconds = (int32)(secondsofday - UTC_TOD + 0.5);		//round op
	else
		leapseconds = (int32)(secondsofday + SECONDS_IN_DAY - UTC_TOD + 0.5); //round op

	if(leapseconds > 2*SECONDS_IN_MINUTE)
		leapseconds = 0;
#else
	leapseconds = 3;
#endif 

#endif
	//sub leapseconds to TOW
	wn += GPS_BD_WN_OFFSET;
	if(tow < leapseconds)
	{
		(*pUTC_wn) =wn-1;
		(*pUTC_tow) = tow + SECONDS_IN_WEEK-leapseconds;
	}
	else
	{
		(*pUTC_wn) =wn;
		(*pUTC_tow) = tow-leapseconds;
	}

	return;
}	


double RTCFreq = RTC_32K_FREQ;
double RTCFreqts = 0;



void Correct32KRTCFreq(void)
{
#if 0
#ifndef _SIMULATE
	static boolean bRecMin3=FALSE;
	static double AheadMin3RTC=0;
	static word32 AheadMin3RTCPulseCnt=0;	
	static double BaseRTC32 = 0;
	static int32  BaseRtcPulseCnt = 0;
	float64 deltatime = 0.0;
	double deltartccnt = 0;
	double ntow = 0.0;
	int32  nwn = 0;
	double IntervalRFCnt = 0;
	int32  curRtcPulseCnt = 0;
	double curRTCLockRFCnt = 0;

	a9RtcIsrMask();
	curRtcPulseCnt = g_RTCPulseCnt;
	curRTCLockRFCnt = gRtcIsrRFCntSample.high*TWO_P32 + gRtcIsrRFCntSample.low;
	a9RtcIsrUnMask();		

	if(GetCurNavSysTime(NAV_SYS_NONE,GPSTIME_SYNC_SRC_FIXRFCNT,&ntow,&nwn))
	{				
		//init the base clock to calculate RTC_32 clock period
		if(b32KRTCCntBaseRTC == TRUE)
		{
			IntervalRFCnt = curRTCLockRFCnt - BaseRTC32;

			if(IntervalRFCnt > RF_SAMP_FREQ*4)
			{
				bRTCFreqCorrected = TRUE;
				if(BaseRtcPulseCnt >= curRtcPulseCnt)
					b32KRTCCntBaseRTC = FALSE;				
				else
					deltartccnt = (curRtcPulseCnt - BaseRtcPulseCnt)*RTC_32K_FREQ;
				
				deltatime = IntervalRFCnt / GetCurRFSampleFrq();
				RTCFreq = ((float64)deltartccnt) / deltatime;
			}

			//update base rtc
			if((IntervalRFCnt > RF_SAMP_FREQ*15) &&  bRecMin3==FALSE)
			{
				AheadMin3RTC = curRTCLockRFCnt;
				AheadMin3RTCPulseCnt = curRtcPulseCnt;
				bRecMin3 = TRUE;
			}
	
			if((IntervalRFCnt > RF_SAMP_FREQ*30) && bRecMin3)
			{
				BaseRTC32 = AheadMin3RTC;
				BaseRtcPulseCnt = AheadMin3RTCPulseCnt; 			
				bRecMin3 = FALSE;
			}			
		}
		else
		{
			BaseRTC32 = curRTCLockRFCnt;
			BaseRtcPulseCnt = curRtcPulseCnt;
			RTCFreq = RTC_32K_FREQ;

			b32KRTCCntBaseRTC = TRUE;
		}	
	 }
#endif
#endif
}


double getRTC32Freq(void)
{
	return RTCFreq;
}


void setRTC32Freq(double rtcfreq)
{
	RTCFreq = rtcfreq;
}

#ifndef _POSTPROC
#ifndef _SIMULATE
int32 getRTCFrmTow(float64 tow, double *pRTC)
{
	float64 diff_t;
	float64 rtc_val;
	float64 ntow;
	int32 nWN;
	float64 curr_rf_samp_freq = 0;

	if(GetCurNavSysTimeRFcnt(NAV_SYS_NONE,GPSTIME_SYNC_SRC_RTCPULSE_ERR30US,&ntow,&nWN,&rtc_val)==FALSE)
		return -1;

	if(ntow >= SECONDS_IN_HALF_WEEK)
	{
		if( tow >= ntow - SECONDS_IN_HALF_WEEK )
			diff_t = tow - ntow;
		else
			diff_t = tow + SECONDS_IN_WEEK - ntow;
	}
	else
	{
		if(tow >= 0 && tow <= ntow + SECONDS_IN_HALF_WEEK)
			diff_t = tow - ntow;
		else
			diff_t = -(SECONDS_IN_WEEK - tow + ntow); //negative value
	}

	curr_rf_samp_freq = GetCurRFSampleFrq();	
	rtc_val += diff_t * curr_rf_samp_freq;

	*pRTC = rtc_val;

	return 0;
}

boolean CheckSyncTime2HotAcqThres(void)
{
	boolean bSyncTimeLevelUp = FALSE;
	SYNC_NAVSYS_TIME syncTime = {0,};
	static SYNC_NAVSYS_TIME last_syncTime = {0,};

	GetSyncNavSysTime(NAV_SYS_NONE,&syncTime);
	if((syncTime.syncsrc > last_syncTime.syncsrc) && (last_syncTime.syncsrc >= GPSTIME_SYNC_SRC_RTCPULSE_ERR30US))
	{
		bSyncTimeLevelUp = TRUE;
	}

	//backup 
	memcpy(&last_syncTime,&syncTime,sizeof(SYNC_NAVSYS_TIME));

	return 	bSyncTimeLevelUp;
}


#endif
#endif

