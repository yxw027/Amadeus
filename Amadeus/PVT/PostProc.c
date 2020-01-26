


#include "coordinate.h"
#include "define.h"
#include "constdef.h"
#include "global.h"

#ifndef _POSTPROC
#ifndef _SIMULATE
#include "FpgaReg.h"
#include "GnssTYProcfg.h"
#endif
#endif

#include "PostProc.h"
#include "PVT.h"
#include "PVTRcvrInfo.h"

#include "timeproc.h"
#include "KFPVT.h"
#include "lspvt.h"
#include "cfgpara.h"
#include "math.h"
#include "TimeProc.h"
static void UpdateTFTCycle();

extern int32 ClkErrGT10usCnt_gps;
extern int32 ClkErrGT10usCnt_bd;
extern double TICLock_RFcnt;

static void CalcTCXOOffset(void);
static void UpdateLongTermAlt(void);
static void UpdateBD2GPSTimeBias(void);
static void Write1PPSToFpga(void);

//time bias
static double BdTime2GpsTimeBias = 0.0;	//unit: m
static double BdTime2GpsTimeBiasTs = 0;	// time of bd2gps tow, unit: second;
static int32 BdTime2GpsTimeBiasCnt = 0;

static double GloTime2GpsTimeBias = 0.0;	//unit: m
static double GloTime2GpsTimeBiasTs = 0;	// time of bd2gps tow, unit: second;
static int32 GloTime2GpsTimeBiasCnt = 0;

#if SUPPORT_GLONASS
static double GloTime2BdTimeBias = 0.0;	//unit: m
static double GloTime2BdTimeBiasTs = 0;	// time of bd2gps tow, unit: second;
static int32 GloTime2BdTimeBiasCnt = 0;
#endif

#define FILTER_COEFFICIENT		(0.2)
static bool bTCXOValid = FALSE;
double TCXOOffset = 0.0;
static double RTTcxo = 0.0;

int32 TTFF=0;	//ms
int32 FixTime = 0;
word32 InitCostRFcnt1 = 0;
word32 InitCostRFcnt2 = 0;

float64 ShortTermAveSysAlt = 0.0;

extern int g_RTCPulseCnt;
extern int g_BackupRTCPulseCnt;


float64 LongTermAveSysAlt = 0.0;
float64 LongTermAltEnv = 5.0;
int32 LongTermAveSysAltCnt = 0;
static boolean bLongTermAltSecurity = FALSE;

static float64 LongTermAltBeforReboot = 0.0;
static boolean LongTermAltBeforRebootValid = FALSE;

int dfff = 0;
bool bPostproOutput = TRUE;
int32 ConPPOutputCnt = 0;
void TaskPostProc(void)
{
	ECEF pos, vel;
	word256 poschmap={{0,}};
	int32 fixFlag = FIX_NOT;
	FIX_DOP dop={0.0,};
//	double tow=0.0;
	static int32 conLsValidCnt=0;
	double cursystime=getSysOnTime();	
	bPostproOutput = TRUE;

	if(!bNeed2CalcSinglePos)
		return;

	fixFlag = getCurFixPosInfo(&pos,&dop,&poschmap);
	getCurFixVelInfo(&vel,NULL);

	if(getCurFilterType() == FILTER_TYPE_NONE)	//ls
	{
		if(bLsFixCoarseChkOK)
			conLsValidCnt++;
		else
			conLsValidCnt=0;

		if(fixFlag==FIX_NOT)
			bPostproOutput = FALSE;
#if (PVT_MODE == CALC_PVT_KF)
		else if(!IsCurPVTKFValid())
#else
		else if(1)
#endif
		{
			if(((TTFF==0 || (cursystime-TTFF*0.001<5)) && (conLsValidCnt>=2))
				||((SysEnvCon == SYS_CONDITION_SIGNAL) && (conLsValidCnt>=1)) 
				|| ((SysEnvCon == SYS_CONDITION_NORMAL) && (conLsValidCnt>=3))
				)
				bPostproOutput = TRUE;
			else
				bPostproOutput = FALSE;
		}
		else
			bPostproOutput = TRUE;
	}
	else  	//KF
	{
		if(fixFlag == FIX_NOT)
			bPostproOutput = FALSE;
		else
			bPostproOutput = TRUE;

		conLsValidCnt=0;
	}

	if(bPostproOutput)
	{
		if(TTFF <= 0)
			TTFF = (int32)((cursystime-((double)InitCostRFcnt1)*RECIP_RF_SAMP_FREQ)*1000.0);
		if(FixTime <=0)
			FixTime = (int32)((cursystime-((double)InitCostRFcnt2)*RECIP_RF_SAMP_FREQ)*1000.0);

		setRcvrFixInfo(&pos, &vel, &dop, &poschmap, fixFlag);
		
#if (PVT_MODE == CALC_PVT_KF)
		if(getCurFilterType()==FILTER_TYPE_EXTEND || (!IsKfPVTOpen()))
#endif
		{
			CalcTCXOOffset();
			UpdateLongTermAlt();

			SyncNavSysTime();
			Write1PPSToFpga();

			UpdateBD2GPSTimeBias();
			UpdateTFTCycle();
		}
	
		ConPPOutputCnt++;
	}
	else
	{
		setRcvrFixInfo(NULL, NULL, NULL, &poschmap, FIX_NOT);
		ConPPOutputCnt = 0;
	}

	return;
}


static void CalcTCXOOffset(void)
{
	float64 ctu;
	if(getCurFixCtu(&ctu)<FIX_3D)
		return;

	RTTcxo = -1.0 * ctu * (RF_SAMP_FREQ * RECIP_SPEED_OF_LIGHT);

	if(bTCXOValid)
	{
		TCXOOffset = TCXOOffset*(1-FILTER_COEFFICIENT) + RTTcxo * FILTER_COEFFICIENT;
	}
	else
	{
	 	TCXOOffset = RTTcxo;			
		bTCXOValid = TRUE;
	}	

	return;	
}

bool GetTCXOOffset(double* pTcxo)
{
	if(bTCXOValid)
	{
		if(pTcxo != NULL)
			*pTcxo = TCXOOffset;
	}

	return bTCXOValid;
}

void ResetTCXOOffset(void)
{
	bTCXOValid = FALSE;
}

boolean getCurSecurityAltitude(float64 *pAlt)
{
	if(bLongTermAltSecurity && (pAlt != NULL))
		*pAlt = LongTermAveSysAlt;

	return bLongTermAltSecurity;
}



void SetTCXOOffsetFlag(boolean bFlag)
{
	bTCXOValid = bFlag;
	if(!bFlag)
		TCXOOffset = 0.0;
}

void setLongTermAltBeforeReboot(float64* pAlt, boolean flag)
{
	if(pAlt != NULL)
		LongTermAltBeforReboot = *pAlt;
	
	LongTermAltBeforRebootValid = flag;
}

void ClearLongTermAltitude()
{
	LongTermAveSysAltCnt = 0;
	LongTermAltBeforRebootValid = FALSE;
}

boolean getLongTermAltBeforeReboot(float64* pAlt)
{
	if(LongTermAltBeforRebootValid && (pAlt != NULL))
		*pAlt = LongTermAltBeforReboot;

	return LongTermAltBeforRebootValid;
}

void UpdateLongTermAlt(void)
{
	const int32 AltSmoothCnt = 200;
	ECEF rcvrpos={0.0,};
	WGS wgspos={0.0,};

	if(!bFixAtSecBoundary)
		return;
	
	if(getCurFixPosInfo(&rcvrpos,NULL,NULL)==FIX_NOT)
		return;

	if(ECEF2WGS(&rcvrpos,&wgspos)!=0)
		return;

	if(LongTermAveSysAltCnt == 0)
	{
		LongTermAveSysAlt = wgspos.alt;
		LongTermAveSysAltCnt++;
	}
	else
	{
		LongTermAveSysAltCnt++;
		if(LongTermAveSysAltCnt>AltSmoothCnt)
			LongTermAveSysAltCnt = AltSmoothCnt;
	
		LongTermAveSysAlt = wgspos.alt/LongTermAveSysAltCnt + LongTermAveSysAlt*(LongTermAveSysAltCnt-1)/LongTermAveSysAltCnt;
	}

	if(LongTermAveSysAltCnt>10)
		bLongTermAltSecurity = TRUE;

	return;
}

void UpdateBD2GPSTimeBias(void)
{
	FIX_DOP curdop={0.0,};
	word256 poschnmap = {{0,}};
	float64 curbias=0.0;
	int32 bdUsedCnt = 0, gpsUsedCnt = 0;
	int32 bdAverageCN0 = 0, gpsAverageCN0 = 0;
	int32 bdmaxcn0 = 0, gpsmaxcn0 = 0;
	int32 trkch=0, cn0=0;
	boolean bGpsSignalOK = FALSE, bBDSignalOK = FALSE;
	int32 wn=0;
	double tow=0.0;
	byte buflag = (BITMAP_DIM_BU_GPS | BITMAP_DIM_BU_BD2);

	if(!bFixAtSecBoundary)
		return;

	if(getCurFixGps2BDBias(&curbias)==FALSE)
		return;

	if((getCurFixBuFlag() & buflag) != buflag)
		return;
	
	if((getCurFixPosInfo(NULL, &curdop, &poschnmap) < FIX_3D) || (curdop.bdtdop>2.5) || (curdop.gpstdop>2.5) || (curdop.gdop > 5.0))
		return;

	if(SysEnvSta > SYS_OPEN_SKY)
		return;

	for(trkch = 0; trkch < MAXCHANNELS; trkch++)
	{
		if(GetBitWord256(&poschnmap,trkch)==1)
		{	
			cn0 = PVTTrkchInfo[trkch].cn1s;
			if(SV_IsBd2(PVTTrkchInfo[trkch].svid))
			{
				bdUsedCnt++;		
				bdAverageCN0 += cn0;
				bdmaxcn0 = ((cn0>bdmaxcn0) ? cn0 : bdmaxcn0);
			}
			else if(SV_IsGps(PVTTrkchInfo[trkch].svid))
			{
				gpsUsedCnt++;
				gpsAverageCN0 += cn0;
				gpsmaxcn0 = ((cn0>gpsmaxcn0) ? cn0 : gpsmaxcn0);
			}
		}
	}

	if((bdUsedCnt == 0) || (gpsUsedCnt == 0))
		return;

	bdAverageCN0 /= bdUsedCnt;
	gpsAverageCN0 /= gpsUsedCnt;

	if(((gpsUsedCnt > 1) && (gpsAverageCN0 > 30)) || ((gpsUsedCnt == 1) && (gpsAverageCN0 >= 40)))
		bGpsSignalOK = TRUE;
	
	if(((bdUsedCnt > 1) && (bdAverageCN0 > 30)) || ((bdUsedCnt == 1) && (bdAverageCN0 >= 40)))
		bBDSignalOK = TRUE;
	
	if((bdUsedCnt + gpsUsedCnt > 6)  && bGpsSignalOK && bBDSignalOK)
	{
		BdTime2GpsTimeBiasCnt = (BdTime2GpsTimeBiasCnt>=COMPASS_TIMEBIAS_UPDATE_MAXCNT)? (COMPASS_TIMEBIAS_UPDATE_MAXCNT):(BdTime2GpsTimeBiasCnt+1) ;

		if(BdTime2GpsTimeBiasCnt <= 50)
			BdTime2GpsTimeBias = curbias;
		else
			BdTime2GpsTimeBias = BdTime2GpsTimeBias * (BdTime2GpsTimeBiasCnt - 1) / BdTime2GpsTimeBiasCnt + curbias / BdTime2GpsTimeBiasCnt;

		GetCurNavSysTime(NAV_SYS_NONE ,GPSTIME_SYNC_SRC_FIXRFCNT,&tow, &wn);

		BdTime2GpsTimeBiasTs = wn * SECONDS_IN_WEEK + tow;
	}
	
	return;
}


boolean getBDTime2GpsTimeBias(double* pBias, double* pBiasTs)
{
	if(BdTime2GpsTimeBiasCnt >= TIMEBIAS_UPDATE_VALIDCNT)
	{
		if(pBias != NULL)
			*pBias = BdTime2GpsTimeBias;	//m

		if(pBiasTs != NULL)
			*pBiasTs = BdTime2GpsTimeBiasTs;

		return TRUE;
	}

	return FALSE;
}


boolean getGloTime2GpsTimeBias(double* pBias, double* pBiasTs)
{
	if(GloTime2GpsTimeBiasCnt >= TIMEBIAS_UPDATE_VALIDCNT)
	{
		if(pBias != NULL)
			*pBias = GloTime2GpsTimeBias;	//m

		if(pBiasTs != NULL)
			*pBiasTs = GloTime2GpsTimeBiasTs;

		return TRUE;
	}

	return FALSE;
}

boolean getGloTime2BdTimeBias(double* pBias, double* pBiasTs)
{
	if(GloTime2GpsTimeBiasCnt >= TIMEBIAS_UPDATE_VALIDCNT)
	{
		if(pBias != NULL)
			*pBias = GloTime2GpsTimeBias;	//m

		if(pBiasTs != NULL)
			*pBiasTs = GloTime2GpsTimeBiasTs;

		return TRUE;
	}

	return FALSE;
}

void setBDTime2GpsTimeBias(int32 biascnt, float64 bias, float64 biasTs)
{
	BdTime2GpsTimeBias = bias;
	BdTime2GpsTimeBiasTs = biasTs;
	BdTime2GpsTimeBiasCnt = biascnt;

	return;
}

void setGLOTime2GpsTimeBias(int32 biascnt, float64 bias, float64 biasTs)
{
	GloTime2GpsTimeBias = bias;
	GloTime2GpsTimeBiasTs = biasTs;
	GloTime2GpsTimeBiasCnt = biascnt;

	return;

}

void setGLOTime2BdTimeBias(int32 biascnt, float64 bias, float64 biasTs)
{
#if SUPPORT_GLONASS
	GloTime2BdTimeBias = bias;
	GloTime2BdTimeBiasTs = biasTs;
	GloTime2BdTimeBiasCnt = biascnt;
#endif
	return;

}


void GetGNSSPRTimeBias(byte buflag, PR_MODIFY PRModify[3])
{
	memset(PRModify, 0, sizeof(PRModify));

	if(buflag & BITMAP_DIM_BU_GPS_BD2)		//gps + bd mix bu, use bd the same as gps sv.
	{
		if(getBDTime2GpsTimeBias(&(PRModify[1].bias), NULL))
			PRModify[1].bPRModify = TRUE;
	}
#if SUPPORT_GLONASS
	else if(buflag & BITMAP_DIM_BU_GPS_GLO)	//gps + glo mix bu, use glo the same as gps sv.
	{
		if(getGloTime2GpsTimeBias(&(PRModify[2].bias), NULL))
			PRModify[2].bPRModify = TRUE;
	}
	else if(buflag & BITMAP_DIM_BU_BD2_GLO)	//bd + glo mix bu, use glo the same as bd sv.
	{
		if(getGloTime2BdTimeBias(&(PRModify[2].bias), NULL))
			PRModify[2].bPRModify = TRUE;
	}
	else if(buflag & BITMAP_DIM_BU_GPS_BD2_GLO)	//gps + bd + glo mix bu, use glo and bd the same as gps sv.
	{
		if(getBDTime2GpsTimeBias(&(PRModify[1].bias), NULL))
			PRModify[1].bPRModify = TRUE;

		if(getGloTime2GpsTimeBias(&(PRModify[2].bias), NULL))
			PRModify[2].bPRModify = TRUE;
	}
#endif

	return;
}

word32 gTFTModifiedCnt=0;
void UpdateTFTCycle()
{
#ifndef _POSTPROC
#ifndef _SIMULATE
	static bool bModified = FALSE;
	double tow=0.0, _10msres=0.0;
	int32 _01mscnt=0;
	double curTFTRFcnt=getTICLockRFcnt();

	if(!bFixAtSecBoundary)
		return;
	
	if(!GetNavSysTimeByRFCnt(NAV_SYS_NONE,GPSTIME_SYNC_SRC_FIXRFCNT, curTFTRFcnt, &tow, NULL))
		return;

	_10msres = tow*100.0-(int32)(tow*100.0+0.5);
	if(_10msres>0.0)
		_01mscnt = ((int32)(_10msres*100.0+0.5))%100;
	else
		_01mscnt = ((int32)(_10msres*100.0-0.5))%100;

	if(bModified)
	{
		if((_01mscnt==0)||((abs(_01mscnt)%5)!=0))
			return;
	}
	else
	{
		_01mscnt=(_01mscnt/5)*5;
	}

	_01mscnt = 100-_01mscnt-1;
	bModified = TRUE;
		
	if((_01mscnt)<1 || (_01mscnt>=199))//?
		return;


	WriteOICR((word16)(_01mscnt));

	gTFTModifiedCnt++;
#endif
#endif
}

unsigned int g_1PPS_RfCnt;
void Write1PPSToFpga(void)
{
	double NextRFcnt100M;
	double difRFcnt=0.0, difTime=0.0,secondnow,futuretime;
	int32 weeknow;
	unsigned int u32_NextRFcntH,u32_NextRFcntL;
	SYNC_NAVSYS_TIME curSyncTime;
	//static word16 TFTcnt;

	if(bFixAtSecBoundary == TRUE)
	{
		if(GetCurNavSysTime(NAV_SYS_NONE,GPSTIME_SYNC_SRC_FIXRFCNT, &secondnow,&weeknow)==TRUE)
		{
			futuretime = (int)secondnow + 1;

			GetSyncNavSysTime(NAV_SYS_NONE, &curSyncTime);
			difTime = futuretime - curSyncTime.tow;
			difRFcnt=(difTime - ((double)pActiveCPT->SysmCptWorkConfig.Delay1PPS) * (1.0e-9))* GetCurRFSampleFrq100M();
			NextRFcnt100M = curSyncTime.rfcntEx + difRFcnt;



			/*RFcnt100M = getTICLockRFcnt100M();
			GetNavSysTimeBy100MRFCnt(NAV_SYS_NONE,GPSTIME_SYNC_SRC_FIXRFCNT,RFcnt100M,&second,&week);

			difTime = (int)(second+0.5) + 1 - second;

			difRFcnt=(difTime - ((double)pActiveCPT->SysmCptWorkConfig.Delay1PPS) * (1.0e-9))* GetCurRFSampleFrq100M();
			F64ToH32L32(RFcnt100M,&u32_RFcntH,&u32_RFcntL);
			NextRFcnt100M = RFcnt100M + difRFcnt;*/
			//debug
			/*if(gtestTFTcnt!=TFTcnt)
				NextRFcnt100M += 0.01 * 1e8 ;
			TFTcnt = gtestTFTcnt;*/
			//debug end
			F64ToH32L32(NextRFcnt100M,&u32_NextRFcntH,&u32_NextRFcntL);
			g_1PPS_RfCnt = u32_NextRFcntL;


			//printf("cnt is %d",u32_NextRFcntL);

			//LOG_printf(&LOG0,"next is %d,future is %d",u32_NextRFcntL,(unsigned int)futuretime);
#ifndef _POSTPROC
#ifndef _SIMULATE
			Write1PPS(u32_NextRFcntL);
			Write1PPSWidth(10e6);
#endif
#endif
		}
	}
}



