
#include "define.h"
#include <math.h>
#include "constdef.h"
#include "PVT.h"
#include "LsPVT.h"
#include "KfPVT.h"
#include "PVTRcvrInfo.h"
#include "TimeProc.h"
#include "coordinate.h"
#include "PostProc.h"
#include "hcp.h"
#include "cfgpara.h"
#include "global.h"
#include "FrameSync.h"
#include "Novatel.h"
#include "RTKCalBaseline.h"

#ifndef _POSTPROC
#ifndef _SIMULATE
#include "sysbackup.h"
#include "UpdateInfo.h"
#include "acq.h"
#include "HDefine.h"
#include "FpgaReg.h"
#include "CalcSVPVT.h"
#include "FrameDecode.h"
#include "BitSync.h"
#include "GnssTYProcfg.h"
#include "CalcSvPvt.h"
#include "rtk.h"
#else
#include "dataprocess.h"
#endif
#endif

static void PrepareFix(void);

#ifndef _POSTPROC
#ifndef _SIMULATE
static void GetFixObsInfo(void);
static void UpdateSVInfoForFix(void);
#endif
#endif
static int8 DifferenceInfoModify(word256* pTrkChMap);
static void CoarseSelect(word256* pTrkChMap);
static void CheckObsValidity(void);
static void CheckSysEnv(int8* pEnvState);
static void CalcPseudorange(void);
static void CalcPVTByKalmanFilter(void);
static double ConvertFixTranTime(double time);
static boolean CalculateTrByMaxMinTs(double* pTr);
static void CalcFixNavSysTime(PVT_FIX_INFO* pFixInfo);
static void InitPVTKalmanFilter(void);
static boolean IsLsSecurity2InitPVTKF(void);
static void setSVCandidateInfo(void);
static boolean getTowFrmBaseTr(double curRFCnt, double* pCurTime, double* pDifTime);
static void setBaseTr(double RFCnt, double tow);
static boolean CheckLsPosAndVel(byte chkmap, PVT_FIX_INFO* pFixInfo);
static int32 CheckLSQuality(word32 chnmap);
static void CheckPRTsDif(void);
static void CheckPRTsByOrbitType(void);
static void CheckPRTsByFrepoint();
static void FindMaxMinSvTs(byte orbitType, MAX_MIN_TS_INFO* pOrbitTsInfo, float64 trkts[MAXCHANNELS]);
static int32 GetReferInfoForNavBit(PVT_TRKCH_INFO* pCurObsInfo,float64* pTrBase, int32* pWN);
static void GetECASVInfo(PVT_TRKCH_INFO* pECATrkchInfo);
static bool IsNearSecBoundary(double RFCnt);
static void SelectECA2Fix(word256* pPosChMap, word256* pVelChMap);
static boolean CheckClkError2RTCRestoreTime(PVT_FIX_INFO* pFixInfo);
static void ResetRefTRKTs(void);
static double GetHalfChipWidth(UINT16 fredot, int32 trkch);
static void GetTrkchSteadyLockedTime(int32 trkch, PVT_TRKCH_INFO *pCurPVTTrkchInfo, double curRFcnt);
#if SUPPORT_GLONASS
static void SetGLOTleaps();
#endif

#if SUPPORT_GLONASS
static void ProcessGloTleaps();
#endif

OBS_BUF    glStruObs;
PVT_TRKCH_INFO PVTTrkchInfo[MAXCHANNELS_PVT];
PVT_FIX_INFO	PVTFixInfo;

static RFCNT_NAVTIME FixNavSysTime;
double TICLock_RFcnt = 0;
double TICLock_NxRFcnt = 0;

word256 CoarseSelchmap = {{0,}};
double FixCycle = 1.0;

bool bNeed2CalcSinglePos = FALSE;
 
static byte bKFStatus = KF_PVT_UNKNOWN;

static double base_PRcnt = 0.0;
static double base_time_rx = 0.0;

SV_CODE_CARRIER_TRK_STATUS CandidateSVList[MAXCHANNELS];
int32 CandidateSVListLen = 0;
int32 candidate_i, candidate_j, candidate_index; 
double candidate_field_val;

boolean bLsFixQualityOk = FALSE;
boolean bLsFixCoarseChkOK = FALSE;
byte lsPosVelChkPath = 0;
byte ls2InitPVTKFChkPath = 0;

bool bFixAtSecBoundary = FALSE;

//int32 PRErrCnt = 0;
int16 lostByPRErrCnt = 0,lostByOutViewCnt = 0,lostByCrossErrCnt = 0,lostBySNR = 0,lostByDifFre = 0,lostByDupliSV = 0;
int16 lostBySyncChk = 0,lostByLongTimeSync = 0,lostByDummySV = 0,lostByIllSV = 0;
int8 lostByOutViewCnt_trk=-1;
int8 lostByCaif = 0;


int32 checkhispospath = 0;
word32 gobsTFTIdx=0;
UINT16 mycoasting = 0;

void ISRTakeMeasPhase(void)
{
#ifndef _POSTPROC
#ifndef _SIMULATE
    int16 trkch;
    OBS_INFO m_StruObs;
	OBS_TRKCH_INFO* pTrkchObs=NULL;
	int16 idlecntTh = 3; 
#ifdef _FAULT_LOST_LOCK	
	int8 bitsyncSrc=0, bitsyncFlag=0;
	int faultlost_thres = 15;
#endif
	memset(&m_StruObs,0,sizeof(m_StruObs));

	readFTFRFClk(&m_StruObs.RFSampleCnt_L);	//time stamp
	readFTFNxRFClk(&m_StruObs.NxRFSampleCnt_L);	//time stamp

	for (trkch=0; trkch<MAXCHANNELS; trkch++)
	{
		pTrkchObs = &(m_StruObs.chObs[trkch]);
		
       	if (glStruAcqu.Info[trkch].SvID<=0)	// 没有收到卫星信息
			continue;

		readNiosTrkchState(trkch,&(pTrkchObs->snr),&(pTrkchObs->C2FAI),&(pTrkchObs->CCBF),&(pTrkchObs->trksta),&(pTrkchObs->weakcnt), &(pTrkchObs->strongcnt));

		if(pTrkchObs->trksta == TRK_STATE_IDLE)
		{
			pTrkchObs->CCBF = 0;

			if(glStruAcqu.Info[trkch].State == START_TRK_PVT )
				glStruAcqu.Info[trkch].idlecnt++;

			if((glStruAcqu.Info[trkch].FreqPoint == DSP_L2P_FRE) || (glStruAcqu.Info[trkch].FreqPoint == DSP_L1P_FRE)
				|| ((glStruAcqu.Info[trkch].Acqmode >= ACQ_MODE_3) && (glStruAcqu.Info[trkch].Acqmode <= ACQ_MODE_5)))
				idlecntTh = 300;
			else
				idlecntTh = 26;	//

			if((glStruAcqu.Info[trkch].State == TRACKING_PVT) || 
				((glStruAcqu.Info[trkch].State == START_TRK_PVT )&& (glStruAcqu.Info[trkch].idlecnt>idlecntTh)))
			{
				LOOP_loselock(trkch,FALSE);
			}
		}	
		else
		{
			if((pTrkchObs->CCBF & 0x3) == 0x3)
			{
				readTrkchOBS(trkch,&(pTrkchObs->nco),&(pTrkchObs->chipcnt),&(pTrkchObs->NCnt),&(pTrkchObs->MCnt),&(pTrkchObs->decp),&(pTrkchObs->intp));
				if(glStruAcqu.Info[trkch].State < TRACKING_PVT)
					glStruAcqu.Info[trkch].lockedStartTime = GetCurRFSampleCntWord64();
				
				if(glStruAcqu.Info[trkch].State >= ACQUSITION_PVT)
				{
					if(glStruAcqu.Info[trkch].State != TRACKING_PVT)
						Clr_LOOP_ISR_Info(trkch);	
					glStruAcqu.Info[trkch].State = TRACKING_PVT;
				}

#ifdef _FAULT_LOST_LOCK	
				bitsyncFlag = getTrkchBitSyncState(trkch, &bitsyncSrc);

				if(bitsyncFlag==BIT_SYNC_FAIL)
				{
					if((pTrkchObs->trksta == TRK_STATE_5) || (pTrkchObs->trksta == TRK_STATE_2)
						|| (pTrkchObs->trksta == TRK_STATE_6) || (pTrkchObs->trksta == TRK_STATE_3)
						|| (pTrkchObs->trksta == TRK_STATE_7))
					{
						if(glStruAcqu.Info[trkch].FreqPoint == DSP_L1CA_FRE)
							faultlost_thres = 40;
						if((glStruAcqu.Info[trkch].FreqPoint == DSP_G1CA_FRE) || (glStruAcqu.Info[trkch].FreqPoint == DSP_G2CA_FRE))
							faultlost_thres = 80;
						
						if(pTrkchObs->C2FAI <= THETA_THRESH_LOSE)
						{
							glStruAcqu.Info[trkch].faultlostcnt++;
							if(glStruAcqu.Info[trkch].faultlostcnt > faultlost_thres)
							{
								lostByCaif++;
								LOOP_loselock(trkch,TRUE);
							}
						}
						else
							glStruAcqu.Info[trkch].faultlostcnt >>= 1;
					}
				}
				else
					glStruAcqu.Info[trkch].faultlostcnt = 0;

				if(glStruAcqu.Info[trkch].FreqPoint == DSP_L2P_FRE)
				{
					if(pTrkchObs->C2FAI <= 10)
					{
						glStruAcqu.Info[trkch].faultlostcnt++;
						if(glStruAcqu.Info[trkch].faultlostcnt > faultlost_thres)
						{
							lostByCaif++;
							LOOP_loselock(trkch,TRUE);
						}
					}
					else
						glStruAcqu.Info[trkch].faultlostcnt >>= 1;
				}
#endif				
			}

			pTrkchObs->SvID = glStruAcqu.Info[trkch].SvID;
			pTrkchObs->FreqPoint = glStruAcqu.Info[trkch].FreqPoint;
		}
	}
	
	// 将原始观测量放入环形buffer
    memcpy(&glStruObs.Obs[glStruObs.SetIndex++], &m_StruObs, sizeof(m_StruObs));
    if (glStruObs.SetIndex >= OBS_BUF_CNT)
	{
		glStruObs.SetIndex = 0;
    }
#endif
#endif
    return;
}


void TaskPVT(void)
{	
	PrepareFix();

	if(!bNeed2CalcSinglePos)
		return;
	
#if (PVT_MODE == CALC_PVT_KF)
	if((!IsKfPVTOpen()) || (!IsCurPVTKFValid()))
	{
		bKFStatus = KF_PVT_UNKNOWN;
		ResetPVTKF();
#endif
		LsSVSelection(&CoarseSelchmap, &(PVTFixInfo.poschmap), PVTTrkchInfo);
		memcpy(&(PVTFixInfo.velchmap), &(PVTFixInfo.poschmap), sizeof(PVTFixInfo.velchmap));

		SelectECA2Fix(&(PVTFixInfo.poschmap), &(PVTFixInfo.velchmap));
		
		PVTFixInfo.buflag = CheckMixFixSVAndBuFlag(PVTTrkchInfo, &(PVTFixInfo.poschmap));	

		//WLS pos, vel, DOP calculate
		LSCalcRcvrPos(PVTTrkchInfo, &PVTFixInfo);

		if(PVTFixInfo.posres != FIX_NOT)
		{
			LSCalcRcvrVel(PVTTrkchInfo, &PVTFixInfo);
			CalcFixDOP(PVTFixInfo.buflag, &(PVTFixInfo.poschmap), PVTTrkchInfo, PVTFixInfo.rcvrpos, &(PVTFixInfo.dop));

			//CheckLSQuality(PVTFixInfo.poschmap);

			CalcFixNavSysTime(&PVTFixInfo);
		}
	
		if((PVTFixInfo.posres != FIX_NOT)
			&& CheckClkError2RTCRestoreTime(&PVTFixInfo)
			&& CheckNavHisPos(&PVTFixInfo)
			//&& CheckLSECAAltitude()
			)	
		{
			//bLsFixQualityOk = CheckLsFixQuality(FIX_POS_VEL);
			bLsFixCoarseChkOK = CheckLsPosAndVel(FIX_POS_VEL, &PVTFixInfo);
	#if (PVT_MODE == CALC_PVT_KF)
			if(IsKfPVTOpen())
				InitPVTKalmanFilter();		
	#endif
		}
		else
		{
			bLsFixQualityOk = FALSE;
			bLsFixCoarseChkOK = FALSE;					
		}
#if (PVT_MODE == CALC_PVT_KF)
	}
	else
	{	
		{	//please add KF PVT sv selection here
			memcpy(&(PVTFixInfo.poschmap), &CoarseSelchmap, sizeof(PVTFixInfo.poschmap));
			memcpy(&(PVTFixInfo.velchmap), &CoarseSelchmap, sizeof(PVTFixInfo.velchmap));
			SelectECA2Fix(&(PVTFixInfo.poschmap), &(PVTFixInfo.velchmap));
			//DelSVByUraIdx(&(KFPVTInfo.poschnmap));
		}
			
		CalcPVTByKalmanFilter();

		//calculate the navigation system time.
		if((PVTFixInfo.posres != FIX_NOT) && (PVTFixInfo.velres != FIX_NOT))
			CalcFixNavSysTime(&PVTFixInfo);
	
	}
#endif

	return;
}

bool IsNeed2CalcSignlePos(void)
{
	bool ret = FALSE;

	if(IsBaseSation() || (pActiveCPT->SysmCptWorkConfig.FixUpdateCycle >= 50))
		ret = TRUE;
	else if(getRcvrInfo(NULL,NULL,NULL,NULL)>=FIX_WIDE_INT)
	{
		if(bFixAtSecBoundary
			||(GetCurNavSysTime(NAV_SYS_NONE,GPSTIME_SYNC_SRC_FIXRFCNT,NULL,NULL)==FALSE)
			||(getRcvrInfo(NULL,NULL,NULL,NULL)==FIX_NOT))
			ret = TRUE;
		else
			ret = FALSE;
	}
	else
		ret = TRUE;

	return ret;
}

void PrepareFix(void)
{
#ifndef _POSTPROC
#ifndef _SIMULATE
	InitPVTTask();
	GetFixObsInfo();
	UpdateSVInfoForFix();
	ProcessUncon();//sv cross check.
#endif
#endif
	CheckObsValidity();

	bNeed2CalcSinglePos = IsNeed2CalcSignlePos();

	if(bNeed2CalcSinglePos)
	{
		CheckSysEnv(&SysEnvSta);
		CoarseSelect(&CoarseSelchmap);

		CalcPseudorange();
		DifferenceInfoModify(&CoarseSelchmap);
	}
	
	return;
}

void InitPVTTask(void)
{
	bLsFixQualityOk = FALSE;
	lsPosVelChkPath = 0;
	ls2InitPVTKFChkPath = 0;
	bFixAtSecBoundary = FALSE;
	gobsTFTIdx =0;
	bNeed2CalcSinglePos = FALSE;

	memset(&CoarseSelchmap, 0, sizeof(CoarseSelchmap));
	memset(PVTTrkchInfo, 0, sizeof(PVTTrkchInfo));
	memset(&PVTFixInfo, 0, sizeof(PVTFixInfo));
	PVTFixInfo.dop.gdop = PVTFixInfo.dop.hdop = PVTFixInfo.dop.vdop = PVTFixInfo.dop.pdop = PVTFixInfo.dop.gpstdop = PVTFixInfo.dop.bdtdop =99.9;
	#if SUPPORT_GLONASS
	PVTFixInfo.dop.glotdop = 99.9;
	#endif
	memset(&FixNavSysTime, 0, sizeof(FixNavSysTime));
	
	return;
}

#ifndef _POSTPROC
#ifndef _SIMULATE
int32 GetWNFrmRefSVEph(int32 svid, double tr)
{
	int32 toe = 0, wn = 0;
#if SUPPORT_GLONASS
	double tr_tod=0.0, tk=0.0, tow=0.0;
	int32 NT=0, N4=0;

	if(SV_IsGlo(svid))
	{
		N4 = Sys3AlmEphUTCInfo.Glo_EphStruct[svid-MinGloFreID].N4;
		NT = Sys3AlmEphUTCInfo.Glo_EphStruct[svid-MinGloFreID].NT;
		tk = Sys3AlmEphUTCInfo.Glo_EphStruct[svid-MinGloFreID].tk;
		
		GpsToGloTimeTod(tr, &tr_tod);
		
		if((tr_tod-tk)<-SECONDS_IN_HALF_DAY)
			NT++;
		else if((tr_tod-tk)>SECONDS_IN_HALF_DAY)
			NT--;

		GloToGpsTime(N4,NT,tr_tod, &wn, &tow);		
	}
	else
#endif
	{
		toe = Sys3AlmEphUTCInfo.GpsBd2_EphStruct[svid-MinGpsSvID].toe;
		wn = Sys3AlmEphUTCInfo.GpsBd2_EphStruct[svid-MinGpsSvID].wkn;
		if(SV_IsGps(svid))
		{
			if((tr<3*SECONDS_IN_HOUR) && ((toe<SECONDS_IN_HOUR) || (toe>SECONDS_IN_HALF_WEEK)))
				wn++;
		}
		else if(SV_IsBd2(svid))
		{
			if((toe>SECONDS_IN_HALF_WEEK) && (tr<3*SECONDS_IN_HOUR))
				wn++;
		}
	}

	return wn;
}

static int32 CalcChObsBitcnt(int32 trkch, int32 svid, int32 frepoint, double curRFcnt, double baseRFcnt, int32 baseBitcnt, int32 chipcnt)
{
	int32 difbitcnt = 0;
	float64 dif = 0.0;
	double tmpbase=0.0, diftime=0.0;
	double halfchipwidth = GetHalfChipWidth(frepoint, trkch);

	tmpbase = RFSampleCnt.high*TWO_P32+baseRFcnt;
	diftime = (curRFcnt-tmpbase);

	if(diftime>TWO_P31)
		diftime -= TWO_P32;
	else if(diftime<(-TWO_P31))
		diftime += TWO_P32;

	dif = (diftime*RECIP_RF_SAMP_FREQ-((double)chipcnt)*halfchipwidth)*1000.0;

	if(dif<0)
		difbitcnt = (int32)(dif-0.5);
	else
		difbitcnt = (int32)(dif+0.5);

	return difbitcnt;	//1ms cnt
}

static OBS_INFO LastObs={0,0,{{0,},}};

void ResetTrkchLastObs(int32 trkch)
{
	int idx;
	memset(&(LastObs.chObs[trkch]), 0, sizeof(OBS_TRKCH_INFO));

	for(idx = 0;idx < OBS_BUF_CNT;idx++)
	{
		memset(&(glStruObs.Obs[idx].chObs[trkch]),0,sizeof(OBS_TRKCH_INFO));
		glStruObs.Obs[idx].chObs[trkch].SvID = -1;
	}
}

static void CalcChObsFd(OBS_INFO* pCurObs)
{
	double diftime = 0.0,  difp=0;
	int32 trkch=0;

	if(LastObs.RFSampleCnt_L == 0)
	{
		memcpy(&LastObs, pCurObs, sizeof(LastObs));	
		memset(pCurObs, 0, sizeof(OBS_INFO));	//make cur obs invalid and not to use.
		return;
	}

	//dif time
	if(pCurObs->RFSampleCnt_L>LastObs.RFSampleCnt_L)
		diftime = (pCurObs->RFSampleCnt_L-LastObs.RFSampleCnt_L)*RECIP_RF_SAMP_FREQ;
	else
		diftime = (pCurObs->RFSampleCnt_L + TWO_P32 - LastObs.RFSampleCnt_L)*RECIP_RF_SAMP_FREQ;

	if((diftime < 1.0e-6) || (diftime>1.5))
	{
		memcpy(&LastObs, pCurObs, sizeof(LastObs));	
		memset(pCurObs, 0, sizeof(OBS_INFO));	//make cur obs invalid and not to use.
		return;
	}

	//dop
	for(trkch=0; trkch<MAXCHANNELS; trkch++)
	{
		if(((pCurObs->chObs[trkch].CCBF & 0x3) == 0x3) && (LastObs.chObs[trkch].CCBF & 0x3 == 0x3)
			&& (pCurObs->chObs[trkch].SvID == LastObs.chObs[trkch].SvID))
		{
			pCurObs->chObs[trkch].bvalid = TRUE;
			
			if(pCurObs->chObs[trkch].intp < LastObs.chObs[trkch].intp)
			{
				pCurObs->chObs[trkch].intpVersCnt = LastObs.chObs[trkch].intpVersCnt+1;
				difp = ((double)pCurObs->chObs[trkch].intp - (double)LastObs.chObs[trkch].intp + TWO_P24) 
					+ ((double)pCurObs->chObs[trkch].decp-(double)LastObs.chObs[trkch].decp)*RECIP_P65536;
			}
			else
			{
				pCurObs->chObs[trkch].intpVersCnt = LastObs.chObs[trkch].intpVersCnt;
				difp = ((double)pCurObs->chObs[trkch].intp - (double)LastObs.chObs[trkch].intp) + 
					((double)pCurObs->chObs[trkch].decp-(double)LastObs.chObs[trkch].decp)*RECIP_P65536;
			}
				
			pCurObs->chObs[trkch].fd = difp/diftime - GetFreqpointIF(pCurObs->chObs[trkch].SvID,pCurObs->chObs[trkch].FreqPoint);
		}
		else
		{
			pCurObs->chObs[trkch].bvalid = FALSE;
			pCurObs->chObs[trkch].fd = 0.0;
			pCurObs->chObs[trkch].intpVersCnt = 0;
		}
	}

	memcpy(&LastObs, pCurObs, sizeof(LastObs));

	return;
}

double GetHalfChipWidth(UINT16 fredot, int32 trkch)
{
	double halfChipWidth=RECIP_P4092000;
	
	if((fredot == DSP_B1I_FRE) || (fredot == DSP_B2I_FRE))
		halfChipWidth = RECIP_P4092000;
	else if(fredot == DSP_B3I_FRE)
		halfChipWidth = RECIP_P20460000;
	else if(fredot == DSP_L1CA_FRE)
	{
		if((trkch>=L1CA_TRKID_MIN) && (trkch<=L1CA_TRKID_MAX))
			halfChipWidth = RECIP_P20460000;
		else
			halfChipWidth = RECIP_P2046000;
	}
	else if(fredot == DSP_L2C_FRE)
		halfChipWidth = RECIP_P1023000;
	else if((fredot == DSP_L1P_FRE) || (fredot == DSP_L2P_FRE) || (fredot == DSP_L5_FRE))
		halfChipWidth = RECIP_P20460000;
#if SUPPORT_GLONASS
	else if((fredot == DSP_G1CA_FRE) || (fredot == DSP_G2CA_FRE))
		halfChipWidth = RECIP_P1022000;
#endif

	return halfChipWidth;
}

double CalcSVTs(int32 trkch, int32 svid, int32 fredot, int32 framenum, int32 bitnum, int32 Mcnt, int32 Ncnt, int32 codechip, int32 codenco)
{
	double ts=0.0;
	int32 _1msnum=0;
	double bitWidth = 0.02, sfwidth = 6.0;
	double halfChipWidth = GetHalfChipWidth(fredot, trkch);
	
	_1msnum = Mcnt*20+Ncnt;

	if(SV_IsBd2Geo(svid))	//GEO
	{
		sfwidth = 0.6;
		bitWidth = 0.002; 
		//_1msnum = _1msnum%2;
	}
#if SUPPORT_GLONASS
	else if(SV_IsGlo(svid))	//Glonass
	{
		sfwidth = 2.0;
		bitWidth = 0.01;
		//_1msnum = _1msnum%10;
	}
#endif
	else					//GPS, MEO
	{
		sfwidth = 6.0;
		bitWidth = 0.02;
		//_1msnum = _1msnum%20;
	}
	
	ts = ((double)framenum)*sfwidth + ((double)bitnum)*bitWidth + ((double)_1msnum)*0.001 + ((double)codechip+((double)codenco)*RECIP_P65536)*halfChipWidth;

	return ts;
}


double baseTr_gps = 0;
int32 baseWn_gps = 0;
int32 baseSV_gps = -1;
void GetFixObsInfo(void)
{
	int32 trkch, pvtIntercnt;
	OBS_INFO* pCurObsInfo=NULL;
	OBS_INFO* pCurObsInfotmp=NULL;
	int32 svid=-1;
	UINT32 codenco = 0, bitnum = 0, codechip = 0;                        // 码chip寄存器
    int32 framenum = 0;                        // 码frame寄存器
    int32 Mnum = 0, Nnum=0;
	int32 baseBitcnt=0;
	word32 baseRFCnt=0;
	double curTicLockRFcnt=0.0, tmpTicLockRFcnt=0.0, curTicLockNxRFcnt=0.0;
	//bool bNavTimeValid = FALSE;
#if SUPPORT_GLONASS
	int32 wn=0;
	double tow=0.0;
#endif
	int16 readix=0, writeidx=0;
	int32 curopcnt=0;
	
	baseTr_gps =0;
	baseSV_gps = -1;
	gobsTFTIdx=0;

	readix = glStruObs.CurIndex;
	writeidx = glStruObs.SetIndex;
	if(readix == writeidx)
	{
		return;
	}
		
	pvtIntercnt = writeidx-readix;
	if(pvtIntercnt<0)
		pvtIntercnt+=OBS_BUF_CNT;

	//collect the average snr
	while (readix != writeidx)
    {
    	curopcnt++;
        pCurObsInfotmp = &glStruObs.Obs[readix];

		for(trkch=0; trkch<MAXCHANNELS; trkch++)
		{
			if(pCurObsInfotmp->chObs[trkch].CCBF & 0x3 == 0x3)
				PVTTrkchInfo[trkch].lockedcnt++;
			else
				PVTTrkchInfo[trkch].lockedcnt=0;
		}

		UpdateRFSampleCnt(pCurObsInfotmp->RFSampleCnt_L);
		UpdateRFNxCnt(pCurObsInfotmp->NxRFSampleCnt_L);
		
		tmpTicLockRFcnt = RFSampleCnt.high*TWO_P32 + RFSampleCnt.low;
		curTicLockNxRFcnt = RFNxSampleCnt.high*TWO_P32 + RFNxSampleCnt.low;

		CalcChObsFd(pCurObsInfotmp);

		readix++;
		if (readix >= OBS_BUF_CNT)
			readix = 0;

		glStruObs.CurIndex = readix;

		if(IsNearSecBoundary(tmpTicLockRFcnt))
		{
			pCurObsInfo = pCurObsInfotmp;
			curTicLockRFcnt = tmpTicLockRFcnt;
			TICLock_NxRFcnt = curTicLockNxRFcnt;
			gobsTFTIdx = curopcnt;
			break;
		}
	}


	if(pCurObsInfo == NULL)
	{
		pCurObsInfo = pCurObsInfotmp;
		curTicLockRFcnt = tmpTicLockRFcnt;
		TICLock_NxRFcnt = curTicLockNxRFcnt;
		gobsTFTIdx = curopcnt;
	}

	//get fix cycle
	if(TICLock_RFcnt < MICRO_NUM)
		FixCycle = 0.5;
	else
		FixCycle = (curTicLockRFcnt - TICLock_RFcnt)/RF_SAMP_FREQ;

	TICLock_RFcnt = curTicLockRFcnt;


	// get the obs items
	for(trkch=0; trkch<MAXCHANNELS; trkch++)
	{
		if(IsNoiseLoop(trkch))
			continue;
		
		svid = pCurObsInfo->chObs[trkch].SvID;	//gps:1~32, bd: 32~64
		if((pCurObsInfo->chObs[trkch].bvalid==FALSE) 
			|| ((pCurObsInfo->chObs[trkch].FreqPoint & pActiveCPT->SysmCptWorkConfig.NavFreqPoint) == 0)
			|| (pCurObsInfo->chObs[trkch].trksta == TRK_STATE_IDLE)
			|| (svid>SV_NUM))
			continue;
		
		PVTTrkchInfo[trkch].svid = svid;	//need to arrange the svid.

		PVTTrkchInfo[trkch].fd = pCurObsInfo->chObs[trkch].fd;
		PVTTrkchInfo[trkch].el = SVInfo[svid-1].el;
		PVTTrkchInfo[trkch].az = SVInfo[svid-1].az;
		
		PVTTrkchInfo[trkch].freq_point = pCurObsInfo->chObs[trkch].FreqPoint;
		//PVTTrkchInfo[trkch].branch = pCurObsInfo->branch;
		PVTTrkchInfo[trkch].trkmode = pCurObsInfo->chObs[trkch].trksta;
		PVTTrkchInfo[trkch].CCBF = pCurObsInfo->chObs[trkch].CCBF;
		PVTTrkchInfo[trkch].C2FAI = pCurObsInfo->chObs[trkch].C2FAI/10;
		PVTTrkchInfo[trkch].strongcnt = pCurObsInfo->chObs[trkch].strongcnt;
		PVTTrkchInfo[trkch].weakcnt = pCurObsInfo->chObs[trkch].weakcnt;
		PVTTrkchInfo[trkch].cn100ms = (UINT8)((float)(log10((float)pCurObsInfo->chObs[trkch].snr) * 10.0)+0.5);
		if(PVTTrkchInfo[trkch].cn1s == 0)
			PVTTrkchInfo[trkch].cn1s = PVTTrkchInfo[trkch].cn100ms;

		GetTrkchSteadyLockedTime(trkch,&(PVTTrkchInfo[trkch]),TICLock_RFcnt);

		//get bitcnt sync info 
		SWI_disable();
		PVTTrkchInfo[trkch].synflag = getTrkchSyncInfo(trkch, &baseBitcnt, &baseRFCnt, &framenum, &(PVTTrkchInfo[trkch]));
		SWI_enable();
		
		// ts, doppler
		if ((PVTTrkchInfo[trkch].synflag == SF_SYNC_SUCCESS) && (pCurObsInfo->chObs[trkch].bvalid))
			PVTTrkchInfo[trkch].bPRValid = TRUE;
		else
			PVTTrkchInfo[trkch].bPRValid = FALSE;
		
		codenco = pCurObsInfo->chObs[trkch].nco;
		codechip = pCurObsInfo->chObs[trkch].chipcnt;
		Mnum = pCurObsInfo->chObs[trkch].MCnt;
		Nnum = pCurObsInfo->chObs[trkch].NCnt;

		PVTTrkchInfo[trkch].bitRFcnt = baseRFCnt;
		if(PVTTrkchInfo[trkch].synflag == SF_SYNC_SUCCESS)
		{
			#if(0)
				bitnum = CalcChObsBitcnt(trkch, svid, PVTTrkchInfo[trkch].freq_point, TICLock_RFcnt, baseRFCnt, baseBitcnt, codechip);
			#else
			{
				bitnum = baseBitcnt;
				Nnum = CalcChObsBitcnt(trkch, svid, PVTTrkchInfo[trkch].freq_point, TICLock_RFcnt, baseRFCnt, baseBitcnt, codechip);
				Mnum = 0;
			}
			#endif
		}
		else
		{
			bitnum = 0;
			framenum = 0;
		}
		
		//test code
		PVTTrkchInfo[trkch].codenco = codenco;
		PVTTrkchInfo[trkch].codechip = codechip;
		PVTTrkchInfo[trkch].Mcnt = Mnum;
		PVTTrkchInfo[trkch].Ncnt = Nnum;
		PVTTrkchInfo[trkch].bitnum = bitnum;
		PVTTrkchInfo[trkch].framenum = framenum;
		//test code end

		PVTTrkchInfo[trkch].ts = CalcSVTs(trkch, svid, PVTTrkchInfo[trkch].freq_point, framenum, bitnum, Mnum, Nnum, codechip, codenco);

		if(SV_IsBd2(svid))
		{
			PVTTrkchInfo[trkch].ts += GPS_BD_SYSTIME_OFFSET;
		}
		else if(PVTTrkchInfo[trkch].freq_point &(DSP_L2C_FRE | DSP_L2P_FRE))
		{
			PVTTrkchInfo[trkch].ts += ((double)pActiveCPT->SysmCptWorkConfig.Delay_L2)*(1.0e-9);
		}
	#if SUPPORT_GLONASS
		else if(SV_IsGlo(svid))
		{
			if(CalcGloSVToGpsTime(svid, PVTTrkchInfo[trkch].ts, &wn, &tow))
			{
				PVTTrkchInfo[trkch].ts = tow;
			}
			else
				PVTTrkchInfo[trkch].bPRValid = FALSE;
		}
	#endif
			
		//carrier phase, carrier integer cycle
		PVTTrkchInfo[trkch].carrInt = pCurObsInfo->chObs[trkch].intp;
		PVTTrkchInfo[trkch].carrDec = pCurObsInfo->chObs[trkch].decp;
		PVTTrkchInfo[trkch].carrIntInvCnt = pCurObsInfo->chObs[trkch].intpVersCnt;

	}
	//////+Tleaps for glonass sys

	#if SUPPORT_GLONASS
	ProcessGloTleaps();
	#endif

	//get navsystem time from SV ts.
	if(!GetNavSysTimeByRFCnt(NAV_SYS_NONE, GPSTIME_SYNC_SRC_REFBIT_RFCNT,TICLock_RFcnt, &baseTr_gps,NULL))	
	{
		baseSV_gps = GetReferInfoForNavBit(PVTTrkchInfo,&baseTr_gps, &baseWn_gps);
		if(baseSV_gps != -1)
		{			
			SyncBDTime.syncsrc = GPSTIME_SYNC_SRC_REFBIT_RFCNT;			
			SyncBDTime.rfcnt = TICLock_RFcnt;
			//SyncBDTime.rfcntEx = TICLock_NxRFcnt;
			SyncBDTime.tow = baseTr_gps;
			SyncBDTime.wn = baseWn_gps;
			memcpy(&SyncGPSTime,&SyncBDTime, sizeof(SyncBDTime));
		}
	}

	for(trkch=0; trkch<MAXCHANNELS; trkch++)
	{	
		if(PVTTrkchInfo[trkch].lockedcnt == pvtIntercnt)
		{
			ClearSVACQFailCnt(PVTTrkchInfo[trkch].svid,PVTTrkchInfo[trkch].freq_point);
		}
	}

	//GetECASVInfo(&(PVTTrkchInfo[ECA_TRKCH_ID]));

	return;
}

#if SUPPORT_GLONASS
void ProcessGloTleaps()
{
	int32 trkch = 0, maxcn1s=0;
	int32 tLeaps = 0;
	double tsGps = 0.0,tsMinGlo=0.0;
	int32 svid = 0;
	bool bBaseTime = FALSE, bMinGloTs = FALSE;
	double curTFTRFcnt = getTICLockRFcnt();

	if(pActiveCPT->SysmCptWorkConfig.NavFreqPoint & FREQ_GROUP_GLO == 0)
		return;

	// ..1 there is navtime of glonass
	if(!GetNavSysTimeByRFCnt(NAV_SYS_NONE,GPSTIME_SYNC_SRC_REFBIT_RFCNT,curTFTRFcnt, &tsGps, NULL))
	{
		for(trkch=0; trkch<MAXCHANNELS; trkch++)
		{
			if (PVTTrkchInfo[trkch].bPRValid != TRUE)
				continue;
			
			svid = PVTTrkchInfo[trkch].svid;
			if (!SV_IsGlo(svid))
			{
				if ((maxcn1s == 0)||(PVTTrkchInfo[trkch].cn1s >maxcn1s))
				{
					tsGps = PVTTrkchInfo[trkch].ts;
					bBaseTime = TRUE;
				}
			}
			else	 //find the min of glonass ts
			{
				if ((tsMinGlo<MICRO_NUM)||(tsMinGlo>PVTTrkchInfo[trkch].ts))
				{
					tsMinGlo = PVTTrkchInfo[trkch].ts;
					bMinGloTs = TRUE;
				}
			}
		}

		if((!bBaseTime) && bMinGloTs)
		{
			tsGps = tsMinGlo;
			bBaseTime = TRUE;
		}
	}
	else
	{
		bBaseTime = TRUE;
	}

	for(trkch=0; trkch<MAXCHANNELS; trkch++)
	{
		if((PVTTrkchInfo[trkch].bPRValid == FALSE) || (!SV_IsGlo(PVTTrkchInfo[trkch].svid)))
			continue;
		
		if (bBaseTime)
		{
			tLeaps = (int32)(tsGps - PVTTrkchInfo[trkch].ts + 0.5) ; //

			if(tLeaps>SECONDS_IN_HALF_WEEK)
				tLeaps -= SECONDS_IN_WEEK;
			else if(tLeaps<-SECONDS_IN_HALF_WEEK)
				tLeaps += SECONDS_IN_WEEK;
								
			PVTTrkchInfo[trkch].ts += tLeaps ;
			PVTTrkchInfo[trkch].Tleaps = tLeaps;
		}
		else
			PVTTrkchInfo[trkch].bPRValid = FALSE;
	}
	
	SetGLOTleaps();	///global.Tleaps out
	
	
	return;
} 

void SetGLOTleaps()
{
	int32 trkch = 0, tmpTleapsnew = 100 ,trkchnew = -1,freslot = -1;

	for(trkch=0; trkch<MAXCHANNELS; trkch++)
	{
		if((PVTTrkchInfo[trkch].bPRValid == FALSE) || (!SV_IsGlo(PVTTrkchInfo[trkch].svid))||(PVTTrkchInfo[trkch].Tleaps ==0))
			continue;

		if(tmpTleapsnew > PVTTrkchInfo[trkch].Tleaps)
		{
			tmpTleapsnew = PVTTrkchInfo[trkch].Tleaps;	
			trkchnew = trkch;
		}
	}

	if(tmpTleapsnew != 100)
	{
		Global_TleapsGLO = tmpTleapsnew;
		freslot = PVTTrkchInfo[trkchnew].svid;
		Global_DayNTGLO = Sys3AlmEphUTCInfo.Glo_EphStruct[freslot].NT;
		Global_DayN4GLO = Sys3AlmEphUTCInfo.Glo_EphStruct[freslot].N4;
	}

	return;
}



#endif

#endif
#endif

#ifndef _POSTPROC
#ifndef _SIMULATE
void UpdateSVInfoForFix(void)
{
	int32 trkch = 0, sv=0,trkcompar=0;
	word256 svmap={{0,0,0,0,0,0,0,0}};
	boolean bNeedCalSVInfo=FALSE;
	int16 trkmap[MAXCHANNELS];

	memset(trkmap,0,sizeof(trkmap));
	
	for(trkch = 0; trkch < MAXCHANNELS; trkch++)
	{
		sv = PVTTrkchInfo[trkch].svid;

		if((!PVTTrkchInfo[trkch].bPRValid) 
			||(CheckEphHealth(sv)!=1) 
#if SUPPORT_GLONASS
			||(SV_IsGlo(sv)&&(SVInfo[sv-1].eph_age>GLO_SV_EPH_AGE_USABLE_THRES))
#endif
			||(SVInfo[sv-1].eph_age>SV_EPH_AGE_USABLE_THRES))		
		{
			PVTTrkchInfo[trkch].bPRValid = FALSE;
			continue;
		}

		bNeedCalSVInfo=TRUE;
		PVTTrkchInfo[trkch].bPRValid = TRUE;

		//update sv clock drift, sv pos and vel.
		if (GetBitWord256(&svmap, sv-1)==1)
		{
			trkcompar = trkmap[sv-1];
			if (abs(PVTTrkchInfo[trkch].ts - PVTTrkchInfo[trkcompar].ts) <= 1e-5)
			{	
				PVTTrkchInfo[trkch].svpos = PVTTrkchInfo[trkcompar].svpos;
				PVTTrkchInfo[trkch].svvel = PVTTrkchInfo[trkcompar].svvel;
				PVTTrkchInfo[trkch].clkerr = PVTTrkchInfo[trkcompar].clkerr;
				PVTTrkchInfo[trkch].frqerr = PVTTrkchInfo[trkcompar].frqerr;
				PVTTrkchInfo[trkch].toertk= PVTTrkchInfo[trkcompar].toertk;
				bNeedCalSVInfo = FALSE;				
			}
		}

		if(bNeedCalSVInfo)
		{
			SetBitWord256(&svmap, sv-1);
			trkmap[sv-1]=trkch;
		
			SWI_disable();
		#if SUPPORT_GLONASS
			if(SV_IsGlo(sv))
				CalcSV_PVT_Eph(sv, PVTTrkchInfo[trkch].ts -PVTTrkchInfo[trkch].Tleaps,  &(PVTTrkchInfo[trkch].svpos), &(PVTTrkchInfo[trkch].svvel), &(PVTTrkchInfo[trkch].clkerr), &(PVTTrkchInfo[trkch].frqerr),&(PVTTrkchInfo[trkch].toertk));	
			else
		#endif
				CalcSV_PVT_Eph(sv, PVTTrkchInfo[trkch].ts,  &(PVTTrkchInfo[trkch].svpos), &(PVTTrkchInfo[trkch].svvel), &(PVTTrkchInfo[trkch].clkerr), &(PVTTrkchInfo[trkch].frqerr),&(PVTTrkchInfo[trkch].toertk));
			SWI_enable();
		}

		PVTTrkchInfo[trkch].ts -= PVTTrkchInfo[trkch].clkerr;
		//PVTTrkchInfo[trkch].fd += PVTTrkchInfo[trkch].frqerr;
		
		if(PVTTrkchInfo[trkch].ts < MICRO_NUM)
			PVTTrkchInfo[trkch].ts += SECONDS_IN_WEEK;
		else if(PVTTrkchInfo[trkch].ts >= SECONDS_IN_WEEK)
			PVTTrkchInfo[trkch].ts -= SECONDS_IN_WEEK;
	}
	
	if(getRcvrInfo(NULL,NULL,NULL,NULL)==FIX_NOT)
	{	
		UpdateSV();
	}

	return;
}
#endif
#endif

void CheckObsValidity(void)
{
	CheckPRTsByOrbitType();
	CheckPRTsByFrepoint();
	if(GetNavSysTimeByRFCnt(NAV_SYS_NONE,GPSTIME_SYNC_SRC_GLO_ONLY, TICLock_RFcnt, NULL, NULL) && (getRcvrHistoricalPos(NULL, NULL) <= HISTORICAL_POS_LEVEL_40KM))
		CheckPRTsDif();
	
	return;
}

static void CheckPRTsDif(void)
{	
	bool bTimeValid = TRUE;
	bool bHisPosValid = TRUE;
	float64 tsErr_by_time=0.0, tsErr_by_dis=0.0;
	int32 trkch=0, svid=0;
	ECEF hisPos={0.0,0.0,0.0},svpos={0.0,0.0,0.0};
	float64 gpsbd_fixtime = 0.0, glo_fixtime = 0.0, fixtime=0.0;
	bool bGpsBdTValid = FALSE;
#if SUPPORT_GLONASS
	bool bGloTValid = FALSE;
	byte glotimeSrc = 0; 
#endif
	word256 errsvmap={{0,}};
	float64 avrtsdif = 0.0, tsdif_th=1000.0;
	int32 tsdifsvcnt=0;
	int32 weaktsdifsvcnt = 0;
	
	bGpsBdTValid = GetNavSysTimeByRFCnt(NAV_SYS_BD|NAV_SYS_GPS, GPSTIME_SYNC_SRC_FIXRFCNT, TICLock_RFcnt, &gpsbd_fixtime, NULL);
#if SUPPORT_GLONASS
	bGloTValid = GetNavSysTimeByRFCnt(NAV_SYS_GLO,GPSTIME_SYNC_SRC_GLO_ONLY, TICLock_RFcnt, &glo_fixtime, NULL);
	glotimeSrc = GetSyncNavSysTimeSrc(NAV_SYS_GLO);
	if((glotimeSrc == GPSTIME_SYNC_SRC_GLO_ONLY) || (glotimeSrc>=GPSTIME_SYNC_SRC_RTCPULSE_ERR05MS))
		bGloTValid = TRUE;
	else
		bGloTValid = FALSE;

	if(!bGloTValid)
	{
		bGloTValid = bGpsBdTValid;
		glo_fixtime = gpsbd_fixtime;
	}
#endif
	getRcvrHistoricalPos(&hisPos,NULL);

	for(trkch=0; trkch<MAXCHANNELS; trkch++)
	{
		if(!PVTTrkchInfo[trkch].bPRValid)
			continue;

		svid = PVTTrkchInfo[trkch].svid;

		if(SV_IsBd2(svid) || SV_IsGps(svid))
		{
			if(bGpsBdTValid)
				fixtime = gpsbd_fixtime;
			else
				continue;
		}
#if SUPPORT_GLONASS
		else if(SV_IsGlo(svid))
		{
			if(bGloTValid)
				fixtime = glo_fixtime;
			else
				continue;
		}
#endif
		else
			continue;

		svpos = PVTTrkchInfo[trkch].svpos;

		tsErr_by_time = ConvertFixTranTime(fixtime - PVTTrkchInfo[trkch].ts)*SPEED_OF_LIGHT;
		tsErr_by_dis = sqrt((svpos.x-hisPos.x)*(svpos.x-hisPos.x)+(svpos.y-hisPos.y)*(svpos.y-hisPos.y)+(svpos.z-hisPos.z)*(svpos.z-hisPos.z));

		PVTTrkchInfo[trkch].difts = fabs(tsErr_by_time-tsErr_by_dis);

		if(PVTTrkchInfo[trkch].difts > 5000.0)	//0.1ms
		{

//#ifdef _SIMULATE
//			if(PVTTrkchInfo[trkch].cn1s>40)
//#else
//		if((PVTTrkchInfo[trkch].cn1s>40) && (glStruAcqu.Info[trkch].Acqmode >= ACQ_MODE_0) && (glStruAcqu.Info[trkch].Acqmode <= ACQ_MODE_1))
//#endif
// 			{
//				bTimeValid = FALSE;
//				bHisPosValid = FALSE;
//			}
//#ifdef _SIMULATE
//			else if(PVTTrkchInfo[trkch].cn1s >= 31)
//#else
			if((PVTTrkchInfo[trkch].cn1s >= 31) 
				&& (PVTTrkchInfo[trkch].syncsrc==SF_SYNC_SOURCE_DATA)
				&& (glStruAcqu.Info[trkch].Acqmode >= ACQ_MODE_0) && (glStruAcqu.Info[trkch].Acqmode <= ACQ_MODE_2))
//#endif
			{
				weaktsdifsvcnt++;
			}

			SetBitWord256(&errsvmap, trkch);

		}

		avrtsdif += PVTTrkchInfo[trkch].difts;
		tsdifsvcnt++;
	}

	if((weaktsdifsvcnt*3>tsdifsvcnt) && (weaktsdifsvcnt>=2))
	{
		bTimeValid = FALSE;
		bHisPosValid = FALSE;
	}

	if(bTimeValid && bHisPosValid)
	{
		if(tsdifsvcnt > 0)
		{
			avrtsdif /= tsdifsvcnt;

			tsdif_th = 2.0*avrtsdif;
			tsdif_th = tsdif_th<1000.0?1000.0:tsdif_th;
		}

	
		for(trkch=0; trkch<MAXCHANNELS; trkch++)
		{
			if(GetBitWord256(&errsvmap ,trkch)==1)/* && (abs(PVTTrkchInfo[trkch].difts-3.0e5)>500)*/
			{
#ifndef _POSTPROC
#ifndef _SIMULATE
				LOOP_loselock(trkch,TRUE);
#endif
#endif
				PVTTrkchInfo[trkch].bPRValid = FALSE;
				PVTTrkchInfo[trkch].selpath = FIX_SV_PR_ERROR;

				lostByPRErrCnt++;
			}
			else if(PVTTrkchInfo[trkch].difts > tsdif_th)
			{
				PVTTrkchInfo[trkch].bPRValid = FALSE;
				PVTTrkchInfo[trkch].selpath = FIX_SV_PR_ERROR;
			}	
		}
	}
	else	//reset navsystime and hispos
	{
		RestSyncSynTime(NAV_SYS_NONE, GPSTIME_SYNC_SRC_NONE);	//reset all the sys sync time
#ifndef _POSTPROC
#ifndef _SIMULATE
		checkhispospath = 6;		//reset all the sys rtc time
#endif
#endif
		setRcvrHistoricalPos(NULL, NULL, HISTORICAL_POS_INVALID); 	//reset hispos
	}

	return;
}


void ResetRefTRKTs(void)
{
	int32 trkch = 0;
	PVT_TRKCH_INFO* pCurObsInfo = &(PVTTrkchInfo[trkch]);

	for(trkch=0; trkch<MAXCHANNELS; trkch++)
	{	
		pCurObsInfo = &(PVTTrkchInfo[trkch]);
		if((pCurObsInfo->bPRValid) && ((pCurObsInfo->CCBF&0x3) == 0x03) && (pCurObsInfo->syncsrc !=0) && (pCurObsInfo->synflag == SF_SYNC_SUCCESS))
		{
			pCurObsInfo->bPRValid = FALSE;
			pCurObsInfo->syncsrc = 0;	
			pCurObsInfo->ts = 0; 	
			pCurObsInfo->framenum = 0;
			pCurObsInfo->bitnum = 0;			
		}
	}
}


boolean CheckClkError2RTCRestoreTime(PVT_FIX_INFO* pFixInfo)
{
	boolean ret = TRUE;
	double gpsclkerr = 0.0, bdclkerr = 0.0, gloclkerr = 0.0;
	double clkerrthres = 0.3e-3;	//s

#if SUPPORT_GLONASS
	if((getCurFixGPSTime(NULL,NULL,NULL,NULL,&gpsclkerr) == -1) && (getCurFixBDTime(NULL,NULL,NULL,NULL,&bdclkerr) == -1) && (getCurFixGloTime(NULL,NULL,NULL,NULL,&gloclkerr) == -1))
#else
	if((getCurFixGPSTime(NULL,NULL,NULL,NULL,&gpsclkerr) == -1) && (getCurFixBDTime(NULL,NULL,NULL,NULL,&bdclkerr) == -1))
#endif
		return FALSE;

	if(pFixInfo->dop.pdop > pActiveCPT->SysmCptWorkConfig.PdopMask)
		return FALSE;

#if SUPPORT_GLONASS
	if((fabs(gpsclkerr) >=  clkerrthres) || (fabs(bdclkerr) >=  clkerrthres) || (fabs(gloclkerr) >=  clkerrthres))
#else
	if((fabs(gpsclkerr) >=  clkerrthres) || (fabs(bdclkerr) >=  clkerrthres))
#endif
	{
		RestSyncSynTime(NAV_SYS_NONE, GPSTIME_SYNC_SRC_NONE);	//reset all the sys sync time
#ifndef _POSTPROC
#ifndef _SIMULATE
		checkhispospath = 7;	
#endif	
#endif
		setRcvrHistoricalPos(NULL, NULL, HISTORICAL_POS_INVALID);	//reset hispos			
		ResetRefTRKTs();
		ret = FALSE;
	}

	return ret;
}


int32 CheckSyncErr(int8 orbitType, int32 trkch0, int32 trkch1)
{
	int32 trk_ch=0;
	int32 errcnt0=0, errcnt1=0;

	for(trk_ch = 0; trk_ch < MAXCHANNELS; trk_ch ++)
	{
		if((!PVTTrkchInfo[trk_ch].bPRValid) || (PVTTrkchInfo[trk_ch].synflag != SF_SYNC_SUCCESS)
			|| (trk_ch == trkch0) || (trk_ch == trkch1))
			continue;
		
		if(fabs(ConvertFixTranTime(PVTTrkchInfo[trk_ch].ts - PVTTrkchInfo[trkch0].ts))>(100.0e-3))
			errcnt0++;

		if(fabs(ConvertFixTranTime(PVTTrkchInfo[trk_ch].ts - PVTTrkchInfo[trkch1].ts))>(100.0e-3))
			errcnt1++;
	}

	if(errcnt0>errcnt1)
		return trkch0;
	else if(errcnt1>errcnt0)
		return trkch1;
	
	return -1;
}

static void CheckPRTsByOrbitType()
{
	MAX_MIN_TS_INFO maxminTsInfo={FALSE, -1, -1, -1.0, -1.0};
	MAX_MIN_TS_INFO maxminTsInfo_1={FALSE, -1, -1, -1.0, -1.0};
	int8 orbittype = SV_ORBIT_TYPE_UNKNOW;
	float64 deltats = 0.0;
	int32 trkch = 0;
	int16 validCount = 0;
	float64 trkts[MAXCHANNELS];	
	int32 errtrk=-1;

	for(trkch = 0; trkch < MAXCHANNELS; trkch ++)
		trkts[trkch] = PVTTrkchInfo[trkch].ts;
	
	for(orbittype = SV_ORBIT_TYPE_GPS;  orbittype <= SV_ORBIT_TYPE_BD_MEO; orbittype++)
	{		
		FindMaxMinSvTs(orbittype, &maxminTsInfo, trkts);
		if(!maxminTsInfo.valid)
			continue;
		
		validCount++;
		if(validCount==1)
			memcpy(&maxminTsInfo_1, &maxminTsInfo, sizeof(maxminTsInfo));
		else
		{
			if(maxminTsInfo.maxTs > maxminTsInfo_1.maxTs)
			{
				maxminTsInfo_1.maxTs = maxminTsInfo.maxTs;
				maxminTsInfo_1.maxTsTrkch = maxminTsInfo.maxTsTrkch;
			}

			if(maxminTsInfo.minTs < maxminTsInfo_1.minTs)
			{
				maxminTsInfo_1.minTs = maxminTsInfo.minTs;
				maxminTsInfo_1.minTsTrkch = maxminTsInfo.minTsTrkch;
			}
		}

		//check the max min ts in the same orbit type.
		deltats = ConvertFixTranTime(maxminTsInfo.maxTs- maxminTsInfo.minTs);
		if(deltats > 0.060)	//23 ms
		{
			errtrk = CheckSyncErr(orbittype, maxminTsInfo.maxTsTrkch, maxminTsInfo.minTsTrkch);

			if(errtrk == -1)
			{
				trkch = maxminTsInfo.maxTsTrkch;
				PVTTrkchInfo[trkch].bPRValid = FALSE;
				PVTTrkchInfo[trkch].selpath = FIX_SV_SYNC_ERROR;
#ifndef _POSTPROC
#ifndef _SIMULATE
				LOOP_loselock(trkch,TRUE);
#endif
#endif
				lostBySyncChk++;

				trkch = maxminTsInfo.minTsTrkch;
				PVTTrkchInfo[trkch].bPRValid = FALSE;
				PVTTrkchInfo[trkch].selpath = FIX_SV_SYNC_ERROR;
#ifndef _POSTPROC
#ifndef _SIMULATE
				LOOP_loselock(trkch,TRUE);
#endif
#endif
				lostBySyncChk++;
			}
			else
			{
				PVTTrkchInfo[errtrk].bPRValid = FALSE;
				PVTTrkchInfo[errtrk].selpath = FIX_SV_SYNC_ERROR;
#ifndef _POSTPROC
#ifndef _SIMULATE
				LOOP_loselock(errtrk,TRUE);
#endif
#endif
				lostBySyncChk++;
			}
		}
	}
	
	//check the max min ts between the different orbit type.
	if((validCount >1) && PVTTrkchInfo[maxminTsInfo.maxTsTrkch].bPRValid && PVTTrkchInfo[maxminTsInfo.minTsTrkch].bPRValid) 
	{
		deltats = ConvertFixTranTime(maxminTsInfo_1.maxTs - maxminTsInfo_1.minTs);
		if(deltats>0.1)		//100ms
		{
			errtrk = CheckSyncErr(orbittype, maxminTsInfo.maxTsTrkch, maxminTsInfo.minTsTrkch);
			if(errtrk == -1)
			{
				trkch = maxminTsInfo_1.maxTsTrkch;
				PVTTrkchInfo[trkch].bPRValid = FALSE;
				PVTTrkchInfo[trkch].selpath = FIX_SV_SYNC_ERROR;
#ifndef _POSTPROC
#ifndef _SIMULATE
				LOOP_loselock(trkch,TRUE);
#endif
#endif
				lostBySyncChk++;

				trkch = maxminTsInfo_1.minTsTrkch;
				PVTTrkchInfo[trkch].bPRValid = FALSE;
				PVTTrkchInfo[trkch].selpath = FIX_SV_SYNC_ERROR;
#ifndef _POSTPROC
#ifndef _SIMULATE
				LOOP_loselock(trkch,TRUE);
#endif
#endif
				lostBySyncChk++;
			}
			else
			{
				PVTTrkchInfo[errtrk].bPRValid = FALSE;
				PVTTrkchInfo[errtrk].selpath = FIX_SV_SYNC_ERROR;
#ifndef _POSTPROC
#ifndef _SIMULATE
				LOOP_loselock(errtrk,TRUE);
#endif
#endif
				lostBySyncChk++;
			}
		}
	}
	
	return;
}

static void FindMaxMinSvTs(byte orbitType, MAX_MIN_TS_INFO* pOrbitTsInfo, float64 trkts[MAXCHANNELS])
{
	int32 svid=0, trk_ch = 0;
	float64 ts = 0.0;
	float64 deltatime = 0.0;
	
	pOrbitTsInfo->valid = FALSE;
	pOrbitTsInfo->minTs = pOrbitTsInfo->maxTs = -1.0;
	pOrbitTsInfo->minTsTrkch = pOrbitTsInfo->maxTsTrkch = -1;

	for(trk_ch = 0; trk_ch < MAXCHANNELS; trk_ch ++)
	{
		if((!PVTTrkchInfo[trk_ch].bPRValid) || (PVTTrkchInfo[trk_ch].synflag != SF_SYNC_SUCCESS))
			continue;
		

		svid = PVTTrkchInfo[trk_ch].svid;
		
		if(FindSVOrbitType(svid) != orbitType)
			continue;
		
		ts = trkts[trk_ch];		
		if(pOrbitTsInfo->valid)
		{
			deltatime = ts - pOrbitTsInfo->maxTs;				
			if(((deltatime > 0) && (deltatime < SECONDS_IN_HALF_WEEK)) || 
				((deltatime < 0) && (deltatime < (-1) * SECONDS_IN_HALF_WEEK)))
			{
				pOrbitTsInfo->maxTs = ts;
				pOrbitTsInfo->maxTsTrkch = trk_ch;
			}

			deltatime = ts - pOrbitTsInfo->minTs;
			if(((deltatime > 0) && (deltatime > SECONDS_IN_HALF_WEEK)) || 
				((deltatime < 0) && (deltatime > (-1) * SECONDS_IN_HALF_WEEK)))
			{
				pOrbitTsInfo->minTs = ts;
				pOrbitTsInfo->minTsTrkch = trk_ch;
			}
		}
		else
		{
			//get initial value 
			pOrbitTsInfo->valid = TRUE;
			pOrbitTsInfo->maxTs = pOrbitTsInfo->minTs = ts;
			pOrbitTsInfo->maxTsTrkch = pOrbitTsInfo->minTsTrkch = trk_ch;
		}
	}
	
	return;
}


static void CheckPRTsByFrepoint()
{
	int32 itrkch=0,jtrkch=0;
	int32 cursvid=0,othersvid=0;
	word32 curfre=0,otherfre=0;
	double curts = 0.0, otherts = 0.0;
	double TH_ts = 3.33e-6;  //1000m
	
	for(itrkch=0; itrkch<MAXCHANNELS-1; itrkch++)
	{
		if(!PVTTrkchInfo[itrkch].bPRValid)
			continue;
		
		cursvid = PVTTrkchInfo[itrkch].svid;
		curfre = PVTTrkchInfo[itrkch].freq_point;
		for(jtrkch=itrkch+1;jtrkch<MAXCHANNELS;jtrkch++)
		{
			othersvid = PVTTrkchInfo[jtrkch].svid;
			otherfre = PVTTrkchInfo[jtrkch].freq_point;
			if((cursvid!=othersvid) || (curfre==otherfre) || (!PVTTrkchInfo[jtrkch].bPRValid)) //same svid and diffrent frepoint
				continue;

			curts = PVTTrkchInfo[itrkch].ts;
			otherts = PVTTrkchInfo[jtrkch].ts;
			if(f_abs(curts-otherts)>TH_ts)
			{
				if(SF_SYNC_SOURCE_DATA!=PVTTrkchInfo[itrkch].syncsrc)
				{
					PVTTrkchInfo[itrkch].bPRValid = FALSE;
					PVTTrkchInfo[itrkch].selpath = FIX_SV_TSDIF_FRE;
					#ifndef _POSTPROC
					#ifndef _SIMULATE
					LOOP_loselock(itrkch,TRUE);
					#endif
					#endif
					lostBySyncChk++;
				}

				if(SF_SYNC_SOURCE_DATA!=PVTTrkchInfo[jtrkch].syncsrc)
				{
					PVTTrkchInfo[jtrkch].bPRValid = FALSE;
					PVTTrkchInfo[jtrkch].selpath = FIX_SV_TSDIF_FRE;
					#ifndef _POSTPROC
					#ifndef _SIMULATE
					LOOP_loselock(jtrkch,TRUE);
					#endif
					#endif
					lostBySyncChk++;
				}
			}	
		}
	}
}


boolean CheckSVTransmitTime(int32 svid, float64 transTime)
{
	float64 transTime_min, transTime_max;
	int32 orbitType = SV_ORBIT_TYPE_UNKNOW;

	orbitType = FindSVOrbitType(svid);

	if(orbitType == SV_ORBIT_TYPE_GPS)
	{
		transTime_min = 0.065;
		transTime_max = 0.088;
	}
	else if(orbitType == SV_ORBIT_TYPE_BD_MEO)
	{
		transTime_min = 0.069;
		transTime_max = 0.093;
	}
	else if((orbitType == SV_ORBIT_TYPE_BD_IGSO) || (orbitType == SV_ORBIT_TYPE_GEO))
	{
		transTime_min = 0.117;
		transTime_max = 0.141;
	}
	else
		return TRUE;
	
	if((transTime < transTime_max) &&  (transTime > transTime_min))
		return TRUE;
	else
		return FALSE;	
}


void CheckSysEnv(int8* pEnvState)
{
	int32 AvailNum = 0;
	byte gpsSvGt40= 0, gpsSvGt35 = 0, gpsSvGt30 = 0, gpsSvLt30 = 0;
	byte bdSvGt40= 0, bdSvGt35 = 0, bdSvGt30 = 0, bdSvLt30 = 0;
	byte maxcn0_gps=0, mincn0_gps=100, maxcn0_bdmeo=0, mincn0_bdmeo=100, maxcn0_bdgeo=0, mincn0_bdgeo=100;
	byte gpscnt=0, bdmeocnt=0, bdgeocnt=0;
	boolean bgpsSigSta= FALSE,bBDSigSta= FALSE;
	int32 i = -1, ephAgeUseTh=0;
	int32 svid = -1, trkch=0;
	byte cn0;

	for(trkch=0; trkch<MAXCHANNELS; trkch++)
	{
		svid = PVTTrkchInfo[trkch].svid;
		//check this sv is in use nav system or not.
		//

		if((PVTTrkchInfo[trkch].CCBF & 0x3) != 0x3)
			continue;

		AvailNum++;

		cn0 = PVTTrkchInfo[trkch].cn1s;

		if(SV_IsGps(svid))
		{
			if(cn0>maxcn0_gps)
				maxcn0_gps = cn0;

			if(cn0<mincn0_gps)
				mincn0_gps = cn0;

			gpscnt++;
		}
		else if(SV_IsBd2Meo(svid))
		{
			if(cn0>maxcn0_bdmeo)
				maxcn0_bdmeo = cn0;

			if(cn0<mincn0_bdmeo)
				mincn0_bdmeo = cn0;

			bdmeocnt++;
		}
		else if(SV_IsBd2Geo(svid))
		{
			if(cn0>maxcn0_bdgeo)
				maxcn0_bdgeo = cn0;

			if(cn0<mincn0_bdgeo)
				mincn0_bdmeo = cn0;

			bdgeocnt++;
		}
		
		if(SV_IsBd2Geo(svid))
			cn0 -=6; 

		if(cn0 >= 40)
		{
			if(SV_IsGps(svid))
				gpsSvGt40 ++;
			else
				bdSvGt40 ++;
		}
		
		if (cn0 >= 35) 
		{
			if(SV_IsGps(svid))
				gpsSvGt35 ++;
			else
				bdSvGt35 ++;
		}
		
		if(cn0 >= 30)
		{
			if(SV_IsGps(svid))
				gpsSvGt30 ++;
			else
				bdSvGt30 ++;
		}
		else
		{
			if(SV_IsGps(svid))
				gpsSvLt30 ++;
			else
				bdSvLt30 ++;
		}	
	}

	if((gpsSvGt40 > 5) || (bdSvGt40 > 5) || (bdSvGt40 + gpsSvGt40 > 7))
	{
		SysEnvSta = SYS_FULL_OPEN_SKY;
	}
	else if ((gpsSvGt35 > 5) || (bdSvGt35 > 5) || (bdSvGt35 + gpsSvGt35 > 7))
	{
		SysEnvSta = SYS_OPEN_SKY;
	}
	else if((gpsSvGt30 > 5) || (bdSvGt30 > 5) || (bdSvGt30 + gpsSvGt30 > 7))
	{
		SysEnvSta = SYS_HALF_OPEN_SKY;
	}
	else if((gpsSvLt30 + bdSvLt30) >= 3 * AvailNum / 4)
	{
		SysEnvSta = SYS_FULL_CLOSE_ENV;
	}
	else
	{
		SysEnvSta = SYS_DOWN_TOWN;
	}

	if(SysEnvCon == SYS_CONDITION_NORMAL)
	{
		if((gpscnt>=5) && (maxcn0_gps-mincn0_gps <2) && (mincn0_gps>30))
			bgpsSigSta = TRUE;

		if(((bdmeocnt>=5) && (maxcn0_bdmeo-mincn0_bdmeo<2) && (mincn0_bdmeo>30)) 
			|| ((bdmeocnt + bdgeocnt>=5) && (maxcn0_bdmeo-mincn0_bdmeo<2) && (maxcn0_bdgeo-mincn0_bdgeo<2) && (mincn0_bdmeo>30) && (mincn0_bdgeo>30)))
			bBDSigSta = TRUE;

		if((bgpsSigSta && bBDSigSta)
			||(bgpsSigSta && (bdmeocnt + bdgeocnt==0))
			||(bBDSigSta && (gpscnt==0)))
			SysEnvCon = SYS_CONDITION_SIGNAL;
	}
	
// check mycoasting
#ifndef _POSTPROC
#ifndef _SIMULATE
	if((FixTime > 0) && (getRcvrInfo(NULL,NULL,NULL,NULL) == FIX_NOT) && (SysEnvSta >= SYS_DOWN_TOWN) && (mycoasting == 0))
	{
		if(pActiveCPT->SysmCptWorkConfig.FixUpdateCycle !=0)
			mycoasting = 6000/pActiveCPT->SysmCptWorkConfig.FixUpdateCycle;//30s
		else
			mycoasting = 6000;
		FixTime = 0;
		InitCostRFcnt2 = getGlobalCnt();
		for(i=0; i < MAXCHANNELS ; i++ )
		{
			if(glStruAcqu.Info[i].State == IDLE_PVT)  // clear 
				memset(&glStruAcqu.Info[i],0,sizeof(STRU_ACQU_CHANNEL_INFO));
		}
		for(i=1; i <= SV_NUM; i++)
		{
			if((SVInfo[i-1].el < SV_ELEVATION_VALID_TH) && (SVInfo[i-1].el > GetSVElMask(i))) //if have eph ,clear total failcnt
			{
#if SUPPORT_GLONASS
				if(SV_IsGlo(i))
					ephAgeUseTh = GLO_SV_EPH_AGE_USABLE_THRES;
				else
#endif
					ephAgeUseTh = SV_EPH_AGE_USABLE_THRES;
				if(SVInfo[i-1].eph_age < ephAgeUseTh)
					ClearSVACQFailCnt(i,0xFFFF);
			}
		}
	}
#endif
#endif
	return;
}


void UpdateSVMultiChInfo(SV_MULTI_INFO* pMultiSVInfo, int32 trkch, int8 cn0, word16 frqdot, int8 synsrc)
{
	int32 idx = pMultiSVInfo->multiCnt;
	int32 fredotidx=0;
	
	pMultiSVInfo->trkch[idx]=trkch;
	pMultiSVInfo->cn0[idx]=cn0;
	
	if(synsrc == 0)
		pMultiSVInfo->normalSfSyncMap &= (0x1<<idx);

	fredotidx = FindBitWord16(frqdot,1);
	pMultiSVInfo->frqdotIdxMap[fredotidx] |= (0x1<<idx);

	if(pMultiSVInfo->maxcn0 < cn0)
	{
		pMultiSVInfo->maxcn0 = cn0;
		pMultiSVInfo->maxcn0idx = idx;
	}

	pMultiSVInfo->multiCnt++;

	return;
}

void SelectSVMultiChInfo(SV_MULTI_INFO* pMultiSVInfo, word256* pcoarsTrkMap, int32* pAvailchcnt)
{

	int8 validIdx=-1;
	word32 idxmap=0, initalmap=0;

	int32 svid=0, idx=0, fredotidx=0;
	SV_MULTI_INFO* pCurMInfo;

	for(svid=1; svid<=SV_NUM; svid++)
	{
		pCurMInfo = &(pMultiSVInfo[svid-1]);
		
		if(pCurMInfo->multiCnt <= 1)
			continue;

		validIdx = -1;
		idxmap=0; 

		//step1: select sf sync src
		if(pCurMInfo->normalSfSyncMap !=0)
			initalmap = pCurMInfo->normalSfSyncMap;
		else
			initalmap = 0xffffffff;

		//step2: select freqdot	
#if SUPPORT_GLONASS
		if(SV_IsGps(svid) || SV_IsGlo(svid))
#else
		if(SV_IsGps(svid))
#endif
		{
			for(fredotidx=0; fredotidx<FREQ_CNT; fredotidx++)
			{
				if(pCurMInfo->frqdotIdxMap[fredotidx]== 0)
//				if((pCurMInfo->frqdotIdxMap[fredotidx]== 0) || (fredotidx == 3))
					continue;

				idxmap =initalmap & pCurMInfo->frqdotIdxMap[fredotidx];
				
				if(idxmap!=0)
				{
					validIdx = FindBitWord16(idxmap,1);
					if((pCurMInfo->maxcn0 - pCurMInfo->cn0[validIdx])<=5)
						break;
				}
			}	
		}
		else if(SV_IsBd2(svid))
		{
			for(fredotidx=FREQ_CNT-1; fredotidx>=0; fredotidx--)
			{
				if(pCurMInfo->frqdotIdxMap[fredotidx]== 0)
					continue;

				idxmap =initalmap & pCurMInfo->frqdotIdxMap[fredotidx];
				
				if(idxmap!=0)
				{
					validIdx = FindBitWord16(idxmap,1);
					if((pCurMInfo->maxcn0 - pCurMInfo->cn0[validIdx])<=4)
						break;
				}
			}	
		}

		if(validIdx<0)
			validIdx = pCurMInfo->maxcn0idx;

		//delete the not use channel.
		for(idx=0; idx<pCurMInfo->multiCnt; idx++)
		{
			if(idx ==validIdx)
				continue;

			ClearBitWord256(pcoarsTrkMap, pCurMInfo->trkch[idx]);
			PVTTrkchInfo[pCurMInfo->trkch[idx]].selpath = FIX_SV_DUPLITE;
			(*pAvailchcnt)--;
		}
	}

	return;
}


void CoarseSelect(word256* pTrkChMap)
{
	static SV_MULTI_INFO multiSVInfo[SV_NUM];
	int32 prn = 0;
	UINT32 frqPoint=0;
	int32 trk_ch = 0;
	int32 AvailNum = 0;
	int32 i = 0;
	float64 difDopTh=40.0; //Hz
	int32 svcnt=0;
	float64 tcxo=0.0;
	bool bTCXOvalid = FALSE;
	float64 avrDifFre=0.0;
	double pllfactor = 0.0;

	memset(multiSVInfo, 0, sizeof(multiSVInfo));

	bTCXOvalid = GetTCXOOffset(&tcxo);

	//satellite selection for fix
	for (trk_ch = 0; trk_ch < MAXCHANNELS; trk_ch++)
	{		
		prn = PVTTrkchInfo[trk_ch].svid;
		frqPoint = PVTTrkchInfo[trk_ch].freq_point;

		if((!PVTTrkchInfo[trk_ch].bPRValid) || (PVTTrkchInfo[trk_ch].synflag!=SF_SYNC_SUCCESS))
		{
			PVTTrkchInfo[trk_ch].selpath = FIX_SV_NO_PSEUDORANGE;
			continue;
		}
		

		if((frqPoint & pActiveCPT->SysmCptWorkConfig.NavFreqPoint) == 0)
		{
			PVTTrkchInfo[trk_ch].selpath = FIX_SV_NAV_SYSTEM;
			continue;
		}

		if(GetBitWord256(&(pActiveCPT->SysmCptWorkConfig.SVBitMap),prn-1)==0)
		{
			PVTTrkchInfo[trk_ch].selpath = FIX_SV_USER_PARAM;
			continue;
		}

		//step1: check signal strength
		if((PVTTrkchInfo[trk_ch].cn1s < pActiveCPT->SysmCptWorkConfig.SnrMaskSingle) || (SV_IsBd2Geo(prn) && (PVTTrkchInfo[trk_ch].cn1s < pActiveCPT->SysmCptWorkConfig.SnrMaskSingle+10)))
		{
			PVTTrkchInfo[trk_ch].selpath = FIX_SV_MINCN0;
			continue;
		}
#ifndef _POSTPROC
#ifndef _SIMULATE
		//step3: check SV URA
		if((SV_IsGps(prn)||SV_IsBd2(prn))&& (Sys3AlmEphUTCInfo.GpsBd2_EphStruct[prn-1].ura >= 10))
		{
			PVTTrkchInfo[trk_ch].selpath = FIX_SV_URA;
			continue;
		}
#endif
#endif
		//step 4: check ephemeris valid and elevation		
		if(SVInfo[prn-1].el < GetSVElMask(prn))
		{	
			PVTTrkchInfo[trk_ch].selpath = FIX_SV_ELEVATION;
			continue;
		}

		if(SV_IsBd2Geo(prn) && bTCXOvalid)
		{
			pllfactor = GetFreqpointPLL(PVTTrkchInfo[trk_ch].freq_point);
			if(fabs(PVTTrkchInfo[trk_ch].fd - tcxo*pllfactor)>150.0)
			{
				PVTTrkchInfo[trk_ch].selpath = FIX_SV_GEO_TCXODIF;
				continue;
			}
		}

#if SUPPORT_GLONASS
		if(SV_IsGlo(prn) && (f_abs(PVTTrkchInfo[trk_ch].fd)>5000.0))
		{	
			PVTTrkchInfo[trk_ch].selpath = FIX_SV_DOPDIF;
			continue;
		}
#endif

		//collect the SV in multi ch info		
		UpdateSVMultiChInfo(&(multiSVInfo[prn-1]), trk_ch,PVTTrkchInfo[trk_ch].cn1s, frqPoint,PVTTrkchInfo[trk_ch].syncsrc);

		SetBitWord256(pTrkChMap,trk_ch);
		AvailNum++;
	}

	//select the SV in multi trkch
	SelectSVMultiChInfo(multiSVInfo, pTrkChMap,&AvailNum);

	setSVCandidateInfo();

	//delete by doppler
	if((AvailNum > 6) 
		&& (getRcvrHistoricalPos(NULL,NULL) <= HISTORICAL_POS_LEVEL_40KM) 
		&& GetCurNavSysTime(NAV_SYS_NONE,GPSTIME_SYNC_SRC_RTCPULSE_ERR05MS,NULL,NULL)&& GetTCXOOffset(NULL))
	{
		sortSVCandidateInfo( SV_CANDIDATE_SORT_TYPE_ASC,  SORT_SV_ITEM_DELTA_FREQ);

		if(getRcvrInfo(NULL,NULL,NULL,NULL)==FIX_NOT)
			difDopTh = 80.0;
		
		svcnt=0;
		for(i=0; i<CandidateSVListLen; i++)
		{
			trk_ch = CandidateSVList[i].trkch;
			if(GetBitWord256(pTrkChMap,trk_ch))
			{
				svcnt++;
#if 1
				if(svcnt<=6)
					avrDifFre += CandidateSVList[i].deltafreq;

				if(svcnt==6)
				{
					avrDifFre = avrDifFre/svcnt;
					difDopTh = avrDifFre*2.0;
					if(difDopTh < 20.0)
						difDopTh = 20.0;
					else if(difDopTh > 100.0)
						difDopTh = 100.0;
				}					
#else
				if((svcnt==6) && (CandidateSVList[i].deltafreq > difDopTh))
					difDopTh = CandidateSVList[i].deltafreq;
#endif

				if((svcnt>6) && (CandidateSVList[i].deltafreq > difDopTh))
				{
					AvailNum--;
					ClearBitWord256(pTrkChMap,trk_ch); //Clear AvailSV 				
					PVTTrkchInfo[trk_ch].selpath = FIX_SV_DOPDIF;
				}		
			}
		}
	}

	// delete by prtsdif
	if(AvailNum > 4)
	{
		sortSVCandidateInfo(SV_CANDIDATE_SORT_TYPE_DESC,  SORT_SV_ITEM_PRTSDIF);

		for(i=0; i<CandidateSVListLen; i++)
		{
			if((AvailNum <= 4) || (CandidateSVList[i].prtsdif < 600))
				break;
			
			trk_ch = CandidateSVList[i].trkch;
			if(GetBitWord256(pTrkChMap,trk_ch))
			{	
				AvailNum--;
				ClearBitWord256(pTrkChMap,trk_ch); //Clear AvailSV 							
				PVTTrkchInfo[trk_ch].selpath = FIX_SV_RESIDUE_PRTSDIF;
			}
		}
	}

	// delete more candidate sv for max selection
	if(AvailNum > MAX_USE_SV_NUM)
	{
		sortSVCandidateInfo( SV_CANDIDATE_SORT_TYPE_ASC,  SORT_SV_ITEM_CN0);

		for(i=0; i<CandidateSVListLen; i++)
		{
			if(AvailNum <= MAX_USE_SV_NUM)
				break;
			
			trk_ch = CandidateSVList[i].trkch;
			if(GetBitWord256(pTrkChMap,trk_ch))
			{	
				AvailNum--;
				ClearBitWord256(pTrkChMap,trk_ch); //Clear AvailSV 							
				PVTTrkchInfo[trk_ch].selpath = FIX_SV_EXCESS;
			}
		}
	}

	return;	
}


int8 DifferenceInfoModify(word256* pTrkChMap)
{
	int32 trkch = 0;

	for(trkch = 0; trkch < MAXCHANNELS; trkch++)
	{
		if((!PVTTrkchInfo[trkch].bPRValid) || (GetBitWord256(pTrkChMap,trkch)==0))
			continue;


		//step1, Iono
		#ifndef _POSTPROC
		#ifndef _SIMULATE
		PVTTrkchInfo[trkch].ionosrc = GetSvIonoInfo(trkch, PVTTrkchInfo[trkch].ts, &(PVTTrkchInfo[trkch].iono));
		#endif
		#endif
		PVTTrkchInfo[trkch].pr -= PVTTrkchInfo[trkch].iono;

		//step2, Tropo
		#ifndef _POSTPROC
		#ifndef _SIMULATE
		GetSvTropoInfo(trkch, &(PVTTrkchInfo[trkch].tropo));
		#endif
		#endif
		PVTTrkchInfo[trkch].pr -= PVTTrkchInfo[trkch].tropo;
	}

	return 0;	
}

void CalcPseudorange()
{
	boolean coarse_rcvrpos_v=FALSE;
	boolean accuracy_arvtime_v=FALSE;
	int32 trkch = 0;
	double _time_rx = 0.0;
	double fixPRIntCnt = TICLock_RFcnt;
	
#if (PVT_MODE == CALC_PVT_KF)
	//if(getTowFrmBaseTr(fixPRIntCnt, &_time_rx, NULL))
	if(IsCurPVTKFValid() && getTowFrmBaseTr(fixPRIntCnt, &_time_rx, NULL))
		accuracy_arvtime_v=FALSE;
	else
#endif
	{
		if(GetNavSysTimeByRFCnt(NAV_SYS_NONE,GPSTIME_SYNC_SRC_REFBIT_RFCNT, fixPRIntCnt, &_time_rx, NULL))
			accuracy_arvtime_v = TRUE;
		else if(CalculateTrByMaxMinTs(&_time_rx))			//No GPSTime or GPSTime error
			accuracy_arvtime_v = FALSE;
		else
			return;
	}

	setBaseTr(fixPRIntCnt, _time_rx);

	//calculate each pseudorange
	for(trkch = 0; trkch < MAXCHANNELS; trkch++)
	{
		if(GetBitWord256(&CoarseSelchmap,trkch))
			PVTTrkchInfo[trkch].pr = ConvertFixTranTime(_time_rx - PVTTrkchInfo[trkch].ts) * SPEED_OF_LIGHT;
		else
			PVTTrkchInfo[trkch].pr = 0.0;
	}
	
	//get receiver historical position, use this as the iterate OP initial value
	if(getRcvrHistoricalPos(NULL,NULL) <= HISTORICAL_POS_LEVEL_40KM)
		coarse_rcvrpos_v = TRUE;

	//calculate iteration threshold
	if(coarse_rcvrpos_v == TRUE && accuracy_arvtime_v == TRUE)
		setLSPosEquItrCnt(7);
	else if(coarse_rcvrpos_v == TRUE && accuracy_arvtime_v == FALSE)
		setLSPosEquItrCnt(10);
	else
		setLSPosEquItrCnt(20);

	return;
}


boolean getTowFrmBaseTr(double curRFCnt, double* pCurTime, double* pDifTime)
{
	boolean ret = FALSE;
	double diftime;
	
	if(base_PRcnt > MICRO_NUM)
	{
		ret = TRUE;

		diftime =  (curRFCnt - base_PRcnt) / RF_SAMP_FREQ;
		
		if(pCurTime != NULL)
		{
			*pCurTime = base_time_rx + diftime;

			if((*pCurTime) > SECONDS_IN_WEEK)
				(*pCurTime) -= SECONDS_IN_WEEK;
			else if((*pCurTime) < MICRO_NUM)
				(*pCurTime) += SECONDS_IN_WEEK;
		}

		if(pDifTime != NULL)
			*pDifTime = diftime;
	}

	return ret;
}

void setBaseTr(double RFCnt, double tow)
{
	base_PRcnt = RFCnt;
	base_time_rx = tow;
}

boolean CalculateTrByMaxMinTs(double* pTr)
{
	byte trkch = 0, svid=0;
	boolean bTrInit = FALSE;
	double temptr = 0.0, deltatime = 0.0;
	byte orbittype;
	double maxtr = -1.0, mintr = -1.0;
	
	for(trkch = 0; trkch < MAXCHANNELS; trkch ++)
	{
		if(!GetBitWord256(&CoarseSelchmap,trkch))
			continue;
		
		svid = PVTTrkchInfo[trkch].svid;

		orbittype = FindSVOrbitType(svid);
		temptr = PVTTrkchInfo[trkch].ts;
		if(orbittype == SV_ORBIT_TYPE_GPS)
			temptr += (76.0e-3);
		else if(orbittype == SV_ORBIT_TYPE_BD_MEO)
			temptr += (81.0e-3);
		else if((orbittype == SV_ORBIT_TYPE_BD_IGSO) || (orbittype == SV_ORBIT_TYPE_GEO))
			temptr += (129.0e-3);
		else if(orbittype == SV_ORBIT_TYPE_GLO)
			temptr += (70.0e-3);
		else
			continue;

		if(!bTrInit)
		{
			bTrInit = TRUE;
			maxtr = temptr;
			mintr = temptr;
		}
		else
		{
			deltatime = temptr -maxtr;				
			if(((deltatime > 0) && (deltatime < SECONDS_IN_HALF_WEEK)) || 
				((deltatime < 0) && (deltatime < (-1) * SECONDS_IN_HALF_WEEK)))
			{
				maxtr = temptr;
			}

			deltatime = temptr -mintr;
			if(((deltatime > 0) && (deltatime > SECONDS_IN_HALF_WEEK)) || 
				((deltatime < 0) && (deltatime > (-1) * SECONDS_IN_HALF_WEEK)))
			{
				mintr = temptr;
			}
		}		
	}

	if(!bTrInit)
		return FALSE;

	if(f_abs(maxtr - mintr) < MICRO_NUM)
		*pTr = maxtr;
	else if(maxtr > mintr)
		*pTr = (maxtr + mintr) *0.5;
	else
		*pTr = (maxtr + mintr + SECONDS_IN_WEEK) *0.5;

	return TRUE;
}


double ConvertFixTranTime(double time)
{
	double tow = 0.0;

	if(time > SECONDS_IN_HALF_WEEK)
		tow = time - SECONDS_IN_WEEK;
	else if(time < (-1) * SECONDS_IN_HALF_WEEK)
		tow = time + SECONDS_IN_WEEK;
	else
		tow = time;
	
	return tow;	
}


int8 KFPath=0;
static void StopStrategy(void)
{
#if (PVT_MODE == CALC_PVT_KF)
	static int32 conKFPVTStopCnt=0, conKFPVTMoveCnt=0;
	static boolean bKFPVTMoved = FALSE;
	static int32 ConFixcnt = 0;
	
	ECEF lastpos = {0,};
	float64 distance=0.0;
	int32 noiseIdxTh = 2;
	
	if(SysEnvSta >= SYS_OPEN_SKY)
		noiseIdxTh = 3;

	if(PVTFixInfo.posres != FIX_NOT)
		ConFixcnt++;
	else
		ConFixcnt=0;
	
	if(ConFixcnt<=5)
		return;

	if(PVTFixInfo.filtertype == FILTER_TYPE_EXTEND)
	{
		KFPath = 1;
		
		if(KalmanNoiseIndex < noiseIdxTh)
		{
			KFPath = 2;
			conKFPVTStopCnt ++;
			conKFPVTMoveCnt = 0;
			
			if(getRcvrInfo(&lastpos, NULL, NULL, NULL) != FIX_NOT)
			{
				KFPath = 3;
				distance = Dis2PointsOnEarth(&(PVTFixInfo.rcvrpos), &lastpos);

				if(bKFPVTMoved && (distance < 10))
				{
					KFPath = 4;
					PVTFixInfo.rcvrpos.x = lastpos.x;
					PVTFixInfo.rcvrpos.y = lastpos.y;
					PVTFixInfo.rcvrpos.z = lastpos.z;

					//if(distance > 10)
					if(conKFPVTStopCnt % 5 == 0)
					{
						KFPath = 5;
						InitPVTKF(&PVTFixInfo, FALSE);
					}
				}
#if 1
				else if((!bKFPVTMoved) && (conKFPVTStopCnt > 100) && (distance < 10))		//for cep test
				{
					KFPath = 6;
					PVTFixInfo.rcvrpos.x = (lastpos.x * 3 + PVTFixInfo.rcvrpos.x) * 0.25;
					PVTFixInfo.rcvrpos.y = (lastpos.y * 3 + PVTFixInfo.rcvrpos.y) * 0.25;
					PVTFixInfo.rcvrpos.z = (lastpos.z * 3 + PVTFixInfo.rcvrpos.z) * 0.25;

					//if(distance > 10)
					if(conKFPVTStopCnt % 20 == 0)
					{						
						KFPath = 7;
						InitPVTKF(&PVTFixInfo, FALSE);
					}
				}
#endif			
				else
				{
					KFPath = 8;
					PVTFixInfo.rcvrpos.x = (lastpos.x + PVTFixInfo.rcvrpos.x) * 0.5;
					PVTFixInfo.rcvrpos.y = (lastpos.y + PVTFixInfo.rcvrpos.y) * 0.5;
					PVTFixInfo.rcvrpos.z = (lastpos.z + PVTFixInfo.rcvrpos.z) * 0.5;
				}

				if(SysEnvSta >= SYS_OPEN_SKY)
				{
					PVTFixInfo.rcvrvel.x = 0.000001;
					PVTFixInfo.rcvrvel.y = 0.000001;
					PVTFixInfo.rcvrvel.z = 0.000001;
				}
			}
		}
		else
		{
			KFPath = 9;
			conKFPVTStopCnt = 0;
			
			if(KalmanNoiseIndex > noiseIdxTh)
			{
				KFPath = 10;
				conKFPVTMoveCnt ++;
				
				if(conKFPVTMoveCnt > 10)
				{
					KFPath = 11;
					bKFPVTMoved = TRUE;
				}
			}
			else
			{
				KFPath = 12;
				conKFPVTMoveCnt = 0;
			}
		}
	}
	else
	{
		KFPath = 13;
		conKFPVTMoveCnt = 0;
		conKFPVTStopCnt = 0;
	}
#endif
}

static void CalcPVTByKalmanFilter(void)
{
#if (PVT_MODE==CALC_PVT_KF)
	//ECEF rtkpos;
	//if(getRcvrInfo(&rtkpos,NULL,NULL,NULL)>=FIX_RTD)
	//	InitPVTKFPos(rtkpos);

	KalmanFilterPVT(FixCycle, PVTTrkchInfo, &PVTFixInfo);	

	if( (pActiveCPT->SysmCptWorkConfig.FunSwitch & FUNSWITCH_STATICNAV ) != 0)
		StopStrategy();
	

	//PVTKFSaticProc(&KFPVTInfo);
#endif
	return;
}

static void setSVCandidateInfo(void)
{
	int32 i = 0;
	int32 trk_ch = 0;
	int32 svid = 0, frepoint=0;
	float64 swfreq = 0.0;

	memset((void*)CandidateSVList, 0, sizeof(CandidateSVList));

	for(trk_ch = 0; trk_ch < MAXCHANNELS; trk_ch++)
	{
		svid = PVTTrkchInfo[trk_ch].svid;
		frepoint = PVTTrkchInfo[trk_ch].freq_point;

		if(GetBitWord256(&CoarseSelchmap,trk_ch))
		{
			CandidateSVList[i].prn = svid;
			CandidateSVList[i].trkch = trk_ch;
#ifndef _POSTPROC
#ifndef _SIMULATE
			if(GetSVSWFreq(svid, frepoint,&swfreq) == SWFREQ_SRC_EPH)
				CandidateSVList[i].deltafreq =(int16)(abs(swfreq - PVTTrkchInfo[trk_ch].fd));
#endif
#endif

#if SUPPORT_GLONASS
			if(SV_IsGps(svid) || SV_IsBd2(svid) || SV_IsGlo(svid))
#else
			if(SV_IsGps(svid) || SV_IsBd2(svid)) 
#endif
				CandidateSVList[i].ephage = SVInfo[svid-1].eph_age;

			if(SV_IsBd2Geo(svid))
				CandidateSVList[i].cn0 = (PVTTrkchInfo[trk_ch].cn1s>5)?(PVTTrkchInfo[trk_ch].cn1s-5):0;
#if SUPPORT_GLONASS
			else if(SV_IsGlo(svid))
				CandidateSVList[i].cn0 = (PVTTrkchInfo[trk_ch].cn1s>5)?(PVTTrkchInfo[trk_ch].cn1s-5):0;
#endif
			else
				CandidateSVList[i].cn0 = PVTTrkchInfo[trk_ch].cn1s;


			CandidateSVList[i].prtsdif = (int32)PVTTrkchInfo[trk_ch].difts;

			i++;
		}
	}

	CandidateSVListLen = i;

	return;
}

/*	
*	@brief	swap the items of candidated SV info
*	@para	index of tow item if CandidateSVList
*	@return	none
*/
void swapSVCandidate(int32 i, int32 j)
{
	SV_CODE_CARRIER_TRK_STATUS tmpSV;

	if(i != j) {
		tmpSV = CandidateSVList[i];
		CandidateSVList[i] = CandidateSVList[j];
		CandidateSVList[j] = tmpSV;
	}
}


void sortSVCandidateInfo(byte sort_type, byte field_name)
{
	int32 i, j;  
	double field_val = 0.0, tmp_val=0.0;
	int32 index;
	                                                                                    	                                
	for(i=0; i<CandidateSVListLen-1; i++)                                                         	                                
	{           
		if(field_name == SORT_SV_ITEM_CN0)
			field_val = (double)CandidateSVList[i].cn0;
		else if(field_name == SORT_SV_ITEM_EPH_AGE)
			field_val = (double)CandidateSVList[i].ephage;
		else if(field_name == SORT_SV_ITEM_LOCK_TIME)
			field_val = (double)CandidateSVList[i].trklocktime;
		else if(field_name == SORT_SV_ITEM_DELTA_FREQ)
			field_val = (double)CandidateSVList[i].deltafreq;
		else if(field_name == SORT_SV_ITEM_PRTSDIF)
			field_val = (double)CandidateSVList[i].prtsdif;
		        	                                
		index = i;                                                                  	                        
		                                                                            	                                
		for(j=i+1; j<CandidateSVListLen; j++)                                                 	                                
		{  
			if(field_name == SORT_SV_ITEM_CN0)
				tmp_val = (double)CandidateSVList[j].cn0;
			else if(field_name == SORT_SV_ITEM_EPH_AGE)
				tmp_val = (double)CandidateSVList[j].ephage;
			else if(field_name == SORT_SV_ITEM_LOCK_TIME)
				tmp_val = (double)CandidateSVList[j].trklocktime;
			else if(field_name == SORT_SV_ITEM_DELTA_FREQ)
				tmp_val = (double)CandidateSVList[j].deltafreq;
			else if(field_name == SORT_SV_ITEM_PRTSDIF)
				tmp_val = (double)CandidateSVList[j].prtsdif;

		
			if(((sort_type == SV_CANDIDATE_SORT_TYPE_ASC) && ( tmp_val < field_val)) ||       
				((sort_type == SV_CANDIDATE_SORT_TYPE_DESC) && ( tmp_val > field_val)))   
			{	                                                            	                                
				field_val = tmp_val; 	                                        
				index = j;                                          	                                        
			}                                                                   	                                
		}                                                                           	                                
                                                                                            	                                
		swapSVCandidate(i, index);                                                  	                                
	} 
}


#define CTU_SMOOTH_WIN_LEN	5
#define BU_SMOOTH_WEIGTH		(0.02)

float64 UpdateECtu(float64 curctu, float64 currfcnt)
{
	static int32 ctusmoothcnt=0;
	static float64 ectu=0.0;
	static float64 ectu_rfcnt=0.0;
	float64 diftime=0.0;
	int32 cursystime = (int32)getSysOnTime();	//s

	if((TTFF==0) || (((cursystime-TTFF*0.001)<60) && (SysEnvSta>=SYS_DOWN_TOWN)))
		ctusmoothcnt = 0;

	if(ctusmoothcnt > 0)
	{
		diftime = (currfcnt - ectu_rfcnt)/RF_SAMP_FREQ;
		if(diftime>3.0)
			ctusmoothcnt = 0;
	}

	if(ctusmoothcnt == 0)
	{
		ectu = curctu;
		ectu_rfcnt = currfcnt;
	}
	else
	{
		if(ctusmoothcnt > CTU_SMOOTH_WIN_LEN)
			ctusmoothcnt = CTU_SMOOTH_WIN_LEN;

		ectu = (ectu * (ctusmoothcnt-1) + curctu)/ctusmoothcnt;
		ectu_rfcnt = currfcnt;
	}
	
	ctusmoothcnt++;

	return ectu;
}

float64 UpdateGpsEbu(float64 eCtu, float64 curbu, float64 currfcnt)
{
	float64 tmp=0.0;
	static float64 eGpsBu=0.0;
#if 0//(PVT_MODE==CALC_PVT_KF)
	static float64 eGpsBu_rfcnt=0.0;
	float64 diftime=0.0;
	float64 curEbu = 0.0;
	float64 curfreq = 0.0;
	int32 cursystime = (int32)getSysOnTime();

	if(IsCurPVTKFValid())
	{
		//if((TTFF==0) || (((cursystime-TTFF*0.001)<60) && (SysEnvSta>=SYS_DOWN_TOWN)))
		if((TTFF==0) || ((cursystime-TTFF*0.001)<60))
			eGpsBu_rfcnt = 0.0;
			
		if(eGpsBu_rfcnt > MICRO_NUM)
		{
			curfreq = GetCurRFSampleFrq();
			diftime = (currfcnt - eGpsBu_rfcnt)/curfreq;

			if(diftime>3.0)
				eGpsBu_rfcnt = 0.0;
		}

		if(eGpsBu_rfcnt > MICRO_NUM)
		{
			curEbu = eGpsBu + eCtu * diftime;
			eGpsBu = curEbu*(1-BU_SMOOTH_WEIGTH) + curbu * BU_SMOOTH_WEIGTH;
			eGpsBu_rfcnt = currfcnt;
		}
		else
		{
			eGpsBu = curbu;
			eGpsBu_rfcnt = currfcnt;
		}
	}
	else
#else
	{
		eGpsBu = curbu;
	}
#endif

	tmp = eGpsBu;
	return tmp;
}


float64 UpdateBdEbu(float64 eCtu, float64 curbu, float64 currfcnt)
{
	float64 tmp=0.0;
	static float64 eBdBu=0.0;
#if 0//(PVT_MODE==CALC_PVT_KF)
	static float64 eBdBu_rfcnt=0.0;
	float64 diftime=0.0;
	float64 curEbu = 0.0;
	float64 curfreq = 0.0;
	int32 cursystime = (int32)getSysOnTime();

	if(IsCurPVTKFValid())
	{
		if((TTFF==0) || (((cursystime-TTFF*0.001)<60) && (SysEnvSta>=SYS_DOWN_TOWN)))
			eBdBu_rfcnt = 0.0;

		if(eBdBu_rfcnt > MICRO_NUM)
		{
			curfreq = GetCurRFSampleFrq();
			diftime = (currfcnt - eBdBu_rfcnt)/curfreq;

			if(diftime>3.0)
				eBdBu_rfcnt = 0.0;
		}

		if(eBdBu_rfcnt > MICRO_NUM)
		{

			curEbu = eBdBu + eCtu * diftime;
			eBdBu = curEbu*(1-BU_SMOOTH_WEIGTH) + curbu * BU_SMOOTH_WEIGTH;
			eBdBu_rfcnt = currfcnt;
		}
		else
		{
			eBdBu = curbu;
			eBdBu_rfcnt = currfcnt;
		}
	}
	else
#else
	{
		eBdBu = curbu;
	}
#endif

	tmp = eBdBu;
	return tmp;
}
	


/**
*	@param	svmap: position sv map
*			bu:	receiver clock error, unit: second
*/
static int CalcRefNavSysWeek(PVT_FIX_INFO* pFixInfo)
{
	int32 trkch=-1, svid=-1;
	int32 maxcn0=0, maxcn0ch=0;
	int16 svweek = 0;
	double sysTr=0.0;
	int maxsv = -1,minsv = -1;
	int maxweek = 0,minweek = 100000;	
	
	//get wn
	for(trkch=0; trkch<MAXCHANNELS; trkch++)
	{
		if(GetBitWord256(&(pFixInfo->poschmap),trkch)==0)
			continue;
			
		svid = PVTTrkchInfo[trkch].svid;
#if SUPPORT_GLONASS
		if(SV_IsGps(svid) || SV_IsBd2(svid) || SV_IsGlo(svid))
#else
		if(SV_IsGps(svid) || SV_IsBd2(svid))
#endif
		{
			svid = PVTTrkchInfo[trkch].svid;
	
			if(SV_IsGps(svid))
				sysTr = FixNavSysTime.GpsTime;
			else if(SV_IsBd2(svid))
				sysTr = FixNavSysTime.Bd2Time;
#if SUPPORT_GLONASS
			else if(SV_IsGlo(svid))
				sysTr = FixNavSysTime.GloTime;
#endif

#ifndef _POSTPROC
#ifndef _SIMULATE
			svweek = GetWNFrmRefSVEph(svid, sysTr);
#else
			svweek = Debug_TrkInfo[trkch].wn;
#endif
#endif
			if(maxweek < svweek)
			{
				maxweek = svweek;
				maxsv = svid;
			}
			else if(minweek > svweek)
			{
				minweek = svweek;
				minsv = svid;
			}
		}
	}

	for(trkch=0; trkch<MAXCHANNELS; trkch++)
	{
		if(GetBitWord256(&(pFixInfo->poschmap),trkch)==0)
			continue;

		svid = PVTTrkchInfo[trkch].svid;
#if SUPPORT_GLONASS
		if(SV_IsGps(svid) || SV_IsBd2(svid) || SV_IsGlo(svid))
#else
		if(SV_IsGps(svid) || SV_IsBd2(svid))
#endif
		{
			if(!((minsv == svid) || (maxsv == svid)))
			{
				if(maxcn0 < PVTTrkchInfo[trkch].cn1s)
				{
					maxcn0 = PVTTrkchInfo[trkch].cn1s;
					maxcn0ch = trkch;
				}
			}
		}
	}


	return maxcn0ch;
}
	
static void CalcFixNavSysTime(PVT_FIX_INFO* pFixInfo)
{
	int32 svid=-1, maxcn0ch=0;
	double tow=0.0;
	double sysTr=0.0;
	double curNavSystime = 0.0;
	double trbase = 0.0;
	static double curEctu=0.0, curEGpsBu=0.0, curEBdBu=0.0;

	if(!getTowFrmBaseTr(TICLock_RFcnt, &trbase, NULL))
		return;

	curEctu = UpdateECtu(pFixInfo->ctu, TICLock_RFcnt);

#if SUPPORT_GLONASS
	if((pFixInfo->buflag & BITMAP_DIM_BU_GPS) 
		|| (pFixInfo->buflag & BITMAP_DIM_BU_GPS_BD2) 
		|| (pFixInfo->buflag & BITMAP_DIM_BU_GPS_GLO) 
		|| (pFixInfo->buflag & BITMAP_DIM_BU_GPS_BD2_GLO))
#else
	if((pFixInfo->buflag & BITMAP_DIM_BU_GPS) 
		|| (pFixInfo->buflag & BITMAP_DIM_BU_GPS_BD2))
#endif
	{		
		curEGpsBu = UpdateGpsEbu(curEctu, pFixInfo->gpsbu, TICLock_RFcnt);
		
		curNavSystime = trbase - curEGpsBu/SPEED_OF_LIGHT;
		if(curNavSystime > SECONDS_IN_WEEK)
			curNavSystime -= SECONDS_IN_WEEK;
		else if(curNavSystime < MICRO_NUM)
			curNavSystime += SECONDS_IN_WEEK;

		FixNavSysTime.GpsTime = curNavSystime;

		if(GetNavSysTimeByRFCnt(NAV_SYS_GPS,GPSTIME_SYNC_SRC_RTCPULSE_ERR05MS, TICLock_RFcnt, &tow, NULL))
			FixNavSysTime.clkerr_gps = ConvertFixTranTime(tow - curNavSystime);	
	}
#if SUPPORT_GLONASS
	if((pFixInfo->buflag & BITMAP_DIM_BU_BD2) 
		|| (pFixInfo->buflag & BITMAP_DIM_BU_GPS_BD2) 
		|| (pFixInfo->buflag & BITMAP_DIM_BU_BD2_GLO) 
		|| (pFixInfo->buflag & BITMAP_DIM_BU_GPS_BD2_GLO))
#else
	if((pFixInfo->buflag & BITMAP_DIM_BU_BD2) 
		|| (pFixInfo->buflag & BITMAP_DIM_BU_GPS_BD2))
#endif
	{
		double ectu;
		ectu = curEctu;
		curEBdBu = UpdateBdEbu(ectu, pFixInfo->bd2bu, TICLock_RFcnt);

		curNavSystime = trbase - curEBdBu/SPEED_OF_LIGHT;

		if(curNavSystime > SECONDS_IN_WEEK)
			curNavSystime -= SECONDS_IN_WEEK;
		else if(curNavSystime < MICRO_NUM)
			curNavSystime += SECONDS_IN_WEEK;

		FixNavSysTime.Bd2Time = curNavSystime;

		if(GetNavSysTimeByRFCnt(NAV_SYS_BD,GPSTIME_SYNC_SRC_RTCPULSE_ERR05MS, TICLock_RFcnt, &tow, NULL))
			FixNavSysTime.clkerr_bd = ConvertFixTranTime(tow - curNavSystime);	
	}

#if SUPPORT_GLONASS
	if((pFixInfo->buflag & BITMAP_DIM_BU_GLO) 
		|| (pFixInfo->buflag & BITMAP_DIM_BU_GPS_GLO) 
		|| (pFixInfo->buflag & BITMAP_DIM_BU_BD2_GLO) 
		|| (pFixInfo->buflag & BITMAP_DIM_BU_GPS_BD2_GLO))
	{
		curNavSystime = trbase - (pFixInfo->globu)/SPEED_OF_LIGHT;
		if(curNavSystime > SECONDS_IN_WEEK)
			curNavSystime -= SECONDS_IN_WEEK;
		else if(curNavSystime < MICRO_NUM)
			curNavSystime += SECONDS_IN_WEEK;

		FixNavSysTime.GloTime = curNavSystime;

		if(GetNavSysTimeByRFCnt(NAV_SYS_GLO,GPSTIME_SYNC_SRC_RTCPULSE_ERR05MS, TICLock_RFcnt, &tow, NULL))
			FixNavSysTime.clkerr_glo = ConvertFixTranTime(tow - curNavSystime);	
	}
#endif

	FixNavSysTime.TimeMap = pFixInfo->buflag;
	FixNavSysTime.RFCnt = TICLock_RFcnt;
	FixNavSysTime.RFCntEx = TICLock_NxRFcnt;

	maxcn0ch = CalcRefNavSysWeek(pFixInfo);

	svid = PVTTrkchInfo[maxcn0ch].svid;
	if(SV_IsGps(svid))
		sysTr = FixNavSysTime.GpsTime;
	else if(SV_IsBd2(svid))
		sysTr = FixNavSysTime.Bd2Time;
#if SUPPORT_GLONASS
	else if(SV_IsGlo(svid))
		sysTr = FixNavSysTime.GloTime;
#endif
	else
	{
		FixNavSysTime.TimeMap = 0;
		return;
	}
#ifndef _POSTPROC
#ifndef _SIMULATE
	FixNavSysTime.WN = GetWNFrmRefSVEph(PVTTrkchInfo[maxcn0ch].svid, sysTr);
#else
	FixNavSysTime.WN = Debug_TrkInfo[maxcn0ch].wn;
#endif
#endif
	//Update Nav system time bias
	if((pFixInfo->buflag & BITMAP_DIM_BU_GPS) &&(pFixInfo->buflag & BITMAP_DIM_BU_BD2))
		FixNavSysTime.bias_gps2bd = pFixInfo->bd2bu - pFixInfo->gpsbu;

#if SUPPORT_GLONASS
	if((pFixInfo->buflag & BITMAP_DIM_BU_GPS) &&(pFixInfo->buflag & BITMAP_DIM_BU_GLO))
		FixNavSysTime.bias_gps2glo = pFixInfo->globu - pFixInfo->gpsbu;

	if((pFixInfo->buflag & BITMAP_DIM_BU_BD2) &&(pFixInfo->buflag & BITMAP_DIM_BU_GLO))
		FixNavSysTime.bias_bd2glo = pFixInfo->globu - pFixInfo->bd2bu;
#endif

	return;
}


static boolean CheckLsPosAndVel(byte chkmap, PVT_FIX_INFO* pFixInfo)
{
	static ECEF lastRcvrPos={0.0,};
	static double lastRcvrAlt=0.0;
	static const double AltTbl[] = {120, 120, 200, 300, 400};
	static const double PosDistTbl[] = {120, 120, 140, 200, 300}; //set different altitude and position distance threshold for different systen environment, unit: meters.
	static const double HDOPTbl[] = {5.0, 10.0, 20.0, 50.0, 50.0};

	ECEF curpos = pFixInfo->rcvrpos;
	ECEF curvel = pFixInfo->rcvrvel;

	WGS wgspos = {0,};
	double speed = 0.0, planespeed = 0.0, head = 0.0, velspeed = 0.0, posDisTh=100.0;	
	double curFixCycle = FixCycle;
	int8 movestate = GetCPTRcvrMoveState();

	
	if(SysEnvCon == SYS_CONDITION_NORMAL)
		curFixCycle = 0.5;
	
	if(((pFixInfo->posres == FIX_NOT) && (chkmap & FIX_POS)) || ((pFixInfo->velres == FIX_NOT) && (chkmap & FIX_VEL)) || (pFixInfo->buflag == 0))
	{		
		lsPosVelChkPath = 1;
		return FALSE;
	}

	if(pFixInfo->dop.gdop > pActiveCPT->SysmCptWorkConfig.PdopMask)
	{
		lsPosVelChkPath = 8;
		return FALSE;
	}
	
	if(pFixInfo->dop.hdop > HDOPTbl[SysEnvSta - 1])
	{
		lsPosVelChkPath = 7;
		return FALSE;
	}

	if(chkmap & FIX_POS)
	{
		ECEF2WGS(&curpos, &wgspos);
		if(wgspos.alt < MIN_ALTITUDE || wgspos.alt > MAX_ALTITUDE)
		{
			lsPosVelChkPath = 2;	//altitude error
			return FALSE;
		}

		if(bLsFixCoarseChkOK)
		{
			if(f_abs(lastRcvrAlt - wgspos.alt) > (AltTbl[SysEnvSta - 1]*curFixCycle))
			{
				lsPosVelChkPath = 3;
				return FALSE;
			}

			if(movestate == RCVR_MOVESTATE_STATIC)
				posDisTh = 30.0;
			else if(movestate == RCVR_MOVESTATE_HIGHDYNAMIC)
				posDisTh = 1000.0*curFixCycle;
			else
				posDisTh = PosDistTbl[SysEnvSta - 1]*curFixCycle;
			
			if(Dis2PointsOnEarth(&lastRcvrPos, &curpos) > posDisTh)
			{
				lsPosVelChkPath = 4;
				return FALSE;
			}
		}

		lastRcvrPos = curpos;
		lastRcvrAlt = wgspos.alt;
	}

	if((chkmap & FIX_VEL) && (SysEnvCon == SYS_CONDITION_NORMAL))
	{
		ECEFVel2SpeedHeading(&curpos, &curvel, &planespeed, &head);
		speed = sqrt(curvel.x * curvel.x  + curvel.y * curvel.y + curvel.z * curvel.z);
		if((speed < planespeed) || (speed > MAX_SPEED)) //invalid case
		{
			lsPosVelChkPath = 5;		//speed error
			return FALSE;
		}
	
		if((movestate==RCVR_MOVESTATE_STATIC) || (movestate==RCVR_MOVESTATE_DYNAMIC))
		{
			velspeed = sqrt(speed * speed - planespeed * planespeed);
			if((ConPPOutputCnt>10) &&((velspeed > MAX_VERTICAL_SPEED_TH) || ((velspeed > MAX_VERTICAL_SPEED) && (velspeed > 0.5 * speed))))
			{
				lsPosVelChkPath = 6;		//vertical speed error
				return FALSE;
			}
		}

		if((movestate==RCVR_MOVESTATE_STATIC) && (speed > 2.0))
		{
			lsPosVelChkPath = 7;		//static rcvr, speed error
			return FALSE;
		}
	}
				
	return TRUE;
}


static void InitPVTKalmanFilter(void)
{
#if (PVT_MODE==CALC_PVT_KF)
	if(IsLsSecurity2InitPVTKF())	//condition to init KF
	{	
		InitPVTKF(&PVTFixInfo, TRUE);	
		
		bLsFixCoarseChkOK = FALSE;	//reset ls check flag	
	}
#endif
	return;
}


static boolean IsLsSecurity2InitPVTKF(void)
{
	static int32 ConLs2InitKFValidCnt = 0;
	static boolean bKFInited = FALSE;

	int32 trkch = 0, sumcn0 = 0, svcnt = 0;

	if(bLsFixQualityOk)
	{
		ConLs2InitKFValidCnt = 0;
		bKFStatus = KF_PVT_UNKNOWN;

		ls2InitPVTKFChkPath = 1;

		bKFInited = TRUE;
		return TRUE;
	}

	if(bLsFixCoarseChkOK)
		ConLs2InitKFValidCnt++;
	else
		ConLs2InitKFValidCnt = 0;

	if(bMixFixMaybeErr && (ConLs2InitKFValidCnt<=15))
	{
		ls2InitPVTKFChkPath = 5;
		return FALSE;
	}
	
	//check sv cn0 used to fix
	for(trkch = 0; trkch < MAXCHANNELS; trkch++)
	{
		if(GetBitWord256(&(PVTFixInfo.poschmap),trkch))
		{
			sumcn0 += PVTTrkchInfo[trkch].cn1s;
			svcnt++;
		}		
	}

	if((SysEnvCon == SYS_CONDITION_SIGNAL) && (bKFInited == TRUE))
	{
		if(((sumcn0 > svcnt * 34) && (ConLs2InitKFValidCnt >= 6) && ((PVTFixInfo.dop.hdop < 2.0) || (svcnt > 6))) || (ConLs2InitKFValidCnt > 15))
		{
			ConLs2InitKFValidCnt = 0;

			//if((svcnt > 6) && (PVTFixInfo.dop.hdop < 2.0) && (sumcn0 > svcnt * 38))
			//	bKFStatus = KF_PVT_GOOD;
			//else
				bKFStatus = KF_PVT_UNKNOWN;

			ls2InitPVTKFChkPath = 4;

			bKFInited = TRUE;
			
			return TRUE;
		}
	}
	else if(((sumcn0 > svcnt * 30) && (ConLs2InitKFValidCnt >= 4) && ((PVTFixInfo.dop.hdop < 2.0) || (svcnt > 6))) || (ConLs2InitKFValidCnt > 10))
	{
		ConLs2InitKFValidCnt = 0;

//		if((svcnt > 6) && (PVTFixInfo.dop.hdop < 2.0) && (sumcn0 > svcnt * 38))
//			bKFStatus = KF_PVT_GOOD;
//		else
			bKFStatus = KF_PVT_UNKNOWN;

		ls2InitPVTKFChkPath = 2;

		bKFInited = TRUE;
		
		return TRUE;
	}
	else
	{
		ls2InitPVTKFChkPath = 3;
		return FALSE;
	}

	return FALSE;
}


int32 FindSVOrbitType(int32 svid)
{
	int32 idx  = SV_ORBIT_TYPE_UNKNOW;
	if(SV_IsGps(svid))
		idx = SV_ORBIT_TYPE_GPS;
	else if(SV_IsBd2Geo(svid))
		idx = SV_ORBIT_TYPE_GEO;
	else if(SV_IsBd2IGSO(svid))
		idx = SV_ORBIT_TYPE_BD_IGSO;
	else if(SV_IsBd2Meo(svid))
		idx = SV_ORBIT_TYPE_BD_MEO;
#if SUPPORT_GLONASS
	else if(SV_IsGlo(svid))
		idx = SV_ORBIT_TYPE_GLO;
#endif

	return idx;
}

int32 CheckLSQuality(word32 chnmap)
{
	int32 trkch=0, svid=0;
	float64 bu=0.0,ctu=0.0;
	ECEF rcvrPos, rcvrVel;
	
	if(PVTFixInfo.posres == FIX_NOT)
		return 0;
	
	for(trkch=0; trkch<MAXCHANNELS; trkch++)
	{
		if(!(chnmap & (1<<trkch)))
			continue;

		svid = PVTTrkchInfo[trkch].svid;
		if(SV_IsGps(svid))
			bu= PVTFixInfo.gpsbu;
		else
			bu= PVTFixInfo.bd2bu;


		rcvrPos = PVTFixInfo.rcvrpos;
		rcvrVel = PVTFixInfo.rcvrvel;
		ctu = PVTFixInfo.ctu;

		
		CalcSVPrFdResidue(svid, &rcvrPos, &bu, &rcvrVel, &ctu, &(PVTTrkchInfo[trkch]));
	}

	return 1;
}



bool getLatestFixStatus(void)
{
	if(PVTFixInfo.posres >= FIX_2D)
		return TRUE;
	else
		return FALSE;
}


//interface for other module
//out put the fix info before postprocess
int32 getCurFixPosInfo(ECEF *pPos, FIX_DOP *pDOP, word256 *pChmap)	
{	
	if(PVTFixInfo.posres != FIX_NOT)
	{
		if(pPos != NULL)
		{
			pPos->x = PVTFixInfo.rcvrpos.x;
			pPos->y = PVTFixInfo.rcvrpos.y;
			pPos->z = PVTFixInfo.rcvrpos.z;
		}

		if(pDOP != NULL)
		{
			memcpy(pDOP, &(PVTFixInfo.dop), sizeof(FIX_DOP));
		}
	}

	if(pChmap != NULL)
		memcpy(pChmap, &(PVTFixInfo.poschmap), sizeof(PVTFixInfo.poschmap));

	return PVTFixInfo.posres;
}

//out put the fix info before postprocess
int32 getCurFixVelInfo(ECEF *pVel, word256 *pChmap)	
{	
	if((pVel != NULL) && (PVTFixInfo.velres != FIX_NOT))
	{
		pVel->x = PVTFixInfo.rcvrvel.x;
		pVel->y = PVTFixInfo.rcvrvel.y;
		pVel->z = PVTFixInfo.rcvrvel.z;
	}

	if(pChmap != NULL)
		memcpy(pChmap,&(PVTFixInfo.velchmap), sizeof(PVTFixInfo.velchmap));

	return PVTFixInfo.velres;
}

int32 getCurFixGPSTime(int16 *pWN, double* pTow, double* pRFcnt, double* pRFcntEx, double* pClkerr)
{
	if(PVTFixInfo.posres == FIX_NOT)
		return -1;
#if SUPPORT_GLONASS
	if((FixNavSysTime.TimeMap & BITMAP_DIM_BU_GPS) 
		|| (FixNavSysTime.TimeMap & BITMAP_DIM_BU_GPS_BD2) 
		|| (FixNavSysTime.TimeMap & BITMAP_DIM_BU_GPS_GLO) 
		|| (FixNavSysTime.TimeMap & BITMAP_DIM_BU_GPS_BD2_GLO))
#else
	if((FixNavSysTime.TimeMap & BITMAP_DIM_BU_GPS) 
		|| (FixNavSysTime.TimeMap & BITMAP_DIM_BU_GPS_BD2) )
#endif
	{
		if(pWN != NULL)
			*pWN = FixNavSysTime.WN;
		
		if(pTow != NULL)
			*pTow = FixNavSysTime.GpsTime;

		if(pRFcnt != NULL)
			*pRFcnt = FixNavSysTime.RFCnt;

		if(pRFcntEx != NULL)
			*pRFcntEx = FixNavSysTime.RFCntEx;

		if(pClkerr != NULL)
			*pClkerr = FixNavSysTime.clkerr_gps;

		return FixNavSysTime.TimeMap;
	}
	else
		return -1;
}

int32 getCurFixBDTime(int16 *pWN, double* pTow, double* pRFcnt,double* pRFcntEx, double* pClkerr)
{
	if(PVTFixInfo.posres == FIX_NOT)
		return -1;
#if SUPPORT_GLONASS
	if((FixNavSysTime.TimeMap & BITMAP_DIM_BU_BD2) 
		|| (FixNavSysTime.TimeMap & BITMAP_DIM_BU_GPS_BD2) 
		|| (FixNavSysTime.TimeMap & BITMAP_DIM_BU_BD2_GLO) 
		|| (FixNavSysTime.TimeMap & BITMAP_DIM_BU_GPS_BD2_GLO))
#else
	if((FixNavSysTime.TimeMap & BITMAP_DIM_BU_BD2) || (FixNavSysTime.TimeMap & BITMAP_DIM_BU_GPS_BD2))
#endif
	{

		if(pWN != NULL)
			*pWN = FixNavSysTime.WN;
		
		if(pTow != NULL)
			*pTow = FixNavSysTime.Bd2Time;

		if(pRFcnt != NULL)
			*pRFcnt = FixNavSysTime.RFCnt;

		if(pRFcntEx != NULL)
			*pRFcntEx = FixNavSysTime.RFCntEx;

		if(pClkerr != NULL)
			*pClkerr = FixNavSysTime.clkerr_bd;

		return FixNavSysTime.TimeMap;
	}
	else
		return -1;
}

#if SUPPORT_GLONASS
int32 getCurFixGloTime(int16 *pWN, double* pTow, double* pRFcnt, double* pRFcntEx,double* pClkerr)
{
	if(PVTFixInfo.posres == FIX_NOT)
		return -1;
	
	if((FixNavSysTime.TimeMap & BITMAP_DIM_BU_GLO) 
		|| (FixNavSysTime.TimeMap & BITMAP_DIM_BU_GPS_GLO) 
		|| (FixNavSysTime.TimeMap & BITMAP_DIM_BU_BD2_GLO) 
		|| (FixNavSysTime.TimeMap & BITMAP_DIM_BU_GPS_BD2_GLO))
	{

		if(pWN != NULL)
			*pWN = FixNavSysTime.WN;
		
		if(pTow != NULL)
			*pTow = FixNavSysTime.GloTime;

		if(pRFcnt != NULL)
			*pRFcnt = FixNavSysTime.RFCnt;

		if(pRFcntEx != NULL)
			*pRFcntEx = FixNavSysTime.RFCntEx;

		if(pClkerr != NULL)
			*pClkerr = FixNavSysTime.clkerr_glo;

		return FixNavSysTime.TimeMap;
	}
	else
		return -1;
}

#endif

byte getCurFixBuFlag(void)
{
	
return PVTFixInfo.buflag;
}

bool getCurFixGps2BDBias(double* pBias)
{
	if((PVTFixInfo.posres >= FIX_3D) && (FixNavSysTime.TimeMap & BITMAP_DIM_BU_GPS) && (FixNavSysTime.TimeMap & BITMAP_DIM_BU_BD2))
	{
		if(pBias != NULL)
			*pBias = FixNavSysTime.bias_gps2bd;

		return TRUE;
	}

	return FALSE;
}


int32 getCurFixCtu(double *pCtu)
{
	if(PVTFixInfo.posres != FIX_NOT)
	{
		if(pCtu != NULL)
			*pCtu = PVTFixInfo.ctu;
	}

	return PVTFixInfo.posres;
}

int8 getCurFilterType(void)
{
	return PVTFixInfo.filtertype;
}

double getTICLockRFcnt(void)
{
	return TICLock_RFcnt;
}

double getTICLockRFcnt100M(void)
{
	return TICLock_NxRFcnt;
}

int32 GetReferInfoForNavBit(PVT_TRKCH_INFO* pCurObsInfo,float64* pTrBase, int32* pWN)
{
	int32 validsvid=-1;
#ifndef _POSTPROC
#ifndef  _SIMULATE
	ECEF hisPos={0.0,};
	int32 trk_ch=0,svid = -1;
	int32 maxcn0=-1;
	float64 validts=0.0, ts = 0.0;
	int32 cn0 = 0;
	float64 clkerr = 0.0, freerr=0.0;
	ECEF baseSvPos = {0,}, baseSVVel = {0,};
	float64 deltax = 0.0, deltay = 0.0, deltaz = 0.0;	
	float64 transtime = 0;
	int32 wn=0, eph_age=0;

	byte hisPosFlag = getRcvrHistoricalPos(&hisPos, NULL);
	if(hisPosFlag > HISTORICAL_POS_LEVEL_40KM)
		return -1;
	
	for(trk_ch=0; trk_ch<MAXCHANNELS; trk_ch++)
	{
		if((pCurObsInfo[trk_ch].bPRValid ==FALSE))
			continue;

		if(((pCurObsInfo[trk_ch].CCBF&0x3) == 0x3) && (pCurObsInfo[trk_ch].synflag==SF_SYNC_SUCCESS) && (pCurObsInfo[trk_ch].syncsrc==0))
		{
			svid = pCurObsInfo[trk_ch].svid;
			cn0 = pCurObsInfo[trk_ch].cn1s;

#if SUPPORT_GLONASS
			if(((SV_IsGps(svid) || SV_IsBd2Meo(svid)) && (cn0 < 30)) 
				|| (SV_IsBd2Geo(svid) && (cn0 < 35)) 
				|| SV_IsGlo(svid))
#else
			if(((SV_IsGps(svid) || SV_IsBd2Meo(svid)) && (cn0 < 30)) 
				|| (SV_IsBd2Geo(svid) && (cn0 < 35)))
#endif
				continue;

			if(CheckEphHealth(svid)==-1)
				continue;

			if((Sys3AlmEphUTCInfo.GpsBd2_EphStruct[svid-MinGpsSvID].vflg == 0))
				continue;

			if(!((glStruAcqu.Info[trk_ch].Acqmode >= ACQ_MODE_0) && (glStruAcqu.Info[trk_ch].Acqmode <= ACQ_MODE_2)))
				continue;
	
			SWI_disable();
			wn = GetWNFrmRefSVEph(svid, pCurObsInfo[trk_ch].ts);
			eph_age = CalcSVEPHAge(svid, wn, pCurObsInfo[trk_ch].ts);
			SWI_enable();

			if((eph_age >= SV_EPH_AGE_USABLE_THRES) 
			|| (eph_age < SV_MIN_EPH_AGE)
#if SUPPORT_GLONASS
			|| (SV_IsGlo(svid) && (eph_age>=GLO_SV_EPH_AGE_USABLE_THRES))
#endif
			)
				continue;

			if(cn0>maxcn0)  
			{
				validsvid = svid;
				maxcn0 = cn0;
				validts = pCurObsInfo[trk_ch].ts;
			}
		}
	}

	if(validsvid <=0)
		return -1;

	CalcSV_PVT_Eph(validsvid, validts, &baseSvPos, &baseSVVel, &clkerr,&freerr,NULL);
	
	ts = validts - clkerr;
	
	deltax = baseSvPos.x-hisPos.x;
	deltay = baseSvPos.y-hisPos.y;
	deltaz = baseSvPos.z-hisPos.z;	
	transtime = sqrt(deltax*deltax+ deltay*deltay + deltaz*deltaz)*RECIP_SPEED_OF_LIGHT;

	ts = ts + transtime;

	if(ts >= SECONDS_IN_WEEK)
		ts -= SECONDS_IN_WEEK;
	else if(ts < 0.0)
		ts += SECONDS_IN_WEEK;

	*pTrBase = ts;
	*pWN = GetWNFrmRefSVEph(validsvid, ts);

#endif
#endif
	return validsvid;
}


void GetECASVInfo(PVT_TRKCH_INFO* pECATrkchInfo)
{
	ECEF rcvrpos={0.0,};
	WGS wgspos={0.0,};
	double alt=0.0;
	if(!getCurSecurityAltitude(&alt))
		return;

	if(getRcvrInfo(&rcvrpos,NULL,NULL,NULL)==FIX_NOT)
		return;

	if(ECEF2WGS(&rcvrpos, &wgspos)!=0)
		return;

	wgspos.alt = alt;
	if(WGS2ECEF(&wgspos,&rcvrpos)!=0)
		return;
	
	pECATrkchInfo->bPRValid = TRUE;

	pECATrkchInfo->cn100ms = 35;
	pECATrkchInfo->cn1s = 35;

	pECATrkchInfo->svid = ECA_SV_ID;

	pECATrkchInfo->svpos.x = 0.0;
	pECATrkchInfo->svpos.y = 0.0;
	pECATrkchInfo->svpos.z = 0.0;

	pECATrkchInfo->svvel.x = 0.0;
	pECATrkchInfo->svvel.y = 0.0;
	pECATrkchInfo->svvel.z = 0.0;
	
	pECATrkchInfo->pr = sqrt(rcvrpos.x*rcvrpos.x + rcvrpos.y*rcvrpos.y + rcvrpos.z*rcvrpos.z);

	return;
}


bool IsNearSecBoundary(double RFCnt)
{
	float64 tow=0.0, res=0.0, timegap=0.0;
	
	if(GetNavSysTimeByRFCnt(NAV_SYS_NONE,GPSTIME_SYNC_SRC_FIXRFCNT, RFCnt, &tow, NULL) == FALSE)
		return FALSE;


	if(TSK1_List[0].cycle>0)
	{
		timegap = TSK1_List[0].cycle*0.01;
		res = tow - ((int32)(tow/timegap+0.5))*timegap;
		if(fabs(res)<0.005)
		{
			res = tow - (int32)(tow+0.5);
			if(fabs(res)<0.01)
				bFixAtSecBoundary = TRUE;

			return TRUE;
		}
		else
			return FALSE;
	}


	return FALSE;
}

bool IsSecBoundaryChange(void)
{
	static bool oldSecBoundary = FALSE;
	bool isChangeFlag = FALSE;

	if(bFixAtSecBoundary == oldSecBoundary)
		isChangeFlag = FALSE;
	else 
		isChangeFlag = TRUE;

	oldSecBoundary = bFixAtSecBoundary;

	return isChangeFlag;
}


void SelectECA2Fix(word256* pPosChMap, word256* pVelChMap)
{
	int32 posusecnt=0, velusecnt=0, trkch=0;
	
	if((!PVTTrkchInfo[ECA_TRKCH_ID].bPRValid) || (SysEnvSta < SYS_DOWN_TOWN))
		return;
	
	for(trkch=0; trkch<MAXCHANNELS; trkch++)
	{
		if(GetBitWord256(pPosChMap,trkch))
			posusecnt++;

		if(GetBitWord256(pVelChMap,trkch))
			velusecnt++;
	}

	if((posusecnt<MAX_USE_SV_NUM) && (SysEnvSta >= SYS_HALF_OPEN_SKY))
		SetBitWord256(pPosChMap,ECA_TRKCH_ID);

	if(velusecnt<MAX_USE_SV_NUM)
	{
		if(((SysEnvSta >= SYS_DOWN_TOWN) && (SysEnvCon == SYS_CONDITION_NORMAL))
			||((SysEnvSta >= SYS_DOWN_TOWN) && (SysEnvCon == SYS_CONDITION_SIGNAL)))
		SetBitWord256(pVelChMap,ECA_TRKCH_ID);
	}

	return;
}

bool CheckNavHisPos(PVT_FIX_INFO* pCurFixInfo)
{
	ECEF   hisPos = {0.0,};
	double ts = 0.0;
	double curTow = 0.0;
	double diftime = 0.0, difpos=0.0;
	int32  nWN = 0;
	int32  res = 0;
	ECEF curpos={0.0,0.0,0.0};
	
	res = getRcvrHistoricalPos(&hisPos,&ts);

	if((res == HISTORICAL_POS_INVALID) || (ts < MICRO_NUM))
		return TRUE;

	if((pCurFixInfo != NULL)&&(res == HISTORICAL_POS_LEVEL_40KM) && (pCurFixInfo->dop.pdop < pActiveCPT->SysmCptWorkConfig.PdopMask))
	{
		curpos = pCurFixInfo->rcvrpos;
		difpos = (curpos.x-hisPos.x)*(curpos.x-hisPos.x)+(curpos.y-hisPos.y)*(curpos.y-hisPos.y)+(curpos.z-hisPos.z)*(curpos.z-hisPos.z);
		if(difpos > 1.0e10)	//100km
		{
			setRcvrHistoricalPos(&hisPos,NULL, HISTORICAL_POS_LEVEL_2000KM);
			return FALSE;
		}
	}

	if(GetCurNavSysTime(NAV_SYS_NONE,GPSTIME_SYNC_SRC_GLO_ONLY,&curTow,&nWN))
	{
		diftime = (nWN * SECONDS_IN_WEEK + curTow) - ts;
		if(diftime > RCVR_POS_SECURITY_TIME)
		{
			if(res == HISTORICAL_POS_LEVEL_40KM)
			{
				setRcvrHistoricalPos(&hisPos,NULL, HISTORICAL_POS_LEVEL_2000KM);

/*
				if(GetSyncNavSysTimeSrc(NAV_SYS_GPS) <= GPSTIME_SYNC_SRC_REFBIT_RFCNT)
					RestSyncSynTime(NAV_SYS_GPS, GPSTIME_SYNC_SRC_NONE);

				if(GetSyncNavSysTimeSrc(NAV_SYS_BD) <= GPSTIME_SYNC_SRC_REFBIT_RFCNT)
					RestSyncSynTime(NAV_SYS_BD, GPSTIME_SYNC_SRC_NONE);

				if(GetSyncNavSysTimeSrc(NAV_SYS_GLO) <= GPSTIME_SYNC_SRC_REFBIT_RFCNT)
					RestSyncSynTime(NAV_SYS_GLO, GPSTIME_SYNC_SRC_NONE);
*/		  			  
				checkhispospath = 2;			

				return FALSE;
			}
		}
		else
		{
			if(res != HISTORICAL_POS_INVALID)
			{
			  setRcvrHistoricalPos(&hisPos,NULL, HISTORICAL_POS_LEVEL_40KM);
			  checkhispospath = 4;									  
			}
		}
	}
	else if(getSysOnTime() > RCVR_POS_SECURITY_TIME)
	{
		checkhispospath = 5;				
		setRcvrHistoricalPos(&hisPos,&ts, HISTORICAL_POS_LEVEL_2000KM);
		return FALSE;
	}


	return TRUE;
}

word64_rtc GetTrkchLockedTime(int32 trkch, word64_rtc curRFcnt)
{
	word64_rtc lockstarttime=glStruAcqu.Info[trkch].lockedStartTime;
	word64_rtc locktime={0,0};

	if((lockstarttime.high==0) && (lockstarttime.low==0))
		return locktime;

	locktime.high = curRFcnt.high - lockstarttime.high;

	if(curRFcnt.low >= lockstarttime.low)
		locktime.low = curRFcnt.low - lockstarttime.low;
	else
	{
		locktime.high--;
		locktime.low = (4294967296 - lockstarttime.low) + curRFcnt.low;
	}
	
	return locktime;	//unit: RFcnt
}


void GetTrkchSteadyLockedTime(int32 trkch, PVT_TRKCH_INFO *pCurPVTTrkchInfo, double curRFcnt)
{
	int32 steadylocktime=0;

	if(glStruAcqu.Info[trkch].steadyStartTime<MICRO_NUM)
		glStruAcqu.Info[trkch].steadyStartTime = curRFcnt;
#if SUPPORT_GLONASS
	if(SV_IsBd2Geo(pCurPVTTrkchInfo->svid) || SV_IsGlo(pCurPVTTrkchInfo->svid))
#else
	if(SV_IsBd2Geo(pCurPVTTrkchInfo->svid))
#endif
	{
		if(pCurPVTTrkchInfo->trkmode != TRK_STATE_7)
		{
			glStruAcqu.Info[trkch].steadyStartTime = curRFcnt;
			ResetTrkchReverse(trkch);
		}
	}
	else if((pCurPVTTrkchInfo->freq_point & DSP_L2P_FRE) == DSP_L2P_FRE )
	{
		if((pCurPVTTrkchInfo->trkmode != TRK_STATE_13) && (pCurPVTTrkchInfo->trkmode !=TRK_STATE_14))
		{
			glStruAcqu.Info[trkch].steadyStartTime = curRFcnt;
			ResetTrkchReverse(trkch);
		}
	}
	else
	{
		if(pCurPVTTrkchInfo->trkmode != TRK_STATE_10)
		{
			glStruAcqu.Info[trkch].steadyStartTime = curRFcnt;
			ResetTrkchReverse(trkch);
		}
	}

	steadylocktime = RECIP_RF_SAMP_FREQ * (curRFcnt - glStruAcqu.Info[trkch].steadyStartTime);
	
	pCurPVTTrkchInfo->steadyLockedtime = (int32)(steadylocktime*10.0+0.5);	//0.1s

	return;
}

#ifndef _POSTPROC
#ifndef _SIMULATE
word16 GetEphFrameBufFlag(int32 trkch, int32 svid)
{
	word16 flag = 0;

#if SUPPORT_GLONASS
	if(SV_IsGps(svid)||SV_IsBd2Meo(svid)||SV_IsGlo(svid))
#else
	if(SV_IsGps(svid)||SV_IsBd2Meo(svid))
#endif
	{
		flag = EphFrameBuffer[trkch].vflg;
	}
	else 	//geo
	{
		if (glStruAcqu.Info[trkch].FreqPoint==DSP_B1I_FRE)
			flag = GeoD2EphFrameBuffer[svid-MinBD2SvID].vflg;
		else if(glStruAcqu.Info[trkch].FreqPoint==DSP_B2I_FRE)
			flag = GeoD2EphFrameBufferB2[svid-MinBD2SvID].vflg;
		else if(glStruAcqu.Info[trkch].FreqPoint==DSP_B3I_FRE)
			flag = GeoD2EphFrameBufferB3[svid-MinBD2SvID].vflg;
	}
	
	return flag;
}
#endif
#endif


static bool bTestPosVel=TRUE;
static ECEF PosTrue={-1336317.4599164,5333575.521585,3222461.85890275};	//chengdu office ANT1
//static ECEF PosTrue={-1248211.0806,5382778.8855,3175956.0213};	//signal generatot chengdu static 2h

static ECEF VelTrue={0.0,0.0,0.0};

void SetTestPosVel(ECEF* pPos, ECEF* pVel)
{
	memcpy(&PosTrue, pPos, sizeof(ECEF));
	memcpy(&VelTrue, pVel, sizeof(ECEF));
	bTestPosVel = TRUE;	
}

bool GetTestPosVel(ECEF* pPos, ECEF* pVel)
{
	if(bTestPosVel)
	{
		*pPos = PosTrue;
		*pVel = VelTrue;
	}

	return bTestPosVel;
}

float64 GetTestPosErr(ECEF* pCurPos)
{
	float64 difx=0.0, dify=0.0, difz=0.0;
	if(bTestPosVel)
	{
		difx = pCurPos->x-PosTrue.x;
		dify = pCurPos->y-PosTrue.y;
		difz = pCurPos->z-PosTrue.z;
		
		return (sqrt(difx*difx + dify*dify+difz*difz));
	}
	else
		return 0.0;
}

float64 GetTestVelErr(ECEF* pCurVel)
{
	float64 difx=0.0, dify=0.0, difz=0.0;
	
	if(bTestPosVel)
	{
		difx = pCurVel->x-VelTrue.x;
		dify = pCurVel->y-VelTrue.y;
		difz = pCurVel->z-VelTrue.z;
		
		return (sqrt(difx*difx + dify*dify+difz*difz));
	}
	else
		return 0.0;

}

void GetACQChDebugData(int32 ACQch, DBG_SYSM_ACQ *pACQDbgData)
{
#ifndef _POSTPROC
#ifndef _SIMULATE
	pACQDbgData->acqState = glStruAcqu.State;
	pACQDbgData->trkchid = glStruAcqu.Channel;
	pACQDbgData->trkchsvid = (UINT8)(glStruAcqu.Info[glStruAcqu.Channel].SvID);
	pACQDbgData->state = glStruAcqu.Info[glStruAcqu.Channel].State;
	pACQDbgData->acqmode = glStruAcqu.Info[glStruAcqu.Channel].Acqmode;
	pACQDbgData->failcnt = SVInfo[pACQDbgData->trkchsvid-1].AcqTotalFailCnt;
	pACQDbgData->freq = (int16)(glStruAcqu.Info[glStruAcqu.Channel].ModifyFre);

	pACQDbgData->spancnt = glStruAcqu.gl_spancnt;
	pACQDbgData->spancntth = glStruAcqu.Info[glStruAcqu.Channel].spancnt;
#endif
#endif
}

bool GetTRKChDebugData(int32 trkch, DBG_SYSM_TRK *pTrkDbgData)
{
#ifndef _POSTPROC
#ifndef _SIMULATE
	word16 bused = 0;
	double swdop = 0.0, tmpdata;
	int32 svid=0, i=0;
#if SUPPORT_GLONASS
	int32 wn=0;
	double tow=0.0;
#endif

#if (!__X2240_SYSTEM)
	if((glStruAcqu.Info[trkch].SvID <=0) || (glStruAcqu.Info[trkch].State<=IDLE_PVT))
		return FALSE;
#endif

	memset(pTrkDbgData, 0, sizeof(DBG_SYSM_TRK));

	svid = glStruAcqu.Info[trkch].SvID;
	
	pTrkDbgData->trkid = trkch;
	pTrkDbgData->svid = svid;
	pTrkDbgData->fredot = FindBitWord16(glStruAcqu.Info[trkch].FreqPoint,1)+1;
	pTrkDbgData->state = glStruAcqu.Info[trkch].State;
	pTrkDbgData->acqmod = glStruAcqu.Info[trkch].Acqmode;
	pTrkDbgData->trkmode = PVTTrkchInfo[trkch].trkmode;
	pTrkDbgData->CCBF = PVTTrkchInfo[trkch].CCBF;
	pTrkDbgData->C2Fai = PVTTrkchInfo[trkch].C2FAI;
	if(PVTTrkchInfo[trkch].steadyLockedtime>2550)
		pTrkDbgData->locktime = 255;
	else
		pTrkDbgData->locktime = PVTTrkchInfo[trkch].steadyLockedtime/10;
	
	if(GetBitWord256(&(PVTFixInfo.poschmap),trkch))
		bused = 1;

	if(GetBitWord256(&(PVTFixInfo.velchmap),trkch))
		bused += 10;

	for(i=0; i<RtkEpoch.nCommonSat; i++)
	{
		if(RtkEpoch.CommonSat[i].SatID == svid)
		{
			if(RtkEpoch.SelectFrepMap & glStruAcqu.Info[trkch].FreqPoint)
				bused += 100;
			
			break;
		}
	}

	if(PVTTrkchInfo[trkch].bPRValid)
		bused += 1000;

	pTrkDbgData->buse = bused;
	pTrkDbgData->cn1s = PVTTrkchInfo[trkch].cn1s;
	pTrkDbgData->sferr = (int8)(SfDataInfo[trkch].erros);
	pTrkDbgData->selpath = PVTTrkchInfo[trkch].selpath;
	
	pTrkDbgData->sync = SfDataInfo[trkch].SfSyncSource*100 + DataBitBuf[trkch].bitSyncSource*10 
						+PVTTrkchInfo[trkch].synflag;

	pTrkDbgData->bitcnt = PVTTrkchInfo[trkch].bitnum;
	pTrkDbgData->sfid = PVTTrkchInfo[trkch].framenum;

	pTrkDbgData->intpInv = PVTTrkchInfo[trkch].carrIntInvCnt;
	pTrkDbgData->intp = PVTTrkchInfo[trkch].carrInt;
	pTrkDbgData->decp = PVTTrkchInfo[trkch].carrDec;
	
	pTrkDbgData->hwfre = (int32)(PVTTrkchInfo[trkch].fd *100.0);
	//pTrkDbgData->freerr = (int32)(PVTTrkchInfo[trkch].frqerr*100.0);
	
	pTrkDbgData->Iono = (int32)(PVTTrkchInfo[trkch].iono);
	pTrkDbgData->Tropo = (int8)(PVTTrkchInfo[trkch].tropo);

	pTrkDbgData->Ncnt = PVTTrkchInfo[trkch].Ncnt;

	pTrkDbgData->tsdif = (int32)(PVTTrkchInfo[trkch].difts);

	pTrkDbgData->ionosrc = PVTTrkchInfo[trkch].ionosrc+PVTTrkchInfo[trkch].reverseValid*10+SfDataInfo[trkch].reverse*100;	
	pTrkDbgData->reserr_pr = (int16)(PVTTrkchInfo[trkch].resiErr_pr);
	pTrkDbgData->reserr_fd = (int8)(PVTTrkchInfo[trkch].resiErr_fd);
	pTrkDbgData->LeapsGlo = (int8) (PVTTrkchInfo[trkch].Tleaps);
#if SUPPORT_GLONASS
	pTrkDbgData->truesvid = (int8) ConvertGloSlotID2SVID(svid);
#endif

	tmpdata = PVTTrkchInfo[trkch].clkerr*SPEED_OF_LIGHT*10.0;	//0.1m
	pTrkDbgData->clkerr = (int32)(tmpdata);
	pTrkDbgData->dclkerr = (int32)((tmpdata-pTrkDbgData->clkerr)*1000.0+0.5);	//0.0001m

	tmpdata = PVTTrkchInfo[trkch].svpos.x*10.0;
	pTrkDbgData->posx = (int32)(tmpdata);	//0.1m
	pTrkDbgData->dposx = (int16)((tmpdata - pTrkDbgData->posx)*1000.0+0.5);	//0.0001m
	tmpdata = PVTTrkchInfo[trkch].svpos.y*10.0;
	pTrkDbgData->posy = (int32)(tmpdata);	//0.1m
	pTrkDbgData->dposy = (int16)((tmpdata - pTrkDbgData->posy)*1000.0+0.5);//0.0001m
	tmpdata = PVTTrkchInfo[trkch].svpos.z*10.0;
	pTrkDbgData->posz = (int32)(tmpdata);	//0.1m
	pTrkDbgData->dposz = (int16)((tmpdata - pTrkDbgData->posz)*1000.0+0.5);//0.0001m

	pTrkDbgData->velx = (int32)(PVTTrkchInfo[trkch].svvel.x*100.0);
	pTrkDbgData->vely = (int32)(PVTTrkchInfo[trkch].svvel.y*100.0);
	pTrkDbgData->velz = (int32)(PVTTrkchInfo[trkch].svvel.z*100.0);

	pTrkDbgData->sfflag = GetEphFrameBufFlag(trkch,svid);
	//pTrkDbgData->wdflag = GetEphFrameBufFlagCH(trkch);//[trkch].partyCheckFlag;
	pTrkDbgData->ts = PVTTrkchInfo[trkch].ts+PVTTrkchInfo[trkch].clkerr;	//not include sv clkerr
	pTrkDbgData->pr = PVTTrkchInfo[trkch].pr;
	pTrkDbgData->phase = PVTTrkchInfo[trkch].phase;
	for(i=0; i<RtkEpoch.nCommonSatAMB; i++)
	{
		if(svid == RtkEpoch.CommonSat_Amb[i].SatID)
		{
			pTrkDbgData->N = RtkEpoch.dAMB_Fix[i];
			break;
		}
	}

#if SUPPORT_GLONASS
	if(SV_IsBd2(svid) || SV_IsGps(svid) || SV_IsGlo(svid))
#else
	if(SV_IsBd2(svid) || SV_IsGps(svid))
#endif
	{
		pTrkDbgData->el = SVInfo[svid-1].el;
		pTrkDbgData->az = SVInfo[svid-1].az;
		pTrkDbgData->ephage = (int16)(SVInfo[svid-1].eph_age/60);
		pTrkDbgData->almage = (int16)(SVInfo[svid-1].alm_age/3600);
#ifndef _SIMULATE
		pTrkDbgData->health = CheckEphHealth(svid);
#else
		pTrkDbgData->health = SVInfo[svid-1].health;
#endif		

#if SUPPORT_GLONASS
		if(SV_IsGlo(svid))
		{
			if(CalcGloSVToGpsTime(svid, Sys3AlmEphUTCInfo.Glo_EphStruct[svid-MinGloFreID].tk, &wn, &tow))
				pTrkDbgData->wn  = wn;
		}
		else	
#endif
			pTrkDbgData->wn  = Sys3AlmEphUTCInfo.GpsBd2_EphStruct[svid-1].wkn;

		if(GetSVSWFreq(svid,PVTTrkchInfo[trkch].freq_point, &swdop) >= SWFREQ_SRC_ALM)
			pTrkDbgData->swfre = (int32)(swdop);

		pTrkDbgData->ephcnt = SVInfo[svid-1].ephparsecnt;	
		pTrkDbgData->acqfailcnt = SVInfo[svid-1].AcqTotalFailCnt;
	}
	else
	{
		pTrkDbgData->el = 100;
		pTrkDbgData->ephage = 25200;
		pTrkDbgData->almage = 25200;
	}
#endif
#endif
	return TRUE;	
}


extern unsigned int g_1PPS_RfCnt;
void GetMiscDebugData(DBG_SYSM_MISC *pMiscDbgData)
{
#ifndef _SIMULATE
	static UINT32 msgid=0;
	UTC_TIME utcTime={0,};
	int32 trkch=0, svid=0;
	WGS wgspos={0.0,0.0,0.0};
	ECEF hispos={0.0,0.0,0.0}, rcvrpos={0.0,0.0,0.0},rcvrvel={0.0,0.0,0.0};
	double tcxo=0.0, clkerr=0.0, alt=0.0;
	SYNC_NAVSYS_TIME syncTime_gps,syncTime_bd;
#if SUPPORT_GLONASS
	SYNC_NAVSYS_TIME syncTime_glo;
#endif
	int8 bECAUsed = 0;
	byte navmode = GetCurNavMode();
	int32 frqdotidx=0;
	word16 frqdot=0, inViewNotUsedSVFdot=0, SVFPMap = 0;
	static word16 svFredotUsed[SV_NUM];
	BD2_Attitude attinfo={0,};
	float64 tmp=0.0;

	memset(svFredotUsed,0,sizeof(svFredotUsed));
	memset(pMiscDbgData,0,sizeof(DBG_SYSM_MISC));

	pMiscDbgData->msgid = msgid++;

	GetTimeOfFixUTC(&utcTime, 0.005);

	sprintf(pMiscDbgData->UTCTime, "%04u%02u%02u%02u%02u%02u"
		, utcTime.year
		, utcTime.mon
		, utcTime.day
		, utcTime.hour
		, utcTime.min
		, utcTime.sec
		);

	pMiscDbgData->TTFF = (int32)(TTFF*0.01);	//0.1s
	pMiscDbgData->sysEnv = SysEnvSta + (10 * SysEnvCon);
	pMiscDbgData->lsckres = bLsFixCoarseChkOK*10+lsPosVelChkPath;

	for(trkch=0; trkch<MAXCHANNELS; trkch++)
	{
		if((PVTTrkchInfo[trkch].CCBF & 0x3)==0x3)
		{
			if(SV_IsGps(PVTTrkchInfo[trkch].svid))
				pMiscDbgData->gpstrked++;
			else if(SV_IsBd2(PVTTrkchInfo[trkch].svid))
				pMiscDbgData->bdtrked++;
#if SUPPORT_GLONASS
			else if(SV_IsGlo(PVTTrkchInfo[trkch].svid))
				pMiscDbgData->glotrked++;
#endif
		}

		if(glStruAcqu.Info[trkch].State == NO_SIG_PVT)
			pMiscDbgData->idlchcnt++;

		if((PVTTrkchInfo[trkch].svid>0) && (glStruAcqu.Info[trkch].State>=IDLE_PVT))
			svFredotUsed[PVTTrkchInfo[trkch].svid-1]|=PVTTrkchInfo[trkch].freq_point;

		if(GetBitWord256(&(PVTFixInfo.poschmap), trkch))
		{
			if(SV_IsGps(PVTTrkchInfo[trkch].svid))
				pMiscDbgData->gpssvused++;
			else if(SV_IsBd2(PVTTrkchInfo[trkch].svid))
				pMiscDbgData->bdsvused++;
		#if SUPPORT_GLONASS
			else if(SV_IsGlo(PVTTrkchInfo[trkch].svid))
				pMiscDbgData->glosvused++;
		#endif
		}		
	}

#ifndef _SIMULATE
#ifndef _POSTPROC
	for(svid=1; svid<=SV_NUM; svid++)
	{
		if((SVInfo[svid-1].el>GetSVElMask(svid)) && (CheckEphHealth(svid)!=-1)&& (GetBitWord256(&DummySVMap,svid-1)==0))
		{
			SVFPMap = 0;
			if(SV_IsGps(svid) && (navmode & NAV_SYS_GPS))
			{
				pMiscDbgData->gpsinview++;
				SVFPMap = FREQ_GROUP_GPS;
			}
			else if(SV_IsBd2(svid) && (navmode & NAV_SYS_BD))
			{
				pMiscDbgData->bdinview++;
				SVFPMap = FREQ_GROUP_BD;
			}
#if SUPPORT_GLONASS
			else if(SV_IsGlo(svid) && (navmode & NAV_SYS_GLO))
			{
				pMiscDbgData->gloinview++;
				SVFPMap = FREQ_GROUP_GLO;
			}
#endif
			for(frqdotidx=0; frqdotidx<FREQ_CNT; frqdotidx++)
			{
				frqdot = (0x1<<frqdotidx);
				if(((frqdot & pActiveCPT->SysmCptWorkConfig.NavFreqPoint)==0) || ((frqdot & SVFPMap)==0))
					continue;

				if((frqdot & svFredotUsed[svid-1])==0)
					inViewNotUsedSVFdot++;	
			}
		}
	}
#endif
#endif
	if(inViewNotUsedSVFdot>255)
		pMiscDbgData->inviewNUseSVFdot = 255;
	else
		pMiscDbgData->inviewNUseSVFdot = (byte)inViewNotUsedSVFdot;

#ifndef _SIMULATE
#ifndef _POSTPROC
	pMiscDbgData->acqstartcnt = ACQStartCnt;
	pMiscDbgData->acqendcnt = ACQEndCnt;
#endif
#endif
	GetTCXOOffset(&tcxo);
	pMiscDbgData->tcxo = tcxo*GetFreqpointPLL(DSP_L1CA_FRE);

	pMiscDbgData->Lock_RFcnt = TICLock_RFcnt;

	pMiscDbgData->fixres = bPostproOutput*100+PVTFixInfo.posres*10+getRcvrInfo(NULL,NULL,NULL,NULL);
#if (PVT_MODE == CALC_PVT_KF)
	pMiscDbgData->filtertype = PVTFixInfo.filtertype+IsCurPVTKFValid()*10;
#endif
#ifndef _SIMULATE
#ifndef _POSTPROC
	pMiscDbgData->bootmode = mc_startup_flag;
#endif
#endif	
	pMiscDbgData->posx = (int32)(PVTFixInfo.rcvrpos.x*10.0);	//0.1m
	pMiscDbgData->posy = (int32)(PVTFixInfo.rcvrpos.y*10.0);
	pMiscDbgData->posz = (int32)(PVTFixInfo.rcvrpos.z*10.0);

	pMiscDbgData->velx = (int16)(PVTFixInfo.rcvrvel.x*10.0);	//0.1m/s
	pMiscDbgData->vely = (int16)(PVTFixInfo.rcvrvel.y*10.0);
	pMiscDbgData->velz = (int16)(PVTFixInfo.rcvrvel.z*10.0);

	if(PVTFixInfo.posres != FIX_NOT)
		ECEF2WGS(&(PVTFixInfo.rcvrpos),&wgspos);
	pMiscDbgData->alt = (int16)(wgspos.alt*10.0);	//0.1m

	pMiscDbgData->gdop = (int32)(PVTFixInfo.dop.gdop*100.0);
	pMiscDbgData->hdop = (int32)(PVTFixInfo.dop.hdop*100.0);
	pMiscDbgData->vdop = (int32)(PVTFixInfo.dop.vdop*100.0);
	pMiscDbgData->tdop_gps = (int32)(PVTFixInfo.dop.gpstdop*100.0);
	pMiscDbgData->tdop_bd = (int32)(PVTFixInfo.dop.bdtdop*100.0);
#if SUPPORT_GLONASS
	pMiscDbgData->tdop_glo = (int32)(PVTFixInfo.dop.glotdop*100.0);
#endif

	if(getRcvrInfo(&rcvrpos,&rcvrvel,NULL,NULL)!= FIX_NOT)
	{
		pMiscDbgData->testPosErr = (word32)(GetTestPosErr(&rcvrpos)*10000.0);	//1e-4m
		pMiscDbgData->testVelErr = (word32)(GetTestVelErr(&rcvrvel)*100.0);		//0.01m/s
	}

	pMiscDbgData->buflag = PVTFixInfo.buflag;
	pMiscDbgData->ctu = (int32)(PVTFixInfo.ctu);

	pMiscDbgData->TFTCnt = TFT_OP_CNT;
	pMiscDbgData->Obscnt=gobsTFTIdx;

	pMiscDbgData->nFixFlag = RtkEpoch.nAmbIndex;

	tmp = RtkEpoch.dRatio*10+0.5;
	if(tmp>=255)
		pMiscDbgData->dRatio = 255;
	else
		pMiscDbgData->dRatio = (byte)tmp;
	
	pMiscDbgData->keySvid_gps = RtkEpoch.KeySatId[0];
	pMiscDbgData->keySvid_bd = RtkEpoch.KeySatId[1];
#if SUPPORT_GLONASS
	pMiscDbgData->keySvid_glo = RtkEpoch.KeySatId[2];
#endif
	pMiscDbgData->commSvcnt = RtkEpoch.nCommonSat;
	pMiscDbgData->RTKErrPath = gRTKErrPath + RtkEpochKeep.RTKErrPath;
	pMiscDbgData->RTKExtTimegps = (I1)(GetRTKExtraTime(NAV_SYS_GPS));

#if 1
	if(getRcvrAttInfo(&attinfo)>=FIX_WIDE_INT)
	{
		pMiscDbgData->att_yaw = (word16)(attinfo.yaw_deg*100.0);	//0.01deg
		pMiscDbgData->att_pitch = (int16)(attinfo.pitch_deg*100.0);	//0.01deg
	}
#else
	pMiscDbgData->att_yaw = (word16)(RtkEpoch.Attitude.yaw_deg*100.0);	//0.01deg
	pMiscDbgData->att_pitch = (int16)(attinfo.pitch_deg*100.0);	//0.01deg
#endif
	
	pMiscDbgData->baseline = (word32)(RtkEpoch.Baseline.BaselineLongth*10000.0);	//1e-4m
	pMiscDbgData->baseline_E = (int32)(RtkEpoch.Baseline.East *10000.0);//1e-4m
	pMiscDbgData->baseline_N = (int32)(RtkEpoch.Baseline.North *10000.0);//1e-4m
	pMiscDbgData->baseline_U = (int32)(RtkEpoch.Baseline.Up *10000.0);//1e-4m

	pMiscDbgData->RTKposx = RtkEpoch.RovePVT_Fix.x;
	pMiscDbgData->RTKposy = RtkEpoch.RovePVT_Fix.y;
	pMiscDbgData->RTKposz = RtkEpoch.RovePVT_Fix.z;

	pMiscDbgData->lostbyPrErr = lostByPRErrCnt;
	pMiscDbgData->lostbyoutview = lostByOutViewCnt;
	pMiscDbgData->lostbyCrossErr = lostByCrossErrCnt;
	pMiscDbgData->lostbySnr = lostBySNR;
	pMiscDbgData->lostbyDifFre = lostByDifFre;
	pMiscDbgData->lostbyDupliSV = lostByDupliSV;
	pMiscDbgData->lostbySyncChk = lostBySyncChk;
	pMiscDbgData->lostbyLongTimeSync = lostByLongTimeSync;
	pMiscDbgData->lostByDummySV = lostByDummySV;
	pMiscDbgData->lostByIllSV = lostByIllSV;
	pMiscDbgData->lostByCaif = (int8)lostByCaif;

	pMiscDbgData->TFTModifyCnt = gTFTModifiedCnt;

	if(GetBitWord256(&PVTFixInfo.poschmap,ECA_TRKCH_ID))
		bECAUsed += 1;

	if(GetBitWord256(&PVTFixInfo.velchmap,ECA_TRKCH_ID))
		bECAUsed += 10;
	
	pMiscDbgData->ECAFlag = PVTTrkchInfo[ECA_TRKCH_ID].bPRValid*100+bECAUsed;

	if(getCurSecurityAltitude(&alt))
		pMiscDbgData->longTimeAlt = (int16)alt;
#ifndef _POSTPROC
	pMiscDbgData->acqMismatchCnt = ACQDisMatchCnt[0]+ACQDisMatchCnt[1]+ACQDisMatchCnt[2];
	pMiscDbgData->flashOpCnt = FlashOPCnt;
#endif
	pMiscDbgData->hisposflag = getRcvrHistoricalPos(&hispos, NULL);
	pMiscDbgData->hisposx = (int32)(hispos.x*10.0);
	pMiscDbgData->hisposy = (int32)(hispos.y*10.0);
	pMiscDbgData->hisposz = (int32)(hispos.z*10.0);

	if(getCurFixGPSTime(NULL,NULL,NULL,NULL,&clkerr)>0)
		pMiscDbgData->clkerr_gps = (int32)(clkerr*1.0e10);	//unit : 0.1ns
	if(getCurFixBDTime(NULL,NULL,NULL,NULL,&clkerr)>0)
		pMiscDbgData->clkerr_bd = (int32)(clkerr*1.0e10);	//unit : 0.1ns
#if SUPPORT_GLONASS
	if(getCurFixGloTime(NULL,NULL,NULL,NULL,&clkerr)>0)
		pMiscDbgData->clkerr_glo = (int32)(clkerr*1.0e10);	//unit : 0.1ns
#endif

	pMiscDbgData->RTKExtTimebd = (I1)(GetRTKExtraTime(NAV_SYS_BD));
	pMiscDbgData->RTKExtTimeglo = (I1)(GetRTKExtraTime(NAV_SYS_GLO));

#ifndef _SIMULATE
#ifndef _POSTPROC
	pMiscDbgData->dummy_gps = DummySVMap.wd[0];
	pMiscDbgData->dummy_bd = DummySVMap.wd[1];
#endif
#endif


	GetSyncNavSysTime(NAV_SYS_GPS, &syncTime_gps);
	GetSyncNavSysTime(NAV_SYS_BD, &syncTime_bd);
#if SUPPORT_GLONASS
	GetSyncNavSysTime(NAV_SYS_GLO, &syncTime_glo);
#endif
	if(syncTime_gps.syncsrc > GPSTIME_SYNC_SRC_NONE)
	{
		pMiscDbgData->SyncGPS_wn = syncTime_gps.wn;
		pMiscDbgData->SyncGPS_tow = syncTime_gps.tow;
		pMiscDbgData->SyncGPS_rfcnt = syncTime_gps.rfcnt;
	}
	if(syncTime_bd.syncsrc > GPSTIME_SYNC_SRC_NONE)
	{
		pMiscDbgData->SyncBD_wn = syncTime_bd.wn;
		pMiscDbgData->SyncBD_tow = syncTime_bd.tow;
		pMiscDbgData->SyncBD_rfcnt = syncTime_bd.rfcnt;
	}
#if SUPPORT_GLONASS
	if(syncTime_glo.syncsrc > GPSTIME_SYNC_SRC_NONE)
	{
		pMiscDbgData->SyncGlo_wn = syncTime_glo.wn;
		pMiscDbgData->SyncGlo_tow = syncTime_glo.tow;
		pMiscDbgData->SyncGlo_rfcnt = syncTime_glo.rfcnt;
	}
	pMiscDbgData->synctimesrc = syncTime_gps.syncsrc + syncTime_bd.syncsrc * 10 + syncTime_glo.syncsrc*100;
#else
	pMiscDbgData->synctimesrc = syncTime_gps.syncsrc + syncTime_bd.syncsrc * 10;
#endif

#endif

	return;
}


