/*
 * rtk.c
 *
 *  Created on: 2015-10-21
 *      Author: dell
 */
#include "typedefine.h"
#include "define.h"
#include "constdef.h"
#include "coordinate.h"
#include "Rtk.h"
#include "RTKConstants.h"
#include "RTKLAMBDA.h"
#include "pvt.h"
#include "timeproc.h"
#include "cfgpara.h"
#include "rtcm3.h"
#include "RTKCalBaseline.h"
#include "CalcSvPVT.h"
#include "LsPVT.h"
#include "PVTRcvrInfo.h"
#include "postproc.h"
#include <math.h>
#include "RTDKF.h"
#include "LSPvt.h"

#ifndef _POSTPROC
#ifdef _SIMULATE
#include "dataprocess.h"
#else 
#include "GnssTYProcfg.h"
#endif
#endif


#define PHi_RANGE 0
#define TSBOUNDARY 0


OBSEV Master_observ;
OBSEV_BUF Slave_Buf;

#ifdef _POSTPROC
OBSEV_BUF Master_Buf[MASTER_SITE_MAXNUM+1];
#endif

OBSEV Master_obs;
OBSEV Slave_obs;
static double RTK_Extra_time[MAX_SYSM] = {-1,-1,-1};


ANTPositionStruct antfixposition={FALSE, 0, 0,{0.0,0.0,0.0}};	//This for base station, Base station info
ANTPositionStruct Basefixposition={FALSE, 0,0,{0.0,0.0,0.0}};	//This is for rove station. record the base stion info
static ANTPositionStruct LastBasefixpos={FALSE, 0,0,{0.0,0.0,0.0}};	//This is for rove station. record the Last base stion info
static bool bMoveBase = FALSE;
static LAST_PHI_INFO LastPhi[MAXCHANNELS];
#if PHi_RANGE
LAST_RANGE_INFO Last_range_info[SV_NUM_TRUE][MAX_FREPIONT_PER_NAVSYS];
#endif

// 历元全局变量
RTK_EPOCH RtkEpoch = {0};
RTK_EPOCH_Keep RtkEpochKeep = {0};

int16 gRTKErrPath=0;

static void ChnoInfoToMaster(PVT_TRKCH_INFO* pPVTTrkchInfo, OBSEV* Master_observ);
static bool GetSlaveObsFromRTCM(double masterTow, OBSEV* pSlave, float64* pDifTow);
static void LoadBaseSationInfoByCPTForBase(void);
static void CheckMoveBaseForRove(void);
#ifndef _POSTPROC
static void CalcBaseStationSVPos(OBSEV* PrmSlave);
#endif

bool CheckCurSlaveObs(OBSEV* pCurSlave,OBSEV* pLastSlave);

#if PHi_RANGE
static void Phase2range(OBSEV* Master_observ, int32 svcnt, double* rangeOut,int32 freqidx,double wavlen);
#endif

void RtkInit(void)
{
	memset(&Master_observ,0,sizeof(OBSEV));
	memset(&Slave_Buf,0,sizeof(OBSEV_BUF));
	memset(LastPhi,0,sizeof(LastPhi));
	memset(&Master_obs,0,sizeof(OBSEV));
	memset(&Slave_obs,0,sizeof(OBSEV));
#if PHi_RANGE
	memset(&Last_range_info,0, sizeof(Last_range_info));
#endif

	memset(&RtkEpoch,0,sizeof(RtkEpoch));
	memset(&RtkEpochKeep,0,sizeof(RtkEpochKeep));
	RtkEpochKeep.bFirstCalAmb = TRUE;

	antfixposition.bValid = FALSE;
	Basefixposition.bValid = FALSE;

#if ENABLE_RTD_KF
	InitRTDKFModule();
#endif

	return;
}

void RestRTKchInfo(int32 trkch)
{
	memset(&(LastPhi[trkch]),0,sizeof(LastPhi[trkch]));
#if PHi_RANGE
	memset(&(Last_range_info[glStruAcqu.Info[trkch].SvID-1][0]),0,MAX_FREPIONT_PER_NAVSYS * sizeof(LAST_RANGE_INFO));//改成按通道管理
#endif

	return;
}

void TaskRTK(void)
{
	//int32 reckon_time=0;
	double dTimeDiff = 0.0, masterTow=0.0, slaveTow=0.0, time_RTKStart=0.0;
	ECEF singlePos = {0.0,};
	int32 singlefixres = FIX_NOT;
	int8 rtkfixres = FIX_NOT; 
	//bool flag_RTK_Extra = FALSE;

	//initial
	memset(&Master_obs,0,sizeof(OBSEV));

	gRTKErrPath=0;

	LoadBaseSationInfoByCPTForBase();

	//get maseter obs from PVT
	ChnoInfoToMaster(PVTTrkchInfo, &Master_observ);
	memcpy(&Master_obs,&Master_observ,sizeof(OBSEV));

	if(IsBaseSation())	//Base sation, or RTD is closed
	{
		LastBasefixpos.bValid = FALSE;	//clear the base station management for rove
		Basefixposition.bValid = FALSE;		//clear the base station management for rove
		memset(&Slave_Buf,0,sizeof(OBSEV_BUF));
		memset(&Slave_obs,0,sizeof(OBSEV));
		return;
	}

	//move station
	if(!IsRTDOpen())	//Base sation, or RTD is closed
	{
		gRTKErrPath=1;	
		memset(&Slave_Buf,0,sizeof(OBSEV_BUF));
		memset(&Slave_obs,0,sizeof(OBSEV));
		return;
	}

	//get slave obs from RTCM3
	time_RTKStart = getSysOnTime();
	GetObsvTime(&Master_obs, &masterTow, NAV_SYS_NONE);

	while(1)
	{
#ifdef _SIMULATE
		if(!ReadRTKData())
			break;

		if(GetSlaveObsFromRTCM(masterTow, &Slave_obs, &dTimeDiff))
			break;

		if(dTimeDiff>0.0)
		{
			ReSeekRTCMFile();
			break;
		}
#else

		dTimeDiff = getSysOnTime() - time_RTKStart;
		//if(IsMSMMsgRXFinished() || (dTimeDiff > pActiveCPT->RTKTimeOut))	
		if((dTimeDiff * 1000) > pActiveCPT->SysmCptWorkConfig.RTCMDelay)	// wait N ms at most
		{
			GetSlaveObsFromRTCM(masterTow, &Slave_obs, NULL);
			break;
		}
#endif
	}
	
	if(GetObsvTime(&Slave_obs, &slaveTow, NAV_SYS_NONE))
	{
		dTimeDiff = f_abs(slaveTow-masterTow);
		RTK_Extra_time[0] = dTimeDiff;
		if(dTimeDiff < pActiveCPT->SysmCptWorkConfig.RTKTimeOut)
		{
			rtkfixres = Rel_Position(&Master_obs, &Slave_obs, &RtkEpoch, &RtkEpochKeep);
		}
		else
		{
			memset(&Slave_obs,0,sizeof(OBSEV));
			gRTKErrPath=3;
		}
	}
	else
	{
		gRTKErrPath=4;
	}

	if((rtkfixres == FIX_NOT) && (!bNeed2CalcSinglePos))
	{
		singlefixres = LSCalcRcvrPos_obs(&Master_obs, &singlePos);
		setRcvrFixInfo(&singlePos, NULL, NULL, NULL, singlefixres);
		//test
		gRTKErrPath = 20;
	}

	return;
}


bool CheckInitialRTK(int32 trkch, PVT_TRKCH_INFO* pPVTTrkchInfo)
{
	bool trkmodeTrue = FALSE;
	int32 cn0 = pPVTTrkchInfo->cn1s;
	int32 minsnrth=pActiveCPT->SysmCptWorkConfig.SnrMaskRTK;
	
	if(pPVTTrkchInfo->C2FAI < 40)
		return FALSE;

	if(pPVTTrkchInfo->el < GetSVRTKElMask(pPVTTrkchInfo->svid))
		return FALSE;

#if SUPPORT_GLONASS
	if(SV_IsBd2Geo(pPVTTrkchInfo->svid) || SV_IsGlo(pPVTTrkchInfo->svid))//GEO, GLONASS
#else
	if(SV_IsBd2Geo(pPVTTrkchInfo->svid))
#endif
	{
		cn0 -= 2; 
		if(pPVTTrkchInfo->trkmode==TRK_STATE_7)
			trkmodeTrue= TRUE;
	}
	else if((pPVTTrkchInfo->freq_point & DSP_L2P_FRE) == DSP_L2P_FRE)//L2P
	{
		if((pPVTTrkchInfo->trkmode==TRK_STATE_13)||(pPVTTrkchInfo->trkmode==TRK_STATE_14))
			trkmodeTrue = TRUE;

		minsnrth -= L2P_LOWER_CN0_TH;
	}
	else //MEO OR L1
	{
		if(pPVTTrkchInfo->trkmode==TRK_STATE_10)
			trkmodeTrue = TRUE;
	}

	if(trkmodeTrue && (pPVTTrkchInfo->reverseValid != 0) && (cn0>=minsnrth))
	{
		if((cn0 >= 40)&&(pPVTTrkchInfo->steadyLockedtime> (2*10)))//40
			return TRUE;
		else if((cn0 >= 37) && (pPVTTrkchInfo->steadyLockedtime > (5*10)))//37
			return TRUE;
		else if((cn0 >= 35) && (pPVTTrkchInfo->steadyLockedtime > (15*10)))//35
			return TRUE;
		else if((cn0 >= 32) && (pPVTTrkchInfo->steadyLockedtime > (30*10)))//35
			return TRUE;
		else if ((cn0 >= 30) && (pPVTTrkchInfo->steadyLockedtime > (45*10))) //30
			return TRUE;
		else if(pPVTTrkchInfo->steadyLockedtime > (55*10))
			return TRUE;
		else
			return FALSE;
	}
	else 
		return FALSE;
}


//get base station observation
void ChnoInfoToMaster(PVT_TRKCH_INFO* pPVTTrkchInfo, OBSEV* Master_observ)
{
	static double  rtkbaseRFcnt=0.0;
	static double  rtkbaseTr=0.0;
	int32 trkch=0,freqidx=0, svid=0, svcnt=0, gpscnt=0, bdcnt=0, glocnt=0,i, trkch_l1, wn=0, bSVPosReCalcTrkch=-1;
	PVT_TRKCH_INFO* pCurChInfo=NULL;
	double tr=0.0, trboundary=0.0, timegap=0.0, deltaPhi=0.0, wavlen=0.0, phi=0.0, difphi=0.0, curIF=0.0, deltT=0.0, tsboundary=0.0;
	double tr_gps=0.0;
	double *pTrBoundry=NULL;
	double curLockRFcnt = getTICLockRFcnt();
	bool bSVObsValid = FALSE;
	word64_rtc currfcnt = GetCurRFSampleCntWord64();
	FIX_DOP curdop;
	//double tftModifyTime = 0.0;
	double curTrUsed=0.0, tr_clkerr;
	word256 signalChUseMap={{0,}};
#if PHi_RANGE
	double rangeOut=0.0;
#endif
	bool bShift2Bound= FALSE;
	
	memset(Master_observ,0,sizeof(OBSEV));

	getRcvrInfo(NULL,NULL,&signalChUseMap,&curdop);
	if(TTFF == 0)
		return;
	
	Master_observ->staid = pActiveCPT->SysmCptWorkConfig.BaseStaID;
	Master_observ->bitmap |= OBSEV_VALID_STATION;
	
	if(!GetNavSysTimeByRFCnt(NAV_SYS_NONE,GPSTIME_SYNC_SRC_FIXRFCNT,curLockRFcnt,&tr_gps,&wn))
		return;

	Master_observ->wn = wn;	//?

	timegap = pActiveCPT->SysmCptWorkConfig.FixUpdateCycle*0.01;
	for(i=1; i<=SV_NUM_TRUE; i++)	//must follow the svid from little to big
	{
		svid = i;
#if SUPPORT_GLONASS
		if(i>=MinGloFreID && i<= MaxGloSvIDTRUE)	//for glonass, this means slot id.
		{
			svid = ConvertGloSVID2SlotID(i);
			if(!SV_IsGlo(svid))
				continue;
		}
#endif
			
		if(SVInfo[svid-1].el<GetSVElMask(svid))
			continue;

		tr = tr_gps;
		if(SV_IsGps(svid))
			pTrBoundry = &(Master_observ->sec_gps);
		else if(SV_IsBd2(svid))
			pTrBoundry = &(Master_observ->sec_bd);
#if SUPPORT_GLONASS
		else if(SV_IsGlo(svid))
			pTrBoundry = &(Master_observ->sec_glo);
#endif
		else
			continue;
		
		bShift2Bound = FALSE;
		if(IsBaseSation()||(TSBOUNDARY==1))	
		{
			trboundary = ((int32)(tr/timegap+0.5))*timegap;
			if(f_abs(trboundary - tr)>0.005)
				trboundary = tr;
			else
				bShift2Bound = TRUE;
		}
		else
			trboundary = tr;

		*pTrBoundry = trboundary; 

		if(rtkbaseRFcnt<MICRO_NUM)
		{
			rtkbaseRFcnt = curLockRFcnt;
			rtkbaseTr = tr;
			curTrUsed = tr;
		}
		else
		{
			curTrUsed = rtkbaseTr + (curLockRFcnt-rtkbaseRFcnt)*RECIP_RF_SAMP_FREQ;
			if(curTrUsed >= SECONDS_IN_WEEK)
			{
				curTrUsed -= SECONDS_IN_WEEK;
				
				rtkbaseRFcnt = curLockRFcnt;
				rtkbaseTr = curTrUsed;
			}
		}

		bSVObsValid = FALSE;
		bSVPosReCalcTrkch = -1;
		for(trkch=0;trkch<MAXCHANNELS;trkch++)
		{
			pCurChInfo = &(pPVTTrkchInfo[trkch]);

			if(svid != pCurChInfo->svid)
				continue;
			
			if((!pCurChInfo->bPRValid) || (!CheckInitialRTK(trkch,pCurChInfo)))
			{	
				LastPhi[trkch].bValid = FALSE;
				continue;
			}

			freqidx = GetFreGropIdx(pCurChInfo->freq_point);
			
			Master_observ->obs[svcnt].StarID = i;	//true svid, for glonass this mean true SVID, GPS:1~32, BD: 33~64; GLO: 65~88
			Master_observ->obs[svcnt].slotID = svid;	//slot id. GPS:1~32, BD: 33~64; GLO: 65~78
			Master_observ->obs[svcnt].el = pCurChInfo->el;
			Master_observ->obs[svcnt].az = pCurChInfo->az;
			//recalculate sv pos and sv clkerr	
#ifndef _SIMULATE
			if(bShift2Bound)
#endif
			{
				if(bSVPosReCalcTrkch<0)
				{
					if(SV_IsGps(svid)||SV_IsBd2(svid))
					{
						tsboundary = pCurChInfo->ts - (tr - trboundary) + pCurChInfo->clkerr;
						if(tsboundary<0.0)
							tsboundary = tsboundary + SECONDS_IN_WEEK;
						if(tsboundary > SECONDS_IN_WEEK)
							tsboundary = tsboundary - SECONDS_IN_WEEK;

						SWI_disable();
						CalcSV_PVT_Eph(svid, tsboundary, &(pCurChInfo->svpos), &(pCurChInfo->svvel), &(pCurChInfo->clkerr), &(pCurChInfo->frqerr),&(pCurChInfo->toertk));

						SWI_enable();
						
						bSVPosReCalcTrkch = trkch;
					}
				}
				else
				{
					memcpy(&(pCurChInfo->svpos), &(pPVTTrkchInfo[bSVPosReCalcTrkch].svpos), sizeof(pCurChInfo->svpos));
					pCurChInfo->clkerr = pPVTTrkchInfo[bSVPosReCalcTrkch].clkerr;
					pCurChInfo->toertk= pPVTTrkchInfo[bSVPosReCalcTrkch].toertk;
				}
			}
		
			memcpy(&(Master_observ->obs[svcnt].svpos), &(pCurChInfo->svpos), sizeof(pCurChInfo->svpos));
			Master_observ->obs[svcnt].clkerr = pCurChInfo->clkerr;
			Master_observ->obs[svcnt].toe= pCurChInfo->toertk;
			
			if(GetBitWord256(&signalChUseMap, trkch)==1)	//signal position channel used map
				Master_observ->obs[svcnt].bUsedMap[freqidx] = 1;
			else
				Master_observ->obs[svcnt].bUsedMap[freqidx] = 0;
			Master_observ->obs[svcnt].code[freqidx] = pCurChInfo->freq_point;
			Master_observ->obs[svcnt].doppler[freqidx] = pCurChInfo->fd;
			if(pCurChInfo->freq_point == DSP_L2P_FRE)
			{
				//Master_observ->obs[svcnt].snr0[freqidx] = pCurChInfo->cn1s+7;
				trkch_l1 = trkch-L2P_MAXCHANNELS;
			}
			//else	
				Master_observ->obs[svcnt].snr0[freqidx] = pCurChInfo->cn1s;

			//calc PR
			wavlen = GetSVWaveLen(svid,pCurChInfo->freq_point);
			deltaPhi = pCurChInfo->fd * (trboundary - tr);	
			
			deltT = (curTrUsed - (pCurChInfo->ts+pCurChInfo->clkerr));
			if(deltT<(-1*SECONDS_HALF_WEEK))
				deltT += SECONDS_IN_WEEK;
			else if(deltT>SECONDS_HALF_WEEK)
				deltT -= SECONDS_IN_WEEK;

			Master_observ->obs[svcnt].range[freqidx] = deltT*SPEED_OF_LIGHT;		

			//calc phase
			phi = pCurChInfo->carrInt + ((double)pCurChInfo->carrDec)*RECIP_P65536;

			//reinital
			if(pCurChInfo->steadyLockedtime>(3*SECONDS_IN_DAY*10))	//continue trk >3 days
				LastPhi[trkch].bValid = FALSE;
			
			if(LastPhi[trkch].bValid) //be initialled
			{
				curIF = GetFreqpointIF(svid, pCurChInfo->freq_point);
				deltT = curTrUsed - LastPhi[trkch].tr_withBias;
				if(deltT<(-1*SECONDS_HALF_WEEK))
					deltT += SECONDS_IN_WEEK;

				difphi = curIF * deltT - (phi - LastPhi[trkch].PhiTrue) - (pCurChInfo->carrIntInvCnt - LastPhi[trkch].PhiTrue_inv)*TWO_P24;
			
				Master_observ->obs[svcnt].phase[freqidx] = LastPhi[trkch].PhiUsed + difphi;
				difphi = Master_observ->obs[svcnt].phase[freqidx]*wavlen - Master_observ->obs[svcnt].range[freqidx];
				if(f_abs(difphi)>(262.0))
					LastPhi[trkch].bValid = FALSE;
			}

			if(!LastPhi[trkch].bValid)	//initial
			{	
				Master_observ->obs[svcnt].phase[freqidx] = (double)((int32)(Master_observ->obs[svcnt].range[freqidx]/wavlen))-((double)pCurChInfo->carrDec)*RECIP_P65536;
				LastPhi[trkch].PhiUsed = Master_observ->obs[svcnt].phase[freqidx];
				LastPhi[trkch].PhiTrue_inv = pCurChInfo->carrIntInvCnt;
				LastPhi[trkch].PhiTrue = phi;
				LastPhi[trkch].tr_withBias = curTrUsed;
				LastPhi[trkch].bValid = TRUE;
			}

			//modify the local Clkerr
			deltT = curTrUsed - tr;
			if(deltT<(-1*SECONDS_HALF_WEEK))
				deltT += SECONDS_IN_WEEK;
			else if(deltT>SECONDS_HALF_WEEK)
				deltT -= SECONDS_IN_WEEK;
			tr_clkerr = deltT*SPEED_OF_LIGHT;
#if PHi_RANGE
			Phase2range(Master_observ,svcnt,&rangeOut,freqidx,wavlen);		
			Master_observ->obs[svcnt].range[freqidx] = rangeOut - tr_clkerr - deltaPhi*wavlen;
#else
			Master_observ->obs[svcnt].range[freqidx] = Master_observ->obs[svcnt].range[freqidx] - tr_clkerr - deltaPhi*wavlen;
#endif
			Master_observ->obs[svcnt].phase[freqidx] = Master_observ->obs[svcnt].phase[freqidx] - (tr_clkerr/wavlen) - deltaPhi;		

			//lock time
			Master_observ->obs[svcnt].LLI[freqidx] = (word32)(pCurChInfo->steadyLockedtime*100);

			//half cycle
			if(((pCurChInfo->reverse == REVERSE) && (pCurChInfo->freq_point != DSP_L2P_FRE)) 
				|| ((pCurChInfo->freq_point == DSP_L2P_FRE) && pPVTTrkchInfo[trkch_l1].bPRValid && (pPVTTrkchInfo[trkch_l1].reverse == REVERSE)))
			{
				Master_observ->obs[svcnt].halfcycle[freqidx] = 0;
				Master_observ->obs[svcnt].phase[freqidx]+=0.5;
			}
			else
				Master_observ->obs[svcnt].halfcycle[freqidx] = 0;

			//valid map
			Master_observ->obs[svcnt].validmap[freqidx] = OBST_VALID_LLI|OBST_VALID_SNR|OBST_VALID_CODE|OBST_VALID_PHASE|OBST_VALID_RANGE|OBST_VALID_DOPPLER|OBST_VALID_HALFCYCLE;

			//debug info			
			pCurChInfo->phase = Master_observ->obs[svcnt].phase[freqidx];
			pCurChInfo->pr = Master_observ->obs[svcnt].range[freqidx];
//#ifdef _SIMULATE
//			Master_observ->obs[svcnt].range[freqidx] = Debug_TrkInfo[trkch].pr;
//			Master_observ->obs[svcnt].phase[freqidx] = Debug_TrkInfo[trkch].phase;
//#endif
			
			bSVObsValid = TRUE;

			if(SV_IsGps(svid)) Master_observ->signmax_gps|=(1<<ConvertFP2RTCM3SigMaskidx(pCurChInfo->freq_point));
			if(SV_IsBd2(svid)) Master_observ->signmax_bd|=(1<<ConvertFP2RTCM3SigMaskidx(pCurChInfo->freq_point));
#if SUPPORT_GLONASS
			if(SV_IsGlo(svid)) Master_observ->signmax_glo|=(1<<ConvertFP2RTCM3SigMaskidx(pCurChInfo->freq_point));
#endif
		}

		if(bSVObsValid)
		{
			if(SV_IsGps(svid)) 
			{
				Master_observ->bitmap |= OBSEV_VALID_GPS_DATA;
				gpscnt++;
			}
			else if(SV_IsBd2(svid)) 
			{
				Master_observ->bitmap |= OBSEV_VALID_BD_DATA;
				bdcnt++;
			}
#if SUPPORT_GLONASS
			else if(SV_IsGlo(svid)) 
			{
				Master_observ->bitmap |= OBSEV_VALID_GLO_DATA;
				glocnt++;
			}
#endif

			svcnt++;
			if(svcnt>=MAX_RTK_OBS_SVCNT)
				break;
		}
		
	}

	Master_observ->satnmax = svcnt;

	//if(fixres<FIX_3D)
	//	Master_observ->clkSteer_gps=Master_observ->clkSteer_bd=Master_observ->clkSteer_glo=2;	//unkown
	//else
	{
		if((gpscnt>=5) && (curdop.gpstdop<2.0))	Master_observ->clkSteer_gps = 1;
		else if((gpscnt>=2) && (curdop.gpstdop<10.0)) Master_observ->clkSteer_gps = 0;	
		else Master_observ->clkSteer_gps = 2;
			
		if((bdcnt>=5) && (curdop.bdtdop<2.0)) Master_observ->clkSteer_bd = 1;
		else if((bdcnt>=2) && (curdop.bdtdop<10.0)) Master_observ->clkSteer_bd = 0;
		else Master_observ->clkSteer_bd = 2;

#if SUPPORT_GLONASS
		if((glocnt>=5) && (curdop.glotdop<2.0)) Master_observ->clkSteer_glo = 1;
		else if((glocnt>=2) && (curdop.glotdop<10.0)) Master_observ->clkSteer_glo = 0;
		else Master_observ->clkSteer_glo = 2;
#endif
	}

	return;
}


#if PHi_RANGE
void Phase2range(OBSEV* Master_observ, int32 svcnt, double* rangeOut,int32 freqidx,double wavlen)
{
	int32 Mbest = 50;
	double W = 0.0, tmp1 = 0.0, tmp2 = 0.0;
	int32 svid = Master_observ->obs[svcnt].StarID;

	//if(slip>0)
	//	Last_range_info[svid-1][freqidx].icnt = 0;
	
	if(Last_range_info[svid-1][freqidx].icnt == 0)
	{
		(*rangeOut) = Master_observ->obs[svcnt].range[freqidx];
		Last_range_info[svid-1][freqidx].lastrange = Master_observ->obs[svcnt].range[freqidx];
		Last_range_info[svid-1][freqidx].lastphase = Master_observ->obs[svcnt].phase[freqidx];
		Last_range_info[svid-1][freqidx].icnt = 1;
		return;
	}

	if(Last_range_info[svid-1][freqidx].icnt < Mbest)
	{
		Last_range_info[svid-1][freqidx].icnt++;
		W = 1.0/(Last_range_info[svid-1][freqidx].icnt);
	}
	else
	{
		W = 1.0/Mbest;
	}

	tmp1 = Master_observ->obs[svcnt].range[freqidx] ;
	tmp2 = Last_range_info[svid-1][freqidx].lastrange + wavlen *(Master_observ->obs[svcnt].phase[freqidx] - Last_range_info[svid-1][freqidx].lastphase);
	(*rangeOut) = W * tmp1 + (1.0 -W) * tmp2;

	Last_range_info[svid-1][freqidx].lastrange = (*rangeOut);
	Last_range_info[svid-1][freqidx].lastphase = Master_observ->obs[svcnt].phase[freqidx];
	
}
#endif


/**************ROVE Station: base station info management start********************/
//These functions are only used when this receiver is a rove station
void SetBaseStationInfoForRove(UINT16 stationID, float64 antHeight, ECEF* pPos)
{	
	Basefixposition.bValid = TRUE;
	Basefixposition.stationID = stationID;
	Basefixposition.antHeight = antHeight;
	memcpy(&(Basefixposition.ecefpos), pPos, sizeof(Basefixposition.ecefpos));

	CheckMoveBaseForRove();

	return;
}

void CheckMoveBaseForRove(void)
{
	double difpos=0.0;
	
	bMoveBase = FALSE;
	if(LastBasefixpos.bValid && Basefixposition.bValid)
	{
		difpos += (LastBasefixpos.ecefpos.x - Basefixposition.ecefpos.x)*(LastBasefixpos.ecefpos.x - Basefixposition.ecefpos.x);
		difpos += (LastBasefixpos.ecefpos.y - Basefixposition.ecefpos.y)*(LastBasefixpos.ecefpos.y - Basefixposition.ecefpos.y);
		difpos += (LastBasefixpos.ecefpos.z - Basefixposition.ecefpos.z)*(LastBasefixpos.ecefpos.z - Basefixposition.ecefpos.z);
		if(difpos>0.000001)	//dif pos < 0.001m
			bMoveBase = TRUE;
	}

	if(Basefixposition.bValid)
		memcpy(&LastBasefixpos, &Basefixposition, sizeof(LastBasefixpos));

	return;
}

bool IsMoveBaseForRove(void)
{
	bool ret = bMoveBase;

	return ret;
}


/**************ROVE Station: base station info management end********************/


/**************Base Station: base station info management start********************/
//These functions are only used when this receiver is a Base station
bool GetBaseStationInfoForBase(UINT16* pStationID, float64* pAntHeight, ECEF* pPos)
{
	if(!antfixposition.bValid)
		return FALSE;
	
	if(pStationID != NULL)
		*pStationID = antfixposition.stationID;
	
	if(pAntHeight != NULL)
		*pAntHeight = 0;
	
	if(pPos != NULL)
		memcpy(pPos, &(antfixposition.ecefpos), sizeof(antfixposition.ecefpos));

	return TRUE;
}

void LoadBaseSationInfoByCPTForBase(void)
{	
	ECEF pos={0.0,};
	
	if(!IsBaseSation())
		antfixposition.bValid = FALSE;
	
	if((pActiveCPT->SysmCptWorkConfig.ValidFlag & CPT_VALIDFLAG_BASEPOS) && (GetCPTRcvrMoveState()==RCVR_MOVESTATE_STATIC))	//static base
	{
		antfixposition.stationID = pActiveCPT->SysmCptWorkConfig.BaseStaID;
		antfixposition.antHeight = pActiveCPT->SysmCptWorkConfig.AntHeight*0.001;	//m
		
		antfixposition.ecefpos.x = pActiveCPT->SysmCptWorkConfig.BaseStaPos_x;
		antfixposition.ecefpos.y = pActiveCPT->SysmCptWorkConfig.BaseStaPos_y;
		antfixposition.ecefpos.z = pActiveCPT->SysmCptWorkConfig.BaseStaPos_z;
		antfixposition.bValid = TRUE;
	}
	else if(GetCPTRcvrMoveState()!=RCVR_MOVESTATE_STATIC)	// move base
	{
		if(getRcvrInfo(&pos,NULL,NULL,NULL)>= FIX_3D)	//use single position
		{

			antfixposition.stationID = pActiveCPT->SysmCptWorkConfig.BaseStaID;
			antfixposition.antHeight = pActiveCPT->SysmCptWorkConfig.AntHeight*0.001;	//m
		
			antfixposition.ecefpos.x = pos.x;
			antfixposition.ecefpos.y = pos.y;
			antfixposition.ecefpos.z = pos.z;
			antfixposition.bValid = TRUE;
		}
		else
			antfixposition.bValid = FALSE;
	}
	else
	{
		antfixposition.bValid = FALSE;
	}	

	if(!antfixposition.bValid)
		antfixposition.antHeight = 0.0;

	return;
}

/**************Base Station: base station info management end********************/


OBSEV* GetCurSlaveBufAddr(byte navsys, double tow)
{
	OBSEV* pObsBuf= NULL;
	int8 writeidx = Slave_Buf.writeidx;
	double obstow = 0.0, diftow=0.0;
	bool bCurObsValid = FALSE;
	word16 curbitmap=0,newbitmap=0;
#if SUPPORT_GLONASS
	int32 weekn=0;
	double tmptow = 0.0;
#endif

	if(navsys == NAV_SYS_BD)
	{
		tow += GPS_BD_SYSTIME_OFFSET;
		if(tow>=SECONDS_IN_WEEK)
			tow -= SECONDS_IN_WEEK;
	}
#if SUPPORT_GLONASS
	else if(navsys == NAV_SYS_GLO)	 //tow = dayt 0~84600s
	{
		GloToGpsTime(Global_DayN4GLO, Global_DayNTGLO, tow + Global_TleapsGLO, &weekn,&tmptow);
		tow = tmptow;
	}
#endif

	bCurObsValid = GetObsvTime(&(Slave_Buf.obsev[writeidx]), &obstow, NAV_SYS_NONE);
	
	if(bCurObsValid)
	{
		diftow = f_abs(obstow - tow);
		if(diftow > SECONDS_HALF_WEEK)
			diftow = diftow - SECONDS_IN_WEEK;

		curbitmap = Slave_Buf.obsev[writeidx].bitmap;
		if(navsys==NAV_SYS_GPS)
			newbitmap = OBSEV_VALID_GPS_DATA;
		else if(navsys == NAV_SYS_BD)
			newbitmap = OBSEV_VALID_BD_DATA;
		else if(navsys == NAV_SYS_GLO)
			newbitmap = OBSEV_VALID_GLO_DATA;

		if((f_abs(diftow)>0.005) ||
		 ((f_abs(diftow)<0.005) && ((curbitmap & newbitmap) != 0)))
		{
			Slave_Buf.writeidx++;
			if(Slave_Buf.writeidx >= SLAVE_BUF_LEN)
				Slave_Buf.writeidx -= SLAVE_BUF_LEN;

			writeidx = Slave_Buf.writeidx;

			memset(&(Slave_Buf.obsev[writeidx]), 0, sizeof(OBSEV));
#ifdef _SIMULATE
			FlagOfNextData = TRUE;
#endif
		}
	}
	
	pObsBuf = &(Slave_Buf.obsev[writeidx]);
		
	return pObsBuf;
}

#ifdef _POSTPROC
OBSEV* GetCurMasterBufAddr(byte navsys, double tow, int masterNumber)
{
	OBSEV* pObsBuf= NULL;
	int8 writeidx = Master_Buf[masterNumber].writeidx;
	double obstow = 0.0, diftow=0.0;
	bool bCurObsValid = FALSE;
#if SUPPORT_GLONASS
	int32 weekn=0;
	double tmptow = 0.0;
#endif

	if(navsys == NAV_SYS_BD)
	{
		tow += GPS_BD_SYSTIME_OFFSET;
		if(tow>=SECONDS_IN_WEEK)
			tow -= SECONDS_IN_WEEK;
	}
#if SUPPORT_GLONASS
	else if(navsys == NAV_SYS_GLO)	 //tow = dayt 0~84600s
	{
		GloToGpsTime(Global_DayN4GLO, Global_DayNTGLO, tow + Global_TleapsGLO, &weekn,&tmptow);
		tow = tmptow;
	}
#endif
	bCurObsValid = GetObsvTime(&(Master_Buf[masterNumber].obsev[writeidx]), &obstow, NAV_SYS_NONE);

	if(bCurObsValid)
	{
		diftow = f_abs(obstow - tow);
		if(diftow > SECONDS_HALF_WEEK)
			diftow = diftow - SECONDS_IN_WEEK;

		if(f_abs(diftow)>0.005)
		{
			Master_Buf[masterNumber].writeidx++;
			if(Master_Buf[masterNumber].writeidx >= SLAVE_BUF_LEN)
				Master_Buf[masterNumber].writeidx -= SLAVE_BUF_LEN;

			writeidx = Master_Buf[masterNumber].writeidx;

			memset(&(Master_Buf[masterNumber].obsev[writeidx]), 0, sizeof(OBSEV));
#ifndef _POSTPROC
#ifdef _SIMULATE
			FlagOfNextData = TRUE;
#endif
#endif
		}
	}

	pObsBuf = &(Master_Buf[masterNumber].obsev[writeidx]);



	return pObsBuf;
}
#endif


bool GetSlaveObsFromRTCM(double masterTow, OBSEV* pSlave, float64* pDifTow)
{
	int8 readidx = 0, mindif_readidx=0;
	double minDiftime = 1000000.0, minObsTow=0.0,obstow=0.0, diftow=0.0;
	double lastSlavetow=0.0;
	bool lastSalveValid = FALSE, bNeed2CopySalve=TRUE,flagCurObs = FALSE,newobsflag = FALSE;
	OBSEV tmp_slave_obs;

	lastSalveValid = GetObsvTime(pSlave, &lastSlavetow, NAV_SYS_NONE);

#ifdef _SIMULATE
	readidx = (Slave_Buf.writeidx-1+SLAVE_BUF_LEN)%SLAVE_BUF_LEN;
#else
	for(readidx = 0; readidx<SLAVE_BUF_LEN; readidx++)
#endif
	{
		if(!GetObsvTime(&(Slave_Buf.obsev[readidx]), &obstow, NAV_SYS_NONE))
#ifdef _SIMULATE
			return FALSE;
#else
			continue;
#endif

		diftow = obstow - masterTow;
		if(diftow > SECONDS_HALF_WEEK)
			diftow = diftow - SECONDS_IN_WEEK;
		else if(diftow < (-1*SECONDS_HALF_WEEK))
			diftow = diftow + SECONDS_IN_WEEK;

		if(pDifTow != NULL)
			*pDifTow = diftow;

		diftow = f_abs(diftow);
		if(diftow < minDiftime)
		{
			minDiftime = diftow;
			minObsTow = obstow;
			mindif_readidx = readidx;
			newobsflag = TRUE;
		}
	}

#ifdef _SIMULATE
	if(minDiftime <0.9)	//s
#endif
	{
		if(lastSalveValid)
		{
			diftow = minObsTow - lastSlavetow;
			if(diftow > SECONDS_HALF_WEEK)
				diftow = diftow - SECONDS_IN_WEEK;
			else if(diftow < (-1*SECONDS_HALF_WEEK))
				diftow = diftow + SECONDS_IN_WEEK;

			if((diftow<0.001)&&(Slave_Buf.obsev[mindif_readidx].satnmax == pSlave->satnmax))	//same with last, do nothing
				bNeed2CopySalve=FALSE;	
		}

		if((bNeed2CopySalve)&&(newobsflag))
		{
			//mindif_readidx = (mindif_readidx-1+SLAVE_BUF_LEN)%SLAVE_BUF_LEN; //for simulate extraobs 1s
			Slave_Buf.readidx = mindif_readidx;	
			memcpy(&tmp_slave_obs, &(Slave_Buf.obsev[mindif_readidx]), sizeof(OBSEV));
			flagCurObs = CheckCurSlaveObs(&tmp_slave_obs,pSlave);
			if(flagCurObs)
			{
				memcpy(pSlave, &(Slave_Buf.obsev[mindif_readidx]), sizeof(OBSEV));
			}
			//memcpy(pSlave, &(Slave_Buf.obsev[(mindif_readidx-1+SLAVE_BUF_LEN)%SLAVE_BUF_LEN]), sizeof(OBSEV));
	#ifdef _SIMULATE
			memset(&(Slave_Buf.obsev[mindif_readidx]), 0, sizeof(OBSEV));
	#endif

			//update base sv pos
			CalcBaseStationSVPos(pSlave);
		}

		return TRUE;
	}

	return FALSE;
}

bool CheckCurSlaveObs(OBSEV* pCurSlave,OBSEV* pLastSlave)
{
	int32 CurSvCnt=0,LastSvCnt=0;
	int32 i = 0,svid = 0;

	word32 validFP_base;
	double snr_base;
	double locktime_base;
		
	CurSvCnt = pCurSlave->satnmax;
	LastSvCnt = pLastSlave->satnmax;

	if(CurSvCnt>LastSvCnt)
		return TRUE;
	
	if((CurSvCnt < 5)&&(LastSvCnt >= 5))
		return FALSE;

	if(((CurSvCnt*3) <= (2*LastSvCnt)) && (LastSvCnt >= 10))
		return FALSE;

	if(0)
	{
		for(i=0; i<pCurSlave->satnmax; i++)
		{
			svid = pCurSlave->obs[i].StarID;
			if(pCurSlave->obs[i].el < GetSVRTKElMask(svid))
			{
				CurSvCnt--;
				continue;
			}

			if(!RTKCoaseSVSelect(&(pCurSlave->obs[i]), &validFP_base, &snr_base, &locktime_base))
			{
				CurSvCnt--;
				continue;
			}
		}
		
		for(i=0; i<pLastSlave->satnmax; i++)
		{
			svid = pLastSlave->obs[i].StarID;
			if(pLastSlave->obs[i].el < GetSVRTKElMask(svid))
			{
				LastSvCnt--;
				continue;
			}

			if(!RTKCoaseSVSelect(&(pLastSlave->obs[i]), &validFP_base, &snr_base, &locktime_base))
			{
				LastSvCnt--;
				continue;
			}
		}

		if((CurSvCnt < 5)&&(LastSvCnt >= 5))
			return FALSE;

		if((CurSvCnt*3 <= LastSvCnt*2) && (CurSvCnt >= 8))
			return FALSE;
	}
	return TRUE;
}

bool GetObsvTime(OBSEV* pObsv, double* pobstow, UINT8 navmod)//根据模式判断bostime，存入第二个参数
{
	double obstow = 0.0;
	if(navmod == NAV_SYS_NONE)
	{
		navmod = GetCurNavMode();
	}
	
	if((pObsv->bitmap & OBSEV_VALID_GPS_DATA) && (navmod & NAV_SYS_GPS))
		obstow = pObsv->sec_gps;
	else if((pObsv->bitmap & OBSEV_VALID_BD_DATA)&& (navmod & NAV_SYS_BD))
	{
		//obstow = pObsv->sec_bd + GPS_BD_SYSTIME_OFFSET;
		obstow = pObsv->sec_bd;
	}
	else if((pObsv->bitmap & OBSEV_VALID_GLO_DATA)&& (navmod & NAV_SYS_GLO))
	{
		//GloToGpsTime(int32 N4,int32 NT,double t,int32 * weekn,double * tow)
		obstow = pObsv->sec_glo;	//?
	}
	else
		return FALSE;

	if(obstow>=SECONDS_IN_WEEK)
		obstow = obstow - SECONDS_IN_WEEK;

	*pobstow = obstow;

	return TRUE;
}



double GetRTKExtraTime(UINT8 navmod) //GPS extratime
{
	double time = -1.0;

	time = RTK_Extra_time[0];

	return time;
	/*
	if(navmod == NAV_SYS_GPS)
		time = RTK_Extra_time[0];
	else if(navmod == NAV_SYS_BD)
		time = RTK_Extra_time[1];
	else if(navmod == NAV_SYS_GLO)
		time = RTK_Extra_time[2];
	else if(navmod == NAV_SYS_NONE)
	{
		if(time<RTK_Extra_time[0])
			time = RTK_Extra_time[0];

		if(time<RTK_Extra_time[1])
			time = RTK_Extra_time[1];

		if(time<RTK_Extra_time[2])
			time = RTK_Extra_time[2];
	}
	else
		time = -1.0;
	
	return (time);*/
}


void CalcBaseStationSVPos(OBSEV* PrmSlave)
{
	bool flagTow = FALSE, bSVPosValid=FALSE;
	double slaveTow= 0.0, ts=0.0, fclkerr=0.0;
#ifdef _POSTPROC
	double az=0.0,el=0.0;
#endif
	int32 idx = 0, i;
	ECEF svvel;
	
	flagTow = GetObsvTime(PrmSlave, &slaveTow,NAV_SYS_NONE);
	if(flagTow==FALSE)
		return;

	for (idx = 0; idx< (PrmSlave->satnmax); idx++)
	{
		bSVPosValid=FALSE;
		if(SVInfo[PrmSlave->obs[idx].slotID-1].eph_age<SV_EPH_AGE_USABLE_THRES)
		{
			for(i=0; i<MAX_FREPIONT_PER_NAVSYS; i++)
			{
				if(PrmSlave->obs[idx].validmap[i] & OBST_VALID_RANGE)
				{
					ts = slaveTow - (PrmSlave->obs[idx].range[i])*RECIP_SPEED_OF_LIGHT;
					if(ts<0.0)
						ts+=SECONDS_IN_WEEK;

					SWI_disable();

					// 解算星历
					// 输入PrmSlave，读取全局结构体Sys3AlmEphUTCInfo，输出PrmSlave、以及（未使用）svvel、fclkerr
					CalcSV_PVT_Eph(PrmSlave->obs[idx].slotID, ts, &(PrmSlave->obs[idx].svpos), &svvel, &(PrmSlave->obs[idx].clkerr), &fclkerr,&(PrmSlave->obs[idx].toe));

					SWI_enable();

					// 由解算的星历计算卫星与观测点的距离
					// 输入PrmSlave、全局结构体Basefixposition，返回PrmSlave
					PrmSlave->obs[idx].distance = RTK_GetSatDist(PrmSlave->obs[idx].slotID,PrmSlave->obs[idx].svpos,Basefixposition.ecefpos,NULL,NULL,NULL);
					bSVPosValid=TRUE;
					break;
				}
			}
		}

		if(!bSVPosValid)
			memset(PrmSlave->obs[idx].validmap, 0, sizeof(PrmSlave->obs[idx].validmap));
	}

#ifdef _POSTPROC
	// 输入PrmSlave,读取全局变量FixEquItrCntThres,输出PrmSlave->SingleRcvrPos
	if(LSCalcRcvrPos_obs(PrmSlave,&(PrmSlave->SingleRcvrPos)) != FIX_NOT)
	{
		PrmSlave->bSinglePosValid=TRUE;
		for (idx = 0; idx< (PrmSlave->satnmax); idx++)
		{
			if(SVInfo[PrmSlave->obs[idx].slotID-1].eph_age<SV_EPH_AGE_USABLE_THRES)
			{
				// 输入PrmSlave，输出az、el
				CalcSVAngle(PrmSlave->obs[idx].svpos, PrmSlave->SingleRcvrPos, &az, &el);
				PrmSlave->obs[idx].az = (int16)(az+0.5);
				PrmSlave->obs[idx].el =	(int8)(el +0.5);
			}
				
		}
		// 查看不了函数定义，显示观测量？
		SatShowObs(PrmSlave);
	}
	else
		PrmSlave->bSinglePosValid=FALSE;
#endif

	return;
}




