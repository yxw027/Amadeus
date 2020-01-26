

#include "define.h"
#include "matrix.h"
#include "LsPVT.h"
#include "PVT.h"
#include "PVTRcvrInfo.h"
#include "postproc.h"

#include "coordinate.h"

#include <math.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "constdef.h"
#include "global.h"
#include "Cfgpara.h"

#ifndef _POSTPROC
#ifndef _SIMULATE
#include "HDefine.h"
#endif
#endif

#define MAX_LS_ROW	 (MAX_USE_SV_NUM)

#if SUPPORT_GLONASS
#define MAX_LS_POS_COL	6
#else
#define MAX_LS_POS_COL	5
#endif

int8 SysEnvSta = SYS_FULL_OPEN_SKY;
int8 SysEnvCon = SYS_CONDITION_NORMAL;

boolean bMixFixMaybeErr = FALSE;

static int32  FixEquItrCntThres=20;
const double ItrCvgMagThres=1.0, ItrDvgMagThres=1.0e8;

static int8 checkLsSVBuIdx(byte buflag, int8 navSysBuIdx[3]);
static double getLSWeight(int32 minCN0, PVT_TRKCH_INFO curTrkchInfo);
static int32 solveEquSetNew(TMatrix *pMatrAlfa, TMatrix *pMatrW, TMatrix *pMatrRho, TMatrix *pMatrRes);

/**	
 *	@brief	select most satellites to take part in position
 *	@para	the satellite number after coarse selection
 *    @author  Juan
 *	@return	avail svcnt
 **/
int32 LsSVSelection(word256* pCoarseChnMap, word256* pMaxChnMap, PVT_TRKCH_INFO *pTrkchInfo)
{
	int32 availSvCnt=0;
	int32 svid = 0, trk_ch = 0;
	byte CN0ThUse, CN100msThUse;
	//satellite selection for fix
	const byte FixCn01sTh[] = {24, 22, 19, 10, 5};
	const byte FixCn0100msTh[] = {24, 21, 17, 4, 3};
	bool onlyGloFreGrop = FALSE;
	
	int8 AvailNumTH = 10,cn0TH=0,i=0;
	
	UINT8 NavMode = GetCurNavMode();

	memcpy(pMaxChnMap, pCoarseChnMap, sizeof(word256));

	if(NavMode == NAV_SYS_GLO)
		onlyGloFreGrop = TRUE;


	for(trk_ch = 0; trk_ch < MAXCHANNELS; trk_ch ++)
	{
		CN0ThUse = FixCn01sTh[SysEnvSta-SYS_FULL_OPEN_SKY];
		CN100msThUse = FixCn0100msTh[SysEnvSta-SYS_FULL_OPEN_SKY];
		
		svid = pTrkchInfo[trk_ch].svid;
		
		if(SV_IsBd2Geo(svid))
		{
			CN0ThUse += 10;
			CN100msThUse += 10;
		}

		if(GetBitWord256(pMaxChnMap,trk_ch))
		{
#if SUPPORT_GLONASS
			if((pTrkchInfo[trk_ch].cn1s < CN0ThUse) || ((!onlyGloFreGrop) && SV_IsGlo(svid)))
#else
			if(pTrkchInfo[trk_ch].cn1s < CN0ThUse)
#endif
			{
				ClearBitWord256(pMaxChnMap,trk_ch);	//Clear AvailSV
				pTrkchInfo[trk_ch].selpath = FIX_SV_SNR_VALUE;
			}
			else
				availSvCnt++;
		}
	}
	//delete weak CN0 SV  before  KF work
	if((NavMode == NAV_SYS_GLO)||(NavMode == NAV_SYS_GPS) ||(NavMode == NAV_SYS_BD))
		AvailNumTH = 6;

	if(availSvCnt > AvailNumTH)
	{
		sortSVCandidateInfo( SV_CANDIDATE_SORT_TYPE_DESC,  SORT_SV_ITEM_CN0);
		
		if(CandidateSVList[AvailNumTH-1].cn0 >= 40)
			cn0TH = 40;
		else if(CandidateSVList[AvailNumTH-1].cn0 >= 38)
			cn0TH = 38;
		else if(CandidateSVList[AvailNumTH-1].cn0 >= 36)
			cn0TH = 36;
		else if(CandidateSVList[AvailNumTH-1].cn0 >= 34)
			cn0TH = 34;
		else if(CandidateSVList[AvailNumTH-1].cn0 >= 32)
			cn0TH = 32;
		else if(CandidateSVList[AvailNumTH-1].cn0 >= 30)
			cn0TH = 30;
		else 
			cn0TH = 5;

		cn0TH -= 5;
			
		for(i=0; i<CandidateSVListLen; i++)
		{
			svid = CandidateSVList[i].prn;
			if(availSvCnt <= AvailNumTH)
				break;
			
			if(CandidateSVList[i].cn0 >= cn0TH )
				continue;
			
			trk_ch = CandidateSVList[i].trkch;
			if(GetBitWord256(pMaxChnMap,trk_ch))
			{	
				availSvCnt--;
				ClearBitWord256(pMaxChnMap,trk_ch); //Clear AvailSV 							
				PVTTrkchInfo[trk_ch].selpath = FIX_SV_KFFALSE_CN0;
			}
		}
	}
	
	return availSvCnt;
}

/**
*	@brief	calculate receiver postion 
*	@param	
*			pPVTData, valid satellite information used for fix calculation
*			validSVCnt, valid satellite number of fix calculation, all the SV after coarse sv selection(GPS+COMPASS+GLONASS+ECA)
*			pFixPosInfo, receiver position information.
*	@return TRUE, calculate successfully
			FALSE, calculate failed
*			
*	@author	juan.gou
*	@date	Aug 12th, 2015
*/
byte lsErrpath=0;
boolean LSCalcRcvrPos(PVT_TRKCH_INFO* pTrkchInfo, PVT_FIX_INFO* pFixInfo)
{
	byte Itr_ExceptCnt = 0;
	double LastCorrect_Mag = 0.0;	
	int32 SvNum2Fix = 0, GpsSvCnt = 0, BdSvCnt = 0, GloSvCnt=0, minCN0 =100;
	ECEF rcvr_pos={0.0,0.0,0.0};//receiver position
	WGS temppos ={0,};
	int32 hisPosFlag = getRcvrHistoricalPos(&rcvr_pos, NULL);
	double bu[3]={0.0,0.0,0.0}, tempbu = 0.0;
	int8 svbuidx[3]={0,1,2};
	double pesudorange=0.0;
	int32 svid = 0, trkch = 0, i=0, dim_bu=0, dim_x=0;
	int32 trueSVCnt=0, itr_cnt=0;
	PR_MODIFY PRModify[3]={{FALSE,0.0},{FALSE,0.0},{FALSE,0.0}};

	TMatrix hMatrAlfa, hVecRho, hResXYZT, hMatrW;
	float64 MatrAlfa[MAX_LS_ROW][MAX_LS_POS_COL] = {0,};
	float64 MatrW[MAX_LS_ROW][MAX_LS_ROW] = {0,};
	float64 VecRho[MAX_LS_ROW];
	float64 ResXYZT[MAX_LS_POS_COL]; 
	float64 correct_mag, dis_rcvr2sv, angle_er;			//angle of earth rotation
	float64 delta_x,delta_y, delta_z; 

	//float64 tmpNoise=0.0;

	int32 row=0;
	lsErrpath=0;
	
	pFixInfo->posres = FIX_NOT;

	for(trkch = 0; trkch < MAXCHANNELS_PVT; trkch ++)
	{
		svid = pTrkchInfo[trkch].svid;
		if(GetBitWord256(&(pFixInfo->poschmap),trkch))
		{
			if((hisPosFlag == HISTORICAL_POS_INVALID) && (!SV_IsECA(svid)))
			{
				rcvr_pos.x += pTrkchInfo[trkch].svpos.x;
				rcvr_pos.y += pTrkchInfo[trkch].svpos.y;
				rcvr_pos.z += pTrkchInfo[trkch].svpos.z;
			}

			SvNum2Fix++;

			if(SV_IsGps(svid))
				GpsSvCnt++;
			else if(SV_IsBd2(svid))
				BdSvCnt++;
		#if SUPPORT_GLONASS
			else if(SV_IsGlo(svid))
				GloSvCnt++;
		#endif

			if(pTrkchInfo[trkch].cn1s < minCN0)
				minCN0 = pTrkchInfo[trkch].cn1s;	
		}
	}

	//calc the bu dimension and navsys bu idx.
	dim_bu = checkLsSVBuIdx(pFixInfo->buflag, svbuidx);
	dim_x = dim_bu+3;	//add the x, y, z 
	
	if((SvNum2Fix < dim_x) || (dim_bu < 1) || (dim_bu > 3))	//svcnt cannot less than unkown parameters count.
	{
		lsErrpath=1;
		return FALSE;
	}
		
	//get the initial  rcvr pos
	trueSVCnt =  GpsSvCnt + BdSvCnt + GloSvCnt;
	if(hisPosFlag == HISTORICAL_POS_INVALID)
	{
		rcvr_pos.x /= trueSVCnt;
		rcvr_pos.y /= trueSVCnt;
		rcvr_pos.z /= trueSVCnt;
		
		if(ECEF2WGS(&rcvr_pos, &temppos) == 0)
		{
			temppos.alt = 0;
			WGS2ECEF(&temppos, &rcvr_pos);
		}
		else
		{
			rcvr_pos.x = 1.0;
			rcvr_pos.y = 1.0;
			rcvr_pos.z = 1.0;
		}
	}

	//check nav system time bias modification 
	GetGNSSPRTimeBias(pFixInfo->buflag,PRModify);

	/******************************start calculate******************************/ 
	
	itr_cnt=0;
	//matrix & matrix handle 
	memset(MatrAlfa, 0, sizeof(MatrAlfa));
	memset(MatrW, 0, sizeof(MatrW));
	memset(VecRho, 0, sizeof(VecRho));
	memset(ResXYZT, 0, sizeof(ResXYZT));
	//initialize all matrix and vector handle
	setMatrHandle(&hMatrAlfa,MAX_LS_ROW, MAX_LS_POS_COL, SvNum2Fix, dim_x, MatrAlfa);
	setMatrHandle(&hMatrW, MAX_LS_ROW, MAX_LS_ROW, SvNum2Fix, SvNum2Fix, MatrW);
	setMatrHandle(&hVecRho, MAX_LS_ROW, 1, SvNum2Fix, 1,	VecRho);
	setMatrHandle(&hResXYZT, MAX_LS_POS_COL, 1, dim_x, 1, ResXYZT);

	//end of matrix handle initialization
	
	ZerosMatrix(&hMatrW);

	//build pseudorange equation: 
	// sqrt((Xi-Xu)*(Xi-Xu)+(Yi-Yu)*(Yi-Yu) + (Zi-Zu)*(Zi-Zu)) + bu = pseu
	//iteration operation
	for(itr_cnt=0; itr_cnt<FixEquItrCntThres; itr_cnt++)
	{
		correct_mag=0.0;	

		row=0;			
		ZerosMatrix(&hMatrAlfa);
		
		for(trkch = 0; trkch < MAXCHANNELS_PVT; trkch ++)
		{
			svid = pTrkchInfo[trkch].svid;
			if(!GetBitWord256(&(pFixInfo->poschmap),trkch))
				continue;

			//angle and PR modify
			tempbu = 0.0;
			pesudorange = pTrkchInfo[trkch].pr;
			
			if(SV_IsECA(svid))
				tempbu = 0.0;
			else 
			{
				if(SV_IsGps(svid))
				{
					tempbu = bu[svbuidx[0]];
					if(PRModify[0].bPRModify)
						pesudorange -= PRModify[0].bias; 
				}
				else if(SV_IsBd2(svid))
				{
					tempbu = bu[svbuidx[1]];
					if(PRModify[1].bPRModify)
						pesudorange -= PRModify[1].bias; 
				}
			#if SUPPORT_GLONASS
				else if(SV_IsGlo(svid))
				{
					tempbu = bu[svbuidx[2]];
					if(PRModify[2].bPRModify)
						pesudorange -= PRModify[2].bias; 
				}
			#endif
			}
			
			angle_er= -((pesudorange- tempbu)*RECIP_SPEED_OF_LIGHT)*OMEGAE_WGS;
			
			//calculate delta_x, delta_y, delta_z
			delta_x = rcvr_pos.x - (pTrkchInfo[trkch].svpos.x*cos(angle_er) - pTrkchInfo[trkch].svpos.y*sin(angle_er));
			delta_y = rcvr_pos.y - (pTrkchInfo[trkch].svpos.y*cos(angle_er) + pTrkchInfo[trkch].svpos.x*sin(angle_er));
			delta_z = rcvr_pos.z - pTrkchInfo[trkch].svpos.z;

			//calculate distance between receiver and SV
			dis_rcvr2sv = sqrt(delta_x*delta_x + delta_y*delta_y + delta_z*delta_z);

			//setup matrix A and V
			VecRho[row] = pesudorange - (dis_rcvr2sv + tempbu);
			
			MatrAlfa[row][0] = delta_x / dis_rcvr2sv;
			MatrAlfa[row][1] = delta_y / dis_rcvr2sv;
			MatrAlfa[row][2] = delta_z / dis_rcvr2sv;	
			
			if(SV_IsGps(svid))
				MatrAlfa[row][svbuidx[0]+3] = 1.0;
			else if(SV_IsBd2(svid))
				MatrAlfa[row][svbuidx[1]+3] = 1.0;
		#if SUPPORT_GLONASS
			else if(SV_IsGlo(svid))
				MatrAlfa[row][svbuidx[2]+3] = 1.0;
		#endif

			if(itr_cnt == 0)
				MatrW[row][row] = getLSWeight(minCN0, pTrkchInfo[trkch]);

			row++;
		}

		//solve fixing equation
		if(solveEquSetNew(&hMatrAlfa, &hMatrW, &hVecRho, &hResXYZT) != 0)
		{
			//lsErrpath=2;
			return FALSE;
		}

		//check equation convergence attribution
		if(dim_x == 4)
			correct_mag=sqrt(ResXYZT[0]*ResXYZT[0] + ResXYZT[1]*ResXYZT[1] + ResXYZT[2]*ResXYZT[2] + ResXYZT[3]*ResXYZT[3]);
		else if(dim_x == 5)
			correct_mag=sqrt(ResXYZT[0]*ResXYZT[0] + ResXYZT[1]*ResXYZT[1] + ResXYZT[2]*ResXYZT[2] + ResXYZT[3]*ResXYZT[3] + ResXYZT[4]*ResXYZT[4]);
#if SUPPORT_GLONASS
		else if(dim_x == 6)
			correct_mag=sqrt(ResXYZT[0]*ResXYZT[0] + ResXYZT[1]*ResXYZT[1] + ResXYZT[2]*ResXYZT[2] + ResXYZT[3]*ResXYZT[3] + ResXYZT[4]*ResXYZT[4] + ResXYZT[5]*ResXYZT[5]);
#endif

		//check iteration exception
		if(/* correct_mag > ItrDvgMagThres||*/ ((correct_mag > ItrCvgMagThres) && (itr_cnt == (FixEquItrCntThres-1))) )
		{
			lsErrpath=3;
			return FALSE;
		}

		//check lspvt iteration divergence
		if((correct_mag > LastCorrect_Mag) && (correct_mag > 1000.0))
		{
			Itr_ExceptCnt++;
			if(Itr_ExceptCnt > 2)
			{
				lsErrpath=4;
				return FALSE;
			}
		}
		else
			Itr_ExceptCnt = 0;

		LastCorrect_Mag = correct_mag;

		rcvr_pos.x = rcvr_pos.x + ResXYZT[0];
		rcvr_pos.y = rcvr_pos.y + ResXYZT[1];
		rcvr_pos.z = rcvr_pos.z + ResXYZT[2];
		for(i=0;i<dim_bu;i++)
			bu[i] += ResXYZT[3+i]; 

		if(correct_mag < ItrCvgMagThres)
			break;
	}

	//output
	pFixInfo->rcvrpos = rcvr_pos;
	if(GpsSvCnt > 0)
	{	
		pFixInfo->gpsbu = bu[svbuidx[0]];
		if(PRModify[0].bPRModify)
			pFixInfo->bd2bu += PRModify[0].bias;
	}
		
	if(BdSvCnt > 0)
	{
		pFixInfo->bd2bu = bu[svbuidx[1]];
		if(PRModify[1].bPRModify)
			pFixInfo->bd2bu += PRModify[1].bias;
	}

#if SUPPORT_GLONASS
	if(GloSvCnt > 0)
	{
		pFixInfo->globu= bu[svbuidx[2]];
		if(PRModify[2].bPRModify)
			pFixInfo->globu += PRModify[2].bias;
	}
#endif

	if(trueSVCnt >= dim_x)
		pFixInfo->posres = FIX_3D;
	else
		pFixInfo->posres = FIX_2D;
	
	return TRUE;
}

int32 LSCalcRcvrPos_obs(OBSEV* pObs, ECEF* pRcvrPos)
{
	int32 fixres=FIX_NOT;
	byte Itr_ExceptCnt = 0, buflag=0;
	double LastCorrect_Mag = 0.0;	
	
	int32 SvNum2Fix = 0, GpsSvCnt = 0, BdSvCnt = 0, GloSvCnt=0;
	ECEF rcvr_pos={0.0,0.0,0.0};//receiver position
	WGS temppos ={0,};
	double bu[3]={0.0,0.0,0.0}, tempbu = 0.0;
	int8 svbuidx[3]={0,1,2};
	double pesudorange=0.0;
	int32 svid = 0, i=0, dim_bu=0, dim_x=0, idx, fpidx, itr_cnt=0;

	TMatrix hMatrAlfa, hVecRho, hResXYZT, hMatrW;
	float64 MatrAlfa[MAX_LS_ROW][MAX_LS_POS_COL] = {0,};
	float64 MatrW[MAX_LS_ROW][MAX_LS_ROW] = {0,};
	float64 VecRho[MAX_LS_ROW];
	float64 ResXYZT[MAX_LS_POS_COL]; 
	float64 correct_mag, dis_rcvr2sv, angle_er;			//angle of earth rotation
	float64 delta_x,delta_y, delta_z; 

	int32 row=0;

	if(pObs->satnmax<5)
		return FIX_NOT;
	
	for(idx = 0; idx < pObs->satnmax; idx++)
	{
		svid = pObs->obs[idx].slotID;

		if(pObs->obs[idx].validmap[0] == 0 && pObs->obs[idx].validmap[1] == 0 && pObs->obs[idx].validmap[2] == 0)
			continue;

		if(idx>=MAX_LS_ROW)
			continue;

		rcvr_pos.x += pObs->obs[idx].svpos.x;
		rcvr_pos.y += pObs->obs[idx].svpos.y;
		rcvr_pos.z += pObs->obs[idx].svpos.z;

		SvNum2Fix++;

		if(SV_IsGps(svid))
		{
			GpsSvCnt++;
			buflag |= (0x1<<0);
		}
		else if(SV_IsBd2(svid))
		{
			BdSvCnt++;
			buflag |= (0x1<<1);
		}
	#if SUPPORT_GLONASS
		else if(SV_IsGlo(svid))
		{
			GloSvCnt++;
			buflag |= (0x1<<2);
		}
	#endif
	}

	//calc the bu dimension and navsys bu idx.
	//输入buflag，输出svbuidx，返回dim_bu
	dim_bu = checkLsSVBuIdx(buflag, svbuidx);
	dim_x = dim_bu+3;	//add the x, y, z 
	
	if((SvNum2Fix < dim_x) || (dim_bu < 1) || (dim_bu > 3))	//svcnt cannot less than unkown parameters count.
		return FIX_NOT;
		
	//get the initial  rcvr pos
	rcvr_pos.x /= SvNum2Fix;
	rcvr_pos.y /= SvNum2Fix;
	rcvr_pos.z /= SvNum2Fix;
	
	if(ECEF2WGS(&rcvr_pos, &temppos) == 0)
	{
		temppos.alt = 0;
		WGS2ECEF(&temppos, &rcvr_pos);
	}
	else
	{
		rcvr_pos.x = 1.0;
		rcvr_pos.y = 1.0;
		rcvr_pos.z = 1.0;
	}

	/******************************start calculate******************************/ 
	itr_cnt=0;
	//matrix & matrix handle 
	memset(MatrAlfa, 0, sizeof(MatrAlfa));
	memset(MatrW, 0, sizeof(MatrW));
	memset(VecRho, 0, sizeof(VecRho));
	memset(ResXYZT, 0, sizeof(ResXYZT));
	//initialize all matrix and vector handle
	setMatrHandle(&hMatrAlfa,MAX_LS_ROW, MAX_LS_POS_COL, SvNum2Fix, dim_x, MatrAlfa);
	setMatrHandle(&hMatrW, MAX_LS_ROW, MAX_LS_ROW, SvNum2Fix, SvNum2Fix, MatrW);
	setMatrHandle(&hVecRho, MAX_LS_ROW, 1, SvNum2Fix, 1,	VecRho);
	setMatrHandle(&hResXYZT, MAX_LS_POS_COL, 1, dim_x, 1, ResXYZT);

	//end of matrix handle initialization
	ZerosMatrix(&hMatrW);

	//build pseudorange equation: 
	// sqrt((Xi-Xu)*(Xi-Xu)+(Yi-Yu)*(Yi-Yu) + (Zi-Zu)*(Zi-Zu)) + bu = pseu
	//iteration operation
	for(itr_cnt=0; itr_cnt<FixEquItrCntThres; itr_cnt++)
	{
		correct_mag=0.0;	

		row=0;			
		ZerosMatrix(&hMatrAlfa);
		
		for(idx = 0; idx < pObs->satnmax; idx++)
		{
			svid = pObs->obs[idx].slotID;

			if(idx>=MAX_LS_ROW)
				continue;
			
			//angle and PR modify
			tempbu = 0.0;

			for(fpidx=0; fpidx<MAX_FREPIONT_PER_NAVSYS; fpidx++)
			{
				if(pObs->obs[idx].validmap[fpidx] & OBST_VALID_RANGE)
				{
					pesudorange = pObs->obs[idx].range[fpidx] + pObs->obs[idx].clkerr*SPEED_OF_LIGHT;
					break;
				}
			}
			if(fpidx >= MAX_FREPIONT_PER_NAVSYS)
				continue;

			if(SV_IsGps(svid))
				tempbu = bu[svbuidx[0]];
			else if(SV_IsBd2(svid))
				tempbu = bu[svbuidx[1]];
		#if SUPPORT_GLONASS
			else if(SV_IsGlo(svid))
				tempbu = bu[svbuidx[2]];
		#endif
			
			angle_er= -((pesudorange- tempbu)*RECIP_SPEED_OF_LIGHT)*OMEGAE_WGS;
			
			//calculate delta_x, delta_y, delta_z
			delta_x = rcvr_pos.x - (pObs->obs[idx].svpos.x*cos(angle_er) - pObs->obs[idx].svpos.y*sin(angle_er));
			delta_y = rcvr_pos.y - (pObs->obs[idx].svpos.y*cos(angle_er) + pObs->obs[idx].svpos.x*sin(angle_er));
			delta_z = rcvr_pos.z - pObs->obs[idx].svpos.z;

			//calculate distance between receiver and SV
			dis_rcvr2sv = sqrt(delta_x*delta_x + delta_y*delta_y + delta_z*delta_z);

			//setup matrix A and V
			VecRho[row] = pesudorange - (dis_rcvr2sv + tempbu);
			
			MatrAlfa[row][0] = delta_x / dis_rcvr2sv;
			MatrAlfa[row][1] = delta_y / dis_rcvr2sv;
			MatrAlfa[row][2] = delta_z / dis_rcvr2sv;	
			
			if(SV_IsGps(svid))
				MatrAlfa[row][svbuidx[0]+3] = 1.0;
			else if(SV_IsBd2(svid))
				MatrAlfa[row][svbuidx[1]+3] = 1.0;
		#if SUPPORT_GLONASS
			else if(SV_IsGlo(svid))
				MatrAlfa[row][svbuidx[2]+3] = 1.0;
		#endif

			if(itr_cnt == 0)
				MatrW[row][row] = 1.0;

			row++;
		}

		//solve fixing equation
		if(solveEquSetNew(&hMatrAlfa, &hMatrW, &hVecRho, &hResXYZT) != 0)
		{
			//lsErrpath=2;
			return FIX_NOT;
		}

		//check equation convergence attribution
		if(dim_x == 4)
			correct_mag=sqrt(ResXYZT[0]*ResXYZT[0] + ResXYZT[1]*ResXYZT[1] + ResXYZT[2]*ResXYZT[2] + ResXYZT[3]*ResXYZT[3]);
		else if(dim_x == 5)
			correct_mag=sqrt(ResXYZT[0]*ResXYZT[0] + ResXYZT[1]*ResXYZT[1] + ResXYZT[2]*ResXYZT[2] + ResXYZT[3]*ResXYZT[3] + ResXYZT[4]*ResXYZT[4]);
#if SUPPORT_GLONASS
		else if(dim_x == 6)
			correct_mag=sqrt(ResXYZT[0]*ResXYZT[0] + ResXYZT[1]*ResXYZT[1] + ResXYZT[2]*ResXYZT[2] + ResXYZT[3]*ResXYZT[3] + ResXYZT[4]*ResXYZT[4] + ResXYZT[5]*ResXYZT[5]);
#endif

		//check iteration exception
		// 访问了全局变量FixEquItrCntThres
		if(/* correct_mag > ItrDvgMagThres||*/ ((correct_mag > ItrCvgMagThres) && (itr_cnt == (FixEquItrCntThres-1))) )
			return FALSE;

		//check lspvt iteration divergence
		if((correct_mag > LastCorrect_Mag) && (correct_mag > 1000.0))
		{
			Itr_ExceptCnt++;
			if(Itr_ExceptCnt > 2)
				return FALSE;
		}
		else
			Itr_ExceptCnt = 0;

		LastCorrect_Mag = correct_mag;

		rcvr_pos.x = rcvr_pos.x + ResXYZT[0];
		rcvr_pos.y = rcvr_pos.y + ResXYZT[1];
		rcvr_pos.z = rcvr_pos.z + ResXYZT[2];
		for(i=0;i<dim_bu;i++)
			bu[i] += ResXYZT[3+i]; 

		if(correct_mag < ItrCvgMagThres)
			break;
	}

	//output
	pRcvrPos->x = rcvr_pos.x;
	pRcvrPos->y = rcvr_pos.y;
	pRcvrPos->z = rcvr_pos.z;

	if(SvNum2Fix >= dim_x)
		fixres = FIX_3D;
	else
		fixres = FIX_2D;
	
	return fixres;
}	



/**
*	@brief	calculate receiver velocity 
*	@param	
*			pPVTData, valid satellite information used for fix calculation
*			validSVCnt, valid satellite number of fix calculation, all the SV after coarse sv selection(GPS+COMPASS+SBAS+ECA)
*			pFixVelInfo, receiver velocity information
*	@return TRUE, calculate successfully
*			FALSE, calculate failed				
*	@author	juan.gou
*	@date	Aug 13th, 2015
*/
boolean LSCalcRcvrVel(PVT_TRKCH_INFO* pTrkchInfo, PVT_FIX_INFO* pFixInfo)
{
	int32 SvNum2Vel = 0, GPSSvCnt = 0, BDSVCnt = 0, GLOSVCnt = 0, minCN0 = 100;
	double wave_len = 0.0;
	int32 trkch=0, svid=0;
	ECEF rcvrPos = {0.0, 0.0, 0.0};
	
	int32 row=0;
	
	TMatrix hMatrD, hMatrVelT, hMatrAlfa, hMatrW;	
	double MatrD[MAX_LS_ROW];
	double MatrVelT[4];
	double MatrAlfa[MAX_LS_ROW][4];
	double MatrW[MAX_LS_ROW][MAX_LS_ROW];
	
	double dis_rcvr2sv;
	double delta_x,delta_y, delta_z; 
	
	pFixInfo->velres = FIX_NOT;
	
	rcvrPos = pFixInfo->rcvrpos;
	
	for(trkch=0; trkch<MAXCHANNELS_PVT; trkch++)
	{
		svid = pTrkchInfo[trkch].svid;
		if(GetBitWord256(&(pFixInfo->velchmap),trkch))
		{
			SvNum2Vel++;
			if(SV_IsGps(svid))
				GPSSvCnt++;
			else if(SV_IsBd2(svid))
				BDSVCnt++;
#if SUPPORT_GLONASS
			else if(SV_IsGlo(svid))
				GLOSVCnt++;
#endif
			if(pTrkchInfo[trkch].cn1s < minCN0)
				minCN0 = pTrkchInfo[trkch].cn1s;
		}
	}
	
	if(SvNum2Vel < 4)
		return FALSE;


	memset(MatrD, 0, sizeof(MatrD));
	memset(MatrVelT, 0, sizeof(MatrVelT));
	memset(MatrAlfa, 0, sizeof(MatrAlfa));
	memset(MatrW, 0, sizeof(MatrW));
	//init matrix handle and re-init hMatrAlfa	
	setMatrHandle(&hMatrAlfa,MAX_LS_ROW, 4, SvNum2Vel, 4, MatrAlfa); //reinitialize this matrix handle
	setMatrHandle(&hMatrW, MAX_LS_ROW, MAX_LS_ROW, SvNum2Vel, SvNum2Vel, MatrW);
	setMatrHandle(&hMatrD, MAX_LS_ROW, 1, SvNum2Vel, 1, MatrD);
	setMatrHandle(&hMatrVelT, 4, 1, 4, 1, MatrVelT);

	
	ZerosMatrix(&hMatrAlfa);
	ZerosMatrix(&hMatrW);

	for(trkch=0; trkch<MAXCHANNELS_PVT; trkch++)
	{
		svid = pTrkchInfo[trkch].svid;
		if(!GetBitWord256(&(pFixInfo->velchmap), trkch))
			continue;

		delta_x = pTrkchInfo[trkch].svpos.x - rcvrPos.x;
		delta_y = pTrkchInfo[trkch].svpos.y - rcvrPos.y;
		delta_z = pTrkchInfo[trkch].svpos.z - rcvrPos.z;

		dis_rcvr2sv = sqrt( delta_x*delta_x + delta_y*delta_y + 	delta_z*delta_z );
							
		MatrAlfa[row][0] = delta_x/dis_rcvr2sv;
		MatrAlfa[row][1] = delta_y/dis_rcvr2sv;
		MatrAlfa[row][2] = delta_z/dis_rcvr2sv;

		if(SV_IsECA(svid))
			MatrAlfa[row][3] = 0.0;
		else
			MatrAlfa[row][3] = 1.0;

		//init vector/matrix MatrD
		if(SV_IsECA(svid))
			wave_len = 0.0;
		else
			wave_len = GetSVWaveLen(svid,pTrkchInfo[trkch].freq_point);
		
		if(SV_IsECA(svid))
			MatrD[row] = 0;
		else
			MatrD[row] = MatrAlfa[row][0]*(pTrkchInfo[trkch].svvel.x) + MatrAlfa[row][1]*(pTrkchInfo[trkch].svvel.y) + MatrAlfa[row][2]*(pTrkchInfo[trkch].svvel.z) + wave_len*(pTrkchInfo[trkch].fd);
		
		MatrW[row][row] = getLSWeight(minCN0, pTrkchInfo[trkch]);	

		row++;
	}

	//solve velocity equation
	if(solveEquSetNew(&hMatrAlfa, &hMatrW, &hMatrD, &hMatrVelT) != 0)
		return FALSE;

	(pFixInfo->rcvrvel).x = MatrVelT[0];
	(pFixInfo->rcvrvel).y = MatrVelT[1];
	(pFixInfo->rcvrvel).z = MatrVelT[2];
	pFixInfo->ctu = -MatrVelT[3];

	if(GPSSvCnt + BDSVCnt + GLOSVCnt >= 4)
		pFixInfo->velres = FIX_3D;
	else
		pFixInfo->velres = FIX_2D;

	return	TRUE;
}


static int8 checkLsSVBuIdx(byte buflag, int8 navSysBuIdx[3])
{
/* bu flag:
	1 0 0  0 0 0  0  gps only
	0 1 0  0 0 0  0  bd only
	0 0 1  0 0 0  0  glo only
	1 1 0  0 0 0  0  gps + bd
	1 0 1  0 0 0  0  gps + glo
	0 1 1  0 0 0  0  bd + glo
	1 1 1  0 0 0  0  gps + bd + glo
	0 0 0  1 0 0  0  gps_bd mix
	0 0 1  1 0 0  0  glo + gps_bd mix
	0 0 0  0 1 0  0  gps_glo mix
	0 1 0  0 1 0  0  bd + gps_glo mix
	0 0 0  0 0 1  0  bd_glo mix
	1 0 0  0 0 1  0  gps + bd_glo mix
	0 0 0  0 0 0  1  gps_bd_glo mix
*/
	int32 i=0, tmp = 0, dim_bu=0;
	navSysBuIdx[0] = 0;
	navSysBuIdx[1] = 0;
#if SUPPORT_GLONASS
	navSysBuIdx[2] = 0;
#endif

	for(i=0; i<7; i++)
	{
		if(buflag & (1<<i))
		{
			tmp = 1<<i;
#if SUPPORT_GLONASS
			if((tmp==BITMAP_DIM_BU_GPS) || (tmp==BITMAP_DIM_BU_GPS_BD2) || (tmp==BITMAP_DIM_BU_GPS_GLO) || (tmp==BITMAP_DIM_BU_GPS_BD2_GLO))
				navSysBuIdx[0] = dim_bu;		//gps bu idx
				
			if((tmp==BITMAP_DIM_BU_BD2) || (tmp==BITMAP_DIM_BU_GPS_BD2) || (tmp==BITMAP_DIM_BU_BD2_GLO) || (tmp==BITMAP_DIM_BU_GPS_BD2_GLO))
				navSysBuIdx[1] = dim_bu;		//bd bu idx

			if((tmp==BITMAP_DIM_BU_GLO) || (tmp==BITMAP_DIM_BU_GPS_GLO) || (tmp==BITMAP_DIM_BU_BD2_GLO) || (tmp==BITMAP_DIM_BU_GPS_BD2_GLO))
				navSysBuIdx[2] = dim_bu;		//glo bu idx
#else
			if((tmp==BITMAP_DIM_BU_GPS) || (tmp==BITMAP_DIM_BU_GPS_BD2))
				navSysBuIdx[0] = dim_bu;		//gps bu idx
				
			if((tmp==BITMAP_DIM_BU_BD2) || (tmp==BITMAP_DIM_BU_GPS_BD2))
				navSysBuIdx[1] = dim_bu;		//bd bu idx
#endif
			
			dim_bu++;		
		}
	}

	return dim_bu;
}


double getLSWeight(int32 minCN0, PVT_TRKCH_INFO curTrkchInfo)
{
	int32 svid = curTrkchInfo.svid;
	int32 el = curTrkchInfo.el;
	int32 weightIdx = (curTrkchInfo.cn1s+curTrkchInfo.cn100ms)/2-minCN0+15;

#if SUPPORT_GLONASS
	if(SV_IsGps(svid) || SV_IsBd2(svid) || SV_IsGlo(svid))
#else
	if(SV_IsGps(svid) || SV_IsBd2(svid))
#endif
	{
		if(SysEnvSta < SYS_HALF_OPEN_SKY)
		{
			if(el<5)
				weightIdx -= 15;
			else if(el<10)
				weightIdx -= 10;
			else if(el<15)
				weightIdx -= 6;
			else if(el<20)
				weightIdx -= 3;
			else if(el<25)
				weightIdx -= 2;
			else if(el<30)
				weightIdx -= 1;
		}
#if 0
		if(curTrkchInfo.lockedtime <5)
			weightIdx -= 15;
		else if(curTrkchInfo.lockedtime <10)
			weightIdx -= 10;
		else if(curTrkchInfo.lockedtime <30)
			weightIdx -= 6;
		else if(curTrkchInfo.lockedtime <100)
			weightIdx -= 3;
#endif
	}

#if SUPPORT_GLONASS
	if(SV_IsGlo(svid))
		weightIdx -= 8;
#endif

	
	weightIdx = weightIdx >= 0 ? weightIdx : 0;
	weightIdx = weightIdx <= 47 ? weightIdx : 47;
	
	return PR_NOISE_DifCN0[weightIdx];
}

static int32 solveEquSetNew(TMatrix *pMatrAlfa, TMatrix *pMatrW, TMatrix *pMatrRho, TMatrix *pMatrRes)
{
	TMatrix InvMatrix;
	double inverse[MAX_LS_POS_COL][MAX_LS_POS_COL]={{0.0,},};
	
	double MatrAlfa_Trans[MAX_LS_POS_COL][MAX_LS_ROW] = {0,};
	double MatrB[MAX_LS_POS_COL][MAX_LS_POS_COL] = {0,};		//square matrix
	double MatrC[MAX_LS_POS_COL][MAX_LS_POS_COL] = {0,};		//square matrix
	double MatrD[MAX_LS_POS_COL][MAX_LS_ROW] = {0,};
	double MatrE[MAX_LS_ROW][MAX_LS_ROW] = {0,};
	
	TMatrix hMatrAlfa_Trans, hMatrB, hMatrC, hMatrD, hMatrE;//
		
	//check matrix Alfa is square or not
	if(pMatrAlfa->rows == pMatrAlfa->cols)
	{
		if(pMatrAlfa->cols != pMatrRho->rows)
		{
			lsErrpath=21;
			return -1;
		}

		setMatrHandle(&InvMatrix, MAX_LS_POS_COL, MAX_LS_POS_COL, pMatrAlfa->rows, pMatrAlfa->cols, inverse);

		if(MatrixInverse(pMatrAlfa, &InvMatrix) != 0)
		{
			lsErrpath=22;
			return -1;
		}

		MatrixMul(&InvMatrix, pMatrRho, pMatrRes);

		return 0;
	}
	else if(pMatrAlfa->rows > pMatrAlfa->cols)	//Alfa is non-square matrix
	{			

		memset(MatrAlfa_Trans, 0, sizeof(MatrAlfa_Trans));
		memset(MatrB, 0, sizeof(MatrB));
		memset(MatrC, 0, sizeof(MatrC));
		memset(MatrD, 0, sizeof(MatrD));
		memset(MatrE, 0, sizeof(MatrE));


		//init all matrix handle
		setMatrHandle(&hMatrAlfa_Trans,MAX_LS_POS_COL, MAX_LS_ROW, pMatrAlfa->cols, pMatrAlfa->rows, MatrAlfa_Trans);
		setMatrHandle(&hMatrB, MAX_LS_POS_COL, MAX_LS_POS_COL, pMatrAlfa->cols, pMatrAlfa->cols, MatrB);
		setMatrHandle(&hMatrC, MAX_LS_POS_COL, MAX_LS_POS_COL, pMatrAlfa->cols, pMatrAlfa->cols, MatrC);
		setMatrHandle(&hMatrD, MAX_LS_POS_COL, MAX_LS_ROW, pMatrAlfa->cols, pMatrAlfa->rows, MatrD);
		setMatrHandle(&hMatrE, MAX_LS_ROW, MAX_LS_ROW, pMatrAlfa->cols, pMatrAlfa->rows, MatrE);

		MatrixTranspose(pMatrAlfa, &hMatrAlfa_Trans);
		MatrixMul(&hMatrAlfa_Trans, pMatrW, &hMatrE);

		MatrixMul(&hMatrE, pMatrAlfa, &hMatrB);

		if(MatrixInverse(&hMatrB, &hMatrC) != 0)
		{
			lsErrpath=23;
			return -1;
		}

		MatrixMul(&hMatrC, &hMatrE, &hMatrD);
		MatrixMul(&hMatrD, pMatrRho, pMatrRes);

		return 0;
	}
	else
	{
		lsErrpath=24;
		return -1;
	}
}


/**
*	@brief	calculate DOP(GDOP, HDOP, VDOP, TDOP, PDOP), This function should be called after positioning
*			DOP info return by pointer pFixDOP. 
*	@param	
*		
*	@author	juan.gou
*	@return 0, success; -1, fail
*	@date	Jan 18, 2012
*/
int32 CalcFixDOP(byte buflag, word256* pPosChnMap, PVT_TRKCH_INFO* pSVPVTInfo, ECEF rcvrpos, FIX_DOP* pDop)
{
#if SUPPORT_GLONASS
	int32 dim_x=0, dim_bu=0, idx = 0;
	int8 svbuidx[3] = {0,1,2};
	int32 trkch=0, svid = 0, svcnt=0, gpssvcnt=0, bdsvcnt=0, glosvcnt=0;
	int32 row=0;
	NEH svneh = {0,};
	double r=0.0;
#else
	int32 dim_x=0, dim_bu=0, idx = 0;
	int8 svbuidx[2] = {0,1};
	int32 trkch=0, svid = 0, svcnt=0, gpssvcnt=0, bdsvcnt=0;
	int32 row=0;
	NEH svneh = {0,};
	double r=0.0;
#endif
	double MatrH[MAX_LS_ROW][MAX_LS_POS_COL];
	TMatrix hMatrH;

	double TranMatrH[MAX_LS_POS_COL][MAX_LS_ROW];
	TMatrix hTransMatrH;
	
	double MatrTransHMH[MAX_LS_POS_COL][MAX_LS_POS_COL];
	TMatrix hMatrTransHMH;
	
	double MatrG[MAX_LS_POS_COL][MAX_LS_POS_COL];
	TMatrix hMatrG;
	
	for(trkch=0; trkch<MAXCHANNELS_PVT; trkch++)
	{
		svid = pSVPVTInfo[trkch].svid;
		if(GetBitWord256(pPosChnMap,trkch))
		{
			svcnt ++;
			if(SV_IsGps(svid))
				gpssvcnt++;
			else if(SV_IsBd2(svid))
				bdsvcnt++;
#if SUPPORT_GLONASS
			else if(SV_IsGlo(svid))
				glosvcnt++;
#endif
		}
	}

	//calc the bu dimension
	dim_bu = checkLsSVBuIdx(buflag, svbuidx);

	dim_x = dim_bu+3;	//add the x, y, z 
	if((svcnt < dim_x) || (dim_bu < 1) || (dim_bu > 3))	//svcnt cannot less than unkown parameters count.
		return FALSE;

	setMatrHandle(&hMatrH,MAX_LS_ROW, MAX_LS_POS_COL, svcnt, dim_x, (double*)MatrH);
	ZerosMatrix(&hMatrH);

	for(trkch=0; trkch<MAXCHANNELS_PVT; trkch++)
	{
		svid = pSVPVTInfo[trkch].svid;
		if(!GetBitWord256(pPosChnMap,trkch))
			continue;

		
		if(ECEF2NEH(&(pSVPVTInfo[trkch].svpos), &rcvrpos, &svneh) != 0)
			return -1;

		r = sqrt(svneh.north*svneh.north + svneh.east*svneh.east + svneh.head*svneh.head);

		MatrH[row][0] = svneh.north / r;
		MatrH[row][1] = svneh.east / r;
		MatrH[row][2] = svneh.head / r;

		if(SV_IsGps(svid))
			MatrH[row][3+svbuidx[0]]=1.0;
		else if(SV_IsBd2(svid))
			MatrH[row][3+svbuidx[1]]=1.0;
#if SUPPORT_GLONASS
		else if(SV_IsGlo(svid))
			MatrH[row][3+svbuidx[2]]=1.0;
#endif
				
		row++;
	}


	setMatrHandle(&hTransMatrH,MAX_LS_POS_COL, MAX_LS_ROW, dim_x, svcnt, (double*)TranMatrH);
	setMatrHandle(&hMatrTransHMH,MAX_LS_POS_COL, MAX_LS_POS_COL, dim_x, dim_x, (double*)MatrTransHMH);
	setMatrHandle(&hMatrG,MAX_LS_POS_COL, MAX_LS_POS_COL, dim_x, dim_x, (double*)MatrG);

	MatrixTranspose(&hMatrH, &hTransMatrH);
	MatrixMul(&hTransMatrH, &hMatrH, &hMatrTransHMH);

	if(MatrixInverse(&hMatrTransHMH, &hMatrG)!=0)
		return -1;

	if(dim_x == 4)
		pDop->gdop = sqrt(MatrG[0][0] + MatrG[1][1] + MatrG[2][2] + MatrG[3][3]);
	else if(dim_x == 5)
		pDop->gdop = sqrt(MatrG[0][0] + MatrG[1][1] + MatrG[2][2] + MatrG[3][3] + MatrG[4][4]);
#if SUPPORT_GLONASS
	else
		pDop->gdop = sqrt(MatrG[0][0] + MatrG[1][1] + MatrG[2][2] + MatrG[3][3] + MatrG[4][4] + MatrG[5][5]);
#endif	

	pDop->pdop = sqrt(MatrG[0][0] + MatrG[1][1] + MatrG[2][2]);
	pDop->vdop = sqrt(MatrG[2][2]);
	pDop->hdop = sqrt(MatrG[0][0] + MatrG[1][1]);

	
	if(gpssvcnt > 0)
	{
		idx = svbuidx[0]+3;
		pDop->gpstdop = sqrt(MatrG[idx][idx]);
	}

	if(bdsvcnt > 0)
	{
		idx = svbuidx[1]+3;
		pDop->bdtdop = sqrt(MatrG[idx][idx]);
	}

#if SUPPORT_GLONASS
	if(glosvcnt > 0)
	{
		idx = svbuidx[2]+3;
		pDop->glotdop = sqrt(MatrG[idx][idx]);
	}
#endif	

	return 0;
}


void setLSPosEquItrCnt(int32 cnt)
{
	FixEquItrCntThres = cnt;
}

void CalcSVPrFdResidue(int32 svid, ECEF* pRcvrpos,  float64* pBu, ECEF* pRcvrVel, float64* pCtu, PVT_TRKCH_INFO * pCurSVPVTInfo)
{
	float64 pseudorange=0.0, angle_er=0.0, disSV2U_Px=0.0, disSV2U_Py=0.0, disSV2U_Pz=0.0, disSV2U=0.0, Alpha_x=0.0, Alpha_y=0.0, Alpha_z=0.0;
	float64 Td=0.0, Ed=0.0;
	ECEF svpos={0.0,0.0,0.0}, svvel={0.0,0.0,0.0};

	if((pRcvrpos == NULL) || (pBu == NULL) || (pCurSVPVTInfo == NULL))
		return;
		
	pseudorange = pCurSVPVTInfo->pr;
	svpos = pCurSVPVTInfo->svpos;
	
	angle_er= -((pseudorange - (*pBu))*RECIP_SPEED_OF_LIGHT)*OMEGAE_WGS;

	disSV2U_Px = pRcvrpos->x - (svpos.x*cos(angle_er) - svpos.y*sin(angle_er));
	disSV2U_Py = pRcvrpos->y - (svpos.y*cos(angle_er) + svpos.x*sin(angle_er));
	disSV2U_Pz = pRcvrpos->z - svpos.z;

	disSV2U = sqrt(disSV2U_Px*disSV2U_Px + disSV2U_Py*disSV2U_Py + disSV2U_Pz*disSV2U_Pz);

	pCurSVPVTInfo->resiErr_pr = pseudorange - (disSV2U + (*pBu));

	Alpha_x = disSV2U_Px/disSV2U;
	Alpha_y = disSV2U_Py/disSV2U;
	Alpha_z = disSV2U_Pz/disSV2U;


	if((pRcvrVel != NULL) && (pCtu != NULL))
	{
		svvel = pCurSVPVTInfo->svvel;
		
		Td = 0.0;
		if(SV_IsGps(svid))
			Td = -Alpha_x * svvel.x - Alpha_y * svvel.y - Alpha_z * svvel.z + L1_WAVELENGTH * pCurSVPVTInfo->fd;
		else if(SV_IsBd2(svid))
			Td = -Alpha_x * svvel.x - Alpha_y * svvel.y - Alpha_z * svvel.z + B1_WAVELENGTH * pCurSVPVTInfo->fd;

		Ed = -Alpha_x * pRcvrVel->x -  Alpha_y * pRcvrVel->y - Alpha_z * pRcvrVel->z - (*pCtu);

		pCurSVPVTInfo->resiErr_fd = Td - Ed;
	}
	
	return;		
}

byte CheckMixFixSVAndBuFlag(PVT_TRKCH_INFO* pTRKChInfo, word256* pChnmap)
{
	byte buflag = 0;
#if SUPPORT_GLONASS
	int32 glousedcnt = 0;
	int32 glomaxcn0 = 0;
	int32 gloavgcn0 = 0;
	word256 gloUsedChnMap = {{0,}};
#endif
	int32 bdusedcnt = 0, gpsusedcnt = 0;
	int32 bdmaxcn0 = 0, gpsmaxcn0 = 0;
	int32 bdavgcn0 = 0, gpsavgcn0 = 0;
	word256 bdUsedChnMap = {{0,}}, gpsUsedChnMap = {{0,}};

	int32 trk_ch=0, svid=0, cn0=0;

	bMixFixMaybeErr = FALSE;
	for(trk_ch=0; trk_ch<MAXCHANNELS; trk_ch++)
	{
		svid = pTRKChInfo[trk_ch].svid;
		cn0 = pTRKChInfo[trk_ch].cn1s;
		if(GetBitWord256(pChnmap, trk_ch))
		{
			if(SV_IsBd2(svid))
			{
				SetBitWord256(&bdUsedChnMap, trk_ch);
				bdusedcnt++;
				if(cn0 > bdmaxcn0)
					bdmaxcn0 = cn0;

				bdavgcn0 += cn0;
			}
			else if(SV_IsGps(svid))
			{
				SetBitWord256(&gpsUsedChnMap, trk_ch);
				gpsusedcnt++;
				if(cn0 > gpsmaxcn0)
					gpsmaxcn0 = cn0;

				gpsavgcn0 += cn0;
			}
#if SUPPORT_GLONASS
			else if(SV_IsGlo(svid))
			{
				SetBitWord256(&gloUsedChnMap, trk_ch);
				glousedcnt++;
				if(cn0 > glomaxcn0)
					glomaxcn0 = cn0;

				gloavgcn0 += cn0;
			}
#endif
		}
	}

	if(gpsusedcnt > 0)
	{
		buflag = BITMAP_DIM_BU_GPS;
		gpsavgcn0 /= gpsusedcnt;
	}

	if(bdusedcnt > 0)
	{
		buflag |= BITMAP_DIM_BU_BD2;
		bdavgcn0 /= bdusedcnt;
	}
#if SUPPORT_GLONASS
	if(glousedcnt > 0)
	{
		buflag |= BITMAP_DIM_BU_GLO;
		gloavgcn0 /= glousedcnt;
	}
#endif
	if(getBDTime2GpsTimeBias(NULL, NULL) && (buflag & BITMAP_DIM_BU_GPS) && (buflag & BITMAP_DIM_BU_BD2))	
	{
		if((((bdusedcnt + gpsusedcnt) <= 6) || (gpsusedcnt<=2) || (bdusedcnt<=2) || (gpsmaxcn0<30) || (bdmaxcn0<30) || (gpsavgcn0<20) || (bdavgcn0<20))	//mix gps and bd
			||(((bdusedcnt + gpsusedcnt) <= 7) || (gpsusedcnt<=3) || (bdusedcnt<=3) || (gpsmaxcn0<40) || (bdmaxcn0<40)) )
		//if(SysEnvSta >= SYS_OPEN_SKY)
		{
			buflag &= ~BITMAP_DIM_BU_GPS;
			buflag &= ~BITMAP_DIM_BU_BD2;
			buflag |= BITMAP_DIM_BU_GPS_BD2;
		}
	}
	else if((SysEnvSta >= SYS_DOWN_TOWN) && (buflag & BITMAP_DIM_BU_GPS) && (buflag & BITMAP_DIM_BU_BD2))	//signal is poor, mix gps and bd
	{
		buflag &= ~BITMAP_DIM_BU_GPS;
		buflag &= ~BITMAP_DIM_BU_BD2;
		buflag |= BITMAP_DIM_BU_GPS_BD2;

		bMixFixMaybeErr = TRUE;
	}

#if SUPPORT_GLONASS
	if(getGloTime2GpsTimeBias(NULL, NULL) && (buflag & BITMAP_DIM_BU_GPS) && (buflag & BITMAP_DIM_BU_GLO))	
	{
		if((((glousedcnt + gpsusedcnt) <= 6) || (gpsusedcnt<=2) || (glousedcnt<=2) || (gpsmaxcn0<30) || (glomaxcn0<30) || (gpsavgcn0<20) || (gloavgcn0<20))	//mix gps and bd
			||(((glousedcnt + gpsusedcnt) <= 7) || (gpsusedcnt<=3) || (glousedcnt<=3) || (gpsmaxcn0<40) || (glomaxcn0<40)) )
		{
			buflag &= ~BITMAP_DIM_BU_GPS;
			buflag &= ~BITMAP_DIM_BU_GLO;
			buflag |= BITMAP_DIM_BU_GPS_GLO;
		}
	}
	else if((SysEnvSta >= SYS_DOWN_TOWN) && (buflag & BITMAP_DIM_BU_GPS) && (buflag & BITMAP_DIM_BU_GPS_GLO))	//signal is poor, mix gps and glo
	{
		buflag &= ~BITMAP_DIM_BU_GPS;
		buflag &= ~BITMAP_DIM_BU_GLO;
		buflag |= BITMAP_DIM_BU_GPS_GLO;
		bMixFixMaybeErr = TRUE;
	}

	if(getGloTime2BdTimeBias(NULL, NULL) && (buflag & BITMAP_DIM_BU_BD2) && (buflag & BITMAP_DIM_BU_GLO))	
	{
		if((((glousedcnt + bdusedcnt) <= 6) || (bdusedcnt<=2) || (glousedcnt<=2) || (bdmaxcn0<30) || (glomaxcn0<30) || (bdavgcn0<20) || (gloavgcn0<20))	//mix gps and bd
			||(((glousedcnt + bdusedcnt) <= 7) || (bdusedcnt<=3) || (glousedcnt<=3) || (bdmaxcn0<40) || (glomaxcn0<40)) )
		{
			buflag &= ~BITMAP_DIM_BU_BD2;
			buflag &= ~BITMAP_DIM_BU_GLO;
			buflag |= BITMAP_DIM_BU_BD2_GLO;
		}
	}
	else if((SysEnvSta >= SYS_DOWN_TOWN) && (buflag & BITMAP_DIM_BU_BD2) && (buflag & BITMAP_DIM_BU_GPS_GLO))	//signal is poor, mix gps and glo
	{
		buflag &= ~BITMAP_DIM_BU_BD2;
		buflag &= ~BITMAP_DIM_BU_GLO;
		buflag |= BITMAP_DIM_BU_BD2_GLO;
		bMixFixMaybeErr = TRUE;
	}
#endif

	if((buflag & BITMAP_DIM_BU_GPS) && (gpsusedcnt == 1) && (gpsmaxcn0 <40))
	{
		buflag &= ~BITMAP_DIM_BU_GPS;
		gpsUsedChnMap = OpReverseWord256(&gpsUsedChnMap);
		OpAndWord256(pChnmap, &gpsUsedChnMap);
	}

	if((buflag & BITMAP_DIM_BU_GPS) && (bdusedcnt == 1) && (bdmaxcn0 <40))	//SysEnvSta is good, delete only one SV navsystem with poor signal.
	{
		buflag &= ~BITMAP_DIM_BU_BD2;
		bdUsedChnMap = OpReverseWord256(&bdUsedChnMap);
		OpAndWord256(pChnmap, &bdUsedChnMap);
	}

#if SUPPORT_GLONASS
	if((buflag & BITMAP_DIM_BU_GLO) && (glousedcnt == 1) && (glomaxcn0 <40))	//SysEnvSta is good, delete only one SV navsystem with poor signal.
	{
		buflag &= ~BITMAP_DIM_BU_GLO;
		gloUsedChnMap = OpReverseWord256(&gloUsedChnMap);
		OpAndWord256(pChnmap, &gloUsedChnMap);
	}
#endif	
	return buflag;
}



