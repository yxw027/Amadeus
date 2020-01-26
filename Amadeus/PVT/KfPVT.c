#include "define.h"

#if (PVT_MODE == CALC_PVT_KF)

#include "define.h"
#include "matrix.h"
#include "KfPVT.h"
#include "LsPVT.h"
#include "coordinate.h"
#include "PVT.h"
#include "PVTRcvrInfo.h"
#include <math.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "postproc.h"
#include "global.h"
#include "constdef.h"


/*****************variables define****************************/
static double VectorX[PVTKF_X_DIM];
//static TMatrix hVectorX={PVTKF_X_DIM,1,PVTKF_X_DIM,1, (double*)VectorX};			//8*1

static double VectorDeltaZ[MAX_Z_DIM];				//Z(k)-Z(k,k-1)
//static TMatrix hVectorDeltaZ={MAX_Z_DIM,1,MAX_Z_DIM,1, (double*)VectorDeltaZ};			// 2N*1

static double MatrixQ[PVTKF_X_DIM][PVTKF_X_DIM];
static TMatrix hMatrixQ={PVTKF_X_DIM,PVTKF_X_DIM,PVTKF_X_DIM,PVTKF_X_DIM, (double*)MatrixQ};		//8*8

//static double MatrixQEnu[PVTKF_X_DIM][PVTKF_X_DIM];
//static TMatrix hMatrixQEnu={PVTKF_X_DIM,PVTKF_X_DIM,PVTKF_X_DIM,PVTKF_X_DIM, (double*)MatrixQEnu};		//8*8

static double MatrixP[PVTKF_X_DIM][PVTKF_X_DIM];
static TMatrix hMatrixP={PVTKF_X_DIM,PVTKF_X_DIM,PVTKF_X_DIM,PVTKF_X_DIM, (double*)MatrixP};		//8*8

static double MatrixPhi[PVTKF_X_DIM][PVTKF_X_DIM];
static TMatrix hMatrixPhi={PVTKF_X_DIM,PVTKF_X_DIM,PVTKF_X_DIM,PVTKF_X_DIM, (double*)MatrixPhi};	//8*8

static double MatrixH[MAX_Z_DIM][PVTKF_X_DIM];
static TMatrix hMatrixH={MAX_Z_DIM,PVTKF_X_DIM,MAX_Z_DIM,PVTKF_X_DIM, (double*)MatrixH};     // 2N*8
static double VectorR[MAX_Z_DIM];
static TVector hVectorR={MAX_Z_DIM, MAX_Z_DIM, VectorR};	// 2N*1


static double MatrixK[PVTKF_X_DIM][MAX_Z_DIM];
static TMatrix hMatrixK={PVTKF_X_DIM,MAX_Z_DIM,PVTKF_X_DIM,MAX_Z_DIM, (double*)MatrixK};	// 8*2N



static boolean bMustbeMove = TRUE;
boolean bPVTKFValid = FALSE;

static void InitStateVectorX(PVT_FIX_INFO* kfpvtinfo0);
static void UpdateEstiX(double cycle);
static void UpdateMatrixEstiP(void);
static int32 UpdateH_AllSVDeltaZ_R(PVT_TRKCH_INFO* pSVPVTInfo, PVT_FIX_INFO* kfpvtinfo);
static boolean UpdateMatrixK(int32 ZLength, int32 measureidx);
static void UpdateVectorX(int32 ZLength, int32 measureidx);
static boolean UpdateMatrixP(int32 ZLength, int32 measureidx);
static void UpdateMatrixQ(double cycle);
static void UpdateMatrixPhi(double cycle);

static void UpdateResiErrAndAlpha(byte buflag, word256* pChnmap, PVT_TRKCH_INFO* pSVPVTInfo);
static void SelectSVToFix(PVT_TRKCH_INFO* pSVPVTInfo, word256* pPosChnMap, word256* pVelChnMap);
static int32 UpdateMatrixH_Z_R(byte buflag, PVT_TRKCH_INFO* pSVPVTInfo, word256* pFixChnMap, word256* pVelChnMap);
static void UpdateProcessNoise(int32 index, ECEF pos, TMatrix* hMatrixECEFNoise);

static void MatrixMulHPHT(TMatrix* pMatrixH, TMatrix* pMatrixP, TMatrix* pMatrixHPHT);

static void checkKfSVBuIdx(byte buflag, int8 navSysBuIdx[3]);
static void InitKFModule(void);

byte KFSvPosCnt = 0, KFSvVelCnt = 0;
static byte KFSvPosCnt_GPS = 0, KFSvPosCnt_BD = 0, KFSvPosCnt_GLO = 0; 
byte KalmanNoiseIndex = 0, KalmanCnt = 0;
byte ConKFPVTReckonCnt = 0;

static PR_MODIFY PRModify[3];

KALMAN_TRKCH_INFO KFChnInfo[MAXCHANNELS_PVT];

#if 0
const KALMAN_NOISE KalmanNoise[KALMAN_NOISE_NUM] = 
{
	/*MaxSpeed,		PosENError,		PosUError, 	ClkDeltaFreq,		ClkDeltaPhase,	CodeLoopErr,		FreqLoopErr*/
	{0.05,			1.0e-8,			1.0e-8,		0.18973,			13.464,			3.0,				0.03},
	{0.25,			0.45,			1.0e-3,		0.18973,			13.464,			7.0,				0.05},
	{1.0,			0.5,				2.5e-3,		0.18973,			13.464,			7.0,				0.05},
	{2.0,			0.5,				5.0e-3,		0.18973,			13.464,			7.0,				0.05},
	{5.0,			0.6,				8.0e-3,		0.18973,			13.464,			7.1,				0.051},
	{8.0,			0.7,				1.0e-2,		0.18973,			13.464,			7.5,				0.06},
	{16.0,			0.8,				9.0e-2,		0.18973,			13.464,			8.0,				0.07},
	{100.0,			0.8,				9.0e-2,		0.18973,			13.464,			8.0,				0.07},	
};

const KALMAN_NOISE KalmanNoise[KALMAN_NOISE_NUM] = 
{
	/*MaxSpeed,		PosENError,		PosUError, 	ClkDeltaFreq,		ClkDeltaPhase,	CodeLoopErr,		FreqLoopErr*/
	{0.05,			1.0e-8,			1.0e-8,		0.18973,			13.464,			2.5,				0.04},
	{0.25,			0.45,			1.0e-3,		0.18973,			13.464,			3.0,				0.05},
	{1.0,			0.5,				2.5e-3,		0.18973,			13.464,			3.0,				0.05},
	{2.0,			0.5,				5.0e-3,		0.18973,			13.464,			3.0,				0.05},
	{5.0,			0.6,				8.0e-3,		0.18973,			13.464,			3.0,				0.05},
	{8.0,			0.7,				1.0e-2,		0.18973,			13.464,			3.5,				0.06},
	{16.0,			0.8,				9.0e-2,		0.18973,			13.464,			4.0,				0.07},
	{100.0,			0.8,				9.0e-2,		0.18973,			13.464,			4.0,				0.07},	
};
#else

#if 0
const KALMAN_NOISE KalmanNoise[KALMAN_NOISE_NUM] = 
{
	/*MaxSpeed,		PosENError,		PosUError, 	ClkDeltaFreq,		ClkDeltaPhase,	CodeLoopErr,		FreqLoopErr*/
	{0.05,			1.0e-8,			1.0e-8,		0.18973,			13.464,			2.5,				0.04},
	{0.25,			0.45,			1.0e-3,		0.18973,			13.464,			3.0,				0.05},
	{1.0,			0.6,				5.0e-3,		0.18973,			13.464,			3.0,				0.05},
	{2.0,			0.6,				8.0e-3,		0.18973,			13.464,			3.0,				0.05},
	{5.0,			0.8,				1.0e-2,		0.18973,			13.464,			3.0,				0.05},
	{8.0,			0.9,				5.0e-2,		0.18973,			13.464,			3.5,				0.06},
	{16.0,			1.0,				1.0e-1,		0.18973,			13.464,			4.0,				0.07},
	{100.0,			1.2,				5.0e-1,		0.18973,			13.464,			4.0,				0.07},	
};
#else
const KALMAN_NOISE KalmanNoise[KALMAN_NOISE_NUM] = 
{
	/*MaxSpeed,		PosENError,		PosUError, 	ClkDeltaFreq,		ClkDeltaPhase,	CodeLoopErr,		FreqLoopErr*/
	{0.1,			1.0e-1,			1.0e-3,		0.18973,			13.464,			7.0,				0.14},
	{0.35,			0.3,			1.0e-3,		0.18973,			13.464,			7.0,				0.15},
	{0.65,			0.45,			1.0e-3,		0.18973,			13.464,			7.0,				0.15},
	{1.0,			0.6,				5.0e-3,		0.18973,			13.464,			7.0,				0.15},
	{2.0,			0.6,				8.0e-3,		0.18973,			13.464,			7.0,				0.15},
	{5.0,			0.8,				1.0e-2,		0.18973,			13.464,			7.0,				0.20},
	{8.0,			0.9,				5.0e-2,		0.18973,			13.464,			7.5,				0.20},
	{16.0,			1.0,				1.0e-1,		0.18973,			13.464,			8.0,				0.20},
	{100.0,			1.2,				5.0e-1,		0.18973,			13.464,			8.0,				0.20},	
};
#endif

#endif

const double CoVarianceCN0Tbl[] = 
{
	// 1.0e4 / (power(10.0, cn0 /10))
	1584.89319246111  ,	1258.92541179417  ,	            1000  ,	794.328234724281  ,	630.957344480193  ,		// 8 ~ 12
	501.187233627272  ,	398.107170553497  ,	316.227766016838  ,	251.188643150958  ,	199.526231496888  ,	// 13 ~ 17
	158.489319246111  ,	125.892541179417  ,	             100  ,	79.4328234724281  ,	63.0957344480193  ,		// 18 ~ 22
	50.1187233627273  ,	39.8107170553497  ,	31.6227766016838  ,	25.1188643150958  ,	19.9526231496888  ,	// 23 ~ 27
	15.8489319246111  ,	12.5892541179417  ,	              10  ,	7.94328234724281  ,	6.30957344480193  ,			// 28 ~ 32
	5.01187233627272  ,	3.98107170553497  ,	3.16227766016838  ,	2.51188643150958  ,	1.99526231496888  ,	// 33 ~ 37
	1.58489319246111  ,	1.25892541179417  ,	               1  ,	0.794328234724282 ,	0.630957344480193 ,			// 38 ~ 42
	0.501187233627272 ,	0.398107170553497 ,	0.316227766016838 ,	0.251188643150958 ,	0.199526231496888 ,	// 43 ~ 47
	0.158489319246111 ,	0.125892541179417 ,	              0.1 ,	0.0794328234724282,	0.0630957344480193,			// 48 ~ 52
	0.0501187233627273,	0.0398107170553497,	0.0316227766016838,	0.0251188643150958,	0.0199526231496888,	// 53 ~ 57
};

void ResetPVTKF(void)
{
	bPVTKFValid = FALSE;
	ConKFPVTReckonCnt = 0;

	return;	
}

void InitKFModule(void)
{
	memset(VectorX, 0, sizeof(VectorX));
	memset(VectorDeltaZ, 0, sizeof(VectorDeltaZ));
	memset(MatrixQ, 0, sizeof(MatrixQ));
	memset(MatrixP, 0, sizeof(MatrixP));
	memset(MatrixPhi, 0, sizeof(MatrixPhi));
	memset(MatrixH, 0, sizeof(MatrixH));
	memset(VectorR, 0, sizeof(VectorR));
	memset(MatrixK, 0, sizeof(MatrixK));
}

/*****************filter process******************************/
void InitPVTKF(PVT_FIX_INFO* kfpvtinfo0, boolean bInitP)
{	
	byte bumap = kfpvtinfo0->buflag;
	double initP = 225;	// 15^2
	double hdop3 = 0.0;
	double bias = 0.0;
	PVT_FIX_INFO fixinfo0;
	memcpy(&fixinfo0, kfpvtinfo0, sizeof(fixinfo0));
	
	//init P
	if(bInitP)
	{
		InitKFModule();
		hdop3 = fixinfo0.dop.hdop * fixinfo0.dop.hdop * fixinfo0.dop.hdop;
		
		EyeMatrix(&hMatrixP,initP*hdop3);

		//if no GPS bu
		if(!((bumap & BITMAP_DIM_BU_GPS) || (bumap & BITMAP_DIM_BU_GPS_BD2) || (bumap & BITMAP_DIM_BU_GPS_GLO) || (bumap & BITMAP_DIM_BU_GPS_BD2_GLO)))
		{
			if((bumap & BITMAP_DIM_BU_BD2) || (bumap & BITMAP_DIM_BU_BD2_GLO))
			{
				if(getBDTime2GpsTimeBias(&bias, NULL))
					fixinfo0.gpsbu = fixinfo0.bd2bu - bias;
				else
					fixinfo0.gpsbu = fixinfo0.bd2bu;
			}
		#if SUPPORT_GLONASS
			else if(bumap & BITMAP_DIM_BU_GLO)
			{

				if(getGloTime2GpsTimeBias(&bias, NULL))
					fixinfo0.gpsbu = fixinfo0.globu - bias;
				else
					fixinfo0.gpsbu = fixinfo0.globu;
			}
		#endif

			MatrixP[GPS_BU_IDX][GPS_BU_IDX] = 1.0e8;
		}

		//if no BD bu
		if(!((bumap & BITMAP_DIM_BU_BD2) || (bumap & BITMAP_DIM_BU_GPS_BD2) || (bumap & BITMAP_DIM_BU_BD2_GLO) || (bumap & BITMAP_DIM_BU_GPS_BD2_GLO)))
		{
			if((bumap & BITMAP_DIM_BU_GPS) || (bumap & BITMAP_DIM_BU_GPS_GLO))
			{
				if(getBDTime2GpsTimeBias(&bias, NULL))
					fixinfo0.bd2bu = fixinfo0.gpsbu + bias;
				else
					fixinfo0.bd2bu = fixinfo0.gpsbu;
			}
		#if SUPPORT_GLONASS
			else if(bumap & BITMAP_DIM_BU_GLO)
			{

				if(getGloTime2BdTimeBias(&bias, NULL))
					fixinfo0.bd2bu = fixinfo0.globu - bias;
				else
					fixinfo0.bd2bu = fixinfo0.globu;
			}
		#endif

			MatrixP[BD_BU_IDX][BD_BU_IDX] = 1.0e8;
		}
		
		//if no Glo bu
		#if SUPPORT_GLONASS
		if(!((bumap & BITMAP_DIM_BU_GLO) || (bumap & BITMAP_DIM_BU_GPS_GLO) || (bumap & BITMAP_DIM_BU_BD2_GLO) || (bumap & BITMAP_DIM_BU_GPS_BD2_GLO)))
		{
			if((bumap & BITMAP_DIM_BU_GPS) || (bumap & BITMAP_DIM_BU_GPS_BD2))
			{
				if(getGloTime2GpsTimeBias(&bias, NULL))
					fixinfo0.globu= fixinfo0.gpsbu + bias;
				else
					fixinfo0.globu= fixinfo0.gpsbu;
			}
			else if(bumap & BITMAP_DIM_BU_BD2)
			{
				if(getGloTime2BdTimeBias(&bias, NULL))
					fixinfo0.globu = fixinfo0.bd2bu + bias;
				else
					fixinfo0.globu = fixinfo0.bd2bu;
			}

			MatrixP[GLO_BU_IDX][GLO_BU_IDX] = 1.0e8;
		}
		#endif
	}
		
	//init X0	
	InitStateVectorX(&fixinfo0);	

	bPVTKFValid = TRUE;
}

void InitPVTKFPos(ECEF pos)
{
	VectorX[0] = pos.x;
	VectorX[1] = pos.y;
	VectorX[2] = pos.z;
}


void InitKFPVTMatrixP(double pValue)
{
	EyeMatrix(&hMatrixP,pValue);

	return;
}

void InitStateVectorX(PVT_FIX_INFO* kfpvtinfo0)
{
	VectorX[0] = kfpvtinfo0->rcvrpos.x;
	VectorX[1] = kfpvtinfo0->rcvrpos.y;
	VectorX[2] = kfpvtinfo0->rcvrpos.z;
	VectorX[3] = kfpvtinfo0->rcvrvel.x;
	VectorX[4] = kfpvtinfo0->rcvrvel.y;
	VectorX[5] = kfpvtinfo0->rcvrvel.z;
	VectorX[GPS_BU_IDX] = kfpvtinfo0->gpsbu;
	VectorX[BD_BU_IDX] = kfpvtinfo0->bd2bu;
#if SUPPORT_GLONASS
	VectorX[GLO_BU_IDX] = kfpvtinfo0->globu;
#endif
	VectorX[CTU_IDX] = kfpvtinfo0->ctu;

	return;
}

void KalmanPredict(double cycle)
{
	//update Phi and Q by T
	UpdateMatrixPhi(cycle);

	//step1 X(k,k-1)=Phi*X(k-1,k-1)
	UpdateEstiX(cycle);

	UpdateMatrixQ(cycle);

	//step2 P(k,k-1)=Phi(k)*P(k-1,k-1)*Trans(Phi(k))+Q(k)
	UpdateMatrixEstiP();
	
	return;
}

boolean KalmanUpdate(int32 MsrNum, int32 MsrIdx)
{	
	boolean ret = FALSE;
#if(KALMAN_UPDATE_FLOW == KALMAN_UPDATE_K_FIRST)
	if(UpdateMatrixK(MsrNum, MsrIdx))		//step5 update K,   K(k)=P(k,k-1)*Trans(H(k))*Invers[H(k)*P(k,k-1)*Trans(H(k))+R(k)]
	{
		//step6 P(k)=(I-K(k)*H(k))*P(k,k-1)
		UpdateMatrixP(MsrNum, MsrIdx);
		
		//step7 X(k,k) = X(k,k-1) + K*(Z(k)-Z(k,k-1))  , DeltaZ = Z(k)-Z(k,k-1)
		UpdateVectorX(MsrNum, MsrIdx);

		ret = TRUE;
	}
#else
	if(UpdateMatrixP(MsrNum, MsrIdx))		//step5 update P,   P(k) = inv(H' * inv(R) * H + inv(P(k, k-1)))
	{
		//step6 K(k) = P(k) * H' * inv(R)
		UpdateMatrixK(MsrNum, MsrIdx);
		
		//step7 X(k,k) = X(k,k-1) + K*(Z(k)-Z(k,k-1))  , DeltaZ = Z(k)-Z(k,k-1)
		UpdateVectorX(MsrNum, MsrIdx);

		ret = TRUE;
	}
#endif

	return ret;
}

void SaveKalmanResult(int32 filtertype, PVT_TRKCH_INFO* pSVPVTInfo, PVT_FIX_INFO* kfpvtinfo)
{
	int32 trkedsvcnt = 0;
	int32 i=0, dim_pos=3;
	int8 svbuidx[3] = {0,0,0};
	int32 trkch = 0, row1 = 0, row2 = KFSvPosCnt;

	for(i=0; i<MAXCHANNELS; i++)
	{
		if(((pSVPVTInfo[i].CCBF & 0x3) == 0x3) && (pSVPVTInfo[i].bPRValid))
			trkedsvcnt++;
	}
	
	//update kalman reckon count
	if(filtertype == FILTER_TYPE_EXTEND)
		ConKFPVTReckonCnt = 0;
	else
	{
		if(trkedsvcnt < 3)
			ConKFPVTReckonCnt += MAX_KFPVT_RECKON_CNT;
		//else if(KFSvPosCnt <= 1)
		//	ConKFPVTReckonCnt += 4;
		else if(KFSvPosCnt <= 3)
			ConKFPVTReckonCnt += 2;
		else
			ConKFPVTReckonCnt += 1;
		
		if(ConKFPVTReckonCnt > MAX_KFPVT_RECKON_CNT)
		{	
			ConKFPVTReckonCnt = 0;
			bPVTKFValid = FALSE;
		}
	}
		
	//save state vector
	kfpvtinfo->rcvrpos.x = VectorX[0];
	kfpvtinfo->rcvrpos.y = VectorX[1];
	kfpvtinfo->rcvrpos.z = VectorX[2];
	kfpvtinfo->rcvrvel.x = VectorX[3];
	kfpvtinfo->rcvrvel.y = VectorX[4];
	kfpvtinfo->rcvrvel.z = VectorX[5];

	checkKfSVBuIdx(kfpvtinfo->buflag,svbuidx);
	
	kfpvtinfo->gpsbu = VectorX[svbuidx[0]];

	kfpvtinfo->bd2bu = VectorX[svbuidx[1]];
	if(PRModify[1].bPRModify)
		kfpvtinfo->bd2bu += PRModify[1].bias;

	if(kfpvtinfo->buflag & BITMAP_DIM_BU_GPS_BD2)
		VectorX[BD_BU_IDX] = kfpvtinfo->bd2bu;

#if SUPPORT_GLONASS
	kfpvtinfo->globu = VectorX[svbuidx[2]];
	if(PRModify[2].bPRModify)
		kfpvtinfo->globu += PRModify[2].bias;
#endif
	
	kfpvtinfo->ctu = VectorX[CTU_IDX];

	for(i=0; i<7; i++)
	{
		if(kfpvtinfo->buflag & (1<<i))
			dim_pos++;
	}

	if((KFSvPosCnt_GPS + KFSvPosCnt_BD + KFSvPosCnt_GLO < dim_pos) || (filtertype == FILTER_TYPE_RECKON))
		kfpvtinfo->posres = FIX_2D;
	else
		kfpvtinfo->posres = FIX_3D;

	kfpvtinfo->velres = kfpvtinfo->posres;

	kfpvtinfo->filtertype = filtertype;

	//save pr/freq covariance and pr/freq residue
	for(trkch = 0; trkch < MAXCHANNELS_PVT; trkch ++)
	{
		KFChnInfo[trkch].svid = pSVPVTInfo[trkch].svid;
		KFChnInfo[trkch].cn0 = pSVPVTInfo[trkch].cn1s;

		if(GetBitWord256(&(kfpvtinfo->poschmap),trkch)==1)
		{
			KFChnInfo[trkch].prcovariance = VectorR[row1];
			row1 ++;
		}

		if(GetBitWord256(&(kfpvtinfo->velchmap),trkch)==1)
		{
			KFChnInfo[trkch].freqcovariance = VectorR[row2];
			row2 ++;
		}

		KFChnInfo[trkch].prresidue = pSVPVTInfo[trkch].resiErr_pr;
		KFChnInfo[trkch].freqresidue = pSVPVTInfo[trkch].resiErr_fd;
	}

	return;
}

boolean PostKalmanResidueCheck(PVT_TRKCH_INFO* pSVPVTInfo, word256* pFixChnMap, word256* pVelChnMap, int32* pErrPosCnt, int32* pErrVelCnt)
{
	int32 trkch = 0, posidx = 0, velidx = KFSvPosCnt;
	double PRPostRMS = 0.0, FreqPostRMS = 0.0;
	double meanPrResidue = 0.0, meanFdResidue = 0.0;
	double prresidue = 0.0, fdresidue = 0.0;
	double ratio = 0.0;
	
	boolean bupdate = FALSE;
	int32 measurecnt = hMatrixH.rows;

	double MatrixTemp[MAX_Z_DIM][MAX_Z_DIM] = {{0,},};
	TMatrix hMatrixTemp;

	setMatrHandle(&hMatrixTemp, MAX_Z_DIM, MAX_Z_DIM, measurecnt, measurecnt, (double*)MatrixTemp);		// R - H*P*H'

	MatrixMulHPHT(&hMatrixH, &hMatrixP, &hMatrixTemp);
	VectorSubMatrix(&hVectorR, &hMatrixTemp,  &hMatrixTemp);	//R - H * P * H'

	UpdateResiErrAndAlpha((BITMAP_DIM_BU_GPS|BITMAP_DIM_BU_BD2|BITMAP_DIM_BU_GLO), pFixChnMap, pSVPVTInfo);		//update residue after kalman time update

	for(trkch = 0; trkch < MAXCHANNELS_PVT; trkch ++)
	{
		if(GetBitWord256(pFixChnMap,trkch)==1)
		{
			prresidue = fabs(pSVPVTInfo[trkch].resiErr_pr);
			
			meanPrResidue += prresidue;
			PRPostRMS += prresidue * prresidue / (MatrixTemp[posidx][posidx]);						
			posidx ++;
		}

		if(GetBitWord256(pVelChnMap,trkch)==1)
		{
			fdresidue = fabs(pSVPVTInfo[trkch].resiErr_fd);

			meanFdResidue += fdresidue;
			FreqPostRMS = fdresidue * fdresidue / (MatrixTemp[velidx][velidx]);	
			velidx ++;
		}
	}

	if(KFSvPosCnt != 0)
	{
		meanPrResidue = meanPrResidue / KFSvPosCnt;
		PRPostRMS = sqrt(PRPostRMS / KFSvPosCnt);
	}

	if(KFSvVelCnt != 0)
	{
		meanFdResidue = meanFdResidue / KFSvVelCnt;
		FreqPostRMS = sqrt(FreqPostRMS / KFSvVelCnt);
	}
		
	 posidx = 0, velidx = KFSvPosCnt;
	for(trkch = 0; trkch < MAXCHANNELS_PVT; trkch ++)
	{
		if(GetBitWord256(pFixChnMap,trkch)==1)
		{
			prresidue = fabs(pSVPVTInfo[trkch].resiErr_pr);
			if(MatrixTemp[posidx][posidx] <= 0)
				continue;
			ratio = prresidue / sqrt(MatrixTemp[posidx][posidx]);

			#if 0
			if((ratio > 10.0) && (prresidue > 15.0))
			{
				(*pErrPosCnt) ++;
				VectorR[posidx] *= 1.0e10;
			}
			else if(ratio > 3.0)
			{
				(*pErrPosCnt) ++;
				if(prresidue > 5.0)
					VectorR[posidx] *= (5.0 * prresidue);
				else if(prresidue > 1.0)
					VectorR[posidx] *= prresidue;//(5.0);
			}			
			#else
			#if 0
			if((ratio > 20) /*&& (prresidue > 15.0)*/)
			{
				(*pErrPosCnt) ++;
				VectorR[posidx] *= 20;
			}
			
			 if(prresidue > 120)
				VectorR[posidx] *= 1e5;
			else if((prresidue > 2.0 * PRPostRMS) && (PRPostRMS > 2.0))
				VectorR[posidx] *= 1e3;			
			#else
			if(ratio > 3.0) // 3*sigma, N(0,1)
			{
				if((prresidue > 50) && (prresidue > 3 * meanPrResidue))
				{
					bupdate = TRUE;
					(*pErrPosCnt) ++;
					VectorR[posidx] *= 1e5;
				 }
				else if((prresidue > 20.0) && (prresidue > 2 * meanPrResidue))
				{
					bupdate = TRUE;
					VectorR[posidx] *= prresidue;	
				}
			}							
			#endif
			#endif
			
			posidx ++;
		}

		if(GetBitWord256(pVelChnMap,trkch)==1)
		{
			fdresidue = fabs(pSVPVTInfo[trkch].resiErr_fd);

			if(MatrixTemp[velidx][velidx] <= 0)
				continue;
			ratio = fdresidue / sqrt(MatrixTemp[velidx][velidx]);

			#if 0
			if((ratio > 16.0) && (fdresidue > 10.0) && (VectorR[velidx]) < 1.0e5)
			{
				(*pErrVelCnt) ++;
				VectorR[velidx] *= 1.0e10;
			}
			else if(ratio > 3.0)
			{				
				if(fdresidue > 1.0)
				{
					(*pErrVelCnt) ++;
					VectorR[velidx] *= (fdresidue);
				}
			}			
			#else
			#if 0
			if(ratio > 3.0)
			{
				(*pErrVelCnt) ++;
				VectorR[velidx] *= 1e2;
			}

			if(fdresidue >= 6.0)
			{
				if(KFSvVelCnt > 4)
					VectorR[velidx] *= 1e4;
				else
					VectorR[velidx] *= 1e5;
			}
			else if((fdresidue >= 2.0 * FreqPostRMS) && ((FreqPostRMS < 0.2) || (FreqPostRMS > 2.0)))
				VectorR[velidx] *= 1e3;
			else if((fdresidue >= 6.0) && (pPVTData[i].cn0 < 26))
				VectorR[velidx] *= 1e3;
			#else			
			if(ratio > 3.0)	 // 3*sigma, N(0,1)
			{			
				if((fdresidue >= 6.0) && (fdresidue > 3*meanFdResidue))
				{
					bupdate = TRUE;
					(*pErrVelCnt) ++;
					VectorR[velidx] *= 1e3;
				}
				else if((fdresidue > 3.0) && (fdresidue > 2*meanFdResidue))
				{
					bupdate = TRUE;
					VectorR[velidx] *= fdresidue;				
				}
			}										
			#endif
			#endif
		
			velidx ++;
		}
	}
	
	return bupdate;
}

void KalmanFilterPVT(double cycle, PVT_TRKCH_INFO* pSVPVTInfo ,PVT_FIX_INFO* kfpvtinfo)
{
	int32 filterType = FILTER_TYPE_RECKON;

	int32 ZLength = 0;
	double backupStateVector[PVTKF_X_DIM], backupMatrixP[PVTKF_X_DIM][PVTKF_X_DIM];
	ECEF pos = {0.0,};
	int32 i=0, dim_pos=3;
	int32 iteratorcnt = 0, svposcnt = KFSvPosCnt, svvelcnt = KFSvVelCnt;
	int32 measureidx = 0;
	int32 errposcnt = 0, errvelcnt = 0;
	//boolean bupdate = FALSE;
	
	KalmanPredict(cycle);

	KalmanCnt = 0;
	KFSvPosCnt = KFSvVelCnt = 0;
	KFSvPosCnt_GPS = KFSvPosCnt_BD = KFSvPosCnt_GLO = 0;
	memset(PRModify, 0, sizeof(PRModify));
	

	ZLength = UpdateH_AllSVDeltaZ_R(pSVPVTInfo, kfpvtinfo);

	memcpy(backupStateVector, VectorX, sizeof(VectorX));
	memcpy(backupMatrixP, MatrixP, sizeof(MatrixP));

	pos.x = VectorX[0];
	pos.y = VectorX[1];
	pos.z = VectorX[2];

	for(i=0; i<7; i++)
	{
		if(kfpvtinfo->buflag & (1<<i))
			dim_pos++;
	}

	if(KFSvPosCnt >= dim_pos)
	{
		//if(kfpvtinfo->poschmap != kfpvtinfo->dopchnmap)
		{
			CalcFixDOP(kfpvtinfo->buflag, &(kfpvtinfo->poschmap), pSVPVTInfo, pos, &(kfpvtinfo->dop));
			//kfpvtinfo->dopchnmap = kfpvtinfo->poschnmap;
		}

		if(kfpvtinfo->dop.hdop < 10.0)
		{
			while(iteratorcnt < 2)
			{
				measureidx = 0;
				#if(SERIAL_KALMAN_EN)
				#if(0)
				while(measureidx < ZLength)
				{
					if(KalmanUpdate(1, measureidx))
					{
						filterType = FILTER_TYPE_EXTEND;
						measureidx ++;
					}
					else
					{
						bPVTKFValid = FALSE;	
						break;
					}
				}
				#else
				if(KalmanUpdate(KFSvPosCnt, measureidx))
				{
					measureidx += KFSvPosCnt;
				}
				else
				{
					bPVTKFValid = FALSE;	
					break;
				}

				if(KalmanUpdate(KFSvVelCnt, measureidx))
				{
					filterType = FILTER_TYPE_EXTEND;
					measureidx += KFSvVelCnt;
				}
				else
				{
					bPVTKFValid = FALSE;	
					break;
				}					
				#endif
				#else			
				if(KalmanUpdate(ZLength, measureidx))
				{
					KalmanCnt ++;
					filterType = FILTER_TYPE_EXTEND;
				}
				else
					bPVTKFValid = FALSE;	
				#endif

				if((svposcnt <= SV_NUM_FIX_3D + 1) || (svvelcnt <= SV_NUM_FIX_3D + 1) || (KalmanNoiseIndex <= 2))
					break;

				errposcnt = errvelcnt = 0;
				#if(POST_RESIDUE_CHECK_EN)
				//bupdate = PostKalmanResidueCheck(pSVPVTInfo, &(kfpvtinfo->poschmap), &(kfpvtinfo->velchmap), &errposcnt, &errvelcnt);
				PostKalmanResidueCheck(pSVPVTInfo, &(kfpvtinfo->poschmap), &(kfpvtinfo->velchmap), &errposcnt, &errvelcnt);
				#endif
				
				svposcnt -= errposcnt;
				svvelcnt -= errvelcnt;
				
				//if((!bupdate) || (svposcnt <= SV_NUM_FIX_3D + 1) || (svvelcnt <= SV_NUM_FIX_3D + 1))
				if((errposcnt == 0) && (errvelcnt == 0))
					break;
				
				//restore kalman state, X / P
				memcpy(VectorX, backupStateVector, sizeof(VectorX));
				memcpy(MatrixP, backupMatrixP, sizeof(MatrixP));
				iteratorcnt ++;
			}
		}
	}

	SaveKalmanResult(filterType, pSVPVTInfo, kfpvtinfo);
	
	return ;
}


void UpdateEstiX(double cycle)
{	
	int32 i=0;
	
	//EX: X(k,k-1)=Phi*X(k-1,k-1)

	VectorX[0] += VectorX[3]*cycle;
	VectorX[1] += VectorX[4]*cycle;
	VectorX[2] += VectorX[5]*cycle;

	for(i=GPS_BU_IDX; i<PVTKF_X_DIM-1; i++)
		VectorX[i] += VectorX[CTU_IDX]*cycle;

	return;
}


void UpdateMatrixEstiP(void)
{
	//P(k,k-1)=Phi(k)*P(k-1,k-1)*Trans(Phi(k))+Q(k)

	double MatrPhiP[PVTKF_X_DIM][PVTKF_X_DIM];
	TMatrix hMatrPhiP;
	
	double MatrPhiT[PVTKF_X_DIM][PVTKF_X_DIM];
	TMatrix hMatrPhiT;

	double MatrPhiPPhiT[PVTKF_X_DIM][PVTKF_X_DIM];
	TMatrix hMatrPhiPPhiT;
	
	setMatrHandle(&hMatrPhiP, PVTKF_X_DIM, PVTKF_X_DIM, PVTKF_X_DIM, PVTKF_X_DIM, (double*)MatrPhiP);
	setMatrHandle(&hMatrPhiT, PVTKF_X_DIM,PVTKF_X_DIM,PVTKF_X_DIM,PVTKF_X_DIM, (double*)MatrPhiT);
	setMatrHandle(&hMatrPhiPPhiT, PVTKF_X_DIM,PVTKF_X_DIM,PVTKF_X_DIM,PVTKF_X_DIM, (double*)MatrPhiPPhiT);
	

	MatrixMul(&hMatrixPhi, &hMatrixP, &hMatrPhiP);

	MatrixTranspose(&hMatrixPhi, &hMatrPhiT);

	MatrixMul(&hMatrPhiP, &hMatrPhiT, &hMatrPhiPPhiT);
	
	MatrixAdd(&hMatrPhiPPhiT, &hMatrixQ, &hMatrixP);
	
	return;
}

static int32 UpdateH_AllSVDeltaZ_R(PVT_TRKCH_INFO* pSVPVTInfo, PVT_FIX_INFO* kfpvtinfo)
{
	int32 row=0;
	
	ZerosMatrix(&hMatrixH);

	UpdateResiErrAndAlpha((BITMAP_DIM_BU_GPS| BITMAP_DIM_BU_BD2 | BITMAP_DIM_BU_GLO), &(kfpvtinfo->poschmap), pSVPVTInfo);
	SelectSVToFix(pSVPVTInfo, &(kfpvtinfo->poschmap), &(kfpvtinfo->velchmap));
	
	kfpvtinfo->buflag = CheckMixFixSVAndBuFlag(pSVPVTInfo, &(kfpvtinfo->poschmap));
	
	//PosEcaSelect(kfpvtinfo->posdim, pSVPVTInfo, &(kfpvtinfo->poschnmap),  &(kfpvtinfo->dopchnmap),  &(kfpvtinfo->dop));
	//VelEcaSelect(&(kfpvtinfo->velchnmap), pSVPVTInfo);
	
	//bDGPSUsed = UpdateSVDifferenceInfo(kfpvtinfo->poschnmap, pSVPVTInfo);

	GetGNSSPRTimeBias(kfpvtinfo->buflag, PRModify);

	UpdateResiErrAndAlpha(kfpvtinfo->buflag, &(kfpvtinfo->poschmap), pSVPVTInfo);
	
	row = UpdateMatrixH_Z_R(kfpvtinfo->buflag, pSVPVTInfo, &(kfpvtinfo->poschmap), &(kfpvtinfo->velchmap));
	
	return row;
}

void UpdateResiErrAndAlpha(byte buflag, word256* pChnmap, PVT_TRKCH_INFO* pSVPVTInfo)
{
	//update residue and Alpha
	double bu = 0.0, ctu = 0.0;
	double pseudorange = 0.0;
	double angle_er = 0.0, disSV2U_Px, disSV2U_Py, disSV2U_Pz, disSV2U, Alpha_x, Alpha_y, Alpha_z, wave_len, Td, Ed;
	int8 svbuidx[3]={0,0,0};

	int32 trkch=0, svid = 0;

	checkKfSVBuIdx(buflag, svbuidx);
		
	for(trkch=0; trkch<MAXCHANNELS_PVT; trkch++)
	{
		if(GetBitWord256(pChnmap,trkch)==0)
			continue;
		
		svid = pSVPVTInfo[trkch].svid;
		pseudorange = pSVPVTInfo[trkch].pr;

		//if(PRModify[0].bPRModify && SV_IsGps(svid))
		//	pseudorange -= PRModify[0].bias;
		
		if(PRModify[1].bPRModify && SV_IsBd2(svid))
			pseudorange -= PRModify[1].bias;

#if SUPPORT_GLONASS
		if(PRModify[2].bPRModify && SV_IsGlo(svid))
			pseudorange -= PRModify[2].bias;
#endif
		
		bu = 0.0;
		ctu = 0.0;
		if(SV_IsGps(svid))
			bu =  VectorX[svbuidx[0]];
		else if(SV_IsBd2(svid))
			bu =  VectorX[svbuidx[1]];
#if SUPPORT_GLONASS
		else if(SV_IsGlo(svid))
			bu =  VectorX[svbuidx[2]];
#endif

		if(!SV_IsECA(svid))
			ctu = VectorX[CTU_IDX];
		
		angle_er= -((pSVPVTInfo[trkch].pr - bu)*RECIP_SPEED_OF_LIGHT)*OMEGAE_WGS;

		disSV2U_Px = VectorX[0] - (pSVPVTInfo[trkch].svpos.x*cos(angle_er) - pSVPVTInfo[trkch].svpos.y*sin(angle_er));
		disSV2U_Py = VectorX[1] - (pSVPVTInfo[trkch].svpos.y*cos(angle_er) + pSVPVTInfo[trkch].svpos.x*sin(angle_er));
		disSV2U_Pz = VectorX[2] - pSVPVTInfo[trkch].svpos.z;

		disSV2U = sqrt(disSV2U_Px*disSV2U_Px + disSV2U_Py*disSV2U_Py + disSV2U_Pz*disSV2U_Pz);

		Alpha_x = disSV2U_Px/disSV2U;
		Alpha_y = disSV2U_Py/disSV2U;
		Alpha_z = disSV2U_Pz/disSV2U;

		pSVPVTInfo[trkch].resiErr_pr = pSVPVTInfo[trkch].pr - (disSV2U + bu);

		wave_len = GetSVWaveLen(svid, pSVPVTInfo[trkch].freq_point);

		if(SV_IsECA(svid))
			Td = 0.0;
		else
			Td = -Alpha_x * pSVPVTInfo[trkch].svvel.x - Alpha_y * pSVPVTInfo[trkch].svvel.y - Alpha_z * pSVPVTInfo[trkch].svvel.z + wave_len * pSVPVTInfo[trkch].fd;
		
		Ed = -Alpha_x * VectorX[3] -  Alpha_y * VectorX[4] - Alpha_z * VectorX[5] - ctu;

		pSVPVTInfo[trkch].resiErr_fd = Td - Ed;

		KFChnInfo[trkch].alpha.x = Alpha_x;
		KFChnInfo[trkch].alpha.y = Alpha_y;
		KFChnInfo[trkch].alpha.z = Alpha_z;
	}

	return;
}


void SelectSVToFix(PVT_TRKCH_INFO* pSVPVTInfo, word256* pPosChnMap, word256* pVelChnMap)
{
	int32 trkch = 0, posvalidsvcnt = 0, velvalidsvcnt = 0;
	double meanPrResidue = 0.0, meanFdResidue = 0.0;
	double prresidue = 0.0, fdresidue = 0.0;
	int32 cn0 = 0;

	for(trkch=0; trkch<MAXCHANNELS_PVT; trkch++)
	{	
		if(SV_IsECA(pSVPVTInfo[trkch].svid))
			continue;

		if(GetBitWord256(pPosChnMap, trkch)==1)
		{
			posvalidsvcnt ++;
			meanPrResidue += f_abs(pSVPVTInfo[trkch].resiErr_pr);
		}

		if(GetBitWord256(pVelChnMap, trkch)==1)
		{
			velvalidsvcnt ++;
			meanFdResidue += f_abs(pSVPVTInfo[trkch].resiErr_fd);
		}
	}

	if(posvalidsvcnt > 0)
		meanPrResidue /= posvalidsvcnt;

	if(velvalidsvcnt > 0)
		meanFdResidue /= velvalidsvcnt;
		
	for(trkch=0; trkch<MAXCHANNELS_PVT; trkch++)
	{
		prresidue = f_abs(pSVPVTInfo[trkch].resiErr_pr);
		fdresidue = f_abs(pSVPVTInfo[trkch].resiErr_fd);

		cn0 = pSVPVTInfo[trkch].cn1s;
		if(SV_IsBd2Geo(pSVPVTInfo[trkch].svid))
			cn0 = cn0 - 8;
			
		//sv used to pos selection
		if(GetBitWord256(pPosChnMap, trkch)==1)
		{	
			if((prresidue > 3.0 * meanPrResidue) && (prresidue > 50.0) && (cn0 <= 38) && (posvalidsvcnt > 5))
			{
				ClearBitWord256(pPosChnMap,trkch);	//not use this SV
				pSVPVTInfo[trkch].selpath = FIX_SV_RESIDUE_PR;
			}
		}

		//sv used to vel selection
		if(GetBitWord256(pVelChnMap, trkch)==1)
		{
			if((fdresidue > 5.0) && (fdresidue > 3.0 * meanFdResidue) && (cn0 <= 38) && (velvalidsvcnt > 5))
			{
				ClearBitWord256(pVelChnMap,trkch);	//not use this SV
				pSVPVTInfo[trkch].selpath = FIX_SV_RESIDUE_FD;
			}	
		}		
	}
	
	return;
}

//H*P*H'
void MatrixMulHPHT(TMatrix* pMatrixH, TMatrix* pMatrixP, TMatrix* pMatrixHPHT)
{
	int32 measurecnt = pMatrixH->rows;
	double MatrixHP[MAX_Z_DIM][PVTKF_X_DIM] = {{0,},};
	double MatrixHT[PVTKF_X_DIM][MAX_Z_DIM] = {{0,},};
	TMatrix hMatrixHP, hMatrixHT;

	setMatrHandle(&hMatrixHP, MAX_Z_DIM, PVTKF_X_DIM, measurecnt, PVTKF_X_DIM, (double*)MatrixHP);		// H*P
	setMatrHandle(&hMatrixHT, PVTKF_X_DIM, MAX_Z_DIM, PVTKF_X_DIM, measurecnt, (double*)MatrixHT);		// H'

	MatrixTranspose(pMatrixH, &hMatrixHT);			//H'
	MatrixMul(pMatrixH, pMatrixP, &hMatrixHP);		//H * P
	MatrixMul(&hMatrixHP, &hMatrixHT, pMatrixHPHT);		//H * P * H'
	
	return;
}

int32 errposcnt = 0, errfreqcnt = 0;
void PreKalmanResidueCheck(PVT_TRKCH_INFO* pSVPVTInfo, word256* pFixChnMap, word256* pVelChnMap)
{	
	int32 measurecnt = hMatrixH.rows;

	int32 trkch = 0, posidx = 0, velidx = KFSvPosCnt;
	int32 sumerrposcn0 = 0, sumerrfreqcn0 = 0;
	byte cn0th = 42 + (SYS_FULL_CLOSE_ENV - SysEnvSta) * 3;

	byte cn0 = 0;	
	boolean bWeakSV = FALSE;
	int32 row = 0;

	double prresidue = 0.0, ChPRRatio = 0.0;
	double fdresidue = 0.0, ChFreqRatio = 0.0;
	
	double MatrixTemp[MAX_Z_DIM][MAX_Z_DIM] = {{0,},};
	double VectorRBackup[MAX_Z_DIM] = {0,};
	TMatrix hMatrixTemp;

	setMatrHandle(&hMatrixTemp, MAX_Z_DIM, MAX_Z_DIM, measurecnt, measurecnt, (double*)MatrixTemp);		//H*P*H' + R

	MatrixMulHPHT(&hMatrixH, &hMatrixP, &hMatrixTemp);
	MatrixAddVector(&hMatrixTemp, &hVectorR, &hMatrixTemp);	//H * P * H' + R

	
	for(row = 0; row < measurecnt; row ++)
		VectorRBackup[row] = VectorR[row];		//backup the previous matrix R
		
	errposcnt = 0, errfreqcnt = 0;
	for(trkch = 0; trkch < MAXCHANNELS_PVT; trkch ++)
	{
		cn0 = pSVPVTInfo[trkch].cn1s;		
		bWeakSV = FALSE;
		
		if(GetBitWord256(pFixChnMap, trkch)==1)
		{
			prresidue = f_abs(pSVPVTInfo[trkch].resiErr_pr);
			ChPRRatio = prresidue / sqrt(MatrixTemp[posidx][posidx]);
						
			if((ChPRRatio > 10.0) && (prresidue > 15.0) && (cn0 < cn0th))
			{				
				errposcnt ++;
				bWeakSV = TRUE;
				sumerrposcn0 += pSVPVTInfo[trkch].cn1s;
				VectorR[posidx] *= (1.0e10);

				if(GetBitWord256(pVelChnMap, trkch)==1)
					VectorR[velidx] *= (1.0e10);
			}
			else if((ChPRRatio > 3.0) && (cn0 < cn0th))
			{
				if(prresidue > 1.0)
				{
					VectorR[posidx] *= (prresidue * prresidue);
				}
			}
			else if(ChPRRatio < 0.3)
			{
				VectorR[posidx] *= (0.5);
			}

			pSVPVTInfo[trkch].rate_pr = ChPRRatio;
			pSVPVTInfo[trkch].R_pr = VectorR[posidx];
		
			posidx ++;
		}

		if(GetBitWord256(pVelChnMap, trkch)==1)
		{
			if(!bWeakSV)
			{
				fdresidue = f_abs(pSVPVTInfo[trkch].resiErr_fd);
				ChFreqRatio = fdresidue / sqrt(MatrixTemp[velidx][velidx]);

				if((ChFreqRatio > 16.0) && (fdresidue > 10.0))
				{
					errfreqcnt ++;
					bWeakSV = TRUE;
					sumerrfreqcn0 += pSVPVTInfo[trkch].cn1s;
					VectorR[velidx] *= (1.0e10);				

					if(GetBitWord256(pFixChnMap, trkch)==1)
						VectorR[posidx-1] *= (1.0e10);
				}
				else if(ChFreqRatio > 3.0)
				{
					if(fdresidue > 1.0)
						VectorR[velidx] *= (fdresidue * fdresidue);
				}
				else if(ChFreqRatio < 0.3)
				{
					VectorR[velidx] *= (0.5);
				}
			}

			pSVPVTInfo[trkch].rate_fd = ChFreqRatio;
			pSVPVTInfo[trkch].R_fd = VectorR[velidx];
				
			velidx ++;
		}

		
	}

	if(((errposcnt > KFSvPosCnt - errposcnt) && (errposcnt > SV_NUM_FIX_2D) && (sumerrposcn0 > errposcnt * 30)) ||
		((errfreqcnt > KFSvVelCnt - errfreqcnt) && (errfreqcnt > SV_NUM_FIX_2D) && (sumerrfreqcn0 > errfreqcnt * 30)))
	{
		//kalman filter exception
		//restore R matrix
		for(row = 0; row < measurecnt; row ++)
			VectorR[row] = VectorRBackup[row];

		EyeMatrix(&hMatrixP,1.0e8);						//reset kalman process noise
	}

	return;
}

int32 UpdateMatrixH_Z_R(byte buflag, PVT_TRKCH_INFO* pSVPVTInfo, word256* pFixChnMap, word256* pVelChnMap)
{
	int8 svbuidx[3]={0,0,0};
	int32 trkch = 0, svid = 0, row = 0, row1 = 0;

	double SVR_Pr=0.0, SVR_fd=0.0;
	int8 cn0=0,cn0_vel=0;
	//double RNoise1 = 0.0, RVar = 0.0;

	checkKfSVBuIdx(buflag, svbuidx);
	
	for(trkch = 0; trkch < MAXCHANNELS_PVT; trkch ++)
	{
		if(GetBitWord256(pFixChnMap,trkch)==1)
		{
			KFSvPosCnt ++;

			svid = pSVPVTInfo[trkch].svid;
			if(SV_IsGps(svid))
				KFSvPosCnt_GPS ++;
			else if(SV_IsBd2(svid))
				KFSvPosCnt_BD ++;
#if SUPPORT_GLONASS
			else if(SV_IsGlo(svid))
				KFSvPosCnt_GLO ++;
#endif
		}

		if(GetBitWord256(pVelChnMap,trkch)==1)
			KFSvVelCnt ++;
	}

	row1 = KFSvPosCnt;	
	for(trkch = 0; trkch < MAXCHANNELS_PVT; trkch ++)
	{
		svid = pSVPVTInfo[trkch].svid;

		//for R
		SVR_Pr=0.0;
		SVR_fd=0.0;
		if((GetBitWord256(pFixChnMap,trkch)==1) ||(GetBitWord256(pVelChnMap,trkch)==1))
		{
			cn0 = (pSVPVTInfo[trkch].cn1s+pSVPVTInfo[trkch].cn100ms)/2;
			cn0_vel = cn0;
			if(SV_IsBd2Geo(svid))
			{
				if(cn0<33)
				{
					cn0 -= 8;	// cn0 model for GEO
					cn0_vel = cn0-16;
				}
				else if(cn0<34)
				{
					cn0 -= 6;	// cn0 model for GEO
					cn0_vel = cn0-8;
				}
				else if(cn0<39)
				{
					cn0 -= 5;
					cn0_vel = cn0-3;
				}
			}
#if SUPPORT_GLONASS
			else if(SV_IsGlo(svid))
			{
				cn0 -= 5;
				cn0_vel = cn0;
			}
#endif
			else
			{
				if(cn0<=21)
				{
					cn0_vel -=10;
				}
				else if(cn0<23)
				{
					cn0_vel -=5;
				}
			}

	
			cn0 -= 8;	
		
			if(cn0 > 49)
				cn0 = 49;
				
			if(cn0<0)
				cn0=0;


			cn0_vel -= 8;

			if(cn0_vel > 49)
				cn0_vel = 49;
				
			if(cn0_vel<0)
				cn0_vel=0;
			
			//RNoise1 = CoVarianceCN0Tbl[cn0];

			//double sin_el = sin(pPVTData[i].el / DEG_PER_RADIAN);	
			//double RNoise2 = 1.0 / (sin_el * sin_el);		// el model
			//double RVar = sqrt(RNoise1 *RNoise2 );	//use both el model and cn0 model
			//RVar = RNoise1;//sqrt(RNoise1 * RNoise1);	//use both el model and cn0 model

			SVR_Pr = KalmanNoise[KalmanNoiseIndex].CodeLoopErr * KalmanNoise[KalmanNoiseIndex].CodeLoopErr * CoVarianceCN0Tbl[cn0];			
			SVR_fd = KalmanNoise[KalmanNoiseIndex].FreqLoopErr * KalmanNoise[KalmanNoiseIndex].FreqLoopErr * CoVarianceCN0Tbl[cn0_vel];
	
			if(GetBitWord256(pFixChnMap,trkch)==1)		//pos used
			{
				MatrixH[row][0] = KFChnInfo[trkch].alpha.x;
				MatrixH[row][1] = KFChnInfo[trkch].alpha.y;
				MatrixH[row][2] = KFChnInfo[trkch].alpha.z;

				if(SV_IsGps(svid))
					MatrixH[row][svbuidx[0]] = 1.0;
				else if(SV_IsBd2(svid))
					MatrixH[row][svbuidx[1]] = 1.0;	
		#if SUPPORT_GLONASS
				else if(SV_IsGlo(svid))
					MatrixH[row][svbuidx[2]] = 1.0;	
		#endif

				VectorDeltaZ[row] = pSVPVTInfo[trkch].resiErr_pr;
				VectorR[row] = SVR_Pr;

				row++;
			}

			if(GetBitWord256(pVelChnMap,trkch)==1)	//vel used
			{
				MatrixH[row1][3] = -KFChnInfo[trkch].alpha.x;
				MatrixH[row1][4] = -KFChnInfo[trkch].alpha.y;
				MatrixH[row1][5] = -KFChnInfo[trkch].alpha.z;

				if(!SV_IsECA(svid))
					MatrixH[row1][CTU_IDX] = -1.0;

				VectorDeltaZ[row1] = pSVPVTInfo[trkch].resiErr_fd;
				VectorR[row1] = SVR_fd;

				row1++;
			}			
		}
	}

	hMatrixH.rows = row1;
	hVectorR.rows = row1;

	#if(PRE_RESIDUE_CHECK_EN)
	PreKalmanResidueCheck(pSVPVTInfo, pFixChnMap, pVelChnMap);
	#endif

	return (row1);
}

boolean UpdateMatrixK(int32 ZLength, int32 measureidx)
{
	//Trans(H)  (8*2N)
	double tempVectorR[MAX_Z_DIM];

#if(KALMAN_UPDATE_FLOW == KALMAN_UPDATE_K_FIRST)
	//H*P*Trans(H)  (2N*2N)
	double MatrHPHT[MAX_Z_DIM][MAX_Z_DIM];
	TMatrix hMatrH;
	TMatrix hMatrHPHT;

	TVector hVectorTempR;

	double MatrInvHPHTR[MAX_Z_DIM][MAX_Z_DIM];
	TMatrix hMatrInvHPHTR;

	int32 ret = 0;
#else

	double VectorInvR[MAX_Z_DIM];
	TVector hVectorInvR;

	int32 i = 0;
#endif

	int32 row = 0, col = 0;

	double MatrH[MAX_Z_DIM][PVTKF_X_DIM];
	double MatrHT[PVTKF_X_DIM][MAX_Z_DIM];
	double MatrPHT[PVTKF_X_DIM][MAX_Z_DIM];	
	TMatrix hMatrHT;
	TMatrix hMatrPHT;

	setMatrHandle(&hMatrHT,PVTKF_X_DIM, MAX_Z_DIM, PVTKF_X_DIM, ZLength, (double*)MatrHT);
	setMatrHandle(&hMatrPHT,PVTKF_X_DIM,MAX_Z_DIM,PVTKF_X_DIM,ZLength, (double*)MatrPHT);


	hMatrixK.cols = ZLength;

	for(row = 0; row < ZLength; row ++)
	{
		for (col = 0; col < PVTKF_X_DIM; col ++)
		{
			MatrH[row][col] = MatrixH[measureidx + row][col];
			MatrHT[col][row] = MatrH[row][col];			
		}

		tempVectorR[row] = VectorR[measureidx + row];
	}
	
	//P*Trans(H)  (8*2N)
	MatrixMul(&hMatrixP, &hMatrHT, &hMatrPHT);
	
#if(KALMAN_UPDATE_FLOW == KALMAN_UPDATE_K_FIRST)
	//K(k)=P(k,k-1)*Trans(H(k))*Invers[H(k)*P(k,k-1)*Trans(H(k))+R(k)]	
	setMatrHandle(&hMatrH, MAX_Z_DIM, PVTKF_X_DIM, ZLength, PVTKF_X_DIM, (double*)MatrH);
	setMatrHandle(&hMatrHPHT, MAX_Z_DIM,MAX_Z_DIM,ZLength,ZLength, (double*)MatrHPHT);
	setMatrHandle(&hMatrInvHPHTR, MAX_Z_DIM, MAX_Z_DIM, ZLength, ZLength, (double*)MatrInvHPHTR);
	setVectorHandle(&hVectorTempR,MAX_Z_DIM,ZLength,(double*)tempVectorR);


	//H*P*Trans(H)  (2N*2N)
	MatrixMul(&hMatrH, &hMatrPHT, &hMatrHPHT);

	//H*P*Trans(H) + R    (2N*2N)
	MatrixAddVector(&hMatrHPHT, &hVectorTempR, &hMatrHPHT);

	//Invers(H*P*Trans(H) + R)  (2N*2N)
	if(ZLength > 1)
		ret = MatrixInverse(&hMatrHPHT, &hMatrInvHPHTR);
	else
	{
		if(f_abs(MatrHPHT[0][0]) > MICRO_NUM)
			MatrInvHPHTR[0][0] = 1.0 / MatrHPHT[0][0];
	}

	if(ret == 0)
	{
		//K(k)=P(k,k-1)*Trans(H(k))*Invers[H(k)*P(k,k-1)*Trans(H(k))+R(k)]
		MatrixMul(&hMatrPHT, &hMatrInvHPHTR, &hMatrixK);

		return TRUE;
	}

	return FALSE;
#else

	setVectorHandle(&hVectorInvR, MAX_Z_DIM, ZLength, (double*)VectorInvR);

	for(i = 0; i < ZLength; i ++)
		VectorInvR[i] = 1.0 / tempVectorR[i];

	//P * H' * inv(R)
	MatrixMulVector(&hMatrPHT, &hVectorInvR, &hMatrixK);

	return TRUE;	
#endif
} 

void UpdateVectorX(int32 ZLength, int32 measureidx)
{
	//X(k)=X(k,k-1)+K*(Z(k)-Z(k,k-1)),  DeltaZ=Z(k)-Z(k,k-1)
	int32 row = 0;
	
	double VectdeltaX[PVTKF_X_DIM];
	double deltaz[MAX_Z_DIM];
	TMatrix hVectdeltaZ;
	TMatrix hVectdeltaX;
	setMatrHandle(&hVectdeltaZ, MAX_Z_DIM, 1, ZLength, 1, (double*)deltaz);
	setMatrHandle(&hVectdeltaX, PVTKF_X_DIM, 1, PVTKF_X_DIM, 1, (double*)VectdeltaX);
		
	for(row = 0; row < ZLength; row ++)
		deltaz[row] = VectorDeltaZ[measureidx + row]; 

	MatrixMul(&hMatrixK, &hVectdeltaZ, &hVectdeltaX);	
	
	//X(k) = X(k-) + K*(Z - H*X(k-))
	for (row = 0; row < PVTKF_X_DIM; row ++)
		VectorX[row] += VectdeltaX[row];

	return;
}

boolean UpdateMatrixP(int32 ZLength, int32 measureidx)
{
	double MatrH[MAX_Z_DIM][PVTKF_X_DIM];
	TMatrix hMatrH;

#if (KALMAN_UPDATE_FLOW	 == KALMAN_UPDATE_K_FIRST)
	//I
	double TempEye[PVTKF_X_DIM][PVTKF_X_DIM];
	TMatrix hTempEye;

	//K*H
	double MatrixKMulH[PVTKF_X_DIM][PVTKF_X_DIM];
	TMatrix hMatrixKMulH;;

	//(I-K(k)*H(k))*P(k,k-1)
	double TempP[PVTKF_X_DIM][PVTKF_X_DIM];
	TMatrix hTempP;
#else
	double MatrixHTInvRH[PVTKF_X_DIM][PVTKF_X_DIM];	//H' * inv(R) * H (9 * 9)
	double MatrixTemp[PVTKF_X_DIM][PVTKF_X_DIM]; // (9*9)

	double VectorInvR[MAX_Z_DIM];
	double MatrixHTInvR[PVTKF_X_DIM][MAX_Z_DIM];		//H' * inv(R), (9 * n)
	
	TMatrix hMatrixHTInvR;
	TMatrix hMatrixHTInvRH;
	TMatrix hMatrixTemp;

	TVector hVectorInvR;

	double MatrHT[PVTKF_X_DIM][MAX_Z_DIM];
	TMatrix hMatrHT;
#endif


	int32 row = 0, col = 0;

	setMatrHandle(&hMatrH,MAX_Z_DIM, PVTKF_X_DIM, ZLength, PVTKF_X_DIM, (double*)MatrH);
	
	for(row = 0; row < ZLength; row ++)
	{
		for (col = 0; col < PVTKF_X_DIM; col ++)
		{
			MatrH[row][col] = MatrixH[measureidx + row][col];
		}
	}
	
#if (KALMAN_UPDATE_FLOW	 == KALMAN_UPDATE_K_FIRST)
	//P(k)=(I-K(k)*H(k))*P(k,k-1)
	setMatrHandle(&hTempEye,PVTKF_X_DIM, PVTKF_X_DIM, PVTKF_X_DIM, PVTKF_X_DIM, (double*)TempEye);
	setMatrHandle(&hMatrixKMulH,PVTKF_X_DIM,PVTKF_X_DIM,PVTKF_X_DIM,PVTKF_X_DIM, (double*)MatrixKMulH);
	setMatrHandle(&hTempP, PVTKF_X_DIM,PVTKF_X_DIM,PVTKF_X_DIM,PVTKF_X_DIM, (double*)TempP);

	//I
	EyeMatrix(&hTempEye, 1);

	//K*H
	MatrixMul(&hMatrixK, &hMatrH, &hMatrixKMulH);

	//I-K*H
	MatrixSub(&hTempEye, &hMatrixKMulH, &hTempEye);

	//(I-K(k)*H(k))*P(k,k-1)
	MatrixMul(&hTempEye, &hMatrixP, &hTempP);
	MatrixCopy(&hTempP, &hMatrixP);

	//set matrix P to symmetrical matrix
	for(row = 0; row < PVTKF_X_DIM; row ++)
	{
		for(col = 0; col < row; col ++)
		{
			MatrixP[row][col] = (MatrixP[row][col] + MatrixP[col][row]) / 2;
			MatrixP[col][row] = MatrixP[row][col];
		}
	}
	
	return TRUE;	
#else
	//P(k) = inv(H'*inv(R)*H + inv(P(k, k-1)))
	setMatrHandle(&hMatrixHTInvR,PVTKF_X_DIM, MAX_Z_DIM, PVTKF_X_DIM, ZLength, (double*)MatrixHTInvR);
	setMatrHandle(&hMatrixHTInvRH,PVTKF_X_DIM, PVTKF_X_DIM, PVTKF_X_DIM, PVTKF_X_DIM, (double*)MatrixHTInvRH);
	setMatrHandle(&hMatrixTemp, PVTKF_X_DIM, PVTKF_X_DIM, PVTKF_X_DIM, PVTKF_X_DIM, (double*)MatrixTemp);

	setVectorHandle(&hVectorInvR,MAX_Z_DIM,ZLength, VectorInvR);

	setMatrHandle(&hMatrHT,PVTKF_X_DIM,MAX_Z_DIM,PVTKF_X_DIM,ZLength, (double*)MatrHT);
	

	for(row = 0; row < ZLength; row ++)
		VectorInvR[row] = 1.0 / VectorR[row + measureidx];

	MatrixTranspose(&hMatrH, &hMatrHT);

	//H'*inv(R)
	MatrixMulVector(&hMatrHT, &hVectorInvR, &hMatrixHTInvR);

	//H' * inv(R) * H
	MatrixMul(&hMatrixHTInvR, &hMatrH, &hMatrixHTInvRH);

	//inv(P(k, k-1))
	if(MatrixInverse(&hMatrixP, &hMatrixTemp) != 0)
		return FALSE;

	//H' * inv(R) * H + inv(P(k, k-1))
	MatrixAdd(&hMatrixHTInvRH, &hMatrixTemp, &hMatrixTemp);

	//inv(H' * inv(R) * H + inv(P(k, k-1)))
	if(MatrixInverse(&hMatrixTemp, &hMatrixP) != 0)
		return FALSE;

	//set matrix P to symmetrical matrix
	for(row = 0; row < PVTKF_X_DIM; row ++)
	{
		for(col = 0; col < row; col ++)
		{
			MatrixP[row][col] = (MatrixP[row][col] + MatrixP[col][row]) / 2;
			MatrixP[col][row] = MatrixP[row][col];
		}
	}

	return TRUE;
	
#endif
}

void UpdateMatrixQ(double cycle)
{
	double clkdeltaphase=0.0, clkdeltafreq=0.0;
	double T3Div3=0.0, T2Div2=0.0;
	double planespeed = 0.0, heading = 0.0;
	int32 row = 0, col = 0;
	
	double MatrixECEFNoise[3][3] = {{0,},};
	TMatrix hMatrixECEFNoise;

	ECEF pos = {0,}, vel = {0,};
	pos.x = VectorX[0];
	pos.y = VectorX[1];
	pos.z = VectorX[2];

	vel.x = VectorX[3];
	vel.y = VectorX[4];
	vel.z = VectorX[5];

	memset(MatrixQ, 0, sizeof(MatrixQ));

	ECEFVel2SpeedHeading(&pos, &vel, &planespeed, &heading);

	for(KalmanNoiseIndex = 0; KalmanNoiseIndex < KALMAN_NOISE_NUM; KalmanNoiseIndex ++)
	{
		if(planespeed < KalmanNoise[KalmanNoiseIndex].MaxSpeed)
			break;
	}

	if(KalmanNoiseIndex == KALMAN_NOISE_NUM)
		KalmanNoiseIndex = KALMAN_NOISE_NUM - 1;

#if 0
	if((NavSysMap & (NAV_SYS_GPS | NAV_SYS_COMPASS)) != (NAV_SYS_GPS | NAV_SYS_COMPASS))
	{
		if((SysEnvSta > SYS_HALF_OPEN_SKY) && (KalmanNoiseIndex < 3))
			KalmanNoiseIndex = 3;
		else if((SysEnvSta == SYS_HALF_OPEN_SKY) && (KalmanNoiseIndex < 2))
			KalmanNoiseIndex = 2;
	}
#endif

	//if(KalmanNoiseIndex <= 1)
	//	KalmanNoiseIndex = 1;

	setMatrHandle(&hMatrixECEFNoise,3, 3, 3, 3, (double*)MatrixECEFNoise);

	UpdateProcessNoise(KalmanNoiseIndex, pos, &hMatrixECEFNoise);

	T3Div3 = (cycle * cycle * cycle) / 3;
	T2Div2 = (cycle * cycle ) / 2;

	//ECEF XYZ noise
	for(row = 0; row < 3; row ++)
	{
		for(col = 0; col < 3; col ++)
		{
			MatrixQ[row][col] = MatrixECEFNoise[row][col] * T3Div3;		
			MatrixQ[row + 3][col + 3] = MatrixECEFNoise[row][col] * cycle;
			MatrixQ[row][col + 3] = MatrixECEFNoise[row][col] * T2Div2;
			MatrixQ[col + 3] [row] = MatrixQ[row][col + 3];
		}
	}

	//clock noise
	clkdeltaphase = KalmanNoise[KalmanNoiseIndex].ClkDeltaPhase;
	clkdeltafreq = KalmanNoise[KalmanNoiseIndex].ClkDeltaFreq;
	
	MatrixQ[GPS_BU_IDX][GPS_BU_IDX] = clkdeltaphase * clkdeltaphase * cycle + clkdeltafreq * clkdeltafreq * T3Div3;
	MatrixQ[GPS_BU_IDX][CTU_IDX] = clkdeltafreq * clkdeltafreq * T2Div2;
	MatrixQ[CTU_IDX][GPS_BU_IDX] = MatrixQ[GPS_BU_IDX][CTU_IDX];
	

	MatrixQ[BD_BU_IDX][BD_BU_IDX] = MatrixQ[GPS_BU_IDX][GPS_BU_IDX];
	MatrixQ[BD_BU_IDX][CTU_IDX] = MatrixQ[GPS_BU_IDX][CTU_IDX];
	MatrixQ[CTU_IDX][BD_BU_IDX] = MatrixQ[BD_BU_IDX][CTU_IDX];


	#if SUPPORT_GLONASS
	MatrixQ[GLO_BU_IDX][GLO_BU_IDX] = MatrixQ[GPS_BU_IDX][GPS_BU_IDX];
	MatrixQ[GLO_BU_IDX][CTU_IDX] = MatrixQ[GPS_BU_IDX][CTU_IDX];
	MatrixQ[CTU_IDX][GLO_BU_IDX] = MatrixQ[GLO_BU_IDX][CTU_IDX];
	#endif

	MatrixQ[CTU_IDX][CTU_IDX] = clkdeltafreq * clkdeltafreq * cycle;
	
	return;
}

void UpdateProcessNoise(int32 index, ECEF pos, TMatrix* hMatrixECEFNoise)
{	
	WGS wgs = {0,};
	double sinLon =0.0, cosLon= 0.0, sinLat = 0.0, cosLat=0.0;
	double posENError = 0.0, posUError = 0.0;
	
	double MatrixEnu2Ecef[3][3] = {{0,},}, MatrixEnuNoise[3][3] = {{0,},};
	double MatrixTmp[3][3] = {{0,},}, MatrixTmp1[3][3] = {{0,},};

	TMatrix hMatrixEnu2ECEF;
	TMatrix hMatrixEnuNoise;
	TMatrix hMatrixTmp;
	TMatrix hMatrixTmp1;

	setMatrHandle(&hMatrixEnu2ECEF,3, 3, 3, 3, (double*)MatrixEnu2Ecef);
	setMatrHandle(&hMatrixEnuNoise,3, 3, 3, 3, (double*)MatrixEnuNoise);
	setMatrHandle(&hMatrixTmp,3, 3, 3, 3, (double*)MatrixTmp);
	setMatrHandle(&hMatrixTmp1,3, 3, 3, 3, (double*)MatrixTmp1);


	ZerosMatrix(&hMatrixEnuNoise);
	ZerosMatrix(&hMatrixTmp);
	ZerosMatrix(&hMatrixTmp1);

	
	ECEF2WGS(&pos, &wgs);

	sinLon = sin(wgs.lon);
	cosLon = cos(wgs.lon);
	sinLat = sin(wgs.lat);
	cosLat = cos(wgs.lat);

	//for pos
	MatrixEnu2Ecef[0][0]=-sinLon;
	MatrixEnu2Ecef[0][1]=-cosLon*sinLat;
	MatrixEnu2Ecef[0][2]=cosLon*cosLat;
	MatrixEnu2Ecef[1][0]=cosLon;
	MatrixEnu2Ecef[1][1]=-sinLon*sinLat;
	MatrixEnu2Ecef[1][2]=sinLon*cosLat;
	MatrixEnu2Ecef[2][0]=0;
	MatrixEnu2Ecef[2][1]=cosLat;
	MatrixEnu2Ecef[2][2]=sinLat;

	posENError = KalmanNoise[index].PosENError;
	posUError = KalmanNoise[index].PosUError;

#if 0
	//if(bKFStatus != KF_PVT_GOOD)
	if((index > 3) /*&& (ShortTermAltEnv > 2.0)*/)
	{
		double tmpenv = ShortTermAltEnv;
		posENError *= (tmpenv);
		posUError *= (tmpenv);
	}
#endif
	
	MatrixEnuNoise[0][0] = posENError * posENError;
	MatrixEnuNoise[1][1] = MatrixEnuNoise[0][0];
	MatrixEnuNoise[2][2] = posUError * posUError;
		
	MatrixTranspose(&hMatrixEnu2ECEF, &hMatrixTmp);
	
	//(pEnu2ECEFMatrix) * (hMatirxENUNoise) * (pEnu2ECEFMatrix)'
	MatrixMul(&hMatrixEnu2ECEF, &hMatrixEnuNoise, &hMatrixTmp1);
	MatrixMul(&hMatrixTmp1, &hMatrixTmp, hMatrixECEFNoise);
		
	return;
}




void UpdateMatrixPhi(double cycle)
{
/*Phi=[1 0 0 T 0 0 0 0
        0 1 0 0 T 0 0 0
        0 0 1 0 0 T 0 0
        0 0 0 1 0 0 0 0
        0 0 0 0 1 0 0 0
        0 0 0 0 0 1 0 0
        0 0 0 0 0 0 1 T
        0 0 0 0 0 0 0 1];

Phi=[1 0 0 T 0 0 0 0 0
        0 1 0 0 T 0 0 0 0
        0 0 1 0 0 T 0 0 0
        0 0 0 1 0 0 0 0 0
        0 0 0 0 1 0 0 0 0
        0 0 0 0 0 1 0 0 0
        0 0 0 0 0 0 1 0 T
        0 0 0 0 0 0 0 1 T
        0 0 0 0 0 0 0 0 1];
 */
	int32 i=0;

	EyeMatrix(&hMatrixPhi,1);

	MatrixPhi[0][3] = cycle;
	MatrixPhi[1][4] = cycle;
	MatrixPhi[2][5] = cycle;

	for(i=GPS_BU_IDX; i<(PVTKF_X_DIM-1); i++)
		MatrixPhi[i][CTU_IDX] = cycle;

	return;
}


boolean IsCurPVTKFValid(void)
{
	return bPVTKFValid;
}

boolean IsMustbeMove(void)
{
	return bMustbeMove;
}

static void checkKfSVBuIdx(byte buflag, int8 navSysBuIdx[3])
{
	//gps sv
	navSysBuIdx[0] = GPS_BU_IDX;

	//bd sv
	if((buflag & BITMAP_DIM_BU_GPS_BD2) || (buflag & BITMAP_DIM_BU_GPS_BD2_GLO))
		navSysBuIdx[1] = GPS_BU_IDX;
	else
		navSysBuIdx[1] = BD_BU_IDX;

	//glo sv
#if SUPPORT_GLONASS
	if((buflag & BITMAP_DIM_BU_GPS_GLO) || (buflag & BITMAP_DIM_BU_GPS_BD2_GLO))
		navSysBuIdx[2] = GPS_BU_IDX;
	else if(buflag & BITMAP_DIM_BU_BD2_GLO)
		navSysBuIdx[2] = BD_BU_IDX;
	else
		navSysBuIdx[2] = GLO_BU_IDX;
#endif

	return;
}




	
#endif


