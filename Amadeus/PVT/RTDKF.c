
#include "RTDKF.h"
#include "RTKCalBaseline.h"
#include <math.h>
#include "PVTRcvrInfo.h"
#include "constdef.h"
#include "cfgpara.h"

#define SPEEDLIM	(1.0)

static bool bRTDKFValid = FALSE;
static BOOL RTDLSProc(RTK_EPOCH* pRtkEpoch, double* RTKDist_DD, ECEF* pBaseline);

#if ENABLE_RTD_KF

static bool GetRTDKfValid();
static void RTDUpdateMatrixQ(ECEF pos, ECEF vel, RTD_KF* pRTDKf);
static void RTDUpdateXAndPhi(ECEF vel, double cycle);
static void RTDUpdateR(RTK_EPOCH* pRtkEpoch, word256* pSVMap, int32 zDim, double* pDD_dds, double speed3D);
static void InitRTDKF(ECEF baseline);
static int32 RTDKFProc(ECEF pos, ECEF vel, double* pEDDPr, RTK_EPOCH* pRtkEpoch, ECEF* pBaseline);
static bool RTDCheckPosValid(ECEF* pRtkPos);
static void CalMatrixR(RTK_EPOCH* pRtkEpoch, double* pDD_dds, int32 zDim, double speed3D);
static double GetRTDWeigth(RTK_EPOCH* pRtkEpoch, int32 idx, double dds, double THdds, double* tmp_base);
static int32 RTDUpdateResiErrAndAlpha(double* pEDDPr, RTK_EPOCH* pRtkEpoch, double* pDD_dds);
static int32 RTDKFSVSelect(RTK_EPOCH* pRtkEpoch, word256* pSVMap, double* DD_dds);
static bool GetDDsflag(double* pDD_dds,int32 zDim,double speed3D);
#endif


bool CalRTDProc(RTK_EPOCH* pRtkEpoch, double* RTKDist_DD, ECEF* pBaseline, RTD_KF* pRTDKf)
{
	bool bRTDResValid= FALSE;
#if ENABLE_RTD_KF	
	ECEF vel, pos;
	double speed2D=0.0, heading=0.0;

	int32 fixflag = getRcvrInfo(&pos, &vel, NULL, NULL);
	
	if(pRTDKf==NULL)
		return FALSE;
	
	if((pRTDKf->bRTDKFValid==FALSE) || (fixflag<FIX_3D))	//wls
	{
		pRTDKf->bRTDKFValid = FALSE;
		bRTDResValid = RTDLSProc(pRtkEpoch, RTKDist_DD, &(pRtkEpoch->Baseline.dxyz));
		if(bRTDResValid)
		{
			InitRTDKF(pRtkEpoch->Baseline.dxyz, pRTDKf);
		}
	}
	else	//kf
	{	
		ECEFVel2SpeedHeading(&(pRtkEpoch->RovePVT_RTD),&vel,&speed2D,&heading);
		if (f_abs(speed2D)<SPEEDLIM)  //速度小于1m/s 则静止处理
			memset(&vel,0,sizeof(vel));
		bRTDResValid = RTDKFProc(Basefixposition.ecefpos, vel, RTKDist_DD, pRTDKf, pRtkEpoch, &(pRtkEpoch->Baseline.dxyz));
		if(!bRTDResValid)
			ResetRTDKF(pRTDKf);
	}
#else
	bRTDResValid = RTDLSProc(pRtkEpoch, RTKDist_DD, &(pRtkEpoch->Baseline.dxyz));
#endif

	return bRTDResValid;
}

//功能：计算双差浮点解
BOOL RTDLSProc(RTK_EPOCH* pRtkEpoch, double* RTKDist_DD, ECEF* pBaseline)
{
	int iteration = 0;
	int nCount = 0;
	int i = 0;
	int SatID = 0, sysidx;
	int n0 = pRtkEpoch->nCommonSatAMB;
	double	RTKDist_Base = 0;
	double	RTKDist_Rove = 0;
	double	RTKDist_SD[LAMBDA_N_DIM];
	double	temp1[LAMBDA_N_DIM*LAMBDA_N_DIM];
	double	temp2[LAMBDA_N_DIM*LAMBDA_N_DIM];
	double	del_L[LAMBDA_N_DIM];


	memset(RTKDist_SD,0,sizeof(double)*LAMBDA_N_DIM);
	memset(temp1,0,sizeof(double)*LAMBDA_N_DIM*LAMBDA_N_DIM);
	memset(temp2,0,sizeof(double)*LAMBDA_N_DIM*LAMBDA_N_DIM);
	memset(del_L,0,sizeof(double)*LAMBDA_N_DIM);

	//---------------------------1 求单历元浮点解---------------------------
	for(iteration=0; iteration<10; iteration++) //限制迭代10次
	{	//RtkEpoch.time.dSeconds
		nCount = 0; 	
		//计算星地间距离
		for(i=0; i<pRtkEpoch->nCommonSat; i++)
		{
			//基准站距离计算
			SatID=pRtkEpoch->CommonSat[i].SatID;

			RTKDist_Base = pRtkEpoch->DisSV2Base[i];

			//移动站距离计算
			RTKDist_Rove = RTK_GetSatDist(SatID, pRtkEpoch->Prm_Rove.obs[i].svpos, pRtkEpoch->RovePVT_RTD, 
				&temp1[nCount*3+0], &temp1[nCount*3+1], &temp1[nCount*3+2]);

			//单差
			RTKDist_SD[nCount] = RTKDist_Rove - RTKDist_Base;
			nCount++;
		}
		//构建O-C双差
		nCount = 0;
		for(i=0; i<pRtkEpoch->nCommonSat; i++)
		{
			sysidx = GetNavSysIdx(pRtkEpoch->CommonSat[i].SatID);
			if(i != pRtkEpoch->KeySatIdIndex[sysidx])
			{
				RTKDist_DD[nCount] = RTKDist_SD[i] - RTKDist_SD[pRtkEpoch->KeySatIdIndex[sysidx]];
				pRtkEpoch->AD[nCount*3+0] = temp1[i*3+0] - temp1[pRtkEpoch->KeySatIdIndex[sysidx]*3+0];
				pRtkEpoch->AD[nCount*3+1] = temp1[i*3+1] - temp1[pRtkEpoch->KeySatIdIndex[sysidx]*3+1];
				pRtkEpoch->AD[nCount*3+2] = temp1[i*3+2] - temp1[pRtkEpoch->KeySatIdIndex[sysidx]*3+2]; 					
				nCount++;
			}
		}
		//双差浮点解解算
		LAMBDA_MO_MatrixSub(pRtkEpoch->Range_DD, RTKDist_DD, del_L, 1, nCount);

		//双差浮点解解算(AD'*Pr*AD)^-1 * AD' * Pr * delL
		LAMBDA_MO_MatrixATPA(pRtkEpoch->AD, pRtkEpoch->RangeCovarianceInv, temp1, n0, 3);
		LAMBDA_MO_MatrixInv_LU(temp1, pRtkEpoch->ApaInv, 3);
		LAMBDA_MO_MatrixABT(pRtkEpoch->RangeCovarianceInv, del_L, temp1, n0, n0, 1);
		LAMBDA_MO_MatrixABT(pRtkEpoch->ApaInv, pRtkEpoch->AD, temp2, 3, 3, n0);
		LAMBDA_MO_MatrixMulti(temp2, temp1, del_L, 3, n0, 1);
		//更新坐标
		pRtkEpoch->RovePVT_RTD.x += del_L[0];
		pRtkEpoch->RovePVT_RTD.y += del_L[1];
		pRtkEpoch->RovePVT_RTD.z += del_L[2];
		temp1[0] = del_L[0]*del_L[0] + del_L[1]*del_L[1] + del_L[2]*del_L[2];
		if(temp1[0]<1E-5)
			break;		
	}
	
	if (iteration>=10)
	{
		if(temp1[0]>1.0)
			return FALSE;
	}

	pBaseline->x = pRtkEpoch->RovePVT_RTD.x - Basefixposition.ecefpos.x;
	pBaseline->y = pRtkEpoch->RovePVT_RTD.y - Basefixposition.ecefpos.y;
	pBaseline->z = pRtkEpoch->RovePVT_RTD.z - Basefixposition.ecefpos.z;
	
	return TRUE;
}

void ResetRTDKF(RTD_KF* pRTDKf)
{
	pRTDKf->bRTDKFValid = FALSE;
}


#if ENABLE_RTD_KF
void InitRTDKFModule(RTD_KF* pRTDKf)
{
	pRTDKf->bRTDKFValid = FALSE;
	memset(pRTDKf->VectorX_BL, 0, sizeof(pRTDKf->VectorX_BL));
	memset(pRTDKf->MatrixPhi_RTD, 0, sizeof(pRTDKf->MatrixPhi_RTD));
	memset(pRTDKf->MatrixP_RTD, 0, sizeof(pRTDKf->MatrixP_RTD));
	memset(pRTDKf->MatrixQ_RTD, 0, sizeof(pRTDKf->MatrixQ_RTD));
	memset(pRTDKf->MatrixH_RTD, 0, sizeof(pRTDKf->MatrixH_RTD));
	memset(pRTDKf->MatrixZ_RTD, 0, sizeof(pRTDKf->MatrixZ_RTD));
	memset(pRTDKf->MatrixR_RTD, 0, sizeof(pRTDKf->MatrixR_RTD));
	memset(pRTDKf->MatrixK_RTD, 0, sizeof(pRTDKf->MatrixK_RTD));

	return;
}

void InitRTDKF(ECEF baseline, RTD_KF* pRTDKf)
{
	int32 i=0;

	InitRTDKFModule(pRTDKf);
	
	//init X
	pRTDKf->VectorX_BL[0]=baseline.x;
	pRTDKf->VectorX_BL[1]=baseline.y;
	pRTDKf->VectorX_BL[2]=baseline.z;

	//init P
	for(i=0; i<KF_RTD_XDIM; i++)
	{
		pRTDKf->MatrixP_RTD[i*KF_RTD_XDIM+i]=10.0;
	}

	pRTDKf->bRTDKFValid = TRUE;

	return;
}


int32 RTDKFProc(ECEF pos, ECEF vel, double* pEDDPr, RTD_KF* pRTDKf, RTK_EPOCH* pRtkEpoch, ECEF* pBaseline)
{
	int32 zDim=0;
	word256 UseSVMap;
	double hpht[LAMBDA_N_DIM*LAMBDA_N_DIM];
	double pht[LAMBDA_N_DIM*KF_RTD_XDIM];
	double hphtPlusR[LAMBDA_N_DIM*LAMBDA_N_DIM];
	double hphtLU[LAMBDA_N_DIM*LAMBDA_N_DIM];
	double deltaX[KF_RTD_XDIM*KF_RTD_XDIM];
	double DD_dds[LAMBDA_N_DIM];
	double speed3D = 0.0;

	memset(&UseSVMap, 0, sizeof(UseSVMap));
	memset(hpht, 0, sizeof(hpht));
	memset(pht, 0, sizeof(pht));
	memset(hphtPlusR, 0, sizeof(hphtPlusR));
	memset(hphtLU, 0, sizeof(hphtLU));
	memset(deltaX,0,sizeof(deltaX));
	memset(DD_dds,0,sizeof(DD_dds));
	
	//updata x
	RTDUpdateXAndPhi(vel,FixCycle, pRTDKf);

	//get Q
	RTDUpdateMatrixQ(pos, vel, pRTDKf);

	//step2 P(k,k-1)=Phi(k)*P(k-1,k-1)*Trans(Phi(k))+Q(k)
	LAMBDA_MO_MatrixAPAT(pRTDKf->MatrixPhi_RTD, pRTDKf->MatrixP_RTD, deltaX, KF_RTD_XDIM, KF_RTD_XDIM);
	LAMBDA_MO_MatrixPlus(deltaX, pRTDKf->MatrixQ_RTD, pRTDKf->MatrixP_RTD, KF_RTD_XDIM, KF_RTD_XDIM);

	//update DD residue and H
	zDim = RTDUpdateResiErrAndAlpha(pEDDPr, pRtkEpoch, DD_dds, pRTDKf);
	if(zDim<3)
		return FALSE;

	//select sv, update H, Z, zDim
	zDim = RTDKFSVSelect(pRtkEpoch, &UseSVMap, DD_dds);

	if(zDim<3)
		return FALSE;

	//update R
	speed3D=sqrt((vel.x*vel.x+vel.y*vel.y+vel.z*vel.z));
	RTDUpdateR(pRtkEpoch, &UseSVMap, zDim, DD_dds, speed3D);

	//update K
	LAMBDA_MO_MatrixAPAT(pRTDKf->MatrixH_RTD, pRTDKf->MatrixP_RTD, hpht, zDim, KF_RTD_XDIM);  //HPH'  	 N*N
	LAMBDA_MO_MatrixPlus(hpht, pRTDKf->MatrixR_RTD, hphtPlusR, zDim, zDim);  //HPH'+R	N*N	
	LAMBDA_MO_MatrixInv_LU(hphtPlusR, hphtLU, zDim); //[HPH'+R]^(-1)		N*N
	LAMBDA_MO_MatrixABT(pRTDKf->MatrixP_RTD, pRTDKf->MatrixH_RTD, pht, KF_RTD_XDIM, KF_RTD_XDIM, zDim); //PH'	3*N
	LAMBDA_MO_MatrixMulti(pht,hphtLU ,pRTDKf->MatrixK_RTD, KF_RTD_XDIM ,zDim, zDim);//K=PH'[HPH'+R]^(-1)		3*N

	//step 4 calc Xk
	memset(pht, 0, sizeof(pht));
	LAMBDA_MO_MatrixMulti(pRTDKf->MatrixH_RTD, pRTDKf->VectorX_BL, pht, zDim, KF_RTD_XDIM, 1);	//HX		N*1
	memset(hpht, 0, sizeof(hpht));
	LAMBDA_MO_MatrixSub(pRTDKf->MatrixZ_RTD, pht, hpht,zDim, 1);	//Z-HX			N*1
	memset(deltaX, 0, sizeof(deltaX));
	LAMBDA_MO_MatrixMulti(pRTDKf->MatrixK_RTD, hpht, deltaX, KF_RTD_XDIM, zDim, 1);			//K(Z-HX)		3*1

	//update X
	pRTDKf->VectorX_BL[0] += deltaX[0];
	pRTDKf->VectorX_BL[1] += deltaX[1];
	pRTDKf->VectorX_BL[2] += deltaX[2];

	//update P
	LAMBDA_MO_MatrixMulti(pRTDKf->MatrixK_RTD, pRTDKf->MatrixH_RTD, hpht,KF_RTD_XDIM, zDim, KF_RTD_XDIM);	//KH   3*3
	LAMBDA_Matrix_Eye(deltaX, KF_RTD_XDIM);	//I 	3*3
	LAMBDA_MO_MatrixSub(deltaX, hpht, deltaX, KF_RTD_XDIM, KF_RTD_XDIM);	//I-KH	3*3
	LAMBDA_MO_MatrixMulti(deltaX, pRTDKf->MatrixP_RTD, hpht, KF_RTD_XDIM, KF_RTD_XDIM, KF_RTD_XDIM); 	//P=(I-KH)P   3*3
	memcpy(pRTDKf->MatrixP_RTD, hpht, KF_RTD_XDIM*KF_RTD_XDIM*sizeof(double));

	//output
	pRtkEpoch->RovePVT_RTD.x = Basefixposition.ecefpos.x + pRTDKf->VectorX_BL[0];
	pRtkEpoch->RovePVT_RTD.y = Basefixposition.ecefpos.y + pRTDKf->VectorX_BL[1];
	pRtkEpoch->RovePVT_RTD.z = Basefixposition.ecefpos.z + pRTDKf->VectorX_BL[2];

	pBaseline->x = pRTDKf->VectorX_BL[0];
	pBaseline->y = pRTDKf->VectorX_BL[1];
	pBaseline->z = pRTDKf->VectorX_BL[2];

	if(IsRTKOpen())
	{
		memcpy(pRtkEpoch->AD, pRTDKf->MatrixH_RTD, sizeof(pRTDKf->MatrixH_RTD));
		LAMBDA_MO_MatrixATPA(pRTDKf->MatrixH_RTD, pRTDKf->MatrixR_RTD, deltaX, zDim,KF_RTD_XDIM);
		LAMBDA_MO_MatrixInv_LU(deltaX, pRtkEpoch->ApaInv, KF_RTD_XDIM);
	}
	

	return TRUE;
}

void RTDUpdateXAndPhi(ECEF vel, double cycle, RTD_KF* pRTDKf)
{
	pRTDKf->MatrixPhi_RTD[0] = 1+vel.x*cycle/pRTDKf->VectorX_BL[0];
	pRTDKf->MatrixPhi_RTD[4] = 1+vel.y*cycle/pRTDKf->VectorX_BL[1];
	pRTDKf->MatrixPhi_RTD[8] = 1+vel.z*cycle/pRTDKf->VectorX_BL[2];

	pRTDKf->VectorX_BL[0] += vel.x*cycle;
 	pRTDKf->VectorX_BL[1] += vel.y*cycle;
 	pRTDKf->VectorX_BL[2] += vel.z*cycle;

	return;
}

void RTDUpdateMatrixQ(ECEF pos, ECEF vel, RTD_KF* pRTDKf)
{
	WGS wgs = {0,};
	double MatrixEnu2Ecef[9] = {0.0,}, MatrixEnuNoise[9]={0.0,},MatrixEnuNoiseT[3]={0.0,};
	double sinLon =0.0, cosLon= 0.0, sinLat = 0.0, cosLat=0.0;
	double tempBaseline=0.0;
	double posNoiseENU[3]={2.0,2.0,5.0};	//定位误差，东北天	m
	double velNoiseENU[3]={2.0,2.0,5.0};	//测速误差，东北天	m/s
	double blNoiseENU[3]={2.0,2.0,5.0};		//基线误差，东北天 	m/km

	NEH speedENU={0.0,};

	//delT = FixCycle;
	memset(pRTDKf->MatrixQ_RTD,0,sizeof(pRTDKf->MatrixQ_RTD));
	
	ECEFVel2ENU(&pos,&vel,&speedENU); //ecef速度转enu

	//enu 2 ecef transfromation matrix
	ECEF2WGS(&pos, &wgs);

	sinLon = sin(wgs.lon);
	cosLon = cos(wgs.lon);
	sinLat = sin(wgs.lat);
	cosLat = cos(wgs.lat);
	
 	MatrixEnu2Ecef[0]=-sinLon;
 	MatrixEnu2Ecef[1]=-cosLon*sinLat;
	MatrixEnu2Ecef[2]=cosLon*cosLat;
	MatrixEnu2Ecef[3]=cosLon;
	MatrixEnu2Ecef[4]=-sinLon*sinLat;
	MatrixEnu2Ecef[5]=sinLon*cosLat;
	MatrixEnu2Ecef[6]=0;
	MatrixEnu2Ecef[7]=cosLat;
	MatrixEnu2Ecef[8]=sinLat;

	tempBaseline = GetNorm2(pRTDKf->VectorX_BL,3,1); 
// 	MatrixEnuNoise[0] = delT*f_abs(speedENU[0]) + noiseENU[0] + tempBaseline/1000*noiseENU[0]/10; //每1km增加误差
// 	MatrixEnuNoise[4] = delT*f_abs(speedENU[1]) + noiseENU[1] + tempBaseline/1000*noiseENU[1]/10;
// 	MatrixEnuNoise[8] = delT*f_abs(speedENU[2]) + noiseENU[2] + tempBaseline/1000*noiseENU[2]/10;

	MatrixEnuNoiseT[0] = velNoiseENU[0]*f_abs(speedENU.east)  + posNoiseENU[0] + tempBaseline/1000*blNoiseENU[0]; //速度引起误差 + 定位误差 + 基线引起误差（每1km增加误差）
	MatrixEnuNoiseT[1] = velNoiseENU[1]*f_abs(speedENU.north) + posNoiseENU[1] + tempBaseline/1000*blNoiseENU[1];
	MatrixEnuNoiseT[2] = velNoiseENU[2]*f_abs(speedENU.head)  + posNoiseENU[2] + tempBaseline/1000*blNoiseENU[2];

	//LAMBDA_MO_MatrixABT(MatrixEnuNoiseT,MatrixEnuNoiseT,MatrixEnuNoise,KF_RTD_XDIM,1,KF_RTD_XDIM);
	MatrixEnuNoise[0] = MatrixEnuNoiseT[0]*MatrixEnuNoiseT[0];
	MatrixEnuNoise[4] = MatrixEnuNoiseT[1]*MatrixEnuNoiseT[1];
	MatrixEnuNoise[8] = MatrixEnuNoiseT[2]*MatrixEnuNoiseT[2];

	//MatrixEnu2Ecef * MatrixEnuNoise * MatrixEnu2Ecef'
#if 1
	LAMBDA_MO_MatrixAPAT(MatrixEnu2Ecef,MatrixEnuNoise,pRTDKf->MatrixQ_RTD,KF_RTD_XDIM,KF_RTD_XDIM);
#else
	LAMBDA_MO_MatrixMulti(MatrixEnu2Ecef,MatrixEnuNoise,MatrixQ_RTD,KF_RTD_XDIM,KF_RTD_XDIM,KF_RTD_XDIM);
#endif

// 	LAMBDA_Matrix_Eye(MatrixQ_RTD, KF_RTD_XDIM);
// 
//  	MatrixQ_RTD[KF_RTD_XDIM*0+0]=(MatrixEcefNoise[0]);
//  	MatrixQ_RTD[KF_RTD_XDIM*1+1]=(MatrixEcefNoise[1]);
//  	MatrixQ_RTD[KF_RTD_XDIM*2+2]=(MatrixEcefNoise[2]);

	return;
}

int32 RTDKFSVSelect(RTK_EPOCH* pRtkEpoch, word256* pSVMap, double* DD_dds)
{
	int32 i=0, zDim=0, svid;
	//double Z_new[LAMBDA_N_DIM] = {0.0,};
	//double H_new[LAMBDA_N_DIM*3] = {0.0,};
	//double D_new[LAMBDA_N_DIM] = {0.0,};
	
	for(i=0; i<pRtkEpoch->nCommonSatAMB; i++)
	{
		svid = pRtkEpoch->CommonSat_Amb[i].SatID;

// 		if(f_abs(DD_dds[i])>8.0)
// 			continue;
		
		SetBitWord256(pSVMap,svid-1);
		//Z_new[zDim] = MatrixZ_RTD[i];
		//H_new[zDim*3+0] = MatrixH_RTD[i*3+0];
		//H_new[zDim*3+1] = MatrixH_RTD[i*3+1];
		//H_new[zDim*3+2] = MatrixH_RTD[i*3+2];
		//D_new[zDim] = DD_dds[i];
		zDim++;
	}

// 	if(zDim*3>pRtkEpoch->nCommonSatAMB*2)  //
// 	{
// 		memcpy(MatrixZ_RTD,Z_new,sizeof(double)*zDim);
// 		memcpy(MatrixH_RTD,H_new,sizeof(double)*zDim*3);
// 		memcpy(DD_dds,D_new,sizeof(double)*zDim);
// 	}
// 	else
//		zDim = pRtkEpoch->nCommonSatAMB;   //剔星个数多余原来1/3 ，则取消剔星


	return zDim;
}

int32 RTDUpdateResiErrAndAlpha(double* pEDDPr, RTK_EPOCH* pRtkEpoch, double* pDD_dds, RTD_KF* pRTDKf)
{
	int32 i=0, nCount = 0, sysidx, SatID;
	ECEF rcvrpos={0.0,};
	double RTKDist_Base=0.0, RTKDist_Rove=0.0;
	double	temp1[LAMBDA_N_DIM*3];
	double	RTKDist_SD[LAMBDA_N_DIM];

	memset(RTKDist_SD,0,sizeof(RTKDist_SD));
	
	rcvrpos.x = Basefixposition.ecefpos.x + pRTDKf->VectorX_BL[0];
	rcvrpos.y = Basefixposition.ecefpos.y + pRTDKf->VectorX_BL[1];
	rcvrpos.z = Basefixposition.ecefpos.z + pRTDKf->VectorX_BL[2];

	for(i=0; i<pRtkEpoch->nCommonSat; i++)
	{
		//基准站距离计算
		SatID=pRtkEpoch->CommonSat[i].SatID;
		RTKDist_Base = RTK_GetSatDist(SatID, pRtkEpoch->Prm_Rove.obs[i].svpos, Basefixposition.ecefpos, 
			&temp1[i*3+0], &temp1[i*3+1], &temp1[i*3+2]);

		//移动站距离计算
		RTKDist_Rove = RTK_GetSatDist(SatID, pRtkEpoch->Prm_Rove.obs[i].svpos, rcvrpos, 
			&temp1[i*3+0], &temp1[i*3+1], &temp1[i*3+2]);

		//单差
		RTKDist_SD[i] = RTKDist_Rove - RTKDist_Base;
	}
	
	//构建O-C双差
	nCount = 0;
	for(i=0; i<pRtkEpoch->nCommonSat; i++)
	{
		sysidx = GetNavSysIdx(pRtkEpoch->CommonSat[i].SatID);
		if(i != pRtkEpoch->KeySatIdIndex[sysidx])
		{
			pEDDPr[nCount] = RTKDist_SD[i] - RTKDist_SD[pRtkEpoch->KeySatIdIndex[sysidx]];
			pRTDKf->MatrixH_RTD[nCount*3+0] = temp1[i*3+0] - temp1[pRtkEpoch->KeySatIdIndex[sysidx]*3+0];
			pRTDKf->MatrixH_RTD[nCount*3+1] = temp1[i*3+1] - temp1[pRtkEpoch->KeySatIdIndex[sysidx]*3+1];
			pRTDKf->MatrixH_RTD[nCount*3+2] = temp1[i*3+2] - temp1[pRtkEpoch->KeySatIdIndex[sysidx]*3+2]; 

			//MatrixZ_RTD[nCount] = pRtkEpoch->Range_DD[nCount] - pEDDPr[nCount];
			pDD_dds[nCount]= pRtkEpoch->Range_DD[nCount] - pEDDPr[nCount];
			pRTDKf->MatrixZ_RTD[nCount] = pRtkEpoch->Range_DD[nCount];
			
			nCount++;
		}
	}

	return nCount;
}


void RTDUpdateR(RTK_EPOCH* pRtkEpoch, word256* pSVMap, int32 zDim, double* pDD_dds, double speed3D, RTD_KF* pRTDKf)
{

	int32 i=0, svid, nCount=0;

	memset(pRTDKf->MatrixR_RTD, 0, sizeof(pRTDKf->MatrixR_RTD));

	for(i=0; i<pRtkEpoch->nCommonSatAMB; i++)
	{
		svid = pRtkEpoch->CommonSat_Amb[i].SatID;
		
		if(GetBitWord256(pSVMap,svid-1)==0)
			continue;

		//MatrixR_RTD[nCount*zDim + nCount] = 1.0; //test

		nCount++;
	}
	CalMatrixR(pRtkEpoch, pDD_dds,zDim,speed3D,pRTDKf);

	return;
}

void CalMatrixR(RTK_EPOCH* pRtkEpoch, double* pDD_dds, int32 zDim, double speed3D, RTD_KF* pRTDKf)
{
	int32 i = 0 ,sysidx = 0,nCount = 0;
	double tmp = 1.0, tmp_cn0_el = 1.0, tmp_base = 0.01;
	bool flag_dds = FALSE;
	double THdds=0.0;

	//由速度 计算残差门限
	if (speed3D>SPEEDLIM)
		THdds = 10.0;  //1m/s以下静态处理，静态门限为5m，速度每增加1，门限增加2
	else
		THdds = 5.0;
	
	flag_dds = GetDDsflag(pDD_dds,zDim,THdds);
	
	for(i=0; i<pRtkEpoch->nCommonSat; i++)
	{
		sysidx = GetNavSysIdx(pRtkEpoch->CommonSat[i].SatID);

		if(i != pRtkEpoch->KeySatIdIndex[sysidx])
		{
			//tmp = pDD_dds[nCount]* pDD_dds[nCount];
			tmp = 1.0;
			if(flag_dds == TRUE)
				tmp_cn0_el =(GetRTDWeigth(pRtkEpoch,pRtkEpoch->KeySatIdIndex[sysidx],NULL,THdds,&tmp_base)/GetRTDWeigth(pRtkEpoch,i,pDD_dds[nCount],THdds,&tmp_base));
			else
				tmp_cn0_el = (GetRTDWeigth(pRtkEpoch,pRtkEpoch->KeySatIdIndex[sysidx],NULL,THdds,&tmp_base)/GetRTDWeigth(pRtkEpoch,i,NULL,THdds,&tmp_base));
			
			pRTDKf->MatrixR_RTD[nCount*zDim + nCount] = tmp* tmp_cn0_el*tmp_base;
			nCount ++;
		}
	}
}

bool GetDDsflag(double* pDD_dds,int32 zDim,double THdds)
{
	int idx = 0,counti = 0;

	for(idx = 0; idx<zDim; idx++)
	{
		if(abs(pDD_dds[idx]<THdds))
			counti++;
	}

	if(counti*3 >= zDim*2)
		return TRUE;
	else   //dds 1/3超过门限，重新初始化
		return FALSE;
}

double GetRTDWeigth(RTK_EPOCH* pRtkEpoch, int32 idx, double dds, double THdds, double* ptmp_base)
{
	double cn0 = (double)(pRtkEpoch->CommonSat[idx].cn0);
	double el = (double)(pRtkEpoch->Prm_Rove.obs[idx].el);
	double tmp = 0.0;

	//tmp = (0.2 * cn0 +  el) * 0.85;
	if(cn0>40)
		tmp = el + sqrt(cn0 - 40) ;
	else if(cn0>=30)
		tmp = el - ((40 -cn0)* (40 -cn0))/2 ;
	else
		tmp = el - 50 - sqrt(30 - cn0);
	
	if(SV_IsBd2Geo(RtkEpoch.CommonSat[idx].SatID))
		tmp -= 5;
	if(SV_IsBd2(RtkEpoch.CommonSat[idx].SatID))
		tmp -= 15;

	if(ptmp_base!=NULL)
		*ptmp_base = 0.01;

	if(dds!=NULL)
	{
		if(abs(dds)>=THdds)		//大于门限，权值降为1
		{
			tmp = 1;
			if(ptmp_base!=NULL)
				*ptmp_base = 100;
		}
// 		else if(abs(dds)>=1)
// 			//tmp += -12.5 + ((abs(dds)-THdds) * (abs(dds)-THdds)) * (16/(THdds*THdds-2*THdds+1));	//抛物线门限
// 			tmp += 3.5 - ((abs(dds)-1) * (abs(dds)-1)) * (16/(THdds*THdds-2*THdds+1));	//抛物线门限
// 		else		//残差小于1m，则权值增加
// 			tmp += 3.5;
	}
	if(tmp <= 0)
	{
		tmp = 1;
		if(ptmp_base!=NULL)
			*ptmp_base = 100;
	}
	if(tmp >= 90)
		tmp = 90;
	
	tmp = tmp*D2R;
	tmp = sin(tmp);
	
	return tmp;

}


bool RTDCheckPosValid(ECEF* pRtkPos)
{
	ECEF sigpos={0.0,0.0,0.0};
	double dif = 0.0;

	if(getRcvrInfo(&sigpos, NULL, NULL, NULL)>=FIX_3D)
	{
		dif = (pRtkPos->x-sigpos.x)*(pRtkPos->x-sigpos.x) + (pRtkPos->y-sigpos.y)*(pRtkPos->y-sigpos.y) + (pRtkPos->z-sigpos.z)*(pRtkPos->z-sigpos.z);
		if(dif<10000.0)	//100m
			return TRUE;
	}

	return FALSE;
}
#endif


