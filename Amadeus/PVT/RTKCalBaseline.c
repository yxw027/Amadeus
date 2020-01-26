/*******************************************************************
Company:   hwacreate
Engineer:  
Create Date: 2015.12.01
File  Name:  RTKCalBaseline.c
Description: calculate baseline & differential positioning
Function List: 
version: V1.0
Revision Date:
Modifier: 
Additional Comments: 
********************************************************************/
#include <math.h>
#include "RTKCalBaseline.h"
#include "coordinate.h"
#include "pvt.h"
#include "timeproc.h"
#include "cfgpara.h"
#include "constdef.h"
#include "pvtrcvrinfo.h"
#include "RTDKF.h"
#include "CalcSvPVT.h"

#ifndef _POSTPROC
#ifdef _SIMULATE
#include "dataprocess.h"
#else
#include "GnssTYProcfg.h"
#endif
#endif

#define CAL_BASELINE_WLS	1
/*--------------------------------------------------------------------------------------
// 全局变量
--------------------------------------------------------------------------------------*/
// 是否是每一次计算整周模糊度
CHAR DGPS_MODE = DGPS_MODE_STATC;

//当前时间(输出调试信息时用)
double g_dSecond = 0;
unsigned int g_nSetAmbN=0;


BOOL g_bSmoothAtt = FALSE;
int g_nSmoothindow = 1*30;
int g_nOIC = 1;
double g_dBaseLineLen = -1;
CHAR g_dB1B2Switch = 1;//1：切换为B1,2：切换为B2；
CHAR g_dL1L2Switch = 1;//1：切换为L1,2：切换为L2；


//根据载噪比过滤卫星
//static BOOL RTKCoaseSVSelect(OBST* prmObv, word32* pValidFrqpiont, double* pSnr, double* pLLI);
static void SelectRTKFrqpoint(bool bSuperWide, word32 frqpoint, word32* pSupWideLaneFP, word32* pWideLaneFP, word32* pSingleFP, RTK_EPOCH* pRtkEpoch);
static bool RTK_GetCurEpochTime(RTK_EPOCH* pRtkEpoch, double* pTime);
static int32 CalcCommSVCntByFreqPoint(int32 navsys, word32 frqpoint, RTK_EPOCH* pRtkEpoch);
static bool IsCurCommKeyIdx(int32 idx, RTK_EPOCH* pRtkEpoch);
static int32 RTKSelectSVL1(RTK_EPOCH* pRtkEpoch, word256* pSVMap, double ddres[SV_NUM_TRUE], int32* errcount);
static int32 CheckRTKResultNW(RTK_EPOCH* pRtkEpoch, RTK_EPOCH_Keep* pRtkEpochKeep, double* ddres, word256 UsedSVMapNW);
static bool RTKCheckPosValid(ECEF* pRtkPos, RTK_EPOCH* pRtkEpoch);
static void DeleteSlipSV(RTK_EPOCH* pRtkEpoch, RTK_EPOCH_Keep* pRtkEpochKeep);
static int32 CalcRTKAverAndStd(RTK_EPOCH_Keep* pRtkEpochKeep, ECEF* pPos, int32 windowLen, ECEF* pAver, ECEF* pStd);
static double GetRTKStdChkTh(void);
static void InitalFloatN(RTK_EPOCH* pRtkEpoch, double* pEDDPr, double* pAMB_PrDD);
static void UpdateDistSV2Base(RTK_EPOCH* pRtkEpoch);
static void UpdateSvPosForDifEph(RTK_EPOCH* pRtkEpoch);
static void CheckAllCommSVGEO(RTK_EPOCH* pRtkEpoch);

//functions about N extend
static int32 RTKSVSelectEN(RTK_EPOCH* pRtkEpoch, RTK_EPOCH_Keep* pRtkEpochKeep, word256* pSVValidMap, double* pN, int32 flag);
static void UpdateCommSV_N(RTK_EPOCH* pRtkEpoch, RTK_EPOCH_Keep* pRtkEpochKeep);
static bool CalNewSvN(double measDDc, double estDDm, double wavelen, double threshold, int32* pN, double* pNf);
static void CalcBaselineByPos(ECEF* pPos, ECEF* pbaseline);
static int32 CheckRTKResultNWF(RTK_EPOCH* pRtkEpoch, double dRatio, double* ddres, word256 UsedSVMapNWF);
static int32 CheckExtRTKRes(RTK_EPOCH* pRtkEpoch, word256* pSVValidMap, double* ddres, double threshold, bool* allmin, bool* bSecCalc);
static void RtkNW2RtkNS(RTK_EPOCH* pRtkEpoch, int32 svENValidCnt, word256* svMapEN, double* pNW, double* pNS, double* pWavelenNW, double *pWavelenNS);
static int32 SelectNWSVByL1Residue(RTK_EPOCH* pRtkEpoch,word256* svMapEN, double*ddResL1, int32 svUsed);
static bool IsAmbResolution(RTK_EPOCH* pRtkEpoch);
static int32 GetMaxRtkSvNum(bool bLastIntValid);

#if CAL_BASELINE_WLS
static double GetRTKLsWeigth(int32 idx, RTK_EPOCH* pRtkEpoch);
#endif

//--------------------------------函数--实现--不被其它文件调用------------------------

//功能：根据双差相位及整周模糊度计算基线
BOOL CalBaseline_ECEF(RTK_EPOCH* pRtkEpoch, double* dPhase_DD, double* pN, double* pWavelen,ECEF* pOutputPos, word256* pUsedSVMapL1, double* pDDRes)
{
	int i = 0, j = 0, sysidx, nCount=0, row=0,jCount = 0;
	double rou_Base[LAMBDA_N_DIM];
	double rou_Rove[LAMBDA_N_DIM];
	double del_L[LAMBDA_N_DIM];
	double del_L_temp[LAMBDA_N_DIM];
	double range_error[LAMBDA_N_DIM];
#if CAL_BASELINE_WLS
	double R_var[LAMBDA_N_DIM*LAMBDA_N_DIM],rev_R_var[LAMBDA_N_DIM*LAMBDA_N_DIM];
	double mtemp[3*LAMBDA_N_DIM];
#endif
	double ad_error = 0;
	char iteration=0;
	ECEF rovepos;
	ECEF delta_key[MAX_SYSM]={0.0,};
	double direction_x, direction_y, direction_z;
	int32 iteration_cnt=10;
	double iteration_res_th=1E-15;

	memset(rou_Base,0,sizeof(double)*LAMBDA_N_DIM);
	memset(rou_Rove,0,sizeof(double)*LAMBDA_N_DIM);
	memset(del_L,0,sizeof(double)*LAMBDA_N_DIM);
	memset(del_L_temp,0,sizeof(double)*LAMBDA_N_DIM);
	memset(range_error,0,sizeof(double)*LAMBDA_N_DIM);
#if CAL_BASELINE_WLS
	memset(R_var,0,sizeof(double)*LAMBDA_N_DIM*LAMBDA_N_DIM);
	memset(rev_R_var,0,sizeof(double)*LAMBDA_N_DIM*LAMBDA_N_DIM);
	memset(mtemp,0,sizeof(double)*3*LAMBDA_N_DIM);
#endif

#ifdef _SIMULATE
	memcpy(&(pRtkEpoch->L1UsedSVMap), pUsedSVMapL1, sizeof(word256));
#endif

	rovepos.x = pRtkEpoch->RovePVT_RTD.x;
	rovepos.y = pRtkEpoch->RovePVT_RTD.y;
	rovepos.z = pRtkEpoch->RovePVT_RTD.z;	

	for(iteration=0; iteration<iteration_cnt; iteration++)
	{
		row=0;
	    //计算基准星的星地距离
		for(i=0; i<pRtkEpoch->nCommonSat; i++)
		{
			if(GetBitWord256(pUsedSVMapL1,pRtkEpoch->CommonSat[i].SatID-1)==0)
				continue;

			rou_Base[i] = pRtkEpoch->DisSV2Base[i];		
			rou_Rove[i] = RTK_GetSatDist(pRtkEpoch->CommonSat[i].SatID, pRtkEpoch->Prm_Rove.obs[i].svpos, rovepos, &direction_x, &direction_y, &direction_z);

			pRtkEpoch->AD[i*3+0] = direction_x;
			pRtkEpoch->AD[i*3+1] = direction_y;
			pRtkEpoch->AD[i*3+2] = direction_z;

			sysidx = GetNavSysIdx(pRtkEpoch->CommonSat[i].SatID);
			if(i==pRtkEpoch->KeySatIdIndex[sysidx])
			{
				delta_key[sysidx].x = direction_x;
				delta_key[sysidx].y = direction_y;
				delta_key[sysidx].z = direction_z;
			}
			else
				row++;
		}		


		//计算其它卫星的星地距离及方向矢量
		j=0;nCount=0;
		for(i=0; i<pRtkEpoch->nCommonSat; i++)
		{	
			sysidx = GetNavSysIdx(pRtkEpoch->CommonSat[i].SatID);
			
			if(i==pRtkEpoch->KeySatIdIndex[sysidx])
			{
				continue;
			}

			if(GetBitWord256(pUsedSVMapL1,pRtkEpoch->CommonSat[i].SatID-1)==0)
			{
				j++;
				continue;
			}
			
			//星地距离
			pRtkEpoch->AD[nCount*3+0] = pRtkEpoch->AD[i*3+0]-delta_key[sysidx].x;
			pRtkEpoch->AD[nCount*3+1] = pRtkEpoch->AD[i*3+1]-delta_key[sysidx].y;
			pRtkEpoch->AD[nCount*3+2] = pRtkEpoch->AD[i*3+2]-delta_key[sysidx].z;

#if CAL_BASELINE_WLS
			if(iteration==0)
			{
				for (jCount = 0; jCount< row; jCount++)
				{
					if(nCount != jCount)
					{
						rev_R_var[nCount*row + jCount] = 1;
					}
				}
				rev_R_var[nCount*row+nCount] = 1 + (GetRTKLsWeigth(pRtkEpoch->KeySatIdIndex[sysidx], pRtkEpoch)/GetRTKLsWeigth(i, pRtkEpoch));
			}
#endif

			del_L[nCount] = (dPhase_DD[j] - pN[j])*pWavelen[j]
				- ((rou_Rove[i] - rou_Rove[pRtkEpoch->KeySatIdIndex[sysidx]]) - (rou_Base[i] - rou_Base[pRtkEpoch->KeySatIdIndex[sysidx]]));	

			if(pDDRes!=NULL)
				pDDRes[pRtkEpoch->CommonSat[i].SatID-1] = f_abs(del_L[nCount]/pWavelen[j]);
			
			j++;
			nCount++;
		}
#if CAL_BASELINE_WLS
		if(iteration==0)
			LAMBDA_MO_MatrixInv_LU(rev_R_var,R_var,row);
#endif
		//LS解算
		if (nCount>=3)
		{
#if CAL_BASELINE_WLS
			LAMBDA_MO_MatrixATPA(pRtkEpoch->AD, R_var, pRtkEpoch->ADD, nCount, 3);
			LAMBDA_MO_MatrixInv_LU(pRtkEpoch->ADD, pRtkEpoch->ApaInv, 3);
			LAMBDA_MO_MatrixABT(pRtkEpoch->ApaInv, pRtkEpoch->AD, mtemp, 3, 3, nCount);
			LAMBDA_MO_MatrixMulti(mtemp, R_var, pRtkEpoch->ApaInv, 3, nCount, nCount);
			LAMBDA_MO_MatrixMulti(pRtkEpoch->ApaInv, del_L, range_error, 3, nCount, 1);
#else		
			LAMBDA_MO_MatrixATB(pRtkEpoch->AD, pRtkEpoch->AD, pRtkEpoch->ADD, nCount, 3, 3);
			LAMBDA_MO_MatrixInv_LU(pRtkEpoch->ADD, pRtkEpoch->ApaInv, 3);
			LAMBDA_MO_MatrixATB(pRtkEpoch->AD, del_L, del_L_temp, nCount, 3, 1);
			LAMBDA_MO_MatrixMulti(pRtkEpoch->ApaInv, del_L_temp, range_error, 3, 3, 1);
#endif
			//更新位置
			rovepos.x += range_error[0];
			rovepos.y += range_error[1];
			rovepos.z += range_error[2];
			//计算误差
			ad_error = range_error[0]*range_error[0]
							+range_error[1]*range_error[1]
							+range_error[2]*range_error[2];
			if(ad_error<iteration_res_th)
			{
				break;
			}
		}
	}

	if(iteration == iteration_cnt)
		return FALSE;

	pOutputPos->x = rovepos.x;
	pOutputPos->y = rovepos.y;
	pOutputPos->z = rovepos.z;
	
	return TRUE;
}


//功能：基线长度校验
BOOL Att_Check_BSL(RTK_EPOCH* pRtkEpoch)
{
	double dBslErr = 0;

	if (g_dBaseLineLen==-1)		return FALSE;

	dBslErr = fabs(pRtkEpoch->Baseline.BaselineLongth - g_dBaseLineLen);

	return (dBslErr>LAMBDA_BSL_ERR) ? FALSE : TRUE;
}


//功能：基线、姿态计算及校验
int8 AttCal_UseAMB(RTK_EPOCH* pRtkEpoch, RTK_EPOCH_Keep* pRtkEpochKeep, double* dAMB, ECEF* pPos, ECEF* baseline, int32 obsFlag, int32 dAMBFlag)
{
	// 可用卫星数据，剔除周跳卫星和整周模糊度未知或未保存的卫星
	double* pPhaseDD;
	double* pLambda;
	double ddRes[SV_NUM_TRUE];

	int32 svused1=0, svused2=0, sverror=0;
	BOOL bCalBSL = FALSE;
	word256 usedSVMapL1={{0,0,0,0,0,0,0,0}};	//单频一次选星结果
	word256 usedSVMapL1_2={{0,0,0,0,0,0,0,0}};	//单频二次选星结果
	int8 chkres=0;

	memset(ddRes, 0, sizeof(double)*SV_NUM_TRUE);

	if(obsFlag==OBS_SINGLE)
	{
		pPhaseDD = pRtkEpoch->Phase_DD_NS;
		pLambda = pRtkEpoch->wavelen_NS;
	}
	else if(obsFlag==OBS_WIDE)
	{
		if(pRtkEpoch->SelectFrepMapL1 == pRtkEpoch->SelectFrepMap)	//no wide
			return 0;

		pPhaseDD = pRtkEpoch->Phase_DD_NW;
		pLambda = pRtkEpoch->wavelen_NW;
	}
	else if(obsFlag==OBS_NARROW)
	{
		if(pRtkEpoch->SelectFrepMapL1 == pRtkEpoch->SelectFrepMap)	//no narrow
			return 0;

		//pPhaseDD = RtkEpoch.Phase_DD_NW;
	}
#if SUPPORT_SUPER_WIDE_LANE
	else if(obsFlag==OBS_SWIDE)
	{
		if(pRtkEpoch->bSuperWide && (pRtkEpoch->SelectFrepMapSW !=0))
		{
			pPhaseDD = pRtkEpoch->Phase_DD_SNW;
			pLambda = pRtkEpoch->wavelen_SNW;
		}
		else
			return 0;
	}
#endif
	else 
		return 0;
	
	svused1 = RTKSelectSVL1(pRtkEpoch, &usedSVMapL1, NULL, NULL);

	if(CalBaseline_ECEF(pRtkEpoch, pPhaseDD, dAMB, pLambda, pPos, &usedSVMapL1, ddRes) 
		&& RTKCheckPosValid(pPos, pRtkEpoch))
	{
		#ifdef _SIMULATE
		memcpy(pRtkEpoch->PhaRes, ddRes, sizeof(ddRes));
		#endif
		if((obsFlag==OBS_SINGLE) && (dAMBFlag == AMB_INT))
		{
			svused2 = RTKSelectSVL1(pRtkEpoch, &usedSVMapL1_2, ddRes, &sverror);
			if((svused1!=svused2) && (svused1>(sverror*2)) && (svused2>=RTK_MIN_OBSCNT))
			{
				memset(ddRes, 0, sizeof(double)*SV_NUM_TRUE);
				if(CalBaseline_ECEF(pRtkEpoch, pPhaseDD, dAMB, pLambda, pPos, &usedSVMapL1_2,ddRes) 
					&& RTKCheckPosValid(pPos, pRtkEpoch))
				{
					bCalBSL = TRUE;
					memcpy(&usedSVMapL1, &usedSVMapL1_2, sizeof(usedSVMapL1_2));
				}
				else
					bCalBSL = FALSE;
			}
			else
			{
				bCalBSL = TRUE;
			}
		}
		else
			bCalBSL = TRUE;
	}

	// result check and calculate baseline
	if(bCalBSL)
	{
		// check the RTK result
		if(dAMBFlag == AMB_INT)	//INT
		{
			if(obsFlag == OBS_SINGLE)
				chkres = CheckRTKResultL1(pRtkEpoch, pRtkEpochKeep, ddRes, usedSVMapL1);	
			else if(obsFlag == OBS_WIDE)
			{
				chkres = CheckRTKResultNW(pRtkEpoch, pRtkEpochKeep, ddRes, usedSVMapL1);	//need to modify		
			}
			else if(obsFlag == OBS_SWIDE)
			{
				pRtkEpochKeep->bFirstCalAmb = FALSE;
				chkres = 1;	//need to modify
			}
		}
		else	//FLOAT
		{
			chkres = CheckRTKResultNWF(pRtkEpoch,pRtkEpoch->dRatio, ddRes, usedSVMapL1);	//need to modify
		}
		
		if (chkres>0)
			memcpy(&(pRtkEpoch->L1UsedSVMap),&usedSVMapL1,sizeof(pRtkEpoch->L1UsedSVMap));
			
		CalcBaselineByPos(pPos, baseline);
	}
	
	return chkres;
}

int32 CheckRTKResultNW(RTK_EPOCH* pRtkEpoch, RTK_EPOCH_Keep* pRtkEpochKeep, double* ddres, word256 UsedSVMapNW)
{
	int32 idx= 0, svid = 0, sysidx =0, goodsvcnt=0, wholesv=0;
	bool bKeyChange= FALSE, bNChkFail_last=FALSE;
	double sumRes=0.0, meanRes=0.0;

	static int32 NKeepConValidcnt=0;
	int32 NkeepValidSVCnt = 0;
	double dRatio = pRtkEpoch->dRatio;

	//check ratio
	if((dRatio>4.5)|| ((dRatio>2.5) && (pRtkEpoch->nCommonSat>=10))) 
	{
		pRtkEpochKeep->bFirstCalAmb = FALSE;
		return RTK_RESCHK_OK_RATIO;
	}

	//phres
	for(idx=0; idx<pRtkEpoch->nCommonSatAMB; idx++)
	{
		svid = pRtkEpoch->CommonSat_Amb[idx].SatID;
		if(GetBitWord256(&UsedSVMapNW, svid-1)==0)
			continue;

		if(ddres[svid-1]<0.05)
			goodsvcnt++;
		sumRes += ddres[svid-1];
		wholesv++;	
	}

	meanRes = sumRes/wholesv;
	if(((goodsvcnt>3) && ((goodsvcnt*2)<wholesv))||(meanRes>0.2))
	{
		return RTK_RESCHK_FAILE_DDRES;
	}
		
	if((meanRes<0.025) && ((goodsvcnt*3)>=(wholesv*2)))
	{
		return RTK_RESCHK_OK_DDRES;
	}

	//N keep check
	for(idx=0; idx<pRtkEpoch->nCommonSatAMB; idx++)
	{
		svid = pRtkEpoch->CommonSat_Amb[idx].SatID;
		sysidx = GetNavSysIdx(svid);

		if((pRtkEpochKeep->nKeySatID[sysidx]<=0) || (pRtkEpoch->KeySatId[sysidx]<=0) 
			|| (pRtkEpochKeep->nKeySatID[sysidx]!=pRtkEpoch->KeySatId[sysidx]))
		{
			bKeyChange = TRUE;
		}
		else
			bKeyChange = FALSE;

		if(GetBitWord256(&UsedSVMapNW, svid-1)==0)
			continue;

		if(!bKeyChange)
		{
			if(GetBitWord256(&(pRtkEpochKeep->bAMB_NW_Fix_Valid),svid-1)==1)
			{
				if((int32)(pRtkEpoch->dAMB_NW_Fix[idx]) != pRtkEpochKeep->dAMB_NW_Fix[svid-1])
					bNChkFail_last = TRUE;
				else
					NkeepValidSVCnt++;
			}			
		}	
	}

	if((bNChkFail_last) || (NkeepValidSVCnt<8))
		NKeepConValidcnt=0;
	else
	{
		NKeepConValidcnt++;
		if(NKeepConValidcnt>30)
			NKeepConValidcnt=30;
	}

	if(NKeepConValidcnt>=5)
	{
		pRtkEpochKeep->bFirstCalAmb = FALSE;
		return RTK_RESCHK_OK_N;
	}

	return RTK_RESCHK_FAILE_NORMAL;
}

int32 CheckRTKResultNWF(RTK_EPOCH* pRtkEpoch, double dRatio, double* ddres, word256 UsedSVMapNWF)
{
	int32 idx=0, svid=0;
	
	//ratio
	if(dRatio>1.1)
		return RTK_RESCHK_OK_RATIO;

	//res
	for(idx=0;idx<pRtkEpoch->nCommonSatAMB;idx++)
	{
		svid = pRtkEpoch->CommonSat_Amb[idx].SatID;
		if(GetBitWord256(&(pRtkEpoch->L1UsedSVMap),svid-1)==0)
			continue;
		if(f_abs(ddres[svid-1])>0.5)
			return RTK_RESCHK_FAILE_DDRES;
	}

	//pos
	if(RTKCheckPosValid(&(pRtkEpoch->RovePVT_FLOAT), pRtkEpoch))
		return RTK_RESCHK_OK_STD;

	return RTK_RESCHK_FAILE_NORMAL;
}

// 功能：基线校验（ChkMode确定校验模式）
int CheckRTKResultL1(RTK_EPOCH* pRtkEpoch, RTK_EPOCH_Keep* pRtkEpochKeep, double* ddres, word256 UsedSVMapL1)
{	
	static double lastRatio=0.0;
	int32 idx=0,sysidx=0,validsvcnt=0, svcnt=0, svid=0, goodsvcnt=0, smoothcnt=0;
	double meanDDResNew = 0.0, std=0.0, stdth = 0.0;
	bool chkNres, bKeyChange=FALSE, bNChkFail=FALSE;
	double dRatio = pRtkEpoch->dRatio;

	//update RTK std
	smoothcnt = CalcRTKAverAndStd(pRtkEpochKeep, &(pRtkEpoch->RovePVT_Fix), 10, &(pRtkEpoch->Pos_aver), &(pRtkEpoch->Pos_std));

	//check ratio 
	if((dRatio>4.5)
		|| ((dRatio>2.5) && (pRtkEpoch->nCommonSat>=10)) 
		|| ((dRatio>2.0) && (pRtkEpochKeep->nAmbIndex>0))) 
	{
		return RTK_RESCHK_OK_RATIO;
	}

	//整周模糊度检查
	chkNres = TRUE;
	for(idx=0; idx<pRtkEpoch->nCommonSatAMB; idx++)
	{
		svid = pRtkEpoch->CommonSat_Amb[idx].SatID;
		sysidx = GetNavSysIdx(svid);

		if((pRtkEpochKeep->nKeySatID[sysidx]<=0) || (pRtkEpoch->KeySatId[sysidx]<=0) 
			|| (pRtkEpochKeep->nKeySatID[sysidx]!=pRtkEpoch->KeySatId[sysidx]))
		{
			bKeyChange = TRUE;
		}
		else
			bKeyChange = FALSE;

		if(GetBitWord256(&UsedSVMapL1, svid-1)==0)
			continue;

		if(!bKeyChange)
		{
			if((GetBitWord256(&(pRtkEpochKeep->bAMB_NS_Fix_Valid),svid-1)==1)
				&& (((int32)(pRtkEpoch->dAMB_Fix[idx]))!= pRtkEpochKeep->dAMB_NS_Fix[svid-1]))
			{
				bNChkFail=TRUE;
			}
		}

		if(bKeyChange || (GetBitWord256(&(pRtkEpochKeep->bAMB_NS_Fix_Valid),svid-1)==0))
			chkNres = FALSE;
	}

	if(pRtkEpochKeep->nAmbIndex>0)
	{
		if(bNChkFail)
			return RTK_RESCHK_FAILE_N;

		if(chkNres)
			return RTK_RESCHK_OK_N;
	}

	//残差检测
	if(ddres==NULL)
	{
		return RTK_RESCHK_FAILE_NODDRES;
	}
	
	for(idx=0; idx<pRtkEpoch->nCommonSat; idx++)
	{	
		svid = pRtkEpoch->CommonSat[idx].SatID;
		sysidx = GetNavSysIdx(svid);
		if(idx==pRtkEpoch->KeySatIdIndex[sysidx])
			continue;

		
		if(GetBitWord256(&UsedSVMapL1, svid-1)==0)
			continue;

		svcnt++;
		
		if(ddres[svid-1]>0.3)
			return RTK_RESCHK_FAILE_DDRES;

		if(ddres[svid-1]>0.25)
		{
			continue;
		}

		if(ddres[svid-1]<0.01)
			goodsvcnt++;
		
		meanDDResNew += ddres[svid-1];
		validsvcnt++;
	}

	if((validsvcnt>3) && (svcnt<(validsvcnt*2)))
	{
		meanDDResNew = meanDDResNew/validsvcnt;

		if((meanDDResNew < 0.025) && (goodsvcnt >= MIN((validsvcnt+1)/2,5)))
		{
			return RTK_RESCHK_OK_DDRES;
		}

		if(meanDDResNew>0.2)
		{
			return RTK_RESCHK_FAILE_DDRES;
		}
	}

	//check std
	std = pRtkEpoch->Pos_std.x*pRtkEpoch->Pos_std.x + pRtkEpoch->Pos_std.y*pRtkEpoch->Pos_std.y + pRtkEpoch->Pos_std.z*pRtkEpoch->Pos_std.z;
	stdth = GetRTKStdChkTh();
	if((smoothcnt>4) && (pRtkEpochKeep->nAmbIndex>0) &&(std<(stdth*stdth)))	// 0.01m
		return RTK_RESCHK_OK_STD;

	//check ratio change
	if((lastRatio>1.1) &&((dRatio-lastRatio)>2.0))
	{
		return RTK_RESCHK_OK_RATIO_CHANGE;
	}

	return RTK_RESCHK_FAILE_NORMAL;
}


// 求向量的2范数
double GetNorm2(double* Vec, int nDim, CHAR bSqrt)
{
	int i = 0;
	double dNorm = 0;
	for (i=0; i<nDim; i++)
	{
		dNorm += Vec[i]*Vec[i];
	}
	if (bSqrt==2)
	{
		return sqrt(dNorm/(nDim-3));
	}
	else
		return (bSqrt) ? sqrt(dNorm) : dNorm;
}


// 根据工作模式设置差分模式，保存定位时各种结果
bool SetRtkPosMode(OBSEV* PrmMaster, OBSEV* PrmSlave, RTK_EPOCH* pRtkEpoch, RTK_EPOCH_Keep* pRtkEpochKeep)
{
	int32 idx;
	double curobsTime=0.0, diftime=20.0;
#ifdef _POSTPROC
	double az,el=0.0;
#endif

	if((pRtkEpochKeep->nAmbIndex>0) || (pRtkEpochKeep->nAmbIndex_wide>0) || (pRtkEpochKeep->nAmbIndex_wfloat>0))
	{
		if(GetObsvTime(PrmMaster, &curobsTime, NAV_SYS_NONE))
		{
			diftime = f_abs(curobsTime-pRtkEpochKeep->tow);
			if(diftime > SECONDS_HALF_WEEK)
				diftime -= SECONDS_IN_WEEK;
		}
	}
	
	if(diftime>3.0)
	{
		memset(pRtkEpochKeep, 0, sizeof(RTK_EPOCH_Keep));
		pRtkEpochKeep->bFirstCalAmb = TRUE;
	#if ENABLE_RTD_KF
		ResetRTDKF(pRtkEpochKeep->RTDKFPara);
	#endif	
	}
	else
	{
		for(idx=0; idx<SV_NUM_TRUE; idx++) //周跳秒减1
		{
			if(pRtkEpochKeep->slip[idx] > 0)
				pRtkEpochKeep->slip[idx]--;
		}
	}
		
	memset(pRtkEpoch, 0, sizeof(RTK_EPOCH));

	
#ifdef _POSTPROC
		// 输入PrmSlave,读取全局变量FixEquItrCntThres,输出PrmSlave->SingleRcvrPos
		if(LSCalcRcvrPos_obs(PrmMaster,&(PrmMaster->SingleRcvrPos)) == FIX_NOT)
		return FALSE;

	for(idx=0;idx<PrmMaster->satnmax;idx++)
	{
		CalcSVAngle(PrmMaster->obs[idx].svpos, PrmMaster->SingleRcvrPos, &az, &el);
		PrmMaster->obs[idx].az = (int32)az+0.5;
		PrmMaster->obs[idx].el = (int32)el+0.5;
	}

	if(LSCalcRcvrPos_obs(PrmSlave,&(PrmSlave->SingleRcvrPos)) == FIX_NOT)
		return FALSE;

	for(idx=0;idx<PrmSlave->satnmax;idx++)
	{
		CalcSVAngle(PrmSlave->obs[idx].svpos, PrmSlave->SingleRcvrPos, &az, &el);
		PrmSlave->obs[idx].az = (int32)az+0.5;
		PrmSlave->obs[idx].el = (int32)el+0.5;
	}

	// 存储观测数据(RTK模式下移动站解算，ATT模式下基准站解算)
	memcpy(&pRtkEpoch->Prm_Base, PrmSlave, sizeof(OBSEV));//从板，基准站，
	memcpy(&pRtkEpoch->Prm_Rove, PrmMaster, sizeof(OBSEV));//主板，移动站，	

#else
	// 存储观测数据(RTK模式下移动站解算，ATT模式下基准站解算)
	memcpy(&(pRtkEpoch->Prm_Base), PrmSlave, sizeof(OBSEV));//从板，基准站，
	memcpy(&(pRtkEpoch->Prm_Rove), PrmMaster, sizeof(OBSEV));//主板，移动站，
	pRtkEpoch->Prm_Rove.bSinglePosValid = (int8)getRcvrInfo(&(pRtkEpoch->Prm_Rove.SingleRcvrPos),NULL,NULL,NULL);
	if(pRtkEpoch->Prm_Rove.bSinglePosValid==FIX_NOT)
		return FALSE;
#endif
	
	memcpy(&(pRtkEpoch->RovePVT_RTD), &(pRtkEpoch->Prm_Rove.SingleRcvrPos), sizeof(pRtkEpoch->RovePVT_RTD));

	//calc coarse baseline lenght
	if(Basefixposition.bValid)
	{
		pRtkEpoch->Baseline.dxyz.x = pRtkEpoch->Prm_Rove.SingleRcvrPos.x - Basefixposition.ecefpos.x;
		pRtkEpoch->Baseline.dxyz.y = pRtkEpoch->Prm_Rove.SingleRcvrPos.y - Basefixposition.ecefpos.y;
		pRtkEpoch->Baseline.dxyz.z = pRtkEpoch->Prm_Rove.SingleRcvrPos.z - Basefixposition.ecefpos.z;

		pRtkEpoch->Baseline.BaselineLongth = sqrt(pRtkEpoch->Baseline.dxyz.x*pRtkEpoch->Baseline.dxyz.x+
			pRtkEpoch->Baseline.dxyz.y*pRtkEpoch->Baseline.dxyz.y+pRtkEpoch->Baseline.dxyz.z*pRtkEpoch->Baseline.dxyz.z);

		return TRUE;
	}
	
	return FALSE;
}


//#pragma CODE_SECTION(GetCommSatInfo,"sect_ECODE_IV");
void GetCommSatInfo(RTK_EPOCH* pRtkEpoch, RTK_EPOCH_Keep* pRtkEpochKeep)
{
	int 	i=0,j=0, m;
	//int n;
	OBSEV prmTemp;
	int 	nLen = 0;
	int32  mincn0=100, mincn0idx=-1, cn0=0;
	int32  minLLI=100;
	//int32 minLLIidx=-1;
	int32  delidx=0,maxRtkSvNum = 0;

	memset(&prmTemp,0,sizeof(OBSEV));

	// 读取全局变量pActiveCPT
	maxRtkSvNum = GetMaxRtkSvNum(pRtkEpochKeep->bLastIntValid);

	//delete the more sv by cn0 and LLI
	while(pRtkEpoch->nCommonSat >= maxRtkSvNum)
	{
		for(m=0; m<(pRtkEpoch->nCommonSat); m++)
		{	
			cn0 = pRtkEpoch->CommonSat[m].cn0;
			if(SV_IsBd2Geo(pRtkEpoch->CommonSat[m].SatID))
				cn0 -= 8;
			else if(SV_IsGps(pRtkEpoch->CommonSat[m].SatID))
				cn0 += 3;
			if(cn0 < mincn0)
			{
				mincn0 = pRtkEpoch->CommonSat[m].cn0;
				mincn0idx = m;
			}

			if(pRtkEpoch->CommonSat[m].LLI < minLLI)
			{
				minLLI = pRtkEpoch->CommonSat[m].LLI;
				//minLLIidx = m;
			}
		}

		//if(minLLI < 200)
		//	delidx=minLLIidx;
		//else
			delidx=mincn0idx;

		if(delidx<(pRtkEpoch->nCommonSat-1))
			memcpy(&(pRtkEpoch->CommonSat[delidx]), &(pRtkEpoch->CommonSat[delidx+1]), sizeof(BD2_SAT_INFO)*(pRtkEpoch->nCommonSat-1-delidx));

		pRtkEpoch->nCommonSat--;
	}
		

	//根据公共卫星信息重排观测量
	memcpy(&prmTemp, &(pRtkEpoch->Prm_Base), sizeof(OBSEV));
	memset(&(pRtkEpoch->Prm_Base.obs[0]), 0, sizeof(OBST)*MAX_RTK_OBS_SVCNT);
	pRtkEpoch->Prm_Base.satnmax = pRtkEpoch->nCommonSat;
	nLen = 0;
	for(i=0; i<pRtkEpoch->nCommonSat; i++)
	{
		for(j=0; j<prmTemp.satnmax; j++)
		{
			if(prmTemp.obs[j].StarID == pRtkEpoch->CommonSat[i].SatID)
			{
				memcpy(&(pRtkEpoch->Prm_Base.obs[nLen++]), &prmTemp.obs[j], sizeof(OBST));
				break;
			}
		}
	}

	//根据公共卫星信息重排观测量
	memcpy(&prmTemp, &(pRtkEpoch->Prm_Rove), sizeof(OBSEV));
	memset(&(pRtkEpoch->Prm_Rove.obs[0]), 0, sizeof(OBST)*MAX_RTK_OBS_SVCNT);
	pRtkEpoch->Prm_Rove.satnmax = pRtkEpoch->nCommonSat;
	nLen = 0;
	for(i=0; i<pRtkEpoch->nCommonSat; i++)
	{
		for(j=0; j<prmTemp.satnmax; j++)
		{
			if(prmTemp.obs[j].StarID == pRtkEpoch->CommonSat[i].SatID)
			{
				memcpy(&(pRtkEpoch->Prm_Rove.obs[nLen++]), &prmTemp.obs[j], sizeof(OBST));
				break;
			}
		}
	}

	return;
}

bool CheckSuperWideLane(double baselineLen, bool bLastSuperWide)
{
	bool ret = FALSE;

#if SUPPORT_SUPER_WIDE_LANE
	if(!bLastSuperWide)
	{
		if(baselineLen > 12e3)	//10km
			ret = TRUE;
	}
	else
	{
		if(baselineLen < 10e3)	//9km
			ret = FALSE;
	}
#endif
	return ret;
}


//#pragma CODE_SECTION(ProcessRTKSatInfo,"sect_ECODE_IV");
// 处理公共卫星及双差信息
void ProcessRTKSatInfo(RTK_EPOCH* pRtkEpoch, RTK_EPOCH_Keep* pRtkEpochKeep)
{
	pRtkEpoch->bSuperWide = CheckSuperWideLane(pRtkEpoch->Baseline.BaselineLongth, pRtkEpochKeep->bSuperWide);
	
	// 判断得到公共卫星
	// 读取全局变量 pActiveCPT
	GetCommonSat(pRtkEpoch, pRtkEpochKeep);
	if (pRtkEpoch->nCommonSat<RTK_MIN_OBSCNT)	
		return;

	// 获得公共卫星的观测量及卫星状态信息
	// 读取全局变量 pActiveCPT
	GetCommSatInfo(pRtkEpoch, pRtkEpochKeep);

	// 周跳检测
	// 无全局变量操作
	if(PhaseCycleSlipCheck(pRtkEpoch, pRtkEpochKeep))
	{
		DeleteSlipSV(pRtkEpoch, pRtkEpochKeep);	
		if (pRtkEpoch->nCommonSat<RTK_MIN_OBSCNT)	
			return;
	}

	// 设置基准星
	// 无全局变量操作
	GetKeySatNo(pRtkEpoch, pRtkEpochKeep);
	if(pRtkEpoch->nCommonSat<RTK_MIN_OBSCNT)
		return;

	//check is all common sv is GEO?
	// 无全局变量操作
	CheckAllCommSVGEO(pRtkEpoch);
	
	//读取全局变量 Sys3AlmEphUTCInfo Basefixposition
	UpdateSvPosForDifEph(pRtkEpoch);

	//update distance SV to base station
	// 读取全局变量 Basefixposition
	UpdateDistSV2Base(pRtkEpoch);
	
	// RTK模式下的误差处理
	// 读取全局变量 DGPS_MODE
	if(DGPS_MODE == DGPS_MODE_STATC)
	{
		//CalRTK_Error();
		;
	}

	// 双差观测量处理
	// 无全局变量操作
	GetDD_OBV(pRtkEpoch);

	return;
}


//#pragma CODE_SECTION(RTK_GetFix,"sect_ECODE_IV");
void RTK_GetFix(RTK_EPOCH* pRtkEpoch)
{
	char i=0;
	char j=0;
	char k=0;
	static char get_fix_flag = 0;
	static word256 get_fix_sat = {{0,0,0,0,0,0,0,0}};	
	static char get_fix_keysat[MAX_SYSM] = {0,};
	static double get_fix[SV_NUM] = {0};
	static double get_fix_lastsec = -2.0;
	static double get_fix_time = 0.0;
	static char get_fix_sust = 0;
	static char get_fix_un_flag =0;
	int32 sysidx;
	double curEpochTime=0.0;

	RTK_GetCurEpochTime(pRtkEpoch, &curEpochTime);

	//if ((RtkEpoch.time.dSeconds+17) > 458373.0)
	//	i=i;

	// ratio值3以上连续稳定60s的存储
	if (pRtkEpoch->dRatio > MBGF_SUSRATIO)
	{
		if (fabs(curEpochTime - get_fix_lastsec) < 1.1)
		{
			get_fix_sust++;
			if (get_fix_sust>MBGF_SUSTIME)
				get_fix_sust = MBGF_SUSTIME;
		}
		if ((get_fix_sust == MBGF_SUSTIME) || (pRtkEpoch->dRatio > MBGF_GETRATIO))
		{
			// 首次进入
			if (!get_fix_flag) 
				get_fix_flag = 1;

			memcpy(get_fix_keysat , pRtkEpoch->KeySatId, sizeof(get_fix_keysat));
			memset(&get_fix_sat, 0, sizeof(get_fix_sat));
			memset(get_fix,0,MBGF_GETNUM_LIMIT*sizeof(double));
			// 存储卫星号以及整周
			j=0;
			for(i=0; i<pRtkEpoch->nCommonSat; i++)
			{
				SetBitWord256(&get_fix_sat, pRtkEpoch->CommonSat[i].SatID-1);
				for(sysidx = 0; sysidx<MAX_SYSM; sysidx++)
				{
					if (i == pRtkEpoch->KeySatIdIndex[sysidx])
						break;
				}

				if(sysidx != MAX_SYSM)
					continue;
				
				get_fix[pRtkEpoch->CommonSat[i].SatID-1] = pRtkEpoch->dAMB_Fix[j];
				j++;
			}

			get_fix_time = curEpochTime;
		}

		get_fix_lastsec = curEpochTime;
	}
	else
		get_fix_sust = 0;

	//if ((RtkEpoch.time.dSeconds+17) > 457784.0)
	//	i=i;

	// ratio值2以下并满足条件时使用
	if (pRtkEpoch->dRatio < MBGF_USERATIO && get_fix_flag)
	{
		// 默认使用
		get_fix_un_flag = MBGF_NU_ALLRIGHT;

		// 关键星不同不使用
		for(sysidx=0; sysidx < MAX_SYSM; sysidx++)
		{
			if(get_fix_keysat[sysidx] != pRtkEpoch->KeySatId[sysidx])

			{
				get_fix_un_flag = MBGF_NU_KEYCHANGE;
				break;
			}
		}

		// 卫星过少不使用
		if (pRtkEpoch->nCommonSat <= MBGF_USESATE_LIMIT)
			get_fix_un_flag = MBGF_NU_SATELESS;

		// 卫星改变不使用
		for (k=0;k<pRtkEpoch->nCommonSat;k++)
		{
			if(GetBitWord256(&get_fix_sat, pRtkEpoch->CommonSat[k].SatID-1)==0)
			{
				get_fix_un_flag = MBGF_NU_SATECHANGE;
				break;
			}
		}

		// 时间过长不用
		if (fabs(curEpochTime - get_fix_time) >= MBGF_USETIME_LIMIT)
			get_fix_un_flag = MBGF_NU_TIMEOUT;

		// 整周赋值
		if (!get_fix_un_flag)
		{
			j=0;
			for(i=0; i<pRtkEpoch->nCommonSat; i++)
			{
				for(sysidx = 0; sysidx<MAX_SYSM; sysidx++)
				{
					if (i == pRtkEpoch->KeySatIdIndex[sysidx])
						break;
				}
				
				if (sysidx!=MAX_SYSM)
					continue;
				
				pRtkEpoch->dAMB_Fix[j] = get_fix[pRtkEpoch->CommonSat[i].SatID-1];
				j++;
			}		
			pRtkEpoch->dRatio = MBGF_USERATIO;
		}
	}
}

/*--------------------------------------------------------------------------------------
// 功能:根据双站信息计算基线长度和姿态
// 参数: 
// [IN] pConfig		配置信息
// [OUT]pOutput		计算结果输出
// 返回: 错误标志，负值表示发生错误
--------------------------------------------------------------------------------------*/
int8 Rel_Position(OBSEV* PrmMaster, OBSEV* PrmSlave, RTK_EPOCH* pRtkEpoch, RTK_EPOCH_Keep* pRtkEpochKeep)
{
	double RTKDist_DD[LAMBDA_N_DIM];
	bool bRTDResValid = FALSE;
	int8 fixres=FIX_NOT;
	int32 svENValidCnt=0;
	bool bAvlibNW2NS=FALSE, bSecCalc=FALSE;
	double ddResL1[SV_NUM_TRUE], ddResNW[SV_NUM_TRUE], ddResNWf[SV_NUM_TRUE];
	bool bNeed2KeepInfo = TRUE;

	memset(RTKDist_DD,0,sizeof(RTKDist_DD));
	memset(ddResL1, 0, sizeof(ddResL1));
	memset(ddResNW, 0, sizeof(ddResNW));
	memset(ddResNWf, 0, sizeof(ddResNWf));

	// 设置RTK模式，同时保存单点解算的部分中间结果
	// 读取全局变量 FixEquItrCntThres、Basefixposition
	if(SetRtkPosMode(PrmMaster, PrmSlave, pRtkEpoch, pRtkEpochKeep) == FALSE)
	{
		pRtkEpochKeep->bLastIntValid = FALSE;
		bNeed2KeepInfo = FALSE;
		goto ___RTK_EXIT;
	}

	// 处理公共卫星及双差信息
	// 读取全局变量 Sys3AlmEphUTCInfo、Basefixposition、pActiveCPT、DGPS_MODE
	ProcessRTKSatInfo(pRtkEpoch, pRtkEpochKeep);

	//判断基准站坐标是否收到，如果没有则不计算
	if((pRtkEpoch->nCommonSat < RTK_MIN_OBSCNT) || (Basefixposition.bValid==FALSE))
	{
		bNeed2KeepInfo = FALSE;
		pRtkEpochKeep->bFirstCalAmb = TRUE;
		pRtkEpochKeep->bLastIntValid = FALSE;
		pRtkEpochKeep->RTKErrPath=5;
		goto ___RTK_EXIT;
	}

	if(pRtkEpochKeep->bLastIntValid)	// Is Last L1_INT?
	{
		//L1_INT
		svENValidCnt = RTKSVSelectEN(pRtkEpoch,pRtkEpochKeep,&(pRtkEpoch->L1UsedSVMap), pRtkEpoch->dAMB_Fix, FIX_L1_INT);
		if(svENValidCnt >= RTK_MIN_OBSCNT)
		{
			if(CalBaseline_ECEF(pRtkEpoch,pRtkEpoch->Phase_DD_NS,pRtkEpoch->dAMB_Fix,pRtkEpoch->wavelen_NS,&(pRtkEpoch->RovePVT_Fix),&(pRtkEpoch->L1UsedSVMap), ddResL1))
			{
				pRtkEpoch->nAmbIndex = CheckExtRTKRes(pRtkEpoch,&(pRtkEpoch->L1UsedSVMap),ddResL1,0.2,NULL,NULL);	//need to modify
				if (pRtkEpoch->nAmbIndex>0)
				{
					// 读取全局变量 Basefixposition
					CalcBaselineByPos(&(pRtkEpoch->RovePVT_Fix), &(pRtkEpoch->Baseline.dxyz));
					pRtkEpochKeep->RTKErrPath = 10;
				}
				else
					ResetWord256(&(pRtkEpochKeep->bAMB_NS_Fix_Valid));
			}
		}

		//NW_INT
		if(pRtkEpoch->nAmbIndex<=0)
		{
			svENValidCnt = RTKSVSelectEN(pRtkEpoch,pRtkEpochKeep,&(pRtkEpoch->L1UsedSVMap), pRtkEpoch->dAMB_NW_Fix, FIX_WIDE_INT);
			if(pRtkEpoch->nAmbIndex<0) //若为单频不过，切到宽巷，用单频计算残差，进行二次选星
				svENValidCnt = SelectNWSVByL1Residue(pRtkEpoch,&(pRtkEpoch->L1UsedSVMap),ddResL1,svENValidCnt);
				
			if(svENValidCnt >= RTK_MIN_OBSCNT)
			{
				if(CalBaseline_ECEF(pRtkEpoch,pRtkEpoch->Phase_DD_NW,pRtkEpoch->dAMB_NW_Fix,pRtkEpoch->wavelen_NW,&(pRtkEpoch->RovePVT_Fix),&(pRtkEpoch->L1UsedSVMap), ddResNW))
				{
					pRtkEpoch->nAmbIndex_wide =  CheckExtRTKRes(pRtkEpoch,&(pRtkEpoch->L1UsedSVMap),ddResNW,0.2,&bAvlibNW2NS,&bSecCalc);					
					//二次选星，二次解算宽巷基线
					if (pRtkEpoch->nAmbIndex_wide<0 && bSecCalc)
					{
						if(CalBaseline_ECEF(pRtkEpoch,pRtkEpoch->Phase_DD_NW,pRtkEpoch->dAMB_NW_Fix,pRtkEpoch->wavelen_NW,&(pRtkEpoch->RovePVT_Fix),&(pRtkEpoch->L1UsedSVMap), ddResNW))
							pRtkEpoch->nAmbIndex_wide =  CheckExtRTKRes(pRtkEpoch,&(pRtkEpoch->L1UsedSVMap),ddResNW,0.2,&bAvlibNW2NS,NULL);
					}
					
					if (pRtkEpoch->nAmbIndex_wide>0)
					{
						// 读取全局变量 Basefixposition
						CalcBaselineByPos(&(pRtkEpoch->RovePVT_Fix), &(pRtkEpoch->Baseline.dxyz));
				
						if (bAvlibNW2NS) //宽巷切单频
						{
							RtkNW2RtkNS(pRtkEpoch,svENValidCnt,&(pRtkEpoch->L1UsedSVMap),pRtkEpoch->dAMB_NW_Fix,pRtkEpoch->dAMB_Fix,pRtkEpoch->wavelen_NW,pRtkEpoch->wavelen_NS);
							pRtkEpoch->nAmbIndex = RTK_RESCHK_OK_NW;
						}

						pRtkEpochKeep->RTKErrPath = 11;
					}
					else
						ResetWord256(&(pRtkEpochKeep->bAMB_NW_Fix_Valid));
				}
			}

			//NW_FLOAT
			if(pRtkEpoch->nAmbIndex_wide<=0)
			{
				svENValidCnt = RTKSVSelectEN(pRtkEpoch,pRtkEpochKeep,&(pRtkEpoch->L1UsedSVMap), pRtkEpochKeep->Rtk_NKeep, FIX_WIDE_FLOAT);
				if(svENValidCnt >= RTK_MIN_OBSCNT)
				{
					if(CalBaseline_ECEF(pRtkEpoch,pRtkEpoch->Phase_DD_NW,pRtkEpochKeep->Rtk_NKeep,pRtkEpoch->wavelen_NW,&(pRtkEpoch->RovePVT_FLOAT),&(pRtkEpoch->L1UsedSVMap), ddResNWf))
					{
						pRtkEpochKeep->RTKErrPath = 13;
						// 读取全局变量 Basefixposition
						CalcBaselineByPos(&(pRtkEpoch->RovePVT_FLOAT), &(pRtkEpoch->Baseline.dxyz));
						pRtkEpoch->nAmbIndex_wfloat = CheckRTKResultNWF(pRtkEpoch,0.0,ddResNWf,pRtkEpoch->L1UsedSVMap);
					}
				}

				if((pRtkEpoch->nAmbIndex_wfloat<=0) || IsAmbResolution(pRtkEpoch))
				{
 					pRtkEpochKeep->bLastIntValid = FALSE;
					pRtkEpochKeep->bFirstCalAmb = TRUE;
				}
			}
		}
	}	
	else	//first  need to lambda search
	{
		// 初始化差分定向需要的全局变量参数
		InitAttDetPara(pRtkEpoch, pRtkEpochKeep);
		
		// 双差浮点解
	#if ENABLE_RTD_KF	
		bRTDResValid = CalRTDProc(pRtkEpoch, RTKDist_DD, &(pRtkEpoch->Baseline.dxyz), &(pRtkEpochKeep->RTDKFPara));
	#else
		bRTDResValid = CalRTDProc(pRtkEpoch, RTKDist_DD, &(pRtkEpoch->Baseline.dxyz), NULL);
	#endif

		/*******************RTK process*****************************/
		if(bRTDResValid && IsRTKOpen())
		{
			// Init NW
			InitalFloatN(pRtkEpoch, RTKDist_DD, pRtkEpoch->dAMB_PrDD);
		
			// 卡尔曼滤波处理浮点解
			FloatSolution_Kalman(pRtkEpoch, pRtkEpochKeep,pRtkEpoch->dAMB_PrDD, pRtkEpochKeep->Rtk_NKeep, pRtkEpochKeep->Rtk_PKeep);
		
			// 整周模糊度搜索
			if(Ambiguity_Resolution(pRtkEpoch, pRtkEpochKeep, pRtkEpochKeep->Rtk_NKeep, pRtkEpochKeep->Rtk_PKeep, &pRtkEpoch->dRatio, pRtkEpoch->dAMB_Fix, pRtkEpoch->dAMB_Float))
			{
				//***MovingBase策略(动态定向)***	
				if(IsMoveBaseForRove())
					RTK_GetFix(pRtkEpoch); 

				// baseline计算及校验
				pRtkEpoch->nAmbIndex = AttCal_UseAMB(pRtkEpoch, pRtkEpochKeep, pRtkEpoch->dAMB_Fix, &(pRtkEpoch->RovePVT_Fix), &(pRtkEpoch->Baseline.dxyz), OBS_SINGLE, AMB_INT);
				if (pRtkEpoch->nAmbIndex<=0)
				{
					if (pRtkEpoch->nAmbIndex == RTK_RESCHK_FAILE_DDRES)
						pRtkEpoch->nAmbIndex_wide = RTK_RESCHK_FAILE_DDRES;
					else
						pRtkEpoch->nAmbIndex_wide = AttCal_UseAMB(pRtkEpoch, pRtkEpochKeep, pRtkEpoch->dAMB_NW_Fix, &(pRtkEpoch->RovePVT_Fix),&(pRtkEpoch->Baseline.dxyz), OBS_WIDE, AMB_INT);
				
					pRtkEpochKeep->bFirstCalAmb = TRUE;
				}
				else
					pRtkEpochKeep->bFirstCalAmb = FALSE;

 				if((pRtkEpoch->nAmbIndex>0) && (pRtkEpochKeep->nAmbIndex>0)) //连续两拍解算成功，则进入保持
 					pRtkEpochKeep->bLastIntValid = TRUE;

			}
			else
			{
				bNeed2KeepInfo = FALSE;
				pRtkEpochKeep->bFirstCalAmb = TRUE;
				pRtkEpochKeep->bLastIntValid = FALSE;
				pRtkEpochKeep->RTKErrPath=6;
			}
		}
		else
		{
			bNeed2KeepInfo = FALSE;
			pRtkEpochKeep->bFirstCalAmb = TRUE;
			pRtkEpochKeep->bLastIntValid = FALSE;
		}

		//calc float
		if((pRtkEpoch->nAmbIndex<=0) && (pRtkEpoch->nAmbIndex_wide<=0))
			pRtkEpoch->nAmbIndex_wfloat = AttCal_UseAMB(pRtkEpoch,pRtkEpochKeep,pRtkEpochKeep->Rtk_NKeep, &(pRtkEpoch->RovePVT_FLOAT), &(pRtkEpoch->Baseline.dxyz), OBS_WIDE, AMB_FLOAT);

	}
	
	//calc baseline and ATT
	if(bRTDResValid || (pRtkEpoch->nAmbIndex_wfloat>0) || (pRtkEpoch->nAmbIndex_wide>0) || (pRtkEpoch->nAmbIndex>0))
	{
		ECEF2ENU_DeltaXYZ(&(pRtkEpoch->Baseline), &(Basefixposition.ecefpos));
		Attitude_Determination_UseBSL(&(pRtkEpoch->Baseline), &(pRtkEpoch->Attitude));
	}
	
	//------------------- 输出相关计算结果-----------------------------
	if(pRtkEpoch->nAmbIndex>0)
	{
		fixres = FIX_L1_INT;
		setRcvrRTKInfo(&(pRtkEpoch->RovePVT_Fix),pRtkEpoch->nCommonSat,pRtkEpoch->CommonSat,&(pRtkEpoch->SelectFrepMap),&(pRtkEpoch->Baseline),&(pRtkEpoch->Attitude),FIX_L1_INT);
	}
	else if(pRtkEpoch->nAmbIndex_wide>0)
	{
		fixres = FIX_WIDE_INT;
		setRcvrRTKInfo(&(pRtkEpoch->RovePVT_Fix),pRtkEpoch->nCommonSat,pRtkEpoch->CommonSat,&(pRtkEpoch->SelectFrepMap),&(pRtkEpoch->Baseline),&(pRtkEpoch->Attitude),FIX_WIDE_INT);
	}
	/*else if(pRtkEpoch->nAmbIndex_wfloat>0)
	{
		fixres = FIX_WIDE_FLOAT;
		setRcvrRTKInfo(&(pRtkEpoch->RovePVT_FLOAT),pRtkEpoch->nCommonSat,pRtkEpoch->CommonSat,&(pRtkEpoch->SelectFrepMap),&(pRtkEpoch->Baseline),&(pRtkEpoch->Attitude),FIX_WIDE_FLOAT);
	}*/
	else if(bRTDResValid)
	{	
		fixres = FIX_RTD;
		setRcvrRTKInfo(&(pRtkEpoch->RovePVT_RTD),pRtkEpoch->nCommonSat,pRtkEpoch->CommonSat,&(pRtkEpoch->SelectFrepMapL1),&(pRtkEpoch->Baseline),&(pRtkEpoch->Attitude),FIX_RTD);
	}


___RTK_EXIT:
	SaveLastCommonSatInfo(bNeed2KeepInfo, pRtkEpoch, pRtkEpochKeep);	//save keep info.
	
	return fixres;
}


//平滑算法
//#pragma CODE_SECTION(SmoothData,"sect_ECODE_IV");
BOOL SmoothData(double* dCurData, double* dSmthData, int nCur)
{
	*dSmthData = *dCurData/nCur + *dSmthData*(nCur-1)/nCur;
	return TRUE;
}

//平滑航向结果
//#pragma CODE_SECTION(SmoothAttData,"sect_ECODE_IV");
void SmoothAttData(BOOL bClear, RTK_EPOCH* pRtkEpoch)
{	
	BD2_Attitude curAtt;
	BD2_BASELINE curBsl;

	static BD2_Attitude SmoothAtt;
	static BD2_BASELINE SmoothBsl;
	static char BasAtt_flag = 0;
	static int nSmoothLen = 1;

	memset(&curAtt,0,sizeof(BD2_Attitude));
	memset(&curBsl,0,sizeof(BD2_BASELINE));

	if (0 == BasAtt_flag)
	{
		memset(&SmoothAtt,0,sizeof(BD2_Attitude));
		memset(&SmoothBsl,0,sizeof(BD2_BASELINE));
		BasAtt_flag = 1;
	}

	//平滑航向角度
	if(g_bSmoothAtt && (!bClear))
	{
		memcpy(&curAtt, &pRtkEpoch->Attitude, sizeof(BD2_Attitude));
		SmoothData(&curAtt.yaw_deg, &SmoothAtt.yaw_deg, nSmoothLen);
		SmoothData(&curAtt.pitch_deg, &SmoothAtt.pitch_deg, nSmoothLen);
		memcpy(&curBsl, &pRtkEpoch->Baseline, sizeof(BD2_BASELINE));
		SmoothData(&curBsl.East, &SmoothBsl.East, nSmoothLen);
		SmoothData(&curBsl.North, &SmoothBsl.North, nSmoothLen);
		SmoothData(&curBsl.Up, &SmoothBsl.Up, nSmoothLen);
		SmoothData(&curBsl.BaselineLongth, &SmoothBsl.BaselineLongth, nSmoothLen);
		SmoothData(&curBsl.dxyz.x, &SmoothBsl.dxyz.x, nSmoothLen);
		SmoothData(&curBsl.dxyz.y, &SmoothBsl.dxyz.y, nSmoothLen);
		SmoothData(&curBsl.dxyz.z, &SmoothBsl.dxyz.z, nSmoothLen);
		if(nSmoothLen>g_nSmoothindow*g_nOIC)	
			nSmoothLen = g_nSmoothindow*g_nOIC;
		else
			nSmoothLen++;
		pRtkEpoch->Attitude.yaw_deg = SmoothAtt.yaw_deg;
		pRtkEpoch->Attitude.pitch_deg = SmoothAtt.pitch_deg;
		pRtkEpoch->Baseline.East = SmoothBsl.East;
		pRtkEpoch->Baseline.North = SmoothBsl.North;
		pRtkEpoch->Baseline.Up = SmoothBsl.Up;
		pRtkEpoch->Baseline.BaselineLongth= SmoothBsl.BaselineLongth;
		pRtkEpoch->Baseline.dxyz.x= SmoothBsl.dxyz.x;
		pRtkEpoch->Baseline.dxyz.y= SmoothBsl.dxyz.y;
		pRtkEpoch->Baseline.dxyz.z= SmoothBsl.dxyz.z;
	}
	else
	{
		nSmoothLen = 1;
		memset(&SmoothAtt, 0, sizeof(BD2_Attitude));
	}

}

/*
BOOL RangeSDFilter(OBST prmSlaveObv, OBST prmMasterObv, double *dErrRange)
{
	int i=0,j=0;
	//int nIQ = RtkEpoch.PosMode_Rtk.PCode;
	double dRangeSD = 0;

	if(RtkEpoch.PosMode_Rtk.FreqUsed==1)
	{
		i = GetFreGropIdx(RtkEpoch.PosMode_Rtk.Freq[0]);
		dRangeSD = prmSlaveObv.range[i] - prmMasterObv.range[i];
	}
	else if(RtkEpoch.PosMode_Rtk.FreqUsed==2)
	{
		i = GetFreGropIdx(RtkEpoch.PosMode_Rtk.Freq[0]);
		j = GetFreGropIdx(RtkEpoch.PosMode_Rtk.Freq[1]);
		dRangeSD = (prmSlaveObv.range[i] - prmMasterObv.range[i])
			* (prmSlaveObv.range[j] - prmMasterObv.range[j]);
	}
	else
	{
		return FALSE;					
	}

	if(dRangeSD==0 || fabs(dRangeSD)>898755178.736818)//29979.2458*299792458
	{
		*dErrRange = dRangeSD;
		return FALSE;
	}
	else
		return TRUE;
}
*/


BOOL RTKCoaseSVSelect(OBST* prmObv, word32* pValidFrqpiont, double* pSnr, double* pLLI)
{
	int idx=0;
	float minsnrth=pActiveCPT->SysmCptWorkConfig.SnrMaskRTK;
	bool ret = FALSE;
	int cnt=0;
	
	*pValidFrqpiont = 0;
	(*pSnr) = 0.0;
	(*pLLI) = 0.0;

	for(idx=0; idx<MAX_FREPIONT_PER_NAVSYS; idx++)
	{
		if(((prmObv->validmap[idx] & OBST_VALID_SNR) ==0) || ((prmObv->validmap[idx] & OBST_VALID_CODE) ==0) || ((prmObv->validmap[idx] & OBST_VALID_LLI) ==0))
		{
			prmObv->validmap[idx]=0;
			continue;
		}

		//check SNR
		if(prmObv->code[idx]==DSP_L2P_FRE)
			minsnrth=pActiveCPT->SysmCptWorkConfig.SnrMaskRTK-L2P_LOWER_CN0_TH;
		else
			minsnrth=pActiveCPT->SysmCptWorkConfig.SnrMaskRTK;

		if((prmObv->snr0[idx]<minsnrth) || (prmObv->snr0[idx]>MAX_SNR))
		{
			prmObv->validmap[idx]=0;	
			continue;
		}

		//check lock time
		if(prmObv->LLI[idx]<(1*1000))	//10s
		{
			prmObv->validmap[idx]=0;	
			continue;	
		}
		
		(*pValidFrqpiont) |= prmObv->code[idx];
		(*pSnr) += prmObv->snr0[idx];
		(*pLLI) += prmObv->LLI[idx];
		cnt++;
		ret=TRUE;
	}

	if(cnt>1)
	{
		(*pSnr) = (*pSnr)/cnt;
		(*pLLI) = (*pLLI)/cnt;
	}

	return ret;
}


/*--------------------------------------------------------------------------------------
// 功能:根据站观擦棵到公共卫星列表
// 参数: 
// [IN] pConfig		配置信息
// [OUT]pOutput		计算结果输出
// 返回: 公共卫星个数
--------------------------------------------------------------------------------------*/
//#pragma CODE_SECTION(GetCommonSat,"sect_ECODE_IV");
INT32 GetCommonSat(RTK_EPOCH* pRtkEpoch, RTK_EPOCH_Keep* pRtkEpochKeep)
{
	//基准站卫星列表在单点解算后存储于epoch中
	//只需要遍历移动站观测数据
	int 	j=0, k=0;
	int 	nRoveSatID = 0;
	word32 validFP_base=0, validFP_rove=0, curvalidFP=0;
	word32 maxFP=0, fpfilter=0, tmpfp=0;
	int32 oldcommsvcnt,newcommsvcnt;
	double snr_base=0.0, locktime_base=0.0,snr_rove=0.0, locktime_rove=0.0;
	
	pRtkEpoch->nCommonSat = 0;
	memset(&(pRtkEpoch->CommonSat), 0, sizeof(pRtkEpoch->CommonSat));
	
	//在移动站解算所用卫星中寻找
	for(j=0; j<pRtkEpoch->Prm_Rove.satnmax; j++)
	{
		nRoveSatID = pRtkEpoch->Prm_Rove.obs[j].StarID;
		
		if (pRtkEpochKeep->slip[nRoveSatID-1]>0)
		{
			continue;
		}
		
		//判断仰角门限
		/*if (pRtkEpoch->Prm_Rove.obs[j].el < GetSVRTKElMask(nRoveSatID))
		{
			continue;
		}*/
		
		// 基准站中查找		
		for(k=0; k<pRtkEpoch->Prm_Base.satnmax; k++)
		{
			if(nRoveSatID == pRtkEpoch->Prm_Base.obs[k].StarID)
				break;
		}		
		if(k==pRtkEpoch->Prm_Base.satnmax)	// 基准站中未找到
		{				
			continue;//not found
		}

		//check obs valid, snr, locktime....
		// 读取全局变量pActiveCPT
		if(!RTKCoaseSVSelect(&(pRtkEpoch->Prm_Base.obs[k]), &validFP_base, &snr_base, &locktime_base))
		{
			continue;
		}
		if(!RTKCoaseSVSelect(&(pRtkEpoch->Prm_Rove.obs[j]), &validFP_rove, &snr_rove, &locktime_rove))
		{
			continue;
		}

		curvalidFP = validFP_base & validFP_rove;
		if(curvalidFP ==0)		// 无共同频点
			continue;

		//update frqpoint and sv for mode check
		maxFP |= curvalidFP;

		//update common sv info
		pRtkEpoch->CommonSat[pRtkEpoch->nCommonSat].SatID = nRoveSatID;
		pRtkEpoch->CommonSat[pRtkEpoch->nCommonSat].validFrqpint = curvalidFP;
		pRtkEpoch->CommonSat[pRtkEpoch->nCommonSat].cn0 = (int8)((snr_base+snr_rove)*0.5+0.5);
		locktime_base = (locktime_base+locktime_rove)*0.5*0.001;	//s
		if(locktime_base > 250)
			locktime_base=250;
		pRtkEpoch->CommonSat[pRtkEpoch->nCommonSat].LLI = (int8)(locktime_base);	//s

		pRtkEpoch->nCommonSat++;
		if(pRtkEpoch->nCommonSat >= MAX_RTK_OBS_SVCNT)
			break;
	}

	//select frqpoint
	// 无全局变量读写
	SelectRTKFrqpoint(pRtkEpoch->bSuperWide, maxFP, &(pRtkEpoch->SelectFrepMapSW), &(pRtkEpoch->SelectFrepMap), &(pRtkEpoch->SelectFrepMapL1), pRtkEpoch);

	//update comm SV info by selected freqpoint
	if((pRtkEpoch->nCommonSat <RTK_MIN_OBSCNT) || (pRtkEpoch->SelectFrepMap==0))
	{
		pRtkEpoch->nCommonSat=0;
	}
	else
	{
		oldcommsvcnt = pRtkEpoch->nCommonSat;
		newcommsvcnt=0;
		for(j=0; j<oldcommsvcnt; j++)
		{
			if(SV_IsGps(pRtkEpoch->CommonSat[newcommsvcnt].SatID))
				fpfilter = FREQ_GROUP_GPS;
			else if(SV_IsBd2(pRtkEpoch->CommonSat[newcommsvcnt].SatID))
				fpfilter = FREQ_GROUP_BD;
#if SUPPORT_GLONASS		
			else if(SV_IsGlo(pRtkEpoch->CommonSat[newcommsvcnt].SatID))
				fpfilter = FREQ_GROUP_GLO;
#endif
			else 
				continue;

#if SUPPORT_SUPER_WIDE_LANE
		#if 0
			if(pRtkEpoch->bSuperWide)
			{
				if((pRtkEpoch->SelectFrepMapSW & fpfilter) ==0 )
					tmpfp=0;
				else
					tmpfp = (pRtkEpoch->SelectFrepMap|pRtkEpoch->SelectFrepMapSW) & fpfilter;
			}
		#else
			if(pRtkEpoch->bSuperWide && (pRtkEpoch->SelectFrepMapSW != 0))	//support super wide lane
			{
				if(((pRtkEpoch->SelectFrepMapSW & fpfilter) ==0 ) || ((pRtkEpoch->SelectFrepMap & fpfilter)==0))
					tmpfp=0;
				else
					tmpfp = (pRtkEpoch->SelectFrepMap|pRtkEpoch->SelectFrepMapSW) & fpfilter;
			}
		#endif
			else
#endif
				tmpfp = pRtkEpoch->SelectFrepMap & fpfilter;

			if((tmpfp==0)
				||((tmpfp & FREQ_GROUP_IDX0) && ((pRtkEpoch->CommonSat[newcommsvcnt].validFrqpint & FREQ_GROUP_IDX0)==0))	//delete this sv
				||((tmpfp & FREQ_GROUP_IDX1) && ((pRtkEpoch->CommonSat[newcommsvcnt].validFrqpint & FREQ_GROUP_IDX1)==0))
				||((tmpfp & FREQ_GROUP_IDX2) && ((pRtkEpoch->CommonSat[newcommsvcnt].validFrqpint & FREQ_GROUP_IDX2)==0)))
			{
				if(j<(oldcommsvcnt-1))
					memcpy(&(pRtkEpoch->CommonSat[newcommsvcnt]), &(pRtkEpoch->CommonSat[newcommsvcnt+1]), sizeof(BD2_SAT_INFO)*(oldcommsvcnt-j-1));
			}
			else
			{
				pRtkEpoch->CommonSat[newcommsvcnt].validFrqpint &= tmpfp;
				newcommsvcnt++;
			}
		}

		pRtkEpoch->nCommonSat = newcommsvcnt;
	}
	
	return 0;
}

void SelectRTKFrqpoint(bool bSuperWide, word32 frqpoint, word32* pSupWideLaneFP, word32* pWideLaneFP, word32* pSingleFP, RTK_EPOCH* pRtkEpoch)
{
	int32 i=0, sysidx=0, tmpsvcnt;
	int32 svcnt_new[MAX_SYSM]={0,0,0}/*, fpcnt_new[MAX_SYSM]={0,0,0}*/;
	word32 frep_new[MAX_SYSM]={0,0,0};
	word32 freqSelect_SW=0, freqSelect_W=0, freqSelect_N=0;
	word32 FrepPority[MAX_SYSM][2]=
	{
		{(FREQ_GROUP_L1|FREQ_GROUP_L2), 0							},
		{(FREQ_GROUP_B1|FREQ_GROUP_B3), (FREQ_GROUP_B1|FREQ_GROUP_B2)},
		{(FREQ_GROUP_G1|FREQ_GROUP_G2),	FREQ_GROUP_G1},
	};

	word32 SuperWLFrep[MAX_SYSM]={(0),(FREQ_GROUP_B2|FREQ_GROUP_B3),(0)};
		
	for(sysidx=0; sysidx<MAX_SYSM; sysidx++)	//GNSS system
	{
		//wide lane or single point
		for(i=0; i<2; i++)	// poriority 
		{
			if(CalcFPointGroupCnt(frqpoint & FrepPority[sysidx][i]) == CalcFPointGroupCnt(FrepPority[sysidx][i]))	
			{
				tmpsvcnt=CalcCommSVCntByFreqPoint(0x1<<sysidx, FrepPority[sysidx][i], pRtkEpoch); 
				if(tmpsvcnt>=RTK_SINGLE_MIN_OBSCNT)	//svcnt
				{
					svcnt_new[sysidx] = tmpsvcnt;
					frep_new[sysidx] = FrepPority[sysidx][i];
					//fpcnt_new[sysidx] = CalcFPointGroupCnt(FrepPority[sysidx][i]);
					break;
				}
			}
		}

		if(svcnt_new[sysidx]<RTK_SINGLE_MIN_OBSCNT)
			continue;

		//super wide lane
		if(bSuperWide)
		{
			if(CalcFPointGroupCnt(frqpoint & SuperWLFrep[sysidx]) == CalcFPointGroupCnt(SuperWLFrep[sysidx]))	
			{
				tmpsvcnt=CalcCommSVCntByFreqPoint(0x1<<sysidx, FrepPority[sysidx][i], pRtkEpoch); 
				if(tmpsvcnt>=RTK_SINGLE_MIN_OBSCNT)	//svcnt
					freqSelect_SW |= SuperWLFrep[sysidx];
			}
		}
		
		//wide lane
		freqSelect_W |= frep_new[sysidx];

		//single point
	#if 1
		if((frep_new[sysidx] & FREQ_GROUP_IDX0)!=0)
			freqSelect_N |=(frep_new[sysidx] & FREQ_GROUP_IDX0);
		else if((frep_new[sysidx] & FREQ_GROUP_IDX1)!=0)
			freqSelect_N |=(frep_new[sysidx] & FREQ_GROUP_IDX1);
		else if((frep_new[sysidx] & FREQ_GROUP_IDX2)!=0)
			freqSelect_N |=(frep_new[sysidx] & FREQ_GROUP_IDX2);
	#else
		if((freqSelect_SW & FREQ_GROUP_IDX0)!=0)
			freqSelect_N |=(freqSelect_SW & FREQ_GROUP_IDX0);
		else if((freqSelect_SW & FREQ_GROUP_IDX1)!=0)
			freqSelect_N |=(freqSelect_SW & FREQ_GROUP_IDX1);
		else if((freqSelect_SW & FREQ_GROUP_IDX2)!=0)
			freqSelect_N |=(freqSelect_SW & FREQ_GROUP_IDX2);
	#endif
	}

	*pSupWideLaneFP = freqSelect_SW;
	*pWideLaneFP = freqSelect_W;
	*pSingleFP = freqSelect_N;

	return;
}


int32 CalcCommSVCntByFreqPoint(int32 navsys, word32 frqpoint, RTK_EPOCH* pRtkEpoch)
{
	int32 idx=0, svcnt=0;
	
	for(idx=0; idx<pRtkEpoch->nCommonSat; idx++)
	{
		if(((frqpoint & FREQ_GROUP_IDX0) && ((pRtkEpoch->CommonSat[idx].validFrqpint & FREQ_GROUP_IDX0)==0))
			||((frqpoint & FREQ_GROUP_IDX1) && ((pRtkEpoch->CommonSat[idx].validFrqpint & FREQ_GROUP_IDX1)==0))
			||((frqpoint & FREQ_GROUP_IDX2) && ((pRtkEpoch->CommonSat[idx].validFrqpint & FREQ_GROUP_IDX2)==0)))
			continue;
			
		if((navsys==NAV_SYS_GPS)&& SV_IsGps(pRtkEpoch->CommonSat[idx].SatID))
			svcnt++;
		else if((navsys==NAV_SYS_BD)&& SV_IsBd2(pRtkEpoch->CommonSat[idx].SatID))
			svcnt++;
#if SUPPORT_GLONASS
		else if((navsys==NAV_SYS_GLO)&& SV_IsGlo(pRtkEpoch->CommonSat[idx].SatID))
			svcnt++;
#endif
	}

	return svcnt;
}

/*--------------------------------------------------------------------------------------
// 功能:根据单点定位时的卫星仰角确定基准星
// 参数: 
// [IN] pConfig		配置信息
// [OUT]pOutput		计算结果输出
// 返回: 
--------------------------------------------------------------------------------------*/
//#pragma CODE_SECTION(GetKeySatNo,"sect_ECODE_IV");
CHAR GetKeySatNo(RTK_EPOCH* pRtkEpoch, RTK_EPOCH_Keep* pRtkEpochKeep)
{
	int32	dMaxEle = 0, dMaxCN0=0;
	int32	dMaxEleIdx=-1, dMaxCN0Idx = -1;
	int32 	i = 0, j = 0,k, sysidx=0, lastKeyIdx=-1, svcnt=0;
	int8 difcn0=0;
	int32 oldcommsvcnt=0, newcommsvcnt=0;
	word32 fpmap=0;

	for(sysidx=0; sysidx<MAX_SYSM; sysidx++)	//inital
	{
		pRtkEpoch->KeySatIdIndex[sysidx] = -1;
		pRtkEpoch->KeySatId[sysidx] = 0;
		
		dMaxEle = 0;dMaxCN0=0;
		lastKeyIdx=-1;
		dMaxEleIdx=-1; dMaxCN0Idx = -1;
		svcnt = 0;
		for(j=0; j<pRtkEpoch->nCommonSat; j++)
		{
			if(GetNavSysIdx(pRtkEpoch->CommonSat[j].SatID) != sysidx)
				continue;

			svcnt++;

			if((pRtkEpochKeep->nKeySatID[sysidx]>0)&&(pRtkEpochKeep->bLastIntValid)) //模糊度保持，则选择原列表中的卫星
			{
				if(GetBitWord256(&(pRtkEpochKeep->L1UsedSVMap), pRtkEpoch->CommonSat[j].SatID-1)==0)
					continue;
			}
			
			//if(SV_IsBd2Geo(pRtkEpoch->CommonSat[j].SatID))
			//	continue;

			if(pRtkEpoch->CommonSat[j].SatID == pRtkEpochKeep->nKeySatID[sysidx])
				lastKeyIdx = j;

			//if((dMaxCN0 < RtkEpoch.CommonSat[j].cn0) && (RtkEpoch.Prm_Rove.obs[j].el>=45))
			//{
			//	dMaxCN0 = RtkEpoch.CommonSat[j].cn0;
			//	dMaxCN0Idx = j;
			//}
			
			if(dMaxEle < pRtkEpoch->Prm_Rove.obs[j].el)
			{
				dMaxEle = pRtkEpoch->Prm_Rove.obs[j].el;
				dMaxEleIdx = j;
			}
		}

		if(dMaxCN0Idx>=0)
			pRtkEpoch->KeySatIdIndex[sysidx] = dMaxCN0Idx;
		else if(dMaxEleIdx>=0)
			pRtkEpoch->KeySatIdIndex[sysidx] = dMaxEleIdx;

		if((lastKeyIdx>=0) && (pRtkEpoch->KeySatIdIndex[sysidx] != lastKeyIdx))
		{
			difcn0 = pRtkEpoch->CommonSat[pRtkEpoch->KeySatIdIndex[sysidx]].cn0 - pRtkEpoch->CommonSat[lastKeyIdx].cn0;
			if((pRtkEpoch->Prm_Rove.obs[lastKeyIdx].el>=45) && (pRtkEpoch->CommonSat[lastKeyIdx].cn0>=37)
				&& (difcn0<5))
			{
				pRtkEpoch->KeySatIdIndex[sysidx] = lastKeyIdx;	
			}
			else	//check new keysv is new sv?
			{
				for(k=0; k<pRtkEpochKeep->nCommonSat; k++)
				{
					if(pRtkEpochKeep->CommonSat[k].SatID == pRtkEpoch->CommonSat[pRtkEpoch->KeySatIdIndex[sysidx]].SatID)
						break;
				}

				if(k>=pRtkEpochKeep->nCommonSat)	//new sv
					pRtkEpoch->KeySatIdIndex[sysidx] = lastKeyIdx;	//use old key sv	
			}	
		}

		if((pRtkEpoch->KeySatIdIndex[sysidx]>=0) && (svcnt>=RTK_SINGLE_MIN_OBSCNT))	// key sv found
			pRtkEpoch->KeySatId[sysidx] = pRtkEpoch->CommonSat[pRtkEpoch->KeySatIdIndex[sysidx]].SatID;	//保存基准星相关结果
		else if(svcnt>0)	//delete this system sv
		{
			fpmap = GetNavSysFPMap(sysidx);
			pRtkEpoch->SelectFrepMap &= (~fpmap);
			pRtkEpoch->SelectFrepMapL1 &= (~fpmap);
			pRtkEpoch->KeySatIdIndex[sysidx] = -1;
			
			oldcommsvcnt = pRtkEpoch->nCommonSat;
			newcommsvcnt=0;
			for(j=0; j<oldcommsvcnt; j++)
			{
				if(GetNavSysIdx(pRtkEpoch->CommonSat[newcommsvcnt].SatID)==sysidx)
				{
					if(j<(oldcommsvcnt-1))
					{
						memcpy(&(pRtkEpoch->CommonSat[newcommsvcnt]), &(pRtkEpoch->CommonSat[newcommsvcnt+1]), sizeof(BD2_SAT_INFO)*(oldcommsvcnt-j-1));
						memcpy(&(pRtkEpoch->Prm_Base.obs[newcommsvcnt]), &(pRtkEpoch->Prm_Base.obs[newcommsvcnt+1]), sizeof(OBST)*(oldcommsvcnt-j-1));
						memcpy(&(pRtkEpoch->Prm_Rove.obs[newcommsvcnt]), &(pRtkEpoch->Prm_Rove.obs[newcommsvcnt+1]), sizeof(OBST)*(oldcommsvcnt-j-1));
					}
				}
				else
					newcommsvcnt++;
			}

			pRtkEpoch->nCommonSat = newcommsvcnt;
		}
			
	}

	//确定无基准星的公共星列表,用于模糊度解算
	j = 0;
	for(i=0; i<pRtkEpoch->nCommonSat; i++)
	{
		if(IsCurCommKeyIdx(i, pRtkEpoch))	// Is key idx?
			continue;

		memcpy(&(pRtkEpoch->CommonSat_Amb[j++]), &(pRtkEpoch->CommonSat[i]), sizeof(BD2_SAT_INFO));
	}

	pRtkEpoch->nCommonSatAMB = j;
	
	return 1;
}

void CycleSlipCheckSingleFp(RTK_EPOCH* pRtkEpoch, RTK_EPOCH_Keep* pRtkEpochKeep, word32 frqpoint)
{
	double phaseDS[LAMBDA_N_DIM];
	int i,j,k, sysidx;
	int SatID;
	double chk1,chk2, chk3,chk4;
	
	RTK_GetObsDS(pRtkEpoch, phaseDS, frqpoint, OBS_PHASE, OBS_SINGLE, DDRESULT_SAT_INDEX);

	//静态
	if(pRtkEpochKeep->begin<=1)
		pRtkEpochKeep->begin++;
	else 	//check
	{	
		for(sysidx=0; sysidx<MAX_SYSM; sysidx++)
		{

			if(((sysidx==0) && (frqpoint&FREQ_GROUP_GPS==0))
				||((sysidx==1) && (frqpoint&FREQ_GROUP_BD==0))
				||((sysidx==2) && (frqpoint&FREQ_GROUP_GLO==0)))
				continue;
			
			chk2=-1E100;				
			//基准星单差的历元三差值
			for(j=0; j<pRtkEpochKeep->nCommonSat; j++)	// 上一时间历元
			{
				if(pRtkEpoch->KeySatId[sysidx]!=pRtkEpochKeep->CommonSat[j].SatID)
					continue;
				for(k=0; k<pRtkEpochKeep->nCommonSat2; k++)	// 上上时间历元
				{
					if(pRtkEpoch->KeySatId[sysidx]!=pRtkEpochKeep->CommonSat2[k].SatID)
						continue;
					chk2=phaseDS[pRtkEpoch->KeySatIdIndex[sysidx]]-2*pRtkEpochKeep->phaseDSKeep[j]+pRtkEpochKeep->phaseDSKeep2[k];
					chk3=phaseDS[pRtkEpoch->KeySatIdIndex[sysidx]]-pRtkEpochKeep->phaseDSKeep[j];
					break;
				}
			}
			
			if(chk2>-0.9E100)
			{
				for(i=0; i<pRtkEpoch->nCommonSat; i++)
				{	
					SatID=pRtkEpoch->CommonSat[i].SatID;
					if ((SatID<=0) || (SatID==pRtkEpoch->KeySatId[sysidx]))
						continue;			
					for(j=0; j<LAMBDA_N_DIM; j++)
					{
						if((SatID!=pRtkEpochKeep->CommonSat[j].SatID) || (pRtkEpoch->KeySatId[sysidx]==pRtkEpochKeep->CommonSat[j].SatID))	// 上一历元公共星（非基准星）
							continue;
						for(k=0; k<LAMBDA_N_DIM; k++)
						{
							if(SatID!=pRtkEpochKeep->CommonSat2[k].SatID)	// 上上一历元公共星（非基准星）
								continue;
							chk1=phaseDS[i]-2*pRtkEpochKeep->phaseDSKeep[j]+pRtkEpochKeep->phaseDSKeep2[k];//公共星单差的历元三差值
							chk1=fabs(chk1-chk2);	// 双差的历元三差
							chk4=phaseDS[i]-pRtkEpochKeep->phaseDSKeep[j];
							pRtkEpoch->Phase_DDD[SatID] = chk1;	//ke zheng li
							if (chk1>0.45)	// 可探测半周
							{
								pRtkEpochKeep->slip[SatID-1]=7;
							}
							else
							{
								pRtkEpochKeep->slip[SatID-1]=0;
							}
							break;
						}
					}
				}
			}
		}
	}		

	//update last and last last DS data
	memcpy(pRtkEpochKeep->phaseDSKeep2,pRtkEpochKeep->phaseDSKeep,LAMBDA_N_DIM*sizeof(double));
	memcpy(pRtkEpochKeep->phaseDSKeep,phaseDS,LAMBDA_N_DIM*sizeof(double));	

	memcpy(pRtkEpochKeep->CommonSat2,pRtkEpochKeep->CommonSat,LAMBDA_N_DIM*sizeof(BD2_SAT_INFO));
	pRtkEpochKeep->nCommonSat2 = pRtkEpochKeep->nCommonSat;
	memcpy(pRtkEpochKeep->CommonSat,pRtkEpoch->CommonSat,LAMBDA_N_DIM*sizeof(BD2_SAT_INFO));	
	pRtkEpochKeep->nCommonSat = pRtkEpoch->nCommonSat;
	return;
}

void CycleSlipCheckDoubleFp(word32 frqpoint, int32 recvrType, char* pSlip, RTK_EPOCH* pRtkEpoch, RTK_EPOCH_Keep* pRtkEpochKeep)
{
    int32 i,j, idx, SatID;
	static double lasttime=0.0;
	static double diff=0.0;
	static bool firsttime=TRUE;
	double gamma;
	double chk1,chk11,chk2;
	double lamda1,lamda2;
	char slipL1[LAMBDA_N_DIM]={0,};
	
	word32 sysfrqpoint=0;
	word32 usedfrqp[2]={0,0}, usedfrqpidx[2]={0,0};
	double phase[2]={0.0,}, phaseOld[2]={0.0,};
	double range=0.0, rangeOld=0.0;
	double rfFreq[2]={0.0,0.0}, lamb[2]={0.0,0.0};
	OBSEV *pOldPrm=NULL;

	if(recvrType == RCV_BASE)
		pOldPrm = &(pRtkEpochKeep->prmOldBase);
	else
		pOldPrm = &(pRtkEpochKeep->prmOldRove);

	if(pOldPrm->satnmax<=0)
		return;

	for(i=0; i<pRtkEpoch->nCommonSat; i++)
	{
		SatID=pRtkEpoch->CommonSat[i].SatID;

		sysfrqpoint = frqpoint & (pRtkEpoch->CommonSat[i].validFrqpint);

		j=0;
		for(idx = 0; idx<FREQ_CNT; idx++)	//get the valid freqpoint info for this sv
		{
			if(sysfrqpoint & (0x1<<idx))
			{
				usedfrqp[j] = (0x1<<idx);	//freqpoint 
				usedfrqpidx[j] = GetFreGropIdx(usedfrqp[j]);	//freqpoint idx  in obs
				rfFreq[j] = GetFreqpointRF(SatID, usedfrqp[j]);	//freqpoint RF frequency
				lamb[j] = GetSVWaveLen(SatID, usedfrqp[j]);		//wave length
				j++;
			}
		}
			
		range = RTK_GetObsData(pRtkEpoch, recvrType, SatID, i, usedfrqp[0], OBS_RANGE, OBS_SINGLE);	
		phase[0] = RTK_GetObsData(pRtkEpoch, recvrType, SatID, i, usedfrqp[0], OBS_PHASE, OBS_SINGLE);	
		phase[1] = RTK_GetObsData(pRtkEpoch, recvrType, SatID, i, usedfrqp[1], OBS_PHASE, OBS_SINGLE);	
		
		for(j=0; j<pOldPrm->satnmax; j++)
		{
			if (SatID==pOldPrm->obs[j].StarID)
			{
				rangeOld = pOldPrm->obs[j].range[usedfrqpidx[0]];
				phaseOld[0] = pOldPrm->obs[j].phase[usedfrqpidx[0]];
				phaseOld[1] = pOldPrm->obs[j].phase[usedfrqpidx[1]];
				break;
			}
		}
		
		if (j==pOldPrm->satnmax)	//not find the same sv
			continue;
		
		//周跳检测处理
		
		//1电离层残差法,探测周跳
		//gamma=1561.098/1207.14;   //BD  B1B2
		//gamma=1575.42/1227.60;  //GPS  L1L2
		gamma = rfFreq[0] / rfFreq[1];
		chk1 = phase[0] - gamma * phase[1];
		chk2 = phaseOld[0] - gamma * phaseOld[1];
		chk1 -= (phaseOld[0] - gamma * phaseOld[1]);
		slipL1[i]=0;
		if (fabs(chk1)>0.28)
		{
			slipL1[i]=1;
		}
		//if(firsttime)
		//diff=fabs(fabs(chk1)-lasttime);
		//lasttime=fabs(chk1);
		
		//2伪距载波相位组合法,探测周跳
		//freqpoint 1
		chk11 = phase[0]-phaseOld[0];
		lamda1=range/phase[0];
		lamda2=rangeOld/phaseOld[0];
		chk2 = (range-rangeOld)/lamb[0];
		chk11 = (range-rangeOld)/lamb[0] - chk11;
		if (fabs(chk11)>7.0)
		{
			slipL1[i]=1;
		}
		
		//freqpoint 2
		chk11 = phase[1]-phaseOld[1];
		lamda1=range/phase[1];
		lamda2=rangeOld/phaseOld[1];
		chk2 = (range-rangeOld)/lamb[1];
		chk11 = (range-rangeOld)/lamb[1] - chk11;

		if (fabs(chk11)>7.0)
		{
			slipL1[i]=1;
		}
	}	

	memcpy(pSlip, &(slipL1[0]), sizeof(CHAR)*LAMBDA_N_DIM);

	return;
}

/*--------------------------------------------------------------------------------------
// 功能:周跳检测与修复
// 参数: 
// [IN/OUT] epochCur         BD2_EPOCH数据
// [IN/OUT] CommonSatAmbOld	 BD2_SAT_INFO数据
// 返回: 错误标志，负值表示发生错误
--------------------------------------------------------------------------------------*/
BOOL PhaseCycleSlipCheck(RTK_EPOCH* pRtkEpoch, RTK_EPOCH_Keep* pRtkEpochKeep)
{
	int32 sysidx, fpcnt=0, j, SatID;
	//int32 idx;
	word32 sysfrqpoint=0, singleFrqp=0, doubleFrqp=0;
	char slip_base[LAMBDA_N_DIM]={0,}, slip_rove[LAMBDA_N_DIM]={0,};
	BOOL isSlip=FALSE;
	word32 frqpoint = pRtkEpoch->SelectFrepMap;

	memset(slip_base, 0, sizeof(slip_base));
	memset(slip_rove, 0, sizeof(slip_rove));

	for(sysidx=0; sysidx<MAX_SYSM; sysidx++)
	{
		if(sysidx ==0)
			sysfrqpoint = frqpoint & FREQ_GROUP_GPS;
		else if(sysidx ==1)
			sysfrqpoint = frqpoint & FREQ_GROUP_BD;
	#if SUPPORT_GLONASS
		else if(sysidx ==2)
			sysfrqpoint = frqpoint & FREQ_GROUP_GLO;
	#endif

		fpcnt = CalcFPointGroupCnt(sysfrqpoint);
		if(fpcnt == 1)	// 单频
			singleFrqp |= sysfrqpoint;
		else if(fpcnt == 2)	// 双频
			doubleFrqp |= sysfrqpoint;
	}

	//if(singleFrqp != 0)	// 单频周跳检测
	//	CycleSlipCheckSingleFp(pRtkEpoch, pRtkEpochKeep, singleFrqp);
	/*else */if(doubleFrqp != 0)	// 双频周跳检测
	{
		CycleSlipCheckDoubleFp(doubleFrqp, RCV_BASE, slip_base, pRtkEpoch, pRtkEpochKeep);
		CycleSlipCheckDoubleFp(doubleFrqp, RCV_ROVE, slip_rove, pRtkEpoch, pRtkEpochKeep);

		for(j=0; j<pRtkEpoch->nCommonSat; j++)
		{
			SatID=pRtkEpoch->CommonSat[j].SatID;	

			if((slip_base[j] == 1) || (slip_rove[j] == 1))	// 基准站或者移动站发生周跳
			{
				pRtkEpochKeep->slip[SatID-1] = 7*(100/(pActiveCPT->SysmCptWorkConfig.FixUpdateCycle));
				isSlip = TRUE;
			}
			else
				pRtkEpochKeep->slip[SatID-1] = 0;
		}
	}
		
	return isSlip;
}

//ObvType   0:Code 1:Phase
//#pragma CODE_SECTION(GetSatELWeight,"sect_ECODE_IV");
double GetSatELWeight(double dSatEl, BOOL ObvType)
{
	double a = 0;
	double b = 0;
	double c = 0;
	double dWeight = 0;

	//Param
	a = ObvType?0.004:1.5;
	b = ObvType?0.003:1.0;
	c = ObvType?0.001:0.3;

	// cal weight
	dWeight = a*a + b*b/sin(dSatEl)/sin(dSatEl) + c*c;

	return dWeight;
}

//#pragma CODE_SECTION(GetSatELWeight_Exp,"sect_ECODE_IV");
double GetSatELWeight_Exp(double dSatEl, BOOL ObvType)
{	
	double dRet = 0;
	double a0 = ObvType?(3):(26);
	double a1 = ObvType?(70):(600);
	dRet = a0+a1*exp(-dSatEl*180/PI/20);
	return dRet*dRet;
}

//#pragma CODE_SECTION(GetDDWeight_ELSNR,"sect_ECODE_IV");
double GetDDWeight_ELSNR(OBST prmObvBase, OBST prmObvRove, RTK_EPOCH* pRtkEpoch)
{
	double dSinH = 0;
	double dSnr = 45;
	int i[2]={0,0}, svid, fpcnt, idx, j;
	word32 curSVFp=0;
	double dSatEl = 0.0;

	//仰角加权
	dSatEl = MIN(prmObvBase.el, prmObvRove.el);
	dSatEl = (dSatEl<15) ? 15 : dSatEl;	//15*Pi/180 = 0.261799388
	dSinH = sin(dSatEl);

	//信噪比加权
	svid = prmObvRove.StarID;
	if(SV_IsGps(svid))
		curSVFp = FREQ_GROUP_GPS & pRtkEpoch->SelectFrepMap;
	else if(SV_IsBd2(svid))
		curSVFp = FREQ_GROUP_BD & pRtkEpoch->SelectFrepMap;
#if SUPPORT_GLONASS
	else if(SV_IsGlo(svid))
		curSVFp = FREQ_GROUP_GLO & pRtkEpoch->SelectFrepMap;
#endif

	fpcnt = CalcFPointGroupCnt(curSVFp);
	
	if(fpcnt==1)
	{
		i[0] = GetFreGropIdx(curSVFp);
		dSnr = 0.5*(prmObvBase.snr0[i[0]] + prmObvRove.snr0[i[0]]);
	}
	else if(fpcnt==2)
	{
		j=0;
		for(idx=0; idx<FREQ_CNT; idx++)
		{
			if(curSVFp & (1<<idx))
			{
				i[j] = GetFreGropIdx(1<<idx);
				j++;
			}
		}

		dSnr = 0.5*(prmObvBase.snr0[i[0]] + prmObvRove.snr0[i[0]]);
		dSnr += 0.5*(prmObvBase.snr0[i[1]] + prmObvRove.snr0[i[1]]);
		dSnr *= 0.5;
	}

	//dSnr = (dSnr<MIN_SNR) ? MIN_SNR : dSnr;
	//dSnr = (dSnr>MAX_SNR) ? MAX_SNR : dSnr;	
	
	dSinH *= dSinH;
	dSnr *= dSnr;	
	return MAX_SNR*MAX_SNR/dSinH/dSnr;
}
//功能：初始化搜索参数
BOOL InitAttDetPara(RTK_EPOCH* pRtkEpoch, RTK_EPOCH_Keep* pRtkEpochKeep)
{
	int i = 0, j = 0, n0 = 0, sysidx, svid;
	double 	RangeVar = 0, PhaseVar = 0;
	double 	RangeVarKey[MAX_SYSM] = {0,};	
	double 	PhaseVarKey[MAX_SYSM] = {0,};

	// 设置定向的校验方式
	pRtkEpoch->cChkMode = 0;
	pRtkEpoch->cChkMode |= RTK_CHECK_RESIDUAL;
	if(IsMoveBaseForRove())
	{
		pRtkEpoch->cChkMode |= RTK_CHECK_PITCH;
		pRtkEpoch->cChkMode |= (g_dBaseLineLen==-1) ? RTK_CHECK_RATIO : RTK_CHECK_BSL;
	}
	else
	{
		pRtkEpoch->cChkMode |= RTK_CHECK_RATIO;
	}
	n0 = pRtkEpoch->nCommonSatAMB;
	
	//初始化数据变量
	if(pRtkEpochKeep->bFirstCalAmb)
	{
		for(i=0; i<n0; i++)
		{
			for(j=0; j<n0; j++)
			{
				pRtkEpochKeep->Rtk_PKeep[i*n0+j] = 0; 
			}
			pRtkEpochKeep->Rtk_PKeep[i*n0+i] = CORVARIANCE_NUM;
			pRtkEpochKeep->Rtk_NKeep[i] = 0;
		}		
		for(i=0; i<SV_NUM_TRUE; i++)
		{
			pRtkEpochKeep->rtk_satinfo[i].Amb = PI;	//默认整周模糊度为PI，即不可用状态
		}
	}	
	
	// 搜索次数与卫星数、输出频率相关
	pRtkEpoch->nSingleSearch = (g_nOIC>=5) ? 2 : LAMBDA_N_CANDS;
	pRtkEpoch->bAMB_SK = (g_nOIC>=5) ? FALSE : TRUE;
	
	// 默认参数
	RangeVarKey[0] = 1*RANGE_VAR;
	RangeVarKey[1] = 1*RANGE_VAR;
	RangeVarKey[2] = 1*RANGE_VAR;

	PhaseVarKey[0] = 1*PHASE_VAR;
	PhaseVarKey[1] = 1*PHASE_VAR;
	PhaseVarKey[2] = 1*PHASE_VAR;
	

	for(i=0; i<n0; i++)
	{	
		//other sat index
		svid = pRtkEpoch->CommonSat_Amb[i].SatID;
		sysidx = GetNavSysIdx(svid);

		RangeVar = RangeVarKey[sysidx];
		PhaseVar = PhaseVarKey[sysidx];

		for(j=0; j<n0; j++)
		{				
			pRtkEpoch->RangeCovariance[i*n0+j] = (i==j)?(2*(RangeVar+RangeVarKey[sysidx])):(2*RangeVarKey[sysidx]);
			pRtkEpoch->PhaseCovariance[i*n0+j] = (i==j)?(2*(PhaseVar+PhaseVarKey[sysidx])):(2*PhaseVarKey[sysidx]);				
		}
	}

	LAMBDA_MO_MatrixInv_LU(pRtkEpoch->RangeCovariance, pRtkEpoch->RangeCovarianceInv, n0);
	LAMBDA_MO_MatrixInv_LU(pRtkEpoch->PhaseCovariance, pRtkEpoch->PhaseCovarianceInv, n0);

	return TRUE;
}


//功能：双差观测量处理
BOOL GetDD_OBV(RTK_EPOCH* pRtkEpoch)
{
	int i=0;
	for(i=0; i<pRtkEpoch->nCommonSatAMB; i++)
	{
#if SUPPORT_SUPER_WIDE_LANE
		if(pRtkEpoch->bSuperWide && (pRtkEpoch->SelectFrepMapSW!=0))
			pRtkEpoch->wavelen_SNW[i] =  RTK_GetLambda(pRtkEpoch->CommonSat_Amb[i].SatID, (pRtkEpoch->CommonSat_Amb[i].validFrqpint & pRtkEpoch->SelectFrepMapSW), OBS_WIDE);
#endif

		pRtkEpoch->wavelen_NW[i] =  RTK_GetLambda(pRtkEpoch->CommonSat_Amb[i].SatID, (pRtkEpoch->CommonSat_Amb[i].validFrqpint & pRtkEpoch->SelectFrepMap), OBS_WIDE);
		if(pRtkEpoch->SelectFrepMapL1 != pRtkEpoch->SelectFrepMap)
			pRtkEpoch->wavelen_NS[i] = RTK_GetLambda(pRtkEpoch->CommonSat_Amb[i].SatID, (pRtkEpoch->CommonSat_Amb[i].validFrqpint & pRtkEpoch->SelectFrepMapL1), OBS_SINGLE);
		else
		 	pRtkEpoch->wavelen_NS[i] = pRtkEpoch->wavelen_NW[i];
	}
	
	RTK_GetObsDD(pRtkEpoch, pRtkEpoch->Range_DD, pRtkEpoch->SelectFrepMapL1, OBS_RANGE, OBS_SINGLE, DDRESULT_SAT_INDEX);

#if SUPPORT_SUPER_WIDE_LANE
	if(pRtkEpoch->bSuperWide && (pRtkEpoch->SelectFrepMapSW!=0))
		RTK_GetObsDD(pRtkEpoch, pRtkEpoch->Phase_DD_SNW, pRtkEpoch->SelectFrepMapSW, OBS_PHASE, OBS_WIDE, DDRESULT_SAT_INDEX);
#endif
	
	RTK_GetObsDD(pRtkEpoch,pRtkEpoch->Phase_DD_NW, pRtkEpoch->SelectFrepMap, OBS_PHASE, OBS_WIDE, DDRESULT_SAT_INDEX);
	
	if(pRtkEpoch->SelectFrepMapL1 != pRtkEpoch->SelectFrepMap)
		RTK_GetObsDD(pRtkEpoch, pRtkEpoch->Phase_DD_NS,  pRtkEpoch->SelectFrepMapL1, OBS_PHASE, OBS_SINGLE, DDRESULT_SAT_INDEX);
	else
		memcpy(pRtkEpoch->Phase_DD_NS, pRtkEpoch->Phase_DD_NW, sizeof(pRtkEpoch->Phase_DD_NW));
	
	return TRUE;
}


//功：整周模糊度浮点解卡尔曼滤波处理
//#pragma CODE_SECTION(FloatSolution_Kalman,"sect_ECODE_IV");
BOOL FloatSolution_Kalman(RTK_EPOCH* pRtkEpoch, RTK_EPOCH_Keep* pRtkEpochKeep,double* dVecN, double* Nk, double* Pk)
{
	int n0 = pRtkEpoch->nCommonSatAMB;
	double	ADTemp[LAMBDA_N_DIM*3], ApaInvTemp[LAMBDA_N_DIM*3];
	double 	temp1[LAMBDA_N_DIM*LAMBDA_N_DIM];
	double 	temp2[LAMBDA_N_DIM*LAMBDA_N_DIM];
	double 	temp3[LAMBDA_N_DIM*LAMBDA_N_DIM];
	double* pWavlen=NULL;
	bool	bInvFlag = 0;
	int32 idx=0;

	memset(temp1,0,sizeof(double)*LAMBDA_N_DIM*LAMBDA_N_DIM);
	memset(temp2,0,sizeof(double)*LAMBDA_N_DIM*LAMBDA_N_DIM);
	memset(temp3,0,sizeof(double)*LAMBDA_N_DIM*LAMBDA_N_DIM);
	memset(ADTemp, 0 , sizeof(double)*LAMBDA_N_DIM*3);
	memset(ApaInvTemp, 0 , sizeof(double)*LAMBDA_N_DIM*3);

	//调整模糊度和方差矩阵，考虑了卫星的升降
	AdjustCalSatAmb(pRtkEpoch, pRtkEpochKeep, Nk, Pk, dVecN);	

#if SUPPORT_SUPER_WIDE_LANE
	if(pRtkEpoch->bSuperWide && (pRtkEpoch->SelectFrepMapSW !=0))
		pWavlen = pRtkEpoch->wavelen_SNW;
	else
#endif
		pWavlen = pRtkEpoch->wavelen_NW;
	
	//update AD and ADInv for phase
	for(idx=0; idx<pRtkEpoch->nCommonSatAMB; idx++)
	{
		ADTemp[idx*3 + 0] = pRtkEpoch->AD[idx*3 + 0]/pWavlen[idx];
		ADTemp[idx*3 + 1] = pRtkEpoch->AD[idx*3 + 1]/pWavlen[idx];
		ADTemp[idx*3 + 2] = pRtkEpoch->AD[idx*3 + 2]/pWavlen[idx];
	}
		
	//滤波浮点解与相关协方差矩阵
	if(pRtkEpochKeep->bFirstCalAmb)//如果首次计算，直接用估算整周模糊度
	{		
		LAMBDA_MO_MatrixAPAT(ADTemp, pRtkEpoch->ApaInv, temp1, n0, 3);
		LAMBDA_MO_MatrixPlus(temp1, pRtkEpoch->PhaseCovariance, Pk, n0, n0);	
	}
	else
	{	
		//第二次计算， 最小二乘法估算整周模糊度
		LAMBDA_MO_MatrixAPAT(ADTemp, pRtkEpoch->ApaInv, temp1, n0, 3);	//H*((H'W1H)^-1)*H'
		
		LAMBDA_MO_MatrixPlus(temp1, pRtkEpoch->PhaseCovariance, temp1, n0, n0);	//H*((H'W1H)^-1)*H'+W2
		bInvFlag = LAMBDA_MO_MatrixInv_LU(temp1, temp2, n0);// temp2 = Pk_float		(H*((H'W1H)^-1)*H'*W2)^-1

		bInvFlag = LAMBDA_MO_MatrixInv_LU(Pk,temp3, n0);// Pk_0		p^-1
		LAMBDA_MO_MatrixPlus(temp2, temp3, temp1, n0, n0);	//	((H*((H'W1H)^-1)*H'*W2)^-1)*(P^-1)
		bInvFlag = LAMBDA_MO_MatrixInv_LU(temp1, Pk, n0);	// Pk_1  	p=(((H*((H'W1H)^-1)*H'*W2)^-1)+(P^-1))^-1
		
		LAMBDA_MO_MatrixSub(dVecN, Nk, temp3, n0, 1);	//temp3 = Nk - Nkeep, 	Z(k)-Z(k-1)
		LAMBDA_MO_MatrixMulti(temp2, temp3, temp1, n0, n0, 1);	//( (H*((H'W1H)^-1)*H'*W2)^-1 )* (Z(k)-Z(k-1))
		LAMBDA_MO_MatrixMulti(Pk, temp1, temp2, n0, n0, 1);
		LAMBDA_MO_MatrixPlus(temp2, Nk, Nk, n0, 1);	
	}

	//保证矩阵对称正定
	LAMBDA_MO_MatrixMakeSymmetry(Pk, n0);	

	return TRUE;
}
//功能：整周模糊度搜索及输出
//#pragma CODE_SECTION(Ambiguity_Resolution,"sect_ECODE_IV");
BOOL Ambiguity_Resolution(RTK_EPOCH* pRtkEpoch, RTK_EPOCH_Keep* pRtkEpochKeep,double* Nk, double* Pk, double* dRatio, double* dVecN, double* dVecNFloat)
{
	int		nCalAmb = -1;
	int n0 = pRtkEpoch->nCommonSatAMB;
	double afixed[LAMBDA_N_DIM*LAMBDA_N_CANDS];
	double sqnorm[LAMBDA_N_CANDS];
	double Qhat[LAMBDA_N_DIM*LAMBDA_N_DIM];
	double Z[LAMBDA_N_DIM*LAMBDA_N_DIM];
	double amplify[LAMBDA_N_DIM];
	int nAmbSearchFail = 0;
	int i = 0;
	int j = 0;

	memset(afixed,0,sizeof(double)*LAMBDA_N_DIM*LAMBDA_N_CANDS);
	memset(sqnorm,0,sizeof(double)*LAMBDA_N_CANDS);
	memset(Qhat,0,sizeof(double)*LAMBDA_N_DIM*LAMBDA_N_DIM);
	memset(Z,0,sizeof(double)*LAMBDA_N_DIM*LAMBDA_N_DIM);
	memset(amplify,0,sizeof(double)*LAMBDA_N_DIM);

	if (pRtkEpoch->nSingleSearch>0)
	{
		nCalAmb = M_LAMBDA(Nk, Pk, n0, pRtkEpoch->nSingleSearch, afixed, sqnorm, Qhat, Z);
	}
	
	if((nCalAmb<0) || (0==sqnorm[0]))
	{	//未搜索到结，输出浮点解
		nAmbSearchFail++;		//如果连续搜索失败3秒，则重新开始滤波
		if(nAmbSearchFail>=3)	pRtkEpochKeep->bFirstCalAmb = TRUE;
		return FALSE;
	}
	else
	{
		nAmbSearchFail = 0;
		//记录候选整周模糊度
		for(j=0;j<pRtkEpoch->nSingleSearch;j++)
		{
			for(i=0;i<n0; i++)
			{
				dVecN[j*n0+i] = floor(0.5+afixed[i*pRtkEpoch->nSingleSearch+j]);
			}			
		}
		dRatio[0] = sqnorm[1]/sqnorm[0];
	}

	//保存模糊度
	if(dRatio[0]>2.5)
	{
		if(dRatio[0]>200)
			dRatio[0]=200;
		if(g_nSetAmbN>=100)
		{
			for(i=0; i<n0; i++)
			{				
				;//RtkEpochKeep.Rtk_NKeep[i]=dVecN[i];			
			}
			g_nSetAmbN=0;
		}
		g_nSetAmbN++;
	}
	else
		g_nSetAmbN=0;	

	//Super wide lane switch to wide lane
#if SUPPORT_SUPER_WIDE_LANE
	if(pRtkEpoch->bSuperWide && (pRtkEpoch->SelectFrepMapSW!=0))
	{
		memcpy(pRtkEpoch->dAMB_SNW_Fix,dVecN,sizeof(double)*LAMBDA_N_DIM*LAMBDA_N_CANDS); //save Super Wide lane N
		
		for(i=0; i<pRtkEpoch->nCommonSatAMB; i++)
		{
			amplify[i] = pRtkEpoch->wavelen_SNW[i] / pRtkEpoch->wavelen_NW[i];
		}

		memset(afixed,0,sizeof(double)*LAMBDA_N_DIM*LAMBDA_N_CANDS);
		LAMBDA_MO_MatrixSub(pRtkEpoch->Phase_DD_SNW, dVecN, afixed, n0, 1);
		LAMBDA_MO_MatrixAmplify1(afixed, amplify, n0, 1);
		LAMBDA_MO_MatrixSub(pRtkEpoch->Phase_DD_NW,afixed, dVecN, n0, 1);	

		LAMBDA_MO_Round(dVecN, 1, n0, dVecN);

		memcpy(pRtkEpoch->dAMB_NW_Fix,dVecN,sizeof(double)*LAMBDA_N_DIM*LAMBDA_N_CANDS); //save NW
	}
	else
#endif
		memcpy(pRtkEpoch->dAMB_NW_Fix,dVecN,sizeof(double)*LAMBDA_N_DIM*LAMBDA_N_CANDS); //save NW

	// 双频整周模糊度转换为单频	
	// 算法: N1 = lam2/(lam2-lam1)*(N12-Phi12)+Phi1 = lam12/lam1*(N12-Phi12)+Phi1
	if(pRtkEpoch->SelectFrepMapL1 != pRtkEpoch->SelectFrepMap)
	{
		for(i=0; i<pRtkEpoch->nCommonSatAMB; i++)
		{
			amplify[i] = pRtkEpoch->wavelen_NW[i] / pRtkEpoch->wavelen_NS[i];
		}
		
		// 浮点解转换
		//LAMBDA_MO_MatrixSub(Qhat,dVecNFloat, afixed, n0, 1);
		//LAMBDA_MO_MatrixAmplify1(afixed, amplify, n0, 1);
		//LAMBDA_MO_MatrixSub(Z,afixed, dVecNFloat, n0, 1);	
		

		//搜索的整数解转换
		//for(i=0; i<RtkEpoch.nSingleSearch; i++)
		for(i=0; i<1; i++)
		{	
			memset(afixed,0,sizeof(double)*LAMBDA_N_DIM*LAMBDA_N_CANDS);
			LAMBDA_MO_MatrixSub(pRtkEpoch->Phase_DD_NW,dVecN+i*n0, afixed, n0, 1);
			LAMBDA_MO_MatrixAmplify1(afixed, amplify, n0, 1);
			LAMBDA_MO_MatrixSub(pRtkEpoch->Phase_DD_NS,afixed, dVecN+i*n0, n0, 1);	
			//if (i==0)	memcpy(RtkEpoch.dAMB_B1B3_S_Float, dVecN, sizeof(DOUBLE)*n0);	// 切单频浮点解
			
			//搜索的整数解转换(平滑)	20130426
			if(i==0)
			{
				SmmothD2S_Float(pRtkEpoch, pRtkEpochKeep, dVecN);
				//memcpy(RtkEpoch.dAMB_B1B3_S_Float, dVecN, sizeof(DOUBLE)*n0);	// 切单频浮点解
			}

			LAMBDA_MO_Round(dVecN+i*n0, 1, n0, dVecN+i*n0);

		}//end of for(i=0; i<RtkEpoch.nSingleSearch; i++)

	}
	return TRUE;
}


//#pragma CODE_SECTION(Kalman_Float_AMB,"sect_ECODE_IV");
void Kalman_Float_AMB(DOUBLE Zk, DOUBLE* X0, DOUBLE* P0)
{
	// 噪声系统
	const double dQ = 1e-3;
	const double dR = 0.5;
	
	// 时间更新，预测过程
	double P_10 = P0[0] + dQ;

	// 状态更新，校正过程
	double K_1 = P_10 / (P_10 + dR);
	double X_1 = X0[0] + K_1*(Zk-X0[0]);
	double P_1 = (1-K_1)*P_10;

	// 更新后的状态矢量和估计误差协方差存在一个矩阵中,第一行为状态矢量,第二行至14行为后验估计误差的协方差
	X0[0] = X_1;
	P0[0] = P_1;
}


void SmmothD2S_Float(RTK_EPOCH* pRtkEpoch, RTK_EPOCH_Keep* pRtkEpochKeep, double* dSF_Float)
{
	int i=0;
	int nSatID = 0;

	// 按卫星分别滤波
	for(i=0; i<pRtkEpoch->nCommonSatAMB; i++)
	{
		nSatID = pRtkEpoch->CommonSat_Amb[i].SatID;
		
		// 滤波预测值与实测值误差超过1周，则认为有周跳，重置滤波。
		if (ROUND(fabs(dSF_Float[i] - pRtkEpochKeep->dSF_AMB_Smooth[nSatID-1]))>=1)
		{
			pRtkEpochKeep->nAMB_Smooth_Flag[nSatID-1] = FALSE;
		}

		if (!pRtkEpochKeep->nAMB_Smooth_Flag[nSatID-1])
		{
			pRtkEpochKeep->nAMB_Smooth_Flag[nSatID-1] = TRUE;
			pRtkEpochKeep->dSF_AMB_Smooth[nSatID-1] = dSF_Float[i];
			pRtkEpochKeep->dSF_AMB_Pk[nSatID-1] = 1.0;
		} 	

		Kalman_Float_AMB(dSF_Float[i], &(pRtkEpochKeep->dSF_AMB_Smooth[nSatID-1]), &(pRtkEpochKeep->dSF_AMB_Pk[nSatID-1]));
	}

	// 输出数据
	for (i=0; i<pRtkEpoch->nCommonSatAMB; i++)
	{
		nSatID = pRtkEpoch->CommonSat_Amb[i].SatID;
		dSF_Float[i] = pRtkEpochKeep->dSF_AMB_Smooth[nSatID-1];
	}

	return;
}


//功能:根据基线矢量确定姿态
//#pragma CODE_SECTION(Attitude_Determination_UseBSL,"sect_ECODE_IV");
BOOL Attitude_Determination_UseBSL(BD2_BASELINE* bsl, BD2_Attitude* att)
{
	double dEastNorth = 0.0;

	//如果基线长度为0(小于1cm，不作定向解算)
	if(fabs(bsl->BaselineLongth)<0.1)
		{
		//航向角	
		att->yaw = 0;
		att->yaw_deg = 0;
		//俯仰角
		dEastNorth = 0;
		att->pitch = 0;
		att->pitch_deg = 0;
		return FALSE;
		}
	
	//航向角	
	att->yaw = atan_2(bsl->East, bsl->North);
	att->yaw_deg = att->yaw*RADIAN_TO_DEGREE;
	//俯仰角
	dEastNorth = sqrt(bsl->East*bsl->East+bsl->North*bsl->North);
	att->pitch = atan(bsl->Up/dEastNorth);
	att->pitch_deg = att->pitch*RADIAN_TO_DEGREE;

	return TRUE;
}


void SaveLastCommonSatInfo(bool bNeed2Keep, RTK_EPOCH* pRtkEpoch, RTK_EPOCH_Keep* pRtkEpochKeep)
{
	double obstow=0.0;

	if(!bNeed2Keep)
	{
		memset(pRtkEpochKeep, 0, sizeof(RTK_EPOCH_Keep));
		pRtkEpochKeep->bFirstCalAmb = TRUE;
		return;
	}
	
	memcpy(pRtkEpochKeep->nKeySatID , pRtkEpoch->KeySatId, sizeof(char)*MAX_SYSM);
	pRtkEpochKeep->nCommonSat = pRtkEpoch->nCommonSat;
	pRtkEpochKeep->nCommonSatAMB = pRtkEpoch->nCommonSatAMB;
	memcpy(pRtkEpochKeep->CommonSat_Amb, pRtkEpoch->CommonSat_Amb, sizeof(BD2_SAT_INFO)*LAMBDA_N_DIM);
	memcpy(pRtkEpochKeep->CommonSat, pRtkEpoch->CommonSat, sizeof(BD2_SAT_INFO)*MAX_RTK_OBS_SVCNT);	

	pRtkEpochKeep->nAmbIndex = pRtkEpoch->nAmbIndex;
	pRtkEpochKeep->nAmbIndex_wide = pRtkEpoch->nAmbIndex_wide;
	pRtkEpochKeep->nAmbIndex_wfloat = pRtkEpoch->nAmbIndex_wfloat;

	pRtkEpochKeep->bSuperWide = pRtkEpoch->bSuperWide;

	if((pRtkEpoch->nAmbIndex>0) || (pRtkEpoch->nAmbIndex_wide>0) || (pRtkEpoch->nAmbIndex_wfloat>0))
	{
		if(GetObsvTime(&(pRtkEpoch->Prm_Rove), &obstow, NAV_SYS_NONE))
			pRtkEpochKeep->tow = obstow;
	}

	memcpy(&(pRtkEpochKeep->prmOldBase), &(pRtkEpoch->Prm_Base), sizeof(OBSEV));
	memcpy(&(pRtkEpochKeep->prmOldRove), &(pRtkEpoch->Prm_Rove), sizeof(OBSEV));

	memcpy(&(pRtkEpochKeep->L1UsedSVMap), &(pRtkEpoch->L1UsedSVMap), sizeof(pRtkEpoch->L1UsedSVMap));

	UpdateCommSV_N(pRtkEpoch,pRtkEpochKeep);

	return;
}

/*--------------------------------------------------------------------------------------
// 功能:调整模糊度和方差矩阵
// 参数: 
// [IN/OUT] NKeep	 模糊度参数(上个历元）
// [IN/OUT] PKeep	 方差矩阵(上个历元）
// [  IN  ] Nk	     模糊度参数(当前历元）
// 返回: 错误标志，负值表示发生错误
--------------------------------------------------------------------------------------*/
//#pragma CODE_SECTION(AdjustCalSatAmb,"sect_ECODE_IV");
BOOL AdjustCalSatAmb(RTK_EPOCH* pRtkEpoch, RTK_EPOCH_Keep* pRtkEpochKeep, double* NKeep,double* PKeep,double* Nk)
{
	int i=0; 
	int j=0;
	int m=0;
	int n=0;
	int satid,satTwo;
	int key1;
	int32 sysidx=0;

	double NkNew[LAMBDA_N_DIM];
	double PkNew[LAMBDA_N_DIM*LAMBDA_N_DIM];
	double Ak[LAMBDA_N_DIM*LAMBDA_N_DIM];
	double Nk2[LAMBDA_N_DIM];
	double Pk2[LAMBDA_N_DIM*LAMBDA_N_DIM];
	
	memset(NkNew,0,sizeof(double)*LAMBDA_N_DIM);
	memset(PkNew,0,sizeof(double)*LAMBDA_N_DIM*LAMBDA_N_DIM);
	memset(Ak,0,sizeof(double)*LAMBDA_N_DIM*LAMBDA_N_DIM);
	memset(Nk2,0,sizeof(double)*LAMBDA_N_DIM);
	memset(Pk2,0,sizeof(double)*LAMBDA_N_DIM*LAMBDA_N_DIM);

	//如果是首次计算，不需要比较基准星和公共星
	if(pRtkEpochKeep->bFirstCalAmb)
	{		
		for(sysidx=0; sysidx<MAX_SYSM; sysidx++)
		{
			if(pRtkEpoch->KeySatIdIndex[sysidx]<0)
				continue;

			pRtkEpochKeep->rtk_satinfo[pRtkEpoch->KeySatId[sysidx]-1].bKeyStar = TRUE;
			pRtkEpoch->KeySatID_NewOld[sysidx][0] = pRtkEpoch->KeySatId[sysidx];
		}
			
		memcpy(NKeep,Nk,sizeof(double)*LAMBDA_N_DIM);
		return TRUE;
	}

	//当第二次解算整周模糊度时，需要比较判断卫星升降
	//Case 1:基准星改变
	pRtkEpoch->bKeySatChanged = FALSE;
	for(sysidx=0; sysidx<MAX_SYSM; sysidx++)
	{
		key1 = -1;
		if((pRtkEpoch->KeySatIdIndex[sysidx]>=0) && (pRtkEpochKeep->nKeySatID[sysidx]>0)
			&& (pRtkEpoch->KeySatId[sysidx] != pRtkEpochKeep->nKeySatID[sysidx]))
		{
			pRtkEpoch->bKeySatChanged = TRUE;
			
			for(m=0; m<pRtkEpochKeep->nCommonSatAMB; m++)	// 在上一历元公共卫星中寻找当前历元的基准星
			{
				if(pRtkEpochKeep->CommonSat_Amb[m].SatID==pRtkEpoch->KeySatId[sysidx])
				{
					key1=m;
				}

				Ak[m*pRtkEpochKeep->nCommonSatAMB+m]=1;
			}	
			if(key1<0)	// 基准星改变且为新上升卫星，暂时重置，有优化空间
			{  
				pRtkEpochKeep->bFirstCalAmb=TRUE;
				return TRUE;
			}
			else	// 新基准星在原有的公共卫星中，只需要旋转矩阵即可
			{
				for(n=0; n<pRtkEpochKeep->nCommonSatAMB; n++)
				{
					if(GetNavSysIdx(pRtkEpochKeep->CommonSat_Amb[n].SatID)!=sysidx)
						continue;
					
					Ak[n*pRtkEpochKeep->nCommonSatAMB+key1]=-1;
				}			
				
				for(n=0; n<pRtkEpochKeep->nCommonSatAMB; n++)
				{
					if(pRtkEpochKeep->CommonSat_Amb[n].SatID==pRtkEpoch->KeySatId[sysidx])
					{
						pRtkEpochKeep->CommonSat_Amb[n].SatID=pRtkEpochKeep->nKeySatID[sysidx];//将新基准星从公共星矩阵中取出，将上一次的基准星放入该位置
					}
				}
				pRtkEpochKeep->bFirstCalAmb=FALSE;
			} 
		}
	}

	if((pRtkEpochKeep->bFirstCalAmb==FALSE) && pRtkEpoch->bKeySatChanged)
	{
		LAMBDA_MO_MatrixMulti(Ak, NKeep, Nk2, pRtkEpochKeep->nCommonSatAMB, pRtkEpochKeep->nCommonSatAMB, 1);
		LAMBDA_MO_MatrixAPAT(Ak,PKeep,Pk2, pRtkEpochKeep->nCommonSatAMB, pRtkEpochKeep->nCommonSatAMB); 
		memcpy(NKeep,Nk2,sizeof(double)*LAMBDA_N_DIM);
		memcpy(PKeep,Pk2,sizeof(double)*LAMBDA_N_DIM*LAMBDA_N_DIM);
	}	
	
	//Case 2:公共卫星中有上升,模糊度改变,方差阵改变
	pRtkEpoch->bSatChanged = FALSE;	
	if (pRtkEpochKeep->nCommonSatAMB<=0 || pRtkEpoch->nCommonSatAMB<=0)
	{
       	pRtkEpochKeep->bFirstCalAmb = TRUE;
		return TRUE;
	}

	for(i=0; i<pRtkEpoch->nCommonSatAMB; i++)
	{
		PkNew[i*pRtkEpoch->nCommonSatAMB+i]=CORVARIANCE_NUM;
		satid = pRtkEpoch->CommonSat_Amb[i].SatID;
		for(m=0; m<pRtkEpochKeep->nCommonSatAMB; m++)
		{
			if (satid==pRtkEpochKeep->CommonSat_Amb[m].SatID)
				break;
		}

		if ((m==pRtkEpochKeep->nCommonSatAMB) || (pRtkEpochKeep->slip[satid-1]>0))//未找到或周跳，有卫星上升
		{
		    NkNew[i]=Nk[i];		
			if (m==pRtkEpochKeep->nCommonSatAMB)
			{
				pRtkEpochKeep->rtk_satinfo[satid-1].UpDown = 1;
				pRtkEpoch->bSatChanged = TRUE;
			}
			pRtkEpochKeep->rtk_satinfo[satid-1].Amb = PI;
			continue;
		}
		else
		{
			NkNew[i]=NKeep[m];
		}
		
		for(j=0; j<pRtkEpoch->nCommonSatAMB; j++)
		{
			satTwo = pRtkEpoch->CommonSat_Amb[j].SatID;			
			for(n=0; n<pRtkEpochKeep->nCommonSatAMB; n++)
			{
				if (satTwo==pRtkEpochKeep->CommonSat_Amb[n].SatID)
					break;
			}
			if ((n!=pRtkEpochKeep->nCommonSatAMB)&& (pRtkEpochKeep->slip[satTwo-1]==0))//找到并且无周跳
			{
				PkNew[i*pRtkEpoch->nCommonSatAMB+j]=PKeep[m*pRtkEpochKeep->nCommonSatAMB+n];	
				pRtkEpochKeep->rtk_satinfo[satid-1].UpDown = 0;
			}	
		} 
	}

	//Case 3 :卫星下降
	for(m=0; m<pRtkEpochKeep->nCommonSatAMB; m++)
	{
		satid = pRtkEpochKeep->CommonSat_Amb[m].SatID;
		for(i=0; i<pRtkEpoch->nCommonSatAMB; i++)//上一历元卫星在当前历元中寻找
		{
			if (satid==pRtkEpoch->CommonSat_Amb[i].SatID)
				break;
		}
		if (i==pRtkEpoch->nCommonSatAMB)//未找到
		{
			pRtkEpochKeep->rtk_satinfo[satid-1].UpDown = -1;
			pRtkEpoch->bSatChanged = TRUE;
			pRtkEpochKeep->rtk_satinfo[satid-1].Amb = PI;
		}
	}
	//存储模糊方差
	memcpy(NKeep,NkNew,sizeof(double)*LAMBDA_N_DIM);
	memcpy(PKeep,PkNew,sizeof(double)*LAMBDA_N_DIM*LAMBDA_N_DIM);

	for(sysidx=0; sysidx<MAX_SYSM; sysidx++)
	{
		pRtkEpoch->KeySatID_NewOld[sysidx][0] = pRtkEpoch->KeySatId[sysidx];
		pRtkEpoch->KeySatID_NewOld[sysidx][1] = pRtkEpochKeep->nKeySatID[sysidx];
	}

	return TRUE;	
}

//获取星地距离及方向矢量
//#pragma CODE_SECTION(RTK_GetSatDist,"sect_ECODE_IV");
double RTK_GetSatDist(int32 svid, ECEF satpos, ECEF rcvpos, double* dDirectVecX, double* dDirectVecY, double* dDirectVecZ)
{
	double dx = 0;
	double dy = 0;
	double dz = 0;
	double dist = 0;
	double omega_e = 0;
	double alpha = 0;
	
	dx = satpos.x - rcvpos.x;
	dy = satpos.y - rcvpos.y;
	dz = satpos.z - rcvpos.z;
	dist = sqrt(dx*dx+dy*dy+dz*dz);		

	// 球自转角速度
	if(SV_IsGps(svid))
		omega_e = GetBDREFParam(BDREF_PARAM_w, GPS_SYSTEM);
	else if(SV_IsBd2(svid))
		omega_e = GetBDREFParam(BDREF_PARAM_w, BD2_SYSTEM);
	else
		omega_e = GetBDREFParam(BDREF_PARAM_w, GPS_SYSTEM);
		
	alpha=(dist/BD2_LIGHT_SPEED) * omega_e;
	dx = (satpos.x*cos(alpha) + satpos.y * sin(alpha)) - rcvpos.x;
	dy = (satpos.y*cos(alpha) - satpos.x * sin(alpha)) - rcvpos.y;
	dz =  satpos.z - rcvpos.z; 
	dist = sqrt(dx*dx+dy*dy+dz*dz);		
	
	//方向矢量
	if((dDirectVecX != NULL) && (dDirectVecY != NULL) && (dDirectVecZ != NULL))
	{
		*dDirectVecX = -dx/dist;
		*dDirectVecY = -dy/dist;
		*dDirectVecZ = -dz/dist;
	}
	return dist;
}

//计算双差数据
// nObsType = 	1：L1伪距双差    
//				2：L2伪距双差    
//				3：L1载波双差    
//				4：L2载波双差    
//				5:	 双频载波双差
// DDRESULT_SAT_ID = 1:按卫星号	2:
//#pragma CODE_SECTION(RTK_GetObsDD,"sect_ECODE_IV");
INT32 RTK_GetObsDD(RTK_EPOCH* pRtkEpoch, double* dObsDD, word32 frqpoint,int nObsType, int fWideOrNarrow, int nDDResultType)
{
	int j=0, sysidx;
	int nsat = 0, svid=0;
	int nCount = 0;
	double y1k[MAX_SYSM] = {0.0,},  y2k[MAX_SYSM] = {0.0,};	//key sv
	double y1i = 0.0, y2i = 0.0;	// other sv
	double baseErr=0.0;
	double waveLen = 0.0;

	//基准星数据
	for(sysidx = 0; sysidx< MAX_SYSM; sysidx++)
	{
		if(pRtkEpoch->KeySatIdIndex[sysidx]<0)
			continue;
		
		svid = pRtkEpoch->KeySatId[sysidx];
		j=pRtkEpoch->KeySatIdIndex[sysidx];
		y1k[sysidx] = RTK_GetObsData(pRtkEpoch, RCV_BASE, svid, pRtkEpoch->KeySatIdIndex[sysidx], (frqpoint&pRtkEpoch->CommonSat[j].validFrqpint), nObsType, fWideOrNarrow);
		baseErr = (pRtkEpoch->Prm_Base.obs[j].clkerr - pRtkEpoch->Prm_Rove.obs[j].clkerr)*SPEED_OF_LIGHT - pRtkEpoch->Prm_Base.obs[j].distance + pRtkEpoch->DisSV2Base[j];
		if(nObsType == OBS_RANGE)
			y1k[sysidx] += baseErr;
		else
		{
			waveLen = RTK_GetLambda(svid, (frqpoint&pRtkEpoch->CommonSat[j].validFrqpint),fWideOrNarrow);
			y1k[sysidx] += (baseErr/waveLen);
		}
		
		y2k[sysidx] = RTK_GetObsData(pRtkEpoch, RCV_ROVE, svid, pRtkEpoch->KeySatIdIndex[sysidx], (frqpoint&pRtkEpoch->CommonSat[j].validFrqpint), nObsType, fWideOrNarrow);
	}

	//观测卫星数据
	for(j=0; j<pRtkEpoch->nCommonSat; j++)
	{
		svid = pRtkEpoch->CommonSat[j].SatID;
		sysidx = GetNavSysIdx(svid);

		if(j==pRtkEpoch->KeySatIdIndex[sysidx])	//j is key svidx
			continue;

		y1i = RTK_GetObsData(pRtkEpoch, RCV_BASE, svid, j, (frqpoint&pRtkEpoch->CommonSat[j].validFrqpint),nObsType, fWideOrNarrow);
		baseErr = (pRtkEpoch->Prm_Base.obs[j].clkerr - pRtkEpoch->Prm_Rove.obs[j].clkerr)*SPEED_OF_LIGHT - pRtkEpoch->Prm_Base.obs[j].distance + pRtkEpoch->DisSV2Base[j];
		if(nObsType == OBS_RANGE)
			y1i += baseErr;
		else
		{
			waveLen = RTK_GetLambda(svid, (frqpoint&pRtkEpoch->CommonSat[j].validFrqpint),fWideOrNarrow);
			y1i += (baseErr/waveLen);
		}
		
		y2i = RTK_GetObsData(pRtkEpoch, RCV_ROVE, svid, j, (frqpoint&pRtkEpoch->CommonSat[j].validFrqpint),nObsType, fWideOrNarrow);
		nsat = (DDRESULT_SAT_INDEX == nDDResultType)? nCount : pRtkEpoch->CommonSat[j].SatID;
		dObsDD[nsat] = (y2i - y2k[sysidx]) - (y1i - y1k[sysidx]);
		nCount++;
	} 

	return 1;
}


//计算单差数据
//#pragma CODE_SECTION(RTK_GetObsDS,"sect_ECODE_IV");
INT32 RTK_GetObsDS(RTK_EPOCH* pRtkEpoch, double* dObsDS, word32 frqpoint, int nObsType, int wideOrNarrow,int nDDResultType)
{
	int j=0;
	int nsat = 0;
	int nCount = 0;
	double y1i = 0;
	double y2i = 0;
	
	//观测卫星数据
	for(j=0; j<pRtkEpoch->nCommonSat; j++)
	{
		y1i = RTK_GetObsData(pRtkEpoch, RCV_BASE, pRtkEpoch->CommonSat[j].SatID, j, frqpoint, nObsType, wideOrNarrow);
		y2i = RTK_GetObsData(pRtkEpoch, RCV_ROVE, pRtkEpoch->CommonSat[j].SatID, j, frqpoint, nObsType, wideOrNarrow);
		nsat = (DDRESULT_SAT_INDEX == nDDResultType)? nCount : pRtkEpoch->CommonSat[j].SatID;
		dObsDS[nsat] = y2i - y1i;
		nCount++;
	} 
	return 1;
}


//获取观测量数据
// nRcvType   = 	1:Base    
//				2:Rove
//nObsType    =	1：L1伪距    
//				2：L2伪距    
//				3：L1载波    
//				4：L2载波    
//				5：L1-L2载波
//#pragma CODE_SECTION(RTK_GetObsData,"sect_ECODE_IV");
double RTK_GetObsData(RTK_EPOCH* pRtkEpoch, int nRcvType, int svid, int nSatIDIndex, word32 frqpoint, int prOrPh, int fWideOrNarrow)
{
	OBSEV *prm=NULL;
	double dObsData = 0.0;
	double dIono = 0.0,dTrop = 0.0,gnssRFFre=0.0,chk;
	int32 frqidx[2]={0,0};
	word32 singlefrqRF[2]={0,0};
	int32 frepointcnt=0;
	int32 idx=0, cnt=0;
	word32 frepFilter=0;

	//根据类型获取数据
	if (nRcvType==RCV_BASE)
	{
		prm=&(pRtkEpoch->Prm_Base);	
	}
	else
	{
		prm=&(pRtkEpoch->Prm_Rove);	
	}

	//if((DGPS_MODE == DGPS_MODE_STATC) && bCalRtkErr)
	if (0)
	{
		// 获取电离层参数
		/*
		SatID = RtkEpoch.CommonSat[nSatIDIndex].SatID;
		if (RtkEpoch.PosMode_Rtk.System == BD2_SYSTEM)
			Freq = FREQ_B1;
		else
			Freq = FREQ_L1;
		pIono = (BDION*)GlobalGetEph(PROTOCOL_TYPE_BDION, SatID, Freq, Branch, 0);

		dIono = KlobucharModel(curSat_azel.azimuth, curSat_azel.elevation, curPVT.blh.latitude, curPVT.blh.longitude, 
			RtkEpoch.time.SecOfDay, pIono, RtkEpoch.PosMode_Rtk.Freq[0], BD2_SYSTEM, nIQ+1);
		dTrop = CalculateTrop2(curPVT.blh.latitude, curPVT.blh.altitude, RtkEpoch.time.DayOfYear, curSat_azel.elevation);
		*/
	}
	else
	{
		dIono = 0.0;
		dTrop = 0.0;
	}

	if(SV_IsGps(svid))
		frepFilter = FREQ_GROUP_GPS;
	else if(SV_IsBd2(svid))
		frepFilter = FREQ_GROUP_BD;
#if SUPPORT_GLONASS
	else if(SV_IsGlo(svid))
		frepFilter = FREQ_GROUP_GLO;
#endif
	else 
		return 0.0;

	frqpoint = frqpoint & frepFilter;	// 每个系统单独处理

	frepointcnt = CalcFPointGroupCnt(frqpoint);
	if((frepointcnt==1) || (prOrPh == OBS_RANGE) || (fWideOrNarrow==OBS_SINGLE))	// only 1 freqpoint
	{
		frqidx[0] = GetFreGropIdx(frqpoint);
		if(prOrPh == OBS_RANGE)
		{
			dObsData = prm->obs[nSatIDIndex].range[frqidx[0]];
			chk=prm->obs[nSatIDIndex].range[frqidx[0]]/prm->obs[nSatIDIndex].phase[frqidx[0]];
			dObsData -= dTrop + dIono;
		}
		else if(prOrPh == OBS_PHASE)
		{
			gnssRFFre = GetFreqpointRF(svid, frqpoint);
			dObsData = prm->obs[nSatIDIndex].phase[frqidx[0]];
			chk=prm->obs[nSatIDIndex].range[frqidx[0]]/prm->obs[nSatIDIndex].phase[frqidx[0]];
			dObsData -= (dTrop - dIono)*gnssRFFre*RECIP_SPEED_OF_LIGHT;
		}
	}
	else if(frepointcnt==2)	// 2 frqpoint, only used for phase
	{
		cnt=0;
		for(idx=0; idx<FREQ_CNT; idx++)
		{
			if((0x1<<idx) & frqpoint)
			{
				singlefrqRF[cnt] = GetFreqpointRF(svid, (0x1<<idx));
				frqidx[cnt] = GetFreGropIdx(0x1<<idx);
				cnt++;
			}
			if(cnt>=2)
				break;
		}

		if(fWideOrNarrow == OBS_WIDE)	//wide
		{
			if(singlefrqRF[0]>singlefrqRF[1])
				dObsData = prm->obs[nSatIDIndex].phase[frqidx[0]]- prm->obs[nSatIDIndex].phase[frqidx[1]];
			else
				dObsData = prm->obs[nSatIDIndex].phase[frqidx[1]]- prm->obs[nSatIDIndex].phase[frqidx[0]];	
		}
		else	//narrow
		{
			dObsData = prm->obs[nSatIDIndex].phase[frqidx[0]]+ prm->obs[nSatIDIndex].phase[frqidx[1]];	
		}
	}
	
	return dObsData;
}

// 获取波长
double RTK_GetLambda(int32 svid, word32 frqpoint, int wideOrNarrow)
{  
	double dLambda;
	double rfFrq[2];
	word32 frp[2]={0,0};
	int32 frqcnt = 0, idx;

	for(idx=0; idx<FREQ_CNT; idx++)
	{
		if(frqpoint & (0x1<<idx))
		{
			frp[frqcnt] = (0x1<<idx);
			frqcnt++;
		}
	}

	//根据算法频率使用确定波长
	if(1 == frqcnt)	//单频
	{	
		dLambda = GetSVWaveLen(svid, frp[0]);	//GPS /BD  not use svid
	}
	else if(2 == frqcnt)	// 双频
	{
		rfFrq[0] = GetFreqpointRF(svid, frp[0]);	//299792458/(1561.098E6-1207.14E6)   0.84698476010629671882619311173304	
		rfFrq[1] = GetFreqpointRF(svid, frp[1]);	//299792458/(1575.42E6-1227.60E6)  0.86191840032200563509861422574895
		if(wideOrNarrow == OBS_WIDE)	//wide
			dLambda = SPEED_OF_LIGHT / fabs(rfFrq[0] - rfFrq[1]);
		else	//narrow
			dLambda = SPEED_OF_LIGHT / fabs(rfFrq[0] + rfFrq[1]);
	}

	return dLambda;
}

bool RTK_GetCurEpochTime(RTK_EPOCH* pRtkEpoch, double* pTime)
{
	bool ret= FALSE;
	
	ret = GetObsvTime(&pRtkEpoch->Prm_Rove, pTime, NAV_SYS_NONE);
	if(!ret)
		ret = GetObsvTime(&pRtkEpoch->Prm_Base, pTime,NAV_SYS_NONE);

	return ret;
}

bool IsCurCommKeyIdx(int32 idx, RTK_EPOCH* pRtkEpoch)
{
	int32 sysidx=0;
	bool ret = FALSE;
	
	for(sysidx=0; sysidx<MAX_SYSM; sysidx++)
	{
		if((pRtkEpoch->KeySatIdIndex[sysidx] >= 0) && (idx == pRtkEpoch->KeySatIdIndex[sysidx]))
		{
			ret = TRUE;
			break;
		}
	}

	return ret;
}

int32 RTKSelectSVL1(RTK_EPOCH* pRtkEpoch, word256* pSVMap, double ddres[SV_NUM_TRUE], int32* errcount)
{
	word256 svusedMap[MAX_SYSM]={{0,0,0,0,0,0,0,0},};
	int32 idx=0, sysdix,svcnt[MAX_SYSM]={0,0,0};
	int32 svcntusd=0;

	memset(svusedMap, 0, sizeof(svusedMap));
	memset(svcnt, 0, sizeof(svcnt));
	ResetWord256(pSVMap);

	for(idx=0; idx<pRtkEpoch->nCommonSat; idx++)
	{
		sysdix = GetNavSysIdx(pRtkEpoch->CommonSat[idx].SatID);
		
		if((ddres!=NULL) && (fabs(ddres[pRtkEpoch->CommonSat[idx].SatID-1])>0.25))
		{
			if(errcount!=NULL) 	
				(*errcount)++;
				
			continue;
		}

		SetBitWord256(&(svusedMap[sysdix]),pRtkEpoch->CommonSat[idx].SatID-1);
		svcnt[sysdix]++;
	}

	for(sysdix=0; sysdix<MAX_SYSM; sysdix++)
	{
		if(svcnt[sysdix]>=RTK_SINGLE_MIN_OBSCNT)
		{
			(*pSVMap) = OpOrWord256(pSVMap, &(svusedMap[sysdix]));
			svcntusd += svcnt[sysdix];
		}
	}

	return svcntusd;
}

bool RTKCheckPosValid(ECEF* pRtkPos, RTK_EPOCH* pRtkEpoch)
{
	ECEF sigpos={0.0,0.0,0.0};
	double dif = 0.0;
	if(pRtkEpoch->Prm_Rove.bSinglePosValid==FIX_NOT)
		return TRUE;

	sigpos.x = pRtkEpoch->Prm_Rove.SingleRcvrPos.x;
	sigpos.y = pRtkEpoch->Prm_Rove.SingleRcvrPos.y;
	sigpos.z = pRtkEpoch->Prm_Rove.SingleRcvrPos.z;
	
	dif = (pRtkPos->x-sigpos.x)*(pRtkPos->x-sigpos.x) + (pRtkPos->y-sigpos.y)*(pRtkPos->y-sigpos.y) + (pRtkPos->z-sigpos.z)*(pRtkPos->z-sigpos.z);
	if(dif<1000000.0)	//1000m
		return TRUE;

	return FALSE;
}

void DeleteSlipSV(RTK_EPOCH* pRtkEpoch, RTK_EPOCH_Keep* pRtkEpochKeep)
{
	int32 oldcommsvcnt=0, newcommsvcnt=0, j=0;

	oldcommsvcnt = pRtkEpoch->nCommonSat;
	newcommsvcnt=0;
	for(j=0; j<oldcommsvcnt; j++)
	{	
		if(pRtkEpochKeep->slip[pRtkEpoch->CommonSat[newcommsvcnt].SatID-1] > 0)	//delete this sv
		{
			if(j<(oldcommsvcnt-1))
			{
				memcpy(&(pRtkEpoch->CommonSat[newcommsvcnt]), &(pRtkEpoch->CommonSat[newcommsvcnt+1]), sizeof(BD2_SAT_INFO)*(oldcommsvcnt-j-1));
				memcpy(&(pRtkEpoch->Prm_Base.obs[newcommsvcnt]), &(pRtkEpoch->Prm_Base.obs[newcommsvcnt+1]), sizeof(OBST)*(oldcommsvcnt-j-1));
				memcpy(&(pRtkEpoch->Prm_Rove.obs[newcommsvcnt]), &(pRtkEpoch->Prm_Rove.obs[newcommsvcnt+1]), sizeof(OBST)*(oldcommsvcnt-j-1));
			}
		}
		else
		{			
			newcommsvcnt++;
		}
	}

	pRtkEpoch->nCommonSat = newcommsvcnt;
	pRtkEpoch->Prm_Base.satnmax = newcommsvcnt;
	pRtkEpoch->Prm_Rove.satnmax = newcommsvcnt;

	return;
}
	

#if CAL_BASELINE_WLS
double GetRTKLsWeigth(int32 idx, RTK_EPOCH* pRtkEpoch)
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
	
	if(SV_IsBd2Geo(pRtkEpoch->CommonSat[idx].SatID))
		tmp -= 5;
	if(SV_IsBd2(pRtkEpoch->CommonSat[idx].SatID))
		tmp -= 15;
	
	if(tmp <= 0)
		tmp = 1;
	if(tmp >= 90)
		tmp = 90;
		
	tmp = tmp*D2R;
	tmp = sin(tmp);
	return tmp;
}

#endif


void UpdateRTKUsedChnMap(word256* pChnmap, word32 fpused, RTK_EPOCH* pRtkEpoch)
{
	int32 trkch=0, svid=0, idx=0;
	word32 frqpint=0;
			
	for(trkch=0; trkch<MAXCHANNELS; trkch++)
	{
		svid = glStruAcqu.Info[trkch].SvID;
		frqpint = glStruAcqu.Info[trkch].FreqPoint;

		for(idx=0; idx<pRtkEpoch->nCommonSat; idx++)
		{
			if(svid==pRtkEpoch->CommonSat[idx].SatID)
			{
				if((pRtkEpoch->CommonSat[idx].validFrqpint & fpused & frqpint) !=0)
					SetBitWord256(pChnmap, trkch);
				
				break;
			}
		}
	}

	return;
}


int32 CalcRTKAverAndStd(RTK_EPOCH_Keep* pRtkEpochKeep, ECEF* pPos, int32 windowLen, ECEF* pAver, ECEF* pStd)
{
	static int32 posCountNumber=0;
	static ECEF aver={0.0,0.0,0.0};
	static ECEF std={0.0,0.0,0.0};

	double rate=0.0;

	if(pRtkEpochKeep->nAmbIndex<=0)
		posCountNumber=0;

	if(posCountNumber==0)	//initial
	{
		posCountNumber++;	
		std.x=std.y=std.z=0.0;
		memcpy(&aver, pPos, sizeof(aver));
	}
	else
	{	
		if(posCountNumber < windowLen)
			posCountNumber++;	
		
		//average
		//lastAver = aver;
		rate = 1.0/((double)posCountNumber);
		//rate2 = ((double)(posCountNumber-1.0)/(posCountNumber*posCountNumber*posCountNumber));
		aver.x= aver.x*((posCountNumber-1)*rate)+ pPos->x*rate;
		aver.y= aver.y*((posCountNumber-1)*rate)+ pPos->y*rate;
		aver.z= aver.z*((posCountNumber-1)*rate)+ pPos->z*rate;
		
		//std
		//std.x=std.x*((posCountNumber-1)*rate)+(lastAver.x-aver.x)*(lastAver.x-aver.x)*rate2+(aver.x-lastAver.x)*(aver.x-lastAver.x)*((posCountNumber-1)*rate2);
		//std.y=std.y*((posCountNumber-1)*rate)+(lastAver.y-aver.y)*(lastAver.y-aver.y)*rate2+(aver.y-lastAver.y)*(aver.y-lastAver.y)*((posCountNumber-1)*rate2);
		//std.z=std.z*((posCountNumber-1)*rate)+(lastAver.z-aver.z)*(lastAver.z-aver.z)*rate2+(aver.z-lastAver.z)*(aver.z-lastAver.z)*((posCountNumber-1)*rate2);
		std.x = pPos->x - aver.x;
		std.y = pPos->y - aver.y;
		std.z = pPos->z - aver.z;	
	}

	memcpy(pAver, &aver, sizeof(aver));
	memcpy(pStd, &std, sizeof(std));

	return posCountNumber;
}

double GetRTKStdChkTh(void)
{
	double thvalue = 1e-4;
	int8 movestate = GetCPTRcvrMoveState();
	ECEF vel={0.0,0.0,0.0};
	double speed = 0.0;

	if(movestate == RCVR_MOVESTATE_STATIC)
		thvalue = 0.02;
	else if(movestate == RCVR_MOVESTATE_HIGHDYNAMIC)
		thvalue = 10000;
	else if((movestate == RCVR_MOVESTATE_DYNAMIC) || (movestate == RCVR_MOVESTATE_AUTO))
	{
		if(getRcvrInfo(NULL,&vel,NULL,NULL)>=FIX_3D)
		{
			speed = sqrt(vel.x*vel.x + vel.y*vel.y + vel.z*vel.z);
			if(speed<1.0)
				thvalue = 0.02;
			else if(speed<3.0)
				thvalue = 50;
			else
				thvalue = 10000;
		}
		else
			thvalue = 10000;
	}

	return thvalue;
}

void InitalFloatN(RTK_EPOCH* pRtkEpoch, double* pEDDPr, double* pAMB_PrDD)
{
	int32 i=0;
	
	for(i=0; i<pRtkEpoch->nCommonSatAMB; i++)
	{
#if SUPPORT_SUPER_WIDE_LANE
		if(pRtkEpoch->bSuperWide & (pRtkEpoch->SelectFrepMapSW!=0))
			pAMB_PrDD[i] = pRtkEpoch->Phase_DD_SNW[i] - pEDDPr[i]/pRtkEpoch->wavelen_SNW[i];
		else
#endif
			pAMB_PrDD[i] = pRtkEpoch->Phase_DD_NW[i] - pEDDPr[i]/pRtkEpoch->wavelen_NW[i];
	}
		
	return;
}


int32 RTKSVSelectEN(RTK_EPOCH* pRtkEpoch, RTK_EPOCH_Keep* pRtkEpochKeep, word256* pSVValidMap, double* pN, int32 flag)
{
	int32 i=0, svcnt=0, svid=0, sysidx=0;
	double lastN=0.0, lastKeyN=0.0;
	word256* pValidMap=NULL;

	memset(pSVValidMap, 0, sizeof(word256));

	if(flag == FIX_L1_INT)
		pValidMap = &(pRtkEpochKeep->bAMB_NS_Fix_Valid);
	else if(flag == FIX_WIDE_INT)
		pValidMap = &(pRtkEpochKeep->bAMB_NW_Fix_Valid);
	else if(flag == FIX_WIDE_FLOAT)
		pValidMap = &(pRtkEpochKeep->bAMB_NWf_Valid);
	
	for(i=0; i<pRtkEpoch->nCommonSatAMB; i++)
	{
		svid = pRtkEpoch->CommonSat_Amb[i].SatID;

		if(GetBitWord256(pValidMap, svid-1)==0)
			continue;

		sysidx = GetNavSysIdx(svid);
		if(pRtkEpoch->KeySatId[sysidx]<=0)
			continue;

		if(flag == FIX_L1_INT)
		{
			lastN = pRtkEpochKeep->dAMB_NS_Fix[svid-1];
			lastKeyN = pRtkEpochKeep->dAMB_NS_Fix[pRtkEpoch->KeySatId[sysidx]-1];
		}
		else if(flag == FIX_WIDE_INT)
		{
			lastN = pRtkEpochKeep->dAMB_NW_Fix[svid-1];
			lastKeyN = pRtkEpochKeep->dAMB_NW_Fix[pRtkEpoch->KeySatId[sysidx]-1];
		}
		else if(flag == FIX_WIDE_FLOAT)
		{
			lastN = pRtkEpochKeep->dAMB_NWf[svid-1];
			lastKeyN = pRtkEpochKeep->dAMB_NWf[pRtkEpoch->KeySatId[sysidx]-1];
		}

		if(pRtkEpochKeep->nKeySatID[sysidx] != pRtkEpoch->KeySatId[sysidx])	// key sv changed
			pN[i] = lastN - lastKeyN;
		else
			pN[i] = lastN;	

		SetBitWord256(pSVValidMap, svid-1);
		svcnt++;
	}

	for(sysidx=0; sysidx<MAX_SYSM; sysidx++)
	{
		if(pRtkEpoch->KeySatIdIndex[sysidx]>=0)
		{
			SetBitWord256(pSVValidMap, pRtkEpoch->KeySatId[sysidx]-1);
			svcnt++;
		}
	}

	return svcnt;
}

void UpdateDistSV2Base(RTK_EPOCH* pRtkEpoch)
{
	int32 i=0, svid;
	
	for(i=0; i<pRtkEpoch->nCommonSat; i++)
	{
		svid = pRtkEpoch->CommonSat[i].SatID;
		
		pRtkEpoch->DisSV2Base[i] = RTK_GetSatDist(svid, pRtkEpoch->Prm_Rove.obs[i].svpos, Basefixposition.ecefpos, NULL, NULL, NULL);
	}

	return;
}

void UpdateSvPosForDifEph(RTK_EPOCH* pRtkEpoch)
{
	int32 idx = 0,jdx = 0, svid;
	word32 basetoe= 0,rovetoe=0,toeehp = 0;
	double tow=0.0,ts=0.0;
	ECEF svvel;
	double frqerr;
	OBSEV* pPrm;
	
	for (idx=0; idx<pRtkEpoch->nCommonSat; idx++ )
	{
		basetoe = pRtkEpoch->Prm_Base.obs[idx].toe;
		rovetoe = pRtkEpoch->Prm_Rove.obs[idx].toe;
		if(basetoe == rovetoe)
			continue;

		toeehp = Sys3AlmEphUTCInfo.GpsBd2_EphStruct[pRtkEpoch->Prm_Base.obs[idx].StarID-1].toe;

		if(toeehp==basetoe)
			pPrm = &(pRtkEpoch->Prm_Rove);
		else if(toeehp==rovetoe)
			pPrm = &(pRtkEpoch->Prm_Base);
		else
			continue;

		svid = pRtkEpoch->CommonSat[idx].SatID;

		if(GetObsvTime(pPrm, &tow,NAV_SYS_NONE)==FALSE)
			continue;
		
		for(jdx=0; jdx<MAX_FREPIONT_PER_NAVSYS; jdx++)
		{
			if(pPrm->obs[idx].validmap[jdx] & OBST_VALID_RANGE)
			{
				ts = tow - (pPrm->obs[idx].range[jdx]) * RECIP_SPEED_OF_LIGHT;	
				
				SWI_disable();
				CalcSV_PVT_Eph(pPrm->obs[idx].StarID, ts, &(pPrm->obs[idx].svpos), &svvel, &(pPrm->obs[idx].clkerr), &frqerr, &(pPrm->obs[idx].toe));
				SWI_enable();

				pPrm->obs[idx].distance = RTK_GetSatDist(svid, pPrm->obs[idx].svpos, Basefixposition.ecefpos, NULL, NULL, NULL);
				break;
			}
		}

	}	
}


void UpdateCommSV_N(RTK_EPOCH* pRtkEpoch, RTK_EPOCH_Keep* pRtkEpochKeep)
{
	int32 i=0, svid=0, sysidx=0, ambidx=0;
	double keySvSD[MAX_SYSM]={0.0,}, rou_Base, rou_Rove, deltaPhi=0.0;
	ECEF rovepos;

	memset(&(pRtkEpochKeep->bAMB_NS_Fix_Valid), 0, sizeof(pRtkEpochKeep->bAMB_NS_Fix_Valid));
	memset(&(pRtkEpochKeep->bAMB_NW_Fix_Valid), 0, sizeof(pRtkEpochKeep->bAMB_NW_Fix_Valid));
	memset(&(pRtkEpochKeep->bAMB_NWf_Valid), 0, sizeof(pRtkEpochKeep->bAMB_NWf_Valid));

	if((pRtkEpoch->nAmbIndex>0) || (pRtkEpoch->nAmbIndex_wide>0))
		memcpy(&rovepos, &(pRtkEpoch->RovePVT_Fix), sizeof(rovepos));
	else if(pRtkEpoch->nAmbIndex_wfloat>0)
		memcpy(&rovepos, &(pRtkEpoch->RovePVT_FLOAT), sizeof(rovepos));
	else
		return;
		
	//update key sv
	for(sysidx=0; sysidx<MAX_SYSM; sysidx++)
	{
		if(pRtkEpoch->KeySatIdIndex[sysidx] < 0)
			continue;
		
		svid = pRtkEpoch->KeySatId[sysidx];

		//update key sv SD
		rou_Base = RTK_GetSatDist(svid, pRtkEpoch->Prm_Rove.obs[pRtkEpoch->KeySatIdIndex[sysidx]].svpos, Basefixposition.ecefpos, NULL, NULL, NULL);
		rou_Rove = RTK_GetSatDist(svid, pRtkEpoch->Prm_Rove.obs[pRtkEpoch->KeySatIdIndex[sysidx]].svpos, rovepos, NULL, NULL, NULL);
		keySvSD[sysidx] = rou_Rove - rou_Base;

		//update key sv N
		pRtkEpochKeep->dAMB_NS_Fix[svid-1] = 0;
		SetBitWord256(&(pRtkEpochKeep->bAMB_NS_Fix_Valid),svid-1);
		
		pRtkEpochKeep->dAMB_NW_Fix[svid-1] = 0;
		SetBitWord256(&(pRtkEpochKeep->bAMB_NW_Fix_Valid),svid-1);
		
		pRtkEpochKeep->dAMB_NWf[svid-1] = 0.0;
		SetBitWord256(&(pRtkEpochKeep->bAMB_NWf_Valid),svid-1);
	}

	//calc other sv
	for(i=0; i<pRtkEpoch->nCommonSat; i++)
	{
		svid = pRtkEpoch->CommonSat[i].SatID;
		sysidx = GetNavSysIdx(svid);
		if((pRtkEpoch->KeySatIdIndex[sysidx]>=0)&&(svid == pRtkEpoch->KeySatId[sysidx]))	//key svid
			continue;
		
		rou_Base = RTK_GetSatDist(svid, pRtkEpoch->Prm_Rove.obs[i].svpos, Basefixposition.ecefpos, NULL, NULL, NULL);
		rou_Rove = RTK_GetSatDist(svid, pRtkEpoch->Prm_Rove.obs[i].svpos, rovepos, NULL, NULL, NULL);

		deltaPhi =  (rou_Rove - rou_Base) - keySvSD[sysidx];

		//L1_INT
		if((pRtkEpoch->nAmbIndex>0))
		{
			//NS
			if(GetBitWord256(&(pRtkEpoch->L1UsedSVMap), svid-1)==1)
			{
				pRtkEpochKeep->dAMB_NS_Fix[svid-1] = pRtkEpoch->dAMB_Fix[ambidx];	//single int N
				SetBitWord256(&(pRtkEpochKeep->bAMB_NS_Fix_Valid),svid-1);
			}
			else
			{
				if(CalNewSvN(pRtkEpoch->Phase_DD_NS[ambidx], deltaPhi, pRtkEpoch->wavelen_NS[ambidx], 0.2, &(pRtkEpochKeep->dAMB_NS_Fix[svid-1]), NULL))
					SetBitWord256(&(pRtkEpochKeep->bAMB_NS_Fix_Valid),svid-1);	//single int N
			}

			//NW  
			if(GetBitWord256(&(pRtkEpochKeep->bAMB_NS_Fix_Valid),svid-1)==1)  //单频通过后，才可估计宽巷
			{
				if(CalNewSvN(pRtkEpoch->Phase_DD_NW[ambidx], deltaPhi, pRtkEpoch->wavelen_NW[ambidx], 0.15, &(pRtkEpochKeep->dAMB_NW_Fix[svid-1]), &(pRtkEpochKeep->dAMB_NWf[svid-1])))
					SetBitWord256(&(pRtkEpochKeep->bAMB_NW_Fix_Valid),svid-1);
				else //宽巷不通过，则将单频取消
					ClearBitWord256(&(pRtkEpochKeep->bAMB_NS_Fix_Valid),svid-1);
					
				SetBitWord256(&(pRtkEpochKeep->bAMB_NWf_Valid),svid-1);
			}
		}
		else if(pRtkEpoch->nAmbIndex_wide>0)	//WIDE_INT
		{
			//only calc NW
			if(GetBitWord256(&pRtkEpoch->L1UsedSVMap, svid-1)==1)
			{
				pRtkEpochKeep->dAMB_NW_Fix[svid-1] = pRtkEpoch->dAMB_NW_Fix[ambidx];	// int NW
				SetBitWord256(&(pRtkEpochKeep->bAMB_NW_Fix_Valid),svid-1);

				pRtkEpochKeep->dAMB_NWf[svid-1] = pRtkEpoch->dAMB_NW_Fix[ambidx];
				SetBitWord256(&(pRtkEpochKeep->bAMB_NWf_Valid),svid-1);
			}
			else
			{
				if(CalNewSvN(pRtkEpoch->Phase_DD_NW[ambidx], deltaPhi, pRtkEpoch->wavelen_NW[ambidx], 0.15, &(pRtkEpochKeep->dAMB_NW_Fix[svid-1]), &(pRtkEpochKeep->dAMB_NWf[svid-1])))
					SetBitWord256(&(pRtkEpochKeep->bAMB_NW_Fix_Valid),svid-1);
				else
					ClearBitWord256(&(pRtkEpochKeep->bAMB_NW_Fix_Valid),svid-1);
					
				SetBitWord256(&(pRtkEpochKeep->bAMB_NWf_Valid),svid-1);
			}
		}
		else if(pRtkEpoch->nAmbIndex_wfloat>0)
		{
			if(GetBitWord256(&pRtkEpoch->L1UsedSVMap, svid-1)==1)
			{
				pRtkEpochKeep->dAMB_NWf[svid-1] = pRtkEpochKeep->Rtk_NKeep[ambidx];
				SetBitWord256(&(pRtkEpochKeep->bAMB_NWf_Valid),svid-1);
			}
			else
			{
				CalNewSvN(pRtkEpoch->Phase_DD_NW[ambidx], deltaPhi, pRtkEpoch->wavelen_NW[ambidx], 0.2, NULL, &(pRtkEpochKeep->dAMB_NWf[svid-1]));
				SetBitWord256(&(pRtkEpochKeep->bAMB_NWf_Valid),svid-1);
			}
		}
		
		ambidx++;
	}

	return;
}

bool CalNewSvN(double measDDc, double estDDm, double wavelen, double threshold, int32* pN, double* pNf)
{
	bool bNValid = FALSE;
	int32 N=0;
	double res;

	res = measDDc - estDDm/wavelen;
	if(pNf != NULL)
		*pNf = res;

	if(pN != NULL)
	{
		N=ROUND(res);
		res = f_abs(res - N);
		if(res<threshold)	// res is not too big, cycle
		{
			*pN = N;
			bNValid = TRUE;
		}
	}

	return bNValid;
}

void CalcBaselineByPos(ECEF* pPos, ECEF* pbaseline)
{
	pbaseline->x = pPos->x - Basefixposition.ecefpos.x;
	pbaseline->y = pPos->y - Basefixposition.ecefpos.y;
	pbaseline->z = pPos->z - Basefixposition.ecefpos.z;

	return;
}

int32 CheckExtRTKRes(RTK_EPOCH* pRtkEpoch, word256* pSVValidMap, double* ddres, double threshold, bool* allmin, bool *bSecCalc)   //检查残差
{
	int32 idx=0, svid=0, countSV=0, countMin=0, countSVNew=0;
	double sumRes=0.0, meanResTemp=0.0;
	word256 tempMap;

	memset(&(tempMap),0,sizeof(tempMap));

	if (allmin!=NULL)
		*allmin = FALSE;

	for (idx=0; idx<pRtkEpoch->nCommonSatAMB; idx++)
	{
		svid = pRtkEpoch->CommonSat_Amb[idx].SatID;

		if (GetBitWord256(&(pRtkEpoch->L1UsedSVMap),svid-1)==0)
			continue;

		if(f_abs(ddres[svid-1]) > threshold)
		{
			if(bSecCalc==NULL)  //无需二次选星，则直接返回残差不过
				return RTK_RESCHK_FAILE_DDRES;
		}
		else
		{
			sumRes += f_abs(ddres[svid-1]);
			countSV++;
			
			if (f_abs(ddres[svid-1])<0.05)
				countMin++;
			if (f_abs(ddres[svid-1])<0.1)
			{
				countSVNew++;
				SetBitWord256(&tempMap,svid-1);
			}
		}
	}

	if(countSV>0)
		meanResTemp = sumRes/countSV;

	if ((meanResTemp>0.25)&&(countSV>6))	//均值大于1/4周，残差不过
		return RTK_RESCHK_FAILE_DDRES;

	if((allmin!=NULL)&&((countMin*5)>=(countSV*4)) && (meanResTemp<=0.03) && (countMin>=RTK_MIN_OBSCNT))  //残差小卫星足够多
	{
		*allmin = TRUE;						//宽巷切单频通过
	}

	if((bSecCalc!=NULL) && ((countSVNew*5)>=(countSV*4)) && (countSVNew!=countSV) && (countSVNew>=RTK_MIN_OBSCNT))
	{
		memcpy(pSVValidMap,&(tempMap),sizeof(tempMap));
		*bSecCalc = TRUE;
	}

	return RTK_RESCHK_OK_DDRES;
}

//宽巷切单频
void RtkNW2RtkNS(RTK_EPOCH* pRtkEpoch, int32 svENValidCnt, word256* pSvMapEN, double* pNW, double* pNS, double* pWavelenNW, double *pWavelenNS)
{
	int32 idx=0, svid=0;
	double temp = 0.0;

	for(idx=0;idx<pRtkEpoch->nCommonSatAMB;idx++)
	{
		svid = pRtkEpoch->CommonSat_Amb[idx].SatID;

		if(GetBitWord256(pSvMapEN,svid-1)==0)
			continue;

		temp = pRtkEpoch->Phase_DD_NS[idx] -  (pWavelenNW[idx]/pWavelenNS[idx]) * (pRtkEpoch->Phase_DD_NW[idx] - pNW[idx]);
		
		pNS[idx] = ROUND(temp);
	}

	return ;
}

//单频检查不过，切宽巷时，通过单频残差，对宽巷进行一次选星
int32 SelectNWSVByL1Residue(RTK_EPOCH* pRtkEpoch,word256* svMapEN, double*ddResL1, int32 svUsed)
{
	int32 idx=0, svid=0, svClear=0;
	word256 tempMap;
	double maxRes = 0.0;

	memcpy(&tempMap,svMapEN,sizeof(tempMap));

	for (idx=0;idx<pRtkEpoch->nCommonSatAMB;idx++)
	{
		svid = pRtkEpoch->CommonSat_Amb[idx].SatID;

		if (GetBitWord256(&tempMap,svid-1)==0)
			continue;

		if (maxRes<f_abs(ddResL1[svid-1]))
			maxRes = f_abs(ddResL1[svid-1]); 
	}

	maxRes = MAX(maxRes*0.8,0.4);

	for (idx=0;idx<pRtkEpoch->nCommonSatAMB;idx++)
	{
		svid = pRtkEpoch->CommonSat_Amb[idx].SatID;

		if (GetBitWord256(&tempMap,svid-1)==0)
			continue;

		if(f_abs(ddResL1[svid-1])>maxRes)
		{
			ClearBitWord256(&tempMap,svid-1);
			svClear++;
			continue;
		}
	}

	if((svClear>0) && (svClear*2<svUsed) && (svUsed-svClear>RTK_MIN_OBSCNT))  //剔星个数在门限内
		memcpy(svMapEN,&tempMap,sizeof(tempMap));

	return (svUsed-svClear);
}

BOOL IsAmbResolution(RTK_EPOCH* pRtkEpoch)
{
	int32 idx=0, svCountAmb=0, svCount=0;
	double sumCN0=0.0;

	for(idx=0;idx<pRtkEpoch->nCommonSatAMB;idx++)
	{
		//计算 卫星个数 CN0均值
		sumCN0 += pRtkEpoch->CommonSat_Amb[idx].cn0;
		svCountAmb++;
	}

	svCount = svCountAmb;
	for(idx=0;idx<MAX_SYSM;idx++)
	{
		if(pRtkEpoch->KeySatId[idx]>0)
			svCount++;
	}

	if((svCount>=RTK_MIN_OBSCNT) && (sumCN0/svCount>36))
		return TRUE;

	if (svCount<RTK_MIN_OBSCNT)
		return TRUE;

	return FALSE;
}

int32 GetMaxRtkSvNum(bool bLastIntValid)
{
	int32 cycle = 0;
	int32 maxSVNum = 0;

	if(bLastIntValid)
	{
		maxSVNum = pActiveCPT->SysmCptWorkConfig.MaxRtkSvCnt;
		return maxSVNum;
	}
	
	cycle = pActiveCPT->SysmCptWorkConfig.FixUpdateCycle;
	
	if(cycle >= 50)
		maxSVNum = 20;
	else if(cycle >= 20)
		maxSVNum = 18;
	else if(cycle >= 10)
		maxSVNum = 16;
	else if(cycle >= 5)
		maxSVNum = 14;
	else
		maxSVNum = 10;

	if(maxSVNum > (pActiveCPT->SysmCptWorkConfig.MaxRtkSvCnt))
		maxSVNum = pActiveCPT->SysmCptWorkConfig.MaxRtkSvCnt;

	return maxSVNum;
}


void CheckAllCommSVGEO(RTK_EPOCH* pRtkEpoch)
{
	int32 i=0;
	bool bAllGEO=TRUE;
	
	for(i=0; i<pRtkEpoch->nCommonSat; i++)
	{
		if(!SV_IsBd2Geo(pRtkEpoch->CommonSat[i].SatID))
		{
			bAllGEO=FALSE;
			break;
		}
	}

	if(bAllGEO)  //全是GEO则RTK不解算
	{
		memset(pRtkEpoch->CommonSat, 0, MAX_RTK_OBS_SVCNT*sizeof(BD2_SAT_INFO));
		pRtkEpoch->nCommonSat=0;

		memset(pRtkEpoch->CommonSat_Amb, 0, LAMBDA_N_DIM*sizeof(BD2_SAT_INFO));
		pRtkEpoch->nCommonSatAMB=0;

		memset(pRtkEpoch->KeySatIdIndex, 0, sizeof(pRtkEpoch->KeySatIdIndex));
		memset(pRtkEpoch->KeySatId, 0, sizeof(pRtkEpoch->KeySatId));
	}

	return;
}




