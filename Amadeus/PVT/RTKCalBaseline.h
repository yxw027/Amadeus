/*******************************************************************
Company:   hwacreate
Engineer:  
Create Date: 2015.12.01
File  Name:  RTKCalBaseline.h
Description: header file of RTKCalBaseline.c
Function List: 
version: V1.0
Revision Date:
Modifier: 
Additional Comments: 
********************************************************************/
#ifndef  RTKCALBASELINE_H__
#define  RTKCALBASELINE_H__

#include "RTKLAMBDA.h"
#include "RTKConstants.h"
#include "RTKTools.h"


#define OBS_RANGE	0
#define OBS_PHASE	1

#define OBS_SWIDE	0	//super wide lane
#define OBS_WIDE	1
#define OBS_NARROW	2
#define OBS_SINGLE	3

#define AMB_FLOAT	0
#define AMB_INT		1


#define RTK_RESCHK_OK_RATIO				1
#define RTK_RESCHK_OK_DDRES				2
#define RTK_RESCHK_OK_N					3
#define RTK_RESCHK_OK_STD				4
#define RTK_RESCHK_OK_RATIO_CHANGE		5
#define RTK_RESCHK_OK_NW				6
#define RTK_RESCHK_FAILE_NODDRES	(-1)
#define RTK_RESCHK_FAILE_DDRES		(-2)
#define RTK_RESCHK_FAILE_N			(-3)
#define RTK_RESCHK_FAILE_NORMAL		(-4)




#ifdef __cplusplus
extern "C" {
#endif
//---------------------------------------------------------------------
// 接口函数声明
//---------------------------------------------------------------------

/*--------------------------------------------------------------------------------------
// 功能:根据双站信息计算基线长度
// 参数: 
// [IN] pConfig		配置信息
// [OUT]pOutput		计算结果输出
// 返回: 错误标志，负值表示发生错误
--------------------------------------------------------------------------------------*/
int8 Rel_Position(OBSEV* PrmMaster, OBSEV* PrmSlave, RTK_EPOCH* pRtkEpoch, RTK_EPOCH_Keep* pRtkEpochKeep);

// 根据工作模式设置差分模式
bool SetRtkPosMode(OBSEV* PrmMaster, OBSEV* PrmSlave, RTK_EPOCH* pRtkEpoch, RTK_EPOCH_Keep* pRtkEpochKeep);

//--------------------------------局部函数--声明--不被其它文件调用------------------------
//功能：根据双差相位及整周模糊度计算基线
BOOL CalBaseline_ECEF(RTK_EPOCH* pRtkEpoch, double* dPhase_DD, double* pN, double* pWavelen,ECEF* pOutputPos, word256* pUsedSVMapL1, double* pDDRes);


/*--------------------------------------------------------------------------------------
// 功能:根据双站观测量得到公共卫星列表
// 参数: 
// [IN] pConfig		配置信息
// [OUT]pOutput		计算结果输出
// 返回: 公共卫星个数
--------------------------------------------------------------------------------------*/
INT32 GetCommonSat(RTK_EPOCH* pRtkEpoch, RTK_EPOCH_Keep* pRtkEpochKeep);

/*--------------------------------------------------------------------------------------
// 功能:根据单点定位时的卫星仰角确定基准星
// 参数: 
// [IN] pConfig		配置信息
// [OUT]pOutput		计算结果输出
// 返回: 
--------------------------------------------------------------------------------------*/
CHAR GetKeySatNo();


//功能:调整模糊度和方差矩阵
BOOL AdjustCalSatAmb(RTK_EPOCH* pRtkEpoch, RTK_EPOCH_Keep* pRtkEpochKeep, double* NKeep,double* PKeep,double* Nk);


void SaveLastCommonSatInfo(bool bNeed2Keep, RTK_EPOCH* pRtkEpoch, RTK_EPOCH_Keep* pRtkEpochKeep);


//功能:周跳检测与修复
BOOL PhaseCycleSlipCheck(RTK_EPOCH* pRtkEpoch, RTK_EPOCH_Keep* pRtkEpochKeep);


//获取星地距离及方向矢量
double RTK_GetSatDist(int32 svid, ECEF satpos, ECEF rcvpos, double* dDirectVecX, double* dDirectVecY, double* dDirectVecZ);

//计算双差数据
// nObsType = 	1：L1伪距双差    
//				2：L2伪距双差    
//				3：L1载波双差    
//				4：L2载波双差    
//				5:	 双频载波双差
// DDRESULT_SAT_ID = 1:按卫星号	2:
INT32 RTK_GetObsDD(RTK_EPOCH* pRtkEpoch, double* dObsDD, word32 frqpoint,int nObsType, int fWideOrNarrow, int nDDResultType);

//获取观测量数据
// nRcvType   = 	1:Base    
//				2:Rove
//nObsType    =	1：L1伪距    
//				2：L2伪距    
//				3：L1载波    
//				4：L2载波    
//				5：L1-L2载波

double RTK_GetObsData(RTK_EPOCH* pRtkEpoch, int nRcvType, int svid, int nSatIDIndex, word32 frqpoint, int prOrPh, int fWideOrNarrow);

//计算单差数据
INT32 RTK_GetObsDS(RTK_EPOCH* pRtkEpoch, double* dObsDS, word32 frqpoint, int nObsType, int wideOrNarrow,int nDDResultType);



// 获取波长
double RTK_GetLambda(int32 svid, word32 frqpoint, int wideOrNarrow);



// 方位角输出，真北 坐标北 磁北 
void Heading_TCM_Output(RTK_OUTPUT CALOUTPUTBuf);


//平滑算法
BOOL SmoothData(double* dCurData, double* dSmthData, int nCur);

void SmmothD2S_Float(RTK_EPOCH* pRtkEpoch, RTK_EPOCH_Keep* pRtkEpochKeep, double* dSF_Float);

void Kalman_Float_AMB(DOUBLE Zk, DOUBLE* X0, DOUBLE* P0);

//功能:根据基线矢量确定姿态
BOOL Attitude_Determination_UseBSL(BD2_BASELINE* bsl, BD2_Attitude* att);

//功能：初始化搜索参数
BOOL InitAttDetPara(RTK_EPOCH* pRtkEpoch, RTK_EPOCH_Keep* pRtkEpochKeep);

//功能：双差观测量处理
BOOL GetDD_OBV();
BOOL CalRTK_Error();


//功能：整周模糊度浮点解卡尔曼滤波处理
BOOL FloatSolution_Kalman(RTK_EPOCH* pRtkEpoch, RTK_EPOCH_Keep* pRtkEpochKeep,double* dVecN, double* Nk, double* Pk);

//功能：整周模糊度搜索及输出
BOOL Ambiguity_Resolution(RTK_EPOCH* pRtkEpoch, RTK_EPOCH_Keep* pRtkEpochKeep,double* Nk, double* Pk, double* dRatio, double* dVecN, double* dVecNFloat);

//功能：基线、姿态计算及校验
int8 AttCal_UseAMB(RTK_EPOCH* pRtkEpoch, RTK_EPOCH_Keep* pRtkEpochKeep, double* dAMB, ECEF* pPos, ECEF* baseline, int32 obsFlag, int32 dAMBFlag);

// 功能：基线校验（ChkMode确定校验模式）
int CheckRTKResultL1(RTK_EPOCH* pRtkEpoch, RTK_EPOCH_Keep* pRtkEpochKeep, double* ddres, word256 UsedSVMapL1);

//功能：基线长度校验
BOOL Att_Check_BSL(RTK_EPOCH* pRtkEpoch);


// 求向量的2范数
double GetNorm2(double* Vec, int nDim, CHAR bSqrt);

extern bool RTKCoaseSVSelect(OBST* prmObv, word32* pValidFrqpiont, double* pSnr, double* pLLI);

#ifdef __cplusplus
}
#endif


#endif	// RTKCALBASELINE_H__


