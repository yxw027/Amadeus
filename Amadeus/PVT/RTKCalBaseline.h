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
// �ӿں�������
//---------------------------------------------------------------------

/*--------------------------------------------------------------------------------------
// ����:����˫վ��Ϣ������߳���
// ����: 
// [IN] pConfig		������Ϣ
// [OUT]pOutput		���������
// ����: �����־����ֵ��ʾ��������
--------------------------------------------------------------------------------------*/
int8 Rel_Position(OBSEV* PrmMaster, OBSEV* PrmSlave, RTK_EPOCH* pRtkEpoch, RTK_EPOCH_Keep* pRtkEpochKeep);

// ���ݹ���ģʽ���ò��ģʽ
bool SetRtkPosMode(OBSEV* PrmMaster, OBSEV* PrmSlave, RTK_EPOCH* pRtkEpoch, RTK_EPOCH_Keep* pRtkEpochKeep);

//--------------------------------�ֲ�����--����--���������ļ�����------------------------
//���ܣ�����˫����λ������ģ���ȼ������
BOOL CalBaseline_ECEF(RTK_EPOCH* pRtkEpoch, double* dPhase_DD, double* pN, double* pWavelen,ECEF* pOutputPos, word256* pUsedSVMapL1, double* pDDRes);


/*--------------------------------------------------------------------------------------
// ����:����˫վ�۲����õ����������б�
// ����: 
// [IN] pConfig		������Ϣ
// [OUT]pOutput		���������
// ����: �������Ǹ���
--------------------------------------------------------------------------------------*/
INT32 GetCommonSat(RTK_EPOCH* pRtkEpoch, RTK_EPOCH_Keep* pRtkEpochKeep);

/*--------------------------------------------------------------------------------------
// ����:���ݵ��㶨λʱ����������ȷ����׼��
// ����: 
// [IN] pConfig		������Ϣ
// [OUT]pOutput		���������
// ����: 
--------------------------------------------------------------------------------------*/
CHAR GetKeySatNo();


//����:����ģ���Ⱥͷ������
BOOL AdjustCalSatAmb(RTK_EPOCH* pRtkEpoch, RTK_EPOCH_Keep* pRtkEpochKeep, double* NKeep,double* PKeep,double* Nk);


void SaveLastCommonSatInfo(bool bNeed2Keep, RTK_EPOCH* pRtkEpoch, RTK_EPOCH_Keep* pRtkEpochKeep);


//����:����������޸�
BOOL PhaseCycleSlipCheck(RTK_EPOCH* pRtkEpoch, RTK_EPOCH_Keep* pRtkEpochKeep);


//��ȡ�ǵؾ��뼰����ʸ��
double RTK_GetSatDist(int32 svid, ECEF satpos, ECEF rcvpos, double* dDirectVecX, double* dDirectVecY, double* dDirectVecZ);

//����˫������
// nObsType = 	1��L1α��˫��    
//				2��L2α��˫��    
//				3��L1�ز�˫��    
//				4��L2�ز�˫��    
//				5:	 ˫Ƶ�ز�˫��
// DDRESULT_SAT_ID = 1:�����Ǻ�	2:
INT32 RTK_GetObsDD(RTK_EPOCH* pRtkEpoch, double* dObsDD, word32 frqpoint,int nObsType, int fWideOrNarrow, int nDDResultType);

//��ȡ�۲�������
// nRcvType   = 	1:Base    
//				2:Rove
//nObsType    =	1��L1α��    
//				2��L2α��    
//				3��L1�ز�    
//				4��L2�ز�    
//				5��L1-L2�ز�

double RTK_GetObsData(RTK_EPOCH* pRtkEpoch, int nRcvType, int svid, int nSatIDIndex, word32 frqpoint, int prOrPh, int fWideOrNarrow);

//���㵥������
INT32 RTK_GetObsDS(RTK_EPOCH* pRtkEpoch, double* dObsDS, word32 frqpoint, int nObsType, int wideOrNarrow,int nDDResultType);



// ��ȡ����
double RTK_GetLambda(int32 svid, word32 frqpoint, int wideOrNarrow);



// ��λ��������汱 ���걱 �ű� 
void Heading_TCM_Output(RTK_OUTPUT CALOUTPUTBuf);


//ƽ���㷨
BOOL SmoothData(double* dCurData, double* dSmthData, int nCur);

void SmmothD2S_Float(RTK_EPOCH* pRtkEpoch, RTK_EPOCH_Keep* pRtkEpochKeep, double* dSF_Float);

void Kalman_Float_AMB(DOUBLE Zk, DOUBLE* X0, DOUBLE* P0);

//����:���ݻ���ʸ��ȷ����̬
BOOL Attitude_Determination_UseBSL(BD2_BASELINE* bsl, BD2_Attitude* att);

//���ܣ���ʼ����������
BOOL InitAttDetPara(RTK_EPOCH* pRtkEpoch, RTK_EPOCH_Keep* pRtkEpochKeep);

//���ܣ�˫��۲�������
BOOL GetDD_OBV();
BOOL CalRTK_Error();


//���ܣ�����ģ���ȸ���⿨�����˲�����
BOOL FloatSolution_Kalman(RTK_EPOCH* pRtkEpoch, RTK_EPOCH_Keep* pRtkEpochKeep,double* dVecN, double* Nk, double* Pk);

//���ܣ�����ģ�������������
BOOL Ambiguity_Resolution(RTK_EPOCH* pRtkEpoch, RTK_EPOCH_Keep* pRtkEpochKeep,double* Nk, double* Pk, double* dRatio, double* dVecN, double* dVecNFloat);

//���ܣ����ߡ���̬���㼰У��
int8 AttCal_UseAMB(RTK_EPOCH* pRtkEpoch, RTK_EPOCH_Keep* pRtkEpochKeep, double* dAMB, ECEF* pPos, ECEF* baseline, int32 obsFlag, int32 dAMBFlag);

// ���ܣ�����У�飨ChkModeȷ��У��ģʽ��
int CheckRTKResultL1(RTK_EPOCH* pRtkEpoch, RTK_EPOCH_Keep* pRtkEpochKeep, double* ddres, word256 UsedSVMapL1);

//���ܣ����߳���У��
BOOL Att_Check_BSL(RTK_EPOCH* pRtkEpoch);


// ��������2����
double GetNorm2(double* Vec, int nDim, CHAR bSqrt);

extern bool RTKCoaseSVSelect(OBST* prmObv, word32* pValidFrqpiont, double* pSnr, double* pLLI);

#ifdef __cplusplus
}
#endif


#endif	// RTKCALBASELINE_H__


