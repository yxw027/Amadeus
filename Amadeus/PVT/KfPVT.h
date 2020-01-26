#ifndef __HEAD_KFPVT_H__
#define __HEAD_KFPVT_H__

#include "define.h"
#include "PVT.h"

#define MAX_Z_DIM	(MAX_USE_SV_NUM * 2)	//(max_use_sv_num)*2	  pr and fd

#if (PVT_MODE == CALC_PVT_KF)

#define SERIAL_KALMAN_EN		0
#define PRE_RESIDUE_CHECK_EN	1
#define POST_RESIDUE_CHECK_EN	1

#define MAX_KFPVT_RECKON_CNT		10

#define PVTKF_X_DIM_1_BU		8
#define PVTKF_X_DIM_2_BU		9
#define PVTKF_X_DIM_3_BU		10

#if SUPPORT_GLONASS
#define PVTKF_X_DIM		PVTKF_X_DIM_3_BU
#else
#define PVTKF_X_DIM		PVTKF_X_DIM_2_BU
#endif

#if (PVTKF_X_DIM==PVTKF_X_DIM_3_BU)
#define GPS_BU_IDX		6
#define BD_BU_IDX		7
#define GLO_BU_IDX		8
#define CTU_IDX			9
#else
#define GPS_BU_IDX		6
#define BD_BU_IDX		7
#define CTU_IDX			8
#endif

#define USE_AKF_PVKF 		0
#define RESIDUAL_ERROR_WINLEN		4
#define AKF_USE_INIT_LEN			1

#define KALMAN_UPDATE_K_FIRST		0
#define KALMAN_UPDATE_P_FIRST		1
#define KALMAN_UPDATE_FLOW		(KALMAN_UPDATE_P_FIRST)

#if (!TARGET_FW)
#define OUTPUT_KF_DEBUG_FILE	1
#endif

typedef struct{
	int32 len;		//residum error data length
	double ECvk_pr;	//residum error, mean of RESIDUAL_ERROR_WINLEN
	double ECvk_fd;	//residum error, mean of RESIDUAL_ERROR_WINLEN
	double R_pr;			//Mesure error^2
	double R_fd;			//Mesure error^2
}AKF_ERROR_INFO;

typedef struct
{
	double MaxSpeed;	//max speed on plane
	double PosENError;	//position EN error
	double PosUError;	//position U error
	double ClkDeltaFreq;		//clock delta frequency error
	double ClkDeltaPhase;	//clock delta phase error
	double CodeLoopErr;	//code loop error
	double FreqLoopErr;	//frequency loop error
}KALMAN_NOISE;

#define KALMAN_NOISE_NUM	9

#ifdef __cplusplus
extern "C" {
#endif


extern byte KalmanNoiseIndex, ConKFPVTReckonCnt, KalmanCnt;
extern KALMAN_TRKCH_INFO KFChnInfo[MAXCHANNELS_PVT];

extern void ResetPVTKF(void);
extern void InitKFPVTMatrixP(double pValues);
extern void InitPVTKF(PVT_FIX_INFO* kfpvtinfo0, boolean bInitP);
extern void InitPVTKFPos(ECEF pos);
extern void KalmanFilterPVT(double cycle, PVT_TRKCH_INFO* pSVPVTInfo, PVT_FIX_INFO* kfpvtinfo);
extern void ReckonKalmanPVT(double cycle, PVT_FIX_INFO* kfpvtinfo);
extern boolean IsCurPVTKFValid(void);
extern boolean IsMustbeMove(void);
extern void ResetChAKFErrInfo(int32 trkch);

#ifdef __cplusplus
}
#endif

#endif

#endif




































