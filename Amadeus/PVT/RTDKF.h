
#ifndef _RTD_KF__
#define _RTD_KF__

#include "define.h"
#include "typedefine.h"
#include "coordinate.h"
#include "RTK.h"

#ifdef __cplusplus
extern "C" {
#endif

#define		KF_RTD_XDIM		3

typedef struct{
	bool bRTDKFValid;

	double VectorX_BL[KF_RTD_XDIM];	//baseline
	double MatrixPhi_RTD[KF_RTD_XDIM*KF_RTD_XDIM];
	double MatrixQ_RTD[KF_RTD_XDIM*KF_RTD_XDIM];
	double MatrixP_RTD[KF_RTD_XDIM*KF_RTD_XDIM];
	double MatrixH_RTD[LAMBDA_N_DIM*KF_RTD_XDIM];
	double MatrixZ_RTD[LAMBDA_N_DIM];
	double MatrixR_RTD[LAMBDA_N_DIM*LAMBDA_N_DIM];
	double MatrixK_RTD[LAMBDA_N_DIM*LAMBDA_N_DIM];
}RTD_KF;

bool CalRTDProc(RTK_EPOCH* pRtkEpoch, double* RTKDist_DD, ECEF* pBaseline, RTD_KF* pRTDKf);
void ResetRTDKF(RTD_KF* pRTDKf);

#if ENABLE_RTD_KF
void InitRTDKFModule(void);
#endif


#ifdef __cplusplus
}
#endif



#endif

