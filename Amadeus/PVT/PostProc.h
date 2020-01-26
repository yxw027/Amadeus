
#ifndef _HEAD_POSTPROC_
#define _HEAD_POSTPROC_


#include "define.h"
#include "PVT.h"


#define COMPASS_TIMEBIAS_UPDATE_VALIDCNT	(200)
#define COMPASS_TIMEBIAS_UPDATE_MAXCNT 		(1000)



#ifdef __cplusplus
extern "C" {
#endif

extern int32 TTFF;
extern int32 FixTime;
extern word32 InitCostRFcnt1,InitCostRFcnt2;
extern double TCXOOffset;
extern bool bPostproOutput;
extern int32 ConPPOutputCnt;
extern word32 gTFTModifiedCnt;

extern void TaskPostProc(void);
extern bool GetTCXOOffset(double* pTcxo);
extern void SetTCXOOffsetFlag(boolean bFlag);
extern void ResetTCXOOffset(void);
#ifndef _POSTPROC
#ifndef _SIMULATE
extern void setLongTermAltBeforeReboot(float64* pAlt, boolean flag);
#endif
#endif
extern boolean getCurSecurityAltitude(float64 *pAlt);

#if SUPPORT_GLONASS
extern boolean getGloTime2GpsTimeBias(double* pBias, double* pBiasTs);
extern boolean getGloTime2BdTimeBias(double* pBias, double* pBiasTs);
#endif

extern void setBDTime2GpsTimeBias(int32 biascnt, float64 bias, float64 biasTs);
#if SUPPORT_GLONASS
extern void setGLOTime2GpsTimeBias(int32 biascnt, float64 bias, float64 biasTs);
extern void setGLOTime2BdTimeBias(int32 biascnt, float64 bias, float64 biasTs);
#endif
extern boolean getBDTime2GpsTimeBias(double* pBias, double* pBiasTs);
extern void GetGNSSPRTimeBias(byte buflag, PR_MODIFY PRModify[3]);


#ifdef __cplusplus
}
#endif


#endif


