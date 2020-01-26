
#ifndef _HEAD_LSPVT_H_
#define _HEAD_LSPVT_H_

#include "define.h"
#include "PVT.h"
#include "rtk.h"



#ifdef __cplusplus
extern "C" {
#endif

extern int8 SysEnvSta;
extern int8 SysEnvCon;
extern boolean bMixFixMaybeErr;

extern int32 LsSVSelection(word256* pCoarseChnMap, word256* pMaxChnMap, PVT_TRKCH_INFO *pTrkchInfo);
extern void setLSPosEquItrCnt(int32 cnt);
extern boolean LSCalcRcvrPos(PVT_TRKCH_INFO* pTrkchInfo, PVT_FIX_INFO* pFixInfo);
extern boolean LSCalcRcvrVel(PVT_TRKCH_INFO* pTrkchInfo, PVT_FIX_INFO* pFixInfo);
extern int32 CalcFixDOP(byte buflag, word256* pPosChnMap, PVT_TRKCH_INFO* pSVPVTInfo, ECEF rcvrpos, FIX_DOP* pDop);
extern void CalcSVPrFdResidue(int32 svid, ECEF* pRcvrpos,  float64* pBu, ECEF* pRcvrVel, float64* pCtu, PVT_TRKCH_INFO * pCurSVPVTInfo);
extern byte CheckMixFixSVAndBuFlag(PVT_TRKCH_INFO* pTRKChInfo, word256* pChnmap);
extern int32 LSCalcRcvrPos_obs(OBSEV* pObs, ECEF* pRcvrPos);

#ifdef __cplusplus
}
#endif

#endif

