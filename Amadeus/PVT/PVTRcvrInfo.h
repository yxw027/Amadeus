#ifndef _HEAD_PVTRCVRINFO_H_
#define _HEAD_PVTRCVRINFO_H_


#include "define.h"
#include "coordinate.h"
#include "PVT.h"
#include "RTK.h"
#define RCVR_POS_SECURITY_TIME	(2 * SECONDS_IN_HOUR)

#define RCVR_POS_CHECK_TIME		(10 * SECONDS_IN_MINUTE)



#define HISTORICAL_POS_LEVEL_40KM		1
#define HISTORICAL_POS_LEVEL_2000KM		3
#define HISTORICAL_POS_INVALID			4

#define TIMEBIAS_UPDATE_VALIDCNT	(200)
#define TIMEBIAS_UPDATE_MAXCNT 		(1000)

#ifdef __cplusplus
extern "C" {
#endif

extern int32 getRcvrHistoricalPos(ECEF * pos, double* ts);
extern void setRcvrHistoricalPos(ECEF *pPos, double* pTs, byte posflag);

extern int32 getRcvrInfo(ECEF *pPos, ECEF *pVel, word256 *pChmap, FIX_DOP *pDOP);
extern int32 getRcvrSpeed(double* pSpeed, double* pHeading);
extern int32 getRcvrAttInfo(BD2_Attitude* pAtt);			//²â×Ë ²âÏò
extern int32 getRcvrBaseLine(BD2_BASELINE* pBaseline);	//base line

extern void setRcvrFixInfo(ECEF *pPos, ECEF *pVel, FIX_DOP *pDOP, word256* pchnmap, int32 fixflag);
extern void setRcvrRTKInfo(ECEF *pPos, int32 commsvcnt,BD2_SAT_INFO* pCommsv, word32* pFrepointused, BD2_BASELINE* pBaseline, BD2_Attitude* pAtt, int32 fixflag);


#ifdef __cplusplus
}
#endif

#endif

