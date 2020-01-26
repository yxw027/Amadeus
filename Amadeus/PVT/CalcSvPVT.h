#ifndef _CALCSVPVT_H
#define _CALCSVPVT_H


#include "define.h"
#include "constdef.h"
#include "coordinate.h"
#include "FrameDecode.h"
#include "TimeProc.h"
#include "PVT.h"

#if SUPPORT_GLONASS
#define RE_GLO   6378137.0 
#define MU_GLO   3.986004418E14  
#define J2_GLO   1.0826257E-3     
#define OMGE_GLO 7.292115E-5  
#define MJD1996 50083
#endif

#define SQR(x)   ((x)*(x))

#if SUPPORT_GLONASS
typedef struct
{
	double k1;
	double k2;
	double k3;
	double k4;
	double k5;
	double k6;
}RKSTRUCT;
#endif


#ifdef __cplusplus
extern "C" {
#endif

extern void CalcSV_PVT_Eph(int32 svid, double ts,  ECEF* p_svpos, ECEF* p_svvel, double* p_svclkbias, double* p_svfreqbias,word32* toe);
extern void CalcSV_ClkErr_Eph(float64 time, float64 svirel,int32 svid, float64 *pClkErr, float64 *pFrqErr );
extern void CalcSV_PVT_Alm(int32 truesvid, int32 wn, float64 time, ECEF *pPos, ECEF *pVel);

extern byte GetSvIonoInfo(int32 trkch, double ts, double* pIono);
extern void GetSvTropoInfo(int32 trkch, double* pTropo);
extern void CalcSVAngle(ECEF SVPos, ECEF rcvrPos, float64 *pAz, float64 *pEl);
extern int32 CheckEphHealth(int32 SvID);
extern int32 CheckAlmHealth(int32 SvID);
extern double CalcCrossVel(int32 svid, ECEF SVPos, ECEF SV_vel, ECEF rcvrPos, ECEF rcvrVel);

extern int32 CalcSVEPHAge(int32 sv_id, int32 wn, int32 sec_of_week);
extern int32 CalcSVALMAge(int32 sv_id, int32 wn, int32 sec_of_week);
#if SUPPORT_GLONASS
extern void GloToGpsTime(int32 N4,int32 NT,double t, int32 *weekn, double *tow);
extern void GpsToGloTime(int32 weekn,double tow,int32 *Day,double *t);
extern void GpsToGloTimeTod(double tow, double *tod);
extern bool GloFreidToSvid(int32 FreId, int32 *Svid, int32 *n);
extern bool CalcGloSVToGpsTime(int32 slotid, double sec_day, int32 *weekn, double *tow);
extern int32 ConvertGloSVID2SlotID(int32 svid);
extern int32 ConvertGloSlotID2SVID(int32 slotid);
#endif

#ifdef __cplusplus
}
#endif


#endif

