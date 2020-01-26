#ifndef _HEAD_COORDINATE_H_
#define _HEAD_COORDINATE_H_

#include "typedefine.h"

// the pose translate of different coordinate (7 parameters struct)
typedef struct
{
	double dx;
	double dy;
	double dz;
	double dEps;
	double dPhi;
	double dOmg;
	double dS;

}CordTrans7Para;

typedef struct
{
	double x;
	double y;
	double z;
}ECEF;



typedef struct{
	double lon;	//unit: radian
	double lat;	//unit: radian
	double alt;	//unit: m
}WGS;


typedef struct{
	double north;
	double east;
	double head;
}NEH;

#ifdef __cplusplus
extern "C" {
#endif


//extern double atan2(double y,double x);
extern int32 WGS2ECEF(WGS *src, ECEF *des);
extern int32 ECEF2WGS(ECEF *src, WGS *des);
extern int32 ECEF2NEH(ECEF *src, ECEF *ref, NEH *des);
extern boolean ECEFVel2SpeedHeading(ECEF* pPos, ECEF* pVel, double* pSpeed, double* pHeading);
extern boolean ECEFVel2ENU(ECEF* pPos, ECEF* pVel, NEH* pNEHVel);
extern double Dis2PointsOnEarth(ECEF *pPos1, ECEF *pPos2);
extern double Dis2PointsOnEarthWGS(WGS *pWGS1, WGS *pWGS2);

extern double sinx(double x);
extern double cosx(double x);


extern void CordTrans_Using7Para(double * x, double * y,double * z, CordTrans7Para* Para);

extern void ClearBitWord64(word64* wd, byte i);
extern void SetBitWord64(word64* wd, byte i);
extern byte GetBitWord64(word64 wd, byte i);
extern boolean IsZeroInWord64(word64 *wd);
extern void ResetWord64(word64 *wd);
extern word64 OpAndWord64(word64 *wd1,word64 *wd2);
extern word64 OpOrWord64(word64 *wd1,word64 *wd2);
extern void ClearBitWord256(word256* wd, byte i);
extern void SetBitWord256(word256* wd, byte i);
extern byte GetBitWord256(word256* wd, byte i);
extern boolean IsZeroInWord256(word256 *wd);
extern void ResetWord256(word256 *wd);
extern word256 OpAndWord256(word256 *wd1,word256 *wd2);
extern word256 OpOrWord256(word256 *wd1,word256 *wd2);
extern word256 OpReverseWord256(word256 *wd);
extern int8 CalcBitcntWord256(word256* wd, word32 bitvalid);
extern int8 FindBitWord16(word16 wd, word16 bitvalue);
extern int8 CalcBitcntWord32(word32 wd, word32 bitvalid);

extern void F64ToH32L32(float64 clk, word32 *high, word32 *low);
extern void H32L32ToF64(float64 *ret, word32 high, word32 low);

extern double SGN(double x);
extern double ROUND(double x);
extern unsigned int ROUND_U(double x);
extern void SWAP(double* x,double* y);


#ifdef __cplusplus
}
#endif

#endif


