
#ifndef _HEAD_UPDATEINFO_H_
#define _HEAD_UPDATEINFO_H_

#include "define.h"


#define TH_DEVIATION   10

typedef struct
{
	double data[TH_DEVIATION][3];
	int16 index;
	int16 N;
	double dev[3];
}BESTPOSDEVIATIONDATA;

typedef struct
{
	bool bPosSaveOpen;
	float time;		//s, smooth time
	double std_horiz;	//horizontal std, m
	double std_verti;	//vertical std, m
}POS_SAVE_PARAM;


#ifdef __cplusplus
extern "C" {
#endif
extern BESTPOSDEVIATIONDATA bestposWGS;
extern word32 ACQDisMatchCnt[3];
extern void TaskUpdateInfo(void);
extern void InitUpdateInfo(void);
extern int32 GetSVSWFreq(int32 svid, int32 frepoint, double* pSWFreq);
extern void ProcessUncon(void);
extern void UpdateSV(void);
extern void UpdateACQMan(void);
extern void CalcSVAge(int32 wn, double tow);
//extern void getPosDeviation(double*lat_deviation_m,double*lon_deviation_m,double*alt_deviation_m);
extern void SetPosSaveParam(bool bopen, float* pSmoothtime, double* pStd_horiz, double* pStd_verti);
#ifdef __cplusplus
}
#endif

#endif






