
#include "define.h"
#include <math.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "global.h"
#include "PVTRcvrInfo.h"
#include "PVT.h"
#include "timeproc.h"
#include "constdef.h"
#include "RTK.h"

static ECEF RcvrHistoricPos;
static byte RcvrHistoricPosValid = HISTORICAL_POS_INVALID;
static double RcvrHistoricPosRecTime;
static ECEF RcvrPos;
static ECEF RcvrVel;
static int8 RcvrFixFlag=FIX_NOT;
static FIX_DOP RcvrFixDop;
static word256 RcvrChnmap={{0,}};
static word32 RcvrFPointUsed=0;
static double RcvrSpeed=0.0;
static double RcvrHeading=0.0;
static BD2_Attitude RcvrAtt={0.0,};	// вкл╛
static BD2_BASELINE RcvrBaseLine={0.0,};	// base line


ECEF hispos = {-1336317.4599164,5333575.521585,3222461.85890275};

int32 getRcvrHistoricalPos(ECEF * pos, double* ts)
{	
#if 1
	if(RcvrHistoricPosValid != HISTORICAL_POS_INVALID)
	{
		if(pos != NULL)
			*pos = RcvrHistoricPos;
		if(ts != NULL )
			*ts = RcvrHistoricPosRecTime;
	}
#else
	if(RcvrHistoricPosValid != HISTORICAL_POS_INVALID)
	{
		if(pos != NULL)
			*pos = hispos;
		if(ts != NULL )
			*ts = RcvrHistoricPosRecTime;
	}
#endif
	return RcvrHistoricPosValid;
}

void setRcvrHistoricalPos(ECEF *pPos, double* pTs, byte posflag)
{
	if(posflag != HISTORICAL_POS_INVALID)
	{
		if(pPos != NULL)
			RcvrHistoricPos = *pPos;

		if(pTs != NULL)
			RcvrHistoricPosRecTime = *pTs;
	}

	RcvrHistoricPosValid = posflag;
	return;
}



//out put the fix info after postprocess
int32 getRcvrInfo(ECEF *pPos, ECEF *pVel, word256 *pChmap, FIX_DOP *pDOP)
{
	if(RcvrFixFlag != FIX_NOT)
	{
		if(pPos != NULL)
			*pPos = RcvrPos;

		if(pVel != NULL)
			*pVel = RcvrVel;

		if(pChmap != NULL)
			memcpy(pChmap, &RcvrChnmap, sizeof(RcvrChnmap));

		if(pDOP != NULL)
			memcpy(pDOP, &RcvrFixDop, sizeof(FIX_DOP));
	}

	return RcvrFixFlag;
}

int32 getRcvrSpeed(double* pSpeed, double* pHeading)
{
	if(RcvrFixFlag != FIX_NOT)
	{
		if(pSpeed != NULL)
			*pSpeed = RcvrSpeed;

		if(pHeading != NULL)
			*pHeading = RcvrHeading;
	}

	return RcvrFixFlag;
}

UINT32 RcvrSpeedTs=0;

float64 smoothHeading(float64 speed, float64 heading)
{
	float64 curheading=0.0;
	float64 lastheading = RcvrHeading;
	
	if(speed < 0.5)
		curheading = lastheading;

	return curheading;
}

int32 getRcvrBaseLine(BD2_BASELINE* pBaseline)
{
	if(RcvrFixFlag>=FIX_RTD)
	{
		if(pBaseline != NULL)
			memcpy(pBaseline, &RcvrBaseLine, sizeof(RcvrBaseLine));
	}

	return RcvrFixFlag;
}

int32 getRcvrAttInfo(BD2_Attitude* pAtt)
{
	if(RcvrFixFlag>=FIX_WIDE_INT)
	{
		if(pAtt != NULL)
			memcpy(pAtt, &RcvrAtt, sizeof(RcvrAtt));
	}

	return RcvrFixFlag;
}

void setRcvrFixInfo(ECEF *pPos, ECEF *pVel, FIX_DOP *pDOP, word256* pchnmap, int32 fixflag)
{
	float64 speed, heading;	
	double tow=0.0;
	int32 nWN = 0;

	RcvrFixFlag = fixflag;
	
	if(pchnmap != NULL)
		memcpy(&RcvrChnmap, pchnmap, sizeof(RcvrChnmap));
	
	if(fixflag != FIX_NOT)
	{
		if(pPos != NULL)
			RcvrPos = *pPos;
		//RcvrPosTs = getSysTime();

		if(pDOP != NULL)
			memcpy(&RcvrFixDop, pDOP, sizeof(FIX_DOP));
		
		if((pVel!=NULL) && ECEFVel2SpeedHeading(&RcvrPos, pVel, &speed, &heading))
		{			
			RcvrVel = *pVel;
			RcvrSpeed = speed;

			if(heading < 0)
				heading += 2*PI;	//round it into [0, 2PI]

			if(RcvrSpeedTs == 0)
				RcvrHeading = heading;
			else
				RcvrHeading = smoothHeading(speed,heading);
			
			RcvrSpeedTs = (word32)getSysOnTime();
		}
		
		if((pPos != NULL) && (fixflag >= FIX_3D) && GetCurNavSysTime(NAV_SYS_NONE,GPSTIME_SYNC_SRC_GLO_ONLY,&tow,&nWN))
		{
			tow = nWN * SECONDS_IN_WEEK  + tow; 
			setRcvrHistoricalPos(pPos, &tow,HISTORICAL_POS_LEVEL_40KM);	
		}		
	}

	return;
}

void setRcvrRTKInfo(ECEF *pPos, int32 commsvcnt,BD2_SAT_INFO* pCommsv, word32* pFrepointused, BD2_BASELINE* pBaseline, BD2_Attitude* pAtt, int32 fixflag)
{
	int32 trkch=0, svid=0, idx=0;
	word32 frqpint=0;

	RcvrFixFlag = fixflag;
	//return; //test
	
	if(pPos != NULL)
		RcvrPos = *pPos;

	memset(&RcvrChnmap, 0, sizeof(RcvrChnmap));
	if((fixflag!=FIX_NOT) && (pFrepointused != NULL) && (pCommsv!=NULL) && (commsvcnt>0))
	{
		RcvrFPointUsed = *pFrepointused;
		
		for(trkch=0; trkch<MAXCHANNELS; trkch++)
		{
			svid = glStruAcqu.Info[trkch].SvID;
			frqpint = glStruAcqu.Info[trkch].FreqPoint;

			for(idx=0; idx<commsvcnt; idx++)
			{
				if(svid==pCommsv[idx].SatID)
				{
					if((pCommsv[idx].validFrqpint & (*pFrepointused) & frqpint) !=0)
						SetBitWord256(&RcvrChnmap, trkch);
					break;
				}

			}
		}
	}
	
	//save base line info
	if(fixflag>=FIX_WIDE_FLOAT)
		memcpy(&RcvrBaseLine, pBaseline, sizeof(RcvrBaseLine));

	else
		memset(&RcvrBaseLine, 0, sizeof(RcvrBaseLine));

	if(fixflag>=FIX_WIDE_INT) //save base line info
		memcpy(&RcvrAtt, pAtt, sizeof(RcvrAtt));
	else
		memset(&RcvrAtt, 0, sizeof(RcvrAtt));
	
	return;
}















