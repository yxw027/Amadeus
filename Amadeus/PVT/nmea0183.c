


#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>
#include "constdef.h"
#include "assitFunc4Bin.h"

#ifndef _POSTPROC
#ifndef _SIMULATE
#include "UartFunc.h"
#else
#include "dataprocess.h"
#include "simulatorGlobal.h"
#endif
#endif

#include "hcp.h"
#include "cfgpara.h"
#include "nmea0183.h"
#include "pvt.h"
#include "timeproc.h"
#include "PVTRcvrInfo.h"
//#include "UpdateInfo.h"
#include "nmeaext.h"
#include "postproc.h"
#include "global.h"
#include "rtk.h"
#include "RTKCalBaseline.h"
#include "CalcSvPVT.h"

#ifdef	USE_KALMAN
#define	USE_KALMAN_POS_GGA	(1)
#define	USE_KALMAN_POS_RMC	(1)
#endif

//static bool s_bAllowDebugMsgVelDir = FALSE;

//static NavStateStruct *s_pNavResult = NULL;
static NMEA_DATA NMEAData;

#define LEN_NAV_TYPE_0183	(3)
static U1 s_strNavType[LEN_NAV_TYPE_0183+1] = {0};
static I1 s_i1NavStateGGA = 1;
static I1 s_i1LatNS = 'N';
static U4 s_uLatDeg = 0;		// ����γ�ȶ���
static U4 s_uLatMinWhole = 0;	// γ�ȷֵ���������
static U4 s_uLatMinFrac = 0;	// γ�ȷֵ�С������

static I1 s_i1NavStateRMC = 'A';
static I1 s_i1LonEW = 'E';
static U4 s_uLonDeg = 0;		// ���ľ��ȶ���
static U4 s_uLonMinWhole = 0;	// ���ȷֵ���������
static U4 s_uLonMinFrac = 0;	// ���ȷֵ�С������

static U1 s_strLat[32] = {0};
static U1 s_strLon[32] = {0};

static U1 s_strTime[32] = {0};
static U1 s_strDate[32] = {0};



//static void Smooth_Speed_Course(double *pf8VelDir);
static void UpdateNMEAData(void);
static float64 calcRcvrGeiodUndulation(float64 lat, float64 lon);


UINT8 CalcCheckSum(const UINT8 *pBuff, UINT32 uLen)
{
	UINT8 u1CheckSum = 0;
	UINT32 i = 0;

	for (i=0; i<uLen; i++)
	{
		u1CheckSum ^= pBuff[i];
	}
	return u1CheckSum;
}

void NMEA0183_Main(byte comid)
{
    F8 fValue = 0.0;
	UTC_TIME dateTime;
	UINT8 navmode = GetCurNavMode();
	static int PeriodNum[MAX_UART_NUM] = {0,0,0};
	UpdateNMEAData();
	if((bFixAtSecBoundary == TRUE)&& (IsSecBoundaryChange()))
		PeriodNum[comid] = 0;
// ��������
	s_strNavType[0] = '$';
	if (navmode == NAVMODE_GPSONLY)
	{
		s_strNavType[1] = 'G';
		s_strNavType[2] = 'P';
	}
	else if (navmode == NAVMODE_BDONLY)
    {
    	s_strNavType[1] = 'B';
		s_strNavType[2] = 'D';
	}
#if SUPPORT_GLONASS
	else if (navmode == NAVMODE_GLOONLY)
    {
    	s_strNavType[1] = 'G';
		s_strNavType[2] = 'L';
	}
#endif
	else
    {
    	s_strNavType[1] = 'G';
		s_strNavType[2] = 'N';
	}
	s_strNavType[3] = 0;

	// ��λ״̬	
	if(NMEAData.fixstatus != FIX_NOT)
	{
		s_i1NavStateGGA = 1;
		s_i1NavStateRMC = 'A';
	}
	else
	{
		s_i1NavStateGGA = 0;
		s_i1NavStateRMC = 'V';
	}

	// ��γ����
	s_i1LatNS = (NMEAData.lat<0.0) ? 'S' : 'N';
	s_i1LonEW = (NMEAData.lon<0.0) ? 'W' : 'E';

	// γ��
	fValue = NMEAData.lat * R2D; 	// �öȱ�ʾ
/*
	UART_Printf(UART_PORT_DSPUart0, "LAT %.5lf %.5lf "
		, s_pNavResult->lat
		, fValue
	);
*/	
	if (fValue <= -1.0E-9)
	{
		fValue = -fValue;
	}
	else if (fValue < 1.0E-9)
	{
		fValue = 0.0;
	}
	s_uLatDeg = (U4)fValue;    			// ���Ķ���
/*
	UART_Printf(UART_PORT_DSPUart0, "%.5lf %u "
		, fValue
		, s_uLatDeg
	);
*/	
	fValue = (fValue-s_uLatDeg) * 60.0;
	s_uLatMinWhole = (U4)fValue;		// ���ķ���
/*
	UART_Printf(UART_PORT_DSPUart0, "%.5lf %u "
		, fValue
		, s_uLatMinWhole
	);
*/	
	fValue = (fValue-s_uLatMinWhole) * 10000000.0;
	s_uLatMinFrac = (U4)fValue;
/*
	UART_Printf(UART_PORT_DSPUart0, "%.5lf %u "
		, fValue
		, s_uLatMinFrac
	);
*/	
	sprintf((I1 *)s_strLat, "%u%02u.%07u,", s_uLatDeg, s_uLatMinWhole, s_uLatMinFrac);

	// ����
	fValue = NMEAData.lon * R2D; 		// �öȱ�ʾ
	if (fValue <= -1.0E-9)
	{
		fValue = -fValue;
	}
	else if (fValue < 1.0E-9)
	{
		fValue = 0.0;
	}
	s_uLonDeg = (U4)fValue;    			// ���Ķ���
/*
	UART_Printf(UART_PORT_DSPUart0, "LON %.5lf %d "
		, fValue
		, s_uLonDeg
	);
*/	
	fValue = (fValue-s_uLonDeg) * 60.0;
	s_uLonMinWhole = (U4)fValue;		//���ķ���
	fValue = (fValue-s_uLonMinWhole) * 10000000.0;
	s_uLonMinFrac = (U4)fValue;
	sprintf((I1 *)s_strLon, "%u%02u.%07u,", s_uLonDeg, s_uLonMinWhole, s_uLonMinFrac);
	
	// UTC
	GetTimeOfFixUTC(&dateTime, 0.005);	// ����С�����2λ��Ч����

 	
	sprintf((I1 *)s_strTime, "%02u%02u%02u.%02u,", dateTime.hour, dateTime.min, dateTime.sec, dateTime.ms/10);
	sprintf((I1 *)s_strDate, "%02u%02u%02u,", dateTime.day, dateTime.mon, dateTime.year%100);

	// ����0183���
	if ((pActiveCPT->SysmCptProtocol[comid].GGA > 0)&& (PeriodNum[comid]%pActiveCPT->SysmCptProtocol[comid].GGA == 0))	// ����GGA ��λ��ʱ�䡢λ������صĶ�λ����
	{ 
		Uart_Send_GGA(comid);
	}

	if ((pActiveCPT->SysmCptProtocol[comid].RMC > 0)&& (PeriodNum[comid]%pActiveCPT->SysmCptProtocol[comid].RMC == 0))	// RMC
	{ 
		Uart_Send_RMC(comid);
	}

	if ((pActiveCPT->SysmCptProtocol[comid].GLL > 0)&& (PeriodNum[comid]%pActiveCPT->SysmCptProtocol[comid].GLL == 0))
	{
		Uart_Send_GLL(comid);
	}

	if ((pActiveCPT->SysmCptProtocol[comid].GSA > 0)&& (PeriodNum[comid]%pActiveCPT->SysmCptProtocol[comid].GSA == 0))
	{
		if (navmode & NAV_SYS_GPS)
		{
			Uart_Send_GSA(NAV_SYS_GPS,comid);
		}
		if (navmode & NAV_SYS_BD)
		{
			Uart_Send_GSA(NAV_SYS_BD,comid);
		}
#if SUPPORT_GLONASS
		if (navmode & NAV_SYS_GLO)
		{
			Uart_Send_GSA(NAV_SYS_GLO,comid);
		}
#endif
	}

	if ((pActiveCPT->SysmCptProtocol[comid].DHV > 0)&& (PeriodNum[comid]%pActiveCPT->SysmCptProtocol[comid].DHV == 0))
	{
		if (1 == s_i1NavStateGGA)
		{
			Uart_Send_DHV(comid);
		}
		Uart_Send_NEMA0183_NPR(comid);
	}
	
	if ((pActiveCPT->SysmCptProtocol[comid].GSV > 0)&& (PeriodNum[comid]%pActiveCPT->SysmCptProtocol[comid].GSV == 0))
	{
		if (navmode & NAV_SYS_GPS)
		{
			Uart_Send_GSV(NAV_SYS_GPS,comid);
		}
		if (navmode & NAV_SYS_BD)
		{
			Uart_Send_GSV(NAV_SYS_BD,comid);
		}
#if SUPPORT_GLONASS
		if (navmode & NAV_SYS_GLO)
		{
			Uart_Send_GSV(NAV_SYS_GLO,comid);
		}
#endif
	}
	PeriodNum[comid]++;
}

void UpdateNMEAData(void)
{
	ECEF pos, vel;
	WGS wgspos;
	NEH nehvel;
#if SUPPORT_GLONASS
	FIX_DOP dop={99.0,99.0,99.0,99.0,99.0,99.0,99.0};
#else
	FIX_DOP dop={99.0,99.0,99.0,99.0,99.0,99.0};
#endif

	int32 trkch=0, svid=-1;
	word256 fixchmap={{0,}};
	
	memset(&NMEAData, 0, sizeof(NMEAData));

	NMEAData.fixstatus=getRcvrInfo(&pos, &vel, &fixchmap, &dop);
	
	if(NMEAData.fixstatus != FIX_NOT)
	{
		ECEF2WGS(&pos,&wgspos);

#if 0
		NMEAData.geoid = calcRcvrGeiodUndulation(wgspos.lat, wgspos.lon);
#else
		NMEAData.geoid = 0.0;
#endif

		NMEAData.lon = wgspos.lon;
		NMEAData.lat = wgspos.lat;
		NMEAData.alt = wgspos.alt - NMEAData.geoid;

		getRcvrSpeed(&(NMEAData.speed), &(NMEAData.head));

		if(NMEAData.head < 0)
			NMEAData.head = 360.0 + NMEAData.head * R2D;
		else
			NMEAData.head = NMEAData.head * R2D;

		ECEF2NEH(&vel ,&pos, &nehvel);
		NMEAData.nvel = nehvel.north;
		NMEAData.evel = nehvel.east;

		NMEAData.vel.x = vel.x;
		NMEAData.vel.y = vel.y;
		NMEAData.vel.z = vel.z;	
	}

	NMEAData.pdop = dop.pdop;
	NMEAData.hdop = dop.hdop;
	NMEAData.vdop = dop.vdop;
	NMEAData.gpstdop = dop.gpstdop;
	NMEAData.bdtdop = dop.bdtdop;

	for(trkch=0; trkch<MAXCHANNELS; trkch++)
	{
		if(GetBitWord256(&fixchmap,trkch)==1)
		{
			svid = GetTRKChSvid(trkch);
#if SUPPORT_GLONASS
			if(SV_IsGlo(svid))
				svid = ConvertGloSlotID2SVID(svid);
#endif
			if(svid<=0)
				continue;
			
			SetBitWord256(&(NMEAData.fixsvmap),svid-1);
		}
	}

	NMEAData.svnuminfix = CalcBitcntWord256(&(NMEAData.fixsvmap), 1);
	
	return;
}

void Uart_Send_GGA(byte comid)
{
	I1 strBuff[20];
	UINT8 u1CheckSum = 0;
	T_OUT_BUFF *pBuffOut = &g_OutBuff;
	int32 fixstateOut=0;
	
	BUFF_Clear(pBuffOut);
	// 0 ���ͱ�ʶ
	BUFF_AppendLittle(pBuffOut, s_strNavType, LEN_NAV_TYPE_0183);
	sprintf(strBuff, "%s", "GGA,");
	BUFF_AppendLittle(pBuffOut, (U1 *)strBuff, strlen((I1 *)strBuff));

    // 1��λʱ��
	BUFF_AppendLittle(pBuffOut, (U1 *)s_strTime, strlen((I1 *)s_strTime));

	// 2γ��
	BUFF_AppendLittle(pBuffOut, (U1 *)s_strLat, strlen((I1 *)s_strLat));
	
	// 3γ�ȷ���
	sprintf(strBuff, "%c,", s_i1LatNS);
	BUFF_AppendLittle(pBuffOut, (U1 *)strBuff, strlen((I1 *)strBuff));
	
	// 4����
	BUFF_AppendLittle(pBuffOut, (U1 *)s_strLon, strlen((I1 *)s_strLon));

    // 5���ȷ���
    sprintf(strBuff, "%c,", s_i1LonEW);
	BUFF_AppendLittle(pBuffOut, (U1 *)strBuff, strlen((I1 *)strBuff));
	
    // 6״ָ̬ʾ 0--δ��λ 1--�޲�ֶ�λ 2--��ֶ�λ 3--˫Ƶ��λ
    //0:fix invalid; 1:fix valid(SPS); 2:RTD(SPS); 3:PPS mode ; 4:RTK_INT; 5 RTK_float; 6:Estimated mode;
    //7:Manual Input mode; 8:Simulator Mode
    if(NMEAData.fixstatus==FIX_NOT)
    	fixstateOut = 0;
	else if((NMEAData.fixstatus == FIX_2D) || (NMEAData.fixstatus == FIX_3D))
		fixstateOut = 1;
	else if(NMEAData.fixstatus == FIX_RTD)
		fixstateOut = 2;
	else if((NMEAData.fixstatus == FIX_L1_INT) ||(NMEAData.fixstatus == FIX_NARROW_INT)||(NMEAData.fixstatus == FIX_WIDE_INT))
		fixstateOut = 4;
	else if((NMEAData.fixstatus == FIX_L1_FLOAT) ||(NMEAData.fixstatus == FIX_NARROW_FLOAT)||(NMEAData.fixstatus == FIX_WIDE_FLOAT))
		fixstateOut = 5;
	else
		fixstateOut = 7;
	
    sprintf(strBuff, "%d,", fixstateOut);
	BUFF_AppendLittle(pBuffOut, (U1 *)strBuff, strlen((I1 *)strBuff));

    // 7���붨λ��������
	sprintf(strBuff, "%02u,", NMEAData.svnuminfix);
	BUFF_AppendLittle(pBuffOut, (U1 *)strBuff, strlen((I1 *)strBuff));
	
    // 8HDOPֵ
    sprintf (strBuff, "%.1lf,", NMEAData.hdop);
	BUFF_AppendLittle(pBuffOut, (U1 *)strBuff, strlen((I1 *)strBuff));
	
    // 9����ƽ�����θ�
   	sprintf (strBuff, "%.6lf,", NMEAData.alt);
	BUFF_AppendLittle(pBuffOut, (U1 *)strBuff, strlen((I1 *)strBuff));
	
    // 10���ߺ��θߵ�λ
    sprintf(strBuff, "%c,", 'M');
	BUFF_AppendLittle(pBuffOut, (U1 *)strBuff, strlen((I1 *)strBuff));
	
    // 11CGS-2000�߳��쳣
    sprintf(strBuff, "%.1f,", NMEAData.geoid);
	BUFF_AppendLittle(pBuffOut, (U1 *)strBuff, strlen((I1 *)strBuff));
	
    // 12�߳��쳣��λ
    sprintf(strBuff, "%c,", 'M');
	BUFF_AppendLittle(pBuffOut, (U1 *)strBuff, strlen((I1 *)strBuff));
	
    // 13������ݵ�����
    sprintf(strBuff, "%c", ',');
	BUFF_AppendLittle(pBuffOut, (U1 *)strBuff, strlen((I1 *)strBuff));
	
    // 14���վ̨ID��
    sprintf(strBuff, "%c", ',');
	BUFF_AppendLittle(pBuffOut, (U1 *)strBuff, strlen((I1 *)strBuff));
	
    // 15VDOPֵ
    sprintf(strBuff, "%.1lf", NMEAData.vdop);
	BUFF_AppendLittle(pBuffOut, (U1 *)strBuff, strlen((I1 *)strBuff));
	
	// 16У���
	u1CheckSum = CalcCheckSum(&pBuffOut->buff[1], pBuffOut->curIdx-1);
	sprintf(strBuff, "*%02X", u1CheckSum);
	BUFF_AppendLittle(pBuffOut, (U1 *)strBuff, strlen((I1 *)strBuff));

	// 17��ֹ�� CR/LF
	strBuff[0] = 0x0D;
	strBuff[1] = 0x0A;
	BUFF_AppendLittle(pBuffOut, (U1 *)strBuff, 2);
#ifndef _POSTPROC
	UART_Send_Buff(comid, (I1 *)pBuffOut->buff, pBuffOut->curIdx);	
#endif
}

void Uart_Send_MSS(void)
{

}
/*
static void Smooth_Speed_Course(double *pf8VelDir)
{
	static unsigned short s_u2SmoothCount = 0;
	static double s_f8VeSmooth = 0.0, s_f8VnSmooth = 0.0;
	static double s_f8LastVeSmooth = 0.0, s_f8LastVnSmooth = 0.0;
	double f8SmoothCoeff = 0.0;
	double f8Alpha = 0.0, f8Beta = 0.0, f8TempCourse = 0.0;

	if ('A' == s_i1NavStateRMC)
	{
#ifdef MODE_DEBUG
		if (s_bAllowDebugMsgVelDir)
		{	
			UART_Printf(UART_PORT_DSPUart0, "evel %f nvel %f x %f y %f z %f posRMS %f velRMS %f ",
				s_pNavResult->evel,
				s_pNavResult->nvel,
				s_pNavResult->x,
				s_pNavResult->y,
				s_pNavResult->z,
				s_pNavResult->PosRMS,
				s_pNavResult->VelRMS
			);
		}
#endif

		if (0 == s_u2SmoothCount)
		{
			s_f8VeSmooth = NMEAData.evel;
			s_f8VnSmooth = NMEAData.nvel;
#ifdef MODE_DEBUG
			if (s_bAllowDebugMsgVelDir)
			{
				UART_Printf(UART_PORT_DSPUart0, "SmoothCount %2d VeSmth %f VnSmth %f ", 
					s_u2SmoothCount, 
					s_f8VeSmooth, 
					s_f8VnSmooth
				);
			}
#endif
		}
		else
		{
			f8SmoothCoeff = (double)s_u2SmoothCount;
			if ( (fabs(s_f8LastVeSmooth)<1.0) && (fabs(s_f8LastVnSmooth)<1.0) )
			{
				f8SmoothCoeff *= 2.0;
				if (f8SmoothCoeff > 50.0)
				{
					f8SmoothCoeff = 50.0;
				}
			}
			else
			{			
				f8SmoothCoeff /= 5.0;
				if (f8SmoothCoeff < 1.0)
				{
					f8SmoothCoeff = 1.0;
				}
				else if (f8SmoothCoeff > 5.0)
				{
					f8SmoothCoeff = 5.0;
				}
			}
			
			f8Alpha = 1.0 / f8SmoothCoeff;
			f8Beta = 1.0 - f8Alpha;
			s_f8VeSmooth = f8Alpha * NMEAData.evel + f8Beta * s_f8LastVeSmooth;
			s_f8VnSmooth = f8Alpha * NMEAData.nvel + f8Beta * s_f8LastVnSmooth;
#ifdef MODE_DEBUG
			if (s_bAllowDebugMsgVelDir)
			{
				UART_Printf(UART_PORT_DSPUart0, "SmoothCount %2d VeSmth %f VnSmth %f LastVesm %f LastVnsm %f SmthCoe %f ", 
					s_u2SmoothCount, 
					s_f8VeSmooth, 
					s_f8VnSmooth,
					s_f8LastVeSmooth,
					s_f8LastVnSmooth,
					f8SmoothCoeff
				);
			}
#endif		
		}
		
		s_u2SmoothCount++;
		if (s_u2SmoothCount > 50)
		{
			s_u2SmoothCount = 50;
		}
			
		s_f8LastVeSmooth = s_f8VeSmooth;
		s_f8LastVnSmooth = s_f8VnSmooth;
		
		f8TempCourse = atan2(s_f8VeSmooth, s_f8VnSmooth)*R2D;
		if (f8TempCourse < 0.0)
		{
			f8TempCourse += 360.0;
		}
#ifdef MODE_DEBUG
		if (s_bAllowDebugMsgVelDir)
		{
			UART_Printf(UART_PORT_DSPUart0, "CalSpdCourse %f ", f8TempCourse);
		}
#endif
		if ( (0==s_u2SmoothCount) || !((fabs(s_f8VeSmooth)<0.5)&&(fabs(s_f8VnSmooth)<0.5)) )
		{
			*pf8VelDir = f8TempCourse;
#ifdef MODE_DEBUG
			if (s_bAllowDebugMsgVelDir)
			{
				UART_Printf(UART_PORT_DSPUart0, "f8Dir %f ", *pf8VelDir);
			}
#endif
		}
#ifdef MODE_DEBUG
		if (s_bAllowDebugMsgVelDir)
		{
			f8TempCourse = atan2(s_pNavResult->evel, s_pNavResult->nvel) * R2D;
			if (f8TempCourse < 0.0)
			{
				f8TempCourse += 360.0;
			}
			UART_Printf(UART_PORT_DSPUart0, "CurCourse %f ", f8TempCourse);
		}
#endif
	}
	else
	{
		if (s_u2SmoothCount > 0)
		{
			s_u2SmoothCount--;
		}
	}
}
*/
void Uart_Send_RMC(byte comid)
{
	I1 strBuff[20];
	UINT8 u1CheckSum = 0;
	T_OUT_BUFF *pBuffOut = &g_OutBuff;
	BUFF_Clear(pBuffOut);

	// 0 ���ͱ�ʶ
	BUFF_AppendLittle(pBuffOut, s_strNavType, LEN_NAV_TYPE_0183);
	sprintf(strBuff, "%s", "RMC,");
	BUFF_AppendLittle(pBuffOut, (U1 *)strBuff, strlen((I1 *)strBuff));
	
	// 1��λʱ��
	BUFF_AppendLittle(pBuffOut, s_strTime, strlen((I1 *)s_strTime));
	
	// 2��λ״̬
	sprintf(strBuff, "%c,", s_i1NavStateRMC);
	BUFF_AppendLittle(pBuffOut, (U1 *)strBuff, strlen((I1 *)strBuff));

	// 3γ��
	BUFF_AppendLittle(pBuffOut, s_strLat, strlen((I1 *)s_strLat));

	// 4γ�ȷ���
	sprintf(strBuff, "%c,", s_i1LatNS);
	BUFF_AppendLittle(pBuffOut, (U1 *)strBuff, strlen((I1 *)strBuff));
	
	// 5����
	BUFF_AppendLittle(pBuffOut, s_strLon, strlen((I1 *)s_strLon));
	
	// 6���ȷ���
	sprintf(strBuff, "%c,", s_i1LonEW);
	BUFF_AppendLittle(pBuffOut, (U1 *)strBuff, strlen((I1 *)strBuff));
	
	// 7�ٶ�
	sprintf(strBuff, "%.4lf,", NMEAData.speed * M_PER_S_TO_KNOT);
	BUFF_AppendLittle(pBuffOut, (U1 *)strBuff, strlen((I1 *)strBuff));

	sprintf(strBuff, "%.4lf,", NMEAData.head);
	BUFF_AppendLittle(pBuffOut, (U1 *)strBuff, strlen((I1 *)strBuff));

	// 9����
	BUFF_AppendLittle(pBuffOut, s_strDate, strlen((I1 *)s_strDate));

	// 10��ƫ��
	sprintf(strBuff, "%.1f,", 0.0);
	BUFF_AppendLittle(pBuffOut, (U1 *)strBuff, strlen((I1 *)strBuff));

	// 11��ƫ�Ƿ���
	sprintf(strBuff, "%c,", 'E');
	BUFF_AppendLittle(pBuffOut, (U1 *)strBuff, strlen((I1 *)strBuff));

	// 12ģʽָʾ
	strBuff[0] = (1==s_i1NavStateGGA) ? 'A' : 'N';
	strBuff[1] = '\0';
	BUFF_AppendLittle(pBuffOut, (U1 *)strBuff, 1);

	// 13У���
	u1CheckSum = CalcCheckSum(&pBuffOut->buff[1], pBuffOut->curIdx-1);
	sprintf(strBuff, "*%02X", u1CheckSum);
	BUFF_AppendLittle(pBuffOut, (U1 *)strBuff, strlen((I1 *)strBuff));

	// 14��ֹ�� CR/LF
	strBuff[0] = 0x0D;
	strBuff[1] = 0x0A;
	BUFF_AppendLittle(pBuffOut, (U1 *)strBuff, 2);
#ifndef _POSTPROC
	UART_Send_Buff(comid, (I1 *)pBuffOut->buff, pBuffOut->curIdx);
#endif
}

void Uart_Send_GLL(byte comid)
{
	I1 strBuff[20];
	UINT8 u1CheckSum = 0;
	T_OUT_BUFF *pBuffOut = &g_OutBuff;
	BUFF_Clear(pBuffOut);
	// 0 ���ͱ�ʶ
	BUFF_AppendLittle(pBuffOut, s_strNavType, LEN_NAV_TYPE_0183);
	sprintf(strBuff, "%s", "GLL,");
	BUFF_AppendLittle(pBuffOut, (U1 *)strBuff, strlen((I1 *)strBuff));

	// γ��
	BUFF_AppendLittle(pBuffOut, s_strLat, strlen((I1 *)s_strLat));

	// γ�ȷ���
	sprintf(strBuff, "%c,", s_i1LatNS);		//��γ
	BUFF_AppendLittle(pBuffOut, (U1 *)strBuff, strlen((I1 *)strBuff));

	// ����
	BUFF_AppendLittle(pBuffOut, s_strLon, strlen((I1 *)s_strLon));

	// ���ȷ���
	sprintf(strBuff, "%c,", s_i1LonEW);		//����
	BUFF_AppendLittle(pBuffOut, (U1 *)strBuff, strlen((I1 *)strBuff));

	// UTC
	BUFF_AppendLittle(pBuffOut, s_strTime, strlen((I1 *)s_strTime));

	// ����״̬
	sprintf(strBuff, "%c,", s_i1NavStateRMC);
	BUFF_AppendLittle(pBuffOut, (U1 *)strBuff, strlen((I1 *)strBuff));

	// ģʽָʾ
	sprintf(strBuff, "%1d", 0);
	BUFF_AppendLittle(pBuffOut, (U1 *)strBuff, strlen((I1 *)strBuff));
	
	// У���
	u1CheckSum = CalcCheckSum(&pBuffOut->buff[1], pBuffOut->curIdx-1);
	sprintf(strBuff, "*%02X", u1CheckSum);
	BUFF_AppendLittle(pBuffOut, (U1 *)strBuff, strlen((I1 *)strBuff));

	// ��ֹ�� CR/LF
	strBuff[0] = 0x0D;
	strBuff[1] = 0x0A;
	BUFF_AppendLittle(pBuffOut, (U1 *)strBuff, 2);
#ifndef _POSTPROC	
	UART_Send_Buff(comid, (I1 *)pBuffOut->buff, pBuffOut->curIdx);	
#endif
}

void Uart_Send_GSA(UINT16 u2SvType,byte comid)
{
	int32 i;
	int32 iCntSvNav = 0;
	I1 strBuff[20];
	UINT8 u1CheckSum = 0;
	T_OUT_BUFF *pBuffOut = &g_OutBuff;
	int32 svid;
	BUFF_Clear(pBuffOut);
	if (NAV_SYS_GPS == u2SvType)
	{
		memcpy(strBuff, "$GPGSA,", 8);
	}
	else if (NAV_SYS_BD == u2SvType)
	{
		memcpy(strBuff, "$BDGSA,", 8);
	}
#if SUPPORT_GLONASS
	else if (NAV_SYS_GLO == u2SvType)
	{
		memcpy(strBuff, "$GLGSA,", 8);
	}
#endif
	BUFF_AppendLittle(pBuffOut, (U1 *)strBuff, strlen((I1 *)strBuff));

	// ģʽָʾ
	// �Զ�������2D/3D�Զ��任
	sprintf(strBuff, "%c,", 'A');
	BUFF_AppendLittle(pBuffOut, (U1 *)strBuff, strlen((I1 *)strBuff));

	// ѡ��ģʽ
	sprintf(strBuff, "%1d,", 3);
	BUFF_AppendLittle(pBuffOut, (U1 *)strBuff, strlen((I1 *)strBuff));

	// ������붨λ������
	iCntSvNav = 0;
	if(NAV_SYS_GPS == u2SvType)
	{
		for (i=0; i<(MaxGpsSvID-MinGpsSvID+1); i++)
		{
			svid = i+MinGpsSvID;
			if(GetBitWord256(&(NMEAData.fixsvmap),svid-1)==0)
				continue;

			sprintf(strBuff, "%02d,", i+1);
			BUFF_AppendLittle(pBuffOut, (U1 *)strBuff, strlen((I1 *)strBuff));
			
			if (++iCntSvNav >= 12)
			{
				break;
			}
		}
	}
	else if(NAV_SYS_BD == u2SvType)
	{
		for (i=0; i<(MaxBD2SvID-MinBD2SvID+1); i++)
		{
			svid = i+MinBD2SvID;
			if(GetBitWord256(&(NMEAData.fixsvmap),svid-1)==0)
				continue;

			sprintf(strBuff, "%02d,", i+1);
			BUFF_AppendLittle(pBuffOut, (U1 *)strBuff, strlen((I1 *)strBuff));
			
			if (++iCntSvNav >= 12)
			{
				break;
			}
		}
	}
#if SUPPORT_GLONASS
	else if(NAV_SYS_GLO == u2SvType)
	{
		for (i=0; i<(MaxGloSvIDTRUE-MinGloFreID+1); i++)
		{
			svid = i+MinGloFreID;
			if(GetBitWord256(&(NMEAData.fixsvmap),svid-1)==0)
				continue;

			sprintf(strBuff, "%02d,", i+1);
			BUFF_AppendLittle(pBuffOut, (U1 *)strBuff, strlen((I1 *)strBuff));
			
			if (++iCntSvNav >= 12)
			{
				break;
			}
		}
	}
#endif
	sprintf(strBuff, "%c", ',');
	for (i=0; i<12-iCntSvNav; i++)
	{
		BUFF_AppendLittle(pBuffOut, (U1 *)strBuff, strlen((I1 *)strBuff));
	}
	
	//PDOPֵ
	sprintf(strBuff, "%.1f,", NMEAData.pdop);
	BUFF_AppendLittle(pBuffOut, (U1 *)strBuff, strlen((I1 *)strBuff));

	//HDOPֵ
	sprintf(strBuff, "%.1f,", NMEAData.hdop);
	BUFF_AppendLittle(pBuffOut, (U1 *)strBuff, strlen((I1 *)strBuff));

	//VDOPֵ
	sprintf(strBuff,"%.1f,", NMEAData.vdop);
	BUFF_AppendLittle(pBuffOut, (U1 *)strBuff, strlen((I1 *)strBuff));

	//TDOPֵ
	if (NAV_SYS_GPS == u2SvType)
	{
		sprintf(strBuff, "%.1f", NMEAData.gpstdop);
	}
	else if (NAV_SYS_BD == u2SvType)
	{
		sprintf(strBuff, "%.1f", NMEAData.bdtdop);
	}
#if SUPPORT_GLONASS
	else if (NAV_SYS_GLO == u2SvType)
	{
		sprintf(strBuff, "%.1f", NMEAData.glotdop);
	}
#endif
	BUFF_AppendLittle(pBuffOut, (U1 *)strBuff, strlen((I1 *)strBuff));
	
	// У���
	u1CheckSum = CalcCheckSum(&pBuffOut->buff[1], pBuffOut->curIdx-1);
	sprintf(strBuff, "*%02X", u1CheckSum);
	BUFF_AppendLittle(pBuffOut, (U1 *)strBuff, strlen((I1 *)strBuff));

	// ��ֹ�� CR/LF
	strBuff[0] = 0x0D;
	strBuff[1] = 0x0A;
	BUFF_AppendLittle(pBuffOut, (U1 *)strBuff, 2);
#ifndef _POSTPROC	
	UART_Send_Buff(comid, (I1 *)pBuffOut->buff, pBuffOut->curIdx);
#endif
}

void Uart_Send_GSV(UINT16 u2SvType ,byte comid)
{
	UINT32 i = 0;
	UINT32 j = 0;
	//obsInfo *pChInfo = NULL;
	//SvInfoOfTransmitTimeStruct *pSvInfo = NULL;
	PVT_TRKCH_INFO *pChInfo = NULL;
	SV_INFO *pSvInfo = NULL;
	
	UINT16 u2SvIdxArray[MAXCHANNELS];	//���Ǻ�
	int16 i2ElvArray[MAXCHANNELS];		//��������
	int16 i2AziArray[MAXCHANNELS];		//���Ƿ�λ��
	UINT16 u2SnrArray[MAXCHANNELS];		//�����
	UINT32 uCntSv = 0;
	UINT32 uSvIdx = 0;
	double fAngleDeg = 0;
	UINT32 uCntLine = 0;
	UINT32 uMod = 0;
	UINT32 uCntCol = 0;
	UINT32 uIndex = 0;
	UINT8 u1CheckSum = 0;
	I1 strBuff[20];
	T_OUT_BUFF *pBuffOut = &g_OutBuff;
	UINT32 		freqgroup=0,SatReptFlag;
	pChInfo = PVTTrkchInfo;
	uCntSv = 0;
	
	for (i=0; i<MAXCHANNELS; i++)
	{	
		if ( (pChInfo[i].CCBF&0x07) != 0x07 )
		{
			continue;
		}

		/*if (pChInfo->obspresent[i] < 2)		// �۲����Ƿ�߱�
        {
			continue;
		}*/

		if (NAV_SYS_GPS == u2SvType)
		{
			freqgroup = FREQ_GROUP_GPS;
		}
		else if (NAV_SYS_BD == u2SvType)
		{
			freqgroup = FREQ_GROUP_BD;
		}
		else if (NAV_SYS_GLO == u2SvType)
		{
			freqgroup = FREQ_GROUP_GLO;
		}

		if (!(pChInfo[i].freq_point & freqgroup))
		{
			continue;
		}

#if SUPPORT_GLONASS
		if((!SV_IsGps(pChInfo[i].svid)) && (!SV_IsBd2(pChInfo[i].svid)&&(!SV_IsGlo(pChInfo[i].svid))))
#else
		if((!SV_IsGps(pChInfo[i].svid)) && (!SV_IsBd2(pChInfo[i].svid)))
#endif
			continue;

		if (NAV_SYS_GPS == u2SvType)
		{
			uSvIdx = pChInfo[i].svid;
			pSvInfo = &(SVInfo[uSvIdx-1]);
			uSvIdx -= (MinGpsSvID - 1);
		}
		else if (NAV_SYS_BD == u2SvType)
		{
			uSvIdx = pChInfo[i].svid;
			pSvInfo = &(SVInfo[uSvIdx-1]);
			uSvIdx -= (MinBD2SvID - 1);
		}
#if SUPPORT_GLONASS
		else if (NAV_SYS_GLO == u2SvType)
		{
			uSvIdx = ConvertGloSlotID2SVID(pChInfo[i].svid);
			pSvInfo = &(SVInfo[pChInfo[i].svid-1]);
			uSvIdx -= (MinGloFreID - 1);
		}
#endif
		else
			return;
		
		//add to delete repeat sat
		SatReptFlag = 0;
		if(i!=0)
		{
			for(j=0;j<i-1;j++)
			{
				if(pChInfo[j].svid == pChInfo[i].svid)
				{
					SatReptFlag = 1;
					break;
				}
			}
		}

		if(SatReptFlag)
		{
			continue;
		}
		//add end

		u2SvIdxArray[uCntSv] = (UINT16)uSvIdx;
		fAngleDeg = pSvInfo->el;
		if(fAngleDeg >90)
			fAngleDeg = 0.0;
		
		if (fAngleDeg < 1.0e-10)
		{
			fAngleDeg -= 0.5;
		}
		else
		{
			fAngleDeg += 0.5;
		}
		
		i2ElvArray[uCntSv] = (int16)fAngleDeg;
		i2AziArray[uCntSv] = (int16)(pSvInfo->az+0.5);
		u2SnrArray[uCntSv] = (UINT32)(pChInfo[i].cn1s);
		uCntSv++;
	}

	if (uCntSv < 1)
	{
		return;
	}
	
	uCntLine = uCntSv / 4;
	uMod = uCntSv % 4;
	if (uMod != 0)
	{
		uCntLine++;
	}

	uIndex = 0;
	for (i=0; i<uCntLine; i++)
	{
		BUFF_Clear(pBuffOut);
		if (NAV_SYS_GPS == u2SvType)
		{
			memcpy(strBuff, "$GPGSV,", 8);
		}
		else if (NAV_SYS_BD == u2SvType)
		{
			memcpy(strBuff, "$BDGSV,", 8);
		}
#if SUPPORT_GLONASS
		else if(NAV_SYS_GLO == u2SvType)
		{
			memcpy(strBuff, "$GLGSV,", 8);
		}
#endif
		BUFF_AppendLittle(pBuffOut, (U1 *)strBuff, strlen((I1 *)strBuff));

		// GSV�������
		sprintf(strBuff, "%u,", uCntLine);
		BUFF_AppendLittle(pBuffOut, (U1 *)strBuff, strlen((I1 *)strBuff));

		// ��ǰGSV������
		sprintf(strBuff, "%u,", i+1);
		BUFF_AppendLittle(pBuffOut, (U1 *)strBuff, strlen((I1 *)strBuff));
		
		// ��Ұ��������
		sprintf(strBuff, "%02u,", uCntSv);
		BUFF_AppendLittle(pBuffOut, (U1 *)strBuff, strlen((I1 *)strBuff));

		if ( (i+1==uCntLine) && (uMod!=0) )
		{
			uCntCol = uMod;
		}
		else
		{
			uCntCol = 4;
		}
		
		for (j=0; j<uCntCol; j++)
		{
			// ���Ǻ�
			sprintf(strBuff, "%02u,", u2SvIdxArray[uIndex]);
			BUFF_AppendLittle(pBuffOut, (U1 *)strBuff, strlen((I1 *)strBuff));

			// ��������
			sprintf(strBuff, "%02d,", i2ElvArray[uIndex]);
			BUFF_AppendLittle(pBuffOut, (U1 *)strBuff, strlen((I1 *)strBuff));

			// ���Ƿ�λ��
			sprintf(strBuff, "%03d,", i2AziArray[uIndex]);
			BUFF_AppendLittle(pBuffOut, (U1 *)strBuff, strlen((I1 *)strBuff));

			// �����
			sprintf(strBuff, "%02u", u2SnrArray[uIndex]);
			BUFF_AppendLittle(pBuffOut, (U1 *)strBuff, strlen((I1 *)strBuff));
			
			if (j+1 < uCntCol)
			{
				strBuff[0] = ',';
				BUFF_AppendLittle(pBuffOut, (U1 *)strBuff, 1);
			}
			
			uIndex++;
		}

		// У���
		u1CheckSum = CalcCheckSum(&pBuffOut->buff[1], pBuffOut->curIdx-1);
		sprintf(strBuff, "*%02X", u1CheckSum);
		BUFF_AppendLittle(pBuffOut, (U1 *)strBuff, strlen((I1 *)strBuff));
		
		// ��ֹ�� CR/LF
		strBuff[0] = 0x0D;
		strBuff[1] = 0x0A;
		BUFF_AppendLittle(pBuffOut, (U1 *)strBuff, 2);
#ifndef _POSTPROC		
		UART_Send_Buff(comid, (I1 *)pBuffOut->buff, pBuffOut->curIdx);
#endif
	}
}

/************************************************************************/
/*    1. ����DHV �ٶ��ർ����Ϣ                                         */
/*               �����Ϊ˫����䡣                                     */
/************************************************************************/
void Uart_Send_DHV(byte comid)
{
    double TempSpeed;
    double TempVelx,TempVely,TempVelz;
	I1 strBuff[20];
	UINT8 u1CheckSum = 0;
	T_OUT_BUFF *pBuffOut = &g_OutBuff;
	BUFF_Clear(pBuffOut);
  	// 0���ͱ�ʶ
	BUFF_AppendLittle(pBuffOut, s_strNavType, LEN_NAV_TYPE_0183);
	sprintf(strBuff, "%s", "DHV,");
	BUFF_AppendLittle(pBuffOut, (U1 *)strBuff, strlen((I1 *)strBuff));
	
    // 1λʱ��(UTCʱ��)s_strTime
    BUFF_AppendLittle(pBuffOut, s_strTime, strlen((char*)s_strTime));

    // 2�ٶ�
    TempSpeed=sqrt(NMEAData.vel.x*NMEAData.vel.x+
                   NMEAData.vel.y*NMEAData.vel.y+
                   NMEAData.vel.z*NMEAData.vel.z);
    TempSpeed *= 3.6;
    sprintf (strBuff, "%.5lf,", TempSpeed);
	BUFF_AppendLittle(pBuffOut, (U1 *)strBuff, strlen((I1 *)strBuff));

    // 3X���ٶ�
    TempVelx = NMEAData.vel.x*3.6;
    sprintf (strBuff, "%.5lf,", TempVelx);
	BUFF_AppendLittle(pBuffOut, (U1 *)strBuff, strlen((I1 *)strBuff));

    // 4Y���ٶ�
    TempVely = NMEAData.vel.y*3.6;
    sprintf (strBuff, "%.5lf,",  TempVely);
	BUFF_AppendLittle(pBuffOut, (U1 *)strBuff, strlen((I1 *)strBuff));
	
    // 5Z���ٶ�
    TempVelz = NMEAData.vel.z*3.6;
    sprintf (strBuff, "%.5lf,",  TempVelz);
	BUFF_AppendLittle(pBuffOut, (U1 *)strBuff, strlen((I1 *)strBuff));
	
    // 6
    sprintf(strBuff, "%c", ',');
	BUFF_AppendLittle(pBuffOut, (U1 *)strBuff, strlen((I1 *)strBuff));
	
    // 7����ٶ�
    sprintf(strBuff, "%c", ',');
	BUFF_AppendLittle(pBuffOut, (U1 *)strBuff, strlen((I1 *)strBuff));
	
    // 8ƽ���ٶ�
    sprintf(strBuff, "%c", ',');
	BUFF_AppendLittle(pBuffOut, (U1 *)strBuff, strlen((I1 *)strBuff));
	
    // 9ȫ��ƽ���ٶ� 
    sprintf(strBuff, "%c", ',');
	BUFF_AppendLittle(pBuffOut, (U1 *)strBuff, strlen((I1 *)strBuff));
	
    // 10��Ч�ٶ�
    sprintf(strBuff, "%c", ',');
	BUFF_AppendLittle(pBuffOut, (U1 *)strBuff, strlen((I1 *)strBuff));
	
    // 11�ٶȵ�λ
    sprintf(strBuff, "%c", 'K');
	BUFF_AppendLittle(pBuffOut, (U1 *)strBuff, strlen((I1 *)strBuff));
	
	// 12У���
	u1CheckSum = CalcCheckSum(&pBuffOut->buff[1], pBuffOut->curIdx-1);
	sprintf(strBuff, "*%02X", u1CheckSum);
	BUFF_AppendLittle(pBuffOut, (U1 *)strBuff, strlen((I1 *)strBuff));

	// 13��ֹ�� CR/LF
	strBuff[0] = 0x0D;
	strBuff[1] = 0x0A;
	BUFF_AppendLittle(pBuffOut, (U1 *)strBuff, 2);
#ifndef _POSTPROC
	UART_Send_Buff(comid, (I1 *)pBuffOut->buff, pBuffOut->curIdx);
#endif
}

void Uart_Send_NEMA0183_NPR(byte comid)
{
	I1 strBuff[20];
	UINT8 u1CheckSum = 0;
	T_OUT_BUFF *pBuffOut = &g_OutBuff;
	BUFF_Clear(pBuffOut);
  	// 0 ���ͱ�ʶ
	BUFF_AppendLittle(pBuffOut, s_strNavType, LEN_NAV_TYPE_0183);
	sprintf(strBuff, "%s", "NPR,");
	BUFF_AppendLittle(pBuffOut, (U1 *)strBuff, strlen((I1 *)strBuff));
	
    // 1λʱ��(UTCʱ��)s_strTime
    BUFF_AppendLittle(pBuffOut, s_strTime, strlen((char*)s_strTime));

    // 2��λ״̬
	sprintf (strBuff, "%08X,", NMEAData.fixstatus);
	BUFF_AppendLittle(pBuffOut, (U1 *)strBuff, strlen((I1 *)strBuff));

    // 3���붨λ��������
	sprintf (strBuff, "%02u,", NMEAData.svnuminfix);
	BUFF_AppendLittle(pBuffOut, (U1 *)strBuff, strlen((I1 *)strBuff));

    // 4PDOP
    sprintf (strBuff, "%.1f,", NMEAData.pdop);
	BUFF_AppendLittle(pBuffOut, (U1 *)strBuff, strlen((I1 *)strBuff));
	
    // 5α��в�
    //sprintf (strBuff, "%.1f,", s_pNavResult->PosRMS);
	sprintf (strBuff, "%.1f,", 0.0);
	BUFF_AppendLittle(pBuffOut, (U1 *)strBuff, strlen((I1 *)strBuff));
	
    // 6α���ʲв�
    //sprintf (strBuff, "%.2f,", s_pNavResult->VelRMS);
	sprintf (strBuff, "%.2f,", 0.0);
	BUFF_AppendLittle(pBuffOut, (U1 *)strBuff, strlen((I1 *)strBuff));
	
	// 7У���
	u1CheckSum = CalcCheckSum(&pBuffOut->buff[1], pBuffOut->curIdx-1);
	sprintf(strBuff, "*%02X", u1CheckSum);
	BUFF_AppendLittle(pBuffOut, (U1 *)strBuff, strlen((I1 *)strBuff));

	// 8��ֹ�� CR/LF
	strBuff[0] = 0x0D;
	strBuff[1] = 0x0A;
	BUFF_AppendLittle(pBuffOut, (U1 *)strBuff, 2);
#ifndef _POSTPROC
	UART_Send_Buff(comid, (I1 *)pBuffOut->buff, pBuffOut->curIdx);
#endif
}

/**
*	@brief		get receiver geiod undulation
*	@param		lat: receiver latitude, radius
*	@param		lon: receiver longitude, radius
*	@return		geiod undulation at receiver point
*	@author		goujuan
*	@date		Nov 18th, 2015
*/
float64 calcRcvrGeiodUndulation(float64 lat, float64 lon)
{
	int32 lat_grid_idx,lon_grid_idx, lat_resid, lon_resid, next_lat_grid_idx, next_lon_grid_idx;
	float64 und1, und2, undul;
	float64 lat_deg, lon_deg;

	lat_deg = lat * R2D;
	lon_deg = lon * R2D;

	//calc grid index
	lat_grid_idx = (int32)((90.0 - lat_deg) / GEIOD_STEP );
	lon_grid_idx = (int32)((lon_deg + 180.0) / GEIOD_STEP);

	lat_grid_idx = (lat_grid_idx<0)? 0 : (lat_grid_idx%19);
	lon_grid_idx = (lon_grid_idx<0)? 0 : (lon_grid_idx%36);
	
	//calc lat & lon residual
	lat_resid = ((int32)(90.0 - lat_deg))%((int32)GEIOD_STEP);
	lon_resid = ((int32)(lon_deg + 180.0))%((int32)GEIOD_STEP);

	//calc average base on the same latitude
	next_lat_grid_idx = (lat_grid_idx == 18)? lat_grid_idx : (lat_grid_idx+1);
	next_lon_grid_idx = (lon_grid_idx + 1)%36;
	
	und1 = GEIOD_UNDULATION[lat_grid_idx][lon_grid_idx]*((GEIOD_STEP - lon_resid)/GEIOD_STEP)
					+ GEIOD_UNDULATION[lat_grid_idx][next_lon_grid_idx]*(lon_resid/GEIOD_STEP);

	und2 = GEIOD_UNDULATION[next_lat_grid_idx][lon_grid_idx]*((GEIOD_STEP - lon_resid)/GEIOD_STEP)
					+ GEIOD_UNDULATION[next_lat_grid_idx][next_lon_grid_idx]*(lon_resid/GEIOD_STEP);

	
	//calc average base on the longitude
	undul = und1*((GEIOD_STEP - lat_resid)/GEIOD_STEP) + und2*(lat_resid/GEIOD_STEP);

	return undul;
}



void SenddataRS232_ATT(char reckon_time , byte comid)
{
#if 0//_SIMULATE
	I1 strBuff[20];
	UINT8 u1CheckSum = 0;
	T_OUT_BUFF *pBuffOut = &g_OutBuff;
	BUFF_Clear(pBuffOut);
	// 0 ���ͱ�ʶ
	BUFF_AppendLittle(pBuffOut, s_strNavType, LEN_NAV_TYPE_0183);

	sprintf(strBuff, "%s", "ATT,");
	BUFF_AppendLittle(pBuffOut, (U1 *)strBuff, strlen((I1 *)strBuff));
// 	if(m_CurUseNavMode == GPS_MODE)
// 		BDAddFieldChar (mSendBuf, &mSendIndex, "GPATT");
// 	else if((m_CurUseNavMode >= B1_CMODE) && (m_CurUseNavMode <= B1_B3_PMODE))
// 		BDAddFieldChar (mSendBuf, &mSendIndex, "BDATT");
// 	else if(m_CurUseNavMode == GLONASS_MODE)
// 		BDAddFieldChar (mSendBuf, &mSendIndex, "GLATT");
// 	else
// 		BDAddFieldChar (mSendBuf, &mSendIndex, "GNATT");
// 	BDAddFieldChar (mSendBuf, &mSendIndex, BD_SENTENCE_COMMA);


	// 1 (����ʱ�䣬ϵͳʱ��)
	sprintf (strBuff, "%.5lf,",  RtkOutData.time.dSeconds);
	BUFF_AppendLittle(pBuffOut, (U1 *)strBuff, strlen((I1 *)strBuff));
	// 2 reckon time
	sprintf (strBuff, "%d,",     0);
	BUFF_AppendLittle(pBuffOut, (U1 *)strBuff, strlen((I1 *)strBuff));

	// 3 �Լ��־
	sprintf (strBuff, "%d,",     RtkOutData.RtkCheck);
	BUFF_AppendLittle(pBuffOut, (U1 *)strBuff, strlen((I1 *)strBuff));

	// 4 �̶����־
	sprintf (strBuff, "%d,",     RtkOutData.nAmbIndex);
	BUFF_AppendLittle(pBuffOut, (U1 *)strBuff, strlen((I1 *)strBuff));

	// 5 ratio
	sprintf (strBuff, "%.5lf,",  RtkOutData.Ratio);
	BUFF_AppendLittle(pBuffOut, (U1 *)strBuff, strlen((I1 *)strBuff));
	// 6 baseline length
	sprintf (strBuff, "%.5lf,",  RtkOutData.baseline.BaselineLongth);
	BUFF_AppendLittle(pBuffOut, (U1 *)strBuff, strlen((I1 *)strBuff));
	// 7 yaw(rove.lat)
	//sprintf (strBuff, "%.3lf",  RtkOutData.attitue.yaw);
	sprintf (strBuff, "%.12lf,",  RtkOutData.RovePos.blh.latitude*180/PI);
	BUFF_AppendLittle(pBuffOut, (U1 *)strBuff, strlen((I1 *)strBuff));
	//8 pitch(rove.lon)
	//sprintf (strBuff, "%.3lf",  RtkOutData.attitue.pitch);
	sprintf (strBuff, "%.12lf,",  RtkOutData.RovePos.blh.longitude*180/PI);
	BUFF_AppendLittle(pBuffOut, (U1 *)strBuff, strlen((I1 *)strBuff));
	//9 pitch(rove.alt)
	sprintf (strBuff, "%.12lf,",  RtkOutData.RovePos.blh.altitude);
	BUFF_AppendLittle(pBuffOut, (U1 *)strBuff, strlen((I1 *)strBuff));
	//10 keysatID
	sprintf (strBuff, "%d,",     RtkOutData.nKeySatID);
	BUFF_AppendLittle(pBuffOut, (U1 *)strBuff, strlen((I1 *)strBuff));
	//11 ComSat
	sprintf (strBuff, "%d,",     RtkOutData.nCommSat);
	BUFF_AppendLittle(pBuffOut, (U1 *)strBuff, strlen((I1 *)strBuff));

// 	if(DGPS_MODE == DGPS_MODE_MOVBS)
// 	{
// 		// 5 yaw(rove.lat)����
// 		sprintf (strBuff, "%.3lf",  RtkOutData.attitue.yaw_deg);
// 		BDAddFieldChar (mSendBuf, &mSendIndex, strBuff);
// 		BDAddFieldChar (mSendBuf, &mSendIndex, BD_SENTENCE_COMMA);
// 		//6 pitch(rove.lon)����
// 		sprintf (strBuff, "%.3lf",  RtkOutData.attitue.pitch_deg);
// 		BDAddFieldChar (mSendBuf, &mSendIndex, strBuff);
// 		BDAddFieldChar (mSendBuf, &mSendIndex, BD_SENTENCE_COMMA);
// 
// 	}
// 	else
// 	{
// 		// 5 yaw(rove.lat)
// 		//sprintf (strBuff, "%.3lf",  RtkOutData.attitue.yaw);
// 		sprintf (strBuff, "%.12lf",  Basefixposition.lat*180/PI);
// 		BDAddFieldChar (mSendBuf, &mSendIndex, strBuff);
// 		BDAddFieldChar (mSendBuf, &mSendIndex, BD_SENTENCE_COMMA);
// 		//6 pitch(rove.lon)
// 		//sprintf (strBuff, "%.3lf",  RtkOutData.attitue.pitch);
// 		sprintf (strBuff, "%.12lf",  Basefixposition.lon*180/PI);
// 		BDAddFieldChar (mSendBuf, &mSendIndex, strBuff);
// 		BDAddFieldChar (mSendBuf, &mSendIndex, BD_SENTENCE_COMMA);
// 		//7 pitch(rove.alt)
// 		sprintf (strBuff, "%.12lf",  Basefixposition.hgt);
// 		BDAddFieldChar (mSendBuf, &mSendIndex, strBuff);
// 		BDAddFieldChar (mSendBuf, &mSendIndex, BD_SENTENCE_COMMA);
// 	}


	// У���
	u1CheckSum = CalcCheckSum(&pBuffOut->buff[1], pBuffOut->curIdx-1);
	sprintf(strBuff, "*%02X", u1CheckSum);
	BUFF_AppendLittle(pBuffOut, (U1 *)strBuff, strlen((I1 *)strBuff));

	// 8��ֹ�� CR/LF
	strBuff[0] = 0x0D;
	strBuff[1] = 0x0A;
	BUFF_AppendLittle(pBuffOut, (U1 *)strBuff, 2);
#ifndef _POSTPROC
	UART_Send_Buff(comid, (I1 *)pBuffOut->buff, pBuffOut->curIdx);
#endif
#endif
}


#if 0
void SenddataRS232_DBG()
{

	I1 strBuff[200];
	UINT8 u1CheckSum = 0;
	T_OUT_BUFF *pBuffOut = &g_OutBuff;
	UINT32 			comid;
	BUFF_Clear(pBuffOut);
	// 0 ���ͱ�ʶ
	BUFF_AppendLittle(pBuffOut, s_strNavType, LEN_NAV_TYPE_0183);

	sprintf(strBuff, "%s", "DBG,");
	BUFF_AppendLittle(pBuffOut, (U1 *)strBuff, strlen((I1 *)strBuff));

	// 1 (PRDD)
	sprintf(strBuff, "%s", "RPDD,");
	BUFF_AppendLittle(pBuffOut, (U1 *)strBuff, strlen((I1 *)strBuff));
   
	sprintf (strBuff, "%.3lf,",  RtkEpoch.Range_DD[0]);
	BUFF_AppendLittle(pBuffOut, (U1 *)strBuff, strlen((I1 *)strBuff));
	sprintf (strBuff, "%.3lf,",  RtkEpoch.Range_DD[1]);
	BUFF_AppendLittle(pBuffOut, (U1 *)strBuff, strlen((I1 *)strBuff));
	sprintf (strBuff, "%.3lf,",  RtkEpoch.Range_DD[2]);
	BUFF_AppendLittle(pBuffOut, (U1 *)strBuff, strlen((I1 *)strBuff));
	sprintf (strBuff, "%.3lf,",  RtkEpoch.Range_DD[3]);
	BUFF_AppendLittle(pBuffOut, (U1 *)strBuff, strlen((I1 *)strBuff));
	sprintf (strBuff, "%.3lf,",  RtkEpoch.Range_DD[4]);
	BUFF_AppendLittle(pBuffOut, (U1 *)strBuff, strlen((I1 *)strBuff));
	sprintf (strBuff, "%.3lf,",  RtkEpoch.Range_DD[5]);
	BUFF_AppendLittle(pBuffOut, (U1 *)strBuff, strlen((I1 *)strBuff));
	sprintf (strBuff, "%.3lf,",  RtkEpoch.Range_DD[6]);
	BUFF_AppendLittle(pBuffOut, (U1 *)strBuff, strlen((I1 *)strBuff));
	sprintf (strBuff, "%.3lf,",  RtkEpoch.Range_DD[7]);
	BUFF_AppendLittle(pBuffOut, (U1 *)strBuff, strlen((I1 *)strBuff));
	sprintf (strBuff, "%.3lf,",  RtkEpoch.Range_DD[8]);
	BUFF_AppendLittle(pBuffOut, (U1 *)strBuff, strlen((I1 *)strBuff));
	sprintf (strBuff, "%.3lf,",  RtkEpoch.Range_DD[9]);
	BUFF_AppendLittle(pBuffOut, (U1 *)strBuff, strlen((I1 *)strBuff));	

	// 2(CPDD)
	sprintf(strBuff, "%s", "CPDD,");
	BUFF_AppendLittle(pBuffOut, (U1 *)strBuff, strlen((I1 *)strBuff));    
	sprintf (strBuff, "%.3lf,",  RtkEpoch.Phase_DD[0]);
	BUFF_AppendLittle(pBuffOut, (U1 *)strBuff, strlen((I1 *)strBuff));    
	sprintf (strBuff, "%.3lf,",  RtkEpoch.Phase_DD[1]);
	BUFF_AppendLittle(pBuffOut, (U1 *)strBuff, strlen((I1 *)strBuff));    
	sprintf (strBuff, "%.3lf,",  RtkEpoch.Phase_DD[2]);
	BUFF_AppendLittle(pBuffOut, (U1 *)strBuff, strlen((I1 *)strBuff));    
	sprintf (strBuff, "%.3lf,",  RtkEpoch.Phase_DD[3]);
	BUFF_AppendLittle(pBuffOut, (U1 *)strBuff, strlen((I1 *)strBuff));    
	sprintf (strBuff, "%.3lf,",  RtkEpoch.Phase_DD[4]);
	BUFF_AppendLittle(pBuffOut, (U1 *)strBuff, strlen((I1 *)strBuff));    
	sprintf (strBuff, "%.3lf,",  RtkEpoch.Phase_DD[5]);
	BUFF_AppendLittle(pBuffOut, (U1 *)strBuff, strlen((I1 *)strBuff));    
	sprintf (strBuff, "%.3lf,",  RtkEpoch.Phase_DD[6]);
	BUFF_AppendLittle(pBuffOut, (U1 *)strBuff, strlen((I1 *)strBuff));    
	sprintf (strBuff, "%.3lf,",  RtkEpoch.Phase_DD[7]);
	BUFF_AppendLittle(pBuffOut, (U1 *)strBuff, strlen((I1 *)strBuff));    
	sprintf (strBuff, "%.3lf,",  RtkEpoch.Phase_DD[8]);
	BUFF_AppendLittle(pBuffOut, (U1 *)strBuff, strlen((I1 *)strBuff));    
	sprintf (strBuff, "%.3lf,",  RtkEpoch.Phase_DD[9]);
	BUFF_AppendLittle(pBuffOut, (U1 *)strBuff, strlen((I1 *)strBuff));    

	// 3(float)
	sprintf(strBuff, "%s", "float,");
	BUFF_AppendLittle(pBuffOut, (U1 *)strBuff, strlen((I1 *)strBuff));    
	sprintf (strBuff, "%.3lf,",  RtkEpoch.dAMB_PrDD[0]);
	BUFF_AppendLittle(pBuffOut, (U1 *)strBuff, strlen((I1 *)strBuff));
	sprintf (strBuff, "%.3lf,",  RtkEpoch.dAMB_PrDD[1]);
	BUFF_AppendLittle(pBuffOut, (U1 *)strBuff, strlen((I1 *)strBuff));
	sprintf (strBuff, "%.3lf,",  RtkEpoch.dAMB_PrDD[2]);
	BUFF_AppendLittle(pBuffOut, (U1 *)strBuff, strlen((I1 *)strBuff));
	sprintf (strBuff, "%.3lf,",  RtkEpoch.dAMB_PrDD[3]);
	BUFF_AppendLittle(pBuffOut, (U1 *)strBuff, strlen((I1 *)strBuff));
	sprintf (strBuff, "%.3lf,",  RtkEpoch.dAMB_PrDD[4]);
	BUFF_AppendLittle(pBuffOut, (U1 *)strBuff, strlen((I1 *)strBuff));
	sprintf (strBuff, "%.3lf,",  RtkEpoch.dAMB_PrDD[5]);
	BUFF_AppendLittle(pBuffOut, (U1 *)strBuff, strlen((I1 *)strBuff));
	sprintf (strBuff, "%.3lf,",  RtkEpoch.dAMB_PrDD[6]);
	BUFF_AppendLittle(pBuffOut, (U1 *)strBuff, strlen((I1 *)strBuff));
	sprintf (strBuff, "%.3lf,",  RtkEpoch.dAMB_PrDD[7]);
	BUFF_AppendLittle(pBuffOut, (U1 *)strBuff, strlen((I1 *)strBuff));
	sprintf (strBuff, "%.3lf,",  RtkEpoch.dAMB_PrDD[8]);
	BUFF_AppendLittle(pBuffOut, (U1 *)strBuff, strlen((I1 *)strBuff));
	sprintf (strBuff, "%.3lf,",  RtkEpoch.dAMB_PrDD[9]);
	BUFF_AppendLittle(pBuffOut, (U1 *)strBuff, strlen((I1 *)strBuff));

	// 4(filter_float)
	sprintf(strBuff, "%s", "filter_float,");
	BUFF_AppendLittle(pBuffOut, (U1 *)strBuff, strlen((I1 *)strBuff));
	sprintf (strBuff, "%.3lf,",  RtkEpochKeep.Rtk_NKeep[0]);
	BUFF_AppendLittle(pBuffOut, (U1 *)strBuff, strlen((I1 *)strBuff));
	sprintf (strBuff, "%.3lf,",  RtkEpochKeep.Rtk_NKeep[1]);
	BUFF_AppendLittle(pBuffOut, (U1 *)strBuff, strlen((I1 *)strBuff));
	sprintf (strBuff, "%.3lf,",  RtkEpochKeep.Rtk_NKeep[2]);
	BUFF_AppendLittle(pBuffOut, (U1 *)strBuff, strlen((I1 *)strBuff));
	sprintf (strBuff, "%.3lf,",  RtkEpochKeep.Rtk_NKeep[3]);
	BUFF_AppendLittle(pBuffOut, (U1 *)strBuff, strlen((I1 *)strBuff));
	sprintf (strBuff, "%.3lf,",  RtkEpochKeep.Rtk_NKeep[4]);
	BUFF_AppendLittle(pBuffOut, (U1 *)strBuff, strlen((I1 *)strBuff));
	sprintf (strBuff, "%.3lf,",  RtkEpochKeep.Rtk_NKeep[5]);
	BUFF_AppendLittle(pBuffOut, (U1 *)strBuff, strlen((I1 *)strBuff));
	sprintf (strBuff, "%.3lf,",  RtkEpochKeep.Rtk_NKeep[6]);
	BUFF_AppendLittle(pBuffOut, (U1 *)strBuff, strlen((I1 *)strBuff));
	sprintf (strBuff, "%.3lf,",  RtkEpochKeep.Rtk_NKeep[7]);
	BUFF_AppendLittle(pBuffOut, (U1 *)strBuff, strlen((I1 *)strBuff));
	sprintf (strBuff, "%.3lf,",  RtkEpochKeep.Rtk_NKeep[8]);
	BUFF_AppendLittle(pBuffOut, (U1 *)strBuff, strlen((I1 *)strBuff));
	sprintf (strBuff, "%.3lf,",  RtkEpochKeep.Rtk_NKeep[9]);
	BUFF_AppendLittle(pBuffOut, (U1 *)strBuff, strlen((I1 *)strBuff));


	// 5(fix)
	sprintf(strBuff, "%s", "fix,");
	BUFF_AppendLittle(pBuffOut, (U1 *)strBuff, strlen((I1 *)strBuff));
	sprintf (strBuff, "%.3lf,",  RtkEpoch.dAMB_Fix[0]);
	BUFF_AppendLittle(pBuffOut, (U1 *)strBuff, strlen((I1 *)strBuff));
	sprintf (strBuff, "%.3lf,",  RtkEpoch.dAMB_Fix[1]);
	BUFF_AppendLittle(pBuffOut, (U1 *)strBuff, strlen((I1 *)strBuff));
	sprintf (strBuff, "%.3lf,",  RtkEpoch.dAMB_Fix[2]);
	BUFF_AppendLittle(pBuffOut, (U1 *)strBuff, strlen((I1 *)strBuff));
	sprintf (strBuff, "%.3lf,",  RtkEpoch.dAMB_Fix[3]);
	BUFF_AppendLittle(pBuffOut, (U1 *)strBuff, strlen((I1 *)strBuff));
	sprintf (strBuff, "%.3lf,",  RtkEpoch.dAMB_Fix[4]);
	BUFF_AppendLittle(pBuffOut, (U1 *)strBuff, strlen((I1 *)strBuff));
	sprintf (strBuff, "%.3lf,",  RtkEpoch.dAMB_Fix[5]);
	BUFF_AppendLittle(pBuffOut, (U1 *)strBuff, strlen((I1 *)strBuff));
	sprintf (strBuff, "%.3lf,",  RtkEpoch.dAMB_Fix[6]);
	BUFF_AppendLittle(pBuffOut, (U1 *)strBuff, strlen((I1 *)strBuff));
	sprintf (strBuff, "%.3lf,",  RtkEpoch.dAMB_Fix[7]);
	BUFF_AppendLittle(pBuffOut, (U1 *)strBuff, strlen((I1 *)strBuff));
	sprintf (strBuff, "%.3lf,",  RtkEpoch.dAMB_Fix[8]);
	BUFF_AppendLittle(pBuffOut, (U1 *)strBuff, strlen((I1 *)strBuff));
	sprintf (strBuff, "%.3lf,",  RtkEpoch.dAMB_Fix[9]);
	BUFF_AppendLittle(pBuffOut, (U1 *)strBuff, strlen((I1 *)strBuff));



	// У���
	u1CheckSum = CalcCheckSum(&pBuffOut->buff[1], pBuffOut->curIdx-1);
	sprintf(strBuff, "*%02X", u1CheckSum);
	BUFF_AppendLittle(pBuffOut, (U1 *)strBuff, strlen((I1 *)strBuff));

	// 8��ֹ�� CR/LF
	strBuff[0] = 0x0D;
	strBuff[1] = 0x0A;
	BUFF_AppendLittle(pBuffOut, (U1 *)strBuff, 2);


	comid = UartSendId(CHECK_DATA_HEAD_NMEA);
	if((comid==0)||(comid==1)||(comid==2))
		UART_Send_Buff(comid, (I1 *)pBuffOut->buff, pBuffOut->curIdx);
}
#endif



