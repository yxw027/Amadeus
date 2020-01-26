/*
 * novatel.c
 *
 *  Created on: 2016-8-3
 *      Author: dell
 */
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>
#include "constdef.h"
#include "assitFunc4Bin.h"
#include "RTKCalBaseline.h"
#ifndef _POSTPROC
#ifndef _SIMULATE
#include "UartFunc.h"
#endif
#endif
#include "PVT.h"
#include "TimeProc.h"
#include "define.h"
#include "coordinate.h"
#include "PVTRcvrInfo.h"
#include "novatel.h"
#include "cfgpara.h"
#include "pvt.h"
#include "hcp.h"
#include "rtk.h"
#include "FrameDecode.h"
#include "CalcSvPVT.h"
#include "UpdateInfo.h"

#define CRC32_POLYNOMIAL 0xEDB88320L
/* --------------------------------------------------------------------------
Calculate a CRC value to be used by CRC calculation functions.
-------------------------------------------------------------------------- */
unsigned long CRC32Value(int i)
{
	int j;
	unsigned long ulCRC;
	ulCRC = i;
	for ( j = 8 ; j > 0; j-- )
	{
		if ( ulCRC & 1 )
		ulCRC = ( ulCRC >> 1 ) ^ CRC32_POLYNOMIAL;
		else
		ulCRC >>= 1;
	}
	return ulCRC;
	}
/* --------------------------------------------------------------------------
Calculates the CRC-32 of a block of data all at once
-------------------------------------------------------------------------- */
unsigned long CalculateBlockCRC32(unsigned long ulCount, /* Number of bytes in the data block */unsigned char *ucBuffer ) /* Data block */
{
	unsigned long ulTemp1;
	unsigned long ulTemp2;
	unsigned long ulCRC = 0;
	while ( ulCount-- != 0 )
	{
	ulTemp1 = ( ulCRC >> 8 ) & 0x00FFFFFFL;
	ulTemp2 = CRC32Value( ((int) ulCRC ^ *ucBuffer++ ) & 0xff );
	ulCRC = ulTemp1 ^ ulTemp2;
	}
	return( ulCRC );
}

void PeriodSendNovPro(void)
{
	static unsigned char SendPeriod_NovBestP[MAX_UART_NUM] = {0,0,0};
	static unsigned char SendPeriod_RangeA[MAX_UART_NUM] = {0,0,0};
	static unsigned char SendPeriod_Satvis2A[MAX_UART_NUM] = {0,0,0};
	static unsigned char SendPeriod_Bestposb[MAX_UART_NUM] = {0,0,0};
	static unsigned char SendPeriod_RangeB[MAX_UART_NUM] = {0,0,0};
	static unsigned char SendPeriod_HeadingA[MAX_UART_NUM] = {0,0,0};
	static unsigned char SendPeriod_Psrdopb[MAX_UART_NUM] = {0,0,0};
	byte comid = 0;	
	
	for(comid =0;comid<MAX_UART_NUM;comid++)
	{
		if((bFixAtSecBoundary == TRUE) && (IsSecBoundaryChange()))
		{
			SendPeriod_NovBestP[comid] = 0;
			SendPeriod_RangeA[comid] = 0;
			SendPeriod_Satvis2A[comid] = 0;
			SendPeriod_RangeB[comid] = 0;
			SendPeriod_HeadingA[comid] = 0;
			SendPeriod_Bestposb[comid] = 0;
			SendPeriod_Psrdopb[comid] = 0;
		}
		
		if(pActiveCPT->SysmCptProtocol[comid].COMInterFaceMode & CHECK_DATA_HEAD_NORATEL)
		{
			if((pActiveCPT->SysmCptProtocol[comid].NovAtel_Bestposa !=0 )&&(SendPeriod_NovBestP[comid] % pActiveCPT->SysmCptProtocol[comid].NovAtel_Bestposa == 0))
				Uart_Send_BESTPOSA(comid);
			if((pActiveCPT->SysmCptProtocol[comid].NovAtel_Rangea != 0 )&&(SendPeriod_RangeA[comid] % pActiveCPT->SysmCptProtocol[comid].NovAtel_Rangea == 0))
				SenddataRS232_RangeA(comid);
			if((pActiveCPT->SysmCptProtocol[comid].NovAtel_Satvis2a !=0 )&&(SendPeriod_Satvis2A[comid] % pActiveCPT->SysmCptProtocol[comid].NovAtel_Satvis2a == 0))
				SenddataRS232_Satvis2A(comid);
			if((pActiveCPT->SysmCptProtocol[comid].NovAtel_Headinga !=0 )&&(SendPeriod_HeadingA[comid] % pActiveCPT->SysmCptProtocol[comid].NovAtel_Headinga == 0))
				Uart_Send_HEADINGA(comid);
			if((pActiveCPT->SysmCptProtocol[comid].NovAtel_Bestposb !=0 )&&(SendPeriod_Bestposb[comid] % pActiveCPT->SysmCptProtocol[comid].NovAtel_Bestposb == 0))
				Uart_Send_BESTPOSB(comid);
			if((pActiveCPT->SysmCptProtocol[comid].NovAtel_Psrdopb !=0 )&&(SendPeriod_Psrdopb[comid] % pActiveCPT->SysmCptProtocol[comid].NovAtel_Psrdopb == 0))
				Uart_Send_PsrdopB(comid);
			if((pActiveCPT->SysmCptProtocol[comid].NovAtel_Rangeb !=0 )&&(SendPeriod_RangeB[comid] % pActiveCPT->SysmCptProtocol[comid].NovAtel_Rangeb == 0))
				Uart_Send_RangeB(comid);
		}
		
		SendPeriod_NovBestP[comid]++;
		SendPeriod_RangeA[comid]++;
		SendPeriod_Satvis2A[comid]++;
		SendPeriod_Bestposb[comid]++;
		SendPeriod_RangeB[comid]++;
		SendPeriod_HeadingA[comid]++;
		SendPeriod_Psrdopb[comid]++;
	}
}

void Uart_Send_BESTPOSA(byte comid)//debug
{
	I1 strBuff[160];
	unsigned int crc32;
	T_OUT_BUFF *pBuffOut = &g_OutBuff;
	int32 fix_result=0;
	int week=0,i,j,TrackSatCnt=0,ValidChCnt=0,TrackChCnt,ValidSatCnt=0;
	int sat1=0,sat2=0,satL1=0,satL2=0,satL5=0,satB1=0,satB2=0,satB3=0,satG1=0,satG2=0,satOther=0;
	int BDMask=0,GPSGLONASSMask = 0;
	double second=0.0,RFcnt,undulat,dSeconds,masterTow;

	double lat_deviation_m=0.0,lon_deviation_m=0.0,alt_deviation_m=0.0;
	ECEF pos;
	word256 Chmap;
	WGS des;
	FIX_DOP DOP;
	char ValidChTrack[MAXCHANNELS_PVT],ValidCh[MAXCHANNELS_PVT];
	BUFF_Clear(pBuffOut);

//1 HEADER
	// 1) SYNC 2) MESSAGE
	sprintf(strBuff, "%s", "#BESTPOSA,");
	BUFF_AppendLittle(pBuffOut, (U1 *)strBuff, strlen((I1 *)strBuff));
	//3) PORT
	if(comid==0)
		sprintf(strBuff, "%s", "COM0,");
	else
		sprintf(strBuff, "%s", "COM1,");
	BUFF_AppendLittle(pBuffOut, (U1 *)strBuff, strlen((I1 *)strBuff));
	//4) Sequence #  0:it is the last one of the set.
	sprintf(strBuff, "%d,", 0);
	BUFF_AppendLittle(pBuffOut, (U1 *)strBuff, strlen((I1 *)strBuff));
	//5) % Idle Time of processor
	sprintf(strBuff, "%0.1f,", 78.5);
	BUFF_AppendLittle(pBuffOut, (U1 *)strBuff, strlen((I1 *)strBuff));
	//6) Time Status
	if((Master_observ.clkSteer_bd==1) || (Master_observ.clkSteer_gps==1) || (Master_observ.clkSteer_glo==1))
		sprintf(strBuff, "%s", "FINESTERING,");
	else if((Master_observ.clkSteer_bd==0) || (Master_observ.clkSteer_gps==0) || (Master_observ.clkSteer_glo==0))
		sprintf(strBuff, "%s", "COARSE,");
	else
		sprintf(strBuff, "%s", "UNKNOWN,");
	BUFF_AppendLittle(pBuffOut, (U1 *)strBuff, strlen((I1 *)strBuff));

	RFcnt = getTICLockRFcnt();
	if(!GetNavSysTimeByRFCnt(NAV_SYS_NONE,GPSTIME_SYNC_SRC_FIXRFCNT,RFcnt,&second,&week))
		week = 0;
		
	//7) WEEK
	sprintf(strBuff, "%d,", week);
	BUFF_AppendLittle(pBuffOut, (U1 *)strBuff, strlen((I1 *)strBuff));
	//8) SECOND
	sprintf(strBuff, "%0.3f,", second);
	BUFF_AppendLittle(pBuffOut, (U1 *)strBuff, strlen((I1 *)strBuff));
	//9) Receiver Status ??????
	sprintf(strBuff, "%x,", 0x00000020);
	BUFF_AppendLittle(pBuffOut, (U1 *)strBuff, strlen((I1 *)strBuff));
	//10) Reserved
	sprintf(strBuff, "%x,", 0xb1f6);
	BUFF_AppendLittle(pBuffOut, (U1 *)strBuff, strlen((I1 *)strBuff));
	//11) Receiver s/w Version    12) ;
	sprintf(strBuff, "%d;", 12996);
	BUFF_AppendLittle(pBuffOut, (U1 *)strBuff, strlen((I1 *)strBuff));
//DATA
	// 2 solstat
	fix_result = getRcvrInfo(&pos,NULL,&Chmap,&DOP);

	if(fix_result == FIX_NOT)
		sprintf(strBuff, "%s", "INVALID_FIX,");
	else if(fix_result == FIX_2D)
		sprintf(strBuff, "%s", "V_H_LIMIT,");
	else
		sprintf(strBuff, "%s", "SOL_COMPUTED,");
	BUFF_AppendLittle(pBuffOut, (U1 *)strBuff, strlen((I1 *)strBuff));

	// 3 postype
	if(fix_result == FIX_NOT)
	{
		sprintf(strBuff, "%s", "NONE,");
	}
	else if(fix_result <= FIX_3D)
	{
		sprintf(strBuff, "%s", "SINGE,");
	}
	else if(fix_result == FIX_RTD)
	{
		sprintf(strBuff, "%s", "PSRDIFF,");
	}
	else if(fix_result == FIX_WIDE_FLOAT)
		sprintf(strBuff, "%s", "WIDE_FLOAT,");
	else if(fix_result == FIX_L1_FLOAT)
		sprintf(strBuff, "%s", "L1_FLOAT,");
	else if(fix_result == FIX_NARROW_FLOAT)
		sprintf(strBuff, "%s", "NARROW_FLOAT,");
	else if(fix_result == FIX_WIDE_INT)
		sprintf(strBuff, "%s", "WIDE_INT,");
	else if(fix_result == FIX_L1_INT)
		sprintf(strBuff, "%s", "L1_INT,");
	else if(fix_result == FIX_NARROW_INT)
		sprintf(strBuff, "%s", "NARROW_INT,");
	else
		sprintf(strBuff, "%s", "UNKOWN,");
		
	BUFF_AppendLittle(pBuffOut, (U1 *)strBuff, strlen((I1 *)strBuff));

////.............20170105
	//chmap bit 1 position
	ECEF2WGS(&pos,&des);
	// 4 lat 纬度
	sprintf(strBuff, "%0.11f,", des.lat*R2D);//(degree)
	BUFF_AppendLittle(pBuffOut, (U1 *)strBuff, strlen((I1 *)strBuff));

	// 5 lon 经度
	sprintf(strBuff, "%0.11f,", des.lon*R2D);//(degree)
	BUFF_AppendLittle(pBuffOut, (U1 *)strBuff, strlen((I1 *)strBuff));

	// 6 hgt
	sprintf(strBuff, "%0.4f,", des.alt);//(m)
	BUFF_AppendLittle(pBuffOut, (U1 *)strBuff, strlen((I1 *)strBuff));

	// 7 undulation
	//geoid_lat = 9 - (int32)(des.lat*R2D /10.0) ;
	//geoid_lon = (int32)(des.lon*R2D /10.0);
	//undulat = GEIOD_UNDULATION[geoid_lat][geoid_lon];//????debug
	undulat = 0.0;
	sprintf(strBuff, "%0.4f,", undulat);
	BUFF_AppendLittle(pBuffOut, (U1 *)strBuff, strlen((I1 *)strBuff));

	// 8 datum id#
	sprintf(strBuff, "%s", "WGS84,");
	BUFF_AppendLittle(pBuffOut, (U1 *)strBuff, strlen((I1 *)strBuff));

	//getPosDeviation(&lat_deviation_m,&lon_deviation_m,&alt_deviation_m);
	//deviation
	// 9 lat deviation
	sprintf(strBuff, "%0.4f,", lat_deviation_m);
	BUFF_AppendLittle(pBuffOut, (U1 *)strBuff, strlen((I1 *)strBuff));

	// 10 LON deviation
	sprintf(strBuff, "%0.4f,", lon_deviation_m);
	BUFF_AppendLittle(pBuffOut, (U1 *)strBuff, strlen((I1 *)strBuff));

	// 11 hgt deviation
	sprintf(strBuff, "%0.4f,", alt_deviation_m);
	BUFF_AppendLittle(pBuffOut, (U1 *)strBuff, strlen((I1 *)strBuff));

	// 12 stn id
	//Master_observ.staid???
	sprintf(strBuff, "%s", "AAAA,"); //??stn id 
	BUFF_AppendLittle(pBuffOut, (U1 *)strBuff, strlen((I1 *)strBuff));

	// 13 diff age
	if (fix_result <= FIX_3D)
		dSeconds = 0.0;
	else if(GetObsvTime(&Master_obs, &masterTow, NAV_SYS_NONE))
	{
		dSeconds = GetRTKExtraTime(NAV_SYS_NONE);
		//dSeconds = masterTow - extraobstow;
	}
	else
		dSeconds = 0.0;

	sprintf(strBuff, "%0.3f,", dSeconds);
	
	BUFF_AppendLittle(pBuffOut, (U1 *)strBuff, strlen((I1 *)strBuff));

	// 14 sol age
	sprintf(strBuff, "%s", "0.000,");
	BUFF_AppendLittle(pBuffOut, (U1 *)strBuff, strlen((I1 *)strBuff));

	// 15 Number of satellites tracked ???????where
	TrackChCnt = 0;
	for(i=0;i<MAXCHANNELS_PVT;i++)
	{
		if(PVTTrkchInfo[i].CCBF & 0x3)
		{
			ValidChTrack[TrackChCnt++]=PVTTrkchInfo[i].svid;
		}
	}
	TrackSatCnt = TrackChCnt;
	for(i=1;i<TrackChCnt;i++)
	{
		for(j=0;j<i-1;j++)
		{
			if(ValidChTrack[j] == ValidChTrack[i])
			{
				TrackSatCnt--;
				break;
			}
		}
	}
	
	sprintf(strBuff, "%d,", TrackSatCnt);
	BUFF_AppendLittle(pBuffOut, (U1 *)strBuff, strlen((I1 *)strBuff));
	//16 Number of satellites used in solution ??????
	ValidChCnt = 0;
	sat1 = 0;

	for(i=0;i<MAXCHANNELS;i++)
	{
		if(0 == GetBitWord256(&Chmap,i))
			continue;
		
		ValidCh[ValidChCnt++] = PVTTrkchInfo[i].svid;
		
		if(PVTTrkchInfo[i].freq_point == DSP_L1CA_FRE)
			satL1++;
		else if(PVTTrkchInfo[i].freq_point == DSP_L2P_FRE)
			satL2++;
		else if(PVTTrkchInfo[i].freq_point == DSP_L5_FRE)
			satL5++;
		else if(PVTTrkchInfo[i].freq_point == DSP_B1I_FRE)
			satB1++;
		else if(PVTTrkchInfo[i].freq_point == DSP_B2I_FRE)
			satB2++;
		else if(PVTTrkchInfo[i].freq_point == DSP_B3I_FRE)
			satB3++;
		else if(PVTTrkchInfo[i].freq_point == DSP_G1CA_FRE)
			satG1++;
		else if(PVTTrkchInfo[i].freq_point == DSP_G2CA_FRE)
			satG1++;
		else
			satOther++;
	}
	
	sat1 = satL1 + satB1 + satG1;
	sat2 = satL2 + satL5 + satB2 + satB3 + satG2 + satOther;

	ValidSatCnt = ValidChCnt;
	for(i=1;i<ValidChCnt;i++)
	{
		for(j=0;j<i-1;j++)
		{
			if(ValidCh[j] == ValidCh[i])
			{
				ValidSatCnt--;
				break;
			}
		}
	}
	sprintf(strBuff, "%d,", ValidSatCnt);
	BUFF_AppendLittle(pBuffOut, (U1 *)strBuff, strlen((I1 *)strBuff));
	//17 Number of satellites with L1/E1/B1 signals used in solution ??????
	sprintf(strBuff, "%d,", sat1);
	BUFF_AppendLittle(pBuffOut, (U1 *)strBuff, strlen((I1 *)strBuff));
	//18 Number of satellites with multi-frequency signals used in solution ??????
	sat2= sat1;//????
	sprintf(strBuff, "%d,", sat2);
	BUFF_AppendLittle(pBuffOut, (U1 *)strBuff, strlen((I1 *)strBuff));
	//19 Reserved
	sprintf(strBuff, "%02x,", 0x0);
	BUFF_AppendLittle(pBuffOut, (U1 *)strBuff, strlen((I1 *)strBuff));
	//20 ext sol stat ??????
	sprintf(strBuff, "%x,", 0x10);
	BUFF_AppendLittle(pBuffOut, (U1 *)strBuff, strlen((I1 *)strBuff));
	//21 Galileo and BeiDou sig mask ??????
	if(satB1>0)
		BDMask += 0x10;
	if(satB2>0)
		BDMask += 0x20;
	if(satB3>0)
		BDMask += 0x40;
	
	sprintf(strBuff, "%02x,", BDMask);
	BUFF_AppendLittle(pBuffOut, (U1 *)strBuff, strlen((I1 *)strBuff));
	//22 GPS and GLONASS sig mask ??????
	if(satL1>0)
		GPSGLONASSMask += 0x01;
	if(satL2>0)
		GPSGLONASSMask += 0x02;
	if(satL5>0)
		GPSGLONASSMask += 0x04;
	if(satG1>0)
		GPSGLONASSMask +=0x10;
	if(satG2>0)
		GPSGLONASSMask +=0x20;
	
	sprintf(strBuff, "%02x", GPSGLONASSMask);
	BUFF_AppendLittle(pBuffOut, (U1 *)strBuff, strlen((I1 *)strBuff));
	//23 crc
	crc32=CalculateBlockCRC32(pBuffOut->curIdx - 1, pBuffOut->buff + 1);
	sprintf (strBuff, "*%08x", (unsigned int)crc32);
	BUFF_AppendLittle(pBuffOut, (U1 *)strBuff, strlen((I1 *)strBuff));

	// 24终止符 CR/LF
	strBuff[0] = 0x0D;
	strBuff[1] = 0x0A;
	BUFF_AppendLittle(pBuffOut, (U1 *)strBuff, 2);
#ifndef _POSTPROC
	UART_Send_Buff(comid, (I1 *)pBuffOut->buff, pBuffOut->curIdx);
#endif	
}

int32 RangeANotOutCnt=0;
void SenddataRS232_RangeA(byte comid)
{
	I1 strBuff[160];
	unsigned int crc32,status=0;
	T_OUT_BUFF *pBuffOut = &g_OutBuff;
	int satcount=0;//,SatCntIndex;
	double second;
	//word256 poschmap;
	int32 svidx=0, freqidx=0;

	BUFF_Clear(pBuffOut);
//1 HEADER
	// 1) SYNC 2) MESSAGE
	sprintf(strBuff, "%s", "#RANGEA,");
	BUFF_AppendLittle(pBuffOut, (U1 *)strBuff, strlen((I1 *)strBuff));
	//3) PORT
	sprintf(strBuff, "%s", "COM1,");
	BUFF_AppendLittle(pBuffOut, (U1 *)strBuff, strlen((I1 *)strBuff));
	//4) Sequence #
	sprintf(strBuff, "%d,", 0);
	BUFF_AppendLittle(pBuffOut, (U1 *)strBuff, strlen((I1 *)strBuff));
	//5) % Idle Time
	sprintf(strBuff, "%0.1f,", 78.5);
	BUFF_AppendLittle(pBuffOut, (U1 *)strBuff, strlen((I1 *)strBuff));
	//6) Time Status
	sprintf(strBuff, "%s", "FINESTERING,");
	BUFF_AppendLittle(pBuffOut, (U1 *)strBuff, strlen((I1 *)strBuff));


	//7) WEEK
	sprintf(strBuff, "%d,", Master_observ.wn);
	BUFF_AppendLittle(pBuffOut, (U1 *)strBuff, strlen((I1 *)strBuff));
	//8) SECOND
	if(!GetObsvTime(&Master_observ,&second,NAV_SYS_NONE))
	{
		RangeANotOutCnt++;
		return;
	}
	
	sprintf(strBuff, "%0.6f,", second);
	BUFF_AppendLittle(pBuffOut, (U1 *)strBuff, strlen((I1 *)strBuff));
	//9) Receiver Status
	sprintf(strBuff, "%x,", 0x00000040);
	BUFF_AppendLittle(pBuffOut, (U1 *)strBuff, strlen((I1 *)strBuff));
	//10) Reserved
	sprintf(strBuff, "%d,", 6145);
	BUFF_AppendLittle(pBuffOut, (U1 *)strBuff, strlen((I1 *)strBuff));
	//11) Receiver s/w Version    12) ;
	sprintf(strBuff, "%d;", 1601);
	BUFF_AppendLittle(pBuffOut, (U1 *)strBuff, strlen((I1 *)strBuff));
//data
	for(svidx = 0; svidx<Master_observ.satnmax; svidx++)
	{
		for(freqidx=0; freqidx<MAX_FREPIONT_PER_NAVSYS; freqidx++)
		{
			if(Master_observ.obs[svidx].validmap[freqidx] !=0)
				satcount++;
		}
	}
	sprintf(strBuff, "%d", satcount);
	//SatCntIndex=pBuffOut->curIdx;
	BUFF_AppendLittle(pBuffOut, (U1 *)strBuff, strlen((I1 *)strBuff));

	satcount=0;
	for(svidx = 0; svidx<Master_observ.satnmax; svidx++)
	{
		for(freqidx=0; freqidx<MAX_FREPIONT_PER_NAVSYS; freqidx++)
		{
			if(Master_observ.obs[svidx].validmap[freqidx] == 0)
				continue;

			satcount++;
			//svid
			sprintf(strBuff, ",%d,", Master_observ.obs[svidx].StarID);
			BUFF_AppendLittle(pBuffOut, (U1 *)strBuff, strlen((I1 *)strBuff));

			//glo freq
#if SUPPORT_GLONASS
			if(Master_observ.obs[svidx].StarID>=MinGloFreID)	
				sprintf(strBuff, "%d,", Master_observ.obs[svidx].slotID-MinGloFreID);
			else
#endif
				sprintf(strBuff, "%d,", 0);
			BUFF_AppendLittle(pBuffOut, (U1 *)strBuff, strlen((I1 *)strBuff));
	
			//PR
			sprintf(strBuff, "%0.3f,", Master_observ.obs[svidx].range[freqidx]);
			BUFF_AppendLittle(pBuffOut, (U1 *)strBuff, strlen((I1 *)strBuff));

			//pr standard deviation
			sprintf(strBuff, "%f,", 0.02);
			BUFF_AppendLittle(pBuffOut, (U1 *)strBuff, strlen((I1 *)strBuff));

			//Carrier phase,
			sprintf(strBuff, "%0.6f,", Master_observ.obs[svidx].phase[freqidx]);
			BUFF_AppendLittle(pBuffOut, (U1 *)strBuff, strlen((I1 *)strBuff));

			//Carrier phase standard deviation
			sprintf(strBuff, "%0.3f,", 0.005);
			BUFF_AppendLittle(pBuffOut, (U1 *)strBuff, strlen((I1 *)strBuff));

			//doppler
			sprintf(strBuff, "%0.3f,", Master_observ.obs[svidx].doppler[freqidx]);
			BUFF_AppendLittle(pBuffOut, (U1 *)strBuff, strlen((I1 *)strBuff));

			//cn0
	        sprintf(strBuff, "%0.1f,", Master_observ.obs[svidx].snr0[freqidx]);
	        BUFF_AppendLittle(pBuffOut, (U1 *)strBuff, strlen((I1 *)strBuff));

			//locktime
	        sprintf(strBuff, "%d,", Master_observ.obs[svidx].LLI[freqidx]);//????
			BUFF_AppendLittle(pBuffOut, (U1 *)strBuff, strlen((I1 *)strBuff));

			status = 0;
			//status
			status |= ( satcount & 0x1f)<<5;//channel number
			status |= 0x00000400;//phase lock
			status |= 0x00000800;//parity known
			status |= 0x00001000;//code lock
			
			//if(Master_observ.obs[svidx].bUsedMap[freqidx]!=0)
				status |= 1<<29;	//used in solution

			if(Master_observ.obs[svidx].code[freqidx] & FREQ_GROUP_L2)
			{
				status |= 0x00100000;//grouping
				if(Master_observ.obs[svidx].code[freqidx]==DSP_L2P_FRE)
					status |= 5<<21;//gps L2P 
				else
					status |= 9<<21;//gps L2P codeless
				
				status |= 0x0000000b;//tracking state:phase lock loop
				status |= 0x00002000;//Standard correlator: spacing = 1 chip
			}
			else if(Master_observ.obs[svidx].code[freqidx] & FREQ_GROUP_L1)
			{
				status |= 0x00100000;//grouping
				status |= 0x08000000;//L1 primary
				status |= 0x00000004;//tracking state:phase lock loop
				status |= 0x00008000;//correlator state:Pulse Aperture Correlator
			}
			else if(Master_observ.obs[svidx].code[freqidx] & FREQ_GROUP_L5)
			{
				status |= 0x00100000;//grouping
				status |= 14<<21;//gps L5Q

				status |= 0x00000004;//tracking state:phase lock loop
				status |= 0x00008000;//correlator state:Pulse Aperture Correlator
			}
			else if(Master_observ.obs[svidx].code[freqidx] & FREQ_GROUP_B1)
			{
				status |= 0x00040000;//Satellite system
				status |= 0x08000000;//L1 channel primary
				if(SV_IsBd2Meo(Master_observ.obs[svidx].StarID))//meo D1 data
					status |= 0<<21;
				else//geo D2 data
					status |= 4ul<<21;
			}
			else if(Master_observ.obs[svidx].code[freqidx] & FREQ_GROUP_B2)
			{
				status |= 0x00040000;//Satellite system
				if(SV_IsBd2Meo(Master_observ.obs[svidx].StarID))//meo D1 data
					status |= 1ul<<21;
				else//geo
					status |= 5ul<<21;
			}
			else if(Master_observ.obs[svidx].code[freqidx] & FREQ_GROUP_B3)
			{
				status |= 0x00040000;//Satellite system
				if(SV_IsBd2Meo(Master_observ.obs[svidx].StarID))//meo D1 data
					status |= 2ul<<21;
				else//geo
					status |= 6ul<<21;
			}
			else if(Master_observ.obs[svidx].code[freqidx] & FREQ_GROUP_G1)//G1
			{
				status |= 0x00010000;//Satellite system
			}
			else if(Master_observ.obs[svidx].code[freqidx] & FREQ_GROUP_G2)//G2
			{
				status |= 0x00010000;//Satellite system
				status |= 1ul<<21;//L2 CA
			}
			else
			{
				status |= 0x00010000;
			}
			sprintf(strBuff, "%x", status);//????
			BUFF_AppendLittle(pBuffOut, (U1 *)strBuff, strlen((I1 *)strBuff));
		}
	}
		
	crc32=CalculateBlockCRC32(pBuffOut->curIdx - 1, pBuffOut->buff + 1);

	sprintf (strBuff, "*%08x", (unsigned int)crc32);
	BUFF_AppendLittle(pBuffOut, (U1 *)strBuff, strlen((I1 *)strBuff));

	// 24终止符 CR/LF
	strBuff[0] = 0x0D;
	strBuff[1] = 0x0A;
	BUFF_AppendLittle(pBuffOut, (U1 *)strBuff, 2);
#ifndef _POSTPROC
	UART_Send_Buff(comid, (I1 *)pBuffOut->buff, pBuffOut->curIdx);
#endif	
	return;
}

void SenddataRS232_Satvis2A(byte comid)
{
	int i,j,week,SatCntIndex,BDChCnt=0,GPSChCnt=0,GLChCnt=0,SatReptFlag,NValidGPSChCnt=0,NValidBDChCnt=0,NValidGLChCnt=0;
	char BDCh[MAXCHANNELS]={0},GPSCh[MAXCHANNELS]={0},GLCh[MAXCHANNELS]={0};
	I1 strBuff[160];
	PVT_TRKCH_INFO *pChInfo = NULL;
	T_OUT_BUFF *pBuffOut = &g_OutBuff;
	double second,RFcnt;
	unsigned int crc32;
	bool flag;

	pChInfo = PVTTrkchInfo;
	
	BUFF_Clear(pBuffOut);
	for (i=0; i<MAXCHANNELS; i++)
	{
		if((pChInfo[i].freq_point & FREQ_GROUP_GPS) && ((pChInfo[i].CCBF & 0x3) == 0x3) && pChInfo[i].bPRValid)
		{
			GPSCh[GPSChCnt++]=i;
		}
		else if((pChInfo[i].freq_point & FREQ_GROUP_BD)&& (pChInfo[i].CCBF & 0x3 == 0x3)&&pChInfo[i].bPRValid)
		{
			BDCh[BDChCnt++]=i;
		}
		else if((pChInfo[i].freq_point & FREQ_GROUP_GLO) && (pChInfo[i].CCBF & 0x3 == 0x3)&&pChInfo[i].bPRValid)
		{
			GLCh[GLChCnt++]=i;
		}
	}


	if(GPSChCnt>0)
	{
		BUFF_Clear(pBuffOut);
	//HEADER
		// 1) SYNC 2) MESSAGE
		sprintf(strBuff, "%s", "#SATVIS2A,");
		BUFF_AppendLittle(pBuffOut, (U1 *)strBuff, strlen((I1 *)strBuff));
		//3) PORT
		sprintf(strBuff, "%s", "COM1,");
		BUFF_AppendLittle(pBuffOut, (U1 *)strBuff, strlen((I1 *)strBuff));
		//4) Sequence #
		sprintf(strBuff, "%d,", 0);
		BUFF_AppendLittle(pBuffOut, (U1 *)strBuff, strlen((I1 *)strBuff));
		//5) % Idle Time
		sprintf(strBuff, "%0.1f,", 78.5);
		BUFF_AppendLittle(pBuffOut, (U1 *)strBuff, strlen((I1 *)strBuff));
		//6) Time Status
		sprintf(strBuff, "%s", "FINESTERING,");
		BUFF_AppendLittle(pBuffOut, (U1 *)strBuff, strlen((I1 *)strBuff));

		RFcnt = getTICLockRFcnt();
		flag = GetNavSysTimeByRFCnt(NAV_SYS_NONE,GPSTIME_SYNC_SRC_FIXRFCNT,RFcnt,&second,&week);
		//7) WEEK
		sprintf(strBuff, "%d,", week);
		BUFF_AppendLittle(pBuffOut, (U1 *)strBuff, strlen((I1 *)strBuff));
		//8) SECOND
		sprintf(strBuff, "%0.3f,", second);
		BUFF_AppendLittle(pBuffOut, (U1 *)strBuff, strlen((I1 *)strBuff));
		//9) Receiver Status
		sprintf(strBuff, "%x,", 0x00000040);
		BUFF_AppendLittle(pBuffOut, (U1 *)strBuff, strlen((I1 *)strBuff));
		//10) Reserved
		sprintf(strBuff, "%d,", 6145);
		BUFF_AppendLittle(pBuffOut, (U1 *)strBuff, strlen((I1 *)strBuff));
		//11) Receiver s/w Version    12) ;
		sprintf(strBuff, "%d;", 1601);
		BUFF_AppendLittle(pBuffOut, (U1 *)strBuff, strlen((I1 *)strBuff));	
	//data
		// 2 Satellite System 
		sprintf(strBuff, "%s", "GPS,");
		BUFF_AppendLittle(pBuffOut, (U1 *)strBuff, strlen((I1 *)strBuff));
		// 3 sat vis
		sprintf(strBuff, "%s", "TRUE,");
		BUFF_AppendLittle(pBuffOut, (U1 *)strBuff, strlen((I1 *)strBuff));
		// 4 comp alm
		sprintf(strBuff, "%s", "TRUE,");
		BUFF_AppendLittle(pBuffOut, (U1 *)strBuff, strlen((I1 *)strBuff));
		// 5 Number of satellites with data to follow	
		sprintf(strBuff, "%02d", GPSChCnt);
		SatCntIndex=pBuffOut->curIdx;
		BUFF_AppendLittle(pBuffOut, (U1 *)strBuff, strlen((I1 *)strBuff));

		for (i=0; i<GPSChCnt; i++)
		{
			if(!flag)
				break;
			
			SatReptFlag = 0;
			if(i!=0)
			{
				for(j=0;j<i-1;j++)
				{
					if(pChInfo[GPSCh[j]].svid == pChInfo[GPSCh[i]].svid)
					{
						SatReptFlag = 1;
						NValidGPSChCnt++;
						break;
					}
				}
			}
			
			if(SatReptFlag)
			{
				continue;
			}
			// 6 svid
			sprintf(strBuff, ",%d,", pChInfo[GPSCh[i]].svid);

			BUFF_AppendLittle(pBuffOut, (U1 *)strBuff, strlen((I1 *)strBuff));

			// 7 Satellite health
			sprintf(strBuff, "%d,", 0);
			BUFF_AppendLittle(pBuffOut, (U1 *)strBuff, strlen((I1 *)strBuff));

			//8 ELEZ
			sprintf(strBuff, "%d,", pChInfo[GPSCh[i]].el);
			BUFF_AppendLittle(pBuffOut, (U1 *)strBuff, strlen((I1 *)strBuff));

			// 9 AZ
			sprintf(strBuff, "%d,",  pChInfo[GPSCh[i]].az);
			BUFF_AppendLittle(pBuffOut, (U1 *)strBuff, strlen((I1 *)strBuff));

			// 10 true dop
			sprintf(strBuff, "%0.3f,", pChInfo[GPSCh[i]].fd);
			BUFF_AppendLittle(pBuffOut, (U1 *)strBuff, strlen((I1 *)strBuff));

			// 11 app dop
			sprintf(strBuff, "%0.3f", pChInfo[GPSCh[i]].fd);
			BUFF_AppendLittle(pBuffOut, (U1 *)strBuff, strlen((I1 *)strBuff));
		}
		//rangea cnt voluate
		sprintf(strBuff, "%02d", GPSChCnt - NValidGPSChCnt);
		for (i=0; i<strlen((I1 *)strBuff); i++)
		{
			pBuffOut->buff[SatCntIndex++] = strBuff[i];
		}
		
		crc32=CalculateBlockCRC32(pBuffOut->curIdx - 1, pBuffOut->buff + 1);
		sprintf (strBuff, "*%08x", (unsigned int)crc32);
		BUFF_AppendLittle(pBuffOut, (U1 *)strBuff, strlen((I1 *)strBuff));
		// 24终止符 CR/LF
		strBuff[0] = 0x0D;
		strBuff[1] = 0x0A;
		BUFF_AppendLittle(pBuffOut, (U1 *)strBuff, 2);
		// 向串口发送数据
#ifndef _POSTPROC
		UART_Send_Buff(comid, (I1 *)pBuffOut->buff, pBuffOut->curIdx);
#endif		
	}


	if(BDChCnt>0)
	{
		BUFF_Clear(pBuffOut);
	//HEADER
		// 1) SYNC 2) MESSAGE
		sprintf(strBuff, "%s", "#SATVIS2A,");
		BUFF_AppendLittle(pBuffOut, (U1 *)strBuff, strlen((I1 *)strBuff));
		//3) PORT
		if(GPSChCnt==0)
			sprintf(strBuff, "%s", "COM1,");
		else
			sprintf(strBuff, "%s", "COM2,");
		BUFF_AppendLittle(pBuffOut, (U1 *)strBuff, strlen((I1 *)strBuff));
		//4) Sequence #
		sprintf(strBuff, "%d,", 0);
		BUFF_AppendLittle(pBuffOut, (U1 *)strBuff, strlen((I1 *)strBuff));
		//5) % Idle Time
		sprintf(strBuff, "%0.1f,", 78.5);
		BUFF_AppendLittle(pBuffOut, (U1 *)strBuff, strlen((I1 *)strBuff));
		//6) Time Status
		sprintf(strBuff, "%s", "FINESTERING,");
		BUFF_AppendLittle(pBuffOut, (U1 *)strBuff, strlen((I1 *)strBuff));

		RFcnt = getTICLockRFcnt();
		flag = GetNavSysTimeByRFCnt(NAV_SYS_NONE,GPSTIME_SYNC_SRC_FIXRFCNT,RFcnt,&second,&week);
		//7) WEEK
		sprintf(strBuff, "%d,", week);
		BUFF_AppendLittle(pBuffOut, (U1 *)strBuff, strlen((I1 *)strBuff));
		//8) SECOND
		sprintf(strBuff, "%0.3f,", second);
		BUFF_AppendLittle(pBuffOut, (U1 *)strBuff, strlen((I1 *)strBuff));
		//9) Receiver Status
		sprintf(strBuff, "%x,", 0x00000040);
		BUFF_AppendLittle(pBuffOut, (U1 *)strBuff, strlen((I1 *)strBuff));
		//10) Reserved
		sprintf(strBuff, "%d,", 6145);
		BUFF_AppendLittle(pBuffOut, (U1 *)strBuff, strlen((I1 *)strBuff));
		//11) Receiver s/w Version    12) ;
		sprintf(strBuff, "%d;", 1601);
		BUFF_AppendLittle(pBuffOut, (U1 *)strBuff, strlen((I1 *)strBuff));	
	//data
		// 2 Satellite System
		sprintf(strBuff, "%s", "BEIDOU,");
		BUFF_AppendLittle(pBuffOut, (U1 *)strBuff, strlen((I1 *)strBuff));
		// 3 sat vis
		sprintf(strBuff, "%s", "TRUE,");
		BUFF_AppendLittle(pBuffOut, (U1 *)strBuff, strlen((I1 *)strBuff));
		// 4 comp alm
		sprintf(strBuff, "%s", "TRUE,");
		BUFF_AppendLittle(pBuffOut, (U1 *)strBuff, strlen((I1 *)strBuff));
		// 5 Number of satellites with data to follow
		sprintf(strBuff, "%02d", BDChCnt);
		SatCntIndex=pBuffOut->curIdx;
		BUFF_AppendLittle(pBuffOut, (U1 *)strBuff, strlen((I1 *)strBuff));

		for (i=0; i<BDChCnt; i++)
		{
			if(!flag)
				break;
			SatReptFlag = 0;
			if(i!=0)
			{
				for(j=0;j<i-1;j++)
				{
					if(pChInfo[BDCh[j]].svid == pChInfo[BDCh[i]].svid)
					{
						SatReptFlag = 1;
						NValidBDChCnt++;
						break;
					}
				}
			}

			if(SatReptFlag)
			{
				continue;
			}
			// 6 svid
			sprintf(strBuff, ",%d,", pChInfo[BDCh[i]].svid);
			BUFF_AppendLittle(pBuffOut, (U1 *)strBuff, strlen((I1 *)strBuff));

			// 7 Satellite health
			sprintf(strBuff, "%d,", 0);
			BUFF_AppendLittle(pBuffOut, (U1 *)strBuff, strlen((I1 *)strBuff));

			//8 ELEZ
			sprintf(strBuff, "%d,", pChInfo[BDCh[i]].el);
			BUFF_AppendLittle(pBuffOut, (U1 *)strBuff, strlen((I1 *)strBuff));

			// 9 AZ
			sprintf(strBuff, "%d,",  pChInfo[BDCh[i]].az);
			BUFF_AppendLittle(pBuffOut, (U1 *)strBuff, strlen((I1 *)strBuff));

			// 10 true dop
			sprintf(strBuff, "%0.3f,", pChInfo[BDCh[i]].fd);
			BUFF_AppendLittle(pBuffOut, (U1 *)strBuff, strlen((I1 *)strBuff));

			// 11 app dop
			sprintf(strBuff, "%0.3f", pChInfo[BDCh[i]].fd);
			BUFF_AppendLittle(pBuffOut, (U1 *)strBuff, strlen((I1 *)strBuff));

		}
		//rangea cnt voluate
		sprintf(strBuff, "%02d", BDChCnt - NValidBDChCnt);
		for (i=0; i<strlen((I1 *)strBuff); i++)
		{
			pBuffOut->buff[SatCntIndex++] = strBuff[i];
		}

		crc32=CalculateBlockCRC32(pBuffOut->curIdx - 1, pBuffOut->buff + 1);
		sprintf (strBuff, "*%08x", (unsigned int)crc32);
		BUFF_AppendLittle(pBuffOut, (U1 *)strBuff, strlen((I1 *)strBuff));
		// 24终止符 CR/LF
		strBuff[0] = 0x0D;
		strBuff[1] = 0x0A;
		BUFF_AppendLittle(pBuffOut, (U1 *)strBuff, 2);
#ifndef _POSTPROC
		UART_Send_Buff(comid, (I1 *)pBuffOut->buff, pBuffOut->curIdx);
#endif
	}

	if(GLChCnt>0)
	{
		BUFF_Clear(pBuffOut);
	//HEADER
		// 1) SYNC 2) MESSAGE
		sprintf(strBuff, "%s", "#SATVIS2A,");
		BUFF_AppendLittle(pBuffOut, (U1 *)strBuff, strlen((I1 *)strBuff));
		//3) PORT
		if((GPSChCnt==0)&&(BDChCnt==0))
			sprintf(strBuff, "%s", "COM1,");
		else if(GPSChCnt&&BDChCnt)
			sprintf(strBuff, "%s", "COM3,");
		else
			sprintf(strBuff, "%s", "COM2,");
		BUFF_AppendLittle(pBuffOut, (U1 *)strBuff, strlen((I1 *)strBuff));
		//4) Sequence #
		sprintf(strBuff, "%d,", 0);
		BUFF_AppendLittle(pBuffOut, (U1 *)strBuff, strlen((I1 *)strBuff));
		//5) % Idle Time
		sprintf(strBuff, "%0.1f,", 78.5);
		BUFF_AppendLittle(pBuffOut, (U1 *)strBuff, strlen((I1 *)strBuff));
		//6) Time Status
		sprintf(strBuff, "%s", "FINESTERING,");
		BUFF_AppendLittle(pBuffOut, (U1 *)strBuff, strlen((I1 *)strBuff));

		RFcnt = getTICLockRFcnt();
		flag = GetNavSysTimeByRFCnt(NAV_SYS_NONE,GPSTIME_SYNC_SRC_GLO_ONLY,RFcnt,&second,&week);
		//7) WEEK
		sprintf(strBuff, "%d,", week);
		BUFF_AppendLittle(pBuffOut, (U1 *)strBuff, strlen((I1 *)strBuff));
		//8) SECOND
		sprintf(strBuff, "%0.3f,", second);
		BUFF_AppendLittle(pBuffOut, (U1 *)strBuff, strlen((I1 *)strBuff));
		//9) Receiver Status
		sprintf(strBuff, "%x,", 0x00000040);
		BUFF_AppendLittle(pBuffOut, (U1 *)strBuff, strlen((I1 *)strBuff));
		//10) Reserved
		sprintf(strBuff, "%d,", 6145);
		BUFF_AppendLittle(pBuffOut, (U1 *)strBuff, strlen((I1 *)strBuff));
		//11) Receiver s/w Version    12) ;
		sprintf(strBuff, "%d;", 1601);
		BUFF_AppendLittle(pBuffOut, (U1 *)strBuff, strlen((I1 *)strBuff));
	//data
		// 2 Satellite System
		sprintf(strBuff, "%s", "GLONASS,");
		BUFF_AppendLittle(pBuffOut, (U1 *)strBuff, strlen((I1 *)strBuff));
		// 3 sat vis
		sprintf(strBuff, "%s", "TRUE,");
		BUFF_AppendLittle(pBuffOut, (U1 *)strBuff, strlen((I1 *)strBuff));
		// 4 comp alm
		sprintf(strBuff, "%s", "TRUE,");
		BUFF_AppendLittle(pBuffOut, (U1 *)strBuff, strlen((I1 *)strBuff));
		// 5 Number of satellites with data to follow
		sprintf(strBuff, "%02d", GLChCnt);
		SatCntIndex=pBuffOut->curIdx;
		BUFF_AppendLittle(pBuffOut, (U1 *)strBuff, strlen((I1 *)strBuff));

		for (i=0; i<GLChCnt; i++)
		{
			if(!flag)
				break;
			SatReptFlag = 0;
			if(i!=0)
			{
				for(j=0;j<i-1;j++)
				{
					if(pChInfo[GLCh[j]].svid == pChInfo[GLCh[i]].svid)
					{
						SatReptFlag = 1;
						NValidGLChCnt++;
						break;
					}
				}
			}

			if(SatReptFlag)
			{
				continue;
			}
			// 6 svid
#if SUPPORT_GLONASS
			//sprintf(strBuff, ",%d,", ConvertGloSlotID2SVID(SV_IsGlo(pChInfo[GLCh[i]].svid)));
			sprintf(strBuff, ",%d,", pChInfo[GLCh[i]].svid);
#endif
			BUFF_AppendLittle(pBuffOut, (U1 *)strBuff, strlen((I1 *)strBuff));

			// 7 Satellite health
			sprintf(strBuff, "%d,", 0);
			BUFF_AppendLittle(pBuffOut, (U1 *)strBuff, strlen((I1 *)strBuff));

			//8 ELEZ
			sprintf(strBuff, "%d,", pChInfo[GLCh[i]].el);
			BUFF_AppendLittle(pBuffOut, (U1 *)strBuff, strlen((I1 *)strBuff));

			// 9 AZ
			sprintf(strBuff, "%d,",  pChInfo[GLCh[i]].az);
			BUFF_AppendLittle(pBuffOut, (U1 *)strBuff, strlen((I1 *)strBuff));

			// 10 true dop
			sprintf(strBuff, "%0.3f,", pChInfo[GLCh[i]].fd);
			BUFF_AppendLittle(pBuffOut, (U1 *)strBuff, strlen((I1 *)strBuff));

			// 11 app dop
			sprintf(strBuff, "%0.3f", pChInfo[GLCh[i]].fd);
			BUFF_AppendLittle(pBuffOut, (U1 *)strBuff, strlen((I1 *)strBuff));

		}
		//rangea cnt voluate
		sprintf(strBuff, "%02d", GLChCnt - NValidGLChCnt);
		for (i=0; i<strlen((I1 *)strBuff); i++)
		{
			pBuffOut->buff[SatCntIndex++] = strBuff[i];
		}

		crc32=CalculateBlockCRC32(pBuffOut->curIdx - 1, pBuffOut->buff + 1);
		sprintf (strBuff, "*%08x", (unsigned int)crc32);
		BUFF_AppendLittle(pBuffOut, (U1 *)strBuff, strlen((I1 *)strBuff));
		// 24终止符 CR/LF
		strBuff[0] = 0x0D;
		strBuff[1] = 0x0A;
		BUFF_AppendLittle(pBuffOut, (U1 *)strBuff, 2);
#ifndef _POSTPROC		
		UART_Send_Buff(comid, (I1 *)pBuffOut->buff, pBuffOut->curIdx);
#endif
	}
}


void Uart_Send_HEADINGA(byte comid)
{
	I1 strBuff[160];
	unsigned int crc32;
	T_OUT_BUFF *pBuffOut = &g_OutBuff;
	int32 fix_result=0;
	int week,i,j,TrackSatCnt=0,ValidChCnt=0,TrackChCnt,ValidSatCnt,ObsAboveElvCnt,ObsL2AboveElvCnt=0;
	int satL1=0,satL2=0,satL5=0,satB1=0,satB2=0,satB3=0,satG1=0,satG2=0,satOther=0;
	double second,RFcnt,baselinelen,heading,pitch=0.0;
	int BDMask=0,GPSGLONASSMask = 0;
	BD2_BASELINE BaselineData;
	BD2_Attitude Att;

	ECEF pos={0};
	word256 Chmap={0};
	FIX_DOP DOP={0};
	char ValidChTrack[MAXCHANNELS_PVT],ValidCh[MAXCHANNELS_PVT];
	BUFF_Clear(pBuffOut);
	memset(&BaselineData,0,sizeof(BD2_BASELINE));
	memset(&Att,0,sizeof(BD2_Attitude));
//1 HEADER
	// 1) SYNC 2) MESSAGE
	sprintf(strBuff, "%s", "#HEADINGA,");
	BUFF_AppendLittle(pBuffOut, (U1 *)strBuff, strlen((I1 *)strBuff));
	//3) PORT
	if(comid==0)
		sprintf(strBuff, "%s", "COM0,");
	else
		sprintf(strBuff, "%s", "COM1,");
	BUFF_AppendLittle(pBuffOut, (U1 *)strBuff, strlen((I1 *)strBuff));
	//4) Sequence #  0:it is the last one of the set.
	sprintf(strBuff, "%d,", 0);
	BUFF_AppendLittle(pBuffOut, (U1 *)strBuff, strlen((I1 *)strBuff));
	//5) % Idle Time of processor
	sprintf(strBuff, "%0.1f,", 78.5);
	BUFF_AppendLittle(pBuffOut, (U1 *)strBuff, strlen((I1 *)strBuff));
	//6) Time Status
	if((Master_observ.clkSteer_bd==1) || (Master_observ.clkSteer_gps==1) || (Master_observ.clkSteer_glo==1))
		sprintf(strBuff, "%s", "FINESTERING,");
	else if((Master_observ.clkSteer_bd==0) || (Master_observ.clkSteer_gps==0) || (Master_observ.clkSteer_glo==0))
		sprintf(strBuff, "%s", "COARSE,");
	else
		sprintf(strBuff, "%s", "UNKNOWN,");
	BUFF_AppendLittle(pBuffOut, (U1 *)strBuff, strlen((I1 *)strBuff));

	RFcnt = getTICLockRFcnt();
	GetNavSysTimeByRFCnt(NAV_SYS_NONE,GPSTIME_SYNC_SRC_FIXRFCNT,RFcnt,&second,&week);
	//7) WEEK
	sprintf(strBuff, "%d,", week);
	BUFF_AppendLittle(pBuffOut, (U1 *)strBuff, strlen((I1 *)strBuff));
	//8) SECOND
	sprintf(strBuff, "%0.3f,", second);
	BUFF_AppendLittle(pBuffOut, (U1 *)strBuff, strlen((I1 *)strBuff));
	//9) Receiver Status ??????
	sprintf(strBuff, "%x,", 0x00000020);
	BUFF_AppendLittle(pBuffOut, (U1 *)strBuff, strlen((I1 *)strBuff));
	//10) Reserved
	sprintf(strBuff, "%x,", 0xb1f6);
	BUFF_AppendLittle(pBuffOut, (U1 *)strBuff, strlen((I1 *)strBuff));
	//11) Receiver s/w Version    12) ;
	sprintf(strBuff, "%d;", 12996);
	BUFF_AppendLittle(pBuffOut, (U1 *)strBuff, strlen((I1 *)strBuff));
//DATA
	// 2 solstat
	fix_result = getRcvrInfo(&pos,NULL,&Chmap,&DOP);

	if(fix_result == FIX_NOT)
		sprintf(strBuff, "%s", "INVALID_FIX,");
	else if(fix_result == FIX_2D)
		sprintf(strBuff, "%s", "V_H_LIMIT,");
	else
		sprintf(strBuff, "%s", "SOL_COMPUTED,");
	BUFF_AppendLittle(pBuffOut, (U1 *)strBuff, strlen((I1 *)strBuff));

	// 3 postype
	if(fix_result == FIX_NOT)
	{
		sprintf(strBuff, "%s", "NONE,");
	}
	else if(fix_result <= FIX_3D)
	{
		sprintf(strBuff, "%s", "SINGE,");
	}
	else if(fix_result == FIX_RTD)
	{
		sprintf(strBuff, "%s", "PSRDIFF,");
	}
	else if(fix_result == FIX_WIDE_FLOAT)
		sprintf(strBuff, "%s", "WIDE_FLOAT,");
	else if(fix_result == FIX_L1_FLOAT)
		sprintf(strBuff, "%s", "L1_FLOAT,");
	else if(fix_result == FIX_NARROW_FLOAT)
		sprintf(strBuff, "%s", "NARROW_FLOAT,");
	else if(fix_result == FIX_WIDE_INT)
		sprintf(strBuff, "%s", "WIDE_INT,");
	else if(fix_result == FIX_L1_INT)
		sprintf(strBuff, "%s", "L1_INT,");
	else if(fix_result == FIX_NARROW_INT)
		sprintf(strBuff, "%s", "NARROW_INT,");
	else
		sprintf(strBuff, "%s", "UNKOWN,");
		
	BUFF_AppendLittle(pBuffOut, (U1 *)strBuff, strlen((I1 *)strBuff));

	getRcvrBaseLine(&BaselineData);
	getRcvrAttInfo(&Att);
	// 4 beseline length
	baselinelen = BaselineData.BaselineLongth;
	sprintf(strBuff, "%.5lf,", baselinelen);
	BUFF_AppendLittle(pBuffOut, (U1 *)strBuff, strlen((I1 *)strBuff));

	// 5 heading in degrees
	heading = Att.yaw_deg;
	sprintf(strBuff, "%.5lf,", heading);
	BUFF_AppendLittle(pBuffOut, (U1 *)strBuff, strlen((I1 *)strBuff));
	
	// 6 pitch degree
	pitch = Att.pitch_deg;
	sprintf(strBuff, "%.5lf,", pitch);
	BUFF_AppendLittle(pBuffOut, (U1 *)strBuff, strlen((I1 *)strBuff));
	
	// 7 Reserved
	sprintf(strBuff, "%.2lf,", 0.0);
	BUFF_AppendLittle(pBuffOut, (U1 *)strBuff, strlen((I1 *)strBuff));
	
	// 8 hdg std dev
	sprintf(strBuff, "%.2lf,", 0.0);
	BUFF_AppendLittle(pBuffOut, (U1 *)strBuff, strlen((I1 *)strBuff));
	
	// 9 pitch std dev
	sprintf(strBuff, "%.2lf,", 0.0);
	BUFF_AppendLittle(pBuffOut, (U1 *)strBuff, strlen((I1 *)strBuff));
	
	// 10 stn ID
	sprintf(strBuff, "%d,", pActiveCPT->SysmCptWorkConfig.BaseStaID); //??stn id 
	BUFF_AppendLittle(pBuffOut, (U1 *)strBuff, strlen((I1 *)strBuff));
	
	// 11 SVs
	TrackChCnt = 0;
	for(i=0;i<MAXCHANNELS_PVT;i++)
	{
		if(PVTTrkchInfo[i].CCBF & 0x3)
		{
			ValidChTrack[TrackChCnt++]=PVTTrkchInfo[i].svid;
		}
	}
	TrackSatCnt = TrackChCnt;
	for(i=1;i<TrackChCnt;i++)
	{
		for(j=0;j<i-1;j++)
		{
			if(ValidChTrack[j] == ValidChTrack[i])
			{
				TrackSatCnt--;
				break;
			}
		}
	}	
	sprintf(strBuff, "%d,", TrackSatCnt);
	BUFF_AppendLittle(pBuffOut, (U1 *)strBuff, strlen((I1 *)strBuff));
	
	// 12 solnSVs
	ValidChCnt = 0;
	//sat1 = 0;

	for(i=0;i<MAXCHANNELS_PVT;i++)
	{
		if(0 == GetBitWord256(&Chmap,i))
			continue;
		
		ValidCh[ValidChCnt++] = PVTTrkchInfo[i].svid;
		
		if(PVTTrkchInfo[i].freq_point == DSP_L1CA_FRE)
			satL1++;
		else if(PVTTrkchInfo[i].freq_point == DSP_L2P_FRE)
			satL2++;
		else if(PVTTrkchInfo[i].freq_point == DSP_L5_FRE)
			satL5++;
		else if(PVTTrkchInfo[i].freq_point == DSP_B1I_FRE)
			satB1++;
		else if(PVTTrkchInfo[i].freq_point == DSP_B2I_FRE)
			satB2++;
		else if(PVTTrkchInfo[i].freq_point == DSP_B3I_FRE)
			satB3++;
		else if(PVTTrkchInfo[i].freq_point == DSP_G1CA_FRE)
			satG1++;
		else if(PVTTrkchInfo[i].freq_point == DSP_G2CA_FRE)
			satG1++;
		else
			satOther++;
	}
	
	//sat1 = satL1 + satB1 + satG1;
	//sat2 = satL2 + satL5 + satB2 + satB3 + satG2 + satOther;

	ValidSatCnt = ValidChCnt;
	for(i=1;i<ValidChCnt;i++)
	{
		for(j=0;j<i-1;j++)
		{
			if(ValidCh[j] == ValidCh[i])
			{
				ValidSatCnt--;
				break;
			}
		}
	}
	sprintf(strBuff, "%d,", ValidSatCnt);
	BUFF_AppendLittle(pBuffOut, (U1 *)strBuff, strlen((I1 *)strBuff));
	
	// 13 obs
	ObsAboveElvCnt = 0;
	for(i = 1; i <= SV_NUM; i++)		//select the min failed count sv list
	{
		if((SVInfo[i-1].el < SV_ELEVATION_VALID_TH) && (SVInfo[i-1].el>GetSVElMask(i)))
		{
			ObsAboveElvCnt++;
		}
	}
	sprintf(strBuff, "%d,", ObsAboveElvCnt);
	BUFF_AppendLittle(pBuffOut, (U1 *)strBuff, strlen((I1 *)strBuff));
	
	// 14 multi L2>el
	for(i=0;i<MAXCHANNELS_PVT;i++)
	{
		if(PVTTrkchInfo[i].freq_point == DSP_L2P_FRE && PVTTrkchInfo[i].el>GetSVElMask(i))
		{
			ObsL2AboveElvCnt++;
		}
	}
	
	sprintf(strBuff, "%d,", ObsL2AboveElvCnt);
	BUFF_AppendLittle(pBuffOut, (U1 *)strBuff, strlen((I1 *)strBuff));
		
	// 15 sol source
	sprintf(strBuff, "%x,", 0x03);
	BUFF_AppendLittle(pBuffOut, (U1 *)strBuff, strlen((I1 *)strBuff));
	
	// 16 ext sol stat
	sprintf(strBuff, "%x,", 0x10);
	BUFF_AppendLittle(pBuffOut, (U1 *)strBuff, strlen((I1 *)strBuff));
	
	// 17 Galileo & BD sig mask
	if(satB1>0)
		BDMask += 0x10;
	if(satB2>0)
		BDMask += 0x20;
	if(satB3>0)
		BDMask += 0x40;
	
	sprintf(strBuff, "%02x,", BDMask);
	BUFF_AppendLittle(pBuffOut, (U1 *)strBuff, strlen((I1 *)strBuff));
	// 18 GPS & GLONASS sig mask
	if(satL1>0)
		GPSGLONASSMask += 0x01;
	if(satL2>0)
		GPSGLONASSMask += 0x02;
	if(satL5>0)
		GPSGLONASSMask += 0x04;
	if(satG1>0)
		GPSGLONASSMask +=0x10;
	if(satG2>0)
		GPSGLONASSMask +=0x20;
	
	sprintf(strBuff, "%02x", GPSGLONASSMask);
	BUFF_AppendLittle(pBuffOut, (U1 *)strBuff, strlen((I1 *)strBuff));
	
	// 19 crc
	crc32=CalculateBlockCRC32(pBuffOut->curIdx - 1, pBuffOut->buff + 1);
	sprintf (strBuff, "*%08x", (unsigned int)crc32);
	BUFF_AppendLittle(pBuffOut, (U1 *)strBuff, strlen((I1 *)strBuff));
	
	// 20终止符 CR/LF
	strBuff[0] = 0x0D;
	strBuff[1] = 0x0A;
	BUFF_AppendLittle(pBuffOut, (U1 *)strBuff, 2);
#ifndef _POSTPROC	
	UART_Send_Buff(comid, (I1 *)pBuffOut->buff, pBuffOut->curIdx);
#endif	
}


void Uart_Send_BESTPOSB(byte comid)//debug
{
	unsigned int crc32;
	T_OUT_BUFF *pBuffOut = &g_OutBuff;
	int32 fix_result=0;
	int week,i,j,TrackSatCnt=0,ValidChCnt=0,TrackChCnt,ValidSatCnt=0;
	int sat1=0,satL1=0,satL2=0,satL5=0,satB1=0,satB2=0,satB3=0,satG1=0,satG2=0,satOther=0;
	int BDMask=0,GPSGLONASSMask = 0;
	double second,RFcnt,undulat,dSeconds,masterTow;

	double lat_deviation_m=0.0,lon_deviation_m=0.0,alt_deviation_m=0.0;
	ECEF pos={0};
	word256 Chmap={0};
	WGS des={0};
	FIX_DOP DOP ={0};
	char ValidChTrack[MAXCHANNELS_PVT],ValidCh[MAXCHANNELS_PVT];
    UINT16  Length;
	STAR_POS PosData = {0};
	BINHEAD MsgHead = {0};

//clear buffer
	BUFF_Clear(pBuffOut);
	

//1 HEADER
	
	
	// 帧头数据赋值
	MsgHead.type=42; //Bestpos
	//MsgHead.comflag=DEBUG_COM;
	MsgHead.Reserved=0;
	MsgHead.IdleTime=164;//78.5
	if((Master_observ.clkSteer_bd==1) || (Master_observ.clkSteer_gps==1) || (Master_observ.clkSteer_glo==1))
		MsgHead.TStatus=180;
	else if((Master_observ.clkSteer_bd==0) || (Master_observ.clkSteer_gps==0) || (Master_observ.clkSteer_glo==0))
		MsgHead.TStatus=100;
	else
		MsgHead.TStatus=20;

	RFcnt = getTICLockRFcnt();
	GetNavSysTimeByRFCnt(NAV_SYS_NONE,GPSTIME_SYNC_SRC_FIXRFCNT,RFcnt,&second,&week);
	MsgHead.mWeeks = (UINT16)week;
	MsgHead.nSeconds=ROUND(second*1e3+0.5);
	MsgHead.Reserved1=0;
	MsgHead.BD2LeapSec=2;
	MsgHead.Reserved2=0;

	pBuffOut->buff[0]=0xAA; //sync
	pBuffOut->buff[1]=0x44;//sync
	pBuffOut->buff[2]=0x12;//sync
	pBuffOut->buff[3]=0x1C;//header length
	pBuffOut->buff[4]=MsgHead.type&0xFF; //message id
	pBuffOut->buff[5]=(MsgHead.type>>8)&0xFF;
	pBuffOut->buff[6]=0x00;//message type
	pBuffOut->buff[7]=(comid+1)&0xFF;//Port
	//8、9位为数据长度，在后面定义
	pBuffOut->buff[10]=MsgHead.Reserved&0xFF;//Reserved
	pBuffOut->buff[11]=(MsgHead.Reserved>>8)&0xFF;
	pBuffOut->buff[12]=MsgHead.IdleTime;//IdleTime
	pBuffOut->buff[13]=MsgHead.TStatus;//Time Status
	pBuffOut->buff[14]=MsgHead.mWeeks&0xFF;//Week
	pBuffOut->buff[15]=(MsgHead.mWeeks>>8)&0xFF;
	pBuffOut->buff[16]=MsgHead.nSeconds&0xFF;//ms
	pBuffOut->buff[17]=(MsgHead.nSeconds>>8)&0xFF;
	pBuffOut->buff[18]=(MsgHead.nSeconds>>16)&0xFF;
	pBuffOut->buff[19]=(MsgHead.nSeconds>>24)&0xFF;
	pBuffOut->buff[20]=0X20; //Receiver Status
	pBuffOut->buff[21]=0X00;
	pBuffOut->buff[22]=0X10;
	pBuffOut->buff[23]=0X00;
	pBuffOut->buff[24]=0XF6;//???Reserved
	pBuffOut->buff[25]=0XB1;
	pBuffOut->buff[26]=0XC4;//Receiver S/W Version
	pBuffOut->buff[27]=0X32;
	
	pBuffOut->curIdx = 28;
	
//2 DATA

	// 2 solstat
	fix_result = getRcvrInfo(&pos,NULL,&Chmap,&DOP);

	if(fix_result == FIX_NOT)
		PosData.sol_stat = 10;//INVALID_FIX
	else if(fix_result == FIX_2D)
		PosData.sol_stat = 6;//V_H_LIMIT
	else
		PosData.sol_stat = 0;//SOL_COMPUTED

	// 3 postype
	if(fix_result == FIX_NOT)
		PosData.pos_type = 0;// NONE
	else if(fix_result <= FIX_3D)
		PosData.pos_type = 16;// SINGE
	else if(fix_result == FIX_RTD)
		PosData.pos_type = 17;// PSRDIFF
	else if(fix_result == FIX_WIDE_FLOAT)
		PosData.pos_type = 73;// WIDE_FLOAT ??
	else if(fix_result == FIX_L1_FLOAT)
		PosData.pos_type = 32;// PSRDIFFL1_FLOAT
	else if(fix_result == FIX_NARROW_FLOAT)
		PosData.pos_type = 34;//NARROW_FLOAT
	else if(fix_result == FIX_WIDE_INT)
		PosData.pos_type = 74;//WIDE_INT ???
	else if(fix_result == FIX_L1_INT)
		PosData.pos_type = 48;//L1_INT
	else if(fix_result == FIX_NARROW_INT)
		PosData.pos_type = 50;//NARROW_INT
	else
		PosData.pos_type = 32;//UNKOWN

	//chmap bit 1 position
	ECEF2WGS(&pos,&des);
	// 4 lat 纬度
	PosData.lat = des.lat*R2D ;//(degree)
	// 5 lon 经度
	PosData.lon = des.lon*R2D ;//(degree)
	// 6 hgt
	PosData.hgt = des.alt;//(m)
	// 7 undulation
	undulat = 0.0;
	PosData.undulation = undulat;
	// 8 datum id#
	PosData.datum = 0;
	// 9 lat deviation
	PosData.lat_delta =  lat_deviation_m;
	// 10 LON deviation
	PosData.lon_delta =  lon_deviation_m;
	// 11 hgt deviation
	PosData.hgt_delta =  alt_deviation_m;
	// 12 stn id
	//Master_observ.staid???
	memset(PosData.base_id,0,4);
	// 13 diff age
	if (fix_result <= FIX_3D)
		dSeconds = 0.0;
	else if(GetObsvTime(&Master_obs, &masterTow, NAV_SYS_NONE))
	{
		dSeconds = GetRTKExtraTime(NAV_SYS_NONE);
		//dSeconds = masterTow - extraobstow;
	}
	else
		dSeconds = 0.0;

	PosData.diff_age = dSeconds;
	
	// 14 sol age
	PosData.sol_age = 0.0;
	// 15 Number of satellites tracked ???????where
	TrackChCnt = 0;
	for(i=0;i<MAXCHANNELS_PVT;i++)
	{
		if(PVTTrkchInfo[i].CCBF & 0x3)
		{
			ValidChTrack[TrackChCnt++]=PVTTrkchInfo[i].svid;
		}
	}
	TrackSatCnt = TrackChCnt;
	for(i=1;i<TrackChCnt;i++)
	{
		for(j=0;j<i-1;j++)
		{
			if(ValidChTrack[j] == ValidChTrack[i])
			{
				TrackSatCnt--;
				break;
			}
		}
	}
	
	PosData.trac_num = TrackSatCnt;
	//16 Number of satellites used in solution ??????
	ValidChCnt = 0;
	sat1 = 0;

	for(i=0;i<MAXCHANNELS_PVT;i++)
	{
		if(0 == GetBitWord256(&Chmap,i))
			continue;
		
		ValidCh[ValidChCnt++] = PVTTrkchInfo[i].svid;
		
		if(PVTTrkchInfo[i].freq_point == DSP_L1CA_FRE)
			satL1++;
		else if(PVTTrkchInfo[i].freq_point == DSP_L2P_FRE)
			satL2++;
		else if(PVTTrkchInfo[i].freq_point == DSP_L5_FRE)
			satL5++;
		else if(PVTTrkchInfo[i].freq_point == DSP_B1I_FRE)
			satB1++;
		else if(PVTTrkchInfo[i].freq_point == DSP_B2I_FRE)
			satB2++;
		else if(PVTTrkchInfo[i].freq_point == DSP_B3I_FRE)
			satB3++;
		else if(PVTTrkchInfo[i].freq_point == DSP_G1CA_FRE)
			satG1++;
		else if(PVTTrkchInfo[i].freq_point == DSP_G2CA_FRE)
			satG1++;
		else
			satOther++;
	}
	
	sat1 = satL1 + satB1 + satG1;
	//sat2 = satL2 + satL5 + satB2 + satB3 + satG2 + satOther;

	ValidSatCnt = ValidChCnt;
	for(i=1;i<ValidChCnt;i++)
	{
		for(j=0;j<i-1;j++)
		{
			if(ValidCh[j] == ValidCh[i])
			{
				ValidSatCnt--;
				break;
			}
		}
	}
	PosData.used_num = ValidSatCnt;
	//17 Number of satellites with L1/E1/B1 signals used in solution ??????
	PosData.L1_num = sat1;
	//18 Number of satellites with multi-frequency signals used in solution ??????
	PosData.muti_num = sat1;//
	//19 Reserved
	PosData.reserved = 0x0;
	//20 ext sol stat ??????
	PosData.ext_sol_stat = 0x10;
	//21 Galileo and BeiDou sig mask
	if(satB1>0)
		BDMask += 0x10;
	if(satB2>0)
		BDMask += 0x20;
	if(satB3>0)
		BDMask += 0x40;
	
	PosData.Ga_BD_sin = BDMask;
	//22 GPS and GLONASS sig mask
	if(satL1>0)
		GPSGLONASSMask += 0x01;
	if(satL2>0)
		GPSGLONASSMask += 0x02;
	if(satL5>0)
		GPSGLONASSMask += 0x04;
	if(satG1>0)
		GPSGLONASSMask +=0x10;
	if(satG2>0)
		GPSGLONASSMask +=0x20;
	
	PosData.Ga_BD_sin = GPSGLONASSMask;
	
	memcpy(&pBuffOut->buff[pBuffOut->curIdx],&PosData.sol_stat, 4*2);
	pBuffOut->curIdx+=4*2;
	memcpy(&pBuffOut->buff[pBuffOut->curIdx],&PosData.lat, 8*3);
	pBuffOut->curIdx+=8*3;
	memcpy(&pBuffOut->buff[pBuffOut->curIdx],&PosData.undulation, 4);
	pBuffOut->curIdx+=4;
	memcpy(&pBuffOut->buff[pBuffOut->curIdx],&PosData.datum, 4);
	pBuffOut->curIdx+=4;
	memcpy(&pBuffOut->buff[pBuffOut->curIdx],&PosData.lat_delta, 4*3);
	pBuffOut->curIdx+=4*3;
	memcpy(&pBuffOut->buff[pBuffOut->curIdx],PosData.base_id, 4);
	pBuffOut->curIdx+=4;
	memcpy(&pBuffOut->buff[pBuffOut->curIdx],&PosData.diff_age, 4*2);
	pBuffOut->curIdx+=4*2;
	memcpy(&pBuffOut->buff[pBuffOut->curIdx],&PosData.trac_num, 1*8);
	pBuffOut->curIdx+=1*8;

	//length
	Length=pBuffOut->curIdx-28;
	pBuffOut->buff[8]=Length&0xFF;
	pBuffOut->buff[9]=(Length>>8)&0xFF;
	//23 crc
	crc32=CalculateBlockCRC32(pBuffOut->curIdx, pBuffOut->buff);
	
	pBuffOut->buff[pBuffOut->curIdx]=crc32&0xFF;
	pBuffOut->buff[pBuffOut->curIdx+1]=(crc32>>8)&0xFF;
	pBuffOut->buff[pBuffOut->curIdx+2]=(crc32>>16)&0xFF;
	pBuffOut->buff[pBuffOut->curIdx+3]=(crc32>>24)&0xFF;
	pBuffOut->curIdx+=4; 
	
	// 24终止符 CR/LF
//	pBuffOut->buff[pBuffOut->curIdx] = 0x0D;
//	pBuffOut->buff[pBuffOut->curIdx+1] = 0x0A;
//	pBuffOut->curIdx+=2; 
#ifndef _POSTPROC
	UART_Send_Buff(comid, (I1 *)pBuffOut->buff, pBuffOut->curIdx);
#endif	
}


void Uart_Send_RangeB(byte comid)
{

	unsigned int crc32;
	T_OUT_BUFF *pBuffOut = &g_OutBuff;
	int32 svidx, freqidx=0;
	int week;
	double second,RFcnt;
	int satcount=0;//,SatCntIndex;
    UINT16  Length;
	STAR_PR ProData ;
	BINHEAD MsgHead ;

//clear buffer
	BUFF_Clear(pBuffOut);
	memset(&ProData,0,sizeof(STAR_PR));
	memset(&MsgHead,0,sizeof(BINHEAD));

//1 HEADER
	
	
	// 帧头数据赋值
	MsgHead.type=43; //Range
	//MsgHead.comflag=DEBUG_COM;
	MsgHead.Reserved=0;
	MsgHead.IdleTime=164;//78.5
	if((Master_observ.clkSteer_bd==1) || (Master_observ.clkSteer_gps==1) || (Master_observ.clkSteer_glo==1))
		MsgHead.TStatus=180;
	else if((Master_observ.clkSteer_bd==0) || (Master_observ.clkSteer_gps==0) || (Master_observ.clkSteer_glo==0))
		MsgHead.TStatus=100;
	else
		MsgHead.TStatus=20;

	RFcnt = getTICLockRFcnt();
	GetNavSysTimeByRFCnt(NAV_SYS_NONE,GPSTIME_SYNC_SRC_FIXRFCNT,RFcnt,&second,&week);
	MsgHead.mWeeks = (UINT16)week;
	MsgHead.nSeconds=ROUND(second*1e3+0.5);
	MsgHead.Reserved1=0;
	MsgHead.BD2LeapSec=2;
	MsgHead.Reserved2=0;

	pBuffOut->buff[0]=0xAA; //sync
	pBuffOut->buff[1]=0x44;//sync
	pBuffOut->buff[2]=0x12;//sync
	pBuffOut->buff[3]=0x1C;//header length
	pBuffOut->buff[4]=MsgHead.type&0xFF; //message id
	pBuffOut->buff[5]=(MsgHead.type>>8)&0xFF;
	pBuffOut->buff[6]=0x00;//message type
	pBuffOut->buff[7]=(comid+1)&0xFF;//Port
	//8、9位为数据长度，在后面定义
	pBuffOut->buff[10]=MsgHead.Reserved&0xFF;//Reserved
	pBuffOut->buff[11]=(MsgHead.Reserved>>8)&0xFF;
	pBuffOut->buff[12]=MsgHead.IdleTime;//IdleTime
	pBuffOut->buff[13]=MsgHead.TStatus;//Time Status
	pBuffOut->buff[14]=MsgHead.mWeeks&0xFF;//Week
	pBuffOut->buff[15]=(MsgHead.mWeeks>>8)&0xFF;
	pBuffOut->buff[16]=MsgHead.nSeconds&0xFF;//ms
	pBuffOut->buff[17]=(MsgHead.nSeconds>>8)&0xFF;
	pBuffOut->buff[18]=(MsgHead.nSeconds>>16)&0xFF;
	pBuffOut->buff[19]=(MsgHead.nSeconds>>24)&0xFF;
	pBuffOut->buff[20]=0X20; //Receiver Status
	pBuffOut->buff[21]=0X00;
	pBuffOut->buff[22]=0X10;
	pBuffOut->buff[23]=0X00;
	pBuffOut->buff[24]=0XF6;//???Reserved
	pBuffOut->buff[25]=0XB1;
	pBuffOut->buff[26]=0XC4;//Receiver S/W Version
	pBuffOut->buff[27]=0X32;
	
	pBuffOut->curIdx = 28;
	
//2 DATA

	for(svidx = 0; svidx<Master_observ.satnmax; svidx++)
	{
		for(freqidx=0; freqidx<MAX_FREPIONT_PER_NAVSYS; freqidx++)
		{
			if(Master_observ.obs[svidx].validmap[freqidx] !=0)
				satcount++;
		}
	}
	ProData.obs = satcount;
	memcpy(&pBuffOut->buff[28],&ProData.obs, 4);
	pBuffOut->curIdx +=4;
	
	satcount=0;
	for(svidx = 0; svidx<Master_observ.satnmax; svidx++)
	{
		for(freqidx=0; freqidx<MAX_FREPIONT_PER_NAVSYS; freqidx++)
		{
			if(Master_observ.obs[svidx].validmap[freqidx] == 0)
				continue;

			satcount++;
			//svid
			ProData.PRN = Master_observ.obs[svidx].StarID;
			
			//glo freq
#if SUPPORT_GLONASS
			if(Master_observ.obs[svidx].StarID>=MinGloFreID)	
				ProData.glofreq = Master_observ.obs[svidx].slotID-MinGloFreID;
			else
#endif
				ProData.glofreq = 0;
	
			//PR
			ProData.psr = Master_observ.obs[svidx].range[freqidx];

			//pr standard deviation
			ProData.psr_std = 0.02;
			
			//Carrier phase,
			ProData.CarrierPhase = Master_observ.obs[svidx].phase[freqidx];
			
			//Carrier phase standard deviation
			ProData.CarrierPhase_std = 0.005;
			
			//doppler
			ProData.Dopple = Master_observ.obs[svidx].doppler[freqidx];
			
			//cn0
	        ProData.dSnr = Master_observ.obs[svidx].snr0[freqidx];
	       
			//locktime
	        ProData.Locktime = Master_observ.obs[svidx].LLI[freqidx];//????
			
			ProData.Channel_State = 0;
			//status
			ProData.Channel_State |= ( satcount & 0x1f)<<5;//channel number
			ProData.Channel_State |= 0x00000400;//phase lock
			ProData.Channel_State |= 0x00000800;//parity known
			ProData.Channel_State |= 0x00001000;//code lock
			
			//if(Master_observ.obs[svidx].bUsedMap[freqidx]!=0)
			ProData.Channel_State |= 1<<29;	//used in solution

			if(Master_observ.obs[svidx].code[freqidx] & FREQ_GROUP_L2)
			{
				ProData.Channel_State |= 0x00100000;//grouping
				if(Master_observ.obs[svidx].code[freqidx]==DSP_L2P_FRE)
					ProData.Channel_State |= 5<<21;//gps L2P 
				else
					ProData.Channel_State |= 9<<21;//gps L2P codeless
				
				ProData.Channel_State |= 0x0000000b;//tracking state:phase lock loop
				ProData.Channel_State |= 0x00002000;//Standard correlator: spacing = 1 chip
			}
			else if(Master_observ.obs[svidx].code[freqidx] & FREQ_GROUP_L1)
			{
				ProData.Channel_State |= 0x00100000;//grouping
				ProData.Channel_State |= 0x08000000;//L1 primary
				ProData.Channel_State |= 0x00000004;//tracking state:phase lock loop
				ProData.Channel_State |= 0x00008000;//correlator state:Pulse Aperture Correlator
			}
			else if(Master_observ.obs[svidx].code[freqidx] & FREQ_GROUP_L5)
			{
				ProData.Channel_State |= 0x00100000;//grouping
				ProData.Channel_State |= 14<<21;//gps L5Q

				ProData.Channel_State |= 0x00000004;//tracking state:phase lock loop
				ProData.Channel_State |= 0x00008000;//correlator state:Pulse Aperture Correlator
			}
			else if(Master_observ.obs[svidx].code[freqidx] & FREQ_GROUP_B1)
			{
				ProData.Channel_State |= 0x00040000;//Satellite system
				ProData.Channel_State |= 0x08000000;//L1 channel primary
				if(SV_IsBd2Meo(Master_observ.obs[svidx].StarID))//meo D1 data
					ProData.Channel_State |= 0<<21;
				else//geo D2 data
					ProData.Channel_State |= 4ul<<21;
			}
			else if(Master_observ.obs[svidx].code[freqidx] & FREQ_GROUP_B2)
			{
				ProData.Channel_State |= 0x00040000;//Satellite system
				if(SV_IsBd2Meo(Master_observ.obs[svidx].StarID))//meo D1 data
					ProData.Channel_State |= 1ul<<21;
				else//geo
					ProData.Channel_State |= 5ul<<21;
			}
			else if(Master_observ.obs[svidx].code[freqidx] & FREQ_GROUP_B3)
			{
				ProData.Channel_State |= 0x00040000;//Satellite system
				if(SV_IsBd2Meo(Master_observ.obs[svidx].StarID))//meo D1 data
					ProData.Channel_State |= 2ul<<21;
				else//geo
					ProData.Channel_State |= 6ul<<21;
			}
			else if(Master_observ.obs[svidx].code[freqidx] & FREQ_GROUP_G1)//G1
			{
				ProData.Channel_State |= 0x00010000;//Satellite system
			}
			else if(Master_observ.obs[svidx].code[freqidx] & FREQ_GROUP_G2)//G2
			{
				ProData.Channel_State |= 0x00010000;//Satellite system
				ProData.Channel_State |= 1ul<<21;//L2 CA
			}
			else
			{
				ProData.Channel_State |= 0x00010000;
			}
	
			memcpy(&pBuffOut->buff[pBuffOut->curIdx],&ProData.PRN, 2*2);
			pBuffOut->curIdx+=2*2;
			memcpy(&pBuffOut->buff[pBuffOut->curIdx],&ProData.psr, 8);
			pBuffOut->curIdx+=8;
			memcpy(&pBuffOut->buff[pBuffOut->curIdx],&ProData.psr_std, 4);
			pBuffOut->curIdx+=4;
			memcpy(&pBuffOut->buff[pBuffOut->curIdx],&ProData.CarrierPhase, 8);
			pBuffOut->curIdx+=8;
			memcpy(&pBuffOut->buff[pBuffOut->curIdx],&ProData.CarrierPhase_std, 4*4);
			pBuffOut->curIdx+=4*4;
			memcpy(&pBuffOut->buff[pBuffOut->curIdx],&ProData.Channel_State, 4);
			pBuffOut->curIdx+=4;
		}
	}
	
	
	
	
	//23 crc

	Length=pBuffOut->curIdx-28;
	pBuffOut->buff[8]=Length&0xFF;
	pBuffOut->buff[9]=(Length>>8)&0xFF;
	
	crc32=CalculateBlockCRC32(pBuffOut->curIdx, pBuffOut->buff);
	
	pBuffOut->buff[pBuffOut->curIdx]=crc32&0xFF;
	pBuffOut->buff[pBuffOut->curIdx+1]=(crc32>>8)&0xFF;
	pBuffOut->buff[pBuffOut->curIdx+2]=(crc32>>16)&0xFF;
	pBuffOut->buff[pBuffOut->curIdx+3]=(crc32>>24)&0xFF;
	pBuffOut->curIdx+=4; 
	
	// 24终止符 CR/LF
//	pBuffOut->buff[pBuffOut->curIdx] = 0x0D;
//	pBuffOut->buff[pBuffOut->curIdx+1] = 0x0A;
//	pBuffOut->curIdx+=2; 
#ifndef _POSTPROC
	UART_Send_Buff(comid, (I1 *)pBuffOut->buff, pBuffOut->curIdx);
#endif	
}


void Uart_Send_PsrdopB(byte comid)
{
	unsigned int crc32;
	T_OUT_BUFF *pBuffOut = &g_OutBuff;
	int32 TrackChCnt, TrackSatCnt=0;
	int week,i,j=0;
	double second,RFcnt;
    UINT16  Length;
	int ValidChTrack[MAXCHANNELS_PVT];
	int SVTrack[MAXCHANNELS_PVT] = {0};
	STAR_DOP ProData = {0};
	BINHEAD MsgHead = {0};
	FIX_DOP DOPinfo;

//clear buffer
	BUFF_Clear(pBuffOut);
	memset(&DOPinfo,0,sizeof(FIX_DOP));

//1 HEADER
	
	
	// 帧头数据赋值
	MsgHead.type=174; //PSRDOP
	//MsgHead.comflag=DEBUG_COM;
	MsgHead.Reserved=0;
	MsgHead.IdleTime=164;//78.5
	if((Master_observ.clkSteer_bd==1) || (Master_observ.clkSteer_gps==1) || (Master_observ.clkSteer_glo==1))
		MsgHead.TStatus=180;
	else if((Master_observ.clkSteer_bd==0) || (Master_observ.clkSteer_gps==0) || (Master_observ.clkSteer_glo==0))
		MsgHead.TStatus=100;
	else
		MsgHead.TStatus=20;

	RFcnt = getTICLockRFcnt();
	GetNavSysTimeByRFCnt(NAV_SYS_NONE,GPSTIME_SYNC_SRC_FIXRFCNT,RFcnt,&second,&week);
	MsgHead.mWeeks = (UINT16)week;
	MsgHead.nSeconds=ROUND(second*1e3+0.5);
	MsgHead.Reserved1=0;
	MsgHead.BD2LeapSec=2;
	MsgHead.Reserved2=0;

	pBuffOut->buff[0]=0xAA; //sync
	pBuffOut->buff[1]=0x44;//sync
	pBuffOut->buff[2]=0x12;//sync
	pBuffOut->buff[3]=0x1C;//header length
	pBuffOut->buff[4]=MsgHead.type&0xFF; //message id
	pBuffOut->buff[5]=(MsgHead.type>>8)&0xFF;
	pBuffOut->buff[6]=0x00;//message type
	pBuffOut->buff[7]=(comid+1)&0xFF;//Port
	//8、9位为数据长度，在后面定义
	pBuffOut->buff[10]=MsgHead.Reserved&0xFF;//Reserved
	pBuffOut->buff[11]=(MsgHead.Reserved>>8)&0xFF;
	pBuffOut->buff[12]=MsgHead.IdleTime;//IdleTime
	pBuffOut->buff[13]=MsgHead.TStatus;//Time Status
	pBuffOut->buff[14]=MsgHead.mWeeks&0xFF;//Week
	pBuffOut->buff[15]=(MsgHead.mWeeks>>8)&0xFF;
	pBuffOut->buff[16]=MsgHead.nSeconds&0xFF;//ms
	pBuffOut->buff[17]=(MsgHead.nSeconds>>8)&0xFF;
	pBuffOut->buff[18]=(MsgHead.nSeconds>>16)&0xFF;
	pBuffOut->buff[19]=(MsgHead.nSeconds>>24)&0xFF;
	pBuffOut->buff[20]=0X20; //Receiver Status
	pBuffOut->buff[21]=0X00;
	pBuffOut->buff[22]=0X10;
	pBuffOut->buff[23]=0X00;
	pBuffOut->buff[24]=0XF6;//???Reserved
	pBuffOut->buff[25]=0XB1;
	pBuffOut->buff[26]=0XC4;//Receiver S/W Version
	pBuffOut->buff[27]=0X32;
	
	pBuffOut->curIdx = 28;
	
//2 DATA

	getRcvrInfo(NULL, NULL, NULL, &DOPinfo);
	ProData.gdop = DOPinfo.gdop;
	ProData.pdop = DOPinfo.pdop;
	ProData.hdop = DOPinfo.hdop;
	ProData.htdop= 0.0;
	ProData.tdop = 0.0;
	ProData.cutoff = (pActiveCPT->SysmCptWorkConfig.ElvMaskRTKGPS)/1.0;
	
	TrackChCnt = 0;
	for(i=0;i<MAXCHANNELS_PVT;i++)
	{
		if(PVTTrkchInfo[i].CCBF & 0x3)
		{
			ValidChTrack[TrackChCnt++]=PVTTrkchInfo[i].svid;
		}
	}
	
	for(i=0;i<TrackChCnt;i++)//排除重复算法
	{
		for(j = i+1;j<TrackChCnt;j++)
		{
			if(ValidChTrack[i] == ValidChTrack[j])
			{
				ValidChTrack[i] = -1; //Repeat SvNumber
				break;
			}
		}
	}
	for(i=0,j=0;i<TrackChCnt;i++)
	{	
		if(ValidChTrack[i] == -1)
			continue;
		SVTrack[j++] = ValidChTrack[i];
	}
	TrackSatCnt = j;
	ProData.obs = TrackSatCnt;
	
	memcpy(&pBuffOut->buff[pBuffOut->curIdx],&ProData.gdop, 4);
	pBuffOut->curIdx+=4;
	memcpy(&pBuffOut->buff[pBuffOut->curIdx],&ProData.pdop, 4);
	pBuffOut->curIdx+=4;
	memcpy(&pBuffOut->buff[pBuffOut->curIdx],&ProData.hdop, 4);
	pBuffOut->curIdx+=4;
	memcpy(&pBuffOut->buff[pBuffOut->curIdx],&ProData.htdop, 4);
	pBuffOut->curIdx+=4;
	memcpy(&pBuffOut->buff[pBuffOut->curIdx],&ProData.tdop, 4);
	pBuffOut->curIdx+=4;
	memcpy(&pBuffOut->buff[pBuffOut->curIdx],&ProData.cutoff, 4);
	pBuffOut->curIdx+=4;
	memcpy(&pBuffOut->buff[pBuffOut->curIdx],&ProData.obs, 4);
	pBuffOut->curIdx+=4;
	
	for(i=0;i<TrackSatCnt;i++)
	{
		memcpy(&pBuffOut->buff[(pBuffOut->curIdx)],&SVTrack[i], 4);
		pBuffOut->curIdx+=4;
	}
	
	//23 crc

	Length=pBuffOut->curIdx-28;
	pBuffOut->buff[8]=Length&0xFF;
	pBuffOut->buff[9]=(Length>>8)&0xFF;
	
	crc32=CalculateBlockCRC32(pBuffOut->curIdx, pBuffOut->buff);
	
	pBuffOut->buff[pBuffOut->curIdx]=crc32&0xFF;
	pBuffOut->buff[pBuffOut->curIdx+1]=(crc32>>8)&0xFF;
	pBuffOut->buff[pBuffOut->curIdx+2]=(crc32>>16)&0xFF;
	pBuffOut->buff[pBuffOut->curIdx+3]=(crc32>>24)&0xFF;
	pBuffOut->curIdx+=4; 
	
	// 24终止符 CR/LF
//	pBuffOut->buff[pBuffOut->curIdx] = 0x0D;
//	pBuffOut->buff[pBuffOut->curIdx+1] = 0x0A;
//	pBuffOut->curIdx+=2; 
#ifndef _POSTPROC
	UART_Send_Buff(comid, (I1 *)pBuffOut->buff, pBuffOut->curIdx);
#endif	
}




