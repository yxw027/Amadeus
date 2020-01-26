/**
* @file  rtcm3handle.c
*
* @brief 
*		This module is used to create or use the message items, in RTCM 3.x messages
*		
*		created by juan. 2013.July.8.
*/

#include "define.h"
#include "coordinate.h"
#include "rtcm3.h"
#include "rtk.h"
#include "cfgpara.h"
#include "constdef.h"
#include "CalcSvPVT.h"

#ifdef _SIMULATE
#include "dataprocess.h"
#endif

static byte FormatRTCM3CSCIntervalIdx(word16 sec);
static byte FormatRTCM3LockTimeIndicate(word32 locktime);
static UINT32 GetRTCM3LockTimeByIndicate(byte indicate);
static word32 GetMSMLockTimeIndicateEx(word16 indicate);

static bool GetRTCM3Msg1001To1004HeadInfo(int32 svcnt, RTCM3_MSG_1001_1004_HEAD* pHeadInfo, OBSEV* pObsv);
static int32 GetRTCM3Msg1004DataInfo(RTCM3_MSG_1004_DATA* pDataInfo, OBSEV* pObsv);
static void SetRTCM3Msg1001To1004HeadInfo(RTCM3_MSG_1001_1004_HEAD* pHeadInfo, OBSEV* pObsv);
static void SetRTCM3Msg1004DataInfo(int32 svcnt, RTCM3_MSG_1004_DATA* pDataInfo, OBSEV* pObsv);
#if SUPPORT_GLONASS 
static void SetRTCM3Msg1009To1012HeadInfo(RTCM3_MSG_1009_1012_HEAD* pHeadInfo, OBSEV* pObsv);
static void SetRTCM3Msg1012DataInfo(int32 svcnt, RTCM3_MSG_1012_DATA* pDataInfo, OBSEV* pObsv);
static int32 GetRTCM3Msg1012DataInfo(RTCM3_MSG_1012_DATA* pDataInfo, OBSEV* pObsv);
static bool GetRTCM3Msg1009To1012HeadInfo(int32 svcnt, RTCM3_MSG_1009_1012_HEAD* pHeadInfo, OBSEV* pObsv);
#endif

//MSM
static bool GetRTCM3MSMHeadInfo(RTCM3_MSM_HEAD_INFO* pHeadInfo, byte navsys, boolean multiMsgFlag, OBSEV* pObsv);
static void ClearMSMDataInfo(RTCM3_MSM_DATA_INFO* pMSMDataInfo);
static boolean IsMoreMSMMsgFollowed(byte comid);
static byte FormatMSMLockTimeIndicate(word32 locktime);
static word16 FormatMSMLockTimeIndicateEx(word32 locktime);
static void SetMSMDataInfo(byte navsys, RTCM3_MSM_HEAD_INFO* pHead, RTCM3_MSM_MAP* pMap, RTCM3_MSM_DATA_INFO* pMSMDataInfo, OBSEV* pObsv,STRU_UART_STREAM *glStruUart);


RTCM3_MSM_DATA_INFO MSMDataInfo;
static RTCM3_MSM_MAP MSMDataMap_GPS[RTCM3_MSM_MAX_PAGECNT];
static RTCM3_MSM_MAP MSMDataMap_BD[RTCM3_MSM_MAX_PAGECNT];
#if SUPPORT_GLONASS
static RTCM3_MSM_MAP MSMDataMap_GLO[RTCM3_MSM_MAX_PAGECNT];
#endif
static int32 CurMSMCnt2TX[MAX_UART_NUM] = {0,0,0};
static int32 TotalMSMCnt2TX[MAX_UART_NUM] = {0,0,0};
bool bRTCMMsgRXFinish = FALSE;

/********************************Receive Handle***********************************/
/**
* @brief the receive handler for RTCM3 message 1001: GPS RTK
* @para: p_data - the configured data buffer, point to the head of this message.
*		data_len - message total length 
*/
void handle_RTCM3_RX_MSG_1001(word16 msgid,  const byte * const p_data, word16 data_len, STRU_UART_STREAM *glStruUart)
{
#if RTCM3_RX_ENABLE
	RTCM3_MSG_1001_1004_HEAD Msg1001HeadInfo;
	RTCM3_MSG_1001_DATA Msg1001DataInfo[GPS_SV_NUM];
	word16 repeatcnt=0;

	ParseRTCM3VarLenItems(msgid, (byte*)p_data, data_len, &Msg1001HeadInfo, Msg1001DataInfo, &repeatcnt);
#endif
	//Configure data to use
	return;
}

/**
* @brief the receive handler for RTCM3 message 1002
* @para: p_data - the configured data buffer, point to the head of this message.
*		data_len - message total length 
*/
void handle_RTCM3_RX_MSG_1002(word16 msgid,  const byte * const p_data, word16 data_len, STRU_UART_STREAM *glStruUart)
{
#if RTCM3_RX_ENABLE
	RTCM3_MSG_1001_1004_HEAD Msg1002HeadInfo;
	RTCM3_MSG_1002_DATA Msg1002DataInfo[GPS_SV_NUM];
	word16 repeatcnt=0;

	ParseRTCM3VarLenItems(msgid, (byte*)p_data, data_len, &Msg1002HeadInfo, Msg1002DataInfo, &repeatcnt);
#endif
	//Configure data to use
	return;
}

/**
* @brief the receive handler for RTCM3 message 1003
* @para: p_data - the configured data buffer, point to the head of this message.
*		data_len - message total length 
*/
void handle_RTCM3_RX_MSG_1003(word16 msgid,  const byte * const p_data, word16 data_len, STRU_UART_STREAM *glStruUart)
{
#if RTCM3_RX_ENABLE
	RTCM3_MSG_1001_1004_HEAD Msg1003HeadInfo;
	RTCM3_MSG_1003_DATA Msg1003DataInfo[GPS_SV_NUM];
	word16 repeatcnt=0;

	ParseRTCM3VarLenItems(msgid, (byte*)p_data, data_len, &Msg1003HeadInfo, Msg1003DataInfo, &repeatcnt);

	//Configure data to use
#endif
	return;
}

/**
* @brief the receive handler for RTCM3 message 1004
* @para: p_data - the configured data buffer, point to the head of this message.
*		data_len - message total length 
*/
void handle_RTCM3_RX_MSG_1004(word16 msgid,  const byte * const p_data, word16 data_len, STRU_UART_STREAM *glStruUart)
{
#if RTCM3_RX_ENABLE
	RTCM3_MSG_1001_1004_HEAD Msg1004HeadInfo;
	RTCM3_MSG_1004_DATA Msg1004DataInfo[GPS_SV_NUM];
	OBSEV* pSlaveBuf=NULL;
	word16 repeatcnt=0;
	
	memset(&Msg1004HeadInfo, 0, sizeof(Msg1004HeadInfo));
	memset(Msg1004DataInfo, 0, sizeof(Msg1004DataInfo));

	ParseRTCM3VarLenItems(msgid, (byte*)p_data, data_len, &Msg1004HeadInfo, Msg1004DataInfo, &repeatcnt);
	Msg1004HeadInfo.SVCnt = repeatcnt;

	//Configure data to use
	pSlaveBuf = GetCurSlaveBufAddr(NAV_SYS_GPS, ((double)Msg1004HeadInfo.TOW)/1000.0);
	SetRTCM3Msg1001To1004HeadInfo(&Msg1004HeadInfo, pSlaveBuf);
	SetRTCM3Msg1004DataInfo(Msg1004HeadInfo.SVCnt, Msg1004DataInfo, pSlaveBuf);
#endif
	return;
}


/**
* @brief the receive handler for RTCM3 message 1005: station information
* @para: p_data - the configured data buffer, point to the head of this message.
*		data_len - message total length 
*/
void handle_RTCM3_RX_MSG_1005(word16 msgid,  const byte * const p_data, word16 data_len, STRU_UART_STREAM *glStruUart)
{
#if RTCM3_RX_ENABLE
	ECEF pos;
	RTCM3_MSG_1005_INFO Msg1005Info;

	//Parse item from data
	ParseRTCM3FixLenItems(msgid, (byte*)p_data, data_len, &Msg1005Info);

	//use
	pos.x = Msg1005Info.RefPox_x*(1e-4);
	pos.y = Msg1005Info.RefPox_y*(1e-4);
	pos.z = Msg1005Info.RefPox_z*(1e-4);
	
	SetBaseStationInfoForRove(Msg1005Info.ReferStationID, 0, &pos);
#endif
	return;
}

/**
* @brief the receive handler for RTCM3 message 1006: station information
* @para: p_data - the configured data buffer, point to the head of this message.
*		data_len - message total length 
*/
void handle_RTCM3_RX_MSG_1006(word16 msgid,  const byte * const p_data, word16 data_len, STRU_UART_STREAM *glStruUart)
{
#if RTCM3_RX_ENABLE
	ECEF pos;
	float64 height;
	RTCM3_MSG_1006_INFO Msg1006Info;

	//Parse item from data
	ParseRTCM3FixLenItems(msgid, (byte*)p_data, data_len, &Msg1006Info);

	//use
	pos.x = Msg1006Info.RefPox_x*(1e-4);
	pos.y = Msg1006Info.RefPox_y*(1e-4);
	pos.z = Msg1006Info.RefPox_z*(1e-4);
	height = ((float64)Msg1006Info.AntHeight)*(1e-4);
#ifndef _POSTPROC
	SetBaseStationInfoForRove(Msg1006Info.ReferStationID, height, &pos);
#else
	SendMessgePOS();
	if(glStruUart->comid == 1)//BASE
	{
		SetBaseStationInfoForRove(Msg1006Info.ReferStationID, height, &pos);
	}
	else//ROVE
	{
		return;
	}
#endif	

#endif
	return;
}

void handle_RTCM3_RX_MSG_1012(word16 msgid,  const byte * const p_data, word16 data_len, STRU_UART_STREAM *glStruUart)
{
#if SUPPORT_GLONASS
#if RTCM3_RX_ENABLE
	RTCM3_MSG_1009_1012_HEAD Msg1012HeadInfo;
	RTCM3_MSG_1012_DATA Msg1012DataInfo[FRE_NUM_GLO]; 
	OBSEV* pSlaveBuf=NULL;
	word16 repeatcnt=0;
	
	memset(&Msg1012HeadInfo, 0, sizeof(Msg1012HeadInfo));
	memset(Msg1012DataInfo, 0, sizeof(Msg1012DataInfo));

	ParseRTCM3VarLenItems(msgid, (byte*)p_data, data_len, &Msg1012HeadInfo, Msg1012DataInfo, &repeatcnt);
	Msg1012HeadInfo.SVCnt = repeatcnt;

	//Configure data to use
	pSlaveBuf = GetCurSlaveBufAddr(NAV_SYS_GLO, ((double)Msg1012HeadInfo.TOW)/1000.0);
	SetRTCM3Msg1009To1012HeadInfo(&Msg1012HeadInfo, pSlaveBuf);
	SetRTCM3Msg1012DataInfo(Msg1012HeadInfo.SVCnt, Msg1012DataInfo, pSlaveBuf);
#endif
#endif
	return;
}

void handle_RTCM3_RX_MSG_1019(word16 msgid,  const byte * const p_data, word16 data_len, STRU_UART_STREAM *glStruUart)
{
	GPSBD2EphFrameStruct Msg1019Info;
	EphemerisBits ephitem;
	double ephage=0.0;
	int32 wn=0;
	double sec_of_week=0.0;
#ifdef _POSTPROC
	int32 toe_next_eph=0, wn_next_eph=0;
#endif

	//Parse item from data
	ParseRTCM3FixLenItems(msgid, (byte*)p_data, data_len, &ephitem);
	ParseGPSEphBits2EphInfo(&ephitem,&Msg1019Info);

#ifndef _POSTPROC
	if(!GetCurNavSysTime(NAV_SYS_NONE, GPSTIME_SYNC_SRC_FIXRFCNT, &sec_of_week, &wn))
		return;
#endif

	if(SVInfo[ephitem.svid-1].eph_age < 0)
		return;

	Msg1019Info.svid = ephitem.svid;
	memcpy(&(Sys3AlmEphUTCInfo.GpsBd2_EphStruct[ephitem.svid-1]), &Msg1019Info, sizeof(GPSBD2EphFrameStruct));
	Sys3AlmEphUTCInfo.GpsBd2_EphStruct[ephitem.svid-1].vflg = 1;
	if(Sys3AlmEphUTCInfo.GpsBd2_EphStruct[ephitem.svid-1].wkn< 800)
		Sys3AlmEphUTCInfo.GpsBd2_EphStruct[ephitem.svid-1].wkn += 2048;
	else
		Sys3AlmEphUTCInfo.GpsBd2_EphStruct[ephitem.svid-1].wkn += 1024;

	SVInfo[ephitem.svid-1].ephsrc = EPHALM_SRC_AGNSS;

#ifdef _POSTPROC
	ephage = 1;
	if(SV_IsGps(ephitem.svid)||SV_IsBd2(ephitem.svid))
	{
		if(Sys3AlmEphUTCInfo.GpsBd2_EphStruct[ephitem.svid-MinGpsSvID].vflg==FALSE)
			return SV_EPH_ALM_AGE_INVALID;

		toe_next_eph = Sys3AlmEphUTCInfo.GpsBd2_EphStruct[ephitem.svid-MinGpsSvID].toe;
		wn_next_eph = Sys3AlmEphUTCInfo.GpsBd2_EphStruct[ephitem.svid-MinGpsSvID].wkn;

		if(SV_IsGps(ephitem.svid))
		{
			if (toe_next_eph < SECONDS_IN_HOUR)
				wn_next_eph ++;
		}
		else	//BD
		{
			sec_of_week -= GPS_BD_SYSTIME_OFFSET;
			toe_next_eph += SECONDS_IN_HOUR;
		}
		SendMessgeNAV(toe_next_eph,wn_next_eph);
	}
#else
	ephage = CalcSVEPHAge(ephitem.svid, wn, (int32)sec_of_week);
#endif

	if((ephage<SV_MIN_EPH_AGE) || (ephage>SV_EPH_AGE_USABLE_THRES))
		SVInfo[ephitem.svid-1].eph_age = SV_EPH_ALM_AGE_INVALID;
	else
	{
		SVInfo[ephitem.svid-1].eph_age = ephage;
#ifdef _SIMULATE
		ClearBitWord256(&SVMAP_EPH,ephitem.svid-1);
#endif
	}
	

	return;
}

void handle_RTCM3_RX_MSG_1047(word16 msgid,  const byte * const p_data, word16 data_len, STRU_UART_STREAM *glStruUart)
{
	GPSBD2EphFrameStruct Msg1047Info;
	EphemerisBits ephitem;
	double ephage=0.0;
	int32 wn=0;
	double sec_of_week=0.0;
#ifdef _POSTPROC
	int32 toe_next_eph=0, wn_next_eph=0;
#endif

	memset(&Msg1047Info, 0, sizeof(Msg1047Info));
	
	//Parse item from data
	ParseRTCM3FixLenItems(msgid, (byte*)p_data, data_len, &ephitem);
	ParseGPSEphBits2EphInfo(&ephitem,&Msg1047Info);
	//use
	ephitem.svid = ephitem.svid + MinBD2SvID -1;
	Msg1047Info.svid = ephitem.svid;

#ifndef _POSTPROC
	if(!GetCurNavSysTime(NAV_SYS_NONE, GPSTIME_SYNC_SRC_FIXRFCNT, &sec_of_week, &wn))
		return;
#endif

	if(SVInfo[ephitem.svid-1].eph_age < 0)
		return;

	memcpy(&(Sys3AlmEphUTCInfo.GpsBd2_EphStruct[ephitem.svid-1]),&Msg1047Info, sizeof(GPSBD2EphFrameStruct));
	Sys3AlmEphUTCInfo.GpsBd2_EphStruct[ephitem.svid-1].vflg = 1;
	Sys3AlmEphUTCInfo.GpsBd2_EphStruct[ephitem.svid-1].wkn += GPS_BD_WN_OFFSET;
	SVInfo[ephitem.svid-1].ephsrc = EPHALM_SRC_AGNSS;

#ifdef _POSTPROC
	ephage = 1;
	if(SV_IsGps(ephitem.svid)||SV_IsBd2(ephitem.svid))
	{
		if(Sys3AlmEphUTCInfo.GpsBd2_EphStruct[ephitem.svid-MinGpsSvID].vflg==FALSE)
			return SV_EPH_ALM_AGE_INVALID;

		toe_next_eph = Sys3AlmEphUTCInfo.GpsBd2_EphStruct[ephitem.svid-MinGpsSvID].toe;
		wn_next_eph = Sys3AlmEphUTCInfo.GpsBd2_EphStruct[ephitem.svid-MinGpsSvID].wkn;

		if(SV_IsGps(ephitem.svid))
		{
			if (toe_next_eph < SECONDS_IN_HOUR)
				wn_next_eph ++;
		}
		else	//BD
		{
			sec_of_week -= GPS_BD_SYSTIME_OFFSET;
			toe_next_eph += SECONDS_IN_HOUR;
		}
		SendMessgeNAV(toe_next_eph,wn_next_eph);
	}

#else
	ephage = CalcSVEPHAge(ephitem.svid, wn, (int32)sec_of_week);
#endif

	if((ephage<(SV_MIN_EPH_AGE/2)) || (ephage>(SV_EPH_AGE_USABLE_THRES/2)))
		SVInfo[ephitem.svid-1].eph_age = SV_EPH_ALM_AGE_INVALID;
	else
	{
		SVInfo[ephitem.svid-1].eph_age = ephage;
#ifdef _SIMULATE
		ClearBitWord256(&SVMAP_EPH,ephitem.svid-1);
#endif
	}


	return;
}


/**
* @brief the receive handler for RTCM3 MSM
* @para: msgid - message id
* 		p_data - the configured data buffer, point to the head of this message.
*		data_len - message total length 
*/
int32 opcntt=0;
OBSEV* pGObasv=NULL;
void handle_RTCM3_RX_MSM(word16 msgid, const byte * const p_data, word16 data_len, STRU_UART_STREAM *glStruUart)
{
	byte msm_type=0, navsys=0;
	RTCM3_MSM_HEAD_INFO HeadInfo;
	RTCM3_MSM_MAP* pMap;
	OBSEV* pSlaveBuf=NULL;
	
	memset(&HeadInfo, 0, sizeof(HeadInfo));
	ClearMSMDataInfo(&MSMDataInfo);
	memset(MSMDataMap_GPS, 0, sizeof(MSMDataMap_GPS));
	memset(MSMDataMap_BD, 0, sizeof(MSMDataMap_BD));
#if SUPPORT_GLONASS
	memset(MSMDataMap_GLO, 0, sizeof(MSMDataMap_GLO));
#endif

	FindMSMType(msgid, &msm_type, &navsys);

	if(msm_type==0 || msm_type>7)
		return;

	if(navsys == NAV_SYS_GPS)
		pMap = &(MSMDataMap_GPS[0]);
	else if(navsys == NAV_SYS_BD)
		pMap = &(MSMDataMap_BD[0]);
#if SUPPORT_GLONASS
	else if(navsys == NAV_SYS_GLO)
	{
		pMap = &(MSMDataMap_GLO[0]);
		return;//only for test, this should be modified when Glo time pro is OK.
	}
#endif
	else
		return;

	ParseRTCM3MSMItems(msm_type, navsys, (byte*) p_data, data_len, &HeadInfo, pMap, &MSMDataInfo);
#ifndef _POSTPROC
	pSlaveBuf = GetCurSlaveBufAddr(navsys, ((double)HeadInfo.TOW)/1000.0);
#else
	if(glStruUart->comid == 0)//BASE
	{
		pSlaveBuf = GetCurSlaveBufAddr(navsys, ((double)HeadInfo.TOW)/1000.0);
	}
	else					 //ROVE
	{
		pSlaveBuf = GetCurMasterBufAddr(navsys, ((double)HeadInfo.TOW)/1000.0, glStruUart->comid);
	}
#endif	
	//debug code
	opcntt++;
	pGObasv=pSlaveBuf;

	SetMSMDataInfo(navsys, &HeadInfo, pMap, &MSMDataInfo, pSlaveBuf,glStruUart);


#ifdef _POSTPROC
	SendMessgeOBSToShow(pSlaveBuf, glStruUart->comid); //Show Obs
#endif

	return;
}

/********************************Transfer Handle***********************************/

/**
* @brief the transfer handler for RTCM3 message 1001: GPS RTK
* @para: p_data - the configured data buffer, point to the head of this message.
*		pdata_len - point to the message length 
*/
void handle_RTCM3_TX_MSG_1001(word16 msgid,  const byte * const p_data, word16* pdata_len, STRU_UART_STREAM *glStruUart)
{
	RTCM3_MSG_1001_1004_HEAD Msg1001HeadInfo;
	RTCM3_MSG_1001_DATA Msg1001DataInfo[GPS_SV_NUM];

	//word32 trkchmap = GetRTCM3Msg1001To1004HeadInfo(&Msg1001HeadInfo);
	//GetRTCM3Msg1001DataInfo(Msg1001DataInfo, trkchmap);
#ifndef _POSTPROC
	*pdata_len = CreateRTCM3VarLenMSGRawData(msgid, &Msg1001HeadInfo, Msg1001DataInfo, Msg1001HeadInfo.SVCnt, (byte*)p_data, glStruUart);
#endif
	return;
}

/**
* @brief the transfer handler for RTCM3 message 1001: GPS RTK
* @para: p_data - the configured data buffer, point to the head of this message.
*		pdata_len - point to the message length 
*/
void handle_RTCM3_TX_MSG_1004(word16 msgid,  const byte * const p_data, word16* pdata_len, STRU_UART_STREAM *glStruUart)
{
	int32 svcnt=0;
	RTCM3_MSG_1001_1004_HEAD Msg1004HeadInfo;
	RTCM3_MSG_1004_DATA Msg1004DataInfo[GPS_SV_NUM];
	memset(&Msg1004HeadInfo, 0, sizeof(Msg1004HeadInfo));
	memset(Msg1004DataInfo, 0, sizeof(Msg1004DataInfo));

	svcnt = GetRTCM3Msg1004DataInfo(Msg1004DataInfo, &Master_observ);
	
	if(svcnt>0)
		GetRTCM3Msg1001To1004HeadInfo(svcnt, &Msg1004HeadInfo, &Master_observ);
	else
		return;

	*pdata_len = CreateRTCM3VarLenMSGRawData(msgid, &Msg1004HeadInfo, Msg1004DataInfo, Msg1004HeadInfo.SVCnt, (byte*)p_data, glStruUart);
	
	return;
}

/**
* @brief the transfer handler for RTCM3 message 1005: station information
* @para: p_data - the configured data buffer, point to the head of this message.
*		pdata_len - point to the message length 
*/
void handle_RTCM3_TX_MSG_1005(word16 msgid,  const byte * const p_data, word16* pdata_len, STRU_UART_STREAM *glStruUart)
{
	//get RTCM3 data
	ECEF pos;
	RTCM3_MSG_1005_INFO Msg1005Info;
	memset(&Msg1005Info, 0, sizeof(RTCM3_MSG_1005_INFO));

	if(!GetBaseStationInfoForBase(&(Msg1005Info.ReferStationID), NULL, &pos))
		return;

	Msg1005Info.RefPox_x = pos.x*1e4;
	Msg1005Info.RefPox_y = pos.y*1e4;
	Msg1005Info.RefPox_z = pos.z*1e4;
	Msg1005Info.GPS_Indicate = 1;
#if SUPPORT_GLONASS
	Msg1005Info.GLONASS_Indicate = 1;
#endif

	Msg1005Info.Osci_Indicate = 1;
	Msg1005Info.ReferStation_Indicate = 0;	//0 - Real, Physical Reference Station;	1 - Non-Physical or Computed Reference Station
	Msg1005Info.Osci_Indicate = 1;		// 0 - All raw data observations in messages 1001-1004 and 1009-1012
									// may be measured at different instants. This indicator should be set
									// to ※0§ unless all the conditions for ※1§ are clearly met.
									// 1 - All raw data observations in messages 1001-1004 and 1009-1012
									// are measured at the same instant, as described in Section 3.1.4.

	//Format to RTCM3 protocol and send to UART
	*pdata_len = CreateRTCM3FixLenMSGRawData(msgid, &Msg1005Info, (byte*)p_data, glStruUart);

	return;
}

/**
* @brief the transfer handler for RTCM3 message 1006: station information
* @para: p_data - the configured data buffer, point to the head of this message.
*		pdata_len - point to the message length 
*/
void handle_RTCM3_TX_MSG_1006(word16 msgid,  const byte * const p_data, word16* pdata_len, STRU_UART_STREAM *glStruUart)
{
	//get RTCM3 data
	ECEF pos;
	float64 height;
	RTCM3_MSG_1006_INFO Msg1006Info;
	memset(&Msg1006Info, 0, sizeof(RTCM3_MSG_1006_INFO));

	if(!GetBaseStationInfoForBase(&(Msg1006Info.ReferStationID), &height, &pos))
		return;

	Msg1006Info.AntHeight = (word16)(height*1e4);
	Msg1006Info.RefPox_x = pos.x*1e4;
	Msg1006Info.RefPox_y = pos.y*1e4;
	Msg1006Info.RefPox_z = pos.z*1e4;
	Msg1006Info.GPS_Indicate = 1;
#if SUPPORT_GLONASS
	Msg1006Info.GLONASS_Indicate = 1;
#endif

	Msg1006Info.Osci_Indicate = 1;
	Msg1006Info.ReferStation_Indicate = 0;	//0 - Real, Physical Reference Station;	1 - Non-Physical or Computed Reference Station
	Msg1006Info.Osci_Indicate = 1;		// 0 - All raw data observations in messages 1001-1004 and 1009-1012
									// may be measured at different instants. This indicator should be set
									// to ※0§ unless all the conditions for ※1§ are clearly met.
									// 1 - All raw data observations in messages 1001-1004 and 1009-1012
									// are measured at the same instant, as described in Section 3.1.4.

	//Format to RTCM3 protocol and send to UART
	*pdata_len = CreateRTCM3FixLenMSGRawData(msgid, &Msg1006Info, (byte*)p_data, glStruUart);

	return;
}

/**
* @brief the transfer handler for RTCM3 message 1012: GLONSS RTK
* @para: p_data - the configured data buffer, point to the head of this message.
*		pdata_len - point to the message length 
*/
void handle_RTCM3_TX_MSG_1012(word16 msgid,  const byte * const p_data, word16* pdata_len, STRU_UART_STREAM *glStruUart)
{
#if SUPPORT_GLONASS
	int32 svcnt=0;
	RTCM3_MSG_1009_1012_HEAD Msg1012HeadInfo;
	RTCM3_MSG_1012_DATA Msg1012DataInfo[FRE_NUM_GLO];
	memset(&Msg1012HeadInfo, 0, sizeof(Msg1012HeadInfo));
	memset(Msg1012DataInfo, 0, sizeof(Msg1012DataInfo));

	svcnt = GetRTCM3Msg1012DataInfo(Msg1012DataInfo, &Master_observ);
	
	if(svcnt>0)
		GetRTCM3Msg1009To1012HeadInfo(svcnt, &Msg1012HeadInfo, &Master_observ);
	else
		return;

	*pdata_len = CreateRTCM3VarLenMSGRawData(msgid, &Msg1012HeadInfo, Msg1012DataInfo, Msg1012HeadInfo.SVCnt, (byte*)p_data, glStruUart);

#endif
	return;
}


/**
* @brief the transfer handler for RTCM3 message 1019 : GPS eph
* @para: p_data - the configured data buffer, point to the head of this message.
*		pdata_len - point to the message length 
*/
void handle_RTCM3_TX_MSG_1019(word16 msgid, const byte * const p_data, word16* pdata_len, STRU_UART_STREAM *glStruUart)
{
	EphemerisBits ephitem;	
	int32 svid = 0, curdatalen=0;
	GPSBD2EphFrameStruct *pEph;
	
	for (svid=MinGpsSvID; svid<=MaxGpsSvID; svid++)
	{
		if ((SVInfo[svid-1].eph_age < SV_EPH_AGE_USABLE_THRES) && (SVInfo[svid-1].el>GetSVRTKElMask(svid)))
		{
			memset(&ephitem, 0, sizeof(ephitem));
			memset(RTCM3OutputDataBuf, 0, sizeof(RTCM3OutputDataBuf));	

			pEph = &(Sys3AlmEphUTCInfo.GpsBd2_EphStruct[svid-1]);
			ephitem.svid = svid;
			ParseGPSEphInfo2EphBits(pEph, &ephitem);
			curdatalen = CreateRTCM3FixLenMSGRawData(msgid, &ephitem, (byte*)p_data, glStruUart);
		}
	}
	*pdata_len = curdatalen;
	
	return;
}

/**
* @brief the transfer handler for RTCM3 message 1049 : BDs eph
* @para: p_data - the configured data buffer, point to the head of this message.
*		pdata_len - point to the message length 
*/
void handle_RTCM3_TX_MSG_1047(word16 msgid, const byte * const p_data, word16* pdata_len, STRU_UART_STREAM *glStruUart)
{
	EphemerisBits ephitem;
	int32 svid = 0, curdatalen=0;
	GPSBD2EphFrameStruct *pEph;
	
	for (svid=MinBD2SvID; svid<=MaxBD2SvID; svid++)
	{
		if (SVInfo[svid-1].eph_age < SV_EPH_AGE_USABLE_THRES && SVInfo[svid-1].el>GetSVRTKElMask(svid))
		{
			memset(&ephitem, 0, sizeof(ephitem));
			memset(RTCM3OutputDataBuf, 0, sizeof(RTCM3OutputDataBuf));	

			pEph = &(Sys3AlmEphUTCInfo.GpsBd2_EphStruct[svid-1]);
			//pEph->wkn -= GPS_BD_WN_OFFSET;
			ephitem.svid = svid;
			ParseGPSEphInfo2EphBits(pEph, &ephitem);
			
			curdatalen = CreateRTCM3FixLenMSGRawData(msgid, &ephitem, (byte*)p_data, glStruUart);
		}
	}
	*pdata_len = curdatalen;
	
	return;
}



/**
* @brief the transfer handler for RTCM3 MSM
* @para: msgid - message id
* 		p_data - the configured data buffer, point to the head of this message.
*		pdata_len - point to the message length 
*/
void handle_RTCM3_TX_MSM(word16 msgid, const byte * const p_data, word16* pdata_len, STRU_UART_STREAM *glStruUart)
{
	RTCM3_MSM_MAP* pMap;
	byte msm_type=0, navsys=0;
	int32 i=0;
	boolean bMoreMSMFollow;
	RTCM3_MSM_HEAD_INFO HeadInfo;

	FindMSMType(msgid, &msm_type, &navsys);
		
	if(navsys == NAV_SYS_GPS)
		pMap = MSMDataMap_GPS;
	else if(navsys == NAV_SYS_BD)
		pMap = MSMDataMap_BD;
#if SUPPORT_GLONASS
	else if(navsys == NAV_SYS_GLO)
		pMap = MSMDataMap_GLO;
#endif
	else
		return;
	
	for(i=0; i<RTCM3_MSM_MAX_PAGECNT; i++)
	{	
		if(pMap[i].NSat > 0)
		{
			bMoreMSMFollow = IsMoreMSMMsgFollowed(glStruUart->comid);
			GetRTCM3MSMHeadInfo(&HeadInfo, navsys, bMoreMSMFollow, &Master_observ);
			*pdata_len = CreateRTCM3MSMMSGRawData(msm_type, navsys, &HeadInfo, &(pMap[i]), &MSMDataInfo, (byte*)p_data, glStruUart);
		}
	} 
		
	return;
}

/******************************get or set the RTCM3 msg item*********************************/
bool GetRTCM3Msg1001To1004HeadInfo(int32 svcnt, RTCM3_MSG_1001_1004_HEAD* pHeadInfo, OBSEV* pObsv)
{
	if((pObsv->bitmap & OBSEV_VALID_GPS_DATA)==0)
		return FALSE;
	
	pHeadInfo->ReferStationID = pObsv->staid;
	pHeadInfo->TOW = (int32)(pObsv->sec_gps* 1000.0 + 0.5);

	//this flag indicates there is other GNSS observables followed or not, since there is no BD RTCM3 RTK messages, so this flag is 0.
	pHeadInfo->SyncGNSSFlag=1;

	pHeadInfo->SVCnt = (byte)svcnt;
	pHeadInfo->DiverFreeSmooth = 0;	//0 - Divergence-free smoothing not used;	1 - Divergence-free smoothing used
	pHeadInfo->SmoothInterval = FormatRTCM3CSCIntervalIdx(0);	

	return TRUE;
}

void SetRTCM3Msg1001To1004HeadInfo(RTCM3_MSG_1001_1004_HEAD* pHeadInfo, OBSEV* pObsv)
{
	pObsv->staid = pHeadInfo->ReferStationID;
	pObsv->sec_gps = ((double)pHeadInfo->TOW)*0.001;
	

}

int32 GetRTCM3Msg1004DataInfo(RTCM3_MSG_1004_DATA* pDataInfo, OBSEV* pObsv)
{
	float64 scale_DF014 = 299792.458, pr_L1=0.0,pr_L2=0.0, phase=0.0, wavelen=0.0;
	int32 svidx=0, fpidx=0, validsvcnt=0;

	if((pObsv->bitmap & OBSEV_VALID_GPS_DATA)==0)
		return 0;
	
	for(svidx=0; svidx<pObsv->satnmax; svidx++)
	{
		if(!SV_IsGps(pObsv->obs[svidx].StarID))
			continue;

		pDataInfo[validsvcnt].svid = pObsv->obs[svidx].StarID;

		//L1
		fpidx = GetFreGropIdx(FREQ_GROUP_L1);
		if(fpidx<0)
			continue;

		if(pObsv->obs[svidx].validmap[fpidx]==0)
			continue;
		
		if(pObsv->obs[svidx].code[fpidx]==DSP_L1P_FRE)
			pDataInfo[validsvcnt].L1CodeIndicate = 1;
		else
			pDataInfo[validsvcnt].L1CodeIndicate = 0;

		pr_L1=pObsv->obs[svidx].range[fpidx];
		pDataInfo[validsvcnt].L1PRAmbiguity = (byte)(pr_L1/scale_DF014);	//unit:299792.458m
		pDataInfo[validsvcnt].L1Pseudorange = (word32)((pr_L1-((double)pDataInfo[svidx].L1PRAmbiguity)*scale_DF014)*50.0+0.5);	//unit:0.02m
		
		wavelen = GetSVWaveLen(pObsv->obs[svidx].StarID,pObsv->obs[svidx].code[fpidx]);
		phase = pObsv->obs[svidx].phase[fpidx]*wavelen;
		pDataInfo[validsvcnt].L1PhaRangeDifPR = (int32)(ROUND((phase-pr_L1)*2000.0));	//unit:0.0005m
		pDataInfo[validsvcnt].L1LockTimeIdx = FormatRTCM3LockTimeIndicate(pObsv->obs[svidx].LLI[fpidx]/1000);	//s
		pDataInfo[validsvcnt].L1CNR = (byte)(pObsv->obs[svidx].snr0[fpidx]*4+0.5);

		//L2
		fpidx = GetFreGropIdx(FREQ_GROUP_L2);
		if(fpidx<0)
			continue;

		if(pObsv->obs[svidx].validmap[fpidx]==0)
			continue;
		
		if(pObsv->obs[svidx].code[fpidx]==DSP_L2P_FRE)
			pDataInfo[validsvcnt].L2CodeIndicate = 1;
		else
			pDataInfo[validsvcnt].L2CodeIndicate = 0;

		pr_L2=pObsv->obs[svidx].range[fpidx];
		pDataInfo[validsvcnt].L2PRDifL1PR = (int16)(ROUND((pr_L2-pr_L1)*50.0));	//unit:0.02m
		
		wavelen = GetSVWaveLen(pObsv->obs[svidx].StarID,pObsv->obs[svidx].code[fpidx]);
		phase = pObsv->obs[svidx].phase[fpidx]*wavelen;
		pDataInfo[validsvcnt].L2PhaRangeDifL1PR = (int32)(ROUND((phase-pr_L1)*2000.0));	//unit:0.0005m
		pDataInfo[validsvcnt].L2LockTimeIdx = FormatRTCM3LockTimeIndicate(pObsv->obs[svidx].LLI[fpidx]/1000);	//s
		pDataInfo[validsvcnt].L2CNR = (byte)(pObsv->obs[svidx].snr0[fpidx]*4+0.5);

		validsvcnt++;		
	}

	return validsvcnt;
}

void SetRTCM3Msg1004DataInfo(int32 svcnt, RTCM3_MSG_1004_DATA* pDataInfo, OBSEV* pObsv)
{
	int32 i=0, svidx=0, validcnt=0, fpidx=0;
	float64 scale_DF014 = 299792.458, wavelen=0.0, pr_L1=0.0;

	for(i=0; i<svcnt; i++)
	{
		svidx = pObsv->satnmax+validcnt;	
		pObsv->obs[svidx].StarID = pDataInfo[i].svid;
		pObsv->obs[svidx].slotID = pDataInfo[i].svid;
		
		//L1
		fpidx = GetFreGropIdx(FREQ_GROUP_L1);
		if(fpidx<0)	//system not support this frequency point.
			continue;

		if(pDataInfo[i].L1CodeIndicate == 1)
			pObsv->obs[svidx].code[fpidx] = DSP_L1P_FRE;
		else
			pObsv->obs[svidx].code[fpidx] = DSP_L1CA_FRE;

		pr_L1 = ((double)pDataInfo[i].L1PRAmbiguity)*scale_DF014+((double)pDataInfo[i].L1Pseudorange)*0.02;
		pObsv->obs[svidx].range[fpidx] = pr_L1;
		
		wavelen = GetSVWaveLen(pObsv->obs[svidx].StarID,pObsv->obs[svidx].code[fpidx]);
		pObsv->obs[svidx].phase[fpidx] = (((double)pDataInfo[i].L1PhaRangeDifPR)*0.0005+pr_L1)/wavelen;	//cycle
		pObsv->obs[svidx].LLI[fpidx] = GetRTCM3LockTimeByIndicate(pDataInfo[i].L1LockTimeIdx)*1000;	//ms
		pObsv->obs[svidx].snr0[fpidx] = pDataInfo[i].L1CNR*0.25;
		pObsv->obs[svidx].validmap[fpidx] = OBST_VALID_CODE|OBST_VALID_RANGE|OBST_VALID_PHASE|OBST_VALID_LLI|OBST_VALID_SNR;

		//L2
		fpidx = GetFreGropIdx(FREQ_GROUP_L2);
		if(fpidx<0) //system not support this frequency point.
			continue;

		if(pDataInfo[i].L2CodeIndicate == 1)
			pObsv->obs[svidx].code[fpidx] = DSP_L2P_FRE;
		else
			pObsv->obs[svidx].code[fpidx] = DSP_L2C_FRE;

		pObsv->obs[svidx].range[fpidx] = ((double)pDataInfo[i].L2PRDifL1PR)*0.02+pr_L1;
		
		wavelen = GetSVWaveLen(pObsv->obs[svidx].StarID,pObsv->obs[svidx].code[fpidx]);
		pObsv->obs[svidx].phase[fpidx] = (((double)pDataInfo[i].L2PhaRangeDifL1PR)*0.0005+pr_L1)/wavelen; //cycle
		
		pObsv->obs[svidx].LLI[fpidx] = GetRTCM3LockTimeByIndicate(pDataInfo[i].L2LockTimeIdx)*1000;	//ms
		pObsv->obs[svidx].snr0[fpidx] = pDataInfo[i].L2CNR*0.25;
		pObsv->obs[svidx].validmap[fpidx] = OBST_VALID_CODE|OBST_VALID_RANGE|OBST_VALID_PHASE|OBST_VALID_LLI|OBST_VALID_SNR;

		validcnt++;
	}

	pObsv->satnmax += validcnt;
	if(validcnt>0)
		pObsv->bitmap |= OBSEV_VALID_GPS_DATA;	

}

#if SUPPORT_GLONASS
bool GetRTCM3Msg1009To1012HeadInfo(int32 svcnt, RTCM3_MSG_1009_1012_HEAD* pHeadInfo, OBSEV* pObsv)
{
	int32 Day=0;
	double tmptow = 0.0;
	if((pObsv->bitmap & OBSEV_VALID_GLO_DATA)==0)
		return FALSE;
	
	pHeadInfo->ReferStationID = pObsv->staid;
	GpsToGloTime(0,(pObsv->sec_glo - Global_TleapsGLO),&Day,&tmptow);
	pHeadInfo->TOW = (int32)(tmptow*1000.0 + 0.5);

	//this flag indicates there is other GNSS observables followed or not, since there is no BD RTCM3 RTK messages, so this flag is 0.
	pHeadInfo->SyncGNSSFlag=1;

	pHeadInfo->SVCnt = (byte)svcnt;
	pHeadInfo->DiverFreeSmooth = 0; //0 - Divergence-free smoothing not used;	1 - Divergence-free smoothing used
	pHeadInfo->SmoothInterval = FormatRTCM3CSCIntervalIdx(0);	

	return TRUE;
}

void SetRTCM3Msg1009To1012HeadInfo(RTCM3_MSG_1009_1012_HEAD* pHeadInfo, OBSEV* pObsv)
{
	int32 weekn = 0;
	double tmptow = 0.0;
	pObsv->staid = pHeadInfo->ReferStationID;
	GloToGpsTime(Global_DayN4GLO, Global_DayNTGLO, ((double)pHeadInfo->TOW)*0.001 + Global_TleapsGLO, &weekn,&tmptow);
	pObsv->sec_glo = tmptow;
}


int32 GetRTCM3Msg1012DataInfo(RTCM3_MSG_1012_DATA* pDataInfo, OBSEV* pObsv)
{
	float64 scale_DF044 = 599584.916, pr_L1=0.0,pr_L2=0.0, phase=0.0, wavelen=0.0;
	int32 svidx=0, fpidx=0, validsvcnt=0;

	if((pObsv->bitmap & OBSEV_VALID_GLO_DATA)==0)
		return 0;
	
	for(svidx=0; svidx<pObsv->satnmax; svidx++)
	{
		if(!SV_IsGlo(pObsv->obs[svidx].slotID))
			continue;

		pDataInfo[validsvcnt].svid = pObsv->obs[svidx].slotID - MinGloFreID;

		//L1
		fpidx = GetFreGropIdx(FREQ_GROUP_G1);
		if(fpidx<0)
			continue;

		if(pObsv->obs[svidx].validmap[fpidx]==0)
			continue;
		
		if(pObsv->obs[svidx].code[fpidx]==DSP_G1CA_FRE)
			pDataInfo[validsvcnt].L1CodeIndicate = 0;
		else
			pDataInfo[validsvcnt].L1CodeIndicate = 1;

		pr_L1=pObsv->obs[svidx].range[fpidx];
		pDataInfo[validsvcnt].L1PRAmbiguity = (byte)(pr_L1/scale_DF044);	//unit:scale_DF044m
		pDataInfo[validsvcnt].L1Pseudorange = (word32)((pr_L1-((double)pDataInfo[svidx].L1PRAmbiguity)*scale_DF044)*50.0+0.5);	//unit:0.02m
		
		wavelen = GetSVWaveLen(pObsv->obs[svidx].slotID,pObsv->obs[svidx].code[fpidx]);
		phase = pObsv->obs[svidx].phase[fpidx]*wavelen;
		pDataInfo[validsvcnt].L1PhaRangeDifPR = (int32)(ROUND((phase-pr_L1)*2000.0));	//unit:0.0005m
		pDataInfo[validsvcnt].L1LockTimeIdx = FormatRTCM3LockTimeIndicate(pObsv->obs[svidx].LLI[fpidx]);
		pDataInfo[validsvcnt].L1CNR = (byte)(pObsv->obs[svidx].snr0[fpidx]*4+0.5);

		//L2
		fpidx = GetFreGropIdx(FREQ_GROUP_L2);
		if(fpidx<0)
			continue;

		if(pObsv->obs[svidx].validmap[fpidx]==0)
			continue;
		
		if(pObsv->obs[svidx].code[fpidx]==DSP_L2P_FRE)
			pDataInfo[validsvcnt].L2CodeIndicate = 1;
		else
			pDataInfo[validsvcnt].L2CodeIndicate = 0;

		pr_L2=pObsv->obs[svidx].range[fpidx];
		pDataInfo[validsvcnt].L2PRDifL1PR = (int16)(ROUND((pr_L2-pr_L1)*50.0));	//unit:0.02m
		
		wavelen = GetSVWaveLen(pObsv->obs[svidx].StarID,pObsv->obs[svidx].code[fpidx]);
		phase = pObsv->obs[svidx].phase[fpidx]*wavelen;
		pDataInfo[validsvcnt].L2PhaRangeDifL1PR = (int32)(ROUND((phase-pr_L1)*2000.0));	//unit:0.0005m
		pDataInfo[validsvcnt].L2LockTimeIdx = FormatRTCM3LockTimeIndicate(pObsv->obs[svidx].LLI[fpidx]);
		pDataInfo[validsvcnt].L2CNR = (byte)(pObsv->obs[svidx].snr0[fpidx]*4+0.5);

		validsvcnt++;		
	}

	return validsvcnt;
}


void SetRTCM3Msg1012DataInfo(int32 svcnt, RTCM3_MSG_1012_DATA* pDataInfo, OBSEV* pObsv)
{
	int32 i=0, svidx=0, validcnt=0, fpidx=0;
	float64 scale_DF041 = 0.02,scale_DF044 = 599584.916,wavelen=0.0, pr_L1=0.0;

	for(i=0; i<svcnt; i++)
	{
		svidx = pObsv->satnmax+validcnt;	
		pObsv->obs[svidx].StarID = pDataInfo[i].svid + MaxBD2SvID; 
		pObsv->obs[svidx].slotID = pDataInfo[i].FreCh + MinGloFreID;
		
		//L1
		fpidx = GetFreGropIdx(FREQ_GROUP_G1);
		if(fpidx<0)	//system not support this frequency point.
			continue;

		if(pDataInfo[i].L1CodeIndicate == 0)
			pObsv->obs[svidx].code[fpidx] = DSP_G1CA_FRE;
		
		pr_L1 = ((double)pDataInfo[i].L1PRAmbiguity)*scale_DF044+((double)pDataInfo[i].L1Pseudorange)*scale_DF041;
		pObsv->obs[svidx].range[fpidx] = pr_L1;
		
		wavelen = GetSVWaveLen(pObsv->obs[svidx].slotID,pObsv->obs[svidx].code[fpidx]);
		pObsv->obs[svidx].phase[fpidx] = (((double)pDataInfo[i].L1PhaRangeDifPR)*0.0005+pr_L1)/wavelen;	//cycle
		pObsv->obs[svidx].LLI[fpidx] = GetRTCM3LockTimeByIndicate(pDataInfo[i].L1LockTimeIdx);	//s
		pObsv->obs[svidx].snr0[fpidx] = pDataInfo[i].L1CNR*0.25;
		pObsv->obs[svidx].validmap[fpidx] = OBST_VALID_CODE|OBST_VALID_RANGE|OBST_VALID_PHASE|OBST_VALID_LLI|OBST_VALID_SNR;

		//L2
		fpidx = GetFreGropIdx(FREQ_GROUP_L2);
		if(fpidx<0) //system not support this frequency point.
			continue;

		if(pDataInfo[i].L2CodeIndicate == 0)
			pObsv->obs[svidx].code[fpidx] = DSP_G2CA_FRE;
		
		pObsv->obs[svidx].range[fpidx] = ((double)pDataInfo[i].L2PRDifL1PR)*scale_DF041+pr_L1;
		
		wavelen = GetSVWaveLen(pObsv->obs[svidx].slotID,pObsv->obs[svidx].code[fpidx]);
		pObsv->obs[svidx].phase[fpidx] = (((double)pDataInfo[i].L2PhaRangeDifL1PR)*0.0005+pr_L1)/wavelen; //cycle
		
		pObsv->obs[svidx].LLI[fpidx] = GetRTCM3LockTimeByIndicate(pDataInfo[i].L2LockTimeIdx);	//s
		pObsv->obs[svidx].snr0[fpidx] = pDataInfo[i].L2CNR*0.25;
		pObsv->obs[svidx].validmap[fpidx] = OBST_VALID_CODE|OBST_VALID_RANGE|OBST_VALID_PHASE|OBST_VALID_LLI|OBST_VALID_SNR;

		validcnt++;
	}

	pObsv->satnmax += validcnt;
	if(validcnt>0)
		pObsv->bitmap |= OBSEV_VALID_GLO_DATA;	

}
#endif


//get MSM head information
bool GetRTCM3MSMHeadInfo(RTCM3_MSM_HEAD_INFO* pHeadInfo, byte navsys, boolean multiMsgFlag, OBSEV* pObsv)
{
	double tow=0.0;
#if SUPPORT_GLONASS
	int32 GLo_Day=0;
#endif
	
	memset(pHeadInfo, 0, sizeof(pHeadInfo));

	pHeadInfo->ReferStationID = pObsv->staid;

	if(navsys==NAV_SYS_GPS)
	{
		pHeadInfo->TOW = (word32)(pObsv->sec_gps*1000.0+0.5);
		pHeadInfo->ClkSteer_Indicate = pObsv->clkSteer_gps;
	}
	else if(navsys==NAV_SYS_BD)
	{
		tow = (pObsv->sec_bd-GPS_BD_SYSTIME_OFFSET)*1000.0+0.5;
		if(tow<0.0)
			tow += (SECONDS_IN_WEEK*1000.0);
		
		pHeadInfo->TOW = (word32)(tow);

		pHeadInfo->ClkSteer_Indicate = pObsv->clkSteer_bd;
	}
#if SUPPORT_GLONASS
	else if(navsys==NAV_SYS_GLO)
	{
		GpsToGloTime(0,(double)(pObsv->sec_glo - Global_TleapsGLO), &GLo_Day,&tow);
		pHeadInfo->TOW = (word32)(tow*1000.0 +0.5);
		pHeadInfo->ClkSteer_Indicate = pObsv->clkSteer_glo;
	}
#endif
	else
		return FALSE;


	pHeadInfo->MultiMsgBit = multiMsgFlag;	//  1 indicates that more MSMs follow for given physical time and reference station ID
											// 0 indicates that it is the last MSM for given physical time and reference station ID

	pHeadInfo->IODS = 0;			// This field is reserved to be used to link MSM with future sitedescription (receiver, antenna description, etc.) messages. A value of  "0' indicates that this field is not utilized.


	
	pHeadInfo->ExClk_Indicate = 0;	// 0 每 internal clock is used
									// 1 每 external clock is used, clock status is ※locked§
									// 2 每 external clock is used, clock status is ※not locked§, which may indicate external clock failure and that the transmitted data may not be reliable.
									// 3 每 unknown clock is used


	pHeadInfo->DiverFreeSmooth = 0;	//0 - Divergence-free smoothing not used;	1 - Divergence-free smoothing used
	pHeadInfo->SmoothInterval = FormatRTCM3CSCIntervalIdx(0);	

	return TRUE;
}


void UpdateMSMDataInfo(OBSEV* pObsv)
{
	int32 i, sv=0,slot=0, fpidx=0, frqpoint=0, svidx=0, sigidx=0, navsys=0, signmax=0, cursigidx=0, svcnt=0;
	int32 signcnt_gps=0, signcnt_bd=0, signcnt_glo=0;
	float64 pr=0.0, phase=0.0, wavelen=0.0, pr_rough= 0.0, doppler=0.0;
	bool bSatDatValid=FALSE;
	RTCM3_MSM_MAP* pcurMap = NULL;
	RTCM3_MSM_SAT_INFO* pSatData = NULL;
	RTCM3_MSM_SIG_INFO* pSigData = NULL;

	for(sigidx=0; sigidx<32; sigidx++)
	{
		if(pObsv->signmax_gps & (0x1<<sigidx)) 
			signcnt_gps++;
		if(pObsv->signmax_bd & (0x1<<sigidx)) 
			signcnt_bd++;
		if(pObsv->signmax_glo & (0x1<<sigidx)) 
			signcnt_glo++;
	}

	for(i=0; i<pObsv->satnmax; i++)
	{
		sv = pObsv->obs[i].StarID;
		slot = pObsv->obs[i].slotID;
		if(SV_IsGps(sv))
		{
			svidx = sv-MinGpsSvID;
			pcurMap = &(MSMDataMap_GPS[0]);
			navsys = NAV_SYS_GPS;
			signmax = signcnt_gps;
		}
		else if(SV_IsBd2(sv))
		{
			svidx = sv-MinBD2SvID;
			pcurMap = &(MSMDataMap_BD[0]);
			navsys = NAV_SYS_BD;
			signmax = signcnt_bd;
		}
#if SUPPORT_GLONASS
		else if(SV_IsGlo(slot))
		{
			svidx = sv-MinGloFreID;
			pcurMap = &(MSMDataMap_GLO[0]);
			navsys = NAV_SYS_GLO;
			signmax = signcnt_glo;
		}
#endif
		else
			continue;

		//update map sat
		if((pcurMap->NSat*pcurMap->NSig)>=64)	//multipul msg
			pcurMap++;

		SetMSMMaskBit(pcurMap->SatMask,svidx);
		pcurMap->NSat++;

		//update sat data
		MSMDataInfo.svidx[sv-1] = svcnt;
		pSatData = &(MSMDataInfo.SatData[svcnt]);
		
		cursigidx=0;
		bSatDatValid=FALSE;
		for(sigidx=0; sigidx<32; sigidx++)
		{
			word32 temp = 0x1<<sigidx;
			if((navsys==NAV_SYS_GPS) && ((pObsv->signmax_gps&temp)==0))
				continue;
			else if((navsys==NAV_SYS_BD) && ((pObsv->signmax_bd&temp)==0))
				continue;
			else if((navsys==NAV_SYS_GLO) && ((pObsv->signmax_glo&temp)==0))
				continue;

			cursigidx++;
			
			frqpoint = ConvertRTCM3SigMaskidx2FP(sigidx, navsys);
			fpidx = GetFreGropIdx(frqpoint);
			if(pObsv->obs[i].validmap[fpidx]==0)
				continue;

			//update sig mask
			if(GetMSMMaskBit(pcurMap->SigMask,sigidx)==0)
			{
				SetMSMMaskBit(pcurMap->SigMask,sigidx);
				pcurMap->NSig++;	
			}

			//update map cell
			SetMSMMaskBit(pcurMap->CellMask,(pcurMap->NSat-1)*signmax+(cursigidx-1));

			/****update sat data********/
			wavelen = GetSVWaveLen(slot,frqpoint);
			pr=(pObsv->obs[i].range[fpidx]*RECIP_SPEED_OF_LIGHT)*1000.0;	//ms
			doppler = pObsv->obs[i].doppler[fpidx]*wavelen;	//m/s
			
			if(!bSatDatValid)
			{
				pSatData->RoughRangeInter = (byte)pr;	//unit: 1ms
				pSatData->RoughRange = (word16)((pr - pSatData->RoughRangeInter)*TWO_P10);	//unit: 2^-10ms
				pSatData->RoughPhasePRRate = (int16)(doppler);
				pSatData->datamap = MSM_SAT_MAPIDX_RouPRInter|MSM_SAT_MAPIDX_RouPR|MSM_SAT_MAPIDX_RouPhaPRate;
				
				pr_rough = (double)pSatData->RoughRangeInter+((double)pSatData->RoughRange)*TWO_N10;	//ms
				bSatDatValid=TRUE;
			}

			/******update sig data*******/
			pSigData = &(MSMDataInfo.SigData[svcnt][fpidx]);
			
			pSigData->HalfCycleIndicate = 0;
			//cnr
			pSigData->CNR = (byte)(pObsv->obs[i].snr0[fpidx]+0.5);
			pSigData->CNREx = (word16)(pObsv->obs[i].snr0[fpidx]*10.0+0.5);

			//locktime
			pSigData->LockTimeIdx = FormatMSMLockTimeIndicate(pObsv->obs[i].LLI[fpidx]);
			pSigData->LockTimeIdxEx = FormatMSMLockTimeIndicateEx(pObsv->obs[i].LLI[fpidx]);
			
			//pr	
			pSigData->FineRange = (int16)((pr - pr_rough)*TWO_P24);
			pSigData->FineRangeEx = (int32)((pr - pr_rough)*TWO_P29);

			//phase
			phase = pObsv->obs[i].phase[fpidx]*wavelen*RECIP_SPEED_OF_LIGHT*1000.0; 	//ms 
			pSigData->FinePhasePR = (int32)((phase - pr_rough)*TWO_P29);
			pSigData->FinePhasePREx = (int32)((phase - pr_rough)*TWO_P31);
			pSigData->FinePhasePRate = (int16)(ROUND(doppler*10000.0));		//0.0001m/s

			//halfcycle
			pSigData->HalfCycleIndicate = pObsv->obs[i].halfcycle[fpidx];

			//bitmap
			pSigData->datamap = (MSM_SIG_MAPIDX_HalfCycleIndicate|MSM_SIG_MAPIDX_LockTimeIdx|MSM_SIG_MAPIDX_CNR
							|MSM_SIG_MAPIDX_FineRange|MSM_SIG_MAPIDX_FinePhasePRate|MSM_SIG_MAPIDX_FinePhasePR
							|MSM_SIG_MAPIDX_FineRangeEx|MSM_SIG_MAPIDX_FinePhasePREx|MSM_SIG_MAPIDX_LockTimeIdxEx
							|MSM_SIG_MAPIDX_CNREx);
		}

		svcnt++;
	}
	
	return;
}

void SetMSMDataInfo(byte navsys, RTCM3_MSM_HEAD_INFO* pHead, RTCM3_MSM_MAP* pMap, RTCM3_MSM_DATA_INFO* pMSMDataInfo, OBSEV* pObsv,STRU_UART_STREAM *glStruUart)
{
	int32 svcnt=0, svid=0, idx, i=0,j=0, frqidx;
	double chk;
	byte chk1;
	word32 frepoint=0;
	double pr_rouf=0.0, fd_rouf=0.0, wavelen=0.0;
	RTCM3_MSM_SAT_INFO *pSatData=NULL;
	RTCM3_MSM_SIG_INFO *pSigData=NULL;
	word32 *pSigMap = NULL;

	//head
	pObsv->staid = pHead->ReferStationID;
	pObsv->bitmap |= OBSEV_VALID_STATION;

	if(pHead->MultiMsgBit == 0)
		bRTCMMsgRXFinish = TRUE;
	else
		bRTCMMsgRXFinish = FALSE;
	
	if(navsys==NAV_SYS_GPS)
	{
		pObsv->sec_gps = pHead->TOW/1000.0;
		pObsv->clkSteer_gps = pHead->ClkSteer_Indicate;
		pSigMap = &(pObsv->signmax_gps);
		pObsv->bitmap |= OBSEV_VALID_GPS_DATA;
	}
	else if(navsys==NAV_SYS_BD)
	{
		pObsv->sec_bd = pHead->TOW/1000.0+GPS_BD_SYSTIME_OFFSET;
		if(pObsv->sec_bd>=SECONDS_IN_WEEK)
			pObsv->sec_bd -= SECONDS_IN_WEEK;
		
		pObsv->clkSteer_bd = pHead->ClkSteer_Indicate;
		pSigMap = &(pObsv->signmax_bd);
		pObsv->bitmap |= OBSEV_VALID_BD_DATA;
	}
	else if(navsys==NAV_SYS_GLO)
	{
		//GloToGpsTime(int32 N4,int32 NT,double t,int32 * weekn,double * tow)
		pObsv->sec_glo = pHead->TOW/1000.0;
		pObsv->clkSteer_glo = pHead->ClkSteer_Indicate;
		pSigMap = &(pObsv->signmax_glo);
		pObsv->bitmap |= OBSEV_VALID_GLO_DATA;
	}
	else
		return;

	//sat and sig
	for(i=0; i<64; i++)
	{
		if(GetMSMMaskBit(pMap->SatMask,i)==0)
			continue;

		svid = GetC1610SVIDFromMSMSatMap(i,navsys);
#if SUPPORT_GLONASS
		if((!SV_IsGps(svid)) && (!SV_IsBd2(svid)) && (!SV_IsGlo(svid)))
#else
		if((!SV_IsGps(svid)) && (!SV_IsBd2(svid)))
#endif
			continue;
		
		idx = FindMSMDataSVIdx(pMSMDataInfo, svid);
		if(idx<0 || idx>=MAX_RTK_OBS_SVCNT)
			continue;

		svcnt = pObsv->satnmax;
		pObsv->obs[svcnt].StarID = svid;
		pObsv->obs[svcnt].slotID = svid;

#if SUPPORT_GLONASS
		if(svid>=MinGloFreID)
		{
			pObsv->obs[svcnt].slotID = ConvertGloSVID2SlotID(svid);
			if(pObsv->obs[svcnt].slotID <=0)
				continue;
		}
#endif		
		pSatData = &(pMSMDataInfo->SatData[idx]);

		//sat data
		if((pSatData->datamap & (1<<MSM_SAT_MAPIDX_RouPRInter))
			&&(pSatData->datamap & (1<<MSM_SAT_MAPIDX_RouPR)))
			pr_rouf = pSatData->RoughRangeInter+pSatData->RoughRange*TWO_N10;	//ms

		if(pSatData->datamap & (1<<MSM_SAT_MAPIDX_RouPhaPRate))
			fd_rouf = pSatData->RoughPhasePRRate;			//m/s

		//sig data
		for(j=0; j<32; j++)
		{
			if(GetMSMMaskBit(pMap->SigMask,j)==0)
				continue;

			frepoint = ConvertRTCM3SigMaskidx2FP(j,navsys);
			frqidx = GetFreGropIdx(frepoint);
			if(frqidx<0 || frqidx>=MAX_FREPIONT_PER_NAVSYS)
				continue;
			
			pSigData = &(pMSMDataInfo->SigData[idx][frqidx]);

			(*pSigMap) |= (0x1<<j);

			//code, freqpoint
			pObsv->obs[svcnt].code[frqidx] = frepoint;
			pObsv->obs[svcnt].validmap[frqidx] |= OBST_VALID_CODE;

			//cn0
			if(pSigData->datamap & (1<<MSM_SIG_MAPIDX_CNREx))
			{
				pObsv->obs[svcnt].snr0[frqidx] = pSigData->CNREx/10.0;
				pObsv->obs[svcnt].validmap[frqidx] |= OBST_VALID_SNR;
			}
			else if(pSigData->datamap & (1<<MSM_SIG_MAPIDX_CNR))
			{
				pObsv->obs[svcnt].snr0[frqidx] = pSigData->CNR;
				pObsv->obs[svcnt].validmap[frqidx] |= OBST_VALID_SNR;
			}

			//lock time
			if(pSigData->datamap & (1<<MSM_SIG_MAPIDX_LockTimeIdxEx))
			{
				pObsv->obs[svcnt].LLI[frqidx]= GetMSMLockTimeIndicateEx(pSigData->LockTimeIdxEx);
				pObsv->obs[svcnt].validmap[frqidx] |= OBST_VALID_LLI;
			}
			else if(pSigData->datamap & (1<<MSM_SIG_MAPIDX_LockTimeIdx))
			{
				pObsv->obs[svcnt].LLI[frqidx] = GetRTCM3LockTimeByIndicate(pSigData->LockTimeIdx);
				pObsv->obs[svcnt].validmap[frqidx] |= OBST_VALID_LLI;
			}

			//half cycle indicate
			if(pSigData->datamap & (1<<MSM_SIG_MAPIDX_HalfCycleIndicate))
			{
				pObsv->obs[svcnt].halfcycle[frqidx]= pSigData->HalfCycleIndicate;
				pObsv->obs[svcnt].validmap[frqidx] |= OBST_VALID_HALFCYCLE;
			}

			//range
			if(pSigData->datamap & (1<<MSM_SIG_MAPIDX_FineRangeEx))
			{
				pObsv->obs[svcnt].range[frqidx]= (pr_rouf + ((double)pSigData->FineRangeEx)*TWO_N29)*0.001*SPEED_OF_LIGHT;	//m
				pObsv->obs[svcnt].validmap[frqidx] |= OBST_VALID_RANGE;
			}
 			else if(pSigData->datamap & (1<<MSM_SIG_MAPIDX_FineRange))
			{
				pObsv->obs[svcnt].range[frqidx] = (pr_rouf + ((double)pSigData->FineRange)*TWO_N24)*0.001*SPEED_OF_LIGHT;	//m
				pObsv->obs[svcnt].validmap[frqidx] |= OBST_VALID_RANGE;
			}
	
			wavelen = GetSVWaveLen(pObsv->obs[svcnt].slotID,  frepoint);
			
			if(pSigData->datamap & (1<<MSM_SIG_MAPIDX_FinePhasePREx))
			{
				pObsv->obs[svcnt].phase[frqidx]= (pr_rouf + ((double)pSigData->FinePhasePREx)*TWO_N31)*0.001*SPEED_OF_LIGHT/wavelen;	//m
				pObsv->obs[svcnt].validmap[frqidx] |= OBST_VALID_PHASE;
			}
			else if(pSigData->datamap & (1<<MSM_SIG_MAPIDX_FinePhasePR))
			{
				pObsv->obs[svcnt].phase[frqidx] = (pr_rouf + ((double)pSigData->FinePhasePR)*TWO_N29)*0.001*SPEED_OF_LIGHT/wavelen;	//m
				pObsv->obs[svcnt].validmap[frqidx] |= OBST_VALID_PHASE;
				chk=pObsv->obs[svcnt].range[frqidx]/pObsv->obs[svcnt].phase[frqidx];
			}
			
			//fd
			if(pSigData->datamap & (1<<MSM_SIG_MAPIDX_FinePhasePRate))
			{
				pObsv->obs[svcnt].doppler[frqidx]= (fd_rouf + ((double)pSigData->FinePhasePRate)*0.0001)/wavelen;	//Hz
				pObsv->obs[svcnt].validmap[frqidx] |= OBST_VALID_DOPPLER;
			}
		}

		pObsv->satnmax++;
	}

	
}

int32 UpdateTotalMSMCnt(byte comid)
{
	int32 totalMSMCnt=0;
	int32 gpsOneMsgMSMCnt = 0, bdOneMsgMSMCnt=0, gloOneMsgMSMCnt=0;
	int32 i=0;
	UINT8 *prtcm_output_cycle;
	
	for(i=0; i<RTCM3_MSM_MAX_PAGECNT; i++)
	{
		if(MSMDataMap_GPS[i].NSat > 0)
			gpsOneMsgMSMCnt++;

		if(MSMDataMap_BD[i].NSat > 0)
			bdOneMsgMSMCnt++;
#if SUPPORT_GLONASS
		if(MSMDataMap_GLO[i].NSat > 0)
			gloOneMsgMSMCnt++;
#endif
	}

	i=0;
	prtcm_output_cycle = &(pActiveCPT->SysmCptProtocol[comid].Rtcm3_1001);
	while(RTCM3_MSG_TBL[i].msg_id != RTCM3_MSG_ID_INVALID)
	{
		if((*prtcm_output_cycle) != 0)
		{	
			if((RTCM3_MSG_TBL[i].msg_id >= 1071) && (RTCM3_MSG_TBL[i].msg_id <= 1077))
				totalMSMCnt += gpsOneMsgMSMCnt;
			else if((RTCM3_MSG_TBL[i].msg_id >= 1121) && (RTCM3_MSG_TBL[i].msg_id <= 1127))
				totalMSMCnt += bdOneMsgMSMCnt;
			else if((RTCM3_MSG_TBL[i].msg_id >= 1081) && (RTCM3_MSG_TBL[i].msg_id <= 1087))
				totalMSMCnt += gloOneMsgMSMCnt;
		}
		
		prtcm_output_cycle++;
		i++;
	}

	TotalMSMCnt2TX[comid] = totalMSMCnt;

	return totalMSMCnt;
}

boolean IsMoreMSMMsgFollowed(byte comid)
{
	CurMSMCnt2TX[comid]++;

	if(CurMSMCnt2TX[comid] == TotalMSMCnt2TX[comid] )
		return FALSE;
	else
		return TRUE;
}

void ResetAllMSMParam(void)
{
	ClearMSMDataInfo(&MSMDataInfo);
	memset(MSMDataMap_GPS, 0, sizeof(MSMDataMap_GPS));
	memset(MSMDataMap_BD, 0, sizeof(MSMDataMap_BD));
#if SUPPORT_GLONASS
	memset(MSMDataMap_GLO, 0, sizeof(MSMDataMap_GLO));
#endif

	memset(&(CurMSMCnt2TX[0]), 0,  MAX_UART_NUM*sizeof(int32));
	memset(&(TotalMSMCnt2TX[0]), 0,MAX_UART_NUM*sizeof(int32));
	bRTCMMsgRXFinish = FALSE;
}

void ClearMSMDataInfo(RTCM3_MSM_DATA_INFO* pMSMDataInfo)
{
	int32 i=0;
	memset(pMSMDataInfo, 0, sizeof(RTCM3_MSM_DATA_INFO));

	for(i=0; i<SV_NUM; i++)
		pMSMDataInfo->svidx[i] = -1;
}



/***************Carrier Smoothing Interval of Code Phse, DF008, DF038************/
static CSC_INTERVAL CSC_INTERVAL_TBL[8] = {
	{0,	0},		// 0s No smoothing
	{1,	29},		// 1~29s
	{2,	59},		// 30~59s
	{3,	119},	// 1~2 min
	{4,	239},	// 2~4 min
	{5,	479},	// 4~8 min
	{6,	65534},	// >8 min  8min~65534s
	{7,	65535},	//Unlimited smoothing interval
};

byte FormatRTCM3CSCIntervalIdx(word16 sec)
{
	int32 i=0;
	for(i=0; i<sizeof(CSC_INTERVAL_TBL)/sizeof(CSC_INTERVAL); i++)
	{
		if(sec <= CSC_INTERVAL_TBL[i].maxInterval)
			return i;
	}

	return 0;
}



/*********************Lock time indicator, DF013, DF019, DF043, DF049*******************/
static LOCK_TIME_INDICATE LOCK_TIME_INDICATE_TBL[6] = {
	{23,		1,	0,		23},
	{47,		2,	24,		71},
	{71,		4,	120,		167},
	{95,		8,	408,		359},
	{119,	16,	1176,	743},
	{126,	32,	3096,	936},
};

/*
	locktime	unit:s
*/
byte FormatRTCM3LockTimeIndicate(word32 locktime)
{
	byte indcate = 127;
	int32 i=0;
	for(i=0; i<sizeof(LOCK_TIME_INDICATE_TBL)/sizeof(LOCK_TIME_INDICATE); i++)
	{
		if(locktime <= LOCK_TIME_INDICATE_TBL[i].maxlocktime)
		{
			indcate =(byte) ((locktime + LOCK_TIME_INDICATE_TBL[i].para_b)/LOCK_TIME_INDICATE_TBL[i].para_a);
			break;
		}
	}

	return indcate;
}

UINT32 GetRTCM3LockTimeByIndicate(byte indicate)
{
	UINT32 locktime=0;

	if(indicate<=0)
		locktime = 0;
	else if(indicate >= 15)
		locktime = 524288;
	else
		locktime = (((UINT32)0x20)<<(indicate-1));

	return locktime;
}


/******************MSM GNSS PhaseRange Lock Time Indicator (DF402) *******************/
byte FormatMSMLockTimeIndicate(word32 locktime)
{
	byte indicate = 0;
	if(locktime < 32)
		indicate = 0;
	else if(locktime < 64)
		indicate = 1;
	else if(locktime < 128)
		indicate = 2;
	else if(locktime < 256)
		indicate = 3;
	else if(locktime < 512)
		indicate = 4;
	else if(locktime < 1024)
		indicate = 5;
	else if(locktime < 2048)
		indicate = 6;
	else if(locktime < 4096)
		indicate = 7;
	else if(locktime < 8192)
		indicate = 8;
	else if(locktime < 16384)
		indicate = 9;
	else if(locktime < 32768)
		indicate = 10;
	else if(locktime < 65536)
		indicate = 11;
	else if(locktime < 131072)
		indicate = 12;
	else if(locktime < 262144)
		indicate = 13;
	else if(locktime < 524288)
		indicate = 14;
	else
		indicate = 15;

	return indicate;
}

static LOCK_TIME_INDICATE LOCK_TIME_INDICATE_MSMEX_TBL[21] = {
	{63,			1,		0,			63},
	{95,			2,		64,			127},
	{127,		4,		256,			255},
	{159,		8,		768,			511},
	{191,		16,		2048,		1023},
	{223,		32,		5120,		2047},
	{255,		64,		12288,		4095},
	{287,		128,		28672,		8191},
	{319,		256,		65536,		16383},
	{351,		512,		147456,		32767},
	{383,		1024,	327680,		65535},
	{415,		2048,	720896,		131071},
	{447,		4096,	1572864,		262143},
	{479,		8192,	3407872,		524287},
	{511,		16384,	7340032,		1048575},
	{543,		32768,	15728640,	2097151},
	{575,		65536,	33554432,	4194303},
	{607,		131072,	71303168,	8388607},
	{639,		262144,	150994944,	16777215},
	{671,		524288,	318767104,	33554431},
	{703,		1048576,	671088640,	67108863},
};

word16 FormatMSMLockTimeIndicateEx(word32 locktime)
{
	word16 indcate = 703;
	int32 i=0;
	for(i=0; i<sizeof(LOCK_TIME_INDICATE_MSMEX_TBL)/sizeof(LOCK_TIME_INDICATE); i++)
	{
		if(locktime <= LOCK_TIME_INDICATE_MSMEX_TBL[i].maxlocktime)
		{
			indcate =(word16) ((locktime + LOCK_TIME_INDICATE_MSMEX_TBL[i].para_b)/LOCK_TIME_INDICATE_MSMEX_TBL[i].para_a);
			break;
		}
	}

	return indcate;
}

word32 GetMSMLockTimeIndicateEx(word16 indicate)
{
	word32 locktime=0;
	int32 i=0;

	if(indicate>=704)
		return 67108863;
	
	for(i=0; i<sizeof(LOCK_TIME_INDICATE_MSMEX_TBL)/sizeof(LOCK_TIME_INDICATE); i++)
	{
		if(indicate <= LOCK_TIME_INDICATE_MSMEX_TBL[i].maxIndicate)
		{
			locktime =(word32)(indicate*LOCK_TIME_INDICATE_MSMEX_TBL[i].para_a - LOCK_TIME_INDICATE_MSMEX_TBL[i].para_b);
			break;
		}
	}

	return locktime;
}


int8 ConvertFP2RTCM3SigMaskidx(word32 frepoint)
{
	int8 maskidx=-1; 
	switch(frepoint)
	{
	case DSP_B1I_FRE:	maskidx=RTCM3_SIGMASK_B1I; break;
	case DSP_B2I_FRE:	maskidx=RTCM3_SIGMASK_B2I; break;
	case DSP_B3I_FRE:	maskidx=RTCM3_SIGMASK_B3I; break;
	case DSP_L1CA_FRE:	maskidx=RTCM3_SIGMASK_L1CA; break;
	case DSP_L1C_FRE:	maskidx=RTCM3_SIGMASK_L1CD; break;
	case DSP_L1P_FRE:	maskidx=RTCM3_SIGMASK_L1P; break;
	case DSP_L2C_FRE:	maskidx=RTCM3_SIGMASK_L2CA; break;
	case DSP_L2P_FRE:	maskidx=RTCM3_SIGMASK_L2P; break;
	case DSP_L5_FRE:	maskidx=RTCM3_SIGMASK_L5I; break;
	case DSP_G1CA_FRE:	maskidx=RTCM3_SIGMASK_G1CA; break;
	case DSP_G2CA_FRE:	maskidx=RTCM3_SIGMASK_G2CA; break;
	default: break;
	}

	return maskidx;
}

word32 ConvertRTCM3SigMaskidx2FP(int8 maskidx, byte navsys)
{
	word32 frepoint=0; 
	if(navsys==NAV_SYS_GPS)
	{
		switch(maskidx)
		{
		case RTCM3_SIGMASK_L1CA:	frepoint=DSP_L1CA_FRE; break;
		case RTCM3_SIGMASK_L1CD:	frepoint=DSP_L1C_FRE; break;
		case RTCM3_SIGMASK_L1P:		frepoint=DSP_L1P_FRE; break;
		case RTCM3_SIGMASK_L2CA:	frepoint=DSP_L2C_FRE; break;
		case RTCM3_SIGMASK_L2Z:		frepoint=DSP_L2P_FRE; break;
		case RTCM3_SIGMASK_L2P:		frepoint=DSP_L2P_FRE; break;
		case RTCM3_SIGMASK_L2CM:	frepoint=DSP_L2C_FRE; break;
		case RTCM3_SIGMASK_L2CL:    frepoint=DSP_L2C_FRE; break;
		case RTCM3_SIGMASK_L5I:		frepoint=DSP_L5_FRE; break;
		case RTCM3_SIGMASK_L5Q:     frepoint=DSP_L5_FRE; break;
		case RTCM3_SIGMASK_L2CML:   frepoint=DSP_L2C_FRE;break;
		default: break;
		}
	}
	else if(navsys==NAV_SYS_BD)
	{
		switch(maskidx)
		{
		case RTCM3_SIGMASK_B1I:		frepoint=DSP_B1I_FRE; break;
		case RTCM3_SIGMASK_B2I:		frepoint=DSP_B2I_FRE; break;
		case RTCM3_SIGMASK_B3I:		frepoint=DSP_B3I_FRE; break;
		default: break;
		}
	}
	else if(navsys==NAV_SYS_GLO)
	{
		switch(maskidx)
		{
		case RTCM3_SIGMASK_G1CA:	frepoint=DSP_G1CA_FRE; break;
		case RTCM3_SIGMASK_G2CA:	frepoint=DSP_G2CA_FRE; break;
		default: break;
		}
	}

	return frepoint;
}

bool IsMSMMsgRXFinished(void)
{
	if(bRTCMMsgRXFinish)
		return TRUE;
	else
		return FALSE;
}

