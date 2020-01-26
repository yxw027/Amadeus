
//#include <stdio.h>
#include "typedefine.h"
#include <string.h>
#include  "stdlib.h"
#include  "hcp.h"
#include "pvt.h"	
#include "Coordinate.h"
#include "cfgpara.h"
#include "PostProc.h"
#include "TimeProc.h"
#include "PVTRcvrInfo.h"
#include "constdef.h"
#include "nmea0183.h"
#include "novatel.h"
#include "RTK.h"
#include "cfgpara.h"
#include "hwaasic.h"

#ifndef _POSTPROC
#ifndef _SIMULATE
#include "UartFunc.h"
#include "hDefine.h"
#include "RTCM3.h"
#include "GnssTYProcfg.h"
#include "sysbackup.h"
#endif
#endif

static PROTOCOL_HCP		ProtocolHCP;
//static PROTOCOL_STATUS		 ProtocolStatus = {CHECK_DATA_HEAD_FALSE};

static char  	HCPDataBuf[UART_SEND_BUF_LENTH];

static int32 HCPRcvProc(int32 *pValidDataLen,STRU_UART_STREAM *glStruUart);

static void 	Handle_HCP_Periodic_DBG_SYSM(byte comid);

static void    	Handle_HCP_NAV_Pos(word16 datalen, byte* pData,byte comid);
static void    	Handle_HCP_SV_GPS_Eph(word16 datalen, byte* pData ,byte comid);
static void    	Handle_HCP_SYSM_RST(word16 datalen, byte* pData ,byte comid);
static void 	Handle_HCP_NAV_Vel(word16 datalen, byte* pData ,byte comid);
static void 	Handle_HCP_NAV_Time_UTC(word16 datalen, byte* pData,byte comid);
static void    	Handle_HCP_NAV_Time_BD(word16 datalen, byte* pData ,byte comid);
static void 	Handle_HCP_NAV_Time_GPS(word16 datalen, byte* pData ,byte comid);
static void 	Handle_HCP_NAV_SYSM(word16 datalen, byte* pData ,byte comid);
static void 	Handle_HCP_SYSM_UParm(word16 datalen, byte* pData ,byte comid);
static void 	Handle_HCP_SYSM_Ver(word16 datalen, byte* pData ,byte comid);
static void 	Handle_HCP_DBG_Pos_Vel(word16 datalen, byte* pData ,byte comid);

int32 FindHCPHead(int validlen, STRU_UART_STREAM *glStruUart);
int32 FindRTCM3Head(int validlen, STRU_UART_STREAM *glStruUart);
int32 FindRMOHead(int validlen, STRU_UART_STREAM *glStruUart);


	
static  HCP_TASK_PARAMS ProcessHcpParams[] = 
{
	{HCP_NAV_CLASSID, 	HCP_NAV_POS_MSGID,		Handle_HCP_NAV_Pos},// NAV position		
	{HCP_NAV_CLASSID, 	HCP_NAV_VEL_MSGID,		Handle_HCP_NAV_Vel},//NAV velocity
	{HCP_NAV_CLASSID, 	HCP_NAV_TIME_UTC_MSGID,	Handle_HCP_NAV_Time_UTC},// utc time		
	{HCP_NAV_CLASSID, 	HCP_NAV_TIME_BD_MSGID,	Handle_HCP_NAV_Time_BD},//bd time
	{HCP_NAV_CLASSID, 	HCP_NAV_TIME_GPS_MSGID,	Handle_HCP_NAV_Time_GPS},//gps time
	{HCP_NAV_CLASSID, 	HCP_NAV_SYSM_MSGID,		Handle_HCP_NAV_SYSM},//TTFF, BootMode,FixMode 

	{HCP_SV_CLASSID, 	HCP_SV_GPS_EPH_MSGID,	Handle_HCP_SV_GPS_Eph},//
	
	{HCP_SYSM_CLASSID, 	HCP_SYSM_RST_MSGID,		Handle_HCP_SYSM_RST},//system reset HOT and COLD
	{HCP_SYSM_CLASSID, 	HCP_SYSM_UPARM_MSGID,	Handle_HCP_SYSM_UParm},//USR Parameters
	{HCP_SYSM_CLASSID, 	HCP_SYSM_VER,			Handle_HCP_SYSM_Ver},//Firmware version

	{HCP_DBG_CLASSID, 	HCP_DBG_POS_VEL_MSGID,	Handle_HCP_DBG_Pos_Vel},//Test Command for pos and vel error

	{HCP_CLASSID_INVALID,	HCP_MSGID_INVALID, NULL},	//invalid
};


void TaskPeriodicOutput(void)
{
	INT16 comid = 0;

	PeriodSendNovPro();

	PeriodSendRtcmPro();
	
	for(comid = 0;comid<MAX_UART_NUM;comid++)
	{
		if(pActiveCPT->SysmCptProtocol[comid].COMInterFaceMode & CHECK_DATA_HEAD_NMEA)
		{
			NMEA0183_Main(comid);
		}
	}

#ifndef _POSTPROC
#ifndef _SIMULATE
	for(comid = 0;comid<MAX_UART_NUM;comid++)
	{
		if(pActiveCPT->SysmCptProtocol[comid].HCPOutputCycle != 0 && 
			(pActiveCPT->SysmCptProtocol[comid].COMInterFaceMode & CHECK_DATA_HEAD_HCP))
		{
			Handle_HCP_Periodic_DBG_SYSM(comid);
		}
	}
#endif
#endif
	return;
}


void TaskProtocolInput(STRU_UART_STREAM *pglStruUart)
{
	static int32 hwasicMsgid=0;
	int32 lastDatalen = 0;
	int 	protocoltype = 0;
	int32 curProtocolRes = PROTO_WAITE_DATA;
	int32 validDataLen = pglStruUart->RecvData.CmdSetPos -  pglStruUart->RecvData.CmdCurPos;

	if(validDataLen < 0)
		validDataLen =  pglStruUart->RecvData.CmdSetPos + UART_RECV_BUF_LENTH -  pglStruUart->RecvData.CmdCurPos;
	
	memset( pglStruUart->CmdBuf,0,sizeof(pglStruUart->CmdBuf));
	memset(&ProtocolHCP,0,sizeof(ProtocolHCP));

	while(validDataLen > 0)
	{
		lastDatalen = validDataLen;
		if(pglStruUart->CheckProtocolHead == CHECK_DATA_HEAD_FALSE)		//find no head
		{
			curProtocolRes = CheckRecvTaskProtocolHead(&validDataLen,&protocoltype,pglStruUart, &hwasicMsgid);//检查协议头,并返回状态

			if(curProtocolRes == PROTO_PARSE_FAIL)
			{
				validDataLen--;
				ReadBufferShiftProcess((unsigned short*)&pglStruUart->RecvData.CmdCurPos,1);
			}
			else if(curProtocolRes == PROTO_PARSE_SUCCESS)
			{
				pglStruUart->CheckProtocolHead = protocoltype;
			}
			else if(curProtocolRes == PROTO_WAITE_DATA)
				break;
		}
		else	//find the head
		{
			switch(pglStruUart->CheckProtocolHead)
			{
			case CHECK_DATA_HEAD_HCP:
				{
					curProtocolRes = HCPRcvProc(&validDataLen,pglStruUart);
					ReadBufferShiftProcess((unsigned short*)&pglStruUart->RecvData.CmdCurPos,(lastDatalen - validDataLen));
				}
				break;
#ifndef _POSTPROC
#ifndef _SIMULATE
			case CHECK_DATA_HEAD_RMO:
				{
					curProtocolRes = NMEARcvProc(&validDataLen,pglStruUart);
					ReadBufferShiftProcess((unsigned short*)&pglStruUart->RecvData.CmdCurPos,(lastDatalen - validDataLen));
				}
				break;
#endif
#endif
			case CHECK_DATA_HEAD_RTCM3:
				{
					curProtocolRes = RTCMRcvProc(&validDataLen,pglStruUart);
					ReadBufferShiftProcess((unsigned short*)&pglStruUart->RecvData.CmdCurPos,(lastDatalen - validDataLen));
				}
				break;
#ifndef _POSTPROC
			case CHECK_DATA_HEAD_HWAASIC:
			{

				curProtocolRes = HWAASICRcvProc(&validDataLen,pglStruUart, hwasicMsgid);
				ReadBufferShiftProcess((unsigned short*)&pglStruUart->RecvData.CmdCurPos,(lastDatalen - validDataLen));
			}
			break;
#endif
			default:break;
			}
			if(curProtocolRes == PROTO_WAITE_DATA)
				break;
			else if(curProtocolRes == PROTO_PARSE_SUCCESS || curProtocolRes == PROTO_PARSE_FAIL)
				pglStruUart->CheckProtocolHead = CHECK_DATA_HEAD_FALSE;
		}

	}
}


int32 HCPRcvProc(INT32 *pValidDataLen,STRU_UART_STREAM *pglStruUart)
{
	INT32 	tmpReadPos = 0;
	UINT16 	payloadlen = 0;	

	if((*pValidDataLen) < HCP_NODATA_LENTH)
		return PROTO_WAITE_DATA;

	tmpReadPos = pglStruUart->RecvData.CmdCurPos;
	ReadBufferShiftProcess((unsigned short*)&tmpReadPos,4);//检查paylen低byte是否溢出

	//?
	payloadlen = (pglStruUart->RecvData.Buf[tmpReadPos]&0xFF) ;
	ReadBufferShiftProcess((unsigned short*)&tmpReadPos,1);//检查paylen高byte是否溢出
	payloadlen = (((pglStruUart->RecvData.Buf[tmpReadPos])&0xFF)<< 8) | payloadlen ;

	//memcpy(&payloadlen, glStruUart.RecvBuf+tmpCmdReadPos, 2);

	if(payloadlen > HCP_MAXDATA_LENTH)	//check payload lenght valid or not
	{
		(*pValidDataLen) -= 1;
		return PROTO_PARSE_FAIL;
	}
	if((*pValidDataLen) < payloadlen + HCP_NODATA_LENTH)	//need to wait data
		return PROTO_WAITE_DATA;

	ReadBufferShiftProcess((unsigned short*)&(tmpReadPos),payloadlen + 1 + 2);

	if(tmpReadPos > pglStruUart->RecvData.CmdCurPos)
		memcpy(pglStruUart->CmdBuf, pglStruUart->RecvData.Buf + pglStruUart->RecvData.CmdCurPos, payloadlen + HCP_NODATA_LENTH);
	else if(tmpReadPos < pglStruUart->RecvData.CmdCurPos)
	{
		memcpy(pglStruUart->CmdBuf,  pglStruUart->RecvData.Buf + pglStruUart->RecvData.CmdCurPos, UART_RECV_BUF_LENTH - pglStruUart->RecvData.CmdCurPos);
		memcpy(pglStruUart->CmdBuf + (UART_RECV_BUF_LENTH - pglStruUart->RecvData.CmdCurPos),  pglStruUart->RecvData.Buf, tmpReadPos);
	}

	if(HCPParityCheck(payloadlen,pglStruUart->CmdBuf + 2))	//parse success
	{
		//add despath to handle
		StoreHCPDate(pglStruUart->CmdBuf);//保存数据到结构体

		RecvProtoHCPHandleProcess(pglStruUart);//数据处理分析程序
		(*pValidDataLen) -= (payloadlen + HCP_NODATA_LENTH);		
		return PROTO_PARSE_SUCCESS;
	}
	else	//parse failed
	{
		(*pValidDataLen) -= 1;
		return PROTO_PARSE_FAIL;
	}
}


	/*
	*	读指针在偏移len 长度后若超过串口buffer长度,
	*	则从buffer[0]开始
	*/
int ReadBufferShiftProcess(unsigned short *CmdReadPos, int len)
{
	(*CmdReadPos) += len;

	if((*CmdReadPos)  >= UART_RECV_BUF_LENTH)
	{
		(*CmdReadPos) = (*CmdReadPos) - UART_RECV_BUF_LENTH;
		
		return CHECK_READ_BUFFER_OVERFLOW;
	}

	return CHECK_READ_BUFFER_NO_OVERFLOW;
}


int32 FindHCPHead(int validlen, STRU_UART_STREAM *glStruUart)
{
	int32 flag=PROTO_PARSE_FAIL;
	int32 tmpCmdReadPos = glStruUart->RecvData.CmdCurPos;
	byte tmpdata[HCP_HEAD_NUM_PROTO]={0,0};
	
	if(validlen < HCP_HEAD_NUM_PROTO)
	{
		flag=PROTO_WAITE_DATA;
		return flag;
	}

	tmpdata[0]=glStruUart->RecvData.Buf[tmpCmdReadPos];
	ReadBufferShiftProcess((unsigned short*)&tmpCmdReadPos,1);
	tmpdata[1]=glStruUart->RecvData.Buf[tmpCmdReadPos];

	if((tmpdata[0]==HEAD1_HCP) && (tmpdata[1]==HEAD2_HCP))
		flag = PROTO_PARSE_SUCCESS;
	else
		flag = PROTO_PARSE_FAIL;
	
	return flag;
}

int32 FindRMOHead(int validlen, STRU_UART_STREAM *glStruUart)
{
	int32 flag=PROTO_PARSE_FAIL;
	int32 tmpCmdReadPos = glStruUart->RecvData.CmdCurPos;
	byte tmpdata[RMO_HEAD_NUM_PROTO]={0,0,0};

	if(validlen < RMO_HEAD_NUM_PROTO)
	{
		flag=PROTO_WAITE_DATA;
		return flag;
	}

	tmpdata[0]=glStruUart->RecvData.Buf[tmpCmdReadPos];
	ReadBufferShiftProcess((unsigned short*)&tmpCmdReadPos,1);
	tmpdata[1]=glStruUart->RecvData.Buf[tmpCmdReadPos];
	ReadBufferShiftProcess((unsigned short*)&tmpCmdReadPos,1);
	tmpdata[2]=glStruUart->RecvData.Buf[tmpCmdReadPos];

	if((tmpdata[0]==HEAD_NMEA) && (tmpdata[1]=='C') && (tmpdata[2]=='C'))
		flag = PROTO_PARSE_SUCCESS;
	else
		flag = PROTO_PARSE_FAIL;
	
	return flag;
}

int32 FindRTCM3Head(int validlen, STRU_UART_STREAM *glStruUart)
{
	int32 flag=PROTO_PARSE_FAIL;
	int32 tmpCmdReadPos = glStruUart->RecvData.CmdCurPos;
	byte tmpdata[RMO_HEAD_NUM_PROTO]={0,0};

	if(validlen < RTCM_HEAD_NUM_PROTO)
	{
		flag=PROTO_WAITE_DATA;
		return flag;
	}

	tmpdata[0]=glStruUart->RecvData.Buf[tmpCmdReadPos];
	ReadBufferShiftProcess((unsigned short*)&tmpCmdReadPos,1);
	tmpdata[1]=glStruUart->RecvData.Buf[tmpCmdReadPos];

	if((tmpdata[0]==HEAD1_RTCM) && ((tmpdata[1]&0xfc)==HEAD2_RTCM))
		flag = PROTO_PARSE_SUCCESS;
	else
		flag = PROTO_PARSE_FAIL;
	
	return flag;
}


	/*
	*	检测协议头,并返回状态
	*	CHECK_DATA_HEAD_TRUE表示找到协议头
	*	HECK_DATA_HEAD_FALSE表示未找到协议头
	*
	*/
int CheckRecvTaskProtocolHead(int *validlen,int *protocoltype,STRU_UART_STREAM *glStruUart, int32* pMsgid)
{
	int32 flag_hcp = 0, flag_rmo = 0, flag_rtmc3=0, flag_hwaasic=0;

	//is HCP?
	flag_hcp = FindHCPHead(*validlen, glStruUart);
	if(flag_hcp == PROTO_PARSE_SUCCESS)
	{
		*protocoltype = CHECK_DATA_HEAD_HCP;
		return flag_hcp;
	}

	//is RMO?
	flag_rmo = FindRMOHead(*validlen, glStruUart);
	if(flag_rmo == PROTO_PARSE_SUCCESS)
	{
		*protocoltype = CHECK_DATA_HEAD_RMO;
		return flag_rmo;
	}

	//is rtcm
	flag_rtmc3 = FindRTCM3Head(*validlen, glStruUart);
	if(flag_rtmc3 == PROTO_PARSE_SUCCESS)
	{
		*protocoltype = CHECK_DATA_HEAD_RTCM3;
		return flag_rtmc3;
	}
#ifndef _POSTPROC	
	//is hwaasic
	flag_hwaasic = FindHWAASICHead(*validlen, glStruUart , pMsgid);
	if(flag_hwaasic == PROTO_PARSE_SUCCESS)
	{
		*protocoltype = CHECK_DATA_HEAD_HWAASIC;
		return flag_hwaasic;
	}

	if((flag_hcp==PROTO_PARSE_FAIL) && (flag_rmo==PROTO_PARSE_FAIL) && (flag_rtmc3==PROTO_PARSE_FAIL) &&(flag_hwaasic==PROTO_PARSE_FAIL))
#else
	if((flag_hcp==PROTO_PARSE_FAIL) && (flag_rmo==PROTO_PARSE_FAIL) && (flag_rtmc3==PROTO_PARSE_FAIL))
#endif
	{
		return PROTO_PARSE_FAIL;
	}
	else
		return PROTO_WAITE_DATA;
}


	/*
	*	检测协议长度,并返回状态
	*	GET_DATA_LEN_TRUE表示得到协议有效长度
	*	GET_DATA_LEN_FALSE表示得到协议有效长度
	*
	*/
int CheckHCPPayloadLen(char *CmdBuf,unsigned short *PayLoadLen,unsigned short  CmdReadPos,unsigned short  CmdWritePos,STRU_UART_STREAM *glStruUart )
{
	//判断Payload Length是否溢出
	
	unsigned short tmpCmdReadPos = 0;
	tmpCmdReadPos = CmdReadPos;
	ReadBufferShiftProcess((unsigned short*)&tmpCmdReadPos,4);//检查paylen低byte是否溢出

	//?
	*PayLoadLen = (glStruUart->RecvData.Buf[tmpCmdReadPos]&0xFF) ;
	ReadBufferShiftProcess((unsigned short*)&tmpCmdReadPos,1);//检查paylen高byte是否溢出
	*PayLoadLen = ((glStruUart->RecvData.Buf[tmpCmdReadPos]&0xFF) << 8) | (*PayLoadLen);
		
	if(CmdWritePos>CmdReadPos)
	{
		if((CmdWritePos-CmdReadPos) >= (HCP_NODATA_LENTH + (*PayLoadLen)))//检查有效协议是否溢出
		{ 
			memcpy(CmdBuf,&glStruUart->RecvData.Buf[CmdReadPos],HCP_NODATA_LENTH + (*PayLoadLen));
		
			return GET_DATA_LEN_TRUE;
		}
		else 
			return GET_DATA_LEN_FALSE;		
	}	
	else if((UART_RECV_BUF_LENTH - CmdReadPos + 1 + CmdWritePos + 1)>=(HCP_NODATA_LENTH + *PayLoadLen))
	{	
		memcpy(CmdBuf,&glStruUart->RecvData.Buf[CmdReadPos],UART_RECV_BUF_LENTH - CmdReadPos + 1);
		
		memcpy(&CmdBuf[UART_RECV_BUF_LENTH - CmdReadPos + 1],&glStruUart->RecvData.Buf[0],CmdWritePos + 1);
	
		return GET_DATA_LEN_TRUE;
	}

	else 
		return GET_DATA_LEN_FALSE;
}

	/*
	*	数据校验正确后保存数据到结构体
	*/
void StoreHCPDate(char *CmdBuf)
{
	char     RecvBuf[UART_SEND_BUF_LENTH];  
	memset(RecvBuf,0,sizeof(RecvBuf));	
	ProtocolHCP.ClassId = CmdBuf[HCP_CLASSID_OFFSET];
	ProtocolHCP.MessageId = CmdBuf[HCP_MESSAGEID_OFFSET];
	ProtocolHCP.DataLenth = (CmdBuf[HCP_PAYLOADLENTH_OFFSET]&0xFF) |((CmdBuf[HCP_PAYLOADLENTH_OFFSET + 1]&0xFF) << 8);
	ProtocolHCP.HCPData = &CmdBuf[HCP_PAYLOAD_OFFSET];
	
}
	/*
	*	协议数据校验
	*	CHECK_DATA_TRUE:校验通过
	*	CHECK_DATA_FALSE:校验未通过
	*
	*/
bool HCPParityCheck(unsigned short PayLoadLen,char *CmdBuf)
{
	unsigned char  CK_1 = 0 , CK_2 = 0 ;
	int i = 0;
	unsigned char  ck1,ck2;
	
	for( i=0;i<PayLoadLen+4;i++)
	{		
		CK_1 = CK_1 + CmdBuf[i];
		CK_2 = CK_2 + CK_1;	
	}
	ck1 = CmdBuf[PayLoadLen + 4];
	ck2 = CmdBuf[PayLoadLen + 5];

	if((ck1 == CK_1)&&(ck2 == CK_2))
		return TRUE;
	else
		return FALSE;
}


void HCPDataCheckConfig( char *CmdBuf,unsigned short HCPLen,unsigned char *CK_1,unsigned char *CK_2  )
{	
	int i = 0;
	
	for(i=0; i<HCPLen ; i++)
	{
		*CK_1 = *CK_1 + CmdBuf[i];
		*CK_2 = *CK_2 + *CK_1;	
	}
}



	/*	
	*	数据分析处理模块，包含二维函数指针指向不同的处理函数
	*/
void RecvProtoHCPHandleProcess(STRU_UART_STREAM *pglStruUart)
{ 
	HCP_TASK_PARAMS HcpParams;	
	int i;
	bool AckInq = FALSE;
	for(i=0;  ;i++)
	{
		HcpParams = ProcessHcpParams[i];	
		if ((HcpParams.ClassId == ProtocolHCP.ClassId) && (HcpParams.MessageId == ProtocolHCP.MessageId))
		{
			AckInq = TRUE;	
			break;	
		}
		else if((HcpParams.ClassId == HCP_CLASSID_INVALID) || (HcpParams.MessageId == HCP_MSGID_INVALID))
			break;
	}

	if(AckInq)
	{
		HCPProtocolACK(ProtocolHCP.ClassId, ProtocolHCP.MessageId,pglStruUart->comid);
		(*(HcpParams.ProtocolHCPProcess))(ProtocolHCP.DataLenth, (byte*)(ProtocolHCP.HCPData),pglStruUart->comid);
	}
	else
		HCPProtocolNACK(ProtocolHCP.ClassId, ProtocolHCP.MessageId,pglStruUart->comid);
		
	return;
}

	/*
	*	
	*	NAV协议数据分析处理模块
	*
	*/
void Handle_HCP_NAV_Pos(word16 datalen, byte* pData ,byte comid)
{
	NAV_POS navpos = {0};
	int32 WN = 0;
	WGS wgspos = {0};	
	ECEF Pos = {0} , Vel = {0};
	FIX_DOP DOP = {0};	
	double tow = 0.0;
	
	if(datalen == 0)		
	{		
		//get navpos data here.
		getRcvrInfo(&Pos, &Vel,NULL, &DOP);
		ECEF2WGS(&Pos, &wgspos);
		navpos.Posx = (int32)(Pos.x*100.0);
		navpos.Posy = (int32)(Pos.y*100.0);
		navpos.Posz = (int32)(Pos.z*100.0);
		navpos.Longitude = (int32)(wgspos.lon*R2D*1000.0);
		navpos.Latitude = (int32)(wgspos.lat*R2D*1000.0);
		navpos.Altitude = (int32)(wgspos.alt*100.0);
		HCPSendData(HCP_NAV_CLASSID, HCP_NAV_POS_MSGID, sizeof(NAV_POS),(byte*)&navpos,comid);
	}
	else
	{
		if(pData == NULL)
			return;
		memcpy(&navpos,pData,sizeof(NAV_POS));
		// set navpos date here.
		Pos.x = (double)(navpos.Posx/100.0);
		Pos.y = (double)(navpos.Posy/100.0);
		Pos.z = (double)(navpos.Posz/100.0);	
		wgspos.lon = (double)(navpos.Longitude*D2R/1000.0);
		wgspos.lat = (double)(navpos.Latitude*D2R/1000.0);
		wgspos.alt = (double)(navpos.Altitude/100.0);

		if(GetCurNavSysTime(NAV_SYS_NONE,GPSTIME_SYNC_SRC_REFBIT_RFCNT,&tow,&WN))
		{
			tow = WN * SECONDS_IN_WEEK + tow;
			setRcvrHistoricalPos(&Pos,&tow,HISTORICAL_POS_LEVEL_40KM);
		}
	}
	return;
}
	/*
	*	
	*	SV协议数据分析处理模块
	*	分为len=0，len=4和len>4;
	*/
void Handle_HCP_SV_GPS_Eph(word16 datalen, byte* pData, byte comid)
{
	int i = 0;
	int svBufLen = 0;
	word32 svmap=0;

	NAV_TIME		NavTime;
	
	if(datalen == 0)	//poll
	{
		memset(HCPDataBuf,0,sizeof(HCPDataBuf));
		svBufLen += HCP_PAYLOAD_OFFSET+4;
		for(i=0; i<MaxGpsSvID; i++)
		{	
			if(1)	//if (getGpsSVEph(i, &SvEPH))
			{
				svmap |= (0x1<<i);
				memcpy(HCPDataBuf + svBufLen,&NavTime,sizeof(NAV_TIME));
			}
			svBufLen += sizeof(NAV_TIME);
		}

		HCPSendData(HCP_SV_CLASSID, HCP_SV_GPS_EPH_MSGID, svBufLen, (byte*)(HCPDataBuf+HCP_PAYLOAD_OFFSET),comid);
	}
	else if(datalen == 4)	//poll
	{
		if(pData == NULL)
			return;
		
		memset(HCPDataBuf,0,sizeof(HCPDataBuf));
		svBufLen += HCP_PAYLOAD_OFFSET+4;
		svmap = *((word32*)pData);
		for(i=0; i<MaxGpsSvID; i++)
		{	
			if(svmap & (1<<i))	
			{
				//if (getGpsSVEph(i, &SvEPH))
				svmap |= (0x1<<i);
				memcpy(HCPDataBuf+svBufLen,&NavTime,sizeof(NAV_TIME));
				svBufLen += sizeof(NAV_TIME);
			}	
		}

		HCPSendData(HCP_SV_CLASSID, HCP_SV_GPS_EPH_MSGID, svBufLen, (byte*)(HCPDataBuf+HCP_PAYLOAD_OFFSET),comid);
	}
	else if(datalen > 4)	//set
	{
		return;
	}

	return;
}

	/*
	*	
	*	DBGSYS协议数据分析处理模块
	*
	*/
void Handle_HCP_Periodic_DBG_SYSM(byte comid)
{
	static word32 opcnt=0; 
	int offset = 0;
	int i = 0;

	DBG_SYSM_ACQ			DBGAcq;
	DBG_SYSM_TRK			DBGTrk;
	DBG_SYSM_MISC			DBGMisc;

	if(((opcnt++) % pActiveCPT->SysmCptProtocol[comid].HCPOutputCycle) != 0)
		return;
	
	memset(HCPDataBuf,0,sizeof(HCPDataBuf));
	memset(&DBGAcq, 0, sizeof(DBGAcq));
	
	offset += HCP_PAYLOAD_OFFSET;

	GetACQChDebugData(0, &DBGAcq);
	offset += sizeof(DBGAcq);	

	for(i = 0;i < MAXCHANNELS;i++)
	{	
		if(GetTRKChDebugData(i,&DBGTrk))
		{
			DBGAcq.validtrkcnt++;
			memcpy(&(HCPDataBuf[offset]),&DBGTrk,sizeof(DBGTrk));
			offset += sizeof(DBGTrk);	
		}
	}
	memcpy(&(HCPDataBuf[HCP_PAYLOAD_OFFSET]),&DBGAcq,sizeof(DBGAcq));

	GetMiscDebugData(&DBGMisc);
	memcpy(&HCPDataBuf[offset],&DBGMisc, sizeof(DBGMisc));
	offset += sizeof(DBGMisc);
	
	HCPSendData(HCP_DBG_CLASSID, HCP_DBG_SYSM_MSGID, offset-HCP_PAYLOAD_OFFSET, (byte*)(HCPDataBuf+HCP_PAYLOAD_OFFSET),comid);
	
}
/*
	*	
	*	SYSMRST协议数据分析处理模块
	*
	*/
void Handle_HCP_SYSM_RST(word16 datalen, byte* pData, byte comid)
{
#ifndef _POSTPROC
#ifndef _SIMULATE
	SYSM_RST SysmRst = {0};
	if(datalen != 2)
		return;
	if(pData == NULL)
		return;
	memcpy((byte*)&SysmRst,pData,sizeof(SYSM_RST));
	
	switch(SysmRst.Mode)
	{
		case 1://cold start
			SW_Restart_Process(COLD_START,pActiveCPT->SysmCptWorkConfig.NavFreqPoint,SysmRst.FlashOpFlag);
			break;
		case 2://hot start 
			SW_Restart_Process(HOT_START,pActiveCPT->SysmCptWorkConfig.NavFreqPoint,0);
			break;
		case 3:
			break;
		default:
			break;
	}

	return;
#endif
#endif
}

void Handle_HCP_NAV_Vel(word16 datalen, byte* pData ,byte comid)
{
	NAV_VEL Navvel = {0};
	ECEF Pos = {0};
	ECEF Vel = {0};
	FIX_DOP DOP = {0};	
	
	if(datalen == 0)		
	{		
		//get Navvel data here.	
		getRcvrInfo(&Pos, &Vel,NULL, &DOP);
		Navvel.Velx = (int32)(Vel.x*100.0);
		Navvel.Vely = (int32)(Vel.y*100.0);
		Navvel.Velz = (int32)(Vel.z*100.0);
		HCPSendData(HCP_NAV_CLASSID, HCP_NAV_VEL_MSGID, sizeof(NAV_VEL),(byte*)(&Navvel),comid);
	}
	else
	{
		if(pData == NULL)
			return;

	}
	return;

}


void Handle_HCP_NAV_Time_UTC(word16 datalen, byte* pData ,byte comid)
{
	UTC_TIME_DATA UtctimeData = {0};
	UTC_TIME UTCTime;

	GetTimeOfFixUTC(&UTCTime, 0.005);

	if(datalen == 0)		
	{		
		//get UTCTime data here.	
		UtctimeData.Year = (UINT16)UTCTime.year;
		UtctimeData.Mon  = (UINT8)UTCTime.mon;
		UtctimeData.Day  = (UINT8)UTCTime.day;
		UtctimeData.Hour = (UINT8)UTCTime.hour;
		UtctimeData.Min  = (UINT8)UTCTime.min;
		UtctimeData.Sec  = (UINT8)UTCTime.sec;
		HCPSendData(HCP_NAV_CLASSID, HCP_NAV_TIME_UTC_MSGID, sizeof(UTC_TIME_DATA),(byte*)(&UtctimeData),comid);
	}
	else
	{
		if(pData == NULL)
			return;

	}
	return;
}


void Handle_HCP_NAV_Time_BD(word16 datalen, byte* pData ,byte comid)
{
	NAV_TIME BdTime = {0};
	double tow = 0.0;
	int32 WN = 0;
	
	//get BDTime data here.	
	if(datalen == 0)
	{
		GetCurNavSysTime(NAV_SYS_BD,GPSTIME_SYNC_SRC_REFBIT_RFCNT,&tow,&WN);

		BdTime.Tow = (UINT32)tow;
		BdTime.Wn  = (UINT32)WN;	
		HCPSendData(HCP_NAV_CLASSID, HCP_NAV_TIME_BD_MSGID, sizeof(NAV_TIME),(byte*)(&BdTime),comid);
	}	

	return;
}

void Handle_HCP_NAV_Time_GPS(word16 datalen, byte* pData ,byte comid)
{
	NAV_TIME GpsTime = {0};
	double tow = 0.0;
	int32 WN = 0;
	//get GPSTime data here.	
	if(datalen == 0)
	{
		GetCurNavSysTime(NAV_SYS_GPS,GPSTIME_SYNC_SRC_REFBIT_RFCNT,&tow,&WN);		
		GpsTime.Tow = (UINT32)tow;
		GpsTime.Wn  = (UINT32)WN;	
		HCPSendData(HCP_NAV_CLASSID, HCP_NAV_TIME_GPS_MSGID, sizeof(NAV_TIME),(byte*)(&GpsTime),comid);
	}	
	return;
	
}

void Handle_HCP_NAV_SYSM(word16 datalen, byte* pData ,byte comid)
{
#ifndef _POSTPROC
#ifndef _SIMULATE
	NAV_SYSM NavSysm = {0};
	if(datalen != 0)
		return;
	//get NavSysm data here.
	//TTFF /= 1000; // TTFF  uint s
	NavSysm.TTFF = TTFF;
	if(pActiveBackupRegion->BootMode == COLD_START)
		NavSysm.BootMode = 1;
	else if(pActiveBackupRegion->BootMode == HOT_START)
		NavSysm.BootMode = 2;
		
	NavSysm.FixMode = pActiveCPT->SysmCptWorkConfig.NavFreqPoint;
	
	HCPSendData(HCP_NAV_CLASSID, HCP_NAV_SYSM_MSGID, sizeof(NAV_SYSM),(byte*)(&NavSysm),comid);
#endif
#endif
}
#ifdef __HOT_START_DELAY
extern double delaytimesec;
#endif
void Handle_HCP_SYSM_UParm(word16 datalen, byte* pData ,byte comid)
{
	SYSM_CPT SysmCpttmp = {0};
	boolean res = FALSE;

	if(pData == NULL)
		return;
	
	if(datalen == 1)
	{
		switch(pData[0])
		{
			case 1:	//get Factory param 
				HCPSendData(HCP_SYSM_CLASSID, HCP_SYSM_UPARM_MSGID, sizeof(SYSM_CPT),(byte*)&FactoryCPT,comid);
				break;
				
			case 2:	// get Active param
				pActiveCPT->SysmCptWorkConfig.Command = pData[0];
				addCPTVerifyInfo(pActiveCPTRegion);
				HCPSendData(HCP_SYSM_CLASSID, HCP_SYSM_UPARM_MSGID, sizeof(SYSM_CPT),(byte*)(pActiveCPT),comid);
				break;
#ifndef _POSTPROC
#ifndef _SIMULATE				
			case 3://get Flash param
				CopyBackupInfo(CPY_CPT_FLG_FLASH2RAM);
				HCPSendData(HCP_SYSM_CLASSID, HCP_SYSM_UPARM_MSGID, sizeof(SYSM_CPT),(byte*)(&(pSharedDDRCPTRegion->sysm_cpt)),comid);

				break;
				
			case 4://set Factory -> active
				if(CheckParam((SYSM_CPT*)&FactoryCPT))
				{
					SetBaudrate((SYSM_CPT*)&FactoryCPT);
					memcpy((byte*)pActiveCPT,(byte*)&FactoryCPT,sizeof(SYSM_CPT));
					addCPTVerifyInfo(pActiveCPTRegion);
				}
				break;
				
				
			case 5://set Flash->active
				CopyBackupInfo(CPY_CPT_FLG_FLASH2RAM);
				res = checkCPTAvail(pSharedDDRCPTRegion);
				if(res)
					memcpy(pActiveCPTRegion, pSharedDDRCPTRegion, sizeof(SYS_CPT_REGION));
				break;
				
			case 6://set active->Flash
				if(gFlashCPTBackUpFlag==FALSE)
				{
					g_u08_FlashWriteFlag |=FLASH_FLG_WRITE_CPT;
					SEM_post(&SEM5);
				}
				break;
				
			case 7://clear Flash and shareDDR data
				if(gFlashCPTBackUpFlag==FALSE)
				{
					g_u08_FlashWriteFlag |=FLASH_FLG_ERASE_CPT;
					SEM_post(&SEM5);
				}
				break;
#endif
#endif
			default:
				break;
		}

	}
	else if(datalen > 1)
	{
		switch(pData[0])
		{
#ifndef _POSTPROC
#ifndef _SIMULATE
			case 1://set Active param by usr
#ifdef __HOT_START_DELAY

				memcpy(&delaytimesec,(byte*)&pData[48],sizeof(double));
#else
				memcpy((byte*)&SysmCpttmp,(byte*)pData,sizeof(SYSM_CPT));
				if(CheckParam(&SysmCpttmp))
				{
					SetBaudrate(&SysmCpttmp);
					memcpy((byte*)pActiveCPT,(byte*)&SysmCpttmp,sizeof(SYSM_CPT));
					addCPTVerifyInfo(pActiveCPTRegion);
				}
#endif								
				break;

			case 2://set Active shareDDR and Flash Param by usr
				if(gFlashCPTBackUpFlag==FALSE)
				{
					memcpy((byte*)&SysmCpttmp,pData,sizeof(SYSM_CPT));
					if(CheckParam(&SysmCpttmp))
					{
						SetBaudrate(&SysmCpttmp);
						memcpy((byte*)pActiveCPT,(byte*)&SysmCpttmp,sizeof(SYSM_CPT));
						addCPTVerifyInfo(pActiveCPTRegion);

						g_u08_FlashWriteFlag |=FLASH_FLG_WRITE_CPT;
						SEM_post(&SEM5);
					}
				}
				break;
#endif	
#endif
			default:
				break;
		}
	}

}
	
void Handle_HCP_SYSM_Ver(word16 datalen, byte* pData ,byte comid)
{
	if(datalen == 0)
	{
		HCPSendData(HCP_SYSM_CLASSID, HCP_SYSM_VER, sizeof(byte)*50,(byte*)VersionInfo,comid);
	}	
	return;

}

void Handle_HCP_DBG_Pos_Vel(word16 datalen, byte* pData ,byte comid)
{
	DBG_POS_VEL testPosVel;
	ECEF pos={0.0,};
	ECEF vel={0.0,};
	
	if(pData == NULL)
		return;
	if(datalen != 0)
	{
		memcpy(&testPosVel,pData,sizeof(DBG_POS_VEL));
		pos.x = testPosVel.Posx/10.0;
		pos.y = testPosVel.Posy/10.0;
		pos.z = testPosVel.Posz/10.0;

		vel.x = testPosVel.Velx/100.0;
		vel.y = testPosVel.Vely/100.0;
		vel.z = testPosVel.Velz/100.0;

		SetTestPosVel(&pos, &vel);
	}

	return;
}

	/*
	*	
	*	协议响应ACK模块
	*
	*/

void HCPProtocolACK(byte classid, byte msgid,byte comid)
{	
	byte tmp[2];
	
	tmp[0]=classid;
	tmp[1]=msgid;

	HCPSendData(HCP_ACK_CLASSID, HCP_ACK_ACK_MSGID, 2, tmp,comid);
}

	/*
	*	
	*	协议响应NACK模块
	*
	*/
void HCPProtocolNACK(byte classid, byte msgid,byte comid)
{	
	byte tmp[2];
	
	tmp[0]=classid;
	tmp[1]=msgid;

	HCPSendData(HCP_ACK_CLASSID, HCP_ACK_NACK_MSGID, 2, tmp,comid);
}
	
void HCPSendData(byte classid, byte msgid, word16 payloadlen, byte* pPayload, byte comid)
{
	
	unsigned char 	CK_1 = 0;
	unsigned char 	CK_2 = 0;
	HCPDataBuf[HCP_HEAD1_OFFSET] = HEAD1_HCP;
	HCPDataBuf[HCP_HEAD2_OFFSET] = HEAD2_HCP;
	HCPDataBuf[HCP_CLASSID_OFFSET] = classid;
	HCPDataBuf[HCP_MESSAGEID_OFFSET] = msgid;
	HCPDataBuf[HCP_PAYLOADLENTH_OFFSET] = payloadlen&0XFF;
	HCPDataBuf[HCP_PAYLOADLENTH_OFFSET + 1] = ((payloadlen >> 8)&0XFF);

	memcpy(HCPDataBuf+HCP_PAYLOAD_OFFSET, pPayload, payloadlen);
	
	HCPDataCheckConfig(HCPDataBuf + HCP_CLASSID_OFFSET,(unsigned short)(payloadlen + 4), &CK_1, &CK_2 );
	
	HCPDataBuf[payloadlen + HCP_NODATA_CK1_OFFSET] = CK_1;
	HCPDataBuf[payloadlen + HCP_NODATA_CK2_OFFSET] = CK_2;
#ifndef _POSTPROC
	if(pActiveCPT->SysmCptProtocol[comid].COMInterFaceMode & CHECK_DATA_HEAD_HCP)
		UART_Send_Buff(comid, HCPDataBuf, payloadlen + HCP_NODATA_LENTH);	
#endif
}

#if 0
bool UartSendId(UINT16 comid, unsigned char ProType)
{
	switch(comid)
	{
		case 0:
			if(pActiveCPT->SysmCptComPort.COM0InterFaceMode & ProType)
				return TRUE;
			else
				return FALSE;
		case 1:
			if(pActiveCPT->SysmCptComPort.COM1InterFaceMode & ProType)
				return TRUE;
			else
				return FALSE;
		case 2:
			if(pActiveCPT->SysmCptComPort.COM2InterFaceMode & ProType)
				return TRUE;
			else
				return FALSE;
		default:
			return FALSE;
	}
}
#endif
