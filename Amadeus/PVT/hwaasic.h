#ifndef _HWAASIC_H
#define _HWAASIC_H

#include <string.h>
#include "global.h"


#define HWAASIC_HEAD_MINLEN	4
#define HWAASIC_MAX_LENTH 100
#define HWAASIC_MAX_ITEMS 20
#define HWAASIC_SINGLE_PARAM_LENTH	20
#define COUNT_HWAASICPARMS	29

#define MAX_HWAAISC_LENTH	6
#define	ACK_MSG_MAX_LENTH	50
#define MESID_MAX_LENTH		13

#define HWASIC_CMD_OK			0
#define HWASIC_CMD_OUTRANGE		1
#define HWASIC_CMD_INVALID		2


#define ECUTOFF_MSG_LENTH	0x07
#define BDSECUTOFF_MSGID_LENTH	0x0a
#define GLOECUTOFF_MSG_LENTH	0x0a
#define EXTERNALCLOCK_MSG_LENTH	0x0d
#define COM_MSG_LENTH	0x03
#define INTERFACEMODE_MSG_LENTH	0x0d
#define FRESET_MSG_LENTH 	0x06
#define RESET_MSG_LENTH	0x05
#define SAVECONFIG_MSG_LENTH	0x0a
#define IPCONFIG_MSG_LENTH	0x08
#define NETPORTCONFIG_MSG_LENTH	0x0d
#define ADJUST1PPS_MSG_LENTH	0x0a
#define BASESTATIONID_MSG_LENTH	0x0d
#define POSAVE_MSG_LENTH	0x06
#define MOVINGBASESTATION_MSG_LENTH	0x11
#define FIX_MSG_LENTH	0x03
#define ANTENNAHEIGHT_MSG_LENTH	0x0d
#define RTKCOMMAND_MSG_LENTH	0x0a
#define RTKDYNAMICS_MSG_LENTH	0x0b
#define RTKTIMEOUT_MSG_LENTH	0x0a
#define RTCMTIMEOUT_MSG_LENTH	0x0b
#define LOG_MSG_LENTH	0x03
#define UNLOG_MSG_LENTH	0x05
#define UNLOGALL_MSG_LENTH	0x08
#define SNRMASK_MSG_LENTH	0x07
#define FIXNAVFREQPOINT_MSG_LENTH	0x0f
#define ASSIGNALL_MSG_LENTH	0x09
#define UNASSIGNALL_MSG_LENTH 	0x0b
#define SVBITMAP_MSG_LENTH	0x08





typedef struct _PROTOCOL_HWAASIC
{	
	CX			HEAD[HWAASIC_SINGLE_PARAM_LENTH];		//¨¤¨¤D¨ª?¨¹¨¢?
	U1			MessageId;		//D-¨°¨¦?¨¹¨¢?
	CX*			HWAASICData;			//¨ºy?Y?¨²¨¨Y
}PROTOCOL_HWAASIC;



typedef struct _PROTOCOL_HWAASIC_TASK_PARAMS
{
	CX*			Head;		//head
	int32		HeadLength;	// head length
	void		(*HWAAsicCMDTbl)(word16 datalen, char* pData, byte comid);			//process func
}HWAASIC_TASK_PARAMS;

typedef struct {
	word16 	data_len;
	int32		msgid;	// table idx
	CX 	p_data[HWAASIC_MAX_LENTH];
}HWAASIC_PACKAGE_INFO;

#ifdef __cplusplus
extern "C" {
#endif

int32 FindHWAASICHead(int validlen, STRU_UART_STREAM *pglStruUart ,int32 *pMsgid);
INT32 HWAASICRcvProc(INT32 *pValidDataLen,STRU_UART_STREAM *pglStruUart , int32 msgid);
void HWA_ClearBitWord256(word256* wd, unsigned int i);
void HWA_SetBitWord256(word256* wd, unsigned int i);
#ifdef __cplusplus
}
#endif


#endif


