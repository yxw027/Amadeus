/**
* @file  rtcm3pro.c
*
* @brief 
*		This module is used to create and parse RTCM 3.x messages
*		
*		created by juan. 2013.July.8.
*/

#include "define.h"
#include "typedefine.h"
#include "rtcm3.h"
#include "hcp.h"
#include "cfgpara.h"
#include "UartFunc.h"
#include "rtk.h"
#include "pvt.h"
#include "constdef.h"




static void RTCM3_dispatch_package(RTCM3_PACKAGE_INFO *p_pkg_info,STRU_UART_STREAM *glStruUart);

static void SetMSMDataSVIdx(RTCM3_MSM_DATA_INFO* pMSMDataInfo, int32 svid, int8 idx);
static unsigned int crc24q( unsigned char *buff, int len);
static bool checkRTCM3Msgid(word16 msgid,word16 payloadlen);
byte RTCM3OutputDataBuf[RTCM3_PAYLOAD_MAX_LEN+RTCM3_MIN_PKG_LEN];


//CRC_24Q generate multinomoal: 0x1864CFB
static const unsigned int tbl_CRC24Q[]={
    0x000000,0x864CFB,0x8AD50D,0x0C99F6,0x93E6E1,0x15AA1A,0x1933EC,0x9F7F17,
    0xA18139,0x27CDC2,0x2B5434,0xAD18CF,0x3267D8,0xB42B23,0xB8B2D5,0x3EFE2E,
    0xC54E89,0x430272,0x4F9B84,0xC9D77F,0x56A868,0xD0E493,0xDC7D65,0x5A319E,
    0x64CFB0,0xE2834B,0xEE1ABD,0x685646,0xF72951,0x7165AA,0x7DFC5C,0xFBB0A7,
    0x0CD1E9,0x8A9D12,0x8604E4,0x00481F,0x9F3708,0x197BF3,0x15E205,0x93AEFE,
    0xAD50D0,0x2B1C2B,0x2785DD,0xA1C926,0x3EB631,0xB8FACA,0xB4633C,0x322FC7,
    0xC99F60,0x4FD39B,0x434A6D,0xC50696,0x5A7981,0xDC357A,0xD0AC8C,0x56E077,
    0x681E59,0xEE52A2,0xE2CB54,0x6487AF,0xFBF8B8,0x7DB443,0x712DB5,0xF7614E,
    0x19A3D2,0x9FEF29,0x9376DF,0x153A24,0x8A4533,0x0C09C8,0x00903E,0x86DCC5,
    0xB822EB,0x3E6E10,0x32F7E6,0xB4BB1D,0x2BC40A,0xAD88F1,0xA11107,0x275DFC,
    0xDCED5B,0x5AA1A0,0x563856,0xD074AD,0x4F0BBA,0xC94741,0xC5DEB7,0x43924C,
    0x7D6C62,0xFB2099,0xF7B96F,0x71F594,0xEE8A83,0x68C678,0x645F8E,0xE21375,
    0x15723B,0x933EC0,0x9FA736,0x19EBCD,0x8694DA,0x00D821,0x0C41D7,0x8A0D2C,
    0xB4F302,0x32BFF9,0x3E260F,0xB86AF4,0x2715E3,0xA15918,0xADC0EE,0x2B8C15,
    0xD03CB2,0x567049,0x5AE9BF,0xDCA544,0x43DA53,0xC596A8,0xC90F5E,0x4F43A5,
    0x71BD8B,0xF7F170,0xFB6886,0x7D247D,0xE25B6A,0x641791,0x688E67,0xEEC29C,
    0x3347A4,0xB50B5F,0xB992A9,0x3FDE52,0xA0A145,0x26EDBE,0x2A7448,0xAC38B3,
    0x92C69D,0x148A66,0x181390,0x9E5F6B,0x01207C,0x876C87,0x8BF571,0x0DB98A,
    0xF6092D,0x7045D6,0x7CDC20,0xFA90DB,0x65EFCC,0xE3A337,0xEF3AC1,0x69763A,
    0x578814,0xD1C4EF,0xDD5D19,0x5B11E2,0xC46EF5,0x42220E,0x4EBBF8,0xC8F703,
    0x3F964D,0xB9DAB6,0xB54340,0x330FBB,0xAC70AC,0x2A3C57,0x26A5A1,0xA0E95A,
    0x9E1774,0x185B8F,0x14C279,0x928E82,0x0DF195,0x8BBD6E,0x872498,0x016863,
    0xFAD8C4,0x7C943F,0x700DC9,0xF64132,0x693E25,0xEF72DE,0xE3EB28,0x65A7D3,
    0x5B59FD,0xDD1506,0xD18CF0,0x57C00B,0xC8BF1C,0x4EF3E7,0x426A11,0xC426EA,
    0x2AE476,0xACA88D,0xA0317B,0x267D80,0xB90297,0x3F4E6C,0x33D79A,0xB59B61,
    0x8B654F,0x0D29B4,0x01B042,0x87FCB9,0x1883AE,0x9ECF55,0x9256A3,0x141A58,
    0xEFAAFF,0x69E604,0x657FF2,0xE33309,0x7C4C1E,0xFA00E5,0xF69913,0x70D5E8,
    0x4E2BC6,0xC8673D,0xC4FECB,0x42B230,0xDDCD27,0x5B81DC,0x57182A,0xD154D1,
    0x26359F,0xA07964,0xACE092,0x2AAC69,0xB5D37E,0x339F85,0x3F0673,0xB94A88,
    0x87B4A6,0x01F85D,0x0D61AB,0x8B2D50,0x145247,0x921EBC,0x9E874A,0x18CBB1,
    0xE37B16,0x6537ED,0x69AE1B,0xEFE2E0,0x709DF7,0xF6D10C,0xFA48FA,0x7C0401,
    0x42FA2F,0xC4B6D4,0xC82F22,0x4E63D9,0xD11CCE,0x575035,0x5BC9C3,0xDD8538
};


/* crc-24q parity --------------------------------------------------------------
* compute crc-24q parity for sbas, rtcm3
* args   : unsigned char *buff I data
*          int    len    I      data length (bytes)
* return : crc-24Q parity
* notes  : see reference [2] A.4.3.3 Parity
*-----------------------------------------------------------------------------*/
unsigned int crc24q( unsigned char *buff, int len)
{
    unsigned int crc=0;
    int i;
  
    for (i=0;i<len;i++)
		crc=((crc<<8)&0xFFFFFF)^tbl_CRC24Q[(crc>>16)^buff[i]];

    return crc;
}


/* *
*@brief		RTCM3 Input Process
*@param  	p_parse_cnt, 	output, the data length processed
*@return		parse state
*/
INT32 RTCMRcvProc(INT32 *pValidDataLen,STRU_UART_STREAM *glStruUart)
{
	word32 	tmpReadPos = 0;
	word16	msgid=0,tmpdata=0;
	UINT16 	payloadlen = 0;
	UINT32 crcres1=0,crcres2=0;
	RTCM3_PACKAGE_INFO 	cur_pkg_info;

	if((*pValidDataLen) < RTCM3_MIN_PKG_LEN)
		return PROTO_WAITE_DATA;

	tmpReadPos = glStruUart->RecvData.CmdCurPos;
	ReadBufferShiftProcess((unsigned short*)&tmpReadPos,1);//检查paylen低byte是否溢出
	payloadlen = (glStruUart->RecvData.Buf[tmpReadPos]&0x3) ;
	ReadBufferShiftProcess((unsigned short*)&tmpReadPos,1);//检查paylen高byte是否溢出
	payloadlen = ((payloadlen&0xFF)<< 8) | (unsigned char)glStruUart->RecvData.Buf[tmpReadPos];

	if(payloadlen > RTCM3_PAYLOAD_MAX_LEN)	//check payload lenght valid or not
	{
		(*pValidDataLen) -= 1;
		return PROTO_PARSE_FAIL;
	}

	ReadBufferShiftProcess((unsigned short*)&tmpReadPos,1);
	msgid=glStruUart->RecvData.Buf[tmpReadPos];
	ReadBufferShiftProcess((unsigned short*)&tmpReadPos,1);
	tmpdata=glStruUart->RecvData.Buf[tmpReadPos];

	msgid = ((msgid&0xff)<<4)|((tmpdata>>4)&0xf);

	//check message id
	if(FALSE==checkRTCM3Msgid(msgid,payloadlen))
	{
		(*pValidDataLen) -= 1;
		return PROTO_PARSE_FAIL;
	}

	if((*pValidDataLen) < payloadlen + RTCM3_MIN_PKG_LEN)	//need to wait data
		return PROTO_WAITE_DATA;

	ReadBufferShiftProcess((unsigned short*)&(tmpReadPos),payloadlen+2);

	if(tmpReadPos > glStruUart->RecvData.CmdCurPos)
		memcpy(glStruUart->CmdBuf, glStruUart->RecvData.Buf + glStruUart->RecvData.CmdCurPos, payloadlen + RTCM3_MIN_PKG_LEN);
	else if(tmpReadPos < glStruUart->RecvData.CmdCurPos)
	{
		memcpy(glStruUart->CmdBuf,	glStruUart->RecvData.Buf + glStruUart->RecvData.CmdCurPos, UART_RECV_BUF_LENTH - glStruUart->RecvData.CmdCurPos);
		memcpy(glStruUart->CmdBuf + (UART_RECV_BUF_LENTH - glStruUart->RecvData.CmdCurPos),  glStruUart->RecvData.Buf, tmpReadPos); 
	}

	crcres1 = crc24q((byte*)glStruUart->CmdBuf, payloadlen+RTCM3_HEAD_LEN);
	crcres2 = getbitu((byte*)glStruUart->CmdBuf,(payloadlen+RTCM3_HEAD_LEN)*8,24);

	if(crcres1==crcres2)	//parse success
	{
		cur_pkg_info.data_len = payloadlen+RTCM3_MIN_PKG_LEN;
		cur_pkg_info.msg_id = getbitu((byte*)(glStruUart->CmdBuf),24,12);
		cur_pkg_info.p_data = (byte*)(glStruUart->CmdBuf);	//???bug  ??buffer???
		RTCM3_dispatch_package(&cur_pkg_info,glStruUart);

		(*pValidDataLen) -= (payloadlen+RTCM3_MIN_PKG_LEN);	//?????
		return PROTO_PARSE_SUCCESS;
	}
	else	//parse failed
	{
		(*pValidDataLen) -= 1;
		return PROTO_PARSE_FAIL;
	}
}

bool checkRTCM3Msgid(word16 msgid,word16 payloadlen)
{
	int32 i = 0;
	if((msgid<1001)||(msgid>1127))
		return FALSE;

	i=0;
	while(RTCM3_MSG_TBL[i].msg_id != RTCM3_MSG_ID_INVALID)
	{
		if(msgid == RTCM3_MSG_TBL[i].msg_id)
		{
			if(payloadlen > RTCM3_MSG_TBL[i].msg_len)
				return FALSE;
			else
				return TRUE;
		}	
		i++;
	}

	return FALSE;		
}


/**
 * @brief  	Output the RTCM3 messages according to the rate table.
 * @return	N/A
 */
void PeriodSendRtcmPro(void)
{
	RTCM3_msg_entry* pEntry=NULL;
	STRU_UART_STREAM* pUartBuff;
	word16 pkg_len=0;
	UINT8 *rtcm_output_cycle=&(pActiveCPT->SysmCptProtocol[0].Rtcm3_1001);
	UINT8 comid = 0;

	for(comid = 0;comid < MAX_UART_NUM; comid++)
	{	
		if((comid == COM_ID_RTK) && (!IsBaseSation()))
			continue;
	
		pEntry = &(RTCM3_MSG_TBL[0]);
		while(pEntry->msg_id != RTCM3_MSG_ID_INVALID)
		{
			if((bFixAtSecBoundary == TRUE) && (IsSecBoundaryChange()))
				pEntry->sendcnt[comid] = 0;
			else
				pEntry->sendcnt[comid]++;
			
			pEntry++;
		}
		
		if((pActiveCPT->SysmCptProtocol[comid].COMInterFaceMode & CHECK_DATA_HEAD_RTCM3) == 0)
			continue;
		else
			pUartBuff = &(glStruUart[comid]);
		
		pEntry = &(RTCM3_MSG_TBL[0]);
		
		ResetAllMSMParam();
		UpdateMSMDataInfo(&Master_observ);
		UpdateTotalMSMCnt(comid);			
 			
		rtcm_output_cycle = &(pActiveCPT->SysmCptProtocol[comid].Rtcm3_1001);		
		while(pEntry->msg_id != RTCM3_MSG_ID_INVALID)
		{
			if((*rtcm_output_cycle != 0)&&(pEntry->sendcnt[comid] % (*rtcm_output_cycle) == 0) )
			{	
				if(pEntry->msg_tx_ptr != NULL)
				{
					memset(RTCM3OutputDataBuf, 0, sizeof(RTCM3OutputDataBuf));		//???

					(pEntry->msg_tx_ptr)(pEntry->msg_id, RTCM3OutputDataBuf, &pkg_len, pUartBuff);
				}
			}
			pEntry++;
			rtcm_output_cycle++;	
		}
	}
	return;
}


/**
 * @brief  	Create the head and CRC to dataflow, then sending to IO
 * @param	msg_id		id of message
 * @param	p_data    		pointer to raw data, include the header, payload and parity bits.
 * @param	pkg_len	  	length of the raw data, include the header, payload and parity bits.
 * @return	state
 */
bool RTCM3SendPackage(word16 msg_id, word16 pkg_len, byte *p_data, STRU_UART_STREAM *glStruUart)
{
	int32 crcres=0;
	byte * ptr = p_data;
	word16 payloadlen = pkg_len -RTCM3_MIN_PKG_LEN;
	byte head_reserve=HEAD2_RTCM;
	
	if(pkg_len < RTCM3_MIN_PKG_LEN)
		return FALSE;

	/* preamble 8bit*/
	ptr[0] = HEAD1_RTCM;

	/*reserve 6bit + payloadlen 10bit*/
	ptr[1] = ((payloadlen>>8) & 0x3) | ((head_reserve & 0x3f)<<2);
	ptr[2] = payloadlen & 0xff;

	/*message ID 12bit*/
	if(payloadlen >= 2)
	{
		ptr[3] = (msg_id>>4) & 0xff;
		ptr[4] = (ptr[4] & 0xf) | ((msg_id & 0xf)<<4);
	}

	/*CRC bits: 24bit*/
	crcres = crc24q(ptr, payloadlen+RTCM3_HEAD_LEN);

	ptr[payloadlen+RTCM3_HEAD_LEN] = (crcres>>16)&0xff;
	ptr[payloadlen+RTCM3_HEAD_LEN+1] = (crcres>>8)&0xff;;
	ptr[payloadlen+RTCM3_HEAD_LEN+2] = crcres&0xff;;

#ifndef _POSTPROC
	/*send to IO*/
	UART_Send_Buff(glStruUart->comid, (char*)ptr, payloadlen+RTCM3_MIN_PKG_LEN);
#endif
	return TRUE;
}


/* *
* @brief		RTCM3 Messages dispatch to handle
* @param 	p_pkg_info, 	input, the message info including message id, payload length, payload.
*/
static void RTCM3_dispatch_package(RTCM3_PACKAGE_INFO *p_pkg_info,STRU_UART_STREAM *p_glStruUart)
{
	RTCM3_msg_entry* pMsgEntry;
	if(FindRTCM3MsgEntry(p_pkg_info->msg_id, &pMsgEntry))
	{
		if(pMsgEntry->msg_rx_ptr != NULL)
		{
			(pMsgEntry->msg_rx_ptr)(pMsgEntry->msg_id, p_pkg_info->p_data, p_pkg_info->data_len,p_glStruUart);
		}
	}
	
	return;
}


/**
 * @brief  	Parse fix length RTCM3 messages
 * @param	msg_id	message ID
 * @param	pData	point to the head of the message.
 * @param	datalen	message length
 * @param	pItems	the struct the parsed message to save. 
 * @return	N/A
 */
void ParseRTCM3FixLenItems(word16 msg_id, byte* pData, word16 datalen, void* pItems)	
{
	byte* p_payload = pData + RTCM3_HEAD_LEN;
	word16 payloadlen = datalen - RTCM3_MIN_PKG_LEN;
	
	BITITEM_FIELD_DESC *pFieldDesc = NULL;

	if(FindRTCM3FixLenMsgDesc(msg_id, &pFieldDesc))	
	{	
		word16 bitoffset = 0;
		ParseBitItemRawData2Items(pFieldDesc, p_payload, payloadlen, &bitoffset, pItems);
	}

	return;
}

/**
 * @brief  	Parse variable length RTCM3 messages
 * @param	msg_id		message ID
 * @param	pData		point to the head of the message.
 * @param	datalen		message length
 * @param	pHeadItems	point to the head struct the parsed message to save. 
 * @param	pDataItems	point to the data struct the parsed message to save. 
 * @param	pRepeatCnt	point to the data field repeat times. 
 * @return	N/A
 */
void ParseRTCM3VarLenItems(word16 msg_id, byte* pData, word16 datalen, void* pHeadItems, void* pDataItems, word16* pRepeatCnt)
{
	BITITEM_FIELD_DESC *pHeadFieldDesc = NULL;
	BITITEM_FIELD_DESC *pDataFieldDesc = NULL;
	byte* p_payload;
	word16 repeatStructLen = 0, payloadlen, bitoffest = 0, repeatcnt=0;

	if(!FindRTCM3VarLenMsgDesc(msg_id, &pHeadFieldDesc, &pDataFieldDesc, &repeatStructLen))
		return;

	p_payload = pData + RTCM3_HEAD_LEN;
	payloadlen = datalen - RTCM3_MIN_PKG_LEN;

	if(!ParseBitItemRawData2Items(pHeadFieldDesc, p_payload, payloadlen, &bitoffest, pHeadItems))
		return;

	while(bitoffest < (payloadlen*8))
	{
		void* curstructAdr = (void*) ((byte*)pDataItems + repeatStructLen * repeatcnt);
		if(ParseBitItemRawData2Items(pDataFieldDesc, p_payload, payloadlen, &bitoffest, curstructAdr))
			repeatcnt++;
		else
			break;
	}

	*pRepeatCnt = repeatcnt;

	return;
}


/**
 * @brief  	Parse variable length RTCM3 messages
 * @param	msm_id		MSM ID
 * @param	navsys		navigation system. GPS or BD
 * @param	pData		point to the head of the message.
 * @param	datalen		message length
 * @param	pItemsHead	point to the head struct the parsed head to save. 
 * @param	pMap		point to the map struct(SatMask, SigMask, CellMask) the parsed map to save. 
 * @param	pMSMDataInfo 	point to the satellite data and signal data the parsed data to save.
 * @return	N/A
 */
void ParseRTCM3MSMItems(byte msm_id, byte navsys, byte* pData, word16 datalen, RTCM3_MSM_HEAD_INFO* pItemsHead, RTCM3_MSM_MAP* pMap, RTCM3_MSM_DATA_INFO* pMSMDataInfo)
{
	byte* p_payload = pData + RTCM3_HEAD_LEN;
	word16 payloadlen = datalen - RTCM3_MIN_PKG_LEN, bitoffest = 0;
	char* info_pos =NULL;
	int32 i=0, svid, j=0,sigidx=0, cellidx, satidx, frqidx;
	byte CellLen;
	byte tmp[8]={0,};
	word32 frepoint;

	BITITEM_FIELD_DESC* pCurDesc = NULL;
	BITITEM_FIELD_DESC* pHeadDesc=NULL;
	BITITEM_FIELD_DESC* pMapDesc=NULL;
	BITITEM_FIELD_DESC* pSatDataDesc=NULL;
	BITITEM_FIELD_DESC* pSigDataDesc=NULL;

	if(!FindRTCM3MSMMsgDesc(msm_id, &pHeadDesc, &pMapDesc, &pSatDataDesc, &pSigDataDesc))
		return;

	//parse head	
	if(!ParseBitItemRawData2Items(pHeadDesc, p_payload, payloadlen, &bitoffest, pItemsHead))
		return;

	//parse satellite mask
	memset(pMap , 0, sizeof(RTCM3_MSM_MAP));
	
	info_pos = (char*)(pMap) + pMapDesc[0].struct_fieldOffset;
	ParseBitItemOneItemFromRawData(&(pMapDesc[0]), p_payload, payloadlen, bitoffest, info_pos);
	bitoffest += pMapDesc[0].DF_Len;

	//parse signal mask
	info_pos = (char*)(pMap) + pMapDesc[1].struct_fieldOffset;
	ParseBitItemOneItemFromRawData(&(pMapDesc[1]), p_payload, payloadlen, bitoffest, info_pos);
	bitoffest += pMapDesc[1].DF_Len;

	//calc Cell Length	
	for(i=0; i<64; i++)
	{
		if(GetMSMMaskBit(pMap->SatMask, i))
			pMap->NSat++;
	}

	for(i=0; i<32; i++)
	{
		if(GetMSMMaskBit(pMap->SigMask, i))
			pMap->NSig++;
	}

	//parse cell mask
	CellLen = pMap->NSat * pMap->NSig;
	pMapDesc[2].DF_Len = CellLen;
	info_pos = (char*)pMap + pMapDesc[2].struct_fieldOffset;
	ParseBitItemOneItemFromRawData(&(pMapDesc[2]), p_payload, payloadlen, bitoffest, info_pos);
	bitoffest += pMapDesc[2].DF_Len;
	
	//arrange to left
	createBitsToDataFlow(tmp, pMap->CellMask, 0, CellLen);
	memcpy(pMap->CellMask, tmp, sizeof(tmp));

	//parse satellite data
	pCurDesc = pSatDataDesc;
	while(pCurDesc->DF_idx != BITITEM_DF_ID_INVALID)
	{
		satidx=0;
		for(i=0; i<64; i++)
		{
			if(GetMSMMaskBit(pMap->SatMask, i)==0)
				continue;

			info_pos = (char*)&(pMSMDataInfo->SatData[satidx]) + pCurDesc->struct_fieldOffset;
			ParseBitItemOneItemFromRawData(pCurDesc, p_payload, payloadlen, bitoffest, info_pos);
			
			pMSMDataInfo->SatData[satidx].datamap |= (0x1<<pCurDesc->validmapidx);
			svid =GetC1610SVIDFromMSMSatMap(i, navsys);
			SetMSMDataSVIdx(pMSMDataInfo, svid , satidx);

			bitoffest += pCurDesc->DF_Len;

			satidx++;
			if(satidx>=pMap->NSat)
				break;
		}

		pCurDesc++;
	}

	//parse signal data
	pCurDesc = pSigDataDesc;
	while(pCurDesc->DF_idx != BITITEM_DF_ID_INVALID)
	{
		satidx=0;
		for(i=0; i<64; i++)
		{
			if(GetMSMMaskBit(pMap->SatMask, i)==0)
				continue;

			satidx++;	
			if(satidx>pMap->NSat)
				break;

			j=0, sigidx = 0;
			for(j=0; j<32; j++)
			{
				if(GetMSMMaskBit(pMap->SigMask, j)==0)
					continue;

				sigidx++;
				if(sigidx>pMap->NSig)
					break;

				cellidx = (satidx-1) * pMap->NSig + (sigidx-1);
				if(GetMSMMaskBit(pMap->CellMask, cellidx)==0)
					continue;

				frepoint = ConvertRTCM3SigMaskidx2FP(j,navsys);
				frqidx = GetFreGropIdx(frepoint);
				if(frqidx>=0 && frqidx<MAX_FREPIONT_PER_NAVSYS)
				{
					info_pos = (char*)&(pMSMDataInfo->SigData[satidx-1][frqidx]) + pCurDesc->struct_fieldOffset;
					ParseBitItemOneItemFromRawData(pCurDesc, p_payload, payloadlen, bitoffest, info_pos);
					(pMSMDataInfo->SigData[satidx-1][frqidx]).datamap |= (0x1<<pCurDesc->validmapidx);
				}
				bitoffest += pCurDesc->DF_Len;
			}
		}
	
		pCurDesc++;
	}
	
	return;
}



/**
 * @brief  	Create fix length RTCM3 messages
 * @param	msg_id		message ID
 * @param	pItems		point to the struct used to create message.
 * @param	pData		point to the buffer that save the message raw data.
 * @return	the created raw data length (byte)
 */
word16 CreateRTCM3FixLenMSGRawData(word16 msg_id, void* pItems, byte* pData, STRU_UART_STREAM *glStruUart)
{
	BITITEM_FIELD_DESC* pFieldDesc=NULL;
	word16 bitoffset, payloadlen;

	if(!FindRTCM3FixLenMsgDesc(msg_id, &pFieldDesc))
		return 0;

	bitoffset = CreateBitItemRawDataFromItems(pFieldDesc, pItems, pData+RTCM3_HEAD_LEN, 0);
	payloadlen = (bitoffset-1)/8 + 1;

	//add head and CRC, send
	RTCM3SendPackage(msg_id, payloadlen + RTCM3_MIN_PKG_LEN, pData, glStruUart);	

	return (payloadlen + RTCM3_MIN_PKG_LEN);
}


/**
 * @brief  	Create variable length RTCM3 messages
 * @param	msg_id		message ID
 * @param	pItemsHead	point to the head struct used to create message.
 * @param	pItemsData	point to the data struct used to create message.
 * @param	repeatCnt	data field repeat times.
 * @param	pData		point to the buffer that save the message raw data.
 * @return	the created raw data length (byte)
 */
word16 CreateRTCM3VarLenMSGRawData(word16 msg_id, void* pItemsHead, void* pItemsData, byte repeatCnt, byte* pData, STRU_UART_STREAM *glStruUart)
{
	BITITEM_FIELD_DESC* pHeadDesc=NULL;
	BITITEM_FIELD_DESC* pDataDesc=NULL;
	word16 pRepeatStruLen=0, bitoffset, curbitoffset, payloadlen;
	int32 i=0;
	
	if(!FindRTCM3VarLenMsgDesc(msg_id, &pHeadDesc, &pDataDesc, &pRepeatStruLen))
		return 0;

	bitoffset = CreateBitItemRawDataFromItems(pHeadDesc, pItemsHead, pData+RTCM3_HEAD_LEN, 0);

	
	for(i=0; i<repeatCnt; i++)
	{
		void* curItemsData = (void*)(((byte*)pItemsData) + pRepeatStruLen*i);
		curbitoffset = CreateBitItemRawDataFromItems(pDataDesc, curItemsData, pData+RTCM3_HEAD_LEN, bitoffset);
		bitoffset = curbitoffset;
	}

	payloadlen = (bitoffset-1)/8 + 1;

	//add head and CRC, send
	RTCM3SendPackage(msg_id, payloadlen + RTCM3_MIN_PKG_LEN, pData, glStruUart);	

	return (payloadlen + RTCM3_MIN_PKG_LEN);
}


	//MSM
/**
 * @brief  	Create RTCM3 MSM messages
 * @param	msm_id		MSM ID
 * @param	navsys		navigation system, gps or bd
 * @param	pItemsHead	point to the head struct used to create message.
 * @param	pMap		point to the map(SatMask, SigMask, CellMask) struct used to create message.
 * @param	pMSMDataInfo 	point to the data struct, this struct include all the MSM items.
 * @param	pData		point to the buffer that save the message raw data.
 * @return	the created raw data length (byte)
 */ 
word16 CreateRTCM3MSMMSGRawData(byte msm_id, byte navsys, RTCM3_MSM_HEAD_INFO* pItemsHead, RTCM3_MSM_MAP* pMap, RTCM3_MSM_DATA_INFO* pMSMDataInfo, byte* pData, STRU_UART_STREAM *glStruUart)
{
	BITITEM_FIELD_DESC* pHeadDesc=NULL;
	BITITEM_FIELD_DESC* pMapDesc=NULL;
	BITITEM_FIELD_DESC* pSatDataDesc=NULL;
	BITITEM_FIELD_DESC* pSigDataDesc=NULL;
	byte* pPayloadData = pData+RTCM3_HEAD_LEN;
	RTCM3_MSM_MAP tmpMap;								//to arrange CellMask to right
	BITITEM_FIELD_DESC* pCurDesc = NULL;
	int32 i=0,j=0, satidx=0, svid,idx,sigidx,cellidx,CellLen, frqidx;
	char* info_pos = NULL;
	word16 bitoffset,curbitoffset,payloadlen, msgid;
	word32 frqpoint;

	if(!FindRTCM3MSMMsgDesc(msm_id, &pHeadDesc, &pMapDesc, &pSatDataDesc, &pSigDataDesc))
		return 0;

	//create head
	bitoffset = CreateBitItemRawDataFromItems(pHeadDesc, (void*)pItemsHead, pPayloadData, 0);

	//create map
	CellLen = pMap->NSat * pMap->NSig;
	pMapDesc[2].DF_Len = CellLen;		//modify the CellMask length

	memcpy(&tmpMap, pMap, sizeof(RTCM3_MSM_MAP));
	memset(tmpMap.CellMask, 0, sizeof(tmpMap.CellMask));
	getBitsFromDataFlow(tmpMap.CellMask, pMap->CellMask, 0, CellLen);

	curbitoffset = CreateBitItemRawDataFromItems(pMapDesc, (void*)&tmpMap, pPayloadData, bitoffset);
	bitoffset = curbitoffset;
	
	//create Satellite data
	pCurDesc = pSatDataDesc;
	while(pCurDesc->DF_idx != BITITEM_DF_ID_INVALID)
	{
		i=0, satidx=0;
		for(i=0; i<64; i++)
		{
			if(GetMSMMaskBit(pMap->SatMask, i)==0)
				continue;

			svid =GetC1610SVIDFromMSMSatMap(i, navsys);
			idx = FindMSMDataSVIdx(pMSMDataInfo, svid);

			info_pos = (char*)&(pMSMDataInfo->SatData[idx]) + pCurDesc->struct_fieldOffset;	
			bitoffset = CreateBitItemRawDataFromOneItem(pCurDesc, info_pos, pPayloadData, bitoffset);

			satidx++;

			if(satidx>=pMap->NSat)
				break;
		}
	
		pCurDesc++;
	}

	//create Signal data
	pCurDesc = pSigDataDesc;
	while(pCurDesc->DF_idx != BITITEM_DF_ID_INVALID)
	{
		i=0, satidx=0;
		for(i=0; i<64; i++)
		{
			if(GetMSMMaskBit(pMap->SatMask, i)==0)
				continue;

			satidx++;	
			if(satidx>pMap->NSat)
				break;

			svid =GetC1610SVIDFromMSMSatMap(i, navsys);
			idx = FindMSMDataSVIdx(pMSMDataInfo, svid);

			sigidx=0, j=0, cellidx=0;	
			for(j=0; j<32; j++)
			{
				if(GetMSMMaskBit(pMap->SigMask, j)==0)
					continue;

				sigidx++;
				if(sigidx>pMap->NSig)
					break;

				cellidx = (satidx-1) * pMap->NSig + (sigidx-1);
				
				if(GetMSMMaskBit(pMap->CellMask, cellidx)==0)
					continue;

				frqpoint = ConvertRTCM3SigMaskidx2FP(j,navsys);
				frqidx = GetFreGropIdx(frqpoint);
				info_pos = (char*)&(pMSMDataInfo->SigData[idx][frqidx]) + pCurDesc->struct_fieldOffset;
				bitoffset = CreateBitItemRawDataFromOneItem(pCurDesc, info_pos, pPayloadData, bitoffset);	
			}	
		}
		pCurDesc++;
	}

	payloadlen = (bitoffset-1)/8 + 1;

	//add head and CRC, send
	msgid = FindMSMMsgID(msm_id, navsys);
	RTCM3SendPackage(msgid, payloadlen + RTCM3_MIN_PKG_LEN, pData, glStruUart);	

	return (payloadlen + RTCM3_MIN_PKG_LEN);
}

boolean IsMSMOpen(void)
{
	boolean ret = FALSE;
#if 0
	int32 i=0;
	UINT8 *rtcm_output_cycle;
	while(RTCM3_MSG_TBL[i].msg_id!= RTCM3_MSG_ID_INVALID)
	{
		if(((RTCM3_MSG_TBL[i].msg_id >= 1071) && (RTCM3_MSG_TBL[i].msg_id <= 1077))
			||((RTCM3_MSG_TBL[i].msg_id >= 1081) && (RTCM3_MSG_TBL[i].msg_id <= 1087))
			||((RTCM3_MSG_TBL[i].msg_id >= 1121) && (RTCM3_MSG_TBL[i].msg_id <= 1127)))
		{
			if((*rtcm_output_cycle) != 0)
			{
				ret = TRUE;
				break;
			}
		}
		
		i++;
	}
#endif
	return ret;
}

byte GetMSMMaskBit(byte* pMask, byte idx)
{

	int32 byteid = idx/8;
	int32 bitid = 7-(idx%8);

	return ((pMask[byteid] >> bitid) & 0x1);
}

void SetMSMMaskBit(byte* pMask, byte idx)
{

	int32 byteid = idx/8;
	int32 bitid = 7-(idx%8);

	pMask[byteid] |= (0x1<<bitid);

	return;
}

int32 FindMSMDataSVIdx(RTCM3_MSM_DATA_INFO* pMSMDataInfo, int32 svid)
{
	return (pMSMDataInfo->svidx[svid-1]);
}

#if RTCM3_RX_ENABLE
void SetMSMDataSVIdx(RTCM3_MSM_DATA_INFO* pMSMDataInfo, int32 svid, int8 idx)
{	
	pMSMDataInfo->svidx[svid-1] = idx;

	return;
}
#endif

int32 GetC1610SVIDFromMSMSatMap(int32 mapidx, byte navsys)
{
	int32 svid = -1;
	if(navsys ==  NAV_SYS_GPS)
	{
		if(mapidx<=MaxGpsSvID)
			svid = mapidx+MinGpsSvID;
	}
	else if(navsys ==  NAV_SYS_BD)
	{
		if(mapidx<(MaxBD2SvID-MinBD2SvID+1))
			svid = mapidx+MinBD2SvID;
	}
#if SUPPORT_GLONASS
	else if(navsys ==  NAV_SYS_GLO)
	{
		if(mapidx<(MaxGloSvIDTRUE-MinGloFreID+1))
			svid = mapidx+MinGloFreID;
	}
#endif

	return svid;
}




void ParseGPSEphInfo2EphBits(GPSBD2EphFrameStruct* p_sveph ,EphemerisBits* p_svephbits)
{
	int32 week = p_sveph->wkn;
	//pasre the eph info from the eph items
	if(SV_IsBd2(p_svephbits->svid))
	{
		p_svephbits->svid = p_svephbits->svid - MinBD2SvID +1;
		week -= GPS_BD_WN_OFFSET;
	}
		
	p_svephbits->week=(week)%1024;

	p_svephbits->toe  =ROUND(p_sveph->toe/16.0);
	p_svephbits->toc  =ROUND(p_sveph->toc/16.0);
	p_svephbits->sqra =ROUND(p_sveph->sqrta/TWO_N19);
	p_svephbits->ecc  =ROUND(p_sveph->ecc/TWO_N33);
	p_svephbits->inc0 =ROUND(p_sveph->i0  /TWO_N31/PI);
	p_svephbits->omega0 =ROUND(p_sveph->omega0/TWO_N31/PI);
	p_svephbits->w  =ROUND(p_sveph->w /TWO_N31/PI); 
	p_svephbits->M0   =ROUND(p_sveph->m0  /TWO_N31/PI);
	p_svephbits->delta_n =ROUND(p_sveph->deltan/TWO_N43/PI);
	p_svephbits->i_dot =ROUND(p_sveph->idot/TWO_N43/PI);
	p_svephbits->omega_dot =ROUND(p_sveph->omegadot/TWO_N43/PI);
	p_svephbits->Crs  =ROUND(p_sveph->crs/TWO_N5 );
	p_svephbits->Crc  =ROUND(p_sveph->crc/TWO_N5 );
	p_svephbits->Cus  =ROUND(p_sveph->cus/TWO_N29);
	p_svephbits->Cuc  =ROUND(p_sveph->cuc/TWO_N29);
	p_svephbits->Cis  =ROUND(p_sveph->cis/TWO_N29);
	p_svephbits->Cic  =ROUND(p_sveph->cic/TWO_N29);
	p_svephbits->af0  =ROUND(p_sveph->af0 /TWO_N31);
	p_svephbits->af1  =ROUND(p_sveph->af1 /TWO_N43);
	p_svephbits->af2  =ROUND(p_sveph->af2 /TWO_N55);
	p_svephbits->Tgd  =ROUND(p_sveph->tgd/TWO_N31);
	p_svephbits->iodc = (word32)(p_sveph->iodc & 0x3ff);
	p_svephbits->iode = (word32)((p_sveph->iode) & 0xff);
	p_svephbits->uraidx = (word32)(p_sveph->ura & 0xf);
	p_svephbits->health = (word32)(p_sveph->s1hlth & 0x3f);

	return;
}


word32 IrregularInt2Word32(byte signbitidx,int32 sd)
{
	word32 unsignwd = (word32)sd;
	if(sd & (0x1<<signbitidx))
	{
		int32 tempwd = 0xffffffff;
		unsignwd = (word32)(sd & (tempwd<<signbitidx));
	}
	return unsignwd;
}


void ParseGPSEphBits2EphInfo(EphemerisBits* p_svephbits,  GPSBD2EphFrameStruct* p_sveph)
{
	//pasre the eph info from the eph items
	
	p_sveph->iodc = (int16)(p_svephbits->iodc & 0x3ff);
	p_sveph->tgd = ((int8)(p_svephbits->Tgd & 0xff)) * TWO_N31;
	p_sveph->toc = (int32)((p_svephbits->toc & 0xffff) * TWO_P4 + 0.5);
	p_sveph->af2 = ((int8)(p_svephbits->af2 & 0xff)) * TWO_N55;
	p_sveph->af1 =  ((int16)(p_svephbits->af1 & 0xffff)) * TWO_N43;
	p_sveph->af0 = IrregularWord2Int32(21, p_svephbits->af0 & 0x3fffff) * TWO_N31;
	p_sveph->iode = (int16)(p_svephbits->iode & 0xff);
	p_sveph->crs = ((int16)(p_svephbits->Crs & 0xffff)) * TWO_N5;
	p_sveph->deltan = ((int16)(p_svephbits->delta_n & 0xffff)) * TWO_N43 * PI;
	p_sveph->m0 = ((int32)p_svephbits->M0) * TWO_N31 * PI;
	p_sveph->cuc = ((int16)(p_svephbits->Cuc & 0xffff)) * TWO_N29;
	p_sveph->ecc = p_svephbits->ecc * TWO_N33;
	p_sveph->cus =  ((int16)(p_svephbits->Cus & 0xffff)) * TWO_N29;
	p_sveph->sqrta = p_svephbits->sqra * TWO_N19;
	p_sveph->toe = (int32)((p_svephbits->toe & 0xffff) * TWO_P4+0.5);
	p_sveph->cic = ((int16)(p_svephbits->Cic & 0xffff)) * TWO_N29;
	p_sveph->cis =  ((int16)(p_svephbits->Cis & 0xffff)) * TWO_N29;
	p_sveph->i0 = ((int32)p_svephbits->inc0) * TWO_N31 * PI;
	p_sveph->omega0 = ((int32)p_svephbits->omega0) * TWO_N31 * PI;
	p_sveph->crc = ((int16)(p_svephbits->Crc & 0xffff)) * TWO_N5;
	p_sveph->w = ((int32)p_svephbits->w) * TWO_N31 * PI;
	p_sveph->omegadot = IrregularWord2Int32(23, p_svephbits->omega_dot & 0xffffff) * TWO_N43 * PI;
	p_sveph->idot = IrregularWord2Int32(13, p_svephbits->i_dot & 0x3fff)* TWO_N43 * PI;
	p_sveph->wkn = (int16)(p_svephbits->week);
	//p_sveph->aodo = (int16)(p_svephbits->aodo * 900);
	p_sveph->ura = (byte)(p_svephbits->uraidx & 0xf);
	p_sveph->s1hlth = (byte)(p_svephbits->health & 0x3f);
	//p_sveph->fit_interval = (byte)(p_svephbits->fit_interval);

	return;
}


int32 IrregularWord2Int32(byte signbitidx,word32 wd)
{
	int32 signwd = (int32)wd;
	if(wd & (0x1<<signbitidx))
	{
		word32 tempwd = 0xffffffff;
		signwd = (int32)(wd | (tempwd<<signbitidx));
	}
		
	return signwd;
}



