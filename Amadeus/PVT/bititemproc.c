
/**
* @file  bititemproc.c
*
* @brief 
*		This module is used to create and parse bitflow binary protocol
*		
*		created by juan. 2013.Oct.16
*/

#include "define.h"
#include <math.h>
#include "bititemproc.h"
#include "constdef.h"
#include "coordinate.h"

// parse
static void InregularIntH32L32ToF64(float64 *ret, word32 high, word32 low, byte bitlen);
static void ConvertInregularSIntToInt( byte* pData, byte bitlen);

//create
static void InregularIntF64ToH32L32(float64 fvalue, word32* phigh, word32* plow, byte bitlen);
static void ConvertInregularIntToSInt(byte* pdata, byte bitlen);

static void ConvertIndian(byte* pDes, byte* pSrc, word16 bytecnt);


static byte ByteOrTable[9]={0x0, 0x1, 0x3, 0x7, 0xf, 0x1f, 0x3f, 0x7f, 0xff};


/**
 * @brief  	Pares a block of RTCM3 raw data.
 * @param	pFieldDesc	point to DF description.
 * @param	pData		point to the RTCM3 data flow.
 * @param	datalen		the data length of this block. 
 * @param	pBitoffset		point to the bit offset pData that has parsed.
 * @param	pItems		point to the struct saving the RTCM3 parsed information.
 * @return	parsed fialed or sucesse.
 */
boolean ParseBitItemRawData2Items(BITITEM_FIELD_DESC* pFieldDesc, byte* pData, word16 datalen, word16* pBitoffset, void* pItems)
{
	char *info_pos;
	BITITEM_FIELD_DESC* pCurFieldDesc = pFieldDesc;

	word16 firstbitoffset = (*pBitoffset);
	while(pCurFieldDesc->DF_idx != BITITEM_DF_ID_INVALID)
	{
		info_pos = (char*)pItems + pCurFieldDesc->struct_fieldOffset;
		(*pBitoffset) = firstbitoffset + pCurFieldDesc->DF_Offset;

		if(!ParseBitItemOneItemFromRawData(pCurFieldDesc, pData, datalen, *pBitoffset, info_pos))
			return FALSE;
		
		(*pBitoffset) += pCurFieldDesc->DF_Len;
				
		pCurFieldDesc++;
	}

	return TRUE;
}


boolean ParseBitItemOneItemFromRawData(BITITEM_FIELD_DESC* pFieldDesc, byte* pData, word16 datalen, word16 bitoffset, char* info_pos)
{
	byte tmpdata[BITITEM_MAX_ITEM_LEN];
	word16 structbytecnt,bytecnt;
	byte signbit,bitidx;
	byte litteIndianData[10];
	int32 i=0;

	memset(tmpdata, 0, sizeof(tmpdata));
	getBitsFromDataFlow(tmpdata, pData, bitoffset, pFieldDesc->DF_Len);

	//float64 scale = pCurFeildDesc->Scale;
	//boolean bNeed2ProScale = FALSE;
	//if(f_abs(scale - 1.0) > MICRO_NUM)
	//	bNeed2ProScale = TRUE;

	structbytecnt = pFieldDesc->struct_fieldSize;
	bytecnt = (pFieldDesc->DF_Len-1) / 8 + 1;

	if((structbytecnt < bytecnt) || (bitoffset + pFieldDesc->DF_Len > 8*datalen))	//table or struct define error
		return FALSE;
	
	switch(pFieldDesc->DF_Type)
	{
	case B_T:
	case UN_T:
		memcpy(info_pos, tmpdata, bytecnt);
		break;
	case S_T:		//max bitcnt 32
		ConvertInregularSIntToInt(tmpdata, pFieldDesc->DF_Len);
	case I_T:		//max bitcnt 38
	case U_T:		//max bitcnt 36
		{
			memset(info_pos, 0, structbytecnt);
			
			signbit = 0;
			if(pFieldDesc->DF_Type == S_T || pFieldDesc->DF_Type == I_T)
			{
				bitidx = (pFieldDesc->DF_Len-1)%8;
				signbit = (tmpdata[0]>>bitidx) & 0x1;

				if(signbit == 1)
				{
					tmpdata[0] |= ByteOrTable[8-bitidx]<<bitidx;

					
				//#if (!TARGET_FW)
					for(i=bytecnt; i<structbytecnt; i++)
						*((byte*)(info_pos+i)) = 0xff;
				//#else
				//	for(i=0; i<structbytecnt-bytecnt; i++)
				//		*((byte*)(info_pos+i)) = 0xff;
				//#endif
				}
			}			
//#if (!TARGET_FW)
			memset(litteIndianData,0,sizeof(litteIndianData));
			ConvertIndian(litteIndianData, tmpdata, bytecnt);
			memcpy(tmpdata, litteIndianData, bytecnt);
//#endif
			if(bytecnt<=4)
			{
			//#if (!TARGET_FW)
				memcpy(info_pos, tmpdata, bytecnt);
			//#else
			//	memcpy(info_pos+(structbytecnt-bytecnt), tmpdata, bytecnt);
			//#endif
			}
			else if((bytecnt>4) && (bytecnt<=8))
			{
				float64 itemvalue=0;
				word32 low=0, high=0;

//#if (!TARGET_FW)
				memcpy(&low, tmpdata, 4);
				memcpy(&high, tmpdata+4, bytecnt-4);	
//#else
//				memcpy(&low, tmpdata+(bytecnt-4), 4);
//				memcpy(((byte*)(&high))+8-bytecnt, tmpdata, bytecnt-4);
//#endif	
				if(pFieldDesc->DF_Type == U_T)
					H32L32ToF64(&itemvalue, high, low);
				else
					InregularIntH32L32ToF64(&itemvalue, high, low, pFieldDesc->DF_Len);

				memcpy(info_pos, &itemvalue, sizeof(itemvalue));
			}
		}
		break;

	default: break;			
	}

	return TRUE;
}


/**
 * @brief  	Get one Data Field from RTCM3 data flow. Saved in Big-India.
 * @param	pDes		point to the buffer saving the data field.
 * @param	pSrc			point to the RTCM3 data flow.
 * @param	bitoffset		the DF bitoffset in source data. 
 * @param	bitcnt		the DF bit cout.
 * @return	N/A
 */
void getBitsFromDataFlow(byte* pDes, byte* pSrc, word16 bitoffset, byte bitcnt)
{
	word16 bitcntProed = 0;

	byte* pTmpDes = pDes+(bitcnt-1)/8;
	byte* pTmpSrc = pSrc+(bitoffset + bitcnt - 1)/8;

	byte SrcMaxBitcnt = (bitoffset + bitcnt)%8;	//des max bitcnt can processed in current byte
	SrcMaxBitcnt = (SrcMaxBitcnt==0)?8:SrcMaxBitcnt;
	
	
	while(bitcntProed < bitcnt)
	{
		byte curDesBitcntPro = MIN(8, (bitcnt - bitcntProed));
		byte curSrcBitcntpro = MIN(SrcMaxBitcnt, curDesBitcntPro);

		(*pTmpDes) |= ((*pTmpSrc) >> (8-SrcMaxBitcnt)) & ByteOrTable[curSrcBitcntpro];
		pTmpSrc--;
		
		if(curDesBitcntPro > curSrcBitcntpro)
			(*pTmpDes) |= ((*pTmpSrc) & ByteOrTable[curDesBitcntPro-curSrcBitcntpro]) << SrcMaxBitcnt;

		pTmpDes--;
		bitcntProed += curDesBitcntPro;
	}

	return;
}


//in big Indian
void ConvertInregularSIntToInt(byte* pdata, byte bitlen)
{
	byte bytecnt = (bitlen-1)/8+1;
	byte signbitidx = (bitlen-1)%8;
	byte signbit = (pdata[0]>>signbitidx) & 0x1;
	if(signbit == 1)		//negative value
	{
		int32 i=0;
		for(i=bytecnt-1; i>0; i--)
			pdata[i] =~pdata[i];

		pdata[0] ^= ByteOrTable[signbitidx];

		for(i=bytecnt-1; i>=0; i--)
		{
			if(pdata[i] == 0xff)
			{
				pdata[i] = 0x0;
			}
			else
			{
				pdata[i] += 0x1;
				break;
			}
		}
	}
	
	return;
}

void InregularIntH32L32ToF64(float64 *ret, word32 high, word32 low, byte bitlen)
{
	byte signbitidx = (bitlen-1)%32;
	byte signbit = (high>>signbitidx) & 0x1;

	if(signbit == 0)
		H32L32ToF64(ret, high,low);
	else
	{
		word32 tmplow = low, tmphigh=high&((0x1<<signbitidx)-1);
		if(tmplow == 0)
		{
			tmphigh -= 1;
			tmplow = 0xffffffff;
		}
		else
			tmplow -= 1;
		
		tmplow ^= 0xffffffff;
		tmphigh ^= ((0x1<<signbitidx)-1);

		H32L32ToF64(ret, tmphigh,tmplow);
		
		(*ret) *= -1.0;
	}

	return;
}


void ConvertIndian(byte* pDes, byte* pSrc, word16 bytecnt)
{
	int32 i=0;

	for(i=0; i<bytecnt; i++)
	{
		pDes[bytecnt-1-i] = pSrc[i];
	}

	return;
}



/******************************functions for create bitflow by items***************************************/
/**
 * @brief  	Create a block of RTCM3 raw data.
 * @param	pFieldDesc	point to DF description.
 * @param	pItems		point to the struct saving the message information.
 * @param	pData		point to the RTCM3 data flow.
 * @param	firstbitoffset	the bit offset of this block in pData. 
 * @return	the parsed bit offset in pData.
 */
word16 CreateBitItemRawDataFromItems(BITITEM_FIELD_DESC* pFieldDesc, void* pItems, byte* pData, word16 firstbitoffset)
{
	char *info_pos;
	word16 bitoffset = firstbitoffset;

	BITITEM_FIELD_DESC* pCurFieldDesc = pFieldDesc;

	while( pCurFieldDesc->DF_idx != BITITEM_DF_ID_INVALID)
	{
		bitoffset = pCurFieldDesc->DF_Offset + firstbitoffset;
		info_pos = (char*)pItems + pCurFieldDesc->struct_fieldOffset;

		CreateBitItemRawDataFromOneItem(pCurFieldDesc, info_pos, pData, bitoffset);

		bitoffset  += pCurFieldDesc->DF_Len;
		pCurFieldDesc++;
	}

	return bitoffset;
}


word16 CreateBitItemRawDataFromOneItem(BITITEM_FIELD_DESC* pFieldDesc, char* pItem, byte* pData, word16 bitoffset)
{
	word16 structbytecnt = pFieldDesc->struct_fieldSize;
	word16 bytecnt = (pFieldDesc->DF_Len-1) / 8 + 1;
	byte bigIndiaData[8];
	byte tmpdata[BITITEM_MAX_ITEM_LEN];
	
	memset(tmpdata, 0, sizeof(tmpdata));

	if(structbytecnt < bytecnt)	//struct is define is error
		return 0;

	switch(pFieldDesc->DF_Type)
	{
	case B_T:
	case UN_T:
		memcpy(tmpdata, pItem, bytecnt);
		break;

	case U_T:
	case I_T:
	case S_T:
		{
			if(bytecnt <=4)
			{
				//#if (!TARGET_FW)
					memcpy(tmpdata, pItem, bytecnt);
				//#else
				//	memcpy(tmpdata, pItem+(structbytecnt-bytecnt), bytecnt);
				//#endif
			}
			else if((bytecnt>4) && (bytecnt<=8))
			{
				word32 low=0, high=0;

				if(pFieldDesc->DF_Type == U_T)
					F64ToH32L32(*(float64*)pItem, &high, &low);
				else
					InregularIntF64ToH32L32(*(float64*)pItem, &high, &low, pFieldDesc->DF_Len);

			//	#if (!TARGET_FW)	//little india
					memcpy(tmpdata, &low, 4);
					memcpy(tmpdata+4, &high, bytecnt-4);
			//	#else
			//		memcpy(tmpdata+bytecnt-4, &low, 4);
			//		memcpy(tmpdata, ((byte*)(&high))+8-bytecnt, bytecnt-4);
			//	#endif
			}

			//#if (!TARGET_FW)		//HMB, convert little-india to big-india
			memset(bigIndiaData, 0, sizeof(bigIndiaData));
			ConvertIndian(bigIndiaData,tmpdata,bytecnt);
			memcpy(tmpdata,bigIndiaData,bytecnt);
			//#endif

			if(pFieldDesc->DF_Type == S_T)
				ConvertInregularIntToSInt(tmpdata, pFieldDesc->DF_Len);
		}
		break;

		default: break;
	}

	createBitsToDataFlow((byte*)pData, tmpdata, bitoffset, pFieldDesc->DF_Len);

	bitoffset  += pFieldDesc->DF_Len;

	return bitoffset;
}

void InregularIntF64ToH32L32(float64 fvalue, word32* phigh, word32* plow, byte bitlen)
{
	byte signbitidx = (bitlen-1)%32;

	if(f_abs(fvalue) < MICRO_NUM)
	{
		*phigh = 0;
		*plow = 0;
	}
	else if(fvalue > MICRO_NUM)
	{
		F64ToH32L32(fvalue, phigh, plow);
	}
	else
	{
		float64 absvalue = -1.0* fvalue;
		F64ToH32L32(absvalue, phigh, plow);

		(*plow) ^= 0xffffffff;
		(*phigh) ^= ((0x1<<signbitidx)-1);

		if((*plow) == 0xffffffff)
		{
			(*plow) = 0x0;
			(*phigh) += 0x1;
		}
		else
			(*plow) += 0x1;

		(*phigh) |= (0x1<<signbitidx);
		
	}

	return;
}


//in big Indian
void ConvertInregularIntToSInt(byte* pdata, byte bitlen)
{
	byte bytecnt = (bitlen-1)/8+1;
	byte signbitidx = (bitlen-1)%8;
	byte signbit = (pdata[0]>>signbitidx) & 0x1;
	if(signbit == 1)		//negative value
	{
		int32 i=0;
		for(i=bytecnt-1; i>=0; i--)
		{
			if(pdata[i] == 0x00)
				pdata[i] = 0xff;
			else
			{
				pdata[i] -= 1;
				break;
			}
		}
		for(i=bytecnt-1; i>0; i--)
			pdata[i] =~pdata[i];

		pdata[0] ^= ByteOrTable[signbitidx];
	}

	return;
}

/**
 * @brief  	Create one Data Field to RTCM3 data flow. Saved in Big-India.
 * @param	pDes		point to RTCM3 data flow.
 * @param	pSrc			point to the data field.
 * @param	bitoffset		the DF bitoffset in distiniation data. 
 * @param	bitcnt		the DF bit cout.
 * @return	N/A
 */
void createBitsToDataFlow(byte* pDes, byte* pSrc, word16 bitoffset, byte bitcnt)
{
	word16 bitcntProed = 0;
	byte* pTmpSrc = pSrc+(bitcnt-1)/8;
	byte* pTmpDes = pDes+(bitoffset + bitcnt - 1)/8;

	byte DesMaxBitcnt = (bitoffset + bitcnt)%8;	//des max bitcnt can processed in current byte
	DesMaxBitcnt = (DesMaxBitcnt==0)?8:DesMaxBitcnt;
	
	
	while(bitcntProed < bitcnt)
	{
		byte curSrcBitcntPro = MIN(8, (bitcnt - bitcntProed));
		byte curDesBitcntpro = MIN(DesMaxBitcnt, curSrcBitcntPro);
		
		(*pTmpDes) |= (((*pTmpSrc) & ByteOrTable[curDesBitcntpro]) << (8-DesMaxBitcnt));		
		pTmpDes--;
		
		if(curSrcBitcntPro > curDesBitcntpro)	
			(*pTmpDes) |= (((*pTmpSrc) >>DesMaxBitcnt) & ByteOrTable[curSrcBitcntPro - curDesBitcntpro]);

		pTmpSrc--;
		bitcntProed += curSrcBitcntPro;
	}

	return;
}

unsigned int getbitu(/*const */unsigned char *buff, int pos, int len)
{
    unsigned int bits=0;
    int i;
    for (i=pos;i<pos+len;i++)
		bits=(bits<<1)+((buff[i/8]>>(7-i%8))&1u);

    return bits;
}




