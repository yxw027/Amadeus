#ifndef __BITITEMPROC_H__
#define __BITITEMPROC_H__

#include "define.h"


#define BITITEM_DF_ID_INVALID		0xffff

#define BITITEM_MAX_ITEM_LEN	256

/************************data type define for RTCM3 item******************************/
enum{
	B_T=1,		//bit type. bit(N)
	C_T,		//char8, 8bit characters. char8(N)
	I_T,			//N bit 2's complement integer
	U_T,		//N bit unsigned integer
	S_T,		//N bit sign-magnitude integer
	UN_T,		//Unicode UTF-8 code unit, 00h to FFh  UN(N)
	BITITEM_INVALID_T = 0xFF
};


//define item parse and create table
typedef struct{
	word16 DF_idx;		//data field idx. For RTCM3.2, value is 1~426. For AGNSS, ingore this item
	word16 DF_Offset;	//data field offset in the RTCM3 data flow.
	byte DF_Type;		//data field type: B_T, C_T, I_T, U_T, S_T, UN_T	
	byte DF_Len;			//N, count of bit		//N, for B_T, I_T, U_T, S_T, N is unit of bit; for C_T, unit is char8; for UN_T, unit is UTF-8
	byte validmapidx;	//only used for RTCM3 MSM, bit map idx, this bit in struct map indicates this DF valid or not.
	int32 struct_fieldOffset;	//struct field offset in cronus
	int32 struct_fieldSize;		//struct field size in cronus
}BITITEM_FIELD_DESC;


//declare function 
#ifdef __cplusplus
extern "C" {
#endif

//receive, parse items from bit flow
boolean ParseBitItemRawData2Items(BITITEM_FIELD_DESC* pFieldDesc, byte* pData, word16 datalen, word16* pBitoffset, void* pItems);
boolean ParseBitItemOneItemFromRawData(BITITEM_FIELD_DESC* pFieldDesc, byte* pData, word16 datalen, word16 bitoffset, char* info_pos);
void getBitsFromDataFlow(byte* pDes, byte* pSrc, word16 bitoffset, byte bitcnt);

//create
word16 CreateBitItemRawDataFromItems(BITITEM_FIELD_DESC* pFieldDesc, void* pItems, byte* pData, word16 firstbitoffset);
word16 CreateBitItemRawDataFromOneItem(BITITEM_FIELD_DESC* pFieldDesc, char* pItem, byte* pData, word16 bitoffset);
void createBitsToDataFlow(byte* pDes, byte* pSrc, word16 bitoffset, byte bitcnt);


unsigned int getbitu(/*const */unsigned char *buff, int pos, int len);

#ifdef __cplusplus
}
#endif

#endif

