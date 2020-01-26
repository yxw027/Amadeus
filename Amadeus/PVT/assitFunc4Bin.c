
#include "assitFunc4Bin.h"

T_OUT_BUFF g_OutBuff;

void BUFF_Clear(T_OUT_BUFF *pBuff)
{
	pBuff->curIdx = 0;
}

void BUFF_AppendLittle(T_OUT_BUFF *pBuff, const U1 *pData, I4 len)
{
	I4 i;
	
	for (i=0; i<len; i++)
	{
		if(pBuff->curIdx >= MAX_LEN_OUT_BUFF)
		{
			pBuff->curIdx = 0;
			break;
		}
		
		pBuff->buff[pBuff->curIdx++] = *pData++;
	}
}

U2 BUFF_GetCurIdx(const T_OUT_BUFF *pBuff)
{
	return (pBuff->curIdx);
}

void BUFF_SetCurIdx(T_OUT_BUFF *pBuff, U2 index)
{
	pBuff->curIdx = index;
}






