
#ifndef _ASSIT_FUNC_4BIN_H_
#define _ASSIT_FUNC_4BIN_H_

#include "define.h"


#define	MAX_LEN_OUT_BUFF	(7*1024)

typedef struct _tag_OUT_BUFF
{
	U1 buff[MAX_LEN_OUT_BUFF];
	I4 curIdx;
} T_OUT_BUFF;


#define BUFF_AppendLittleU1(pBuff, data)	BUFF_AppendLittle((pBuff), (U1*)(&data), 1)
#define BUFF_AppendLittleI1(pBuff, data)	BUFF_AppendLittle((pBuff), (U1*)(&data), 1)
#define BUFF_AppendLittleU2(pBuff, data)	BUFF_AppendLittle((pBuff), (U1*)(&data), 2)
#define BUFF_AppendLittleI2(pBuff, data)	BUFF_AppendLittle((pBuff), (U1*)(&data), 2)
#define BUFF_AppendLittleU4(pBuff, data)	BUFF_AppendLittle((pBuff), (U1*)(&data), 4)
#define BUFF_AppendLittleI4(pBuff, data)	BUFF_AppendLittle((pBuff), (U1*)(&data), 4)
#define BUFF_AppendLittleF4(pBuff, data)	BUFF_AppendLittle((pBuff), (U1*)(&data), 4)
#define BUFF_AppendLittleF8(pBuff, data)	BUFF_AppendLittle((pBuff), (U1*)(&data), 8)

extern T_OUT_BUFF g_OutBuff;

void BUFF_Clear(T_OUT_BUFF *pBuff);
void BUFF_AppendLittle(T_OUT_BUFF *pBuff, const U1 *pData, I4 len);
U2 BUFF_GetCurIdx(const T_OUT_BUFF *pBuff);
void BUFF_SetCurIdx(T_OUT_BUFF *pBuff, U2 index);


#endif


