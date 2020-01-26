#ifndef _UART_FUNC_
#define _UART_FUNC_

#include "define.h"
#include "typedefine.h"

extern INT32 UART_Send_Buff(UINT32 uCommId, const char *p, INT32 len);
extern void UartSend(UINT32 comId);

#endif


