#ifndef _HEAD_GLOBAL_H_
#define _HEAD_GLOBAL_H_

#include "typedefine.h"
#include "define.h"

// 通道状态结构体，用于提取原始观测量和通道控制
typedef struct STRU_ACQU_CHANNEL_tag
{   
	int16      SvID;
    UINT16     FreqPoint;
	int8      State;
	byte		branch;
	int8		Acqmode;
	byte       spancnt; 
	
	UINT16 		idlecnt;
	int16      ModifyFre;
	byte       freFlag;
	int8 		sid;	
	int8		wordindex;
	
	int16		bitcnt;
	int16		deltabit;	
	int32		zcounter;
	
	word32		swAssistRFHigh;
	word32 		swAssistRFLow;
	word64_rtc  lockedStartTime;
	double  steadyStartTime;
#ifdef _FAULT_LOST_LOCK	
	byte 		faultlostcnt;
#endif	
}STRU_ACQU_CHANNEL_INFO;

// 捕获状态
typedef struct STRU_ACQU_tag
{
    UINT16                Channel;            // 当前捕获通道
    int8                  State;              // 当前捕获状态  0--ACQU_IDLE 1--ACQU_BUSY
    int8                  gl_spancnt;
    STRU_ACQU_CHANNEL_INFO  Info[MAXCHANNELS];
}STRU_ACQU;


typedef struct {
	//byte trkch[MAX_FREPIONT_PER_NAVSYS];
	//int8 health; //SV health situation, 0 is healthy; others are not healthy
	int8 AcqTotalFailCnt;			//total acq failed count
	int8 AcqFreFailCnt[MAX_FREPIONT_PER_NAVSYS][ACQ_MODE_CNT];
	int8 swdopSrc;
	int8 almsrc;
	int8 ephsrc;
	UINT8 ephparsecnt;
	UINT8 almparsecnt;

	int16 turesvid;	//only used for glonass
	
	int32 eph_age;	//ephemeris age, signed interger, unit: second
	int32 alm_age;  //almanac age, signed interger, unit: second
	
	double el; //SV elevation, degree
	double az; //SV azimuth, degree

	double sw_doppler[MAX_FREPIONT_PER_NAVSYS];	//software Doppler frequency shift, GPS: 0 - L1, 1-L2, 2-L5; BD: 0 - B1, 1-B2, 2-B3; GLO: 0 - G1, 1-G2;
}SV_INFO;

typedef struct STRU_UART_STREAM_tag
{
    UINT32  CmdCurPos;
    UINT32  CmdSetPos;
    char    Buf[UART_RECV_BUF_LENTH];
}STRU_UART_DATA;

typedef struct {
	byte 	comid;	
	STRU_UART_DATA RecvData;
	STRU_UART_DATA SendData;
	char    CmdBuf[ONE_MSG_MAX_LENTH];
	int 	CheckProtocolHead;
	int 	ActiveCPT;
}STRU_UART_STREAM;

typedef struct {
	void (*task_ptr)(void);		/* task pointer 	 */
	int16	cycle;		/* task open/close flag */
}sys_task_rec;	/* task record in system package */

#ifdef __cplusplus
extern "C" {
#endif

extern sys_task_rec TSK1_List[];
extern STRU_ACQU glStruAcqu;
extern SV_INFO SVInfo[SV_NUM];
extern STRU_UART_STREAM glStruUart[MAX_UART_NUM];
extern char VersionInfo[];
extern double RF_SAMP_FREQ;
extern double RF_SAMP_FREQ100M;
extern double RECIP_RF_SAMP_FREQ;
extern int32  RF_SAMP_FREQ_HALF_INT;
extern int32  RF_SAMP_FREQ_INT;
extern int32 RFCNT_HIGH_REVERSE_TIME;
extern int32  LOW_32BIT_INVERT;
extern UINT32 TFT_OP_CNT;

extern void Task1_PVT(void);
extern void Task2_NavDecode(void);
extern void InitGlobalData(void);
extern void InitSysIFParam(void);
extern void RestGlStruAcquChInfo(int32 trkch);
extern int32 GetTRKChSvid(int32 trkch);
extern UINT16 GetTRKChFrepoint(int32 trkch);
extern void Clear_LOOP(int32 trkch);
extern void LOOP_loselock(int32 trkch, boolean bCfgToBB);
extern void Clr_LOOP_ISR_Info(int32 trkch);
extern double GetFreqpointIF(int16 svid, UINT16 frepoint);
extern double GetFreqpointPLL(UINT16 frepoint);
extern double GetFreqpointRF(int16 svid, UINT16 frepoint);
extern bool GetSVTrkchID(int16 svid, int16 trkch[MAX_TRKCHCNT_PER_ONESV]);
extern bool IsTrkChValid(int32 trkch);
extern double GetSVWaveLen(int16 svid, UINT16 freqpoint);
extern int8 GetFreGropIdx(UINT16 frepoint);
extern int32 CalcFPointGroupCnt(word32 frepoint);
extern UINT16 GetFrepointByFPGrpidx(int32 svid, int8 idx);
extern void ResetSVInfo(void);
extern void ResetSystem(void);
extern int32 GetNavSysIdx(int32 svid);
extern word32 GetNavSysFPMap(int32 sysidx);

#ifdef __cplusplus
}
#endif

#endif


