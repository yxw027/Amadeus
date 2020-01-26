
#ifndef RMO_H_
#define RMO_H_
#include <string.h>
#include "TimeProc.h"
#include "Coordinate.h"
//#ifndef _SIMULATE
//#include "UartFunc.h"
//#endif


#define NMEA_FIELD_LENTH 100
#define NMEA_END_SYMBOL '*'
#define RMP_STAR_OFFSET  4

#define NMEA_STYLE_NUM  5
#define RMO_INSTRUCT_NUM 3
#define RMO_OPENMODE_NUM 1


/***Extended NMEA ID Define****/
#define EXT_NMEA_RMO_ID 		0X1
#define EXT_NMEA_NAVMOD_ID 		0X2
#define EXT_NMEA_MSS_ID  		0X3
#define EXT_NMEA_RIS_ID			0X4
#define EXT_NMEA_SIR_ID         0X5
#define EXT_NMEA_BMI_ID			0x6
#define EXT_NMEA_HCP_ID         0X7



/******RMO cmd sentence define******/
#define RMO_SENTENCE_NO 	0X0
#define RMO_SENTENCE_GGA 	0X1
#define RMO_SENTENCE_GSA 	0X2
#define RMO_SENTENCE_GSV 	0X3
#define RMO_SENTENCE_RMC 	0X4
#define RMO_SENTENCE_DHV 	0X5


/********CC cmd define*******/
#define CCRIS_MODE  1
#define CCBDS_MODE  2
#define CCGPS_MODE  3
#if SUPPORT_GLONASS
#define CCGLO_MODE  4
#endif
#define CCGNS_MODE  5
#define CCB1S_MODE  6
#define CCBLS_MODE  7


#define BD_RNSS_POS_MODE		0x1
#define GPS_POS_MODE			0x4
#define BD_GPS_POS_MODE			0x5


typedef enum _MSS_TEST
{	
	MISS_CODE_RATE			= 0,
	NAVMODE_TEST			= 1,
	COLD_START_TEST			= 2,
	WARM_START_TEST			= 3,
	HOT_START_TEST          = 4	
}MSS_TEST;


typedef struct _RMO_PROTO
{
	char 	NMEACmdStyle[5];
	char 	RmoInstruct[3];
	int 	OpenMode;
	double   Freq;
	int 	reserve;	
}RMO_PROTO;

typedef struct _MSS_PROTO
{
	char 	NMEACmdStyle[5];
	char 	INQorSetNAV;
	char	MSSmode;
	int 	NAVorTestmode;
	char	freq1[2];
	char 	reserv1;
	char	freq2[2];
	char 	reserv2;
	char	freq3[2];
	char 	reserv3;
}MSS_PROTO;

typedef struct _BMI_PROTO
{
	char 	NMEACmdStyle[5];
	char	Lon;	//E/W
	char	Lat;  //S/N
	char 	AltUnit;//m
	WGS		wgspos;
	UTC_TIME UTCTime;
}BMI_PROTO;

typedef struct _SIR_PROTO
{
	char 	NMEACmdStyle[5];
	char    NavFreqPoint;
	char    bootmode;
}SIR_PROTO;

typedef struct _HCP_COMMAND
{
	char	NMEACmdStyle[5];
	char	OpenCmd;
}HCP_COMMAND;
typedef struct _INSTRUCT_RMO
{
    int GGA:1;            
	int GSA:1;
	int RMC:1;
    int GSV:1;
    int DHV:1; // 1--发送该语句，2--停止发送该语句     
}INSTRUCT_RMO;


typedef struct _CMD_NMEA
{
	char CCRMO;
	char CCRIS;
	char CCBDS;
	char CCGPS;
#if SUPPORT_GLONASS
	char CCGLO;
#endif
	char CCRNS;
	char CCB1S;
	char CCBLS;
}CMD_NMEA;


typedef struct _NMEA_TASK_PROCESS
{
	char	NMEACmdStyle;		//类型命令
//	char 	RmoOpenMode;
	void	(*ProtocolNMEAProcess)(byte comid,char* CmdBuf);			//数据内容
}NMEA_TASK_PROCESS;

void  NMEAHandleProcess(STRU_UART_STREAM *glStruUart);
void  StoreNMEAData( char*);
int   NMEAParityCheck(int ,char*);
int   NMEARcvProc(int*,STRU_UART_STREAM * );
void  NMEAField(char* ,char *,unsigned short);
void  NMEACmdAnalysis(char* CmdBuf);
void  Str2Hex(char *, char *, int);
void  Handle_RMO_Process(void);

#endif
