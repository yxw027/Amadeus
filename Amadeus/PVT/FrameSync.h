
#ifndef _FRAME_SYNC_H
#define _FRAME_SYNC_H

#include "typedefine.h"
#include "define.h"
#include "pvt.h"

#ifndef _SIMULATE
#ifndef _POSTPROC
#include "BitSync.h"
#endif
#endif

#define SUBFRAME_LEN 10
#define FRAME_SYNO_BUF_LEN 10
#define SIZE_INT 32
#define SIZE_SF_WORD_GPS_BD2 30
#if SUPPORT_GLONASS
#define SIZE_SF_WORD_GLO 20
#define SIZE_SF_OUT_WORD_GLO 30
#define N_STRING_GLO  85
#endif
#define NORMAL 0
#define REVERSE 1
#define SBUFRAME_NUMBER_MAX 5
#define TH_SF_ERROS   10
#if SUPPORT_GLONASS
#define GLO_UNVAILED_Tk 1  //
#endif
#define SF_SYNC_FALL 0       //bit sync invaild
#define SF_SYNC_SUCCESS 1       //bit sync succeed
#if SUPPORT_GLONASS
#define SF_SYNC_GLO_UNKOUWN  3       //special for glonass(no time info.)
#endif

#define SF_SYNC_SOURCE_DATA 	0
#define SF_SYNC_SOURCE_FRE 		1
#define SF_SYNC_SOURCE_TR 		2
//data_in..........................................
//bit stream from bitbuffer

#ifndef _SIMULATE
//data_out...............................................
typedef struct
{
	word16 FreqPoint;
	word16 SV;               //satllite  No.
	int8 erros;
	int8 syncflag;          // 0:Sfsync unvail; 1 : Sfsync Succeedd;  3 state : (special for GLONASS time check)
	int8 reverse;
	int8 reverseValid;
	int8 SfId;

	int8 SfSyncSource;
	int8 health;		//only used for BD
	int16 bitCnt;          //GLONASS:(0~199) 100Hz;  
	
	int32 zcounter;             //  GPS/BD2Meo :(0~100799)  ; BD2Geo:(0~1007999); GLONASS:(0~302399)
	
	word32 rfcnt;				//clk of the last bit of data_in
	
    word32 subframe[2];      //SFhead = 2words 
    word32 OneSfData[10];//subframe（10words?

	

} SUBFRAME_BUFFER;



///////////////


//sub frame struct
typedef struct 
{  
    word16 vflg;                 // vaild flag
    word16 Sv;                   // satellite No.
    word16 FreqPoint;
    word32 word[10];             // 10words = 1sub frame
    
}newsf123;

typedef struct 
{
	word16 vflg;   //vflg 的低位对应是每一帧的有效标志。
	word16 Sv;
	word16 FreqPoint;
	word32 word[3][10];
}Ephsf123;


// BD2GEO sub frame struct
typedef struct 
{  
    word16 vflg;         // vaild flag
    word16 FreqPoint;
    word32 word[10][10];     // 10words = 1sub frame
}BD2GeoD2newsf;

extern newsf123 AlmFrameBuffer[];
extern Ephsf123 EphFrameBuffer[];  //3
extern BD2GeoD2newsf GeoD2EphFrameBuffer[];		// 5 GEO staellites, 10 pages
extern BD2GeoD2newsf GeoD2EphFrameBufferB2[];
extern BD2GeoD2newsf GeoD2EphFrameBufferB3[];

extern SUBFRAME_BUFFER SfDataInfo[];

extern void  InitalSFSync();
extern void ResetSfSync(int32 trkch);
extern int8 getTrkchSyncInfo(int32 trkid, int32 *bitCnt, word32 *RFclk, int32 *zcounter, PVT_TRKCH_INFO *pCurPVTTrkchInfo);
extern bool CheckCHSyncInfo(int32 trkch0, int32 zounter);
extern void ResetTrkchReverse(int32 trkid);

#endif
#endif
