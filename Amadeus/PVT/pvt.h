
#ifndef _HEAD_DEFINE_H_
#define _HEAD_DEFINE_H_

#include "define.h"
#include "coordinate.h"
#ifndef _POSTPROC
#ifndef _SIMULATE
#include "hcp.h"
#endif
#endif

#define OBS_BUF_CNT				120

#define FIX_SV_OK				0
#define FIX_SV_NO_PSEUDORANGE	1
#define FIX_SV_NAV_SYSTEM		2
#define FIX_SV_MINCN0			3
#define FIX_SV_URA				4
#define FIX_SV_ELEVATION		5
#define FIX_SV_DOPDIF			6
#define FIX_SV_DUPLITE			7
#define FIX_SV_USER_PARAM		8
#define FIX_SV_GEO_TCXODIF		9
#define FIX_SV_EXCESS			10
#define FIX_SV_SNR_VALUE		11
#define FIX_SV_RESIDUE_PR		12
#define FIX_SV_RESIDUE_FD		13
#define FIX_SV_RESIDUE_PRTSDIF	14
#define FIX_SV_TSDIF_FRE        15
#define FIX_SV_PR_ERROR			19
#define FIX_SV_SYNC_ERROR		20
#define FIX_SV_GEO_SWFREQDIF	21
#define FIX_SV_KFFALSE_CN0	    22
 
#define KF_PVT_UNKNOWN		0
#define KF_PVT_FAIL			1
#define KF_PVT_GOOD			2

#define BITMAP_DIM_BU_GPS			(1<<0)
#define BITMAP_DIM_BU_BD2			(1<<1)
#define BITMAP_DIM_BU_GLO			(1<<2)
#define BITMAP_DIM_BU_GPS_BD2		(1<<3)
#define BITMAP_DIM_BU_GPS_GLO		(1<<4)
#define BITMAP_DIM_BU_BD2_GLO		(1<<5)
#define BITMAP_DIM_BU_GPS_BD2_GLO	(1<<6)

#define FIX_NOT		0
#define FIX_2D		1
#define FIX_3D		2
#define FIX_RTD		3
#define FIX_WIDE_FLOAT		4
#define FIX_L1_FLOAT		5
#define FIX_NARROW_FLOAT	6
#define FIX_WIDE_INT		7
#define FIX_L1_INT			8
#define FIX_NARROW_INT		9


#define MAXCHANNELS_PVT	(MAXCHANNELS+1)	//use the last channel for ECA

#define SV_TYPE_GPS	(1<<0)
#define SV_TYPE_BD	(1<<1)
#define SV_TYPE_GLO	(1<<2)

#define FILTER_TYPE_NONE		0
#define FILTER_TYPE_EXTEND		2
#define FILTER_TYPE_RECKON		3

#define SV_NUM_FIX_3D	4
#define SV_NUM_FIX_2D	3

#define SV_CANDIDATE_SORT_TYPE_ASC		1
#define SV_CANDIDATE_SORT_TYPE_DESC		2

#define SORT_SV_ITEM_CN0		0
#define SORT_SV_ITEM_EPH_AGE	1
#define SORT_SV_ITEM_LOCK_TIME	2
#define SORT_SV_ITEM_DELTA_FREQ	3
#define SORT_SV_ITEM_PRTSDIF	4

#define SYS_FULL_OPEN_SKY		1
#define SYS_OPEN_SKY			2
#define SYS_HALF_OPEN_SKY		3
#define SYS_DOWN_TOWN			4
#define SYS_FULL_CLOSE_ENV		5

#define SYS_CONDITION_NORMAL		1
#define SYS_CONDITION_SIGNAL		2

#define	FIX_POS					(1<<0)
#define	FIX_VEL					(1<<1)
#define	FIX_POS_VEL			(FIX_POS | FIX_VEL)

#define MAX_SPEED					(600)	//unit: m/s
#define MAX_VERTICAL_SPEED			(1.5)	//MAX vetical speed: 1.1m/s by china road technical standard,
#define MAX_VERTICAL_SPEED_TH		(3 * MAX_VERTICAL_SPEED)	// set 3*max_vel_speed as the threshold.
#define MAX_ALTITUDE			(40000)  //unit:m
#define MIN_ALTITUDE			(-2000)  //unit:m

//载波相位跟踪质量 失锁门限
#define THETA_THRESH_LOSE 	352

typedef struct 
{
	double gdop;//general
	double pdop;//position
	double bdtdop;	//BD time
	double gpstdop;//gps time
#if SUPPORT_GLONASS
	double glotdop;//glo time
#endif
	double vdop;//vertical
	double hdop;//horizontal
}FIX_DOP;

typedef struct
{
	bool bvalid;
	UINT8 trksta;	//TRK parameter state.
	int16 C2FAI;
	UINT8 CCBF;
	UINT8 NCnt;
	UINT8 MCnt;

	UINT16 FreqPoint;
	int16 SvID;
	UINT16 nco;
	UINT16 chipcnt;
	UINT16 decp;
	UINT32 intp;
	UINT32 snr;
	UINT32 intpVersCnt;
	double fd;	//calc by dsp

	UINT16 weakcnt;
	UINT16 strongcnt;
} OBS_TRKCH_INFO;

typedef struct
{
	UINT32 RFSampleCnt_L;					//TIC lock RF sample cnt;
	UINT32 NxRFSampleCnt_L;					//TIC lock RF sample cnt;
	OBS_TRKCH_INFO chObs[MAXCHANNELS];
} OBS_INFO;

typedef struct STRU_OBS_tag
{
    int16          CurIndex;
    int16          SetIndex;
    OBS_INFO       Obs[OBS_BUF_CNT];
}OBS_BUF;

typedef struct
{
	bool bPRValid;	//Is PR valid?
	int8 svid;	//1~110, for glonass this means slotid.
	word16 freq_point;	
	int8 antenna;	//0, defualt antenna; 1, antenna1; 2, antenna2
	int8 branch;	//data or pilot branch information
	int8 trkmode;	// trkmode 1~N, parameter index.
	int8 C2FAI;		//C2Fai
	
	byte CCBF;		//CCBF
	int8 synflag;
	int8 cn1s;	//1s CN0
	int8 cn100ms; //100ms CN0
	int8 selpath;	//select to fix path, debug info.
	int8 ionosrc;
	int8 syncsrc;	//normal ~ 0 , calc  ~1
	int8 lockedcnt;
	
	//test code start
	int16 Ncnt;
	int8 Mcnt;
	byte strongcnt;
	byte weakcnt;
	int8 el;
	int16 az;

	UINT32 codenco;
	UINT32 codechip;

	UINT32 framenum;
	UINT16 bitnum;
	//test code end
	UINT16 carrDec;	//carrier phase

	UINT32 carrIntInvCnt;
	UINT32 carrInt;	//carrier integer cycle

	double ts;	//sv transmit time;
	double fd;	//sv doppler;
	double clkerr;	//sv clock error;
	double frqerr;	//sv frequency error;
	double pr;	//RTK use pseudorang, Unit: m
	double phase;	//RTK use phase, Unit: cycle
	ECEF svpos;	//sv position;
	ECEF svvel;	//sv velocity;
	double iono;
	double tropo;
	double resiErr_pr;	//pr residue error;
	double resiErr_fd;	//fd residue error;

	//test code start
	double difts;	//ts error
	double mserrper;
	double rate_pr;
	double rate_fd;
	double R_pr;
	double R_fd;
	UINT32 bitRFcnt;
	//int32 lockedtime;	//Unit: 0.1s
	int32 steadyLockedtime;	//Unit: 0.1s
	int8 Tleaps;	//only used for glonass
	int8 reverse;
	int8 reverseValid;
	word32 toertk;

	//test code end
} PVT_TRKCH_INFO;

typedef struct
{
	int8 posres;	//NOT_FIX=0; FIX_2D=1; FIX_3D=2; FIX_RECKON=3; FIX_DR=4;
	int8 velres;	//NOT_FIX=0; FIX_2D=1; FIX_3D=2; FIX_RECKON=3; FIX_DR=4;
	int8 filtertype;	//FILTER_TYPE_NONE=0, FILTER_TYPE_EXTEND=2, FILTER_TYPE_RECKON=3;  LS, this item is always FILTER_TYPE_NONE;
	byte buflag;	//local clock error flag. bit value 0, invalid; 1,valid;
					//bit0, gpsbu
					//bit1, bd2bu
					//bit2, globu
					//bit3, gps+bd2 mix bu
					//bit4, gps+glo mix bu
					//bit5, bd2+glo mix bu
					//bit6, gps+bd2+glo mix bu
	word256 poschmap;	//trk channel map used to position calculation;
	word256 velchmap;	//trk channel map used to velocity calculation;
	double bd2bu; //local clock error to bd2 system;
	double gpsbu; //local clock error to gps system;
	double globu; //local clock error to glonass system;
	double ctu;   //local clock drift;
	ECEF rcvrpos;	//receiver position
	ECEF rcvrvel;	//receiver velocity
	FIX_DOP dop; //gdop,pdop,hdop,vdop,tdop_gps,tdop_bd2,tdop_glo
} PVT_FIX_INFO;

typedef struct
{
	byte TimeMap;
	int16	WN;		//week number
	double RFCnt;
	double RFCntEx;
	double GpsTime;
	double Bd2Time;
	double GloTime;
	double clkerr_gps;
	double clkerr_bd;
	double clkerr_glo;
	double bias_gps2bd;	//m
	double bias_gps2glo;//m
	double bias_bd2glo;	//m
}RFCNT_NAVTIME;


typedef struct {
	byte prn;
	byte trkch;
	byte cn0;
	int16 trklocktime;
	int16 deltafreq;
	int32 ephage;
	int32 prtsdif;

}SV_CODE_CARRIER_TRK_STATUS;

typedef struct{
	int16 multiCnt;
	int8 cn0[MAX_TRKCHCNT_PER_ONESV];
	int16 trkch[MAX_TRKCHCNT_PER_ONESV];

	int32 normalSfSyncMap;
	int32 frqdotIdxMap[FREQ_CNT];
	int8 maxcn0;
	int8 maxcn0idx;
}SV_MULTI_INFO;

typedef struct{
	bool bPRModify;
	double bias;
}PR_MODIFY;

typedef struct{
	double x;
	double y;
	double z;
}ALPHA;

typedef struct{
	int8 selpath;
	byte svid;
	byte cn0;
	double prcovariance;		//Pseudorange covariance (R)
	double freqcovariance;	//Frequency covariance (R)
	double prresidue;			//pseudorange residue, unit: m
	double freqresidue;		//frequency residue, unit: m/s	
	ALPHA alpha;
}KALMAN_TRKCH_INFO;

typedef struct{
	bool valid;
	int32 minTsTrkch;
	int32 maxTsTrkch;
	double minTs;
	double maxTs;
}MAX_MIN_TS_INFO;


extern UINT16 BdRefWeek;
extern double BdRefSec;


#ifdef __cplusplus
extern "C" {
#endif

extern PVT_TRKCH_INFO PVTTrkchInfo[MAXCHANNELS_PVT];
extern double TICLock_RFcnt;
extern word256 CoarseSelchmap;
extern double FixCycle;

extern  boolean bLsFixQualityOk;
extern boolean bLsFixCoarseChkOK;
extern bool bFixAtSecBoundary;

extern bool bNeed2CalcSinglePos;

extern int16 lostByPRErrCnt;
extern int16 lostByOutViewCnt;
extern int16 lostByCrossErrCnt;
extern int16 lostBySNR;
extern int16 lostByDifFre;
extern int16 lostByDupliSV;
extern int16 lostBySyncChk;
extern int16 lostByLongTimeSync;
extern int16 lostByDummySV;
extern int16 lostByIllSV;
extern int8 lostByOutViewCnt_trk;
extern int8 lostByCaif;
extern int32 checkhispospath;
extern OBS_BUF    glStruObs;
extern word32 gobsTFTIdx;
extern SV_CODE_CARRIER_TRK_STATUS CandidateSVList[MAXCHANNELS];
extern int32 CandidateSVListLen;
extern UINT16 mycoasting;

extern void ISRTakeMeasPhase(void);
extern void InitPVTTask(void);
extern void ResetTrkchLastObs(int32 trkch);
extern void TaskPVT(void);
extern void sortSVCandidateInfo(byte sort_type, byte field_name); 
extern int32 getCurFixPosInfo(ECEF *pPos, FIX_DOP *pDOP, word256 *pChmap);
extern int32 getCurFixVelInfo(ECEF *pVel, word256 *pChmap);
extern int32 getCurFixGPSTime(int16 *pWN, double* pTow, double* pRFcnt, double* pRFcntEx,double* pClkerr);
extern int32 getCurFixBDTime(int16*pWN, double* pTow, double* pRFcnt, double* pRFcntEx,double* pClkerr);
#if SUPPORT_GLONASS
extern int32 getCurFixGloTime(int16 *pWN, double* pTow, double* pRFcnt, double* pRFcntEx,double* pClkerr);
#endif
extern int32 getCurFixCtu(double *pCtu);
extern int8 getCurFilterType(void);
extern double getTICLockRFcnt(void);
extern double getTICLockRFcnt100M(void);
	

extern bool getLatestFixStatus(void);
extern byte getCurFixBuFlag(void);
extern bool getCurFixGps2BDBias(double* pBias);
extern bool CheckNavHisPos(PVT_FIX_INFO* pCurFixInfo);
extern int32 FindSVOrbitType(int32 svid);
extern word64_rtc GetTrkchLockedTime(int32 trkch, word64_rtc curRFcnt);
#ifndef _POSTPROC
#ifndef _SIMULATE
extern void SetTestPosVel(ECEF* pPos, ECEF* pVel);
extern bool GetTestPosVel(ECEF* pPos, ECEF* pVel);
extern bool GetTRKChDebugData(int32 trkch, DBG_SYSM_TRK *pTrkDbgData);
extern void GetMiscDebugData(DBG_SYSM_MISC *pMiscDbgData);
extern void GetACQChDebugData(int32 ACQch, DBG_SYSM_ACQ *pACQDbgData);
extern bool IsSecBoundaryChange(void);

#endif
#endif

#ifdef __cplusplus
}
#endif

#endif

