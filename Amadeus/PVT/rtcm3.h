#ifndef __RTCM3_INTERFACE_H__
#define __RTCM3_INTERFACE_H__

#include "define.h"
#include "typedefine.h"
#include "bititemproc.h"
#include "global.h"
#include "rtk.h"
#include "FrameDecode.h"

#define RTCM3_MIN_PKG_LEN			6
#define RTCM3_HEAD_LEN				3
#define RTCM3_PARITY_LEN			3

#define RTCM3_PREAMBLE_SIZE		1
#define RTCM3_PAYLOAD_MAX_LEN	1023
#define RTCM3_PAYLOAD_OFFSET		3

#define RTCM3_MSM_MAX_PAGECNT	2

#define RTCM3_SIGIDX_GPS_1C		1	//Cronus RX and TX signal, L1 C/A
#define RTCM3_SIGIDX_BD_1I			1	//Cronus RX and TX signal, B1 I

/****************************Signal Mask Define***0~31********************************/
//GPS
#define RTCM3_SIGMASK_L1CA	1
#define RTCM3_SIGMASK_L1P	2
#define RTCM3_SIGMASK_L1Z	3
#define RTCM3_SIGMASK_L2CA	7
#define RTCM3_SIGMASK_L2P	8
#define RTCM3_SIGMASK_L2Z	9
#define RTCM3_SIGMASK_L2W   10
#define RTCM3_SIGMASK_L2CM	14
#define RTCM3_SIGMASK_L2CL	15
#define RTCM3_SIGMASK_L2CML	16
#define RTCM3_SIGMASK_L5I	21
#define RTCM3_SIGMASK_L5Q	22
#define RTCM3_SIGMASK_L5IQ	23
#define RTCM3_SIGMASK_L1CD	29
#define RTCM3_SIGMASK_L1CP	30
#define RTCM3_SIGMASK_L1CDP	31
//GLO
#define RTCM3_SIGMASK_G1CA	1
#define RTCM3_SIGMASK_G1P	2
#define RTCM3_SIGMASK_G2CA	7
#define RTCM3_SIGMASK_G2P	8
//BD
#define RTCM3_SIGMASK_B1I	1
#define RTCM3_SIGMASK_B1Q	2
#define RTCM3_SIGMASK_B1IQ	3
#define RTCM3_SIGMASK_B3I	7
#define RTCM3_SIGMASK_B3Q	8
#define RTCM3_SIGMASK_B3IQ	9
#define RTCM3_SIGMASK_B2I	13
#define RTCM3_SIGMASK_B2Q	14
#define RTCM3_SIGMASK_B2IQ	15


/***************************struct define for RTCM3 parse*********************************/
typedef struct {
	word16 	msg_id;
	word16 	data_len;
	byte *	p_data;
}RTCM3_PACKAGE_INFO;


typedef void (*pRTCM3RxHandleFun)(word16 msgid, byte const * const p_data, word16 data_len, STRU_UART_STREAM* pUartBuff);		// RX handle
typedef void (*pRTCM3TxHandleFun)(word16 msgid, byte const * const p_data, word16* pdata_len, STRU_UART_STREAM* pUartBuff);		// TX handle

typedef struct 
{
	word16 msg_id;												//message id	
	word16 msg_len;												//max message payload length
	pRTCM3RxHandleFun msg_rx_ptr;		// RX handle
	pRTCM3TxHandleFun msg_tx_ptr;	// TX handle	
	UINT16 sendcnt[MAX_UART_NUM];			//bit index in CPT table RTCM3 output map.
}RTCM3_msg_entry;

// define fix length Message description 
typedef struct{
	word16					msg_id;
	BITITEM_FIELD_DESC *		pFieldDesc;
}RTCM3_FIXLEN_MSG_DESC;

// defien variable length message description
typedef struct{
	word16				msg_id;
	BITITEM_FIELD_DESC *		pMsgHeadDesc;
	BITITEM_FIELD_DESC *		pMsgDataDesc;
	word16				repeatStructLen;
}RTCM3_VARLEN_MSG_DESC;

// defien MSM message description
typedef struct{
	byte					msm_id;
	BITITEM_FIELD_DESC *		pHeadDesc;
	BITITEM_FIELD_DESC *		pMapDesc;
	BITITEM_FIELD_DESC *		pSatDataDesc;
	BITITEM_FIELD_DESC *		pSigDataDesc;
}RTCM3_MSM_MSG_DESC;

/***************define for RTCM3.x message struct***********************************/
#define RTCM3_MSG_ID_INVALID		0xffff
#define RTCM3_MSM_ID_INVALID		0xff


//message 1001~1004 head
typedef struct {
	word16 	ReferStationID;	//DF003, reference station id.
	word32 	TOW;			//DF004, GPS epoch time.
	byte		SyncGNSSFlag;	//DF005, synchronous GNSS flag.
	byte		SVCnt;			//DF006, No. of GPS Satellite Signals Processed.
	byte		DiverFreeSmooth;	//DF007, GPS Divergence-free Smoothing Indicator
	byte		SmoothInterval;		//DF008, GPS somoothing interval.
}RTCM3_MSG_1001_1004_HEAD;



//message 1001 data
typedef struct {
	byte		svid;				//DF009, GPS satellite id.
	byte		L1CodeIndicate;		//DF010, GPS L1 Code Indicator
	word32	L1Pseudorange;		//DF011, GPS L1 Pseudorange, 
	int32	L1PhaRangeDifPR;	//DF012, GPS L1 PhaseRange-L1 Pseudorange, 
	byte		L1LockTimeIdx;		//DF013, GPS L1 Lock time Indicator
}RTCM3_MSG_1001_DATA;

//message 1002 data
typedef struct {
	byte		svid;				//DF009, GPS satellite id.
	byte		L1CodeIndicate;		//DF010, GPS L1 Code Indicator
	word32	L1Pseudorange;		//DF011, GPS L1 Pseudorange, unit:m
	int32	L1PhaRangeDifPR;	//DF012, GPS L1 PhaseRange-L1 Pseudorange, 
	byte		L1LockTimeIdx;		//DF013, GPS L1 Lock time Indicator
	byte		L1PRAmbiguity;		//DF014, GPS Integer L1 Pseudorange Modulus Ambiguity
	byte		L1CNR;				//DF015, GPS L1 CNR
}RTCM3_MSG_1002_DATA;

//message 1003 data
typedef struct {
	byte		svid;				//DF009, GPS satellite id.
	byte		L1CodeIndicate;		//DF010, GPS L1 COde Indicator
	word32	L1Pseudorange;		//DF011, GPS L1 Pseudorange, unit:m
	int32	L1PhaRangeDifPR;	//DF012, GPS L1 PhaseRange-L1 Pseudorange 
	byte		L1LockTimeIdx;		//DF013, GPS L1 Lock time Indicator
	byte		L2CodeIndicate;		//DF016, GPS L2 Code Indicator
	int16	L2PRDifL1PR;		//DF017, GPS L2-L1 Pseudorange Difference
	int32	L2PhaRangeDifL1PR;	//DF018, GPS L2 PhaseRange 每 L1 Pseudorange
	byte		L2LockTimeIdx;		//DF019, GPS L2 Lock time Indicator
}RTCM3_MSG_1003_DATA;

//message 1004 data
typedef struct {
	byte		svid;				//DF009, GPS satellite id.
	byte		L1CodeIndicate;		//DF010, GPS L1 Code Indicator
	word32	L1Pseudorange;		//DF011, GPS L1 Pseudorange, unit:m
	int32	L1PhaRangeDifPR;	//DF012, GPS L1 PhaseRange-L1 Pseudorange, 
	byte		L1LockTimeIdx;		//DF013, GPS L1 Lock time Indicator
	byte		L1PRAmbiguity;		//DF014, GPS Integer L1 Pseudorange Modulus Ambiguity
	byte		L1CNR;				//DF015, GPS L1 CNR
	byte		L2CodeIndicate;		//DF016, GPS L2 Code Indicator
	int16	L2PRDifL1PR;		//DF017, GPS L2-L1 Pseudorange Difference
	int32	L2PhaRangeDifL1PR;	//DF018, GPS L2 PhaseRange 每 L1 Pseudorange
	byte		L2LockTimeIdx;		//DF019, GPS L2 Lock time Indicator
	byte		L2CNR;				//DF020, GPS L2 CNR
}RTCM3_MSG_1004_DATA;

//message 1005 info
typedef struct {
	word16	ReferStationID;			//DF003, reference station id.	
	byte		ITRF_year;				//DF021, Reserved for ITRF Realization Year
	byte		GPS_Indicate;			//DF022, GPS Indicator
	byte		GLONASS_Indicate;		//DF023, GLONASS Indicator
	byte		Galileo_Indicate;		//DF024, Reserved for Galileo Indicator
	byte		ReferStation_Indicate;	//DF141, Reference-Station Indicator
	float64	RefPox_x;				//DF025, Antenna Reference Point ECEF-X
	byte		Osci_Indicate;			//DF142, Single Receiver Oscillator Indicator
	float64	RefPox_y;				//DF026, Antenna Reference Point ECEF-Y
	float64	RefPox_z;				//DF027, Antenna Reference Point ECEF-Z
}RTCM3_MSG_1005_INFO;

//message 1006 info
typedef struct {
	word16	ReferStationID;			//DF003, reference station id.	
	byte		ITRF_year;				//DF021, Reserved for ITRF Realization Year
	byte		GPS_Indicate;			//DF022, GPS Indicator
	byte		GLONASS_Indicate;		//DF023, GLONASS Indicator
	byte		Galileo_Indicate;		//DF024, Reserved for Galileo Indicator
	byte		ReferStation_Indicate;	//DF141, Reference-Station Indicator
	float64	RefPox_x;				//DF025, Antenna Reference Point ECEF-X
	byte		Osci_Indicate;			//DF142, Single Receiver Oscillator Indicator
	float64	RefPox_y;				//DF026, Antenna Reference Point ECEF-Y
	byte		QuCycle_Indicate;		//DF364, Quarter Cycle Indicator
	float64	RefPox_z;				//DF027, Antenna Reference Point ECEF-Z
	word16	AntHeight;				//DF028, Antenna Height
}RTCM3_MSG_1006_INFO;
//message 1009~1012 head
typedef struct {
	word16 ReferStationID; //DG003, reference station id.
	word32 TOW;          //DF034, GLONASS epoch time.
	byte SyncGNSSFlag;	//DF005, synchronous GNSS flag.
	byte SVCnt;			//DF035, No. of GLONASS Satellite Signals Processed.
	byte DiverFreeSmooth;	//DF036, GLONASS Divergence-free Smoothing Indicator
	byte SmoothInterval; 	//DF037, GLONASS somoothing interval.	
}RTCM3_MSG_1009_1012_HEAD;

//message 1012 data
typedef struct {
	byte		svid;				//DF038, GLONASS satellite svid.
	byte		L1CodeIndicate;		//DF039, GLONASS L1 Code Indicator
	byte		FreCh;				//DF040 ,GLONASS Satellite Frequency CHannel No.
	word32	L1Pseudorange;		//DF041, GLONASS L1 Pseudorange, unit:m
	int32	L1PhaRangeDifPR;	//DF042, GLONASS L1 PhaseRange-L1 Pseudorange, 
	byte		L1LockTimeIdx;		//DF043, GLONASS L1 Lock time Indicator
	byte 		L1PRAmbiguity;    //DF044,GLONASS Integer L1 Pseudorange Modulus Ambiguity
	byte		L1CNR;				//DF045, GLONASS L1 CNR
	byte		L2CodeIndicate;		//DF046, GLONASS L2 Code Indicator
	word16		L2PRDifL1PR;		//DF047, GLONASS L2-L1 Pseudorange Difference
	int32	L2PhaRangeDifL1PR;	//DF048, GLONASS L2 PhaseRange 每 L1 Pseudorange
	byte		L2LockTimeIdx;		//DF049, GLONASS L2 Lock time Indicator
	byte		L2CNR;				//DF050, GLONASS L2 CNR
}RTCM3_MSG_1012_DATA;

/** message 1019 and 1047 data
 * @brief  struct definition of Ephemeris, fields in bit mode
 */
typedef struct
{
	word32 svid;		//only used for RTCM3, AGNSS protocal
	word32 Tgd;	
	//word32 Tgd2;	//time group delay for B3
	word32 toc;
	word32 af2;
	word32 af1;
	word32 af0;
	word32 Crs; // Amplitude of the Sine Harmonic Correction Term to the Orbit Radius
	word32 delta_n; // Mean Motion Difference from Computed Value
	word32 M0; // Mean Anomaly at Reference Time
	word32 Cuc; // Amplitude of the Cosine Harmonic Correction Term to the Argument of Latitude
	word32 ecc; // Eccentricity
	word32 Cus; // Amplitude of the Sine Harmonic Correction Term to the Argument of Latitude
	word32 sqra; // Square Root of the Semi-Major Axis
	word32 toe; // Time of Ephemeris
	word32 Cic; // Amplitude of the Cosine Harmonic Correction Term to the Angle of Inclination
	word32 omega0; // Longitude of Ascending Node of Orbit Plane at Weekly Epoch
	word32 Cis; // Amplitude of the Sine Harmonic Correction Term to the Angle of Inclination
	word32 inc0; // Inclination Angle at Reference Time
	word32 Crc; // Amplitude of the Cosine Harmonic Correction Term to the Orbit Radius
	word32 w; // Argument of Perigee
	word32 omega_dot; // Rate of Right Ascension
	word32 i_dot; // Rate of Inclination Angle
	word32 iodc;
	word32 iode; // Issue of Data (Ephemeris)
	word32 week; // GPS week number
	word32 aodo;	//aodo
	word32 health;//sv health code, 0 ~ healthy, else ~ not health, GPS health: 6bit, Compass health: 1bit
	word32 uraidx;//URA Index
	word32 fit_interval;
	word32 codeOnL2;
	word32 L2Pflag;
}EphemerisBits;

//MSM head
typedef struct {
	word16 	ReferStationID;			//DF003, reference station id.	
	word32	TOW;					//GNSS Epoch Time, Specific for each GNSS. UINT30
	byte		MultiMsgBit;				//DF393, Multiple Message Bit
	byte		IODS;					//DF409, IODS 每 Issue of Data Station
	byte		ClkSteer_Indicate;		//DF411, Clock Steering Indicator
	byte		ExClk_Indicate;			//DF412, External Clock Indicator
	byte		DiverFreeSmooth;		//DF417 GNSS Divergence-free Smoothing Indicator
	byte 	SmoothInterval;			//DF418, GNSS Smoothing Interval
}RTCM3_MSM_HEAD_INFO;

typedef struct{
	byte 	SatMask[8];				//DF394, GNSS Satellite Mask
	byte 	SigMask[4];				//DF395, GNSS Signal Mask
	byte		CellMask[8];				//DF396, GNSS Cell Mask

	byte		NSat;
	byte		NSig;
}RTCM3_MSM_MAP;


//MSM satellite info, this struct include all the MSM satellite DF
typedef struct {
	byte		datamap;				//this map indicate the DF is valid or not. 
									//bit0, RoughRangeInter
									//bit1, ExSatInfo
									//bit2, RoughRange
									//bit3, RoughPhasePRRate
									
	byte		RoughRangeInter;		//DF397, The number of integer milliseconds in GNSS Satellite rough ranges
	byte		ExSatInfo;				//Specific for each GNSS, Extended Satellite Information
	word16 	RoughRange;			//DF398, GNSS Satellite rough ranges modulo 1 millisecond
	int16	RoughPhasePRRate;		//DF399, GNSS Satellite rough PhaseRangeRates
}RTCM3_MSM_SAT_INFO;

#define MSM_SAT_MAPIDX_RouPRInter		0
#define MSM_SAT_MAPIDX_ExSatInfo		1
#define MSM_SAT_MAPIDX_RouPR			2
#define MSM_SAT_MAPIDX_RouPhaPRate	3

//MSM satellite info, this struct include all the MSM signal DF
typedef struct {
	byte		HalfCycleIndicate;		//DF420, Half-cycle ambiguity indicator
	byte		LockTimeIdx;			//DF402, GNSS PhaseRange Lock Time Indicator
	byte		CNR;					//DF403, GNSS signal CNRs
	int16 	FineRange;				//DF400, GNSS signal fine Pseudoranges
	int16	FinePhasePRate;			//DF404, GNSS signal fine PhaseRangeRates
	int32 	FinePhasePR;			//DF401, GNSS signal fine PhaseRange data
	int32	FineRangeEx;			//DF405, GNSS signal fine Pseudoranges with extended resolution
	int32	FinePhasePREx;			//DF406, GNSS signal fine PhaseRange data with extended resolution
	word16	LockTimeIdxEx;			//DF407, GNSS PhaseRange Lock Time Indicator with extended range and resolution.
	word16	CNREx;					//DF408, GNSS signal CNRs with extended resolution
	word16	datamap;				//this map indicate the DF is valid or not. 
}RTCM3_MSM_SIG_INFO;

#define MSM_SIG_MAPIDX_HalfCycleIndicate	0
#define MSM_SIG_MAPIDX_LockTimeIdx		1
#define MSM_SIG_MAPIDX_CNR				2
#define MSM_SIG_MAPIDX_FineRange			3
#define MSM_SIG_MAPIDX_FinePhasePRate		4
#define MSM_SIG_MAPIDX_FinePhasePR		5
#define MSM_SIG_MAPIDX_FineRangeEx		6
#define MSM_SIG_MAPIDX_FinePhasePREx		7
#define MSM_SIG_MAPIDX_LockTimeIdxEx		8
#define MSM_SIG_MAPIDX_CNREx				9

typedef struct {
	int8 svidx[SV_NUM_TRUE];						//the same as c1610define. GPS:1~32, BD:33~64, SBAS:65~88
	RTCM3_MSM_SAT_INFO SatData[MAX_RTK_OBS_SVCNT];
	RTCM3_MSM_SIG_INFO SigData[MAX_RTK_OBS_SVCNT][MAX_FREPIONT_PER_NAVSYS];
}RTCM3_MSM_DATA_INFO;


/***********************structs for DF parse******************************/
typedef struct{
	byte indicator;
	word16 maxInterval;
}CSC_INTERVAL;

typedef struct{
	word16 maxIndicate;
	int32 para_a;
	int32 para_b;
	word32 maxlocktime;
}LOCK_TIME_INDICATE;

/************************************************************/

//declare function 
#ifdef __cplusplus
extern "C" {
#endif

extern RTCM3_MSM_DATA_INFO MSMDataInfo;
extern RTCM3_msg_entry RTCM3_MSG_TBL[];
extern byte RTCM3OutputDataBuf[RTCM3_PAYLOAD_MAX_LEN+RTCM3_MIN_PKG_LEN];

//process
INT32 RTCMRcvProc(INT32 *pValidDataLen,STRU_UART_STREAM *glStruUart);
void PeriodSendRtcmPro(void);

word32 ConvertRTCM3SigMaskidx2FP(int8 maskidx, byte navsys);
int8 ConvertFP2RTCM3SigMaskidx(word32 frepoint);
word32 IrregularInt2Word32(byte signbitidx,int32 sd);
int32 IrregularWord2Int32(byte signbitidx,word32 wd);


void ParseGPSEphInfo2EphBits(GPSBD2EphFrameStruct* p_sveph ,EphemerisBits* p_svephbits);
void ParseGPSEphBits2EphInfo(EphemerisBits* p_svephbits,  GPSBD2EphFrameStruct* p_sveph);



//table process
boolean FindRTCM3MsgEntry(word16 msg_id, RTCM3_msg_entry** pEntry);
boolean FindRTCM3FixLenMsgDesc(word16 msg_id, BITITEM_FIELD_DESC** pDesc);
boolean FindRTCM3VarLenMsgDesc(word16 msg_id, BITITEM_FIELD_DESC** pHeadDesc, BITITEM_FIELD_DESC** pDataDesc, word16* pRepeatStructLen);

boolean FindRTCM3MSMMsgDesc(byte msm_id, BITITEM_FIELD_DESC** pHeadDesc, BITITEM_FIELD_DESC** pMapDesc, BITITEM_FIELD_DESC** pSatDataDesc, BITITEM_FIELD_DESC** pSigDataDesc);

//MSM
void UpdateMSMDataInfo();
void SetMSMMaskBit(byte* pMask, byte idx);
byte GetMSMMaskBit(byte* pMask, byte idx);

int32 FindMSMDataSVIdx(RTCM3_MSM_DATA_INFO* pMSMDataInfo, int32 svid);
int32 GetC1610SVIDFromMSMSatMap(int32 mapidx, byte navsys);
boolean IsMSMOpen(void);
int32 UpdateTotalMSMCnt(byte comid);
void ResetAllMSMParam();
void FindMSMType(word16 msg_id, byte* p_msm_type, byte* p_navsys);
word16 FindMSMMsgID(byte msm_type, byte navsys);
bool IsMSMMsgRXFinished(void);

//create RTCM3 message
word16 CreateRTCM3MSMMSGRawData(byte msm_id, byte navsys, RTCM3_MSM_HEAD_INFO* pItemsHead, RTCM3_MSM_MAP* pMap, RTCM3_MSM_DATA_INFO* pMSMDataInfo, byte* pData, STRU_UART_STREAM *glStruUart);

word16 CreateRTCM3FixLenMSGRawData(word16 msg_id, void* pItems, byte* pData, STRU_UART_STREAM *glStruUart);
word16 CreateRTCM3VarLenMSGRawData(word16 msg_id, void* pItemsHead, void* pItemsData, byte repeatCnt, byte* pData, STRU_UART_STREAM *glStruUart);
bool RTCM3SendPackage(word16 msg_id, word16 pkg_len, byte *p_data, STRU_UART_STREAM *glStruUart);


//parse RTCM3 message
void ParseRTCM3FixLenItems(word16 msg_id, byte* pData, word16 datalen, void* pItems);
void ParseRTCM3VarLenItems(word16 msg_id, byte* pData, word16 datalen, void* pHeadItems, void* pDataItems, word16* pRepeatCnt);
void ParseRTCM3MSMItems(byte msm_id, byte navsys, byte* pData, word16 datalen, RTCM3_MSM_HEAD_INFO* pItemsHead, RTCM3_MSM_MAP* pMap, RTCM3_MSM_DATA_INFO* pMSMDataInfo);


//message handle
void handle_RTCM3_RX_MSG_1001(word16 msgid,  const byte * const p_data, word16 data_len, STRU_UART_STREAM *glStruUart);
void handle_RTCM3_RX_MSG_1002(word16 msgid,  const byte * const p_data, word16 data_len, STRU_UART_STREAM *glStruUart);
void handle_RTCM3_RX_MSG_1003(word16 msgid,  const byte * const p_data, word16 data_len, STRU_UART_STREAM *glStruUart);
void handle_RTCM3_RX_MSG_1004(word16 msgid,  const byte * const p_data, word16 data_len, STRU_UART_STREAM *glStruUart);
void handle_RTCM3_RX_MSG_1005(word16 msgid,  const byte * const p_data, word16 data_len, STRU_UART_STREAM *glStruUart);
void handle_RTCM3_RX_MSG_1006(word16 msgid, const byte * const p_data, word16 data_len, STRU_UART_STREAM *glStruUart);
void handle_RTCM3_RX_MSG_1012(word16 msgid,  const byte * const p_data, word16 data_len, STRU_UART_STREAM *glStruUart);
void handle_RTCM3_RX_MSG_1019(word16 msgid, const byte * const p_data, word16 data_len, STRU_UART_STREAM *glStruUart);
void handle_RTCM3_RX_MSG_1047(word16 msgid, const byte * const p_data, word16 data_len, STRU_UART_STREAM *glStruUart);
void handle_RTCM3_RX_MSM(word16 msgid, const byte * const p_data, word16 data_len, STRU_UART_STREAM *glStruUart);

void handle_RTCM3_TX_MSG_1004(word16 msgid,  const byte * const p_data, word16* pdata_len, STRU_UART_STREAM *glStruUart);
void handle_RTCM3_TX_MSG_1005(word16 msgid, const byte * const p_data, word16* pdata_len, STRU_UART_STREAM *glStruUart);
void handle_RTCM3_TX_MSG_1006(word16 msgid, const byte * const p_data, word16* pdata_len, STRU_UART_STREAM *glStruUart);
void handle_RTCM3_TX_MSG_1012(word16 msgid,  const byte * const p_data, word16* pdata_len, STRU_UART_STREAM *glStruUart);
void handle_RTCM3_TX_MSG_1019(word16 msgid, const byte * const p_data, word16* pdata_len, STRU_UART_STREAM *glStruUart);
void handle_RTCM3_TX_MSG_1047(word16 msgid, const byte * const p_data, word16* pdata_len, STRU_UART_STREAM *glStruUart);
void handle_RTCM3_TX_MSM(word16 msgid, const byte * const p_data, word16* pdata_len, STRU_UART_STREAM *glStruUart);



#ifdef __cplusplus
}
#endif


#endif

