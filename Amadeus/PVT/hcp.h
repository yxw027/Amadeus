
#ifndef HCP_H_
#define HCP_H_
#include <string.h>
#include "typedefine.h"
#include "global.h"
//#ifndef _SIMULATE
//#include "UartFunc.h"
//#endif

#define DEBUG 1
#define dprintf if(DEBUG) printf 


#define  HEAD1_HCP  0X48
#define  HEAD2_HCP  0X43
#define  HEAD_NMEA	'$'
#define  HEAD1_RTCM  0XD3
#define  HEAD2_RTCM  0X00

#define  HCP_HEAD_NUM_PROTO  2
#define  RMO_HEAD_NUM_PROTO  3
#define  RTCM_HEAD_NUM_PROTO  2

#define  HCP_NODATA_LENTH 8
#define  HCP_MAXDATA_LENTH 2048
#define  RMO_MAXDATA_LENTH 256

#define  HCP_HEAD1_OFFSET	0
#define  HCP_HEAD2_OFFSET	1
#define  HCP_CLASSID_OFFSET	2
#define  HCP_MESSAGEID_OFFSET    3
#define  HCP_PAYLOADLENTH_OFFSET	4
#define	 HCP_PAYLOAD_OFFSET	6
#define	 HCP_NODATA_CK1_OFFSET	6
#define	 HCP_NODATA_CK2_OFFSET	7

#define	PROTO_WAITE_DATA	1
#define PROTO_PARSE_SUCCESS	2
#define PROTO_PARSE_FAIL	3


#define	 GET_DATA_LEN_TRUE	1
#define  GET_DATA_LEN_FALSE  0

#define	 CHECK_DATA_TRUE	1
#define  CHECK_DATA_FALSE  0


#define  CHECK_DATA_HEAD_FALSE  0
#define  CHECK_DATA_HEAD_NMEA  		(0x1)
#define  CHECK_DATA_HEAD_HCP  		(0x1<<1)
#define  CHECK_DATA_HEAD_RMO  		(0x1<<2)
#define  CHECK_DATA_HEAD_RTCM3  	(0x1<<3)
#define  CHECK_DATA_HEAD_NORATEL  	(0x1<<4)
#define  CHECK_DATA_HEAD_HWAASIC	(0x1<<5)

#define  CHECK_READ_BUFFER_OVERFLOW  1
#define  CHECK_READ_BUFFER_NO_OVERFLOW  0

#define  HCP_CLASSID_NUM	16
#define  HCP_MESSAGEID_NUM	16

#define  HCP_PAYLOAD_LEN_0  0
#define  HCP_PAYLOAD_LEN_2  2
#define  HCP_PAYLOAD_LEN_4  4


#define  HCP_ACK_MESSAGEID  0X1
#define  HCP_NACK_MESSAGEID 0X2
#define  HCP_ACK_PAYLOADLEN  2
#define  HCP_NACK_PAYLOADLEN  2

#define  HCP_PROCESS_PARAMS_NUM 10

#define  HCP_CLASSID_INVALID	0xFF
#define  HCP_MSGID_INVALID		0xFF


//class id define
#define  HCP_NAV_CLASSID	0x01
#define  HCP_SV_CLASSID		0x02
#define  HCP_SYSM_CLASSID	0x03
#define  HCP_ACK_CLASSID	0x0A
#define  HCP_DBG_CLASSID	0x0E


//class NAV(0x01), message id define
#define  HCP_NAV_POS_MSGID			0x01
#define  HCP_NAV_VEL_MSGID			0x02
#define  HCP_NAV_TIME_UTC_MSGID		0x03
#define  HCP_NAV_TIME_BD_MSGID		0x04
#define  HCP_NAV_TIME_GPS_MSGID		0x05
#define  HCP_NAV_SYSM_MSGID			0x06


//class SV(0x02), message id define
#define  HCP_SV_GPS_EPH_MSGID	0x01

//class SYSM(0x03), message id define
#define  HCP_SYSM_RST_MSGID			0x01
#define  HCP_SYSM_UPARM_MSGID		0x02
#define  HCP_SYSM_VER				0x03


//class DBG(0x0E), message id define
#define  HCP_DBG_SYSM_MSGID		0x01
#define  HCP_DBG_POS_VEL_MSGID		0x02


//class ACK(0x0E), message id define
#define  HCP_ACK_ACK_MSGID		0x01
#define  HCP_ACK_NACK_MSGID		0x02




//****///
#define NAV_MODE_STATIONARY	1
#define NAV_MODE_WALKER		2
#define NAV_MODE_LSVEHICLE		3
#define NAV_MODE_HSVEHICLE	4
#define NAV_MODE_SEACRAFT		5
#define NAV_MODE_AIRCRAFT1G	6
#define NAV_MODE_AIRCRAFT2G	7
#define NAV_MODE_AIRCRAFT4G	8

#define MIN_SV_NUM_FIRST_FIX	3
#define MIN_NAV_SV_NUM			3	//support 3-sv fix
#define MAX_NAV_SV_NUM		16
#define MIN_CN0_FIRST_FIX		10
#define MIN_CN0_IN_FIX			10
#define MIN_CN0_IN_PPS			30
#define TRK_ELEVATION_MASK			0
#define NAV_ELEVATION_MASK		5
#define MIN_NAV_ELEVATION_MASK	1
#define MIN_PPS_NAV_ELEVATION_MASK	15

#define FIX_MODE_2D				1
#define FIX_MODE_3D				2
#define FIX_MODE_AUTO			3

#define STATIC_NAV_VEL_THRES	0		//unit: cm/s
#define STATIC_NAV_DIST_THRES	0		//unit: m

#define MC_IO_PROTOCOL_NMEA_EN			(1<<0)
#define MC_IO_PROTOCOL_MCB_EN			(1<<1)
#define MC_IO_PROTOCOL_USERDEF_EN		(1<<2)
#define MC_IO_PROTOCOL_RTCM_EN		(1<<3)
#define MC_IO_PROTOCOL_CMPPS_EN		(1<<4)
#define MC_IO_PROTOCOL_UBX_EN			(1<<5)
#define MC_IO_PROTOCOL_FIE_EN			(1<<6)


#define NMEA_FORMAT_SEPARATE           0
#define NMEA_FORMAT_COMBINE            1
#define NMEA_FORMAT_SEPARATE2          2
#define NMEA_FORMAT_BDS_EXT_NMEA       3
#define NMEA_FORMAT					   NMEA_FORMAT_SEPARATE2



typedef	union _CLASS_ID{
	U1  NAV_CLASS_ID;
	U1	SV_CLASS_ID;
	U1  SYSM_RST_CLASS_ID;
	U1  ACK_CLASS_ID;
	U1	DBG_SYSM_CLASS_ID;
	
	}CLASSID;

	
typedef	union _MESSAGE_ID{
	U1  NAV_MESSAGE_ID;
	U1  SV_MESSAGE_ID;
	U1  SYSM_RST_MESSAGE_ID;
	U1  ACK_MESSAGE_ID;
	U1 	DBG_SYSM_MESSAGE_ID;
	
	}MESSAGEID;

typedef struct _PROTOCOL_STATUS
{	
	I4 CheckProtocolHead;			//offset:0			
}PROTOCOL_STATUS;



typedef struct _NAV_POS_DATA
{	
	I4 Posx;			//offset:0		m	|ì?D?|ì?1¨??á??à¨o?|ì(ECEF)X????
	I4 Posy;			//offset:4		m	|ì?D?|ì?1¨??á??à¨o?|ì(ECEF)Y????
	I4 Posz;			//offset:8		m	|ì?D?|ì?1¨??á??à¨o?|ì(ECEF)z????
	I4 Longitude;		//offset:12	deg	¨????ì??¨?|ì??á??à¨o?|ì(WGS84)?-?¨¨	
	I4 Latitude;		//offset:16	deg	¨????ì??¨?|ì??á??à¨o?|ì(WGS84)???¨¨
	I4 Altitude;		//offset:20	m	¨????ì??¨?|ì??á??à¨o?|ì(WGS84)???¨¨
}NAV_POS;

typedef struct _NAV_VEL_DATA
{	
	I4 Velx;			//offset:0		m/s	|ì?D?|ì?1¨??á??à¨o?|ì(ECEF) X?¨￠?¨′?¨¨
	I4 Vely;			//offset:4		m/s |ì?D?|ì?1¨??á??à¨o?|ì(ECEF) Y?¨￠?¨′?¨¨
	I4 Velz;			//offset:8		m/s |ì?D?|ì?1¨??á??à¨o?|ì(ECEF) Z?¨￠?¨′?¨¨
	
}NAV_VEL;

typedef struct _NAV_UTC_TIME_DATA
{	
	U2 Year;			//offset:0		m/s	|ì?D?|ì?1¨??á??à¨o?|ì(ECEF) X?¨￠?¨′?¨¨
	U1 Mon;	
	U1 Day;	
	U1 Hour;	
	U1 Min;	
	U1 Sec;	
	U1 Reserve;	
	
}UTC_TIME_DATA;

typedef struct _NAV_SYSM_DATA
{	
	U4 TTFF;			//offset:0		m/s	|ì?D?|ì?1¨??á??à¨o?|ì(ECEF) X?¨￠?¨′?¨¨
	U1 BootMode;		//
	U1 FixMode;	
	U2 Reserve;	
	
}NAV_SYSM;


typedef struct _NAV_TIME_DATA
{
	U4 Wn	;		//offset:0	 	BD??éGPS?|ì¨a3¨o?à??Week number
	U4 Tow;			//offset:4 	BD??éGPS?|ì¨a3¨o?à???¨1?¨2??
}NAV_TIME;


typedef struct _SYSM_RST_DATA
{
	U1 Mode	;		 		//offset:0	  SYSTEM STARAT MODE
	U1 FlashOpFlag;			//offset:1	  CLEAR FLASH WHEN COLD START 
}SYSM_RST;


typedef struct _DBG_ACQ
{
	U1 acqState;		//0, idl; 1, busy.
	U1 trkchid;			//
	U1 trkchsvid;
	U1 state;
	U1 acqmode;
	U1 failcnt;
	I2 freq;

	U1 spancnt;
	U1 spancntth;
	U2 res1;
	U2 res2;
	U1 res3;
	U1 validtrkcnt;
}DBG_SYSM_ACQ;

typedef struct _DBG_POS_VEL
{
	I4 Posx;			//offset:0		m     *10
	I4 Posy;			//offset:0		m	*10
	I4 Posz;			//offset:0		m	*10
	I4 Velx;			//offset:0		m/s		*100
	I4 Vely;			//offset:0		m/s		*100
	I4 Velz;			//offset:0		m/s		*100
}DBG_POS_VEL;


typedef struct _DBG_TRK
{
	I1 trkid;
	I1 svid;
	I1 fredot;
	I1 state;
	I1 acqmod;
	I1 acqfailcnt;
	I1 trkmode;
	U1 CCBF;
	
	I1 C2Fai;
	U1 cn1s;
	U2 buse;
	U1 locktime;
	U1 sferr;
	I1 selpath;
	U1 el;
	
	U2 az;
	I2 ephage;
	I2 almage;
	I1 health;
	U1 sync;
	
	I2 wn;
	I2 swfre;	// Hz
	I4 hwfre;	//0.01 Hz	
	
	I4 sfid;
	I4 tsdif;	//m
	
	U4 intpInv;	
	U2 decp;
	U2 bitcnt;
	
	I2 Ncnt;
	U2 sfflag;	//sf buffer valid bitmap
	U2 wdflag;	//word parity bitmap
	U1 ephcnt;
	I1 N;	// 整周模糊度
	
	I1 ionosrc;	
	I1 Iono;	//m
	I1 Tropo;	//m
	I1 reserr_fd;	//m/s
	I2 reserr_pr;	//m
	I1 LeapsGlo;	//for glonass, leap sec
	I1 truesvid;	//for glonass, ture svid

	U4 intp;
	I4 clkerr;	//0.1m
	
	I4 posx;	//0.1m
	I4 posy;	//0.1m
	
	I4 posz;	//0.1m
	I4 velx;	//0.01m/s
	
	I4 vely;	//0.01m/s
	I4 velz;	//0.01m/s

	I2 dclkerr;	//0.0001m
	I2 dposx;	//0.0001m
	I2 dposy;	//0.0001m
	I2 dposz;	//0.0001m
	
	R8 ts;		//s	
	R8 pr;		//m
	R8 phase;	//cycle
}DBG_SYSM_TRK;


typedef struct _DBG_MISC
{
	U4 msgid;
	CX UTCTime[14];
	U1 inviewNUseSVFdot;
	U1 gpsinview;
	U1 bdinview;	
	U1 gloinview;
	U1 gpstrked;	
	U1 bdtrked;

	U1 glotrked;
	U1 idlchcnt;
	U1 gpssvused;
	U1 bdsvused;
	U1 glosvused;
	U1 acqstartcnt;
	U1 acqendcnt;
	U1 fixres;
	
	U1 filtertype;
	I1 lsckres;
	I2 tcxo;
	I2 TTFF;
	I1 sysEnv;
	I1 bootmode;	
	
	I4 posx;	//0.1m, single position
	I4 posy;	//0.1m, single position
	
	I4 posz;	//0.1m, single position
	I2 alt;		//0.1m, single position
	I2 velx;	//0.1m/s
	
	I2 vely;	//0.1m/s
	I2 velz;	//0.1m/s
	U2 gdop;	//0.01
	U2 hdop;	//0.01
	
	U2 vdop;	//0.01
	U2 tdop_gps;//0.01
	U2 tdop_bd;	//0.01
	U2 tdop_glo;//0.01

	U4 testPosErr;	//0.0001m
	U2 testVelErr;	//0.01m/s
	I2 ctu;		//m/s	
	
	U1 buflag;
	U1 TFTCnt;
	U1 Obscnt;
	U1 res1;
	I1 nFixFlag;
	U1 dRatio;
	U1 keySvid_gps;
	U1 keySvid_bd;
	
	U1 keySvid_glo;
	U1 commSvcnt;
	U1 RTKErrPath;
	I1 RTKExtTimegps;
	U2 att_yaw;		//0.01degree
	I2 att_pitch;	//0.01degree

	U4 baseline;	//0.0001m
	I4 baseline_E;	//0.0001m
	
	I4 baseline_N;	//0.0001m
	I4 baseline_U;	//0.0001m
			
	R8 RTKposx;		//m
	R8 RTKposy;		//m
	R8 RTKposz;		//m
	
	U1 lostbyPrErr;
	U1 lostbyoutview;
	U1 lostbyCrossErr;
	U1 lostbySnr;
	U1 lostbyDifFre;
	U1 lostbyDupliSV;	
	U1 lostbySyncChk;
	U1 lostbyLongTimeSync;
	
	U1 lostByDummySV;
	U1 lostByIllSV;
	U1 lostByCaif;
	U1 TFTModifyCnt;
	U1 acqMismatchCnt;
	U1 flashOpCnt;
	U1 syncErrcnt;
	U1 syncErrch;

	I4 hisposx;	//0.1m
	I4 hisposy;	//0.1m
	
	I4 hisposz;	//0.1m
	I1 hisposflag;
	U1 ECAFlag;
	I2 longTimeAlt;
	
	I2 clkerr_gps;	//0.1ns
	I2 clkerr_bd;	//0.1ns
	I2 clkerr_glo;	//0.1ns
	I1 RTKExtTimebd;
	I1 RTKExtTimeglo;
	
	U4 dummy_gps;	
	U4 dummy_bd;

	U2 synctimesrc;		//sync time src
	I2 SyncGPS_wn;
	I2 SyncBD_wn;
	I2 SyncGlo_wn;

	R8 SyncGPS_tow;
	R8 SyncGPS_rfcnt;
	R8 SyncBD_tow;
	R8 SyncBD_rfcnt;
	R8 SyncGlo_tow;
	R8 SyncGlo_rfcnt;
	
	R8 Lock_RFcnt;
}DBG_SYSM_MISC;


typedef struct _PROTOCOL_HCP
{	
	U1			ClassId;		//ààDí?üá?
	U1			MessageId;		//D-òé?üá?
	U2          DataLenth;
	CX*			HCPData;			//êy?Y?úèY
}PROTOCOL_HCP;



typedef struct _PROTOCOL_HCP_TASK_PARAMS
{
	U1			ClassId;		//ààDí?üá?
	U1			MessageId;		//D-òé?üá?	
	void		(*ProtocolHCPProcess)(word16 datalen, byte* pData, byte comid);			//process func
}HCP_TASK_PARAMS;

#ifdef __cplusplus
extern "C" {
#endif

void 	TaskProtocolInput(STRU_UART_STREAM *glStruUart);
int	    CheckHCPPayloadLen(char *,unsigned short*,unsigned short ,unsigned short ,STRU_UART_STREAM * );
void	CheckRecvTaskProtocolUartBuffer(void);
void    StoreHCPDate(char *);
int     HCPParityCheck(unsigned short,char *);
void    RecvProtoHCPHandleProcess(STRU_UART_STREAM *glStruUart);
int		CheckRecvTaskProtocolHead(int *validlen,int *protocoltype,STRU_UART_STREAM *glStruUart, int32* pMsgid);
int		ReadBufferShiftProcess(unsigned short* , int);

void    HCPProtocolNACK(byte classid, byte msgid,byte comid);
void    HCPProtocolACK(byte classid, byte msgid,byte comid);
void 	HCPSendData(byte classid, byte msgid, word16 payloadlen, byte* pPayload,byte comid);
void 	HCPDataCheckConfig(char *,unsigned short ,unsigned char *,unsigned char*  );
int     NMEARcvProc(int *,STRU_UART_STREAM * );

void 	TaskPeriodicOutput(void);

bool UartSendId(UINT16 comid ,unsigned char ProType);

#ifdef __cplusplus
}
#endif

#endif

