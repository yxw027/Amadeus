/*
 * rtk.h
 *
 *  Created on: 2015-10-21
 *      Author: dell
 */

#ifndef RTK_H_
#define RTK_H_

#include "define.h"
#include "typedefine.h"
#include "coordinate.h"
#include "RTKLAMBDA.h"
#include "RTKConstants.h"
#include "FrameDecode.h"

#define ENABLE_RTD_KF	0


#define MAX_SNR	(68)

//#define nfreq  5
#define MAX_RTK_OBS_SVCNT	36	//12 * 3system
#define	RTK_MIN_TIMEDIFF	(0.02)	//�������ʱ��ͬ����С���
#define RTK_SINGLE_MIN_OBSCNT	3
#define RTK_MIN_OBSCNT		5

//define for postproc SW
#define MASTER_SITE_MAXNUM	4 //�ƶ�վ�������
#define SLAVE_BUF_LEN	7

#define L2P_LOWER_CN0_TH	4

//�ز����ģʽ
enum
{
	//DGPS_MODE_MBATT = 0,	// ����վ(moving base)����ģʽ
	//DGPS_MODE_RTK = 1		// ����վRTKģʽ
	DGPS_MODE_STATC = 0,
	DGPS_MODE_KINEM = 1,
	DGPS_MODE_MOVBS = 2
	
};
//#define DGPS_MODE 	(DGPS_MODE_STATC)
//#define DGPS_MODE   (DGPS_MODE_MOVBS)

#define MBGF_GETRATIO 10.0
#define MBGF_SUSRATIO 3.0
#define MBGF_USERATIO 2.0
#define MBGF_SUSTIME  60.0
#define MBGF_USETIME_LIMIT  200.0
#define MBGF_USESATE_LIMIT  5
#define MBGF_GETNUM_LIMIT  36

enum
{
	MBGF_NU_ALLRIGHT = 0,
	MBGF_NU_KEYCHANGE = 1,
	MBGF_NU_SATELESS = 2,
	MBGF_NU_SATECHANGE = 3,
	MBGF_NU_TIMEOUT = 4
};

//�۲�������
enum
{
	RCV_BASE = 0,
	RCV_ROVE = 1
};

//˫��۲�������
// nObsType = 	1��L1α��˫��    
//				2��L2α��˫��    
//				3��L1�ز�˫��    
//				4��L2�ز�˫��    
//				5:    L1L2˫Ƶ�ز�˫��
//          		6��B1α��˫��    
//				7��B2α��˫��    
//				8��B1�ز�˫��    
//				9��B2�ز�˫��    
//				10:   B1B2˫Ƶ�ز�˫��
enum
{
	DDOBSTYPE_L1RANGE = 1,
	DDOBSTYPE_L2RANGE = 2,
	DDOBSTYPE_L1PHASE = 3,
	DDOBSTYPE_L2PHASE = 4,
	DDOBSTYPE_L1L2PHASE = 5,
	DDOBSTYPE_B1RANGE = 6,
	DDOBSTYPE_B2RANGE = 7,
	DDOBSTYPE_B1PHASE = 8,
	DDOBSTYPE_B2PHASE = 9,
	DDOBSTYPE_B1B2PHASE = 10	
};

// BD2/GPS
enum BD2GpsSystem
{
	BD2_SYSTEM  = 1,
	GPS_SYSTEM  = 2,
	BD2_GPS     = 3,
	GLN_SYSTEM  = 4,
	BD2_GLN	    = 5,
	GPS_GLN     = 6,
	BD2_GPS_GLN = 7
};

// �ο������������
enum 
{
	BDREF_PARAM_a,		// �ο����򳤰���
	BDREF_PARAM_f,		// �ο�������ʵ���
	BDREF_PARAM_w,		// �ο�������ת���ٶ�
	BDREF_PARAM_GM		// ������������
};


//˫������������
enum
{
	DDRESULT_SAT_ID = 1,	//�����Ǻţ����鳤��Ϊ32
	DDRESULT_SAT_INDEX = 2	//���±�ţ����鳤��Ϊ����������
		
};

//���߼���������
enum
{
	BLY_CHK_RESIDUAL	 = -5,	//�в����
	BLY_CHK_PRIMSAT		 = -4,	//��������У��ʧ��
	BLN_NOAMB_PRDD       = -3, 	//�޻��ߣ�δ�����������α��˫���
	BLY_CHK_FLOAT_FAILED = -2,	//�л��ߣ�����ʧ�ܣ������
	BLN_CHK_FLOAT        = -1, 	//�޻��ߣ������
	BLN_CHK_FLOAT_ERR    = 0,	//�޻��ߣ��������ϴ�
	BLN_CHK_FIXED     	 = 1,	//�޻��ߣ��̶���
	BLY_CHK_FIXED		 = 2,	//�л��ߣ��̶���
	BLN_SEARCH_FIXED	 = 3,	//�޻��ߣ������ɹ��û���
	BLN_WIDELANE_FIXED	 = 4	//����̶���
};

//�۲���Ȩ�ط�ʽ
enum
{
	OBSVDD_WEIGHT_UNIT = 0,	//��λȨ�أ�������һ�������ֵ
	OBSVDD_WEIGHT_SATEL,	//���Ǹ߶ȽǶ�Ȩ
	OBSVDD_WEIGHT_CN0,		//�������رȶ�Ȩ
	OBSVDD_WEIGHT_EXP,		//ָ��ģ��
	OBSVDD_WEIGHT_EL_SNR	//���ϼ�Ȩ���ο�����09_erenoglu_285-301.pdf
};

// ���У�鷽ʽ
enum
{
	RTK_CHECK_BSL		= 0x01,	//	���߳���
	RTK_CHECK_RATIO		= 0x02,	//	���Ƿ���������֤
	RTK_CHECK_PITCH		= 0x04,	//	����ģʽ��������֤
	RTK_CHECK_RESIDUAL	= 0x08	//	�в����
};
enum
{
	RTK_C_LOST		= 0x01,	//	������ݶ�ʧ
	RTK_C_RATIO		= 0x02,	//	ratioֵ��С
	RTK_C_SLIP		= 0x04,	//	��������
	RTK_C_RABAD		= 0x08	//	ratio�����仵
};

#define OBST_VALID_LLI		0x1
#define OBST_VALID_SNR		(0x1<<1)
#define OBST_VALID_CODE		(0x1<<2)
#define OBST_VALID_PHASE	(0x1<<3)
#define OBST_VALID_RANGE	(0x1<<4)
#define OBST_VALID_DOPPLER	(0x1<<5)
#define OBST_VALID_HALFCYCLE	(0x1<<6)

typedef struct {        // observation data record
	byte StarID; // satellite/receiver number, gps:1~32, BD:33~64, glonass: 65~88
	byte slotID;	//used only for glonass. 65~78, define as RTCM3.2, 65~0:G1 1598.0626 G2 1242.9375
	int8 el;
	int16 az;
	word32 toe;
	double clkerr;	//s
	double distance;	//m
    ECEF svpos;
	byte bUsedMap[MAX_FREPIONT_PER_NAVSYS];	//bit0: used at signal position, bit1: used at RTK
    byte validmap[MAX_FREPIONT_PER_NAVSYS];//bitmap: 1, valid; 0, not valid. OBST_VALID_LLI,OBST_VALID_SNR.......
    byte halfcycle[MAX_FREPIONT_PER_NAVSYS];		//half cycle indicate. 0, no; 1, half cycle
    word32 LLI [MAX_FREPIONT_PER_NAVSYS]; // locked time, ms
    float snr0 [MAX_FREPIONT_PER_NAVSYS]; // signal strength (0.25 dBHz)
    word16 code[MAX_FREPIONT_PER_NAVSYS]; // freq point and code define
    double phase[MAX_FREPIONT_PER_NAVSYS]; // observation data carrier-phase (cycle)
    double range[MAX_FREPIONT_PER_NAVSYS]; // observation data pseudorange (m)α��
    float  doppler[MAX_FREPIONT_PER_NAVSYS]; // observation data doppler frequency (Hz) 
} OBST;
								
#define OBSEV_VALID_STATION		(0x1<<0)
#define OBSEV_VALID_GPS_DATA	(0x1<<1)	
#define OBSEV_VALID_BD_DATA		(0x1<<2)	
#define OBSEV_VALID_GLO_DATA	(0x1<<3)	
		
typedef struct {        // RTCM control struct type
	word16 bitmap;		//data valid map. 
	int16 satnmax;         // number of obervation data/allocated
    int16 staid;          // station id,N
    //int8 stah;           // station health,N
    //int8 seqno;          // sequence number for rtcm 2 or iods msm, N
    //int8 outtype;        // output message type, N
    int8 bSinglePosValid;	//fixres of singleRcvrPos
    byte clkSteer_gps;		//0, 300km; 1, 300m; 2, unkown; 
    byte clkSteer_bd;		//0, 300km; 1, 300m; 2, unkown; 
    byte clkSteer_glo;		//0, 300km; 1, 300m; 2, unkown; 
    word32 signmax_gps;			//sig mask bitmap, define as RTCM3
    word32 signmax_bd;			//sig mask bitmap, define as RTCM3
    word32 signmax_glo;			//sig mask bitmap, define as RTCM3
	word32 wn;
    double sec_gps;         // tow,s
    double sec_bd;         // tow,s 
    double sec_glo;         // tow,s
	ECEF SingleRcvrPos;		// ���㶨λ���
    OBST obs[MAX_RTK_OBS_SVCNT];          // observation data (uncorrected)
} OBSEV;

typedef struct{
	int8 readidx;
	int8 writeidx;
	OBSEV obsev[SLAVE_BUF_LEN];
}OBSEV_BUF;



// ��λ����ͨ����Ӧ�����Ǻš�Ƶ�㡢֧·��Ϣ
typedef struct tag_RTK_COMMONSAT_INFO
{
	CHAR	SatID;		// ���Ǻ�
	CHAR	Freq;		// Ƶ��
	CHAR	Branch;		// ֧·
	CHAR	UpDown; 	// ��������:0����1����-1�½�
	BOOL	bKeyStar;	//�Ƿ��ǻ�׼��
	DOUBLE 	Amb;		// ����ģ����
} RTK_COMMONSAT_INFO;

// �۲�ʱ��ṹ����
typedef struct tag_BD2_OBSTIME
{
	INT16  year;	// ������
	INT16  month;
	INT16  day;
	INT16  weekday;
	INT16  hour;	// ʱ����
	INT16  min;
	DOUBLE sec;
		
	DOUBLE SecOfDay ;   // ����
	INT32 DayOfYear;	// �����

	UINT   nWeeks;		// ������
	DOUBLE dSeconds;	// ����
	BYTE   precistype;  //��������
} BD2_OBSTIME;


// ������վ������ϵ�еķ�λ�Ǻ͸߶Ƚǽṹ����
typedef struct tag_BD2_SAT_AZEL
{
	DOUBLE azimuth;		// ��λ��
	DOUBLE elevation;	// �߶Ƚ�
	DOUBLE elevation2;	// �߶Ƚ�
	DOUBLE snr;         //SNR
} BD2_SAT_AZEL;


// ����λ�ü������ṹ
typedef struct tag_BD2_SAT_POS
{
	DOUBLE x;	// �ռ�ʱ������
	DOUBLE y;
	DOUBLE z;
	DOUBLE tsv;	// ����ʱ��ƫ���λ����

	DOUBLE	dx, dy, dz, dtsv;	// �ٶȷ�����ʱ�����
	DOUBLE ddx,ddy,ddz;	// ���Ǽ��ٶ�

	CHAR	SatID;		// ���Ǳ��
	CHAR	Freq;		// Ƶ��
	CHAR	Branch;	    // ͨ��	
} BD2_SAT_POS;


// ��λģʽ
typedef struct tag_BD2_POS_MODE
{
	// ģʽ����
	BOOL	System;			// I·��BD2��λ�� Q·��GPS��λ
	BOOL	UseWaas;		// �Ƿ�ʹ�ò��
	BOOL    Use2D;          // �Ƿ�2D��λ
	word32	FreqUsed;		// ����Ƶ�������1��2��3,4
	word32	Freq[MAX_FREPIONT_PER_NAVSYS];	// ��ʹ�õ�Ƶ��
	CHAR    PCode;          // ʹ��P�붨λ

	CHAR	cPosType;			// ����ģʽ
	CHAR	cWeightType;		// ��Ȩģʽ
	UINT32  RnssMode;
} BD2_POS_MODE;


// ���������ṹ��
typedef struct tag_BD2_BASELINE
{	
	CHAR 	nFixed;			//�Ƿ�̶�

	double	East;			//��
	double	North;			//��
	double	Up;				//��
	
	double	B;			//γ
	double	L;			//��
	double	H;			//��

	ECEF	dxyz;
	
	double	BaselineLongth;	//���߳���
	double 	Gauss_dx;
	double 	Gauss_dy;
	double  dVtV;	
} BD2_BASELINE;

// ��̬�����ṹ��
typedef struct tag_BD2_Attitude
{
	double 	yaw;	//�����
	double	pitch;	//������
	double	roll;	//������
	double 	yaw_deg;	//�����
	double	pitch_deg;	//������
	double	roll_deg;	//������

	// �ٶȷ���
	double dSpeedCourse;
	BOOL  bSpeedCourseValid;
	// ���걱
	double	headingCoord;
	double	headingCoord_deg;
}BD2_Attitude;

typedef struct TGaussCoord
{
	DOUBLE X;
	DOUBLE Y;
	DOUBLE H;
} GAUSS_COORD;


// �û�����λ�þ�γ�ȸ߳̽ṹ����
typedef struct tag_BD2_USER_BLH
{
	DOUBLE latitude;	// γ��
	DOUBLE longitude;	// ����
	DOUBLE altitude;	// �߳�
} BD2_USER_BLH;

// �������
typedef struct TEarthCoord
{
		DOUBLE dLongitude;	// ����(��)
		DOUBLE dLatitude;	// γ��(��)
		DOUBLE dAltitude;	// �߳�(��)
		DOUBLE dAltDelta;	// �쳣(��)
} EARTH_COORD;


// ��λ����ͨ����Ӧ�����Ǻš�Ƶ�㡢֧·��Ϣ
typedef struct tag_BD2_SAT_INFO
{
	CHAR	SatID;	// ���Ǻ�
	int8	cn0;
	UINT8	LLI;	//s
	word32	validFrqpint;	//valid frqpoint map
} BD2_SAT_INFO;

// ��λ����ͨ����Ӧ�����Ǻš�Ƶ�㡢֧·��Ϣ
typedef struct tag_RTK_COMMONSAT
{
	// ��׼��
	CHAR nKeySat[MAX_SYSM];
	//CHAR nKeySatIndex;
	// ��������
	CHAR nCommSat;
	BD2_SAT_INFO CommonSat[LAMBDA_N_DIM];		// ��������
	// ˫������Ǽ�ģ���ȣ��޻�׼�ǣ�
	CHAR nCommSat_AMB;
	BD2_SAT_INFO CommonSat_AMB[LAMBDA_N_DIM];		// ��������
} RTK_COMMONSAT;


// ��ֽ�������Ҫ������һ��Ԫ������������
typedef struct tag_RTK_EPOCH_Keep
{	
	double tow;	//last RTK time
	
	bool bFirstCalAmb;	//be first calc float N filter�����
	bool bLastIntValid;	// is last int fix valid
	bool bSuperWide;	// is super wide lane used?
	int8 RTKErrPath;	//debug info for Error path
	
	// ���������Ϣ
	CHAR nKeySatID[MAX_SYSM];		//��һ��Ԫ��׼���Ǻ�
	CHAR nCommonSat; 	//�������Ǹ���	
	BD2_SAT_INFO CommonSat[MAX_RTK_OBS_SVCNT];	//����������Ϣ

	CHAR nCommonSatAMB;
	BD2_SAT_INFO CommonSat_Amb[LAMBDA_N_DIM];//����������Ϣ��������׼��
	
	int32 dAMB_NS_Fix[SV_NUM_TRUE];	//��Ƶ����ģ����
	word256 bAMB_NS_Fix_Valid;

	int32 dAMB_NW_Fix[SV_NUM_TRUE];	//��������ģ����
	word256 bAMB_NW_Fix_Valid;

	double dAMB_NWf[SV_NUM_TRUE];	//��������ģ���ȸ���ֵ
	word256 bAMB_NWf_Valid;

	CHAR nCommonSat2; 	//�������Ǹ���	
	BD2_SAT_INFO CommonSat2[MAX_RTK_OBS_SVCNT]; // ������Ԫ�Ĺ���������Ϣ����������Ԫ��۲����仯̽������
	
	// �۲��������Ϣ	
	OBSEV  prmOldBase;
	OBSEV  prmOldRove;
	int begin;
	double  phaseDSKeep[LAMBDA_N_DIM];
	double  phaseDSKeep2[LAMBDA_N_DIM];

	// ����Ŀ���AMB���������Ǹ�������һ�����ϸ���Ԫ����Ϣ
	BOOL bGetAmb;
	RTK_COMMONSAT_INFO rtk_satinfo[SV_NUM_TRUE];

	// ��ֽ�������
	double	Rtk_NKeep[LAMBDA_N_DIM];
	double	Rtk_PKeep[LAMBDA_N_DIM*LAMBDA_N_DIM];

	//����
	CHAR	slip[SV_NUM_TRUE];

	//RTK flag
	int8 nAmbIndex;		//single frequency
	int8 nAmbIndex_wide;	//wide  frequency
	int8 nAmbIndex_wfloat;	//RTK wide float res

	// ��Ƶģ����ƽ��
	double	dSF_AMB_Smooth[SV_NUM_TRUE];	// �˲��������ģ����
	double	dSF_AMB_Pk[SV_NUM_TRUE];		// �˲���Э�������
	bool	nAMB_Smooth_Flag[SV_NUM_TRUE];	// �˲�ƽ����־

	//sv used
	word256 L1UsedSVMap;

#if ENABLE_RTD_KF
	RTD_KF RTDKFPara;
#endif
}RTK_EPOCH_Keep;

typedef struct tag_BD2_PVT
{
	DOUBLE BD2_Time;		// �۲�ʱ��
	DOUBLE x,y,z,tb,tg,tgln,tf;		// ��λ������������ջ�����ͽ��ջ�ʱ��ƫ��
	DOUBLE dx,dy,dz,dtb;
	BD2_USER_BLH blh;	// ��γ�Ⱥ͸߳�

	DOUBLE kax,kay,kaz;
	BD2_USER_BLH blhka;	
	
	double dVe, dVu, dVn, dVGround;
	DOUBLE  HDOP,		// ˮƽ��������
		VDOP,		// ��ֱ
		TDOP,		// ʱ��
		PDOP,		// ��λ��������
		GDOP;		// ���ξ�������

	DOUBLE error;       // ��λ������������
	INT16 valid;		// �������ֵ�Ƿ���Ч
	INT16 nUsedSat;		// ��λ���õ������Ǹ���
} BD2_PVT;


// �������
typedef struct tag_BD2_RTK_OUTPUT
{	
	BD2_OBSTIME		time;
	BD2_POS_MODE 	PosMode_Rtk;	// ��λģʽ
	BD2_PVT			RovePos;	// RNSS��λ�����Ϣ
	GAUSS_COORD     BaseCoord_Gauss;

	int8 	nAmbIndex;		//RTK single fix res
	int8 	nAmbIndex_wide;		//RTK wide fix res
	int8 	nAmbIndex_wfloat;		//RTK wide float res
	double 	Ratio;			
	double 	Rdop;			//��Զ�λ����

	BD2_BASELINE	baseline;	// ��������
	BD2_Attitude	attitue;	// ��̬����

	char 	nKeySatID;
	char 	nCommSat;
	word256 nRtkUseID;	
	double	dAMB[LAMBDA_N_DIM];

	unsigned short RtkCheck;
} RTK_OUTPUT;


// ��Ԫ����
typedef struct tag_RTK_EPOCH
{
	bool bSuperWide;	// is super wide lane used?
	
	// ��ֹ۲���
	OBSEV Prm_Base;
	OBSEV Prm_Rove;
	// ��������
	CHAR				nCommonSat;
	BD2_SAT_INFO		CommonSat[MAX_RTK_OBS_SVCNT];		// ��������
	CHAR				nCommonSatAMB;
	BD2_SAT_INFO		CommonSat_Amb[LAMBDA_N_DIM];		// ��������
	
	// ��׼��
	int8				KeySatIdIndex[MAX_SYSM];
	int8				KeySatId[MAX_SYSM];

	// ���ù���������Ϣ
	RTK_COMMONSAT UsableCommSat;	

	//SV to base station distance
	double DisSV2Base[MAX_RTK_OBS_SVCNT];

	// ����
	DOUBLE  Phase_DDD[SV_NUM_TRUE];
	
	// ��ֽ��
	ECEF RovePVT_RTD;	//rove position, RTD
	ECEF RovePVT_FLOAT;	//rove position, NWf
	ECEF RovePVT_Fix;	//rove position, Int
	ECEF Pos_aver;
	ECEF Pos_std;

	DOUBLE		ApaInv[LAMBDA_N_DIM*3];			// ��ƾ���inv(AD*P*AD)
	DOUBLE		AD[LAMBDA_N_DIM*3];			// A���󵥲�
	DOUBLE		ADD[LAMBDA_N_DIM*3];			// A���󵥲�
	
	// ��Ƶ����ģ����
	double dAMB_Fix[LAMBDA_N_DIM*LAMBDA_N_CANDS];
	double dAMB_Float[LAMBDA_N_DIM];
	double dAMB_PrDD[LAMBDA_N_DIM];
	
	// ˫Ƶ����ģ����
#if SUPPORT_SUPER_WIDE_LANE
	double dAMB_SNW_Fix[LAMBDA_N_DIM*LAMBDA_N_CANDS];
#endif
	double dAMB_NW_Fix[LAMBDA_N_DIM*LAMBDA_N_CANDS];

	BOOL bSatChanged;
	BOOL bKeySatChanged;
	CHAR KeySatID_NewOld[MAX_SYSM][2];	// 0:new  1:old

    double    Range_DD[LAMBDA_N_DIM];
	
#if SUPPORT_SUPER_WIDE_LANE
	double    Phase_DD_SNW[LAMBDA_N_DIM];	//super wide lane
#endif
	double    Phase_DD_NW[LAMBDA_N_DIM];	//wide lane
	double	  Phase_DD_NS[LAMBDA_N_DIM];
	
#if SUPPORT_SUPER_WIDE_LANE
	double	  wavelen_SNW[LAMBDA_N_DIM];	//super wide lane
#endif
	double	  wavelen_NW[LAMBDA_N_DIM];		//wide lane
	double	  wavelen_NS[LAMBDA_N_DIM];

	int nSingleSearch;
	int nMultiSearch;

	UINT16 cChkMode;	// �����У�鷽ʽ
	
	double 	RangeCovariance[LAMBDA_N_DIM*LAMBDA_N_DIM];
	double 	PhaseCovariance[LAMBDA_N_DIM*LAMBDA_N_DIM];
	double 	RangeCovarianceInv[LAMBDA_N_DIM*LAMBDA_N_DIM];

	CHAR CommSat_AMB_SwitchErr_List[LAMBDA_N_DIM];

	int8 nAmbIndex;
	int8 nAmbIndex_wide;		//RTK wide fix res
	int8 nAmbIndex_wfloat;		//RTK wide float res
	double dRatio;	//0:����Ԫֵ	1:����Ԫ�����Ԫ���ֵ
	BD2_BASELINE	Baseline;	//��������	
	BD2_Attitude	Attitude;	//��̬����

	double dInv_HTH_HT[LAMBDA_N_DIM*3];	//����inv(H'H)*H*lambda��ֻ�ü���Phi-N���ɡ�
	double 	PhaseCovarianceInv[LAMBDA_N_DIM*LAMBDA_N_DIM];

	BOOL bAMB_SK;	// ����ģ���ȵ������뱣�ֹ��ܱ�־

	word32 SelectFrepMapSW;	//super wide lane
	word32 SelectFrepMap;	//wide lane
	word32 SelectFrepMapL1;	//˫Ƶ�е�ƵѡƵ���

	word256	RTKChnMap;	//Used to RTK channel map
	word256 L1UsedSVMap;
#ifdef _SIMULATE
	double PhaRes[SV_NUM_TRUE];
#endif
} RTK_EPOCH;

typedef struct{
	bool bValid;
	UINT32 PhiTrue_inv;
	double PhiTrue;
	double PhiUsed;
	double tr_withBias;
}LAST_PHI_INFO;

#if PHi_RANGE
typedef struct{
	int32 icnt;
	double lastrange;
	double lastphase;
}LAST_RANGE_INFO;
#endif

typedef struct{
	bool bValid;
	UINT16	stationID;
	float64	antHeight;	//antenna height, unit: m
	ECEF ecefpos;
}ANTPositionStruct;

// RTK�Ĺ۲�������
typedef struct tag_RTK_RECKON_OBS
{
	bool bValid;
	double pr0;
	double ph0;
} RTK_RECKON_OBS;

typedef struct
{
	bool bValid;
	byte LLI;	//s
	byte cn0;	//dBHz
	word32 code;	//freqpoint
	double apr_b;
	double aph_b;
}RTK_RECKON_PARAM;


typedef struct tag_RTK_RECKON_INFO
{
	float64 tow_gps;
	float64 tow_bd;
	float64 tow_glo;
	bool bValidtow_gps;
	bool bValidtow_bd;
	bool bValidtow_glo;
	RTK_RECKON_PARAM reckParam[SV_NUM_TRUE][MAX_FREPIONT_PER_NAVSYS];
} RTK_RECKON_INFO;


#ifdef __cplusplus
extern "C" {
#endif


extern OBSEV_BUF Slave_Buf;
#ifdef _POSTPROC
extern OBSEV_BUF Master_Buf[MASTER_SITE_MAXNUM+1];
#endif
extern OBSEV Master_observ;
extern ANTPositionStruct antfixposition;
extern ANTPositionStruct Basefixposition;
extern int16 gRTKErrPath;
extern OBSEV Master_obs;
extern OBSEV Slave_obs;

extern RTK_EPOCH RtkEpoch;
extern RTK_EPOCH_Keep RtkEpochKeep;

extern void RtkInit(void);
extern void RestRTKchInfo(int32 trkch);
extern void TaskRTK(void);
extern void SetBaseStationInfoForRove(UINT16 stationID, float64 antHeight, ECEF* pPos);
extern bool GetBaseStationInfoForBase(UINT16* pStationID, float64* pAntHeight, ECEF* pPos);
extern OBSEV* GetCurSlaveBufAddr(byte navsys, double tow);

#ifdef _POSTPROC
extern OBSEV* GetCurMasterBufAddr(byte navsys, double tow, int masterNumber);

extern void CalcBaseStationSVPos(OBSEV* PrmSlave);
#endif

extern bool GetObsvTime(OBSEV* pObsv, double* pobstow, UINT8 navmod);
extern bool IsMoveBaseForRove(void);
extern double GetRTKExtraTime(UINT8 navmod);

#if PHi_RANGE
extern LAST_RANGE_INFO Last_range_info[SV_NUM_TRUE][MAX_FREPIONT_PER_NAVSYS];
#endif

#ifdef __cplusplus
}
#endif


#endif /* RTK_H_ */
