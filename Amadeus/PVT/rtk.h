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
#define	RTK_MIN_TIMEDIFF	(0.02)	//差分数据时间同步最小误差
#define RTK_SINGLE_MIN_OBSCNT	3
#define RTK_MIN_OBSCNT		5

//define for postproc SW
#define MASTER_SITE_MAXNUM	4 //移动站最大数量
#define SLAVE_BUF_LEN	7

#define L2P_LOWER_CN0_TH	4

//载波差分模式
enum
{
	//DGPS_MODE_MBATT = 0,	// 动基站(moving base)定向模式
	//DGPS_MODE_RTK = 1		// 静基站RTK模式
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

//观测量类型
enum
{
	RCV_BASE = 0,
	RCV_ROVE = 1
};

//双差观测量类型
// nObsType = 	1：L1伪距双差    
//				2：L2伪距双差    
//				3：L1载波双差    
//				4：L2载波双差    
//				5:    L1L2双频载波双差
//          		6：B1伪距双差    
//				7：B2伪距双差    
//				8：B1载波双差    
//				9：B2载波双差    
//				10:   B1B2双频载波双差
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

// 参考椭球参数类型
enum 
{
	BDREF_PARAM_a,		// 参考椭球长半轴
	BDREF_PARAM_f,		// 参考椭球扁率倒数
	BDREF_PARAM_w,		// 参考椭球自转角速度
	BDREF_PARAM_GM		// 地球引力常数
};


//双差结果排列类型
enum
{
	DDRESULT_SAT_ID = 1,	//按卫星号，数组长度为32
	DDRESULT_SAT_INDEX = 2	//按下标号，数组长度为公共卫星数
		
};

//基线检验结果类型
enum
{
	BLY_CHK_RESIDUAL	 = -5,	//残差过大
	BLY_CHK_PRIMSAT		 = -4,	//主组卫星校验失败
	BLN_NOAMB_PRDD       = -3, 	//无基线，未搜索到结果，伪距双差解
	BLY_CHK_FLOAT_FAILED = -2,	//有基线，检验失败，浮点解
	BLN_CHK_FLOAT        = -1, 	//无基线，浮点解
	BLN_CHK_FLOAT_ERR    = 0,	//无基线，基线误差较大
	BLN_CHK_FIXED     	 = 1,	//无基线，固定解
	BLY_CHK_FIXED		 = 2,	//有基线，固定解
	BLN_SEARCH_FIXED	 = 3,	//无基线，搜索成功得基线
	BLN_WIDELANE_FIXED	 = 4	//宽巷固定解
};

//观测量权重方式
enum
{
	OBSVDD_WEIGHT_UNIT = 0,	//单位权重，各卫星一样的误差值
	OBSVDD_WEIGHT_SATEL,	//卫星高度角定权
	OBSVDD_WEIGHT_CN0,		//卫星信载比定权
	OBSVDD_WEIGHT_EXP,		//指数模型
	OBSVDD_WEIGHT_EL_SNR	//复合加权，参考文献09_erenoglu_285-301.pdf
};

// 差分校验方式
enum
{
	RTK_CHECK_BSL		= 0x01,	//	基线长度
	RTK_CHECK_RATIO		= 0x02,	//	卫星分主副组验证
	RTK_CHECK_PITCH		= 0x04,	//	载体模式俯仰角验证
	RTK_CHECK_RESIDUAL	= 0x08	//	残差检验
};
enum
{
	RTK_C_LOST		= 0x01,	//	差分数据丢失
	RTK_C_RATIO		= 0x02,	//	ratio值过小
	RTK_C_SLIP		= 0x04,	//	存在跳周
	RTK_C_RABAD		= 0x08	//	ratio持续变坏
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
    double range[MAX_FREPIONT_PER_NAVSYS]; // observation data pseudorange (m)伪距
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
	ECEF SingleRcvrPos;		// 单点定位结果
    OBST obs[MAX_RTK_OBS_SVCNT];          // observation data (uncorrected)
} OBSEV;

typedef struct{
	int8 readidx;
	int8 writeidx;
	OBSEV obsev[SLAVE_BUF_LEN];
}OBSEV_BUF;



// 定位所用通道对应的卫星号、频点、支路信息
typedef struct tag_RTK_COMMONSAT_INFO
{
	CHAR	SatID;		// 卫星号
	CHAR	Freq;		// 频点
	CHAR	Branch;		// 支路
	CHAR	UpDown; 	// 卫星升降:0保持1上升-1下降
	BOOL	bKeyStar;	//是否是基准星
	DOUBLE 	Amb;		// 整周模糊度
} RTK_COMMONSAT_INFO;

// 观测时间结构定义
typedef struct tag_BD2_OBSTIME
{
	INT16  year;	// 年月日
	INT16  month;
	INT16  day;
	INT16  weekday;
	INT16  hour;	// 时分秒
	INT16  min;
	DOUBLE sec;
		
	DOUBLE SecOfDay ;   // 天秒
	INT32 DayOfYear;	// 年积日

	UINT   nWeeks;		// 整周数
	DOUBLE dSeconds;	// 周秒
	BYTE   precistype;  //精度类型
} BD2_OBSTIME;


// 卫星在站心坐标系中的方位角和高度角结构定义
typedef struct tag_BD2_SAT_AZEL
{
	DOUBLE azimuth;		// 方位角
	DOUBLE elevation;	// 高度角
	DOUBLE elevation2;	// 高度角
	DOUBLE snr;         //SNR
} BD2_SAT_AZEL;


// 卫星位置计算结果结构
typedef struct tag_BD2_SAT_POS
{
	DOUBLE x;	// 空间时间坐标
	DOUBLE y;
	DOUBLE z;
	DOUBLE tsv;	// 卫星时间偏差，单位：米

	DOUBLE	dx, dy, dz, dtsv;	// 速度分量和时间改正
	DOUBLE ddx,ddy,ddz;	// 卫星加速度

	CHAR	SatID;		// 卫星编号
	CHAR	Freq;		// 频点
	CHAR	Branch;	    // 通道	
} BD2_SAT_POS;


// 定位模式
typedef struct tag_BD2_POS_MODE
{
	// 模式参数
	BOOL	System;			// I路是BD2定位， Q路是GPS定位
	BOOL	UseWaas;		// 是否使用差分
	BOOL    Use2D;          // 是否2D定位
	word32	FreqUsed;		// 所用频点个数：1、2、3,4
	word32	Freq[MAX_FREPIONT_PER_NAVSYS];	// 所使用的频点
	CHAR    PCode;          // 使用P码定位

	CHAR	cPosType;			// 解算模式
	CHAR	cWeightType;		// 定权模式
	UINT32  RnssMode;
} BD2_POS_MODE;


// 基线向量结构体
typedef struct tag_BD2_BASELINE
{	
	CHAR 	nFixed;			//是否固定

	double	East;			//东
	double	North;			//北
	double	Up;				//天
	
	double	B;			//纬
	double	L;			//经
	double	H;			//高

	ECEF	dxyz;
	
	double	BaselineLongth;	//基线长度
	double 	Gauss_dx;
	double 	Gauss_dy;
	double  dVtV;	
} BD2_BASELINE;

// 姿态向量结构体
typedef struct tag_BD2_Attitude
{
	double 	yaw;	//航向角
	double	pitch;	//俯仰角
	double	roll;	//翻滚角
	double 	yaw_deg;	//航向角
	double	pitch_deg;	//俯仰角
	double	roll_deg;	//翻滚角

	// 速度方向
	double dSpeedCourse;
	BOOL  bSpeedCourseValid;
	// 坐标北
	double	headingCoord;
	double	headingCoord_deg;
}BD2_Attitude;

typedef struct TGaussCoord
{
	DOUBLE X;
	DOUBLE Y;
	DOUBLE H;
} GAUSS_COORD;


// 用户所在位置经纬度高程结构定义
typedef struct tag_BD2_USER_BLH
{
	DOUBLE latitude;	// 纬度
	DOUBLE longitude;	// 经度
	DOUBLE altitude;	// 高程
} BD2_USER_BLH;

// 大地坐标
typedef struct TEarthCoord
{
		DOUBLE dLongitude;	// 经度(度)
		DOUBLE dLatitude;	// 纬度(度)
		DOUBLE dAltitude;	// 高程(米)
		DOUBLE dAltDelta;	// 异常(米)
} EARTH_COORD;


// 定位所用通道对应的卫星号、频点、支路信息
typedef struct tag_BD2_SAT_INFO
{
	CHAR	SatID;	// 卫星号
	int8	cn0;
	UINT8	LLI;	//s
	word32	validFrqpint;	//valid frqpoint map
} BD2_SAT_INFO;

// 定位所用通道对应的卫星号、频点、支路信息
typedef struct tag_RTK_COMMONSAT
{
	// 基准星
	CHAR nKeySat[MAX_SYSM];
	//CHAR nKeySatIndex;
	// 公共卫星
	CHAR nCommSat;
	BD2_SAT_INFO CommonSat[LAMBDA_N_DIM];		// 公共卫星
	// 双差公共卫星及模糊度（无基准星）
	CHAR nCommSat_AMB;
	BD2_SAT_INFO CommonSat_AMB[LAMBDA_N_DIM];		// 公共卫星
} RTK_COMMONSAT;


// 差分解算中需要保存上一历元结果的相关数据
typedef struct tag_RTK_EPOCH_Keep
{	
	double tow;	//last RTK time
	
	bool bFirstCalAmb;	//be first calc float N filter浮点解
	bool bLastIntValid;	// is last int fix valid
	bool bSuperWide;	// is super wide lane used?
	int8 RTKErrPath;	//debug info for Error path
	
	// 卫星相关信息
	CHAR nKeySatID[MAX_SYSM];		//上一历元基准卫星号
	CHAR nCommonSat; 	//公共卫星个数	
	BD2_SAT_INFO CommonSat[MAX_RTK_OBS_SVCNT];	//公共卫星信息

	CHAR nCommonSatAMB;
	BD2_SAT_INFO CommonSat_Amb[LAMBDA_N_DIM];//公共卫星信息，不含基准星
	
	int32 dAMB_NS_Fix[SV_NUM_TRUE];	//单频整周模糊度
	word256 bAMB_NS_Fix_Valid;

	int32 dAMB_NW_Fix[SV_NUM_TRUE];	//宽巷整周模糊度
	word256 bAMB_NW_Fix_Valid;

	double dAMB_NWf[SV_NUM_TRUE];	//宽巷整周模糊度浮点值
	word256 bAMB_NWf_Valid;

	CHAR nCommonSat2; 	//公共卫星个数	
	BD2_SAT_INFO CommonSat2[MAX_RTK_OBS_SVCNT]; // 上上历元的公共卫星信息，用于三历元间观测量变化探测周跳
	
	// 观测量相关信息	
	OBSEV  prmOldBase;
	OBSEV  prmOldRove;
	int begin;
	double  phaseDSKeep[LAMBDA_N_DIM];
	double  phaseDSKeep2[LAMBDA_N_DIM];

	// 保存的可用AMB及公共卫星个数，不一定是上个历元的信息
	BOOL bGetAmb;
	RTK_COMMONSAT_INFO rtk_satinfo[SV_NUM_TRUE];

	// 差分解算数据
	double	Rtk_NKeep[LAMBDA_N_DIM];
	double	Rtk_PKeep[LAMBDA_N_DIM*LAMBDA_N_DIM];

	//周跳
	CHAR	slip[SV_NUM_TRUE];

	//RTK flag
	int8 nAmbIndex;		//single frequency
	int8 nAmbIndex_wide;	//wide  frequency
	int8 nAmbIndex_wfloat;	//RTK wide float res

	// 单频模糊度平滑
	double	dSF_AMB_Smooth[SV_NUM_TRUE];	// 滤波后的整周模糊度
	double	dSF_AMB_Pk[SV_NUM_TRUE];		// 滤波的协方差矩阵。
	bool	nAMB_Smooth_Flag[SV_NUM_TRUE];	// 滤波平滑标志

	//sv used
	word256 L1UsedSVMap;

#if ENABLE_RTD_KF
	RTD_KF RTDKFPara;
#endif
}RTK_EPOCH_Keep;

typedef struct tag_BD2_PVT
{
	DOUBLE BD2_Time;		// 观测时间
	DOUBLE x,y,z,tb,tg,tgln,tf;		// 定位结果，包括接收机坐标和接收机时间偏差
	DOUBLE dx,dy,dz,dtb;
	BD2_USER_BLH blh;	// 经纬度和高程

	DOUBLE kax,kay,kaz;
	BD2_USER_BLH blhka;	
	
	double dVe, dVu, dVn, dVGround;
	DOUBLE  HDOP,		// 水平精度因子
		VDOP,		// 垂直
		TDOP,		// 时间
		PDOP,		// 定位精度因子
		GDOP;		// 几何精度因子

	DOUBLE error;       // 定位计算的收敛误差
	INT16 valid;		// 上述误差值是否有效
	INT16 nUsedSat;		// 定位解用到的卫星个数
} BD2_PVT;


// 输出参数
typedef struct tag_BD2_RTK_OUTPUT
{	
	BD2_OBSTIME		time;
	BD2_POS_MODE 	PosMode_Rtk;	// 定位模式
	BD2_PVT			RovePos;	// RNSS定位结果信息
	GAUSS_COORD     BaseCoord_Gauss;

	int8 	nAmbIndex;		//RTK single fix res
	int8 	nAmbIndex_wide;		//RTK wide fix res
	int8 	nAmbIndex_wfloat;		//RTK wide float res
	double 	Ratio;			
	double 	Rdop;			//相对定位精度

	BD2_BASELINE	baseline;	// 基线向量
	BD2_Attitude	attitue;	// 姿态参数

	char 	nKeySatID;
	char 	nCommSat;
	word256 nRtkUseID;	
	double	dAMB[LAMBDA_N_DIM];

	unsigned short RtkCheck;
} RTK_OUTPUT;


// 历元变量
typedef struct tag_RTK_EPOCH
{
	bool bSuperWide;	// is super wide lane used?
	
	// 差分观测量
	OBSEV Prm_Base;
	OBSEV Prm_Rove;
	// 公共卫星
	CHAR				nCommonSat;
	BD2_SAT_INFO		CommonSat[MAX_RTK_OBS_SVCNT];		// 公共卫星
	CHAR				nCommonSatAMB;
	BD2_SAT_INFO		CommonSat_Amb[LAMBDA_N_DIM];		// 公共卫星
	
	// 基准星
	int8				KeySatIdIndex[MAX_SYSM];
	int8				KeySatId[MAX_SYSM];

	// 可用公共卫星信息
	RTK_COMMONSAT UsableCommSat;	

	//SV to base station distance
	double DisSV2Base[MAX_RTK_OBS_SVCNT];

	// 周跳
	DOUBLE  Phase_DDD[SV_NUM_TRUE];
	
	// 差分结果
	ECEF RovePVT_RTD;	//rove position, RTD
	ECEF RovePVT_FLOAT;	//rove position, NWf
	ECEF RovePVT_Fix;	//rove position, Int
	ECEF Pos_aver;
	ECEF Pos_std;

	DOUBLE		ApaInv[LAMBDA_N_DIM*3];			// 设计矩阵inv(AD*P*AD)
	DOUBLE		AD[LAMBDA_N_DIM*3];			// A矩阵单差
	DOUBLE		ADD[LAMBDA_N_DIM*3];			// A矩阵单差
	
	// 单频整周模糊度
	double dAMB_Fix[LAMBDA_N_DIM*LAMBDA_N_CANDS];
	double dAMB_Float[LAMBDA_N_DIM];
	double dAMB_PrDD[LAMBDA_N_DIM];
	
	// 双频整周模糊度
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

	UINT16 cChkMode;	// 定向的校验方式
	
	double 	RangeCovariance[LAMBDA_N_DIM*LAMBDA_N_DIM];
	double 	PhaseCovariance[LAMBDA_N_DIM*LAMBDA_N_DIM];
	double 	RangeCovarianceInv[LAMBDA_N_DIM*LAMBDA_N_DIM];

	CHAR CommSat_AMB_SwitchErr_List[LAMBDA_N_DIM];

	int8 nAmbIndex;
	int8 nAmbIndex_wide;		//RTK wide fix res
	int8 nAmbIndex_wfloat;		//RTK wide float res
	double dRatio;	//0:单历元值	1:单历元与多历元最大值
	BD2_BASELINE	Baseline;	//基线向量	
	BD2_Attitude	Attitude;	//姿态向量

	double dInv_HTH_HT[LAMBDA_N_DIM*3];	//记忆inv(H'H)*H*lambda，只用计算Phi-N即可。
	double 	PhaseCovarianceInv[LAMBDA_N_DIM*LAMBDA_N_DIM];

	BOOL bAMB_SK;	// 整周模糊度的搜索与保持功能标志

	word32 SelectFrepMapSW;	//super wide lane
	word32 SelectFrepMap;	//wide lane
	word32 SelectFrepMapL1;	//双频切单频选频结果

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

// RTK的观测量推算
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
