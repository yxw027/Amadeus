#ifndef _DEFINE_H
#define _DEFINE_H

#include "typedefine.h"
#include <string.h>
#include <stdio.h>

//System support define
#define SUPPORT_GLONASS	1
#define FLASH_SUPPORT	1
#define SUPPORT_SUPER_WIDE_LANE	0
#define __WATCHDOG_EN	1

#define RTCM3_RX_ENABLE	1
#define __X2240_SYSTEM				(0)

#define OFFLINE_DEBUG_ENABLE  0		// 1, debug DSP and NISO; 0, not debug

#if __X2240_SYSTEM
#define BOOTLODER_ADDR  0x8000bec0
#else 
#define BOOTLODER_ADDR  0x80006cc0
#endif 
//GNSS Max system support define
#define MAX_SYSM	3


//Compile define
#define CALC_PVT_LS		0
#define CALC_PVT_KF		1
#define PVT_MODE	CALC_PVT_KF

//Max Freqpoint in one nav system 
#define MAX_FREPIONT_PER_NAVSYS		3			//单系统最大支持的频点数
#define MAX_TRKCHCNT_PER_ONESV		8		//	单系统单颗卫星最大支持通道数


//Channel define
#define MAXACQCHANNELS          (1)
#define INVALID_TRKCHID			(-1)


#define _FAULT_LOST_LOCK		(0)

#if __X2240_SYSTEM
#define TRKCH_START_ID			(0)
#define NAVSYS_MAX_SV			(11)
#else
#define TRKCH_START_ID			(4)	
#define NAVSYS_MAX_SV			(16)
#endif

#if __X2240_SYSTEM
#define MAXCHANNELS             (48)
#define L2P_MAXCHANNELS			(12)
#define L1CA_TRKID_MIN			(24)
#define L1CA_TRKID_MAX			(L1CA_TRKID_MIN + L2P_MAXCHANNELS - 1)
#define L2P_TRKID_MIN			(L1CA_TRKID_MIN + L2P_MAXCHANNELS)
#define L2P_TRKID_MAX			(L2P_TRKID_MIN + L2P_MAXCHANNELS - 1)
#else
#define MAXCHANNELS             (120)
#define L2P_MAXCHANNELS			(12)
#define L1CA_TRKID_MIN			(96)
#define L1CA_TRKID_MAX			(L1CA_TRKID_MIN + L2P_MAXCHANNELS - 1)
#define L2P_TRKID_MIN			(L1CA_TRKID_MIN + L2P_MAXCHANNELS)
#define L2P_TRKID_MAX			(L2P_TRKID_MIN + L2P_MAXCHANNELS - 1)
#endif

//SV id define
#define MinGpsSvID      		(1)
#define MaxGpsSvID      		(32)
#define MinBD2SvID      		(33)
#define MaxBD2SvID      		(64)
#define MaxGEOSvID      		(37)
#if SUPPORT_GLONASS
#define MinGloFreID				(65)
#define MaxGloFreID				(78)
#define MaxGloSvIDTRUE              (88)
#endif

#define SV_IsGps(uSvId)	( ((uSvId)>=MinGpsSvID) && ((uSvId)<=MaxGpsSvID) )
#define SV_IsBd2(uSvId)	( ((uSvId)>=MinBD2SvID) && ((uSvId)<=MaxBD2SvID) )
#define SV_IsBd2Geo(uSvId)	( ((uSvId)>=MinBD2SvID) && ((uSvId)<=MaxGEOSvID) )
#define SV_IsBd2Meo(uSvId)	( ((uSvId)>=(MaxGEOSvID+1)) && ((uSvId)<=MaxBD2SvID) )
#define SV_IsBd2IGSO(uSvId)	( ((uSvId)>=(MinBD2SvID+5)) && ((uSvId)<MinBD2SvID+10) )
#if SUPPORT_GLONASS
#define SV_IsGlo(uSvId)	( ((uSvId)>=MinGloFreID) && ((uSvId)<=MaxGloSvIDTRUE) )
#endif

#define ECA_SV_ID				(100)
#define ECA_TRKCH_ID			(MAXCHANNELS)
#define SV_IsECA(uSvID)			(uSvID==ECA_SV_ID)

#if SUPPORT_GLONASS
#define SV_NUM				(MaxGloFreID)
#define SV_NUM_TRUE			(MaxGloSvIDTRUE)
#else
#define SV_NUM          (MaxBD2SvID)
#define SV_NUM_TRUE     (MaxBD2SvID)

#endif

#define SV_NUM_GPSBD		(MaxBD2SvID)
#define GPS_SV_NUM			(MaxGpsSvID - MinGpsSvID +1)

#if SUPPORT_GLONASS
#define FRE_NUM_GLO			(MaxGloFreID-MinGloFreID+1)
#define SV_NUM_GLO_TRUE     (24)
#endif

//UART define
#define MAX_UART_NUM 3
#if __X2240_SYSTEM
#define COM_ID_RTK		2
#define COM_ID_DEBUG0	1
#define COM_ID_DEBUG1	0	//back up com
#else
#define COM_ID_RTK		1
#define COM_ID_DEBUG0	0
#define COM_ID_DEBUG1	2	//back up com
#endif

#if __X2240_SYSTEM
#define DEFAULT_FREQ_GROUP_BD (DSP_B1I_FRE | DSP_B2I_FRE)
#define DEFAULT_FREQ_GROUP_GPS (DSP_L1CA_FRE | DSP_L2P_FRE)
#else
#define DEFAULT_FREQ_GROUP_BD (DSP_B1I_FRE | DSP_B2I_FRE| DSP_B3I_FRE)
#define DEFAULT_FREQ_GROUP_GPS (DSP_L1CA_FRE | DSP_L2P_FRE|DSP_L5_FRE)
#define	DEFAULT_FREQ_GROUP_GLO	(DSP_G1CA_FRE | DSP_G2CA_FRE)
#endif

//Frequency point
#define 	DSP_B1I_FRE		(0x1)
#define 	DSP_B2I_FRE		(0x1<<1)
#define		DSP_B3I_FRE 	(0x1<<2)
#define		DSP_L1CA_FRE	(0x1<<3)
#define		DSP_L1C_FRE 	(0x1<<4)
#define		DSP_L1P_FRE 	(0x1<<5)
#define		DSP_L2C_FRE 	(0x1<<6)
#define		DSP_L2P_FRE 	(0x1<<7)
#define		DSP_L5_FRE		(0x1<<8)
#define		DSP_G1CA_FRE	(0x1<<9)
#define		DSP_G2CA_FRE	(0x1<<10)
//#define		DSP_E1_FRE	(0x1<<11)
//#define		DSP_E5a_FRE 	(0x1<<12)
//#define		DSP_E5b_FRE 	(0x1<<13)
#define		FREQ_CNT	11

#define		FREQ_GROUP_BD	(DSP_B1I_FRE | DSP_B2I_FRE | DSP_B3I_FRE)
#define		FREQ_GROUP_GPS	(DSP_L1CA_FRE | DSP_L1C_FRE | DSP_L1P_FRE| DSP_L2C_FRE | DSP_L2P_FRE| DSP_L5_FRE)
#define		FREQ_GROUP_GLO	(DSP_G1CA_FRE | DSP_G2CA_FRE)
//#define		FREQ_GROUP_GEL	(DSP_E1_FRE | DSP_E5a_FRE | DSP_E5b_FRE)

#define		FREQ_GROUP_B1	(DSP_B1I_FRE)
#define		FREQ_GROUP_B2	(DSP_B2I_FRE)
#define		FREQ_GROUP_B3	(DSP_B3I_FRE)
#define		FREQ_GROUP_L1	(DSP_L1CA_FRE | DSP_L1C_FRE | DSP_L1P_FRE)
#define		FREQ_GROUP_L2	(DSP_L2C_FRE | DSP_L2P_FRE)
#define		FREQ_GROUP_L5	(DSP_L5_FRE)
#define		FREQ_GROUP_G1	(DSP_G1CA_FRE)
#define		FREQ_GROUP_G2	(DSP_G2CA_FRE)
//#define		FREQ_GROUP_E1	(DSP_E1_FRE)
//#define		FREQ_GROUP_E2	(DSP_E5a_FRE | DSP_E5b_FRE)

#define		FREQ_GROUP_IDX0	(DSP_B1I_FRE | DSP_L1CA_FRE | DSP_L1C_FRE | DSP_L1P_FRE | DSP_G1CA_FRE)
#define		FREQ_GROUP_IDX1	(DSP_B2I_FRE | DSP_L2C_FRE | DSP_L2P_FRE | DSP_G2CA_FRE)
#define		FREQ_GROUP_IDX2	(DSP_B3I_FRE | DSP_L5_FRE)


//Modulation 
#define		MODU_L1_C	0
#define		MODU_L1_P	1
#define		MODU_L1_CA	2


//branch define
#define		BRANCH_DATA			0
#define		BRANCH_PILOT		1
#define		BRANCH_DATA_PILOT	2

//startup mode
#define		COLD_START	1
#define		WARM_START	2
#define		HOT_START	3

//ACQ channel state.
#define		ACQU_IDLE	0
#define		ACQU_BUSY	1

//TRK channel state.
#define		NO_SIG_PVT		0
#define		IDLE_PVT		3
#define		ACQUSITION_PVT	4
#define		START_TRK_PVT	5
#define		TRACKING_PVT	7

//ACQ mode
#define ACQ_IDLE					0xf
#define ACQ_MODE_0					0		//8ms	
#define ACQ_MODE_1					1		//20ms
#define ACQ_MODE_2					2		//20Nms
#define ACQ_MODE_3					3		//2ms
#define ACQ_MODE_4					4	
#define ACQ_MODE_5					5		//20ms
#define ACQ_MODE_6					6		//
#define ACQ_MODE_7					7		//
#define ACQ_MODE_8					8		//
#define ACQ_MODE_9					9		//
#define ACQ_MODE_CNT				10	

//TRK mode
#define TRK_STATE_IDLE						0xff
#define TRK_STATE_0					0
#define TRK_STATE_1					1
#define TRK_STATE_2					2
#define TRK_STATE_3					3
#define TRK_STATE_4					4
#define TRK_STATE_5					5
#define TRK_STATE_6					6
#define TRK_STATE_7					7
#define TRK_STATE_8					8
#define TRK_STATE_9					9
#define TRK_STATE_10				10
#define TRK_STATE_11				11
#define TRK_STATE_12				12
#define TRK_STATE_13				13
#define TRK_STATE_14				14
#define TRK_STATE_15				15
#define TRK_STATE_16				16
#define TRK_STATE_17				17
#define TRK_STATE_18				18
#define TRK_MODE_PARA_TBL_LEN		19



//Navigation mode
#define NAV_SYS_NONE	0
#define NAV_SYS_GPS		(0x1)
#define NAV_SYS_BD		(0x1<<1)
#define NAV_SYS_GLO		(0x1<<2)

#define NAVMODE_GPSONLY	 (NAV_SYS_GPS)
#define NAVMODE_BDONLY	 (NAV_SYS_BD)
#define NAVMODE_GLOONLY	 (NAV_SYS_GLO)
#define NAVMODE_GPS_BD	 (NAV_SYS_GPS|NAV_SYS_BD)
#define NAVMODE_GPS_GLO	 (NAV_SYS_GPS|NAV_SYS_GLO)
#define NAVMODE_BD_GLO	 (NAV_SYS_BD|NAV_SYS_GLO)
#define NAVMODE_GPS_BD_GLO	 (NAV_SYS_GPS|NAV_SYS_BD|NAV_SYS_GLO)


// SV Health parameter
#define SV_HEALTH_WELL			0		//All signal transfered from SV is healthy
#define SV_HEALTH_USABLE		1		//SV has some problems, but still can be used in fix
#define SV_HEALTH_BAD			2		//SV is too bad to fix
#define SV_HEALTH_TIMEOUT		3		//SV health status is timeout, which need to update

#define EPHALM_SRC_NORMAL		0
#define EPHALM_SRC_BFSFSYNC		1
#define EPHALM_SRC_BACKUP		2
#define EPHALM_SRC_AGNSS		3

#define IONO_SRC_NONE		0
#define IONO_SRC_GPS_ALM	1
#define IONO_SRC_BD2_MEO	2
#define IONO_SRC_BD2_GEO	3

#define SV_ORBIT_TYPE_UNKNOW		(-1)
#define SV_ORBIT_TYPE_GPS			0
#define SV_ORBIT_TYPE_GEO			1
#define SV_ORBIT_TYPE_BD_IGSO		2
#define SV_ORBIT_TYPE_BD_MEO		3
#define SV_ORBIT_TYPE_GLO			4


//sv EPH ALM age 
#define SV_EPH_ALM_AGE_INITIAL		0
#define SV_EPH_ALM_AGE_INVALID		(1512000)	// 2.5 week, units:s
#define SV_EPH_AGE_USABLE_THRES		(7200)	// 2 hours, units:s
#define GLO_SV_EPH_AGE_USABLE_THRES		(3600)	// 2.5 hours, units:s
#define SV_ALM_AGE_USABLE_THRES		(604800)	//  1 week, units:s
#define SV_MIN_EPH_AGE				(-7200)	//  4 hours, units:s
#define SV_MIN_ALM_AGE				(-604800)	//  1 week, units:s
#define SV_EPH_ALM_AGE_UPD_TIME		0


/* Doppler shift source flag */
#define NO_DOPPLER 				0 // no valid Doppler shift
#define SWFREQ_SRC_TCXO		1
#define SWFREQ_SRC_ALM		2
#define SWFREQ_SRC_EPH		3

//SV el
#define SV_ELEVATION_INVALID	(100.0)
#define SV_ELEVATION_VALID_TH	(90.0001)

#define	MAX_USE_SV_NUM			16

//UART define
#define UART_PORT_DSPUart0  0
#define UART_PORT_DSPUart1  1
#define USP_1               0x03
#define UART_PORT_ALL       0x04
#define USP_2               0x05

#define UART_SEND_BUF_LENTH 	(16*1024)
#define UART_RECV_BUF_LENTH 	(1024*40)
#define ONE_MSG_MAX_LENTH		(1024*20)
#define UART_RECV_FIELD_LENTH 	100

#define MAX(a,b)		\
		(a > b? a : b)

#define MIN(a,b)		\
		(a > b? b : a)






/**************************define for NIOS******************************/


#endif


