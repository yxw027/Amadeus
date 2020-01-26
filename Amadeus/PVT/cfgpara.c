
#include "define.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "cfgpara.h"
#include "HCP.h"
#include "rtcm3.h"
#ifndef _POSTPROC
#ifndef _SIMULATE
#include "HDefine.h"
#include "evmc6747_sys.h"
#include "sysbackup.h"
#include "UartFunc.h"
#endif
#endif

// pointer CPT in CPT saved space(Flash). when cold start, system load parameter from this table  
SYS_CPT_REGION ActiveCPTRegion;
SYS_CPT_REGION *pActiveCPTRegion = NULL;	//pActiveCPTRegion only be used in
SYS_CPT_REGION *pSharedDDRCPTRegion =  NULL;
SYSM_CPT *pActiveCPT = NULL;

#if __X2240_SYSTEM
const SYSM_CPT FactoryCPT=
{
	{	1,								//command  	
		(RTDKFLAG_KF|RTDKFLAG_RTD|RTDKFLAG_RTK|RTDKFLAG_AUTO),//RTDKFlag
		60,							//RTKTimeOut, s
		(FUNSWITCH_IONOCORRECT|FUNSWITCH_TROPOCORRECT|FUNSWITCH_STATICNAV),	//FunSwitch
		100,								//FixUpdateCycle  Fix cycle. Uints: 10ms
		20,								//pdop mask
		5,								//SnrMaskSingle
		35,								//SnrMaskRTK

		15,								//ElvMaskRTKGPS, degree
		15,								//ElvMaskRTKBD, degree
		15,								//ElvMaskRTKGLO, degree
		15,								//ElvMaskRTKGAL, degree
		5,								//ElvMaskSinGPS, degree
		5,								//ElvMaskSinBD, degree
		10,								//ElvMaskSinGLO, degree
		10,								//ElvMaskSinGAL, degree

		1111,							//BaseStaID
		0,		//ValidFlag;			//Bit0: 基准站位置是否有效其他预留
		14,								//Leap second;			
		200,							//RTCMDelay;				//RTCM3 transfer delay,ms

		-1336317.4599164,				//BaseStaPos_x; //m, chengdu Antnna1
		5333575.521585,					//BaseStaPos_y;	//m, chengdu Antnna1
		3222461.85890275,				//BaseStaPos_z;	//m, chengdu Antnna1

		0,								//	AntHeight mm
		25000000,						// 	RFSampleFrq  Hz
		0,								//Delay1PPS, unit: ns
		100,							//PulsWidth,  unit:ms

		0xFFFFFFFF,						//	UINT32		FixNavFreqPoint;			//用户设置模式
		20,                //MaxRtkSvcnt
		0,			//res
		0,								//Res2;				//预留

		{0xffffffff,0xffffffff,0x00003fff,0,0,0,0,0},		//SVBitMAp
		0.42e6, 	//	IF_L1
		2.6e6,			//	IF_L2
		1.45e6, 	//	IF_L5
		1.098e6,		//	IF_B1
		2.14e6, 	//	IF_B2
		23.52e6,		//	IF_B3
		2.0e6,			//	IF_G1
		1.0e6,			//	IF_G2
		0.42e6, 		//	IF_Galilo
		0,				//	Delay_L1  units : ns
		3000,			//	Delay_L2  units : ns
		0,				//	Delay_L5  units : ns
		0,				//	Delay_B1  units : ns	

		
		0,				//	Delay_B2  units : ns
		0,				//	Delay_B3  units : ns
		0,				//	Delay_G1  units : ns
		0,				//	Delay_G2  units : ns	

		0,				//	Delay_Galilo  units : ns
		0,				//	Res28  units : ns
		0,				//	Res29  units : ns
		0,				//	Res30  units : ns			
		(DEFAULT_FREQ_GROUP_BD|DEFAULT_FREQ_GROUP_GPS),		//NavFreqPoint
		0,					//Res40;						//预留
		0,					//Res41;					//预留

		{0,0},
	},

//协议参数配置
	{
		{//COM0
			0,		//UINT8 	COMInterFaceMode;
			0,		//UINT8 	GGA;
			0,		//UINT8 	RMC;
			0,		//UINT8 	DHV;
			0,		//UINT8 	GSV;
			0,		//UINT8 	GLL;
			0,		//UINT8 	GSA;
			0,		//UINT8 	Res4;
			
			0,		//UINT8 	Res5;
			0,		//UINT8 	Res6;
			0,		//UINT8 	Res7;
			0,		//UINT8 	Res8;
			0,		//UINT8 	Res9;
			0,		//UINT8 	HCPOutputCycle;
			0,		//UINT8 	NovAtel_Bestposa;
			0,		//UINT8 	NovAtel_Rangea;
			
			0,		//UINT8 	NovAtel_Satvis2a;
			0,		//UINT8 	Res10;
			0,		//UINT8 	Res11;
			0,		//UINT8 	Res12;
			0,		//UINT8 	Rtcm3_1001;
			0,		//UINT8 	Rtcm3_1002;
			0,		//UINT8 	Rtcm3_1003;
			0,		//UINT8 	Rtcm3_1004;
			
			0,		//UINT8 	Rtcm3_1005;
			0,		//UINT8 	Rtcm3_1006;
			0,		//UINT8 	Rtcm3_1012;
			0,		//UINT8 	Rtcm3_1071;
			0,		//UINT8 	Rtcm3_1072;
			0,		//UINT8 	Rtcm3_1073;
			0,		//UINT8 	Rtcm3_1074;
			0,		//UINT8 	Rtcm3_1075;
			
			0,		//UINT8 	Rtcm3_1076;
			0,		//UINT8 	Rtcm3_1077;
			0,		//UINT8 	Rtcm3_1081;
			0,		//UINT8 	Rtcm3_1082;
			0,		//UINT8 	Rtcm3_1083;
			0,		//UINT8 	Rtcm3_1084;
			0,		//UINT8 	Rtcm3_1085;
			0,		//UINT8 	Rtcm3_1086;
			
			0,		//UINT8 	Rtcm3_1087;
			0,		//UINT8 	Rtcm3_1121;
			0,		//UINT8 	Rtcm3_1122;
			0,		//UINT8 	Rtcm3_1123;
			0,		//UINT8 	Rtcm3_1124;
			0,		//UINT8 	Rtcm3_1125;
			0,		//UINT8 	Rtcm3_1126;
			0,		//UINT8 	Rtcm3_1127;
			
			0,		//UINT8 	Rtcm3_1104;
			0,		//UINT8	Rtcm3_1019;
			0,		//UINT8	Rtcm3_1047;
			0,		//UINT8 	Res15;
			0,		//UINT8 	Res16;
			0,		//UINT8 	Res17;
			0,		//UINT8 	Res18;
			0,		//UINT8 	Res19;
			
			0,		//UINT8 	Res31;
			0,		//UINT8 	Res32;
			0,		//UINT8 	Res33;
			0,		//UINT8 	Res34;
			0,		//UINT8 	Res35;
			0,		//UINT8 	Res36;
			0,		//UINT8 	Res37;
			0,		//UINT8 	Res38;
		},
		{  //COM1
			(CHECK_DATA_HEAD_NMEA|CHECK_DATA_HEAD_HCP|CHECK_DATA_HEAD_RMO|CHECK_DATA_HEAD_NORATEL), 	//UINT8 	COMInterFaceMode;
			1,		//UINT8 	GGA;
			1,		//UINT8 	RMC;
			0,		//UINT8 	DHV;
			1,		//UINT8 	GSV;
			0,		//UINT8 	GLL;
			1,		//UINT8 	GSA;
			0,		//UINT8 	Res4;
			
			0,		//UINT8 	Res5;
			0,		//UINT8 	Res6;
			0,		//UINT8 	Res7;
			0,		//UINT8 	Res8;
			0,		//UINT8 	Res9;
			1,		//UINT8 	HCPOutputCycle;
			1,		//UINT8 	NovAtel_Bestposa;
			1,		//UINT8 	NovAtel_Rangea;
			
			1,		//UINT8 	NovAtel_Satvis2a;
			0,		//UINT8 	Res10;
			0,		//UINT8 	Res11;
			0,		//UINT8 	Res12;
			0,		//UINT8 	Rtcm3_1001;
			0,		//UINT8 	Rtcm3_1002;
			0,		//UINT8 	Rtcm3_1003;
			0,		//UINT8 	Rtcm3_1004;
			
			0,		//UINT8 	Rtcm3_1005;
			0,		//UINT8 	Rtcm3_1006;
			0,		//UINT8 	Rtcm3_1012;
			0,		//UINT8 	Rtcm3_1071;
			0,		//UINT8 	Rtcm3_1072;
			0,		//UINT8 	Rtcm3_1073;
			0,		//UINT8 	Rtcm3_1074;
			0,		//UINT8 	Rtcm3_1075;
			
			0,		//UINT8 	Rtcm3_1076;
			0,		//UINT8 	Rtcm3_1077;
			0,		//UINT8 	Rtcm3_1081;
			0,		//UINT8 	Rtcm3_1082;
			0,		//UINT8 	Rtcm3_1083;
			0,		//UINT8 	Rtcm3_1084;
			0,		//UINT8 	Rtcm3_1085;
			0,		//UINT8 	Rtcm3_1086;
			
			0,		//UINT8 	Rtcm3_1087;
			0,		//UINT8 	Rtcm3_1121;
			0,		//UINT8 	Rtcm3_1122;
			0,		//UINT8 	Rtcm3_1123;
			0,		//UINT8 	Rtcm3_1124;
			0,		//UINT8 	Rtcm3_1125;
			0,		//UINT8 	Rtcm3_1126;
			0,		//UINT8 	Rtcm3_1127;
			
			0,		//UINT8 	Rtcm3_1104;
			60,		//UINT8	Rtcm3_1019;
			60,		//UINT8	Rtcm3_1047;
			0,		//UINT8 	Res15;
			0,		//UINT8 	Res16;
			0,		//UINT8 	Res17;
			0,		//UINT8 	Res18;
			0,		//UINT8 	Res19;
			
			0,		//UINT8 	Res31;
			0,		//UINT8 	Res32;
			0,		//UINT8 	Res33;
			0,		//UINT8 	Res34;
			0,		//UINT8 	Res35;
			0,		//UINT8 	Res36;
			0,		//UINT8 	Res37;
			0,		//UINT8 	Res38;
		},
		//COM2
		{
			(CHECK_DATA_HEAD_RTCM3|CHECK_DATA_HEAD_RMO|CHECK_DATA_HEAD_NMEA|CHECK_DATA_HEAD_HCP|CHECK_DATA_HEAD_NORATEL),		//UINT8 	COMInterFaceMode;
			0,		//UINT8 	GGA;
			0,		//UINT8 	RMC;
			0,		//UINT8 	DHV;
			0,		//UINT8 	GSV;
			0,		//UINT8 	GLL;
			0,		//UINT8 	GSA;
			0,		//UINT8 	Res4;
			
			0,		//UINT8 	Res5;
			0,		//UINT8 	Res6;
			0,		//UINT8 	Res7;
			0,		//UINT8 	Res8;
			0,		//UINT8 	Res9;
			0,		//UINT8 	HCPOutputCycle;
			0,		//UINT8 	NovAtel_Bestposa;
			0,		//UINT8 	NovAtel_Rangea;
			
			0,		//UINT8 	NovAtel_Satvis2a;
			0,		//UINT8 	Res10;
			0,		//UINT8 	Res11;
			0,		//UINT8 	Res12;
			0,		//UINT8 	Rtcm3_1001;
			0,		//UINT8 	Rtcm3_1002;
			0,		//UINT8 	Rtcm3_1003;
			0,		//UINT8 	Rtcm3_1004;
			
			0,		//UINT8 	Rtcm3_1005;
			1,		//UINT8 	Rtcm3_1006;
			0,		//UINT8 	Rtcm3_1012;
			0,		//UINT8 	Rtcm3_1071;
			0,		//UINT8 	Rtcm3_1072;
			0,		//UINT8 	Rtcm3_1073;
			0,		//UINT8 	Rtcm3_1074;
			0,		//UINT8 	Rtcm3_1075;
			
			0,		//UINT8 	Rtcm3_1076;
			1,		//UINT8 	Rtcm3_1077;
			0,		//UINT8 	Rtcm3_1081;
			0,		//UINT8 	Rtcm3_1082;
			0,		//UINT8 	Rtcm3_1083;
			0,		//UINT8 	Rtcm3_1084;
			0,		//UINT8 	Rtcm3_1085;
			0,		//UINT8 	Rtcm3_1086;
			
			1,		//UINT8 	Rtcm3_1087;
			0,		//UINT8 	Rtcm3_1121;
			0,		//UINT8 	Rtcm3_1122;
			0,		//UINT8 	Rtcm3_1123;
			0,		//UINT8 	Rtcm3_1124;
			0,		//UINT8 	Rtcm3_1125;
			0,		//UINT8 	Rtcm3_1126;
			1,		//UINT8 	Rtcm3_1127;
			
			0,		//UINT8 	Rtcm3_1104;
			0,		//UINT8	Rtcm3_1019;
			0,		//UINT8	Rtcm3_1047;
			0,		//UINT8 	Res15;
			0,		//UINT8 	Res16;
			0,		//UINT8 	Res17;
			0,		//UINT8 	Res18;
			0,		//UINT8 	Res19;
			
			0,		//UINT8 	Res31;
			0,		//UINT8 	Res32;
			0,		//UINT8 	Res33;
			0,		//UINT8 	Res34;
			0,		//UINT8 	Res35;
			0,		//UINT8 	Res36;
			0,		//UINT8 	Res37;
			0,		//UINT8 	Res38;
		},
	},
//串口参数配置
	{
		//COM0
		{
			9,		//UINT8		COMBaudRate;      1,4800bps; 2,9600bps; 3,38400bps; 4,38400bps;  5,57600bps;  6,115200bps;  7,230400bps;  8,460800bps;  9,921600bps;  
			1,		//UINT8 		COMStopBit;
			0,		//UINT8		COMParityCheck;
			0,		//UINT8		COMNumDatBits;
			0,		//UINT8 		Res1;
			0,		//UINT8 		Res2;
			0,		//UINT8		Res3;
			0,		//UINT8		Res4;
		},
		//COM1
		{
			8,		//UINT8		COMBaudRate;      1,4800bps; 2,9600bps; 3,38400bps; 4,38400bps;  5,57600bps;  6,115200bps;  7,230400bps;  8,460800bps;  9,921600bps;
			1,		//UINT8 		COMStopBit;
			0,		//UINT8		COMParityCheck;
			0,		//UINT8		COMNumDatBits;
			0,		//UINT8 		Res1;
			0,		//UINT8 		Res2;
			0,		//UINT8		Res3;
			0,		//UINT8		Res4;
		},
		//COM2
		{
			6,		//UINT8		COMBaudRate;      1,4800bps; 2,9600bps; 3,38400bps; 4,38400bps;  5,57600bps;  6,115200bps;  7,230400bps;  8,460800bps;  9,921600bps;
			1,		//UINT8 		COMStopBit;
			0,		//UINT8		COMParityCheck;
			0,		//UINT8		COMNumDatBits;
			0,		//UINT8 		Res1;
			0,		//UINT8 		Res2;
			0,		//UINT8		Res3;
			0,		//UINT8		Res4;
		},
	},
//网络参数配置表
	{
		{'1','9','2','.','1','6','8','.','1','.','1','0','0','\0'},//  ClientIP;	//接收机IP地址，缺省192.168.1.100
		{'0','\0'},									//ServerIP;	//DHCP服务器的IP地址
		{'1','9','2','.','1','6','8','.','1','.','1','\0'},								//GwP	网关
		{'2','5','5','.','2','5','5','.','2','5','5','.','0','\0'},		//	 NetMask;	//本地网络接口的网络掩码，缺省为255.255.255.0
		{'H','w','A','\0'},	//HostName
		{'H','w','A','\0'},	//Device
		{'H','w','A','\0'},	//DevName
		{' ','\0'},			//IPAndName
		{'H','W','A','1','2','3','\0'},			//PassWord
		
		0,		//Prototype  //0:TCP;1:UDP
		0,			//Res25
		40000,		//Port  网络连接的端口，40000-5000	
		0,			//Res26
		0,			//Res27
	},
};
#else
//manufacturer CPT, this table can not be changed  
const SYSM_CPT FactoryCPT=
{
	{	1,								//command  	
		(RTDKFLAG_KF|RTDKFLAG_RTD|RTDKFLAG_RTK|RTDKFLAG_AUTO),//RTDKFlag
		60,							//RTKTimeOut, s
		(FUNSWITCH_IONOCORRECT|FUNSWITCH_TROPOCORRECT|FUNSWITCH_STATICNAV),	//FunSwitch
		100,								//FixUpdateCycle  Fix cycle. Uints: 10ms
		20,								//pdop mask
		5,								//SnrMaskSingle
		25,								//SnrMaskRTK

		10,								//ElvMaskRTKGPS, degree
		10,								//ElvMaskRTKBD, degree
		10,								//ElvMaskRTKGLO, degree
		10,								//ElvMaskRTKGAL, degree
		5,								//ElvMaskSinGPS, degree
		5,								//ElvMaskSinBD, degree
		10,								//ElvMaskSinGLO, degree
		10,								//ElvMaskSinGAL, degree

		1111,							//BaseStaID
		0,		//ValidFlag;			//Bit0: 基准站位置是否有效其他预留
		14,								//Leap second;				
		200,							//RTCMDelay;				//RTCM3 transfer delay,ms

		-1336317.4599164,				//BaseStaPos_x; //m, chengdu Antnna1
		5333575.521585,					//BaseStaPos_y;	//m, chengdu Antnna1
		3222461.85890275,				//BaseStaPos_z;	//m, chengdu Antnna1

		0,								//	AntHeight mm
		50000000,						// 	RFSampleFrq  Hz

		0,								//Delay1PPS, unit: ns
		100,							//PulsWidth,  unit:ms

		0xFFFFFFFF,						//	UINT32		FixNavFreqPoint;			//用户设置模式
		20,
		0,								//Res2;				//预留
		0,

		{0xffffffff,0xffffffff,0x00003fff,0,0,0,0,0},		//SVBitMAp

		10.42e6,		//	IF_L1
		12.6e6,			//	IF_L2
		11.45e6,		//	IF_L5
		11.098e6,		//	IF_B1
		12.14e6,		//	IF_B2
		13.52e6,		//	IF_B3
		12.0e6,			//	IF_G1
		11.0e6,			//	IF_G2
		11.0e6,			//	IF_Galilo

		0,				//	Delay_L1  units : ns
		3000,			//	Delay_L2  units : ns
		0,				//	Delay_L5  units : ns
		0,				//	Delay_B1  units : ns	

		
		0,				//	Delay_B2  units : ns
		0,				//	Delay_B3  units : ns
		0,				//	Delay_G1  units : ns
		0,				//	Delay_G2  units : ns	

		0,				//	Delay_Galilo  units : ns
		0,				//	Res28  units : ns
		0,				//	Res29  units : ns
		0,				//	Res30  units : ns	
		
		(DEFAULT_FREQ_GROUP_BD|DEFAULT_FREQ_GROUP_GPS|DEFAULT_FREQ_GROUP_GLO),		//NavFreqPoint
		0,					//Res40;						//预留
		0,					//Res41;					//预留

		{0,0},
	},

//协议参数配置
	{
		{  //COM0
			(CHECK_DATA_HEAD_RTCM3|CHECK_DATA_HEAD_RMO|CHECK_DATA_HEAD_NMEA|CHECK_DATA_HEAD_HCP|CHECK_DATA_HEAD_NORATEL),		//UINT8		COMInterFaceMode;
			1,		//UINT8		GGA;
			1,		//UINT8		RMC;
			0,		//UINT8		DHV;
			1,		//UINT8		GSV;
			0,		//UINT8		GLL;
			1,		//UINT8		GSA;
			0,		//UINT8		Res4;

			0,		//UINT8		Res5;
			0,		//UINT8		Res6;
			0,		//UINT8		Res7;
			0,		//UINT8		Res8;
			0,		//UINT8		NovAtel_Rangeb;
			1,		//UINT8		HCPOutputCycle;
			1,		//UINT8		NovAtel_Bestposa;
			1,		//UINT8		NovAtel_Rangea;

			1,		//UINT8		NovAtel_Satvis2a;
			0,		//UINT8		NovAtel_Headinga;
			0,		//UINT8		NovAtel_Bestposb;
			0,		//UINT8		NovAtel_Psrdopb;
			0,		//UINT8		Rtcm3_1001;
			0,		//UINT8		Rtcm3_1002;
			0,		//UINT8		Rtcm3_1003;
			0,		//UINT8		Rtcm3_1004;

			0,		//UINT8		Rtcm3_1005;
			0,		//UINT8		Rtcm3_1006;
			0,		//UINT8		Rtcm3_1012;
			0,		//UINT8		Rtcm3_1071;
			0,		//UINT8		Rtcm3_1072;
			0,		//UINT8		Rtcm3_1073;
			0,		//UINT8		Rtcm3_1074;
			0,		//UINT8		Rtcm3_1075;

			0,		//UINT8		Rtcm3_1076;
			0,		//UINT8		Rtcm3_1077;
			0,		//UINT8		Rtcm3_1081;
			0,		//UINT8		Rtcm3_1082;
			0,		//UINT8		Rtcm3_1083;
			0,		//UINT8		Rtcm3_1084;
			0,		//UINT8		Rtcm3_1085;
			0,		//UINT8		Rtcm3_1086;

			0,		//UINT8		Rtcm3_1087;
			0,		//UINT8		Rtcm3_1121;
			0,		//UINT8		Rtcm3_1122;
			0,		//UINT8		Rtcm3_1123;
			0,		//UINT8		Rtcm3_1124;
			0,		//UINT8		Rtcm3_1125;
			0,		//UINT8		Rtcm3_1126;
			0,		//UINT8		Rtcm3_1127;

			0,		//UINT8		Rtcm3_1104;
			60,		//UINT8		Rtcm3_1019;
			60,		//UINT8		Rtcm3_1047;
			0,		//UINT8		Res15;
			0,		//UINT8		Res16;
			0,		//UINT8		Res17;
			0,		//UINT8		Res18;
			0,		//UINT8		Res19;

			0,		//UINT8		Res31;
			0,		//UINT8		Res32;
			0,		//UINT8		Res33;
			0,		//UINT8		Res34;
			0,		//UINT8		Res35;
			0,		//UINT8		Res36;
			0,		//UINT8		Res37;
			0,		//UINT8		Res38;
		},
			{//COM1
			(CHECK_DATA_HEAD_RTCM3|CHECK_DATA_HEAD_RMO|CHECK_DATA_HEAD_NMEA|CHECK_DATA_HEAD_HCP|CHECK_DATA_HEAD_NORATEL),		//UINT8		COMInterFaceMode;
			0,		//UINT8		GGA;
			0,		//UINT8		RMC;
			0,		//UINT8		DHV;
			0,		//UINT8		GSV;
			0,		//UINT8		GLL;
			0,		//UINT8		GSA;
			0,		//UINT8		Res4;
			
			0,		//UINT8		Res5;
			0,		//UINT8		Res6;
			0,		//UINT8		Res7;
			0,		//UINT8		Res8;
			0,		//UINT8		NovAtel_Rangeb;
			0,		//UINT8		HCPOutputCycle;
			0,		//UINT8		NovAtel_Bestposa;
			0,		//UINT8		NovAtel_Rangea;
			
			0,		//UINT8		NovAtel_Satvis2a;
			0,		//UINT8		NovAtel_Headinga;
			0,		//UINT8		NovAtel_Bestposb;
			0,		//UINT8		NovAtel_Psrdopb;
			0,		//UINT8		Rtcm3_1001;
			0,		//UINT8		Rtcm3_1002;
			0,		//UINT8		Rtcm3_1003;
			0,		//UINT8		Rtcm3_1004;
			
			0,		//UINT8		Rtcm3_1005;
			1,		//UINT8		Rtcm3_1006;
			0,		//UINT8		Rtcm3_1012;
			0,		//UINT8		Rtcm3_1071;
			0,		//UINT8		Rtcm3_1072;
			0,		//UINT8		Rtcm3_1073;
			1,		//UINT8		Rtcm3_1074;
			0,		//UINT8		Rtcm3_1075;
			
			0,		//UINT8		Rtcm3_1076;
			0,		//UINT8		Rtcm3_1077;
			0,		//UINT8		Rtcm3_1081;
			0,		//UINT8		Rtcm3_1082;
			0,		//UINT8		Rtcm3_1083;
			0,		//UINT8		Rtcm3_1084;
			0,		//UINT8		Rtcm3_1085;
			0,		//UINT8		Rtcm3_1086;
			
			0,		//UINT8		Rtcm3_1087;
			0,		//UINT8		Rtcm3_1121;
			0,		//UINT8		Rtcm3_1122;
			0,		//UINT8		Rtcm3_1123;
			1,		//UINT8		Rtcm3_1124;
			0,		//UINT8		Rtcm3_1125;
			0,		//UINT8		Rtcm3_1126;
			0,		//UINT8		Rtcm3_1127;
			
			0,		//UINT8		Rtcm3_1104;
			0,		//UINT8		Rtcm3_1019;
			0,		//UINT8		Rtcm3_1047;
			0,		//UINT8		Res15;
			0,		//UINT8		Res16;
			0,		//UINT8		Res17;
			0,		//UINT8		Res18;
			0,		//UINT8		Res19;
			
			0,		//UINT8		Res31;
			0,		//UINT8		Res32;
			0,		//UINT8		Res33;
			0,		//UINT8		Res34;
			0,		//UINT8		Res35;
			0,		//UINT8		Res36;
			0,		//UINT8		Res37;
			0,		//UINT8		Res38;
		},
		{//COM2
			0,		//UINT8		COMInterFaceMode;
			0,		//UINT8		GGA;
			0,		//UINT8		RMC;
			0,		//UINT8		DHV;
			0,		//UINT8		GSV;
			0,		//UINT8		GLL;
			0,		//UINT8		GSA;
			0,		//UINT8		Res4;
			
			0,		//UINT8		Res5;
			0,		//UINT8		Res6;
			0,		//UINT8		Res7;
			0,		//UINT8		Res8;
			0,		//UINT8		NovAtel_Rangeb;
			0,		//UINT8		HCPOutputCycle;
			0,		//UINT8		NovAtel_Bestposa;
			0,		//UINT8		NovAtel_Rangea;
			
			0,		//UINT8		NovAtel_Satvis2a;
			0,		//UINT8		NovAtel_Headinga;
			0,		//UINT8		NovAtel_Bestposb;
			0,		//UINT8		NovAtel_Psrdopb;
			0,		//UINT8		Rtcm3_1001;
			0,		//UINT8		Rtcm3_1002;
			0,		//UINT8		Rtcm3_1003;
			0,		//UINT8		Rtcm3_1004;
			
			0,		//UINT8		Rtcm3_1005;
			0,		//UINT8		Rtcm3_1006;
			0,		//UINT8		Rtcm3_1012;
			0,		//UINT8		Rtcm3_1071;
			0,		//UINT8		Rtcm3_1072;
			0,		//UINT8		Rtcm3_1073;
			0,		//UINT8		Rtcm3_1074;
			0,		//UINT8		Rtcm3_1075;
			
			0,		//UINT8		Rtcm3_1076;
			0,		//UINT8		Rtcm3_1077;
			0,		//UINT8		Rtcm3_1081;
			0,		//UINT8		Rtcm3_1082;
			0,		//UINT8		Rtcm3_1083;
			0,		//UINT8		Rtcm3_1084;
			0,		//UINT8		Rtcm3_1085;
			0,		//UINT8		Rtcm3_1086;
			
			0,		//UINT8		Rtcm3_1087;
			0,		//UINT8		Rtcm3_1121;
			0,		//UINT8		Rtcm3_1122;
			0,		//UINT8		Rtcm3_1123;
			0,		//UINT8		Rtcm3_1124;
			0,		//UINT8		Rtcm3_1125;
			0,		//UINT8		Rtcm3_1126;
			0,		//UINT8		Rtcm3_1127;
			
			0,		//UINT8		Rtcm3_1104;
			0,		//UINT8		Rtcm3_1019;
			0,		//UINT8		Rtcm3_1047;
			0,		//UINT8		Res15;
			0,		//UINT8		Res16;
			0,		//UINT8		Res17;
			0,		//UINT8		Res18;
			0,		//UINT8		Res19;
			
			0,		//UINT8		Res31;
			0,		//UINT8		Res32;
			0,		//UINT8		Res33;
			0,		//UINT8		Res34;
			0,		//UINT8		Res35;
			0,		//UINT8		Res36;
			0,		//UINT8		Res37;
			0,		//UINT8		Res38;
		},
	},
//串口参数配置
	{
		//COM0
		{
			8,		//UINT8		COMBaudRate;      1,4800bps; 2,9600bps; 3,38400bps; 4,38400bps;  5,57600bps;  6,115200bps;  7,230400bps;  8,460800bps;  9,921600bps;
			1,		//UINT8 		COMStopBit;
			0,		//UINT8		COMParityCheck;
			0,		//UINT8		COMNumDatBits;
			0,		//UINT8 		Res1;
			0,		//UINT8 		Res2;
			0,		//UINT8		Res3;
			0,		//UINT8		Res4;
		},
		//COM1
		{
			6,		//UINT8		COMBaudRate;      1,4800bps; 2,9600bps; 3,38400bps; 4,38400bps;  5,57600bps;  6,115200bps;  7,230400bps;  8,460800bps;  9,921600bps;
			1,		//UINT8 		COMStopBit;
			0,		//UINT8		COMParityCheck;
			0,		//UINT8		COMNumDatBits;
			0,		//UINT8 		Res1;
			0,		//UINT8 		Res2;
			0,		//UINT8		Res3;
			0,		//UINT8		Res4;
		},
		//COM2
		{
			9,		//UINT8		COMBaudRate;      1,4800bps; 2,9600bps; 3,38400bps; 4,38400bps;  5,57600bps;  6,115200bps;  7,230400bps;  8,460800bps;  9,921600bps;  
			1,		//UINT8 		COMStopBit;
			0,		//UINT8		COMParityCheck;
			0,		//UINT8		COMNumDatBits;
			0,		//UINT8 		Res1;
			0,		//UINT8 		Res2;
			0,		//UINT8		Res3;
			0,		//UINT8		Res4;
		},
	},
//网络参数配置表
	{
		{'1','9','2','.','1','6','8','.','1','.','1','0','0','\0'},//  ClientIP;	//接收机IP地址，缺省192.168.1.100
		{'0','\0'},									//ServerIP;	//DHCP服务器的IP地址
		{'1','9','2','.','1','6','8','.','1','.','1','\0'},								//GwP	网关
		{'2','5','5','.','2','5','5','.','2','5','5','.','0','\0'},		//	 NetMask;	//本地网络接口的网络掩码，缺省为255.255.255.0
		{'H','w','A','\0'},	//HostName
		{'H','w','A','\0'},	//Device
		{'H','w','A','\0'},	//DevName
		{' ','\0'},			//IPAndName
		{'H','W','A','1','2','3','\0'},			//PassWord
		
		0,		//Prototype  //0:TCP;1:UDP
		0,			//Res25
		40000,		//Port  网络连接的端口，40000-5000	
		0,			//Res26
		0,			//Res27
	},
};
#endif
boolean checkCPTAvail(SYS_CPT_REGION* p_cpt)
{
	boolean res = FALSE;
	if(p_cpt->pattern[0] == BAKUP_MAGIC_PATTERN1 && p_cpt->pattern[1] == BAKUP_MAGIC_PATTERN2 &&
		p_cpt ->pattern[2] == BAKUP_MAGIC_PATTERN3 && p_cpt->pattern[3] == BAKUP_MAGIC_PATTERN4 && p_cpt->pattern[4] == BAKUP_MAGIC_PATTERN5 )
	{
		const byte *pCPTChkStart = (const byte *)(&(p_cpt->sysm_cpt));
		int32 CPTChkSize = sizeof(SYSM_CPT) ;
		word16 checksum = genChkSum_16bit(pCPTChkStart, CPTChkSize);
		
		if(checksum == p_cpt->checksum)
		{
			if(CheckParam(&(p_cpt->sysm_cpt)))
				res = TRUE;
		}
	}

	return res;
}

void ClearCPTRam(void)
{
	pSharedDDRCPTRegion->checksum=0;
	pSharedDDRCPTRegion->pattern[0]=0;
	pSharedDDRCPTRegion->pattern[1]=0;
	pSharedDDRCPTRegion->pattern[2]=0;
	pSharedDDRCPTRegion->pattern[3]=0;
}

void addCPTVerifyInfo(SYS_CPT_REGION* p_cpt)
{
	addCPTChkSum(p_cpt);
	addCPTPattern(p_cpt);
}

/**
*	append checksum on the end of CPT
*	@return	N/A
*/
void addCPTChkSum(SYS_CPT_REGION* p_CPT)
{
	const byte *pCPTChkStart = (const byte *) (&(p_CPT->sysm_cpt));
	int32 CPTChkSize = sizeof(SYSM_CPT);
	p_CPT->checksum = genChkSum_16bit(pCPTChkStart, CPTChkSize);
	p_CPT->res = 0;
}

/**
*	append magic pattern on the end of CPT
*	@return	N/A
*/
void addCPTPattern(SYS_CPT_REGION* p_CPT)
{
	p_CPT->pattern[0] = BAKUP_MAGIC_PATTERN1;
	p_CPT->pattern[1] = BAKUP_MAGIC_PATTERN2;
	p_CPT->pattern[2] = BAKUP_MAGIC_PATTERN3;
	p_CPT->pattern[3] = BAKUP_MAGIC_PATTERN4;
	p_CPT->pattern[4] = BAKUP_MAGIC_PATTERN5;
}

word16 genChkSum_16bit(const byte* p_data, int32 len)
{
	word16 checksum = 0;
	int32 i = 0;
	
	for(i = 0; i < len; i++)
	{
		checksum += *p_data;
		p_data++;
	}

	return checksum;
}

boolean CheckParam(SYSM_CPT* p_CPT)
{
	byte comid =0;
	if((p_CPT->SysmCptWorkConfig.NavFreqPoint & 0xffff)==0)
		return FALSE;
	//if((p_CPT->RFSampleFrq <10000000)||(p_CPT->RFSampleFrq >80000000))
		//return FALSE;
//	if((p_CPT->PLLFactor_B1< 10.0) || (p_CPT->PLLFactor_L1 <10.0))
//		return FALSE;
	for(comid = 0; comid < MAX_UART_NUM ; comid++)
	{
		if((p_CPT->SysmCptComPort[comid].COMBaudRate> 9) || (p_CPT->SysmCptComPort[comid].COMBaudRate == 0))
			return FALSE;
	}
	
	return TRUE;
}

UINT8 GetCurNavMode(void)
{
	UINT8 navmode = 0;
	UINT16 frqPointMode = pActiveCPT->SysmCptWorkConfig.NavFreqPoint;

	if(frqPointMode & FREQ_GROUP_GPS)
		navmode |= NAV_SYS_GPS;

	if(frqPointMode & FREQ_GROUP_BD)
		navmode |= NAV_SYS_BD;
#if SUPPORT_GLONASS
	if(frqPointMode & FREQ_GROUP_GLO)
		navmode |= NAV_SYS_GLO;
#endif
	return navmode;
}

UINT8 GetNavModByFp(word32 frqpoint)
{
	UINT8 navmode = 0;

	if(frqpoint & FREQ_GROUP_GPS)
		navmode |= NAV_SYS_GPS;

	if(frqpoint & FREQ_GROUP_BD)
		navmode |= NAV_SYS_BD;
#if SUPPORT_GLONASS
	if(frqpoint & FREQ_GROUP_GLO)
		navmode |= NAV_SYS_GLO;
#endif
	return navmode;
}

byte GetSVElMask(int32 sv)
{
	byte elmask=pActiveCPT->SysmCptWorkConfig.ElvMaskSinGPS;
	if(SV_IsGps(sv))
		elmask=pActiveCPT->SysmCptWorkConfig.ElvMaskSinGPS;
	else if(SV_IsBd2(sv))
		elmask=pActiveCPT->SysmCptWorkConfig.ElvMaskSinBD;
#if SUPPORT_GLONASS
	else if(SV_IsGlo(sv))
		elmask=pActiveCPT->SysmCptWorkConfig.ElvMaskSinGLO;
#endif

	return elmask;
}

byte GetSVRTKElMask(int32 sv)
{
	byte elmask=pActiveCPT->SysmCptWorkConfig.ElvMaskRTKGPS;
	if(SV_IsGps(sv))
		elmask=pActiveCPT->SysmCptWorkConfig.ElvMaskRTKGPS;
	else if(SV_IsBd2(sv))
		elmask=pActiveCPT->SysmCptWorkConfig.ElvMaskRTKBD;
#if SUPPORT_GLONASS
	else if(SV_IsGlo(sv))
		elmask=pActiveCPT->SysmCptWorkConfig.ElvMaskRTKGLO;
#endif

	return elmask;
}


bool IsDoubleBranchOpen(void)
{
	bool ret=FALSE;
	
	if((pActiveCPT->SysmCptWorkConfig.FunSwitch & FUNSWITCH_DOUBLEBRANCH) != 0)
		ret = TRUE;

	return ret;
}

bool IsBaseSation(void)
{
	bool ret=FALSE;
	
	if((pActiveCPT->SysmCptWorkConfig.RTDKFlag & RTDKFLAG_STATIONTYPE) != 0)	//Base sation,bit7  1 basestation, 0 mobil receiver
		ret = TRUE;

	return ret;
}

bool IsKfPVTOpen(void)
{
	bool ret=FALSE;
	
	if((pActiveCPT->SysmCptWorkConfig.RTDKFlag & RTDKFLAG_KF) != 0)	//KF module open?  bit0,  0 close, 1 open
		ret = TRUE;

	return ret;
}

bool IsRTDOpen(void)
{
	bool ret=FALSE;
	
	if((pActiveCPT->SysmCptWorkConfig.RTDKFlag & RTDKFLAG_RTD) != 0)	//RTD module open?  bit0,  0 close, 1 open
		ret = TRUE;

	return ret;
}

bool IsRTKOpen(void)
{
	bool ret=FALSE;
	
	if((pActiveCPT->SysmCptWorkConfig.RTDKFlag & RTDKFLAG_RTK) != 0)	//RTK module open?  bit0,  0 close, 1 open
		ret = TRUE;

	return ret;

}

	
int8 GetCPTRcvrMoveState(void)
{
	int8 state=RCVR_MOVESTATE_AUTO;

	state = (pActiveCPT->SysmCptWorkConfig.RTDKFlag>>5)&0x3;
	
	return state;
}

void SetCPTBaseStationInfo(UINT16 stationID, float64 antHeight, ECEF pos)
{
	pActiveCPT->SysmCptWorkConfig.ValidFlag |=CPT_VALIDFLAG_BASEPOS;

	pActiveCPT->SysmCptWorkConfig.BaseStaID = stationID;
	pActiveCPT->SysmCptWorkConfig.AntHeight = (UINT32)(antHeight*1000.0+0.5);	//mm

	pActiveCPT->SysmCptWorkConfig.BaseStaPos_x = pos.x;	//m
	pActiveCPT->SysmCptWorkConfig.BaseStaPos_y = pos.y;	//m
	pActiveCPT->SysmCptWorkConfig.BaseStaPos_z = pos.z;	//m

	return;
}

void SetCPTBaseStationPos(ECEF pos)
{
	pActiveCPT->SysmCptWorkConfig.ValidFlag |=CPT_VALIDFLAG_BASEPOS;
	
	pActiveCPT->SysmCptWorkConfig.BaseStaPos_x = pos.x;
	pActiveCPT->SysmCptWorkConfig.BaseStaPos_y = pos.y;
	pActiveCPT->SysmCptWorkConfig.BaseStaPos_z = pos.z;

	return;
}

void SetCPTBaseStationID(UINT32 stationid)
{
	pActiveCPT->SysmCptWorkConfig.BaseStaID = stationid;

	return;
}

void SetCPTAntHeight(float64 antHeight)
{
	pActiveCPT->SysmCptWorkConfig.AntHeight = (UINT32)(antHeight*1000.0+0.5);	//mm

	return;
}

void ClearCPTBaseStationInfo(void)
{
	pActiveCPT->SysmCptWorkConfig.ValidFlag &=(~CPT_VALIDFLAG_BASEPOS);
	pActiveCPT->SysmCptWorkConfig.AntHeight = 0;
}

bool GetCurCPTSenMinCycle(UINT32* pMinCycleInt,int32* pMinIdx,UINT32* pNewSenCycleInt)
{
	int32 comidx=0, itemidx=0;
	UINT32 minCycle[2] = {10000000,10000000}, curcycle=0;
	UINT32 minValue = 10000000;
	int32 minIdx[2] = {0,0};
	int8 idx = 0;
	int8 fidx = 0;
	bool ret=FALSE;

	int32 fieldOffset=FIELD_OFFSET(SYSM_CPT_PROTOCOL, GGA);
	int32 fieldSize=FIELD_SIZE(SYSM_CPT_PROTOCOL, GGA);

	for(comidx = 0; comidx<MAX_UART_NUM; comidx++)
	{
		for(itemidx=0; itemidx<SYSM_CPT_PROTOCOL_MAX_ITEMCNT; itemidx++)
		{
			curcycle = (*(((UINT8*)(&pActiveCPT->SysmCptProtocol[comidx]))+fieldOffset+itemidx*fieldSize))
						*pActiveCPT->SysmCptWorkConfig.FixUpdateCycle;			
			if(curcycle >0)
			{
				if(fidx < 2)
				{
					minCycle[fidx] = curcycle;
					minIdx[fidx] = itemidx+1;
					fidx++;
				}
				else if(curcycle < minValue)
				{
					idx = minCycle[0]>minCycle[1]?1:0;
					if(idx)
					{
						minCycle[0] = curcycle;
						minIdx[0] = itemidx+1;
					}
					else
					{
						minCycle[1] = curcycle;
						minIdx[1] = itemidx+1;
					}
				}
				minValue = MIN(minCycle[0],minCycle[1]);
				ret = TRUE;
			}
		}
	}
	idx = minCycle[0]<minCycle[1]?0:1;
	if((minCycle[0]!=minCycle[1])&&(minIdx[idx] == (*pMinIdx))&&((*pNewSenCycleInt) > minCycle[idx]))
	{
		if((*pNewSenCycleInt) <= MAX(minCycle[0],minCycle[1]))
			minValue = *pNewSenCycleInt;
		else
			minValue = MAX(minCycle[0],minCycle[1]);
	}
	else
		minValue = MIN(minCycle[0],minCycle[1]);
				
	if((pMinCycleInt != NULL) && (ret==TRUE))
		(*pMinCycleInt) = minValue;
	
	return ret;
}

bool UpdateCPTFixSenCycle(double newSenCycle, int32 comid, int32 newItemOffset)
{
	double rat = 0.0, curCycle;
	int32 comidx=0, itemidx, fieldOffset, fieldSize;
	UINT32 newFixCycleInt, newSenCycleInt, minSenCycleInt;
	UINT8* pItem=NULL;

	//New sentence cycle, unit:TFTcnt
	newSenCycleInt = (UINT32)(newSenCycle*100.0+0.5);	// TFTcnt;

	//Is the cycle we support? 
	if((newSenCycleInt != 5)								// 20Hz
		&&(newSenCycleInt != 10)&&(newSenCycleInt != 20)	// 10Hz, 5Hz
		&&(newSenCycleInt != 50)&&(newSenCycleInt != 100)	// 2Hz, 1Hz
		&&(newSenCycleInt != 200)&&(newSenCycleInt != 500)	// 0.5Hz, 0.2Hz
		&&(newSenCycleInt != 1000)) 						// 0.1Hz
		return FALSE;

	//find the min sentence cycle
	if(GetCurCPTSenMinCycle(&minSenCycleInt,&newItemOffset,&newSenCycleInt))
		minSenCycleInt = MIN(minSenCycleInt, newSenCycleInt);	
	else
		minSenCycleInt = newSenCycleInt;

	if(minSenCycleInt>100)
		newFixCycleInt = 100;	// max fix cycle 1s
	else
		newFixCycleInt = minSenCycleInt;

	// fix cycle not change
	if(newFixCycleInt == pActiveCPT->SysmCptWorkConfig.FixUpdateCycle)	
	{
		//no need to change fix cycle, only change this item
		pItem = ((UINT8*)(&pActiveCPT->SysmCptProtocol[comid])) + newItemOffset;
		
		(*pItem) = newSenCycleInt/pActiveCPT->SysmCptWorkConfig.FixUpdateCycle;
		addCPTVerifyInfo(pActiveCPTRegion);
		return TRUE;
	}

	//fix cycle changed
	fieldOffset=FIELD_OFFSET(SYSM_CPT_PROTOCOL, GGA);
	fieldSize=FIELD_SIZE(SYSM_CPT_PROTOCOL, GGA);

	rat = ((double)pActiveCPT->SysmCptWorkConfig.FixUpdateCycle)/((double)newFixCycleInt);
	for(comidx = 0; comidx<MAX_UART_NUM; comidx++)
	{
		for(itemidx=0; itemidx<SYSM_CPT_PROTOCOL_MAX_ITEMCNT; itemidx++)
		{
			pItem = ((UINT8*)(&pActiveCPT->SysmCptProtocol[comidx])) + fieldOffset + itemidx*fieldSize;
			if((*pItem)>0)
			{
				curCycle = (*pItem)*rat;
				(*pItem) = (UINT8)(curCycle+0.5);
			}
		}
	}

	pItem = ((UINT8*)(&pActiveCPT->SysmCptProtocol[comid])) + newItemOffset;
	(*pItem) = newSenCycleInt/newFixCycleInt;
	pActiveCPT->SysmCptWorkConfig.FixUpdateCycle = newFixCycleInt;

	addCPTVerifyInfo(pActiveCPTRegion);

	return TRUE;
}

#ifndef _POSTPROC
#ifndef _SIMULATE

void SetBaudrate( SYSM_CPT* p_CPT)
{
	byte comid=0;
	int32 userBdIdx=0, curBdIdx=0;
	word32 trueBaudrate=115200;

	for(comid=0; comid<MAX_UART_NUM; comid++)
	{
		
		userBdIdx = p_CPT->SysmCptComPort[comid].COMBaudRate;
		curBdIdx = pActiveCPT->SysmCptComPort[comid].COMBaudRate;
			
		if(userBdIdx != curBdIdx)
		{
			switch(userBdIdx)
			{
			case 1:
				trueBaudrate = 4800;
				break;
			case 2:
				trueBaudrate = 9600;
				break;
			case 3:
				trueBaudrate = 19200;
				break;
			case 4:
				trueBaudrate = 38400;
				break;
			case 5:
				trueBaudrate = 57600;
				break;
			case 6:
				trueBaudrate = 115200;
				break;
			case 7:
				trueBaudrate = 230400;
				break;
			case 8:
				trueBaudrate = 460800;
				break;
			case 9:
				trueBaudrate = 921600;
				break;
			default:
				break;
			}

			SetupUART(comid,trueBaudrate);
			EnableC6747Uart(comid);
		}	
	}
}
#endif
#endif

