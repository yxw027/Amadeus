#ifndef __CFGPARA_H__
#define __CFGPARA_H__


#include "define.h"
#include "coordinate.h"

/************************************************************************/
/* Global data definition														*/
/************************************************************************/

#define BAKUP_PATTERN_NUM						5
#define BAKUP_MAGIC_PATTERN1					0xF0F0F0F0
#define BAKUP_MAGIC_PATTERN2					0xe7ffdefe
#define BAKUP_MAGIC_PATTERN3					0xE7E7E7E7
#define BAKUP_MAGIC_PATTERN4					0x98899889
#define BAKUP_MAGIC_PATTERN5					0xAA5555AA

#define	MAX_PDOP_ALLOW					(20)
#define	MIN_SV_ELV_ALLOW				(5)
#define	MIN_SV_SNR_ALLOW				(4)

#define RCVR_MOVESTATE_AUTO			(0x00)
#define RCVR_MOVESTATE_STATIC		(0x01)
#define RCVR_MOVESTATE_DYNAMIC		(0x02)
#define RCVR_MOVESTATE_HIGHDYNAMIC	(0x03)

#define FUNSWITCH_IONOCORRECT   (0x1)
#define FUNSWITCH_TROPOCORRECT   (0x1<<1)
#define FUNSWITCH_SMOOTH		 (0x1<<2)
#define FUNSWITCH_STATICNAV     (0x1<<3)
#define FUNSWITCH_EXTCLOCK          (0x1<<4)
#define FUNSWITCH_DOUBLEBRANCH   (0x1<<5)
#define FUNSWITCH_RTKCORRECT   	 (0x1<<6)
#define FUNSWITCH_RESBIT7   	 (0x1<<7)

#define RTDKFLAG_KF					(0x1)		//bitmap for KF PVT, 0: close; 1: open
#define RTDKFLAG_RTD				(0x1<<1)	//bitmap for RTD, 0: close; 1: open
#define RTDKFLAG_RTK				(0x1<<2)	//bitmap for RTK, 0: close; 1: open
#define RTDKFLAG_ORI				(0x1<<3)	//bitmap for ���嶨��, 0: close; 1: open
#define RTDKFLAG_ORA				(0x1<<4)	//bitmap for �������, 0: close; 1: open
#define RTDKFLAG_AUTO				(RCVR_MOVESTATE_AUTO<<5)
#define RTDKFLAG_STATICRTK			(RCVR_MOVESTATE_STATIC<<5)
#define RTDKFLAG_DYNAMICRTK			(RCVR_MOVESTATE_DYNAMIC<<5)
#define RTDKFLAG_HIGHDYNAMICRTK		(RCVR_MOVESTATE_HIGHDYNAMIC<<5)
#define RTDKFLAG_STATIONTYPE		(0X1<<7)	//value 0, rove station; 1, base station

#define CPT_VALIDFLAG_BASEPOS			(0x1)	//bitmap for base station postion valid or not. 0, invalid; 1, valid

//ϵͳ������������
typedef struct{
	UINT8		Command;				//�û�����
	UINT8		RTDKFlag;				//����bitλ�μ�Э��
	UINT8		RTKTimeOut; 			//����վ����ʱ���ӳ�����
	UINT8		FunSwitch;				//���ܿ���
	UINT8		FixUpdateCycle; 	//��λ��������10MS
	UINT8		PdopMask;			   // PDOP����ֵ�趨
	UINT8		SnrMaskSingle;			//���㶨λ���������ֵ�趨
	UINT8		SnrMaskRTK; 			//RTK���������ֵ�趨
	
	UINT8		ElvMaskRTKGPS;			//�߶Ƚ�����ֵ�趨GPSRTK ����90���������
	UINT8		ElvMaskRTKBD;			//�߶Ƚ�����ֵ�趨BDRTK ����90���������
	UINT8		ElvMaskRTKGLO;			//�߶Ƚ�����ֵ�趨GLORTK ����90���������
	UINT8		ElvMaskRTKGAL;			//�߶Ƚ�����ֵ�趨GALRTK ����90���������
	UINT8		ElvMaskSinGPS;			//�߶Ƚ�����ֵ�趨GPSSIN ����90���������	
	UINT8		ElvMaskSinBD;			//�߶Ƚ�����ֵ�趨BDSIN ����90���������
	UINT8		ElvMaskSinGLO;			//�߶Ƚ�����ֵ�趨GLOSIN ����90���������
	UINT8		ElvMaskSinGAL;			//�߶Ƚ�����ֵ�趨GALSIN ����90���������

	UINT32		BaseStaID;			//	��׼վID
	UINT8		ValidFlag;			//Bit0: ��׼վλ���Ƿ���Ч����Ԥ��
	int8		LeapSec;				//Ԥ��
	UINT16		RTCMDelay;				//

	double		BaseStaPos_x; 	//m, Base station pos x
	double		BaseStaPos_y; 	//m, Base station pos y
	double		BaseStaPos_z; 	//m, Base station pos z

	UINT32		AntHeight;			//���߸߶�
	UINT32		RFSampleFrq;		//��Ƶ����Ƶ��

	UINT32		Delay1PPS;			   // 1PPS�̶��ӳ�, uints: ns
	UINT32		PulsWidth;			   // 1PPS������, ms

	UINT32		FixNavFreqPoint;			//�û�����ģʽ
	byte		MaxRtkSvCnt;
	byte		Res2;
	UINT16		Res22;				//Ԥ��

	word256 	SVBitMap;			//bit set to 1 means that sv is valid, others invalid.

	double		IF_L1;					// L1Ƶ����Ƶ
	double		IF_L2;
	double		IF_L5;	
	double		IF_B1;	
	double		IF_B2;	
	double		IF_B3;	
	double		IF_G1;	
	double		IF_G2;	
	double		IF_Galilo;

	int16		Delay_L1;  // L1Ƶ���ӳ�units:ns
	int16		Delay_L2;  // L2Ƶ���ӳ�units:ns
	int16		Delay_L5;  // L5Ƶ���ӳ�units:ns
	int16		Delay_B1;  // B1Ƶ���ӳ�units:ns
	
	int16		Delay_B2;  // B2Ƶ���ӳ�units:ns
	int16		Delay_B3;  // B3Ƶ���ӳ�units:ns	
	int16		Delay_G1;  // G1Ƶ���ӳ�units:ns	
	int16		Delay_G2;  // G2Ƶ���ӳ�units:ns

	int16		Delay_Galilo;  // Galilo Ƶ���ӳ�units:ns
	int16		Res28;	// Ԥ��
	int16		Res29;	// Ԥ��
	int16		Res30;	// Ԥ��
	
	UINT16		NavFreqPoint;			   // �û����õ�ģʽ	
	UINT16		Res40;						//Ԥ��
	UINT32		Res41;					//Ԥ��

	word64		NavFreqPoint_Res;	   //�û����õ�ģʽ ��ʱԤ��
}SYSM_CPT_WORKCONFIG;

#define SYSM_CPT_PROTOCOL_MAX_ITEMCNT	63
//Э���������
typedef struct{
	UINT8		COMInterFaceMode;  //������֧�ֵ�Э�鿪��bit0-4 NEMA HCP RMO RTCM3 NOVTEL
	UINT8		GGA;
	UINT8		RMC;
	UINT8		DHV;
	UINT8		GSV;
	UINT8		GLL;
	UINT8		GSA;
	UINT8		Res4;
	
	UINT8		Res5;
	UINT8		Res6;
	UINT8		Res7;
	UINT8		Res8;
	UINT8		NovAtel_Rangeb;
	UINT8		HCPOutputCycle;
	UINT8		NovAtel_Bestposa;
	UINT8		NovAtel_Rangea;
	
	UINT8		NovAtel_Satvis2a;
	UINT8		NovAtel_Headinga;
	UINT8		NovAtel_Bestposb;
	UINT8		NovAtel_Psrdopb;
	UINT8		Rtcm3_1001;
	UINT8		Rtcm3_1002;
	UINT8		Rtcm3_1003;
	UINT8		Rtcm3_1004;
	
	UINT8		Rtcm3_1005;
	UINT8		Rtcm3_1006;
	UINT8		Rtcm3_1012;
	UINT8		Rtcm3_1071;
	UINT8		Rtcm3_1072;
	UINT8		Rtcm3_1073;
	UINT8		Rtcm3_1074;
	UINT8		Rtcm3_1075;
	
	UINT8		Rtcm3_1076;
	UINT8		Rtcm3_1077;
	UINT8		Rtcm3_1081;
	UINT8		Rtcm3_1082;
	UINT8		Rtcm3_1083;
	UINT8		Rtcm3_1084;
	UINT8		Rtcm3_1085;
	UINT8		Rtcm3_1086;
	
	UINT8		Rtcm3_1087;
	UINT8		Rtcm3_1121;
	UINT8		Rtcm3_1122;
	UINT8		Rtcm3_1123;
	UINT8		Rtcm3_1124;
	UINT8		Rtcm3_1125;
	UINT8		Rtcm3_1126;
	UINT8		Rtcm3_1127;
	
	UINT8		Rtcm3_1104;
	UINT8		Rtcm3_1019;
	UINT8		Rtcm3_1047;
	UINT8		Res15;
	UINT8		Res16;
	UINT8		Res17;
	UINT8		Res18;
	UINT8		Res19;
	
	UINT8		Res31;
	UINT8		Res32;
	UINT8		Res33;
	UINT8		Res34;
	UINT8		Res35;
	UINT8		Res36;
	UINT8		Res37;
	UINT8		Res38;
}SYSM_CPT_PROTOCOL;

//���ڲ�������
typedef struct{
	UINT8		COMBaudRate; //1,4800bps; 2,9600bps; 3,38400bps; 4,38400bps;  5,57600bps;  6,115200bps;  7,230400bps;  8,460800bps;  9,921600bps;   
	UINT8 		COMStopBit;
	UINT8		COMParityCheck;
	UINT8		COMNumDatBits;
	UINT8 		Res1;
	UINT8 		Res2;
	UINT8		Res3;
	UINT8		Res4;	
}SYSM_CPT_COMPORT;

//����������ñ�
typedef struct{
	C16     ClientIP;	//���ջ�IP��ַ��ȱʡ192.168.1.100
	C16     ServerIP;	//DHCP��������IP��ַ
	C16     GwP;		//����
	C16     NetMask;	//��������ӿڵ��������룬ȱʡΪ255.255.255.0
	C16 	HostName;	//���ջ���������Ϊָ���򲻻ᴥ���Զ�����
	C16		Device;		//��ʹ�õ������豸����
	C16 	DevName;	//�忨������ͨѶ�豸����
	C16		IPAndName;	//�ڿͻ���ģʽ�£�ΪԶ�̷�������IP��ַ��������//�ڷ�����ģʽ������ҪIP����������
	C16		PassWord;	//���룬���ʽ��ջ�ʱ��Ҫ��֤������

	UINT8		Prototype;	//0:TCP;1:UDP
	UINT8		Res25	;   //Ԥ��
	UINT16		Port	;	//�������ӵĶ˿ڣ�40000-50000
	UINT16		Res26	;   //Ԥ��
	UINT16		Res27	;   //Ԥ��	
} SYSM_CPT_NETWORK;

typedef struct{
	SYSM_CPT_WORKCONFIG		SysmCptWorkConfig;
	SYSM_CPT_PROTOCOL		SysmCptProtocol[MAX_UART_NUM];
	SYSM_CPT_COMPORT		SysmCptComPort[MAX_UART_NUM];
	SYSM_CPT_NETWORK		SysmCptNetWork;
} SYSM_CPT;


typedef struct{
	SYSM_CPT sysm_cpt;
	word16 checksum;
	word16 res;		// for memory allign
	word32 pattern[BAKUP_PATTERN_NUM];	
} SYS_CPT_REGION;


#define MANUFACTURER_INFO_LENS		21
#define MODULE_TYPE_LENS			21
#define MODULE_ID_LENS				33


/************************************************************************/
/* Define interface															*/
/************************************************************************/
#ifdef __cplusplus
extern "C" {
#endif

extern const SYSM_CPT FactoryCPT;

extern SYS_CPT_REGION *pSharedDDRCPTRegion;
extern SYS_CPT_REGION *pActiveCPTRegion;
extern SYSM_CPT *pActiveCPT;
extern SYS_CPT_REGION ActiveCPTRegion;

extern boolean checkCPTAvail(SYS_CPT_REGION* p_cpt);
extern void addCPTVerifyInfo(SYS_CPT_REGION* p_cpt);
extern boolean checkCPTAvail(SYS_CPT_REGION* p_cpt);
extern void addCPTPattern(SYS_CPT_REGION* p_CPT);
extern void addCPTChkSum(SYS_CPT_REGION* p_CPT);
extern word16 genChkSum_16bit(const byte* p_data, int32 len);
extern boolean  CheckParam(SYSM_CPT* p_CPT);
extern void SetBaudrate(SYSM_CPT* p_CPT);
extern UINT8 GetCurNavMode(void);
extern UINT8 GetNavModByFp(word32 frqpoint);
extern byte GetSVElMask(int32 sv);
extern byte GetSVRTKElMask(int32 sv);
extern bool IsDoubleBranchOpen(void);
extern bool IsBaseSation(void);
extern bool IsKfPVTOpen(void);
extern bool IsRTDOpen(void);
extern bool IsRTKOpen(void);
extern int8 GetCPTRcvrMoveState(void);
extern void SetCPTBaseStationPos(ECEF pos);
extern void ClearCPTRam(void);

bool GetCurCPTSenMinCycle(UINT32* pMinCycleInt,int32* pMinIdx,UINT32* pNewSenCycleInt);
extern bool UpdateCPTFixSenCycle(double newSenCycle, int32 comid, int32 newItemOffset);
#ifdef __cplusplus
}
#endif

#endif

