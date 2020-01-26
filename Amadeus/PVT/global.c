
#include "typedefine.h"
#include "define.h"
#include "global.h"
#include "constdef.h"
#include "cfgpara.h"
#include "hcp.h"
#include "pvt.h"
#include "postproc.h"
#include "nmea0183.h"
#include "UpdateInfo.h"
#include "rtk.h"
#ifndef _POSTPROC
#ifndef _SIMULATE
#include "GnssTYProcfg.h"
#include "UartFunc.h"
#include "sysbackup.h"
#include "FpgaReg.h"
#include "led.h"
#endif
#endif

STRU_ACQU glStruAcqu;
SV_INFO SVInfo[SV_NUM];

STRU_UART_STREAM glStruUart[MAX_UART_NUM];

char VersionInfo[50]="C1610 V0.62 20170830\r\n";
void (*EntryPoint)(void);


//RF define
static double IF_L1=0.0,IF_L2=0.0,IF_L5=0.0,IF_B1=0.0,IF_B2=0.0,IF_B3=0.0;
static double PLL_FACTOR_L1,PLL_FACTOR_L2,PLL_FACTOR_L5,PLL_FACTOR_B1,PLL_FACTOR_B2,PLL_FACTOR_B3;
#if SUPPORT_GLONASS
static double IF_G1=0.0,IF_G2=0.0,PLL_FACTOR_G1,PLL_FACTOR_G2;
#endif
double RF_SAMP_FREQ = 50000000.0;		//RF sample frequency
double RF_SAMP_FREQ100M = 100000000.0;		//RF sample frequency


double RECIP_RF_SAMP_FREQ = 2.0e-8;
int32  RF_SAMP_FREQ_HALF_INT =  25000000;	//half of RF sample frequency
int32  RF_SAMP_FREQ_INT =  50000000;
#if __X2240_SYSTEM
int32  LOW_32BIT_INVERT	= 80;
#else
int32  LOW_32BIT_INVERT	= 40;
#endif

int32 RFCNT_HIGH_REVERSE_TIME = 86;	//s

//TFT intterrupt cnt
UINT32 TFT_OP_CNT=0;
//Task op cycle define
#define TASK1_GRP_NUM	8

#ifndef _POSTPROC
#ifndef _SIMULATE
sys_task_rec TSK1_List[TASK1_GRP_NUM]=
{
	{TaskPVT, 				100},
	{TaskPostProc, 			100},
	{TaskRTK,				100},
	{TaskUpdateInfo, 		100},
	{UpdateACQMan, 			100},
	{TaskPeriodicOutput, 	100},
	//{TaskSysBackup, 		400},	
	{NULL, 					0},
};

word64 TSK1_LastOpTFTcnt[TASK1_GRP_NUM]={{0,0},{0,0},{0,0},{0,0},{0,0},{0,0},{0,0},{0,0}};

void UpdateTask1_List_Cycle(void)
{
	TSK1_List[0].cycle = pActiveCPT->SysmCptWorkConfig.FixUpdateCycle; //cycle =  pActiveCPT->SysmCptWorkConfig.FixUpdateCycle*10ms
	TSK1_List[1].cycle = pActiveCPT->SysmCptWorkConfig.FixUpdateCycle;
	TSK1_List[2].cycle = pActiveCPT->SysmCptWorkConfig.FixUpdateCycle;
	TSK1_List[3].cycle = pActiveCPT->SysmCptWorkConfig.FixUpdateCycle;
	TSK1_List[4].cycle = pActiveCPT->SysmCptWorkConfig.FixUpdateCycle;
	TSK1_List[5].cycle = pActiveCPT->SysmCptWorkConfig.FixUpdateCycle;

	return;
}

void Task1_PVT(void)
{
	/*int32 i=0;
	static word64 curTFTcnt={{0,0}};
	int32 difTFTcnt=0;
	static bool bModifiedTFT=FALSE;
	float64 fvalue=0.0;
	
	while(1)
	{
		SEM_pend(&SEM4,SYS_FOREVER);

		UpdateTask1_List_Cycle();

		if((curTFTcnt.wd[0]==0) && (curTFTcnt.wd[1]==0))
			curTFTcnt.wd[0] = TFT_OP_CNT;
		else
		{
			if(TFT_OP_CNT < curTFTcnt.wd[0])
				curTFTcnt.wd[1]++;

			curTFTcnt.wd[0]=TFT_OP_CNT;
		}

		if((!bModifiedTFT) && (gobsTFTIdx>=1) && (TSK1_List[0].cycle > 3)&&(gobsTFTIdx<TSK1_List[0].cycle))
		{
			for(i=0; i<TASK1_GRP_NUM; i++)
			{
				if(TSK1_List[i].task_ptr == NULL)
					continue;

				H32L32ToF64(&fvalue,TSK1_LastOpTFTcnt[i].wd[1],TSK1_LastOpTFTcnt[i].wd[0]);
				if(gobsTFTIdx<TSK1_List[0].cycle/2)
					fvalue +=gobsTFTIdx;
				else
					fvalue -=(TSK1_List[0].cycle-gobsTFTIdx);
				F64ToH32L32(fvalue,&(TSK1_LastOpTFTcnt[i].wd[1]),&(TSK1_LastOpTFTcnt[i].wd[0]));
			}

			bModifiedTFT=TRUE;
		}
		gobsTFTIdx=0;
		
		for(i=0; i<TASK1_GRP_NUM; i++)
		{
			if(TSK1_List[i].task_ptr == NULL)
				continue;

			difTFTcnt = (int32)(((double)(curTFTcnt.wd[1])-(double)(TSK1_LastOpTFTcnt[i].wd[1]))*4294967296+((double)curTFTcnt.wd[0]-(double)(TSK1_LastOpTFTcnt[i].wd[0])+0.5));
				
			if(difTFTcnt >= TSK1_List[i].cycle)
			{
				TSK1_LastOpTFTcnt[i] = curTFTcnt;
				(*TSK1_List[i].task_ptr)();

				if(i==0)
				{
					if(bModifiedTFT)
					{
						gobsTFTIdx=0;
						bModifiedTFT = FALSE;
					}
					Led();
				}
			}
		}

		TSK_yield();
	}*/
}



void Task4_UartRecv_Debug0(void)
{
	//while(1)
	//{
	//	SEM_pend(&SEM1,SYS_FOREVER);
	//	TaskProtocolInput(&(glStruUart[COM_ID_DEBUG0]));
	//	TSK_yield();
	//}
}

void Task5_UartSend_Debug1(void)
{
	//while(1)
	//{
	//	SEM_pend(&SEM6,SYS_FOREVER);
	//	UartSend(COM_ID_DEBUG1);
	//	TSK_yield();
	//}
}

void Task6_UartSend_Debug0(void) 
{
	//while(1)
	//{
	//	SEM_pend(&SEM3,SYS_FOREVER);
	//	UartSend(COM_ID_DEBUG0);
	//	TSK_yield();
	//}
}
void Task8_UartRecv_Debug1(void)
{
	//while(1)
	//{
	//	SEM_pend(&SEM7,SYS_FOREVER);
	//	TaskProtocolInput(&(glStruUart[COM_ID_DEBUG1]));
	//	TSK_yield();
	//}
}

void Task7_Flash_BackUp(void)
{
	//while(1)
	//{
	//	SEM_pend(&SEM5,SYS_FOREVER);
	//	TaskOpFlash();
	//	TSK_yield();
	//}
}

void ResetSystem(void)
{
	//_disable_interrupts();

	//TimerWatchdogForbid(TIMER1_BASE);                             
	//TimerSetUpWatchDog(0x50,0x0);
	//TimerWatchdogActivate(TIMER1_BASE);
	//while(1);
}

#endif
#endif

void InitGlobalData(void)
{
	int16 trkch=0, svid=0;
	int16 comid = 0;
	
	memset(&glStruAcqu, 0, sizeof(glStruAcqu));

#ifndef _SIMULATE
	for(trkch=0; trkch<MAXCHANNELS; trkch++)
	{
		LOOP_loselock(trkch,FALSE);
	}
#endif
	memset(SVInfo, 0, sizeof(SVInfo));
	for(svid=0; svid<SV_NUM; svid++)
	{
		SVInfo[svid].eph_age = SV_EPH_ALM_AGE_INVALID;
		SVInfo[svid].alm_age = SV_EPH_ALM_AGE_INVALID;
		SVInfo[svid].el = SV_ELEVATION_INVALID;
	}

	memset(glStruUart,0,sizeof(glStruUart));
	for(comid = 0;comid < MAX_UART_NUM; comid++)
	{
		glStruUart[comid].comid = comid;	
	}
	memset(&glStruObs,0,sizeof(glStruObs));

	return;
}

void ResetSVInfo(void)
{
	int32 svid = 0;
	for(svid=0; svid<SV_NUM; svid++)
	{
		SVInfo[svid].eph_age = SV_EPH_ALM_AGE_INVALID;
		SVInfo[svid].alm_age = SV_EPH_ALM_AGE_INVALID;
		SVInfo[svid].swdopSrc = NO_DOPPLER;
		SVInfo[svid].el = SV_ELEVATION_INVALID;
		SVInfo[svid].az = 0;
	}
}

void InitSysIFParam(void)
{
	RF_SAMP_FREQ = pActiveCPT->SysmCptWorkConfig.RFSampleFrq;
	RECIP_RF_SAMP_FREQ = 1.0/RF_SAMP_FREQ;
	RF_SAMP_FREQ_HALF_INT = (int32)(RF_SAMP_FREQ*0.5+0.5);//half of RF sample frequency
	RF_SAMP_FREQ_INT = (int32)(RF_SAMP_FREQ+0.5);
	RFCNT_HIGH_REVERSE_TIME = (int32)(TWO_P32*RECIP_RF_SAMP_FREQ); //s

	IF_L1 = pActiveCPT->SysmCptWorkConfig.IF_L1;
	IF_L2 = pActiveCPT->SysmCptWorkConfig.IF_L2;
	IF_L5 = pActiveCPT->SysmCptWorkConfig.IF_L5;
	IF_B1 = pActiveCPT->SysmCptWorkConfig.IF_B1;
	IF_B2 = pActiveCPT->SysmCptWorkConfig.IF_B2;
	IF_B3 = pActiveCPT->SysmCptWorkConfig.IF_B3;
#if SUPPORT_GLONASS
	IF_G1 = pActiveCPT->SysmCptWorkConfig.IF_G1;
	IF_G2 = pActiveCPT->SysmCptWorkConfig.IF_G2;	
#endif

#if 0
	PLL_FACTOR_L1 = (L1_FREQUENCE - IF_L1)*RECIP_RF_SAMP_FREQ;
	PLL_FACTOR_L2 = (L2_FREQUENCE - IF_L2)*RECIP_RF_SAMP_FREQ;
	PLL_FACTOR_L5 = (L5_FREQUENCE - IF_L5)*RECIP_RF_SAMP_FREQ;
	PLL_FACTOR_B1 = (B1_FREQUENCE - IF_B1)*RECIP_RF_SAMP_FREQ;
	PLL_FACTOR_B2 = (B2_FREQUENCE - IF_B2)*RECIP_RF_SAMP_FREQ;
	PLL_FACTOR_B3 = (B3_FREQUENCE - IF_B3)*RECIP_RF_SAMP_FREQ;
#if SUPPORT_GLONASS
	PLL_FACTOR_G1 = (G1_FREQUENCE - IF_G1)*RECIP_RF_SAMP_FREQ;
	PLL_FACTOR_G2 = (G2_FREQUENCE - IF_G2)*RECIP_RF_SAMP_FREQ;
#endif
#else
	PLL_FACTOR_L1 = (L1_FREQUENCE )*RECIP_RF_SAMP_FREQ;
	PLL_FACTOR_L2 = (L2_FREQUENCE )*RECIP_RF_SAMP_FREQ;
	PLL_FACTOR_L5 = (L5_FREQUENCE )*RECIP_RF_SAMP_FREQ;
	PLL_FACTOR_B1 = (B1_FREQUENCE )*RECIP_RF_SAMP_FREQ;
	PLL_FACTOR_B2 = (B2_FREQUENCE )*RECIP_RF_SAMP_FREQ;
	PLL_FACTOR_B3 = (B3_FREQUENCE )*RECIP_RF_SAMP_FREQ;
#if SUPPORT_GLONASS
	PLL_FACTOR_G1 = (G1_FREQUENCE )*RECIP_RF_SAMP_FREQ;
	PLL_FACTOR_G2 = (G2_FREQUENCE )*RECIP_RF_SAMP_FREQ;
#endif

#endif
	return;
}

void RestGlStruAcquChInfo(int32 trkch)
{
	memset(&(glStruAcqu.Info[trkch]), 0, sizeof(glStruAcqu.Info[0]));
	glStruAcqu.Info[trkch].SvID = -1;
}

int32 GetTRKChSvid(int32 trkch)
{
	return (glStruAcqu.Info[trkch].SvID);
}

UINT16 GetTRKChFrepoint(int32 trkch)
{
	return (glStruAcqu.Info[trkch].FreqPoint);
}


void Clear_LOOP(int32 trkch)
{
#ifndef _POSTPROC
#ifndef _SIMULATE
	//set to Nios, make nios trk channel reset.
	WriteTrkchLoseLock(trkch);
#endif
#endif
	return;
}


void LOOP_loselock(int32 trkch, boolean bCfgToBB)
{
	if((glStruAcqu.Info[trkch].FreqPoint == DSP_L5_FRE) &&(glStruAcqu.Info[trkch].branch == BRANCH_DATA)
		&& (glStruAcqu.Info[trkch+1].FreqPoint == DSP_L5_FRE)&& (glStruAcqu.Info[trkch+1].branch == BRANCH_PILOT))
	{//for pilot track
	#ifndef _POSTPROC
	#ifndef _SIMULATE
		ResetTrkchLastObs(trkch+1);
		ResetBitSync(trkch+1);
		ResetSfSync(trkch+1);
	#endif
	#endif
		RestGlStruAcquChInfo(trkch+1);
		
		//set to Nios
		if(bCfgToBB)
			Clear_LOOP(trkch+1);
	}
	#ifndef _POSTPROC
	#ifndef _SIMULATE
	ResetTrkchLastObs(trkch);
	ResetBitSync(trkch);
	ResetSfSync(trkch);
	#endif
	#endif
	RestGlStruAcquChInfo(trkch);
	
	//set to Nios
	if(bCfgToBB)
		Clear_LOOP(trkch);
	
	return;
}


void Clr_LOOP_ISR_Info(int32 trkch)
{
	if((glStruAcqu.Info[trkch].FreqPoint == DSP_L5_FRE) &&(glStruAcqu.Info[trkch].branch == BRANCH_DATA)
		&& (glStruAcqu.Info[trkch+1].FreqPoint == DSP_L5_FRE)&& (glStruAcqu.Info[trkch+1].branch == BRANCH_PILOT))
	{//for pilot track
	#ifndef _POSTPROC
	#ifndef _SIMULATE
		ResetTrkchLastObs(trkch+1);
		ResetBitSync(trkch+1);
//		ResetSfSync(trkch+1);
	#endif
	#endif
	}
	#ifndef _POSTPROC
	#ifndef _SIMULATE
	ResetTrkchLastObs(trkch);
	ResetBitSync(trkch);
//	ResetSfSync(trkch);
	#endif
	#endif
	return;
}


double GetFreqpointIF(int16 svid, UINT16 frepoint)
{
	double iIFFreq = IF_L1;

	if(frepoint & FREQ_GROUP_L1)
		iIFFreq = IF_L1;
	else if(frepoint & FREQ_GROUP_L2)
		iIFFreq = IF_L2;
	else if(frepoint & FREQ_GROUP_L5)
		iIFFreq = IF_L5;
	else if(frepoint & FREQ_GROUP_B1)
		iIFFreq = IF_B1;
	else if(frepoint & FREQ_GROUP_B2)
		iIFFreq = IF_B2;
	else if(frepoint & FREQ_GROUP_B3)
		iIFFreq = IF_B3;
#if SUPPORT_GLONASS
	else if(frepoint & FREQ_GROUP_G1)
		iIFFreq = IF_G1+(svid-MinGloFreID-7)*G1_DETA_FREQUENCE;
	else if(frepoint & FREQ_GROUP_G2)
		iIFFreq = IF_G2+(svid-MinGloFreID-7)*G2_DETA_FREQUENCE;
#endif
	return iIFFreq;
}

double GetFreqpointRF(int16 svid, UINT16 frepoint)
{
	double iRFFreq = L1_FREQUENCE;
	if(SV_IsGps(svid))
		frepoint &= FREQ_GROUP_GPS;
	else if(SV_IsBd2(svid))
		frepoint &= FREQ_GROUP_BD;
#if SUPPORT_GLONASS
	else if(SV_IsGlo(svid))
		frepoint &= FREQ_GROUP_GLO;
#endif

	if(frepoint & FREQ_GROUP_L1)
		iRFFreq = L1_FREQUENCE;
	else if(frepoint & FREQ_GROUP_L2)
		iRFFreq = L2_FREQUENCE;
	else if(frepoint & FREQ_GROUP_L5)
		iRFFreq = L5_FREQUENCE;
	else if(frepoint & FREQ_GROUP_B1)
		iRFFreq = B1_FREQUENCE;
	else if(frepoint & FREQ_GROUP_B2)
		iRFFreq = B2_FREQUENCE;
	else if(frepoint & FREQ_GROUP_B3)
		iRFFreq = B3_FREQUENCE;
#if SUPPORT_GLONASS
	else if(frepoint & FREQ_GROUP_G1)
		iRFFreq = G1_FREQUENCE+(svid-MinGloFreID-7)*G1_DETA_FREQUENCE;
	else if(frepoint & FREQ_GROUP_G2)
		iRFFreq = G2_FREQUENCE+(svid-MinGloFreID-7)*G2_DETA_FREQUENCE;
#endif
	return iRFFreq;
}


double GetFreqpointPLL(UINT16 frepoint)
{
	double iPLL = PLL_FACTOR_L1;
	
	if(frepoint & FREQ_GROUP_L1)
		iPLL = PLL_FACTOR_L1;
	else if(frepoint & FREQ_GROUP_L2)
		iPLL = PLL_FACTOR_L2;
	else if(frepoint & FREQ_GROUP_L5)
		iPLL = PLL_FACTOR_L5;
	else if(frepoint & FREQ_GROUP_B1)
		iPLL = PLL_FACTOR_B1;
	else if(frepoint & FREQ_GROUP_B2)
		iPLL = PLL_FACTOR_B2;
	else if(frepoint & FREQ_GROUP_B3)
		iPLL = PLL_FACTOR_B3;
#if SUPPORT_GLONASS
	else if(frepoint & FREQ_GROUP_G1)
		iPLL = PLL_FACTOR_G1;
	else if(frepoint & FREQ_GROUP_G2)
		iPLL = PLL_FACTOR_G2;
#endif
	return iPLL;
}

bool GetSVTrkchID(int16 svid, int16 trkch[MAX_TRKCHCNT_PER_ONESV])
{
	bool ret = FALSE;
	int16 i=0, idx=0;
	for(i=0; i<MAX_TRKCHCNT_PER_ONESV; i++)
		trkch[i]=-1;

	for(i=0; i<MAXCHANNELS; i++)
	{	
		if((svid == glStruAcqu.Info[i].SvID) && (glStruAcqu.Info[i].State >= TRACKING_PVT))
		{
			ret = TRUE;
			trkch[idx] = i;
			idx++;
			
			if(idx>=MAX_TRKCHCNT_PER_ONESV)
				break;
		}
	}

	return ret;
}

bool IsTrkChValid(int32 trkch)
{
	if((trkch >= 0) && (trkch < MAXCHANNELS))
		return TRUE;
	else
		return FALSE;
}

double GetSVWaveLen(int16 svid, UINT16 freqpoint)
{
	double wavelen=B1_WAVELENGTH;
	if(freqpoint & FREQ_GROUP_B1)
		wavelen = B1_WAVELENGTH;
	else if(freqpoint & FREQ_GROUP_B2)
		wavelen = B2_WAVELENGTH;
	else if(freqpoint & FREQ_GROUP_B3)
		wavelen = B3_WAVELENGTH;
	else if(freqpoint & FREQ_GROUP_L1)
		wavelen = L1_WAVELENGTH;
	else if(freqpoint & FREQ_GROUP_L2)
		wavelen = L2_WAVELENGTH;
	else if(freqpoint & FREQ_GROUP_L5)
		wavelen = L5_WAVELENGTH;
#if SUPPORT_GLONASS
	else if(freqpoint & FREQ_GROUP_G1)
		wavelen = G1_WAVELENGTH[svid-MinGloFreID];
	else if(freqpoint & FREQ_GROUP_G2)
		wavelen = G2_WAVELENGTH[svid-MinGloFreID];
#endif
	return wavelen;
}

int8 GetFreGropIdx(UINT16 frepoint)
{
	int8 idx=0;
	
	if((frepoint & FREQ_GROUP_IDX0) && (MAX_FREPIONT_PER_NAVSYS>=1))
		idx = 0;
	else if((frepoint & FREQ_GROUP_IDX1) && (MAX_FREPIONT_PER_NAVSYS>=2))
		idx = 1;
	else if((frepoint & FREQ_GROUP_IDX2) && (MAX_FREPIONT_PER_NAVSYS>=3))
		idx = 2;
	
	return idx;
}

UINT16 GetFrepointByFPGrpidx(int32 svid, int8 idx)
{
	UINT16 frepoint=0;

	if(SV_IsGps(svid))
	{
		if(0==idx)
			frepoint = FREQ_GROUP_L1;
		else if(1 == idx)
			frepoint = FREQ_GROUP_L2;
		else if(2 == idx)
			frepoint = FREQ_GROUP_L5;
		else
			return 0;	
	}
	else if(SV_IsBd2(svid))
	{
		if(0==idx)
			frepoint = FREQ_GROUP_B1;
		else if(1 == idx)
			frepoint = FREQ_GROUP_B2;
		else if(2 == idx)
			frepoint = FREQ_GROUP_B3;
		else
			return 0;
	}
#if SUPPORT_GLONASS
	else if(SV_IsGlo(svid))
	{
		if(0==idx)
			frepoint = FREQ_GROUP_G1;
		else if(1 == idx)
			frepoint = FREQ_GROUP_G2;
		else
			return 0;
	}
#endif
	else
		return 0;

	return frepoint;		
}


int32 CalcFPointGroupCnt(word32 frepoint)
{
	int cnt=0;
	if(frepoint & FREQ_GROUP_IDX0)
		cnt++;

	if(frepoint & FREQ_GROUP_IDX1)
		cnt++;


	if(frepoint & FREQ_GROUP_IDX2)
		cnt++;


	return cnt;
}


int32 GetNavSysIdx(int32 svid)
{
	int32 idx=-1;
	
	if(SV_IsGps(svid))
		idx = 0;
	else if(SV_IsBd2(svid))
		idx = 1;
#if SUPPORT_GLONASS
	else if(SV_IsGlo(svid))
		idx = 2;
#endif

	return idx;
}

word32 GetNavSysFPMap(int32 sysidx)
{
	word32 fpmap=0;
	
	if(sysidx==0)
		fpmap = FREQ_GROUP_GPS;
	else if(sysidx==1)
		fpmap = FREQ_GROUP_BD;
#if SUPPORT_GLONASS
	else if(sysidx==2)
		fpmap = FREQ_GROUP_GLO;
#endif

	return fpmap;
}







