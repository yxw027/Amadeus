#ifndef _DATA_PROCESS_HEAD_
#define _DATA_PROCESS_HEAD_

#include "PVT/define.h"
#include "PVT/rtk.h"
#include "PVT/FrameDecode.h"
#include "Communication/NtripSocket.h"

#define AVER_ST 50
#define Aver_E 0.02
#define Aver_N 0.02
#define Aver_U 0.04

#define WindowLenth 7200
#define NetSlnLenth 900
#define OutPutPeriod 30
#define Angle_NtoE 80 //实际测量结果 东北天坐标系的转换角度

typedef struct 
{
	OBSEV* Station_USE_obs;
	CString SiteName;
	int sitenum;
}OBSEV_STRUCT;

typedef struct
{
	char	sys;	//卫星系统
	int	prn;	//卫星号
	double	ath;	//方位角
	double	ele;	//高度角
	double  pdop; //PDOP
	double  gdop; //GDOP
	double  hdop; //HDOP
	double  vdop; //VDOP
	double	srn[3];	//信噪比
}NAV_showpos;

typedef struct NAV_SHOWSATPOS
{
	NAV_showpos SAT_pos[SV_NUM_GPSBD];
	int satnum;
};

typedef struct _showObsInfo
{
	OBSEV	Obsev_Info;
	CString	sitename;
};

typedef struct Global_var
{
	char master_sitename[MASTER_SITE_MAXNUM+1][10];
	bool SiteFlag[MASTER_SITE_MAXNUM+1];
	double East[MASTER_SITE_MAXNUM+1];
	double North[MASTER_SITE_MAXNUM+1];
	double Up[MASTER_SITE_MAXNUM+1];
};

typedef struct del_unit
{
	char    sitename[MAX_PATH];
	int	sys;
	int	slntype;
	double	del[5][5]; // 0= delx, 1= dely, 2= delz, 3= vsn, 4= ratio
	time_t	time;
};

typedef struct
{
 	BOOL bValid;
	int32 wn;
	int32 toe;
 }WN_TOE_INFO;

// typedef struct pos_del
// {
// 	del_unit	posinfo;
// };


#ifdef __cplusplus
extern "C" {
#endif

extern int ReadRTKData(char *rtcmData,int len, CNtripSocket* pSiteinfo);
extern void InitSysUserParam(void);
extern void SendMessgeOBSToShow(OBSEV* pObsBuf, int stationtype);
extern void SendMessgeNAV(int32 toe_eph,int32 wn_eph);
extern void SendMessgePOS();
extern DWORD WINAPI multi_site_process(void* arg);
extern DWORD WINAPI data_deal_process(void* arg);
extern DWORD WINAPI deal_close_process(void* arg);
extern void SatShowObs(OBSEV* pObsBuf);
extern void BaselineShow(BD2_BASELINE*	BaselineInfo,OBSEV* ObsBuf,Global_var* obsSiteInfo,int StationNum);
extern void DataShow(OBSEV* ObsBuf,Global_var* obsSiteInfo,int StationNum);
extern BOOL CalcObsWN(OBSEV* pObsBuf);
extern void RtkCloseSet(void);
extern bool GetReadidxSlaveBuf(OBSEV_BUF* pObsBuf,OBSEV_BUF* pSlaveObs, OBSEV* p_reMasterObs, OBSEV* p_reSlaveObs);

#ifdef __cplusplus
}
#endif

#endif