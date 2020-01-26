#include "StdAfx.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "PVT/CalcSvPVT.h"
#include "dataprocess.h"
#include "PVT/define.h"
#include "PVT/global.h"
#include "PVT/hcp.h"
#include "PVT/rtk.h"
#include "PVT/cfgpara.h"
#include "PVT/nmea0183.h"
#include "EBAS_V_UI_R.h"
#include "RTKCalBaseline.h"
#include "nrtk\rtklib.h"
#include "RTKLAMBDA.h"
#include "math.h"
#include "flag.h"

int MessageShowType = 0;
unsigned int tow = 0;
//CString sitename = "";

static Global_var _globalvar;
int stationnum=0;					
int master_num=0;
int mSLN_SYS = 0;
int ToltNum=0;
STRU_UART_STREAM PostProc_glStruUart[MASTER_SITE_MAXNUM+1];
_showObsInfo obsshow;
NAV_SHOWSATPOS satpos_struct;
WN_TOE_INFO wnToeInfo;
RTK_EPOCH RtkEpochStruct[MASTER_SITE_MAXNUM];
RTK_EPOCH_Keep RtkEpochKeepStruct[MASTER_SITE_MAXNUM];
del_unit posinfoshow[MASTER_SITE_MAXNUM];


bool Proc_start[MASTER_SITE_MAXNUM+1];

int ReadRTKData(char *rtcmData ,int len,CNtripSocket* pSiteinfo)
{
	int ret =0,i=0, StationId=0;
	int tick;
	STRU_UART_STREAM *glURT;

	pSiteinfo->openState = TRUE;//线程打开标志
	if(len <=0 )
	{
		return 0;
	}

	tick = tickget();
	////////////////////////多个ROVE 管理
	if(pSiteinfo->sitetype == 2)//ROVE
	{
		for(i=0;i<MASTER_SITE_MAXNUM;i++)
		{
			if(strcmp(_globalvar.master_sitename[i],pSiteinfo->MountPoint) == 0)
			{
				StationId = i;
				break;
			}
			else
			{
				if(i==MASTER_SITE_MAXNUM-1)
				{
					strcpy(_globalvar.master_sitename[master_num+1],pSiteinfo->MountPoint);
					master_num++;
					StationId = master_num;

					_globalvar.East[StationId] = pSiteinfo->XYZ[0];
					_globalvar.North[StationId] = pSiteinfo->XYZ[1];
					_globalvar.Up[StationId] = pSiteinfo->XYZ[2];

					if(master_num >= MASTER_SITE_MAXNUM-1)
					{
						master_num = 0;
						return -1;
					}
					break;
				}
			}
		}
	}
	else if(pSiteinfo->sitetype == 1)//BASE
	{
		strcpy(_globalvar.master_sitename[0],pSiteinfo->MountPoint);
		_globalvar.East[0] = pSiteinfo->XYZ[0];
		_globalvar.North[0] = pSiteinfo->XYZ[1];
		_globalvar.Up[0] = pSiteinfo->XYZ[2];
		StationId = 0;
	}
	else
	{
		return -1;
	}

///////////////////////////////	数据封装
	glURT = &(PostProc_glStruUart[StationId]);
	glURT->comid = StationId;
	EnterCriticalSection(&theApp.m_rtcm_lock);
	if(glURT->RecvData.CmdSetPos+len<=UART_RECV_BUF_LENTH)
	{
		memcpy(&(glURT->RecvData.Buf[glURT->RecvData.CmdSetPos]), rtcmData, len);
		glURT->RecvData.CmdSetPos += len;
	}
	else
	{
		int tmplen=0;
		tmplen = UART_RECV_BUF_LENTH-glURT->RecvData.CmdSetPos;
		memcpy(&(glURT->RecvData.Buf[glURT->RecvData.CmdSetPos]), rtcmData, tmplen);
		memcpy(&glURT->RecvData.Buf[0], rtcmData + tmplen, len - tmplen);
		glURT->RecvData.CmdSetPos = len - tmplen;
	}
	LeaveCriticalSection(&theApp.m_rtcm_lock);

	sleepms(50);
////////////////////////////// 数据处理
	_globalvar.SiteFlag[StationId] = true;
////////////////////////////////////////////
	ret = MessageShowType;
	MessageShowType = 0;
	return ret;
}

void InitSysUserParam(void)
{
	//memset(master_sitename,0,sizeof(master_sitename));
	memset(&_globalvar,0,sizeof(Global_var));
	memset(PostProc_glStruUart,0,(MASTER_SITE_MAXNUM+1)*sizeof(STRU_UART_STREAM));
	//memset(&obsshow,0,sizeof(_showObsInfo));

	pActiveCPTRegion = &ActiveCPTRegion;
	pActiveCPT = &(pActiveCPTRegion->sysm_cpt);
	memset(pActiveCPTRegion,0,sizeof(SYS_CPT_REGION));

	if(CheckParam((SYSM_CPT*)&FactoryCPT))
	{
		memcpy(pActiveCPT, &FactoryCPT, sizeof(SYSM_CPT));
		addCPTVerifyInfo(pActiveCPTRegion);
	}

	return;
}

void SendMessgeOBSToShow(OBSEV* pObsBuf, int stationtype)
{
	int i =0 ,SvID=0;
	double el=0.0,az=0.0;
	double svclkbias=0.0,freerr =0.0;
	double gps_obstow,bd_obstow = 0.0;
	bool GPS_OBSTIME_TRUE = false;
	bool BD_OBSTIME_TRUE = false;
	CString tempChar="";

	//memset(satpos,0,sizeof(NAV_SHOWSATPOS));

	if(theApp.m_hwndShowsignal)
	{
		tempChar.Format("%s",_globalvar.master_sitename[stationtype]);
		obsshow.sitename = tempChar;
	//	obsshow->sitename = sitename[stationtype];
		memcpy(&(obsshow.Obsev_Info),pObsBuf,sizeof(OBSEV));
		::PostMessage(theApp.m_hwndShowsignal,WM_DATA_OBS_SHOW,NULL,(LPARAM)&obsshow);
	}	

	if(stationtype != 0) //移动站数据触发解算
	{
		if(mSLN_SYS == 1) //GPS
		{
			 GPS_OBSTIME_TRUE = GetObsvTime(pObsBuf, &gps_obstow, NAV_SYS_GPS);
			 if(GPS_OBSTIME_TRUE)
			 {		 
				 Proc_start[stationtype] = true;//当前GPS数据收齐 开启解算
			 }
		}
		else if(mSLN_SYS == 2)//BDS
		{
			BD_OBSTIME_TRUE = GetObsvTime(pObsBuf, &bd_obstow, NAV_SYS_BD);
			if(BD_OBSTIME_TRUE)
			{			
				Proc_start[stationtype] = true;//当前BD数据收齐 开启解算
			}
		}
		else if(mSLN_SYS == 3)//BDS + GPS
		{
			 GPS_OBSTIME_TRUE = GetObsvTime(pObsBuf, &gps_obstow, NAV_SYS_GPS);
			 BD_OBSTIME_TRUE = GetObsvTime(pObsBuf, &bd_obstow, NAV_SYS_BD);
			 if(f_abs(gps_obstow-bd_obstow)<0.001/*BD_OBSTIME_TRUE == true && GPS_OBSTIME_TRUE == true*/)
			 {
				EnterCriticalSection(&theApp.m_show_lock);  
				Proc_start[stationtype] = true;//当前GPS+BD数据收齐 开启解算
				LeaveCriticalSection(&theApp.m_show_lock);
			 }
		}
		else
		{
			MessageShowType |= 0x01;
			return;
		}			
	}
	else
	{
// 		SlaveUseOBS.Station_USE_obs = pObsBuf;
// 		SlaveUseOBS.SiteName = _globalvar.master_sitename[0];
// 		SlaveUseOBS.sitenum = 0;
	}

	//poll base obs ready??
	//delete satpos;
	MessageShowType |= 0x01;
}

void SendMessgeNAV(int32 toe_eph,int32 wn_eph)
{	
	MessageShowType |= 0x02;

	wnToeInfo.bValid = TRUE;
	wnToeInfo.toe = toe_eph;
	wnToeInfo.wn = wn_eph;
}

void SendMessgePOS()
{	
	MessageShowType |= 0x04;
}


static int32 posCountNum[MASTER_SITE_MAXNUM]={};
static 	BD2_BASELINE aver_BaselineInfo[MASTER_SITE_MAXNUM];
del_unit infoshow[MASTER_SITE_MAXNUM];
static void CalcAverageBaseline(BD2_BASELINE* BaselineInfo,OBSEV* ObsBuf,Global_var* obsSiteInfo, int* fixcount, NetStr* pNet,int StationNum)
{
	double rate=0.0, rate2=0.0;
	gtime_t temptime;
	double time[6];
	double obstow = 0.0;
	double scale = 0.0;
	double x,y = 0.0;
	int tempCount = 0;
	int j = StationNum-1;

	tempCount = *fixcount;
	//	ToltNum = 90;
	if(posCountNum[j]==0)	//initial
	{
		posCountNum[j]++;
		memset(&aver_BaselineInfo[j],0,sizeof(BD2_BASELINE));
		aver_BaselineInfo[j].East =  BaselineInfo->East;
		aver_BaselineInfo[j].North =  BaselineInfo->North;
		aver_BaselineInfo[j].Up = BaselineInfo->Up;

	}
	else
	{	
		if(posCountNum[j] > AVER_ST)
		{
			if(f_abs(BaselineInfo->East - aver_BaselineInfo[j].East)>Aver_E || f_abs(BaselineInfo->North - aver_BaselineInfo[j].North)>Aver_N || f_abs(BaselineInfo->Up - aver_BaselineInfo[j].Up)>Aver_U )
				return;
		}

		posCountNum[j]++;	

		//average
		rate = 1.0/((double)posCountNum[j]);
		aver_BaselineInfo[j].East = (aver_BaselineInfo[j].East) * (1.0-rate) + BaselineInfo->East*rate;
		aver_BaselineInfo[j].North = (aver_BaselineInfo[j].North) * (1.0-rate) + BaselineInfo->North*rate;
		aver_BaselineInfo[j].Up = (aver_BaselineInfo[j].Up) * (1.0-rate) + BaselineInfo->Up*rate;

		if( (posCountNum[j] > ToltNum) || (tempCount>ToltNum))
		{
			scale = ((double)(posCountNum[j]))/((double)(tempCount));
			if(scale>0.5) //有一半以上的结果被丢掉，则此结果不做保存,重新计数
			{
				infoshow[j].del[POSSTA][0] = aver_BaselineInfo[j].East - obsSiteInfo->East[StationNum]; //减去标定的基准变量，得到形变量
				infoshow[j].del[POSSTA][1] = aver_BaselineInfo[j].North- obsSiteInfo->North[StationNum];//减去标定的基准变量，得到形变量
				infoshow[j].del[POSSTA][2] = aver_BaselineInfo[j].Up - obsSiteInfo->Up[StationNum];//减去标定的基准变量，得到形变量

				x = infoshow[j].del[POSSTA][0] * sin(Angle_NtoE*PI/180) + infoshow[j].del[POSSTA][1] * cos(Angle_NtoE*PI/180); //将形变量的东北天方向变化转为Angel角度方向xy变化
				y =  infoshow[j].del[POSSTA][1] * sin(Angle_NtoE*PI/180) - infoshow[j].del[POSSTA][0] * cos(Angle_NtoE*PI/180);
				infoshow[j].del[POSSTA][0] = x;
				infoshow[j].del[POSSTA][1] = y;

				strcpy(infoshow[j].sitename,obsSiteInfo->master_sitename[StationNum]);

				GetObsvTime(ObsBuf, &obstow, NAV_SYS_NONE);
				temptime = gpst2time(ObsBuf->wn,obstow);
				time2epoch(temptime,time);

				infoshow[j].time = temptime.time;
				infoshow[j].slntype = 10 + pNet->m_Netinfo.SLNSES;
				// 写入数据库进程
				del_unit* staticpos=new del_unit;
				memcpy(staticpos,&infoshow[j],sizeof(del_unit));		
				CreateThread(NULL,0,sqlinrt_process,staticpos,0,NULL); //静态解存入数据库

				//静态解显示到界面
				if(theApp.m_hwndShowpos)
				{
					::PostMessage(theApp.m_hwndShowpos,WM_SHOW_AVRPOS,NULL,(LPARAM)(&infoshow[j]));
				}
			}

			//清空滑动平均值
			//memset(&infoshow[j],0,sizeof(del_unit));
			aver_BaselineInfo[j].East = 0;
			aver_BaselineInfo[j].North = 0;
			aver_BaselineInfo[j].Up = 0;
			posCountNum[j] = 0;
			*fixcount = 0;
		}

	}

	return;
}


static void BaselinFake(BD2_BASELINE* BaselineInfo,OBSEV* ObsBuf,Global_var* obsSiteInfo,int StationNum ,int ret)
{
	//	UTC_TIME pTime;

}



DWORD WINAPI multi_site_process(void* arg)
{
	NetStr* pNet = (NetStr*)arg;
	double dTimeDiff = 0.0, masterTow=0.0, slaveTow=0.0, time_RTKStart=0.0;
	bool flag_RTK_Extra = FALSE;
	int ret_ManageStation=0;
	int fixIntcount[MASTER_SITE_MAXNUM] ={0};
	CString sitename;
	char msg[MAX_MSGSIZE]={0};
	int8 validDataLen[MASTER_SITE_MAXNUM+1];
	bool bMasterObsValid = false,bSlaveObsValid = false;
	bool bFindSlavTime =false;
	double obstowMaster = 0.0,obstowSlave = 0.0, diftow=0.0;
	int8 T_readidx =0;
	int k=0;

	OBSEV MasterObsStation;
	OBSEV SlaveObsStation;
	
	int id=0;

	double obtowMaster=0.0,obtowSlave=0.0;
	mSLN_SYS = pNet->m_Netinfo.SLNSYS;

	// 1、测站坐标初始化
	int basecount =0, rovercount =0;
	for (auto psocket = pNet->m_SiteList.begin(); psocket != pNet->m_SiteList.end() ; psocket++)
	{
		CString sitename = psocket->first;
		int sitetype = psocket->second->sitetype; // 测站类型 1 基准站 2 移动站 3 星历站
		switch (sitetype)
		{
		case 1:
			basecount += 1;	
			break;
		case 2: 
			rovercount++;	
			break;
		case 3: 
			break;
		default:
			break;
		}
	}
	if(basecount < 1 || rovercount < 1 || basecount>1){
		// 图形界面状态栏显示	
		char msg[MAX_MSGSIZE]={0};
		sprintf(msg,"> Error：%s set base num %d or rover num %d",pNet->m_Netinfo.NETNAME,basecount,rovercount);
		sendmsginfo(msg);	
		pNet->m_runflag = false;
		return -1;
	}
	// 限制总共只能连接三个参考站
	//if(basecount+rovercount>3){
		//exit(0);
	//}

	// 3、获取当前工程的GNSS卫星解算系统，静态解算时长
	switch (pNet->m_Netinfo.SLNSYS)
	{
	case 1: // GPS
		{
			pActiveCPT->SysmCptWorkConfig.NavFreqPoint|= DSP_L1CA_FRE;
			//pActiveCPT->SysmCptWorkConfig.FixNavFreqPoint |= DSP_L1C_FRE;
			//pActiveCPT->SysmCptWorkConfig.FixNavFreqPoint |= DSP_L1P_FRE;
			//pActiveCPT->SysmCptWorkConfig.FixNavFreqPoint |= DSP_L2C_FRE;
			pActiveCPT->SysmCptWorkConfig.NavFreqPoint |= DSP_L2P_FRE;
			pActiveCPT->SysmCptWorkConfig.NavFreqPoint |= DSP_L5_FRE;
			addCPTVerifyInfo(pActiveCPTRegion);

			for(k=0;k<MASTER_SITE_MAXNUM;k++)
			{
				posinfoshow[k].sys = SYS_GPS;
				infoshow[k].sys = SYS_GPS;
			}
			
			break;  
		}	
	case 2:	// BDS
		{
			pActiveCPT->SysmCptWorkConfig.NavFreqPoint |= DSP_B1I_FRE;
			pActiveCPT->SysmCptWorkConfig.NavFreqPoint |= DSP_B2I_FRE;
			pActiveCPT->SysmCptWorkConfig.NavFreqPoint |= DSP_B3I_FRE; 
			addCPTVerifyInfo(pActiveCPTRegion);

			for(k=0;k<MASTER_SITE_MAXNUM;k++)
			{
				posinfoshow[k].sys = SYS_CMP;
				infoshow[k].sys = SYS_CMP;
			}
			break;
		}
	case 3: // GPS+BDS
		{
			pActiveCPT->SysmCptWorkConfig.NavFreqPoint|= DSP_L1CA_FRE;
			//pActiveCPT->SysmCptWorkConfig.FixNavFreqPoint |= DSP_L1C_FRE;
			//pActiveCPT->SysmCptWorkConfig.FixNavFreqPoint |= DSP_L1P_FRE;
			//pActiveCPT->SysmCptWorkConfig.FixNavFreqPoint |= DSP_L2C_FRE;
			pActiveCPT->SysmCptWorkConfig.NavFreqPoint |= DSP_L2P_FRE;
			pActiveCPT->SysmCptWorkConfig.NavFreqPoint |= DSP_L5_FRE;
			pActiveCPT->SysmCptWorkConfig.NavFreqPoint |= DSP_B1I_FRE;
			pActiveCPT->SysmCptWorkConfig.NavFreqPoint |= DSP_B2I_FRE;
			pActiveCPT->SysmCptWorkConfig.NavFreqPoint |= DSP_B3I_FRE; 
			addCPTVerifyInfo(pActiveCPTRegion);

			for(k=0;k<MASTER_SITE_MAXNUM;k++)
			{
				posinfoshow[k].sys = SYS_GC;
				infoshow[k].sys = SYS_GC;
			}
			break;  
		}	
	default:
		break;
	}

	//int phridx = pNet->m_Netinfo.SLNPHS;

	switch (pNet->m_Netinfo.SLNSES)
	{
	case 0: ToltNum=   900.0; break; // 15分钟
	case 1: ToltNum=  1800.0; break; // 30分钟
	case 2: ToltNum=  3600.0; break; //  1小时
	case 3: ToltNum= 86400.0; break; // 24小时
	default:
		ToltNum=900.0;
		break;
	}


 //倒计时，收星历使用
	
	sprintf(msg,"> OK：The config of %s has been checked！",pNet->m_Netinfo.NETNAME);
	sendmsginfo(msg);	
	sprintf(msg,"> wait ephemeris ... ...");
	sendmsginfo(msg);	
	for (int i = 5; 0<i; i--)
	{
		sleepms(1000);
		sprintf(msg,"> %2d seconds left",i);
		sendmsginfo(msg);	
	}
	sprintf(msg,"> solution start! ");
	sendmsginfo(msg);	

//start sln
	while(pNet->m_runflag)
	{
		for(int j=1;j<MASTER_SITE_MAXNUM+1;j++)
		{
			if(Proc_start[j])
			{
				Proc_start[j] = false;

// 				if(!GetReadidxSlaveBuf(&(Master_Buf[j-1]),&Slave_Buf, &MasterObsStation,&SlaveObsStation)) //时间对齐
// 				{
// 					continue;
// 				}
///////////////////////////////////////////////时间对齐
				validDataLen[j] = Master_Buf[j].writeidx -   Master_Buf[j].readidx;

				if(validDataLen[j] < 0)
					validDataLen[j] =   Master_Buf[j].writeidx + SLAVE_BUF_LEN -   Master_Buf[j].readidx;

				while(validDataLen[j]>=0)
				{
					bFindSlavTime = false;
					T_readidx = Master_Buf[j].readidx;
					bMasterObsValid = GetObsvTime(&Master_Buf[j].obsev[T_readidx], &obstowMaster, NAV_SYS_NONE);

					for(int i=0;i<SLAVE_BUF_LEN;i++)
					{
						bSlaveObsValid = GetObsvTime(&Slave_Buf.obsev[i], &obstowSlave, NAV_SYS_NONE);
						if(!bSlaveObsValid)
							continue;

						diftow = f_abs(obstowMaster - obstowSlave);
						if(diftow > SECONDS_HALF_WEEK)
							diftow = diftow - SECONDS_IN_WEEK;

						if(diftow < 0.005) 
						{
							EnterCriticalSection(&theApp.m_proc_lock); 
							bFindSlavTime = true;
							//pSlaveUse = &(pSlaveObs->obsev[i]);
							/**idreadSlave = i;*/
							//p_reSlaveObs = &(pSlaveObs->obsev[i]);
							memcpy(&SlaveObsStation,&Slave_Buf.obsev[i],sizeof(OBSEV));
							LeaveCriticalSection(&theApp.m_proc_lock);		
							break;
						}
					}

					if(bFindSlavTime == true)
					{
						EnterCriticalSection(&theApp.m_proc_lock); 
						/*			pRetObs = &(Master_Buf_New->obsev[readidx]);*/
						memcpy(&MasterObsStation,&(Master_Buf[j].obsev[T_readidx]),sizeof(OBSEV));
						//p_reMasterObs = &(Master_Buf_New->obsev[readidx]);

						Master_Buf[j].readidx++;
						if(Master_Buf[j].readidx>=SLAVE_BUF_LEN)
							Master_Buf[j].readidx = 0;

						T_readidx = Master_Buf[j].readidx;
						LeaveCriticalSection(&theApp.m_proc_lock);		
						break;
					}
					else
					{
						Master_Buf[j].readidx++;
						if(Master_Buf[j].readidx>=SLAVE_BUF_LEN)
							Master_Buf[j].readidx = 0;

						T_readidx = Master_Buf[j].readidx;
					}

					validDataLen[j]--;

				}

///////////////////////////////////////////////
				EnterCriticalSection(&theApp.m_proc_lock); 
				/*SlaveObsStation = &Slave_Buf.obsev[id];*/
				GetObsvTime(&MasterObsStation, &obtowMaster, NAV_SYS_NONE);
				GetObsvTime(&SlaveObsStation, &obtowSlave, NAV_SYS_NONE);
				LeaveCriticalSection(&theApp.m_proc_lock);	
// 				if(obtowMaster != obtowSlave)
// 					continue;
				if(f_abs(obtowMaster-obtowSlave)>1.001)
				{
					sprintf(msg,">%s：Obs Time Not Match!",_globalvar.master_sitename[j]);
					sendmsginfo(msg);	
					continue;
				}
					

				CalcBaseStationSVPos(&MasterObsStation);
				CalcBaseStationSVPos(&SlaveObsStation);

				
				if( (MasterObsStation.bSinglePosValid) && (SlaveObsStation.bSinglePosValid) )
				{
					EnterCriticalSection(&theApp.m_proc_lock);
					int ret = Rel_Position(&MasterObsStation, &SlaveObsStation, &(RtkEpochStruct[j-1]), &(RtkEpochKeepStruct[j-1]));

					(fixIntcount[j-1])++;//解一次计数一次,到15分钟、30分钟、1小时后在函数中清0
					LeaveCriticalSection(&theApp.m_proc_lock);	
					if(ret>=FIX_L1_INT/*FIX_WIDE_FLOAT*/)
					{
						               bvbvbvbvbvcSKY-20170412QGP
						if(!CalcObsWN(&MasterObsStation))
							return -1;

						BaselineShow(&(RtkEpochStruct[j-1].Baseline),&MasterObsStation,&_globalvar,j); //实时解显示结果
						//CalcAverageBaseline(&(RtkEpochStruct[j-1].Baseline),&MasterObsStation,&_globalvar ,&(fixIntcount[j-1]), pNet,j); // 静态解

						sprintf(msg,">%s： Baseline Fixed------FIX_L1_INT  Path:%d",_globalvar.master_sitename[j],RtkEpochKeepStruct[j-1].RTKErrPath);
						sendmsginfo(msg);	
					}	
					else
					{
						//BaselinFake(&(RtkEpochStruct[j-1].Baseline),&MasterObsStation,&_globalvar,j,ret);//
						switch (ret)
						{
						case FIX_NOT: sprintf(msg,">%s： Baseline not Fixed!!!------FIX_NOT  ErrorPath:%d  nAmbIndex:%d nAmbIndex_w:%d",_globalvar.master_sitename[j],RtkEpochKeepStruct[j-1].RTKErrPath,RtkEpochStruct[j-1].nAmbIndex,RtkEpochStruct[j-1].nAmbIndex_wide);sendmsginfo(msg);	break;
						case FIX_2D: sprintf(msg,">%s： Baseline not Fixed!!!------FIX_2D",_globalvar.master_sitename[j]);sendmsginfo(msg);	break;
						case FIX_3D: sprintf(msg,">%s： Baseline not Fixed!!!------FIX_3D",_globalvar.master_sitename[j]);sendmsginfo(msg);	break;
						case FIX_WIDE_FLOAT: sprintf(msg,">%s： Baseline not Fixed!!!------FIX_WIDE_FLOAT  ErrorPath:%d  nAmbIndex:%d nAmbIndex_w:%d",_globalvar.master_sitename[j],RtkEpochKeepStruct[j-1].RTKErrPath,RtkEpochStruct[j-1].nAmbIndex,RtkEpochStruct[j-1].nAmbIndex_wide);sendmsginfo(msg);	break;
						case FIX_L1_FLOAT: sprintf(msg,">%s： Baseline not Fixed!!!------FIX_L1_FLOAT",_globalvar.master_sitename[j]);sendmsginfo(msg);	break;
						case FIX_NARROW_FLOAT: sprintf(msg,">%s： Baseline not Fixed!!!------FIX_NARROW_FLOAT",_globalvar.master_sitename[j]);sendmsginfo(msg);	break;
						case FIX_WIDE_INT: sprintf(msg,">%s： Baseline not Fixed!!!------FIX_WIDE_INT  ErrorPath:%d  nAmbIndex:%d nAmbIndex_w:%d",_globalvar.master_sitename[j],RtkEpochKeepStruct[j-1].RTKErrPath,RtkEpochStruct[j-1].nAmbIndex,RtkEpochStruct[j-1].nAmbIndex_wide);sendmsginfo(msg);	break;

						default: sprintf(msg,"Baseline not Fixed!!!------FIX_NOT");sendmsginfo(msg);
							break;
						}
					}
						
				}
				else
				{
					sprintf(msg,">%s： bSinglePosValid Not Fixed",_globalvar.master_sitename[j]);
					sendmsginfo(msg);	
				}

				//DataShow(&MasterObsStation,&_globalvar,j); 
			}
			else
			{
// 				sprintf(msg,">%s：  NO OBS Proc_start FLASE",_globalvar.master_sitename[j]);
// 				sendmsginfo(msg);	
			}
		}
	}
	pNet->m_runflag = false;

	return 1;
}


DWORD WINAPI data_deal_process(void* arg)
{
	CNtripSocket* psinfo = (CNtripSocket*)arg;
	int StationId = 0,i=0;
	int tick = 0;
	
	////////////////////////多个ROVE 管理
	if(psinfo->sitetype == 2)//ROVE
	{
		for(i=0;i<MASTER_SITE_MAXNUM;i++)
		{
			if(strcmp(_globalvar.master_sitename[i],psinfo->MountPoint) == 0)
			{
				StationId = i;
				break;
			}
			else
			{
				if(i==MASTER_SITE_MAXNUM-1)
				{
					strcpy(_globalvar.master_sitename[stationnum+1],psinfo->MountPoint);
					stationnum++;
					StationId = stationnum;
					_globalvar.East[StationId] = psinfo->XYZ[0];
					_globalvar.North[StationId] = psinfo->XYZ[1];
					_globalvar.Up[StationId] = psinfo->XYZ[2];
					if(stationnum >= MASTER_SITE_MAXNUM-1)//???
					{
						stationnum = 0;
						return -1;
					}
					break;
				}
			}
		}
	}
	else if(psinfo->sitetype == 1)//BASE
	{
		strcpy(_globalvar.master_sitename[0],psinfo->MountPoint);
		_globalvar.East[0] = psinfo->XYZ[0];
		_globalvar.North[0] = psinfo->XYZ[1];
		_globalvar.Up[0] = psinfo->XYZ[2];
		StationId = 0;
	}
	else
	{
		return -1;
	}

	while (1)
	{
		if(_globalvar.SiteFlag[StationId])
		{
	//		tick = tickget();
			EnterCriticalSection(&theApp.m_rtcm_lock);
			TaskProtocolInput(&(PostProc_glStruUart[StationId])); //接收
	//		int cputime = (int)(tickget() - tick);
	//		sleepms(900-cputime);
			LeaveCriticalSection(&theApp.m_rtcm_lock);
		}			
	}
	return 1;
}

void RtkCloseSet(void)
{
// 	memset(&Master_observ,0,sizeof(OBSEV));
// 	memset(&Slave_Buf,0,sizeof(OBSEV_BUF));
// 	memset(LastPhi,0,sizeof(LastPhi));
// 	memset(&Master_obs,0,sizeof(OBSEV));
// 	memset(&Slave_obs,0,sizeof(OBSEV));
// 
// 
// 	memset(&RtkEpoch,0,sizeof(RtkEpoch));
// 	memset(&RtkEpochKeep,0,sizeof(RtkEpochKeep));
// 	RtkEpochKeep.bFirstCalAmb = TRUE;
// 
// 	antfixposition.bValid = FALSE;
// 	Basefixposition.bValid = FALSE;


	return;
}

DWORD WINAPI deal_close_process(void* arg)
{
	CNtripSocket* info = (CNtripSocket*)arg;
	if(!info->openState)
	{
		return 0;
	}

	if(info->hdata == INVALID_HANDLE_VALUE)
		return 0;
	WaitForSingleObject(info->hdata,100);
	CloseHandle(info->hdata);
	strclose(&info->psock);
	strclose(&info->psock_f);
	strclose(&info->psock_s);
	// show in the picture
	char msg[MAX_MSGSIZE]={0};
	sprintf(msg,"> %s is closed!",info->MountPoint);
	sendmsginfo(msg);	

	return 1;
}

void SatShowObs(OBSEV* pObsBuf)
{
	int i=0;
	int SvID=0;
	double el=0.0,az=0.0;
	double svclkbias=0.0,freerr =0.0;
	double obstow = 0.0;


	if(theApp.m_hwndShowsatpos)
	{
		memset(&satpos_struct,0,sizeof(NAV_SHOWSATPOS));
		for(i=0;i<pObsBuf->satnmax;i++)
		{
			satpos_struct.SAT_pos[i].ath = pObsBuf->obs[i].az;
			satpos_struct.SAT_pos[i].ele = pObsBuf->obs[i].el;
			satpos_struct.SAT_pos[i].prn = pObsBuf->obs[i].StarID;		
			satpos_struct.satnum++;
		}
		if(satpos_struct.satnum != 0)
			::PostMessage(theApp.m_hwndShowsatpos,WM_SHOW_SATPOS,NULL,(LPARAM)&satpos_struct);
	}
}

static int32 countNum[MASTER_SITE_MAXNUM]={};
static int32 timeCount[MASTER_SITE_MAXNUM]={};
static 	BD2_BASELINE aver_Basel[MASTER_SITE_MAXNUM];
static 	BD2_BASELINE GlobAver[2];

void DataShow(OBSEV* ObsBuf,Global_var* obsSiteInfo,int StationNum)
{
	SYSTEMTIME ts;
	double obstow=0.0;
	gtime_t temptime;
	double time[6];
	int j=StationNum-1;
	double x,y=0.0;
	double rate=0.0;

	GetSystemTime(&ts); 
	if(timeCount[j] < OutPutPeriod && ts.wSecond != 30 && ts.wSecond != 0)
		return;

	timeCount[j]=0;

	posinfoshow[j].del[0][0] = posinfoshow[j].del[0][0] - obsSiteInfo->East[StationNum]; //减去标定的基准变量，得到形变量
	posinfoshow[j].del[0][1] = posinfoshow[j].del[0][1] - obsSiteInfo->North[StationNum];//减去标定的基准变量，得到形变量
	posinfoshow[j].del[0][2] = posinfoshow[j].del[0][2] - obsSiteInfo->Up[StationNum]; //减去标定的基准变量，得到形变量

	x = posinfoshow[j].del[0][0] * sin(Angle_NtoE*D2R) + posinfoshow[j].del[0][1] * cos(Angle_NtoE*D2R); //将形变量的东北天方向变化转为Angel角度方向xy变化
	y =  posinfoshow[j].del[0][1] * sin(Angle_NtoE*D2R) - posinfoshow[j].del[0][0] * cos(Angle_NtoE*D2R);
	posinfoshow[j].del[0][0] = x;
	posinfoshow[j].del[0][1] = y;


	strcpy(posinfoshow[j].sitename,obsSiteInfo->master_sitename[StationNum]);

	GetObsvTime(ObsBuf, &obstow, NAV_SYS_NONE);
	//	GPSTimeToYMDHMS(ObsBuf.Station_USE_obs->wn, obstow, &pTime);
	temptime = gpst2time(ObsBuf->wn,obstow);
	time2epoch(temptime,time);
	posinfoshow[j].time = temptime.time;


	// 写入数据库进程 实时解
	if(is_tint(temptime,1.0))// && 1.5< pinforeal->ratio )
	{
		EnterCriticalSection(&theApp.m_proc_lock); 	
		del_unit* realpos=new del_unit;
		memcpy(realpos,&posinfoshow[j],sizeof(del_unit));
		realpos->slntype = POSREAL;
		LeaveCriticalSection(&theApp.m_proc_lock);
		CreateThread(NULL,0,sqlinrt_process,realpos,0,NULL);

	}

	//实时解显示到界面
	if(theApp.m_hwndShowpos)
	{	
		::PostMessage(theApp.m_hwndShowpos,WM_SHOW_PPPPOS,NULL,(LPARAM)(&posinfoshow[j]));
	}
}


void BaselineShow(BD2_BASELINE*	BaselineInfo,OBSEV* ObsBuf,Global_var* obsSiteInfo,int StationNum)
{
	//	UTC_TIME pTime;
	SYSTEMTIME ts;
	double obstow=0.0;
	gtime_t temptime;
	double time[6];
	int j=StationNum-1;
	double x,y=0.0;
	double rate=0.0;
	double chk1,chk2;

	static double Naa[4] = {0.6666667, -0.6666667, -0.6666667, 2.6666667};   // 2*2  A = [1,1,1;-1,0,0] P = eye(0.5,1,0.5); Naa = (A*P*A')^-1
	static double QAT[6] = {2.0, 2.0, 1.0, 0.0, -2.0, 0.0}; //3*2  QAT = P\A'
	double W1[2] = {0.0, 0.0};  //2*1
	double K1[2] = {0.0, 0.0};  //2*1
	double V1[3] = {0.0, 0.0, 0.0};  //3*1

	posinfoshow[j].del[0][0] =  BaselineInfo->East; //减去标定的基准变量，得到形变量
	posinfoshow[j].del[0][1] =  BaselineInfo->North;//减去标定的基准变量，得到形变量
	posinfoshow[j].del[0][2] =  BaselineInfo->Up; //减去标定的基准变量，得到形变量

	timeCount[j]++;
	if(countNum[j] == 0)
	{
		aver_Basel[j].East = posinfoshow[j].del[0][0];
		aver_Basel[j].North = posinfoshow[j].del[0][1];
		aver_Basel[j].Up = posinfoshow[j].del[0][2];

		GlobAver[0].East =  92.2641;
		GlobAver[0].North =  68.69235 ;
		GlobAver[0].Up =  13.6153;
		GlobAver[1].East =  96.1195;
		GlobAver[1].North =  52.9180195839 ;
		GlobAver[1].Up =  13.6592;
		countNum[j]++;
	}
	else
	{
		if(f_abs(posinfoshow[j].del[0][0] - aver_Basel[j].East)>Aver_E || f_abs(posinfoshow[j].del[0][1] - aver_Basel[j].North)>Aver_N || f_abs(posinfoshow[j].del[0][2] - aver_Basel[j].Up)>Aver_U )
			return;

		if(countNum[j]<WindowLenth)
			countNum[j]++;

		//average
		rate = 1.0/((double)countNum[j]);
		aver_Basel[j].East = (aver_Basel[j].East) * (1.0-rate) + (posinfoshow[j].del[0][0])*rate;
		aver_Basel[j].North = (aver_Basel[j].North) * (1.0-rate) + (posinfoshow[j].del[0][1])*rate;
		aver_Basel[j].Up = (aver_Basel[j].Up) * (1.0-rate) + (posinfoshow[j].del[0][2])*rate;

		posinfoshow[j].del[0][0] = aver_Basel[j].East;
		posinfoshow[j].del[0][1] = aver_Basel[j].North;
		posinfoshow[j].del[0][2] = aver_Basel[j].Up;

	}

	//平差
/*	if(countNum[j]>=NetSlnLenth)
	{
		if(!strcmp(posinfoshow[j].sitename,"ROV2"))
		{
			EnterCriticalSection(&theApp.m_proc_lock); 	
			GlobAver[1].East =  posinfoshow[j].del[0][0] ;
			GlobAver[1].North =  posinfoshow[j].del[0][1] ;
			GlobAver[1].Up =  posinfoshow[j].del[0][2] ;

			W1[1] = GlobAver[0].East -  92.2641; // X
			LAMBDA_MO_MatrixMulti(Naa,W1,K1,2,2,1);
			LAMBDA_MO_MatrixMulti(QAT,K1,V1,3,2,1);
			posinfoshow[j].del[0][0] = V1[1] +  GlobAver[1].East;

			W1[1] = GlobAver[0].North -  68.69235; // Y
			LAMBDA_MO_MatrixMulti(Naa,W1,K1,2,2,1);
			LAMBDA_MO_MatrixMulti(QAT,K1,V1,3,2,1);
			posinfoshow[j].del[0][1] = V1[1] +  GlobAver[1].North;

			W1[1] = GlobAver[0].Up -  13.6153; // Z
			LAMBDA_MO_MatrixMulti(Naa,W1,K1,2,2,1);
			LAMBDA_MO_MatrixMulti(QAT,K1,V1,3,2,1);
			posinfoshow[j].del[0][2] = V1[1] +  GlobAver[1].Up;
			LeaveCriticalSection(&theApp.m_proc_lock);

		}
		else
		{
			EnterCriticalSection(&theApp.m_proc_lock); 
			GlobAver[0].East =  posinfoshow[j].del[0][0] ;
			GlobAver[0].North =  posinfoshow[j].del[0][1] ;
			GlobAver[0].Up =  posinfoshow[j].del[0][2] ;

			W1[1] = GlobAver[1].East -   96.1195; // X
			LAMBDA_MO_MatrixMulti(Naa,W1,K1,2,2,1);
			LAMBDA_MO_MatrixMulti(QAT,K1,V1,3,2,1);
			posinfoshow[j].del[0][0] = V1[1] +  GlobAver[0].East;

			W1[1] = GlobAver[1].North -  52.9180; // Y
			LAMBDA_MO_MatrixMulti(Naa,W1,K1,2,2,1);
			LAMBDA_MO_MatrixMulti(QAT,K1,V1,3,2,1);
			posinfoshow[j].del[0][1] = V1[1] +  GlobAver[0].North;

			W1[1] = GlobAver[1].Up -  13.6592; // Z
			LAMBDA_MO_MatrixMulti(Naa,W1,K1,2,2,1);
			LAMBDA_MO_MatrixMulti(QAT,K1,V1,3,2,1);
			posinfoshow[j].del[0][2] = V1[1] +  GlobAver[0].Up;
			LeaveCriticalSection(&theApp.m_proc_lock);
		}
	}*/
	
	GetSystemTime(&ts); 
	if(timeCount[j] < OutPutPeriod && ts.wSecond != 30 && ts.wSecond != 0)
		return;

	timeCount[j]=0;

	posinfoshow[j].del[0][0] = posinfoshow[j].del[0][0] - obsSiteInfo->East[StationNum]; //减去标定的基准变量，得到形变量
	posinfoshow[j].del[0][1] = posinfoshow[j].del[0][1] - obsSiteInfo->North[StationNum];//减去标定的基准变量，得到形变量
	posinfoshow[j].del[0][2] = posinfoshow[j].del[0][2] - obsSiteInfo->Up[StationNum]; //减去标定的基准变量，得到形变量
	/*
	chk1 = sqrt(posinfoshow[j].del[0][0]*posinfoshow[j].del[0][0]+posinfoshow[j].del[0][1]*posinfoshow[j].del[0][1]+posinfoshow[j].del[0][2]*posinfoshow[j].del[0][2]);
	*/
	x = posinfoshow[j].del[0][0] * sin(Angle_NtoE*D2R) + posinfoshow[j].del[0][1] * cos(Angle_NtoE*D2R); //将形变量的东北天方向变化转为Angel角度方向xy变化
	y =  posinfoshow[j].del[0][1] * sin(Angle_NtoE*D2R) - posinfoshow[j].del[0][0] * cos(Angle_NtoE*D2R);
	/*
	posinfoshow[j].del[0][0] = x;
	posinfoshow[j].del[0][1] = y;
	chk2 = sqrt(x*x+y*y+posinfoshow[j].del[0][2]*posinfoshow[j].del[0][2]);
	*/



	strcpy(posinfoshow[j].sitename,obsSiteInfo->master_sitename[StationNum]);

	GetObsvTime(ObsBuf, &obstow, NAV_SYS_NONE);
	//	GPSTimeToYMDHMS(ObsBuf.Station_USE_obs->wn, obstow, &pTime);
	temptime = gpst2time(ObsBuf->wn,obstow);
	time2epoch(temptime,time);
	posinfoshow[j].time = temptime.time;


	// 写入数据库进程 实时解
	if(is_tint(temptime,1.0))// && 1.5< pinforeal->ratio )
	{
		EnterCriticalSection(&theApp.m_proc_lock); 	
		del_unit* realpos=new del_unit;
		memcpy(realpos,&posinfoshow[j],sizeof(del_unit));
		realpos->slntype = POSREAL;
		LeaveCriticalSection(&theApp.m_proc_lock);
		CreateThread(NULL,0,sqlinrt_process,realpos,0,NULL);

	}

	//实时解显示到界面
	if(theApp.m_hwndShowpos)
	{	
		::PostMessage(theApp.m_hwndShowpos,WM_SHOW_PPPPOS,NULL,(LPARAM)(&posinfoshow[j]));
	}


	

}

BOOL CalcObsWN(OBSEV* pObsBuf)
{
	int wkn=0;
	double obstow = 0.0;

	if(!wnToeInfo.bValid)
		return FALSE;

	if(!GetObsvTime(pObsBuf, &obstow, NAV_SYS_NONE))
		return FALSE;

	wkn = wnToeInfo.wn;
	
	if((obstow - wnToeInfo.toe)>SECONDS_IN_HALF_WEEK)
		wkn--;
	else if((obstow - wnToeInfo.toe)<(-1*SECONDS_IN_HALF_WEEK))
		wkn++;

	pObsBuf->wn = wkn;

	return TRUE;	
}

bool GetReadidxSlaveBuf(OBSEV_BUF* pObsBuf,OBSEV_BUF* pSlaveObs, OBSEV* p_reMasterObs, OBSEV* p_reSlaveObs)
{
/*	OBSEV* pRetObs= NULL;*/
	OBSEV_BUF* Master_Buf_New = pObsBuf; // 读指针 读rtk中的全局变量
	int8 validDataLen;
	bool bMasterObsValid = false,bSlaveObsValid = false;
	bool bFindSlavTime =false,ret = false;
	double obstowMaster = 0.0,obstowSlave = 0.0, diftow=0.0;
	int8 readidx =0;

	validDataLen = Master_Buf_New->writeidx - Master_Buf_New->readidx;
	
	if(validDataLen < 0)
		validDataLen =  Master_Buf_New->writeidx + MASTER_SITE_MAXNUM -  Master_Buf_New->readidx;

	while(validDataLen>=0)
	{
		
		bMasterObsValid = GetObsvTime(&(Master_Buf_New->obsev[readidx]), &obstowMaster, NAV_SYS_NONE);

		for(int i=0;i<SLAVE_BUF_LEN;i++)
		{
			bSlaveObsValid = GetObsvTime(&(pSlaveObs->obsev[i]), &obstowSlave, NAV_SYS_NONE);
			if(!bSlaveObsValid)
				continue;

			diftow = f_abs(obstowMaster - obstowSlave);
			if(diftow > SECONDS_HALF_WEEK)
				diftow = diftow - SECONDS_IN_WEEK;

			if(diftow < 0.005) 
			{
				bFindSlavTime = true;
				//pSlaveUse = &(pSlaveObs->obsev[i]);
				/**idreadSlave = i;*/
				//p_reSlaveObs = &(pSlaveObs->obsev[i]);
				memcpy(&p_reSlaveObs,&(pSlaveObs->obsev[i]),sizeof(OBSEV));
				break;
			}
		}

		if(bFindSlavTime == true)
		{
/*			pRetObs = &(Master_Buf_New->obsev[readidx]);*/
			memcpy(&p_reMasterObs,&(Master_Buf_New->obsev[readidx]),sizeof(OBSEV));
			//p_reMasterObs = &(Master_Buf_New->obsev[readidx]);
			ret = true;
			break;
		}

		Master_Buf_New->readidx++;
		if(Master_Buf_New->readidx>=SLAVE_BUF_LEN)
			Master_Buf_New->readidx = 0;

		readidx = Master_Buf_New->readidx;

		validDataLen--;

	}


	return ret;
}

