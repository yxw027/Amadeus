
#include "../stdafx.h"
#include "NtripSocket.h"
#include "../EBAS_V_UI_R.h"
#include <QString>
//#include "dataprocess.h"

static DWORD WINAPI strthread(void *arg)
{
//	// initial communication model
//	CNtripSocket* psinfo = (CNtripSocket*)arg;
//	stream_t *svr = &psinfo->psock;
//	rtcm_t* prtcm = &psinfo->conv->rtcm;
//	// 	raw_t* praw = &psinfo->conv->raw;
//	// 	FILE* rawfile = NULL; // raw file received
//	// 	FILE* obsfile = NULL; // rnx file decode
//	// 	FILE* navfile = NULL; // rnx file decode
//	QString workpath(psinfo->rtcm_fwrt.workpath);
//
//	char buff[MAXSRCTBL] = { 0 }, msg[MAXSTRMSG] = { 0 }, path[256] = { 0 };
//	char *p = buff;
//	int stat, ns;
//	unsigned int tick = tickget();
//
//#ifdef OBS_FILE_DAY
//	int fileinv = 86400;
//#else
//	int fileinv = 3600;
//#endif
//
//	strinitcom();
//	int PORT = (psinfo->Port).toInt();
//	QString port = QString::number(PORT+1);
//	if (psinfo->m_ConnectType == STR_SERIAL)
//	{
//		psinfo->m_ConnectType = (ConnectType)STR_TCPCLI;
//	}
//	// 所有的数据都从这几个网口进入，调用QString::toUtf8().data()将QString转换为char *
//	if (psinfo->MountPoint == "BASE")
//		sprintf(path, "%s:%s@%s:%s/%s", psinfo->User.toUtf8().data(), psinfo->Pwd.toUtf8().data(), "127.0.0.1", "30001", psinfo->MountPoint.toUtf8().data());
//	else if (psinfo->MountPoint == "ROV1")
//		sprintf(path, "%s:%s@%s:%s/%s", psinfo->User.toUtf8().data(), psinfo->Pwd.toUtf8().data(), "127.0.0.1", "30002", psinfo->MountPoint.toUtf8().data());
//	else if (psinfo->MountPoint == "ROV2")
//		sprintf(path, "%s:%s@%s:%s/%s", psinfo->User.toUtf8().data(), psinfo->Pwd.toUtf8().data(), "127.0.0.1", "30003", psinfo->MountPoint.toUtf8().data());
//	else if (psinfo->MountPoint == "ROV3")
//		sprintf(path, "%s:%s@%s:%s/%s", psinfo->User.toUtf8().data(), psinfo->Pwd.toUtf8().data(), "127.0.0.1", "30004", psinfo->MountPoint.toUtf8().data());
//	else if (psinfo->MountPoint == "ROV4")
//		sprintf(path, "%s:%s@%s:%s/%s", psinfo->User.toUtf8().data(), psinfo->Pwd.toUtf8().data(), "127.0.0.1", "30005", psinfo->MountPoint.toUtf8().data());
//
//	//	sprintf(path,"C:\Users\Administrator\Desktop\test.DAT::N::+0::x0::S=0");
//
//
//	if (!stropen(svr, psinfo->m_ConnectType, STR_MODE_R, path))
//	{
//		char msg[MAX_MSGSIZE] = { 0 };
//		sprintf(msg, "Error: %s cannot open!", path);
//		////////sendmsginfo(msg);
//		strclose(svr);
//		return -1;
//	}
//
//	char Msg[MAX_MSGSIZE] = { 0 };
//	sprintf(Msg, "> %s is connecting ... ... !", psinfo->MountPoint);
//	////////sendmsginfo(Msg);
//	int cntsize = 0; // 最大连接数
//	while (p<buff + MAXSRCTBL - 1) {
//		if (ns = strread(svr, (unsigned char*)p, buff + MAXSRCTBL - p - 1))
//		{
//			char msg[MAX_MSGSIZE] = { 0 };
//			sprintf(msg, "> %s is connected!", path);
//			////////sendmsginfo(msg);
//			p += ns;
//			break;
//		}
//		Sleep(NTRIP_CYCLE);
//		stat = strstat(svr, msg);
//		if (stat<0) break;
//		/*if ((NTRIP_CYCLE*cntsize++)>NTRIP_TIMEOUT) {
//
//		char msg[MAX_MSGSIZE]={0};
//		sprintf(msg,"Error: %s connection is timeout!",path);
//		sendmsginfo(msg);
//
//		strclose(svr);
//		strclose(&psinfo->psock_f);
//		strclose(&psinfo->psock_s);
//		return -1;
//		}*/
//	}
//	if (strstr(buff, NTRIP_RSP_OK_CLI) || (buff[0] == '\r'&&buff[1] == '\n') || (buff[0] != '\0'))
//	{
//		int tick;
//		unsigned char Buff[MAX_BUFFSIZE];
//		int cputime = 0;
//		while (svr->state)
//		{
//			//			tick = tickget();
//			memset(Buff, 0, MAX_BUFFSIZE);
//			if (0<(ns = strread(svr, Buff, MAX_BUFFSIZE)))
//			{
//				// 图形界面状态栏显示
//				int obsflag = 0;
//				int navflag = 0;
//				int posflag = 0;
//				// 图形界面柱状图显示
//				////////if (theApp.m_hwndShow)
//				////////	::SendMessage(theApp.m_hwndShow, WM_DATA_RECEIVED, (WPARAM)psinfo, (LPARAM)ns);
//
//				// 				obs_t* obst=NULL;
//				// 				nav_t* navt=NULL;
//				// 				sta_t* stat=NULL;
//				// 				gtime_t rcvtime;
//				//				convertUNCH2CH(Buff_char,Buff,ns);
//
//				int ret = psinfo->sitetype;
//				////////int MesageReturn = ReadRTKData((char*)Buff, ns, psinfo); //进入RTCM数据读取解析部分
//
//				////////if (MesageReturn & 0x01 == 1)
//				////////{
//				////////	obsflag += 1;
//				////////}
//
//				////////if (MesageReturn & 0x02 == 0x02)
//				////////{
//				////////	navflag += 1;
//				////////}
//
//				////////if (MesageReturn & 0x04 == 0x04)
//				////////{
//				////////	posflag += 1;
//				////////}
//
//				char msg[MAX_MSGSIZE] = { 0 };
//				sprintf(msg, "> %s received : %8d bytes  obs:%-3s  nav:%-3s  pos:%-3s", psinfo->MountPoint, ns, (obsflag ? "yes" : "no"), (navflag ? "yes" : "no"), (posflag ? "yes" : "no"));
//				////////sendmsginfo(msg, STRMSG);
//				if (0 < navflag)
//				{
//					// 					lock(&nav_lock);
//					// 					navcpy(&prtcm->nav, &gBrdc.navt);
//					// 					if(gBrdc.navt.n+gBrdc.navt.ng)
//					// 						gBrdc.IsUpdate = true;
//					// 				
//					// 					unlock(&nav_lock);		
//				}
//			}
//			// 			cputime = (int)(tickget() - tick);
//			// 			sleepms(1000-cputime);
//		}
//		// 		if(rawfile)fclose(rawfile);
//		strclose(svr);
//		return 2;
//	}
//	else if (strstr(buff, NTRIP_RSP_SRCTBL))
//	{
//		return 1;
//	}
//	else if (strstr(buff, NTRIP_PSP_ERROR))
//	{
//		return 0;
//	}
	return 0;
}



static DWORD WINAPI strstop(void *arg)
{
//	CNtripSocket* psinfo = (CNtripSocket*)arg;
//	stream_t *svr = &psinfo->psock;
//
//#ifdef WIN32
//	svr->state = 0;
//	if (psinfo->hsock == INVALID_HANDLE_VALUE)
//		return 1;
//	WaitForSingleObject(psinfo->hsock, 100);
//	CloseHandle(psinfo->hsock);
//	// show in the picture
//	char msg[MAX_MSGSIZE] = { 0 };
//	sprintf(msg, "> %s is closed!", psinfo->MountPoint);
//	////////sendmsginfo(msg);
//#else
//	pthread_join(svr->thread, NULL);
//#endif
	return 0;
}

// CNtripSocket
CNtripSocket::CNtripSocket()
{
	//srand((int)time(NULL));
	//IP = "";
	//Port = "";
	//User = "";
	//Pwd = "";
	//MountPoint = "";
	//sitelable = "";
	//sitetype = 2;
	//netname = "";
	//Port_s = "";
	//BPS = "115200";
	//memset(XYZ, 0.0, sizeof(double) * 3);
	//memset(BLH, 0.0, sizeof(double) * 3);
	//m_Connect = false;
	//m_ConnectType = CONNECT_STR_NONE;
	//m_MsgType = STRFMT_RTCM3;
	//memset(buff, 0, sizeof(buff));
	//m_nLength = 0;
	//initlock(&m_SocketLock); //for syn
	//						 //  	initlock(&m_navlock);
	//initlock(&m_obslock);
	////  	initlock(&m_vellock);
	//initlock(&m_poslock);
	//conv = strconvnew(STRFMT_RTCM3, STRFMT_RTCM3, "rinex3,30", 1, 1, "raw");
	//rtcm_fwrt.flag = TRUE;
	//rtcm_fwrt.last_time = conv->rtcm.time = timeget();
	//rtcm_fwrt.m_NetName = _T("");
	//memset(rtcm_fwrt.navpath, 0, sizeof(char)*MAX_PATH);
	//memset(rtcm_fwrt.obspath, 0, sizeof(char)*MAX_PATH);
	//memset(rtcm_fwrt.workpath, 0, sizeof(char)*MAX_PATH);
	//memset(rtcm_fwrt.rawpath, 0, sizeof(char)*MAX_PATH);

	//strinit(&psock);
	//hsock = NULL;
	//hdata = NULL;
	////char filepath[] = "D://KUN106240.15o";
	////FILE* fp;
	////if(fp = fopen(filepath,"w"))
	////	svr.outrnxobsh(fp,topt,&conv->rtcm.nav);
	////fclose(fp);
}

CNtripSocket::~CNtripSocket()
{
	//if (m_Connect)
	//{
	//	LinkClose();
	//}
	//m_ConnectType = CONNECT_STR_NONE;
	//m_nDelLength = 0;
	//m_Status = CONNECT_CLOSE;
	//m_nDelLength = 0;
	//DeleteCriticalSection(&m_SocketLock);
	//DeleteCriticalSection(&m_obslock);
	//// 	DeleteCriticalSection(&m_navlock);
	//// 	DeleteCriticalSection(&m_vellock);
	//DeleteCriticalSection(&m_poslock);
	//free_rtcm(&conv->rtcm);
	//free_rtcm(&conv->out);
	//free_raw(&conv->raw);
	//free(conv); conv = NULL;
	//// 	delete opt;opt=NULL;
}

// CNtripSocket member functions
void CNtripSocket::LinkInitial(ConnectType ConType, QString IP, QString Port, QString User, QString Password, QString MountPoint)
{
	//this->IP = IP.empty() ? "" : IP;
	//this->Port = Port == "" ? "2101" : Port;
	//this->User = User.empty() ? "" : User;
	//this->Pwd = Password.empty() ? "" : Password;
	//this->MountPoint = MountPoint.empty() ? "" : MountPoint;
	//this->m_Connect = false;
	//this->m_ConnectType = ConType;
}

void CNtripSocket::connect()
{
//#ifdef WIN32
//	//if (!psock.state && !(hsock=CreateThread(NULL,0,strthread,this,0,NULL))) {
//
//	char msg[MAX_MSGSIZE] = { 0 };
//	//sprintf(msg,"port is %s,bps is %s",this->Port_s,this->BPS);
//	//sendmsginfo(msg);
//	CNtripSocket *psinfo = (CNtripSocket *)this;
//
//
//	if (m_MsgType == STRFMT_OEM6 || m_MsgType == STRFMT_RTCM3)
//	{
//		strsvr_t *strvr;
//		strvr = (strsvr_t *)malloc(sizeof(strsvr_t));
//		strsvrinit(strvr, 1);
//
//		//stream_t *svr=&psinfo->psock;//将套接字中的stream变量赋值
//		//stream_t *svr=(stream_t *)malloc(sizeof(stream_t));
//		stream_t *ssvr = (stream_t *)malloc(sizeof(stream_t));
//		stream_t *svr = (stream_t *)malloc(sizeof(stream_t));
//		strvr->stream[0] = *svr;
//		strvr->stream[1] = *ssvr;//将stream数据赋值过去
//
//
//		int str[2] = { STR_TCPCLI,STR_TCPSVR };
//		int str1[2] = { STR_SERIAL,STR_TCPSVR };
//		int str2[2] = { STR_TCPSVR,STR_TCPSVR };
//
//		int opt[7] = { 10000,10000,1000,32768,10,0,30 };//定义参数opt 在strsvr中是可设置的
//
//		char pathin[256] = { 0 }, pathout[256] = { 0 };
//		//strinitcom();
//		string port;
//		int PORT = stoi(psinfo->Port);
//		PORT = PORT + 1;
//		port = to_string(PORT + 1);//将input的端口号设置为链接的端口号+1 因为经历一个strsvr变换 不能使用同一端口号 cstring转为int+1再换回cstring
//		if (psinfo->m_ConnectType == STR_SERIAL)// 不管是串口还是网口进来的数据 最后转换都通过网口传出到解算线程中去
//		{
//			sprintf(pathin, "%s:%s:%s:%s:%s:%s", psinfo->Port_s, psinfo->BPS, "8", "n", "1", "off");
//		}
//		else
//		{
//			sprintf(pathin, "%s:%s@%s:%s/%s", psinfo->User, psinfo->Pwd, psinfo->IP, psinfo->Port, psinfo->MountPoint);
//		}
//		if (psinfo->MountPoint == "BASE")
//		{
//			sprintf(pathout, "%s:%s@%s:%s/%s", psinfo->User, psinfo->Pwd, "127.0.0.1", "30001", psinfo->MountPoint);
//		}
//		else if (psinfo->MountPoint == "ROV1")
//		{
//			sprintf(pathout, "%s:%s@%s:%s/%s", psinfo->User, psinfo->Pwd, "127.0.0.1", "30002", psinfo->MountPoint);
//		}
//		else if (psinfo->MountPoint == "ROV2")
//		{
//			sprintf(pathout, "%s:%s@%s:%s/%s", psinfo->User, psinfo->Pwd, "127.0.0.1", "30003", psinfo->MountPoint);
//		}
//		else if (psinfo->MountPoint == "ROV3")
//		{
//			sprintf(pathout, "%s:%s@%s:%s/%s", psinfo->User, psinfo->Pwd, "127.0.0.1", "30004", psinfo->MountPoint);
//		}
//		else if (psinfo->MountPoint == "ROV4")
//		{
//			sprintf(pathout, "%s:%s@%s:%s/%s", psinfo->User, psinfo->Pwd, "127.0.0.1", "30005", psinfo->MountPoint);
//		}
//		/*char msg[MAX_MSGSIZE]={0};
//		sprintf(msg,"%s port in ;%s port out",port,psinfo->Port);
//		sendmsginfo(msg);*/
//		char *PATH[2] = { pathin,pathout };//path参数是stream参数中的变量
//		strconv_t *cconv = (strconv_t *)malloc(sizeof(strconv_t));//字节长初定义
//		if (m_MsgType == STRFMT_OEM6)
//		{
//			cconv = strconvnew(STRFMT_OEM6, STRFMT_RTCM3, "1006(1),1047(1),1074(1),1019(1),1124(1),1084(1)", 1, 1, "rtcm");
//		}//标准Novatel
//		else if (m_MsgType == STRFMT_RTCM3)
//		{
//			cconv = strconvnew(STRFMT_RTCM3, STRFMT_RTCM3, "1047(1)", 1, 1, "rtcm");
//		}//司南专用
//		 //cconv=strconvnew(STRFMT_RTCM3,STRFMT_RTCM3,"1074(1)",1,1,"rtcm");
//		strconv_t *conva[1] = { cconv };
//		if (psinfo->m_ConnectType == STR_TCPCLI)
//		{
//			if (!strsvrstart(strvr, opt, str, PATH, conva, NULL, NULL))//调用strsvrstart函数 在此函数中会进入线程strsvrthread 进行strsvr软件所做的工作
//			{
//				strclose(strvr->stream);//失败的话关闭的应该是strsvr_t中的stream 而非套接字中的stream
//
//			}
//		}
//		else if (psinfo->m_ConnectType == STR_SERIAL)
//		{
//			if (!strsvrstart(strvr, opt, str1, PATH, conva, NULL, NULL))//调用strsvrstart函数 在此函数中会进入线程strsvrthread 进行strsvr软件所做的工作
//			{
//				strclose(strvr->stream);//失败的话关闭的应该是strsvr_t中的stream 而非套接字中的stream
//
//			}
//		}
//		else if (psinfo->m_ConnectType == STR_TCPSVR)
//		{
//			if (!strsvrstart(strvr, opt, str2, PATH, conva, NULL, NULL))//调用strsvrstart函数 在此函数中会进入线程strsvrthread 进行strsvr软件所做的工作
//			{
//				strclose(strvr->stream);//失败的话关闭的应该是strsvr_t中的stream 而非套接字中的stream
//
//			}
//		}
//
//		psinfo->psock_f = strvr->stream[0];
//		psinfo->psock_s = strvr->stream[1];
//	}
//
//	char msgg[MAX_MSGSIZE] = { 0 };
//	sprintf(msgg, "%d enter connect(),mode is %d,type is %d", psinfo->m_MsgType, psinfo->psock.mode, psinfo->psock.type);
//	////////sendmsginfo(msgg);
//	if (!(hsock = CreateThread(NULL, 0, strthread, psinfo, 0, NULL)))//连接线程 进行文件读取，封装，为以后的解算线程做数据准备
//	{
//
//#else
//	if (pthread_create(&hsock, NULL, strsvrthread, svr)) {
//#endif
//		strclose(&psock);
//		;
//	}
	}

void CNtripSocket::close()
{
//	if (!psock.state) {
//		return;
//	}
//	else {
//#ifdef WIN32
//		if (psock.state && !(hsock = CreateThread(NULL, 0, strstop, this, 0, NULL)))
//#else
//		if (pthread_create(&hsock, NULL, strsvrthread, svr))
//#endif
//			strclose(&psock);
//	}
//	// clear obs data
//	// 	lock(&m_obslock);
//	// 	auto obsi=obs_quene.begin();
//	// 	while(obsi!= obs_quene.end()){
//	// 		freeobs(obsi->second);
//	// 		++obsi;
//	// 	}
//	// 	obs_quene.clear();
//	// 	unlock(&m_obslock);
}

int CNtripSocket::LinkClose()
{

////	try
//	{
//		Close();
//		Sleep(100);
//	/*	string* Msg = new string;
//		Msg->Format("IP: %s:%d:%s 连接关闭！\r\n", IP, Port, MountPoint);
//		::SendMessage(AfxGetMainWnd()->GetSafeHwnd(), WM_OUTPUT_MSG, (WPARAM)Msg, (LPARAM)0);
//		TRACE("IP: %s:%d:%s 连接关闭！\r\n", this->IP, this->Port, this->MountPoint);*/
//		m_Connect = false;
//	}
	//catch (Exception* pException)
	//{
	//	pException->Delete();
		return -1;
	//}
}





