#pragma once

#include <QObject>
#include <map>
#include <string>

#include "Communication\NtripClient.h"
//#include "Communication\NtripServer.h"

using namespace std;

typedef struct TriNet
{
	string   PointArray[3];
	double   pos[3][2];
};
typedef map<int, TriNet*> TriArray;

// 测站-> 通信结构体
typedef map<string, CNtripClient*>  SiteList;

// 控制台结构体
typedef struct NetStr {
	HANDLE	m_hsln;	// 解算线程句柄
	bool	m_runflag;
//	CNtripServer*	m_Server;
	SiteList	m_SiteList;
	TriArray	m_TriList;
//	netinfo	m_Netinfo;
};
// 子网-> 测站-> 通信结构体
typedef map<string, NetStr* > NetList;

class EBAS_V_UI_RApp : public QObject
{
	Q_OBJECT

public:
	EBAS_V_UI_RApp(QObject *parent);
	~EBAS_V_UI_RApp();

public:
	NetList	m_NetList;	    // 用于WORK线程处理的容器
	lock_t	m_rtcm_lock;	// rtcm
	lock_t	m_proc_lock;	// 解算锁
	lock_t	m_show_lock;    // 显示锁
};

extern EBAS_V_UI_RApp theApp;