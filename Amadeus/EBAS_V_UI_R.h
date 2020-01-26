#pragma once

#include <QObject>
#include <Qthread>
using namespace std;

typedef struct TriNet
{
	QString   PointArray[3];
	double    pos[3][2];
};

typedef map<int, TriNet*> TriArray;

// 控制台结构体
typedef struct NetStr {
	QThread	*m_hsln;	// 解算线程句柄
	bool	m_runflag;
//	CNtripServer*	m_Server;
	//global_turui*	pGLB;
//	SiteList	m_SiteList;
	TriArray	m_TriList;
//	netinfo	m_Netinfo;
};

// 子网-> 测站-> 通信结构体
typedef std::map<QString, NetStr* > NetList;

class CEBAS_V_UI_RApp : public QObject
{
	Q_OBJECT

public:
	CEBAS_V_UI_RApp(QObject *parent);
	~CEBAS_V_UI_RApp();

public:
	NetList	m_NetList;	    // 用于WORK线程处理的容器
//	lock_t	m_rtcm_lock;	// rtcm
//	lock_t	m_proc_lock;	// 解算锁
//	lock_t	m_show_lock;    // 显示锁
};
