#pragma once

#include <QObject>
#include <QMap>
#include <QString>

#include "Communication\NtripClient.h"
//#include "Communication\NtripServer.h"

// 子网信息结构体
typedef struct netinfo
{
	QString	NETNAME;	// 子网名称
	QString	WORKPATH;	// 工作目录
	int	SLNSYS;			// 解算卫星系统 sys 0=gps, 1=cmp, 2=gps+cmp
	int	SLNPHS;			// 解算频点 phrase 0=single, 1=double
	int	SLNSES;			// 静态解时长 ses 0=15m 1=30m 2=1h 3=1d
	QString	OWNER;
	QString	PINCHARGE;
	QString	PHONE;
	QString	EMAIL;
	double	COLAT;
	double	COLON;
	int	BASENUM;
	int	ROVERNUM;
	int	BRDCTYPE;
	QString	COMMENT;
};

typedef struct TriNet
{
	QString   PointArray[3];
	double	pos[3][2];
};
typedef QMap<int, TriNet*> TriArray;

// 测站结构体字典
typedef QMap<QString, CNtripClient*>  SiteList;


// 子网结构体
typedef struct NetStr {
	HANDLE		m_hsln;		// 解算线程句柄
	bool		m_runflag;	// 解算线程状态
	SiteList	m_SiteList;	// 测站结构体字典
	TriArray	m_TriList;
	netinfo		m_Netinfo;	// 子网信息结构体
//	CNtripServer*	m_Server;
};
// 子网结构体字典
typedef QMap<QString, NetStr* > NetList;


// APP结构体
class EBAS_V_UI_RApp : public QObject
{
	Q_OBJECT

public:
	EBAS_V_UI_RApp();
	~EBAS_V_UI_RApp();

public:
	NetList	m_NetList;	    // 子网结构体字典
	lock_t	m_rtcm_lock;	// 解析锁
	lock_t	m_proc_lock;	// 解算锁
	lock_t	m_show_lock;    // 显示锁
};

extern EBAS_V_UI_RApp theApp;