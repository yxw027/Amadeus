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

// ��վ-> ͨ�Žṹ��
typedef map<string, CNtripClient*>  SiteList;

// ����̨�ṹ��
typedef struct NetStr {
	HANDLE	m_hsln;	// �����߳̾��
	bool	m_runflag;
//	CNtripServer*	m_Server;
	SiteList	m_SiteList;
	TriArray	m_TriList;
//	netinfo	m_Netinfo;
};
// ����-> ��վ-> ͨ�Žṹ��
typedef map<string, NetStr* > NetList;

class EBAS_V_UI_RApp : public QObject
{
	Q_OBJECT

public:
	EBAS_V_UI_RApp(QObject *parent);
	~EBAS_V_UI_RApp();

public:
	NetList	m_NetList;	    // ����WORK�̴߳��������
	lock_t	m_rtcm_lock;	// rtcm
	lock_t	m_proc_lock;	// ������
	lock_t	m_show_lock;    // ��ʾ��
};

extern EBAS_V_UI_RApp theApp;