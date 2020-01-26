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

// ����̨�ṹ��
typedef struct NetStr {
	QThread	*m_hsln;	// �����߳̾��
	bool	m_runflag;
//	CNtripServer*	m_Server;
	//global_turui*	pGLB;
//	SiteList	m_SiteList;
	TriArray	m_TriList;
//	netinfo	m_Netinfo;
};

// ����-> ��վ-> ͨ�Žṹ��
typedef std::map<QString, NetStr* > NetList;

class CEBAS_V_UI_RApp : public QObject
{
	Q_OBJECT

public:
	CEBAS_V_UI_RApp(QObject *parent);
	~CEBAS_V_UI_RApp();

public:
	NetList	m_NetList;	    // ����WORK�̴߳��������
//	lock_t	m_rtcm_lock;	// rtcm
//	lock_t	m_proc_lock;	// ������
//	lock_t	m_show_lock;    // ��ʾ��
};
