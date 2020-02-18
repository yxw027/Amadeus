#pragma once

#include <QObject>
#include <QMap>
#include <QString>

#include "Communication\NtripClient.h"
//#include "Communication\NtripServer.h"

// ������Ϣ�ṹ��
typedef struct netinfo
{
	QString	NETNAME;	// ��������
	QString	WORKPATH;	// ����Ŀ¼
	int	SLNSYS;			// ��������ϵͳ sys 0=gps, 1=cmp, 2=gps+cmp
	int	SLNPHS;			// ����Ƶ�� phrase 0=single, 1=double
	int	SLNSES;			// ��̬��ʱ�� ses 0=15m 1=30m 2=1h 3=1d
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

// ��վ�ṹ���ֵ�
typedef QMap<QString, CNtripClient*>  SiteList;


// �����ṹ��
typedef struct NetStr {
	HANDLE		m_hsln;		// �����߳̾��
	bool		m_runflag;	// �����߳�״̬
	SiteList	m_SiteList;	// ��վ�ṹ���ֵ�
	TriArray	m_TriList;
	netinfo		m_Netinfo;	// ������Ϣ�ṹ��
//	CNtripServer*	m_Server;
};
// �����ṹ���ֵ�
typedef QMap<QString, NetStr* > NetList;


// APP�ṹ��
class EBAS_V_UI_RApp : public QObject
{
	Q_OBJECT

public:
	EBAS_V_UI_RApp();
	~EBAS_V_UI_RApp();

public:
	NetList	m_NetList;	    // �����ṹ���ֵ�
	lock_t	m_rtcm_lock;	// ������
	lock_t	m_proc_lock;	// ������
	lock_t	m_show_lock;    // ��ʾ��
};

extern EBAS_V_UI_RApp theApp;