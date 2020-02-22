#pragma once
#include <QObject>
#include "..\Nrtk\rtklib.h"
#include <QMap>

const size_t MAX_BUFFSIZE = 4096;//4096;

typedef struct wrt_rnx
{
	BOOL	flag; // true= record ; flase= not
	char	workpath[MAX_PATH];
	char	obspath[MAX_PATH];
	char	navpath[MAX_PATH];
	char	rawpath[MAX_PATH];
	gtime_t	last_time;
	QString	m_NetName;
};

// �����ڱ��ֻ࣬��DlgSiteSignal.cpp��ʹ�ã�������Ҫ����
typedef struct _showsignal
{
	QString	sitename;
	obs_t	obsinfo;
};

typedef struct _showpos
{
	char	sys;	// ����ϵͳ
	int		prn;	// ���Ǻ�
	double	ath;	// ��λ��
	double	ele;	// �߶Ƚ�
	double  pdop;	// PDOP
	double  gdop;	// GDOP
	double  hdop;	// HDOP
	double  vdop;	// VDOP
	double	srn[3];	// �����
}showpos;

enum POSSLNTYPE {
	POSREAL = 0, // ʵʱ��
	POSHOUR = 1, // Сʱ��
	POSDAY = 2,	// ���
	POSSTA = 3,	// ��̬��
	POSFLT = 4	// �˲���
};

#define MAX_MSGSIZE	1024	/* max size of message */
#define NTRIP_CYCLE	50	// processing cycle (ms)
#define MAXSRCTBL	512000	// max source table size (bytes)

#define NTRIP_RSP_OK_CLI	"ICY 200 OK\r\n"	/* ntrip response: client */
#define NTRIP_RSP_SRCTBL	"SOURCETABLE 200 OK\r\n"	/* ntrip response: source table */
#define NTRIP_RSP_TBLEND	"ENDSOURCETABLE"	
#define NTRIP_PSP_ERROR	"401 Unauthorized"	

enum ConnectType
{
	CONNECT_STR_NONE = 0,	  /* stream type: none */
	CONNECT_STR_SERIAL = 1,	  /* stream type: serial */
	CONNECT_STR_FILE = 2,	  /* stream type: file */
	CONNECT_STR_TCPSVR = 3,	  /* stream type: TCP server */
	CONNECT_STR_TCPCLI = 4,	  /* stream type: TCP client */
	CONNECT_STR_UDP = 5,	  /* stream type: UDP stream */
	CONNECT_STR_NTRIPSVR = 6,	  /* stream type: NTRIP server */
	CONNECT_STR_NTRIPCLI = 7,	  /* stream type: NTRIP client */
	CONNECT_STR_FTP = 8,	  /* stream type: ftp */
	CONNECT_STR_HTTP = 9	  /* stream type: http */
};

enum ConnectStatus
{
	CONNECT_CLOSE = 0,
	CONNECT_WARNING = 1,
	CONNECT_LINK = 2
};

class CNtripSocket : public QObject
{
private:
	//int encbase64(char *str, const unsigned char *byte, int n);

public:
	ConnectType	m_ConnectType;
	BOOL	m_Connect;
	UINT	m_Status;                // ��վ״̬
	size_t	m_nLength;
	size_t	m_nDelLength;
	QString	Port;
	QString	IP;
	QString	sitelable;	// add by hjq 2016-04-24
	QString	netname;	// add by hjq 2016-04-24
	QString Port_s;//���ں�
	QString BPS;//������
	int	sitetype;	// 1=[base] 2=[rover] or 3=[brdc]
	int	m_MsgType; // massege type  rtcm ubx rt17 and so on
	double       BLH[3];
	double       XYZ[3];
	QString      User;
	QString      Pwd;
	QString      MountPoint;             // ���ص�
	int	MountNumber;						//���ص���
	QStringList Information;            // Source Table ��ѡ�еĹ��ص���Ϣ 
	BYTE         buff[MAX_BUFFSIZE];

	BOOL	 openState; //�̴߳򿪱�־λ
						// rtcm ����
public:
	CRITICAL_SECTION	  m_SocketLock;
	CRITICAL_SECTION	  m_obslock;
	CRITICAL_SECTION	  m_poslock;

	stream_t	psock;
	stream_t    psock_f;
	stream_t    psock_s;
	HANDLE	hsock;
	HANDLE	hdata;
	strconv_t*	conv;
	gtime_t	last_time;	// ��¼����������Ԫʱ�䣬��ֹ�������
	wrt_rnx	rtcm_fwrt;	// �ļ����rtcm
	QMap<long, obs_t*>	obs_quene;	// �۲���Ԫ��¼
	QMap<long, nav_t>	nav_quene;	// �����ļ���¼
	QMap<int, struct _showpos> satpos;

public:
	CNtripSocket();
	virtual ~CNtripSocket();
public:
	void connect();	// connect by using rtklib
	void close();	// close by using rtklib
	void LinkInitial(ConnectType ConType, QString IP, QString Port, QString User, QString Password, QString MountPoint);
	int  LinkClose();
};


