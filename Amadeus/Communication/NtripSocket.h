#pragma once
#include <iostream>
#include "Nrtk\rtklib.h"
#include <map>
//#include "rsltprc.h"

#include <QString>
#include <QVector>
#define CString QString
#define CStringArray QVector<QString>

using namespace std;

// CNtripSocket command target
const size_t MAX_BUFFSIZE = 4096;//4096;

static DWORD WINAPI thread_pro(void* arg);
// ���÷���:HANDLE  thread = CreateThread(NULL,0,writernx,this,0,NULL);

typedef struct wrt_rnx
{
	BOOL	flag; // true= record ; flase= not
	char	workpath[MAX_PATH];
	char	obspath[MAX_PATH];
	char	navpath[MAX_PATH];
	char	rawpath[MAX_PATH];
	gtime_t	last_time;
	CString	m_NetName;
};

typedef struct _showpos
{
	char	sys;	//����ϵͳ
	int	prn;	//���Ǻ�
	double	ath;	//��λ��
	double	ele;	//�߶Ƚ�
	double  pdop; //PDOP
	double  gdop; //GDOP
	double  hdop; //HDOP
	double  vdop; //VDOP
	double	srn[3];	//�����
}showpos;

typedef map<char, map<int, struct _showpos> > _satinfo;

typedef struct _showsignal
{
	CString	sitename;
	obs_t	obsinfo;
};


typedef map<CString, _satinfo> satinfomap;

typedef struct _SatInfo
{
	bool	update;	//���±�ʶ
	CString	netname;	//��վ����
	satinfomap info;
};

enum POSSLNTYPE{
	POSREAL	=0, // ʵʱ��
	POSHOUR	=1, // Сʱ��
	POSDAY	=2,	// ���
	POSSTA	=3,	// ��̬��
	POSFLT	=4	// �˲���
};


// typedef struct del_unit
// {
// 	char    sitename[MAX_PATH];
// 	int	sys;
// 	int	slntype;
// 	double	del[5][5]; // 0= delx, 1= dely, 2= delz, 3= vsn, 4= ratio
// 	time_t	time;
// };


// typedef struct pos_del
// {
// 	del_unit	posinfo;
// };

// information lables in source table
const char* const INFORMATION_DSC[] = {
	"Stream",
	"Mountpoint",
	"Identifier", 
	"Format", 
	"Details",
	// 5
	"Carrier", 
	"System",
	"NetWork",
	"Country",
	"Latitude", 
	// 10
	"Longitude",
	"Client must send NMEA",
	"Solution",
	"Generator",
	"Compression",
	// 15
	"Authentication",
	"Registration",
	"Bitrate",
	"Micellaneous"
};

#define NTRIP_CLI_PORT	2101	/* default ntrip-client connection port */
#define NTRIP_SVR_PORT	80	/* default ntrip-server connection port */
#define NTRIP_MAXRSP	32768	/* max size of ntrip response */
#define NTRIP_MAXSTR	256	/* max length of mountpoint string */
#define MAX_CONNECTION	300	/* max size of connection */
#define MAX_MSGSIZE	1024	/* max size of message */

#define PRGNAME	"Ntrip Browser"
#define NTRIP_HOME	"rtcm-ntrip.org:2101"	// caster list home
#define NTRIP_TIMEOUT	8000	// response timeout (ms)
#define NTRIP_CYCLE	50	// processing cycle (ms)
#define MAXSRCTBL	512000	// max source table size (bytes)

#define NTRIP_RSP_OK_CLI	"ICY 200 OK\r\n"	/* ntrip response: client */
#define NTRIP_RSP_OK_SVR	"OK\r\n"	/* ntrip response: server */
#define NTRIP_RSP_SRCTBL	"SOURCETABLE 200 OK\r\n"	/* ntrip response: source table */
#define NTRIP_RSP_TBLEND	"ENDSOURCETABLE"	
#define NTRIP_PSP_ERROR	"401 Unauthorized"	
#define NTRIP_RSP_HTTP	"HTTP/"	/* ntrip response: http */

enum ConnectType
{
	CONNECT_STR_NONE	= 0,	  /* stream type: none */
	CONNECT_STR_SERIAL	= 1,	  /* stream type: serial */ 
	CONNECT_STR_FILE	= 2,	  /* stream type: file */ 
	CONNECT_STR_TCPSVR	= 3,	  /* stream type: TCP server */ 
	CONNECT_STR_TCPCLI	= 4,	  /* stream type: TCP client */
	CONNECT_STR_UDP	= 5,	  /* stream type: UDP stream */ 
	CONNECT_STR_NTRIPSVR	= 6,	  /* stream type: NTRIP server */
	CONNECT_STR_NTRIPCLI	= 7,	  /* stream type: NTRIP client */  
	CONNECT_STR_FTP	= 8,	  /* stream type: ftp */
	CONNECT_STR_HTTP	= 9	  /* stream type: http */
};

enum ConnectStatus
{
	CONNECT_CLOSE    = 0,
	CONNECT_WARNING  = 1,
	CONNECT_LINK     = 2
};

class CNtripSocket : public CAsyncSocket
{
private:
	//int encbase64(char *str, const unsigned char *byte, int n);

public:
	ConnectType	m_ConnectType;
	BOOL	m_Connect;
	UINT	m_Status;                // ��վ״̬
	size_t	m_nLength;
	size_t	m_nDelLength;
	CString	Port;
	CString	IP;
	CString	sitelable;	// add by hjq 2016-04-24
	CString	netname;	// add by hjq 2016-04-24
	CString Port_s;//���ں�
	CString BPS;//������
	int	sitetype;	// 1=[base] 2=[rover] or 3=[brdc]
	int	m_MsgType; // massege type  rtcm ubx rt17 and so on
 	double       BLH[3];
 	double       XYZ[3];
	CString      User;
	CString      Pwd;
	CString      MountPoint;             // ���ص�
	int	MountNumber;						//���ص���
	CStringArray Information;            // Source Table ��ѡ�еĹ��ص���Ϣ 
	BYTE         buff[MAX_BUFFSIZE];

	BOOL	 openState; //�̴߳򿪱�־λ
// rtcm ����
public:
 	CRITICAL_SECTION	  m_SocketLock;
// 	CRITICAL_SECTION	  m_navlock;
 	CRITICAL_SECTION	  m_obslock;
// 	CRITICAL_SECTION	  m_vellock;
 	CRITICAL_SECTION	  m_poslock;
 
 	stream_t	psock;
	stream_t    psock_f;
	stream_t    psock_s;
 	HANDLE	hsock;
	HANDLE	hdata;
 	strconv_t*	conv;
// 	rnxopt_t*	opt;
 	gtime_t	last_time;	// ��¼����������Ԫʱ�䣬��ֹ�������
 	wrt_rnx	rtcm_fwrt;	// �ļ����rtcm
 	map<long, obs_t*>	obs_quene;	// �۲���Ԫ��¼
 	map<long, nav_t>	nav_quene;	// �����ļ���¼
 	map<int, struct _showpos> satpos;
// 	rsltprc	flttool;	// ����˲�����
// 	double	sitevel[3][6];	// ��վ�仯���� Сʱ ���� ����;  V = ([3] - [0])/del_t

public:
	CNtripSocket();
	virtual ~CNtripSocket();
public:
	virtual void   OnConnect(int nErrorCode);
	virtual void   OnClose(int nErrorCode);
	virtual void   OnReceive(int nErrorCode);
	virtual void   OnSend(int nErrorCode);
	virtual void   OnAccept(int nErrorCode);
	void connect();	// connect by using rtklib
	void close();	// close by using rtklib

	int            SendRequest(void);
	void           LinkInitial(ConnectType ConType, CString IP, CString Port, CString User, CString Password, CString MountPoint);

	int            LinkConnect();
	int            Linkconnect(ConnectType ConType, CString IP, CString Port);
	int            LinkConnect(ConnectType ConType, CString IP, CString Port, CString User, CString Password, CString MountPoint);

	int            LinkClose();

	BYTE*     CString2BYTE(CString cstr);
	//int       CString2BYTE(CString cstr, BYTE* Buff);    // δʵ��
	CString   BYTE2CString(BYTE* byte);
	CString   BYTE2CString(BYTE* byte, size_t nLength);  // δʵ��
	UINT      SplitCString(CString& str,CStringArray& Arr,CString strDiv);
	UINT      SplitCString(CString& str,CStringArray& Arr,char ch);
private:
	int TCPServerConnect(void);
//	inline bool TimeOutControl(void);
	int TCPClientConnect(CString Address, UINT PORT);
	int NtripClientConnect(CString Address , UINT PORT, CString USER, CString PWD, CString MP);
public:
//	void writernx(void);
};

//extern void navcpy(void *input, void * output);


