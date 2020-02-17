#pragma once

#include <QString>

//namespace _globalfunc
//{

	//typedef struct _fittedcoef
	//{
	//	double a; // y=ax+b
	//	double b; // y=ax+b
	//};

	//typedef struct _cfg
	//{
	//	BOOL	runflag;	//ϵͳ���б�ʶ
	//	int	slnsys;	//gnss system used in solution
	//	CString	netname;	//net name
	//	CString	workpath;	//ϵͳĿ¼ 
	//};

	//typedef struct _shwdpos {
	//	double	delx[5]; // 0=[real] 1=[hour] 2=[day] 3=[any] 4=[flt]
	//	double	dely[5]; // 0=[real] 1=[hour] 2=[day] 3=[any] 4=[flt]
	//	double	delz[5]; // 0=[real] 1=[hour] 2=[day] 3=[any] 4=[flt]
	//	double	delr[5]; // 0=[real] 1=[hour] 2=[day] 3=[any] 4=[flt]
	//};

	//typedef struct _showspos {
	//	short	sys;	// gnss system
	//	short	slntype;	// solution type 1=realtime, 2=hourly, 3=daily
	//	int	svn;	// validity satellite number
	//	_shwdpos	delpos;	// delx y z r of position
	//};

//	typedef struct netinfo
	struct netinfo
	{
		QString	NETNAME;
		QString	WORKPATH; // workpath
		int	SLNSYS; // sys 0=gps, 1=cmp, 2=gps+cmp
		int	SLNPHS; // phrase 0=single, 1=double
		int	SLNSES; // ses 0=15m 1=30m 2=1h 3=1d
		QString	OWNER; // charge
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

	//enum fitedmode {
	//	LINEM = 1,
	//};

	////	void QueryFromDatabase(CDatabase* db, CString query, CStringArray* Des);
	//// ����ϵͳ����
	//void UpdateCfg(void);
	//// ���²�վ�б�
	//void UpdateNetList(void);
	//// ��ȡ����IP��ַ�����ػػ���ַ
	//// add by hjq 2016-01-11
	//CString GetHostIP(void);
	//// ��������Ŀ¼ 
	//void DirPrepare(CString& Workpath, int year, int doy);
	//void DirPrepare(CString& Workpath, COleDateTime& T);
	//void DirPrepare(CString& Workpath, time_t time);
	//// IGU ��Ʒ����
	//int igu_download(void* arg);
//}