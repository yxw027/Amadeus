#pragma once

#include "EBAS_V_UI_R.h"
#include <QObject>
#include <QSqlDatabase>

const QString NET_CFG_TB = "Config_DetectNet";
const QString SITE_CFG_TB = "Config_DetectStation";

const QString NETNAME_TB = "DetectNetName";
const QString SITENAME_TB = "DetectStationName";

enum connectState { StateOpen, StateClose };

class msql : public QObject
{
	Q_OBJECT

public:
	msql();
	~msql();

	bool cfginit();
	bool connect();
	bool connect(QString IP, QString username, QString password);
	bool connect(QString IP, QString dbname, QString username, QString password);
	void savecfg();
	void close();

	int gettblist(QStringList& tblist);
	int getlist(QString field, QStringList& list, QString cmd);
	int getnetlist(QStringList& netlist);
	int getsitelist(QString netname, QStringList& sitelist);
	int getsiteinfo(QString sitename, QStringList& infolist, int flag = 0);

	bool netIsExist(QString netname);
	void insert_net2db(const netinfo* pnet);

	void update_netinfo2str(QString NetName, netinfo* pnet);

	QString getdbpath();
	QString getdbname();
	QString getuser();
	QString getpwd();
	void setdbpath(QString arg);
	void setdbname(QString arg);
	void setuser(QString arg);
	void setpwd(QString arg);

	int getstate();

private:
	bool fieldcheck(QString indx_field, QString obj_field, QString val);
	int countQueryRecord(QSqlQuery query);

private:
	QSqlDatabase DB;
	QString dbpath;  // 服务器名称
	QString dbname;  // 数据库名称
	QString user;    // 登陆用户名
	QString pwd;	 // 登陆密码

	connectState state;	// 数据库连接状态标志位
};

extern msql m_sql;