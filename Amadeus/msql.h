#pragma once

#include <QObject>
#include <QSqlDatabase>

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
	int gettblist(QStringList& tblist);
	int getlist(QString field, QStringList& list, QString cmd);

	QString getdbpath();
	QString getdbname();
	QString getuser();
	QString getpwd();

private:
	QSqlDatabase DB;
	//long	state;
	QString dbpath;  // 服务器名称
	QString dbname;  // 数据库名称
	QString user;    // 登陆用户名
	QString pwd;	 // 登陆密码
};