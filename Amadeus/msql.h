#pragma once

#include <QObject>

class msql : public QObject
{
	Q_OBJECT

public:
	msql();
	~msql();

	bool cfginit();
	QString getdbpath();
	QString getdbname();
	QString getuser();
	QString getpwd();

private:
	long	state;
	QString dbpath;  // 服务器名称
	QString dbname;  // 数据库名称
	QString user;    // 登陆用户名
	QString pwd;	 // 登陆密码
};