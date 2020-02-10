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
	QString dbpath;  // ����������
	QString dbname;  // ���ݿ�����
	QString user;    // ��½�û���
	QString pwd;	 // ��½����
};