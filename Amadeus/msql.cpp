#include "msql.h"
#include <QTextBlock>
#include <QTextStream>
#include <QSqlQuery>
#include <QMessageBox>

msql::msql()
{
	dbpath = "127.0.0.1";
	dbname = "";
	user = "admin";
	pwd = "123456";
	//state = false;

	QString * ccheck = new QString("");
}

msql::~msql()
{
}

bool msql::cfginit()
{
	QString filename = "..\\x64\\Debug\\sql.cfg";
	QFile ifile(filename);
	if (!ifile.open(QIODevice::ReadOnly | QIODevice::Text))
	{
		//sendmsginfo("Warning: .\\bin\\sql.cfg can not open!");
		return false;
	}

	QTextStream iStream(&ifile);
	QString line;
	while (!iStream.atEnd())
	{
		line = iStream.readLine();
		if (line.isEmpty())
			continue;
		else if (line.contains("IP:"))
			dbpath = line.mid(line.indexOf(":") + 1, line.length() - 1 - line.indexOf(":"));
		else if (line.contains("DB:"))
			dbname = line.mid(line.indexOf(":") + 1, line.size() - 1 - line.indexOf(":"));
		else if (line.contains("USER:"))
			user = line.mid(line.indexOf(":") + 1, line.size() - 1 - line.indexOf(":"));
		else if (line.contains("PWD:"))
			pwd = line.mid(line.indexOf(":") + 1, line.size() - 1 - line.indexOf(":"));
	}

	ifile.close();
	return true;
}

bool msql::connect()
{
	if (cfginit()) // read config file 
		return connect(dbpath, dbname, user, pwd);
}

bool msql::connect(QString IP, QString username, QString password)
{
	return connect(IP, "", username, password);
}

bool msql::connect(QString IP, QString dbname, QString username, QString password)
{
	DB = QSqlDatabase::addDatabase("QODBC3");

	// SQL server��MySQL��SQLite�Ȳ�ͬ��δ����DSN�ģ�ʹ���ֹ�ƴ��DSN�ַ�����������
	QString dsn = "DRIVER={SQL SERVER};""SERVER=%1;""DATABASE=%2;""UID=%3;"  "PWD=%4;";
	DB.setDatabaseName(dsn
		.arg(IP)
		.arg(dbname)
		.arg(username)
		.arg(password)
	);

	if(DB.open())
		//state = m_sqlcnt->GetState(); // connected
		return true;
	else
		//state = m_sqlcnt->GetState(); // connected
		return false;
}

int msql::gettblist(QStringList& tblist)
{
	getlist("NAME", tblist, "SELECT NAME FROM MASTER..SYSDATABASES ORDER BY NAME");
	return tblist.count();
}

int msql::getlist(QString field, QStringList& list, QString cmd)
{
	Q_ASSERT(!field.isEmpty());
	Q_ASSERT(!cmd.isEmpty());

	QSqlQuery query;
	query.exec(cmd);

	// query.size()ʼ�յ���-1����SQL SERVER������֧���йأ��ʴ˴���ʹ�øú������в�ѯ��¼�����жϣ�
	// ��ԭ�汾��m_sqlset->GetRecordCount()����Ҳͳ�Ʋ�׼ȷ����ֻ���з����жϣ�����ʹ��list.GetCount()׼ȷͳ�ƣ�

	//while (query.next())
	{
		QMessageBox::information(NULL, "Tips", query.value(0).toString());
	}

	return 0;



	//m_sqlset->MoveFirst();
	//while (!m_sqlset->adoEOF)
	//{
	//	QString sitename = m_sqlset->GetCollect((_variant_t)field);
	//	list.Add(sitename);
	//	m_sqlset->MoveNext();
	//}
	//m_sqlset->Close();
	//return list.count();
}

QString msql::getdbpath()
{
	return dbpath;
}

QString msql::getdbname()
{
	return dbname;
}

QString  msql::getuser()
{
	return user;
}

QString msql::getpwd()
{
	return pwd;
}