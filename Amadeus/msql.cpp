#include "msql.h"
#include <QTextBlock>
#include <QTextStream>
#include <QSqlQuery>

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

	// SQL server与MySQL、SQLite等不同，未设置DSN的，使用手工拼接DSN字符串进行连接
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

void msql::savecfg()
{
	QString filename = "..\\x64\\Debug\\sql.cfg";
	QFile ifile(filename);
	if (!ifile.open(QIODevice::WriteOnly | QIODevice::Text))
	{
		//sendmsginfo("Warning: .\\bin\\sql.cfg can not open!");
		return;
	}

	QTextStream iStream(&ifile);
	iStream << ("IP:" + dbpath + "\n");
	iStream << ("DB:" + dbname + "\n");
	iStream << ("USER:" + user + "\n");
	iStream << ("PWD:" + pwd + "\n");
	
	ifile.close();
	return;
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

	// query.size()始终等于-1，与SQL SERVER的驱动有关，故此处不使用该函数判断是否查询到记录
	// 而原版本的m_sqlset->GetRecordCount()函数也统计不准确，只用来判断是否查询到记录，后续使用list.GetCount()准确统计
	// 统计查询到的记录数
	int recordCnt = 0;
	while (query.next())
	{
		recordCnt++;
	}
	if (recordCnt == 0)
		return 0;
		
	// 从第一条记录开始处理
	query.first();
	do {
		QString sitename = (query.value(field)).toString();
		list.append(sitename);
	} while (query.next());

	return recordCnt;
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

void msql::setdbpath(QString arg)
{
	dbpath = arg;
}

void msql::setdbname(QString arg)
{
	dbname = arg;
}

void msql::setuser(QString arg)
{
	user = arg;
}

void msql::setpwd(QString arg)
{
	pwd = arg;
}