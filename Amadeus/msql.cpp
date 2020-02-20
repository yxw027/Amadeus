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

	state = StateOpen;

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

	if (DB.open())
	{
		state = StateOpen;
		return true;
	}
	else
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

void msql::close()
{
	//if (m_sqlcnt && m_sqlcnt->State)
	if (state== StateOpen)
	{
		DB.close();
		state = StateClose;
	}
}

int msql::gettblist(QStringList& tblist)
{
	getlist("NAME", tblist, "SELECT NAME FROM MASTER..SYSDATABASES ORDER BY NAME");
	return tblist.count();
}

int msql::getnetlist(QStringList& netlist)
{
	QString cstr = QString("SELECT DISTINCT DetectNetName FROM %1").arg(NET_CFG_TB);
	return getlist(NETNAME_TB, netlist, cstr);
}

int msql::getsitelist(QString netname, QStringList& sitelist)
{
	Q_ASSERT(!netname.isEmpty());
	QString cstr = QString("SELECT %1 FROM %2 WHERE DetectNetID=(SELECT DetectNetID FROM %3 WHERE DetectNetName='%4') ORDER BY %5"
						).arg(SITENAME_TB, SITE_CFG_TB, NET_CFG_TB, netname, SITENAME_TB);
	return getlist(SITENAME_TB, sitelist, cstr);
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
	int recordCnt = countQueryRecord(query);
	if (recordCnt == 0)
		return 0;
		
	// 从第一条记录开始处理
	do {
		QString sitename = (query.value(field)).toString();
		list.append(sitename);
	} while (query.next());

	return recordCnt;
}

int msql::getsiteinfo(QString sitename, QStringList& infolist, int flag)
{
	Q_ASSERT(!sitename.isEmpty());
	QString cstr = QString("SELECT [DetectStationID],[DetectStationName],[DetectStatioLabel],[Remarks],[DetectNetID],[StationType],[CommType],[IP],[Com],[Usename],[PASSWORD],[Coord_X],[Coord_Y],[Coord_Z]\
				FROM [%1].[DBO].[%2] WHERE %3='%4'").arg(dbname, SITE_CFG_TB, (flag == 0 ? SITENAME_TB : "DetectStatioLabel"), sitename);
	//try
	{
		QSqlQuery query;
		query.exec(cstr);
		int recordCnt = countQueryRecord(query);
		if (recordCnt == 0) return 0;

		infolist.append(query.value("DetectStationID").toString());
		infolist.append(query.value("DetectStationName").toString());
		infolist.append(query.value("DetectStatioLabel").toString());
		infolist.append(query.value("StationType").toString());
		infolist.append(query.value("CommType").toString());
		infolist.append(query.value("IP").toString());
		infolist.append(query.value("Com").toString());
		infolist.append(query.value("Usename").toString());
		infolist.append(query.value("PASSWORD").toString());
		infolist.append(query.value("Coord_X").toString());
		infolist.append(query.value("Coord_Y").toString());
		infolist.append(query.value("Coord_Z").toString());

		return infolist.count();
	}//end try
	//catch (_com_error &e)
	//{
	//	sendsqlmsg(e);
	//}
}

bool msql::netIsExist(QString netname)
{
	return fieldcheck(NETNAME_TB, NET_CFG_TB, netname);
}

void msql::insert_net2db(const netinfo* pnet)
{
	Q_ASSERT(pnet != NULL);

	// QString.arg()不方便拼浮点数，故采用QString::sprintf();
	// 该函数输入为char *，要用QString::toUtf8().data()将QStinrg时转化为char *;
	QString cstr;
	cstr.sprintf("INSERT INTO [%s].[DBO].[%s] ([DetectNetName] ,[filepath] ,[SLNSYS] ,[SLNPHS] ,[owner] ,[PINCHARGE] ,[phone] ,[email] ,[Clatitude] ,[Clongitude])\
     VALUES ('%s','%s',%d,%d,'%s','%s','%s','%s',%18.6f,%18.6f)",
		dbname.toUtf8().data(), NET_CFG_TB.toUtf8().data(), pnet->NETNAME.toUtf8().data(), pnet->WORKPATH.toUtf8().data(), pnet->SLNSYS, pnet->SLNPHS, pnet->OWNER.toUtf8().data(), pnet->PINCHARGE.toUtf8().data(), pnet->PHONE.toUtf8().data(), pnet->EMAIL.toUtf8().data(), 0.0, 0.0);
	//try {
		QSqlQuery query;
		query.exec(cstr);
	//}
	//catch (_com_error &e)
	//{
	//	sendsqlmsg(e);
	//}
}

/* update socket struct infomation from sql */
void msql::update_netinfo2str(QString NetName, netinfo* pnet)
{
	Q_ASSERT(!NetName.isEmpty());
	Q_ASSERT(pnet != NULL);
	
	QString cstr = QString("SELECT * FROM CONFIG_DETECTNET WHERE DETECTNETNAME='%1'").arg(NetName);
	//try 
	{
		QSqlQuery query;
		query.exec(cstr);

		int recordCnt = countQueryRecord(query);
		if (recordCnt == 0)
			return;

		pnet->NETNAME = query.value("DETECTNETNAME").toString();
		pnet->WORKPATH = query.value("FILEPATH").toString();
		pnet->SLNSYS = query.value("SLNSYS").toInt();
		pnet->SLNPHS = query.value("SLNPHS").toInt();
		pnet->SLNSES = 0; // ???? add for debug
		pnet->OWNER = query.value("OWNER").toString();
		pnet->PINCHARGE = query.value("PINCHARGE").toString();
		pnet->PHONE = query.value("PHONE").toString();
		pnet->EMAIL = query.value("EMAIL").toString();
		return;
	}
	//catch (_com_error &e)
	//{
	//	sendsqlmsg(e);
	//}
}

void msql::update_netinfo2db(QString NetName, const netinfo* pnet)
{
	Q_ASSERT(pnet != NULL);
	Q_ASSERT(!NetName.isEmpty());
	QString cstr;
	cstr.sprintf("UPDATE [%s].[DBO].[%s] SET \
				 [DETECTNETNAME]	= '%s'\
				,[FILEPATH]	= '%s'\
				,[SLNSYS]	= %d \
				,[SLNPHS]	= %d \
				,[OWNER]	= '%s'\
				,[PINCHARGE]	= '%s'\
				,[PHONE]	= '%11s'\
				,[EMAIL]	= '%s'\
				,[CLATITUDE]	= %18.6f \
				,[CLONGITUDE]	= %18.6f \
				,[BASESTATION_AMOUNT]	= %d \
				,[DETECTSTATION_AMOUNT]	= %d \
				,[BRDCTYPE]	= %d \
				,[REMARK]	= '%s' \
				WHERE DETECTNETID=(SELECT DETECTNETID FROM CONFIG_DETECTNET WHERE DETECTNETNAME='%s')",
		dbname.toUtf8().data()
		, NET_CFG_TB.toUtf8().data()
		, pnet->NETNAME.toUtf8().data()
		, pnet->WORKPATH.toUtf8().data()
		, pnet->SLNSYS
		, pnet->SLNPHS
		, pnet->OWNER.toUtf8().data()
		, pnet->PINCHARGE.toUtf8().data()
		, pnet->PHONE.toUtf8().data()
		, pnet->EMAIL.toUtf8().data()
		, pnet->COLAT
		, pnet->COLON
		, pnet->BASENUM
		, pnet->ROVERNUM
		, pnet->BRDCTYPE
		, pnet->COMMENT.toUtf8().data()
		, NetName.toUtf8().data());
	//try
	{
		QSqlQuery query;
		query.exec(cstr);
	}//try
	//catch (_com_error &e)
	//{
	//	sendsqlmsg(e);
	//}
}

void msql::erasenet(QString netname)
{
	Q_ASSERT(!netname.isEmpty());
	QString cstr;
	//try 
	{
		cstr.sprintf("DELETE FROM %s WHERE DetectNetID=(SELECT DetectNetID FROM %s WHERE DetectNetName='%s')", SITE_CFG_TB.toUtf8().data(), NET_CFG_TB.toUtf8().data(), netname.toUtf8().data());
		QSqlQuery query;
		query.exec(cstr);
		cstr.sprintf("DELETE FROM %s WHERE DetectNetName='%s'", NET_CFG_TB.toUtf8().data(), netname.toUtf8().data());
		query.exec(cstr);
	}
	//catch (_com_error &e)
	//{
	//	sendsqlmsg(e);
	//}
}

bool msql::fieldcheck(QString indx_field, QString obj_field, QString val)
{
	Q_ASSERT(!val.isEmpty());
	QString cstr = QString("select %1 from %2 where %3='%4'").arg(indx_field, obj_field, indx_field, val);
	//try
	{
		QSqlQuery query;
		query.exec(cstr);
		int recordCnt = countQueryRecord(query);
		return (recordCnt > 0) ? true : false;
	}
	//catch (_com_error &e)
	//{
	//	sendsqlmsg(e);
	//}
}

int msql::countQueryRecord(QSqlQuery query)
{
	int count = 0;
	while (query.next())
	{
		count++;
	}
	query.first();
	return count;
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



int msql::getstate()
{
	return state;
}