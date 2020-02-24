#include "DialogMenuDBSet.h"
#include "msql.h"
#include "Platform/utf8.h"

msql m_sql;

DialogMenuDBSet::DialogMenuDBSet(QWidget *parent)
	: QDialog(parent)
	, DBPath("")
	, DBUser("")
	, DBPassword("")
	, DBname("")
{
	ui.setupUi(this);
	setWindowTitle("数据库设置");
	initDialog();
}

DialogMenuDBSet::~DialogMenuDBSet()
{
}

bool DialogMenuDBSet::initDialog()
{
	cntflag = false;
	ui.comboBoxDBType->addItem("SQL Server2008");
	ui.comboBoxDBType->setCurrentIndex(0);

	if (m_sql.cfginit())
	{
		ui.lineEditDBPath->setText(m_sql.getdbpath());
		ui.comboBoxDBName->addItem(m_sql.getdbname());
		ui.comboBoxDBName->setCurrentIndex(0);
		ui.comboBoxDBName->setEnabled(true);
		ui.lineEditDBUser->setText(m_sql.getuser());
		ui.lineEditDBPassword->setText(m_sql.getpwd());
		cntflag = true;
	}
	else
	{
		ui.lineEditDBPath->setText("127.0.0.1");
		ui.lineEditDBUser->setText("sa");
		ui.lineEditDBPassword->setText("123456");
		ui.comboBoxDBName->setEnabled(false);
	}
	return true;  
}

void DialogMenuDBSet::on_pushButtonTestConnect_clicked()
{
	ui.labelTips->setText("正在连接……");
	
	// 读取设置参数
	DBPath = ui.lineEditDBPath->text();
	DBname = ui.comboBoxDBName->currentText();
	DBUser = ui.lineEditDBUser->text();
	DBPassword = ui.lineEditDBPassword->text();

	msql sqltest;
	if (sqltest.connect(DBPath, DBname, DBUser, DBPassword))
	{
		QStringList tblist;
		int count = sqltest.gettblist(tblist);
		ui.comboBoxDBName->setEnabled(cntflag = (0 == count ? false : true));
		ui.comboBoxDBName->clear();
		for (int i = 0; i < count; i++)
			ui.comboBoxDBName->addItem(tblist.at(i));
		if (count!=0) {
			ui.comboBoxDBName->setCurrentIndex(0);
			DBname = tblist.at(0);
		}
		ui.labelTips->setText("连接成功!");
	}
	else
	{
		ui.labelTips->setText("连接失败!"); 
	}
}

void DialogMenuDBSet::on_pushButtonOK_clicked()
{
	// 读取设置参数,旧版程序在该布仅对数据库名进行读取设置，其他参数保持之前设置不变
	DBname = ui.comboBoxDBName->currentText();

	m_sql.setdbpath(cntflag == true ? DBPath : "127.0.0.1");
	m_sql.setdbname(cntflag == true ? (DBname.isEmpty() ? "" : DBname) : "");
	m_sql.setuser(cntflag == true ? DBUser : "");
	m_sql.setpwd(cntflag == true ? DBPassword : "");
	m_sql.savecfg();
}