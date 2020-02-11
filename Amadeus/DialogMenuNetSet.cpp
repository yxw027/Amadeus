#include "DialogMenuNetSet.h"
#include "msql.h"

// 解决QT在VS里中文乱码问题
#ifdef WIN32
#pragma execution_character_set("utf-8")
#endif

msql m_sql;

DialogMenuNetSet::DialogMenuNetSet(QWidget *parent)
	: QDialog(parent)
	, DBPath("")
	, DBUser("")
	, DBPassword("")
	, DBname("")
{
	ui.setupUi(this);
	initDialog();
}

DialogMenuNetSet::~DialogMenuNetSet()
{
}

bool DialogMenuNetSet::initDialog()
{
	cntflag = false;
	ui.comboBoxDBType->addItem("SQL Server2008");
	ui.comboBoxDBType->setCurrentIndex(0);

	if (m_sql.cfginit())
	{
		ui.lineEditDBPath->setText(m_sql.getdbpath());
		ui.comboBoxDBName->addItem(m_sql.getdbname());
		ui.comboBoxDBName->setCurrentIndex(0);
		ui.comboBoxDBName->setEditable(true);
		ui.lineEditDBUser->setText(m_sql.getuser());
		ui.lineEditDBPassword->setText(m_sql.getpwd());
		cntflag = true;
	}
	else
	{
		ui.lineEditDBPath->setText("127.0.0.1");
		ui.lineEditDBUser->setText("sa");
		ui.lineEditDBPassword->setText("123456");
		ui.comboBoxDBName->setEditable(false);
	}
	return true;  
}

void DialogMenuNetSet::on_pushButtonTestConnect_clicked()
{
	ui.labelTips->setText("正在连接……");
	
	// 读取最终设置参数
	DBPath = ui.lineEditDBPath->text();
	DBname = ui.comboBoxDBName->currentText();
	DBUser = ui.lineEditDBUser->text();
	DBPassword = ui.lineEditDBPassword->text();

	msql sqltest;
	if (sqltest.connect(DBPath, DBname, DBUser, DBPassword))
	{
		QStringList tblist;
		int count = sqltest.gettblist(tblist);
		//mc_BaseType.EnableWindow(cntflag = (0 == count ? FALSE : TRUE));
		//mc_BaseType.ResetContent();  // clear control items
		//for (int i = 0; i < count; i++)
		//	mc_BaseType.InsertString(0, tblist.GetAt(i));
		//if (mc_BaseType.GetCount()) {
		//	mc_BaseType.SetCurSel(0);
		//	mv_DBname = tblist.GetAt(0);
		//}
		ui.labelTips->setText("连接成功!");
	}
	else
	{
		ui.labelTips->setText("连接失败!"); 
	}
}