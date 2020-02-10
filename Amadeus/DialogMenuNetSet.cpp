#include "DialogMenuNetSet.h"
#include "msql.h"

msql m_sql;

DialogMenuNetSet::DialogMenuNetSet(QWidget *parent)
	: QDialog(parent)
	, mv_DBPath("")
	, mv_DBUser("")
	, mv_DBPassword("")
	, mv_DBname("")
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