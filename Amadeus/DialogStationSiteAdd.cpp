#include "DialogStationSiteAdd.h"
#include "msql.h"

const QString ALL_HEADER[] = { "ID","MOUNTPOINT","LABEL","TYPE","COM_TYPE","IP","PORT","USERNAME","PASSWORD","LATITUDE","LONGITUDE","HEIGHT" };

DialogStationSiteAdd::DialogStationSiteAdd(QWidget *parent)
	: QDialog(parent)
{
	ui.setupUi(this);
	initDialog();
}

DialogStationSiteAdd::DialogStationSiteAdd(const QString arg, QWidget *parent)
	: QDialog(parent)
{
	netName = arg;
	ui.setupUi(this);
	initDialog();
}

DialogStationSiteAdd::~DialogStationSiteAdd()
{
}

void DialogStationSiteAdd::initDialog()
{
	ui.comboBoxConnectType->addItem("TCP Server");
	ui.comboBoxConnectType->addItem("TCP Client");
	ui.comboBoxConnectType->addItem("Serial");
	ui.comboBoxConnectType->setCurrentIndex(0);

	QString cstr;
	cstr.sprintf("SELECT * FROM %s WHERE DetectNetID=(SELECT DetectNetID FROM %s WHERE DetectNetName='%s') ORDER BY %s",
		SITE_CFG_TB.toUtf8().data(), NET_CFG_TB.toUtf8().data(), netName.toUtf8().data(), SITENAME_TB.toUtf8().data());

	qryModel = new QSqlQueryModel(this);
	qryModel->setQuery(cstr);	
	ui.tableViewSite->setModel(qryModel);

//	mc_MountPointList.SetExtendedStyle(LVS_EX_CHECKBOXES | LVS_EX_GRIDLINES | LVS_EX_FULLROWSELECT | LVS_EX_TRACKSELECT);
//	mc_MountPointList.SetColumnHeader(cstr);
	// 初始化数据库列表到控件中
//	UpdateList();

	return;
}