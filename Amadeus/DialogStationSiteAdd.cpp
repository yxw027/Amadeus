#include "DialogStationSiteAdd.h"
#include "msql.h"
#include <QMessageBox>
#include "qsqlerror.h"
#include "Platform\utf8.h"
#include <QtSerialPort/QtSerialPort>
#include <QtSerialPort/QSerialPortInfo>

const QStringList listHeader = { "ID","MOUNTPOINT","LABEL","REMARKS","NETID","TYPE","COM_TYPE","IP","PORT","USERNAME","PASSWORD","LATITUDE","LONGITUDE","HEIGHT" };

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
	initComboBoxConnectType();
	initTableViewSite();
}

void DialogStationSiteAdd::initComboBoxConnectType()
{
	ui.comboBoxConnectType->addItem("TCP Server");
	ui.comboBoxConnectType->addItem("TCP Client");
	ui.comboBoxConnectType->addItem("Serial");
	ui.comboBoxConnectType->setCurrentIndex(0);
}

void DialogStationSiteAdd::initTableViewSite()
{
	// Model/View方式显示测站信息，数据库变化自动更新到界面
	QString cstr;
	cstr.sprintf("SELECT * FROM %s WHERE DetectNetID=(SELECT DetectNetID FROM %s WHERE DetectNetName='%s') ORDER BY %s",
		SITE_CFG_TB.toUtf8().data(), NET_CFG_TB.toUtf8().data(), netName.toUtf8().data(), SITENAME_TB.toUtf8().data());
	qryModel = new QSqlQueryModel(this);

	// 全局msql对象中已经打开DB连接，默认对应到该连接的数据库
	qryModel->setQuery(cstr);
	if (qryModel->lastError().isValid())
	{
		QMessageBox::critical(this, "Tips", qryModel->lastError().text(), QMessageBox::Ok, QMessageBox::NoButton);
		return;
	}
	ui.tableViewSite->setModel(qryModel);

	// 设置字段显示名
	for (int i = 0; i < listHeader.length(); i++)
	{
		qryModel->setHeaderData(i, Qt::Horizontal, listHeader.at(i));
	}

	// 设置隐藏字段
	ui.tableViewSite->setColumnHidden(listHeader.indexOf("REMARKS"),true);
	ui.tableViewSite->setColumnHidden(listHeader.indexOf("NETID"), true);
}

void DialogStationSiteAdd::on_comboBoxConnectType_currentIndexChanged(int index)
{
	 //TODO: Add your control notification handler code here
	switch (index)
	{	
		case 0: // tcp server
			initNetPortUI();
		break;

		case 1: // tcp client
			initNetPortUI();
		break;
		
		case 2: // serial
			initSerialPortUI();
		break;

		default:
		break;
	}
	//switch (mc_ConnectType.GetCurSel())
	//{
	//case 0: mv_strtype = CONNECT_STR_TCPCLI;	break;
	//	//case 1: mv_strtype = CONNECT_STR_TCPSVR;	break;
	//	//case 2: mv_strtype = CONNECT_STR_NTRIPCLI;	break;
	//case 1: mv_strtype = CONNECT_STR_SERIAL; break;
	//default:
	//	break;
	//}
}

void DialogStationSiteAdd::initNetPortUI()
{
	ui.comboBoxNetAddressOrSerialPort->clear();
	ui.comboBoxNetAddressOrSerialPort->setEditable(true);
	ui.comboBoxPortNetPortOrSerialBaudRate->clear();
	ui.comboBoxPortNetPortOrSerialBaudRate->setEditable(true);
	ui.labelNetAddressOrSerialPort->setText("IP地址");
	ui.labelNetPortOrSerialBaudRate->setText("IP端口");
}

void DialogStationSiteAdd::initSerialPortUI()
{
	ui.comboBoxNetAddressOrSerialPort->clear();
	ui.comboBoxNetAddressOrSerialPort->setEditable(false);
	ui.comboBoxPortNetPortOrSerialBaudRate->clear();
	ui.comboBoxPortNetPortOrSerialBaudRate->setEditable(false);
	ui.labelNetAddressOrSerialPort->setText("串口号");
	ui.labelNetPortOrSerialBaudRate->setText("波特率");

	// 查找可用串口
	foreach(const QSerialPortInfo &info, QSerialPortInfo::availablePorts())
	{
		QSerialPort serial;
		serial.setPort(info);
		if (serial.open(QIODevice::ReadWrite))
		{
			ui.comboBoxNetAddressOrSerialPort->addItem(serial.portName());
			serial.close();
		}
	}

	//设置波特率下拉菜单的第0项默认值
	ui.comboBoxNetAddressOrSerialPort->setCurrentIndex(0);
}