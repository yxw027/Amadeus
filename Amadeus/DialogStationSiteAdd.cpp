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

	// 列出可用串口
	foreach(const QSerialPortInfo &portInfo, QSerialPortInfo::availablePorts())
	{
		QSerialPort serial;
		serial.setPort(portInfo);
		if (serial.open(QIODevice::ReadWrite))
		{
			ui.comboBoxNetAddressOrSerialPort->addItem(serial.portName());
			serial.close();
		}
	}
	ui.comboBoxNetAddressOrSerialPort->setCurrentIndex(0);

	//列出可用波特率
	foreach(const int &baudeInfo, QSerialPortInfo::standardBaudRates())
	{
		ui.comboBoxPortNetPortOrSerialBaudRate->addItem(QString::number(baudeInfo));
	}
	ui.comboBoxPortNetPortOrSerialBaudRate->setCurrentIndex(0);
}

void DialogStationSiteAdd::on_pushButtonConnectTest_clicked()
{
	//QString temp;
	//mc_Connect.GetWindowTextA(temp);
	//if (temp == "连接") // 测试策略 需调整
	//{
	//	UpdateData(TRUE);
	//	size_t iPort;
	//	QString IP, Port, User, Pwd, MPoint, cstr, Bps;
	//	if (mv_strtype == 4)
	//	{
	//		mc_Address.GetWindowTextA(IP);
	//		mc_Port.GetWindowTextA(Port);
	//		mc_Username.GetWindowTextA(User);
	//		mc_Password.GetWindowTextA(Pwd);
	//		iPort = atoi(Port.GetBuffer(0));
	//	}
	//	else
	//	{
	//		mc_port_s.GetWindowTextA(Port);
	//		mc_Bps_s.GetWindowTextA(Bps);
	//	}
	//	Port.ReleaseBuffer();
	//	if (mc_SiteName.IsWindowVisible() && mv_SiteName.IsEmpty())
	//	{
	//		MessageBoxEx(this->m_hWnd, "标识输入不能为空！", "提示", MB_ICONWARNING | MB_OK, MAKELANGID(LANG_CHINESE_SIMPLIFIED, SUBLANG_CHINESE_TRADITIONAL));
	//		mc_SiteName.SetFocus();
	//		return;
	//	}
	//	MPoint = (mc_SiteName.IsWindowVisible() ? mv_SiteName : "TEMP");
	//	mc_Connect.SetWindowTextA(_T("正在连接"));
	//	char buff[MAXSRCTBL] = { 0 }, msg[MAXSTRMSG] = { 0 }, path[256] = { 0 };
	//	char *p = buff;
	//	int stat, ns;
	//	int tick = 0;// max link size

	//	stream_t svr;
	//	strinit(&svr);
	//	strinitcom();

	//	if (mv_downlistflag) {
	//		sprintf(path, "%s:%s@%s:%s/%s", User, Pwd, IP, Port, MPoint);
	//	}
	//	else if (mv_strtype == 4)
	//	{
	//		sprintf(path, ":@%s:%s/", IP, Port);
	//	}
	//	else if (mv_strtype == 1)
	//	{
	//		sprintf(path, "%s:%s:%s:%s:%s:%s", Port, Bps, "8", "n", "1", "off");
	//	}

	//	if (!stropen(&svr, mv_strtype, STR_MODE_R, path))
	//	{
	//		strclose(&svr);
	//		mc_OutputMsg.SetWindowTextA("连接失败");
	//		mc_Connect.SetWindowTextA("连接");
	//		return;
	//	}
	//	mc_OutputMsg.SetWindowTextA("connecting ...");
	//	int len = strlen(NTRIP_RSP_TBLEND);
	//	while (p<buff + MAXSRCTBL - 1) {
	//		ns = strread(&svr, (unsigned char*)p, buff + MAXSRCTBL - p - 1); *(p + ns) = '\0';
	//		//if(mv_downlistflag && ns) // 通信连接测试
	//		if (0<ns) // 通信连接测试
	//			break;
	//		if (p - len - 3>buff&&strstr(p - len - 3, NTRIP_RSP_TBLEND)) // 测站列表下载完成
	//			break;
	//		p += ns;
	//		Sleep(NTRIP_CYCLE);
	//		stat = strstat(&svr, msg);
	//		if (stat<0) break;
	//		//int tick1 = tickget();
	//		if (tick++ > 60) {
	//			mc_OutputMsg.SetWindowTextA("连接超时");
	//			mc_Connect.SetWindowTextA("连接");
	//			strclose(&svr);
	//			return;
	//		}
	//	}
	//	strclose(&svr);
	//	mc_OutputMsg.SetWindowTextA("连接成功");
	//	mc_Connect.SetWindowTextA("连接");
	//}
	//else
	//{
	//	return;
	//}// end 点击连接
}