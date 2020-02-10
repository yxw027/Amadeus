#include "MainWindow.h"
#include "DialogStation.h"
#include "qdockwidget.h"
#include "DialogMenuNetSet.h"
#include <QMessageBox>

// 解决QT在VS里中文乱码问题
#ifdef WIN32
#pragma execution_character_set("utf-8")
#endif

MainWindow::MainWindow(QWidget *parent)
	: QMainWindow(parent)
{
	ui.setupUi(this);

	// MDI区域最大化显示
	this->setCentralWidget(ui.mdiArea);
	this->setWindowState(Qt::WindowMaximized); 

	createDialogStationManege();
}

void MainWindow::createDialogStationManege()
{
	// 创建停靠窗口
	QDockWidget *DockWidgetStaion = new QDockWidget("监测站管理", this);

	// 创建对话框
	DialogStation *DialogStationManege = new DialogStation(this);

	// 添加话框到停靠窗口
	DockWidgetStaion->setWidget(DialogStationManege);

	// 添加停靠窗口
	addDockWidget(Qt::LeftDockWidgetArea, DockWidgetStaion);

	//显示对话框
	DialogStationManege->show();
}

// 网络配置菜单槽函数
void MainWindow::on_actionNetSet_triggered()
{
	QMessageBox::information(this, "提示", "请确认测站数据未连接或子网未解算！");

	// 创建对话框
	DialogMenuNetSet *dlgNetset = new DialogMenuNetSet(this);

	// 设置对话框大小为固定
	Qt::WindowFlags flags = dlgNetset->windowFlags();
	dlgNetset->setWindowFlags(flags | Qt::MSWindowsFixedSizeDialogHint);

	// 以模态方式显示对话框
	int ret = dlgNetset->exec();
	//if (ret = QDialog::Accepted)
	//{
	//	// OK按钮被按下
	//	netinfo netifo;
	//	netifo.NETNAME = dlgNetCreate->getWorkName();
	//	netifo.WORKPATH = dlgNetCreate->getWorkDirection();
	//	netifo.OWNER = dlgNetCreate->getWorker();
	//	netifo.PINCHARGE = dlgNetCreate->getOrganization();
	//	netifo.PHONE = dlgNetCreate->getPhoneNum();
	//	netifo.EMAIL = dlgNetCreate->getPostalCode();
	//	netifo.SLNSYS = dlgNetCreate->getProSystem();
	//	netifo.SLNPHS = dlgNetCreate->getProFreq();
	//	// 原程序在此处就没有处理静态解时长
	//	//		netifo.SLNSES = dlgNetCreate->getProDuration();
	//	netifo.BASENUM = 0;
	//	netifo.BRDCTYPE = 0;
	//	netifo.COLAT = 0.0;
	//	netifo.COLON = 0.0;
	//	netifo.ROVERNUM = 0;
	//}

	//// 检查子网是否已经存在，不存在则添加
	////if (IDOK == NetAdd.DoModal() && !m_sql.netIsExist(NetAdd.mv_NetName))
	//{
	//	////////netinfo netifo;
	//	////////netifo.NETNAME = NetAdd.mv_NetName;
	//	////////netifo.SLNSYS = NetAdd.mv_slnsys;
	//	////////netifo.SLNPHS = NetAdd.mv_slnphs;
	//	////////netifo.EMAIL = NetAdd.mv_email;
	//	////////netifo.OWNER = NetAdd.mv_Admin;
	//	////////netifo.PHONE = NetAdd.mv_phone;
	//	////////netifo.PINCHARGE = NetAdd.mv_Company;
	//	////////netifo.WORKPATH = NetAdd.mv_WorkPath;
	//	////////netifo.BASENUM = 0;
	//	////////netifo.BRDCTYPE = 0;
	//	////////netifo.COLAT = 0.0;
	//	////////netifo.COLON = 0.0;
	//	////////netifo.ROVERNUM = 0;
	//	////////netifo.COMMENT = "";
	//	//m_sql.insert_net2db(&netifo);
	//	//UpdateTreeList();
	//}
	////else if (!NetAdd.mv_NetName.IsEmpty() && m_sql.netIsExist(NetAdd.mv_NetName))
	////{
	////	QMessageBox::information(this, "提示", "子网名称已存在，请勿重复添加！");
	////	return;
	////}

	//// 删除对话框
	delete dlgNetset;
}