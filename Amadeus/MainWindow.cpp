#include "MainWindow.h"
#include "qdockwidget.h"
#include <QSettings>
#include <QMessageBox>
#include <QCloseEvent>
#include "Platform/utf8.h"
#include "globalfunc.h"
#include "msql.h"

MainWindow::MainWindow(QWidget *parent)
	: QMainWindow(parent)
{
	ui.setupUi(this);
	connect(&m_sql, SIGNAL(signalShowMsg(const QString &, const QString &)), this, SLOT(on_showMsg_triggered(const QString &, const QString &)));
	// 恢复布局设置
	restoreDockWidgetLayoutSet();

	// 创建各子区域，各Msg窗口最好最先创建，否则后面各界面都会通过emit()使得Msg窗口中的plainTextEdit组件被调用，若没有先创建好则会程序报错
	createPlainTextEditMsgPro();
	createPlainTextEditMsgRcv();
	createPlainTextEditMsgFind();
	createDialogStation();


	//这个操作不该在此处，应该在各子负责的模块内，老程序在此处进行了全部程序的初始化，故暂时保留，后续再在各子的模块内延迟初始化
	UpdateNetList();
}




void MainWindow::createDialogStation()
{
	DialogStationManege = new DialogStation(this);
	ui.dockWidgetStation->setWidget(DialogStationManege);
	DialogStationManege->show();
}

void MainWindow::createPlainTextEditMsgPro()
{
	plainTextEditMsgPro = new QPlainTextEdit(this);
	plainTextEditMsgPro->setReadOnly(true);
	ui.dockWidgetMsgPro->setWidget(plainTextEditMsgPro);
	plainTextEditMsgPro->show();
}

void MainWindow::createPlainTextEditMsgRcv()
{
	plainTextEditMsgRcv = new QPlainTextEdit(this);
	plainTextEditMsgRcv->setReadOnly(true);
	ui.dockWidgetMsgRcv->setWidget(plainTextEditMsgRcv);
	plainTextEditMsgRcv->show();
}

void MainWindow::createPlainTextEditMsgFind()
{
	plainTextEditMsgFind = new QPlainTextEdit(this);
	plainTextEditMsgFind->setReadOnly(true);
	ui.dockWidgetMsgFind->setWidget(plainTextEditMsgFind);
	plainTextEditMsgFind->show();
}



void MainWindow::saveDockWidgetLayoutSet()
{
	QSettings settings("HwaCreate", "Amadeus");
	settings.beginGroup("mainWindow");
	settings.setValue("geometry", saveGeometry());
	settings.setValue("state", saveState());
	settings.endGroup();
}

void MainWindow::restoreDockWidgetLayoutSet()
{
	QSettings settings("HwaCreate", "Amadeus");
	settings.beginGroup("mainWindow");
	restoreGeometry(settings.value("geometry").toByteArray());
	restoreState(settings.value("state").toByteArray());
	settings.endGroup();
}

// 重写closeEvent事件函数，保存布局设置
void MainWindow::closeEvent(QCloseEvent* event)
{    
	// 保存布局设置
	saveDockWidgetLayoutSet();

	// 执行关闭操作
	event->accept();
}




// 网络配置菜单槽函数
void MainWindow::on_actionDBSet_triggered()
{
	QMessageBox::information(this, "提示", "请确认测站数据未连接或子网未解算！");

	// 创建对话框
	dlgDBSet = new DialogMenuDBSet(this);

	// 设置对话框大小为固定
	Qt::WindowFlags flags = dlgDBSet->windowFlags();
	dlgDBSet->setWindowFlags(flags | Qt::MSWindowsFixedSizeDialogHint);

	// 以模态方式显示对话框
	int ret = dlgDBSet->exec();

	// 删除对话框
	delete dlgDBSet;
	UpdateNetList();
}

void MainWindow::on_actionCtrlDialogStaion_triggered()
{
	ui.dockWidgetStation->show();
}

void MainWindow::on_actionCtrlMsgPro_triggered()
{
	ui.dockWidgetMsgPro->show();
}

void MainWindow::on_actionCtrlMsgRcv_triggered()
{
	ui.dockWidgetMsgRcv->show();
}

void MainWindow::on_actionCtrlMsgFind_triggered()
{
	ui.dockWidgetMsgFind->show();
}

void MainWindow::on_showMsg_triggered(const QString &type, const QString &msg)
{
	if(type == "msgPro")
		plainTextEditMsgPro->appendPlainText(msg);
	else if(type == "msgRcv")
		plainTextEditMsgRcv->appendPlainText(msg);
	else if(type == "msgFind")
		plainTextEditMsgFind->appendPlainText(msg);
	else;
}