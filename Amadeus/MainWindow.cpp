#include "MainWindow.h"
#include "DialogStation.h"
#include "qdockwidget.h"
#include "DialogMenuDBSet.h"
#include <QSettings>
#include <QMessageBox>
#include <QCloseEvent>
#include "Platform/utf8.h"
#include "globalfunc.h"

MainWindow::MainWindow(QWidget *parent)
	: QMainWindow(parent)
{
	ui.setupUi(this);

	// 恢复布局设置
	restoreDockWidgetLayoutSet();
	createDialogStation();

	//这个操作不该在此处，应该在各子负责的模块内，老程序在此处进行了全部程序的初始化，故暂时保留，后续再在各子的模块内延迟初始化
	UpdateNetList();
}



void MainWindow::createDialogStation()
{
	// 创建对话框
	DialogStation *DialogStationManege = new DialogStation(this);

	// 添加话框到停靠窗口
	ui.dockWidgetStation->setWidget(DialogStationManege);

	//显示对话框
	DialogStationManege->show();
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
	DialogMenuDBSet *dlgDBSet = new DialogMenuDBSet(this);

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