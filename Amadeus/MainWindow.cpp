#include "MainWindow.h"
#include "DialogStation.h"
#include "qdockwidget.h"
#include "DialogMenuDBSet.h"
#include <QMessageBox>
#include "Platform/utf8.h"
#include "globalfunc.h"

MainWindow::MainWindow(QWidget *parent)
	: QMainWindow(parent)
{
	ui.setupUi(this);

	// MDI区域最大化显示
	this->setCentralWidget(ui.mdiArea);
	this->setWindowState(Qt::WindowMaximized); 

	createDialogStationManege();
	UpdateNetList();
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