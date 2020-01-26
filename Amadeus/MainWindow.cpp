#include "MainWindow.h"
#include "DialogStation.h"
#include "qdockwidget.h"

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

	createDialogStationManege(this);
}

QDialog* MainWindow::createDialogStationManege(MainWindow *const parent)
{

	// 创建停靠窗口
	QDockWidget *DockWidgetStaion = new QDockWidget("监测站管理", parent);

	// 创建对话框
	DialogStation *DialogStationManege = new DialogStation(parent);

	// 添加话框到停靠窗口
	DockWidgetStaion->setWidget(DialogStationManege);

	// 添加停靠窗口
	addDockWidget(Qt::LeftDockWidgetArea, DockWidgetStaion);

	//显示对话框
	DialogStationManege->show();

	return DialogStationManege;
}