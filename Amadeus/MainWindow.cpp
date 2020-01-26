#include "MainWindow.h"
#include "DialogStation.h"
#include "qdockwidget.h"

// ���QT��VS��������������
#ifdef WIN32
#pragma execution_character_set("utf-8")
#endif

MainWindow::MainWindow(QWidget *parent)
	: QMainWindow(parent)
{
	ui.setupUi(this);

	// MDI���������ʾ
	this->setCentralWidget(ui.mdiArea);
	this->setWindowState(Qt::WindowMaximized); 

	createDialogStationManege(this);
}

QDialog* MainWindow::createDialogStationManege(MainWindow *const parent)
{

	// ����ͣ������
	QDockWidget *DockWidgetStaion = new QDockWidget("���վ����", parent);

	// �����Ի���
	DialogStation *DialogStationManege = new DialogStation(parent);

	// ��ӻ���ͣ������
	DockWidgetStaion->setWidget(DialogStationManege);

	// ���ͣ������
	addDockWidget(Qt::LeftDockWidgetArea, DockWidgetStaion);

	//��ʾ�Ի���
	DialogStationManege->show();

	return DialogStationManege;
}