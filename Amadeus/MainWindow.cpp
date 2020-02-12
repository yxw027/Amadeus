#include "MainWindow.h"
#include "DialogStation.h"
#include "qdockwidget.h"
#include "DialogMenuNetSet.h"
#include <QMessageBox>

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

	createDialogStationManege();
}

void MainWindow::createDialogStationManege()
{
	// ����ͣ������
	QDockWidget *DockWidgetStaion = new QDockWidget("���վ����", this);

	// �����Ի���
	DialogStation *DialogStationManege = new DialogStation(this);

	// ��ӻ���ͣ������
	DockWidgetStaion->setWidget(DialogStationManege);

	// ���ͣ������
	addDockWidget(Qt::LeftDockWidgetArea, DockWidgetStaion);

	//��ʾ�Ի���
	DialogStationManege->show();
}

// �������ò˵��ۺ���
void MainWindow::on_actionNetSet_triggered()
{
	QMessageBox::information(this, "��ʾ", "��ȷ�ϲ�վ����δ���ӻ�����δ���㣡");

	// �����Ի���
	DialogMenuNetSet *dlgNetset = new DialogMenuNetSet(this);

	// ���öԻ����СΪ�̶�
	Qt::WindowFlags flags = dlgNetset->windowFlags();
	dlgNetset->setWindowFlags(flags | Qt::MSWindowsFixedSizeDialogHint);

	// ��ģ̬��ʽ��ʾ�Ի���
	int ret = dlgNetset->exec();

	// ɾ���Ի���
	delete dlgNetset;
}