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

	// MDI���������ʾ
	this->setCentralWidget(ui.mdiArea);
	this->setWindowState(Qt::WindowMaximized); 

	createDialogStationManege();
	UpdateNetList();
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
void MainWindow::on_actionDBSet_triggered()
{
	QMessageBox::information(this, "��ʾ", "��ȷ�ϲ�վ����δ���ӻ�����δ���㣡");

	// �����Ի���
	DialogMenuDBSet *dlgDBSet = new DialogMenuDBSet(this);

	// ���öԻ����СΪ�̶�
	Qt::WindowFlags flags = dlgDBSet->windowFlags();
	dlgDBSet->setWindowFlags(flags | Qt::MSWindowsFixedSizeDialogHint);

	// ��ģ̬��ʽ��ʾ�Ի���
	int ret = dlgDBSet->exec();

	// ɾ���Ի���
	delete dlgDBSet;
	UpdateNetList();
}