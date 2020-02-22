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

	createDialogStationManege();
	UpdateNetList();
	restoreDockWidgetLayoutSets();
}



void MainWindow::createDialogStationManege()
{
	// �����Ի���
	DialogStation *DialogStationManege = new DialogStation(this);

	// ��ӻ���ͣ������
	ui.dockWidgetStation->setWidget(DialogStationManege);

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

void MainWindow::saveDockWidgetLayoutSets()
{
	QSettings settings("HwaCreate", "Amadeus");
	settings.beginGroup("mainWindow");
	settings.setValue("geometry", saveGeometry());
	settings.setValue("state", saveState());
	settings.endGroup();
}

void MainWindow::restoreDockWidgetLayoutSets()
{
	QSettings settings("HwaCreate", "Amadeus");
	settings.beginGroup("mainWindow");
	restoreGeometry(settings.value("geometry").toByteArray());
	restoreState(settings.value("state").toByteArray());
	settings.endGroup();
}

void MainWindow::closeEvent(QCloseEvent* event)
{    
	saveDockWidgetLayoutSets();
	event->accept();
}