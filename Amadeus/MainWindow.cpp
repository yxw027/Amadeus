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

	// �ָ���������
	restoreDockWidgetLayoutSet();
	createDialogStation();

	//������������ڴ˴���Ӧ���ڸ��Ӹ����ģ���ڣ��ϳ����ڴ˴�������ȫ������ĳ�ʼ��������ʱ�������������ڸ��ӵ�ģ�����ӳٳ�ʼ��
	UpdateNetList();
}



void MainWindow::createDialogStation()
{
	// �����Ի���
	DialogStation *DialogStationManege = new DialogStation(this);

	// ��ӻ���ͣ������
	ui.dockWidgetStation->setWidget(DialogStationManege);

	//��ʾ�Ի���
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
	// ���沼������
	saveDockWidgetLayoutSet();

	// ִ�йرղ���
	event->accept();
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