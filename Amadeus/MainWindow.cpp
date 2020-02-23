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
	// �ָ���������
	restoreDockWidgetLayoutSet();

	// �����������򣬸�Msg����������ȴ����������������涼��ͨ��emit()ʹ��Msg�����е�plainTextEdit��������ã���û���ȴ����������򱨴�
	createPlainTextEditMsgPro();
	createPlainTextEditMsgRcv();
	createPlainTextEditMsgFind();
	createDialogStation();


	//������������ڴ˴���Ӧ���ڸ��Ӹ����ģ���ڣ��ϳ����ڴ˴�������ȫ������ĳ�ʼ��������ʱ�������������ڸ��ӵ�ģ�����ӳٳ�ʼ��
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

// ��дcloseEvent�¼����������沼������
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
	dlgDBSet = new DialogMenuDBSet(this);

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