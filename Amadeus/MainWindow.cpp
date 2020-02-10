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
	//if (ret = QDialog::Accepted)
	//{
	//	// OK��ť������
	//	netinfo netifo;
	//	netifo.NETNAME = dlgNetCreate->getWorkName();
	//	netifo.WORKPATH = dlgNetCreate->getWorkDirection();
	//	netifo.OWNER = dlgNetCreate->getWorker();
	//	netifo.PINCHARGE = dlgNetCreate->getOrganization();
	//	netifo.PHONE = dlgNetCreate->getPhoneNum();
	//	netifo.EMAIL = dlgNetCreate->getPostalCode();
	//	netifo.SLNSYS = dlgNetCreate->getProSystem();
	//	netifo.SLNPHS = dlgNetCreate->getProFreq();
	//	// ԭ�����ڴ˴���û�д���̬��ʱ��
	//	//		netifo.SLNSES = dlgNetCreate->getProDuration();
	//	netifo.BASENUM = 0;
	//	netifo.BRDCTYPE = 0;
	//	netifo.COLAT = 0.0;
	//	netifo.COLON = 0.0;
	//	netifo.ROVERNUM = 0;
	//}

	//// ��������Ƿ��Ѿ����ڣ������������
	////if (IDOK == NetAdd.DoModal() && !m_sql.netIsExist(NetAdd.mv_NetName))
	//{
	//	////////netinfo netifo;
	//	////////netifo.NETNAME = NetAdd.mv_NetName;
	//	////////netifo.SLNSYS = NetAdd.mv_slnsys;
	//	////////netifo.SLNPHS = NetAdd.mv_slnphs;
	//	////////netifo.EMAIL = NetAdd.mv_email;
	//	////////netifo.OWNER = NetAdd.mv_Admin;
	//	////////netifo.PHONE = NetAdd.mv_phone;
	//	////////netifo.PINCHARGE = NetAdd.mv_Company;
	//	////////netifo.WORKPATH = NetAdd.mv_WorkPath;
	//	////////netifo.BASENUM = 0;
	//	////////netifo.BRDCTYPE = 0;
	//	////////netifo.COLAT = 0.0;
	//	////////netifo.COLON = 0.0;
	//	////////netifo.ROVERNUM = 0;
	//	////////netifo.COMMENT = "";
	//	//m_sql.insert_net2db(&netifo);
	//	//UpdateTreeList();
	//}
	////else if (!NetAdd.mv_NetName.IsEmpty() && m_sql.netIsExist(NetAdd.mv_NetName))
	////{
	////	QMessageBox::information(this, "��ʾ", "���������Ѵ��ڣ������ظ���ӣ�");
	////	return;
	////}

	//// ɾ���Ի���
	delete dlgNetset;
}