#include "DialogStation.h"
#include "DialogStationNetAdd.h"
#include "DialogStationSiteAdd.h"
#include "msql.h"
#include "./Platform/utf8.h"
#include <QMenu>
#include <QMessageBox>
#include "globalfunc.h"

const QString ROOT_LB = "���վ";

DialogStation::DialogStation(QWidget *parent)
	: QDialog(parent)
{
	ui.setupUi(this);

	// ����Ҽ��˵����������⣬�����ݲ����
	ui.treeWidget->setContextMenuPolicy(Qt::CustomContextMenu);

	iniTree();

	// ��ȡ���ݿ����ݸ������ڵ�
	updateTreeList();
}

DialogStation::~DialogStation()
{

}

void DialogStation::iniTree()
{
	ui.treeWidget->clear();
	addRootNode();
}

// ��Ӹ��ڵ�
QTreeWidgetItem * DialogStation::addRootNode()
{ 
	// Item��Data�洢��string
	QString dataStr = ""; 

	// ����ICON��ͼ��
	QIcon icon(":/icons/Resources/obsRoot.bmp"); 

	// �½��ڵ�
	QTreeWidgetItem *item = new QTreeWidgetItem(DialogStation::TREE_NOTE_CORSTITLE);
	item->setIcon(DialogStation::nodeName, icon);
	item->setText(DialogStation::nodeName, ROOT_LB);
	item->setText(DialogStation::nodeState, "");
	item->setFlags(Qt::ItemIsSelectable | Qt::ItemIsEnabled);
	item->setData(DialogStation::nodeName, Qt::UserRole, QVariant(dataStr));

	// ���Ϊ����ڵ�
	ui.treeWidget->addTopLevelItem(item);
	return item;
}

// ���һ���������ڵ�
QTreeWidgetItem * DialogStation::addNetNode(QTreeWidgetItem *parItem, QString netname)
{
	// ����ICON��ͼ��
	QIcon icon(":/icons/Resources/obsNet.bmp");

	// �½��ڵ�
	QTreeWidgetItem *item = new QTreeWidgetItem(DialogStation::TREE_NOTE_CORSNAME);
	item->setIcon(nodeName, icon);
	item->setText(nodeName, netname);
	item->setText(nodeState, "");
	item->setFlags(Qt::ItemIsSelectable | Qt::ItemIsUserCheckable | Qt::ItemIsEnabled );
	item->setData(nodeName, Qt::UserRole, QVariant(""));

	// �ڸ��ڵ�������ӽڵ�
	parItem->addChild(item); 

	return item;
}

// ���һ�����վ�ڵ�
QTreeWidgetItem * DialogStation::addStationNode(QTreeWidgetItem *parItem, QString stationname, QString stationtype)
{
	// ����ICON��ͼ��
	QIcon icon;
	switch (stationtype.toInt())
	{
	case 1: icon = QIcon(":/icons/Resources/obsBase.bmp"); break;
	case 2: icon = QIcon(":/icons/Resources/obsRover.bmp");  break;
	case 3: icon = QIcon(":/icons/Resources/obsRover.bmp");  break;
	default:
		break;
	}

	// �½��ڵ�
	QTreeWidgetItem *item = new QTreeWidgetItem(DialogStation::TREE_NOTE_MPNAME);
	item->setIcon(nodeName, icon);
	item->setText(nodeName, stationname);
	item->setText(nodeState, "");
	item->setFlags(Qt::ItemIsSelectable | Qt::ItemIsUserCheckable | Qt::ItemIsEnabled);
	item->setData(nodeName, Qt::UserRole, QVariant(""));

	// �ڸ��ڵ�������ӽڵ�
	parItem->addChild(item);

	return item;
}

void DialogStation::updateTreeList()
{
	deleteAllChild();

	for (int rootIndex = 0; rootIndex < ui.treeWidget->topLevelItemCount(); rootIndex++)
	{
		QTreeWidgetItem *rootNode = ui.treeWidget->topLevelItem(rootIndex);
		QString rootName = rootNode->text(nodeName);
		// ����µĽڵ�
//		try
		{
			QStringList netList;
			QString netName;
			// ʵʱ��������Դ�б�ɸѡ
			if (rootName == ROOT_LB)
			{
				if (m_sql.getstate() == StateOpen)
					m_sql.close();
				m_sql.connect();
				m_sql.getnetlist(netList);
				int netsize = netList.count();

				// Ϊ���н��������ڵ���Ӳ�վ�ڵ�
				for (int netIndex = 0; netIndex < netsize; netIndex++)
				{
					netName = netList.at(netIndex);
					QTreeWidgetItem *netNode = addNetNode(rootNode, netName);
					QStringList sitelist;
					m_sql.getsitelist(netName, sitelist);
					int MPsize = sitelist.count();
					for (int mp_i = 0; mp_i < MPsize; mp_i++)
					{
						QStringList SiteInfo;
						QString sitename = sitelist.at(mp_i);
						m_sql.getsiteinfo(sitename, SiteInfo);

						QString labelname = SiteInfo.at(2);
						QString stationtype = SiteInfo.at(3);
						addStationNode(netNode, labelname.toUpper(), stationtype);
					}
					ui.treeWidget->expandAll();
#ifdef VER_OPEN
					break;	//���һ����������
#endif				
				}
			}
			else if (rootName == "��ǿ��Ϣ��ʾ")
			{
			}
			else if (rootName == "�û���Ϣ")
			{
			}
//		}
//		catch (CDBException* e)
//		{
//			MessageBox(e->m_strError);
		}
	}
}

void DialogStation::deleteAllChild()
{
	// �������и��ڵ�
	for (int rootIndex = 0; rootIndex < ui.treeWidget->topLevelItemCount(); rootIndex++)
	{
		QTreeWidgetItem *rootItem = ui.treeWidget->topLevelItem(rootIndex);
		// ���������ڵ�
		int netNum = rootItem->childCount();
		for (int netIndex = 0; netIndex < netNum; netIndex++)
		{
			QTreeWidgetItem *netItem = rootItem->child(0);
			// �����豸�ڵ�
			int devNum = netItem->childCount();
			for (int devIndex = 0; devIndex < devNum; devIndex++)
			{
				// ɾ���豸�ڵ�,ɾ��һ�������ڵ���Զ���ǰ��
				QTreeWidgetItem *devItem = netItem->child(0);
				netItem->removeChild(devItem);
				delete devItem;
			}
			// ɾ�������ڵ�,ɾ��һ�������ڵ���Զ���ǰ��
			rootItem->removeChild(netItem);
			delete netItem;
		}
	}
}

void DialogStation::on_treeWidget_customContextMenuRequested()
{
	// �����Ҽ������˵�
	QMenu* menuList = new QMenu(this);

	// �����˵���Action
	int itemType = ui.treeWidget->currentItem()->type();
	switch (itemType)
	{
	case TREE_NOTE_CORSTITLE:
		menuList->addAction(ui.actionNetAdd);
		break;
	case TREE_NOTE_CORSNAME:
		menuList->addAction(ui.actionNetSet);
		//menuList->addAction(ui.actionSiteAdd);
		//menuList->addAction(ui.action_Disconnect);
		//menuList->addAction(ui.action_Disconnect);
		// menuList->addSeparator();
		break;
	case TREE_NOTE_MPNAME:
		break;
	default:
		break;
	}

	// ��ʾ�˵��� 
	menuList->exec(QCursor::pos());

	// ɾ���˵���
	delete menuList; 
}

void DialogStation::on_actionNetAdd_triggered()
{
	//HTREEITEM hChild = m_wndFileView.GetChildItem(m_wndFileView.GetSelectedItem());
	//while (hChild) {
	//	CString NetName = m_wndFileView.GetItemText(hChild);
	//	if (theApp.m_NetList.find(NetName) != theApp.m_NetList.end() && theApp.m_NetList.at(NetName)->m_runflag) {
	//		AfxMessageBox("����ֹͣ�������㣡");
	//		return;
	//	}
	//	hChild = m_wndFileView.GetNextSiblingItem(hChild);
	//}

	// �����Ի���
	DialogStationNetAdd *dlgNetAdd = new DialogStationNetAdd(this);

	// ���öԻ����СΪ�̶�
	Qt::WindowFlags flags = dlgNetAdd->windowFlags();
	dlgNetAdd->setWindowFlags(flags | Qt::MSWindowsFixedSizeDialogHint);

	// ��ģ̬��ʽ��ʾ�Ի���
	int ret = dlgNetAdd->exec();
	if (ret = QDialog::Accepted)
	{
		if ((dlgNetAdd->getWorkName()).isEmpty())
			return;

		if (m_sql.netIsExist(dlgNetAdd->getWorkName()))
		{
			QMessageBox::information(NULL, "��ʾ��", "�����Ѵ��ڣ������ظ����");
			return;
		}

		netinfo netifo;
		netifo.NETNAME = dlgNetAdd->getWorkName();
		netifo.WORKPATH = dlgNetAdd->getWorkDirection();
		netifo.OWNER = dlgNetAdd->getWorker();
		netifo.PINCHARGE = dlgNetAdd->getOrganization();
		netifo.PHONE = dlgNetAdd->getPhoneNum();
		netifo.EMAIL = dlgNetAdd->getPostalCode();
		netifo.SLNSYS = dlgNetAdd->getProSystem();
		netifo.SLNPHS = dlgNetAdd->getProFreq();
		// ԭ�����ڴ˴���û�д���̬��ʱ��
		//		netifo.SLNSES = dlgNetCreate->getProDuration();
		netifo.BASENUM = 0;
		netifo.BRDCTYPE = 0;
		netifo.COLAT = 0.0;
		netifo.COLON = 0.0;
		netifo.ROVERNUM = 0;
		m_sql.insert_net2db(&netifo);
		updateTreeList();
	}

	// ɾ���Ի���
	delete dlgNetAdd;
}

void DialogStation::on_actionNetSet_triggered()
{
	//HTREEITEM hChild = m_wndFileView.GetChildItem(m_wndFileView.GetSelectedItem());
	//while (hChild) {
	//	CString NetName = m_wndFileView.GetItemText(hChild);
	//	if (theApp.m_NetList.find(NetName) != theApp.m_NetList.end() && theApp.m_NetList.at(NetName)->m_runflag) {
	//		AfxMessageBox("����ֹͣ�������㣡");
	//		return;
	//	}
	//	hChild = m_wndFileView.GetNextSiblingItem(hChild);
	//}

	//// �����Ի���
	//DialogStationNetAdd *dlgNetAdd = new DialogStationNetAdd(this);

	//// ���öԻ����СΪ�̶�
	//Qt::WindowFlags flags = dlgNetAdd->windowFlags();
	//dlgNetAdd->setWindowFlags(flags | Qt::MSWindowsFixedSizeDialogHint);

	//// ��ģ̬��ʽ��ʾ�Ի���
	//int ret = dlgNetAdd->exec();
	//if (ret = QDialog::Accepted)
	//{
	//	if ((dlgNetAdd->getWorkName()).isEmpty())
	//		return;

	//	if (m_sql.netIsExist(dlgNetAdd->getWorkName()))
	//	{
	//		QMessageBox::information(NULL, "��ʾ��", "�����Ѵ��ڣ������ظ����");
	//		return;
	//	}

	//	netinfo netifo;
	//	netifo.NETNAME = dlgNetAdd->getWorkName();
	//	netifo.WORKPATH = dlgNetAdd->getWorkDirection();
	//	netifo.OWNER = dlgNetAdd->getWorker();
	//	netifo.PINCHARGE = dlgNetAdd->getOrganization();
	//	netifo.PHONE = dlgNetAdd->getPhoneNum();
	//	netifo.EMAIL = dlgNetAdd->getPostalCode();
	//	netifo.SLNSYS = dlgNetAdd->getProSystem();
	//	netifo.SLNPHS = dlgNetAdd->getProFreq();
	//	// ԭ�����ڴ˴���û�д���̬��ʱ��
	//	//		netifo.SLNSES = dlgNetCreate->getProDuration();
	//	netifo.BASENUM = 0;
	//	netifo.BRDCTYPE = 0;
	//	netifo.COLAT = 0.0;
	//	netifo.COLON = 0.0;
	//	netifo.ROVERNUM = 0;
	//	m_sql.insert_net2db(&netifo);
	//	updateTreeList();
	//}

	//// ɾ���Ի���
	//delete dlgNetAdd;
	UpdateNetList();
}

void DialogStation::on_actionSiteAdd_triggered()
{
	//CString NetName = m_wndFileView.GetItemText(m_wndFileView.GetSelectedItem());
	//if (theApp.m_NetList.find(NetName) != theApp.m_NetList.end() && theApp.m_NetList.at(NetName)->m_runflag) {
	//	AfxMessageBox("����ֹͣ�������㣡");
	//	return;
	//}
	//if (theApp.m_NetList.find(NetName) != theApp.m_NetList.end()) {
	//	for (auto psock = theApp.m_NetList.at(NetName)->m_SiteList.begin(); psock != theApp.m_NetList.at(NetName)->m_SiteList.end(); psock++)
	//	{
	//		if (psock->second->psock.state) {
	//			AfxMessageBox("����ֹͣ���ݽ��գ�");
	//			return;
	//		}
	//	}
	//}
	DialogStationSiteAdd *dlgSiteAdd = new DialogStationSiteAdd(this);

	// ���öԻ����С�̶�
	Qt::WindowFlags flags = dlgSiteAdd->windowFlags();
	dlgSiteAdd->setWindowFlags(flags | Qt::MSWindowsFixedSizeDialogHint);

	int ret = dlgSiteAdd->exec();

	delete dlgSiteAdd;
	updateTreeList();
}