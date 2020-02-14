#include "DialogStation.h"
#include "DialogStationNetCreate.h"
#include "globalfunc.h"
#include "msql.h"
#include <QMenu>
#include <QMessageBox>

// ���QT��VS��������������
#ifdef WIN32
#pragma execution_character_set("utf-8")
#endif

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
QTreeWidgetItem * DialogStation::addNetItem(QTreeWidgetItem *parItem, QString netname)
{
	// ����ICON��ͼ��
	QIcon icon(":/icons/Resources/obsNet.bmp");

	// �½��ڵ�
	QTreeWidgetItem *item = new QTreeWidgetItem(DialogStation::TREE_NOTE_CORSNAME);
	item->setIcon(nodeName, icon);
	item->setText(nodeName, netname);
	item->setText(nodeState, "type=TREE_NOTE_CORSNAME");
	item->setFlags(Qt::ItemIsSelectable | Qt::ItemIsUserCheckable | Qt::ItemIsEnabled );
	item->setData(nodeName, Qt::UserRole, QVariant(""));

	// �ڸ��ڵ�������ӽڵ�
	parItem->addChild(item); 

	return item;
}

// ���һ�����վ�ڵ�
QTreeWidgetItem * DialogStation::addStationItem(QTreeWidgetItem *parItem, QString stationname, QString stationtype)
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
	item->setText(nodeState, "type=TREE_NOTE_MPNAME");
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
				for (int netIndex = 0; netIndex < netsize; netIndex++)
				{
					netName = netList.at(netIndex);
					QTreeWidgetItem *netNode = addNetItem(rootNode, netName);

					// ���ص��б�ɸѡ
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
						addStationItem(netNode, labelname.toUpper(), stationtype);
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
	// �����˵�
	QMenu* menuList = new QMenu(this);

	int currentItemType = ui.treeWidget->currentItem()->type();

	switch (currentItemType)
	{
	case TREE_NOTE_CORSTITLE:
		// ���Actions�����˵���
		menuList->addAction(ui.actionNetCreate);
		//// ��ӷָ���
		// menuList->addSeparator();
		break;
	case TREE_NOTE_CORSNAME:
		//menuList->addAction(ui.action_Connect);
		//menuList->addAction(ui.action_Disconnect);
		//menuList->addAction(ui.action_Disconnect);
		break;
	case TREE_NOTE_MPNAME:
		break;
	default:
		break;
	}
	menuList->exec(QCursor::pos());
	delete menuList; 
}

void DialogStation::on_actionNetCreate_triggered()
{
	// �����Ի���
	DialogStationNetCreate *dlgNetCreate = new DialogStationNetCreate(this);

	// ���öԻ����СΪ�̶�
	Qt::WindowFlags flags = dlgNetCreate->windowFlags();
	dlgNetCreate->setWindowFlags(flags | Qt::MSWindowsFixedSizeDialogHint);

	// ��ģ̬��ʽ��ʾ�Ի���
	int ret = dlgNetCreate->exec();
	if (ret = QDialog::Accepted)
	{
		// OK��ť������
		netinfo netifo;
		netifo.NETNAME = dlgNetCreate->getWorkName();
		netifo.WORKPATH = dlgNetCreate->getWorkDirection();
		netifo.OWNER = dlgNetCreate->getWorker();
		netifo.PINCHARGE = dlgNetCreate->getOrganization();
		netifo.PHONE = dlgNetCreate->getPhoneNum();
		netifo.EMAIL = dlgNetCreate->getPostalCode();
		netifo.SLNSYS = dlgNetCreate->getProSystem();
		netifo.SLNPHS = dlgNetCreate->getProFreq();
		// ԭ�����ڴ˴���û�д���̬��ʱ��
		//		netifo.SLNSES = dlgNetCreate->getProDuration();
		netifo.BASENUM = 0;
		netifo.BRDCTYPE = 0;
		netifo.COLAT = 0.0;
		netifo.COLON = 0.0;
		netifo.ROVERNUM = 0;
	}


	// ��������Ƿ��Ѿ����ڣ������������
	//if (IDOK == NetAdd.DoModal() && !m_sql.netIsExist(NetAdd.mv_NetName))
	{
		////////netinfo netifo;
		////////netifo.NETNAME = NetAdd.mv_NetName;
		////////netifo.SLNSYS = NetAdd.mv_slnsys;
		////////netifo.SLNPHS = NetAdd.mv_slnphs;
		////////netifo.EMAIL = NetAdd.mv_email;
		////////netifo.OWNER = NetAdd.mv_Admin;
		////////netifo.PHONE = NetAdd.mv_phone;
		////////netifo.PINCHARGE = NetAdd.mv_Company;
		////////netifo.WORKPATH = NetAdd.mv_WorkPath;
		////////netifo.BASENUM = 0;
		////////netifo.BRDCTYPE = 0;
		////////netifo.COLAT = 0.0;
		////////netifo.COLON = 0.0;
		////////netifo.ROVERNUM = 0;
		////////netifo.COMMENT = "";
		//m_sql.insert_net2db(&netifo);
		//updateTreeList();
	}
	//else if (!NetAdd.mv_NetName.IsEmpty() && m_sql.netIsExist(NetAdd.mv_NetName))
	//{
	//	QMessageBox::information(this, "��ʾ", "���������Ѵ��ڣ������ظ���ӣ�");
	//	return;
	//}

	// ɾ���Ի���
	delete dlgNetCreate;
}