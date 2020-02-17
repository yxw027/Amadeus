#include "DialogStation.h"
#include "DialogStationNetCreate.h"
#include "globalfunc.h"
#include "msql.h"
#include <QMenu>
#include <QMessageBox>

// 解决QT在VS里中文乱码问题
#ifdef WIN32
#pragma execution_character_set("utf-8")
#endif

const QString ROOT_LB = "监测站";

DialogStation::DialogStation(QWidget *parent)
	: QDialog(parent)
{
	ui.setupUi(this);

	// 解决右键菜单不弹出问题，机制暂不清楚
	ui.treeWidget->setContextMenuPolicy(Qt::CustomContextMenu);

	iniTree();

	// 读取数据库数据更新树节点
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

// 添加根节点
QTreeWidgetItem * DialogStation::addRootNode()
{ 
	// Item的Data存储的string
	QString dataStr = ""; 

	// 设置ICON的图标
	QIcon icon(":/icons/Resources/obsRoot.bmp"); 

	// 新建节点
	QTreeWidgetItem *item = new QTreeWidgetItem(DialogStation::TREE_NOTE_CORSTITLE);
	item->setIcon(DialogStation::nodeName, icon);
	item->setText(DialogStation::nodeName, ROOT_LB);
	item->setText(DialogStation::nodeState, "");
	item->setFlags(Qt::ItemIsSelectable | Qt::ItemIsEnabled);
	item->setData(DialogStation::nodeName, Qt::UserRole, QVariant(dataStr));

	// 添加为顶层节点
	ui.treeWidget->addTopLevelItem(item);
	return item;
}

// 添加一个监测网络节点
QTreeWidgetItem * DialogStation::addNetNode(QTreeWidgetItem *parItem, QString netname)
{
	// 设置ICON的图标
	QIcon icon(":/icons/Resources/obsNet.bmp");

	// 新建节点
	QTreeWidgetItem *item = new QTreeWidgetItem(DialogStation::TREE_NOTE_CORSNAME);
	item->setIcon(nodeName, icon);
	item->setText(nodeName, netname);
	item->setText(nodeState, "");
	item->setFlags(Qt::ItemIsSelectable | Qt::ItemIsUserCheckable | Qt::ItemIsEnabled );
	item->setData(nodeName, Qt::UserRole, QVariant(""));

	// 在父节点下添加子节点
	parItem->addChild(item); 

	return item;
}

// 添加一个监测站节点
QTreeWidgetItem * DialogStation::addStationNode(QTreeWidgetItem *parItem, QString stationname, QString stationtype)
{
	// 设置ICON的图标
	QIcon icon;
	switch (stationtype.toInt())
	{
	case 1: icon = QIcon(":/icons/Resources/obsBase.bmp"); break;
	case 2: icon = QIcon(":/icons/Resources/obsRover.bmp");  break;
	case 3: icon = QIcon(":/icons/Resources/obsRover.bmp");  break;
	default:
		break;
	}

	// 新建节点
	QTreeWidgetItem *item = new QTreeWidgetItem(DialogStation::TREE_NOTE_MPNAME);
	item->setIcon(nodeName, icon);
	item->setText(nodeName, stationname);
	item->setText(nodeState, "");
	item->setFlags(Qt::ItemIsSelectable | Qt::ItemIsUserCheckable | Qt::ItemIsEnabled);
	item->setData(nodeName, Qt::UserRole, QVariant(""));

	// 在父节点下添加子节点
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
		// 添加新的节点
//		try
		{
			QStringList netList;
			QString netName;
			// 实时流数据来源列表筛选
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
					QTreeWidgetItem *netNode = addNetNode(rootNode, netName);

					// 挂载点列表筛选
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
					break;	//添加一个子网限制
#endif				
				}
			}
			else if (rootName == "增强信息显示")
			{
			}
			else if (rootName == "用户信息")
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
	// 遍历所有根节点
	for (int rootIndex = 0; rootIndex < ui.treeWidget->topLevelItemCount(); rootIndex++)
	{
		QTreeWidgetItem *rootItem = ui.treeWidget->topLevelItem(rootIndex);
		// 遍历子网节点
		int netNum = rootItem->childCount();
		for (int netIndex = 0; netIndex < netNum; netIndex++)
		{
			QTreeWidgetItem *netItem = rootItem->child(0);
			// 遍历设备节点
			int devNum = netItem->childCount();
			for (int devIndex = 0; devIndex < devNum; devIndex++)
			{
				// 删除设备节点,删掉一个其后面节点会自动向前靠
				QTreeWidgetItem *devItem = netItem->child(0);
				netItem->removeChild(devItem);
				delete devItem;
			}
			// 删除子网节点,删掉一个其后面节点会自动向前靠
			rootItem->removeChild(netItem);
			delete netItem;
		}
	}
}



void DialogStation::on_treeWidget_customContextMenuRequested()
{
	// 创建菜单
	QMenu* menuList = new QMenu(this);

	int itemType = ui.treeWidget->currentItem()->type();

	switch (itemType)
	{
	case TREE_NOTE_CORSTITLE:
		// 添加Actions创建菜单项
		menuList->addAction(ui.actionNetCreate);
		break;
	case TREE_NOTE_CORSNAME:
		//menuList->addAction(ui.action_Connect);
		//menuList->addAction(ui.action_Disconnect);
		//menuList->addAction(ui.action_Disconnect);
		//// 添加分隔条
		// menuList->addSeparator();
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
	// 创建对话框
	DialogStationNetCreate *dlgNetCreate = new DialogStationNetCreate(this);

	// 设置对话框大小为固定
	Qt::WindowFlags flags = dlgNetCreate->windowFlags();
	dlgNetCreate->setWindowFlags(flags | Qt::MSWindowsFixedSizeDialogHint);

	//HTREEITEM hChild = m_wndFileView.GetChildItem(m_wndFileView.GetSelectedItem());
	//while (hChild) {
	//	CString NetName = m_wndFileView.GetItemText(hChild);
	//	if (theApp.m_NetList.find(NetName) != theApp.m_NetList.end() && theApp.m_NetList.at(NetName)->m_runflag) {
	//		AfxMessageBox("请先停止子网解算！");
	//		return;
	//	}
	//	hChild = m_wndFileView.GetNextSiblingItem(hChild);
	//}

	// 以模态方式显示对话框
	int ret = dlgNetCreate->exec();
	if (ret = QDialog::Accepted)
	{
		if ((dlgNetCreate->getWorkName()).isEmpty())
		{
			QMessageBox::information(NULL, "提示：", "子网名称不能为空");
			return;
		}

		if (m_sql.netIsExist(dlgNetCreate->getWorkName()))
		{
			QMessageBox::information(NULL,"提示：","子网已存在，请勿重复添加");
			return;
		}
				
		netinfo netifo;
		netifo.NETNAME = dlgNetCreate->getWorkName();
		netifo.WORKPATH = dlgNetCreate->getWorkDirection();
		netifo.OWNER = dlgNetCreate->getWorker();
		netifo.PINCHARGE = dlgNetCreate->getOrganization();
		netifo.PHONE = dlgNetCreate->getPhoneNum();
		netifo.EMAIL = dlgNetCreate->getPostalCode();
		netifo.SLNSYS = dlgNetCreate->getProSystem();
		netifo.SLNPHS = dlgNetCreate->getProFreq();
		// 原程序在此处就没有处理静态解时长
		//		netifo.SLNSES = dlgNetCreate->getProDuration();
		netifo.BASENUM = 0;
		netifo.BRDCTYPE = 0;
		netifo.COLAT = 0.0;
		netifo.COLON = 0.0;
		netifo.ROVERNUM = 0;
		m_sql.insert_net2db(&netifo);
		updateTreeList();
	}

	// 删除对话框
	delete dlgNetCreate;
}