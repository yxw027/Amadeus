#include "DialogStation.h"
#include "DialogStationNetAdd.h"
#include "DialogStationSiteAdd.h"
#include "msql.h"
#include "Platform/utf8.h"
#include <QMenu>
#include <QMessageBox>
#include "globalfunc.h"

const QString ROOT_LB = "监测站";

DialogStation::DialogStation(QWidget *parent)
	: QDialog(parent)
{
	ui.setupUi(this);
	setWindowTitle("测站管理");

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

				// 为所有解算子网节点添加测站节点
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
	// 创建右键弹出菜单
	QMenu* menuList = new QMenu(this);

	// 创建菜单项Action
	int itemType = ui.treeWidget->currentItem()->type();
	switch (itemType)
	{
	case TREE_NOTE_CORSTITLE:
		menuList->addAction(ui.actionNetAdd);
		break;
	case TREE_NOTE_CORSNAME:
		menuList->addAction(ui.actionNetSet);
		//menuList->addAction(ui.actionNetConnect);
		//menuList->addAction(ui.actionNetDisconnect);
		menuList->addAction(ui.actionNetDelete);	
		menuList->addAction(ui.actionSiteAdd);
		break;
	case TREE_NOTE_MPNAME:
		break;
	default:
		break;
	}

	// 显示菜单项 
	menuList->exec(QCursor::pos());

	// 删除菜单项
	delete menuList; 
}

void DialogStation::on_actionNetAdd_triggered()
{
	// 添加新子网，无需停止其他子网的解算

	// 创建对话框
	DialogStationNetAdd *dlgNetAdd = new DialogStationNetAdd(this);

	// 设置对话框大小为固定
	Qt::WindowFlags flags = dlgNetAdd->windowFlags();
	dlgNetAdd->setWindowFlags(flags | Qt::MSWindowsFixedSizeDialogHint);

	// 以模态方式显示对话框
	int ret = dlgNetAdd->exec();
	if (ret = QDialog::Accepted)
	{
		QString netName = dlgNetAdd->getNetArg().netName;
		if (netName.isEmpty())
			return;

		if (m_sql.netIsExist(netName))
		{
			QMessageBox::information(NULL, "提示：", "子网已存在，请勿重复添加");
			return;
		}

		netArg arg = dlgNetAdd->getNetArg();
		netinfo netifo;
		netifo.NETNAME = arg.netName;
		netifo.WORKPATH = arg.workPath;
		netifo.OWNER = arg.admin;
		netifo.PINCHARGE = arg.company;
		netifo.PHONE = arg.phone;
		netifo.EMAIL = arg.email;
		netifo.SLNSYS = arg.slnsys;
		netifo.SLNPHS = arg.slnphs;
		netifo.SLNSES = arg.slnsys;
		netifo.BASENUM = 0;
		netifo.BRDCTYPE = 0;
		netifo.COLAT = 0.0;
		netifo.COLON = 0.0;
		netifo.ROVERNUM = 0;
		m_sql.insert_net2db(&netifo);
		updateTreeList();
	}

	// 删除对话框
	delete dlgNetAdd;
}

void DialogStation::on_actionNetSet_triggered()
{
	QTreeWidgetItem *currentItem = ui.treeWidget->currentItem();
	QString cstr = currentItem->text(nodeName);
	if (theApp.m_NetList.find(cstr) != theApp.m_NetList.end()) 
	{
		if (theApp.m_NetList.value(cstr)->m_runflag)
		{
			QMessageBox::information(this,"提示", "请先停止子网解算！");
			return;
		}
		//for (auto psock = theApp.m_NetList.value(cstr)->m_SiteList.begin(); psock != theApp.m_NetList.value(cstr)->m_SiteList.end(); psock++)
		//{
		//	if (psock.value()->psock.state) 
		//	{
		//		QMessageBox::information(this, "提示", "请先停止数据接收！");
		//		return;
		//	}
		//}
	}

	// 创建对话框
	DialogStationNetAdd *dlgNetAdd = new DialogStationNetAdd(this);

	// 设置对话框大小为固定
	Qt::WindowFlags flags = dlgNetAdd->windowFlags();
	dlgNetAdd->setWindowFlags(flags | Qt::MSWindowsFixedSizeDialogHint);

	netinfo netifo;
	m_sql.update_netinfo2str(cstr, &netifo);

	netArg arg;
	arg.netName = netifo.NETNAME;
	arg.workPath = netifo.WORKPATH;
	arg.company = netifo.PINCHARGE;
	arg.admin = netifo.OWNER;
	arg.phone = netifo.PHONE;
	arg.email = netifo.EMAIL;
	arg.slnsys = netifo.SLNSYS;
	arg.slnphs = netifo.SLNPHS;
	arg.slnses = netifo.SLNSES;
	dlgNetAdd->setNetArg(arg);

	// 以模态方式显示对话框
	int ret = dlgNetAdd->exec();
	if (ret = QDialog::Accepted)
	{
		QString tNetName = arg.netName;
		if (tNetName.isEmpty())
			return;

		auto pNet = theApp.m_NetList.value(tNetName);
		// workpath没处理
		arg = dlgNetAdd->getNetArg();
		pNet->m_Netinfo.NETNAME = arg.netName;
		pNet->m_Netinfo.SLNSYS = arg.slnsys;
		pNet->m_Netinfo.SLNSES = arg.slnses;
		pNet->m_Netinfo.SLNPHS = arg.slnphs;
		pNet->m_Netinfo.EMAIL = arg.email;
		pNet->m_Netinfo.OWNER = arg.admin;
		pNet->m_Netinfo.PHONE = arg.phone;
		pNet->m_Netinfo.PINCHARGE = arg.company;

		// 删除旧项，插入新项
		NetList::iterator Net_i(theApp.m_NetList.find(tNetName));
		theApp.m_NetList.erase(Net_i);
		theApp.m_NetList[arg.netName] = pNet;

		// 更新该项在数据库中的记录
		netinfo netifo;
		netifo.NETNAME = arg.netName;
		netifo.SLNSYS = arg.slnsys;
		netifo.SLNPHS = arg.slnphs;
		netifo.SLNSES = arg.slnses;
		netifo.EMAIL = arg.email;
		netifo.OWNER = arg.admin;
		netifo.PHONE = arg.phone;
		netifo.PINCHARGE = arg.company;
		netifo.WORKPATH = arg.workPath;
		netifo.BASENUM = 0;
		netifo.BRDCTYPE = 0;
		netifo.COLAT = 0.0;
		netifo.COLON = 0.0;
		netifo.ROVERNUM = 0;
		netifo.COMMENT = "";
		m_sql.update_netinfo2db(tNetName, &netifo);

		// 更新新挂载点名称至界面
		QTreeWidgetItem *item = new QTreeWidgetItem(DialogStation::TREE_NOTE_CORSNAME);
		currentItem->setText(nodeName, arg.netName);
	}

	// 删除对话框
	delete dlgNetAdd;
}

void DialogStation::on_actionNetDelete_triggered()
{
	// 确认删除
	QString dlgTitle = "确认";
	QString strInfo = "是否确认删除？";
	QMessageBox::StandardButton  defaultBtn = QMessageBox::NoButton; 
	QMessageBox::StandardButton result;
	result = QMessageBox::question(this, dlgTitle, strInfo,
		QMessageBox::Yes | QMessageBox::No ,
		defaultBtn);
	 if (result == QMessageBox::No)
		return;

	QTreeWidgetItem *currentItem = ui.treeWidget->currentItem();
	QString cstr = currentItem->text(nodeName);
	auto netidx = theApp.m_NetList.find(cstr);
	if (netidx != theApp.m_NetList.end() && netidx.value()->m_runflag) 
	{
		QMessageBox::information(this,"提示","请先停止子网解算");
		return;
	}
	m_sql.erasenet(cstr);
	auto siteidx = netidx.value()->m_SiteList.begin();
	while (netidx.value()->m_SiteList.size())
	{
		netidx.value()->m_SiteList.erase(netidx.value()->m_SiteList.begin());
	}
	updateTreeList();
}

void DialogStation::on_actionSiteAdd_triggered()
{
	QTreeWidgetItem *currentItem = ui.treeWidget->currentItem();
	QString netName = currentItem->text(nodeName);
	if (theApp.m_NetList.find(netName) != theApp.m_NetList.end())
	{
		if (theApp.m_NetList.value(netName)->m_runflag)
		{
			QMessageBox::information(this, "提示", "请先停止子网解算！");
			return;
		}
		//for (auto psock = theApp.m_NetList.value(cstr)->m_SiteList.begin(); psock != theApp.m_NetList.value(cstr)->m_SiteList.end(); psock++)
		//{
		//	if (psock.value()->psock.state) 
		//	{
		//		QMessageBox::information(this, "提示", "请先停止数据接收！");
		//		return;
		//	}
		//}
	}
	DialogStationSiteAdd *dlgSiteAdd = new DialogStationSiteAdd(netName,this);
	// 设置对话框大小为固定
	Qt::WindowFlags flags = dlgSiteAdd->windowFlags();
	dlgSiteAdd->setWindowFlags(flags | Qt::MSWindowsFixedSizeDialogHint);

	// 以模态方式显示对话框
	int ret = dlgSiteAdd->exec();
	if (ret = QDialog::Accepted)
	{
//		UpdateData(false);
//		updateTreeList();
	}

	// 删除对话框
	delete dlgSiteAdd;
}