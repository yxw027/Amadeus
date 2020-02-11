#include "DialogStation.h"
#include "DialogStationNetCreate.h"
#include "globalfunc.h"
#include <QMenu>

// 解决QT在VS里中文乱码问题
#ifdef WIN32
#pragma execution_character_set("utf-8")
#endif

DialogStation::DialogStation(QWidget *parent)
	: QDialog(parent)
{
	ui.setupUi(this);

	// 解决右键菜单不弹出问题，机制暂不清楚
	ui.treeWidget->setContextMenuPolicy(Qt::CustomContextMenu);

	iniTree();
	addRootNode();
}

DialogStation::~DialogStation()
{

}

void DialogStation::iniTree()
{
	// 清除目录树所有节点
	ui.treeWidget->clear();
}

// 添加根节点
void DialogStation::addRootNode()
{ 
	// Item的Data存储的string
	QString dataStr = ""; 

	// 设置ICON的图标
	QIcon icon(":/icons/Resources/obsRoot.bmp"); 

	// 新建节点
	QTreeWidgetItem *item = new QTreeWidgetItem(DialogStation::TREE_NOTE_CORSTITLE);
	item->setIcon(DialogStation::nodeName, icon);
	item->setText(DialogStation::nodeName, "监测站");
	item->setText(DialogStation::nodeState, "");
	item->setFlags(Qt::ItemIsSelectable | Qt::ItemIsEnabled);
	item->setData(DialogStation::nodeName, Qt::UserRole, QVariant(dataStr));

	// 添加为顶层节点
	ui.treeWidget->addTopLevelItem(item);

	// 测试用
	addNetItem(item);
}

// 添加一个监测网络节点
void DialogStation::addNetItem(QTreeWidgetItem *parItem)
{
	// 设置ICON的图标
	QIcon icon(":/icons/Resources/obsNet.bmp");

	// 新建节点
	QTreeWidgetItem *item = new QTreeWidgetItem(DialogStation::TREE_NOTE_CORSNAME);
	item->setIcon(nodeName, icon);
	item->setText(nodeName, "监测网络1");
	item->setText(nodeState, "type=TREE_NOTE_CORSNAME");
	item->setFlags(Qt::ItemIsSelectable | Qt::ItemIsUserCheckable | Qt::ItemIsEnabled );
	item->setData(nodeName, Qt::UserRole, QVariant(""));

	// 在父节点下面添加子节点
	parItem->addChild(item); 
}

void DialogStation::on_treeWidget_customContextMenuRequested()
{
	ShowMenuStation();
}

void DialogStation::ShowMenuStation()
{
	// 创建菜单
	QMenu* menuList = new QMenu(this);

	int currentItemType = ui.treeWidget->currentItem()->type();

	switch (currentItemType)
	{
	case TREE_NOTE_CORSTITLE:
		// 添加Actions创建菜单项
		menuList->addAction(ui.actionNetCreate);
		//// 添加分隔条
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
	// 创建对话框
	DialogStationNetCreate *dlgNetCreate = new DialogStationNetCreate(this);

	// 设置对话框大小为固定
	Qt::WindowFlags flags = dlgNetCreate->windowFlags();
	dlgNetCreate->setWindowFlags(flags | Qt::MSWindowsFixedSizeDialogHint);

	// 以模态方式显示对话框
	int ret = dlgNetCreate->exec();
	if (ret = QDialog::Accepted)
	{
		// OK按钮被按下
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
	}


	// 检查子网是否已经存在，不存在则添加
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
		//UpdateTreeList();
	}
	//else if (!NetAdd.mv_NetName.IsEmpty() && m_sql.netIsExist(NetAdd.mv_NetName))
	//{
	//	QMessageBox::information(this, "提示", "子网名称已存在，请勿重复添加！");
	//	return;
	//}

	// 删除对话框
	delete dlgNetCreate;
}