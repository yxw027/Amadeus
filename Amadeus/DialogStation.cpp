#include "DialogStation.h"
#include <QMenu>
#include <QMessageBox>

// 解决QT在VS里中文乱码问题
#ifdef WIN32
#pragma execution_character_set("utf-8")
#endif

DialogStation::DialogStation(QWidget *parent)
	: QDialog(parent)
{
	ui.setupUi(this);
	iniTree();

	// 解决右键菜单不弹出问题，机制暂不清楚
	ui.treeWidget->setContextMenuPolicy(Qt::CustomContextMenu);
}

DialogStation::~DialogStation()
{

}

// 添加根节点
void DialogStation::iniTree()
{ 
	// Item的Data存储的string
	QString dataStr = ""; 

	// 清除目录树所有节点
	ui.treeWidget->clear();

	// 设置ICON的图标
	QIcon icon(":/icons/Resources/obsRoot.bmp"); 

	// 新建节点
	QTreeWidgetItem *item = new QTreeWidgetItem(DialogStation::itTopItem); 
	item->setIcon(DialogStation::colItem, icon);
	item->setText(DialogStation::colItem, "监测站");
	item->setText(DialogStation::colItemType, "");
	item->setFlags(Qt::ItemIsSelectable | Qt::ItemIsEnabled);
	item->setData(DialogStation::colItem, Qt::UserRole, QVariant(dataStr)); 

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
	QTreeWidgetItem *item = new QTreeWidgetItem(DialogStation::itGroupItem);
	item->setIcon(colItem, icon); 
	item->setText(colItem, "监测网络1");
	item->setText(colItemType, "type=itGroupItem"); 
	item->setFlags(Qt::ItemIsSelectable | Qt::ItemIsUserCheckable | Qt::ItemIsEnabled );
	item->setData(colItem, Qt::UserRole, QVariant("")); 

	// 在父节点下面添加子节点
	parItem->addChild(item); 
}

void DialogStation::on_treeWidget_customContextMenuRequested(const QPoint &pos)
{
	Q_UNUSED(pos);

	// 创建菜单
	QMenu* menuList = new QMenu(this); 

	// 添加Actions创建菜单项
	menuList->addAction(ui.action_CreateNewNet);

	// 添加分隔条
	menuList->addSeparator();

	menuList->addAction(ui.action_Add);

	// 在鼠标光标位置显示右键快捷菜单
	menuList->exec(QCursor::pos()); 

	// 手工创建的指针必须手工删除
	delete menuList; 
}
