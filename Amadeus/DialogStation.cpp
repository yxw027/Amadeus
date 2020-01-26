#include "DialogStation.h"
#include <QMenu>
#include <QMessageBox>

// ���QT��VS��������������
#ifdef WIN32
#pragma execution_character_set("utf-8")
#endif

DialogStation::DialogStation(QWidget *parent)
	: QDialog(parent)
{
	ui.setupUi(this);
	iniTree();

	// ����Ҽ��˵����������⣬�����ݲ����
	ui.treeWidget->setContextMenuPolicy(Qt::CustomContextMenu);
}

DialogStation::~DialogStation()
{

}

// ��Ӹ��ڵ�
void DialogStation::iniTree()
{ 
	// Item��Data�洢��string
	QString dataStr = ""; 

	// ���Ŀ¼�����нڵ�
	ui.treeWidget->clear();

	// ����ICON��ͼ��
	QIcon icon(":/icons/Resources/obsRoot.bmp"); 

	// �½��ڵ�
	QTreeWidgetItem *item = new QTreeWidgetItem(DialogStation::itTopItem); 
	item->setIcon(DialogStation::colItem, icon);
	item->setText(DialogStation::colItem, "���վ");
	item->setText(DialogStation::colItemType, "");
	item->setFlags(Qt::ItemIsSelectable | Qt::ItemIsEnabled);
	item->setData(DialogStation::colItem, Qt::UserRole, QVariant(dataStr)); 

	// ���Ϊ����ڵ�
	ui.treeWidget->addTopLevelItem(item);

	// ������
	addNetItem(item);
}

// ���һ���������ڵ�
void DialogStation::addNetItem(QTreeWidgetItem *parItem)
{
	// ����ICON��ͼ��
	QIcon icon(":/icons/Resources/obsNet.bmp");

	// �½��ڵ�
	QTreeWidgetItem *item = new QTreeWidgetItem(DialogStation::itGroupItem);
	item->setIcon(colItem, icon); 
	item->setText(colItem, "�������1");
	item->setText(colItemType, "type=itGroupItem"); 
	item->setFlags(Qt::ItemIsSelectable | Qt::ItemIsUserCheckable | Qt::ItemIsEnabled );
	item->setData(colItem, Qt::UserRole, QVariant("")); 

	// �ڸ��ڵ���������ӽڵ�
	parItem->addChild(item); 
}

void DialogStation::on_treeWidget_customContextMenuRequested(const QPoint &pos)
{
	Q_UNUSED(pos);

	// �����˵�
	QMenu* menuList = new QMenu(this); 

	// ���Actions�����˵���
	menuList->addAction(ui.action_CreateNewNet);

	// ��ӷָ���
	menuList->addSeparator();

	menuList->addAction(ui.action_Add);

	// �������λ����ʾ�Ҽ���ݲ˵�
	menuList->exec(QCursor::pos()); 

	// �ֹ�������ָ������ֹ�ɾ��
	delete menuList; 
}
