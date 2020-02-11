#include "DialogStation.h"
#include "DialogStationNetCreate.h"
#include "globalfunc.h"
#include <QMenu>

// ���QT��VS��������������
#ifdef WIN32
#pragma execution_character_set("utf-8")
#endif

DialogStation::DialogStation(QWidget *parent)
	: QDialog(parent)
{
	ui.setupUi(this);

	// ����Ҽ��˵����������⣬�����ݲ����
	ui.treeWidget->setContextMenuPolicy(Qt::CustomContextMenu);

	iniTree();
	addRootNode();
}

DialogStation::~DialogStation()
{

}

void DialogStation::iniTree()
{
	// ���Ŀ¼�����нڵ�
	ui.treeWidget->clear();
}

// ��Ӹ��ڵ�
void DialogStation::addRootNode()
{ 
	// Item��Data�洢��string
	QString dataStr = ""; 

	// ����ICON��ͼ��
	QIcon icon(":/icons/Resources/obsRoot.bmp"); 

	// �½��ڵ�
	QTreeWidgetItem *item = new QTreeWidgetItem(DialogStation::TREE_NOTE_CORSTITLE);
	item->setIcon(DialogStation::nodeName, icon);
	item->setText(DialogStation::nodeName, "���վ");
	item->setText(DialogStation::nodeState, "");
	item->setFlags(Qt::ItemIsSelectable | Qt::ItemIsEnabled);
	item->setData(DialogStation::nodeName, Qt::UserRole, QVariant(dataStr));

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
	QTreeWidgetItem *item = new QTreeWidgetItem(DialogStation::TREE_NOTE_CORSNAME);
	item->setIcon(nodeName, icon);
	item->setText(nodeName, "�������1");
	item->setText(nodeState, "type=TREE_NOTE_CORSNAME");
	item->setFlags(Qt::ItemIsSelectable | Qt::ItemIsUserCheckable | Qt::ItemIsEnabled );
	item->setData(nodeName, Qt::UserRole, QVariant(""));

	// �ڸ��ڵ���������ӽڵ�
	parItem->addChild(item); 
}

void DialogStation::on_treeWidget_customContextMenuRequested()
{
	ShowMenuStation();
}

void DialogStation::ShowMenuStation()
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
		//UpdateTreeList();
	}
	//else if (!NetAdd.mv_NetName.IsEmpty() && m_sql.netIsExist(NetAdd.mv_NetName))
	//{
	//	QMessageBox::information(this, "��ʾ", "���������Ѵ��ڣ������ظ���ӣ�");
	//	return;
	//}

	// ɾ���Ի���
	delete dlgNetCreate;
}