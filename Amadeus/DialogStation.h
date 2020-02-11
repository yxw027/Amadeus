#pragma once

#include <QDialog>
#include "ui_DialogStation.h"

class DialogStation : public QDialog
{
	Q_OBJECT

public:
	DialogStation(QWidget *parent = Q_NULLPTR);
	~DialogStation();

private:
	Ui::DialogStation ui;

private:

	// TREE_NOTE_CORSTITLE  ���ڵ�
	// TREE_NOTE_CORSNAME   ����ڵ�
	// TREE_NOTE_MPNAME     �豸�ڵ�
	enum    treeItemType { TREE_NOTE_CORSTITLE = 1001, TREE_NOTE_CORSNAME, TREE_NOTE_MPNAME };

	//ö�����ͣ���ʾ�к�
	enum    treeColNum { nodeName = 0, nodeState = 1 }; //Ŀ¼���еı�Ŷ���

	void    iniTree();		// Ŀ¼����ʼ��
	void	addRootNode();  // ��Ӹ��ڵ�
	void    addNetItem(QTreeWidgetItem *parItem);	// ���һ��Ŀ¼�ڵ�
	void    ShowMenuStation();	// �����Ҽ�����˵�

private slots:
	void on_treeWidget_customContextMenuRequested();   //�����˵�
	void on_actionNetCreate_triggered();
};
