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

	// TREE_NOTE_CORSTITLE  根节点
	// TREE_NOTE_CORSNAME   网络节点
	// TREE_NOTE_MPNAME     设备节点
	enum    treeItemType { TREE_NOTE_CORSTITLE = 1001, TREE_NOTE_CORSNAME, TREE_NOTE_MPNAME };

	//枚举类型，表示列号
	enum    treeColNum { nodeName = 0, nodeState = 1 }; //目录树列的编号定义

	void    iniTree();		// 目录树初始化
	void	addRootNode();  // 添加根节点
	void    addNetItem(QTreeWidgetItem *parItem);	// 添加一个目录节点
	void    ShowMenuStation();	// 弹出右键管理菜单

private slots:
	void on_treeWidget_customContextMenuRequested();   //弹出菜单
	void on_actionNetCreate_triggered();
};
