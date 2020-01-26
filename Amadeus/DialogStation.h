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
	//itTopItem 顶层节点;  itGroupItem 组节点； itImageItem 图片
	enum    treeItemType { itTopItem = 1001, itGroupItem, itImageItem };

	//枚举类型，表示列号
	enum    treeColNum { colItem = 0, colItemType = 1 }; //目录树列的编号定义

	void    iniTree();//目录树初始化
	void    addNetItem(QTreeWidgetItem *parItem);//添加一个目录节点

	//QString getFinalFolderName(const QString &fullPathName);//从目录全名称中获取最后的文件夹名称

	//void    addImageItem(QTreeWidgetItem *parItem, QString aFilename);//添加一个图片节点

	//void    displayImage(QTreeWidgetItem *item); //显示一个图片节点的图片

	//void    changeItemCaption(QTreeWidgetItem *item); //遍历改变节点标题

private slots:
	void on_treeWidget_customContextMenuRequested(const QPoint &pos);   //弹出菜单
};
