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
	//itTopItem ����ڵ�;  itGroupItem ��ڵ㣻 itImageItem ͼƬ
	enum    treeItemType { itTopItem = 1001, itGroupItem, itImageItem };

	//ö�����ͣ���ʾ�к�
	enum    treeColNum { colItem = 0, colItemType = 1 }; //Ŀ¼���еı�Ŷ���

	void    iniTree();//Ŀ¼����ʼ��
	void    addNetItem(QTreeWidgetItem *parItem);//���һ��Ŀ¼�ڵ�

	//QString getFinalFolderName(const QString &fullPathName);//��Ŀ¼ȫ�����л�ȡ�����ļ�������

	//void    addImageItem(QTreeWidgetItem *parItem, QString aFilename);//���һ��ͼƬ�ڵ�

	//void    displayImage(QTreeWidgetItem *item); //��ʾһ��ͼƬ�ڵ��ͼƬ

	//void    changeItemCaption(QTreeWidgetItem *item); //�����ı�ڵ����

private slots:
	void on_treeWidget_customContextMenuRequested(const QPoint &pos);   //�����˵�
};
