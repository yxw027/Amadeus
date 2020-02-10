#pragma once

#include <QDialog>
#include "ui_DialogMenuNetSet.h"

class DialogMenuNetSet : public QDialog
{
	Q_OBJECT

public:
	DialogMenuNetSet(QWidget *parent = Q_NULLPTR);
	~DialogMenuNetSet();

private:
	Ui::DialogMenuNetSet ui;

private:
	bool initDialog();

private:
	bool cntflag;
	QString	mv_DBPath;
	QString	mv_DBUser;
	QString	mv_DBname;
	QString	mv_DBPassword;
};
