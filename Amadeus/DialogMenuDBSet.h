#pragma once

#include <QDialog>
#include "ui_DialogMenuDBSet.h"

class DialogMenuDBSet : public QDialog
{
	Q_OBJECT

public:
	DialogMenuDBSet(QWidget *parent = Q_NULLPTR);
	~DialogMenuDBSet();

private:
	Ui::DialogMenuDBSet ui;

private:
	bool initDialog();

private:
	bool cntflag;
	QString	DBPath;
	QString	DBUser;
	QString	DBname;
	QString	DBPassword;

private slots:
	void on_pushButtonTestConnect_clicked();
	void on_pushButtonOK_clicked();
};
