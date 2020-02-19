#pragma once

#include <QDialog>
#include <QString>
#include "ui_DialogStationNetAdd.h"

struct netArg
{
	QString	netName;
	QString	workPath;
	QString	company;
	QString	admin;
	QString	phone;
	QString	email;
	int	slnsys;
	int	slnphs;
	int	slnses;
};

class DialogStationNetAdd : public QDialog
{
	Q_OBJECT

public:
	DialogStationNetAdd(QWidget *parent = Q_NULLPTR);
	~DialogStationNetAdd();

	netArg getNetArg();
	void setNetArg(const netArg arg);

private:
	Ui::DialogStationNetAdd ui;

	private slots:
	void on_pushButtonDirSelect_clicked();
};
