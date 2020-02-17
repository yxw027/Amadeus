#pragma once

#include <QDialog>
#include <QString>
#include "ui_DialogStationNetAdd.h"


class DialogStationNetAdd : public QDialog
{
	Q_OBJECT

public:
	DialogStationNetAdd(QWidget *parent = Q_NULLPTR);
	~DialogStationNetAdd();

	QString DialogStationNetAdd::getWorkName();
	QString DialogStationNetAdd::getWorkDirection();
	QString DialogStationNetAdd::getWorker();
	QString DialogStationNetAdd::getOrganization();
	QString DialogStationNetAdd::getPhoneNum();
	QString DialogStationNetAdd::getPostalCode();
	int DialogStationNetAdd::getProSystem();
	int DialogStationNetAdd::getProFreq();
	int DialogStationNetAdd::getProDuration();

private:
	Ui::DialogStationNetAdd ui;

	private slots:
	void on_pushButtonDirSelect_clicked();
};
