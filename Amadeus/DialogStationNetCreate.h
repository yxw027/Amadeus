#pragma once

#include <QDialog>
#include <QString>
#include "ui_DialogStationNetCreate.h"


class DialogStationNetCreate : public QDialog
{
	Q_OBJECT

public:
	DialogStationNetCreate(QWidget *parent = Q_NULLPTR);
	~DialogStationNetCreate();

	QString DialogStationNetCreate::getWorkName();
	QString DialogStationNetCreate::getWorkDirection();
	QString DialogStationNetCreate::getWorker();
	QString DialogStationNetCreate::getOrganization();
	QString DialogStationNetCreate::getPhoneNum();
	QString DialogStationNetCreate::getPostalCode();
	int DialogStationNetCreate::getProSystem();
	int DialogStationNetCreate::getProFreq();
	int DialogStationNetCreate::getProDuration();

private:
	Ui::DialogStationNetCreate ui;

	private slots:
	void on_pushButtonDirSelect_clicked();
};
