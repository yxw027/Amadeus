#pragma once

#include <QDialog>
#include "ui_DialogStationSiteAdd.h"
#include <QSqlQueryModel>

class DialogStationSiteAdd : public QDialog
{
	Q_OBJECT

public:
	DialogStationSiteAdd(QWidget *parent = Q_NULLPTR);
	DialogStationSiteAdd(const QString arg, QWidget *parent = Q_NULLPTR);
	~DialogStationSiteAdd();

public:
	void initDialog();
	void initComboBoxConnectType();
	void initTableViewSite();

private:
	Ui::DialogStationSiteAdd ui;

	QString netName;
	QSqlQueryModel *qryModel;
	QItemSelectionModel *theSelection;

	QString connectType;
	QString netAddress;
	QString netPort;

private:
	void initSerialPortUI();
	void initNetPortUI();

private slots:
	void on_comboBoxConnectType_currentIndexChanged(int index);
	void on_pushButtonConnectTest_clicked();
};
