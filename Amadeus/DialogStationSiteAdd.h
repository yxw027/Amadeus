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

private:
	Ui::DialogStationSiteAdd ui;
	void initDialog();

private:
	QString netName;

	QSqlQueryModel *qryModel;
	QItemSelectionModel *theSelection;
};
