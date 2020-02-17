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

private:
	Ui::DialogStationNetCreate ui;

private slots:
	void on_pushButtonDirSelect_clicked();
	void on_pushButtonOK_clicked();
};
