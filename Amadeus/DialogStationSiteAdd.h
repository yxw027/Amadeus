#pragma once

#include <QDialog>
#include "ui_DialogStationSiteAdd.h"

class DialogStationSiteAdd : public QDialog
{
	Q_OBJECT

public:
	DialogStationSiteAdd(QWidget *parent = Q_NULLPTR);
	~DialogStationSiteAdd();

private:
	Ui::DialogStationSiteAdd ui;
};
