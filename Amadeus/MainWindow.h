#pragma once

#include <QtWidgets/QMainWindow>
#include "ui_MainWindow.h"

class MainWindow : public QMainWindow
{
	Q_OBJECT

public:
	MainWindow(QWidget *parent = Q_NULLPTR);
	void closeEvent(QCloseEvent *event);

private:
	Ui::MainWindowClass ui;

private:
	void createDialogStationManege();
	void saveDockWidgetLayoutSets();
	void restoreDockWidgetLayoutSets();

private slots:
	void on_actionDBSet_triggered();
};
