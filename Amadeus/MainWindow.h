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
	void createDialogStation();
	void saveDockWidgetLayoutSet();
	void restoreDockWidgetLayoutSet();

private slots:
	void on_actionDBSet_triggered();
	void on_actionCtrlDialogStaion_triggered();
	void on_actionCtrlMsgPro_triggered();
	void on_actionCtrlMsgRcv_triggered();
	void on_actionCtrlMsgFind_triggered();
};
