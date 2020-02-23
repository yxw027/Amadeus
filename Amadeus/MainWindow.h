#pragma once

#include <QtWidgets/QMainWindow>
#include "ui_MainWindow.h"
#include <qplaintextedit.h>
#include "DialogStation.h"
#include "DialogMenuDBSet.h"

class MainWindow : public QMainWindow
{
	Q_OBJECT

public:
	MainWindow(QWidget *parent = Q_NULLPTR);
	void closeEvent(QCloseEvent *event);

private:
	Ui::MainWindowClass ui;
	DialogMenuDBSet *dlgDBSet;
	DialogStation *DialogStationManege;
	QPlainTextEdit *plainTextEditMsgPro;
	QPlainTextEdit *plainTextEditMsgRcv;
	QPlainTextEdit *plainTextEditMsgFind;

private:
	void createPlainTextEditMsgPro();
	void createPlainTextEditMsgRcv();
	void createPlainTextEditMsgFind();
	void createDialogStation();
	void saveDockWidgetLayoutSet();
	void restoreDockWidgetLayoutSet();

private slots:
	void on_actionDBSet_triggered();
	void on_actionCtrlDialogStaion_triggered();
	void on_actionCtrlMsgPro_triggered();
	void on_actionCtrlMsgRcv_triggered();
	void on_actionCtrlMsgFind_triggered();
	void on_showMsg_triggered(const QString &type, const QString &msg);
};
