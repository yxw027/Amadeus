#include "DialogStationNetAdd.h"
#include <QFileDialog>
#include "./Platform/utf8.h"


DialogStationNetAdd::DialogStationNetAdd(QWidget *parent)
	: QDialog(parent)
{
	ui.setupUi(this);
}

DialogStationNetAdd::~DialogStationNetAdd()
{
}


// 参数配置函数
QString DialogStationNetAdd::getWorkName()
{
	return ui.lineEditWorkName->text();
}

QString DialogStationNetAdd::getWorkDirection()
{
	return ui.lineEditWorkDirection->text();
}

QString DialogStationNetAdd::getWorker()
{
	return ui.lineEditWorker->text();
}

QString DialogStationNetAdd::getOrganization()
{
	return ui.lineEditOrganization->text();
}

QString DialogStationNetAdd::getPhoneNum()
{
	return ui.lineEditPhoneNum->text();
}

QString DialogStationNetAdd::getPostalCode()
{
	return ui.lineEditPostalCode->text();
}

int DialogStationNetAdd::getProSystem()
{
	return ui.comboBoxProSystem->currentIndex();
}

int DialogStationNetAdd::getProFreq()
{
	return ui.comboBoxProFreq->currentIndex();
}

int DialogStationNetAdd::getProDuration()
{
	return ui.comboBoxProDuration->currentIndex();
}


// 路径选择按钮槽函数
void DialogStationNetAdd::on_pushButtonDirSelect_clicked()
{
	QString curPath = QCoreApplication::applicationDirPath();
	QString dlgTitle = "选择一个目录";
	QString selectedDir = QFileDialog::getExistingDirectory(this,
		dlgTitle, curPath, QFileDialog::ShowDirsOnly);
	if (!selectedDir.isEmpty())
		ui.lineEditWorkDirection->setText(selectedDir);
}