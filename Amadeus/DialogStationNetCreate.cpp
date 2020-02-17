#include "DialogStationNetCreate.h"
#include <QFileDialog>
#include "./Platform/utf8.h"


DialogStationNetCreate::DialogStationNetCreate(QWidget *parent)
	: QDialog(parent)
{
	ui.setupUi(this);
}

DialogStationNetCreate::~DialogStationNetCreate()
{
}


// 参数配置函数
QString DialogStationNetCreate::getWorkName()
{
	return ui.lineEditWorkName->text();
}

QString DialogStationNetCreate::getWorkDirection()
{
	return ui.lineEditWorkDirection->text();
}

QString DialogStationNetCreate::getWorker()
{
	return ui.lineEditWorker->text();
}

QString DialogStationNetCreate::getOrganization()
{
	return ui.lineEditOrganization->text();
}

QString DialogStationNetCreate::getPhoneNum()
{
	return ui.lineEditPhoneNum->text();
}

QString DialogStationNetCreate::getPostalCode()
{
	return ui.lineEditPostalCode->text();
}

int DialogStationNetCreate::getProSystem()
{
	return ui.comboBoxProSystem->currentIndex();
}

int DialogStationNetCreate::getProFreq()
{
	return ui.comboBoxProFreq->currentIndex();
}

int DialogStationNetCreate::getProDuration()
{
	return ui.comboBoxProDuration->currentIndex();
}


// 路径选择按钮槽函数
void DialogStationNetCreate::on_pushButtonDirSelect_clicked()
{
	QString curPath = QCoreApplication::applicationDirPath();
	QString dlgTitle = "选择一个目录";
	QString selectedDir = QFileDialog::getExistingDirectory(this,
		dlgTitle, curPath, QFileDialog::ShowDirsOnly);
	if (!selectedDir.isEmpty())
		ui.lineEditWorkDirection->setText(selectedDir);
}