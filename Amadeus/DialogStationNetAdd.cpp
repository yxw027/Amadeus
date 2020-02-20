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

netArg DialogStationNetAdd::getNetArg()
{
	netArg arg;
	arg.netName = ui.lineEditWorkName->text();
	arg.workPath = ui.lineEditWorkDirection->text();
	arg.company = ui.lineEditOrganization->text();
	arg.admin = ui.lineEditWorker->text();
	arg.phone = ui.lineEditPhoneNum->text();
	arg.email = ui.lineEditPostalCode->text();
	arg.slnsys = ui.comboBoxProSystem->currentIndex();
	arg.slnphs = ui.comboBoxProFreq->currentIndex();
	arg.slnses = ui.comboBoxProDuration->currentIndex();
	return arg;
}

void DialogStationNetAdd::setNetArg(const netArg arg)
{
	 ui.lineEditWorkName->setText(arg.netName);
	 ui.lineEditWorkDirection->setText(arg.workPath);
	 ui.lineEditOrganization->setText(arg.company);
	 ui.lineEditWorker->setText(arg.admin);
	 ui.lineEditPhoneNum->setText(arg.phone);
	 ui.lineEditPostalCode->setText(arg.email);
	 ui.comboBoxProSystem->setCurrentIndex(arg.slnsys);
	 ui.comboBoxProFreq->setCurrentIndex(arg.slnphs);
	 ui.comboBoxProDuration->setCurrentIndex(arg.slnses);
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