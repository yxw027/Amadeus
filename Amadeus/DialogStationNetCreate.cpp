#include "DialogStationNetCreate.h"
#include <QFileDialog>

// ���QT��VS��������������
#ifdef WIN32
#pragma execution_character_set("utf-8")
#endif

DialogStationNetCreate::DialogStationNetCreate(QWidget *parent)
	: QDialog(parent)
{
	ui.setupUi(this);
}

DialogStationNetCreate::~DialogStationNetCreate()
{
}


// �������ú���
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


// ·��ѡ��ť�ۺ���
void DialogStationNetCreate::on_pushButtonDirSelect_clicked()
{
	QString curPath = QCoreApplication::applicationDirPath();
	QString dlgTitle = "ѡ��һ��Ŀ¼";
	QString selectedDir = QFileDialog::getExistingDirectory(this,
		dlgTitle, curPath, QFileDialog::ShowDirsOnly);
	if (!selectedDir.isEmpty())
		ui.lineEditWorkDirection->setText(selectedDir);
}