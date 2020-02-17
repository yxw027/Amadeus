#include "DialogStationNetCreate.h"
#include <QFileDialog>
#include <QMessageBox>
#include "msql.h"
#include "globalfunc.h"

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

void DialogStationNetCreate::on_pushButtonDirSelect_clicked()
{
	QString curPath = QCoreApplication::applicationDirPath();
	QString dlgTitle = "ѡ��һ��Ŀ¼";
	QString selectedDir = QFileDialog::getExistingDirectory(this,
		dlgTitle, curPath, QFileDialog::ShowDirsOnly);
	if (!selectedDir.isEmpty())
		ui.lineEditWorkDirection->setText(selectedDir);
}

void DialogStationNetCreate::on_pushButtonOK_clicked()
{
	if (ui.lineEditWorkName->text().isEmpty())
		return;

	if (m_sql.netIsExist(ui.lineEditWorkName->text()))
	{
		QMessageBox::information(NULL, "��ʾ��", "�����Ѵ��ڣ������ظ����");
		return;
	}

	netinfo netifo;
	netifo.NETNAME = ui.lineEditWorkName->text();
	netifo.WORKPATH = ui.lineEditWorkDirection->text();
	netifo.OWNER = ui.lineEditWorker->text();
	netifo.PINCHARGE = ui.lineEditOrganization->text();
	netifo.PHONE = ui.lineEditPhoneNum->text();
	netifo.EMAIL = ui.lineEditPostalCode->text();
	netifo.SLNSYS = ui.comboBoxProSystem->currentIndex();
	netifo.SLNPHS = ui.comboBoxProFreq->currentIndex();
	// ԭ�����ڴ˴�δû�д���̬��ʱ��
	//		netifo.SLNSES = ui.comboBoxProDuration->currentIndex();
	netifo.BASENUM = 0;
	netifo.BRDCTYPE = 0;
	netifo.COLAT = 0.0;
	netifo.COLON = 0.0;
	netifo.ROVERNUM = 0;
	m_sql.insert_net2db(&netifo);
}