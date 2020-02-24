#include "MainWindow.h"
#include <QtWidgets/QApplication>

int main(int argc, char *argv[])
{
	QApplication a(argc, argv);
	MainWindow w;

	// ���������ڱ���
	w.setWindowTitle("Amadeus 1.0.1");

	// ����������ͼ��
	QIcon icon = QIcon(":/icons/Resources/MainWindowTitle.png");
	w.setWindowIcon(icon);

	w.show();
	return a.exec();
}
