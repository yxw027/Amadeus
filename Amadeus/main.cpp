#include "MainWindow.h"
#include <QtWidgets/QApplication>

int main(int argc, char *argv[])
{
	QApplication a(argc, argv);
	MainWindow w;

	// 设置主窗口标题
	w.setWindowTitle("Amadeus 1.0.1");

	// 设置主窗口图标
	QIcon icon = QIcon(":/icons/Resources/MainWindowTitle.png");
	w.setWindowIcon(icon);

	w.show();
	return a.exec();
}
