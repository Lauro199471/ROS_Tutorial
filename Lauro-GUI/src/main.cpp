#include <QtGui>
#include <QApplication>
#include <string>
#include "../include/main_window.hpp"
#include "../include/communication.hpp"
#include "../include/common_vars.hpp"

int count ; // global count used in communication.cpp
int user_count_val;
std::string received_string;

int main(int argc, char **argv)
{

  QApplication app(argc, argv);
  Communication comm(argc,argv);
  MainWindow w(&comm);
  w.show();
  app.connect(&app, SIGNAL(lastWindowClosed()), &app, SLOT(quit()));
  int result = app.exec();

	return result;
}
