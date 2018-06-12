#ifndef ros_gui_MAIN_WINDOW_H
#define ros_gui_MAIN_WINDOW_H

#include <QtGui/QMainWindow>
#include "ui_main_window.h"
#include "qnode.hpp"
#include <QTimer>
#include <QtGui>
#include <QMessageBox>
#include <iostream>

/*****************************************************************************
** Interface [MainWindow]
*****************************************************************************/

class MainWindow : public QMainWindow {
Q_OBJECT

public:
  MainWindow(QNode *node, QWidget *parent = 0);
	~MainWindow();

	void ReadSettings(); // Load up qt program settings at startup
	void WriteSettings(); // Save qt program settings when closing

	void closeEvent(QCloseEvent *event); // Overloaded function
	void showNoMasterMessage();


public Q_SLOTS:
	/******************************************
	** Auto-connections (connectSlotsByName())
	*******************************************/
	void on_actionAbout_triggered();
	void on_button_connect_clicked(bool check );
	void on_checkbox_use_environment_stateChanged(int state);
  void on_userButton_clicked();
  void on_oddButton_clicked();
  void on_evenButton_clicked();
  void timerEvent_1s();
  void timerEvent_1ms();


private:
	Ui::MainWindowDesign ui;
    QNode *qnode;
    QTimer *timer1s;
    QTimer *timer1ms;

};


#endif
