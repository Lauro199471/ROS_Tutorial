/*****************************************************************************
** Includes
*****************************************************************************/

#include <QtGui>
#include <QMessageBox>
#include <iostream>
#include "../include/main_window.hpp"
#include "../include/listner.hpp"
#include <string>

using namespace Qt;

/*****************************************************************************
** Implementation [MainWindow]
*****************************************************************************/

MainWindow::MainWindow(QNode *node, QWidget *parent) :
    QMainWindow(parent),
    qnode(node)
{
    ui.setupUi(this); // Calling this incidentally connects all ui's triggers to on_...() callbacks in this class.
    QObject::connect(ui.actionAbout_Qt, SIGNAL(triggered(bool)), qApp, SLOT(aboutQt())); // qApp is a global variable for the application

    ReadSettings();
    //setWindowIcon(QIcon(":/images/Xbox.png"));
    setWindowTitle("My Title");

    /*********************
    ** Logging
    **********************/
    QObject::connect(qnode, SIGNAL(loggingUpdated()), this, SLOT(updateLoggingView()));
    QObject::connect(qnode, SIGNAL(rosShutdown()), this, SLOT(close()));

    /*********************
    ** Auto Start
    **********************/
    //if (ui.checkbox_remember_settings->isChecked())
    //    on_button_connect_clicked(true);

    /*********************
     ** Timer
     **********************/
     timer = new QTimer(this);
     connect(timer,SIGNAL(timeout()),this,SLOT(timerEvent()));
     timer->start(1);

}

MainWindow::~MainWindow() {}

/*****************************************************************************
** Implementation [Slots]
*****************************************************************************/

void MainWindow::showNoMasterMessage() {
  QMessageBox msgBox;
  msgBox.setText("Couldn't find the ros master.");
  msgBox.exec();
    close();
}


void MainWindow::show_NO_ENVIROMENTAL_Message() {
  QMessageBox msgBox;
  msgBox.setText("Couldn't start without IP Values.");
  msgBox.exec();
  close();
}

/*
 * These triggers whenever the button is clicked, regardless of whether it
 * is already checked or not.
 */

void MainWindow::on_button_connect_clicked(bool check )
{
  std::string MasterIP = "http://" + ui.line_edit_master->text().toStdString() + ":11311";

  // If Enviromental Button is Check , then Connect Node to ROSCORE
  if ( ui.checkbox_use_environment->isChecked() )
  {
    if ( ! qnode->on_init(MasterIP,ui.line_edit_host->text().toStdString() ) )
    {
      showNoMasterMessage();
    }
    else
    {
      ui.button_connect->setEnabled(false);
      ui.line_edit_master->setReadOnly(true);
      ui.line_edit_host->setReadOnly(true);
    }
  }

  // else display error for not locking IP values
  else
  {
    show_NO_ENVIROMENTAL_Message();
  }
}

void MainWindow::on_checkbox_use_environment_stateChanged(int state) {
  bool enabled;
  if ( state == 0 ) {
    enabled = true;
  } else {
    enabled = false;
  }
  ui.line_edit_master->setEnabled(enabled);
  ui.line_edit_host->setEnabled(enabled);
}

/*****************************************************************************
** Implementation [Menu]
*****************************************************************************/

void MainWindow::on_actionAbout_triggered() {
    QMessageBox::about(this, tr("About ..."),tr("<h2>PACKAGE_NAME Test Program 0.10</h2><p>Copyright Yujin Robot</p><p>This package needs an about description.</p>"));
}

/*****************************************************************************
** Implementation [Configuration]
*****************************************************************************/

void MainWindow::ReadSettings() {
    QSettings settings("Qt-Ros Package", qnode->nodeName().c_str());

    // Find QSettings File location
    qDebug() << settings.fileName();
    //

    restoreGeometry(settings.value("geometry").toByteArray());
    restoreState(settings.value("windowState").toByteArray());

    bool remember = settings.value("remember_settings").toBool();
    ui.checkbox_remember_settings->setChecked(remember); // display checkbox on GUI

    bool checked = settings.value("use_environment_variables").toBool();
    ui.checkbox_use_environment->setChecked(checked);
    ui.line_edit_master->setEnabled(!checked);
    ui.line_edit_host->setEnabled(!checked);

    // If rememnber is check , dont change values
    if(remember)
    {

      // Master
      QString master_url = settings.value("master_url").toString();
      ui.line_edit_master->setText(master_url);
      // Host
      QString host_url = settings.value("host_url").toString();
      ui.line_edit_host->setText(host_url);
      // Use Enviroment Button

    }
    // If not check , then change values
    else
    {
      // Master
      settings.setValue("master_url","http://{PUT MASTER IP}:11311/");
      ui.line_edit_master->setText("http://{PUT MASTER IP}:11311/");
      //Host
      settings.setValue("host_url","{PUT HOST IP}");
      ui.line_edit_host->setText("{PUT HOST IP}");
      //ui.line_edit_master->setEnabled(false);
      //ui.line_edit_host->setEnabled(false);
    }
}

void MainWindow::WriteSettings() {
    QSettings settings("Qt-Ros Package", qnode->nodeName().c_str());
    settings.setValue("geometry", geometry());
    settings.setValue("master_url",ui.line_edit_master->text());
    settings.setValue("host_url",ui.line_edit_host->text());
    settings.setValue("use_environment_variables",QVariant(ui.checkbox_use_environment->isChecked()));
    settings.setValue("windowState", saveState());
    settings.setValue("remember_settings",QVariant(ui.checkbox_remember_settings->isChecked()));
}

void MainWindow::closeEvent(QCloseEvent *event)
{
  qnode->shutdown();
  WriteSettings();
  QMainWindow::closeEvent(event);
}

