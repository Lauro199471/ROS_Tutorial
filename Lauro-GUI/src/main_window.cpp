
#include <QtGui>
#include <QMessageBox>
#include <iostream>
#include "../include/main_window.hpp"
#include "../include/common_vars.hpp"

using namespace Qt;
int status = 0;
MainWindow::MainWindow(QNode *node, QWidget *parent) :
    QMainWindow(parent),
    qnode(node)
{
    ui.setupUi(this); // Calling this incidentally connects all ui's triggers to on_...() callbacks in this class.
    QObject::connect(ui.actionAbout_Qt, SIGNAL(triggered(bool)), qApp, SLOT(aboutQt())); // qApp is a global variable for the application

    ReadSettings();
    //setWindowIcon(QIcon(":/images/Xbox.png"));
    setWindowTitle("Boeing GUI");


    // -------------------------
    //        Autostart
    // -------------------------
    if (ui.checkbox_remember_settings->isChecked())
        on_button_connect_clicked(true);

    // -------------------------
    //        Timer
    // -------------------------

    //used to update count
    timer1s = new QTimer(this);
    connect(timer1s,SIGNAL(timeout()),this,SLOT(timerEvent_1s()));
    timer1s->start(1000);

    //used to update gui
    timer1ms = new QTimer(this);
    connect(timer1ms,SIGNAL(timeout()),this,SLOT(timerEvent_1ms()));
    timer1ms->start(1);
}

MainWindow::~MainWindow() {}

/*
 *     Implementation [SLOTS]
 *
 *     Description: The slots below are used to implement the ROS
 *                  side menu used for the GUI
 */


void MainWindow::showNoMasterMessage() {
  QMessageBox msgBox;
  msgBox.setText("Couldn't find the ros master.");
  msgBox.exec();
    close();
}

/*
 * These triggers whenever the button is clicked,
 * regardless of whether it is already checked or not.
 *
 */

void MainWindow::on_button_connect_clicked(bool check ) {
  if ( ui.checkbox_use_environment->isChecked() ) {
    if ( !qnode->on_init() ) {
      showNoMasterMessage();
    } else {
      ui.button_connect->setEnabled(false);
    }
  } else {
    if ( ! qnode->on_init(
          ui.line_edit_master->text().toStdString(),
          ui.line_edit_host->text().toStdString() )
        ) {
      showNoMasterMessage();
    } else {
      ui.button_connect->setEnabled(false);
      ui.line_edit_master->setReadOnly(true);
      ui.line_edit_host->setReadOnly(true);
    }
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
    restoreGeometry(settings.value("geometry").toByteArray());
    restoreState(settings.value("windowState").toByteArray());
    QString master_url = settings.value("master_url",QString("http://192.168.1.2:11311/")).toString();
    QString host_url = settings.value("host_url", QString("192.168.1.3")).toString();
    QString topic_name = settings.value("topic_name", QString("/chatter")).toString();
    ui.line_edit_master->setText(master_url);
    ui.line_edit_host->setText(host_url);
    bool remember = settings.value("remember_settings", false).toBool();
    ui.checkbox_remember_settings->setChecked(remember);
    bool checked = settings.value("use_environment_variables", false).toBool();
    ui.checkbox_use_environment->setChecked(checked);
    if ( checked ) {
      ui.line_edit_master->setEnabled(false);
      ui.line_edit_host->setEnabled(false);
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

