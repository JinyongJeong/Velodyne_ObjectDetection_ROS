#include "mainwindow.h"
#include "ui_mainwindow.h"

using namespace std;

MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    ui->setupUi(this);
    cout << "Start window" << endl;

}

MainWindow::~MainWindow()
{
    emit setThreadFinished(true); //Tell the thread to finish
    delete ui;
}

void MainWindow::cpc_copy(QVector<QVector3D> &des, QVector<QVector3D> src)
{
    for (int i=0; i<src.size(); i++)
        des.push_back(src[i]);
}

