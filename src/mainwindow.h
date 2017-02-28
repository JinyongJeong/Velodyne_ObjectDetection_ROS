#ifndef MAINWINDOW_H
#define MAINWINDOW_H
#include <iostream>
#include <QMainWindow>
#include <QThread>
#include <QVector>
#include <QMutex>
#include <QDateTime>
#include <QDoubleSpinBox>


#define R2D 180/PI
#define D2R PI/180

extern QMutex mutex;

namespace Ui {
class MainWindow;
}

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(QWidget *parent = 0);
    ~MainWindow();
    QMutex mutex;

    void cpc_copy (QVector <QVector3D> &des, QVector <QVector3D> src);


signals:
    void setThreadFinished(bool);

private:
    Ui::MainWindow *ui;
};

#endif // MAINWINDOW_H
