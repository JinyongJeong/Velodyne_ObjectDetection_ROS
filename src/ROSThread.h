#ifndef VIEWER_ROS_H
#define VIEWER_ROS_H

#include <fstream>
#include <iostream>
#include <QObject>
#include <QThread>
#include <QMutex>
#include <QPixmap>
#include <QVector>
#include <QVector3D>
#include <QDateTime>
#include <QReadLocker>
#include <QPainter>
#include <QLabel>
#include <algorithm>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

using namespace std;

struct Point3D {

  float x;
  float y;
  float z;
  Point3D(float x_, float y_, float z_)
    : x(x_), y(y_), z(z_) {

  }
};


class ROSThread : public QThread
{
    Q_OBJECT

public:
    explicit ROSThread(QObject *parent = 0, QMutex *th_mutex = 0);

    QMutex *mutex;

    std::vector<Point3D> _pc;

    ros::Subscriber sub;
    void ros_initialize(ros::NodeHandle &n);

    void run();
    void Velodyne_Callback(const sensor_msgs::PointCloud2ConstPtr& msg);

signals:
    void pc_callback();

private:
    //void run (void);

public slots:

};

#endif // VIEWER_LCM_H
