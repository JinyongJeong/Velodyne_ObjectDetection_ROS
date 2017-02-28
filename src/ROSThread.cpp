#include <QMutexLocker>

#include "ROSThread.h"



using namespace std;

ROSThread::ROSThread(QObject *parent, QMutex *th_mutex) :
    QThread(parent), mutex(th_mutex)
{


}

void ROSThread::ros_initialize(ros::NodeHandle &n)
{
    sub = n.subscribe<sensor_msgs::PointCloud2>("/velodyne_points", 2, boost::bind(&ROSThread::Velodyne_Callback, this, _1));
}

void ROSThread::run()
{
    ros::spin();
}

void ROSThread::Velodyne_Callback(const sensor_msgs::PointCloud2ConstPtr& msg)
{

//  cout << "Velodyne data is subscribed" << endl;

  int numPts = msg->height * msg->width;
  char* raw3DPtsData = (char*)(msg->data.data());
  std::vector<Point3D> pc(numPts, Point3D(0.,0.,0.));

  for (int i = 0; i < numPts; ++i) {

    float* base = (float*)(raw3DPtsData + i * msg->point_step);
    // float angle = atan2(base[1], base[0]) * 180. / M_PI;

    Point3D point3d(0.,0.,0.);

    point3d.x = base[0];
    point3d.y = base[1];
    point3d.z = base[2];

    //std::cout << point3d.x << " " << point3d.y << " " << point3d.z << std::endl;

    pc[i] = point3d;
  }

    mutex->lock();
    _pc = pc;
    mutex->unlock();

    emit pc_callback();
}
