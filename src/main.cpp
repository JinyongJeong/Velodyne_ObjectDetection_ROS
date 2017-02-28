#include "mainwindow.h"
#include <QApplication>
#include <GL/glew.h>
#include <GL/gl.h>
#include <GL/glext.h>
#include <GL/glu.h>
#include <GL/glut.h>
#include "glwidget.h"
#include <ros/ros.h>

int main(int argc, char *argv[])
{
    glutInit(&argc, argv);
    glewInit();

    QApplication a(argc, argv);
//    MainWindow w;
//    w.show();

    ros::init(argc, argv, "labsviwer");
    ros::NodeHandle nh;

    GLWidget glwidget;
    glwidget.ros_init(nh);
    glwidget.resize(1280,720);
    glwidget.show();

    return a.exec();
}
