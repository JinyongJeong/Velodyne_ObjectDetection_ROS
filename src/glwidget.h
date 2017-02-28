#ifndef GLWIDGET_H
#define GLWIDGET_H

#include <QGLWidget>
#include <QGLShaderProgram>
#include <QGLShader>
#include <QTimer>
#include <QDebug>
#include <vector>
#include <QVector3D>
#include <QGLBuffer>
#include <QMouseEvent>
#include <QKeyEvent>
#include <QEvent>
#include <QtMath>
#include <QGLBuffer>
#include <vector>
#include <QGLFunctions>
#include <GL/gl.h>

#include <QWidget>
#include <QOpenGLWidget>
#include <QOpenGLShader>
#include <QOpenGLShaderProgram>
#include <QOpenGLFunctions>
#include <QOpenGLTexture>
#include <QOpenGLBuffer>
#include <QMouseEvent>
#include <QKeyEvent>

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include "ROSThread.h"

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/search/search.h>
#include <pcl/search/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/segmentation/region_growing_rgb.h>


#define grid_width 1.0

#define view_rotationo_speed 0.003  //initial rotation speed
#define voxel_size 0.1    //voxel size
#define voxel_color_R 0.9 //voxel color
#define voxel_color_G 0.9
#define voxel_color_B 0.9
#define voxel_line_color_R 0.6  //voxel line color
#define voxel_line_color_G 0.6
#define voxel_line_color_B 0.6
#define distance_threshold voxel_size*0.1 //distance threshold to check background or not

#define min_z -2.2  //initial minimun z value to show in viewer
#define max_z 1.8   //initial maximum z value to show in viewer

#define search_region_x 9.8 //initial serach x range
#define search_region_y 9.8 //initial search y range
#define search_region_z 2.2 //initial search z range

#define velodyne_height -min_z  //veldyne height, height of ground grid depend on this value

#define tracking_threshold_value 1.5

using namespace std;


class GLWidget : public QGLWidget
{
    Q_OBJECT

public:
    explicit GLWidget(QWidget *parent = 0);
    ~GLWidget();
    QSize sizeHint() const;
    QMutex mutex;
    ROSThread *my_ros;

protected:
    void initializeGL();
    void resizeGL(int width, int height);
    void paintGL();
    //! [2]
    void mousePressEvent(QMouseEvent *event);
    void mouseMoveEvent(QMouseEvent *event);
    void wheelEvent(QWheelEvent *event);
    void keyPressEvent(QKeyEvent *event);

    void drawShader (QGLShaderProgram &shader, GLenum type, QMatrix4x4 T, QVector<QVector3D> data, QColor color);
    void drawShader (QGLShaderProgram &shader, GLenum type, QMatrix4x4 T, QVector <QVector3D> data, Qt::GlobalColor color);
    void drawShader(QGLShaderProgram &shader, GLenum type, QMatrix4x4 T, QVector <GLfloat> data, QColor color, int stride);
    void drawShader (QGLShaderProgram &shader, GLenum type, QMatrix4x4 T, QVector <QVector3D> data, QVector <QVector3D> data_color);
    void drawShader(QGLShaderProgram &shader, GLenum type, QMatrix4x4 T, QVector<GLfloat> data, QVector<GLfloat> data_color, int stride);
    void drawVehicle(QOpenGLShaderProgram &shader, GLenum type, QMatrix4x4 T, QVector3D cur_translation, QVector3D cur_rotation, QColor color);

    void loadVehiclefile();
    void initial_voxel_gen();

public:

    int view_mode;
    int view_point_auto_change;
    int initial_point_flag;
    int object_detection_flag;
    int voxel_draw_flag;
    float change_angle_amount;
    int point_cut_flag;
    float height_max;
    float height_min;
    float search_region_X;
    float search_region_Y;
    float search_region_Z;
    float search_region_show_flag;

    ros::Subscriber sub;

    QVector <QVector3D> axes;
    QVector <QVector3D> axes_color;

    QVector <QVector3D> cpc;
    QVector <QVector3D> cpc_color;

    QVector <QVector3D> initial_cpc;
    QVector <QVector3D> initial_cpc_color;

    QVector <QVector3D> subsampled_initial_cpc;
    QVector <QVector3D> subsampled_initial_cpc_color;

    pcl::KdTreeFLANN<pcl::PointXYZRGB> kdtree;

    QVector <QVector3D> cpc_object;
    QVector <QVector3D> cpc_object_color;

    QVector <QVector3D> subsampled_initial_cpc_voxel;
    QVector <QVector3D> subsampled_initial_cpc_voxel_color;

    QVector <QVector3D> subsampled_initial_cpc_voxel_line;
    QVector <QVector3D> subsampled_initial_cpc_voxel_line_color;

    QMap<int, QVector<float>> pre_segment_center_info;
    QMap<int, QVector<int>> pre_segment_color_info;

    QMap<int, QVector<float>> new_segment_center_info;
    QMap<int, QVector<int>> new_segment_color_info;

    QVector <QVector3D> search_region;

    QVector <QVector3D> grid;

    QVector <QVector3D> init_cube;

    QVector3D current_translation;
    QVector3D current_rotation;    

    QGLShaderProgram shaderProgram;
    QGLShaderProgram shaderProgramColor;
    QOpenGLShaderProgram shaderProgram_obj;

    QOpenGLTexture *textures;

    QMatrix4x4 pMatrix;
    double alpha;
    double beta;
    double distance;
    QPoint lastMousePosition;
    double tr_x;
    double tr_y;
    float yaw;
    float pitch;

    GLuint texture;

    QVector3D cameraPosition;
    QVector3D cameraFront;
    QVector3D cameraUpDirection;

    QVector<QVector3D> vpose_trans;
    QVector<QVector3D> vpose_rot;

    QVector<QVector3D> vehicle_vertex;
    QVector<QVector2D> vehicle_texture;
    QVector<QVector3D> vehicle_normal;

    QVector<QVector3D> vehicle_face;
    QVector<QVector3D> vehicle_face_texture;
    QVector<QVector3D> vehicle_face_normal;

    QVector<QVector3D> vehicle_shape;
    QVector<QVector2D> vehicle_shape_texture;
    QVector<QVector3D> vehicle_shape_normal;

    QVector<QVector3D> vehicle_shape_global;   

    void ros_init(ros::NodeHandle &n);
    void search_region_vertex_recal();


private:
\
signals:
    void mouseEvent();

private:

    QTimer timer;
    QGLBuffer arrayBuf;


    QPoint lastPos;
private slots:
    void set_pc();
};

#endif // GLWIDGET_H
