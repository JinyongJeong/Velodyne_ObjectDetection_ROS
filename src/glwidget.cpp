#include <iostream>
#include "glwidget.h"
#include <GL/glut.h>

using namespace std;

GLWidget::GLWidget(QWidget *parent)
    : QGLWidget(parent)
{
    my_ros = new ROSThread(this, &mutex);

    connect(my_ros, SIGNAL(pc_callback()), this, SLOT(set_pc()));

    view_point_auto_change = 0;
    initial_point_flag = 0;
    object_detection_flag = 0;
    voxel_draw_flag = 0;
    point_cut_flag = 0;
    search_region_show_flag = 0;
    change_angle_amount = view_rotationo_speed;
    height_max = max_z;
    height_min = min_z;
    search_region_X = search_region_x;
    search_region_Y = search_region_y;
    search_region_Z = search_region_z;

    pre_segment_center_info.clear();
    pre_segment_color_info.clear();

    new_segment_center_info.clear();
    new_segment_color_info.clear();

    search_region.clear();
    cpc.clear();
    cpc_color.clear();
    initial_cpc.clear();
    initial_cpc_color.clear();
    subsampled_initial_cpc.clear();
    subsampled_initial_cpc_color.clear();

    alpha = 0;
    beta = 0;
    distance = 2.5;
    tr_x = 0;
    tr_y = 0;
    yaw = -90.f;
    pitch = 0;
    cameraPosition = QVector3D(0, 0, 25);
    cameraFront = QVector3D(0, 0, -1);
    cameraUpDirection = QVector3D(0, 1, 0);

    setFocusPolicy(Qt::StrongFocus);

    vpose_trans.push_back(QVector3D(0, 0, 0));
    vpose_rot.push_back(QVector3D(0, 0, 0));

    init_cube.clear();

    float a = 0.1;
    float b = 0.07;
    float c = 0.07;
    init_cube << QVector3D(a, -b, c) << QVector3D(a, -b, -c) << QVector3D(a, b, -c)
              << QVector3D(a, b, -c) << QVector3D(a, b, c) << QVector3D(a, -b, c) // 1
              << QVector3D(-a, -b, c) << QVector3D(-a, -b, -c) << QVector3D(a, -b, -c)
              << QVector3D(a, -b, -c) << QVector3D(a, -b, c) << QVector3D(-a, -b, c) // 2
              << QVector3D(a, b, c) << QVector3D(a, b, -c) << QVector3D(-a, b, -c)
              << QVector3D(-a, b, -c) << QVector3D(-a, b, c) << QVector3D(a, b, c) //3
              << QVector3D(-a, -b, c) << QVector3D(a, -b, c) << QVector3D(a, b, c)
              << QVector3D(a, b, c) << QVector3D(-a, b, c) << QVector3D(-a, -b, c) // 4
              << QVector3D(-a, -b, -c) << QVector3D(a, -b, -c) << QVector3D(a, b, -c)
              << QVector3D(a, b, -c) << QVector3D(-a, b, -c) << QVector3D(-a, -b, -c) // 5
              << QVector3D(-a, -b, c) << QVector3D(-a, -b, -c) << QVector3D(-a, b, -c)
              << QVector3D(-a, b, -c) << QVector3D(-a, b, c) << QVector3D(-a, -b, c) //6;
              << QVector3D(a, -b, c) << QVector3D(a, -b, -c) << QVector3D(a, b, -c);
   
    vehicle_vertex.clear();
    vehicle_texture.clear();
    vehicle_normal.clear();
    vehicle_face.clear();
    vehicle_face_texture.clear();
    vehicle_face_normal.clear();
    vehicle_shape.clear();
    vehicle_shape_texture.clear();
    vehicle_shape_global.clear();

    loadVehiclefile();

    my_ros->start();
//    my_ros->run();
}

GLWidget::~GLWidget()
{

}

QSize GLWidget::sizeHint() const
{
    return QSize(640, 480);
}

void GLWidget::initializeGL()
{
    glEnable(GL_DEPTH_TEST);

    qglClearColor(QColor(Qt::black));

    shaderProgram.addShaderFromSourceFile(QGLShader::Vertex, ":resources/vertexShader.vsh");
    shaderProgram.addShaderFromSourceFile(QGLShader::Fragment, ":resources/fragmentShader.fsh");
    shaderProgram.link();

    shaderProgramColor.addShaderFromSourceFile(QGLShader::Vertex, ":resources/vertexShaderColor.vsh");
    shaderProgramColor.addShaderFromSourceFile(QGLShader::Fragment, ":resources/fragmentShaderColor.fsh");
    shaderProgramColor.link();

    shaderProgram_obj.addShaderFromSourceFile(QOpenGLShader::Vertex, ":resources/vertexShaderOBJ.vsh"); 
    shaderProgram_obj.addShaderFromSourceFile(QOpenGLShader::Fragment, ":resources/fragmentShaderOBJ.fsh");
    shaderProgram_obj.link();
    
    textures =  new QOpenGLTexture(QImage(QString("./src/labs_viewer/data/Lexus.png")));

    axes << QVector3D(0, 0, 0) << QVector3D(1, 0, 0)
         << QVector3D(0, 0, 0) << QVector3D(0, 1, 0)
         << QVector3D(0, 0, 0) << QVector3D(0, 0, 1);
    axes_color << QVector3D(1, 0, 0) << QVector3D(1, 0, 0)
               << QVector3D(0, 1, 0) << QVector3D(0, 1, 0)
               << QVector3D(0, 0, 1) << QVector3D(0, 0, 1);

    float max_size = 100;
    for (float i=0; i<=max_size; i = i+ grid_width) {
        grid.push_back(QVector3D(-max_size, i, -velodyne_height)); grid.push_back(QVector3D(max_size, i, -velodyne_height));
        grid.push_back(QVector3D(-max_size, -i, -velodyne_height)); grid.push_back(QVector3D(max_size, -i, -velodyne_height));
        grid.push_back(QVector3D(i, -max_size, -velodyne_height)); grid.push_back(QVector3D(i, max_size, -velodyne_height));
        grid.push_back(QVector3D(-i, -max_size, -velodyne_height)); grid.push_back(QVector3D(-i, max_size, -velodyne_height));
    }

}

void GLWidget::resizeGL(int width, int height)
{
    if (height == 0) {
        height = 1;
    }

    pMatrix.setToIdentity();
    pMatrix.perspective(60.0, (float) width / (float) height, 0.001, 1000);

    glViewport(0, 0, width, height);
    glMatrixMode(GL_PROJECTION);   //<--- add
    glLoadIdentity();              //<--- add
}

void GLWidget::paintGL()
{
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    QMatrix4x4 mMatrix;
    QMatrix4x4 vMatrix;

    mMatrix.setToIdentity();

    vMatrix.lookAt(cameraPosition, cameraPosition+cameraFront, cameraUpDirection);
    QMatrix4x4 T = pMatrix * vMatrix * mMatrix;

    if(point_cut_flag == 1){
      //Point filter using Z_max and Z_min
      QVector<QVector3D>::iterator iter2 = cpc_color.begin();
      for(QVector<QVector3D>::iterator iter = cpc.begin(); iter != cpc.end();++iter){
        if(iter->z() > height_max || iter->z() < height_min){
          iter2->setX(0);
          iter2->setY(0);
          iter2->setZ(0);
        }

        ++iter2;
      }
    }

    glLineWidth(1);
    drawShader(shaderProgram, GL_LINES, T, grid, QColor(50,50,50));
    glLineWidth(3);
    drawShader(shaderProgramColor, GL_LINES, T, axes, axes_color);
    glPointSize(1);   

    if(initial_point_flag == 1 && voxel_draw_flag == 1){
      glEnable(GL_BLEND);
      glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
      glEnable(GL_LINE_SMOOTH);
      glHint(GL_LINE_SMOOTH_HINT, GL_NICEST);
      drawShader(shaderProgramColor, GL_TRIANGLES,T,subsampled_initial_cpc_voxel,subsampled_initial_cpc_voxel_color);
      drawShader(shaderProgramColor, GL_LINES,T,subsampled_initial_cpc_voxel_line,subsampled_initial_cpc_voxel_line_color);

    }

    if(search_region_show_flag == 1){
      glLineWidth(2);
      drawShader(shaderProgram, GL_LINES, T, search_region, QColor(255,0,0));
    }


    if(object_detection_flag == 1){

      glPointSize(3);
      drawShader(shaderProgramColor, GL_POINTS,T,cpc_object,cpc_object_color);
      glPointSize(1);
      drawShader(shaderProgramColor, GL_POINTS, T, cpc, cpc_color);
    }else{
      drawShader(shaderProgramColor, GL_POINTS, T, cpc, cpc_color);
    }


//    drawVehicle(shaderProgram_obj, GL_TRIANGLES, T, current_translation, current_rotation, QColor(240,240,240));

    glFlush();
}

void GLWidget::drawShader(QGLShaderProgram &shader, GLenum type, QMatrix4x4 T, QVector<QVector3D> data, QColor color)
{
    shader.bind();
    shader.setUniformValue("color", color);
    shader.setAttributeArray("vertex", data.constData());
    shader.enableAttributeArray("vertex");
    glDrawArrays(type, 0, data.size());
    shader.disableAttributeArray("vertex");
    shader.setUniformValue("mvpMatrix", T);
    shader.release();
}

void GLWidget::drawShader(QGLShaderProgram &shader, GLenum type, QMatrix4x4 T, QVector<QVector3D> data, Qt::GlobalColor color)
{
    shader.bind();
    shader.setUniformValue("color", QColor(color));
    shader.setAttributeArray("vertex", data.constData());
    shader.enableAttributeArray("vertex");
    glDrawArrays(type, 0, data.size());
    shader.disableAttributeArray("vertex");
    shader.setUniformValue("mvpMatrix", T);
    shader.release();
}

void GLWidget::drawShader(QGLShaderProgram &shader, GLenum type, QMatrix4x4 T, QVector <GLfloat> data, QVector <GLfloat> data_color, int stride)
{
    shader.bind();
    shader.setAttributeArray("vertex", data.constData(), 3, 3*stride*sizeof(GLfloat));
    shader.enableAttributeArray("vertex");
    shader.setAttributeArray("color", data_color.constData(), 3, 3*stride*sizeof(GLfloat));
    shader.enableAttributeArray("color");
    if (stride==0) stride = 1;
    glDrawArrays(type, 0, data.size()/(3*stride));
    shader.disableAttributeArray("color");
    shader.disableAttributeArray("vertex");
    shader.setUniformValue("mvpMatrix", T);
    shader.release();
}
void GLWidget::drawShader(QGLShaderProgram &shader, GLenum type, QMatrix4x4 T, QVector <GLfloat> data, QColor color, int stride)
{
    shader.bind();
    shader.setUniformValue("color", color);
    shader.setAttributeArray("vertex", data.constData(), 3, 3*stride*sizeof(GLfloat));
    shader.enableAttributeArray("vertex");
    // shader.setAttributeArray("color", data_color.constData(), 3, 3*stride*sizeof(GLfloat));
    // shader.enableAttributeArray("color");
    if (stride==0) stride = 1;
    glDrawArrays(type, 0, data.size()/(3*stride));
    // shader.disableAttributeArray("color");
    shader.disableAttributeArray("vertex");
    shader.setUniformValue("mvpMatrix", T);
    shader.release();
}
void GLWidget::drawShader(QGLShaderProgram &shader, GLenum type, QMatrix4x4 T, QVector<QVector3D> data, QVector<QVector3D> data_color)
{
    shader.bind();
    shader.setAttributeArray("vertex", data.constData(), sizeof(QVector3D));
    shader.enableAttributeArray("vertex");
    shader.setAttributeArray("color", data_color.constData(), sizeof(QVector3D));
    shader.enableAttributeArray("color");
    glDrawArrays(type, 0, data.size());
    shader.disableAttributeArray("color");
    shader.disableAttributeArray("vertex");
    shader.setUniformValue("mvpMatrix", T);
    shader.release();
}

void GLWidget::drawVehicle(QOpenGLShaderProgram &shader, GLenum type, QMatrix4x4 T, QVector3D cur_translation, QVector3D cur_rotation, QColor color)
{
    vehicle_shape_global.clear();
    QMatrix4x4 tf;
    tf.setToIdentity();
    tf.rotate(qRadiansToDegrees(cur_rotation.x()), 1,0,0);
    tf.rotate(qRadiansToDegrees(cur_rotation.y()), 0,1,0);
    tf.rotate(qRadiansToDegrees(cur_rotation.z())+180, 0,0,1);
    for (int i=0; i<vehicle_shape.size(); i++) {
        vehicle_shape_global.push_back(tf*vehicle_shape[i]+cur_translation);
    }

    shader.bind();
    // shader.setUniformValue("color", QColor(color));
    shader.setUniformValue("mvpMatrix", T);
    shader.setAttributeArray("vertex", vehicle_shape_global.constData());
    shader.enableAttributeArray("vertex");

    shader.setAttributeArray("textureCoordinate", vehicle_shape_texture.constData());
    shader.enableAttributeArray("textureCoordinate");

    shader.setUniformValue("texture", 0);
    textures->bind();
    glFrontFace(GL_CW);
    glDrawArrays(type, 0, vehicle_shape_global.size());
    shader.disableAttributeArray("vertex");
    shader.disableAttributeArray("textureCoordinate");
    shader.release();

}


void GLWidget::mousePressEvent(QMouseEvent *event)
{
    lastMousePosition = event->pos();
    // cout << "mouse click event" << endl;
    event->accept();

    glPushMatrix();
    glBegin(GL_POINT);
    glColor3d(1, 1, 0);
    glVertex3f(0, 0, 0);
    glEnd();
    glPopMatrix();
}



void GLWidget::mouseMoveEvent(QMouseEvent *event)
{
    if(view_point_auto_change == 1) return;
    // cout << "mouse event" << endl;
    float deltaX = event->x() - lastMousePosition.x();
    float deltaY = lastMousePosition.y() - event->y();
    float sensitivity = 0.1;

    float alpha;
    QVector3D pt_xy;
    QVector3D tmp_cam_pos;

    if (abs(cameraFront.z()) > 0.000005) {
        alpha = -cameraPosition.z() / cameraFront.z();
        pt_xy = cameraPosition + alpha*cameraFront;
    }
    else {
        pt_xy = cameraPosition;
    }

    tmp_cam_pos = cameraPosition - pt_xy;

    // cout << "cam front " << cameraFront.x() << " / " << cameraFront.y() << " / " << cameraFront.z() << endl;
    // cout << "cam up " << cameraUpDirection.x() << " / " << cameraUpDirection.y() << " / " << cameraUpDirection.z() << endl;
    // cout << "cam pose " << cameraPosition.x() << " / " << cameraPosition.y() << " / " << cameraPosition.z() << endl;
    // cout << "inte pt_xy " << pt_xy.x() << " / " << pt_xy.y() << " / " << pt_xy.z() << endl;

    if (event->buttons() & Qt::LeftButton) { // translation motion
        QVector3D left = QVector3D::normal(cameraUpDirection, cameraFront);
        QVector3D z_fix_front = QVector3D(cameraUpDirection[0], cameraUpDirection[1], 0).normalized();
        cameraPosition = cameraPosition - (cameraPosition.z()*0.02*sensitivity)*deltaY*z_fix_front;
        cameraPosition = cameraPosition + (cameraPosition.z()*0.02*sensitivity)*deltaX*left;
        update();
    }
    if (event->buttons() & Qt::RightButton) {

        QMatrix4x4 rotation;
        rotation.setToIdentity();

        QVector3D right = QVector3D::normal(cameraFront, cameraUpDirection);

        deltaX *= sensitivity;
        deltaY *= sensitivity;

        yaw += deltaX;
        pitch += deltaY;

        if(pitch > 89.9f)
            pitch = 89.9f;
        if(pitch < -89.9f)
            pitch = -89.9f;

        rotation.rotate(deltaY, right);
        rotation.rotate(-deltaX, 0, 0, 1);


        cameraPosition = rotation*tmp_cam_pos + pt_xy;
        cameraFront = rotation*cameraFront;
        cameraUpDirection = rotation*cameraUpDirection;

        update();
    }

    lastMousePosition = event->pos();
    emit mouseEvent();

    event->accept();
}

void GLWidget::wheelEvent(QWheelEvent *event)
{
    if(view_point_auto_change == 1) return;
    int delta = (float)event->delta();

    if (event->orientation() == Qt::Vertical) {
        if (delta < 0) {
            cameraPosition -= 0.3 * cameraFront;
        } else if (delta > 0) {
            cameraPosition += 0.3 * cameraFront;
        }
        update();
    }
    emit mouseEvent();

    event->accept();

}

void GLWidget::keyPressEvent(QKeyEvent *event)
{
    float cameraSpeed = 0.2f;
    float adjust_height_rate = 0.1;
    float adjust_search_bound_rate = 0.1;
    QVector3D z_fix_front = QVector3D(cameraUpDirection[0], cameraUpDirection[1], 0).normalized();
    if (event->key() == Qt::Key_1) {
        cout << "1 Key Pressed" << endl;
        height_min = height_min - adjust_height_rate;
        cout << "height_min: " << height_min << " /height_max: " << height_max << endl;

    }
    if (event->key() == Qt::Key_2) {
        cout << "2 Key Pressed" << endl;
        height_min = height_min + adjust_height_rate;
        cout << "height_min: " << height_min << " /height_max: " << height_max << endl;
    }
    if (event->key() == Qt::Key_3) {
        cout << "3 Key Pressed" << endl;
        height_max = height_max - adjust_height_rate;
        cout << "height_min: " << height_min << " /height_max: " << height_max << endl;
    }
    if (event->key() == Qt::Key_4) {
        cout << "4 Key Pressed" << endl;
        height_max = height_max + adjust_height_rate;
        cout << "height_min: " << height_min << " /height_max: " << height_max << endl;
    }
    if (event->key() == Qt::Key_5) {
        cout << "5 Key Pressed" << endl;
        search_region_X = search_region_X - adjust_search_bound_rate;
        cout << "X bound : " << search_region_X << " /Y bound: " << search_region_Y << " /Z bound: " << search_region_Z <<endl;
        search_region_vertex_recal();
    }
    if (event->key() == Qt::Key_6) {
        cout << "6 Key Pressed" << endl;
        search_region_X = search_region_X + adjust_search_bound_rate;
        cout << "X bound : " << search_region_X << " /Y bound: " << search_region_Y << " /Z bound: " << search_region_Z <<endl;
        search_region_vertex_recal();
    }
    if (event->key() == Qt::Key_7) {
        cout << "7 Key Pressed" << endl;
        search_region_Y = search_region_Y - adjust_search_bound_rate;
        cout << "X bound : " << search_region_X << " /Y bound: " << search_region_Y << " /Z bound: " << search_region_Z <<endl;
        search_region_vertex_recal();
    }
    if (event->key() == Qt::Key_8) {
        cout << "8 Key Pressed" << endl;
        search_region_Y = search_region_Y + adjust_search_bound_rate;
        cout << "X bound : " << search_region_X << " /Y bound: " << search_region_Y << " /Z bound: " << search_region_Z <<endl;
        search_region_vertex_recal();
    }
    if (event->key() == Qt::Key_9) {
        cout << "9 Key Pressed" << endl;
        search_region_Z = search_region_Z - adjust_search_bound_rate;
        cout << "X bound : " << search_region_X << " /Y bound: " << search_region_Y << " /Z bound: " << search_region_Z <<endl;
        search_region_vertex_recal();
    }
    if (event->key() == Qt::Key_0) {
        cout << "0 Key Pressed" << endl;
        search_region_Z = search_region_Z + adjust_search_bound_rate;
        cout << "X bound : " << search_region_X << " /Y bound: " << search_region_Y << " /Z bound: " << search_region_Z <<endl;
        search_region_vertex_recal();
    }


    if (event->key() == Qt::Key_R) {
        cout << "R Key Pressed" << endl;
        if(search_region_show_flag == 0){
          search_region_show_flag = 1;
          search_region_vertex_recal();
        }else{
          search_region_show_flag = 0;
        }
    }


    if (event->key() == Qt::Key_W) {
        cout << "W Key Pressed" << endl;
          //cameraPosition += cameraSpeed * z_fix_front;
    }

    if (event->key() == Qt::Key_A) {
        cout << "A Key Pressed" << endl;
        if(view_point_auto_change == 1){
          view_point_auto_change = 0;
        }else if(view_point_auto_change == 0){
          view_point_auto_change = 1;
        }
    }

    if (event->key() == Qt::Key_Q) {
        cout << "Q Key Pressed" << endl;
        cout << "Program is finishing.. " << endl;
        exit(1);
    }

    if (event->key() == Qt::Key_I) {
        cout << "I Key Pressed" << endl;
        if(initial_point_flag == 0){
          cout << "Initial Point is catching... " << endl;
          for(int i = 0 ; i < (int)cpc.size(); i ++){
            initial_cpc.push_back(cpc[i]);
            initial_cpc_color.push_back(cpc_color[i]);
          }
          cout << "Initial Point is catched. " << endl;
          initial_point_flag = 1;

          pcl::PointCloud <pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud <pcl::PointXYZRGB>);
          pcl::PointCloud <pcl::PointXYZRGB>::Ptr cloud_downsampled (new pcl::PointCloud <pcl::PointXYZRGB>);

          pcl::PointXYZRGB point;
          for(int i = 0 ; (int)i < initial_cpc.size(); i ++){
            point.x = initial_cpc[i].x();
            point.y = initial_cpc[i].y();
            point.z = initial_cpc[i].z();
            point.r = initial_cpc_color[i].x();
            point.g = initial_cpc_color[i].y();
            point.b = initial_cpc_color[i].z();
            cloud->push_back(point);
          }

          pcl::VoxelGrid<pcl::PointXYZRGB> voxel_grid;
          voxel_grid.setInputCloud (cloud);
          voxel_grid.setLeafSize (voxel_size,voxel_size,voxel_size);
          voxel_grid.filter (* cloud_downsampled);

          //kdtree generation
          kdtree.setInputCloud (cloud_downsampled);
          QVector3D tmp;
          for(int i = 0 ; (int)i < cloud_downsampled->size(); i ++){
            tmp.setX(cloud_downsampled->points[i].x);
            tmp.setY(cloud_downsampled->points[i].y);
            tmp.setZ(cloud_downsampled->points[i].z);
            subsampled_initial_cpc.push_back(tmp);
            tmp.setX(cloud_downsampled->points[i].r);
            tmp.setY(cloud_downsampled->points[i].g);
            tmp.setZ(cloud_downsampled->points[i].b);
            subsampled_initial_cpc_color.push_back(tmp);
          }
          initial_voxel_gen();
        }
    }

    if (event->key() == Qt::Key_S) {
        cout << "S Key Pressed" << endl;
        if(object_detection_flag == 0){
          cout << "Object Detection start " << endl;
          object_detection_flag = 1;
        }else if(object_detection_flag == 1){
          cout << "Object Detection finish " << endl;
          object_detection_flag = 0;
        }
    }

    if (event->key() == Qt::Key_F) {
        cout << "F Key pressed " << endl;
        if(isFullScreen()) {
          this->setWindowState(Qt::WindowMaximized);
          cout << "Set normal screen size" << endl;
        } else {
          this->setWindowState(Qt::WindowFullScreen);
          cout << "Set FullScreen" << endl;
        } 
        this->show();
    }

    if (event->key() == Qt::Key_U) {
      cout << "U: Rotation Speed up" << endl;
      change_angle_amount = change_angle_amount + 0.001;
    }
    if (event->key() == Qt::Key_C) {
      cout << "C Key pressed" << endl;
      if(point_cut_flag == 0){
        point_cut_flag = 1;
      }else{
        point_cut_flag = 0;
      }
    }


    if (event->key() == Qt::Key_D) {
      cout << "D: Rotation Speed Down" << endl;
      change_angle_amount = change_angle_amount - 0.001;
    }
    if (event->key() == Qt::Key_V) {
      cout << "V Key pressed" << endl;
      if(voxel_draw_flag == 1){
        voxel_draw_flag = 0;
      }else if(voxel_draw_flag == 0){
        voxel_draw_flag = 1;
      }
    }

    update();
    event->accept();
}

void GLWidget::loadVehiclefile(){


    FILE *file = fopen("./src/labs_viewer/data/lexus_hs.obj","r");
    float scale = 0.08;
    if(file == NULL){
        printf("fail to read obj file\n");
        return;
    }
    bool flag = true;
    while(1){
        char lineHeader[128];
        int res = fscanf(file,"%s",lineHeader);
        if(res == EOF) break;

        if(strcmp(lineHeader,"v") == 0){
            QVector3D vertex;
            float data[3];
            fscanf(file,"%f %f %f\n",&data[0],&data[1],&data[2]);
            vertex.setX(data[0]*scale);
            vertex.setY(data[1]*scale);
            vertex.setZ(data[2]*scale);
            
            vehicle_vertex.push_back(vertex);
        }else if(strcmp(lineHeader,"vn") == 0){
            QVector3D normal;
            float data[3];
            fscanf(file,"%f %f %f\n",&data[0],&data[1],&data[2]);
            normal.setX(data[0]);
            normal.setY(data[1]);
            normal.setZ(data[2]);
            vehicle_normal.push_back(normal);

        }else if(strcmp(lineHeader,"vt") == 0){
            QVector2D texture;
            float data[3];
            fscanf(file,"%f %f %f\n",&data[0],&data[1],&data[2]);
            texture.setX(data[0]);
            texture.setY(1-data[1]);
            //texture.setZ(data[2]);
            vehicle_texture.push_back(texture);

        }else if(strcmp(lineHeader,"f") == 0){
            QVector3D face;
            QVector3D texture;
            QVector3D normal;
            flag = true;
            int data[12];
            int return_v = fscanf(file,"%d/%d/%d %d/%d/%d %d/%d/%d %d/%d/%d\n",&data[0],&data[1],&data[2]
                ,&data[3],&data[4],&data[5],&data[6],&data[7],&data[8],&data[9],&data[10],&data[11]);
            if(return_v == 9&&flag == true){
                face.setX(data[0]);
                face.setY(data[3]);
                face.setZ(data[6]);
                texture.setX(data[1]);
                texture.setY(data[4]);
                texture.setZ(data[7]);
                normal.setX(data[2]);
                normal.setY(data[5]);
                normal.setZ(data[8]);
                
                vehicle_face.push_back(face);
                vehicle_face_texture.push_back(texture);
                vehicle_face_normal.push_back(normal);
            }
            if(return_v == 12&&flag == true){
                face.setX(data[0]);
                face.setY(data[3]);
                face.setZ(data[6]);
                texture.setX(data[1]);
                texture.setY(data[4]);
                texture.setZ(data[7]);
                normal.setX(data[2]);
                normal.setY(data[5]);
                normal.setZ(data[8]);
                
                vehicle_face.push_back(face);
                vehicle_face_texture.push_back(texture);
                vehicle_face_normal.push_back(normal);
                face.setX(data[6]);
                face.setY(data[9]);
                face.setZ(data[0]);
                texture.setX(data[7]);
                texture.setY(data[10]);
                texture.setZ(data[1]);
                normal.setX(data[8]);
                normal.setY(data[11]);
                normal.setZ(data[2]);
                vehicle_face.push_back(face);
                vehicle_face_texture.push_back(texture);
                vehicle_face_normal.push_back(normal);
            }

        }else if(strcmp(lineHeader,"s") == 0){
            int value;
            int return_v = fscanf(file,"%d\n",&value);
            if(value == 1){
                flag = true;
            }else{
                flag = false;
            }

        }else{


        }
        //set vehicle node
        for(int i = 0 ; i < vehicle_face.size(); i ++){
            vehicle_shape.push_back(vehicle_vertex[vehicle_face[i].x()-1]);
            vehicle_shape.push_back(vehicle_vertex[vehicle_face[i].y()-1]);
            vehicle_shape.push_back(vehicle_vertex[vehicle_face[i].z()-1]);
            vehicle_shape_texture.push_back(vehicle_texture[vehicle_face_texture[i].x()-1]);
            vehicle_shape_texture.push_back(vehicle_texture[vehicle_face_texture[i].y()-1]);
            vehicle_shape_texture.push_back(vehicle_texture[vehicle_face_texture[i].z()-1]);
            vehicle_shape_normal.push_back(vehicle_normal[vehicle_face_normal[i].x()-1]);
            vehicle_shape_normal.push_back(vehicle_normal[vehicle_face_normal[i].y()-1]);
            vehicle_shape_normal.push_back(vehicle_normal[vehicle_face_normal[i].z()-1]);

        }
    }

}

void GLWidget::ros_init(ros::NodeHandle &n)
{
   my_ros->ros_initialize(n);
}

void GLWidget::set_pc()
{
    cpc.clear();
    cpc_color.clear();
    QVector3D tmp;
    for(int i = 0 ; i < (int)my_ros->_pc.size(); i ++){
      tmp.setX(my_ros->_pc[i].x);
      tmp.setY(my_ros->_pc[i].y);
      tmp.setZ(my_ros->_pc[i].z);
      cpc.push_back(tmp);
      tmp.setX(255);
      tmp.setY(255);
      tmp.setZ(255);
      cpc_color.push_back(tmp);
    }

    //viewpoint change
    if(view_point_auto_change == 1){
      //viewpoint change
      float x = cameraPosition[0];
      float y = cameraPosition[1];
      float z = cameraPosition[2];
      float r = sqrt(x*x + y*y);
      float theta;
      if((x == 0) && (y == 0)){
        view_point_auto_change = 0;
        return;
      }else if(x == 0 ){
        if(y > 0){
          theta = -3.141592/2;
        }else{
          theta = 3.141592/2;
        }
      }else{
        if(x>0){
          theta = atan(y/x);
        }else{
          theta = atan(y/x) + 3.141592;
        }
      }
      theta = theta + change_angle_amount;
      cameraPosition[0] = r*cos(theta);
      cameraPosition[1] = r*sin(theta);
      cameraFront = - cameraPosition;
      cameraUpDirection[0] = cameraFront[0];
      cameraUpDirection[1] = cameraFront[1];
      cameraUpDirection[2] = 0;

    }

    //Object detection(people)
    if(object_detection_flag == 1){

      cpc_object.clear();
      cpc_object_color.clear();

      int K = 1;	//number of point to find
      float filtering_threshold = distance_threshold;
      QVector3D color;
      color.setX(1);
      color.setY(0);
      color.setZ(0);

      //single Thread
//      pcl::PointXYZRGB searchPoint;
//      std::vector<int> pointIdxNKNSearch(K);
//      std::vector<float> pointNKNSquaredDistance(K);
//      for(int i = 0 ; i < (int)cpc.size() ; i ++){
//        pointIdxNKNSearch.clear();
//        pointNKNSquaredDistance.clear();
//        searchPoint.x = cpc[i].x();
//        searchPoint.y = cpc[i].y();
//        searchPoint.z = cpc[i].z();
//        searchPoint.r = 0; searchPoint.g = 0; searchPoint.b = 0;

//        if ( kdtree.nearestKSearch (searchPoint, K, pointIdxNKNSearch, pointNKNSquaredDistance) > 0 ){	//find NN
//          for(int j = 0 ; j < (int)pointNKNSquaredDistance.size() ; j ++){
//            if(pointNKNSquaredDistance[j] > filtering_threshold){
//              cpc_object.push_back(cpc[i]);
//              cpc_object_color.push_back(color);
//            }
//          }
//        }
//      }

      //Muti Thread uisng OpenMP
      std::vector<std::vector<int>> pointIdxNKNSearch(cpc.size(),std::vector<int>(K));
      std::vector<std::vector<float>> pointNKNSquaredDistance(cpc.size(),std::vector<float>(K));

      #pragma omp parallel for
      for(int i = 0 ; i < (int)cpc.size() ; i ++){
        //search bound
        if(fabs(cpc[i].x())>search_region_X || fabs(cpc[i].y())>search_region_Y || fabs(cpc[i].z())>search_region_Z){
          continue;
        }
        //
        pcl::PointXYZRGB searchPoint;
        pointIdxNKNSearch[i].clear();
        pointNKNSquaredDistance[i].clear();
        searchPoint.x = cpc[i].x();
        searchPoint.y = cpc[i].y();
        searchPoint.z = cpc[i].z();
        searchPoint.r = 0; searchPoint.g = 0; searchPoint.b = 0;

        if ( kdtree.nearestKSearch (searchPoint, K, pointIdxNKNSearch[i], pointNKNSquaredDistance[i]) > 0 ){	//find NN
          for(int j = 0 ; j < (int)pointNKNSquaredDistance[i].size() ; j ++){
            if(pointNKNSquaredDistance[i][j] > filtering_threshold){
              #pragma omp critical
              {
                cpc_object.push_back(cpc[i]);
                cpc_object_color.push_back(color);
              }
            }
          }
        }
      }

      //cpc to point cloud
      pcl::PointCloud <pcl::PointXYZRGB>::Ptr object_cloud (new pcl::PointCloud <pcl::PointXYZRGB>);
      pcl::PointCloud <pcl::PointXYZRGB>::Ptr object_cloud_filtered (new pcl::PointCloud <pcl::PointXYZRGB>);

      pcl::PointXYZRGB point;
      for(int i = 0 ; (int)i < cpc_object.size(); i ++){
        point.x = cpc_object[i].x();
        point.y = cpc_object[i].y();
        point.z = cpc_object[i].z();
        point.r = cpc_object_color[i].x();
        point.g = cpc_object_color[i].y();
        point.b = cpc_object_color[i].z();
        object_cloud->push_back(point);
      }

      if(object_cloud->points.size()<40){
        update();
        cpc_object.clear();
        cpc_object_color.clear();
        return;
      }
      //noise filtering(small object)
      pcl::RadiusOutlierRemoval<pcl::PointXYZRGB> radius_outlier_removal;
      radius_outlier_removal.setInputCloud (object_cloud);
      radius_outlier_removal.setRadiusSearch (0.3);
      radius_outlier_removal.setMinNeighborsInRadius (20);
      radius_outlier_removal.filter (* object_cloud_filtered);


      if(object_cloud_filtered->points.size()<40){
        update();
        cpc_object.clear();
        cpc_object_color.clear();
        return;
      }
//      QVector3D tmp_point;
//      cpc_object.clear();
//      cpc_object_color.clear();
//      for(int i = 0 ; i < (int)object_cloud_filtered->points.size(); i ++){
//        tmp_point.setX(object_cloud_filtered->points[i].x);
//        tmp_point.setY(object_cloud_filtered->points[i].y);
//        tmp_point.setZ(object_cloud_filtered->points[i].z);
//        cpc_object.push_back(tmp_point);
//        tmp_point.setX(object_cloud_filtered->points[i].r);
//        tmp_point.setY(object_cloud_filtered->points[i].g);
//        tmp_point.setZ(object_cloud_filtered->points[i].b);
//        cpc_object_color.push_back(tmp_point);
//      }

      pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGB>);

      //segmentation1

//      if(object_cloud_filtered->size() == 0 ) return;
//      tree->setInputCloud (object_cloud_filtered);
//      std::vector<pcl::PointIndices> cluster_indices;
//      pcl::EuclideanClusterExtraction<pcl::PointXYZRGB> ec;
//      ec.setClusterTolerance (0.3);
//      ec.setMinClusterSize (10);
//      ec.setMaxClusterSize (25000);
//      ec.setSearchMethod (tree);
//      ec.setInputCloud (object_cloud_filtered);
//      ec.extract (cluster_indices);
//      for(int i = 0 ; i < (int)cluster_indices.size() ; i ++){	//for each segment
//        pcl::PointXYZ color;
//        color.x = ((double) rand() / (RAND_MAX));
//        color.y = ((double) rand() / (RAND_MAX));
//        color.z = ((double) rand() / (RAND_MAX));
//        for(int j = 0 ; j < (int) cluster_indices[i].indices.size() ; j ++){
//          int index = cluster_indices[i].indices[j];
//          object_cloud_filtered->points[index].r = color.x;
//          object_cloud_filtered->points[index].g = color.y;
//          object_cloud_filtered->points[index].b = color.z;
//        }
//      }
//      cpc_object.clear();
//      cpc_object_color.clear();

//      QVector3D tmp_point;
//      for(int i = 0 ; i < (int)object_cloud_filtered->points.size(); i ++){
//        tmp_point.setX(object_cloud_filtered->points[i].x);
//        tmp_point.setY(object_cloud_filtered->points[i].y);
//        tmp_point.setZ(object_cloud_filtered->points[i].z);
//        cpc_object.push_back(tmp_point);
//        tmp_point.setX(object_cloud_filtered->points[i].r);
//        tmp_point.setY(object_cloud_filtered->points[i].g);
//        tmp_point.setZ(object_cloud_filtered->points[i].b);
//        cpc_object_color.push_back(tmp_point);
//      }

      //segmentation2

       pcl::RegionGrowingRGB<pcl::PointXYZRGB> reg;
       reg.setInputCloud (object_cloud_filtered);
       reg.setSearchMethod (tree);
       reg.setDistanceThreshold (1.0);
       reg.setPointColorThreshold (30);
       reg.setRegionColorThreshold (30);
       reg.setMinClusterSize (10);
       reg.setNumberOfRegionNeighbours (30);	//high -> under seg
       reg.setNumberOfNeighbours (20);		//high -> under seg
       std::vector <pcl::PointIndices> clusters;
       reg.extract (clusters);
       pcl::PointCloud <pcl::PointXYZRGB>::Ptr colored_cloud (new pcl::PointCloud <pcl::PointXYZRGB>);
       colored_cloud = reg.getColoredCloud();

       //      for(int i = 0 ; i < (int)cluster_indices.size() ; i ++){	//for each segment
       //        pcl::PointXYZ color;
       //        color.x = ((double) rand() / (RAND_MAX));
       //        color.y = ((double) rand() / (RAND_MAX));
       //        color.z = ((double) rand() / (RAND_MAX));
       //        for(int j = 0 ; j < (int) cluster_indices[i].indices.size() ; j ++){
       //          int index = cluster_indices[i].indices[j];
       //          object_cloud_filtered->points[index].r = color.x;
       //          object_cloud_filtered->points[index].g = color.y;
       //          object_cloud_filtered->points[index].b = color.z;
       //        }
       //      }

       //Tracking!!

       pre_segment_center_info.clear();
       pre_segment_color_info.clear();

       pre_segment_center_info = new_segment_center_info;
       pre_segment_color_info = new_segment_color_info;

       new_segment_center_info.clear();
       new_segment_color_info.clear();

       for(int i = 0 ; i < (int)clusters.size() ; i ++){	//for each segment
         QVector<float> average_point;
         average_point.clear();
         average_point.push_back(0); average_point.push_back(0); average_point.push_back(0);
         QVector<int> color_info;
         color_info.clear();
         for(int j = 0 ; j < (int) clusters[i].indices.size() ; j ++){
            int index = clusters[i].indices[j];
            if(j == 0){
              color_info.push_back(colored_cloud->points[index].r);
              color_info.push_back(colored_cloud->points[index].g);
              color_info.push_back(colored_cloud->points[index].b);
            }
            average_point[0] = average_point[0]+colored_cloud->points[index].x;
            average_point[1] = average_point[1]+colored_cloud->points[index].y;
            average_point[2] = average_point[2]+colored_cloud->points[index].z;
          }
          average_point[0] = average_point[0]/clusters[i].indices.size();
          average_point[1] = average_point[1]/clusters[i].indices.size();
          average_point[2] = average_point[2]/clusters[i].indices.size();

          float distance;
          float tracking_threshold = tracking_threshold_value;
          bool find_matching = false;

          QMap<int, QVector<float>>::iterator iter;

          for (iter = pre_segment_center_info.begin(); iter != pre_segment_center_info.end(); ++iter){

             distance =sqrt((average_point[0]-iter.value()[0])*(average_point[0]-iter.value()[0]) +
                            (average_point[1]-iter.value()[1])*(average_point[1]-iter.value()[1]) +
                            (average_point[2]-iter.value()[2])*(average_point[2]-iter.value()[2]));
             if(distance < tracking_threshold){
                find_matching = true;
                //add to map
                new_segment_center_info[new_segment_center_info.size()] = average_point;
                new_segment_color_info[new_segment_color_info.size()] = pre_segment_color_info[iter.key()];
                //change point color
                for(int j = 0 ; j < (int) clusters[i].indices.size() ; j ++){
                   int index = clusters[i].indices[j];
                   colored_cloud->points[index].r = pre_segment_color_info[iter.key()][0];
                   colored_cloud->points[index].g = pre_segment_color_info[iter.key()][1];
                   colored_cloud->points[index].b = pre_segment_color_info[iter.key()][2];
                 }
                break;
             }
          }
          if(find_matching == false){
            //add new data to map
            new_segment_center_info[new_segment_center_info.size()] = average_point;
            new_segment_color_info[new_segment_color_info.size()] = color_info;
          }
       }

       //check same color segment

      QMap<int, QVector<int>>::iterator iter1;
      QMap<int, QVector<int>>::iterator iter2;
      iter1 = new_segment_color_info.begin();
      iter2 = new_segment_color_info.begin();

      for(iter1 = new_segment_color_info.begin(); iter1 != (new_segment_color_info.end() - 1); ++iter1){
        for(iter2 = iter1 + 1; iter2 != new_segment_color_info.end(); ++iter2){
           if((iter1.value()[0]==iter2.value()[0])&&(iter1.value()[1]==iter2.value()[1])&&(iter1.value()[2]==iter2.value()[2])){
              iter2.value()[0] = (int)(((double) rand() / (RAND_MAX))*255);
              iter2.value()[1] = (int)(((double) rand() / (RAND_MAX))*255);
              iter2.value()[2] = (int)(((double) rand() / (RAND_MAX))*255);
           }
        }
      }

      //set cpc

       cpc_object.clear();
       cpc_object_color.clear();

       QVector3D tmp_point;
       for(int i = 0 ; i < (int)colored_cloud->points.size(); i ++){
         tmp_point.setX(colored_cloud->points[i].x);
         tmp_point.setY(colored_cloud->points[i].y);
         tmp_point.setZ(colored_cloud->points[i].z);
         cpc_object.push_back(tmp_point);
         if(colored_cloud->points[i].z > height_max || colored_cloud->points[i].z < height_min)
         {
           tmp_point.setX(0);
           tmp_point.setY(0);
           tmp_point.setZ(0);
           cpc_object_color.push_back(tmp_point);
         }else{
           tmp_point.setX((double)colored_cloud->points[i].r/255.0);
           tmp_point.setY((double)colored_cloud->points[i].g/255.0);
           tmp_point.setZ((double)colored_cloud->points[i].b/255.0);
           cpc_object_color.push_back(tmp_point);
         }
       }

    }
    update();
}

void GLWidget::initial_voxel_gen()
{

  QVector3D voxel_color_3D;
  voxel_color_3D.setX(voxel_color_R);
  voxel_color_3D.setY(voxel_color_G);
  voxel_color_3D.setZ(voxel_color_B);
  QVector3D voxel_line_color_3D;
  voxel_line_color_3D.setX(voxel_line_color_R);
  voxel_line_color_3D.setY(voxel_line_color_G);
  voxel_line_color_3D.setZ(voxel_line_color_B);

  subsampled_initial_cpc_voxel.clear();
  subsampled_initial_cpc_voxel_color.clear();

  subsampled_initial_cpc_voxel_line.clear();
  subsampled_initial_cpc_voxel_line_color.clear();



  for(int i = 0 ; (int)i < subsampled_initial_cpc.size(); i ++){

      QVector3D points[8];
      points[0].setX(subsampled_initial_cpc[i].x()-0.5*voxel_size); points[0].setY(subsampled_initial_cpc[i].y()-0.5*voxel_size); points[0].setZ(subsampled_initial_cpc[i].z()+0.5*voxel_size);
      points[1].setX(subsampled_initial_cpc[i].x()-0.5*voxel_size); points[1].setY(subsampled_initial_cpc[i].y()+0.5*voxel_size); points[1].setZ(subsampled_initial_cpc[i].z()+0.5*voxel_size);
      points[2].setX(subsampled_initial_cpc[i].x()+0.5*voxel_size); points[2].setY(subsampled_initial_cpc[i].y()-0.5*voxel_size); points[2].setZ(subsampled_initial_cpc[i].z()+0.5*voxel_size);
      points[3].setX(subsampled_initial_cpc[i].x()+0.5*voxel_size); points[3].setY(subsampled_initial_cpc[i].y()+0.5*voxel_size); points[3].setZ(subsampled_initial_cpc[i].z()+0.5*voxel_size);
      points[4].setX(subsampled_initial_cpc[i].x()-0.5*voxel_size); points[4].setY(subsampled_initial_cpc[i].y()-0.5*voxel_size); points[4].setZ(subsampled_initial_cpc[i].z()-0.5*voxel_size);
      points[5].setX(subsampled_initial_cpc[i].x()-0.5*voxel_size); points[5].setY(subsampled_initial_cpc[i].y()+0.5*voxel_size); points[5].setZ(subsampled_initial_cpc[i].z()-0.5*voxel_size);
      points[6].setX(subsampled_initial_cpc[i].x()+0.5*voxel_size); points[6].setY(subsampled_initial_cpc[i].y()-0.5*voxel_size); points[6].setZ(subsampled_initial_cpc[i].z()-0.5*voxel_size);
      points[7].setX(subsampled_initial_cpc[i].x()+0.5*voxel_size); points[7].setY(subsampled_initial_cpc[i].y()+0.5*voxel_size); points[7].setZ(subsampled_initial_cpc[i].z()-0.5*voxel_size);

      subsampled_initial_cpc_voxel.push_back(points[0]); subsampled_initial_cpc_voxel_color.push_back(voxel_color_3D);
      subsampled_initial_cpc_voxel.push_back(points[1]); subsampled_initial_cpc_voxel_color.push_back(voxel_color_3D);
      subsampled_initial_cpc_voxel.push_back(points[2]); subsampled_initial_cpc_voxel_color.push_back(voxel_color_3D);

      subsampled_initial_cpc_voxel.push_back(points[2]); subsampled_initial_cpc_voxel_color.push_back(voxel_color_3D);
      subsampled_initial_cpc_voxel.push_back(points[3]); subsampled_initial_cpc_voxel_color.push_back(voxel_color_3D);
      subsampled_initial_cpc_voxel.push_back(points[1]); subsampled_initial_cpc_voxel_color.push_back(voxel_color_3D);

      subsampled_initial_cpc_voxel.push_back(points[2]); subsampled_initial_cpc_voxel_color.push_back(voxel_color_3D);
      subsampled_initial_cpc_voxel.push_back(points[6]); subsampled_initial_cpc_voxel_color.push_back(voxel_color_3D);
      subsampled_initial_cpc_voxel.push_back(points[7]); subsampled_initial_cpc_voxel_color.push_back(voxel_color_3D);

      subsampled_initial_cpc_voxel.push_back(points[7]); subsampled_initial_cpc_voxel_color.push_back(voxel_color_3D);
      subsampled_initial_cpc_voxel.push_back(points[2]); subsampled_initial_cpc_voxel_color.push_back(voxel_color_3D);
      subsampled_initial_cpc_voxel.push_back(points[3]); subsampled_initial_cpc_voxel_color.push_back(voxel_color_3D);

      subsampled_initial_cpc_voxel.push_back(points[3]); subsampled_initial_cpc_voxel_color.push_back(voxel_color_3D);
      subsampled_initial_cpc_voxel.push_back(points[7]); subsampled_initial_cpc_voxel_color.push_back(voxel_color_3D);
      subsampled_initial_cpc_voxel.push_back(points[5]); subsampled_initial_cpc_voxel_color.push_back(voxel_color_3D);

      subsampled_initial_cpc_voxel.push_back(points[5]); subsampled_initial_cpc_voxel_color.push_back(voxel_color_3D);
      subsampled_initial_cpc_voxel.push_back(points[3]); subsampled_initial_cpc_voxel_color.push_back(voxel_color_3D);
      subsampled_initial_cpc_voxel.push_back(points[1]); subsampled_initial_cpc_voxel_color.push_back(voxel_color_3D);

      subsampled_initial_cpc_voxel.push_back(points[0]); subsampled_initial_cpc_voxel_color.push_back(voxel_color_3D);
      subsampled_initial_cpc_voxel.push_back(points[2]); subsampled_initial_cpc_voxel_color.push_back(voxel_color_3D);
      subsampled_initial_cpc_voxel.push_back(points[6]); subsampled_initial_cpc_voxel_color.push_back(voxel_color_3D);

      subsampled_initial_cpc_voxel.push_back(points[6]); subsampled_initial_cpc_voxel_color.push_back(voxel_color_3D);
      subsampled_initial_cpc_voxel.push_back(points[0]); subsampled_initial_cpc_voxel_color.push_back(voxel_color_3D);
      subsampled_initial_cpc_voxel.push_back(points[4]); subsampled_initial_cpc_voxel_color.push_back(voxel_color_3D);

      subsampled_initial_cpc_voxel.push_back(points[0]); subsampled_initial_cpc_voxel_color.push_back(voxel_color_3D);
      subsampled_initial_cpc_voxel.push_back(points[4]); subsampled_initial_cpc_voxel_color.push_back(voxel_color_3D);
      subsampled_initial_cpc_voxel.push_back(points[1]); subsampled_initial_cpc_voxel_color.push_back(voxel_color_3D);

      subsampled_initial_cpc_voxel.push_back(points[1]); subsampled_initial_cpc_voxel_color.push_back(voxel_color_3D);
      subsampled_initial_cpc_voxel.push_back(points[5]); subsampled_initial_cpc_voxel_color.push_back(voxel_color_3D);
      subsampled_initial_cpc_voxel.push_back(points[4]); subsampled_initial_cpc_voxel_color.push_back(voxel_color_3D);

      subsampled_initial_cpc_voxel.push_back(points[4]); subsampled_initial_cpc_voxel_color.push_back(voxel_color_3D);
      subsampled_initial_cpc_voxel.push_back(points[6]); subsampled_initial_cpc_voxel_color.push_back(voxel_color_3D);
      subsampled_initial_cpc_voxel.push_back(points[7]); subsampled_initial_cpc_voxel_color.push_back(voxel_color_3D);

      subsampled_initial_cpc_voxel.push_back(points[7]); subsampled_initial_cpc_voxel_color.push_back(voxel_color_3D);
      subsampled_initial_cpc_voxel.push_back(points[4]); subsampled_initial_cpc_voxel_color.push_back(voxel_color_3D);
      subsampled_initial_cpc_voxel.push_back(points[5]); subsampled_initial_cpc_voxel_color.push_back(voxel_color_3D);

      subsampled_initial_cpc_voxel_line.push_back(points[0]); subsampled_initial_cpc_voxel_line_color.push_back(voxel_line_color_3D);
      subsampled_initial_cpc_voxel_line.push_back(points[1]); subsampled_initial_cpc_voxel_line_color.push_back(voxel_line_color_3D);

      subsampled_initial_cpc_voxel_line.push_back(points[1]); subsampled_initial_cpc_voxel_line_color.push_back(voxel_line_color_3D);
      subsampled_initial_cpc_voxel_line.push_back(points[3]); subsampled_initial_cpc_voxel_line_color.push_back(voxel_line_color_3D);

      subsampled_initial_cpc_voxel_line.push_back(points[3]); subsampled_initial_cpc_voxel_line_color.push_back(voxel_line_color_3D);
      subsampled_initial_cpc_voxel_line.push_back(points[2]); subsampled_initial_cpc_voxel_line_color.push_back(voxel_line_color_3D);

      subsampled_initial_cpc_voxel_line.push_back(points[2]); subsampled_initial_cpc_voxel_line_color.push_back(voxel_line_color_3D);
      subsampled_initial_cpc_voxel_line.push_back(points[0]); subsampled_initial_cpc_voxel_line_color.push_back(voxel_line_color_3D);

      subsampled_initial_cpc_voxel_line.push_back(points[0]); subsampled_initial_cpc_voxel_line_color.push_back(voxel_line_color_3D);
      subsampled_initial_cpc_voxel_line.push_back(points[4]); subsampled_initial_cpc_voxel_line_color.push_back(voxel_line_color_3D);

      subsampled_initial_cpc_voxel_line.push_back(points[1]); subsampled_initial_cpc_voxel_line_color.push_back(voxel_line_color_3D);
      subsampled_initial_cpc_voxel_line.push_back(points[5]); subsampled_initial_cpc_voxel_line_color.push_back(voxel_line_color_3D);

      subsampled_initial_cpc_voxel_line.push_back(points[3]); subsampled_initial_cpc_voxel_line_color.push_back(voxel_line_color_3D);
      subsampled_initial_cpc_voxel_line.push_back(points[7]); subsampled_initial_cpc_voxel_line_color.push_back(voxel_line_color_3D);

      subsampled_initial_cpc_voxel_line.push_back(points[2]); subsampled_initial_cpc_voxel_line_color.push_back(voxel_line_color_3D);
      subsampled_initial_cpc_voxel_line.push_back(points[6]); subsampled_initial_cpc_voxel_line_color.push_back(voxel_line_color_3D);

      subsampled_initial_cpc_voxel_line.push_back(points[4]); subsampled_initial_cpc_voxel_line_color.push_back(voxel_line_color_3D);
      subsampled_initial_cpc_voxel_line.push_back(points[5]); subsampled_initial_cpc_voxel_line_color.push_back(voxel_line_color_3D);

      subsampled_initial_cpc_voxel_line.push_back(points[5]); subsampled_initial_cpc_voxel_line_color.push_back(voxel_line_color_3D);
      subsampled_initial_cpc_voxel_line.push_back(points[7]); subsampled_initial_cpc_voxel_line_color.push_back(voxel_line_color_3D);

      subsampled_initial_cpc_voxel_line.push_back(points[7]); subsampled_initial_cpc_voxel_line_color.push_back(voxel_line_color_3D);
      subsampled_initial_cpc_voxel_line.push_back(points[6]); subsampled_initial_cpc_voxel_line_color.push_back(voxel_line_color_3D);

      subsampled_initial_cpc_voxel_line.push_back(points[6]); subsampled_initial_cpc_voxel_line_color.push_back(voxel_line_color_3D);
      subsampled_initial_cpc_voxel_line.push_back(points[4]); subsampled_initial_cpc_voxel_line_color.push_back(voxel_line_color_3D);



  }

}

void GLWidget::search_region_vertex_recal()
{
  search_region.clear();
  QVector3D points[8];
  points[0].setX(-search_region_X); points[0].setY(-search_region_Y); points[0].setZ(+search_region_Z);
  points[1].setX(-search_region_X); points[1].setY(+search_region_Y); points[1].setZ(+search_region_Z);
  points[2].setX(+search_region_X); points[2].setY(-search_region_Y); points[2].setZ(+search_region_Z);
  points[3].setX(+search_region_X); points[3].setY(+search_region_Y); points[3].setZ(+search_region_Z);
  points[4].setX(-search_region_X); points[4].setY(-search_region_Y); points[4].setZ(-search_region_Z);
  points[5].setX(-search_region_X); points[5].setY(+search_region_Y); points[5].setZ(-search_region_Z);
  points[6].setX(+search_region_X); points[6].setY(-search_region_Y); points[6].setZ(-search_region_Z);
  points[7].setX(+search_region_X); points[7].setY(+search_region_Y); points[7].setZ(-search_region_Z);

  search_region.push_back(points[0]);
  search_region.push_back(points[1]);

  search_region.push_back(points[1]);
  search_region.push_back(points[3]);

  search_region.push_back(points[3]);
  search_region.push_back(points[2]);

  search_region.push_back(points[2]);
  search_region.push_back(points[0]);

  search_region.push_back(points[0]);
  search_region.push_back(points[4]);

  search_region.push_back(points[1]);
  search_region.push_back(points[5]);

  search_region.push_back(points[3]);
  search_region.push_back(points[7]);

  search_region.push_back(points[2]);
  search_region.push_back(points[6]);

  search_region.push_back(points[4]);
  search_region.push_back(points[5]);

  search_region.push_back(points[5]);
  search_region.push_back(points[7]);

  search_region.push_back(points[7]);
  search_region.push_back(points[6]);

  search_region.push_back(points[6]);
  search_region.push_back(points[4]);

}
