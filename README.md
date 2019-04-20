# Now this code is not maintained

# README #

## Environment

install environment: ubuntu 16.04

## Dependency

* OpenGL
* QT5
* PCL
* ROS

## Installation

You must install these dependency in order because VTK have dependency with qt5 and PCL have dependecy with VTK. 

1. git clone this repository

2. run essential_install.sh

```
cd third-party
chmod 777 ./essential_install.sh
sudo ./essential_install.sh
```

2. install qt5

There is shell file for installation of qt5 in third party folder.

```
sudo ./install_qt.sh
```

3. Install VTK

```
git clone https://github.com/Kitware/VTK.git
```

make build folder and cmake with option to inform qt path. 

```
cd VTK
mkdir build
cd build
cmake -DVTK_QT_VERSION:STRING=5 -DQT_QMAKE_EXECUTABLE:PATH=/opt/Qt5.6.1/5.6/gcc_64/bin/qmake -DVTK_Group_Qt:BOOL=ON -DCMAKE_PREFIX_PATH:PATH=/opt/Qt5.6.1/5.6/gcc_64/lib/cmake -DBUILD_SHARED_LIBS:BOOL=ON ..
```

```
make -j8
sudo make install
```

4. INstall PCL

other dependent library is installed when you run `essential_install.sh`.

```
git clone https://github.com/PointCloudLibrary/pcl.git
```

To use Qt5 you installed, cmake prefix should be set in `/pcl/cmake/pcl_find_qt5.cmake` file.
Add follow line on the top of `/pcl/cmake/pcl_find_qt5.cmake` file. 

```
set(CMAKE_PREFIX_PATH /opt/Qt5.6.1/5.6/gcc_64/lib/cmake)
```

Then, make build folder and make.

```
mkdir build
cd build
cmake ..
make -j8
make install
```


4. Install ROS

refer to [page](http://wiki.ros.org/kinetic/Installation/Ubuntu)

ROS-Base version is recommended because some package of ROS is comflict to pcl or qt

```
sudo apt-get install ros-kinetic-ros-base
```

Fianlly, git clone to catkin workspace, and make!


## shortcut

* I: initial point grab(This step is essential to detect moving object)

* A: Auto rotation

* D: Auto rotation speed down

* U: Auto rotation speed up

* V: background vexel on/off

* S: Moving object detection on/off

* F: Full screen

* C: Point filtering depend on height(z-value)

* 1: Minimum z value decrease

* 2: Minimum z value increase

* 3: Maximum z value decrease

* 4: Maximum 2 value increase

* C: Show object search region

* 5: x range of object search region decrease

* 6: x range of object search region increase

* 7: y range of object search region decrease

* 8: y range of object search region increase

* 9: z range of object search region decrease

* 0: z range of object search region increase


