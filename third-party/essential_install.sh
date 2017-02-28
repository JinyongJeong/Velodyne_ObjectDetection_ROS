#!/usr/bin/env bash
PACKAGES=""
function addpkg {
    PACKAGES="$PACKAGES $@"
}

#sudo apt-get install bc python-software-properties

addpkg\
    build-essential\
    git\
    libboost-dev\
    libboost-all-dev\
    cmake\
    cmake-curses-gui\
    libflann-dev\
    libeigen3-dev\
    libqhull*\
    libusb-dev\
    libgtest-dev\
    freeglut3-dev\
    pkg-config\
    libpcap-dev\
    libglew-dev\
    libxmu-dev\
    libxi-dev\
    libopenni-dev\

# go forth!
echo "apt-get install $PACKAGES"
sudo apt-get update
sudo apt-get install $PACKAGES
