#!/bin/bash

set -x

cd ~
git clone https://github.com/kauailabs/allwpilib.git

cd allwpilib/
git checkout master
git checkout

cd ~
# Retrieve repo containing required Phoenix headers allowing
# VMX-pi to become a Phoenix platform
git clone https://github.com/CrossTheRoadElec/Phoenix-Linux-SocketCAN-Example.git

#Run the commands between these two comments to rebuild the libraries

#Select from here until next comment for rebuilding the libraries
# BUILDTYPE must be either Debug (default) or Release.
BUILDTYPE=Debug
# BUILD_CPP_ONLY must be either OFF (default) or ON.  If ON, Java & JNI components are not built
BUILD_CPP_ONLY=OFF

if [ "$BUILD_CPP_ONLY" == "ON" ]; then
	SHARED_LIBS=OFF
	BUILD_DIR=${BUILDTYPE}_Static
else
	SHARED_LIBS=ON
	BUILD_DIR=${BUILDTYPE}_Shared
fi

cd ~
sudo rm -r wpilib_build_${BUILD_DIR}
mkdir wpilib_build_${BUILD_DIR}/
cd wpilib_build_${BUILD_DIR}
export JAVA_HOME=/usr/lib/jvm/jdk-8-oracle-arm32-vfp-hflt
cmake -D WITHOUT_ALLWPILIB=OFF -D WITHOUT_JAVA=${BUILD_CPP_ONLY} -DCMAKE_BUILD_TYPE=${BUILDTYPE} -DBUILD_SHARED_LIBS=${SHARED_LIBS} -D OpenCV_DIR=/home/pi/opencv-3.3.1/build ~/allwpilib
make -j5
sudo make install

# Publish the HAL libraries to the local maven repo
# (These will be published to the central maven repo in a later step)

cd hal
gradle publishMavenPublicationToMavenLocal

# Copy VMX-pi Channel Maps.  Note that this copying is now optional; defaults are used if the ChannelMap is not present.
#sudo cp -r ~/allwpilib/hal/src/main/native/mau/Translator/Maps/ChannelMap.json /usr/local/wpilib/lib

# Not sure why this is here - it may no longer be necessary
sudo mkdir /tmp/frc_versions
