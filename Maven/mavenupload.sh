set -x

mkdir ~/maven-up/opencv-jni/linux/mau -p
cp -R /usr/local/share/OpenCV/java/libopencv_java331.so ~/maven-up/opencv-jni/linux/mau
pushd ~/maven-up/opencv-jni
jar cf ../opencv-jni.jar ./
popd

#******TODO replace with the custom library under same name
mkdir ~/maven-up/hal-mau/linux/mau/shared -p
cp -R /usr/local/wpilib/lib/libwpiHal.so ~/maven-up/hal-mau/linux/mau/shared/libwpiHalMau.so
pushd ~/maven-up/hal-mau
zip -r ../hal-mau.zip ./
popd

mkdir ~/maven-up/cscore/linux/mau/shared -p
cp -R /usr/local/wpilib/lib/libcscore.so ~/maven-up/cscore/linux/mau/shared
pushd ~/maven-up/cscore
zip -r ../cscore.zip ./
popd

mkdir ~/maven-up/ntcore/linux/mau/shared -p
cp -R /usr/local/wpilib/lib/libntcore.so ~/maven-up/ntcore/linux/mau/shared
pushd ~/maven-up/ntcore
zip -r ../ntcore.zip ./
popd

mkdir ~/maven-up/wpiutil/linux/mau/shared -p
cp -R /usr/local/wpilib/lib/libwpiutil.so ~/maven-up/wpiutil/linux/mau/shared
pushd ~/maven-up/wpiutil
zip -r ../wpiutil.zip ./
popd


mkdir ~/maven-up/opencv-cpp/linux/mau/shared -p
cp -R /usr/lib/arm-linux-gnueabihf/libopencv_*.* ~/maven-up/opencv-cpp/linux/mau/shared
cp -R /usr/local/share/OpenCV/java/libopencv_java*.so ~/maven-up/opencv-cpp/linux/mau/shared
pushd ~/maven-up/opencv-cpp
zip -r ../opencv-cpp.zip ./
popd


pushd poms/opencv-jni
sudo mvn deploy:deploy-file -DpomFile=pom.xml -Dfile=/home/pi/maven-up/opencv-jni.jar -DrepositoryId=kauailabs-maven-ftp -Durl=ftp://ftp.kauailabs.com/kauailabs.com/maven2
popd
pushd poms/hal-mau
sudo mvn deploy:deploy-file -DpomFile=pom.xml -Dfile=/home/pi/maven-up/hal-mau.zip -DrepositoryId=kauailabs-maven-ftp -Durl=ftp://ftp.kauailabs.com/kauailabs.com/maven2
popd
pushd poms/cscore
sudo mvn deploy:deploy-file -DpomFile=pom.xml -Dfile=/home/pi/maven-up/cscore.zip -DrepositoryId=kauailabs-maven-ftp -Durl=ftp://ftp.kauailabs.com/kauailabs.com/maven2
popd
pushd poms/ntcore
sudo mvn deploy:deploy-file -DpomFile=pom.xml -Dfile=/home/pi/maven-up/ntcore.zip -DrepositoryId=kauailabs-maven-ftp -Durl=ftp://ftp.kauailabs.com/kauailabs.com/maven2
popd
pushd poms/wpiutil
sudo mvn deploy:deploy-file -DpomFile=pom.xml -Dfile=/home/pi/maven-up/wpiutil.zip -DrepositoryId=kauailabs-maven-ftp -Durl=ftp://ftp.kauailabs.com/kauailabs.com/maven2
popd
pushd poms/opencv-cpp
sudo mvn deploy:deploy-file -DpomFile=pom.xml -Dfile=/home/pi/maven-up/opencv-cpp.zip -DrepositoryId=kauailabs-maven-ftp -Durl=ftp://ftp.kauailabs.com/kauailabs.com/maven2
popd

rm -r ~/maven-up