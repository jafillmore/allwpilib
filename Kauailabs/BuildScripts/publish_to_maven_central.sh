#!/bin/bash

if [ ! -f "login" ]; then
  echo 'Login file does not exist'
  exit
else
  LOGIN=$(cat "login")
fi

if [ ! -f "pass" ]; then
  echo 'Password file does not exist'
  exit
else
  PASS=$(cat "pass")
fi

cd ~/.m2/repository/com/kauailabs/vmx/first/hal/hal-cpp
latest_subdir=$(ls -td -- */ | head -n 1)
cd $latest_subdir
ls *
jar -cvf bundle.jar *.pom *.asc *.zip
ls *

# Stage artifact bundle to oss.sonatype.org

curl -ujorlina2 -u $LOGIN:$PASS --request POST -F "file=@bundle.jar" "https://oss.sonatype.org/service/local/staging/bundle_upload"

publish_bundle_dir=$(pwd)
echo
echo "*****************  MANUAL STEP REQUIRED  ***************************"
echo "The VMX-pi WPI HAL artifact bundle (bundle.jar) has been uploaded to"
echo "https://oss.sonatype.org (to the 'Staging Upload' page, using the "
echo "'Artifact Bundle' upload mode)."
echo
echo "This artifact should now be released on the 'Staging Repositories' "
echo "page, by selecting the artifact and pressing 'Release'."
echo
echo "Released artifacts appear in Maven Central approximately 20 minutes"
echo "after being released."
echo "*********************************************************************"
echo
