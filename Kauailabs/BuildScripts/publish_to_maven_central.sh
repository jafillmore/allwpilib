#!/bin/bash

cd ~/.m2/repository/com/kauailabs/vmx/first/hal/hal-cpp
latest_subdir=$(ls -td -- */ | head -n 1)
cd $latest_subdir
ls *
jar -cvf bundle.jar *.pom *.asc *.zip
ls *

publish_bundle_dir=$(pwd)
echo "******************************************************"
echo
echo "Upload the bundle file ($publish_bundle_dir/bundle.jar) as an Artifact Bundle on the Staging Upload page at the Sonatype OSS site (oss.sonatype.org)"
echo
echo "******************************************************"
