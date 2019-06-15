#!/bin/bash
HOST=ftp.kauailabs.com
USER=slibert
PASSWORD=`echo UmVkc2tpbjcK | base64 --decode`

ncftp <<EOF
open -u $USER -p $PASSWORD $HOST 
debug 1
cd /kauailabs.com
mkdir maven2
cd maven2
mkdir edu
cd edu
mkdir wpi
cd wpi
mkdir first
cd first
mkdir hal
cd hal
put -R ~/.m2/repository/edu/wpi/first/hal/hal-cpp
close
bye
EOF
