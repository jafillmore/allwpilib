#!/bin/bash
CURR_TAG=`git describe --exact-match --tags $(git log -n1 --pretty='%h')`
echo -n ${CURR_TAG:1}