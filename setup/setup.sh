#!/bin/bash

pushd `dirname $0` > /dev/null
SCRIPTPATH=`pwd -P`
popd > /dev/null

cp $SCRIPTPATH/FindWiringPi.cmake /usr/share/cmake-3.5/Modules/FindWiringPi.cmake
