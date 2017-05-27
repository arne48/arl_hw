#!/bin/sh

cd `dirname $0`

wget -O bcm2835-1.52.tar.gz http://www.airspayce.com/mikem/bcm2835/bcm2835-1.52.tar.gz

tar zxvf bcm2835-1.52.tar.gz
cd bcm2835-1.52
cat ../to_shared.patch | patch -p1
./configure
autoreconf --force --install
./configure
make
sudo make install
