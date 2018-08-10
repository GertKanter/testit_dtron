#!/bin/bash

cd /catkin_ws

CURDIR=`pwd`

mkdir stuff
cd stuff
git clone https://github.com/ooici/protobuf-2.3.0.git
mv protobuf-2.3.0 protobuf
cd protobuf
./autogen.sh
./configure --prefix=$CURDIR/devel
make
make check
make install

cd $CURDIR

mv devel protobuf

cd stuff

git clone https://github.com/glycerine/spread-src-4.4.0.git

cd spread-src-4.4.0
./configure --prefix=$CURDIR/devel
make
make install

cd $CURDIR
mv devel spread

ln -sf $CURDIR/devel $CURDIR/src/testit/dtron/protobuf
ln -sf $CURDIR/devel $CURDIR/src/testit/dtron/spread

cd stuff/protobuf
make install

cd $CURDIR

cd stuff/spread-src-4.4.0
make install

cd $CURDIR

#rm -r stuff