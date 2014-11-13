#!/bin/sh


export ARCH=arm
export CROSS_COMPILE=arm-linux-gnueabihf-
export LOADADDR=10800000
export INSTALL_MOD_PATH=./mod_install
rm -rf $INSTALL_MOD_PATH
mkdir $INSTALL_MOD_PATH

make distclean
make clean
make imx6solo_defconfig
make -j24 uImage 
make -j24 dtbs 
make -j24 modules
make -j24 modules_install
