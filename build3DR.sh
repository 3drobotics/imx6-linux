#!/bin/sh


export ARCH=arm
export CROSS_COMPILE=arm-linux-gnueabihf-
export LOADADDR=10800000

make distclean
make clean
make imx6solo_defconfig
make -j24 uImage 
make -j24 dtbs 
make -j24 modules

export INSTALL_MOD_PATH=~/Desktop/tmp
rm -rf $INSTALL_MOD_PATH/*
#make -j24 modules_install
