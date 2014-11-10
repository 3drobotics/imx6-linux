#!/bin/sh

ROOTFS=$1
BOOTPART=$2

export ARCH="arm"
export CROSS_COMPILE="arm-linux-gnueabihf-"
export LOADADDR="0x10800000"
export INSTALL_MOD_PATH="./mod_install"

rm -rf ./mod_install
mkdir ./mod_install

make clean distclean
make -j16 imx6solo_defconfig
make -j16 uImage modules
make -j16 modules_install
make -j16 headers_install
make -j16 dtbs

cp arch/arm/boot/uImage "$BOOTPART/"
cp arch/arm/boot/dts/imx6solo-3dr.dtb "$BOOTPART/"
sudo cp -r mod_install/lib/modules/* "$ROOTFS/lib/modules/"
sudo cp -r usr/include/* "$ROOTFS/usr/include/"
