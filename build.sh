#!/bin/bash
export KBUILD_BUILD_USER="nillerusr"
export KBUILD_BUILD_HOST="Eclipse"
export CROSS_COMPILE=aarch64-unknown-linux-gnu-
export USE_CCACHE=1
export ARCH=arm64 ARCH_MTK_PLATFORM=mt6735
export TARGET=out
if ! [ -d $TARGET ];then mkdir $TARGET;fi
if ! [ -f $TARGET/.config ];then make O=$TARGET ARCH=$ARCH tit_l01_defconfig;fi
make O=$TARGET ARCH=$ARCH $1 $2
