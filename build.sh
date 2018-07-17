#!/bin/bash
#export KBUILD_BUILD_USER=
#export KBUILD_BUILD_HOST=
export CROSS_COMPILE=/mnt/build/kernel/aarch64-linux-android-4.9/bin/aarch64-linux-android-
#export USE_CCACHE=1
export ARCH=arm64 ARCH_MTK_PLATFORM=mt6735
export TARGET=out
make O=$TARGET ARCH=$ARCH CROSS_COMPILE=$CROSS_COMPILE tit_l01_defconfig
make O=$TARGET ARCH=$ARCH CROSS_COMPILE=$CROSS_COMPILE -j4
