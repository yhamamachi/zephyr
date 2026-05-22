#!/bin/bash

SCRIPT_DIR=$(cd `dirname $0` && pwd)
cd $SCRIPT_DIR
#TARGET=samples/subsys/display/lvgl
TARGET=lvgl-demo
Usage () {
    echo "Usage: $0"
    exit
}

if [[ "${1:-dummy}" == "-h" ]]; then
    Usage ; exit -1
elif [[ "${1:-dummy}" == "can_loopback" ]]; then
    TARGET=can_loopback
fi

#mkdir -p _work
#cd _work
if [[ ! -e ./.venv ]]; then
    python3 -m venv ./.venv
    source ./.venv/bin/activate
    pip install west
    pip install -U pip
else
    source ./.venv/bin/activate
fi

if [[ ! -e ./.west ]]; then
    west init -l ./
    west update cmsis lvgl
    west zephyr-export
    west packages pip --install
fi

ZEPHYR_SDK_VER=1.0.1
#TARGET_BOARD=rcar_sparrow_hawk/r8a779g3/r52
#TARGET_BOARD=sparrowhawk_rcar_v4h/r8a779g0/r52
TARGET_BOARD=native_sim/native/64
if [[ ! -e "${HOME}/zephyr-sdk-${ZEPHYR_SDK_VER}/gnu/arm-zephyr-eabi" ]]; then
    west sdk install --gnu-toolchain arm-zephyr-eabi
fi

west -z ../zephyr build -p always -b ${TARGET_BOARD} -S socketcan-native-sim ./${TARGET}

