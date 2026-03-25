#!/bin/bash

SCRIPT_DIR=$(cd `dirname $0` && pwd)
TARGET=can_echoback

Usage () {
    echo "Usage: $0"
    exit
}

cd $SCRIPT_DIR
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
    west update zephyr-xenlib
    west zephyr-export
    west packages pip --install
fi

ZEPHYR_SDK_VER=0.16.9
TARGET_BOARD=sparrowhawk_rcar_v4h/r8a779g0/a76
#TARGET_BOARD=xenvm/xenvm/gicv3
#TARGET_BOARD=sparrow-hawk-xenvm/r8a779g0/r52
#TARGET_BOARD=sparrowhawk_rcar_v4h/r8a779g0/xenvm
if [[ ! -e "${HOME}/zephyr-sdk-${ZEPHYR_SDK_VER}/aarch64-zephyr-elf" ]]; then
    west sdk install --toolchain aarch64-zephyr-elf
fi

west build -p always -b ${TARGET_BOARD} -S sparrowhawk_rcar_v4h_xen_domd ./${TARGET}
#west build -p always -b ${TARGET_BOARD} ./${TARGET}
dtc -I dts -O dtb build/zephyr/zephyr.dts -o build/zephyr/zephyr.dtb

