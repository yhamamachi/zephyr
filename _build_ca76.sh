#!/bin/bash

SCRIPT_DIR=$(cd `dirname $0` && pwd)
cd $SCRIPT_DIR
TARGET=can_echoback
Usage () {
    echo "Usage: $0"
    exit
}

if [[ "${1:-dummy}" == "-h" ]]; then
    Usage ; exit -1
elif [[ "${1:-dummy}" == "can_loopback" ]]; then
    TARGET=can_loopback
fi

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
    west update cmsis
    west zephyr-export
    west packages pip --install
fi

ZEPHYR_SDK_VER=0.17.4
TARGET_BOARD=sparrowhawk_rcar_v4h/r8a779g0/a76
if [[ ! -e "${HOME}/zephyr-sdk-${ZEPHYR_SDK_VER}/aarch64-zephyr-elf" ]]; then
    west sdk install --toolchain aarch64-zephyr-elf
fi

west -z ../zephyr build -p always -b ${TARGET_BOARD} ./${TARGET}

