#!/bin/bash

SCRIPT_DIR=$(cd `dirname $0` && pwd)
cd $SCRIPT_DIR

Usage () {
    echo "Usage: $0"
    exit
}

if [[ "${1:-dummy}" == "-h" ]]; then
    Usage ; exit -1
fi

#mkdir -p _work
#cd _work
if [[ ! -e ./.venv ]]; then
    python3 -m venv ./.venv
    source ./.venv/bin/activate
    pip install west
    pip install -U pip

    CLONE_OPT_FLAG=""
    if [[ "$(west help init | grep CLONE_OPT)" != "" ]]; then
        CLONE_OPT_FLAG=" -o--depth=1 "
    fi
    west init ${CLONE_OPT_FLAG} -m ./
    west update zephyr cmsis
    west zephyr-export
    west packages pip --install
else
    source ./.venv/bin/activate
fi

ZEPHYR_SDK_VER=0.17.4
#TARGET_BOARD=rcar_sparrow_hawk/r8a779g3/r52
TARGET_BOARD=sparrowhawk_rcar_v4h/r8a779g0/r52
if [[ ! -e "${HOME}/zephyr-sdk-${ZEPHYR_SDK_VER}/arm-zephyr-eabi" ]]; then
    west sdk install --toolchain arm-zephyr-eabi
fi

#cd zephyr
#west build -p always -b ${TARGET_BOARD} samples/hello_world
west -z ../zephyr build -p always -b ${TARGET_BOARD} ./can_loopback
#west -z ../zephyr build -p always -b ${TARGET_BOARD} ./samples/hello_world

