#!/bin/bash

SCRIPT_DIR=$(cd `dirname $0` && pwd)
cd $SCRIPT_DIR

Usage () {
    echo "Usage: $0 <target_core>"
    echo "target_core:"
    echo "    - ca: Cortex-A76"
    echo "    - cr: Cortex-R52"
    exit
}

if [[ $# -ne 1 ]]; then
    Usage; exit -1
elif [[ "$1" != "ca" ]] && [[ "$1" != "cr" ]]; then
    Usage ; exit -1
fi

mkdir -p _work
cd _work
python3 -m venv ./.venv
source ./.venv/bin/activate
pip install west
pip install -U pip

CLONE_OPT_FLAG=""
if [[ "$(west help init | grep CLONE_OPT)" != "" ]]; then
    CLONE_OPT_FLAG=" -o--depth=1 "
fi
west init ${CLONE_OPT_FLAG} -m https://github.com/yhamamachi/zephyr --mr v4.2-branch-sh
west update zephyr cmsis
west zephyr-export
west packages pip --install

ZEPHYR_SDK_VER=0.17.4
TARGET_BOARD=dummy
if [[ "$1" == "ca" ]] ;then
    TARGET_BOARD=rcar_sparrow_hawk/r8a779g3/a76
    if [[ ! -e "${HOME}/zephyr-sdk-${ZEPHYR_SDK_VER}/aarch64-zephyr-elf" ]]; then
        west sdk install --toolchain aarch64-zephyr-elf
    fi
elif [[ "$1" == "cr" ]]; then
    TARGET_BOARD=rcar_sparrow_hawk/r8a779g3/r52
    if [[ ! -e "${HOME}/zephyr-sdk-${ZEPHYR_SDK_VER}/arm-zephyr-eabi" ]]; then
        west sdk install --toolchain arm-zephyr-eabi
    fi
else
    Usage ; exit -1
fi

cd zephyr
#west build -p always -b ${TARGET_BOARD} samples/hello_world
west build -p always -b ${TARGET_BOARD} samples/subsys/shell/shell_module

# Generate fitImage for CA76
if [[ "$1" == "ca" ]] ;then
    dtc -I dts -O dtb ./build/zephyr/zephyr.dts -o ./build/zephyr/zephyr.dtb
    mkimage -f ${SCRIPT_DIR}/fit-image.its ${SCRIPT_DIR}/fitImage
    \cp ${SCRIPT_DIR}/fitImage -f /tftp/
fi

