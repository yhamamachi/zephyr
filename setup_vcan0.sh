#!/bin/bash -x

sudo modprobe vcan
sudo ip link add dev vcan0 type vcan
sudo ip link set up vcan0
sudo ip link property add dev vcan0 altname zcan0
