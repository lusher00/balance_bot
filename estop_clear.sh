#!/bin/bash
GPIO=/sys/class/gpio/gpio537
if [ ! -d "$GPIO" ]; then
    echo 537 > /sys/class/gpio/export
    sleep 0.15
fi
echo out > ${GPIO}/direction
echo 1   > ${GPIO}/value
echo "E-stop cleared, GPIO537=1"
