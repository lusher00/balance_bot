#!/bin/bash
GPIO=/sys/class/gpio/gpio57
if [ ! -d "$GPIO" ]; then
    echo 57 > /sys/class/gpio/export
    sleep 0.15
fi
echo out > ${GPIO}/direction
echo 1   > ${GPIO}/value
echo "E-stop cleared, GPIO57=1"
