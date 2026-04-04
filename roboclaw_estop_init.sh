#!/bin/bash
# Initialize RoboClaw E-stop GPIO 57
# Active-low: 0=estop, 1=release

GPIO=57
GPIO_PATH=/sys/class/gpio/gpio${GPIO}

# Export if not already exported
if [ ! -d "$GPIO_PATH" ]; then
    echo $GPIO > /sys/class/gpio/export
    sleep 0.1
fi

echo out > ${GPIO_PATH}/direction
echo 1 > ${GPIO_PATH}/value  # Release e-stop on start

exit 0
