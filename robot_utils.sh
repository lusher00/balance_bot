estop_init() {
    if [ ! -d /sys/class/gpio/gpio537 ]; then
        sudo sh -c 'echo 537 > /sys/class/gpio/export'
        sleep 0.2
    fi
    echo out > /sys/class/gpio/gpio537/direction
    echo 1   > /sys/class/gpio/gpio537/value
}
estop_active() {
    echo 0 > /sys/class/gpio/gpio537/value
}
estop_release() {
    echo 1 > /sys/class/gpio/gpio537/value
}
