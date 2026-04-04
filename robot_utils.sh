estop_init() {
    echo 57 > /sys/class/gpio/export 2>/dev/null
    echo out > /sys/class/gpio/gpio57/direction
}

estop_active() {
    echo 0 > /sys/class/gpio/gpio57/value
}

estop_release() {
    echo 1 > /sys/class/gpio/gpio57/value
}
