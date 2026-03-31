#include <fcntl.h>
#include <unistd.h>
#include <string.h>
#include <stdio.h>

#define ESTOP_GPIO     57
#define ESTOP_GPIO_STR "57"

static int gpio_write_file(const char *path, const char *val) {
    int fd = open(path, O_WRONLY);
    if (fd < 0) return -1;
    write(fd, val, strlen(val));
    close(fd);
    return 0;
}

int roboclaw_estop_init(void) {
    // Export if not already
    if (access("/sys/class/gpio/gpio57", F_OK) != 0)
        gpio_write_file("/sys/class/gpio/export", ESTOP_GPIO_STR);
    usleep(50000);
    gpio_write_file("/sys/class/gpio/gpio57/direction", "out");
    gpio_write_file("/sys/class/gpio/gpio57/value",     "1");
    return 0;
}

void roboclaw_estop_assert(void) {
    gpio_write_file("/sys/class/gpio/gpio57/value", "0");
}

void roboclaw_estop_deassert(void) {
    gpio_write_file("/sys/class/gpio/gpio57/value", "1");
}

int roboclaw_estop_get(void) {
    char buf[4] = {0};
    int fd = open("/sys/class/gpio/gpio57/value", O_RDONLY);
    if (fd < 0) return -1;
    read(fd, buf, 1);
    close(fd);
    return buf[0] == '1' ? 1 : 0;
}