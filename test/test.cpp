#include <string.h>
#include <stdio.h>
#include <unistd.h>
#include <errno.h>
#include <termios.h>

constexpr char KOBUKI_DEVICE_FN[] = "/dev/kobuki";

int main() {
    FILE* file = fopen(KOBUKI_DEVICE_FN, "rw");
    if (!file)
    {
        fprintf(stderr, "Could not open %s : %s\n", KOBUKI_DEVICE_FN, strerror(errno));
        return -1;
    }

    struct termios tty;
    if (tcgetattr(fileno(file), &tty))
    {
        fprintf(stderr, "Could not retrieve teletype attributes\n");
        return -1;
    }

    cfsetospeed(&tty, B115200);
    cfsetispeed(&tty, B115200);

    tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8; // 8 bit chars
    tty.c_cflag &= ~CSTOPB; // 1 stop bit
    tty.c_cflag &= ~PARENB; // no parity bit

    tty.c_cc[VMIN] = 64; // Need to read 64 bytes at least before returning
    tty.c_cc[VTIME] = 0; // Wait forever to get VMIN bytes

    if (tcsetattr(fileno(file), TCSANOW, &tty))
    {
        fprintf(stderr, "Could not set new teletype attributes\n");
        return -1;
    }

    char data[1024] = {0};
    const int n = fread(data, 1, sizeof(data), file);
    fprintf(stderr, "Read %d bytes\n", n);
    for (int i = 0; i < n; ++i) putchar(data[i]);

    fclose(file);

    return 0;
}
