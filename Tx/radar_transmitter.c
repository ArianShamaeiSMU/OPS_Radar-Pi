#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <errno.h>
#include <wiringPi.h>
#include <stdint.h>
#include <dirent.h>
#include <math.h>
#include <bluetooth/bluetooth.h>
#include <bluetooth/l2cap.h>

#define BT_DEST_ADDR "B8:27:EB:62:2E:90"  // Receiver's Bluetooth MAC address
#define L2CAP_PSM    0x1003               // Chosen L2CAP PSM for communication

// Smoothing and update parameters (Exponential Moving Average)
#define TX_UPDATE_INTERVAL 200   // Transmit filtered value every 200ms
#define ALPHA              0.7f // Smoothing factor (tunable)

// Helper to map raw sensor reading → true mph via linear interpolation
static inline float calibrate_raw(float raw) {
    const float rec1 =  9.00f, true1 = 25.25f;
    const float rec2 = 12.00f, true2 = 35.30f;
    float m = (true2 - true1) / (rec2 - rec1);
    float b = true1 - m * rec1;
    return m * raw + b;
}

// Forward declarations
int  open_serial(const char *port, speed_t baud);
void send_command(int fd, const char* cmd);
int  connect_bt_client(const char *dest_addr);

// Open a serial port with the given baud rate.
int open_serial(const char *port, speed_t baud) {
    int fd = open(port, O_RDWR | O_NOCTTY);
    if (fd == -1) {
        perror("open_serial: Unable to open port");
        return -1;
    }
    tcflush(fd, TCIFLUSH);
    struct termios options;
    tcgetattr(fd, &options);
    cfsetispeed(&options, baud);
    cfsetospeed(&options, baud);
    options.c_cflag |= (CLOCAL | CREAD);
    options.c_cflag &= ~PARENB;
    options.c_cflag &= ~CSTOPB;
    options.c_cflag &= ~CSIZE;
    options.c_cflag |= CS8;
    options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
    options.c_iflag &= ~(IXON | IXOFF | IXANY);
    options.c_oflag &= ~OPOST;
    tcsetattr(fd, TCSANOW, &options);
    return fd;
}

// Sends a command string (appending CR+LF) over the serial port.
void send_command(int fd, const char* cmd) {
    write(fd, cmd, strlen(cmd));
    write(fd, "\r\n", 2);
    delay(100);
}

// Establishes an L2CAP connection as a client.
int connect_bt_client(const char *dest_addr) {
    struct sockaddr_l2 addr = {0};
    int sock = socket(AF_BLUETOOTH, SOCK_SEQPACKET, BTPROTO_L2CAP);
    if (sock < 0) {
        perror("L2CAP socket");
        return -1;
    }
    addr.l2_family = AF_BLUETOOTH;
    addr.l2_psm    = htobs(L2CAP_PSM);
    str2ba(dest_addr, &addr.l2_bdaddr);
    if (connect(sock, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
        perror("L2CAP connect");
        close(sock);
        return -1;
    }
    printf("Connected to receiver over L2CAP.\n");
    return sock;
}

int main(void) {
    // Initialize wiringPi.
    if (wiringPiSetupGpio() == -1) {
        fprintf(stderr, "Error: wiringPi setup failed\n");
        return 1;
    }

    // Discover USB serial device under /dev/ttyACM*
    char usb_serial_port[PATH_MAX] = {0};
    DIR *d = opendir("/dev");
    if (d) {
        struct dirent *dir;
        while ((dir = readdir(d)) != NULL) {
            if (strncmp(dir->d_name, "ttyACM", 6) == 0) {
                snprintf(usb_serial_port, sizeof(usb_serial_port),
                         "/dev/%s", dir->d_name);
                break;
            }
        }
        closedir(d);
    }
    if (usb_serial_port[0] == '\0') {
        fprintf(stderr, "No USB serial device found at /dev/ttyACM*\n");
        exit(EXIT_FAILURE);
    }

    // Open at 57600 to reset and reconfigure sensor
    int serial_fd = open_serial(usb_serial_port, B57600);
    if (serial_fd < 0) {
        printf("Radar is NOT connected.\n");
        return 1;
    }
    printf("Radar connected on %s at 57600 baud.\n", usb_serial_port);

    // Reset & switch to 115200
    send_command(serial_fd, "P!"); delay(500);
    send_command(serial_fd, "I4"); delay(500);
    close(serial_fd);

    // Reopen at 115200
    serial_fd = open_serial(usb_serial_port, B115200);
    if (serial_fd < 0) {
        printf("Failed to reopen serial port at 115200.\n");
        return 1;
    }
    printf("Reopened serial port at 115200 baud.\n");

    // Configure sensor
    send_command(serial_fd, "Z!");	 // full reset
    delay(1000);
    send_command(serial_fd, "US");       // mph units
    send_command(serial_fd, "R+");       // inbound only
    send_command(serial_fd, "R>0.2");    // ignore <0.2 mph
    send_command(serial_fd, "BZ");       // zero output when no target
    send_command(serial_fd, "S<");       // 512-sample buffer
    send_command(serial_fd, "OS");       // speed reporting
    send_command(serial_fd, "K+");       // peak averaging
    send_command(serial_fd, "^/+45.0");  // cosine correction
    send_command(serial_fd, "A!");       // save
    send_command(serial_fd, "Oj");       // plain numeric output

    // Bluetooth connection
    int bt_sock = -1;
    while ((bt_sock = connect_bt_client(BT_DEST_ADDR)) < 0) {
        printf("Retrying L2CAP Bluetooth connection in 5 seconds...\n");
        sleep(5);
    }

    // EMA filter state
    float filtered_speed = 0.0f;
    int   first_sample   = 1;
    unsigned long last_tx_time = millis();

    // Serial read buffers
    char serialBuffer[256] = {0};
    int  bufferPos = 0;
    char readBuffer[64];
    char txBuf[32];

    // Main loop
    while (1) {
        int n = read(serial_fd, readBuffer, sizeof(readBuffer) - 1);
        if (n > 0) {
            readBuffer[n] = '\0';
            if (bufferPos + n < (int)sizeof(serialBuffer)) {
                strncat(serialBuffer, readBuffer, n);
                bufferPos += n;
            } else {
                bufferPos = 0;
                serialBuffer[0] = '\0';
            }

            // Process complete lines
            char *newline = strchr(serialBuffer, '\n');
            while (newline) {
                int lineLen = newline - serialBuffer;
                char line[128] = {0};
                if (lineLen < (int)sizeof(line)) {
                    strncpy(line, serialBuffer, lineLen);
                    line[lineLen] = '\0';
                }

                // Parse raw speed
                float speed_val = atof(line);
                float raw = fabsf(speed_val);

                // 1) Calibrate raw reading → true mph
                float cal = calibrate_raw(raw);

                // 2) EMA filter on calibrated value
                if (first_sample) {
                    filtered_speed = cal;
                    first_sample = 0;
                } else {
                    filtered_speed = ALPHA * cal + (1.0f - ALPHA) * filtered_speed;
                }

                // Shift buffer
                int remainingLen = strlen(newline + 1);
                memmove(serialBuffer, newline + 1, remainingLen + 1);
                bufferPos = remainingLen;
                newline = strchr(serialBuffer, '\n');
            }
        }

        // Transmit every TX_UPDATE_INTERVAL
        unsigned long now = millis();
        if (!first_sample && now - last_tx_time >= TX_UPDATE_INTERVAL) {
            int roundedSpeed = (int)ceilf(filtered_speed);
            sprintf(txBuf, "%d\n", roundedSpeed);
            if (write(bt_sock, txBuf, strlen(txBuf)) < 0) {
                perror("L2CAP write");
                close(bt_sock);
                while ((bt_sock = connect_bt_client(BT_DEST_ADDR)) < 0) {
                    printf("Retrying L2CAP Bluetooth connection in 5 seconds...\n");
                    sleep(5);
                }
            }
            printf("Filtered speed: %d mph\n", roundedSpeed);
            last_tx_time = now;
        }

        usleep(5000);  // 5 ms delay
    }

    // Cleanup (never reached)
    close(bt_sock);
    close(serial_fd);
    return 0;
}
