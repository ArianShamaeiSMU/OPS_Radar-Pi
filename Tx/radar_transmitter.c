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
#include <signal.h>

// Logging macros
#define LOG_LEVEL 2  // 0=none, 1=errors, 2=warnings, 3=info, 4=debug
#define LOG_ERROR(...) if (LOG_LEVEL >= 1) { fprintf(stderr, "ERROR: "); fprintf(stderr, __VA_ARGS__); fprintf(stderr, "\n"); }
#define LOG_WARN(...)  if (LOG_LEVEL >= 2) { fprintf(stderr, "WARN:  "); fprintf(stderr, __VA_ARGS__); fprintf(stderr, "\n"); }
#define LOG_INFO(...)  if (LOG_LEVEL >= 3) { fprintf(stdout, "INFO:  "); fprintf(stdout, __VA_ARGS__); fprintf(stdout, "\n"); }
#define LOG_DEBUG(...) if (LOG_LEVEL >= 4) { fprintf(stdout, "DEBUG: "); fprintf(stdout, __VA_ARGS__); fprintf(stdout, "\n"); }

// Configuration - can be overridden by command line args
#define DEFAULT_BT_DEST_ADDR "B8:27:EB:62:2E:90"  // Receiver's Bluetooth MAC address
#define DEFAULT_L2CAP_PSM    0x1003               // Chosen L2CAP PSM for communication
#define DEFAULT_ALPHA        0.7f                 // Smoothing factor (tunable)
#define TX_UPDATE_INTERVAL   200                  // Transmit filtered value every 200ms
#define COMM_TIMEOUT_MS      5000                 // 5 seconds timeout for serial comm
#define RECONNECT_DELAY_SEC  5                    // Bluetooth reconnect delay

// Global variables for cleanup
int serial_fd = -1;
int bt_sock = -1;
char *bt_dest_addr = NULL;
int l2cap_psm = DEFAULT_L2CAP_PSM;
float alpha = DEFAULT_ALPHA;

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
void cleanup_and_exit(int sig);
void parse_args(int argc, char *argv[]);
void initialize_radar(int fd);

// Open a serial port with the given baud rate.
int open_serial(const char *port, speed_t baud) {
    int fd = open(port, O_RDWR | O_NOCTTY);
    if (fd == -1) {
        LOG_ERROR("Unable to open port %s: %s", port, strerror(errno));
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
    if (fd < 0) return;
    
    LOG_DEBUG("Sending command: %s", cmd);
    write(fd, cmd, strlen(cmd));
    write(fd, "\r\n", 2);
    delay(100);
}

// Establishes an L2CAP connection as a client.
int connect_bt_client(const char *dest_addr) {
    struct sockaddr_l2 addr = {0};
    int sock = socket(AF_BLUETOOTH, SOCK_SEQPACKET, BTPROTO_L2CAP);
    if (sock < 0) {
        LOG_ERROR("L2CAP socket creation failed: %s", strerror(errno));
        return -1;
    }
    addr.l2_family = AF_BLUETOOTH;
    addr.l2_psm    = htobs(l2cap_psm);
    str2ba(dest_addr, &addr.l2_bdaddr);
    
    LOG_INFO("Attempting to connect to %s on PSM 0x%04X", dest_addr, l2cap_psm);
    
    if (connect(sock, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
        LOG_ERROR("L2CAP connect failed: %s", strerror(errno));
        close(sock);
        return -1;
    }
    
    LOG_INFO("Connected to receiver over L2CAP");
    return sock;
}

// Handle clean shutdown
void cleanup_and_exit(int sig) {
    LOG_INFO("Shutting down gracefully...");
    
    if (serial_fd >= 0) {
        send_command(serial_fd, "Z!"); // Reset sensor
        close(serial_fd);
    }
    
    if (bt_sock >= 0) {
        close(bt_sock);
    }
    
    exit(0);
}

// Parse command line arguments
void parse_args(int argc, char *argv[]) {
    bt_dest_addr = strdup(DEFAULT_BT_DEST_ADDR);
    
    for (int i = 1; i < argc; i++) {
        if (strcmp(argv[i], "--alpha") == 0 && i+1 < argc) {
            alpha = atof(argv[i+1]);
            if (alpha <= 0.0f || alpha > 1.0f) {
                LOG_WARN("Alpha must be between 0 and 1, using default: %f", DEFAULT_ALPHA);
                alpha = DEFAULT_ALPHA;
            }
            i++;
        } else if (strcmp(argv[i], "--bt-addr") == 0 && i+1 < argc) {
            free(bt_dest_addr);
            bt_dest_addr = strdup(argv[i+1]);
            i++;
        } else if (strcmp(argv[i], "--psm") == 0 && i+1 < argc) {
            l2cap_psm = (int)strtol(argv[i+1], NULL, 0);
            i++;
        } else if (strcmp(argv[i], "--help") == 0) {
            printf("Usage: %s [options]\n\n", argv[0]);
            printf("Options:\n");
            printf("  --alpha VALUE      Set smoothing factor (0-1, default: %.1f)\n", DEFAULT_ALPHA);
            printf("  --bt-addr ADDR     Set Bluetooth MAC address of receiver\n");
            printf("  --psm VALUE        Set L2CAP PSM value (default: 0x%04X)\n", DEFAULT_L2CAP_PSM);
            printf("  --help             Show this help message\n");
            exit(0);
        }
    }
    
    LOG_INFO("Using configuration:");
    LOG_INFO("  Alpha: %.2f", alpha);
    LOG_INFO("  BT Address: %s", bt_dest_addr);
    LOG_INFO("  L2CAP PSM: 0x%04X", l2cap_psm);
}

// Find the radar device
char* find_radar_device() {
    static char usb_serial_port[PATH_MAX] = {0};
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
        LOG_ERROR("No USB serial device found at /dev/ttyACM*");
        return NULL;
    }
    
    return usb_serial_port;
}

// Initialize and configure the radar sensor
void initialize_radar(int fd) {
    if (fd < 0) return;
    
    LOG_INFO("Configuring radar sensor");
    
    // Reset & configure
    send_command(fd, "Z!");       // full reset
    delay(1000);
    send_command(fd, "US");       // mph units
    send_command(fd, "R+");       // inbound only
    send_command(fd, "R>0.2");    // ignore <0.2 mph
    send_command(fd, "BZ");       // zero output when no target
    send_command(fd, "S<");       // 512-sample buffer
    send_command(fd, "OS");       // speed reporting
    send_command(fd, "K+");       // peak averaging
    send_command(fd, "^/+45.0");  // cosine correction
    send_command(fd, "A!");       // save
    send_command(fd, "Oj");       // plain numeric output
    
    LOG_INFO("Radar configuration complete");
}

int main(int argc, char *argv[]) {
    // Set up signal handlers for clean shutdown
    signal(SIGINT, cleanup_and_exit);
    signal(SIGTERM, cleanup_and_exit);
    
    // Parse command line arguments
    parse_args(argc, argv);
    
    // Initialize wiringPi.
    if (wiringPiSetupGpio() == -1) {
        LOG_ERROR("wiringPi setup failed");
        return 1;
    }

    // Discover USB serial device
    char *usb_serial_port = find_radar_device();
    if (usb_serial_port == NULL) {
        exit(EXIT_FAILURE);
    }

    // Open at 57600 to reset and reconfigure sensor
    serial_fd = open_serial(usb_serial_port, B57600);
    if (serial_fd < 0) {
        LOG_ERROR("Radar is NOT connected");
        return 1;
    }
    LOG_INFO("Radar connected on %s at 57600 baud", usb_serial_port);

    // Reset & switch to 115200
    send_command(serial_fd, "P!"); delay(500);
    send_command(serial_fd, "I4"); delay(500);
    close(serial_fd);

    // Reopen at 115200
    serial_fd = open_serial(usb_serial_port, B115200);
    if (serial_fd < 0) {
        LOG_ERROR("Failed to reopen serial port at 115200");
        return 1;
    }
    LOG_INFO("Reopened serial port at 115200 baud");

    // Configure sensor
    initialize_radar(serial_fd);

    // Bluetooth connection loop - retry until connected
    while ((bt_sock = connect_bt_client(bt_dest_addr)) < 0) {
        LOG_WARN("Retrying L2CAP Bluetooth connection in %d seconds...", RECONNECT_DELAY_SEC);
        sleep(RECONNECT_DELAY_SEC);
    }

    // EMA filter state
    float filtered_speed = 0.0f;
    int   first_sample   = 1;
    unsigned long last_tx_time = millis();
    unsigned long last_valid_read = millis();

    // Serial read buffers
    char serialBuffer[256] = {0};
    int  bufferPos = 0;
    char readBuffer[64];
    char txBuf[32];

    // Main loop
    while (1) {
        int n = read(serial_fd, readBuffer, sizeof(readBuffer) - 1);
        
        if (n > 0) {
            last_valid_read = millis();
            readBuffer[n] = '\0';
            
            if (bufferPos + n < (int)sizeof(serialBuffer)) {
                strncat(serialBuffer, readBuffer, n);
                bufferPos += n;
            } else {
                // Buffer overflow protection
                LOG_WARN("Serial buffer overflow, resetting buffer");
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

                // Calibrate raw reading → true mph
                float cal = calibrate_raw(raw);

                // EMA filter on calibrated value
                if (first_sample) {
                    filtered_speed = cal;
                    first_sample = 0;
                } else {
                    filtered_speed = alpha * cal + (1.0f - alpha) * filtered_speed;
                }

                // Shift buffer
                int remainingLen = strlen(newline + 1);
                memmove(serialBuffer, newline + 1, remainingLen + 1);
                bufferPos = remainingLen;
                newline = strchr(serialBuffer, '\n');
            }
        } else {
            // Check for serial timeout
            if (millis() - last_valid_read > COMM_TIMEOUT_MS) {
                LOG_WARN("Serial communication timeout. Resetting...");
                close(serial_fd);
                
                // Re-open and re-initialize serial port
                serial_fd = open_serial(usb_serial_port, B115200);
                if (serial_fd >= 0) {
                    initialize_radar(serial_fd);
                    last_valid_read = millis();
                } else {
                    LOG_ERROR("Failed to reopen serial port after timeout");
                    sleep(1);
                }
            }
        }

        // Transmit every TX_UPDATE_INTERVAL
        unsigned long now = millis();
        if (!first_sample && now - last_tx_time >= TX_UPDATE_INTERVAL) {
            int roundedSpeed = (int)ceilf(filtered_speed);
            sprintf(txBuf, "%d\n", roundedSpeed);
            
            // Check for valid Bluetooth connection
            if (bt_sock < 0) {
                LOG_INFO("Trying to reconnect Bluetooth...");
                bt_sock = connect_bt_client(bt_dest_addr);
                if (bt_sock < 0) {
                    LOG_WARN("Reconnection failed, will retry later");
                }
            } else {
		// Send data over Bluetooth
                if (write(bt_sock, txBuf, strlen(txBuf)) < 0) {
                    LOG_ERROR("L2CAP write failed: %s", strerror(errno));
                    close(bt_sock);
                    bt_sock = -1;
                } else {
                    LOG_DEBUG("Filtered speed: %d mph", roundedSpeed);
                }
            }
            
            last_tx_time = now;
        }

        usleep(5000);  // 5 ms delay for responsiveness
    }

    // Cleanup (never reached due to infinite loop)
    cleanup_and_exit(0);
    return 0;
}
