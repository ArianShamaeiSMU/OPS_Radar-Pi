#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <errno.h>
#include <wiringPi.h>
#include <stdint.h>
#include <sys/socket.h>
#include <bluetooth/bluetooth.h>
#include <bluetooth/l2cap.h>
#include <signal.h>

// Logging macros
#define LOG_LEVEL 2  // 0=none, 1=errors, 2=warnings, 3=info, 4=debug
#define LOG_ERROR(...) if (LOG_LEVEL >= 1) { fprintf(stderr, "ERROR: "); fprintf(stderr, __VA_ARGS__); fprintf(stderr, "\n"); }
#define LOG_WARN(...)  if (LOG_LEVEL >= 2) { fprintf(stderr, "WARN:  "); fprintf(stderr, __VA_ARGS__); fprintf(stderr, "\n"); }
#define LOG_INFO(...)  if (LOG_LEVEL >= 3) { fprintf(stdout, "INFO:  "); fprintf(stdout, __VA_ARGS__); fprintf(stdout, "\n"); }
#define LOG_DEBUG(...) if (LOG_LEVEL >= 4) { fprintf(stdout, "DEBUG: "); fprintf(stdout, __VA_ARGS__); fprintf(stdout, "\n"); }

// ---------------------------------------------------------------------------
// TM1637 7-Segment Display Pin Assignments (using BCM numbering)
// ---------------------------------------------------------------------------
#define TM1637_CLK 20
#define TM1637_DIO 21

// TM1637 commands
#define TM1637_CMD_DATA    0x40  // Data command (auto-increment mode)
#define TM1637_CMD_ADDRESS 0xC0  // Address command (starting address 0)
#define TM1637_CMD_CONTROL 0x88  // Display control command (on + brightness)

// 7-segment encoding for digits 0-9 (0x00 means blank)
static const uint8_t digitToSegment[] = {
    0x3F, // 0
    0x06, // 1
    0x5B, // 2
    0x4F, // 3
    0x66, // 4
    0x6D, // 5
    0x7D, // 6
    0x07, // 7
    0x7F, // 8
    0x6F  // 9
};

// Additional segment patterns
#define SEG_E 0x79
#define SEG_M 0x54
#define SEG_P 0x73
#define SEG_H 0x74

// Configuration
#define L2CAP_PSM 0x1003
#define RX_BUF_SIZE 64
#define DATA_TIMEOUT_MS 5000       // 5 seconds timeout for data
#define DISPLAY_DIM_TIMEOUT_MS 60000 // Dim display after 1 minute of inactivity

// Global variables for cleanup
int bt_server = -1;
int client_sock = -1;

// Forward declarations for TM1637 functions:
void tm1637_start(void);
void tm1637_stop(void);
int tm1637_write_byte(uint8_t data);
void tm1637_set_display(uint8_t segments[4], uint8_t brightness);
void display_integer(int value);
void display_speed_with_unit(int speed);
void display_test_pattern(void);
void show_all_digits_0_to_9(void);
void display_error(void);
void display_no_data(void);
void cleanup_and_exit(int sig);

// Handle clean shutdown
void cleanup_and_exit(int sig) {
    LOG_INFO("Shutting down gracefully...");
    
    // Turn off display
    uint8_t segments[4] = {0, 0, 0, 0};
    tm1637_set_display(segments, 0);
    
    // Close sockets
    if (client_sock >= 0) close(client_sock);
    if (bt_server >= 0) close(bt_server);
    
    exit(0);
}

// TM1637 bit-banging functions:
void tm1637_start(void) {
    pinMode(TM1637_DIO, OUTPUT);
    digitalWrite(TM1637_DIO, HIGH);
    digitalWrite(TM1637_CLK, HIGH);
    delayMicroseconds(2);
    digitalWrite(TM1637_DIO, LOW);
}

void tm1637_stop(void) {
    pinMode(TM1637_DIO, OUTPUT);
    digitalWrite(TM1637_CLK, LOW);
    delayMicroseconds(2);
    digitalWrite(TM1637_DIO, LOW);
    digitalWrite(TM1637_CLK, HIGH);
    delayMicroseconds(2);
    digitalWrite(TM1637_DIO, HIGH);
}

int tm1637_write_byte(uint8_t data) {
    for (int i = 0; i < 8; i++) {
        digitalWrite(TM1637_CLK, LOW);
        digitalWrite(TM1637_DIO, (data & 0x01) ? HIGH : LOW);
        delayMicroseconds(3);
        digitalWrite(TM1637_CLK, HIGH);
        delayMicroseconds(3);
        data >>= 1;
    }
    digitalWrite(TM1637_CLK, LOW);
    pinMode(TM1637_DIO, INPUT);
    digitalWrite(TM1637_DIO, HIGH);
    digitalWrite(TM1637_CLK, HIGH);
    delayMicroseconds(3);
    int ack = digitalRead(TM1637_DIO);
    digitalWrite(TM1637_CLK, LOW);
    pinMode(TM1637_DIO, OUTPUT);
    return ack;
}

void tm1637_set_display(uint8_t segments[4], uint8_t brightness) {
    tm1637_start();
    tm1637_write_byte(TM1637_CMD_DATA);
    tm1637_stop();

    tm1637_start();
    tm1637_write_byte(TM1637_CMD_ADDRESS);
    for (int i = 0; i < 4; i++) {
        tm1637_write_byte(segments[i]);
    }
    tm1637_stop();

    tm1637_start();
    tm1637_write_byte(TM1637_CMD_CONTROL | (brightness & 0x07));
    tm1637_stop();
}

void display_integer(int value) {
    char buf[10];
    sprintf(buf, "%d", value);
    int len = strlen(buf);
    uint8_t segments[4] = {0, 0, 0, 0}; // Blanks for unused positions.
    int startPos = 4 - len;  // Right-align the number.
    for (int i = 0; i < len && i < 4; i++) {
        int digit = buf[i] - '0';
        segments[startPos + i] = digitToSegment[digit];
    }
    tm1637_set_display(segments, 7);
}

void display_speed_with_unit(int speed) {
    uint8_t segments[4] = {0, 0, 0, 0};
    
    if (speed < 10) {
        // Single digit + "MPH"
        segments[0] = digitToSegment[speed];
        segments[1] = SEG_M; // m
        segments[2] = SEG_P; // p
        segments[3] = SEG_H; // h
    } else if (speed < 100) {
        // Double digit + "PH" 
        segments[0] = digitToSegment[speed / 10];
        segments[1] = digitToSegment[speed % 10];
        segments[2] = SEG_P; // p
        segments[3] = SEG_H; // h
    } else {
        // Just the number, no room for units
        display_integer(speed);
        return;
    }
    
    tm1637_set_display(segments, 7);
}

void display_test_pattern(void) {
    char pattern[] = "1234";
    int len = strlen(pattern);
    uint8_t segments[4] = {0, 0, 0, 0};
    int startPos = 4 - len;
    for (int i = 0; i < len; i++) {
        int digit = pattern[i] - '0';
        segments[startPos + i] = digitToSegment[digit];
    }
    tm1637_set_display(segments, 7);
}

void show_all_digits_0_to_9(void) {
    for (int d = 0; d < 10; d++) {
        uint8_t seg = digitToSegment[d];
        uint8_t segments[4] = {seg, seg, seg, seg};
        tm1637_set_display(segments, 7);
        delay(500);
    }
}

void display_error(void) {
    uint8_t segments[4] = {SEG_E, SEG_E, SEG_E, SEG_E};
    tm1637_set_display(segments, 7);
}

void display_no_data(void) {
    uint8_t segments[4] = {0x5E, 0x5F, 0x78, 0x80};
    tm1637_set_display(segments, 7);
}

// Set up a Bluetooth L2CAP server socket.
int setup_bt_server(void) {
    struct sockaddr_l2 loc_addr = {0};
    int sock;
    sock = socket(AF_BLUETOOTH, SOCK_SEQPACKET, BTPROTO_L2CAP);
    if(sock < 0) {
        LOG_ERROR("L2CAP socket creation failed: %s", strerror(errno));
        exit(1);
    }
    loc_addr.l2_family = AF_BLUETOOTH;
    loc_addr.l2_bdaddr = *BDADDR_ANY;
    loc_addr.l2_psm = htobs(L2CAP_PSM);
    if(bind(sock, (struct sockaddr *)&loc_addr, sizeof(loc_addr)) < 0) {
        LOG_ERROR("L2CAP bind failed: %s", strerror(errno));
        exit(1);
    }
    listen(sock, 1);
    return sock;
}

// Accept a Bluetooth connection
int accept_bt_connection(int server_sock) {
    struct sockaddr_l2 rem_addr = {0};
    socklen_t opt = sizeof(rem_addr);
    int sock = accept(server_sock, (struct sockaddr *)&rem_addr, &opt);
    
    if(sock >= 0) {
        char bt_addr_str[18] = {0};
        ba2str(&rem_addr.l2_bdaddr, bt_addr_str);
        LOG_INFO("Accepted connection from %s", bt_addr_str);
    }
    
    return sock;
}

// Main receiver program.
int main(void) {
    // Set up signal handlers for clean shutdown
    signal(SIGINT, cleanup_and_exit);
    signal(SIGTERM, cleanup_and_exit);
    
    // Initialize wiringPi for TM1637.
    if(wiringPiSetupGpio() == -1) {
        LOG_ERROR("wiringPi setup failed");
        return 1;
    }
    pinMode(TM1637_CLK, OUTPUT);
    pinMode(TM1637_DIO, OUTPUT);
    digitalWrite(TM1637_CLK, HIGH);
    digitalWrite(TM1637_DIO, HIGH);
    
    // Startup display test.
    show_all_digits_0_to_9();
    display_test_pattern();
    delay(2000);
    
    // Setup Bluetooth L2CAP server.
    bt_server = setup_bt_server();
    LOG_INFO("Waiting for Bluetooth connection on L2CAP PSM 0x%04X...", L2CAP_PSM);
    
    // Buffer for receiving Bluetooth data.
    char rxBuffer[RX_BUF_SIZE];
    int rxPos = 0;
    memset(rxBuffer, 0, sizeof(rxBuffer));
    
    int speed_val = 0;
    int display_mode = 0;  // 0 = standard, 1 = with unit
    unsigned long last_data_time = millis();
    unsigned long last_activity = millis();
    uint8_t current_brightness = 7;
    
    // Initial connection
    client_sock = accept_bt_connection(bt_server);
    if(client_sock < 0) {
        LOG_ERROR("L2CAP accept failed: %s", strerror(errno));
        display_error();
        delay(2000);
    }
    
    // Main loop: read incoming data until newline, then update the display.
    while(1) {
        // Check connection status
        if(client_sock < 0) {
            display_no_data();
            LOG_INFO("Waiting for new connection...");
            client_sock = accept_bt_connection(bt_server);
            if(client_sock < 0) {
                delay(1000);  // Retry delay
                continue;
            }
            rxPos = 0;
            memset(rxBuffer, 0, sizeof(rxBuffer));
        }
        
        // Read data from connection
        int bytes_read = read(client_sock, rxBuffer + rxPos, RX_BUF_SIZE - rxPos - 1);
        
        // Check for disconnection or error
        if(bytes_read <= 0) {
            if(bytes_read < 0) {
                LOG_WARN("L2CAP read error: %s", strerror(errno));
            } else {
                LOG_INFO("Connection closed by peer");
            }
            close(client_sock);
            client_sock = -1;
            continue;
        }
        
        // Process data
        rxPos += bytes_read;
        rxBuffer[rxPos] = '\0';
        last_data_time = millis();
        last_activity = millis();
        current_brightness = 7;  // Full brightness when active
            
        // Process complete lines
        char *newline;
        while((newline = strchr(rxBuffer, '\n')) != NULL) {
            *newline = '\0';  // Terminate the current line.
            speed_val = atoi(rxBuffer);
            LOG_DEBUG("Received speed: %d mph", speed_val);
                
            // Display in current mode
            if(display_mode == 0) {
                display_integer(speed_val);
            } else {
                display_speed_with_unit(speed_val);
            }
                
            // Shift remaining data in buffer
            int remaining = strlen(newline + 1);
            memmove(rxBuffer, newline + 1, remaining + 1);
            rxPos = remaining;
        }
        
        // Check for data timeout
        unsigned long now = millis();
        if(now - last_data_time > DATA_TIMEOUT_MS) {
            display_no_data();
        }
        
        // Check for display dimming
        if(now - last_activity > DISPLAY_DIM_TIMEOUT_MS) {
            current_brightness = 1;  // Dim
        }
        
        // Toggle display mode every 10 seconds
        if(now % 10000 < 20 && now != 0) {  // Toggle briefly at 10s boundaries
            display_mode = 1 - display_mode;  // Toggle between 0 and 1
        }
        
        usleep(5000); // 5ms delay for responsiveness.
    }
    
    // Should never reach here, but cleanup in case
    close(client_sock);
    close(bt_server);
    return 0;
}
