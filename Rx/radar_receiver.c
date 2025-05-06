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

#define SEG_E 0x79

// Forward declarations for TM1637 functions:
void tm1637_start(void);
void tm1637_stop(void);
int tm1637_write_byte(uint8_t data);
void tm1637_set_display(uint8_t segments[4], uint8_t brightness);
void display_integer(int value);
void display_test_pattern(void);
void show_all_digits_0_to_9(void);
void display_error(void);
void display_no_data(void);

// Bluetooth L2CAP PSM and buffer size.
#define L2CAP_PSM 0x1003
#define RX_BUF_SIZE 64

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
        perror("L2CAP socket");
        exit(1);
    }
    loc_addr.l2_family = AF_BLUETOOTH;
    loc_addr.l2_bdaddr = *BDADDR_ANY;
    loc_addr.l2_psm = htobs(L2CAP_PSM);
    if(bind(sock, (struct sockaddr *)&loc_addr, sizeof(loc_addr)) < 0) {
        perror("L2CAP bind");
        exit(1);
    }
    listen(sock, 1);
    return sock;
}

// Main receiver program.
int main(void) {
    // Initialize wiringPi for TM1637.
    if(wiringPiSetupGpio() == -1) {
        fprintf(stderr, "Error: wiringPi setup failed\n");
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
    int bt_server = setup_bt_server();
    printf("Waiting for Bluetooth connection on L2CAP PSM 0x%04X...\n", L2CAP_PSM);
    struct sockaddr_l2 rem_addr = {0};
    socklen_t opt = sizeof(rem_addr);
    int client_sock = accept(bt_server, (struct sockaddr *)&rem_addr, &opt);
    if(client_sock < 0) {
        perror("L2CAP accept");
        return 1;
    }
    char bt_addr_str[18] = {0};
    ba2str(&rem_addr.l2_bdaddr, bt_addr_str);
    printf("Accepted connection from %s\n", bt_addr_str);
    
    // Buffer for receiving Bluetooth data.
    char rxBuffer[RX_BUF_SIZE];
    int rxPos = 0;
    memset(rxBuffer, 0, sizeof(rxBuffer));
    
    int speed_val = 0;
    
    // Main loop: read incoming data until newline, then update the display.
    while(1) {
        int bytes_read = read(client_sock, rxBuffer + rxPos, RX_BUF_SIZE - rxPos - 1);
        if(bytes_read > 0) {
            rxPos += bytes_read;
            rxBuffer[rxPos] = '\0';
            char *newline;
            while((newline = strchr(rxBuffer, '\n')) != NULL) {
                *newline = '\0';  // Terminate the current line.
                speed_val = atoi(rxBuffer);
                printf("Received speed: %d mph\n", speed_val);
                display_integer(speed_val);
                int remaining = strlen(newline + 1);
                memmove(rxBuffer, newline + 1, remaining + 1);
                rxPos = remaining;
            }
        } else {
            display_no_data();
        }
        usleep(5000); // 5ms delay for responsiveness.
    }
    
    close(client_sock);
    close(bt_server);
    return 0;
}
