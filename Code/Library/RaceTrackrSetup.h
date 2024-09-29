#include "driver/gpio.h"

// Define the adapter board used in the node

#define M5STAMP_S3
// #define M5STAMP_C3
// #define M5STAMP_PICO
// #define ESP32_DEV
// #define ESP32_C3_XIAO
// #define WT32SC01
// #define ESP32_TTGO_LORA_v2_1_6
// #define TTGO_TBEAM
// #define WAVESHARE_7

// Define the controller board used in the node

// #define USE_EXTERN_ADS
// #define USE_TEMP_SENSOR
// #define USE_RPM

// Define the interface board used in the node
// #define RACETRACKR_MOSFET
// #define RACETRACKR_LORA
// #define RACETRACKR_CUSTOM

#ifdef M5STAMP_S3
#define USE_STATUS_LED
#define CAN_RX_PIN GPIO_NUM_5
#define CAN_TX_PIN GPIO_NUM_4
#define LED_PIN 21 // RGB LED
#define SDA_PIN 13
#define SCL_PIN 15

#define G0 0 // Shared pin with button
#define G1 1
#define G2 2
#define G3 3
#define G4 44 // Shared pin with UART RX
#define G5 46
#define G6 6
#define G7 7
#define G8 8
#define G9 9
#define G10 10
#define G11 11
#define G12 12
#define G13 13 // Shared pin with I2C SDA
#define G14 14
#define G15 15 // Shared pin with I2C SCL
#define G16 39
#define G17 40
#define G18 41
#define G19 42
#endif

#ifdef M5STAMP_C3
#endif

#ifdef M5STAMP_PICO
#endif

#ifdef ESP32_DEV
#define USE_BLUETOOTH
#define CAN_RX_PIN GPIO_NUM_5
#define CAN_TX_PIN GPIO_NUM_4
#define GPS_RX_PIN 12
#define GPS_TX_PIN 13

#endif

#ifdef ESP32_C3_XIAO
#endif

#ifdef WT32SC01
#define CAN_RX_PIN GPIO_NUM_33
#define CAN_TX_PIN GPIO_NUM_32
#define SDA_PIN 18
#define SCL_PIN 19
#endif

#ifdef ESP32_TTGO_LORA_v2_1_6
#define USE_BLUETOOTH
// #define USE_LORA
// #define USE_GPS
// #define USE_SD
#define GPS_BAUD 115200
#define GPS_UPDATE_RATE 250

#define BAND 433E6

#define CAN_RX_PIN GPIO_NUM_35
#define CAN_TX_PIN GPIO_NUM_4

#define GPS_RX_PIN 34
#define GPS_TX_PIN 12

#define SDA_PIN 21
#define SCL_PIN 22

#define SD_CS 13
#define SD_MISO 2
#define SD_MOSI 15
#define SD_SCK 14

#define LORA_CS 18
#define LORA_MISO 19
#define LORA_MOSI 27
#define LORA_SCK 5
#define LORA_DIO 26
#define LORA_RST 23

#endif

#ifdef TTGO_TBEAM

#define BLUETOOTH_COMMUNICATION
#define LORA_COMMUNICATION
#define LORA_433
#define GPS

#define CAN_RX_PIN GPIO_NUM_25
#define CAN_TX_PIN GPIO_NUM_13

#define GPS_TX 34
#define GPS_RX 12

#define LORA_CS 18
#define LORA_MISO 19
#define LORA_MOSI 27
#define LORA_SCK 5
// #define LORA_DIO
#define LORA_RST 14
#endif

#ifdef WAVESHARE_7
#define CAN_RX_PIN GPIO_NUM_19
#define CAN_TX_PIN GPIO_NUM_20
#define GPIO_OUTPUT_PIN_SEL ((1ULL << 19) | (1ULL << 20))
#endif