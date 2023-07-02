#include "driver/gpio.h"

// Define the adapter board used in the node

// #define M5STAMP_C3
#define M5STAMP_S3
// #define M5STAMP_PICO
// #define ESP32_DEV
// #define ESP32_C3_XIAO
// #define WT32SC01
// #define ESP32_TTGO_LORA_v2_1_6

// Define the controller board used in the node

#define RACETRACKR_CONTROLLER
// #define MCP2515

// Define the interface board used in the node
#define RACETRACKR_MOSFET
// #define RACETRACKR_LORA
// #define RACETRACKR_CUSTOM

#ifdef M5STAMP_C3
#endif

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

#ifdef M5STAMP_PICO
#endif

#ifdef ESP32_DEV
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
#define BLUETOOTH_COMMUNICATION
#define LORA_COMMUNICATION
#define LORA_433
// #define LORA_868
#define CAN_RX_PIN GPIO_NUM_5
#define CAN_TX_PIN GPIO_NUM_4

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