
#ifndef _RACETRACKR
#define _RACETRACKR

#include "RaceTrackrSetup.h"
#include "Arduino.h"
#include "RaceTrackrSetup.h"
#include "RaceTrackrIDs.h"
#include "driver/gpio.h"
#include "driver/twai.h"
// #include <SPI.h>

#ifdef USE_STATUS_LED
#include <FastLED.h>
#endif

inline float degToRad(float degrees)
{
  return degrees * (PI / 180);
}

#define MIN(x, y) (((x) < (y)) ? (x) : (y))
#define MAX(x, y) (((x) > (y)) ? (x) : (y))

struct node_message
{
  twai_message_t message;
  bool messageReceivedFlag;
};

struct lora_message
{
  byte voltageT_A;
  byte voltageT_B;
  byte current_A;
  byte current_B;
  byte amphour_A;
  byte amphour_B;
  byte motorRPM;
  byte throttle;
  byte lat_A;
  byte lat_B;
  byte lat_C;
  byte lat_D;
  byte lon_A;
  byte lon_B;
  byte lon_C;
  byte lon_D;
};

enum CANBUS_MODE
{
  NORMAL,
  NO_ACK,
  LISTEN
};

enum COLOUR
{
  RED,
  BLUE,
  GREEN
};

union intToTwoBytes
{
  int16_t value;
  byte bytes[2];
};

union intToFourBytes
{
  int value;
  byte bytes[4];
};

union loraPacket
{
  lora_message message;
  byte packet[sizeof(lora_message)];
};

class Mosfet
{
public:
  Mosfet(int pin, int max_pwm = 255);
  void begin(int PWM_FREQ = 10000);
  void on();
  void onPWM(byte PWM);
  void off();
  void toggle();

private:
  int _pin;
  bool _state;
  int _max_pwm;
  int _channel;
};

#ifdef USE_BLUETOOTH
#include "BluetoothSerial.h"
inline BluetoothSerial eChookBluetooth;
inline bool BLUETOOTH_CONNECTED = false;
inline bool confirmRequestPending = false;

const char SPEED_ID = 's';
const char MOTOR_ID = 'm';
const char CURRENT_ID = 'i';
const char VOLTAGE_ID = 'v';
const char VOLTAGE_LOWER_ID = 'w';
const char THROTTLE_INPUT_ID = 't';
const char THROTTLE_ACTUAL_ID = 'd';
const char TEMP1_ID = 'a';
const char TEMP2_ID = 'b';
const char TEMP3_ID = 'c';
const char LAUNCH_MODE_ID = 'L';
const char CYCLE_VIEW_ID = 'C';
const char GEAR_RATIO_ID = 'r';
const char BRAKE_PRESSED_ID = 'B';

void BTConfirmRequestCallback(uint32_t numVal);
void BTAuthCompleteCallback(boolean success);
void callback(esp_spp_cb_event_t event, esp_spp_cb_param_t *param);

class RaceTrackreChook
{
public:
  RaceTrackreChook();
  bool begin(String deviceName = "RaceTrackr");
  void sendData(char identifier, float value);
  void sendData(char identifier, int value);
  void update();

private:
  unsigned int _millis_last_message;
};
#endif

#ifdef USE_SD
#include "FS.h"
#include "SD.h"
#include <SPI.h>

class RaceTrackrSD
{
public:
  RaceTrackrSD();
  bool begin();
  bool logData(float latitude, float longitude, int laps, float voltageT, float voltageA, float current, float amphour, int rpm, byte throttle);
  String sdMessage;
  uint8_t sdCardType;
  bool sdCardConnected;

private:
  void _writeFile(fs::FS &fs, const char *path, const char *message);
  void _appendFile(fs::FS &fs, const char *path, const char *message);
};

#endif

#ifdef USE_LORA
#include <LoRa.h>
class RaceTrackrLoRa
{
public:
  RaceTrackrLoRa();
  bool begin(byte localAddress, byte destinationAddress, int loraUpdateTime = 250);
  lora_message formatData(float batteryVoltage, float current, float amphour, int rpm, byte throttle, float latitude = 0, float longitude = 0);
  bool sendData(lora_message message);
  unsigned int millisLastLora;

private:
  byte _local;
  byte _destination;
  byte _id;
  loraPacket _loraPacket;
  int _lora_update_time;
};
#endif

#ifdef USE_GPS
#include <SoftwareSerial.h>
#include <TinyGPSPlus.h>
inline EspSoftwareSerial::UART gpsSerial;
inline TinyGPSPlus gps;
#define DEGTORAD(deg) (deg * 57.29577995)
struct point_t
{
  float x, y;
};
struct line_t
{
  point_t p0, p1;
};

class RaceTrackrGPS
{
public:
  RaceTrackrGPS();
  bool begin();
  void update();
  bool lapCheck(line_t track);
  void createStartFinish(double sy, double sx, int shdg);
  bool locationUpdated;
  unsigned int lastUpdateMillis;
  int satellites;
  float latitude;
  float longitude;
  float lastLatitude;
  float lastLongitude;

  float speed;
  float lastSpeed;

  float heading;
  float lastHeading;

  int timeHours;
  int timeMinutes;
  int timeSeconds;
  int timeMilliseconds;

  int laps;
  bool newLap;

  bool startLineSet = false;

  // Coordinate of start/finish location.
  point_t startPoint;
  // Startline endpoints.
  line_t startingLine;
  // Heading crossing start/finish.
  uint16_t startHeading;
  // Coordinates of current & previous GPS location.
  line_t track;
  const float LINE_WIDTH = 50.0f;
  const float LINE_WIDTH_2 = 25.0f;
  const float PROJECTION_DISTANCE = 100.0f;

private:
};
#endif

class RaceTrackrNode
{
public:
  RaceTrackrNode();
  bool begin(int MODE = NORMAL, bool FILTER = false, unsigned int MASK_ID = 0x000, unsigned int FILTER_ID = 0x000);
  bool requestAllData();
  bool dataPulse(int DELAY = 250);
  bool sendMessage(int id, byte message[8], unsigned int wait_for_transmission_delay = 0);
  bool sendRTRMessage(int id, unsigned int wait_for_transmission_delay = 0);
  node_message receivedMessage(unsigned int wait_for_receive_delay = 0);

#ifdef USE_STATUS_LED
  void ledBegin(int PIN = LED_PIN);
  void ledOn(int COLOUR, byte BRIGHTNESS = 255);
  void ledOff();
#endif
private:
  twai_general_config_t _g_config;
  twai_timing_config_t _t_config;
  twai_filter_config_t _f_config;
  byte _empty_message[8] = {0, 0, 0, 0, 0, 0, 0, 0};
  unsigned int _millisLastReqest;
  int _requestsReturned;
};

class Debouncer
{
private:
  static const uint8_t DEBOUNCED_STATE = 0b00000001; // Final returned calculated debounced state
  static const uint8_t UNSTABLE_STATE = 0b00000010;  // Actual last state value behind the scene
  static const uint8_t CHANGED_STATE = 0b00000100;   // The DEBOUNCED_STATE has changed since last update()

private:
  inline void changeState();
  inline void setStateFlag(const uint8_t flag) { state |= flag; }
  inline void unsetStateFlag(const uint8_t flag) { state &= ~flag; }
  inline void toggleStateFlag(const uint8_t flag) { state ^= flag; }
  inline bool getStateFlag(const uint8_t flag) const { return ((state & flag) != 0); }

public:
  Debouncer();
  void interval(uint16_t interval_millis);
  bool update();
  bool read() const;
  bool fell() const;
  bool rose() const;

public:
  bool changed() const { return getStateFlag(CHANGED_STATE); }
  unsigned long currentDuration() const;
  unsigned long previousDuration() const;

protected:
  void begin();
  virtual bool readCurrentState() = 0;
  unsigned long previous_millis;
  uint16_t interval_millis;
  uint8_t state;
  unsigned long stateChangeLastTime;
  unsigned long durationOfPreviousState;
};

class Bounce : public Debouncer
{
public:
  Bounce();
  void attach(int pin, int mode);
  void attach(int pin);

  Bounce(uint8_t pin, unsigned long interval_millis) : Bounce()
  {
    attach(pin);
    interval(interval_millis);
  }
  inline int getPin() const
  {
    return this->pin;
  };

protected:
  uint8_t pin;
  virtual bool readCurrentState() { return digitalRead(pin); }
  virtual void setPinMode(int pin, int mode)
  {
    pinMode(pin, mode);
  }
};

class Button : public Bounce
{
protected:
  bool stateForPressed = 1; //
public:
  Button() {}
  void setPressedState(bool state)
  {
    stateForPressed = state;
  }
  inline bool getPressedState() const
  {
    return stateForPressed;
  };
  inline bool isPressed() const
  {
    return read() == getPressedState();
  };
  inline bool pressed() const
  {
    return changed() && isPressed();
  };
  inline bool released() const
  {
    return changed() && !isPressed();
  };
};

class LatchedButton : public Bounce
{
protected:
  bool stateForPressed = 1;

private:
  volatile bool latchedState = false;

public:
  LatchedButton() {}
  void setPressedState(bool state)
  {
    stateForPressed = state;
  }
  inline bool getPressedState() const
  {
    return stateForPressed;
  };
  inline bool isPressed() const
  {
    return read() == getPressedState();
  };
  inline bool pressed() const
  {
    return changed() && isPressed();
  };
  inline bool released() const
  {
    return changed() && !isPressed();
  };
  inline bool toggledOn()
  {
    if (pressed() && !latchedState)
    {
      latchedState = true;
      return true;
    }
    else
    {
      return false;
    }
  }
  inline bool toggledOff()
  {
    if (pressed() && latchedState)
    {
      latchedState = false;
      return true;
    }
    else
    {
      return false;
    }
  }
};
#endif

#ifdef USE_EXTERN_ADS
#include <Adafruit_ADS1X15.h>

class RaceTrackrADS
{
public:
  RaceTrackrADS();
  bool begin(int SDA = SDA_PIN, int SCL = SCL_PIN);
  void setZeroValue(int CHANNEL, int SAMPLES);
  int readADS(int CHANNEL);
  float readADStoVolts(int CHANNEL);
  void update();

  float vccValue = 3.300;

  uint16_t adsValue[4] = {0, 0, 0, 0};
  uint16_t adsOffsetValue[4] = {0, 0, 0, 0};
  float adsVoltage[4] = {0.0, 0.0, 0.0, 0.0};
  float calibrationConstant[4] = {1.0, 1.0, 1.0, 1.0};

private:
  Adafruit_ADS1115 ads;
};
#endif

#ifdef USE_TEMP_SENSOR
#include <OneWire.h>
#include <DallasTemperature.h>

class RaceTrackrTemp
{
public:
  RaceTrackrTemp();
  void begin(int PIN = 10);
  float update();
  float tempC;
  OneWire *ow;
  DallasTemperature *tempSensor;

private:
  int _pin = 10;
};
#endif

#ifdef USE_RPM
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/portmacro.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/gpio.h"
#include "driver/pcnt.h"
#include <soc/pcnt_struct.h>
#include "esp_attr.h"
#include "esp_log.h"

static xQueueHandle pcnt_evt_queue;              // A queue to handle pulse counter events
static pcnt_isr_handle_t user_isr_handle = NULL; // user's ISR service handle

typedef struct
{
  int unit;                // the PCNT unit that originated an interrupt
  uint32_t status;         // information on the event type that caused the interrupt
  unsigned long timeStamp; // The time the event occured
} pcnt_evt_t;

static void IRAM_ATTR pcnt_intr_handler(void *arg)
{
  unsigned long currentMillis = millis(); // Time at instant ISR was called
  uint32_t intr_status = PCNT.int_st.val;
  int i = 0;
  pcnt_evt_t evt;
  portBASE_TYPE HPTaskAwoken = pdFALSE;
  if (intr_status & (BIT(i)))
  {
    evt.unit = 0;
    /* Save the PCNT event type that caused an interrupt
        to pass it to the main program */
    evt.status = PCNT.status_unit[i].val;
    evt.timeStamp = currentMillis;
    PCNT.int_clr.val = BIT(i);
    xQueueSendFromISR(pcnt_evt_queue, &evt, &HPTaskAwoken);
    if (HPTaskAwoken == pdTRUE)
    {
      portYIELD_FROM_ISR();
    }
  }
}

class RaceTrackrRPM
{
public:
  RaceTrackrRPM(int RPM_PIN, int SENSOR_COUNT = 1);
  int update();

private:
  int _trigger_pin;
  int _sensor_count;
};

static void initRPMPCNT(int TRIGGER_PIN)
{
  pcnt_unit_t PCNT_UNIT_0;
  int PCNT_INPUT_SIG_IO = TRIGGER_PIN;
  int PCNT_INPUT_CTRL_IO = PCNT_PIN_NOT_USED;
  pcnt_channel_t PCNT_CHANNEL = PCNT_CHANNEL_0;
  int PCNT_H_LIM_VAL = 28;
  int PCNT_L_LIM_VAL = -100;
  int PCNT_THRESH1_VAL = 100;
  int PCNT_THRESH0_VAL = -100;

  pcnt_config_t pcnt_config;
  // Set PCNT input signal and control GPIOs
  pcnt_config.pulse_gpio_num = TRIGGER_PIN;
  pcnt_config.ctrl_gpio_num = PCNT_PIN_NOT_USED;
  pcnt_config.channel = PCNT_CHANNEL_0;
  pcnt_config.unit = PCNT_UNIT_0;
  // What to do on the positive / negative edge of pulse input?
  pcnt_config.pos_mode = PCNT_COUNT_INC; // Count up on the positive edge
  pcnt_config.neg_mode = PCNT_COUNT_DIS; // Keep the counter value on the negative edge
  // What to do when control input is low or high?
  pcnt_config.lctrl_mode = PCNT_MODE_REVERSE; // Reverse counting direction if low
  pcnt_config.hctrl_mode = PCNT_MODE_KEEP;    // Keep the primary counter mode if high
  // Set the maximum and minimum limit values to watch
  pcnt_config.counter_h_lim = PCNT_H_LIM_VAL;
  pcnt_config.counter_l_lim = PCNT_L_LIM_VAL;

  /* Initialize PCNT unit */
  pcnt_unit_config(&pcnt_config);
  /* Configure and enable the input filter */
  pcnt_set_filter_value(PCNT_UNIT_0, 100);
  pcnt_filter_enable(PCNT_UNIT_0);

  /* Set threshold 0 and 1 values and enable events to watch */
  // pcnt_set_event_value(PCNT_UNIT, PCNT_EVT_THRES_1, PCNT_THRESH1_VAL);
  // pcnt_event_enable(PCNT_UNIT, PCNT_EVT_THRES_1);
  // pcnt_set_event_value(PCNT_UNIT, PCNT_EVT_THRES_0, PCNT_THRESH0_VAL);
  // pcnt_event_enable(PCNT_UNIT, PCNT_EVT_THRES_0);
  /* Enable events on zero, maximum and minimum limit values */
  // pcnt_event_enable(PCNT_UNIT, PCNT_EVT_ZERO);
  pcnt_event_enable(PCNT_UNIT_0, PCNT_EVT_H_LIM);
  pcnt_event_enable(PCNT_UNIT_0, PCNT_EVT_L_LIM);

  /* Initialize PCNT's counter */
  pcnt_counter_pause(PCNT_UNIT_0);
  pcnt_counter_clear(PCNT_UNIT_0);
  /* Register ISR handler and enable interrupts for PCNT unit */
  pcnt_isr_register(pcnt_intr_handler, NULL, 0, &user_isr_handle);
  pcnt_intr_enable(PCNT_UNIT_0);

  /* Everything is set up, now go to counting */
  pcnt_counter_resume(PCNT_UNIT_0);
}

#endif