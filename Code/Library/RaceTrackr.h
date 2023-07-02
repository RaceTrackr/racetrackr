
#ifndef _RACETRACKR
#define _RACETRACKR

#include "RaceTrackrSetup.h"
#include "Arduino.h"
#include "RaceTrackrSetup.h"
#include "driver/gpio.h"
#include "driver/twai.h"
#include <SPI.h>

#ifdef USE_STATUS_LED
#include <FastLED.h>
#endif

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
  Mosfet(int pin);
  void begin();
  void on();
  void onPWM(byte PWM);
  void off();
  void toggle();

private:
  int _pin;
  bool _state;
};

#ifdef BLUETOOTH_COMMUNICATION
#include "BluetoothSerial.h"

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

class RaceTrackreChook
{
public:
  RaceTrackreChook();
  bool begin(String deviceName = "RaceTrackr");
  void sendData(char identifier, float value);
  void sendData(char identifier, int value);
  void update();

private:
  BluetoothSerial BT;
  inline static void bluetoothCallback(esp_spp_cb_event_t event, esp_spp_cb_param_t *param);
  inline static bool _connected = false;
  int _millis_last_message;
};
#endif

#ifdef LORA_COMMUNICATION
#include <LoRa.h>
#ifdef LORA_433
#define BAND 433E6
#endif
#ifdef LORA_868
#define BAND 868E6
#endif
class RaceTrackrLoRa
{
public:
  RaceTrackrLoRa();
  bool begin(byte localAddress, byte destinationAddress, int loraUpdateTime = 250);
  lora_message formatData(float batteryVoltage, float current, float amphour, int rpm, byte throttle, float latitude = 0, float longitude = 0);
  void sendData(lora_message message);

private:
  byte _local;
  byte _destination;
  unsigned int _id;
  loraPacket _loraPacket;
  int _millis_last_lora;
  int _lora_update_time;
};

#endif

class RaceTrackrNode
{
public:
  RaceTrackrNode();
  bool begin(int MODE = NORMAL, bool FILTER = false, unsigned int maskId = 0x000, unsigned int filterId = 0x000);
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
};

extern int twoBytesToInt(byte BYTE_1, byte BYTE_2);

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