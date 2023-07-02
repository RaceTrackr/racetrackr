#include "Arduino.h"
#include "driver/gpio.h"
#include "RaceTrackr.h"
#include "driver/twai.h"
#include <SoftwareSerial.h>

// #define DEBUG

Mosfet::Mosfet(int pin)
{
  _pin = pin;
  _state = false;
}

void Mosfet::begin()
{
  pinMode(_pin, OUTPUT);
  digitalWrite(_pin, LOW);
}

void Mosfet::on()
{
  _state = true;
  digitalWrite(_pin, HIGH);
}

void Mosfet::onPWM(byte PWM)
{
  _state = true;
  analogWrite(_pin, PWM);
}

void Mosfet::off()
{
  _state = false;
  digitalWrite(_pin, LOW);
}

void Mosfet::toggle()
{
  _state = !_state;
  digitalWrite(_pin, _state);
}

RaceTrackrNode::RaceTrackrNode()
{
}

bool RaceTrackrNode::begin(int MODE, bool FILTER, unsigned int maskId, unsigned int filterId)
{
  switch (MODE)
  {
  case NORMAL:
    _g_config = TWAI_GENERAL_CONFIG_DEFAULT(CAN_TX_PIN, CAN_RX_PIN, TWAI_MODE_NORMAL);
    break;
  case NO_ACK:
    _g_config = TWAI_GENERAL_CONFIG_DEFAULT(CAN_TX_PIN, CAN_RX_PIN, TWAI_MODE_NO_ACK);
    break;
  case LISTEN:
    _g_config = TWAI_GENERAL_CONFIG_DEFAULT(CAN_TX_PIN, CAN_RX_PIN, TWAI_MODE_LISTEN_ONLY);
  }
  _t_config = TWAI_TIMING_CONFIG_500KBITS();

  if (FILTER)
  {
    _f_config = {.acceptance_code = (filterId << 21), .acceptance_mask = ~(maskId << 21), .single_filter = true};
  }
  else
  {
    _f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();
  }

  if (twai_driver_install(&_g_config, &_t_config, &_f_config) == ESP_OK)
  {
    if (twai_start() == ESP_OK)
    {
      return true;
    }
    else
    {
      return false;
    }
  }
  else
  {
    return false;
  }
}

bool RaceTrackrNode::sendMessage(int id, byte message[8], unsigned int wait_for_transmission_delay)
{
  twai_message_t _message;
  _message.identifier = id;
  _message.data_length_code = 8;
  _message.rtr = 0;
  _message.extd = 0;
  _message.ss = 1;
  for (int i = 0; i < _message.data_length_code; i++)
  {
    _message.data[i] = message[i];
  };
  if (twai_transmit(&_message, pdMS_TO_TICKS(wait_for_transmission_delay)) == ESP_OK)
  {
    return true;
  }
  else
  {
    return false;
  }
}

bool RaceTrackrNode::sendRTRMessage(int id, unsigned int wait_for_transmission_delay)
{
  twai_message_t _message;
  _message.identifier = id;
  _message.rtr = 1;
  _message.ss = 1;
  if (twai_transmit(&_message, pdMS_TO_TICKS(wait_for_transmission_delay)) == ESP_OK)
  {
    return true;
  }
  else
  {
    return false;
  }
}

node_message RaceTrackrNode::receivedMessage(unsigned int wait_for_receive_delay)
{
  twai_message_t _message;
  node_message receivedMessage;
#ifdef DEBUG
  printf("Waiting to receive message\n");
#endif
  if (twai_receive(&_message, pdMS_TO_TICKS(wait_for_receive_delay)) == ESP_OK)
  {
    receivedMessage.messageReceivedFlag = true;
#ifdef DEBUG
    printf("Message Received\n");
#endif
  }
  else
  {
    receivedMessage.messageReceivedFlag = false;
#ifdef DEBUG
    printf("No Message Received\n");
#endif
  }
  receivedMessage.message = _message;
  return receivedMessage;
}

#ifdef USE_STATUS_LED
CRGB led[1];
void RaceTrackrNode::ledBegin(int PIN)
{
  FastLED.addLeds<SK6822, LED_PIN, GRB>(led, 1);
  FastLED.setMaxPowerInVoltsAndMilliamps(3.3, 50);
}

void RaceTrackrNode::ledOn(int COLOUR, byte BRIGHTNESS)
{
  switch (COLOUR)
  {
  case RED:
    led[0] = CRGB::Red;
    break;
  case GREEN:
    led[0] = CRGB::Green;
    break;
  case BLUE:
    led[0] = CRGB::Blue;
    break;
  }
  FastLED.setBrightness(BRIGHTNESS);
  FastLED.show();
}

void RaceTrackrNode::ledOff()
{
  FastLED.clear(true);
}
#endif

int twoBytesToInt(byte BYTE_1, byte BYTE_2)
{
  union twoBytesToIntUnion
  {
    int integer;
    byte byte_values[2];
  };
  twoBytesToIntUnion value;
  value.byte_values[0] = BYTE_1;
  value.byte_values[0] = BYTE_2;
  return value.integer;
}

Debouncer::Debouncer() : previous_millis(0), interval_millis(10), state(0) {}

void Debouncer::interval(uint16_t interval_millis)
{
  this->interval_millis = interval_millis;
}

void Debouncer::begin()
{
  state = 0;
  if (readCurrentState())
  {
    setStateFlag(DEBOUNCED_STATE | UNSTABLE_STATE);
  }
}

bool Debouncer::update()
{
  unsetStateFlag(CHANGED_STATE);
  bool currentState = readCurrentState();
  if (currentState != getStateFlag(UNSTABLE_STATE))
  {
    previous_millis = millis();
    toggleStateFlag(UNSTABLE_STATE);
  }
  else if (millis() - previous_millis >= interval_millis)
  {
    // We have passed the threshold time, so the input is now stable
    // If it is different from last state, set the STATE_CHANGED flag
    if (currentState != getStateFlag(DEBOUNCED_STATE))
    {
      previous_millis = millis();
      changeState();
    }
  }
  return changed();
}

unsigned long Debouncer::previousDuration() const
{
  return durationOfPreviousState;
}

unsigned long Debouncer::currentDuration() const
{
  return (millis() - stateChangeLastTime);
}

inline void Debouncer::changeState()
{
  toggleStateFlag(DEBOUNCED_STATE);
  setStateFlag(CHANGED_STATE);
  durationOfPreviousState = millis() - stateChangeLastTime;
  stateChangeLastTime = millis();
}

bool Debouncer::read() const
{
  return getStateFlag(DEBOUNCED_STATE);
}

bool Debouncer::rose() const
{
  return getStateFlag(DEBOUNCED_STATE) && getStateFlag(CHANGED_STATE);
}

bool Debouncer::fell() const
{
  return !getStateFlag(DEBOUNCED_STATE) && getStateFlag(CHANGED_STATE);
}

Bounce::Bounce()
    : pin(0)
{
}

void Bounce::attach(int pin)
{
  this->pin = pin;

  // SET INITIAL STATE
  begin();
}

void Bounce::attach(int pin, int mode)
{
  setPinMode(pin, mode);
  this->attach(pin);
}

#ifdef BLUETOOTH_COMMUNICATION
RaceTrackreChook::RaceTrackreChook()
{
}

bool RaceTrackreChook::begin(String deviceName)
{
  _connected = false;
  BT.enableSSP();
  BT.begin(deviceName);
  BT.register_callback(bluetoothCallback);
  _millis_last_message = 0;
  return true;
}

void RaceTrackreChook::sendData(char identifier, float value)
{
  if (_connected)
  {
    byte dataByte1;
    byte dataByte2;

    if (value == 0)
    {
      dataByte1 = 0xFF;
      dataByte2 = 0xFF;
    }
    else if (value <= 127)
    {
      int integer;
      int decimal;
      float tempDecimal;

      integer = (int)value;
      tempDecimal = (value - (float)integer) * 100;
      decimal = (int)tempDecimal;

      dataByte1 = (byte)integer;
      dataByte2 = (byte)decimal;

      if (decimal == 0)
      {
        dataByte2 = 0xFF;
      }

      if (integer == 0)
      {
        dataByte1 = 0xFF;
      }
    }
    else
    {
      int tens;
      int hundreds;

      hundreds = (int)(value / 100);
      tens = value - hundreds * 100;

      dataByte1 = (byte)hundreds;
      dataByte1 += 128;
      dataByte2 = (byte)tens;

      if (tens == 0)
      {
        dataByte2 = 0xFF;
      }

      if (hundreds == 0)
      {
        dataByte1 = 0xFF;
      }
    }
    BT.write(123);
    BT.write(identifier);
    BT.write(dataByte1);
    BT.write(dataByte2);
    BT.write(125);
    _millis_last_message = millis();
  }
}

void RaceTrackreChook::sendData(char identifier, int value)
{
  if (_connected)
  {
    byte dataByte1;
    byte dataByte2;

    if (value == 0)
    {
      dataByte1 = 0xFF;
      dataByte2 = 0xFF;
    }
    else if (value <= 127)
    {

      dataByte1 = (byte)value;
      dataByte2 = 0xFF;
    }
    else
    {
      int tens;
      int hundreds;

      hundreds = (int)(value / 100);
      tens = value - hundreds * 100;

      dataByte1 = (byte)hundreds;
      dataByte1 += 128;
      dataByte2 = (byte)tens;

      if (tens == 0)
      {
        dataByte2 = 0xFF;
      }

      if (hundreds == 0)
      {
        dataByte1 = 0xFF;
      }
    }

    BT.write(123);
    BT.write(identifier);
    BT.write(dataByte1);
    BT.write(dataByte2);
    BT.write(125);
    _millis_last_message = millis();
  }
}

void RaceTrackreChook::update()
{
  if (millis() - _millis_last_message > 1500)
  {
    _millis_last_message = millis();
    BT.write(123);
    BT.write('x');
    BT.write(0);
    BT.write(0);
    BT.write(125);
  }
}

void RaceTrackreChook::bluetoothCallback(esp_spp_cb_event_t event, esp_spp_cb_param_t *param)
{
  if (event == ESP_SPP_SRV_OPEN_EVT)
  {
    _connected = true;
#ifdef DEBUG
    printf("BLUETOOTH CONNECTED\n");
#endif
  }
  else if (event == ESP_SPP_CLOSE_EVT)
  {
    _connected = false;
#ifdef DEBUG
    printf("BLUETOOTH DISCONNECTED\n");
#endif
  }
}
#endif

#ifdef LORA_COMMUNICATION
RaceTrackrLoRa::RaceTrackrLoRa()
{
}

bool RaceTrackrLoRa::begin(byte localAddress, byte destinationAddress, int loraUpdateTime)
{
  _local = localAddress;
  _destination = destinationAddress;
  _lora_update_time = loraUpdateTime;
  _millis_last_lora = 0;
  _id = 0;
  SPI.begin(LORA_SCK, LORA_MISO, LORA_MOSI, LORA_CS);
  LoRa.setPins(LORA_CS, LORA_RST, LORA_DIO);
  if(!LoRa.begin(BAND)){
    return false;
  }
  return true;
}

lora_message RaceTrackrLoRa::formatData(float batteryVoltage, float current, float amphour, int rpm, byte throttle, float latitude, float longitude)
{
  lora_message message;
  int16_t batteryVoltageInt = batteryVoltage * 100;
  int16_t currentInt = current * 100;
  int16_t amphourInt = amphour * 100;
  byte rpmByte = rpm / 10;
  int latitudeInt = latitude * 10000;
  int longitudeInt = longitude * 10000;

  intToTwoBytes batteryVoltageUnion;
  batteryVoltageUnion.value = batteryVoltageInt;
  message.voltageT_A = batteryVoltageUnion.bytes[0];
  message.voltageT_B = batteryVoltageUnion.bytes[1];

  intToTwoBytes currentUnion;
  currentUnion.value = currentInt;
  message.current_A = currentUnion.bytes[0];
  message.current_B = currentUnion.bytes[1];

  intToTwoBytes amphourUnion;
  amphourUnion.value = amphourInt;
  message.amphour_A = amphourUnion.bytes[0];
  message.amphour_B = amphourUnion.bytes[1];

  message.motorRPM = rpmByte;
  message.throttle = throttle;

  intToFourBytes latitudeUnion;
  latitudeUnion.value = latitudeInt;
  message.lat_A = latitudeUnion.bytes[0];
  message.lat_B = latitudeUnion.bytes[1];
  message.lat_C = latitudeUnion.bytes[2];
  message.lat_D = latitudeUnion.bytes[3];

  intToFourBytes longitudeUnion;
  longitudeUnion.value = longitudeInt;
  message.lon_A = longitudeUnion.bytes[0];
  message.lon_B = longitudeUnion.bytes[1];
  message.lon_C = longitudeUnion.bytes[2];
  message.lon_D = longitudeUnion.bytes[3];
  return message;
}

void RaceTrackrLoRa::sendData(lora_message message)
{
  if (millis() - _millis_last_lora > _lora_update_time)
  {
    #ifdef DEBUG
    printf("LoRa sent!\n");
    #endif
    _millis_last_lora = millis();
    _loraPacket.message = message;
    LoRa.beginPacket();
    LoRa.write(_destination);
    LoRa.write(_local);
    LoRa.write(_id);
    LoRa.write(sizeof(message));
    LoRa.write(_loraPacket.packet, sizeof(message));
    LoRa.endPacket();
    _id++;
  }
}
#endif