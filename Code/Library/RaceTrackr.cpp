#include "Arduino.h"
#include "driver/gpio.h"
#include "RaceTrackr.h"
#include "driver/twai.h"

#define DEBUG

// MOSFET BOARD CONTROL

int _number_pwm_channels = 0;

Mosfet::Mosfet(int pin, int max_pwm)
{
  _pin = pin;
  _state = false;
  _max_pwm = max_pwm;
}

void Mosfet::begin(int PWM_FREQ)
{
  _channel = _number_pwm_channels;
  _number_pwm_channels++;
  printf("CHANNEL FOR MOSFET: %i\n", _channel);
  ledcSetup(_channel, PWM_FREQ, 8);
  // pinMode(_pin, OUTPUT);
  // digitalWrite(_pin, LOW);
  ledcAttachPin(_pin, _channel);
}

void Mosfet::on()
{
  _state = true;
  // analogWrite(_pin, 255);
  // digitalWrite(_pin, HIGH);
  ledcWrite(_channel, 255);
}

void Mosfet::onPWM(byte PWM)
{
  _state = true;
  byte pwm_value = PWM;
  // byte pwm_value = constrain(PWM, 0, _max_pwm);
  // analogWrite(_pin, pwm_value);
  ledcWrite(_channel, pwm_value);
}

void Mosfet::off()
{
  _state = false;
  // analogWrite(_pin, 0);
  // digitalWrite(_pin, LOW);
  ledcWrite(_channel, 0);
}

void Mosfet::toggle()
{
  _state = !_state;
  digitalWrite(_pin, _state);
}

RaceTrackrNode::RaceTrackrNode()
{
}

bool RaceTrackrNode::begin(int MODE, bool FILTER, unsigned int MASK_ID, unsigned int FILTER_ID)
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
    _f_config = {.acceptance_code = (FILTER_ID << 21), .acceptance_mask = ~(MASK_ID << 21), .single_filter = true};
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

bool RaceTrackrNode::requestAllData()
{
  if (!sendRTRMessage(ID_DATA_PULSE, 0))
  {
    return false;
  }
  _millisLastReqest = millis();
  return true;
}

bool RaceTrackrNode::dataPulse(int DELAY)
{
  // if (millis() - _millisLastReqest > DELAY)
  // {
  //   if (!sendRTRMessage(ID_DATA_PULSE, 0))
  //   {
  //     return false;
  //   }
  //   _millisLastReqest = millis();
  //   return true;
  // }
  // return false;
  if (millis() - _millisLastReqest > DELAY)
  {
    sendMessage(ID_DATA_PULSE, _empty_message);
    _millisLastReqest = millis();
    return true;
  }
  return false;
}

node_message RaceTrackrNode::receivedMessage(unsigned int wait_for_receive_delay)
{
  twai_message_t _message;
  node_message receivedMessage;
#ifdef DEBUG
  // printf("Waiting to receive message\n");
#endif
  if (twai_receive(&_message, pdMS_TO_TICKS(wait_for_receive_delay)) == ESP_OK)
  {
    receivedMessage.messageReceivedFlag = true;
#ifdef DEBUG
    // printf("Message Received\n");
#endif
  }
  else
  {
    receivedMessage.messageReceivedFlag = false;
#ifdef DEBUG
    // printf("No Message Received\n");
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

#ifdef USE_BLUETOOTH

void callback(esp_spp_cb_event_t event, esp_spp_cb_param_t *param)
{
  if (event == ESP_SPP_SRV_OPEN_EVT)
  {
    BLUETOOTH_CONNECTED = true;
    printf("BLUETOOTH CONNECTED\n");
  }
  else if (event == ESP_SPP_CLOSE_EVT)
  {
    BLUETOOTH_CONNECTED = false;
    printf("BLUETOOTH DISCONNECTED\n");
  }
}

void BTConfirmRequestCallback(uint32_t numVal)
{
  confirmRequestPending = true;
  Serial.println(numVal);
}

void BTAuthCompleteCallback(boolean success)
{
  confirmRequestPending = false;
  if (success)
  {
    Serial.println("Pairing success!!");
  }
  else
  {
    Serial.println("Pairing failed, rejected by user!!");
  }
}

RaceTrackreChook::RaceTrackreChook()
{
}

bool RaceTrackreChook::begin(String deviceName)
{
  // eChookBluetooth.enableSSP();
  eChookBluetooth.onConfirmRequest(BTConfirmRequestCallback);
  eChookBluetooth.onAuthComplete(BTAuthCompleteCallback);
  eChookBluetooth.begin(deviceName);
  eChookBluetooth.register_callback(callback);
  return true;
}

void RaceTrackreChook::sendData(char identifier, float value)
{
  if (BLUETOOTH_CONNECTED)
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
    eChookBluetooth.write(123);
    eChookBluetooth.write(identifier);
    eChookBluetooth.write(dataByte1);
    eChookBluetooth.write(dataByte2);
    eChookBluetooth.write(125);
  }
  _millis_last_message = millis();
}

void RaceTrackreChook::sendData(char identifier, int value)
{
  if (BLUETOOTH_CONNECTED)
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

    eChookBluetooth.write(123);
    eChookBluetooth.write(identifier);
    eChookBluetooth.write(dataByte1);
    eChookBluetooth.write(dataByte2);
    eChookBluetooth.write(125);
    _millis_last_message = millis();
  }
}

void RaceTrackreChook::update()
{
  if (millis() - _millis_last_message > 1500)
  {
    _millis_last_message = millis();
    eChookBluetooth.write(123);
    eChookBluetooth.write('x');
    eChookBluetooth.write(0);
    eChookBluetooth.write(0);
    eChookBluetooth.write(125);
  }
}
#endif

#ifdef USE_SD
RaceTrackrSD::RaceTrackrSD()
{
}
bool RaceTrackrSD::begin()
{
  SPI.begin(SD_SCK, SD_MISO, SD_MOSI, SD_CS);
  SD.begin(SD_CS);
  if (!SD.begin(SD_CS))
  {
    return false;
  }
  sdCardType = SD.cardType();
  if (sdCardType == CARD_NONE)
  {
    sdCardConnected = false;
    return false;
  }
  if (!SD.begin(SD_CS))
  {
    return false;
  }
  File file = SD.open("/data.txt");
  if (!file)
  {
    Serial.println("File doens't exist");
    Serial.println("Creating file...");
    _writeFile(SD, "/data.txt", "Millis,Latitude,Longitude,VoltageT,VoltageA,Current,Amphour,RPM,Throttle \r\n");
  }
  file.close();
}

void RaceTrackrSD::_appendFile(fs::FS &fs, const char *path, const char *message)
{
  File file = fs.open(path, FILE_APPEND);
  if (!file)
  {
    return;
  }
  if (file.print(message))
  {
  }
  file.close();
}

bool RaceTrackrSD::logData(float latitude, float longitude, int laps, float voltageT, float voltageA, float current, float amphour, int rpm, byte throttle)
{
  unsigned int time = millis();
  String dataMessage = String(time) + "," + String(latitude, 5) + "," + String(longitude, 5) + "," + String(laps) + "," + String(voltageT, 2) + "," + String(current, 2) + "," + String(amphour, 2) + "," + String(rpm) + "," + String(throttle) + "\r\n";
  _appendFile(SD, "/data.txt", dataMessage.c_str());
}

void RaceTrackrSD::_writeFile(fs::FS &fs, const char *path, const char *message)
{
  File file = fs.open(path, FILE_WRITE);
  if (!file)
  {
    return;
  }
  if (file.print(message))
  {
  }
  file.close();
}
#endif

#ifdef USE_LORA
RaceTrackrLoRa::RaceTrackrLoRa()
{
}

bool RaceTrackrLoRa::begin(byte localAddress, byte destinationAddress, int loraUpdateTime)
{
  _local = localAddress;
  _destination = destinationAddress;
  _lora_update_time = loraUpdateTime;
  millisLastLora = 0;
  _id = 0;
  SPI.begin(LORA_SCK, LORA_MISO, LORA_MOSI, LORA_CS);
#ifdef TTGO_TBEAM
  LoRa.setPins(LORA_CS, LORA_RST);
#else
  LoRa.setPins(LORA_CS, LORA_RST, LORA_DIO);
#endif
  if (!LoRa.begin(BAND))
  {
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

bool RaceTrackrLoRa::sendData(lora_message message)
{
  if (millis() - millisLastLora > _lora_update_time)
  {
#ifdef DEBUG
    printf("LoRa sent!\n");
#endif
    millisLastLora = millis();
    _loraPacket.message = message;
    LoRa.beginPacket();
    LoRa.write(_destination);
    LoRa.write(_local);
    LoRa.write(_id);
    LoRa.write(sizeof(message));
    LoRa.write(_loraPacket.packet, sizeof(message));
    LoRa.endPacket();
    _id++;
    if (_id > 255)
    {
      _id = 0;
    }
    return true;
  }
  return false;
}
#endif

#ifdef USE_GPS

RaceTrackrGPS::RaceTrackrGPS()
{
}

bool RaceTrackrGPS::begin()
{
  gpsSerial.begin(GPS_BAUD, EspSoftwareSerial::SWSERIAL_8N1, GPS_RX_PIN, GPS_TX_PIN);
  latitude = 0;
  longitude = 0;
  speed = 0;
  heading = 0;
  return true;
}

void RaceTrackrGPS::update()
{
  while (gpsSerial.available() > 0)
  {
    gps.encode(gpsSerial.read());
    printf("GPS\n");
  }
  if (gps.location.isUpdated())
  {
    printf("NEW LOCATION INFO\n");
    track.p0.x = track.p1.x;
    track.p0.y = track.p1.y;
    latitude = gps.location.lat();
    longitude = gps.location.lng();
    track.p1.x = latitude;
    track.p1.y = longitude;
    // printf("LAT: %6f, LON: %6f\n", gps.location.lat(), gps.location.lng());
    locationUpdated = true;
    lastUpdateMillis = millis();
    // lastLatitude = latitude;
    // latitude = gps.location.lat();
    // lastLongitude = longitude;
    // longitude = gps.location.lng();
    // lastHeading = heading;
    heading = gps.course.deg();
    satellites = gps.satellites.value();
#ifdef DEBUG
    printf("LAT: %f     ", latitude);
    printf("LON: %f     ", longitude);
    printf("HDG: %f     ", heading);
    printf("SAT: %i     ", satellites);
    printf("SPD: %f \n", speed);
#endif
  }
}

#define USE_HOME_LOCATION

void RaceTrackrGPS::createStartFinish(double sy, double sx, int shdg)
{
#ifdef USE_HOME_LOCATION
  startPoint.x = 54.952347;
  startPoint.y = -1.790566;
  int startHeading = 250;

#else

#endif
  // Define startline.
  // StartLine((float)startPoint.x, (float)startPoint.y, (float)startHeading);
  float tx, ty; // Projected track coordinates.
  float m, b;   // Slope & y intercept.

  // Project racetrack along current heading.
  tx = sx + (float(PROJECTION_DISTANCE) / 10000.0) * cos(degToRad(shdg));
  ty = sy + (float(PROJECTION_DISTANCE) / 10000.0) * sin(degToRad(shdg));
  // Projected racetrack slope & y-intercept.
  m = (sy - ty) / (sx - tx);
  b = sy - (m * sx);

  // Construct perpendicular (startline) slope & y-intercept.
  m = -1.0f / m;
  b = sy - (m * sx);

  // Define endpoints of the perpendicular.
  tx = sx + (LINE_WIDTH_2 / 200000.0); // Note: tx re-used as a temporary value here.
  startingLine.p0.y = (m * tx + b);
  startingLine.p0.x = tx;
  tx -= (LINE_WIDTH / 200000.0);
  startingLine.p1.y = (m * tx + b);
  startingLine.p1.x = tx;
  track.p0.x = startPoint.x;
  track.p0.y = startPoint.y;
  startLineSet = true;
  printf("Start/Finish Created at:\nLatA: %f\tLonA: %f\tLatB: %f\tLonB: %f\n", startingLine.p0.y, startingLine.p0.x, startingLine.p1.y, startingLine.p1.x);
}

// void RaceTrackrGPS::createStartFinish(float sy, float sx, float shdg)

// {
//   //https://ucexperiment.wordpress.com/2012/03/31/predicitive-gps-lap-timer-pc-prototype-version/
//   // float tx, ty;
//   // float m, b;

//   // // tx = xlongitude + 100 * cos(degToRad(xheading));
//   // // ty = xlatitude + 100 * sin(degToRad(xheading));

//   // // m = (xlatitude - ty) / (xlongitude - tx);
//   // // b = xlatitude - (m * xlongitude);

//   // // m = -1.0f / m;
//   // // b = xlatitude - (m * xlongitude);

//   // // tx = xlongitude + 100;

//   // // startLineLon1 = tx;
//   // // startLineLat1 = (m * tx + b);

//   // // tx -= 100;
//   // // startLineLon2 = tx;
//   // // startLineLat2 = (m * tx + b);

//   // // startLineSet = true;
//   // // printf("Start/Finish Created at:\nLatA: %f\tLonA: %f\tLatB: %f\tLonB: %f\n",startLineLat1,startLineLon1,startLineLat2,startLineLon2);
// 	// float tx, ty; // Projected track coordinates.
// 	// float m, b;   // Slope & y intercept.

// 	// // Project racetrack along current heading.
// 	// tx = sx + PROJECTION_DISTANCE * cos(degToRad(shdg));
// 	// ty = sy + PROJECTION_DISTANCE * sin(degToRad(shdg));
// 	// // Projected racetrack slope & y-intercept.
// 	// m = (sy - ty) / (sx - tx);
// 	// b = sy - (m * sx);

// 	// // Construct perpendicular (startline) slope & y-intercept.
// 	// m = -1.0f / m;
// 	// b = sy - (m * sx);

// 	// // Define endpoints of the perpendicular.
// 	// tx = sx + LINE_WIDTH_2; // Note: tx re-used as a temporary value here.
// 	// // startingLine.p0.y = (m * tx + b);
// 	// // startingLine.p0.x = tx;
//   // startLineLat1 = (m * tx + b);
//   // startLineLon1 = tx;

// 	// tx -= LINE_WIDTH;
// 	// // startingLine.p1.y = (m * tx + b);
// 	// // startingLine.p1.x = tx;
//   // startLineLat2 = (m * tx + b);
//   // startLineLon2 = tx;

//   // printf("Start/Finish Created at:\nLatA: %f\tLonA: %f\tLatB: %f\tLonB: %f\n",startLineLat1,startLineLon1,startLineLat2,startLineLon2);

// }

bool RaceTrackrGPS::lapCheck(line_t track)
{
  float z;
  int16_t s1, s2, s3, s4;

  // Quick rejection test.
  if (!(MAX(startingLine.p0.x, startingLine.p1.x) >= MIN(track.p0.x, track.p1.x) &&
        MAX(track.p0.x, track.p1.x) >= MIN(startingLine.p0.x, startingLine.p1.x) &&
        MAX(startingLine.p0.y, startingLine.p1.y) >= MIN(track.p0.y, track.p1.y) &&
        MAX(track.p0.y, track.p1.y) >= MIN(startingLine.p0.y, startingLine.p1.y)))
    return false;

  // Straddle tests.
  if ((z = ((track.p0.x - startingLine.p0.x) * (startingLine.p1.y - startingLine.p0.y)) - ((track.p0.y - startingLine.p0.y) * (startingLine.p1.x - startingLine.p0.x))) < 0.0f)
    s1 = -1; // Counterclockwise.
  else if (z > 0.0f)
    s1 = 1; // Clockwise.
  else
    s1 = 0; // Collinear.

  if ((z = ((track.p1.x - startingLine.p0.x) * (startingLine.p1.y - startingLine.p0.y)) - ((track.p1.y - startingLine.p0.y) * (startingLine.p1.x - startingLine.p0.x))) < 0.0f)
    s2 = -1;
  else if (z > 0.0f)
    s2 = 1;
  else
    s2 = 0;

  if ((z = ((startingLine.p0.x - track.p0.x) * (track.p1.y - track.p0.y)) - ((startingLine.p0.y - track.p0.y) * (track.p1.x - track.p0.x))) < 0.0f)
    s3 = -1;
  else if (z > 0.0f)
    s3 = 1;
  else
    s3 = 0;

  if ((z = ((startingLine.p1.x - track.p0.x) * (track.p1.y - track.p0.y)) - ((startingLine.p1.y - track.p0.y) * (track.p1.x - track.p0.x))) < 0.0f)
    s4 = -1;
  else if (z > 0.0f)
    s4 = 1;
  else
    s4 = 0;

  if ((s1 * s2 <= 0) && (s3 * s4 <= 0))
    return true;

  // Line segments do not intersect.
  return false;
}
#endif

#ifdef USE_EXTERN_ADS
RaceTrackrADS::RaceTrackrADS()
{
}

bool RaceTrackrADS::begin(int SDA, int SCL)
{
  Wire.begin(SDA, SCL);
  if (!ads.begin())
  {
    return false;
  }
  return true;
}

void RaceTrackrADS::setZeroValue(int CHANNEL, int SAMPLES)
{
  int adsReading;
  for (int i = 0; i < SAMPLES; i++)
  {
    adsReading += ads.readADC_SingleEnded(CHANNEL);
  }
  adsReading = adsReading / SAMPLES;
  adsOffsetValue[CHANNEL] = adsReading;
}

int RaceTrackrADS::readADS(int CHANNEL)
{
  int value = constrain(ads.readADC_SingleEnded(CHANNEL) - adsOffsetValue[CHANNEL], 0, 65535);
  return value;
}

float RaceTrackrADS::readADStoVolts(int CHANNEL)
{
  int value = constrain(ads.readADC_SingleEnded(CHANNEL) - adsOffsetValue[CHANNEL], 0, 65535);
  float voltage = ads.computeVolts(value);
  return voltage;
}

void RaceTrackrADS::update()
{
  for (int i = 0; i < 4; i++)
  {
    adsValue[i] = constrain(ads.readADC_SingleEnded(i) - adsOffsetValue[i], 0, 65535);
    adsVoltage[i] = ads.computeVolts(adsValue[i]);
  }
}
#endif

#ifdef USE_TEMP_SENSOR
RaceTrackrTemp::RaceTrackrTemp()
{
}
void RaceTrackrTemp::begin(int PIN)
{
  _pin = PIN;
  ow = new OneWire(_pin);
  tempSensor = new DallasTemperature(ow);
  tempSensor->begin();
  tempSensor->setResolution(10);
}

float RaceTrackrTemp::update()
{
  tempSensor->requestTemperatures();
  tempC = tempSensor->getTempCByIndex(0);
  return tempC;
}
#endif

#ifdef USE_RPM
RaceTrackrRPM::RaceTrackrRPM(int RPM_PIN, int SENSOR_COUNT)
{
  _trigger_pin = RPM_PIN;
  _sensor_count = SENSOR_COUNT;
}
int RaceTrackrRPM::update()
{
  pcnt_evt_queue = xQueueCreate(10, sizeof(pcnt_evt_t));
  initRPMPCNT(_trigger_pin);

  int RPM0 = -1;      // Fan 0 RPM
  int lastStamp0 = 0; // for previous time stamp for fan 0

  pcnt_evt_t evt;
  portBASE_TYPE res;

  res = xQueueReceive(pcnt_evt_queue, &evt, 0);
  if (res == pdTRUE)
  {
    printf("Event PCNT unit[%d]; Status: %u\n", evt.unit, evt.status);
    if (evt.unit == 0)
    { // Fan 0 - TURN THIS BLOCK INTO A FUNCTION THAT TAKES THE FAN OBJECT
      if (lastStamp0 == 0)
      {
        lastStamp0 = evt.timeStamp;
      }

      int timeDelta = (evt.timeStamp - lastStamp0); // lastTime - firstTime
      if (timeDelta <= 0)
      { // This means we've gotten something wrong
        RPM0 = -1;
      }
      else
      {
        RPM0 = (60000 * (28 / _sensor_count)) / timeDelta;
      }
    }
    lastStamp0 = evt.timeStamp;
  }
  if (user_isr_handle)
  {
    esp_intr_free(user_isr_handle);
    user_isr_handle = NULL;
  }
  return RPM0;
}
#endif