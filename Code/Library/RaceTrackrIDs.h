#define ID_EMERGENCY_STOP 0x000
#define ID_DATA_PULSE 0x001

#define ID_BRAKE_ON 0x004
#define ID_BRAKE_OFF 0x005
#define ID_HORN_ON 0x006
#define ID_HORN_OFF 0x007

#define ID_IGNITION_STATE 0x008
#define ID_MOTOR_PWM 0x009
#define ID_THROTTLE_RAW 0x00A
#define ID_THROTTLE_MODE 0x00B
#define ID_THROTTLE_PARAMETERS 0x00C
#define ID_MOTOR_VOLTAGE_CURRENT 0x00D

#define ID_DISPLAY_VOLTAGE_CURRENT 0x010
#define ID_DISPLAY_AMP_HOURS  0x011
#define ID_DISPLAY_MOTOR_INFO 0x012
#define ID_DISPLAY_TTE 0x013
#define ID_DISPLAY_DASHBOARD_BUTTON 0x014

typedef struct{
    int id;
    const char* name;
    const char* description;
} CAN_ADDRESS;

#define NUMBER_CAN_IDS 12

const CAN_ADDRESS idEmergencyStop = {ID_EMERGENCY_STOP, "EMERGENCY", "Emergency Stop"};
const CAN_ADDRESS idDataPulse = {ID_DATA_PULSE, "DATAPULSE", "Data Pulse"};

const CAN_ADDRESS idBrakeOn = {ID_BRAKE_ON, "BRAKE ON", "Brake On"};
const CAN_ADDRESS idBrakeOff = {ID_BRAKE_OFF, "BRAKE OFF", "Brake Off"};
const CAN_ADDRESS idHornOn = {ID_HORN_ON, "HORN ON", "Horn On"};
const CAN_ADDRESS idHornOff = {ID_HORN_OFF, "HORN OFF", "Horn Off"};

const CAN_ADDRESS idIgnitionState = {ID_IGNITION_STATE, "IGNITION", "Ignition State"};
const CAN_ADDRESS idMotorPwm = {ID_MOTOR_PWM, "MOTOR PWM", "Motor PWM"};
const CAN_ADDRESS idThrottleRaw = {ID_THROTTLE_RAW, "THR READ", "Throttle Reading"};
const CAN_ADDRESS idThrottleMode = {ID_THROTTLE_MODE, "THR MODE", "Throttle Mode"};
const CAN_ADDRESS idThrottleParameters = {ID_THROTTLE_PARAMETERS, "THR PARAM", "Throttle Parameters"};
const CAN_ADDRESS idMotorVoltageCurrent = {ID_MOTOR_VOLTAGE_CURRENT, "VOLT/CURR", "Voltage / Current"};

const CAN_ADDRESS CAN_LOOKUP[NUMBER_CAN_IDS] = {
    idEmergencyStop,
    idDataPulse,
    idBrakeOn,
    idBrakeOff,
    idHornOn,
    idHornOff,
    idIgnitionState,
    idMotorPwm,
    idThrottleRaw,
    idThrottleMode,
    idThrottleParameters,
    idMotorVoltageCurrent
};