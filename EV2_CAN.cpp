#include "EV2_CAN.h"
#include "math.h"

/****************************************************************/
/**
 * Data Logging
 * -------------------
 */
// Motor Controller
int Global_MC_speed  = -1; // 1 revolution = 0.533m = 0.032 kph //RPM
int Global_MC_kph = -1;
int Global_MC_torque = -1;        
int Global_Air_temp = -1;                   
int Global_MC_temp   = -1;
int Global_MC_motortemp = -1;
int Global_MC_voltage = -1;
int Global_MC_current = -1;
int Global_MC_power = -1;
int Global_MC_corestatus = -1;
int Global_MC_error = -1;
int Global_rfe = -1;
int Global_frg = -1;
int Global_MC_go = -1;

// BMS
int Global_BMS_voltage = -1;
float Global_BMS_minvoltage = -1;
float Global_BMS_maxvoltage = -1;
int Global_BMS_current = -1;
int Global_BMS_soc = -1;
int Global_BMS_temp = -1;
int Global_BMS_mintemp = -1;
int Global_BMS_maxtemp = -1;
int Global_BMS_status = -1;
int Global_BMS_state = -1;
int Global_BMS_fault = -1;
int Global_BMS_capacity = -1;

// EV2
int Global_car_state = -1;
#define IDLE 0
#define DRIVE 1
#define FAULT 2
int Global_battfault = -1;
int Global_isofault = -1;
int Global_throttle1 = -1;
int Global_throttle2 = -1;
int Global_avethrottle = -1;
int Global_brake = -1;
float Global_LVBATT_V = -1;
float Global_LVTEMP = -1;
float Global_HV_V = -1;
String Global_error = "OK";
int Global_tsa = -1;
int Global_relay = -1;
int Global_high_current = -1;
int Global_insulation_pwm = -1;
int Global_start_button = -1;
bool throttleEnable = true;

// Comms
unsigned int mc_message_count = 0;
unsigned int bms_message_count = 0;
bool newCANMessages = false;
unsigned int MC_comms = 0;

unsigned int conditionCounter = 0;

bool speakerFirstToggle = true;
bool speakerOn = false;

void updateDB3(void) {
    Serial.print("@");
    // Motor Controller
    Serial.print(Global_MC_speed);
    Serial.print(",");
    Serial.print(Global_MC_kph);
    Serial.print(",");
    Serial.print(Global_MC_torque);
    Serial.print(",");
    Serial.print(Global_Air_temp);
    Serial.print(",");
    Serial.print(Global_MC_temp);
    Serial.print(",");
    Serial.print(Global_MC_motortemp);
    Serial.print(",");
    Serial.print(Global_MC_voltage);
    Serial.print(",");
    Serial.print(Global_MC_current);
    Serial.print(",");
    Serial.print(Global_MC_power);
    Serial.print(",");
    Serial.print(Global_MC_corestatus);
    Serial.print(",");
    Serial.print(Global_MC_error);
    Serial.print(",");
    Serial.print(mc_message_count);
    Serial.print(",");
    Serial.print(Global_rfe);
    Serial.print(",");
    Serial.print(Global_frg);
    Serial.print(",");
    Serial.print(Global_MC_go);
    Serial.print(",");
    // BMS
    Serial.print(Global_BMS_voltage);
    Serial.print(",");
    Serial.print(Global_BMS_minvoltage);
    Serial.print(",");
    Serial.print(Global_BMS_maxvoltage);
    Serial.print(",");
    Serial.print(Global_BMS_current);
    Serial.print(",");
    Serial.print(Global_BMS_soc);
    Serial.print(",");
    Serial.print(Global_BMS_temp);
    Serial.print(",");
    Serial.print(Global_BMS_mintemp);
    Serial.print(",");
    Serial.print(Global_BMS_maxtemp);
    Serial.print(",");
    Serial.print(Global_BMS_status);
    Serial.print(",");
    Serial.print(Global_BMS_state);
    Serial.print(",");
    Serial.print(Global_BMS_capacity);
    Serial.print(",");
    // EV2
    Serial.print(Global_car_state);
    Serial.print(",");
    Serial.print(Global_battfault);
    Serial.print(",");
    Serial.print(Global_isofault);
    Serial.print(",");
    Serial.print(Global_throttle1);
    Serial.print(",");
    Serial.print(Global_throttle2);
    Serial.print(",");
    Serial.print(Global_avethrottle);
    Serial.print(",");
    Serial.print(Global_brake);
    Serial.print(",");
    Serial.print(Global_LVBATT_V);
    Serial.print(",");
    Serial.print(Global_HV_V);
    Serial.print(",");
    Serial.print(Global_error);
    Serial.print(",");
    Serial.print(Global_tsa);
    Serial.print(",");
    Serial.print(Global_relay);
    Serial.print(",");
    Serial.print(Global_high_current);
    Serial.print(",");
    Serial.print(Global_insulation_pwm);
    Serial.print(",");
    Serial.print(Global_start_button);
    
    Serial.print("#");
}
void updateDB4(void) {
    String output ="@";
    // Motor Controller
    output += Global_MC_speed;
    output += ",";
    output += Global_MC_kph;
    output += ",";
    output += Global_MC_torque;
    output += ",";
    output += Global_Air_temp;
    output += ",";
    output += Global_MC_temp;
    output += ",";
    output += Global_MC_motortemp;
    output += ",";
    output += Global_MC_voltage;
    output += ",";
    output += Global_MC_current;
    output += ",";
    output += Global_MC_power;
    output += ",";
    output += Global_MC_corestatus;
    output += ",";
    output += Global_MC_error;
    output += ",";
    output += mc_message_count;
    output += ",";
    output += Global_rfe;
    output += ",";
    output += Global_frg;
    output += ",";
    output += Global_MC_go;
    output += ",";
    // BMS
    output += Global_BMS_voltage;
    output += ",";
    output += Global_BMS_minvoltage;
    output += ",";
    output += Global_BMS_maxvoltage;
    output += ",";
    output += Global_BMS_current;
    output += ",";
    output += Global_BMS_soc;
    output += ",";
    output += Global_BMS_temp;
    output += ",";
    output += Global_BMS_mintemp;
    output += ",";
    output += Global_BMS_maxtemp;
    output += ",";
    output += Global_BMS_status;
    output += ",";
    output += Global_BMS_state;
    output += ",";
    output += Global_BMS_capacity;
    output += ",";
    // EV2
    output += Global_car_state;
    output += ",";
    output += Global_battfault;
    output += ",";
    output += Global_isofault;
    output += ",";
    output += Global_throttle1;
    output += ",";
    output += Global_throttle2;
    output += ",";
    output += Global_avethrottle;
    output += ",";
    output += Global_brake;
    output += ",";
    output += Global_LVBATT_V;
    output += ",";
    output += Global_HV_V;
    output += ",";
    output += Global_error;
    output += ",";
    output += Global_tsa;
    output += ",";
    output += Global_relay;
    output += ",";
    output += Global_high_current;
    output += ",";
    output += Global_insulation_pwm;
    output += ",";
    output += Global_start_button;
    
    output += "#";
    Serial.print(output);
}
void updateDB5(void) {
    SerialUSB.print("@");
    // Motor Controller
    SerialUSB.print(Global_MC_speed);
    SerialUSB.print(",");
    SerialUSB.print(Global_MC_kph);
    SerialUSB.print(",");
    SerialUSB.print(Global_MC_torque);
    SerialUSB.print(",");
    SerialUSB.print(Global_Air_temp);
    SerialUSB.print(",");
    SerialUSB.print(Global_MC_temp);
    SerialUSB.print(",");
    SerialUSB.print(Global_MC_motortemp);
    SerialUSB.print(",");
    SerialUSB.print(Global_MC_voltage);
    SerialUSB.print(",");
    SerialUSB.print(Global_MC_current);
    SerialUSB.print(",");
    SerialUSB.print(Global_MC_power);
    SerialUSB.print(",");
    SerialUSB.print(Global_MC_corestatus);
    SerialUSB.print(",");
    SerialUSB.print(Global_MC_error);
    SerialUSB.print(",");
    SerialUSB.print(mc_message_count);
    SerialUSB.print(",");
    SerialUSB.print(Global_rfe);
    SerialUSB.print(",");
    SerialUSB.print(Global_frg);
    SerialUSB.print(",");
    SerialUSB.print(Global_MC_go);
    SerialUSB.print(",");
    // BMS
    SerialUSB.print(Global_BMS_voltage);
    SerialUSB.print(",");
    SerialUSB.print(Global_BMS_minvoltage);
    SerialUSB.print(",");
    SerialUSB.print(Global_BMS_maxvoltage);
    SerialUSB.print(",");
    SerialUSB.print(Global_BMS_current);
    SerialUSB.print(",");
    SerialUSB.print(Global_BMS_soc);
    SerialUSB.print(",");
    SerialUSB.print(Global_BMS_temp);
    SerialUSB.print(",");
    SerialUSB.print(Global_BMS_mintemp);
    SerialUSB.print(",");
    SerialUSB.print(Global_BMS_maxtemp);
    SerialUSB.print(",");
    SerialUSB.print(Global_BMS_status);
    SerialUSB.print(",");
    SerialUSB.print(Global_BMS_state);
    SerialUSB.print(",");
    SerialUSB.print(Global_BMS_capacity);
    SerialUSB.print(",");
    // EV2
    SerialUSB.print(Global_car_state);
    SerialUSB.print(",");
    SerialUSB.print(Global_battfault);
    SerialUSB.print(",");
    SerialUSB.print(Global_isofault);
    SerialUSB.print(",");
    SerialUSB.print(Global_throttle1);
    SerialUSB.print(",");
    SerialUSB.print(Global_throttle2);
    SerialUSB.print(",");
    SerialUSB.print(Global_avethrottle);
    SerialUSB.print(",");
    SerialUSB.print(Global_brake);
    SerialUSB.print(",");
    SerialUSB.print(Global_LVBATT_V);
    SerialUSB.print(",");
    SerialUSB.print(Global_HV_V);
    SerialUSB.print(",");
    SerialUSB.print(Global_error);
    SerialUSB.print(",");
    SerialUSB.print(Global_tsa);
    SerialUSB.print(",");
    SerialUSB.print(Global_relay);
    SerialUSB.print(",");
    SerialUSB.print(Global_high_current);
    SerialUSB.print(",");
    SerialUSB.print(Global_insulation_pwm);
    SerialUSB.print(",");
    SerialUSB.print(Global_start_button);
    
    SerialUSB.print("#");
}

/****************************************************************/
/**
 * Input/Output Pins related functions 
 * -------------------
 */
void enable_drive(bool enable) {
    set_rfe_frg(enable,enable);
    set_tracsys_relay(enable);
}

void set_speaker(bool enable) {
    pinMode(39,OUTPUT);
    digitalWrite(39,enable);
    speakerOn = enable;
    // Global_Air_temp = (int)speakerOn;
}

void offSpeaker(void) {
    set_speaker(false);
    Timer7.stop();
}

void set_rfe_frg(bool rfe, bool frg) {
    // RFE (p37)
    pinMode(37, OUTPUT);
    digitalWrite(37, rfe);
    // FRG (p35)
    pinMode(35, OUTPUT);
    digitalWrite(35, frg);

    Global_rfe = (int)rfe;
    Global_frg = (int)frg;
}
void set_tracsys_relay(bool relay) {
    // tractive system shutdown relay
    pinMode(33, OUTPUT);
    digitalWrite(33, relay);

    Global_relay = (int)relay;
}
void inputChanged(void) {
    Global_battfault = (int)digitalRead(43);
    Global_isofault = (int)digitalRead(49);
    Global_start_button = (int)digitalRead(41);

    if(Global_car_state == DRIVE) {
        // switch turns off drive but locked to FAULT state
        if (Global_start_button == 0) {
            Global_car_state == IDLE;
            enable_drive(false);
            set_tracsys_relay(true);
            setIdleState();
        }
        // Check if TSA went from High to Low
        if (Global_tsa == 1 && (int)digitalRead(45) == 0) {
            Global_tsa = (int)digitalRead(45);
            announceError("E7");
            emergency_stop();
        }
    }
    else if(Global_car_state == IDLE) {
        if (Global_tsa == 1 && (int)digitalRead(45) == 0) {
            Global_tsa = (int)digitalRead(45);
            announceError("E7");
            emergency_stop();
        }
        // In IDLE state, if Switch switched on, turn on relay (get ready for enable DRIVE)
        if (Global_start_button == 1) {
            set_tracsys_relay(true);
        }
    }
    Global_tsa = (int)digitalRead(45);
}

/****************************************************************/
/**
 * State Checks
 * -------------------
 */

void setIdleState() {
    Global_car_state = IDLE;
    Timer3.stop();
    Timer3.attachInterrupt(idleStateChecks).setFrequency(10).start();
}

void idleStateChecks() {
    Global_car_state = IDLE;
    
    // MC Torque set to 0
    CAN_FRAME outgoing;
    createTorqueWriteFrame(outgoing,0);
    CAN.sendFrame(outgoing);

    if (Global_tsa == 1) {
        if(speakerFirstToggle) {
            // set_speaker(true);
            Timer7.attachInterrupt(offSpeaker).setFrequency(1).start();
        }
    }

    // Global_Air_temp = (int)speakerOn;

    // if start button on and tsa on, drive state
    if (Global_start_button ==  1 && Global_tsa == 1) {
        // Global_brake = get_average_brake_reading_value();
        // if (Global_brake >= 0.25 * MAX_THROTTLE) {
            setDriveState();
        // }
    }
}

void setDriveState() {
    Global_car_state = DRIVE;
    enable_drive(true);
    Timer3.stop();
    Timer3.attachInterrupt(sendThrottle).setFrequency(100).start();
}

void checkBrakeThrottle() {
    get_average_brake_reading_value();
    if (Global_car_state != DRIVE) {
        get_average_pedal_reading_value();
    }

    if (Global_brake > BRAKE_LIMIT && Global_avethrottle > THROTTLE_LIMIT) {
        announceError("E4");
        emergency_stop();
    }

    if(throttleEnable) {
        if(Global_brake >= BRAKE_ACTUATED && Global_avethrottle >= 0.25 * MAX_THROTTLE) {
            throttleEnable = false;
            announceError("E9");
        }
    }
    else {
        if (Global_avethrottle < 0.05 * MAX_THROTTLE) {
            throttleEnable = true;
            // announceError("E10");
        }
    }
}

/****************************************************************/
/**
 * CAN-Bus functions
 * -------------------
 */

bool CAN_setup() {
	if (CAN.init(CAN_BPS_1000K) && CAN2.init(CAN_BTR)) {}
    else
		return false;

    //By default there are 7 mailboxes for each device that are RX boxes
    //This sets each mailbox to have an open filter that will accept extended
    //or standard frames
    int filter;
    for (int filter = 0; filter < 7; filter++) {
        CAN.setRXFilter(filter, 0, 0, false);
        CAN2.setRXFilter(filter, 0, 0, false);
    }      

    return true;
}

void printFrame(CAN_FRAME &frame) {
    Serial.print("ID: 0x");
    Serial.print(frame.id, HEX);
    Serial.print(" Len: ");
    Serial.print(frame.length);
    Serial.print(" Data: 0x");
    for (int count = 0; count < frame.length; count++) {
        Serial.print(frame.data.bytes[count], HEX);
        Serial.print(" ");
    }
    Serial.print("\r\n");
}

void createFrame(CAN_FRAME &frame, int RXID, int length, int REGID, int DATA_1, int DATA_2) {
    frame.id = RXID;
    frame.length = length;
    frame.extended = 0;

    frame.data.bytes[0] = REGID;
    frame.data.bytes[1] = DATA_1;
    frame.data.bytes[2] = DATA_2;

    // frame.data.low = (REGID, data1 and data2 combined)
}

void createMCTempRequestFrame(CAN_FRAME &frame) {
    frame.id = NDRIVE_RXID;
    frame.length = 3;
    frame.data.bytes[0] = DS_SERVO;
    frame.data.bytes[1] = MC_TEMP;
    frame.data.bytes[2] = 0;
}

void createMotorTempRequestFrame(CAN_FRAME &frame) {
    frame.id = NDRIVE_RXID;
    frame.length = 3;
    frame.data.bytes[0] = DS_SERVO;
    frame.data.bytes[1] = MC_MOTORTEMP;
    frame.data.bytes[2] = 0;
}

void createSpeedRequestFrame(CAN_FRAME &frame, int repetition) {
    if (repetition >= 0) {
        frame.id = NDRIVE_RXID;
        frame.length = 3;
        frame.data.bytes[0] = DS_SERVO;
        frame.data.bytes[1] = SPEED_READ_ADD;
        frame.data.bytes[2] = repetition;
    }
}

void createTorqueRequestFrame(CAN_FRAME &frame, int repetition) {
    if (repetition >= 0) {
        frame.id = NDRIVE_RXID;
        frame.length = 3;
        frame.data.bytes[0] = DS_SERVO;
        frame.data.bytes[1] = TORQUE_WRITE_ADD;
        frame.data.bytes[2] = repetition;
    }
}

void createCurrentRequestFrame(CAN_FRAME &frame, int repetition) {
    if (repetition >= 0) {
        frame.id = NDRIVE_RXID;
        frame.length = 3;
        frame.data.bytes[0] = DS_SERVO;
        frame.data.bytes[1] = MC_CURRENT_READ;
        frame.data.bytes[2] = repetition;
    }
}

void createVoltageRequestFrame(CAN_FRAME &frame, int repetition) {
    if (repetition >= 0) {
        frame.id = NDRIVE_RXID;
        frame.length = 3;
        frame.data.bytes[0] = DS_SERVO;
        frame.data.bytes[1] = MC_VOLTAGE_READ;
        frame.data.bytes[2] = repetition;
    }
}

void createCoreStatusRequestFrame(CAN_FRAME &frame) {
    frame.id = NDRIVE_RXID;
    frame.length = 3;
    frame.data.bytes[0] = DS_SERVO;
    frame.data.bytes[1] = CORE_STATUS;
    frame.data.bytes[2] = 0x00;
}

void createSpeedWriteFrame(CAN_FRAME &frame, float speed) {
    speed = speed * MAX_SPEED_WRITE;
    frame.id = NDRIVE_RXID;
    frame.length = 3;
    frame.data.bytes[0] = SPEED_WRITE_ADD;
    frame.data.bytes[1] = (int)speed;
    frame.data.bytes[2] = (int)speed >> 8;
}

void createTorqueWriteFrame(CAN_FRAME &frame, float torque) {
    torque = torque * MAX_TORQUE_WRITE; 
    frame.id = NDRIVE_RXID;
    frame.length = 3;
    frame.data.bytes[0] = TORQUE_WRITE_ADD;
    frame.data.bytes[1] = (int)torque;
    frame.data.bytes[2] = (int)torque >> 8;
}

void request_MC_speed(void) {
    CAN_FRAME speedRequest;
    int repetition = 200; // 100ms
    // int repetition = 0;
    createSpeedRequestFrame(speedRequest,repetition);
    CAN.sendFrame(speedRequest);
}

void request_MC_torque(void) {
    CAN_FRAME torqueRequest;
    int repetition = 200; // 100ms
    createTorqueRequestFrame(torqueRequest,repetition);
    CAN.sendFrame(torqueRequest);
}

void request_MC_current(void) {
    CAN_FRAME currentRequest;
    int repetition = 500; // 500ms
    createCurrentRequestFrame(currentRequest,repetition);
    CAN.sendFrame(currentRequest);
}

void request_MC_voltage(void) {
    CAN_FRAME voltageRequest;
    int repetition = 500; // 500ms
    createVoltageRequestFrame(voltageRequest,repetition);
    CAN.sendFrame(voltageRequest);
}

bool parseFrame(CAN_FRAME &frame) {

    newCANMessages = true;
    if (frame.id == NDRIVE_TXID) {
        mc_message_count++;
        switch(frame.data.bytes[0]) {
            // MC Related
            case SPEED_READ_ADD: {
                int16_t speed = (frame.data.bytes[2] << 8) | frame.data.bytes[1];
                Global_MC_speed = 2000 * speed/MAX_SPEED_READ;
                Global_MC_kph = Global_MC_speed * 0.032;
                break;
            }
            case TORQUE_WRITE_ADD: {
                unsigned int torque = (frame.data.bytes[2] << 8) | frame.data.bytes[1];
                torque = torque * 100 / MAX_TORQUE_WRITE;
                Global_MC_torque = torque;
                break;
            }
            case MC_VOLTAGE_READ: {
                int16_t voltage = (frame.data.bytes[2] << 8) | frame.data.bytes[1];
                Global_MC_voltage = voltage/54.54;
                Global_MC_power = Global_MC_voltage*Global_MC_current;
                break;
            }
            case MC_CURRENT_READ: {
                int16_t current = (frame.data.bytes[2] << 8) | frame.data.bytes[1];
                Global_MC_current = current/3.925;
                Global_MC_power = Global_MC_voltage*Global_MC_current;
                break;
            }
            case CORE_STATUS: {
                int16_t status = (frame.data.bytes[2] << 8) | frame.data.bytes[1];
                Global_MC_corestatus = status;
                Global_MC_go = (bool)(status & 1);
                break;
            }
            case MC_TEMP: {
                float temp = (frame.data.bytes[2] << 8) | frame.data.bytes[1];
                Global_MC_temp = log(temp/15536)/0.0053;
                if (Global_MC_temp > MC_TEMP_LIMIT) {
                    announceError("M1");
                    // emergency_stop();
                }
                break;
            }
            case MC_MOTORTEMP: {
                float temp = (frame.data.bytes[2] << 8) | frame.data.bytes[1];
                Global_MC_motortemp = (temp - 10110)/51.7;
                if (Global_MC_temp > MOTOR_TEMP_LIMIT) {
                    announceError("M2");
                    // emergency_stop();
                }
                break;
            }
        }
    }
    // BMS Related
    else {
        bms_message_count++;
        switch (frame.id) {
            case BMS_STATE: {
                Global_BMS_state = frame.data.bytes[0];
                Global_BMS_status = (int)(Global_BMS_state&1);
                Global_BMS_fault = frame.data.bytes[5];
                if (Global_BMS_status != 0) {
                    announceError("B1");
                }
                if (Global_BMS_fault != 0) {
                    announceError("B5");
                    bool underVoltage = (Global_BMS_fault & ( 1 << 6 )) >> 7 & 6;
                    bool overVoltage = (Global_BMS_fault & ( 1 << 7 )) >> 7 & 1;
                    // voltage too low B2
                    if (underVoltage) {
                        announceError("B2");
                    }
                    // voltage too high B3
                    if (overVoltage) {
                        announceError("B3");
                    }
                    emergency_stop();
                }   
                break;
            }
            case PACK_VOLTAGE: {
                int voltage = (frame.data.bytes[0] << 8) | frame.data.bytes[1];
                Global_BMS_voltage = voltage;
                float minVoltage = (frame.data.bytes[2])/10.0;
                Global_BMS_minvoltage = minVoltage;
                float maxVoltage = (frame.data.bytes[4])/10.0;
                Global_BMS_maxvoltage = maxVoltage;

                if (minVoltage < MIN_VOLT_BMS_LIMIT) {
                    announceError("B2");
                }
                if (maxVoltage > MAX_VOLT_BMS_LIMIT) {
                    announceError("B3");
                }
                break;
            }

            case PACK_CURRENT: {
                int current = (frame.data.bytes[0] << 8) | frame.data.bytes[1];
                Global_BMS_current = current;
                
                break;
            }
            case PACK_SOC: {
                Global_BMS_soc = frame.data.bytes[0];
                Global_BMS_capacity = (frame.data.bytes[3] << 8) | frame.data.bytes[4];
                break;
            }
            case PACK_TEMP: {
                Global_BMS_temp = frame.data.bytes[0];
                Global_BMS_mintemp = frame.data.bytes[2];
                Global_BMS_maxtemp = frame.data.bytes[4];

                if (Global_BMS_maxtemp > MAX_BMS_TEMP) {
                    announceError("B4");
                    return false;
                }

                break;
            }
        }
    }
    return true;
}

/****************************************************************/
/**
 * ADC functions
 * -------------------
 */

volatile int pedal1_raw = -1;
volatile int pedal2_raw = -1;
volatile int brake_raw = -1;

void adc_setup(void) {
    adc_init(ADC, SystemCoreClock, ADC_FREQ_MAX, 8);
    adc_configure_timing(ADC, 0, ADC_SETTLING_TIME_3, 1);
    adc_set_resolution(ADC, ADC_12_BITS);

    // Enable ADC channels arrange by arduino pins from A0 to A9
    // adc_enable_channel(ADC, ADC_CHANNEL_7); // A0
    // adc_enable_channel(ADC, ADC_CHANNEL_6); // A1
    // adc_enable_channel(ADC, ADC_CHANNEL_5); // A2
    // adc_enable_channel(ADC, ADC_CHANNEL_4); // A3
    // adc_enable_channel(ADC, ADC_CHANNEL_3); // A4
    // adc_enable_channel(ADC, ADC_CHANNEL_2); // A5
    // adc_enable_channel(ADC, ADC_CHANNEL_1); // A6
    // adc_enable_channel(ADC, ADC_CHANNEL_0); // A7
    // adc_enable_channel(ADC, ADC_CHANNEL_10); // A8
    // adc_enable_channel(ADC, ADC_CHANNEL_11); // A9
    // adc_enable_channel(ADC, ADC_CHANNEL_12); // A10
    // adc_enable_channel(ADC, ADC_CHANNEL_13); // A11

    // Enable ADC channels for pedals
    adc_enable_channel(ADC, PEDAL1_ADC_CHANNEL);
    adc_enable_channel(ADC, PEDAL2_ADC_CHANNEL);
    adc_enable_channel(ADC, BRAKE_PEDAL_CHANNEL);

    // Enable ADC channels for LV and HV measuring
    adc_enable_channel(ADC, LV_BATTERY_V);
    adc_enable_channel(ADC, HV_V);
    adc_enable_channel(ADC, HV_CURRENT);

    // Temperature Sensor
    adc_enable_channel(ADC, LV_TEMP);

    // Enable ADC interrupt
    adc_enable_interrupt(ADC, ADC_IER_EOC7); //EOC9 so that interrupt triggered when analogue input channerl 9 has reached end of conversion
    
    // Trigger configuration
    adc_configure_trigger(ADC, ADC_TRIG_SW, 0);
    
    // Enable ADC interrupt
    NVIC_EnableIRQ(ADC_IRQn);
    
    //start ADC conversion, note that ADC conversion has to be restarted once conversion is finished
    adc_start(ADC);
}

void ADC_Handler(void) {
    // Check the ADC conversion status
    if ((adc_get_status(ADC) & ADC_ISR_EOC7) == ADC_ISR_EOC7) {
        // // Get digital data value from ADC channels and can be used by application

        pedal1_raw = adc_get_channel_value(ADC, PEDAL1_ADC_CHANNEL);
        pedal2_raw = adc_get_channel_value(ADC, PEDAL2_ADC_CHANNEL);

        brake_raw = adc_get_channel_value(ADC, BRAKE_PEDAL_CHANNEL);

        Global_LVBATT_V = adc_get_channel_value(ADC, LV_BATTERY_V)/233.3;;
        Global_HV_V = adc_get_channel_value(ADC, HV_V);
        Global_high_current = adc_get_channel_value(ADC, HV_CURRENT);

        Global_LVTEMP = adc_get_channel_value(ADC, LV_TEMP);
    }
    adc_start(ADC);
}

void checkForFaults(void) {
    if(Global_LVBATT_V != -1 && Global_LVBATT_V < MIN_LV) {
        announceError("E6");
        emergency_stop();
    }
}

/****************************************************************/
/**
 * Emergency functions
 * -------------------
 */

/**
 * Stops the vehicle completely
 */
void emergency_stop() {
    // 1. stop sending throttle values
    Timer3.stop();
    Timer3.attachInterrupt(checkBrakeThrottle).setFrequency(10).start();

    // 2. send 0 throttle to MC
    CAN_FRAME outgoing;
    createTorqueWriteFrame(outgoing,0);
    CAN.sendFrame(outgoing);

    // 3. Disable Drive (RFE, FRG, TSA RELAY)
    enable_drive(false);

    Global_car_state = FAULT;
}

/**
 * Emergency stop the vehicle if condition is invalid.
 * @param condition
 */
void assert_or_abort(bool condition) {
    if ( ! condition) {
        // announceError("E12");
        // emergency_stop();
    }
}

/****************************************************************/
/**
 * Pedal reading functions
 * -----------------------
 */

const int pedal1_min = 450;  // pedal1 min value in 12-bit range
const int pedal1_max = 1200; // pedal1 max value in 12-bit range
const int pedal2_min = 450;  // pedal2 min value in 12-bit range
const int pedal2_max = 1100; // pedal2 max value in 12-bit range
const int brake_min = 1250;
const int brake_max = 2500;

/**
 * Processes pedal reading. Maps value to 16-bit range.
 * @param  raw_value Raw value
 * @param  min_value Minimum value
 * @param  max_value Maximum value
 * @return value     Processed value
 */
int get_pedal_reading(const int raw_value, const int min_value, const int max_value) {
    // Map to 16-bit range
    return constrain(map(raw_value, min_value, max_value, 0, 65536), 0, 65536);
}

/**
 * Returns average of two readings
 * @param  reading_1
 * @param  reading_2
 * @return
 */
int get_average_pedal_reading(const int reading_1, const int reading_2) {
    return (reading_1 + reading_2) / 2;
}


int get_average_pedal_reading_value() {
    
    if(pedal1_raw > 1.5 * pedal1_max) {
        announceError("E11");
    }

    if(pedal2_raw > 1.5 * pedal2_max) {
        announceError("E12");
    }

    int reading_1 = get_pedal_reading(pedal1_raw, pedal1_min, pedal1_max);
    int reading_2 = get_pedal_reading(pedal2_raw, pedal2_min, pedal2_max);
    
    // Global_throttle1 = reading_1;
    // Global_throttle2 = reading_1;
    Global_throttle1 = 0;
    Global_throttle2 = 0;

    // assert_pedal_in_threshold(reading_1, reading_2, MAX_THROTTLE_DIFF);

    Global_avethrottle = get_average_pedal_reading(reading_1,reading_2);

    if (pedal1_raw < 200) {
        announceError("E1");
        emergency_stop();
    }
    if (pedal2_raw < 200) {
        announceError("E2");
        emergency_stop();
    }

    return Global_avethrottle;
    // return 0;
}

int get_average_brake_reading_value() {
    Global_brake = get_pedal_reading(brake_raw, brake_min, brake_max);
    // Global_brake = brake_raw;
    if (brake_raw < 400) {
        // announceError("E3");
        // emergency_stop();
    }
    return Global_brake;
}

/**
 * Asserts pedal readings are in threshold
 * @param  reading_1
 * @param  reading_2
 * @param  threshold
 */
void assert_pedal_in_threshold(const int reading_1, const int reading_2, const int threshold) {
    int difference = abs(reading_1 - reading_2);
    bool condition = difference < threshold;

    #ifdef SerialDebug
    if ( ! condition) {
        SerialDebug.println("Pedal reading discrepancy detected!");
        SerialDebug.print("Reading (1): ");
        SerialDebug.println(reading_1);
        SerialDebug.print("Reading (2): ");
        SerialDebug.println(reading_2);
        SerialDebug.print("Threshold: ");
        SerialDebug.println(threshold);
        SerialDebug.print("Difference: ");
        SerialDebug.println(difference);
    }
    #endif

    if(!condition) {
        conditionCounter++;
    }
    else {
        conditionCounter = 0;
    }

    if(conditionCounter == 2) {
        String error = "E5";
        error += ":";
        error += "[R1]";
        error += reading_1;
        error += "[R2]";
        error += reading_2;
        error += "[R1-R2]";
        error += abs(reading_1-reading_2);
        error += "\n";

        announceError(error);
        emergency_stop();
    }
}

void sendThrottle(void) {
    float torque = get_average_pedal_reading_value();
    torque /= MAX_THROTTLE;
    // Global_throttle1 = torque;
    torque *= 0.5;
    // Global_throttle2 = torque;
    if (!throttleEnable) {
        torque = 0;
    }
    CAN_FRAME outgoing;
    createTorqueWriteFrame(outgoing,torque);
    CAN.sendFrame(outgoing);
}

void announceError(String error) {
    if (Global_error == "OK") {
        Global_error = error;
    }
    else {
        // Global_error += ";";
        Global_error = error;
    }
}
void checkCANComms(void) {
    if (newCANMessages) {
        newCANMessages = false;
    }
    else {
        // announceError("E8");
    }
}

void check_MC_comms(void) {
    // If no MC response
    if(mc_message_count == MC_comms || Global_MC_speed == -1 || Global_MC_torque == -1 || Global_MC_current == -1 || Global_MC_voltage == -1) {
        request_MC_speed();
        request_MC_torque();
        request_MC_current();
        request_MC_voltage();
    }
    else {
        MC_comms = mc_message_count;
    }
}