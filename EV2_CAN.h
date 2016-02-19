#ifndef EV2_CAN
#define EV2_CAN

#include "due_can.h"
#include "DueTimer.h"
/**
* CANBus related constants
**/
#define CAN_BTR CAN_BPS_500K
#define NDRIVE_RXID 0x210
#define NDRIVE_TXID 0x190

// Writing
#define SPEED_WRITE_ADD		0x31
#define MAX_SPEED_WRITE 	0x7FFF
#define TORQUE_WRITE_ADD	0x90
#define MAX_TORQUE_WRITE	32760

#define MAX_THROTTLE 65536
#define MAX_THROTTLE_DIFF 6554
#define BRAKE_ACTUATED 10000

// Reading
#define DS_SERVO		0x3D		//Motor Controller
#define SPEED_READ_ADD	0x30
#define MAX_SPEED_READ	0x7FFF
#define MC_CURRENT_READ	0x20
#define MC_VOLTAGE_READ	0xEB
#define MC_TEMP		 	0x4A
#define MC_AIRTEMP		0x4B
#define MC_MOTORTEMP 	0x49	
#define CORE_STATUS		0x40
#define KERN_STATUS		0x181 		// Bit 0 = Drive Enabled, Bit 7 = Position Control, Bit 8 = Speed Control

void updateDB3(void);	// All variables
void updateDB4(void);	// Optimisation
void updateDB5(void);	// Navtive USB

void enable_drive(bool enable);
void set_rfe_frg(bool rfe, bool frg);
void set_tracsys_relay(bool x);
void set_speaker(bool enable);
void inputChanged(void);

bool CAN_setup();
void printFrame(CAN_FRAME &frame);
bool parseFrame(CAN_FRAME &frame);	// for logging
void createFrame(CAN_FRAME &frame, int RXID, int length, int REGID, int DATA_1, int DATA_2);
void abort_requests(int REGID);
bool status();

void request_MC_speed(void);
void request_MC_torque(void);
void request_MC_current(void);
void request_MC_voltage(void);

// void sendTorque(int torque_percent);
void emergency_stop();
void assert_or_abort(bool condition);

#define SPEED_REPETITION 100
void createMCTempRequestFrame(CAN_FRAME &frame);
void createMotorTempRequestFrame(CAN_FRAME &frame);
void createSpeedRequestFrame(CAN_FRAME &frame, int repetition); // repition in ms
void createTorqueRequestFrame(CAN_FRAME &frame, int repetition); // repition in ms
void createCurrentRequestFrame(CAN_FRAME &frame, int repetition); // repition in ms
void createVoltageRequestFrame(CAN_FRAME &frame, int repetition);
void createCoreStatusRequestFrame(CAN_FRAME &frame);

void createSpeedWriteFrame(CAN_FRAME &frame, float speed);
void createTorqueWriteFrame(CAN_FRAME &frame, float torque);

/**
*	Limits
**/
#define MC_TEMP_LIMIT 60
#define MOTOR_TEMP_LIMIT 60


/**
*	Pedal Reading
**/
// Pedal channels
#define LV_BATTERY_V ADC_CHANNEL_1
#define LV_TEMP ADC_CHANNEL_0
#define HV_V ADC_CHANNEL_2
#define HV_CURRENT ADC_CHANNEL_12
#define PEDAL1_ADC_CHANNEL ADC_CHANNEL_7 // this is A0
#define PEDAL2_ADC_CHANNEL ADC_CHANNEL_6 // this is A1
#define BRAKE_PEDAL_CHANNEL ADC_CHANNEL_5 // this is A2
#define BRAKE_LIMIT 60000
#define THROTTLE_LIMIT 60000

void checkBrakeThrottle(void);
void adc_setup(void);
void ADC_Handler(void);
int get_pedal_reading(const int raw_value, const int min_value, const int max_value);
int get_average_pedal_reading(const int reading_1, const int reading_2);

int get_average_brake_reading_value();
int get_average_pedal_reading_value();

void assert_pedal_in_threshold(const int reading_1, const int reading_2, const int threshold);
void sendThrottle();

void setIdleState(void);
void idleStateChecks(void);
void setDriveState(void);
void checkForFaults(void);

void announceError(String error);

/**
*	BMS Related Constants
**/
#define BMS_STATE		0x622
#define PACK_VOLTAGE	0x623
#define PACK_CURRENT	0x624
#define PACK_SOC		0x626
#define PACK_TEMP		0x627

#define MAX_BMS_TEMP	100
#define MIN_BMS_TEMP	0
#define MIN_VOLT_BMS_LIMIT	0
#define MAX_VOLT_BMS_LIMIT	350

/**
*	Fault Checks
**/
#define MIN_LV 11.8

void checkCANComms(void);
void check_MC_comms(void);
void offSpeaker(void);
#endif