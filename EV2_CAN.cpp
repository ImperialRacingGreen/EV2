#include "EV2_CAN.h"

/****************************************************************/
int Global_rfe = -1;
int Global_frg = -1;
int Global_relay = -1;

void set_rfe_frg(bool rfe, bool frg)
{
    Global_rfe = (int)rfe;
    Global_frg = (int)frg;
}
void set_tracsys_relay(bool x)
{
    Global_relay = (int)x;
}

int Global_battfault = -1;
int Global_isofault = -1;
int Global_tsa = -1;

void inputChanged(void)
{
    Global_battfault = (int)digitalRead(43);
    Global_isofault = (int)digitalRead(49);
    Global_tsa = (int)digitalRead(45);
}

int Global_MC_temp   = -1;
int Global_MC_speed  = -1;
int Global_MC_torque = -1;
int Global_MC_voltage = -1;
int Global_MC_current = -1;
int Global_MC_motortemp = -1;

int Global_BMS_voltage = -1;
int Global_BMS_current = -1;
int Global_BMS_soc = -1;
int Global_BMS_temp = -1;
int Global_BMS_mintemp = -1;
int Global_BMS_maxtemp = -1;
int Global_BMS_status = -1;

int Global_avethrottle = -1;

float Global_LVBATT_V = -1;
float Global_HV_V = -1;

void updateDB(void)
{
    Serial.print("RFE = ");
    Serial.println(Global_rfe);
    Serial.print("FRG = ");
    Serial.println(Global_frg);
    Serial.print("SRELAY = ");
    Serial.println(Global_relay);
    Serial.print("BFAULT = ");
    Serial.println(Global_battfault);
    Serial.print("IFAULT = ");
    Serial.println(Global_isofault);
    Serial.print("TSA = ");
    Serial.println(Global_tsa);
    Serial.print("MCTMP = ");
    Serial.println(Global_MC_temp);
    Serial.print("MTRTMP = ");
    Serial.println(Global_MC_motortemp);
    Serial.print("RPM = ");
    Serial.println(Global_MC_speed);
    Serial.print("TQE = ");
    Serial.println(Global_MC_torque);
    Serial.print("MCC = ");
    Serial.println(Global_MC_current);
    Serial.print("MCV = ");
    Serial.println(Global_MC_voltage);

    Serial.print("BVOLT = ");
    Serial.println(Global_BMS_voltage);
    Serial.print("BCUR = ");
    Serial.println(Global_BMS_current);
    Serial.print("SOC = ");
    Serial.println(Global_BMS_soc);
    Serial.print("BTMP = ");
    Serial.println(Global_BMS_temp);
    Serial.print("BMINTMP = ");
    Serial.println(Global_BMS_mintemp);
    Serial.print("BMAXTMP = ");
    Serial.println(Global_BMS_maxtemp);
    Serial.print("BSTAT = ");
    Serial.println(Global_BMS_status);

    Serial.print("LV = ");
    Serial.println(Global_LVBATT_V);
    Serial.print("HV = ");
    Serial.println(Global_HV_V);

    Serial.print("AVE_THROTTLE = ");
    Serial.println(Global_avethrottle);


    Serial.println();
    Serial.println();
}

void updateDB2(void)
{

    Serial.print("MA");
    Serial.print(Global_MC_speed);

    Serial.print("MB");
    Serial.print(Global_MC_torque);

    Serial.print("MD");
    Serial.print(Global_MC_temp);

    Serial.print("MB");
    Serial.print(Global_MC_voltage);

    Serial.print("MD");
    Serial.print(Global_MC_current);

    Serial.print("BA");
    Serial.print(Global_BMS_voltage);

    Serial.print("BB");
    Serial.print(Global_BMS_current);

    Serial.print("BC");
    Serial.print(Global_BMS_soc);

    Serial.print("BD");
    Serial.print(Global_BMS_temp);

    Serial.print("BH");
    Serial.print(Global_BMS_status);
}

void updateDB_Processing(void)
{
    Serial.print(Global_MC_speed);
    Serial.print(",");
    Serial.print(Global_MC_torque);
    Serial.print(",");
    Serial.print(Global_MC_motortemp);
    Serial.print(",");
    Serial.print(Global_MC_temp);
    Serial.print(",");
    Serial.print(Global_MC_voltage);
    Serial.print(",");
    Serial.print(Global_MC_current);
    Serial.print(",");
    Serial.print(Global_rfe);
    Serial.print(",");
    Serial.print(Global_frg);
    Serial.print(",");
    
    Serial.print(Global_BMS_voltage);
    Serial.print(",");
    Serial.print(Global_BMS_temp);
    Serial.print(",");
    Serial.print(Global_BMS_mintemp);
    Serial.print(",");
    Serial.print(Global_BMS_maxtemp);
    Serial.print(",");
    Serial.print(Global_BMS_status);
    Serial.print(",");
    
    Serial.print(Global_battfault);
    Serial.print(",");
    Serial.print(Global_isofault);
    Serial.print(",");
    Serial.print(Global_avethrottle);
    Serial.print(",");
    Serial.print(Global_LVBATT_V);
    Serial.print(",");
    Serial.print(Global_HV_V);
    Serial.print(",");
    Serial.print(Global_tsa);
    Serial.print(",");
    Serial.print(Global_relay);

    Serial.print("\n");
}
/****************************************************************/

// Registers to store raw pedal readings during interrupt
volatile int pedal1_raw = -1;
volatile int pedal2_raw = -1;

bool CAN_setup() {
	if (CAN.init(CAN_BAUD_RATE) && CAN2.init(CAN_BAUD_RATE)) {}
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


bool parseFrame(CAN_FRAME &frame) {
    if (frame.id == NDRIVE_TXID) {
        switch(frame.data.bytes[0]) {
            // MC Related
            case SPEED_READ_ADD: {
                int16_t speed = (frame.data.bytes[2] << 8) | frame.data.bytes[1];
                // speed = speed * 100 / MAX_SPEED_READ;
                Global_MC_speed = speed;
                break;
            }
            case TORQUE_WRITE_ADD: {
                unsigned int torque = (frame.data.bytes[2] << 8) | frame.data.bytes[1];
                torque = torque * 100 / MAX_TORQUE_WRITE;
                Global_MC_torque = torque;
                break;
            }
            case MC_CURRENT_READ: {
                int16_t current = (frame.data.bytes[2] << 8) | frame.data.bytes[1];
                Global_MC_current = current;
                break;
            }
            case MC_VOLTAGE_READ: {
                int16_t voltage = (frame.data.bytes[2] << 8) | frame.data.bytes[1];
                Global_MC_voltage = voltage;
                break;
            }
            case CORE_STATUS: {
                int status = (frame.data.bytes[2] << 8) | frame.data.bytes[1];
                if (status == KERN_STATUS) {
                    // DRIVE ENABLED, POSITION CONTROL ENABLED, SPEED CONTROL IS ENABLED
                    // Serial.println("NDRIVE CORE_STATUS = KERN_STATUS");
                } 
                break;
            }
            case MC_TEMP: {
                float temp = (frame.data.bytes[2] << 8) | frame.data.bytes[1];
                Global_MC_temp = temp;
            }
            case MC_MOTORTEMP: {
                float temp = (frame.data.bytes[2] << 8) | frame.data.bytes[1];
                Global_MC_motortemp = temp;
            }
            

        }
    }
    // BMS Related
    else {
    	switch (frame.id) {
    		case BMS_STATUS: {
                if (frame.data.bytes[0] != 0) {
                    // Serial.println("BMS Error : State of System = 1");
                    Global_BMS_status = frame.data.bytes[0];
                    // emergency_stop();
                    return false;
                }
                else {
                    Global_BMS_status = frame.data.bytes[0];
                }
    			break;
            }
            case PACK_VOLTAGE: {
                int voltage = (frame.data.bytes[0] << 8) | frame.data.bytes[1];
                Global_BMS_voltage = voltage;
                break;
            }
            case PACK_CURRENT: {
                int current = (frame.data.bytes[0] << 8) | frame.data.bytes[1];
                Global_BMS_current = current;
                break;
            }
            case PACK_SOC: {
                Global_BMS_soc = frame.data.bytes[0];
                break;
            }
            case PACK_TEMP: {
                if (frame.data.bytes[0] > MAX_TEMP) {
                    Serial.println(frame.data.bytes[0]);
                    Global_BMS_temp = frame.data.bytes[0];
                    Global_BMS_mintemp = frame.data.bytes[2];
                    Global_BMS_maxtemp = frame.data.bytes[3];
                    // emergency_stop();
                    return false;
                }
                else {
                    Global_BMS_temp = frame.data.bytes[0];
                    Global_BMS_mintemp = frame.data.bytes[2];
                    Global_BMS_maxtemp = frame.data.bytes[3];
                }
                break;
            }
    	}
    }
    return true;
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
    // Serial.println(speed, HEX);
}

void createTorqueWriteFrame(CAN_FRAME &frame, float torque) {
    torque = torque * MAX_TORQUE_WRITE; 
    frame.id = NDRIVE_RXID;
    frame.length = 3;
    frame.data.bytes[0] = TORQUE_WRITE_ADD;
    frame.data.bytes[1] = (int)torque;
    frame.data.bytes[2] = (int)torque >> 8;
    // Serial.println(torque, HEX);
}

void abort_requests(int REGID) {
	CAN_FRAME frame_abort;
	createFrame(frame_abort, NDRIVE_RXID, 3, DS_SERVO, REGID, 0xFF);
	CAN.sendFrame(frame_abort);
	delayMicroseconds(100);
}

void abort_all_requests() {
    abort_requests(SPEED_READ_ADD);
}

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

    // Enable ADC channels for pedals
    adc_enable_channel(ADC, PEDAL1_ADC_CHANNEL);
    adc_enable_channel(ADC, PEDAL2_ADC_CHANNEL);

    // Enable ADC channels for LV and HV measuring
    adc_enable_channel(ADC, LV_BATTERY_V);
    adc_enable_channel(ADC, HV_V);

    // Enable ADC interrupt
    adc_enable_interrupt(ADC, ADC_IER_EOC7); //EOC9 so that interrupt triggered when analogue input channerl 9 has reached end of conversion
    
    // Trigger configuration
    adc_configure_trigger(ADC, ADC_TRIG_SW, 0);
    
    // Enable ADC interrupt
    NVIC_EnableIRQ(ADC_IRQn);
    
    //start ADC conversion, note that ADC conversion has to be restarted once conversion is finished
    adc_start(ADC);
}

void ADC_Handler(void)
{
    // Check the ADC conversion status
    if ((adc_get_status(ADC) & ADC_ISR_EOC7) == ADC_ISR_EOC7)
    {
        // // Get digital data value from ADC channels and can be used by application
        // CHANNEL_0_REG = adc_get_channel_value(ADC, ADC_CHANNEL_0);
        // CHANNEL_1_REG = adc_get_channel_value(ADC, ADC_CHANNEL_1);
        // CHANNEL_2_REG = adc_get_channel_value(ADC, ADC_CHANNEL_2);
        // CHANNEL_3_REG = adc_get_channel_value(ADC, ADC_CHANNEL_3);
        // CHANNEL_4_REG = adc_get_channel_value(ADC, ADC_CHANNEL_4);
        // CHANNEL_5_REG = adc_get_channel_value(ADC, ADC_CHANNEL_5);
        // CHANNEL_6_REG = adc_get_channel_value(ADC, ADC_CHANNEL_6);
        // CHANNEL_7_REG = adc_get_channel_value(ADC, ADC_CHANNEL_7);
        // CHANNEL_8_REG = adc_get_channel_value(ADC, ADC_CHANNEL_10); //notice that its channel 10
        // CHANNEL_9_REG = adc_get_channel_value(ADC, ADC_CHANNEL_11); //notice that its channel 11

        pedal1_raw = adc_get_channel_value(ADC, PEDAL1_ADC_CHANNEL);
        pedal2_raw = adc_get_channel_value(ADC, PEDAL2_ADC_CHANNEL);

        Global_LVBATT_V = adc_get_channel_value(ADC, LV_BATTERY_V);
        Global_HV_V = adc_get_channel_value(ADC, HV_V);
    }
    adc_start(ADC);
}


/**
 * Emergency functions
 * -------------------
 */

/**
 * Stops the vehicle completely
 */
void emergency_stop()
{
    // TODO send shutdown commands, functions, etc.

    #ifdef SerialDebug
    SerialDebug.println("Aborted!");
    #endif

    while (true) {
        // Infinite loop
    }
}

/**
 * Emergency stop the vehicle if condition is invalid.
 * @param condition
 */
void assert_or_abort(bool condition)
{
    if ( ! condition) {
        emergency_stop();
    }
}

/**
 * Pedal reading functions
 * -----------------------
 */

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

/**
 * Calibrated values
 */
const int pedal1_min = 690;  // pedal1 min value in 12-bit range
const int pedal1_max = 1400; // pedal1 max value in 12-bit range
const int pedal2_min = 650;  // pedal2 min value in 12-bit range
const int pedal2_max = 1370; // pedal2 max value in 12-bit range

int get_average_pedal_reading_value() {
    // Serial.print("PEDAL_1 RAW = ");
    // Serial.print(pedal1_raw);
    // Serial.print("      PEDAL_2 RAW = ");
    // Serial.println(pedal2_raw);
    int reading_1 = get_pedal_reading(pedal1_raw, pedal1_min, pedal1_max);
    int reading_2 = get_pedal_reading(pedal2_raw, pedal2_min, pedal2_max);
    // Serial.print("READING_1 = ");
    // Serial.print(reading_1);
    // Serial.print("      READING_2 = ");
    // Serial.println(reading_2);
    Global_avethrottle = get_average_pedal_reading(reading_1,reading_2);
    return Global_avethrottle;
}

/**
 * Asserts pedal readings are in threshold
 * @param  reading_1
 * @param  reading_2
 * @param  threshold
 */
void assert_pedal_in_threshold(const int reading_1, const int reading_2, const int threshold)
{
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

    assert_or_abort(condition);
}

