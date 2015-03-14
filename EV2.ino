#include "EV2_CAN.h"
#include "DueTimer.h"

#define MAX_THROTTLE 65536
#define MAX_MC_SPEED 0x7FFF


void MC_setup(void)
{
    // pin37 rfe, 35 frg for DRIVE
    pinMode(37, OUTPUT);
    digitalWrite(37, HIGH);
    pinMode(35, OUTPUT);
    digitalWrite(35, HIGH);
}

void sendThrottle(void)
{   
    float torque = get_average_pedal_reading_value()*100;
    torque /= MAX_THROTTLE;
    // Serial.print("TORQUE (%%) = ");
    // Serial.println(torque);
    CAN_FRAME outgoing;
    createTorqueWriteFrame(outgoing,torque/100);
    CAN.sendFrame(outgoing);
}

void request_MC_temperature(void)
{
    CAN_FRAME tempRequest;
    createTempRequestFrame(tempRequest);
    CAN.sendFrame(tempRequest);   
}

void request_MC_speed(void)
{
    CAN_FRAME speedRequest;
    int repetition = 100; // 100ms
    createSpeedRequestFrame(speedRequest,repetition);
    printFrame(speedRequest);
    Serial.println();
    CAN.sendFrame(speedRequest);
}

void request_MC_torque(void)
{
    CAN_FRAME torqueRequest;
    int repetition = 100; // 100ms
    createTorqueRequestFrame(torqueRequest,repetition);
    printFrame(torqueRequest);
    Serial.println();
    CAN.sendFrame(torqueRequest);
}

void setup() {
    
    MC_setup();

    Serial.begin(115200);
	CAN_setup();

    delay(100);

    request_MC_speed();
    delay(100);
    request_MC_torque();

	// adc setup
	adc_setup();

	//set up hardware interrupt for reading throttle
	Timer3.attachInterrupt(sendThrottle).setFrequency(1).start();
    Timer4.attachInterrupt(request_MC_temperature).setFrequency(1).start();
    Timer5.attachInterrupt(updateDB).setFrequency(1).start();
}

void loop() {

    CAN_FRAME incoming;

    if (CAN.rx_avail()) {
        CAN.get_rx_buff(incoming); 
        parseFrame(incoming);
    }
    if (CAN2.rx_avail()) {
        CAN2.get_rx_buff(incoming); 
        parseFrame(incoming);
    }

}