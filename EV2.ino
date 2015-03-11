#include "EV2_CAN.h"
#include "DueTimer.h"

#define MAX_THROTTLE 65536
#define MAX_MC_SPEED 0x7FFF



void sendThrottle()
{   
    float torque = get_average_pedal_reading_value()*100;
    torque /= MAX_THROTTLE;
    Serial.print("TORQUE (%%) = ");
    Serial.println(torque);
    CAN_FRAME outgoing;
    createTorqueWriteFrame(outgoing,torque/100);
    CAN.sendFrame(outgoing);
}

void setup() {
    // pin37 rfe, 35 frg
    pinMode(37, OUTPUT);
    digitalWrite(37, HIGH);
    pinMode(35, OUTPUT);
    digitalWrite(35, HIGH);

    Serial.begin(115200);
	CAN_setup();

	CAN_FRAME speedRequest;
	int repetition = 100; // 100ms
	createSpeedRequestFrame(speedRequest,repetition);
    CAN.sendFrame(speedRequest);

	// adc setup
	adc_setup();

	//set up hardware interrupt for reading throttle
	Timer3.attachInterrupt(sendThrottle).setFrequency(10).start();
}

void loop() {
    // CAN_FRAME temp;
    // createTempRequestFrame(temp);
    // CAN.sendFrame(temp);

    // delay(100);

    // CAN_FRAME incoming;

    // if (CAN.rx_avail()) {
    //     CAN.get_rx_buff(incoming); 
    //     printFrame(incoming);
    //     parseFrame(incoming);
    // }
    // if (CAN2.rx_avail()) {
    //     CAN2.get_rx_buff(incoming); 
    //     printFrame(incoming);
    //     parseFrame(incoming);
    // }
}