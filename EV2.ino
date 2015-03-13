#include "EV2_CAN.h"
#include "DueTimer.h"

#define MAX_THROTTLE 65536
#define MAX_MC_SPEED 0x7FFF

void MC_setup(void)
{
    // pin37 rfe, 35 frg
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
    repetition = 0;
    createTempRequestFrame(tempRequest,repetition);
    CAN.sendFrame(tempRequest);   
}

void setup() {
    
    // MC_setup();

    Serial.begin(115200);
	CAN_setup();

	CAN_FRAME speedRequest;
	int repetition = 100; // 100ms
	createSpeedRequestFrame(speedRequest,repetition);
    CAN.sendFrame(speedRequest);

    CAN_FRAME torqueRequest;
    repetition = 100; // 100ms
    createTorqueRequestFrame(torqueRequest,repetition);
    CAN.sendFrame(torqueRequest);


	// adc setup
	adc_setup();

	//set up hardware interrupt for reading throttle
	Timer3.attachInterrupt(sendThrottle).setFrequency(1).start();
    Timer3.attachInterrupt(request_MC_temperature).setFrequency(1).start();
}

void loop() {
    // CAN_FRAME temp;
    // createTempRequestFrame(temp);
    // CAN.sendFrame(temp);

    // Serial.println("waiting");
    // delay(1000);

    CAN_FRAME incoming;

    if (CAN.rx_avail()) {
        Serial.print("CAN1 ~ ");
        CAN.get_rx_buff(incoming); 
        printFrame(incoming);
        parseFrame(incoming);
    }
    if (CAN2.rx_avail()) {
        Serial.print("CAN2 ~ ");
        CAN2.get_rx_buff(incoming); 
        printFrame(incoming);
        parseFrame(incoming);
    }

}