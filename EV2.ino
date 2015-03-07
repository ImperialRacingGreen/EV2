#include "EV2_CAN.h"
#include "DueTimer.h"


void sendThrottle()
{
    int speed = get_average_pedal_reading_value();

    CAN_FRAME outgoing;
    createSpeedWriteFrame(outgoing,speed);
    CAN.sendFrame(outgoing);
}

void setup() {
	CAN_setup();

	CAN_FRAME speedRequest;
	int repetition = 100; // 100ms
	createSpeedRequestFrame(speedRequest,repetition);

	// adc setup
	adc_setup();

	//set up hardware interrupt for reading throttle
	Timer3.attachInterrupt(sendThrottle).start(1000);
}

void loop() {
    CAN_FRAME incoming;

    if (CAN.rx_avail()) {
        CAN.get_rx_buff(incoming); 
        printFrame(incoming);
        parseFrame(incoming);
    }
    if (CAN2.rx_avail()) {
        CAN2.get_rx_buff(incoming); 
        printFrame(incoming);
        parseFrame(incoming);
    }
}