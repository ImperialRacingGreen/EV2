#include "EV2_CAN.h"

void MC_setup(void) {   
    // 1. RFE (p37) 2. FRG (p35)
    pinMode(37, OUTPUT);
    pinMode(35, OUTPUT);
    set_rfe_frg(false,false);

    // 3. tractive system shutdown relay
    pinMode(33, OUTPUT);
    set_tracsys_relay(true);
}

void EV2_setup(void) {
    // Startup switch
    pinMode(41, INPUT);
    attachInterrupt(41, inputChanged, CHANGE);

    // hardware interrupts for inputs  
    // 1. battery fault (p43)
    pinMode(43, INPUT);
    attachInterrupt(43, inputChanged, CHANGE);
    // 2. isolation fault (p49)
    pinMode(49, INPUT);
    attachInterrupt(49, inputChanged, CHANGE);
    // 3. TSA (p45)
    pinMode(45, INPUT);
    attachInterrupt(45, inputChanged, CHANGE);

    inputChanged();
}

void slow_requests(void) {
    check_MC_comms();
    request_temperatures();
    checkBrakeThrottle();
    request_MC_status();
}

void request_temperatures(void) {
    CAN_FRAME MCtempRequest;
    createMCTempRequestFrame(MCtempRequest);
    CAN.sendFrame(MCtempRequest); 

    CAN_FRAME MCmotortempRequest;
    createMotorTempRequestFrame(MCmotortempRequest);
    CAN.sendFrame(MCmotortempRequest);  
}

void request_MC_status(void) {
    CAN_FRAME MCstatusRequest;
    createCoreStatusRequestFrame(MCstatusRequest);
    CAN.sendFrame(MCstatusRequest);     
}

void MC_request(void) {
    delay(100);
    request_MC_speed();
    delay(100);
    request_MC_torque();
    delay(100);
    request_MC_current();
    delay(100);
    request_MC_voltage();
    delay(100);
}

void setup() {
    CAN_setup();
    EV2_setup();

    MC_setup();
    MC_request();

    // adc setup
    adc_setup();

    // Set car to IDLE state
    setIdleState();

    //set up hardware interrupt for reading throttle
    Timer4.attachInterrupt(slow_requests).setFrequency(2).start();

    // Logging Data
    Timer.getAvailable().attachInterrupt(updateDB3).setFrequency(10).start();
    Serial.begin(115200);
    
    // Timer5.attachInterrupt(updateDB5).setFrequency(10).start();
    // SerialUSB.begin(115200);

    // Timer5.attachInterrupt(updateDB4).setFrequency(10).start();

    MC_request();

    Timer6.attachInterrupt(checkCANComms).setFrequency(1).start();

}

void loop() {
    CAN_FRAME incoming;
    if (CAN.rx_avail()) {
        CAN.get_rx_buff(incoming); 
        // printFrame(incoming);
        parseFrame(incoming);
    }
    if (CAN2.rx_avail()) {
        CAN2.get_rx_buff(incoming); 
        // printFrame(incoming);
        parseFrame(incoming);
    }
}