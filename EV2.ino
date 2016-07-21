#include "EV2_CAN.h"
#define Serial SerialUSB

void pin_setup(void) {
    // Inputs
    pinMode(DBSSTSD, INPUT);
    pinMode(BATFLT, INPUT);
    pinMode(TSA, INPUT);
    pinMode(IMDSTIN, INPUT);
    pinMode(IMDDIN, INPUT);

    // Outputs
    pinMode(SPKR, OUTPUT);
    pinMode(MCRFE, OUTPUT);
    pinMode(MCFRG, OUTPUT);
    pinMode(TSDUESD, OUTPUT);

    // pinMode(DAC1, OUTPUT);
    // analogWriteResolution(12);
}

void MC_setup(void) {  
    set_rfe_frg(false,false);
    set_tracsys_relay(true);
}

void EV2_setup(void) {
    attachInterrupt(DBSSTSD, setDriveState, CHANGE);
    attachInterrupt(BATFLT, inputChanged, CHANGE);
    attachInterrupt(IMDSTIN, inputChanged, CHANGE);
    attachInterrupt(TSA, tsaChanged, CHANGE);

    inputChanged();
    tsaChanged();
}

void request_temperatures(void) {
    CAN_FRAME MCtempRequest;
    createMCTempRequestFrame(MCtempRequest);
    Can1.sendFrame(MCtempRequest); 

    CAN_FRAME MCmotortempRequest;
    createMotorTempRequestFrame(MCmotortempRequest);
    Can1.sendFrame(MCmotortempRequest);  
}

void request_MC_status(void) {
    CAN_FRAME MCstatusRequest;
    createCoreStatusRequestFrame(MCstatusRequest);
    Can1.sendFrame(MCstatusRequest);     
}

void MC_request(void) {
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
    pin_setup();
    while(!CAN_setup()){};
    EV2_setup();

    MC_setup();
    MC_request();

    // adc setup
    adc_setup();

    // Set car to IDLE state
    setIdleState();

    int16_t analogOutput = 0;
    // analogWriteResolution(12);
    analogWrite(DAC1, analogOutput);

    //set up hardware interrupt for reading throttle
    Timer4.attachInterrupt(checkForFaults).setFrequency(1).start();
    Timer5.attachInterrupt(updateDB).setFrequency(5).start();
    // Timer8.attachInterrupt(clearErrors).setFrequency(0.5).start();
    
    Serial.begin(115200);
    updateDB();
    // Timer5.attachInterrupt(updateTime).setFrequency(50).start();
    delay(3000);

    pinMode(23, OUTPUT);
    pinMode(22, OUTPUT);
    pinMode(52, OUTPUT);
}

int time_x = 0;
void loop() {
    checkCANComms();

    // if(time_x == 0){
    //     digitalWrite(52, HIGH);
    //     updateDB();
    //     digitalWrite(52, LOW);
    // }
    
    CAN_FRAME incoming;
    if (Can0.available()) {
        digitalWrite(23, HIGH);
        Can0.read(incoming); 
        parseFrame(incoming);
        digitalWrite(23, LOW);
    }
    if (Can1.available()) {
        digitalWrite(22, HIGH);
        Can1.read(incoming); 
        parseFrame(incoming);
        digitalWrite(22, LOW);
    }
}

void updateTime() {
    time_x += 1;
    time_x = time_x % 50;
}