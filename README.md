# EV2

## EV2.ino

The microcontroller(MCU) runs the code from file with .ino extension.
In general any arduino code has 2 main functions: setup and loop.
Function setup contains the code that runs once at the start.

The setup has calls to many functions:

* The `CAN_setup` function that initialises and setups the CAN Rx and Tx buffers.

* By calling the `EV2_setup` function it setups the hardware interrupts. The hardware interrupts include:
	* Startup switch: If a 0 is received on this pin while the car is in drive state (the drive state is set when the this pin along with Tractive signal active (TSA) pin is set to high), the MCU sends a 0 to the 2 output motor controllers and sends a 1 to the tractive system shutdown relay. 
	If a 1 is receieved on this pin while the car is in idle state, the MCU sends a 1 to the tractive system shutdown relay. 

	* Isolation fault pin which is currently only printed to the debbugger and is not used for any other purpose.
	* Battery fault pin which is currently only printed to the debbugger and is not used for any other purpose.

	* Tractive system active (TSA) pin which causes an emergency stop of the car when this pin goes from high to low.

* MotorController (MC) setup (`MC_setup` function) that sets the 2 MC pins as well as the Tractive system shutdown relay pins as OUTPUT pins.

* `adc_setup` function that setups the ADC and enables the ADC channels for the 2 pedals, break pedal, LV, HV, Current measuring and temprature sensors. 

* Setup a timer to check MC comms (which involves checking Motor speed, torque, current and voltage), check temprature, check throttle, and the MC core status at a rate of 2 Hz.

* Setup a timer to check if new CAN messages have been received at a rate of 1 Hz.

Function `loop` has the main code that runs forever. This is similar to if the function  was called like: 
	
	while(1){
		loop();
	} 

The `loop` function continuosly checks if a CAN frame is received from either of the CAN instances (see below), parses the frame in the `parseFrame` function in `EV2_CAN.cpp` to deal with the data being received.

Following are the schematics for the pin configurations of the old microcontroller:
![alt text](images/PCB%20image%20old%20microcontroller.png)

![alt text](images/PCB%20image%20old%20pedal.png)

Following are the schematics for the pin configurations of the old microcontroller:

![alt text](images/PCB%20image.png)

![alt text](images/PCB%20image%20zoomed.png)



## CAN

The below definition is from [wikipedia](https://en.wikipedia.org/wiki/CAN_bus).

A Controller Area Network (CAN bus) is a robust vehicle bus standard designed to allow microcontrollers and devices to communicate with each other in applications without a host computer. It is a message-based protocol, designed originally for multiplex electrical wiring within automobiles to save on copper, but is also used in many other contexts.

[This](http://www.eeherald.com/section/design-guide/esmod9.html) is a good read on CAN. I recommend you read it till **CAN code Error detection and correction** (should take around 15 minutes) before progressing in code.	

The code for the micorontroller that configures, enables, disables, etc the CAN
bus is in the **due_can.h and .cpp** files. The class that does all the above is called CANRaw. There are currently 2 instance of CANRaw (called CAN and CAN2) that are used throughout the code. The data is sent and received using a struct called CAN_FRAME.

The sn65hvd234 is a device on the pcb on whihc the arduino is. It is CAN transceiver. Look at figure 1-b in the [eeherald](http://www.eeherald.com/section/design-guide/esmod9.html) link. There isnt a lot in the driver implementation for this in the sn65hvd234.h and sn65hvd234.cpp files.

The EV2_CAN.h and .cpp files have the code for doing the majority of the functions of the mictocontroller from creating request frames for temprature, spped, torque, etc to getting pedal reading and sending throttle.
