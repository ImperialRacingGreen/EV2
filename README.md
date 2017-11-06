# EV2

## EV2.ino

The microcontroller runs the code from file with .ino extension.
In general any arduino code has 2 main functions: setup and loop.
Function setup contains the code that runs once at the start.
Function loop has the main code that runs forever. (this is similar if the function  was called like: while(1){loop();} )

## CAN

The below definition is from [wikipedia](https://en.wikipedia.org/wiki/CAN_bus).

A Controller Area Network (CAN bus) is a robust vehicle bus standard designed to allow microcontrollers and devices to communicate with each other in applications without a host computer. It is a message-based protocol, designed originally for multiplex electrical wiring within automobiles to save on copper, but is also used in many other contexts.

[This](http://www.eeherald.com/section/design-guide/esmod9.html) is a good read on CAN. I recommend you read it till **CAN code Error detection and correction** (should take around 15 minutes) before progressing in code.	

The code for the micorontroller that configures, enables, disables, etc the CAN
bus is in the **due_can.h and .cpp** files. The class that does all the above is called CANRaw. There are currently 2 instance of CANRaw (called CAN and CAN2) that are used throughout the code. The data is sent and received using a struct called CAN_FRAME.

The sn65hvd234 is a device on the pcb on whihc the arduino is. It is CAN transceiver. Look at figure 1-b in the [eeherald](http://www.eeherald.com/section/design-guide/esmod9.html) link. There isnt a lot in the driver implementation for this in the sn65hvd234.h and sn65hvd234.cpp files.

The EV2_CAN.h and .cpp files have the code for doing the majority of the functions of the mictocontroller from creating request frames for temprature, spped, torque, etc to getting pedal reading and sending throttle.
