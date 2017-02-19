# SSI VALBAL:
###A cost effective endurance platform using a valve and ballast controller for high altitude balloons

#Flight States:
The avionics uses an Event Driven Programming model in order to clearly transition between states of operation.

#### States
1. Pre-Launch  
 - 1a. Startup initialization
 - 1b. Debug Mode
2. Launch
 - 2a. Flight Mode
 - 2b. Apogee
3. Termination
 - 3a. Descent
 - 3b. Recovery

#Code Architecture:
The avionics flight software operates on a read-eval loop in order to change states and respond to its environment.

The avionics flight software was written in compliance with NASA JPL's  Safety-Critical Code standards.

#### Files
`main.cpp` - Start point of flight controller.

`config.h` - Mission specific configuration values.

`data.h` - Structure of current data frame.

#### Classes
`Avionics` - Implementation of flight logic.

`Sensors` - Interface to filtered data from hardware.

`Hardware` - Interface to PCB mechatronics.

`Controller` - Interface to feedback control algorithm.

#Implementation Details:
Here is the current status of the code:

####Flight Critical Systems
1. MicroSD logging of current data frame to data.txt.
2. Altitude readings from filtered and error checked BMP280.
3. Valve mechanical actuation
4. Ballast mechanical actuation
5. Feedback control algorithm to equilibrate at altitude
6. PID Heating on inboard heater trace.
7. Integration of uBlox M8Q GPS.
8. RockBlock data downlink.
9. Flight termination optionally based on altitude and GPS setpoints.

####Useful Flight Features
1. MicroSD logging of errors and notable events to log.txt.
2. Ascent rate calculations from filtered and error checked data.
3. Current readings.
4. Debug mode disabled at altitude.
5. GPS successful set to flight mode.
6. Compression of data frame into bitstream for comms.
7. RockBlock command parsing for satcomms uplink.
