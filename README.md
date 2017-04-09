# SSI VALBAL:
### A cost effective endurance platform using a valve and ballast controller for high altitude balloons

# Flight States:
The avionics uses an Event Driven Programming model in order to clearly transition between states of operation.

#### States
1. Pre-Launch  
 - 1a. Startup initialization
 - 1b. Debug Mode
2. Launch
 - 2a. Flight Mode
 - 2b. Equilibration
3. Termination
 - 3a. Descent
 - 3b. Recovery

# Code Architecture:
The avionics flight software operates on a read-eval loop in order to change states and respond to its environment.

The avionics flight software was written in compliance with NASA JPL's  Safety-Critical Code standards.

#### Files
`main.cpp` - Start point of flight controller.

`config.h` - Mission specific configuration values.

`data.h` - Structure of current data frame.

#### Classes
`Avionics` - Implementation of flight logic.

`Sensors` - Interface to raw data from hardware.

`Filters` - Interface to corrected data from filters.

`Hardware` - Interface to PCB mechatronics.

`Controller` - Interface to feedback control algorithm.

`Parser` - Decompression algorithm for satcomms.

`Simulator` - Client side code for Hardware in the Loop simulations.

#### Libraries
`GPS` - Wrapper library with added features.

`RockBLOCK` - Wrapper library with added features.

# Implementation Details:
Here is the current status of the code:

#### Flight Critical Systems
1. MicroSD logging of current data frame to LOGGERXX.txt.
2. Altitude readings from filtered and error checked BMP280s.
3. Valve mechanical actuation
4. Ballast mechanical actuation
5. Feedback control algorithm to equilibrate at altitude
6. PID Heating on inboard heater trace.
7. Integration of uBlox M8Q GPS.
8. RockBlock data downlink.
9. Flight termination optionally based on altitude and GPS setpoints.
10. 1Hz LED in compliance with FAA.

#### Useful Flight Features
1. MicroSD logging of errors and notable events to log.txt.
2. Ascent rate calculations from filtered and error checked data.
3. Balloon neck temperature readings.
4. Subsystem current readings.
5. Subsystem shutdown if failure to restart.
6. Low power mode for ARM Cortex M4.
7. GPS successful set to flight mode.
8. Compression of data frame into bitstream for comms.
9. RockBlock command parsing for satcomms uplink.
10. Manual control of flight Parameters.
11. Debug mode disabled at altitude.
12. HITL simulations testing suite.
