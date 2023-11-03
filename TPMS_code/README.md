# Telemetry_code SX1262
The code is written for the STM32 MCU family - specifically used STM32F4
Telemetry_code includes SX1262_Radio_Transceiver Drivers (Based on the Semtech drivers) for the telemetry system of an FSAE race car.


Most custom libraries are included in the following paths:
Telemetry_code\Core\Inc
Telemetry_code\Core\Src


The SX1262 Drivers are:

* SX1262.c / SX1262.h : They include the basic structs / functions for the operation of the SX1262. They can be used by any MCU.
* SX1262_hal.c / SX1262_hal.h: They include low level functions pertaining the SPI communication with the HAL libraries. They can be used by any STM32.
* SX1262_board.c & SX1262_board.h: They include functions that have to be initialized accordinf to the specific PCB design of each application.


Also, there are other libraries concerning the operation of the telemetry system:

* TeleMsgs_USB.h / TeleMsgs_USB.c : Responsible for receiving all the data from the race car using the CAN bus protocol and creating packets for the wireless transmission. Also, these libraries extract the information from the packets when they are received and transmit them through USB to a connected laptop using the Json format, in order to be displayed to a GUI


- If the code is intended for a transmitting node, the variable FUNCTION has to be defined equal to TELEMETRY_TX (= 0) in the SX1262-board.h file
- If the code is intended for a receiving node, the valiable FUNCTION has to be defined equal to TELEMETRY_RX (= 1) in the SX1262-board.h file



-------------- EXTRA FUNTIONS OF THE TELEMETRY SYSTEM ----------------

The telemetry has the ability to communicate with custom made Tire Pressure Monitoring System (TPMS) PCBs and get useful data like tire pressure and temperature. The TPMS are mounted inside the car's wheels so they transmit their measurements wirelessly to the telemetry antenna on the car, which receives their messages. When the telemetry board on the car is supplied, it transmits 2 different types of messages with different sync words every 20ms. The first 50-Bytes packet is intended for the receiving node (normal telemetry receiver) while the second 5-Bytes packet is intended for the TPMS. This message informs all four TPMS on when the next Rx window of the telemetry will be. When the Rx window time arrives, the telemetry dedicates 120ms for Rx and receives the messages from the TPMS.
Since the radio transceiver is always the SX1262, the same libraries are used for the telemetry receiver, the telemetry transmitter (which communicates with the TPMS) and the TPMS boards. 

In order to differentiate between the different functionalities, the variable FUNCTION inside the SX1262-board.h file has to be defined. The possible values are:

TELEMETRY_TX:   The code is intended for a Telemetry transmitter (without connection to TPMS)
TELEMETRY_RX:   The code is intended for a Telemetry receiver   
TELEMETRY_TPMS: The code is intended for a Telemetry transmitter, able to connect to the TPMS and get their data
TPMS_DEBUG:     The code is intended for the TPMS MCU (STM32L412). The consumption is not down to minimum because diagnostic / debugging functions are still running
TPMS_RELEASE:   The code is intended for the TPMS MCU and the consumption is brought down to absolute minimum. Only essential functions run
STNBY: 		The code does none of the above and the operation is defined by the user


If the FUNCTION is defined as TELEMETRY_TPMS, the following libraries are also needed:
* TPMS.h / TPMS.c : They include functions to change the configuration of the telemetry board from telemetry tramsnitter to TPMS transmitter / receiver.

The project of the TPMS module can be found inslide the TPMS_code folder