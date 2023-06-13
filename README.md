# motor_control
This project is based on ...

## Getting started

- Clone the repository
- Import STM32 project and build it
- Flash device and try it out

## Description

![SW block diagram](./documentation/images/motor_control_sw_block_diagram.jpg)

### CAN Communication Module

The CAN communication module, used for motor control communication, performs the following tasks:

- Initialization
    - Initializes the interface with specific parameters like prescaler, synchronization jump width, and time segments.
    - Configures the receive message structure with filter settings for incoming messages.

- Transmitting Data
    - Initializes the transmit message structure with parameters like data length code (DLC), identifier type, remote transmission request (RTR) type, and recipient ID.
    - Packs data into a reply packet by converting the data from floating-point format to the transmit data format.

- Receiving Data
    - Unpacks received command packets by extracting the data.
    - Converts these values from the received format to floating-point for further processing.


