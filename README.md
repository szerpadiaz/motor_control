# motor_control
This project is based on https://github.com/bgkatz/motorcontrol

## Getting started

- Clone the repository
- Import STM32 project and build it
- Flash device and try it out

## Board connections

![Board connections](./documentation/images/Board_connections/Board_connections.001.jpeg)

## Description

![SW block diagram](./documentation/images/motor_control_sw_block_diagram.jpg)

### The Finite State Machine (FSM) module

The FSM is responsible for managing different states of the motor control system and transitioning between them based on certain conditions.

- Manages the state transitions based on the current state and the input signals (e.g. serial-terminal input, CAN input, and other triggrers). 
- Depending on the state, perform actions when entering and exiting the current state. Those actions perform any necessary setup or cleanup operations associated with the states.

The FSM consists of the following states:
  - **MENU_MODE**: In this state, the user can select different options from the main menu.
  - **SETUP_MODE**: In this state, the user can configure various parameters and settings related to the motor control.
  - **ENCODER_MODE**: This state is used for displaying encoder-related information.
  - **MOTOR_MODE**: This state represents the motor control mode where the motor is driven and controlled.
  - **CALIBRATION_MODE**: This state is used for calibrating the encoder.

The diagram below shows the state actions and transitions.

![FSM](./documentation/images/FSM.png)

### Field-Oriented Control (FOC) module

This module  utilizes the Field-Oriented Control technique to regulate torque and speed in three-phase AC motors. The key functions are:

- **Initialization**: 
  - set up parameters such as gains, scaling factors, and lookup tables.
- **Command Initialization**: 
  - set control commands such as position, velocity, gains, and torque to zero.
- **ADC Offset Measurement**: 
  - measure ADC offset values to establish a zero current condition.
- **FOC execution**: 
  - computing the motor's currents in the DQ coordinate system.
  - applying proportional-integral (PI) control.
  - generating suitable PWM signals for commutation.
- **Duty Cycle Adjustment**: 
  - adjust duty cycle values for the motor's three phases
  - inverte the duty cycle and/or swap phase orders.
- **Current and Voltage Sensing**:
  - read the current and voltage values from the motor phases using ADC channels.
- **Coordinate Transformations**: 
  - convert coordinates between DQ and ABC systems
- **Space Vector Modulation (SVM)**: 
  - generate PWM signals for the motor based on the desired voltages for the U, V, and W phases.
- **Duty Cycle Linearization**: 
  - linearize the duty cycle using a lookup table.
- **Field Weakening**: 
  - control the motor in high-speed regions beyond the base speed.
- **(Optional) Observer Implementation**: 
  - estimate resistance and temperature by leveraging the measured current and voltage values.

### Motor position sensor (encoder) module:

- **Initialization**:
    * Clears noise data during startup.
- **Obtaining position**:
    - Shifts previous angle samples to make room for new samples.
    - Reads raw value from the encoder.
    - Performs linearization using a lookup table and interpolation.
    - Calculates real angles in radians for single-turn and electrical angle.
    - Handles rollover of the single-turn angle by comparing the difference with the previous angle
    - Updates multi-turn position by adding the single-turn angle with the number of turns.
- **Velocity calculation**:
    - Calculates velocity based on angle change and elapsed time.
    - Also calculates electrical velocity.

### Motor driver module

This module control the DRV8323 IC which is a three-phase gate driver for brushless DC (BLDC) motors. It provides the control signals to drive the power MOSFETs that control the motor's phases.

Some important functions are:

- Read fault status registers
- Configure control registers to set operating parameters (e.g.  motor speed, torque)
- Enable or disable the gate drive that controls the motor's power stage. This allows for seamless motor startup, shutdown, and control, depending on the operational state.
- Perform calibration for accurate current sensing. This helps to optimize the motor's performance and efficiency by accurately measuring and regulating the motor's current flow.

### CAN communication module

The CAN communication module, used for motor control communication, performs the following tasks:
- **Initialization**:
    - Initializes the interface with specific parameters like prescaler, synchronization jump width, and time segments.
    - Configures the receive message structure with filter settings for incoming messages.

- **Transmitting data about the motor controller's position, velocity, and current**:
    - Initializes the transmit message structure with parameters like data length code (DLC), identifier type, remote transmission request (RTR) type, and recipient ID.
    - Packs data into a reply packet by converting the data from floating-point format to the transmit data format.

- **Receiving data such as motor control commands (position, velocity) and control parameters (proportional gain (kp), derivative gain (kd), and feed-forward torque)**:
    - Unpacks received command packets by extracting the data.
    - Converts these values from the received format to floating-point for further processing.

### Serial communication module

This module provides functionality for the serial communication:

- **Initialization**: 
    - Initialize USART peripheral with specific settings, such as a baud rate of 9600, word length of 8 bits, stop bits of 1, no parity, no hardware flow control, and an oversampling ratio of 16. 
    - Enable the reception interrupt
- **Standard Library Function**: 
    - Implement the `putchar` function from the standard library. This function allows redirecting `printf` statements to the USART interface.
