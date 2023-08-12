# Motivation
The main motivation behind this embedded project is to explore the functionalities and capabilities of the MiniCheetah motor controller. We aim to see how it could be adapted for our Dodo robot in the future. Our focus is on understanding this controller in detail. To do this, we have set up a system based on the motor control implementation available at https://github.com/bgkatz/motorcontrol. The primary goal is to thoroughly document this implementation, making it easier for future teams to adapt it for the Dodo robot.

<img src="./Image & Video/System 1.jpeg" alt="Final system" width="70%"/>

[Video demonstation](https://gitlab.lrz.de/dodo/motor_control/-/blob/main/Image%20%26%20Video/Motor%20mode.mp4)


# System overview

The diagrams that follow depict the software architecture and the implementation of the system. For an extended documentation, refer to ...

<img src="./documentation/images/Board_connections/Board_connections.001.jpeg" alt="System overview" width="70%"/>

## System Architecture
<img src="./documentation/images/motor_control_sw_block_diagram.png" alt="System Architecture" width="70%"/>

## Finite State Machine
<img src="./documentation/images/FSM.png" alt="Finite State Machine" width="70%"/>

## Sequence diagram for motor control
<img src="./documentation/images/flow_diagram.png" alt="Sequence diagram for motor control" width="50%"/>

## FOC module
<img src="./documentation/images/FOC_diagram.png" alt="FOC module" width="70%"/>
