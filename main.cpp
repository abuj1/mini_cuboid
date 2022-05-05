/*
    Boulus Abu Joudom
    Cube Mini (Mini Cuboid) - Prototype 3
    Mbed Version: April 28, 2022
    ZHAW - IMS - RT 
 
    Cubiod Balancing Experiment - Control Theory Lab
    Template Version: ??
    Microcontroller: Nucleo-L432KC
*/
 
/*
 
    // Physical Model of Cube Mini
 
    % Mass Parameters
    Cube Mini - Mass = 0.6 + 0.170 + 0.015 = 0.7850 Kg, Mass of the Full Cuboid with Motor and Flywheel
    Cube Mini - Flywheel Mass = 0.170 kg
    Cube Mini - Without Flywheel Mass = 0.6150 kg
 
    % Dimensions Parameters
    % Cube Mini Dimensions (10x10x10) cm
    Cube Mini - Length = 0.90*100e-3 m (The multiplication by 0.9 is to compensate for the rounded edges)
    Cube Mini - CoM = Cube Mini - Length * 0.5,  Center of Mass Location
    % Inertia Parameters
    Cube Mini - Inertia (J) of the Body from the edge Edge = 0.003044 Kg.m^2,  Inertia about the edge
    Cube Mini - Inertia (J) of Flyhweel and Rotor from Center = (0.0002104064 + 0.0000181) kg.m^2,
 
    % Motor Parameters
    % Motor: Maxon Flat EC45, Part number: 411812, Specs: 24V, Km 36.9e-3
    Cube Mini - Motor.Km = 36.9e-3 % Torque Constant % Unit: Nm/A
    Cube Mini - Motor.Resistance = 0.608 % Terminal Resistance % Unit: Ohm
    Cube Mini - Motor.Inductance = 0.463e-3 % Unit: H, Henry
 
    % PI Controller Escon
    Cube Mini - Escon.PI_Tn = 500e-6; %945e-6
    Cube Mini - Escon.PI_Kp = 100*(10/2^11);%576*(10/2^11)
 
    % RC Physical Velocity Voltage Filter
    Filter.Velocity.R = 1e3 Ohm
    Filter.Velocity.C = 10e-6 F
 
    Settings for Maxon ESCON controller (upload via ESCON Studio)
    Escon Studio file Location: ...
 
    Hardware Connections
        Serial PC Communication
            UART_TX     USBTX
            UART_RX     USBRX
        IMU MPU6500 SPI Connection
            MOSI PA_12
            MISO PA_11
            SCLK PA_1
            CS   PB_0

        Velocity Encoder Connections
            Timer 1:
                Encoder A: PA_8
                Encoder B: PA_9
            Timer 2: 
                Encoder A: PA_0 
                Encoder B: PB_3

        Velocity Input as Analogue Signal from Escon
            Pin: PA_6
        Button Pin
            Pin: PA_10
        Current setpoint output (desired current) to Escon through analog output
            Pin: PA_4
 
    Escon Mapping of voltage to current and rotational velocity to voltage
        Escon Linear Mapping of input current from voltage is 0V is -15A and 3.3V is 15A, hence 1.65V is 0A !!
        Escon Linear Mapping of output rotational velocity (in rpm) to voltage is -4000rpm is 0V and 4000rpm is 3.0V, hence 1.5V is 0 RPM !!!
 
    Notes
        The Maximum output Current allowed is 15A and the Minimum is -15A !!!
        Escon Linear Mapping of input current from voltage is 0V is -15A and 3.3V is 15A, hence 1.65V is 0A !!
        All needed Libraries for the IMU (Accelerometer and Gyroscope) are included.
        The PID is to be written by students. Use PID_Cntrl.cpp and PID_Cntrl.h as templates
        ...
 
 
*/
// Libraries
#include "mbed.h"
#include <stdint.h>
#include "math.h" 
#include "ControllerLoop.h"
#include "sensors_actuators.h"
#include "uart_comm_thread.h"
#include "state_machine.h"

 
static BufferedSerial serial_port(USBTX, USBRX);

// -------------------------------
// Initialisation of Variables
// -------------------------------

float Ts = 0.007f;    // sampling time, typically 143 Hz

// -------------------------------
//---------- main loop -----------
// -------------------------------

int main()
{
    
    // User-Defined Functions
    sensors_actuators hardware(Ts);           // All the physical IOs are handled in this class
    ControllerLoop loop(&hardware, Ts);       // Main controller loop
    state_machine sm(&hardware, &loop, 0.02); // Button states are defined in this class
    thread_sleep_for(200);
    // -------------------------------

    // Serial Communication Initialisation
    serial_port.set_baud(115200);
    serial_port.set_format(8, BufferedSerial::None, 1);
    serial_port.set_blocking(false); // force to send whenever possible and data is there
    // ----------------------------------

    // Setup for starting the controller and the button state machine loops
    loop.start_loop();
    thread_sleep_for(20);
    sm.start_loop();
    // ----------------------------------

    // Main loop
    while(1)
        thread_sleep_for(200);
        
}   // END OF main


