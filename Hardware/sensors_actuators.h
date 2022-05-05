#pragma once
/* class sensors_actuators
Tasks for students:
    - scale ios correctly
    - define derivative filter correctly
*/
#include <cstdint>
#include "EncoderCounter.h"
#include "EncoderCounterIndex.h"
#include "IIR_filter.h"
#include "LinearCharacteristics.h"
#include "Enc_unwrap_scale.h"
#include "mpu6500_spi.h"


class sensors_actuators
{
public:
    sensors_actuators(float Ts);         // default constructor
    virtual ~sensors_actuators();        // deconstructor
    void read_sensors_calc_speed(void);  // read both encoders and calculate speeds
    float get_phi(void);                 // get angle of motor k
    float get_vphi(void);                // get speed of motor k
    float get_ax(void);
    float get_ay(void);
    float get_gz(void);
    void write_current(float);           // write current to motors (0,...) for motor 1, (1,...) for motor 2
    void enable_escon();
    void disable_escon();
    bool key_was_pressed;
   
private:
    IIR_filter diff;
    ///------------- Encoder -----------------------
    EncoderCounter counter;    // initialize counter on PA_6 and PC_7
    AnalogOut i_des;           // desired current values
    DigitalOut Escon_Enable;
    InterruptIn button;
    mpu6500_spi imu;
    //-------------------------------------
    SPI spi;                    // mosi, miso, sclk
    LinearCharacteristics i2u;
    LinearCharacteristics ax2ax,ay2ay,gz2gz;    // map imu raw values to m/s^2 and rad/s
    Enc_unwrap_scale uw;
    Timer t_but;                            // define button time        // 
    // sensor states
    float phi;          // motor angle /rad
    float Vphi;           // motor speed / rad / s
    float accx,accy,gyrz;       // accelerations and gyroscope
    void but_pressed(void);
    void but_released(void);

};