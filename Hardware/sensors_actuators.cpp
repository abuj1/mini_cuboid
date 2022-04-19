#include "sensors_actuators.h"

#define PI 3.1415927
#define pi 3.1415927
// constructors

sensors_actuators::sensors_actuators(float Ts) : diff(0.05,Ts), counter(PA_8, PA_9),
                            Escon_Enable(PB_1), button(PA_10), i_des(PA_4), uw(4*2048, 16), spi(PA_12, PA_11, PA_1), imu(spi, PB_0)
{
    // Current to Voltage (Eson Input) Linear Scaling
    i2u.setup(-15.0f, 15.0f, 0.0f, 1.0f);

    // Convert Raw data from the IMU to SI Units
    //ax2ax.setup(0, 1, 0, 1);     // use these for first time, adapt values according 
    //ay2ay.setup(0, 1, 0, 1);     // use these for first time, adapt values according      
    ax2ax.setup(-16150, 16680, -9.81, 9.81);     // ±2g: LSB/g: 16,384
    ay2ay.setup(-17440, 15350, -9.81, 9.81);     // ±2g: LSB/g: 16,384
    gz2gz.setup(-32.767f, 32.768f, -1*PI/180, PI/180);     // ±1000º/s: 32.768 LSB/(º/s)
    // Check offset (value at stand-still)

    // --------------------------------------------------
    button.fall(callback(this, &sensors_actuators::but_pressed));          // attach key pressed function
    button.rise(callback(this, &sensors_actuators::but_released));         // attach key pressed function
    key_was_pressed = false;
    Escon_Enable = 0;
    counter.reset();   // encoder reset
    imu.initialize();
    imu.configuration();
    
}
// Deconstructor
sensors_actuators::~sensors_actuators() {} 

void sensors_actuators::read_sensors_calc_speed(void)
{
    //-------------- Read Flywheel Speed ------------
    phi = uw(counter);
    Vphi = diff(phi);
    //-------------- Read IMU ------------
    accx = ax2ax(imu.readAcc_raw(1));
    accy = ay2ay(-imu.readAcc_raw(0));
    gyrz = gz2gz(imu.readGyro_raw(2));
}

void sensors_actuators::enable_escon(void)
{
    Escon_Enable = 1;    
}
void sensors_actuators::disable_escon(void)
{
    Escon_Enable = 0;    
}

void sensors_actuators::write_current(float i_des_)
{
        // Set Voltage output equal to the scaled desired current value
        i_des = i2u(i_des_);   
        
}

float sensors_actuators::get_phi(void)
{
    return phi;
}
float sensors_actuators::get_vphi(void)
{
    return Vphi;
}
float sensors_actuators::get_ax(void)
{
    return accx;
}
float sensors_actuators::get_ay(void)
{
    return accy;
}
float sensors_actuators::get_gz(void)
{
    return gyrz;
}
// start timer as soon as Button is pressed
void sensors_actuators::but_pressed()
{
    t_but.start();
    key_was_pressed = false;
}
 
// evaluating statemachine
void sensors_actuators::but_released()
{
     // readout, stop and reset timer
    float ButtonTime = chrono::duration<float>(t_but.elapsed_time()).count();
    t_but.stop();
    t_but.reset();
    if(ButtonTime > 0.05f && ButtonTime < 0.5) {
        key_was_pressed = true;
        //printf("key_was_pressed: %d; \r\n",key_was_pressed);
    }
}
 