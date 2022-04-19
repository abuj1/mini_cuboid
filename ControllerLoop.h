#pragma once

#include "mbed.h"
#include "EncoderCounter.h"
#include "LinearCharacteristics.h"
#include "ThreadFlag.h"
#include "PID_Cntrl.h"
#include "sensors_actuators.h"
#include "IIR_filter.h"

#define INITIAL 1
#define FLAT 2
#define BALANCE 3
#define STUCK 3

// This is the loop class, it is not a controller at first hand, it guarantees a cyclic call
class ControllerLoop
{
public:
    ControllerLoop(sensors_actuators *, float Ts);
    virtual     ~ControllerLoop();
    void start_loop(void);
    void enable_vel_cntrl(void);
    void enable_bal_cntrl(void);
    void reset_cntrl(void);
    void disable_all_cntrl();
    
    int Button_Status = 1;                  // User Button Status

private:
    void loop(void);
    Thread thread;
    Ticker ticker;
    ThreadFlag threadFlag;
    Mutex          mutex;
    Timer ti;
    PID_Cntrl flat_vel_cntrl;
    PID_Cntrl bal_vel_cntrl;

    IIR_filter     Gaccx, Gaccy, Ggyro;

    std::chrono::milliseconds Ts;
    bool bal_cntrl_enabled;
    bool vel_cntrl_enabled;
    void sendSignal();
    float estimate_angle();
    sensors_actuators *m_sa;

    float          time, accx, accy, gyro, phi1, phi2, dphi2, M;

    int            write_counter, write_counter_write_val;


    
};
