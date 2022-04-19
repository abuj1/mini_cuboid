#include "mbed.h"
#include "sensors_actuators.h"
#include "ControllerLoop.h"

#define INITIAL 1
#define FLAT 2
#define BALANCE 3
#define STUCK 3

// This is the loop class, it is not a controller at first hand, it guarantees a cyclic call
class state_machine
{
public:
    state_machine(sensors_actuators *,ControllerLoop *,float Ts);
    virtual     ~state_machine();
    void start_loop(void);

private:
    void loop(void);
    uint8_t CS;             // the current state
    Thread thread;
    Ticker ticker;
    ThreadFlag threadFlag;
    Timer ti;
    std::chrono::milliseconds Ts;
    void sendSignal();
    sensors_actuators *m_sa;
    ControllerLoop *m_loop;
};
