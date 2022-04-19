#include "state_machine.h"
using namespace std;

// contructor for controller loop
state_machine::state_machine(sensors_actuators *sa, ControllerLoop *loop, float Ts) : thread(osPriorityNormal,4096)
{
    this->Ts = std::chrono::milliseconds {static_cast<long int>(1000*Ts)};;
    this->CS = INITIAL;
    this->m_sa = sa;
    this->m_loop = loop;
    ti.reset();
    ti.start();
    }

// decontructor for controller loop
state_machine::~state_machine() {}

// ----------------------------------------------------------------------------
void state_machine::loop(void){
    
    while(1)
        {
        ThisThread::flags_wait_any(threadFlag);
        
        // THE LOOP ------------------------------------------------------------
        switch(CS) {

            case INITIAL:
                if(m_sa->key_was_pressed)
                    {
                    printf("switch to FLAT\r\n");
                    m_sa->key_was_pressed = false;
                    m_loop->Button_Status = CS = FLAT;
                    //m_loop->Button_Status = CS;
                    }
                break;

            case FLAT:
                if(m_sa->key_was_pressed)
                    //m_sa->enable_escon();
                    //m_sa->write_current(1);
                    {
                    printf("switch to BALANCE\r\n");
                    //m_sa->disable_escon();
                    //m_sa->write_current(0);
                    m_sa->key_was_pressed = false;
                    m_loop->Button_Status = CS = BALANCE;
                    //m_loop->Button_Status = CS;
                    }
                break;

            case BALANCE:
                if(m_sa->key_was_pressed)
                    {
                    printf("switch to INIT\r\n");
                    m_sa->key_was_pressed = false;
                    m_loop->Button_Status = CS = INITIAL;
                    //m_loop->Button_Status = CS;
                    }
                break;

            default:
                break;

            }   // end switch

        }// endof the main loop
}

void state_machine::sendSignal() {
    thread.flags_set(threadFlag);
}
void state_machine::start_loop(void)
{
    thread.start(callback(this, &state_machine::loop));
    ticker.attach(callback(this, &state_machine::sendSignal), Ts);
}
