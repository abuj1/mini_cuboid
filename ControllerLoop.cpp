#include "ControllerLoop.h"
using namespace std;

#define PI 3.1415927
#define pi 3.1415927

// contructor for controller loop
ControllerLoop::ControllerLoop(sensors_actuators *sa, float Ts) : thread(osPriorityHigh,4096)
{
    this->Ts = std::chrono::milliseconds {static_cast<long int>(1000*Ts)};
    this->m_sa = sa;

    bal_cntrl_enabled = false;
    vel_cntrl_enabled = false;

    // PID (PI Parameters)
    
    // PID 1 - Velocity control after lift-up
    Kp_1 = -0.02;//- 0.01;
    Ki_1 = -0.004;//- 0.05;
    Kd_1 = 0; // No D-Part
    Tf_1 = 1; // No D-Part
    
    // Controller Loop (PI-Part) in Case 2 (breaking case)
    Kp_2 = 0.25/4.0;
    Ki_2 = 0.25/4.0;
    Kd_2 = 0; // No D-Part
    Tf_2 = 1; // No D-Part
    
    // Saturation Parameters
    // PI Controller Limits
    uMin1 = -1.0f; //-5.0f;
    uMax1 = 1.0f; //  5.0f;
    
    // Cuboid Escon Input Limits in Amps
    uMin = -15.0f;        // Minimum Current Allowed
    uMax =  15.0f;        // Maximum Current Allowed
    //flat_vel_cntrl.setup(...);
    //bal_vel_cntrl.setup(...);
    C1.setup(Kp_1, Ki_1, Kd_1, Tf_1, Ts, uMin1, uMax1);
    C2.setup(Kp_2, Ki_2, Kd_2, Tf_2, Ts, uMin1, uMax1);
    C1.reset(0.0f);
    C2.reset(0.0f);

    ti.reset();
    ti.start();

    // 

    /* setup all IIR_filter objects */
    float tau = 0.5f;
    Gaccx .setup(tau, Ts, 1.0f);
    Gaccy .setup(tau, Ts, 1.0f);
    Ggyro .setup(tau, Ts, tau);

    /* write_counter_write_val = 14 means every 14th measurement is send via serial */
    write_counter_write_val = 14;
    write_counter = 0;
    }

// decontructor for controller loop
ControllerLoop::~ControllerLoop() {}

// ----------------------------------------------------------------------------
// this is the main loop called every Ts with high priority
void ControllerLoop::loop(void){
    
    float i_des = 0;
    //uint8_t k = 0;
    
    while(true)
    {
        ThisThread::flags_wait_any(threadFlag);
        mutex.lock();
        // THE LOOP ------------------------------------------------------------
        //printf("HI");

        m_sa->read_sensors_calc_speed();       // first read all sensors, calculate motor speed
        phi1 = estimate_angle();            
        phi1 = phi1*180.0f/pi; // Convert to Degrees

        // ------------------------- Controller -----------------------------
 
        // Switch Statement Maybe?......
        switch(Button_Status) {

            case INITIAL:
                m_sa->enable_escon();
                m_sa->write_current(0.0f);
                C2.reset(0.0f);
                break;

            case FLAT:
                //m_sa->write_current(0.5f);
                PID_Input = 0.0f - m_sa->get_phi();
                PID_Output = C2.update(PID_Input);
                m_sa->write_current(PID_Output);
                break;

            case BALANCE:
                m_sa->write_current(0.25f);
                break;

            default:
                break;

            }   // end switch


        

        /*
        if(++k == 0)        
            //printf("ax: %f ay: %f gz: %f phi:%f, phi1: %0.6f \r\n",m_sa->get_ax(),m_sa->get_ay(),m_sa->get_gz(),m_sa->get_phi(), phi1);

        // -------------------------------------------------------------
        m_sa->disable_escon();
        //m_sa->enable_escon();
        m_sa->write_current(i_des);                   // write to motor 0 
        // handle enable
        }// endof the main loop
        */

        if(++write_counter == write_counter_write_val) {
            write_counter = 0;
            /* write output via serial buffer */
            printf(
                    "ax: %f ay: %f gz: %f phi:%f, phi1: %0.6f, V: %0.2f, Button Status: %d; \r\n",
                    m_sa->get_ax(),m_sa->get_ay(),m_sa->get_gz(),m_sa->get_phi(), phi1, m_sa->get_vphi(),Button_Status);
        
        }
        
        mutex.unlock();
    }
}

void ControllerLoop::sendSignal() {
    thread.flags_set(threadFlag);
}
void ControllerLoop::start_loop(void)
{
    thread.start(callback(this, &ControllerLoop::loop));
    ticker.attach(callback(this, &ControllerLoop::sendSignal), Ts);
}

/* estimate_angle: estimate angle from acc and gyro data. This function would also fit to the "sensors_actuators"- class
but here it is better visible for students. 
*/
float ControllerLoop::estimate_angle(void)
{
    double AccX_g = m_sa->get_ax();
    double AccY_g = m_sa->get_ay();
    double GyroZ_RadiansPerSecond = m_sa->get_gz();
    // ----- Combine Accelerometer Data and Gyro Data to Get Angle ------
    double Cuboid_Angle_Radians =  -1*atan2(-Gaccx(AccX_g), Gaccy(AccY_g)) + Ggyro(GyroZ_RadiansPerSecond) - PI/4.0f; // + 0.7854f

    return Cuboid_Angle_Radians;
    //return 0;
}

void ControllerLoop::enable_vel_cntrl(void)
{
    vel_cntrl_enabled = true;
}
void ControllerLoop::enable_bal_cntrl(void)
{
    bal_cntrl_enabled = true;
}
void ControllerLoop::reset_cntrl(void)
{

}
void ControllerLoop::disable_all_cntrl()
{
    bal_cntrl_enabled = false;
    vel_cntrl_enabled = false;
}
