#include "ControllerLoop.h"
using namespace std;

#define PI 3.1415927
#define pi 3.1415927

//******************************************************************************
//------------------ Control Loop (called via interrupt) -----------------------
//******************************************************************************

// contructors for controller loop
ControllerLoop::ControllerLoop(sensors_actuators *sa, float Ts):
                    thread(osPriorityHigh,4096), 
                    C1(-0.035, -0.004, 0, 1, 0.007f, -3.0f, 3.0f), 
                    C2(0.25/4.0, 0.25/4.0, 0, 1, 0.007f, -3.0f, 3.0f)
{
    this->Ts = std::chrono::milliseconds{static_cast<long int>(1000*Ts)};
    this->m_sa = sa;
    
    // -------------------------------
    // Initialisations
    // -------------------------------

    // Controller Variables: SS Controller Values and Saturation Limits
    // Sate Space Controller Values from Matlab
    K_SS_Controller[0]  = -64.69; // phi1
    K_SS_Controller[1]  = -4.272; // Vphi1
    
    // Escon Input Limits in Amps
    u_min1 = -15.0f;        // Minimum Current Allowed
    u_max1 =  15.0f;        // Maximum Current Allowed
    // -------------------------------
    
    // PID initialisation
    // PID (PI Parameters from Matlab)
    
    // PID 1: Velocity control during balancing (slow controller)
    Kp_1 = -0.02;   // -0.01;
    Ki_1 = -0.004;  // -0.05;
    Kd_1 = 0;       // No D-Part
    Tf_1 = 1;       // No D-Part
    
    // PID 2: Velocity control during breaking (fast controller)
    Kp_2 = 0.25/4.0;
    Ki_2 = 0.25/4.0;
    Kd_2 = 0;       // No D-Part
    Tf_2 = 1;       // No D-Part
    
    // Saturation Parameters
    // PI Controller Limits
    u_min2 = -3.0f;
    u_max2 =  3.0f; 

    // PID Setup
    //C1.setup(Kp_1, Ki_1, Kd_1, Tf_1, Ts, uMin1, uMax1);
    //C2.setup(Kp_2, Ki_2, Kd_2, Tf_2, Ts, uMin1, uMax1);
    C1.reset(0.0f);
    C2.reset(0.0f);
    // -------------------------------

    // Accelerometer and Gyroscope IIR Filters
    float tau = 0.9f;
    Gaccx .setup(tau, Ts, 1.0f); // low pass
    Gaccy .setup(tau, Ts, 1.0f); // low pass
    Ggyro .setup(tau, Ts, tau);  // high pass
    // -------------------------------

    // Serial print related variables
    // Declare how frequenctly do you want the print statement to print
    write_counter_write_val = 14; // every 14th measurement is send via serial 
    write_counter = 0;
    // -------------------------------

    // Timer ti
    ti.reset();
    ti.start();
    // -------------------------------

    // Initialisation of variables
    i_desired = 0;
    is_stuck = 0;
    angle_offset = -2.0f; // degrees

    
    }

// decontructor for controller loop
ControllerLoop::~ControllerLoop() {}

// -------------------------------
// Main Controller Loop
// -------------------------------

// this is the main loop called every Ts with high priority
void ControllerLoop::loop(void){
    
    while(true)
    {
        ThisThread::flags_wait_any(threadFlag);
        mutex.lock();

        // ----- Start of Controller Loop -----

        // Read raw sensor readings and condition them to useful data
        /* Read raw IMU data 
           Read raw encoder data thru quadrature encoder (QDEC) chip and calculate motor speed */
        m_sa->read_sensors_calc_speed();       
        phi1 = estimate_angle();       // Complementary filter (implemented below)  
        phi1_degrees = phi1*180.0f/pi; // Convert to Degrees
        // -------------------------------
        
        // Controller State Machine
        switch(Button_Status) {

            case INITIAL: // desired current is set to zero
                m_sa->enable_escon();
                C1.reset(0.0f);
                C2.reset(0.0f);
                i_desired = 0;
                break;

            case FLAT: // Break flywheel (control flywheel speed to zero)
                PID_Input = 0.0f - m_sa->get_vphi();
                PID_Output = C2.update(PID_Input);
                i_desired = PID_Output;
                break;

            case BALANCE: // // Lift-up and balancing (state space controller)
                // Current controller loop
                // Loop 1
                loop1_output = phi1*K_SS_Controller[0];
                // Loop 2
                loop2_output = (m_sa->get_gz())*K_SS_Controller[1];
                // PI Controller
                PID_Input = 0.0f - m_sa->get_vphi();
                PID_Output = C1.update(PID_Input);
                // System input (desired current)
                i_desired = PID_Output - loop1_output - loop2_output; 
                break;
            case STUCK:
                i_desired = 0;
                C1.reset(0.0f);
                C2.reset(0.0f);
                Button_Status = FLAT;
                
        }  
        // ------------------------------

        // Check if the cuboid is not stuck and is in the correct position to run
        if (stuck() && !is_stuck && Button_Status != INITIAL) {
            Button_Status = STUCK;
            is_stuck = 1;
        } else if (safe_to_run()) {
            saturate_and_write_current(i_desired);
        } else {
            i_desired = 0.0f;
            saturate_and_write_current(i_desired);
        }
        if(abs(m_sa->get_vphi()) < 50.0f && !stuck() && is_stuck && Button_Status != INITIAL) {
            is_stuck = 0;
            Button_Status = BALANCE;
        }
        // ------------------------------

        // Print desired variables
        print_variables(); // Check function to declare the desired variables that you want to print

        // ----- End of Controller Loop -----
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
    double Cuboid_Angle_Radians =  -1*atan2(-Gaccx(AccX_g), Gaccy(AccY_g)) + Ggyro(GyroZ_RadiansPerSecond) - PI/4.0f + angle_offset*0.01745; // + 0.7854f

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

bool ControllerLoop::safe_to_run()
{
    if (phi1_degrees > -60.0f && phi1_degrees < 60.0f) {
        return true;
    }
    else return false; 
}
bool ControllerLoop::stuck()
{
    if ( abs(m_sa->get_vphi()) > 300.0f && (abs(phi1_degrees)<55.0f && abs(phi1_degrees)>35.0f) ) {
        return true;
    } 
    else return false;
}

void ControllerLoop::saturate_and_write_current(float i_desired_)
{
    if (i_desired_ > u_max1) {
        i_desired_ = u_max1;
    }
    else if (i_desired_ < u_min1) {
        i_desired_ = u_min1;
    }
    m_sa->write_current(i_desired_);
}

void ControllerLoop::print_variables() 
{
    if(++write_counter == write_counter_write_val) {
        write_counter = 0;
        // Write output via serial buffer 
        printf(
                "ax: %f ay: %f gz: %f phi:%f, phi1: %0.6f, V: %0.2f, Button Status: %d; \r\n",
                m_sa->get_ax(),m_sa->get_ay(),m_sa->get_gz(),m_sa->get_phi(), phi1_degrees, m_sa->get_vphi(), Button_Status);
    }
}