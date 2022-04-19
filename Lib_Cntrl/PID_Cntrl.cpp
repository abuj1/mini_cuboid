#include "PID_Cntrl.h"

// Matlab
// Tn = .005;
// Gpi= tf([Tn 1],[Tn 0]);
// Kp = 0.0158;
// pid(Kp*Gpi);

PID_Cntrl::PID_Cntrl(float P, float I, float D, float tau_f, float Ts, float uMin, float uMax)
{
    setup(P, I, D, tau_f, Ts, uMin, uMax);
}

void PID_Cntrl::setup(float P, float I, float D, float tau_f, float Ts, float uMin, float uMax)
{
    // ------------------
    this->kp = P;
    this->ki = I;
    this->kd = D;
    this->tau_f = tau_f;
    this->Ts = Ts;
    this->uMin = uMin;
    this->uMax = uMax;
    reset(0);
}
void PID_Cntrl::setCoefficients(float P, float I, float D, float tau_f, float Ts, float uMin, float uMax)
{
    // ------------------
    this->kp = P;
    this->ki = I;
    this->kd = D;
    this->tau_f = tau_f;
    this->Ts = Ts;
    this->uMin = uMin;
    this->uMax = uMax;
    reset(0);
}

PID_Cntrl::~PID_Cntrl() {}

void PID_Cntrl::reset(float initValue)
{
    // -----------------------
    Ipart = initValue;
    e_old = 0;
    
}


float PID_Cntrl::update(float e)
{
    // the main update 
    float Ppart = kp * e;
    Ipart = Ipart + ki*Ts/2.0f * ( e + e_old);
    e_old = e;
    Ipart = saturate(Ipart);
    Dpart = 0;      // will follow

   return saturate(Ppart + Ipart + Dpart); 
   
}

float PID_Cntrl::saturate(float x)
{
if(x > uMax)
    return uMax;
else if(x < uMin)
    return uMin;
return x;
}