#include <PID.h>

void initPID(PID &pid,double Kp, double Ki, double Kd, double limitIntegral, double minOutput, double maxOutput){
    pid.Kp = Kp;
    pid.Ki = Ki;
    pid.Kd = Kd;
    pid.limitIntegral = limitIntegral;
    pid.minOutput = minOutput;
    pid.maxOutput = maxOutput;

}

double calculatePID(PID &pid, double destination, double current){
    double errror;
    double proportionalGain;
    double integralGain;
    double derivativeGain;

    error = destination - current;
    proportionalGain = pid.Kp * error;

    if(error < pid.limitIntegral){
        pid.integral = pid.integral + error;
    }
    else{
        pid.integral = 0;
    }
    integralGain = pid.Ki * pid.integral;
    
    derivativeGain = (error - pid.prevError) * pid.Kd;

    pid.prevError = error;
    return proportionalGain + integralGain + derivativeGain;
}