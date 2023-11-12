struct PID{
    double Kp;
    double Ki;
    double Kd;
    double integral;
    double prevError;
    double limitIntegral;
    double maxOutput;
    double minOutput;
}

void initPID(double Kp, double Ki, double Kd, double limitIntegral, double minOutput, double maxOutput);
double calculatePID(PID &pid, double destination, double current);