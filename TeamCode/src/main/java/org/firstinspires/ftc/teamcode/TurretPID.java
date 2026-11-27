package org.firstinspires.ftc.teamcode;

public class TurretPID {

private double kP;
private double kI;
private double kD;

private double integral = 0;
private double previousError = 0;

public TurretPID(double p, double i, double d) {
kP = p;
kI = i;
kD = d;
}

public double calculate(double target, double current) {

double error = target - current;

integral += error;

double derivative = error - previousError;

previousError = error;

double output =
(kP * error) +
(kI * integral) +
(kD * derivative);

return output;
}

public void reset() {
integral = 0;
previousError = 0;
}
}