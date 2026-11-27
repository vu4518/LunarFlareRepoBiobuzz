package org.firstinspires.ftc.teamcode;

public class TurretAimer {

private TurretPID pid;

// Turret calibration values
private double ticksPerRevolution;
private double zeroPosition;

// Store the latest calculations
private double desiredAngle;
private double targetTicks;
private double distance;
private double error;
private double pidOutput;

public TurretAimer(
double kP,
double kI,
double kD,
double ticksPerRev,
double zeroPos
) {
pid = new TurretPID(kP, kI, kD);

ticksPerRevolution = ticksPerRev;
zeroPosition = zeroPos;
}

// Calculate the turret's desired position
public void update(
double robotX,
double robotY,
double robotHeading,
double targetX,
double targetY,
double currentTurretTicks
) {

// Step 1: Calculate desired turret angle
desiredAngle = TurretMath.calculateTurretAngle(
robotX,
robotY,
robotHeading,
targetX,
targetY
);

// Step 2: Calculate distance to target
distance = TurretMath.calculateDistance(
robotX,
robotY,
targetX,
targetY
);

// Step 3: Convert angle into encoder ticks
targetTicks = TurretController.angleToTicks(
desiredAngle,
ticksPerRevolution,
zeroPosition
);

// Step 4: Calculate encoder error
error = TurretController.calculateError(
targetTicks,
currentTurretTicks
);

// Step 5: Calculate PID output
pidOutput = pid.calculate(
targetTicks,
currentTurretTicks
);
}

public double getDesiredAngle() {
return desiredAngle;
}

public double getTargetTicks() {
return targetTicks;
}

public double getDistance() {
return distance;
}

public double getError() {
return error;
}

public double getPIDOutput() {
return pidOutput;
}
}