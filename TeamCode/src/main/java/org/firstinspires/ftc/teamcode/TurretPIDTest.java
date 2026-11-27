package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "Turret PID Test", group = "Test")
public class TurretPIDTest extends LinearOpMode {

@Override
public void runOpMode() {

// Example PID constants
double kP = 0.02;
double kI = 0.0001;
double kD = 0.001;

TurretPID pid = new TurretPID(kP, kI, kD);

// Simulated turret positions
double targetPosition = 100;
double currentPosition = 60;

waitForStart();

if (opModeIsActive()) {

double output = pid.calculate(
targetPosition,
currentPosition
);

double error = targetPosition - currentPosition;

telemetry.addData("Target Position", targetPosition);
telemetry.addData("Current Position", currentPosition);
telemetry.addData("Position Error", error);
telemetry.addData("PID Output", output);

telemetry.update();

while (opModeIsActive()) {
idle();
}
}
}
}