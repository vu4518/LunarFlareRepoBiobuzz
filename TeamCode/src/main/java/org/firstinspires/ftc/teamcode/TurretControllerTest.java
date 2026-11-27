package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "Turret Controller Test", group = "Test")
public class TurretControllerTest extends LinearOpMode {

@Override
public void runOpMode() {

// Example values (not actual turret measurements)
double desiredAngle = 30;
double ticksPerRevolution = 1000;
double zeroPosition = 0;
double currentTicks = 60;

double targetTicks = TurretController.angleToTicks(
desiredAngle,
ticksPerRevolution,
zeroPosition
);

double error = TurretController.calculateError(
targetTicks,
currentTicks
);

telemetry.addData("Desired Angle", desiredAngle);
telemetry.addData("Target Encoder Ticks", targetTicks);
telemetry.addData("Current Encoder Ticks", currentTicks);
telemetry.addData("Remaining Error", error);

telemetry.update();

waitForStart();

while (opModeIsActive()) {
idle();
}
}
}