package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@TeleOp(name = "Turret Aimer Test", group = "Test")
public class TurretAimerTest extends LinearOpMode {

private GoBildaPinpointDriver pinpoint;

@Override
public void runOpMode() {

// Connect to Pinpoint
pinpoint = hardwareMap.get(
GoBildaPinpointDriver.class, "pinpoint"
);

pinpoint.setOffsets(0, 0, DistanceUnit.MM);

pinpoint.setEncoderResolution(
GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD
);

pinpoint.setEncoderDirections(
GoBildaPinpointDriver.EncoderDirection.FORWARD,
GoBildaPinpointDriver.EncoderDirection.FORWARD
);

pinpoint.resetPosAndIMU();

// Create our aimer
TurretAimer aimer = new TurretAimer(
0.02, // kP
0.0001, // kI
0.001, // kD
1000, // Example ticks per turret revolution
0 // Example zero position
);

// Hypothetical target coordinates
double targetX = 30;
double targetY = 60;

// Simulated turret encoder reading
double simulatedTurretTicks = 60;

telemetry.addLine("Turret Aimer Ready");
telemetry.update();

waitForStart();

while (opModeIsActive()) {

// Read actual robot position
pinpoint.update();

double robotX =
pinpoint.getPosX(DistanceUnit.INCH);

double robotY =
pinpoint.getPosY(DistanceUnit.INCH);

double heading =
pinpoint.getHeading(AngleUnit.DEGREES);

// Calculate aiming information
aimer.update(
robotX,
robotY,
heading,
targetX,
targetY,
simulatedTurretTicks
);

// Display results
telemetry.addData("Robot X", robotX);
telemetry.addData("Robot Y", robotY);
telemetry.addData("Robot Heading", heading);

telemetry.addData("Target X", targetX);
telemetry.addData("Target Y", targetY);

telemetry.addData(
"Desired Turret Angle",
aimer.getDesiredAngle()
);

telemetry.addData(
"Target Encoder Ticks",
aimer.getTargetTicks()
);

telemetry.addData(
"Simulated Current Ticks",
simulatedTurretTicks
);

telemetry.addData(
"Encoder Error",
aimer.getError()
);

telemetry.addData(
"PID Output",
aimer.getPIDOutput()
);

telemetry.addData(
"Distance to Target",
aimer.getDistance()
);

telemetry.update();
}
}
}