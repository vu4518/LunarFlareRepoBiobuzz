package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@TeleOp(name = "Live Turret Math Test", group = "Test")
public class TurretMathTest extends LinearOpMode {

GoBildaPinpointDriver pinpoint;

@Override
public void runOpMode() {

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

telemetry.addLine("Pinpoint initialized");
telemetry.update();

waitForStart();

while (opModeIsActive()) {

pinpoint.update();

double robotX = pinpoint.getPosX(DistanceUnit.INCH);
double robotY = pinpoint.getPosY(DistanceUnit.INCH);
double heading = pinpoint.getHeading(AngleUnit.DEGREES);

double angle = TurretMath.calculateTurretAngle(
robotX, robotY, heading, 30, 60
);

double distance = TurretMath.calculateDistance(
robotX, robotY, 30, 60
);

telemetry.addData("Robot X", robotX);
telemetry.addData("Robot Y", robotY);
telemetry.addData("Heading", heading);
telemetry.addData("Desired Turret Angle", angle);
telemetry.addData("Distance to Target", distance);

telemetry.update();
}
}
}