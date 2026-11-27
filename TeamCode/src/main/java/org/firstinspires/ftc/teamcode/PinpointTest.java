package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@TeleOp(name = "Pinpoint Test", group = "Test")
public class PinpointTest extends LinearOpMode {

GoBildaPinpointDriver pinpoint;

@Override
public void runOpMode() {

pinpoint = hardwareMap.get(
GoBildaPinpointDriver.class, "pinpoint"
);

// Temporary zero offsets.
// We will replace these with measured offsets.
pinpoint.setOffsets(0, 0, DistanceUnit.MM);

pinpoint.setEncoderResolution(
GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD
);

pinpoint.setEncoderDirections(
GoBildaPinpointDriver.EncoderDirection.FORWARD,
GoBildaPinpointDriver.EncoderDirection.FORWARD
);

pinpoint.resetPosAndIMU();

telemetry.addLine("Pinpoint initialized!");
telemetry.update();

waitForStart();

while (opModeIsActive()) {

pinpoint.update();

telemetry.addData("X (inches)",
pinpoint.getPosX(DistanceUnit.INCH));

telemetry.addData("Y (inches)",
pinpoint.getPosY(DistanceUnit.INCH));

telemetry.addData("Heading (degrees)",
pinpoint.getHeading(AngleUnit.DEGREES));

telemetry.update();
}
}
}
