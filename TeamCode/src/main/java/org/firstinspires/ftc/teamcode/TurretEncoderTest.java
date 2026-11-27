package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "Turret Encoder Test", group = "Test")
public class TurretEncoderTest extends LinearOpMode {

private DcMotor turret;

@Override
public void runOpMode() {

turret = hardwareMap.get(DcMotor.class, "Turret");

telemetry.addLine("Turret encoder connected!");
telemetry.update();

waitForStart();

while (opModeIsActive()) {

int position = turret.getCurrentPosition();

telemetry.addData("Turret Encoder", position);
telemetry.update();
}
}
}
