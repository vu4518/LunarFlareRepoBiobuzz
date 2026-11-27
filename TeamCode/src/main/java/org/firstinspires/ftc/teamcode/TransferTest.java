
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name="Transfer Test", group="TeleOp")
public class TransferTest extends LinearOpMode {

    @Override
    public void runOpMode() {

        DcMotor transfer = hardwareMap.get(DcMotor.class, "transfer");

        transfer.setDirection(DcMotor.Direction.REVERSE);
        transfer.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        telemetry.addData("Status", "Transfer initialized");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            transfer.setPower(0.5);

            telemetry.addData("Transfer Power", 0.5);
            telemetry.update();
        }

        transfer.setPower(0);
    }
}
