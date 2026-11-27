package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.AnalogInput;
import java.util.Map;
import java.util.TreeMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

@TeleOp(name = "TeleOp1", group = "Linear Opmode")


public class TeleOp1 extends LinearOpMode {

    GoBildaPinpointDriver pinpoint;

    private DcMotorEx leftFrontDrive;
    private DcMotorEx leftBackDrive;
    private DcMotorEx rightFrontDrive;
    private DcMotorEx rightBackDrive;

    private DcMotorEx intake;
    private DcMotorEx transfer;
    private DcMotorEx flywheel;
    private DcMotor turret;
    private AnalogInput elc;
    private Servo ramp;
    private Servo arm;

    private static final double NORMAL_SPEED = 1.0;
    private static final double TURBO_SPEED = 1.0;
    private static final double SLOW_SPEED = 1.0;

    private static final double RAMP_MAX_ANGLE = 45.0;
    private static final double SERVO_MAX_ANGLE = 180.0;
    private static final double RAMP_SPEED = 45.0;

    //shut the fuck up chat gpt.

    // Turret joystick speed is limited to 25% power.
    private static final double TURRET_POWER = 0.25;
    private static final double TURRET_DEADZONE = 0.08;

    // ARM SETTINGS
    private static final double ARM_REFERENCE_POSITION = 0.5;
    private static final double ARM_SERVO_RANGE = 360.0;
    private static final double ARM_START_ANGLE = 60.0;
    private static final double ARM_MAX_CCW_ANGLE = ARM_START_ANGLE;
    private static final double ARM_SPEED = 45.0;

    // Intake activates after 30% of the arm's clockwise travel.
    private static final double ARM_ACTIVATION_PERCENT = 0.30;

    private double driveSpeed = NORMAL_SPEED;

    // Driver 1 intake controls
    private int intakeMode = 0;
    private boolean driver1IntakeOverride = false;

    // Driver 2 automatic sequence
    private boolean sequenceOn = false;

    // Flywheel starts OFF. Driver 2 RT toggles it ON/OFF.




    private boolean previousDpadUp2 = false;
    private boolean previousDpadDown2 = false;

    private boolean previousRightTrigger1 = false;
    private boolean previousLeftTrigger1 = false;

    private boolean previousRightBumper1 = false;
    private boolean previousLeftBumper1 = false;

    private boolean previousRightTrigger2 = false;

    private double rampAngle = 0.0;
    private double armAngle = -ARM_START_ANGLE;

    private ElapsedTime rampTimer = new ElapsedTime();
    static  TreeMap<Double, Double> SHOT_TABLE = new TreeMap<>();

    @Override
    public void runOpMode() {
        //all important coordinates
        double startX = 36;
        double startY = 12;
        double startHeading = 0;
        double[] hiveX = {66,66};
        double[] hiveY = {60, 84};
        SHOT_TABLE.put(18.0, 1830.0);
        SHOT_TABLE.put(24.0, 1840.0);
        SHOT_TABLE.put(30.0, 1850.0);
        SHOT_TABLE.put(36.0, 1895.0);
        SHOT_TABLE.put(39.0, 1920.0);
        SHOT_TABLE.put(45.0, 1970.0);
        SHOT_TABLE.put(54.0, 2000.0);
        SHOT_TABLE.put(60.0, 2090.0);
        leftFrontDrive = hardwareMap.get(DcMotorEx.class, "left_front_drive");
        leftBackDrive = hardwareMap.get(DcMotorEx.class, "left_back_drive");
        rightFrontDrive = hardwareMap.get(DcMotorEx.class, "right_front_drive");
        rightBackDrive = hardwareMap.get(DcMotorEx.class, "right_back_drive");
        elc = hardwareMap.get(AnalogInput.class, "elc");

        intake = hardwareMap.get(DcMotorEx.class, "intake");
        transfer = hardwareMap.get(DcMotorEx.class, "transfer");
        flywheel = hardwareMap.get(DcMotorEx.class, "flywheel");

        // Read the configured motor's encoder specifications for TPS and RPM.
        turret = hardwareMap.get(DcMotor.class, "turret");

        ramp = hardwareMap.get(Servo.class, "ramp");
        arm = hardwareMap.get(Servo.class, "arm");

        // Drive motor directions
        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotor.Direction.FORWARD);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotor.Direction.REVERSE);
        flywheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        transfer.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        intake.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Mechanism motor directions
        transfer.setDirection(DcMotor.Direction.FORWARD);
        flywheel.setDirection(DcMotor.Direction.REVERSE);
        intake.setDirection(DcMotor.Direction.REVERSE);
        turret.setDirection(DcMotor.Direction.REVERSE);

        pinpoint = hardwareMap.get(
                GoBildaPinpointDriver.class, "pinpoint"
        );
        //pinpoint stuff
        // Temporary zero offsets.
        // We will replace these with measured offsets.
        pinpoint.setOffsets(0, 0, DistanceUnit.MM);

        pinpoint.setEncoderResolution(
                GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD
        );

        pinpoint.setEncoderDirections(
                GoBildaPinpointDriver.EncoderDirection.REVERSED,
                GoBildaPinpointDriver.EncoderDirection.FORWARD
        );

        pinpoint.resetPosAndIMU();

        telemetry.addLine("Pinpoint initialized!");
        telemetry.update();

        // Brake motors when no power is applied
        leftFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        transfer.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        flywheel.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        turret.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //intializing all flywheel variables (because chat gpts method is lowkeygay)
        double flywheelcpr = 28.0;
        double flywheelinputrpm = 2000;
        double flywheeltargetrpm = 0;
        boolean flywheelon = false;
        double flywheelvelocitytps = flywheeltargetrpm*flywheelcpr/60;
        double flywheelactualrpm = (flywheel.getVelocity()*60)/flywheelcpr;
        double flywheelrpmchange = 0;
        double calculatedRpm = 0;

        // Initialize ramp
        rampAngle = 0.0;
        ramp.setPosition(0.0);

        // Initialize arm at 60 degrees counterclockwise
        armAngle = -ARM_START_ANGLE;

        double initialArmPosition = Math.min(1.0,
                ARM_REFERENCE_POSITION +
                        (armAngle / ARM_SERVO_RANGE));

        arm.setPosition(initialArmPosition);

        // Keep the flywheel OFF until Driver 2 presses RT.
        flywheelon = false;
        flywheel.setVelocity(0.0);

        telemetry.addLine("Lunar Flare TeleOp Ready");
        telemetry.addData("Drive Speed", "Normal");
        telemetry.addData("Arm Angle", armAngle);
        telemetry.addData("Arm Position", initialArmPosition);
        telemetry.addData("Sequence", "OFF");
        telemetry.addData("Flywheel", "OFF");
        telemetry.addData("Flywheel Target RPM ",
                flywheeltargetrpm);
        telemetry.addData("Turret", "Ready");
        telemetry.update();

        waitForStart();

        rampTimer.reset();
        double initialElcVoltage = elc.getVoltage();
        double initialElcAngle = (initialElcVoltage / 3.3) * 360.0;
        double turretAngle;
        double totalEncoderMovement = initialElcAngle;
        double targetTurretAngle = -30;
        double turretSpeed = 0;
        double xcoords = 0;
        double ycoords = 0;
        double heading = 0;
        double targetHiveX = 72;
        double targetHiveY = 72;
        boolean previousLeftTrigger2 = false;

        while (opModeIsActive()) {
            //Odometry stuff
            pinpoint.update();
            xcoords = pinpoint.getPosX(DistanceUnit.INCH)+startX;
            ycoords = pinpoint.getPosY(DistanceUnit.INCH)+startY;
            heading = -pinpoint.getHeading(AngleUnit.DEGREES)+startHeading;
            // =====================================
            // DRIVER 1 - DRIVE CONTROLS
            // =====================================

            double axial = -gamepad1.left_stick_y;
            double lateral = gamepad1.left_stick_x;
            double yaw = gamepad1.right_stick_x;
            double speed = 1.0;
            if (gamepad1.right_bumper) {
                speed = 1.0;
            }
            if (gamepad1.left_bumper) {
                speed = 0.50;
            }
            double leftFrontPower = axial + lateral + yaw;

            double rightFrontPower = axial - lateral - yaw;

            double leftBackPower = axial - lateral + yaw;

            double rightBackPower = axial + lateral - yaw;

            // Normalize so no wheel goes above 1.0

            double max = Math.max(

                    Math.abs(leftFrontPower),

                    Math.abs(rightFrontPower)

            );

            max = Math.max(max, Math.abs(leftBackPower));

            max = Math.max(max, Math.abs(rightBackPower));

            if (max > 1.0) {

                leftFrontPower /= max;

                rightFrontPower /= max;

                leftBackPower /= max;

                rightBackPower /= max;

            }
            leftFrontPower *= speed;

            rightFrontPower *= speed;

            leftBackPower *= speed;

            rightBackPower *= speed;
            leftFrontDrive.setPower(leftFrontPower);

            rightFrontDrive.setPower(rightFrontPower);

            leftBackDrive.setPower(leftBackPower);

            rightBackDrive.setPower(rightBackPower);

            // =====================================
            // DRIVER 1 - INTAKE AND TRANSFER
            // =====================================

            boolean rightTrigger1 = gamepad1.right_trigger > 0.5;
            boolean leftTrigger1 = gamepad1.left_trigger > 0.5;

            if (rightTrigger1 && !previousRightTrigger1) {

                driver1IntakeOverride = true;

                if (intakeMode == 1) {
                    intakeMode = 0;
                } else {
                    intakeMode = 1;
                }
            }

            if (leftTrigger1 && !previousLeftTrigger1) {

                driver1IntakeOverride = true;

                if (intakeMode == -1) {
                    intakeMode = 0;
                } else {
                    intakeMode = -1;
                }
            }

            previousRightTrigger1 = rightTrigger1;
            previousLeftTrigger1 = leftTrigger1;


            // =====================================
            // DRIVER 2 - FLYWHEEL SPEED
            // =====================================

            boolean dpadUp2 = gamepad2.dpad_up;
            boolean dpadDown2 = gamepad2.dpad_down;
            if (dpadUp2&flywheelrpmchange<5) {
                flywheelrpmchange+=2;
            }

            if (dpadDown2&flywheelrpmchange>-5) {
                flywheelrpmchange-=2;
            }
            if (flywheelrpmchange>=5&flywheelinputrpm<6000){
                flywheelinputrpm+=5;
                flywheelrpmchange = 0;
            }
            if (flywheelrpmchange<=-5&flywheelinputrpm>1500){
                flywheelinputrpm-=5;
                flywheelrpmchange = 0;
            }


            // =====================================
            // DRIVER 2 - AUTOMATIC SEQUENCE
            // =====================================

            boolean rightTrigger2 = gamepad2.right_trigger > 0.5;

            if (rightTrigger2 && !previousRightTrigger2) {

                sequenceOn = !sequenceOn;
                flywheelon = !flywheelon;

                // Reset manual intake override when sequence toggles
                driver1IntakeOverride = false;
                intakeMode = 0;
            }
            boolean previousrightTrigger2 = rightTrigger2;

            boolean leftTrigger2 = gamepad2.left_trigger > 0.5;

            if (leftTrigger2 && !previousLeftTrigger2) {
                flywheelon = !flywheelon;
            }

            boolean previousleftTrigger2 = leftTrigger2;


            // =====================================
            // SHARED TIMER
            // =====================================

            double elapsedTime = rampTimer.seconds();
            rampTimer.reset();


            // =====================================
            // DRIVER 2 - DEPLOYABLE RAMP
            // =====================================

            boolean rampClockwise = gamepad2.right_bumper;
            boolean rampCounterClockwise = gamepad2.left_bumper;

            if (rampClockwise && !rampCounterClockwise) {
                rampAngle += RAMP_SPEED * elapsedTime;
            } else if (rampCounterClockwise && !rampClockwise) {
                rampAngle -= RAMP_SPEED * elapsedTime;
            }

            rampAngle = Math.min(1.0,Math.max(
                    0.0,
                    Math.min(RAMP_MAX_ANGLE, rampAngle)
            ));

            double rampPosition = rampAngle / SERVO_MAX_ANGLE;
            ramp.setPosition(rampPosition);


            // =====================================
            // DRIVER 2 - AUTOMATIC ARM SERVO
            // =====================================

            if (sequenceOn) {

                // Move clockwise toward 0 degrees.
                if (armAngle < 0.0) armAngle = Math.min(0.0, armAngle + ARM_SPEED * elapsedTime);

            } else {

                // Return to original starting position.
                if (armAngle > -ARM_START_ANGLE) armAngle = Math.max(-ARM_START_ANGLE, armAngle - ARM_SPEED * elapsedTime);
            };
            // Keep arm within its limits.
            armAngle = Math.max(
                    -ARM_MAX_CCW_ANGLE,
                    Math.min(-1.0, armAngle)
            );

            double armPosition = Math.min(1.0, Math.max(
                    0.0,
                    ARM_REFERENCE_POSITION +
                            (armAngle / ARM_SERVO_RANGE)
            ));

            arm.setPosition(armPosition);


            // =====================================
            // AUTOMATIC INTAKE AND FLYWHEEL
            // =====================================

            double activationAngle =
                    -ARM_START_ANGLE * (1.0 - ARM_ACTIVATION_PERCENT);

            boolean automaticIntakeOn =
                    sequenceOn && armAngle >= activationAngle;

            // Driver 1 has priority over automatic intake.
            double intakePower;

            if (sequenceOn && driver1IntakeOverride) {

                if (intakeMode == 1) {
                    intakePower = 1.0;
                } else if (intakeMode == -1) {
                    intakePower = -1.0;
                } else {
                    intakePower = 0.0;
                }

            } else if (automaticIntakeOn) {

                intakePower = 1.0;

            } else if (sequenceOn) {

                intakePower = 0.0;

            } else if (driver1IntakeOverride) {

                if (intakeMode == 1) {
                    intakePower = 1.0;
                } else if (intakeMode == -1) {
                    intakePower = -1.0;
                } else {
                    intakePower = 0.0;
                }

            } else {

                intakePower = 0.0;
            }

            intake.setPower(intakePower);
            transfer.setPower(intakePower);
            if (Math.abs(transfer.getPower())>=0.1&Math.abs(transfer.getCurrent(CurrentUnit.AMPS))>=5&sequenceOn){
                transfer.setPower(intakePower*0.2);
            }

            // =====================================
            // FLYWHEEL POWER
            // =====================================
            if (flywheelon){
                flywheeltargetrpm = flywheelinputrpm;
                if (sequenceOn){
                    calculatedRpm = interpolateAngle(calculateDistance(xcoords,ycoords,targetHiveX,targetHiveY));
                    flywheeltargetrpm = calculatedRpm;
                }
            }else{
                flywheeltargetrpm = 0;
            }
            flywheelvelocitytps = flywheeltargetrpm*flywheelcpr/60;
            flywheelactualrpm = (flywheel.getVelocity()*60)/flywheelcpr;
            flywheel.setVelocity(flywheelvelocitytps);




            // =====================================
            // ELC ABSOLUTE ENCODER
            // =====================================


            double elcVoltage = elc.getVoltage();
            double elcAngle = (elcVoltage / 3.3) * 360.0;
            double previousRawAngle = ((totalEncoderMovement % 360.0) + 360.0) % 360.0;
            double delta = elcAngle - previousRawAngle;

            if (delta > 180.0) delta -= 360.0;
            else if (delta < -180.0) delta += 360.0;

// Accumulate total encoder movement
            totalEncoderMovement += delta;

// Calculate turret displacement relative to startup
            turretAngle = (totalEncoderMovement - initialElcAngle) / -2.75;

            // =====================================
            // DRIVER 2 - TURRET
            // =====================================
            //this is to choose the location of the target
            if(ycoords<96){
                targetHiveX = hiveX[0];
                targetHiveY = hiveY[0];
            }else{
                targetHiveX = hiveX[1];
                targetHiveY = hiveY[1];
            }
            targetTurretAngle = calculateTurretAngle(xcoords,ycoords,heading,targetHiveX,targetHiveY);

            //goofy ass math but its just to slow down as you get closer to the target
            double turretError = targetTurretAngle - turretAngle;

            while (turretError > 180) {
                turretError -= 360;
            }

            while (turretError <= -180) {
                turretError += 360;
            }

            turretSpeed = Math.max(0.1, Math.min(1.0,
                    Math.abs(turretError) / 60.0));

            if (Math.abs(turretError) >= 5) {
                turret.setPower(Math.signum(turretError) * turretSpeed);
            } else {
                turret.setPower(0);
            }



            //turret.setPower(turretInput * TURRET_POWER);


            // =====================================
            // TELEMETRY
            // =====================================

            telemetry.addData("Drive Speed", driveSpeed);

            telemetry.addData("Driver 1 Intake Mode", intakeMode);
            telemetry.addData("Driver 1 Override", driver1IntakeOverride);
            telemetry.addData("Intake Power", intakePower);
            telemetry.addData("Left Back", leftBackPower);
            telemetry.addData("Right Back", rightBackPower);
            telemetry.addData("Right Front", rightFrontPower);
            telemetry.addData("Left Front", leftBackPower);
            telemetry.addData("Sequence", sequenceOn ? "ON" : "OFF");
            telemetry.addData("Automatic Intake",
                    automaticIntakeOn ? "ON" : "OFF");

            telemetry.addData("Flywheel", flywheelon ? "ON" : "OFF");
            telemetry.addData("Flywheel Target RPM", "%.1f", flywheeltargetrpm);
            telemetry.addData("Flywheel Input RPM", "%.1f", flywheelinputrpm);
            telemetry.addData("Flywheel Actual RPM", "%.1f", flywheelactualrpm);

            telemetry.addData("Ramp Angle", "%.1f", rampAngle);
            telemetry.addData("Ramp Position", "%.3f", rampPosition);

            telemetry.addData("Arm Angle", "%.1f", armAngle);
            telemetry.addData("Arm Position", "%.4f", armPosition);
            telemetry.addData("Activation Angle", "%.1f", activationAngle);

            telemetry.addData("Turret Power", "%.2f", turret.getPower());
            telemetry.addData("ELC Voltage", "%.3f V", elcVoltage);
            telemetry.addData("ELC Angle", "%.2f degrees", elcAngle);
            telemetry.addData("Initial Voltage", initialElcAngle);
            telemetry.addData("Current Voltage", elc.getVoltage());
            telemetry.addData("Raw ELC Angle",
                    (elc.getVoltage() / 3.3) * 360.0);
            telemetry.addData("ELC Difference", elcAngle);
            telemetry.addData("Turret Angle", turretAngle);
            telemetry.addData("Target Turret Angle", targetTurretAngle);
            telemetry.addData("Turret Speed", turretSpeed);
            telemetry.addData("Calculated Distance", calculateDistance(xcoords,ycoords,targetHiveX,targetHiveY));
            telemetry.addData("Calculated RPM", calculatedRpm);
            telemetry.addData("X (inches)",
                    xcoords);

            telemetry.addData("Y (inches)",
                    ycoords);

            telemetry.addData("Heading (degrees)",
                    heading);

            telemetry.update();
        }

        // Stop motors when OpMode ends
        leftFrontDrive.setPower(0.0);
        leftBackDrive.setPower(0.0);
        rightFrontDrive.setPower(0.0);
        rightBackDrive.setPower(0.0);

        intake.setPower(0.0);
        transfer.setPower(0.0);
        flywheel.setPower(0.0);
        turret.setPower(0.0);
    }
    // Calculate the turret's desired angle
    public static double calculateTurretAngle(
            double robotX,
            double robotY,
            double robotHeading,
            double targetX,
            double targetY
    ) {

        double dx = targetX - robotX;
        double dy = targetY - robotY;

        // Calculate field-relative bearing
        double bearing = Math.toDegrees(
                Math.atan2(dx, dy)
        );

        // Convert to robot-relative angle
        double turretAngle = bearing - robotHeading;

        // Keep angle between -180 and +180
        while (turretAngle > 180) {
            turretAngle -= 360;
        }

        while (turretAngle <= -180) {
            turretAngle += 360;
        }

        return turretAngle;
    }


    // Calculate distance from robot to target
    private static double interpolateAngle(double distance) {
        Map.Entry<Double, Double> lower =SHOT_TABLE.floorEntry(distance);
        Map.Entry<Double, Double> upper =SHOT_TABLE.ceilingEntry(distance);
        if (lower == null)
            return SHOT_TABLE.firstEntry().getValue();
        if (upper == null)
            return SHOT_TABLE.lastEntry().getValue();
        if (lower.getKey().equals(upper.getKey()))
            return lower.getValue();
        double d1 = lower.getKey();
        double a1 = lower.getValue();
        double d2 = upper.getKey();
        double a2 = upper.getValue();
        return a1 +(distance - d1)* (a2 - a1)/ (d2 - d1);
    }
    public static double calculateDistance(
            double robotX,
            double robotY,
            double targetX,
            double targetY
    ) {

        double dx = targetX - robotX;
        double dy = targetY - robotY;

        double distance = Math.sqrt(dx * dx + dy * dy);
        return distance;
    }
}

