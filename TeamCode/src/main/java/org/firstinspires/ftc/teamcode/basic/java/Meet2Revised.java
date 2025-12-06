package org.firstinspires.ftc.teamcode.basic.java;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.basic.java.helper.fieldCentric;
import org.firstinspires.ftc.teamcode.basic.java.helper.imuCall;


@TeleOp(name = "Meet 2 Revised", group = "Competition")
public class Meet2Revised extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {

        // --- Initialize drivetrain class
        fieldCentric drive = new fieldCentric(hardwareMap, "fl", "fr", "bl", "br");

        // --- Other motors and servos
        DcMotor intakeLeft  = hardwareMap.get(DcMotor.class, "intakeR");
        DcMotor intakeRight = hardwareMap.get(DcMotor.class, "intakeL");

        DcMotor shooterRight = hardwareMap.get(DcMotor.class, "flyWheelR");
        DcMotor shooterLeft  = hardwareMap.get(DcMotor.class, "flyWheelL");

        //Servo upchargeLeft  = hardwareMap.get(Servo.class, "upLeft");
        //Servo upchargeRight = hardwareMap.get(Servo.class, "upRight");

        shooterLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        // --- Initialize IMU
        IMU imu = imuCall.initIMU(hardwareMap, "imu");

        // --- Variable for re-centering field orientation
        double headingOffset = 0;

        waitForStart();

        while (opModeIsActive()) {
            // Reset field-centric reference
            if (gamepad1.back) {
                headingOffset = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
            }

            // Get adjusted heading
            double rawHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
            double botHeading = rawHeading - headingOffset;

            // Gamepad input (note the clean inversion for strafe)
            double y  = gamepad1.left_stick_y;  // forward is negative on stick
            double x  =  -gamepad1.left_stick_x;  // strafe
            double rx =  -gamepad1.right_stick_x; // rotation (negated for natural turning)

            drive.drive(x, y, rx, botHeading);

            // --- Intake controls
            if (gamepad1.left_bumper) {
                intakeLeft.setPower(1);
                intakeRight.setPower(1);
            } else if (gamepad1.left_trigger > 0.1) {
                intakeLeft.setPower(-1);
                intakeRight.setPower(-1);
            } else {
                intakeLeft.setPower(0);
                intakeRight.setPower(0);
            }

            // --- Shooter controls
            double shooter = gamepad1.right_trigger;
            shooter -= 0.25;
            if (shooter > 0.1) {
                shooterRight.setPower(shooter);
                shooterLeft.setPower(shooter);
            } else {
                shooterRight.setPower(0);
                shooterLeft.setPower(0);
            }

            // --- Upcharge + reverse intake
            if (gamepad1.right_bumper) {
                //upchargeLeft.setPosition(0.25); // up
                //upchargeRight.setPosition(0.0); // up
                intakeLeft.setPower(-0.5);
                intakeRight.setPower(-0.5);
            } else {
                //upchargeLeft.setPosition(0.0);  // down
                //upchargeRight.setPosition(0.25); // down
            }

            telemetry.addData("Heading (°)", Math.toDegrees(botHeading));
            telemetry.addData("Offset (°)", Math.toDegrees(headingOffset));
            telemetry.update();
        }
    }
}
