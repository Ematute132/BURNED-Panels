package org.firstinspires.ftc.teamcode.Old.java.helper;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class fieldCentric {
    private final DcMotor fl, fr, bl, br;
    private static final double DEADZONE = 0.1; // Add deadzone here too

    public fieldCentric(HardwareMap hw, String flName, String frName, String blName, String brName) {
        fl = hw.get(DcMotor.class, flName);
        fr = hw.get(DcMotor.class, frName);
        bl = hw.get(DcMotor.class, blName);
        br = hw.get(DcMotor.class, brName);

        // Set directions
        fr.setDirection(DcMotorSimple.Direction.REVERSE);
        br.setDirection(DcMotorSimple.Direction.REVERSE);
        fl.setDirection(DcMotorSimple.Direction.FORWARD);
        bl.setDirection(DcMotorSimple.Direction.FORWARD);

        // SET BRAKE MODE ONCE HERE

    }

    /** Drive field-centric. headingRad is robot yaw in radians. */
    public void drive(double x, double y, double rx, double botHeading) {
        // Rotate joystick vector by -heading
        double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
        double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);

        rotX *= 1.1; // Compensate for imperfect strafing

        // APPLY DEADZONE TO ROTATED VALUES (this is key!)
        if (Math.abs(rotX) < DEADZONE) rotX = 0;
        if (Math.abs(rotY) < DEADZONE) rotY = 0;
        if (Math.abs(rx) < DEADZONE) rx = 0;

        // If all inputs are zero, stop motors completely
        if (rotX == 0 && rotY == 0 && rx == 0) {
            fl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            fr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            bl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            br.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }

        double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
        double flPow = (rotY + rotX + rx) / denominator;
        double blPow = (rotY - rotX + rx) / denominator;
        double frPow = (rotY - rotX - rx) / denominator;
        double brPow = (rotY + rotX - rx) / denominator;

        fl.setPower(flPow);
        bl.setPower(blPow);
        fr.setPower(frPow);
        br.setPower(brPow);
    }
}