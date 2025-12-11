package org.firstinspires.ftc.teamcode.Old.java.helper;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Mechanum {
    private final DcMotor fl, fr, bl, br;

    public Mechanum(HardwareMap hw, String flName, String frName, String blName, String brName) {
        fl = hw.get(DcMotor.class, flName);
        fr = hw.get(DcMotor.class, frName);
        bl = hw.get(DcMotor.class, blName);
        br = hw.get(DcMotor.class, brName);

        // Example: reverse the right side if your build needs it
        fr.setDirection(DcMotorSimple.Direction.REVERSE);
        br.setDirection(DcMotorSimple.Direction.REVERSE);
    }


    /** Drive field-centric. headingRad is robot yaw in radians. */
    public void drive(double x, double y, double rx) {
        // rotate joystick vector by -heading

        double denom = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1.0);
        double flPow = (y + x + rx) / denom;
        double blPow = (y - x + rx) / denom;
        double frPow = (y - x - rx) / denom;
        double brPow = (y + x - rx) / denom;

        fl.setPower(flPow);
        bl.setPower(blPow);
        fr.setPower(frPow);
        br.setPower(brPow);
        if (x == 0 && y == 0 && rx == 0) {
            fl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            fr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            bl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            br.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }

    }
}