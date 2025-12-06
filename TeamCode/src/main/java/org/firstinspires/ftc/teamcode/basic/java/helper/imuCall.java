package org.firstinspires.ftc.teamcode.basic.java.helper;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

public class imuCall {

    public static IMU initIMU(HardwareMap hw, String name) {
        IMU imu = hw.get(IMU.class, name);

        IMU.Parameters parameters = new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
                        RevHubOrientationOnRobot.UsbFacingDirection.UP
                )
        );

        imu.initialize(parameters);
        return imu;
    }
}