package org.firstinspires.ftc.teamcode.pedroPathing;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.control.FilteredPIDFCoefficients;
import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.ftc.localization.constants.PinpointConstants;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Configurable
public class Constants {
    // ===== TUNABLE PID VALUES - Edit these on Panels dashboard! =====
    
    // Translational PID (main)
    @JvmField public static double transP = 0.4;
    @JvmField public static double transI = 0;
    @JvmField public static double transD = 0.05;
    @JvmField public static double transF = 0;
    
    // Translational PID (secondary)
    @JvmField public static double trans2P = 0.1;
    @JvmField public static double trans2I = 0;
    @JvmField public static double trans2D = 0.01;
    @JvmField public static double trans2F = 0;
    
    // Heading PID (main)
    @JvmField public static double headingP = 0.8;
    @JvmField public static double headingI = 0;
    @JvmField public static double headingD = 0.1;
    @JvmField public static double headingF = 0.01;
    
    // Heading PID (secondary)
    @JvmField public static double heading2P = 0.6;
    @JvmField public static double heading2I = 0;
    @JvmField public static double heading2D = 0.02;
    @JvmField public static double heading2F = 0;
    
    // Drive PID (main)
    @JvmField public static double driveP = 0.025;
    @JvmField public static double driveI = 0;
    @JvmField public static double driveD = 0.00001;
    @JvmField public static double driveF = 0.6;
    
    // Drive PID (secondary)
    @JvmField public static double drive2P = 0.02;
    @JvmField public static double drive2I = 0;
    @JvmField public static double drive2D = 0.000005;
    @JvmField public static double drive2F = 0.6;
    
    // Other tunables
    @JvmField public static double mass = 11.0;
    @JvmField public static double forwardZPA = -36.46;
    @JvmField public static double lateralZPA = -52.58;
    
    // Path constraints
    @JvmField public static double maxVelocity = 0.99;
    @JvmField public static double maxAccel = 100;
    @JvmField public static double jerk = 1;
    
    // Drive constants
    @JvmField public static double xVelocity = 59.4;
    @JvmField public static double yVelocity = 48.57;
    @JvmField public static double maxPower = 1.0;
    
    // Localizer constants
    @JvmField public static double forwardPodY = 4.0;
    @JvmField public static double strafePodX = -5.25;
    
    // This method rebuilds the FollowerConstants with current tunable values
    // Call this in your OpMode's loop() to update values live
    public static FollowerConstants getFollowerConstants() {
        return new FollowerConstants()
            .forwardZeroPowerAcceleration(forwardZPA)
            .lateralZeroPowerAcceleration(lateralZPA)
            .useSecondaryTranslationalPIDF(true)
            .translationalPIDFCoefficients(new PIDFCoefficients(transP, transI, transD, transF))
            .secondaryTranslationalPIDFCoefficients(new PIDFCoefficients(trans2P, trans2I, trans2D, trans2F))
            .useSecondaryHeadingPIDF(true)
            .headingPIDFCoefficients(new PIDFCoefficients(headingP, headingI, headingD, headingF))
            .secondaryHeadingPIDFCoefficients(new PIDFCoefficients(heading2P, heading2I, heading2D, heading2F))
            .useSecondaryDrivePIDF(true)
            .drivePIDFCoefficients(new FilteredPIDFCoefficients(driveP, driveI, driveD, driveF, 0.01))
            .secondaryDrivePIDFCoefficients(new FilteredPIDFCoefficients(drive2P, drive2I, drive2D, drive2F, 0.01))
            .mass(mass);
    }
    
    public static PathConstraints getPathConstraints() {
        return new PathConstraints(maxVelocity, maxAccel, jerk, jerk);
    }
    
    public static Follower createFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(getFollowerConstants(), hardwareMap)
                .pathConstraints(getPathConstraints())
                .mecanumDrivetrain(getDriveConstants())
                .pinpointLocalizer(getLocalizerConstants())
                .build();
    }
    
    public static PinpointConstants getLocalizerConstants() {
        return new PinpointConstants()
            .forwardPodY(forwardPodY)
            .strafePodX(strafePodX)
            .distanceUnit(DistanceUnit.INCH)
            .hardwareMapName("odo")
            .encoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD)
            .forwardEncoderDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD)
            .strafeEncoderDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD);
    }
    
    public static MecanumConstants getDriveConstants() {
        return new MecanumConstants()
            .xVelocity(xVelocity)
            .yVelocity(yVelocity)
            .maxPower(maxPower)
            .rightFrontMotorName("fr")
            .rightRearMotorName("br")
            .leftRearMotorName("bl")
            .leftFrontMotorName("fl")
            .leftFrontMotorDirection(DcMotorSimple.Direction.REVERSE)
            .leftRearMotorDirection(DcMotorSimple.Direction.REVERSE)
            .rightFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
            .rightRearMotorDirection(DcMotorSimple.Direction.FORWARD)
            .useBrakeModeInTeleOp(true);
    }
}
