package org.firstinspires.ftc.teamcode.ILT.Java.SubsystemJava.OuttakeJava.ShooterJava;

import static org.firstinspires.ftc.teamcode.ILT.Java.LimeLightJava.LLTurretJava.toggleAutoAimLL;
import static org.firstinspires.ftc.teamcode.ILT.Java.SubsystemJava.DriveTrainJava.currentHeading;
import static org.firstinspires.ftc.teamcode.ILT.Java.SubsystemJava.DriveTrainJava.currentX;
import static org.firstinspires.ftc.teamcode.ILT.Java.SubsystemJava.DriveTrainJava.currentY;
import static org.firstinspires.ftc.teamcode.ILT.Java.SubsystemJava.OuttakeJava.ImprovedOuttakeJava.goalX;
import static org.firstinspires.ftc.teamcode.ILT.Java.SubsystemJava.OuttakeJava.ImprovedOuttakeJava.goalY;

import com.qualcomm.robotcore.hardware.DcMotor;

import com.bylazar.configurables.annotations.Configurable;

import org.firstinspires.ftc.teamcode.next.kotlin.subsystems.LLTurret;

import dev.nextftc.control.KineticState;
import dev.nextftc.control.builder.ControlSystemBuilderKt;
import dev.nextftc.control.feedback.PIDCoefficients;
import dev.nextftc.core.commands.utility.InstantCommand;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.ftc.ActiveOpMode;
import dev.nextftc.hardware.impl.MotorEx;


@Configurable
public class TurretJava implements Subsystem {

    // Motor that drives the turret. MotorEx wraps the hardware DcMotor with utilities.
    public static final MotorEx turret = new MotorEx("turret");

    // Manual gear power (for driver-controlled turret spinning).
    public static double gP = 0.0;

    // Gear ratio between motor and turret output (motor rotations to turret rotations).
    // This is used to convert encoder ticks into actual turret angle.
    // 105/29 = 3.62068965517
    private static final double gearRatio = 3.62068965517;

 
    public static boolean autoTurret = true; // Enables automatic aiming behavior when true.

    // PID coefficients for position control of the turret (tuned empirically).
    public static PIDCoefficients turretPID = new PIDCoefficients(0.011, 0.0, 0.2);

    // Control system that uses the position PID to compute motor power based on goal vs current state.
    public static final dev.nextftc.control.ControlSystem turretController =
            ControlSystemBuilderKt.controlSystem(builder -> {
                builder.posPid(turretPID);
                return null;
            });

    // Encoder resolution (ticks per revolution) for the motor (goBilda 312 RPM, 537.7 PPR).
    private static final double ppr = 537.7; // The resolution of our motor encoder on the goBilda site

    // Radians per encoder tick at the turret output.
    // 2π radians per full rotation, divided by ticks per motor rev and the gear ratio.
    private static final double rpt = 2 * Math.PI / (ppr * gearRatio); // The amount of radians per turn of the motor

    // Precomputed desired field angle to goal using currentX/currentY (will be recomputed in autoAim anyway).
    public static final double mu2 = Math.atan2(goalY - currentY, goalX - currentX);

    public static final double deltaHeading2 = normalizeAngle(mu2 - currentHeading);

    // Commands for manual turret power control + zeroing encoder.
    public static final InstantCommand zeroMotor = new InstantCommand(() ->
            turret.getMotor().setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER)
    );

    public static final InstantCommand spinGearLeft       = new InstantCommand(() -> gP = 0.6);
    public static final InstantCommand spinGearRight      = new InstantCommand(() -> gP = -0.6);
    public static final InstantCommand gearAlittleLeft    = new InstantCommand(() -> gP = -0.2);
    public static final InstantCommand gearAlittleRight   = new InstantCommand(() -> gP = 0.2);
    public static final InstantCommand stopGear           = new InstantCommand(() -> gP = 0.0);

    @Override
    public void periodic() {
        if (autoTurret) {
            // If automatic aiming is enabled, compute target angle and drive the turret.
            // Manually auto aim based on field coordinates.
            autoAim();
            // Or, to use Limelight-based auto aim instead, call autoAimLL().
        }

        // Report the current goal and measured yaw for debugging/driver info.
        ActiveOpMode.telemetry().addData("goal", turretController.getGoal().getPosition());
        ActiveOpMode.telemetry().addData("turret Pos", getYaw());
        ActiveOpMode.telemetry().update();
    }

    // Placeholder for LL-based auto-aim; calls into Kotlin LLAutoTurret helper.
    public static void autoAimLL() {
        toggleAutoAimLL.run();
    }

    // Computes the desired turret heading to point at the current goal (goalX, goalY),
    // relative to the robot's current field position (currentX, currentY) and heading.
    public static void autoAim() {
        // Angle from robot position to goal in field coordinates.
        double mu = Math.atan2(goalY - currentY, goalX - currentX);

        // Desired turret offset relative to the robot's current heading.
        double deltaHeading = normalizeAngle(mu - currentHeading);

        // Safety clamp to keep command within [-π, π] before sending to controller.
        double clampedHeading = Math.max(-Math.PI, Math.min(Math.PI, deltaHeading));

        // Set the controller's goal to the angle offset; zero desired velocity (position hold).
        turretController.setGoal(new KineticState(clampedHeading, 0.0));

        // Calculate motor power based on current turret yaw vs goal, then apply it.
        double power = turretController.calculate(new KineticState(getYaw(), 0.0));
        turret.setPower(power);
    }

    // Set a direct yaw target for the turret controller (in radians).
    public static void goToYaw(double yaw) { // Go to a specific position
        turretController.setGoal(new KineticState(yaw, 0.0));
    }

    // Convert encoder ticks to a normalized yaw angle in [-π, π].
    public static double getYaw() { // Get the current yaw of the turret from [-pi, pi]
        return normalizeAngle(turret.getCurrentPosition() * rpt);
    }

    // Normalize any angle (radians) to the principal range [-π, π] to avoid wrap-around issues.
    public static double normalizeAngle(double angleRadians) { // Returns a normalized angle between [-pi, pi]
        double angle = angleRadians % (2.0 * Math.PI);
        if (angle <= -Math.PI) {
            angle += 2.0 * Math.PI;
        }
        if (angle > Math.PI) {
            angle -= 2.0 * Math.PI;
        }
        return angle;
    }
}
