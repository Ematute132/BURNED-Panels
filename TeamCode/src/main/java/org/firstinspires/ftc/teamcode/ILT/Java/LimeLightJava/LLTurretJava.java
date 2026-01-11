package org.firstinspires.ftc.teamcode.ILT.Java.LimeLightJava;

import dev.nextftc.control.KineticState;
import dev.nextftc.core.commands.utility.InstantCommand;
import dev.nextftc.core.subsystems.Subsystem;
import org.firstinspires.ftc.teamcode.ILT.Java.SubsystemJava.OuttakeJava.ShooterJava.TurretJava;

public class LLTurretJava implements Subsystem {

    // Config
    // This flag controls whether the Limelight is allowed to automatically move the turret.
    // When it's false, the turret logic elsewhere in the code is in charge instead.
    public static boolean autoAimEnabled = false;

    public static double angleToleranceDeg = 1.0;        // |tx| <= this => aligned
    // angleToleranceDeg basically means "if the target is within 1 degree of center, we consider it lined up."

    public static double maxOffsetDeg = 90.0;           // safety clamp on turret offset
    // maxOffsetDeg is a safety limit so the turret never tries to spin more than 90° away from center in either direction.

    // State
    // desiredOffsetRad is the angle (in radians) that we want the turret to turn to, based on tx.
    private static double desiredOffsetRad = 0.0;

    // isAligned is a simple boolean so we can quickly tell if the turret is on target or not.
    private static boolean isAligned = false;

    @Override
    public void initialize() {
        // No initialization needed for this subsystem
    }

    @Override
    public void periodic() {
        // This function runs every loop, and here we decide whether to let Limelight control the turret.

        if (!autoAimEnabled) {
            // If auto-aim is turned off, we do nothing here.
            // Let normal Turret logic run; do not overwrite its power
            return;
        }

        // If auto-aim is on, then we check if the Limelight can actually see the target.
        if (!LimeLightJava.hasValidTarget()) {
            isAligned = false;
            // Option: you can zero power here if you want LL to fully own the turret:
            // TurretJava.turret.setPower(0.0);
            // Since there's no target, we exit without moving the turret.
            return;
        }

        // If we reach here, the Limelight has a valid target and auto-aim is enabled.
        double tx = LimeLightJava.getCurrentTx();  // degrees, LL convention
        // tx is the horizontal offset from the center crosshair — positive is usually to one side and negative to the other.

        // Aligned check purely based on tx
        // Here we say "aligned" if the absolute value of tx is within our tolerance.
        isAligned = Math.abs(tx) <= angleToleranceDeg;

        // Convert tx to radians and use as offset
        // If turret turns the wrong way, flip the sign on desiredOffsetRad.
        double txRad = Math.toRadians(tx);

        // We negate txRad so that a positive tx makes the turret rotate in the direction that reduces error.
        // Then we clamp it to make sure it stays within ±maxOffsetDeg.
        double minOffset = -Math.toRadians(maxOffsetDeg);
        double maxOffset = Math.toRadians(maxOffsetDeg);
        desiredOffsetRad = Math.max(minOffset, Math.min(maxOffset, -txRad));

        // Goal: desired turret angle relative to turret's zero, same frame as TurretJava.getYaw()
        // Here we tell the turret's controller what angle it should be at.
        TurretJava.turretController.setGoal(new KineticState(desiredOffsetRad, 0.0));

        // Measurement: current turret yaw
        // We read the current position of the turret to feed into the controller.
        double currentYaw = TurretJava.getYaw();

        // Controller output to turret motor
        // The controller compares currentYaw to desiredOffsetRad and returns a motor power to correct the error.
        double output = TurretJava.turretController.calculate(new KineticState(currentYaw, 0.0));
        TurretJava.turret.setPower(output);
        // That output gets applied directly to the turret motor so it rotates toward the target.
    }

    // Commands for button binding
    // These commands are designed to be bound to gamepad buttons to control auto-aim.

    public static final InstantCommand enableAutoAim = new InstantCommand(() -> {
        // When this runs, we turn on autoAimEnabled so the periodic loop starts steering the turret with Limelight.
        autoAimEnabled = true;
    });

    public static final InstantCommand disableAutoAim = new InstantCommand(() -> {
        autoAimEnabled = false;
        // Optional: stop turret when disabling
        // TurretJava.turret.setPower(0.0);
        // Here we just turn off auto aim; if we wanted, we could also stop the turret motor.
    });

    public static final InstantCommand toggleAutoAimLL = new InstantCommand(() -> {
        // This acts like a toggle switch: if auto-aim is on, turn it off; if it's off, turn it on.
        if (autoAimEnabled) {
            disableAutoAim.run();
        } else {
            enableAutoAim.run();
        }
    });

    // Getters for private state
    public static double getDesiredOffsetRad() {
        return desiredOffsetRad;
    }

    public static boolean isAligned() {
        return isAligned;
    }

    public static String getTelemetryString() {
        // This builds a text block we can send to telemetry so we can see what the auto-aim system is doing.
        StringBuilder sb = new StringBuilder();
        sb.append("=== LL TURRET AUTO-AIM ===").append("\n");
        sb.append("Enabled: ").append(autoAimEnabled).append("\n");
        sb.append("Valid Target: ").append(LimeLightJava.hasValidTarget()).append("\n");
        sb.append("TX: ").append(String.format("%.2f", LimeLightJava.getCurrentTx())).append("°").append("\n");
        sb.append("Desired Offset: ").append(String.format("%.1f", Math.toDegrees(desiredOffsetRad))).append("°").append("\n");
        sb.append("ALIGNED: ").append(isAligned);
        return sb.toString();
    }
}