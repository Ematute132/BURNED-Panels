package org.firstinspires.ftc.teamcode.next.kotlin.subsystems

import dev.nextftc.control.KineticState
import dev.nextftc.core.commands.utility.InstantCommand
import dev.nextftc.core.subsystems.Subsystem
import org.firstinspires.ftc.teamcode.ILT.Next.Subsystem.Outtake.Shooter.Turret
import org.firstinspires.ftc.teamcode.ILT.Next.Subsystem.limeLight.limeLight
import kotlin.math.abs


object LLTurret : Subsystem {

    // Config
    // This flag controls whether the Limelight is allowed to automatically move the turret.
    // When it's false, the turret logic elsewhere in the code is in charge instead.
    var autoAimEnabled = false

    var angleToleranceDeg = 1.0        // |tx| <= this => aligned
    // angleToleranceDeg basically means “if the target is within 1 degree of center, we consider it lined up.”

    var maxOffsetDeg = 90.0           // safety clamp on turret offset
    // maxOffsetDeg is a safety limit so the turret never tries to spin more than 90° away from center in either direction.

    // State
    // desiredOffsetRad is the angle (in radians) that we want the turret to turn to, based on tx.
    var desiredOffsetRad = 0.0

    // isAligned is a simple boolean so we can quickly tell if the turret is on target or not.
    var isAligned = false

    override fun periodic() {
        // This function runs every loop, and here we decide whether to let Limelight control the turret.

        if (!autoAimEnabled) {
            // If auto-aim is turned off, we do nothing here.
            // Let normal Turret logic run; do not overwrite its power
            return
        }

        // If auto-aim is on, then we check if the Limelight can actually see the target.
        if (!limeLight.hasValidTarget) {
            isAligned = false
            // Option: you can zero power here if you want LL to fully own the turret:
            // Turret.turret.power = 0.0
            // Since there’s no target, we exit without moving the turret.
            return
        }

        // If we reach here, the Limelight has a valid target and auto-aim is enabled.
        val tx = limeLight.currentTx  // degrees, LL convention
        // tx is the horizontal offset from the center crosshair — positive is usually to one side and negative to the other.

        // Aligned check purely based on tx
        // Here we say “aligned” if the absolute value of tx is within our tolerance.
        isAligned = abs(tx) <= angleToleranceDeg

        // Convert tx to radians and use as offset
        // If turret turns the wrong way, flip the sign on desiredOffsetRad.
        val txRad = Math.toRadians(tx)

        // We negate txRad so that a positive tx makes the turret rotate in the direction that reduces error.
        // Then we clamp it to make sure it stays within ±maxOffsetDeg.
        desiredOffsetRad = (-txRad).coerceIn(
            -Math.toRadians(maxOffsetDeg),
            Math.toRadians(maxOffsetDeg)
        )

        // Goal: desired turret angle relative to turret's zero, same frame as Turret.getYaw()
        // Here we tell the turret’s controller what angle it should be at.
        Turret.turretController.goal = KineticState(desiredOffsetRad, 0.0)

        // Measurement: current turret yaw
        // We read the current position of the turret to feed into the controller.
        val currentYaw = Turret.getYaw()

        // Controller output to turret motor
        // The controller compares currentYaw to desiredOffsetRad and returns a motor power to correct the error.
        val output = Turret.turretController.calculate(KineticState(currentYaw, 0.0))
        Turret.turret.power = output
        // That output gets applied directly to the turret motor so it rotates toward the target.
    }

    // Commands for button binding
    // These commands are designed to be bound to gamepad buttons to control auto-aim.

    val enableAutoAim = InstantCommand {
        // When this runs, we turn on autoAimEnabled so the periodic loop starts steering the turret with Limelight.
        autoAimEnabled = true
    }

    val disableAutoAim = InstantCommand {
        autoAimEnabled = false
        // Optional: stop turret when disabling
        // Turret.turret.power = 0.0
        // Here we just turn off auto aim; if we wanted, we could also stop the turret motor.
    }

    val toggleAutoAimLL = InstantCommand {
        // This acts like a toggle switch: if auto-aim is on, turn it off; if it’s off, turn it on.
        if (autoAimEnabled) {
            disableAutoAim.run()
        } else {
            enableAutoAim.run()
        }
    }

    fun getTelemetryString(): String {
        // This builds a text block we can send to telemetry so we can see what the auto-aim system is doing.
        return buildString {
            appendLine("=== LL TURRET AUTO-AIM ===")
            appendLine("Enabled: $autoAimEnabled")
            appendLine("Valid Target: ${limeLight.hasValidTarget}")
            appendLine("TX: ${"%.2f".format(limeLight.currentTx)}°")
            appendLine("Desired Offset: ${"%.1f".format(Math.toDegrees(desiredOffsetRad))}°")
            appendLine("ALIGNED: $isAligned")
        }
    }
}
