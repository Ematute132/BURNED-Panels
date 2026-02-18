package org.firstinspires.ftc.teamcode.nextFtc.Subsystem.Shooter.TurretMech

import dev.nextftc.core.commands.Command
import dev.nextftc.core.units.Angle
import dev.nextftc.core.units.rad

import kotlin.math.atan2

/**
 * Odometry-based turret aim command.
 * Uses robot position and heading to calculate angle to goal.
 */
class OdometryAim(
     val goalX: Double,        // Field X coordinate of goal
     val goalY: Double,        // Field Y coordinate of goal
     val poseX: () -> Double,  // Robot X supplier (live)
     val poseY: () -> Double, // Robot Y supplier (live)
     val poseH: () -> Double, // Robot heading supplier (RADIANS, normalized to [-PI, PI])
     val ofsTurret: Angle = 0.0.rad  // Optional turret offset
) : Command() {

    // Continuous tracking - never completes
    override val isDone = false

    override fun start() {
        Turret.currentState = Turret.State.AIMING
    }

    override fun update() {
        // Get current robot pose
        val x = poseX()
        val y = poseY()
        val h = poseH()

        // Calculate angle from robot to goal
        val deltaX = goalX - x
        val deltaY = goalY - y
        val fieldAngle = atan2(deltaY, deltaX)  // Angle to goal in field space

        // Convert to robot-relative angle
        // turretAngle = where goal is relative to robot heading
        val turretAngle = fieldAngle - h + ofsTurret.inRad

        // Normalize to [-PI, PI] to minimize rotation
        val targetAngle = Turret.normalizeAngle(turretAngle)

        // Apply to turret with velocity compensation
        Turret.setTarget(targetAngle, velocityComp = true)
    }

    override fun stop(interrupted: Boolean) {
        if (!interrupted) {
            Turret.currentState = Turret.State.IDLE
        }
    }
}
