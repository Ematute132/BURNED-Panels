@file:Suppress("PackageName", "unused", "SameParameterValue")

package org.firstinspires.ftc.teamcode.Systems

import com.pedropathing.geometry.Pose
import dev.nextftc.core.subsystems.SubsystemGroup

import org.firstinspires.ftc.teamcode.Systems.ShooterSubsystems.Flywheel
import org.firstinspires.ftc.teamcode.Systems.ShooterSubsystems.FlywheelState

import org.firstinspires.ftc.teamcode.Util.ROBOT
import org.firstinspires.ftc.teamcode.nextFtc.Subsystem.Shooter.TurretMech.Turret

object Shooter : SubsystemGroup(Turret, Flywheel) {

    internal var flywheelState: FlywheelState = FlywheelState.AUTO_AIM

    fun update() {
        updateTurret()

        when (flywheelState) {
            FlywheelState.AUTO_AIM -> {
                // FIX: set usePID before updateFlywheel() so the first call uses the right mode
                Flywheel.usePID = true
                updateFlywheel()
            }
            FlywheelState.MANUAL -> {
                // flywheelTarget was already set by setFlywheelManualVelocity().
                // PID remains on so the controller tracks the manually set target.
                Flywheel.usePID = true
            }
            FlywheelState.IDLE -> {
                Flywheel.usePID = true
                Flywheel.flywheelTarget = Flywheel.IDLE_VELOCITY
            }
            FlywheelState.STOPPED -> {
                Flywheel.usePID = false
                Flywheel.flywheelTarget = 0.0
            }
        }

        // Voltage compensation only makes sense when actively targeting; skip when stopped/idle
        val voltageComp = (flywheelState == FlywheelState.AUTO_AIM ||
                flywheelState == FlywheelState.MANUAL)
        Flywheel.update(voltageComp)
    }

    /**
     * Returns the raw open-loop flywheel velocity for a given robot pose.
     * Used for UI preview or manual overrides; updateFlywheel() uses the corrected version.
     */
    fun getVelocity(pose: Pose): Double {
        val distance = pose.distanceFrom(ROBOT.currAlliance.goalPoses.flywheelGoalPose)
        return velocityForDistance(distance)
    }

    fun setFlywheelAutoAim() {
        flywheelState = FlywheelState.AUTO_AIM
    }

    fun setFlywheelManualVelocity(velocity: Double) {
        Flywheel.flywheelTarget = velocity
        flywheelState = FlywheelState.MANUAL
    }

    fun setFlywheelIdle() {
        flywheelState = FlywheelState.IDLE
    }

    fun setFlywheelStopped() {
        flywheelState = FlywheelState.STOPPED
    }

    // ============================================
    // PRIVATE HELPERS
    // ============================================

    private fun velocityForDistance(distance: Double): Double {
        val base = 0.0142645 * distance * distance + 1.26161 * distance + 748.88095
        return if (distance >= 120.0) base + 40.0 else base
    }

    private fun updateFlywheel() {
        // Lead time correction: estimate where the robot will be when the disc arrives
        val d = ROBOT.shooterPose().distanceFrom(ROBOT.currAlliance.goalPoses.flywheelGoalPose)
        val t = -0.0000464477 * d * d + 0.0151342 * d - 0.348423
        val correctedDistance = ROBOT.correctedPose(t, t)
            .distanceFrom(ROBOT.currAlliance.goalPoses.flywheelGoalPose)
        Flywheel.flywheelTarget = velocityForDistance(correctedDistance)
    }

    private fun updateTurret() {
        val targetPose = when {
            ROBOT.inCloseZone() -> ROBOT.currAlliance.goalPoses.turretGoalPoseClose
            ROBOT.inFarZone()   -> ROBOT.currAlliance.goalPoses.turretGoalPoseFar
            else                -> ROBOT.currAlliance.goalPoses.flywheelGoalPose
        }
        Turret.aimAt(targetPose, ROBOT.shooterPose())
    }
}