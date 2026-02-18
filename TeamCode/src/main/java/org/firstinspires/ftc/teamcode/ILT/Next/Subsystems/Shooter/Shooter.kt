@file:Suppress("PackageName", "unused", "SameParameterValue")

package org.firstinspires.ftc.teamcode.Systems

import com.pedropathing.geometry.Pose
import dev.nextftc.core.subsystems.SubsystemGroup
import dev.nextftc.hardware.impl.ServoEx

import org.firstinspires.ftc.teamcode.Systems.ShooterSubsystems.Flywheel
import org.firstinspires.ftc.teamcode.Systems.ShooterSubsystems.FlywheelState

import org.firstinspires.ftc.teamcode.Util.Alliance
import org.firstinspires.ftc.teamcode.Util.ROBOT
import org.firstinspires.ftc.teamcode.Util.Stage
import org.firstinspires.ftc.teamcode.nextFtc.Subsystem.Shooter.TurretMech.Turret
import kotlin.math.atan2

object Shooter: SubsystemGroup(Turret, Flywheel) {
    //   private val frontRGBLight: ServoEx = ServoEx("light")
    // private val backRGBLight: ServoEx = ServoEx("Light2")

    private const val VEL_SCALAR: Double = 0.0
    private const val ANG_SCALAR: Double = 0.0
    internal var flywheelState: FlywheelState = FlywheelState.AUTO_AIM

    fun update() {
        updateTurret()
        when (flywheelState) {
            FlywheelState.AUTO_AIM -> {
                Flywheel.usePID = true.also { updateFlywheel() }
            }
            //  .also { backRGBLight.position = 0.47 } }
            FlywheelState.MANUAL -> {
                // Do NOT call updateFlywheel() here.
                // Flywheel.flywheelTarget was already set by setFlywheelManualVelocity()
                //    backRGBLight.position = 0.722
            }

            FlywheelState.IDLE -> {
                Flywheel.flywheelTarget = Flywheel.IDLE_VELOCITY
            }
            //.also { backRGBLight.position = 0.28 } }
            FlywheelState.STOPPED -> {
                Flywheel.usePID = false.also { Flywheel.flywheelTarget = 0.0 }
            }
            //.also{backRGBLight.position = 0.28} }
        }

        //Turret.update()
        val voltageComp = (flywheelState == FlywheelState.AUTO_AIM)
        Flywheel.update(voltageComp)
        //if (Flywheel.isAtTarget()) { frontRGBLight.position = 0.47 }
        // else { frontRGBLight.position = 0.28 }
    }

    fun getVelocity(pose: Pose): Double {
        val distance = pose.distanceFrom(ROBOT.currAlliance.goalPoses.flywheelGoalPose)
        val flywheelVelocity = if (distance < 120.0) {
            0.0142645 * distance * distance + 1.26161 * distance + 748.88095
        } else {
            0.0142645 * distance * distance + 1.26161 * distance + 748.88095 + 40.0
        }
        return flywheelVelocity
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


    private fun updateFlywheel() {
        val d = ROBOT.shooterPose().distanceFrom(ROBOT.currAlliance.goalPoses.flywheelGoalPose)
        val t = -0.0000464477 * d * d + 0.0151342 * d - 0.348423
        val D =
            ROBOT.correctedPose(t, t).distanceFrom(ROBOT.currAlliance.goalPoses.flywheelGoalPose)
        val base = 0.0142645 * D * D + 1.26161 * D + 748.88095
        Flywheel.flywheelTarget = if (ROBOT.inCloseZone()) base else base + 40.0
    }

    private fun updateTurret() {
        if (!Turret.hasRelocalized) return
        val targetPose = when {
            ROBOT.inCloseZone() -> ROBOT.currAlliance.goalPoses.turretGoalPoseClose
            ROBOT.inFarZone()   -> ROBOT.currAlliance.goalPoses.turretGoalPoseFar
            else                -> ROBOT.currAlliance.goalPoses.flywheelGoalPose
        }
        Turret.aimAt(targetPose, ROBOT.shooterPose())
    }
}