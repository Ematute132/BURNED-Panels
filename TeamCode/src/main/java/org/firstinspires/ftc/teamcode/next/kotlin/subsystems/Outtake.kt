package org.firstinspires.ftc.teamcode.next.kotlin.subsystems

import com.bylazar.configurables.annotations.Configurable
import dev.nextftc.control.KineticState
import dev.nextftc.control.builder.controlSystem
import dev.nextftc.control.feedback.PIDCoefficients
import dev.nextftc.control.feedforward.BasicFeedforwardParameters
import dev.nextftc.core.commands.Command
import dev.nextftc.core.commands.utility.InstantCommand
import dev.nextftc.core.subsystems.Subsystem
import dev.nextftc.ftc.ActiveOpMode
import dev.nextftc.hardware.controllable.MotorGroup
import dev.nextftc.hardware.delegates.Velocity
import dev.nextftc.hardware.impl.MotorEx
import org.firstinspires.ftc.teamcode.next.kotlin.subsystems.Limelight.limeLight
import org.firstinspires.ftc.teamcode.next.kotlin.subsystems.Limelight.limeLight.distanceToGoal
import kotlin.math.*

@Configurable
object Outtake : Subsystem {

    // ==================== HARDWARE ====================
    private val flyR = MotorEx("flyWheelR")
    private val flyL = MotorEx("flyWheelL")
    val fly = MotorGroup(flyL, flyR)


    var targetVelo = 0.0
    var targetOnVelo = 500.0
    var fP = 0.0
    var autoAdjustVelocity = false


    @JvmField
    var velocityTrue = true  // Use the VPID


    const val LAUNCH_ANGLE_DEG = 34.36
    const val SHOOTER_HEIGHT_IN = 12.9774972441
    const val GOAL_HEIGHT_IN = 37.85
    const val GRAVITY_IN_PER_S2 = 386.0  // Gravity in inches per second squared (not meters!)
    const val SHOOTER_DIAMETER_IN = 2.83465
    const val SHOOTER_RADIUS_IN = SHOOTER_DIAMETER_IN / 2.0

    val HEIGHT_DIFF_IN = GOAL_HEIGHT_IN - SHOOTER_HEIGHT_IN



    //find velo needed for arc

    fun calculateMotorVelocity(distanceToTarget: Double, motorTicksPerRev: Double = 28.0): Double {
        // Convert angle to radians
        val theta = Math.toRadians(LAUNCH_ANGLE_DEG)
        val cosTheta = cos(theta)
        val tanTheta = tan(theta)

        // Physics formula: v = sqrt(g * d^2 / (2 * cos^2(θ) * (d * tan(θ) - h)))
        val numerator = GRAVITY_IN_PER_S2 * distanceToTarget * distanceToTarget
        val denominator = 2.0 * cosTheta * cosTheta * (distanceToTarget * tanTheta - HEIGHT_DIFF_IN)

        // Guard against impossible geometry
        if (denominator <= 0) return 0.0

        // Calculate linear velocity in inches per second
        val velocityInPerSec = sqrt(numerator / denominator)

        // Convert linear velocity to RPM: v = ω * r → RPM = (v / r) * (60 / 2π)
        val rpm = 60.0 * velocityInPerSec / (2.0 * PI * SHOOTER_RADIUS_IN)

        // Convert RPM to ticks per second
        val ticksPerSec = (rpm / 60.0) * motorTicksPerRev

        return ticksPerSec
    }

    val targetVeloLL = calculateMotorVelocity(distanceToGoal)

    // ==================== GETTERS ====================

    val flywheelOff: Command
        get() = InstantCommand {
            //velocityTrue = false
            targetVelo = 0.0
            fP = 0.0
        }
    val flyAuto: Command
        get() = InstantCommand {
           //gotta change
            fly.power = 1.0
        }


    val flywheelBack: Command
        get() = InstantCommand {

            fP = -1.0
        }

    val flywheelOn: Command
        get() = InstantCommand {
           fP = 0.75


        }

    val flywheelOnFancy: Command
        get() = InstantCommand {
            velocityTrue = true

        }

    val flywheelBackSlow: Command
        get() = InstantCommand {
            velocityTrue = false
            fP = -0.5
        }



    // ==================== PERIODIC ====================
// fix this so that it works
    override fun periodic() {
        if (autoAdjustVelocity) {
            val distance = limeLight.getDistanceToTarget()
            if (distance != null && distance > 0) {
                Outtake.calculateMotorVelocity(distanceToGoal)
            } else {
                // Fallback to default velocity if no target
                Outtake.targetVelo = Outtake.targetOnVelo
            }
        }
    }




}