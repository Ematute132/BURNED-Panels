package org.firstinspires.ftc.teamcode.ILT.Next.Subsystem.Outtake.Shooter

import com.qualcomm.robotcore.eventloop.opmode.Disabled
import dev.nextftc.control.KineticState
import dev.nextftc.control.builder.controlSystem
import dev.nextftc.control.feedback.PIDCoefficients
import dev.nextftc.core.subsystems.Subsystem
import dev.nextftc.extensions.pedro.PedroComponent.Companion.follower
import dev.nextftc.ftc.ActiveOpMode
import dev.nextftc.hardware.impl.MotorEx
import kotlin.math.PI
import kotlin.math.atan2
import org.firstinspires.ftc.teamcode.ILT.Next.Subsystem.DriveTrain.currentX
import org.firstinspires.ftc.teamcode.ILT.Next.Subsystem.DriveTrain.currentY
import org.firstinspires.ftc.teamcode.ILT.Next.Subsystem.DriveTrain.currentHeading
import org.firstinspires.ftc.teamcode.ILT.Next.Subsystem.DriveTrain.imu
import org.firstinspires.ftc.teamcode.ILT.Next.Subsystem.Outtake.ImprovedOuttake.goalY
import org.firstinspires.ftc.teamcode.ILT.Next.Subsystem.Outtake.ImprovedOuttake.goalX

// gotta absolute encoder
// heading lock

object Turret: Subsystem {


    // Motor that drives the turret. MotorEx wraps the hardware DcMotor with utilities.
    private val turret = MotorEx("turret")

    // Gear ratio between motor and turret output (motor rotations to turret rotations).
    // This is used to convert encoder ticks into actual turret angle.
    private val gearRatio = 3.62068965517
    //105/29

    @JvmField
    var autoTurret = true // Enables automatic aiming behavior when true.

    // PID coefficients for position control of the turret (tuned empirically).
    @JvmField
    var turretPID = PIDCoefficients(0.011, 0.0, 0.2)

    // Control system that uses the position PID to compute motor power based on goal vs current state.
    var turretController = controlSystem {
        posPid(turretPID)
    }

    // Encoder resolution (ticks per revolution) for the motor (goBilda 312 RPM, 537.7 PPR).
    private val ppr = 537.7 // The resolution of our motor encoder on the goBilda site

    // Radians per encoder tick at the turret output.
    // 2π radians per full rotation, divided by ticks per motor rev and the gear ratio.
    private val rpt = 2* PI /(ppr * gearRatio) // The amount of radians per turn of the motor

    // Called every loop; handles auto-aim and telemetry.
    override fun periodic() {
        if(autoTurret) {
            // If automatic aiming is enabled, compute target angle and drive the turret.
            autoAim()
        }

        // Report the current goal and measured yaw for debugging/driver info.
        ActiveOpMode.telemetry.run {
            addData("goal", turretController.goal.position)
            addData("turret Pos", getYaw())
        }
    }

    // Computes the desired turret heading to point at the current goal (goalX, goalY),
    // relative to the robot's current field position (currentX, currentY) and heading.
    private fun autoAim() {
        // Angle from robot position to goal in field coordinates.
        val mu = atan2(goalY - currentY, goalX - currentX)

        // Desired turret offset relative to the robot's current heading.
        val deltaHeading = normalizeAngle(mu - currentHeading)

        // Safety clamp to keep command within [-π, π] before sending to controller.
        val clampedHeading = deltaHeading.coerceIn(-PI, PI)

        // Set the controller's goal to the angle offset; zero desired velocity (position hold).
        turretController.goal = KineticState(clampedHeading, 0.0)

        // Calculate motor power based on current turret yaw vs goal, then apply it.
        turret.power = turretController.calculate(KineticState(getYaw(), 0.0))
    }

    // Set a direct yaw target for the turret controller (in radians).
    fun goToYaw(yaw:Double) { // Go to a specific position
        turretController.goal = KineticState(yaw, 0.0)
    }

    // Convert encoder ticks to a normalized yaw angle in [-π, π].
    fun getYaw(): Double { // Get the current yaw of the turret from [-pi, pi]
        return normalizeAngle(turret.currentPosition * rpt)
    }

    // Normalize any angle (radians) to the principal range [-π, π] to avoid wrap-around issues.
    fun normalizeAngle(angleRadians: Double): Double { // Returns a normalized angle between [-pi, pi]
        var angle = angleRadians % (2.0 * PI)
        if (angle <= -PI) {
            angle += 2.0 * PI
        }
        if (angle > PI) {
            angle -= 2.0 * PI
        }
        return angle
    }
}