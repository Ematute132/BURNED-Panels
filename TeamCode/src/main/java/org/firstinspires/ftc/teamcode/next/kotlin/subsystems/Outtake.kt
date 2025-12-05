package org.firstinspires.ftc.teamcode.next.kotlin.subsystems

import com.bylazar.configurables.annotations.Configurable
import dev.nextftc.control.KineticState
import dev.nextftc.control.builder.controlSystem
import dev.nextftc.control.feedback.PIDCoefficients
import dev.nextftc.control.feedforward.BasicFeedforwardParameters
import dev.nextftc.core.commands.utility.InstantCommand
import dev.nextftc.core.subsystems.Subsystem
import dev.nextftc.ftc.ActiveOpMode
import dev.nextftc.hardware.controllable.MotorGroup
import dev.nextftc.hardware.impl.MotorEx
import kotlin.math.*

@Configurable
object Outtake : Subsystem {
    val flyR = MotorEx("flyWheelR")
    val flyL = MotorEx("flyWheelL").reversed()
    val fly = MotorGroup(flyL, flyR)

    var pid = PIDCoefficients(0.0033, 0.0, 0.0)
    var ff = BasicFeedforwardParameters(1.66667E-4, 0.0, 0.003)
    var controller = controlSystem {
        velPid(pid)
        basicFF(ff)
    }

    var targetVelo = 0.0
    var targetOnVelo = 500.0
    var velocityTrue = true // Use the VPID

    // Voltage compensation
    private val nominalVoltage = 12.6  // Voltage when you tuned your PID/FF (fully charged battery)
    var voltageCompensationEnabled = true  // Toggle for testing

    // Physics constants for distance-to-RPM calculation
    private const val GOAL_HEIGHT_IN = 37.85
    private const val G_IN_PER_S2 = 386.0
    private const val LAUNCH_ANGLE_DEG = 34.36
    private const val SHOOTER_DIAMETER_IN = 2.83465
    private const val SHOOTER_RADIUS_IN = SHOOTER_DIAMETER_IN / 2.0
    // shooterHeightInches = 329.62843 / 25.4 ≈ 12.98
    private const val SHOOTER_HEIGHT_IN = 329.62843 / 25.4

    // Motor specs - goBILDA 6000 RPM (1:1 ratio, 5202/5203 series)
    // Encoder resolution: 28 countable events per revolution
    var motorTicksPerRev = 28.0

    // Exposed telemetry values
    var compensatedVelo: Double = 0.0
    var velocityError: Double = 0.0
    var controlOutput: Double = 0.0
    var isSpinning: Boolean = false
    var lastCalculatedRpm: Double = 0.0
    var lastCalculatedTicksPerSec: Double = 0.0

    val actualVelo: Double
        get() = flyR.state.velocity  // Use one motor for reading

    val currentVoltage: Double
        get() = try {
            ActiveOpMode.hardwareMap.voltageSensor.iterator().next().voltage
        } catch (e: Exception) {
            12.0  // Default fallback
        }

    val voltageCompensation: Double
        get() = if (voltageCompensationEnabled) nominalVoltage / currentVoltage else 1.0

    val voltageDrop: Double
        get() = nominalVoltage - currentVoltage

    /**
     * Calculate the required RPM for the shooter to reach a target at a given distance.
     * Uses projectile motion physics with a fixed launch angle and shooter height.
     *
     * @param distanceInInches Horizontal distance to target in inches
     * @return Required RPM for the flywheel, or 0.0 if geometry is impossible
     */
    fun distanceToRequiredRpm(distanceInInches: Double): Double {
        val theta = Math.toRadians(LAUNCH_ANGLE_DEG)
        val tanTheta = tan(theta)
        val cosTheta = cos(theta)

        // physics: required linear velocity
        val numerator = G_IN_PER_S2 * distanceInInches * distanceInInches
        val denomInner = SHOOTER_HEIGHT_IN + distanceInInches * tanTheta - GOAL_HEIGHT_IN
        val denominator = 2.0 * cosTheta * cosTheta * denomInner

        // if shot is impossible at this distance/angle, return 0
        if (denominator <= 0) return 0.0

        val vInPerSec = sqrt(numerator / denominator)

        // convert linear speed to wheel RPM
        val rpm = 60.0 * vInPerSec / (2.0 * Math.PI * SHOOTER_RADIUS_IN)
        return rpm
    }

    /**
     * Convert RPM to encoder ticks per second for motor velocity control.
     *
     * @param rpm Rotations per minute
     * @return Velocity in ticks per second
     */
    fun rpmToTicksPerSecond(rpm: Double): Double {
        // RPM → revolutions per second → ticks per second
        return (rpm / 60.0) * motorTicksPerRev
    }

    /**
     * Set the target velocity based on distance to target.
     * This calculates the required RPM and converts it to motor velocity (ticks/sec).
     *
     * @param distanceInInches Horizontal distance to target in inches
     */
    fun setVelocityForDistance(distanceInInches: Double) {
        lastCalculatedRpm = distanceToRequiredRpm(distanceInInches)
        lastCalculatedTicksPerSec = rpmToTicksPerSecond(lastCalculatedRpm)
        targetVelo = lastCalculatedTicksPerSec
        velocityTrue = true
    }

    fun Shoot() {
        if (velocityTrue) {
            // Apply voltage compensation to target velocity
            compensatedVelo = targetVelo * voltageCompensation

            // Set goal with compensated velocity
            controller.goal = KineticState(0.0, compensatedVelo)

            // Calculate control output for left motor
            controlOutput = controller.calculate(flyL.state)

            // Apply to both motors
            flyL.power = controlOutput
            flyR.power = controlOutput

            // Update telemetry values
            velocityError = targetVelo - actualVelo
            isSpinning = targetVelo > 0
        } else {
            // Direct power control (no velocity control)
            isSpinning = fly.power != 0.0
            velocityError = 0.0
            compensatedVelo = 0.0
            controlOutput = fly.power
        }
    }

    val flywheelOff: InstantCommand =
        InstantCommand {
            velocityTrue = false
            targetVelo = 0.0
            fly.power = 0.0
        }

    val flywheelBack: InstantCommand =
        InstantCommand {
            velocityTrue = false
            fly.power = -1.0
        }

    val flywheelOn: InstantCommand =
        InstantCommand {
            velocityTrue = true
            targetVelo = targetOnVelo
        }

    val flywheelBackSlow: InstantCommand =
        InstantCommand {
            velocityTrue = false
            fly.power = -0.5
        }

    override fun periodic() {
        Shoot()
    }

    // Get all telemetry as a formatted string
    fun getTelemetryString(): String {
        return buildString {
            appendLine("=== FLYWHEEL STATUS ===")
            appendLine("Spinning: $isSpinning")
            appendLine("Velocity Control: $velocityTrue")
            if (velocityTrue) {
                appendLine("Target Velo: ${"%.0f".format(targetVelo)} ticks/sec")
                appendLine("Target RPM: ${"%.0f".format(lastCalculatedRpm)} RPM")
                appendLine("Actual Velo: ${"%.0f".format(actualVelo)} ticks/sec")
                appendLine("Compensated Velo: ${"%.0f".format(compensatedVelo)} ticks/sec")
                appendLine("Velocity Error: ${"%.0f".format(velocityError)} ticks/sec")
                appendLine("Control Output: ${"%.3f".format(controlOutput)}")
            } else {
                appendLine("Power: ${"%.2f".format(fly.power)}")
            }
            appendLine()
            appendLine("=== VOLTAGE COMPENSATION ===")
            appendLine("Current Voltage: ${"%.2f".format(currentVoltage)}V")
            appendLine("Nominal Voltage: ${"%.2f".format(nominalVoltage)}V")
            appendLine("Voltage Drop: ${"%.2f".format(voltageDrop)}V")
            appendLine("Compensation: ${"%.3f".format(voltageCompensation)}x")
            appendLine("Compensation Enabled: $voltageCompensationEnabled")
        }
    }

    // Debug info for tuning
    fun getDebugInfo(): String {
        return buildString {
            appendLine("=== PID COEFFICIENTS ===")
            appendLine("Kp: ${pid.kP}")
            appendLine("Ki: ${pid.kI}")
            appendLine("Kd: ${pid.kD}")
            appendLine()
            appendLine("=== FEEDFORWARD ===")
            appendLine("kV: ${ff.kV}")
            appendLine("kA: ${ff.kA}")
            appendLine()
            appendLine("=== MOTOR CONFIGURATION ===")
            appendLine("Motor TPR: $motorTicksPerRev")
            appendLine("Shooter Diameter: $SHOOTER_DIAMETER_IN in")
            appendLine("Shooter Height: ${"%.2f".format(SHOOTER_HEIGHT_IN)} in")
            appendLine("Launch Angle: $LAUNCH_ANGLE_DEG°")
            appendLine()
            appendLine("=== MOTOR STATES ===")
            appendLine("Left Motor Velo: ${"%.0f".format(flyL.state.velocity)}")
            appendLine("Right Motor Velo: ${"%.0f".format(flyR.state.velocity)}")
            appendLine("Left Motor Power: ${"%.3f".format(flyL.power)}")
            appendLine("Right Motor Power: ${"%.3f".format(flyR.power)}")
        }
    }
}