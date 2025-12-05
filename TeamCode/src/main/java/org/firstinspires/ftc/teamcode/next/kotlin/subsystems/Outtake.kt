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
import dev.nextftc.hardware.impl.MotorEx
import kotlin.math.*

@Configurable
object Outtake : Subsystem {  // Added parentheses

    // ==================== HARDWARE ====================
    private val flyR = MotorEx("flyWheelR")
    private val flyL = MotorEx("flyWheelL")
    val fly = MotorGroup(flyL, flyR)

    // ==================== CONTROL SYSTEM ====================
    @JvmField
    var pid = PIDCoefficients(0.0033, 0.0, 0.0)

    @JvmField
    var ff = BasicFeedforwardParameters(1.66667E-4, 0.0, 0.003)

    var controller = controlSystem {
        velPid(pid)
        basicFF(ff)
    }

    // ==================== TARGET VELOCITY ====================
    @JvmField
    var targetVelo = 0.0


    @JvmField
    var velocityTrue = true  // Use the VPID

    // ==================== VOLTAGE COMPENSATION ====================
    @JvmField
    var nominalVoltage = 12.6  // Voltage when you tuned your PID/FF

    @JvmField
    var voltageCompensationEnabled = true

    // ==================== DISTANCE TRACKING ====================
    @JvmField
    var manualDistanceInches = 96.0  // Manual distance override

    @JvmField
    var useManualDistance = true  // Use manual distance vs sensor

    @JvmField
    var distanceAlpha = 0.3  // Low-pass filter coefficient (0 = no filtering, 1 = instant)

    private var filteredDistanceInches = 96.0  // Filtered distance value

    // ==================== PHYSICS CONSTANTS ====================
    private const val GOAL_HEIGHT_IN = 37.85
    private const val G_IN_PER_S2 = 386.0
    private const val LAUNCH_ANGLE_DEG = 34.36
    private const val SHOOTER_DIAMETER_IN = 2.83465
    private const val SHOOTER_RADIUS_IN = SHOOTER_DIAMETER_IN / 2.0
    private const val SHOOTER_HEIGHT_IN = 329.62843 / 25.4  // Convert mm to inches

    // ==================== MOTOR SPECS ====================
    @JvmField
    var motorTicksPerRev = 28.0  // goBILDA 6000 RPM motor

    // ==================== TELEMETRY VALUES ====================
    var compensatedVelo: Double = 0.0
    var velocityError: Double = 0.0
    var controlOutput: Double = 0.0
    var isSpinning: Boolean = false
    var lastCalculatedRpm: Double = 0.0
    var lastCalculatedTicksPerSec: Double = 0.0
    // Outtake.kt (add these lines)
    @JvmField var returnDrivePower: Double = 1.0
    @JvmStatic fun getReturnDrivePower(): Double = returnDrivePower


    // ==================== GETTERS ====================

    val actualVelo: Double
        get() = flyR.state.velocity

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

    // ==================== PHYSICS CALCULATIONS ====================

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

        // Physics: required linear velocity
        val numerator = G_IN_PER_S2 * distanceInInches * distanceInInches
        val denomInner = SHOOTER_HEIGHT_IN + distanceInInches * tanTheta - GOAL_HEIGHT_IN
        val denominator = 2.0 * cosTheta * cosTheta * denomInner

        // If shot is impossible at this distance/angle, return 0
        if (denominator <= 0) return 0.0

        val vInPerSec = sqrt(numerator / denominator)

        // Convert linear speed to wheel RPM
        val rpm = 60.0 * vInPerSec / (2.0 * Math.PI * SHOOTER_RADIUS_IN)
        return rpm
    }

    /**
     * Convert RPM to encoder ticks per second for motor velocity control.
     */
    fun rpmToTicksPerSecond(rpm: Double): Double {
        return (rpm / 60.0) * motorTicksPerRev
    }

    /**
     * Set the target velocity based on distance to target.
     */
    fun setVelocityForDistance(distanceInInches: Double) {
        lastCalculatedRpm = distanceToRequiredRpm(distanceInInches)
        lastCalculatedTicksPerSec = rpmToTicksPerSecond(lastCalculatedRpm)
        targetVelo = lastCalculatedTicksPerSec
        velocityTrue = true
    }

    /**
     * Get current distance to target (either manual or from sensor/vision)
     */
    private fun getCurrentDistance(): Double {
        return if (useManualDistance) {
            manualDistanceInches
        } else {
            // TODO: Integrate with Limelight or distance sensor
            // Example: Limelight.getDistanceToTarget() ?: manualDistanceInches
            manualDistanceInches  // Fallback to manual
        }
    }

    /**
     * Update target velocity based on current distance
     */
    private fun updateTargetFromDistance() {
        val distanceNow = getCurrentDistance()  // Fixed: was calling itself recursively!

        // Low-pass filter to reduce jitter
        filteredDistanceInches = distanceAlpha * distanceNow + (1.0 - distanceAlpha) * filteredDistanceInches

        lastCalculatedRpm = distanceToRequiredRpm(filteredDistanceInches)
        lastCalculatedTicksPerSec = rpmToTicksPerSecond(lastCalculatedRpm)
        targetVelo = lastCalculatedTicksPerSec
    }

    /**
     * Run velocity control loop
     */
    private fun shoot() {  // Renamed from Shoot (lowercase)
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
            /*isSpinning = fly.power != 0.0
            velocityError = 0.0
            compensatedVelo = 0.0
            controlOutput = fly.power
             */
            fly.power = 0.0
        }
    }

    // ==================== COMMANDS ====================

    val flywheelOff: Command
        get() = InstantCommand {
            velocityTrue = false
            targetVelo = 0.0
            fly.power = 0.0
        }
    val flyAuto: Command
        get() = InstantCommand {
           //gotta change
            fly.power = 1.0
        }


    val flywheelBack: Command
        get() = InstantCommand {

            fly.power = -1.0
        }
    val flywheelBackFancy: Command
        get() = InstantCommand {
            velocityTrue = false
            fly.power = -1.0
        }
    val flywheelOn: Command
        get() = InstantCommand {
           fly.power = 0.75


        }

    val flywheelOnFancy: Command
        get() = InstantCommand {
            velocityTrue = true
            updateTargetFromDistance()
        }

    val flywheelBackSlow: Command
        get() = InstantCommand {
            velocityTrue = false
            fly.power = -0.5
        }


    /**
     * Set manual distance and update velocity
     */
    fun setManualDistance(distanceInches: Double): Command {
        return InstantCommand {
            manualDistanceInches = distanceInches
            useManualDistance = true
            updateTargetFromDistance()
        }
    }

    // ==================== PERIODIC ====================

    override fun periodic() {
        // Update PID/FF coefficients from dashboard

        /*
        // Always update target from live distance each cycle
        if (velocityTrue) {
            updateTargetFromDistance()

        }

         */

       shoot()
        // Run the velocity loop

    }

    // ==================== TELEMETRY ====================

    fun getTelemetryString(): String {
        return buildString {
            appendLine("=== FLYWHEEL STATUS ===")
            appendLine("Spinning: $isSpinning")
            appendLine("Velocity Control: $velocityTrue")
            appendLine("Distance: ${"%.1f".format(filteredDistanceInches)} in")
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
            appendLine("Voltage Drop: ${"%.2f".format(voltageDrop)}V")
            appendLine("Compensation: ${"%.3f".format(voltageCompensation)}x")
        }
    }

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
            appendLine("kS: ${ff.kS}")
            appendLine()
            appendLine("=== PHYSICS ===")
            appendLine("Launch Angle: $LAUNCH_ANGLE_DEG°")
            appendLine("Shooter Height: ${"%.2f".format(SHOOTER_HEIGHT_IN)} in")
            appendLine("Goal Height: $GOAL_HEIGHT_IN in")
            appendLine()
            appendLine("=== MOTOR STATES ===")
            appendLine("Left Velo: ${"%.0f".format(flyL.state.velocity)} ticks/sec")
            appendLine("Right Velo: ${"%.0f".format(flyR.state.velocity)} ticks/sec")
            appendLine("Left Power: ${"%.3f".format(flyL.power)}")
            appendLine("Right Power: ${"%.3f".format(flyR.power)}")
        }
    }
}