package org.firstinspires.ftc.teamcode.next.kotlin.subsystems

import com.qualcomm.hardware.limelightvision.Limelight3A
import dev.nextftc.core.subsystems.Subsystem
import dev.nextftc.ftc.ActiveOpMode
import kotlin.math.*

object LLAutoVelo : Subsystem {

    lateinit var ll: Limelight3A

    // Limelight configuration
    var llAngle = 9.895942 // Limelight mounting angle in degrees
    var llLensHeight = 10.2756  // Height of limelight lens in inches
    var goalHeight = 29.5  // Height of target in inches

    // Shooter physics constants
    private const val LAUNCH_ANGLE_DEG = 34.36
    private const val SHOOTER_HEIGHT_IN = 12.9774972441
    private const val GOAL_HEIGHT_IN = 37.85
    private const val GRAVITY_IN_PER_S2 = 386.0  // Gravity in inches per second squared
    private const val SHOOTER_DIAMETER_IN = 2.83465
    private const val SHOOTER_RADIUS_IN = SHOOTER_DIAMETER_IN / 2.0

    // Motor specs
    var motorTicksPerRev = 28.0  // goBILDA 6000 RPM motors

    // Distance calculation properties
    var targetDistance = 24.0
    var distanceTolerance = 3.0
    var angleToGoalDegrees: Double = 0.0
    var angleToGoalRadians: Double = 0.0
    var distanceToGoal: Double? = null
    var isAtTargetDistance: Boolean = false
    var currentTy: Double = 0.0
    var hasValidTarget: Boolean = false

    // Calculated velocity
    var calculatedVelocity: Double = 0.0
    var calculatedRPM: Double = 0.0

    override fun initialize() {
        ll = ActiveOpMode.hardwareMap.get(Limelight3A::class.java, "ll")
        ll.setPollRateHz(100)
        ll.pipelineSwitch(0)
        ll.start()
    }

    override fun periodic() {
        updateDistanceCalculation()
        updateVelocityCalculation()
    }

    private fun updateDistanceCalculation() {
        val result = ll.latestResult
        if (result != null && result.isValid) {
            hasValidTarget = true
            val targetOffsetAngleVertical = result.ty
            currentTy = targetOffsetAngleVertical

            // Calculate angles
            angleToGoalDegrees = llAngle + targetOffsetAngleVertical
            angleToGoalRadians = angleToGoalDegrees * (PI / 180.0)

            // Calculate distance
            distanceToGoal = (goalHeight - llLensHeight) / tan(angleToGoalRadians)

            // Check if at target distance
            distanceToGoal?.let { dist ->
                isAtTargetDistance = abs(dist - targetDistance) <= distanceTolerance
            } ?: run {
                isAtTargetDistance = false
            }
        } else {
            hasValidTarget = false
            distanceToGoal = null
            angleToGoalDegrees = 0.0
            angleToGoalRadians = 0.0
            isAtTargetDistance = false
            currentTy = 0.0
        }
    }

    private fun updateVelocityCalculation() {
        val distance = distanceToGoal
        if (distance != null && distance > 0) {
            calculatedVelocity = calculateMotorVelocity(distance)
            calculatedRPM = calculateRPM(distance)
        } else {
            calculatedVelocity = 0.0
            calculatedRPM = 0.0
        }
    }


    fun calculateRPM(distanceToTarget: Double): Double {
        // Convert angle to radians
        val theta = Math.toRadians(LAUNCH_ANGLE_DEG)
        val cosTheta = cos(theta)
        val tanTheta = tan(theta)

        val heightDiff = GOAL_HEIGHT_IN - SHOOTER_HEIGHT_IN

        // Physics formula: v = sqrt(g * d^2 / (2 * cos^2(θ) * (d * tan(θ) - h)))
        val numerator = GRAVITY_IN_PER_S2 * distanceToTarget * distanceToTarget
        val denominator = 2.0 * cosTheta * cosTheta * (distanceToTarget * tanTheta - heightDiff)

        // Guard against impossible geometry
        if (denominator <= 0) return 0.0

        // Calculate linear velocity in inches per second
        val velocityInPerSec = sqrt(numerator / denominator)

        // Convert linear velocity to RPM: v = ω * r → RPM = (v / r) * (60 / 2π)
        val rpm = 60.0 * velocityInPerSec / (2.0 * PI * SHOOTER_RADIUS_IN)

        return rpm
    }

    fun calculateMotorVelocity(distanceToTarget: Double): Double {
        val rpm = calculateRPM(distanceToTarget)
        if (rpm == 0.0) return 0.0

        // Convert RPM to ticks per second
        val ticksPerSec = (rpm / 60.0) * motorTicksPerRev

        return ticksPerSec
    }

    fun getDistanceToTarget(): Double? {
        return distanceToGoal
    }

    fun getCalculatedVelocity(): Double {
        return calculatedVelocity
    }

    fun getCalculatedRPM(): Double {
        return calculatedRPM
    }

    fun getDistanceDebugInfo(): String {
        return buildString {
            appendLine("=== DISTANCE DEBUG ===")
            appendLine("Valid Target: $hasValidTarget")
            appendLine("llAngle: $llAngle°")
            appendLine("TY: ${"%.2f".format(currentTy)}°")
            appendLine("Angle to Goal: ${"%.2f".format(angleToGoalDegrees)}°")
            appendLine("Angle (radians): ${"%.4f".format(angleToGoalRadians)}")
            appendLine("Goal Height: $goalHeight\"")
            appendLine("Lens Height: $llLensHeight\"")
            appendLine("Height Diff: ${goalHeight - llLensHeight}\"")
            appendLine("tan(angle): ${"%.4f".format(Math.tan(angleToGoalRadians))}")
            appendLine("Distance: ${distanceToGoal?.let { "%.2f".format(it) } ?: "N/A"}\"")
            appendLine()
            appendLine("=== VELOCITY CALCULATION ===")
            appendLine("Calculated RPM: ${"%.0f".format(calculatedRPM)}")
            appendLine("Calculated Velocity: ${"%.0f".format(calculatedVelocity)} ticks/sec")
            appendLine("Motor TPR: $motorTicksPerRev")
            appendLine("Launch Angle: $LAUNCH_ANGLE_DEG°")
            appendLine("Shooter Height: ${"%.2f".format(SHOOTER_HEIGHT_IN)}\"")
            appendLine("Target Height: $GOAL_HEIGHT_IN\"")
        }
    }

    fun getTelemetryString(): String {
        return buildString {
            appendLine("=== LIMELIGHT AUTO VELO ===")
            appendLine("Valid Target: $hasValidTarget")
            if (hasValidTarget) {
                appendLine("Distance: ${distanceToGoal?.let { "%.2f".format(it) } ?: "N/A"} inches")
                appendLine("Required RPM: ${"%.0f".format(calculatedRPM)}")
                appendLine("Required Velocity: ${"%.0f".format(calculatedVelocity)} ticks/sec")
                appendLine("At Target Distance: $isAtTargetDistance")
            } else {
                appendLine("No target detected")
            }
        }
    }
}