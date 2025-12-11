package org.firstinspires.ftc.teamcode.next.kotlin.subsystems

import com.qualcomm.hardware.limelightvision.Limelight3A
import dev.nextftc.core.subsystems.Subsystem
import dev.nextftc.ftc.ActiveOpMode
import kotlin.math.*

object LLAutoVelo2 : Subsystem {

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
    private const val SHOOTER_RADIUS_IN = LLAutoVelo.SHOOTER_DIAMETER_IN / 2.0

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
        LLAutoVelo.ll = ActiveOpMode.hardwareMap.get(Limelight3A::class.java, "ll")
        LLAutoVelo.ll.setPollRateHz(100)
        LLAutoVelo.ll.pipelineSwitch(0)
        LLAutoVelo.ll.start()
    }

    override fun periodic() {
        LLAutoVelo.updateDistanceCalculation()
        LLAutoVelo.updateVelocityCalculation()
    }

    private fun updateDistanceCalculation() {
        val result = LLAutoVelo.ll.latestResult
        if (result != null && result.isValid) {
            LLAutoVelo.hasValidTarget = true
            val targetOffsetAngleVertical = result.ty
            LLAutoVelo.currentTy = targetOffsetAngleVertical

            // Calculate angles
            LLAutoVelo.angleToGoalDegrees = LLAutoVelo.llAngle + targetOffsetAngleVertical
            LLAutoVelo.angleToGoalRadians = LLAutoVelo.angleToGoalDegrees * (PI / 180.0)

            // Calculate distance
            LLAutoVelo.distanceToGoal = (LLAutoVelo.goalHeight - LLAutoVelo.llLensHeight) / tan(
                LLAutoVelo.angleToGoalRadians
            )

            // Check if at target distance
            LLAutoVelo.distanceToGoal?.let { dist ->
                LLAutoVelo.isAtTargetDistance = abs(dist - LLAutoVelo.targetDistance) <= LLAutoVelo.distanceTolerance
            } ?: run {
                LLAutoVelo.isAtTargetDistance = false
            }
        } else {
            LLAutoVelo.hasValidTarget = false
            LLAutoVelo.distanceToGoal = null
            LLAutoVelo.angleToGoalDegrees = 0.0
            LLAutoVelo.angleToGoalRadians = 0.0
            LLAutoVelo.isAtTargetDistance = false
            LLAutoVelo.currentTy = 0.0
        }
    }

    private fun updateVelocityCalculation() {
        val distance = LLAutoVelo.distanceToGoal
        if (distance != null && distance > 0) {
            LLAutoVelo.calculatedVelocity = LLAutoVelo.calculateMotorVelocity(distance)
            LLAutoVelo.calculatedRPM = LLAutoVelo.calculateRPM(distance)
        } else {
            LLAutoVelo.calculatedVelocity = 0.0
            LLAutoVelo.calculatedRPM = 0.0
        }
    }


    fun calculateRPM(distanceToTarget: Double): Double {
        // Convert angle to radians
        val theta = Math.toRadians(LLAutoVelo.LAUNCH_ANGLE_DEG)
        val cosTheta = cos(theta)
        val tanTheta = tan(theta)

        val heightDiff = LLAutoVelo.GOAL_HEIGHT_IN - LLAutoVelo.SHOOTER_HEIGHT_IN

        // Physics formula: v = sqrt(g * d^2 / (2 * cos^2(θ) * (d * tan(θ) - h)))
        val numerator = LLAutoVelo.GRAVITY_IN_PER_S2 * distanceToTarget * distanceToTarget
        val denominator = 2.0 * cosTheta * cosTheta * (distanceToTarget * tanTheta - heightDiff)

        // Guard against impossible geometry
        if (denominator <= 0) return 0.0

        // Calculate linear velocity in inches per second
        val velocityInPerSec = sqrt(numerator / denominator)

        // Convert linear velocity to RPM: v = ω * r → RPM = (v / r) * (60 / 2π)
        val rpm = 60.0 * velocityInPerSec / (2.0 * PI * LLAutoVelo.SHOOTER_RADIUS_IN)

        return rpm
    }

    fun calculateMotorVelocity(distanceToTarget: Double): Double {
        val rpm = LLAutoVelo.calculateRPM(distanceToTarget)
        if (rpm == 0.0) return 0.0

        // Convert RPM to ticks per second
        val ticksPerSec = (rpm / 60.0) * LLAutoVelo.motorTicksPerRev

        return ticksPerSec
    }

    fun getDistanceToTarget(): Double? {
        return LLAutoVelo.distanceToGoal
    }

    fun getCalculatedVelocity(): Double {
        return LLAutoVelo.calculatedVelocity
    }

    fun getCalculatedRPM(): Double {
        return LLAutoVelo.calculatedRPM
    }

    fun getDistanceDebugInfo(): String {
        return buildString {
            appendLine("=== DISTANCE DEBUG ===")
            appendLine("Valid Target: ${LLAutoVelo.hasValidTarget}")
            appendLine("llAngle: ${LLAutoVelo.llAngle}°")
            appendLine("TY: ${"%.2f".format(LLAutoVelo.currentTy)}°")
            appendLine("Angle to Goal: ${"%.2f".format(LLAutoVelo.angleToGoalDegrees)}°")
            appendLine("Angle (radians): ${"%.4f".format(LLAutoVelo.angleToGoalRadians)}")
            appendLine("Goal Height: ${LLAutoVelo.goalHeight}\"")
            appendLine("Lens Height: ${LLAutoVelo.llLensHeight}\"")
            appendLine("Height Diff: ${LLAutoVelo.goalHeight - LLAutoVelo.llLensHeight}\"")
            appendLine("tan(angle): ${"%.4f".format(Math.tan(LLAutoVelo.angleToGoalRadians))}")
            appendLine("Distance: ${LLAutoVelo.distanceToGoal?.let { "%.2f".format(it) } ?: "N/A"}\"")
            appendLine()
            appendLine("=== VELOCITY CALCULATION ===")
            appendLine("Calculated RPM: ${"%.0f".format(LLAutoVelo.calculatedRPM)}")
            appendLine("Calculated Velocity: ${"%.0f".format(LLAutoVelo.calculatedVelocity)} ticks/sec")
            appendLine("Motor TPR: ${LLAutoVelo.motorTicksPerRev}")
            appendLine("Launch Angle: ${LLAutoVelo.LAUNCH_ANGLE_DEG}°")
            appendLine("Shooter Height: ${"%.2f".format(LLAutoVelo.SHOOTER_HEIGHT_IN)}\"")
            appendLine("Target Height: ${LLAutoVelo.GOAL_HEIGHT_IN}\"")
        }
    }

    fun getTelemetryString(): String {
        return buildString {
            appendLine("=== LIMELIGHT AUTO VELO ===")
            appendLine("Valid Target: ${LLAutoVelo.hasValidTarget}")
            if (LLAutoVelo.hasValidTarget) {
                appendLine("Distance: ${LLAutoVelo.distanceToGoal?.let { "%.2f".format(it) } ?: "N/A"} inches")
                appendLine("Required RPM: ${"%.0f".format(LLAutoVelo.calculatedRPM)}")
                appendLine("Required Velocity: ${"%.0f".format(LLAutoVelo.calculatedVelocity)} ticks/sec")
                appendLine("At Target Distance: ${LLAutoVelo.isAtTargetDistance}")
            } else {
                appendLine("No target detected")
            }
        }
    }
}