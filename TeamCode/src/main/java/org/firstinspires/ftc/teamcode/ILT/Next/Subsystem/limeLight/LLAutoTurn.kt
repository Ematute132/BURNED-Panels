package org.firstinspires.ftc.teamcode.next.kotlin.subsystems

import dev.nextftc.core.commands.utility.InstantCommand
import dev.nextftc.core.subsystems.Subsystem
import org.firstinspires.ftc.teamcode.next.subsystems.DriveTrain
import kotlin.math.atan2
import kotlin.math.abs

/**
 * Auto-turning subsystem using heading lock with arctan to align with target
 * Uses Pedro Pathing's heading lock for smooth, accurate alignment
 */
object LLAutoTurn : Subsystem {

    // Target position (goal basket location on field)
    var targetX = 0.0  // X coordinate of goal in field coordinates
    var targetY = 0.0  // Y coordinate of goal in field coordinates

    // Auto-turn configuration
    var autoTurnEnabled = false
    var useHeadingLock = true        // Use Pedro's heading lock (recommended)
    var useBlindTurn = false         // Alternative: blind turn until aligned (simpler but less smooth)

    // Heading lock parameters
    var headingLockTolerance = 2.0   // Degrees - how close to consider "aligned"
    var headingLockPower = 0.4       // Power multiplier for heading corrections

    // Blind turn parameters (if not using heading lock)
    var blindTurnPower = 0.15        // Constant power when blind turning
    var angleTolerance = 1.0         // Degrees - when to stop blind turning

    // Current state (updated every loop)
    var currentTx: Double = 0.0
    var targetHeading: Double = 0.0  // Calculated heading to goal
    var currentHeading: Double = 0.0
    var headingError: Double = 0.0
    var hasValidTarget: Boolean = false
    var isAligned: Boolean = false

    override fun periodic() {
        // Get current robot position and limelight data
        currentHeading = DriveTrain.currentHeading
        hasValidTarget = limeLight.hasValidTarget
        currentTx = limeLight.currentTx

        if (autoTurnEnabled) {
            if (useHeadingLock) {
                applyHeadingLock()
            } else if (useBlindTurn) {
                applyBlindTurn()
            }
        }
    }

    /**
     * Method 1 (RECOMMENDED): Use arctan to calculate target heading and apply heading lock
     * This uses Pedro Pathing's built-in heading correction
     */
    private fun applyHeadingLock() {
        val robotX = DriveTrain.currentX
        val robotY = DriveTrain.currentY

        // Calculate angle to goal: arctan(deltaY, deltaX)
        val deltaX = targetX - robotX
        val deltaY = targetY - robotY
        targetHeading = atan2(deltaY, deltaX)

        // Calculate heading error
        headingError = normalizeAngle(targetHeading - currentHeading)
        isAligned = abs(Math.toDegrees(headingError)) <= headingLockTolerance

        // Apply heading lock to drivetrain
        // Pedro Pathing will handle the actual turning using its controller
       // DriveTrain.setTargetHeading(targetHeading)
    }

    /**
     * Method 2: Blind turn - keep turning right/left until aligned
     * Simple but less smooth than heading lock
     */
    private fun applyBlindTurn() {
        if (!hasValidTarget) {
            isAligned = false
            return
        }

        // Check if aligned
        isAligned = abs(currentTx) <= angleTolerance

        if (isAligned) {
            // Stop turning
          //  DriveTrain.setDrivePowers(forward = 0.0, strafe = 0.0, turn = 0.0)
        } else {
            // Turn in direction needed to reduce TX
            // TX negative = target is left, turn left (negative power)
            // TX positive = target is right, turn right (positive power)
            val turnDirection = if (currentTx < 0) -1.0 else 1.0
            /*DriveTrain.setDrivePowers(
                forward = 0.0,
                strafe = 0.0,
                turn = turnDirection * blindTurnPower
            )

             */
        }
    }

    /**
     * Normalize angle to [-π, π]
     */
    private fun normalizeAngle(angle: Double): Double {
        var normalized = angle
        while (normalized > Math.PI) normalized -= 2 * Math.PI
        while (normalized < -Math.PI) normalized += 2 * Math.PI
        return normalized
    }

    /**
     * Set the target position (goal location on field)
     * @param x X coordinate in field coordinates (inches)
     * @param y Y coordinate in field coordinates (inches)
     */
    fun setTargetPosition(x: Double, y: Double) {
        targetX = x
        targetY = y
    }

    // Commands
    val enableAutoTurn = InstantCommand {
        autoTurnEnabled = true
    }

    val disableAutoTurn = InstantCommand {
        autoTurnEnabled = false
       // DriveTrain.clearTargetHeading()  // Release heading lock
    }

    val toggleAutoTurn = InstantCommand {
        if (autoTurnEnabled) {
            disableAutoTurn.run()
        } else {
            enableAutoTurn.run()
        }
    }

    val useHeadingLockMode = InstantCommand {
        useHeadingLock = true
        useBlindTurn = false
    }

    val useBlindTurnMode = InstantCommand {
        useHeadingLock = false
        useBlindTurn = true
    }

    // Telemetry
    fun getTelemetryString(): String {
        return buildString {
            appendLine("=== AUTO TURN ===")
            appendLine("Enabled: $autoTurnEnabled")
            appendLine("Mode: ${if (useHeadingLock) "Heading Lock" else if (useBlindTurn) "Blind Turn" else "Disabled"}")

            if (useHeadingLock) {
                appendLine("Target Heading: ${"%.1f".format(Math.toDegrees(targetHeading))}°")
                appendLine("Current Heading: ${"%.1f".format(Math.toDegrees(currentHeading))}°")
                appendLine("Heading Error: ${"%.1f".format(Math.toDegrees(headingError))}°")
                appendLine("Aligned: $isAligned")
            } else if (useBlindTurn) {
                appendLine("Valid Target: $hasValidTarget")
                appendLine("TX Error: ${"%.2f".format(currentTx)}°")
                appendLine("Aligned: $isAligned")
            }

            appendLine("Target Position: (${"%.1f".format(targetX)}, ${"%.1f".format(targetY)})")
        }
    }

    // Debug info
    fun getDebugInfo(): String {
        return buildString {
            appendLine("=== AUTO TURN CONFIG ===")
            if (useHeadingLock) {
                appendLine("Heading Lock Tolerance: $headingLockTolerance°")
                appendLine("Heading Lock Power: $headingLockPower")
            } else if (useBlindTurn) {
                appendLine("Blind Turn Power: $blindTurnPower")
                appendLine("Angle Tolerance: $angleTolerance°")
            }
            appendLine()
            appendLine("Robot Position: (${"%.1f".format(DriveTrain.currentX)}, ${"%.1f".format(DriveTrain.currentX)})")
        }
    }
}