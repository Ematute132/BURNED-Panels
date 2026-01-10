package org.firstinspires.ftc.teamcode.next.kotlin.subsystems

import dev.nextftc.core.commands.utility.InstantCommand
import dev.nextftc.core.subsystems.Subsystem
import org.firstinspires.ftc.teamcode.ILT.Next.Subsystem.DriveTrain
import kotlin.math.atan2
import kotlin.math.abs
import kotlin.math.hypot

/**
 * Auto-turning subsystem using heading lock with arctan to align with target
 * Works with direct motor control DriveTrain
 */
object LLAutoTurn : Subsystem {

    // Target position (goal basket location on field)
    var targetX = 0.0
    var targetY = 0.0

    // Auto-turn configuration
    var autoTurnEnabled = false
    var useHeadingLock = true
    var useBlindTurn = false

    // Tuning parameters (tune these numbers directly)
    private var headingLockTolerance = 2.0   // Degrees - how close = "aligned"
    private var blindTurnPower = 0.15        // Constant power for blind mode
    private var angleTolerance = 1.0         // Degrees for blind turn

    // Current state (updated every loop)
    var currentTx: Double = 0.0
    var targetHeading: Double = 0.0
    var currentHeading: Double = 0.0
    var headingError: Double = 0.0
    var hasValidTarget: Boolean = false
    var isAligned: Boolean = false

    override fun periodic() {
        currentHeading = DriveTrain.currentHeading
        // TODO: Add Limelight integration later
        // hasValidTarget = limeLight.hasValidTarget
        // currentTx = limeLight.currentTx

        if (autoTurnEnabled) {
            if (useHeadingLock) {
                applyHeadingLock()
            } else if (useBlindTurn) {
                applyBlindTurn()
            }
        }
    }

    /**
     * Method 1 (RECOMMENDED): Heading Lock - uses field position + arctan
     * Smoothly turns to face exact goal location
     */
    private fun applyHeadingLock() {
        val robotX = DriveTrain.currentX
        val robotY = DriveTrain.currentY

        // Calculate angle to goal: arctan2(deltaY, deltaX)
        val deltaX = targetX - robotX
        val deltaY = targetY - robotY
        targetHeading = atan2(deltaY, deltaX)

        // Calculate heading error (normalized to shortest path)
        headingError = normalizeAngle(targetHeading - currentHeading)
        isAligned = abs(Math.toDegrees(headingError)) <= headingLockTolerance

        // Tell DriveTrain to smoothly turn (uses direct motor control)
        DriveTrain.setTargetHeading(targetHeading)
    }

    /**
     * Method 2: Blind Turn - turns based on Limelight TX offset
     * Simple but less accurate than heading lock
     */
    private fun applyBlindTurn() {
        if (!hasValidTarget) {
            isAligned = false
            DriveTrain.clearTargetHeading()
            return
        }

        // Check if Limelight shows target centered
        isAligned = abs(currentTx) <= angleTolerance

        if (isAligned) {
            DriveTrain.clearTargetHeading()
        } else {
            // Turn toward target: TX<0=left (negative), TX>0=right (positive)
            val turnDirection = if (currentTx < 0) -1.0 else 1.0
            val targetHeading = currentHeading + Math.toRadians(currentTx)
            DriveTrain.setTargetHeading(targetHeading)
        }
    }

    /**
     * Normalize angle to shortest path [-180°, 180°]
     */
    private fun normalizeAngle(angle: Double): Double {
        var normalized = angle
        while (normalized > Math.PI) normalized -= 2 * Math.PI
        while (normalized < -Math.PI) normalized += 2 * Math.PI
        return normalized
    }

    /**
     * Set specific goal position on field
     */
    fun setTargetPosition(x: Double, y: Double) {
        targetX = x
        targetY = y
    }

    /**
     * Auto-select closest goal using distance (barycentric-style)
     */
    fun autoSelectClosestGoal() {
        val goals = listOf(
            72.0 to 36.0,    // Red near goal
            120.0 to 72.0,   // Red far goal
            -72.0 to 36.0,   // Blue near goal
            -120.0 to 72.0   // Blue far goal
        )

        val robotX = DriveTrain.currentX
        val robotY = DriveTrain.currentY

        var closestDist = Double.MAX_VALUE
        var bestGoalX = 0.0
        var bestGoalY = 0.0

        for ((gx, gy) in goals) {
            val dist = hypot(gx - robotX, gy - robotY)
            if (dist < closestDist) {
                closestDist = dist
                bestGoalX = gx
                bestGoalY = gy
            }
        }

        setTargetPosition(bestGoalX, bestGoalY)
    }

    // Commands for button binding
    val enableAutoTurn = InstantCommand {
        autoTurnEnabled = true
    }

    val disableAutoTurn = InstantCommand {
        autoTurnEnabled = false
        DriveTrain.clearTargetHeading()
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

    /**
     * Rich telemetry for driver station
     */
    fun getTelemetryString(): String {
        return buildString {
            appendLine("=== AUTO TURN ===")
            appendLine("Enabled: $autoTurnEnabled")
            appendLine("Mode: ${if (useHeadingLock) "HEADING LOCK" else if (useBlindTurn) "BLIND TURN" else "OFF"}")

            if (useHeadingLock) {
                appendLine("Target: ${"%.1f".format(Math.toDegrees(targetHeading))}°")
                appendLine("Current: ${"%.1f".format(Math.toDegrees(currentHeading))}°")
                appendLine("Error: ${"%.1f".format(Math.toDegrees(headingError))}°")
                appendLine("✅ ALIGNED: $isAligned")
            } else if (useBlindTurn) {
                appendLine("Target: $hasValidTarget")
                appendLine("TX: ${"%.2f".format(currentTx)}°")
                appendLine("✅ ALIGNED: $isAligned")
            }

            appendLine("Goal: (${"%.1f".format(targetX)}, ${"%.1f".format(targetY)})")
            appendLine("Zone: ${DriveTrain.inShootZone()}")
        }
    }
}