package org.firstinspires.ftc.teamcode.nextFtc.Subsystem.Shooter

import com.bylazar.telemetry.PanelsTelemetry
import dev.nextftc.core.subsystems.Subsystem
import dev.nextftc.hardware.impl.ServoEx
import dev.nextftc.hardware.positionable.SetPosition
import org.firstinspires.ftc.teamcode.ILT.Next.Subsystems.Drive

object Hood : Subsystem {

    // ============================================
    // HOOD POSITIONS - TUNE THESE ON FIELD!
    // ============================================
    @JvmField var DOWN = 0.0
    @JvmField var MID  = 0.5
    @JvmField var FAR  = 1.0

    // ============================================
    // DISTANCE THRESHOLDS - TUNE THESE ON FIELD!
    // ============================================
    @JvmField var CLOSE_THRESHOLD = 20.0
    @JvmField var MID_THRESHOLD   = 40.0

    private val servo = ServoEx("hood", 0.01)

    var currentTargetPosition: Double = MID
        private set

    // Robot position suppliers — set from TeleOp before use, or rely on periodic() auto-update
    var robotX: () -> Double = { 0.0 }
    var robotY: () -> Double = { 0.0 }
    var goalX: Double = 55.0
    var goalY: Double = 0.0

    // FIX: auto-update flag so a manual override (setPosition / cmdFar etc.) can suppress
    // the distance-based auto logic for one cycle. Reset to true each periodic unless held.
    var autoUpdate: Boolean = true

    // Commands capture fresh tuned values at creation time (fun, not val)
    fun cmdFar()   = SetPosition(servo, FAR)
    fun cmdMid()   = SetPosition(servo, MID)
    fun cmdClose() = SetPosition(servo, DOWN)

    fun setGoalPosition(gx: Double, gy: Double) {
        goalX = gx
        goalY = gy
    }

    fun setPositionProviders(x: () -> Double, y: () -> Double) {
        robotX = x
        robotY = y
    }

    /**
     * Manually set a servo position and disable auto-update so the manual value is not
     * immediately overwritten by the distance logic in the same periodic() call.
     * Call autoUpdate = true to restore autonomous distance tracking.
     */
    fun setPosition(newPosition: Double) {
        autoUpdate = false          // FIX: prevent periodic() from overwriting this immediately
        currentTargetPosition = newPosition.coerceIn(0.0, 1.0)
        servo.position = currentTargetPosition
    }

    fun setForDistance(distanceInches: Double): Double {
        val position = when {
            distanceInches < CLOSE_THRESHOLD -> DOWN
            distanceInches < MID_THRESHOLD   -> MID
            else                             -> FAR
        }
        // Use internal write so we don't toggle autoUpdate
        currentTargetPosition = position
        servo.position = position
        return position
    }

    val currentPosition: Double get() = servo.position

    private fun getDistanceToGoal(): Double {
        val dx = goalX - robotX()
        val dy = goalY - robotY()
        return kotlin.math.hypot(dx, dy)
    }

    override fun periodic() {
        // FIX: update position providers BEFORE computing distance so the same-cycle distance
        // calculation uses the freshest odometry values (was updating AFTER in original code).
        setPositionProviders({ Drive.currentX }, { Drive.currentY })

        if (autoUpdate) {
            val distance = getDistanceToGoal()
            setForDistance(distance)
        }

        PanelsTelemetry.telemetry.addData("Hood Auto", autoUpdate)
        PanelsTelemetry.telemetry.addData("Hood Distance", "%.1f".format(getDistanceToGoal()))
        PanelsTelemetry.telemetry.addData("Hood Position", "%.3f".format(currentPosition))
        PanelsTelemetry.telemetry.addData("Hood Target", "%.3f".format(currentTargetPosition))
    }
}