package org.firstinspires.ftc.teamcode.nextFtc.Subsystem.Shooter

import com.bylazar.configurables.annotations.Configurable
import com.bylazar.telemetry.PanelsTelemetry
import dev.nextftc.core.subsystems.Subsystem
import dev.nextftc.hardware.impl.ServoEx
import dev.nextftc.hardware.positionable.SetPosition
import kotlin.math.hypot

@Configurable
object Hood {
    // ============================================
    // HOOD POSITIONS - TUNE ON FIELD
    // ============================================
    @JvmField var closePosition = 0.0    // Near goal
    @JvmField var midPosition = 0.5       // Middle distance
    @JvmField var farPosition = 1.0       // Far from goal

    // ============================================
    // DISTANCE THRESHOLDS (inches)
    // ============================================
    @JvmField var closeThreshold = 15.0   // Below this = close
    @JvmField var midThreshold = 35.0     // Below this = mid, above = far

    // ============================================
    // SERVO CONFIG
    // ============================================
    // Axon servos may need different turnaround time
    // Try 0.0 first, increase if jittery
    @JvmField var turnaroundTime = 0.0
    
    // Smoothing - higher = smoother but slower (0.0-1.0)
    @JvmField var smoothing = 0.15
    
    // Deadband to prevent oscillation
    @JvmField var hysteresis = 2.0

    // ============================================
    // HARDWARE
    // ============================================
    private val servo = ServoEx("hood", turnaroundTime)
    
    // Track last position to detect changes
    private var lastTargetPosition = midPosition
    private var currentPosition = midPosition

    // ============================================
    // GOAL POSITION
    // ============================================
    var goalX = 55.0   // Adjust to your goal X
    var goalY = 140.0  // Adjust to your goal Y

    // Robot position - set these from Drive in TeleOp
    var robotX = 0.0
    var robotY = 0.0

    // ============================================
    // INITIALIZATION
    // ============================================
    fun init() {
        // Initialize servo to mid position
        servo.position = midPosition
        currentPosition = midPosition
        
        PanelsTelemetry.telemetry.addLine("Hood initialized")
    }

    // ============================================
    // MAIN LOOP
    // ============================================
    fun periodic() {
        val distance = getDistanceToGoal()
        
        // Calculate target based on distance with hysteresis
        val targetFromDistance = calculateTargetPosition(distance)
        
        // Only update if changed significantly (prevents oscillation)
        if (kotlin.math.abs(targetFromDistance - lastTargetPosition) > (hysteresis / 10.0)) {
            lastTargetPosition = targetFromDistance
        }
        
        // Smooth interpolation toward target
        currentPosition = lerp(currentPosition, lastTargetPosition, smoothing)
        
        // Apply to servo
        servo.position = currentPosition
        
        // Telemetry
        PanelsTelemetry.telemetry.addData("Hood Distance", "%.1f in".format(distance))
        PanelsTelemetry.telemetry.addData("Hood Position", "%.3f".format(currentPosition))
        PanelsTelemetry.telemetry.addData("Hood Target", "%.3f".format(lastTargetPosition))
        PanelsTelemetry.telemetry.addData("Hood Raw", "%.3f".format(servo.position))
    }

    // ============================================
    // CALCULATIONS
    // ============================================
    private fun calculateTargetPosition(distance: Double): Double {
        return when {
            distance < closeThreshold -> closePosition
            distance < midThreshold -> midPosition
            else -> farPosition
        }
    }

    private fun getDistanceToGoal(): Double {
        val dx = goalX - robotX
        val dy = goalY - robotY
        return hypot(dx, dy)
    }

    private fun lerp(start: Double, end: Double, t: Double): Double {
        return start + (end - start) * t
    }

    // ============================================
    // PUBLIC API
    // ============================================
    fun setPosition(position: Double) {
        lastTargetPosition = position.coerceIn(0.0, 1.0)
        currentPosition = lastTargetPosition
        servo.position = currentPosition
    }

    fun cmdFar() = SetPosition(servo, farPosition)
    fun cmdMid() = SetPosition(servo, midPosition)
    fun cmdClose() = SetPosition(servo, closePosition)

    fun setRobotPosition(x: Double, y: Double) {
        robotX = x
        robotY = y
    }
    
    fun getPosition(): Double = servo.position
}
