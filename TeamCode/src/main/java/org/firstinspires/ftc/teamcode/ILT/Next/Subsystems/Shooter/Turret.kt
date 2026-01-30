package org.firstinspires.ftc.teamcode.ILT.Next.Subsystems.Shooter

import com.bylazar.configurables.annotations.Configurable
import com.qualcomm.robotcore.hardware.DcMotor
import dev.nextftc.control.KineticState
import dev.nextftc.control.builder.controlSystem
import dev.nextftc.control.feedback.PIDCoefficients
import dev.nextftc.control.feedforward.BasicFeedforwardParameters
import dev.nextftc.core.subsystems.Subsystem
import dev.nextftc.hardware.impl.MotorEx
import org.firstinspires.ftc.teamcode.ILT.Next.Data.Alliance
import org.firstinspires.ftc.teamcode.ILT.Next.Subsystems.Drive.currentHeading
import org.firstinspires.ftc.teamcode.ILT.Next.Subsystems.Drive.currentX
import org.firstinspires.ftc.teamcode.ILT.Next.Subsystems.Drive.currentY
import org.firstinspires.ftc.teamcode.ILT.Next.Subsystems.Drive.poseValid

import kotlin.math.*

@Configurable
object Turret : Subsystem {

    enum class State { IDLE,ODOMETRY }

    var alliance = Alliance.RED
    val turret = MotorEx("turret")
    @JvmField var pid = PIDCoefficients(0.5, 0.0, 0.1)
    @JvmField var feedForward = BasicFeedforwardParameters(0.1,0.0,0.0)
    var controller = controlSystem {
        posPid(pid)
        basicFF(feedForward)
    }
    @JvmField var RED_GOAL_X = 9.25
    @JvmField var BLUE_GOAL_X = 134.75
    val goalX: Double
        get() = if (alliance == Alliance.RED) {
            RED_GOAL_X
        } else {
            BLUE_GOAL_X
        }


    @JvmField var goalY = 132.0


    const val GEAR_RATIO = 3.62068965517  // 105/29
    const val MOTOR_TICKS_PER_REV = 537.7
    var turretYaw: Double = 0.0


    @JvmField var currentState = State.IDLE
    // ==================== TUNING PARAMETERS ====================
    @JvmField var minPower: Double = 0.10  // Reduced - let controller handle most of it
    @JvmField var maxPower: Double = 1.0 // Increased ceiling for faster tracking


    // Robot rotation compensation
    @JvmField var useRobotVelocityCompensation: Boolean = true
    @JvmField var robotVelocityGain: Double = 0.9
    private var filteredRobotAngularVelocity: Double = 0.0

    // Velocity tracking
    private var lastYaw: Double = 0.0
    private var lastTime: Long = System.nanoTime()
    private var currentVelocity: Double = 0.0

    private var lastRobotHeading: Double = 0.0
    private var lastHeadingTime: Long = System.nanoTime()

    // Motion profiling
    @JvmField var maxVelocity: Double = 3.5
    @JvmField var maxAcceleration: Double = 6.0
    @JvmField var useMotionProfile: Boolean = true
    @JvmField var nearTargetErrorDeg: Double = 5.0   // Cap desired vel when error < this
    @JvmField var nearTargetMaxVel: Double = 0.8    // rad/s when close

    // Filtered Limelight tx (EMA) to reduce oscillation from vision noise
    private var filteredTxDeg: Double = 0.0

    private const val RADIANS_PER_TICK = 2.0 * PI /
            (MOTOR_TICKS_PER_REV * GEAR_RATIO)

    override fun initialize() {
        turret.motor.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
        turret.motor.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
        lastYaw = getYaw()
        lastTime = System.nanoTime()
        lastRobotHeading = currentHeading
        lastHeadingTime = System.nanoTime()
    }

    override fun periodic() {
        updateVelocity()
        updateRobotVelocity()

        turretYaw = getYaw()

        when (currentState) {
            State.IDLE -> turret.power = 0.0
            State.ODOMETRY -> aimWithOdometryOnly()
        }
    }

    /** Track robot rotation velocity; low-pass filtered to reduce noise. */
    private fun updateRobotVelocity() {
        val currentHeading = currentHeading
        val currentTime = System.nanoTime()
        val dt = (currentTime - lastHeadingTime) / 1e9

        if (dt > 0) {
            var deltaHeading = currentHeading - lastRobotHeading
            if (deltaHeading > PI) deltaHeading -= 2.0 * PI
            if (deltaHeading < -PI) deltaHeading += 2.0 * PI
            val raw = deltaHeading / dt
            val alpha = 0.3
            filteredRobotAngularVelocity = alpha * raw + (1.0 - alpha) * filteredRobotAngularVelocity
        }

        lastRobotHeading = currentHeading
        lastHeadingTime = currentTime
    }

    private fun updateVelocity() {
        val currentYaw = getYaw()
        val currentTime = System.nanoTime()
        val dt = (currentTime - lastTime) / 1e9

        if (dt > 0) {
            var deltaYaw = currentYaw - lastYaw

            if (deltaYaw > PI) deltaYaw -= 2.0 * PI
            if (deltaYaw < -PI) deltaYaw += 2.0 * PI

            currentVelocity = deltaYaw / dt
        }

        lastYaw = currentYaw
        lastTime = currentTime
    }






    fun aimWithOdometryOnly() {
        if (!poseValid) {
            turret.power = 0.0
            return
        }

        val deltaX = goalX - currentX
        val deltaY = goalY - currentY
        val fieldAngle = atan2(deltaY, deltaX)

        // REMOVE the IF check. Ensure currentHeading is ALWAYS Radians.
        val targetYaw = normalizeAngle(fieldAngle - currentHeading)

        applyControlWithVelocity(targetYaw)
    }

    private fun applyControlWithVelocity(targetYaw: Double) {
        val currentYaw = getYaw()

        // 1. CLAMP FIRST: Logic should treat the boundary as a wall
        val clampedTarget = targetYaw.coerceIn(-3 * PI/4, 3 * PI/4)

        // 2. ERROR: Find the shortest path TO THE CLAMPED TARGET
        // Using normalizeAngle(target - current) is safer than a custom while loop
        val errorRad = normalizeAngle(clampedTarget - currentYaw)
        val errorDeg = Math.toDegrees(abs(errorRad))

        // 3. VELOCITY: If using Robot Comp, it should be a subtraction from the feedforward
        var profiledVel = if (useMotionProfile) calculateProfiledVelocity(currentYaw, clampedTarget) else 0.0

        if (useRobotVelocityCompensation) {
            // This 'predicts' where the robot is going
            profiledVel -= filteredRobotAngularVelocity * robotVelocityGain
        }

        // 4. CONTROL: Feed the error-corrected goal to the controller
        controller.goal = KineticState(clampedTarget, profiledVel)
        var power = controller.calculate(KineticState(currentYaw, currentVelocity))

        // 5. ANTI-STUTTER: Simple deadband
        if (errorDeg < 0.5) power = 0.0

        turret.power = power.coerceIn(-maxPower, maxPower)
    }
    /**
     * Shortest angular error in [-PI, PI]. Use this for tolerance checks and control.
     */
    private fun shortestAngularError(current: Double, target: Double): Double {
        var e = target - current
        while (e > PI) e -= 2.0 * PI
        while (e < -PI) e += 2.0 * PI
        return e
    }



    private fun calculateProfiledVelocity(currentPos: Double, targetPos: Double): Double {
        var error = targetPos - currentPos

        if (error > PI) error -= 2.0 * PI
        if (error < -PI) error += 2.0 * PI

        val errorAbs = abs(error)
        val direction = if (error > 0) 1.0 else -1.0

        // Deceleration distance based on current velocity
        val decelDistance = (currentVelocity * currentVelocity) / (2.0 * maxAcceleration)

        val targetVelocity = if (errorAbs < decelDistance) {
            // Deceleration phase
            sqrt(2.0 * maxAcceleration * errorAbs) * direction
        } else {
            // Acceleration/constant velocity phase
            maxVelocity * direction
        }

        return targetVelocity.coerceIn(-maxVelocity, maxVelocity)
    }

    // ==================== UTILITIES ====================

    fun getYaw(): Double = normalizeAngle(turret.currentPosition * RADIANS_PER_TICK)
    fun normalizeAngle(radians: Double): Double {
        var angle = radians % (2.0 * PI)
        if (angle <= -PI) angle += 2.0 * PI
        if (angle > PI) angle -= 2.0 * PI
        return angle
    }

    // ==================== COMMANDS ====================


    fun aimWithOdometry() { currentState = State.ODOMETRY }

    fun stop() {
        currentState = State.IDLE

    }
}