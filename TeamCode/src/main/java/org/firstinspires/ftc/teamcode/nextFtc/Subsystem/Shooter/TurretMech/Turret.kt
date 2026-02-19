package org.firstinspires.ftc.teamcode.nextFtc.Subsystem.Shooter.TurretMech

import com.pedropathing.geometry.Pose
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.util.ElapsedTime
import dev.nextftc.control.KineticState
import dev.nextftc.control.builder.controlSystem
import dev.nextftc.control.feedback.PIDCoefficients
import dev.nextftc.control.feedforward.BasicFeedforward
import dev.nextftc.core.commands.Command
import dev.nextftc.core.commands.CommandManager
import dev.nextftc.core.subsystems.Subsystem
import dev.nextftc.ftc.ActiveOpMode
import dev.nextftc.hardware.impl.MotorEx
import kotlin.math.PI
import kotlin.math.abs
import kotlin.math.atan2

object Turret : Subsystem {
    enum class State {
        IDLE,   // Motor stopped
        MANUAL, // Driver control
        AIMING  // Auto-tracking mode
    }

    var currentState = State.IDLE
        private set

    var motor = MotorEx("turret")

    private const val MOTOR_TICKS = 537.7
    private const val GEAR_RATIO = 105.0 / 29.0
    private val TICKS_PER_RADIAN = MOTOR_TICKS * GEAR_RATIO / (2 * PI)

    const val MIN_ANGLE_DEG = -135.0
    const val MAX_ANGLE_DEG = 135.0

    // ================================================================
    // TUNING PARAMETERS
    // ================================================================
    /**
     * PID Coefficients
     *
     * TUNING GUIDE:
     * 1. Start with P only (I=0, D=0)
     * 2. Increase P until oscillation, then reduce by 20%
     * 3. Add small I (0.01-0.05) to eliminate steady-state error
     * 4. Add D (0.1-0.3) to reduce overshoot and dampen oscillation
     *
     * Current values (increased for faster response):
     * - kP = 1.5: Proportional gain (responsiveness)
     * - kI = 0.01: Integral gain (accuracy)
     * - kD = 0.15: Derivative gain (damping)
     */
    @JvmField var turretPID = PIDCoefficients(1.5, 0.01, 0.15)

    /**
     * Feedforward Coefficients
     *
     * Helps maintain consistent speed during motion:
     * - kV = 0.1: Velocity feedforward (compensates for friction at speed)
     * - kA = 0.0: Acceleration feedforward (not needed for this mechanism)
     * - kS = 0.0: Static friction (handled by PID)
     */
    @JvmField var turretFF = BasicFeedforward(0.1, 0.0, 0.0)

    @JvmField var maxPower = 0.75
    @JvmField var manualPower = 0.8

    /** Velocity compensation gain - INCREASED for faster robot spinning */
    @JvmField var velocityCompensationGain = 0.25

    @JvmField var alignmentToleranceDeg = 2.0

    /** Predictive lookahead for spinning robots */
    @JvmField var predictiveGain = 0.3

    private val controller = controlSystem {
        posPid(turretPID)
        feedforward(turretFF)
    }

    // ================================================================
    // STATE VARIABLES
    // ================================================================
    private var targetAngle = 0.0
    private var targetVelocity = 0.0

    var angularVelocity = 0.0
        private set

    private val velTimer = ElapsedTime()
    private var lastHeading = 0.0

    internal var lastCommand: Command? = null

    // ================================================================
    // INITIALIZATION
    // ================================================================
    override fun initialize() {
        motor.motor.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
        motor.motor.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
        velTimer.reset()
        
        ActiveOpMode.telemetry.addLine("Turret Initialized - Ensure pointing FORWARD")
    }

    // ================================================================
    // MAIN LOOP
    // ================================================================
    override fun periodic() {
        updateVelocity()

        when (currentState) {
            State.IDLE -> {
                motor.power = 0.0
            }
            State.MANUAL -> {
                // Driver control (joystick input)
                motor.power = manualPower.coerceIn(-maxPower, maxPower)
            }
            State.AIMING -> {
                // Auto-tracking using PID + feedforward
                applyControl()
            }
        }

        // Telemetry for debugging
        ActiveOpMode.telemetry.run {
            addData("=== TURRET ===", "")
            addData("State", currentState.name)
            addData("Current Angle", "%.1f".format(Math.toDegrees(getHeading())))
            addData("Target Angle", "%.1f".format(Math.toDegrees(targetAngle)))
            addData("Error", "%.1f".format(Math.toDegrees(targetAngle - getHeading())))
            addData("Motor Power", "%.2f".format(motor.power))
            addData("Angular Vel", "%.2f rad/s".format(angularVelocity))
            addData("Aligned", if (isAligned()) "YES" else "NO")
        }
    }

    // ================================================================
    // CONTROL
    // ================================================================
    private fun applyControl() {
        controller.goal = KineticState(angleToTicks(targetAngle))
        
        val calculatedPower = controller.calculate(motor.state)
        
        motor.power = calculatedPower.coerceIn(-maxPower, maxPower)
    }

    private fun updateVelocity() {
        val dt = velTimer.seconds()
        if (dt > 0.001) {
            val currentHeading = getHeading()
            val deltaAngle = normalizeAngle(currentHeading - lastHeading)
            
            angularVelocity = deltaAngle / dt
            lastHeading = currentHeading
            
            velTimer.reset()
        }
    }

    // ================================================================
    // PUBLIC API
    // ================================================================
    
    /**
     * Aim at a field-space target pose
     * Uses predictive targeting to account for robot spinning
     */
    fun aimAt(targetPose: Pose, botPose: Pose, velocityComp: Boolean = true) {
        // Calculate field-absolute angle to target
        val fieldAngle = atan2(
            targetPose.y - botPose.y,  // ΔY
            targetPose.x - botPose.x   // ΔX
        )

        // Predictive targeting - compensates for robot rotation
        val predictiveOffset = angularVelocity * predictiveGain

        // Convert to robot-relative angle (turret reference frame)
        val robotRelativeAngle = normalizeAngle(fieldAngle - botPose.heading + predictiveOffset)

        // Set target and switch to aiming mode
        currentState = State.AIMING
        setTarget(robotRelativeAngle, velocityComp)
    }

    fun setTarget(angleRad: Double, velocityComp: Boolean = false) {
        // Convert limits to radians
        val minRad = degToRad(MIN_ANGLE_DEG)
        val maxRad = degToRad(MAX_ANGLE_DEG)

        // Clamp to mechanical limits
        targetAngle = angleRad.coerceIn(minRad, maxRad)

        // Velocity compensation - predicts where turret needs to be
        targetVelocity = if (velocityComp) {
            -angularVelocity * velocityCompensationGain
        } else {
            0.0
        }
    }

    fun setManual(power: Double) {
        manualPower = power
        currentState = State.MANUAL
    }

    fun stop() {
        currentState = State.IDLE
        motor.power = 0.0

        // Cancel any scheduled commands
        lastCommand?.let {
            CommandManager.cancelCommand(it)
            lastCommand = null
        }
    }

    fun getHeading(): Double {
        return normalizeAngle(motor.currentPosition.toDouble() / TICKS_PER_RADIAN)
    }

    fun isAligned(toleranceDeg: Double = alignmentToleranceDeg): Boolean {
        val errorRad = normalizeAngle(targetAngle - getHeading())
        val errorDeg = Math.toDegrees(abs(errorRad))
        return errorDeg < toleranceDeg
    }

    fun registerCommand(command: Command) {
        // Cancel previous command if different
        if (lastCommand != null && lastCommand != command) {
            CommandManager.cancelCommand(lastCommand!!)
        }
        lastCommand = command
    }

    fun normalizeAngle(angle: Double): Double {
        var a = angle % (2 * PI)
        if (a <= -PI) a += 2 * PI
        if (a > PI) a -= 2 * PI
        return a
    }

    private fun angleToTicks(rad: Double): Double {
        return rad * TICKS_PER_RADIAN
    }

    private fun degToRad(deg: Double): Double {
        return deg * PI / 180.0
    }

    private fun radToDeg(rad: Double): Double {
        return rad * 180.0 / PI
    }
}
