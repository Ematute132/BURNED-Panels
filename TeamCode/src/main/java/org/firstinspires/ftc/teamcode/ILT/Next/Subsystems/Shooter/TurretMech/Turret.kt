package org.firstinspires.ftc.teamcode.nextFtc.Subsystem.Shooter.TurretMech

import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.util.ElapsedTime
import dev.nextftc.control.KineticState
import dev.nextftc.control.builder.controlSystem
import dev.nextftc.core.commands.Command
import dev.nextftc.core.commands.CommandManager
import dev.nextftc.core.subsystems.Subsystem
import dev.nextftc.hardware.impl.MotorEx
import kotlin.math.PI
import kotlin.math.abs

object Turret : Subsystem {

    enum class State { IDLE, MANUAL, AIMING }

    var motor = MotorEx("turret")

    // ============================================
    // PHYSICAL CONSTANTS - MEASURE YOUR ROBOT
    // ============================================
    const val GEAR_RATIO = 105.0 / 29.0     // FIX: exact fraction instead of truncated 3.62
    const val MOTOR_TICKS = 537.7
    private const val TICKS_PER_RADIAN = MOTOR_TICKS * GEAR_RATIO / (2 * PI)

    const val MIN_ANGLE = -135.0
    const val MAX_ANGLE = 135.0

    // ============================================
    // PID VALUES - TUNE THESE
    // ============================================
    var controller = controlSystem {
        posPid(0.3, 0.0, 0.05)
        basicFF(0.25, 0.0, 0.0)
    }

    @JvmField var minPower = 0.15
    @JvmField var maxPower = 0.75

    // ============================================
    // STATE
    // ============================================
    var currentState = State.IDLE
    var manualPower = 0.0

    // Velocity tracking
    private val velTimer = ElapsedTime()
    private var lastHeading = 0.0
    var angularVelocity = 0.0
        private set

    // Target
    private var targetAngle = 0.0
    private var targetVelocity = 0.0

    // Goal position

    // Command tracking
    internal var lastCommand: Command? = null

    // ============================================
    // INITIALIZATION
    // ============================================
    override fun initialize() {
        motor.motor.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
        motor.motor.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
        velTimer.reset()
    }

    // ============================================
    // MAIN LOOP
    // ============================================
    override fun periodic() {
        updateVelocity()

        when (currentState) {
            State.IDLE -> motor.power = 0.0
            State.MANUAL -> motor.power = manualPower.coerceIn(-maxPower, maxPower)
            State.AIMING -> applyControl()
        }
    }

    private fun applyControl() {
        controller.goal = KineticState(
            angleToTicks(targetAngle),
            targetVelocity
        )

        var power = controller.calculate(motor.state)

        // FIX: use normalized error to avoid huge error values from un-normalized getHeading()
        val error = normalizeAngle(targetAngle - getHeading())
        if (abs(Math.toDegrees(error)) > 0.5) {
            power += if (power >= 0) minPower else -minPower
        } else if (abs(targetVelocity) < 0.1) {
            power = 0.0
        }

        motor.power = power.coerceIn(-maxPower, maxPower)
    }

    private fun updateVelocity() {
        val dt = velTimer.seconds()
        if (dt > 0.001) {
            val currentHeading = getHeading()
            val delta = normalizeAngle(currentHeading - lastHeading)
            angularVelocity = delta / dt
            lastHeading = currentHeading
            velTimer.reset()
        }
    }

    // ============================================
    // PUBLIC API
    // ============================================

    fun setTarget(angleRad: Double, velocityComp: Boolean = true) {
        val minRad = degToRad(MIN_ANGLE)
        val maxRad = degToRad(MAX_ANGLE)
        targetAngle = angleRad.coerceIn(minRad, maxRad)
        targetVelocity = if (velocityComp) -angularVelocity * 0.25 else 0.0
    }

    fun setManual(power: Double) {
        manualPower = power
        currentState = State.MANUAL
    }

    fun stop() {
        currentState = State.IDLE
        motor.power = 0.0
        lastCommand?.let {
            CommandManager.cancelCommand(it)
            lastCommand = null
        }
    }



    // FIX: normalize so heading stays within [-PI, PI] instead of accumulating unbounded
    fun getHeading(): Double = normalizeAngle(motor.currentPosition.toDouble() / TICKS_PER_RADIAN)

    fun normalizeAngle(angle: Double): Double {
        var a = angle % (2 * PI)
        if (a <= -PI) a += 2 * PI
        if (a > PI) a -= 2 * PI
        return a
    }

    // FIX: added registerCommand back for safe multi-command scheduling
    fun registerCommand(command: Command) {
        if (lastCommand != null && lastCommand != command) {
            CommandManager.cancelCommand(lastCommand!!)
        }
        lastCommand = command
    }

    private fun angleToTicks(rad: Double) = rad * TICKS_PER_RADIAN
    private fun degToRad(deg: Double) = deg * PI / 180.0
}