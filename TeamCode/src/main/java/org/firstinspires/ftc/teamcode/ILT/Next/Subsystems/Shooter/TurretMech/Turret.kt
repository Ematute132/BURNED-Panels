package org.firstinspires.ftc.teamcode.nextFtc.Subsystem.Shooter.TurretMech

import com.pedropathing.geometry.Pose
import com.qualcomm.hardware.limelightvision.Limelight3A
import com.qualcomm.hardware.limelightvision.LLResult
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.util.ElapsedTime
import dev.nextftc.control.KineticState
import dev.nextftc.control.builder.controlSystem
import dev.nextftc.core.commands.Command
import dev.nextftc.core.commands.CommandManager
import dev.nextftc.core.subsystems.Subsystem
import dev.nextftc.extensions.pedro.PedroComponent.Companion.follower
import dev.nextftc.hardware.impl.MotorEx
import dev.nextftc.ftc.ActiveOpMode
import kotlin.math.PI
import kotlin.math.abs
import kotlin.math.atan2

object Turret : Subsystem {

    enum class State { IDLE, MANUAL, AIMING }

    var motor = MotorEx("turret")

    // ============================================
    // PHYSICAL CONSTANTS
    // ============================================
    val GEAR_RATIO = 105.0 / 29.0
    const val MOTOR_TICKS = 537.7
    private val TICKS_PER_RADIAN = MOTOR_TICKS * (105.0 / 29.0) / (2 * PI)

    const val MIN_ANGLE_DEG = -135.0
    const val MAX_ANGLE_DEG = 135.0

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

    private val velTimer = ElapsedTime()
    private var lastHeading = 0.0
    var angularVelocity = 0.0
        private set

    private var targetAngle = 0.0
    private var targetVelocity = 0.0

    internal var lastCommand: Command? = null

    // ============================================
    // LIMELIGHT
    // ============================================
    private lateinit var limelight: Limelight3A
    var hasRelocalized = false
        private set
    var limelightValid = false
        private set

    // ============================================
    // INITIALIZATION
    // ============================================
    override fun initialize() {
        motor.motor.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
        motor.motor.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
        velTimer.reset()

        // Init limelight
        limelight = ActiveOpMode.hardwareMap.get(Limelight3A::class.java, "limelight")
        limelight.pipelineSwitch(8)
        limelight.start()

        // Relocalize at home using encoder position as baseline
        relocalizeAtHome()
    }

    // ============================================
    // MAIN LOOP
    // ============================================
    override fun periodic() {
        updateVelocity()
        relocalizeWithLimelight()

        when (currentState) {
            State.IDLE -> motor.power = 0.0
            State.MANUAL -> motor.power = manualPower.coerceIn(-maxPower, maxPower)
            State.AIMING -> applyControl()
        }

        // Telemetry
        val result = limelight.latestResult
        limelightValid = result != null && result.isValid
        ActiveOpMode.telemetry.addData("Turret State", currentState.name)
        ActiveOpMode.telemetry.addData("Turret Heading (deg)", "%.1f".format(Math.toDegrees(getHeading())))
        ActiveOpMode.telemetry.addData("Turret Target (deg)", "%.1f".format(Math.toDegrees(targetAngle)))
        ActiveOpMode.telemetry.addData("Motor Power", "%.2f".format(motor.power))
        ActiveOpMode.telemetry.addData("AngVel (rad/s)", "%.2f".format(angularVelocity))
        ActiveOpMode.telemetry.addData("Has Relocalized", hasRelocalized)
        ActiveOpMode.telemetry.addData("Limelight Valid", limelightValid)
        if (limelightValid) {
            ActiveOpMode.telemetry.addData("Limelight Tags", result!!.fiducialResults.size)
        }
    }

    // ============================================
    // LIMELIGHT RELOCALIZATION
    // ============================================

    /**
     * Called every loop. If Limelight has a valid MT2 result, updates Pedro's
     * pose estimate and resyncs the turret heading baseline.
     */
    private fun relocalizeWithLimelight() {
        val currentPose = follower.pose
        limelight.updateRobotOrientation(Math.toDegrees(currentPose.heading))

        val result: LLResult? = limelight.latestResult
        if (result != null && result.isValid) {
            val botpose = result.botpose_MT2 ?: return

            val xInches = botpose.position.x * 39.3701
            val yInches = botpose.position.y * 39.3701
            val yawRad = Math.toRadians(botpose.orientation.yaw)

            // Update Pedro's pose with vision fix
            follower.pose = Pose(xInches, yInches, yawRad)

            hasRelocalized = true
        }
    }

    /**
     * Baseline relocalization using current encoder position.
     * Called on init before Limelight has data.
     */
    private fun relocalizeAtHome() {
        hasRelocalized = true
    }

    // ============================================
    // CONTROL
    // ============================================

    private fun applyControl() {
        controller.goal = KineticState(
            angleToTicks(targetAngle),
            targetVelocity
        )

        var power = controller.calculate(motor.state)

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

    /**
     * Aim at a field-space target pose given the robot's current pose.
     * Handles all angle math internally.
     */
    fun aimAt(targetPose: Pose, botPose: Pose, velocityComp: Boolean = true) {
        val fieldAngle = atan2(
            targetPose.y - botPose.y,
            targetPose.x - botPose.x
        )
        val angleRad = normalizeAngle(fieldAngle - botPose.heading)
        currentState = State.AIMING
        setTarget(angleRad, velocityComp)
    }

    fun setTarget(angleRad: Double, velocityComp: Boolean = true) {
        val minRad = degToRad(MIN_ANGLE_DEG)
        val maxRad = degToRad(MAX_ANGLE_DEG)
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

    fun getHeading(): Double = normalizeAngle(motor.currentPosition.toDouble() / TICKS_PER_RADIAN)

    fun normalizeAngle(angle: Double): Double {
        var a = angle % (2 * PI)
        if (a <= -PI) a += 2 * PI
        if (a > PI) a -= 2 * PI
        return a
    }

    fun registerCommand(command: Command) {
        if (lastCommand != null && lastCommand != command) {
            CommandManager.cancelCommand(lastCommand!!)
        }
        lastCommand = command
    }

    fun isAligned(toleranceDeg: Double = 2.0): Boolean =
        abs(Math.toDegrees(normalizeAngle(targetAngle - getHeading()))) < toleranceDeg

    private fun angleToTicks(rad: Double) = rad * TICKS_PER_RADIAN
    private fun degToRad(deg: Double) = deg * PI / 180.0
}