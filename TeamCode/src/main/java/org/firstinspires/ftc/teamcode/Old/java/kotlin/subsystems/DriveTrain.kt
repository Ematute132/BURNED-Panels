package org.firstinspires.ftc.teamcode.next.subsystems

import com.bylazar.configurables.annotations.Configurable
import com.pedropathing.geometry.Pose
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.robot.Robot
import dev.nextftc.core.commands.Command
import dev.nextftc.core.subsystems.Subsystem
import dev.nextftc.ftc.Gamepads
import dev.nextftc.hardware.driving.FieldCentric
import dev.nextftc.hardware.driving.MecanumDriverControlled
import dev.nextftc.hardware.impl.Direction
import dev.nextftc.hardware.impl.IMUEx
import dev.nextftc.hardware.impl.MotorEx
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit
import kotlin.math.abs
import kotlin.math.cos
import kotlin.math.sin

private val IMUEx.robotYawPitchRollAngles: Any

@Configurable
object DriveTrain: Subsystem {
    val fL = MotorEx("fl").reversed().brakeMode()
    val fR = MotorEx("fr").brakeMode()
    val bL = MotorEx("bl").reversed().brakeMode()
    val bR = MotorEx("br").brakeMode()
    val imu = IMUEx("imu", Direction.RIGHT, Direction.UP)
    val heading: Double
        get() = imu

    @JvmField
    var sensistivity = 1.0

    var currentPose = Pose(0.0, 0.0, 0.0)
    val distanceFromGoal = 0
    val llAngle = 0
    val llLensHeight = 20.0
    val goalHeight = 29.5

    // Heading lock state
    private var targetHeadingLocked: Double? = null
    var headingLockEnabled = false
    var headingLockKp = 0.8  // Proportional gain for heading correction
    var headingLockMaxPower = 0.5  // Maximum turn power when correcting

    // Manual control override (when not using default command)
    private var manualControlActive = false
    private var manualForward = 0.0
    private var manualStrafe = 0.0
    private var manualTurn = 0.0

    override val defaultCommand: Command
        get() = MecanumDriverControlled(
            fL,
            fR,
            bL,
            bR,
            -Gamepads.gamepad1.leftStickY.map { it * sensistivity },
            Gamepads.gamepad1.leftStickX.map { it * sensistivity },
            Gamepads.gamepad1.rightStickX.map {
                // Apply heading lock override if enabled
                if (headingLockEnabled && targetHeadingLocked != null) {
                    calculateHeadingCorrection()
                } else {
                    it * sensistivity
                }
            },
            FieldCentric(imu)
        )

    /**
     * Set drive powers for manual control (overrides default command)
     * @param forward Forward/backward power (-1 to 1)
     * @param strafe Left/right power (-1 to 1)
     * @param turn Rotation power (-1 to 1)
     */
    fun setDrivePowers(forward: Double, strafe: Double, turn: Double) {
        manualControlActive = true
        manualForward = forward
        manualStrafe = strafe

        // If heading lock is active, override turn with heading correction
        manualTurn = if (headingLockEnabled && targetHeadingLocked != null) {
            calculateHeadingCorrection()
        } else {
            turn
        }

        // Apply powers to motors using field-centric drive
        applyFieldCentricPowers(manualForward, manualStrafe, manualTurn)
    }

    /**
     * Apply field-centric drive powers to motors
     */
    private fun applyFieldCentricPowers(forward: Double, strafe: Double, turn: Double) {
        // Get robot heading for field-centric transformation


        // Field-centric transformation
        val rotatedForward = forward * cos(heading) + strafe * sin(heading)
        val rotatedStrafe = -forward * sin(heading) + strafe * cos(heading)

        // Mecanum drive calculations
        val fLPower = rotatedForward + rotatedStrafe + turn
        val fRPower = rotatedForward - rotatedStrafe - turn
        val bLPower = rotatedForward - rotatedStrafe + turn
        val bRPower = rotatedForward + rotatedStrafe - turn

        // Normalize powers if any exceed 1.0
        val maxPower = maxOf(
            abs(fLPower),
            abs(fRPower),
            abs(bLPower),
            abs(bRPower),
            1.0
        )

        // Apply normalized powers
        fL.power = fLPower / maxPower
        fR.power = fRPower / maxPower
        bL.power = bLPower / maxPower
        bR.power = bRPower / maxPower
    }

    /**
     * Set the target heading for heading lock
     * @param heading Target heading in radians
     */
    fun setTargetHeading(heading: Double) {
        targetHeadingLocked = normalizeAngle(heading)
        headingLockEnabled = true
    }

    /**
     * Clear the target heading and disable heading lock
     */
    fun clearTargetHeading() {
        targetHeadingLocked = null
        headingLockEnabled = false
    }

    /**
     * Calculate the turn power needed to reach target heading
     */
    private fun calculateHeadingCorrection(): Double {
        val target = targetHeadingLocked ?: return 0.0
        val current = imu.heading

        // Calculate shortest angle difference
        val error = normalizeAngle(target - current)

        // Proportional control
        var turnPower = error * headingLockKp

        // Clamp to max power
        turnPower = turnPower.coerceIn(-headingLockMaxPower, headingLockMaxPower)

        return turnPower
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
     * Check if robot is aligned with target heading
     * @param tolerance Tolerance in radians
     */
    fun isHeadingAligned(tolerance: Double = Math.toRadians(2.0)): Boolean {
        val target = targetHeadingLocked ?: return false
        val current = imu.heading
        val error = normalizeAngle(target - current)
        return abs(error) <= tolerance
    }

    private fun updateZeroPowerBehavior() {
        // Small epsilon so noisy values like 0.0001 don't break the logic
        val eps = 1e-3

        val stopped =
            abs(fL.power) < eps &&
                    abs(fR.power) < eps &&
                    abs(bL.power) < eps &&
                    abs(bR.power) < eps

        val mode = if (stopped) DcMotor.ZeroPowerBehavior.BRAKE else DcMotor.ZeroPowerBehavior.FLOAT

        fL.zeroPowerBehavior = mode
        fR.zeroPowerBehavior = mode
        bL.zeroPowerBehavior = mode
        bR.zeroPowerBehavior = mode
    }

    override fun periodic() {
        // Update current pose from IMU
        currentPose = Pose(currentPose.x, currentPose.y, imu.heading)

        // Called every loop by NextFTC → keep brake/float in sync with motion
        updateZeroPowerBehavior()

        // Reset manual control flag (will be set again if setDrivePowers is called)
        manualControlActive = false
    }

    fun relocalizeWithLimelight() {
        // TODO: Implement limelight relocalization
    }

    fun ResetImu() {
        imu.zeroed()
    }

    // Telemetry for heading lock
    fun getHeadingLockTelemetry(): String {
        return buildString {
            appendLine("=== HEADING LOCK ===")
            appendLine("Enabled: $headingLockEnabled")
            if (headingLockEnabled && targetHeadingLocked != null) {
                val target = targetHeadingLocked!!
                val current = imu.heading
                val error = normalizeAngle(target - current)
                appendLine("Target: ${"%.1f".format(Math.toDegrees(target))}°")
                appendLine("Current: ${"%.1f".format(Math.toDegrees(current))}°")
                appendLine("Error: ${"%.1f".format(Math.toDegrees(error))}°")
                appendLine("Aligned: ${isHeadingAligned()}")
            }
        }
    }
}