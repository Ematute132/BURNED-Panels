@file:Suppress("PackageName")

package org.firstinspires.ftc.teamcode.Systems.ShooterSubsystems

import com.bylazar.configurables.annotations.Configurable
import dev.nextftc.control2.feedback.PIDController
import dev.nextftc.control2.feedforward.SimpleFFCoefficients
import dev.nextftc.control2.feedforward.SimpleFeedforward
import dev.nextftc.core.subsystems.Subsystem
import dev.nextftc.hardware.controllable.MotorGroup
import dev.nextftc.hardware.impl.MotorEx
import com.qualcomm.robotcore.hardware.VoltageSensor
import dev.nextftc.ftc.ActiveOpMode
import kotlin.math.round

@Configurable
object Flywheel : Subsystem {
    val topFlywheelMotor: MotorEx = MotorEx("Fly1")
    val bottomFlywheelMotor: MotorEx = MotorEx("Fly2")
    val flywheelMotors: MotorGroup = MotorGroup(topFlywheelMotor, bottomFlywheelMotor)

    // FIX: lazy init so hardware map is ready before access
    private val battery: VoltageSensor by lazy {
        ActiveOpMode.hardwareMap.get(VoltageSensor::class.java, "Control Hub")
    }

    @JvmField var flywheelPIDCoefficients = Triple(0.0075, 0.0, 0.0)
    var flywheelPIDController: PIDController = PIDController(
        flywheelPIDCoefficients.first,
        flywheelPIDCoefficients.second,
        flywheelPIDCoefficients.third
    )

    var flywheelFFCoefficients: SimpleFFCoefficients = SimpleFFCoefficients(0.064, 0.00043, 0.0)
    private val flywheelFFController: SimpleFeedforward = SimpleFeedforward(flywheelFFCoefficients)

    private const val V_NOMINAL = 12.0
    internal const val IDLE_VELOCITY: Double = 1140.0

    internal var flywheelTarget: Double = 0.0

    private var velFilt = 0.0
    private var voltFilt = 12.0
    private const val ALPHA_VEL = 0.25
    private const val ALPHA_VOLT = 0.08

    // FIX: usePID default is true; was being set redundantly in multiple places in Shooter.kt
    internal var usePID = true

    internal fun isAtTarget(): Boolean =
        flywheelMotors.velocity > (flywheelTarget - 20.0) &&
                flywheelMotors.velocity < (flywheelTarget + 40.0)

    internal fun update(voltageCompEnabled: Boolean) {
        val target = roundToNearest20(flywheelTarget)

        val velRaw = flywheelMotors.velocity
        val voltRaw = battery.voltage.coerceAtLeast(9.0)

        velFilt += ALPHA_VEL * (velRaw - velFilt)
        voltFilt += ALPHA_VOLT * (voltRaw - voltFilt)

        val error = target - velFilt

        // FIX: removed dead PIDController(0,0,0) stub; raw is only used when usePID == false
        val raw: Double = if (usePID) {
            val pid = flywheelPIDController.calculate(error = error)
            val ff  = flywheelFFController.calculate(target)
            (pid + ff).coerceIn(-1.0, 1.0)
        } else {
            // When PID is off (STOPPED state), output 0 — target is already 0
            0.0
        }

        val pow = if (voltageCompEnabled) {
            (raw * (V_NOMINAL / voltFilt)).coerceIn(-1.0, 1.0)
        } else {
            raw
        }

        flywheelMotors.power = pow
        ActiveOpMode.telemetry.addData("flywheel power", pow)
        ActiveOpMode.telemetry.addData("goal velocity", flywheelTarget)
        ActiveOpMode.telemetry.addData("flywheel velocity", topFlywheelMotor.velocity)
    }

    internal fun roundToNearest20(velocity: Double): Double =
        round(velocity / 20.0) * 20.0
}

internal enum class FlywheelState {
    AUTO_AIM,
    MANUAL,
    IDLE,
    STOPPED;
}