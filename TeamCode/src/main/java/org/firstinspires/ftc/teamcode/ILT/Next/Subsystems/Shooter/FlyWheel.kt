@file:Suppress("PackageName")

package org.firstinspires.ftc.teamcode.Systems.ShooterSubsystems

import com.bylazar.configurables.annotations.Configurable
import com.bylazar.telemetry.PanelsTelemetry
import dev.nextftc.control2.feedback.PIDController
import dev.nextftc.control2.feedforward.SimpleFFCoefficients
import dev.nextftc.control2.feedforward.SimpleFeedforward
import dev.nextftc.core.subsystems.Subsystem
import dev.nextftc.hardware.controllable.MotorGroup
import dev.nextftc.hardware.impl.MotorEx

import com.qualcomm.robotcore.hardware.VoltageSensor
import dev.nextftc.ftc.ActiveOpMode
import dev.nextftc.ftc.ActiveOpMode.hardwareMap
import kotlin.math.round

@Configurable
object Flywheel: Subsystem {
    val topFlywheelMotor: MotorEx = MotorEx("Fly1")
    val bottomFlywheelMotor: MotorEx = MotorEx("Fly2")
    val flywheelMotors: MotorGroup = MotorGroup(topFlywheelMotor, bottomFlywheelMotor)
    private val battery: VoltageSensor by lazy { ActiveOpMode.hardwareMap.get(VoltageSensor::class.java, "Control Hub") }


   var flywheelPIDController: PIDController = PIDController(0.0075,0.0,0.0)

    var flywheelFFCoefficients: SimpleFFCoefficients = SimpleFFCoefficients(0.064,0.00043,0.0)
    private val flywheelFFController: SimpleFeedforward = SimpleFeedforward(flywheelFFCoefficients)

    private const val V_NOMINAL = 12.0
    internal const val IDLE_VELOCITY: Double = 1140.0

    internal var flywheelTarget: Double = 0.0
    private var velFilt = 0.0
    private var voltFilt = 12.0
    private const val ALPHA_VEL = 0.25
    private const val ALPHA_VOLT = 0.08

    internal fun isAtTarget(): Boolean { return ((flywheelTarget - 20.0) < flywheelMotors.velocity) && ((flywheelTarget + 40.0) > flywheelMotors.velocity) }

    internal var usePID = true


    internal fun update(voltageCompEnabled: Boolean) {
        val target = roundToNearest20(flywheelTarget)

        val velRaw = flywheelMotors.velocity
        val voltRaw = battery.voltage.coerceAtLeast(9.0)

        velFilt += ALPHA_VEL * (velRaw - velFilt)
        voltFilt += ALPHA_VOLT * (voltRaw - voltFilt)

        val error = target - velFilt
        val pid = flywheelPIDController.calculate(error = error)
        val ff  = flywheelFFController.calculate(target)
        var raw = (PIDController(0.0, 0.0, 0.0).calculate(error = error) + 0.0).coerceIn(-1.0, 1.0)
        if (usePID) {
            raw = (pid + ff).coerceIn(-1.0, 1.0)
        }

        val pow = if (voltageCompEnabled) {
            (raw * (V_NOMINAL / voltFilt)).coerceIn(-1.0,1.0)
        } else {
            raw
        }

        flywheelMotors.power = pow
        ActiveOpMode.telemetry.addData("flywheel power:", pow)
        ActiveOpMode.telemetry.addData("goal velocity:", flywheelTarget)
        ActiveOpMode.telemetry.addData("flywheel velocity", topFlywheelMotor.velocity)
    }
    internal fun roundToNearest20(velocity: Double): Double {
        // Divide by 20, round to the nearest integer, then multiply by 20
        return round(velocity / 20.0) * 20.0
    }
}

internal enum class FlywheelState {
    AUTO_AIM,
    MANUAL,
    IDLE,
    STOPPED;
}