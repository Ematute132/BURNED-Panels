package org.firstinspires.ftc.teamcode.ILT.Next.Subsystems.Shooter

import com.bylazar.configurables.annotations.Configurable
import com.bylazar.telemetry.PanelsTelemetry
import com.qualcomm.robotcore.util.ElapsedTime
import dev.nextftc.control.ControlSystem
import dev.nextftc.control.KineticState
import dev.nextftc.control.builder.controlSystem
import dev.nextftc.control.feedback.PIDCoefficients
import dev.nextftc.control.feedforward.BasicFeedforwardParameters
import dev.nextftc.core.commands.Command
import dev.nextftc.core.subsystems.Subsystem
import dev.nextftc.hardware.controllable.RunToState
import dev.nextftc.hardware.controllable.RunToVelocity
import dev.nextftc.hardware.impl.MotorEx
import java.util.function.Supplier

@Configurable
object FlyWheel : Subsystem {
    private val motor1 = MotorEx("Fly1").floatMode()
    private val motor2 = MotorEx("Fly2").floatMode()

    @JvmField var ffCoefficients = BasicFeedforwardParameters(0.0, 0.0, 0.75);
    @JvmField var pidCoefficients = PIDCoefficients(0.016, 0.0, 0.0)

    private val controller: ControlSystem = controlSystem {
        basicFF(ffCoefficients)
        velPid(pidCoefficients)
    };

    fun setMotorPowers(power: Double) {
        val clampedPower = power.coerceIn(0.0, 1.0)
        motor1.power = clampedPower
        motor2.power = clampedPower

    }

    class On(speed: Double) : RunToState(controller, KineticState(0.0, speed, 0.0));
    var off = RunToVelocity(controller, 0.0).requires(this).named("FlywheelOff").setInterruptible(true);

    class Manual(
        private val shooterPower: Supplier<Double>
    ) : Command() {
        override val isDone = false;

        override fun update() {
            setMotorPowers(shooterPower.get())
        }
    }

     /*class AutoAim(
        private val dxy: Double,
        private val powerByDistance: (Double) -> Double,  // get by running curve of best fit on collected data
    ) : Command() {
        override val isDone = true;

        init {
            requires(FlyWheel)
        }

        override fun start() {
            controller.goal = KineticState(velocity=powerByDistance(dxy));
        }
    }

      */

    var lastPos = 0.0;
    var elapsedTime: ElapsedTime = ElapsedTime();
    override fun periodic() {
        val power = controller.calculate(
            motor1.state.times(-1.0)
        ).coerceIn(0.0, 1.0);
        setMotorPowers(power);

        val measuredVel = (motor2.currentPosition - lastPos)/elapsedTime.time();
        lastPos = motor2.currentPosition;
        elapsedTime.reset()

        PanelsTelemetry.telemetry.addData("power", power)

        PanelsTelemetry.telemetry.addData("vel measured", measuredVel)
        PanelsTelemetry.telemetry.addData("vel est", controller.lastMeasurement.velocity)
        PanelsTelemetry.telemetry.addData("vel ref", controller.reference.velocity)
        PanelsTelemetry.telemetry.addData("vel goal", controller.goal.velocity)

//        telemetry.addData("pos measured 1", motor1.currentPosition)
        PanelsTelemetry.telemetry.addData("pos measured 2", motor2.currentPosition)
        PanelsTelemetry.telemetry.addData("pos est", -controller.lastMeasurement.position)
        PanelsTelemetry.telemetry.addData("pos ref", controller.reference.position)
    }
}