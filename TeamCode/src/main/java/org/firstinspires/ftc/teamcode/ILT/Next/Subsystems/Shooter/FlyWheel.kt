package org.firstinspires.ftc.teamcode.ILT.Next.Subsystems.Shooter

import com.bylazar.configurables.annotations.Configurable
import com.bylazar.telemetry.PanelsTelemetry
import dev.nextftc.control.ControlSystem
import dev.nextftc.control.KineticState
import dev.nextftc.control.builder.controlSystem
import dev.nextftc.control.feedback.PIDCoefficients
import dev.nextftc.control.feedforward.BasicFeedforwardParameters
import dev.nextftc.core.commands.Command
import dev.nextftc.core.commands.groups.ParallelGroup
import dev.nextftc.core.commands.utility.InstantCommand
import dev.nextftc.core.subsystems.Subsystem
import dev.nextftc.hardware.controllable.RunToVelocity
import dev.nextftc.hardware.impl.MotorEx
import java.util.function.Supplier

@Configurable
object FlyWheel : Subsystem {
    // 1. Hardware Definition for 2 Motors
    private val motor1 = MotorEx("Fly1").floatMode()
    private val motor2 = MotorEx("Fly2").floatMode()

    @JvmField var ffCoefficients = BasicFeedforwardParameters(0.003, 0.08, 0.0) // Combined from your example
    @JvmField var pidCoefficients = PIDCoefficients(0.008, 0.0, 0.01)

    // 2. Control System
    val controller: ControlSystem = controlSystem {
        basicFF(ffCoefficients)
        velPid(pidCoefficients)
    }

    // 3. Command Definitions
    fun setVelocity(speed: Double) {
        controller.goal = KineticState(0.0,speed)
    }

    // Velocity presets based on your OpMode example
    val off = RunToVelocity(controller, 0.0).requires(this).named("FlywheelOff")
    fun runHigh() = setVelocity(2000.0)
    fun runMid() = setVelocity(1000.0)

    private fun setMotorPowers(power: Double) {
        val clampedPower = power.coerceIn(-0.85, 0.85)
        motor1.power = clampedPower
        motor2.power = clampedPower
    }

    override fun periodic() {
        // Calculate power based on current motor state
        // We use motor1 as the primary feedback source
        val power = controller.calculate(motor1.state)

        setMotorPowers(power)

        // Telemetry for debugging sync and performance
        PanelsTelemetry.telemetry.addData("Flywheel Power", power)
        PanelsTelemetry.telemetry.addData("Target Vel", controller.goal.velocity)
        PanelsTelemetry.telemetry.addData("Actual Vel", motor1.velocity)
        PanelsTelemetry.telemetry.addData("Motor 2 Vel", motor2.velocity)
    }

    // Manual Override Command
    class Manual(private val shooterPower: Supplier<Double>) : Command() {
        override val isDone = false
        init { requires(FlyWheel) }
        override fun update() {
            FlyWheel.setMotorPowers(shooterPower.get())
        }
    }
    val maxShoot = InstantCommand{
        controller.goal =
            KineticState(0.0, 1500.0)

    }
    val midShoot = InstantCommand{
        controller.goal =
            KineticState(0.0, 1250.0)

    }
    val closeShoot = InstantCommand{
        controller.goal =
            KineticState(0.0, 1000.0)

    }
}