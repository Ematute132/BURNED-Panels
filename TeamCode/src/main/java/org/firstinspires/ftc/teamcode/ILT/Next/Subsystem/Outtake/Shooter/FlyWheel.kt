package org.firstinspires.ftc.teamcode.ILT.Next.Subsystem.Outtake.Shooter

import dev.nextftc.control.KineticState
import dev.nextftc.control.builder.controlSystem
import dev.nextftc.control.feedback.PIDCoefficients
import dev.nextftc.control.feedforward.BasicFeedforwardParameters
import dev.nextftc.core.commands.delays.Delay
import dev.nextftc.core.commands.groups.SequentialGroup
import dev.nextftc.core.commands.utility.InstantCommand
import dev.nextftc.core.subsystems.Subsystem
import dev.nextftc.ftc.ActiveOpMode
import dev.nextftc.hardware.impl.MotorEx
import org.firstinspires.ftc.teamcode.ILT.Next.Subsystem.Intake

import kotlin.time.Duration.Companion.seconds

// Flywheel subsystem controls two shooter wheels with combined PID + feedforward velocity control.
object FlyWheel: Subsystem {
    // Primary flywheel motor.
    val f1 = MotorEx("f1M")

    // Secondary flywheel motor, reversed to match mechanical orientation.
     val f2 = MotorEx("f2M").reversed()

    // Velocity PID coefficients (tune for your drivetrain and inertia).
    @JvmField
    var flywheelPID = PIDCoefficients(0.0033, 0.0, 0.0)

    // Basic feedforward parameters: kV (per-tick), kA, kS (static). Tune to reduce error and improve spin-up.
    @JvmField
    var flywheelFF = BasicFeedforwardParameters(1/2400.0, 0.0, 0.03)

    // Controller combining velocity PID and feedforward for stable target tracking.
     var flywheelController = controlSystem {
        velPid(flywheelPID)     // Use velocity PID loop
        basicFF(flywheelFF)     // Add simple feedforward model
    }

    // Desired wheel linear velocity in ticks/sec (controller uses this as goal velocity).
    @JvmField
    var targetVelocity = 0.0


    // On/off state for flywheels; when off, the controller targets zero velocity.
    @JvmField
    var flywheelsOn = false

    // Convenience metric for driver feedback: estimated motor RPM.
    var motorRpm: Double = 0.0

    // Periodic loop: compute RPM, run controller, mirror power to second motor, set goals, and report telemetry.
    override fun periodic() {
        // Convert measured motor velocity (ticks/sec) to RPM; 60 sec/min divided by 28 ticks per motor rev.
        motorRpm = f1.velocity * 60.0 / 28.0

        // Compute power from controller using current measured motor state.
        f1.power = flywheelController.calculate(f1.state)
        // Mirror power to second motor to keep both wheels synchronized.
        f2.power = f1.power

        // Update controller goal based on flywheelsOn flag.
        if (flywheelsOn) {
            // Track the desired target velocity when enabled.
            flywheelController.goal = KineticState(0.0, targetVelocity)
        } else {
            // Stop the wheels by commanding zero velocity.
            flywheelController.goal = KineticState(0.0, 0.0)
        }

        // Driver-station telemetry: show targets and actuals for tuning and match awareness.
        ActiveOpMode.telemetry.run {
            addData("targetVelo", targetVelocity)                 // Controller target (ticks/sec)
            addData("Current RPM", motorRpm)                      // Estimated actual RPM
            addData("RPM target", targetVelocity * 60.0 / 28.0)       // Target expressed in RPM
            addData("flywheel goal", flywheelController.goal)     // Full KineticState goal
        }
    }

    // External API to set a new velocity target (ticks/sec). Does not auto-enable the wheels.
    fun updatePid(velocity: Double) {
        targetVelocity = velocity
    }

    // Command: enable flywheels (controller will pursue targetVelocity).
    val spin = InstantCommand {
        flywheelsOn = true
    }

    // Command: disable flywheels (controller will target zero).
    val stop = InstantCommand {
        flywheelsOn = false
    }

    // Command: briefly reverse to clear jams; schedules stop first, then sets negative power.
    val backOutSlow = InstantCommand {
        stop.schedule()       // Ensure controller is not trying to maintain positive velocity.
        f1.power = -0.5       // Manual reverse power on primary motor.
        f2.power = f1.power   // Mirror reverse power on secondary motor.
    }
    val backOut = InstantCommand {
        stop.schedule()       // Ensure controller is not trying to maintain positive velocity.
        f1.power = -1.0       // Manual reverse power on primary motor.
        f2.power = f1.power   // Mirror reverse power on secondary motor.
    }


    // SequentialCommand to automate shooting sequence
    // might need to change the seconds
    val Shoot = SequentialGroup(
        spin,
        Delay(0.1.seconds),
        Intake.runIntake,
        Delay(0.5.seconds),
        stop,
    )

}