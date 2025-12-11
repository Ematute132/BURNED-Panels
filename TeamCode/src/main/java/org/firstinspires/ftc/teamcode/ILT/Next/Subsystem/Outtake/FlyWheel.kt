package org.firstinspires.ftc.teamcode.ILT.Next.Subsystem.Outtake

import dev.nextftc.control.KineticState
import dev.nextftc.control.builder.controlSystem
import dev.nextftc.control.feedback.PIDCoefficients
import dev.nextftc.control.feedforward.BasicFeedforwardParameters
import dev.nextftc.core.commands.utility.InstantCommand
import dev.nextftc.core.subsystems.Subsystem
import dev.nextftc.hardware.impl.MotorEx

object FlyWheel : Subsystem {
    private val f1 = MotorEx("flyRight")
    private val f2 = MotorEx("flyLeft")

    //change these values when u tune the robot
    @JvmField var flywheelPID = PIDCoefficients(0.0, 0.0, 0.0)
    @JvmField var flywheelFF = BasicFeedforwardParameters(0.0, 0.0, 0.0)

    //calling the control system that will control pid and feedforawd correcting.
    private var flywheelController = controlSystem {
        velPid(flywheelPID)
        basicFF(flywheelFF)
    }
    // change target velo to match that of limelight
    @JvmField var targetVelocity = 0.0

    @JvmField var flywheelsOn = false
    // create motorRpm to convert motor velocity to rpm
    var motorRpm: Double = 0.0

    override fun periodic() {
        motorRpm = f1.velocity * 60.0/28.0

        f1.power = flywheelController.calculate(f1.state)
        f2.power = f1.power
        if (flywheelsOn) {
            flywheelController.goal = KineticState(0.0, targetVelocity)
        }
        else {
            flywheelController.goal = KineticState(0.0, 0.0)
        }
        /* telemetry for flywheel
        ActiveOpMode.telemetry.run {
            addData("targetVelo", targetVelocity)
            addData("Current RPM", motorRpm)
            addData("RPM target", targetVelocity*60.0/28.0)
            addData("flywheel goal", flywheelController.goal)
            }
         */

    }


    fun updatePid(velocity:Double) {
        targetVelocity = velocity
    }

    val spin = InstantCommand {
        flywheelsOn = true
    }

    val stop = InstantCommand {
        flywheelsOn = false
    }
    // incase ball gets stuck
    val backOut = InstantCommand {
        stop.schedule()
        f1.power = -0.5
        f2.power = f1.power
    }
}