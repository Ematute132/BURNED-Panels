package org.firstinspires.ftc.teamcode.ILT.Next.Subsystem.Outtake.Shooter

import com.qualcomm.robotcore.util.ElapsedTime
import dev.nextftc.control.KineticState
import dev.nextftc.control.builder.controlSystem
import dev.nextftc.control.feedback.PIDCoefficients
import dev.nextftc.control.feedforward.BasicFeedforwardParameters
import dev.nextftc.core.commands.utility.InstantCommand
import dev.nextftc.core.subsystems.Subsystem
import dev.nextftc.ftc.ActiveOpMode
import dev.nextftc.hardware.controllable.MotorGroup
import dev.nextftc.hardware.controllable.RunToState
import dev.nextftc.hardware.controllable.RunToVelocity
import dev.nextftc.hardware.impl.MotorEx

object FlyWheel : Subsystem {
    private val f1 = MotorEx("flyRight")
    private val f2 = MotorEx("flyLeft")
    private val fly = MotorGroup(f1, f2)

    //change these values when u tune the robot
    // increase the
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

    class On(speed: Double) : RunToState(flywheelController, KineticState(velocity = speed))
    @JvmField var off = RunToVelocity(flywheelController, 0.0).requires(this).named("FlywheelOff").setInterruptible(true);

    var lastPos = 0.0;
    var elapsedTime: ElapsedTime = ElapsedTime();

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
        val measuredVel = (fly.currentPosition - lastPos)/elapsedTime.time();
        lastPos = fly.currentPosition;
        elapsedTime.reset()


        ActiveOpMode.telemetry.addData("targetVelo", targetVelocity)
        ActiveOpMode.telemetry.addData("RPM target", targetVelocity*60.0/28.0)
        ActiveOpMode.telemetry.addData("flywheel goal", flywheelController.goal)


        ActiveOpMode.telemetry.addData("power R", flywheelController.calculate(f1.state))
        ActiveOpMode.telemetry.addData("power L", flywheelController.calculate(f1.state))

        ActiveOpMode.telemetry.addData("vel measured", measuredVel)
        ActiveOpMode.telemetry.addData("vel est", flywheelController.lastMeasurement.velocity)
        ActiveOpMode.telemetry.addData("vel ref", flywheelController.reference.velocity)
        ActiveOpMode.telemetry.addData("vel goal", flywheelController.goal.velocity)

        ActiveOpMode.telemetry.addData("pos measured", f1.currentPosition)
        ActiveOpMode.telemetry.addData("pos measured", f2.currentPosition)
        ActiveOpMode.telemetry.addData("pos est", -flywheelController.lastMeasurement.position)
        ActiveOpMode.telemetry.addData("pos ref", flywheelController.reference.position)


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