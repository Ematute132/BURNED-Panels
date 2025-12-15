package org.firstinspires.ftc.teamcode.ILT.Next.Subsystem

import com.bylazar.configurables.annotations.Configurable
import dev.nextftc.core.commands.utility.InstantCommand
import dev.nextftc.core.subsystems.Subsystem
import dev.nextftc.hardware.controllable.MotorGroup
import dev.nextftc.hardware.impl.MotorEx
import org.firstinspires.ftc.teamcode.ILT.Next.Subsystem.Outtake.Shooter.FlyWheel
import kotlin.math.abs

@Configurable
object Intake : Subsystem {
    val iMR = MotorEx("iMR")
    val iML = MotorEx("iML")
    val iM = MotorGroup(iML, iMR)

    // Driver-requested intake power (what you WANT the intake to do)
    @JvmField
    var requestedPower = 0.0

    // How close (in ticks/s) the flywheel velocity must be to target to allow feeding
    @JvmField
    var velocityTolerance = 100.0

    override fun periodic() {
        // Compute whether flywheel is ready
        val ready = flywheelReady()

        // If flywheel is ready, actually run the requested power
        // If not ready, hold intake still at 0
        val actualPower = if (ready) requestedPower else 0.0
        iM.power = actualPower
    }

    /**
     * Returns true if the flywheel is on and its measured velocity
     * is within velocityTolerance of the targetVelocity.
     * Uses FlyWheel.motorRpm converted back to ticks/s.
     */
    fun flywheelReady(): Boolean {
        if (!FlyWheel.flywheelsOn) return false
        // motorRpm = ticks/s * 60 / 28  =>  ticks/s = motorRpm * 28 / 60
        val measuredVel = FlyWheel.motorRpm * 28.0 / 60.0
        return abs(measuredVel - FlyWheel.targetVelocity) <= velocityTolerance
    }

    // Commands now set requestedPower instead of iM.power directly

    val runIntake = InstantCommand {
        requestedPower = 1.0
    }

    val reverseIntake = InstantCommand {
        requestedPower = -1.0
    }

    val reverseIntakeSlow = InstantCommand {
        requestedPower = -0.5
    }

    val reverseIntakeVerySlow = InstantCommand {
        requestedPower = -0.2
    }

    val stopIntake = InstantCommand {
        requestedPower = 0.0
    }
}

fun indexing() {
    // this is where the color sorting based off the color sensor will go
}
