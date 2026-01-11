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
    // Here I declare two intake motors (left and right) and group them together.
    // The MotorGroup makes it easy to control both at once.
    val iMR = MotorEx("iMR")
    val iML = MotorEx("iML")
    val iM = MotorGroup(iML, iMR)

    // Driver-requested intake power (what you WANT the intake to do)
    // iP is the power level we want to apply — positive sucks balls in, negative ejects them.
    @JvmField
    var iP = 0.0


    override fun periodic() {
        // Every loop, I just apply whatever power level the driver requested.
        // It’s super simple — no PID or fancy control, just direct motor power.
        iM.power = iP
    }

    // These InstantCommands are designed to be bound to gamepad buttons.
    // They’re one-press actions to quickly change intake behavior.

    val runIntake = InstantCommand {
        iP = 1.0 // Full power to suck balls into the robot.
    }

    val reverseIntake = InstantCommand {
        iP = -1.0 // Full power ejection (for clearing jams or dumping).
    }

    val reverseIntakeSlow = InstantCommand {
        iP = -0.5 // Half power ejection (gentler clearing).
    }

    val reverseIntakeVerySlow = InstantCommand {
        iP = -0.2 // Very gentle ejection (fine control or testing).
    }

    val stopIntake = InstantCommand {
        iP = 0.0 // Stop everything (safety or between actions).
    }
}

fun indexing() {
    // this is where the color sorting based off the color sensor will go
    //students can do this and look at artifacts and ll for motif
    // This is a placeholder for a student task: using a color sensor to sort balls.
    // For example, they could detect ring colors or use Limelight motif data to decide which balls to keep.
    // Right now it’s just a comment so the team knows what goes here.
}
