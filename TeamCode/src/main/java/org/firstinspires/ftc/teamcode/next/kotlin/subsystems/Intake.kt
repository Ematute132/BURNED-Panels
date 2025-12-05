package org.firstinspires.ftc.teamcode.next.kotlin.subsystems

import dev.nextftc.core.commands.utility.InstantCommand
import dev.nextftc.core.subsystems.Subsystem
import dev.nextftc.hardware.controllable.MotorGroup
import dev.nextftc.hardware.impl.MotorEx

object Intake : Subsystem{
    val iMR = MotorEx("intakeR").reversed()
    val iML = MotorEx("intakeL").reversed()
    val iM = MotorGroup(iMR,iML)

    var iP = 0.0

    override fun periodic() {
        iM.power = iP
    }

    val runIntake = InstantCommand {
        iP = 1.0 // Some constant
    }

    val reverseIntake = InstantCommand {
        iP = -1.0
    }

    val reverseIntakeSlow = InstantCommand {
        iP = -0.75
    }

    val reverseIntakeVerySlow = InstantCommand {
        iP = -0.2
    }


    val stopIntake = InstantCommand {
        iP = 0.0
    }
}