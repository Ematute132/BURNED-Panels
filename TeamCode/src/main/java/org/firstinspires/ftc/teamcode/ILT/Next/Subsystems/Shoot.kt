@file:Suppress("PackageName")

package org.firstinspires.ftc.teamcode.Systems

import dev.nextftc.core.commands.Command
import dev.nextftc.core.commands.conditionals.IfElseCommand
import dev.nextftc.core.commands.delays.Delay
import dev.nextftc.core.commands.groups.SequentialGroup
import dev.nextftc.core.commands.utility.InstantCommand
import dev.nextftc.core.subsystems.Subsystem
import org.firstinspires.ftc.teamcode.ILT.Next.Subsystems.Gate
import org.firstinspires.ftc.teamcode.ILT.Next.Subsystems.Intake
import org.firstinspires.ftc.teamcode.Util.ROBOT


object Shoot: Subsystem {
    val shootTripleCommand: Command = IfElseCommand(
        { ROBOT.getDistanceFromGoal() > 110.0},
        shootCommand(1.2,0.55,0.55),
        shootCommand(0.6, 1.0, 1.0)
    )

    fun shootCommand(waitTime: Double, tPow: Double, iPow: Double): Command =
        SequentialGroup(
            InstantCommand {
                Intake.run
                Gate.open
            },
            Delay(waitTime),
            InstantCommand {
                Intake.stop
              Gate.close
            }
        )
}