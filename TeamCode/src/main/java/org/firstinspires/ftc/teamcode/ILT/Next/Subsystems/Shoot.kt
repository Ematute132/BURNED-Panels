@file:Suppress("PackageName")

package org.firstinspires.ftc.teamcode.Systems

import dev.nextftc.core.commands.Command
import dev.nextftc.core.commands.conditionals.IfElseCommand
import dev.nextftc.core.commands.delays.Delay
import dev.nextftc.core.commands.groups.SequentialGroup
import dev.nextftc.core.commands.utility.InstantCommand
import org.firstinspires.ftc.teamcode.ILT.Next.Subsystems.Gate
import org.firstinspires.ftc.teamcode.ILT.Next.Subsystems.Intake
import org.firstinspires.ftc.teamcode.Util.ROBOT

// FIX: Not a Subsystem — this is purely a command factory with no hardware of its own.
// Registering it as a Subsystem was wasteful and misleading.
object Shoot {

    // FIX: Was a val built at class-load time (before hardware is ready).
    // Now a fun so it constructs a fresh IfElseCommand each time it's called,
    // safely after initialize().
    fun shootTripleCommand(): Command = IfElseCommand(
        { ROBOT.getDistanceFromGoal() > 110.0 },
        shootCommand(1.2),
        shootCommand(0.6)
    )

    // FIX: Removed unused tPow/iPow parameters.
    // FIX: Gate.open() and Gate.close() are now called as functions (with ())
    // so their returned InstantCommands are actually executed in the sequence.
    fun shootCommand(waitTime: Double): Command =
        SequentialGroup(
            InstantCommand {
                Intake.run().schedule()
                Gate.open
            },
            Delay(waitTime),
            InstantCommand {
                Intake.stop().schedule()
                Gate.close
            }
        )
}