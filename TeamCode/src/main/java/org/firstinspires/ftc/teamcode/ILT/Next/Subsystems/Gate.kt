package org.firstinspires.ftc.teamcode.ILT.Next.Subsystems

import dev.nextftc.core.commands.utility.InstantCommand
import dev.nextftc.core.subsystems.Subsystem
import dev.nextftc.hardware.impl.ServoEx



/**
 * Gate subsystem for controlling ball flow to shooter.
 */
object Gate : Subsystem {

    private var servo = ServoEx("gate", 0.01)
    private var position = 0.0



    override fun periodic() {

        servo.position = position
    }



    // ==================== COMMANDS ====================

    val open = InstantCommand {
        position = 1.0
    }

    val close = InstantCommand {
        position = 0.0
    }
}