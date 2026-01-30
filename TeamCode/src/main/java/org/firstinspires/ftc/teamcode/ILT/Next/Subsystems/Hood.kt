package org.firstinspires.ftc.teamcode.ILT.Next.Subsystems

import dev.nextftc.core.commands.utility.InstantCommand
import dev.nextftc.core.subsystems.Subsystem
import dev.nextftc.hardware.impl.ServoEx



/**
 * Gate subsystem for controlling ball flow to shooter.
 */
object Hood : Subsystem {

    private var servo = ServoEx("gate")
    private var position = 0.0



    override fun periodic() {

        servo.position = position
    }

    /**
     * Set gate position (0.0 = open, 1.0 = closed, typically).
     */
    fun setPosition(newPosition: Double) {
        position = newPosition.coerceIn(0.0, 1.0)
    }

    /**
     * Check if gate is open.
     */


    // ==================== COMMANDS ====================

    val open = InstantCommand {
        position = 1.0
    }

    val close = InstantCommand {
        position = 0.0
    }
}