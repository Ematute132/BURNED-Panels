package org.firstinspires.ftc.teamcode.ILT.Next.Subsystems.Shooter

import dev.nextftc.core.commands.utility.InstantCommand
import dev.nextftc.core.subsystems.Subsystem
import dev.nextftc.hardware.impl.ServoEx
import kotlin.math.PI

/**
 * Gate subsystem for controlling ball flow to shooter.
 */
object Hood : Subsystem {

    private var servo = ServoEx("hood", 0.01)
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

    val full = InstantCommand {
        position = 1.0
    }

    val close = InstantCommand {
        position = 0.0
    }
    val half = InstantCommand{
        position = 0.5
    }
    val open = InstantCommand{
        position = 0.75
    }
}