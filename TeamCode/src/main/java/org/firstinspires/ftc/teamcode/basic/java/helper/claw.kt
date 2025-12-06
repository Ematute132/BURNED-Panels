package org.firstinspires.ftc.teamcode.basic.java.helper

import dev.nextftc.core.subsystems.Subsystem
import dev.nextftc.hardware.impl.ServoEx
import dev.nextftc.hardware.positionable.SetPosition

object claw : Subsystem{

    private val servo = ServoEx("claw-Servo")

    val open = SetPosition(servo,0.1).requires(this)
    val closed = SetPosition(servo,0.2).requires(this)
}