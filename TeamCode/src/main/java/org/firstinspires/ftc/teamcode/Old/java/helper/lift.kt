package org.firstinspires.ftc.teamcode.Old.java.helper

import dev.nextftc.control.builder.controlSystem
import dev.nextftc.core.subsystems.Subsystem
import dev.nextftc.hardware.controllable.RunToPosition
import dev.nextftc.hardware.impl.MotorEx

object lift : Subsystem{

    private val motor = MotorEx("lift-Motor")

    private val controlSystem = controlSystem {
        posPid(0.0,0.0,0.0)
        //what is elevatoroff
        elevatorFF(0.0)
    }

    val toLow = RunToPosition(controlSystem, 0.0).requires(this)
    val toMid = RunToPosition(controlSystem,500.0).requires(this)
    val toHigh = RunToPosition(controlSystem,1200.0).requires(this)

    override fun periodic() {
        motor.power = controlSystem.calculate(motor.state)

    }
}