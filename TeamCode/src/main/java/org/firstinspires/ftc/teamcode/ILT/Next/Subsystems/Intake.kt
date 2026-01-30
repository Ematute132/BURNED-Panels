package org.firstinspires.ftc.teamcode.ILT.Next.Subsystems

import dev.nextftc.core.commands.utility.InstantCommand
import dev.nextftc.core.subsystems.Subsystem
import dev.nextftc.hardware.impl.MotorEx


object Intake : Subsystem{

    private var intakeMotor = MotorEx("Intake")
    private var power = 0.0
    private var isInitialized = false
    var intakeState: IntakeState = IntakeState.STOPPED
    enum class IntakeState {
        STOPPED,
        INTAKING,
        EJECTING,
        FEEDING
    }


    override fun periodic() {

        intakeMotor.power = power

    }

    /**
     * Set intake power directly.
     */
    fun setPower(newPower: Double) {
        power = newPower
    }

    /**
     * Check if intake is currently running.
     */

    // ==================== COMMANDS ====================

    val run = InstantCommand {
        power = 1.0
        intakeState = IntakeState.INTAKING
    }

    val reverse = InstantCommand {
        power = -1.0
        intakeState = IntakeState.EJECTING
    }

    val reverseSlow = InstantCommand {
        power = -0.5
        intakeState = IntakeState.EJECTING
    }

    val feed = InstantCommand {
        power = 1.0
       intakeState = IntakeState.FEEDING
    }

    val stop = InstantCommand {
        power = 0.0
        intakeState = IntakeState.STOPPED
    }
}