package org.firstinspires.ftc.teamcode.ILT.Next.Subsystems

import dev.nextftc.core.commands.utility.InstantCommand
import dev.nextftc.core.subsystems.Subsystem
import dev.nextftc.hardware.impl.MotorEx

object Intake : Subsystem {

    // FIX: Use lateinit so the motor is only constructed during initialize(),
    // when the hardware map is actually available. Declaring it as a val at
    // object-level causes MotorEx("intake") to run at class-load time — before
    // any op mode has registered a hardware map — producing a broken motor reference.
    private lateinit var intakeMotor: MotorEx

    private var power = 0.0
    var intakeState: IntakeState = IntakeState.STOPPED

    enum class IntakeState {
        STOPPED,
        INTAKING,
        EJECTING,
        FEEDING
    }

    // FIX: initialize() is where hardware should be constructed in NextFTC subsystems.
    override fun initialize() {
        intakeMotor = MotorEx("intake")
        power = 0.0
        intakeState = IntakeState.STOPPED
    }

    override fun periodic() {
        intakeMotor.power = power
    }

    fun setPower(newPower: Double) {
        power = newPower
    }

    // ==================== COMMANDS ====================
    // FIX: Use fun instead of val so each call returns a fresh InstantCommand.
    // val commands are created at object-init time (before initialize()), so they
    // close over whatever state exists then. fun ensures the command is built
    // fresh when scheduled, using the correct post-init state.

    fun run() = InstantCommand {
        setPower(0.9)
        intakeState = IntakeState.INTAKING
    }

    fun reverse() = InstantCommand {
        setPower(-1.0)
        intakeState = IntakeState.EJECTING
    }

    fun reverseSlow() = InstantCommand {
        setPower(-0.5)
        intakeState = IntakeState.EJECTING
    }

    fun feed() = InstantCommand {
        setPower(1.0)
        intakeState = IntakeState.FEEDING
    }

    fun stop() = InstantCommand {
        setPower(0.0)
        intakeState = IntakeState.STOPPED
    }
}