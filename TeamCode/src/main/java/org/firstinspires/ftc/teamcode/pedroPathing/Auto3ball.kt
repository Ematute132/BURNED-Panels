package org.firstinspires.ftc.teamcode.pedroPathing

import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import dev.nextftc.core.commands.Command
import dev.nextftc.core.commands.delays.Delay
import dev.nextftc.core.commands.groups.ParallelGroup
import dev.nextftc.core.commands.groups.ParallelRaceGroup
import dev.nextftc.core.commands.groups.SequentialGroup
import dev.nextftc.core.commands.utility.InstantCommand
import dev.nextftc.core.components.BindingsComponent
import dev.nextftc.core.components.SubsystemComponent
import dev.nextftc.extensions.pedro.PedroComponent
import dev.nextftc.ftc.NextFTCOpMode
import dev.nextftc.ftc.components.BulkReadComponent
import dev.nextftc.hardware.controllable.MotorGroup
import dev.nextftc.hardware.impl.MotorEx
import org.firstinspires.ftc.teamcode.next.kotlin.subsystems.Intake
import org.firstinspires.ftc.teamcode.next.kotlin.subsystems.Outtake
import kotlin.time.Duration.Companion.seconds

@Autonomous
class Auto3ball : NextFTCOpMode() {
    init{
        addComponents(
            SubsystemComponent(Outtake, Intake),
            PedroComponent(Constants::createFollower),
            BulkReadComponent,
            BindingsComponent
        )
    }
    var fl = MotorEx("fl")
    var fR = MotorEx("fr")
    var bR = MotorEx("br")
    var bL = MotorEx("bl")
    val drive = MotorGroup(fl,fR,bR,bL)

    private val prepToShootFT: Command
        get() = SequentialGroup(
            Intake.reverseIntake,
            Delay(1.seconds),
            ParallelGroup(
                Outtake.flywheelOn,
                Delay(1.seconds),
                Intake.runIntake
            ),
            Delay(0.5.seconds),
            ParallelGroup(
                Intake.stopIntake,
                Outtake.flywheelOff

            )
        )
    private val preIntakeLine : Command
        get() = SequentialGroup(
            Intake.runIntake,
            Delay(3.seconds),
            Intake.reverseIntake,
            Delay(1.seconds),
            ParallelGroup(
                Intake.stopIntake,
                Outtake.flywheelOn
            )
        )
    private val Shoot : Command
        get()= SequentialGroup(
            Intake.runIntake,
            Delay(5.seconds),
            ParallelGroup(
                Intake.stopIntake,
                Outtake.flywheelOff
            )
        )

    override fun onStartButtonPressed() {
        SequentialGroup(
        InstantCommand {
            drive.power = 0.5
        },
        Delay(0.1.seconds),
        InstantCommand {
            drive.power = 0.0
        },
            prepToShootFT




        )
    }


}