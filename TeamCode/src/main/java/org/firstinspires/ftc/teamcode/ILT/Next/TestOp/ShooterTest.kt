package org.firstinspires.ftc.teamcode.ILT.Next.TestOp

import com.bylazar.telemetry.PanelsTelemetry
import dev.nextftc.core.commands.CommandManager
import dev.nextftc.core.commands.groups.SequentialGroup
import dev.nextftc.core.commands.utility.InstantCommand
import dev.nextftc.core.components.BindingsComponent
import dev.nextftc.core.components.SubsystemComponent
import dev.nextftc.ftc.Gamepads
import dev.nextftc.ftc.NextFTCOpMode
import dev.nextftc.ftc.components.BulkReadComponent
import dev.nextftc.hardware.controllable.MotorGroup
import dev.nextftc.hardware.impl.MotorEx
import org.firstinspires.ftc.teamcode.ILT.Next.Subsystem.Intake
import org.firstinspires.ftc.teamcode.ILT.Next.Subsystem.Outtake.Shooter.FlyWheel
import org.firstinspires.ftc.teamcode.next.kotlin.subsystems.LLAutoVelo
import org.firstinspires.ftc.teamcode.ILT.Next.Subsystem.Outtake.Outtake

class ShooterTest: NextFTCOpMode() {
    init {
        addComponents(SubsystemComponent(Outtake),
            BindingsComponent,
            BulkReadComponent, )
    }

    private val flyR = MotorEx("FlyRight")
    private val flyL = MotorEx("FlyLeft").reversed()
    private val fly = MotorGroup(flyR,flyL)

    private val LLSpeed = LLAutoVelo.getCalculatedRPM()
    private val targetSpeed = 1546.0

    override fun onStartButtonPressed() {
        // Button A: Limelight-based speed
        Gamepads.gamepad1.a whenBecomesTrue InstantCommand {
            FlyWheel.On(LLSpeed)
        }

        // Button B: fixed target speed
        Gamepads.gamepad1.b whenBecomesTrue InstantCommand {
            FlyWheel.On(targetSpeed)
        }

        // Optional: button X to turn shooter off
        Gamepads.gamepad1.x whenBecomesTrue InstantCommand {
            FlyWheel.off()
        }
    }
    override fun onUpdate() {

    }

    override fun onStop() {
        CommandManager.cancelAll()
    }
}