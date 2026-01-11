package org.firstinspires.ftc.teamcode.next

import com.bylazar.telemetry.JoinedTelemetry
import com.bylazar.telemetry.PanelsTelemetry
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import dev.nextftc.core.commands.utility.InstantCommand
import dev.nextftc.core.components.BindingsComponent
import dev.nextftc.core.components.SubsystemComponent
import dev.nextftc.extensions.pedro.PedroComponent
import dev.nextftc.ftc.Gamepads
import dev.nextftc.ftc.NextFTCOpMode
import dev.nextftc.ftc.components.BulkReadComponent
import org.firstinspires.ftc.teamcode.ILT.Next.Subsystem.DriveTrain
import org.firstinspires.ftc.teamcode.ILT.Next.Subsystem.Outtake.ImprovedOuttake
import org.firstinspires.ftc.teamcode.ILT.Next.Subsystem.Outtake.Shooter.Turret


import org.firstinspires.ftc.teamcode.pedroPathing.Constants

@TeleOp
class TurretAimTuning: NextFTCOpMode() {
    val tele = JoinedTelemetry(PanelsTelemetry.ftcTelemetry, telemetry)

    init {
        addComponents(
            PedroComponent(Constants::createFollower),
            // gotta update the tuning for the new intake
            SubsystemComponent(ImprovedOuttake, DriveTrain),
            BulkReadComponent,
            BindingsComponent,
        )
    }

    override fun onInit() {
        Gamepads.gamepad1.x whenBecomesTrue InstantCommand { Turret.zeroMotor }
    }

    override fun onUpdate() {
        telemetry.run {
            addData("x", DriveTrain.currentX)
            addData("y", DriveTrain.currentY)
            addData("heading", DriveTrain.currentHeading)
            addData("current yaw", Turret.getYaw())
            addData("goal", Turret.turretController.goal)
            update()
        }
    }
}