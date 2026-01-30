package org.firstinspires.ftc.teamcode.ILT.Next

import com.bylazar.telemetry.JoinedTelemetry

import com.bylazar.telemetry.PanelsTelemetry
import com.pedropathing.geometry.Pose
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import dev.nextftc.core.commands.groups.ParallelGroup
import dev.nextftc.core.commands.utility.InstantCommand
import dev.nextftc.core.components.BindingsComponent
import dev.nextftc.core.components.SubsystemComponent
import dev.nextftc.extensions.pedro.PedroComponent
import dev.nextftc.extensions.pedro.PedroComponent.Companion.follower
import dev.nextftc.extensions.pedro.PedroDriverControlled
import dev.nextftc.ftc.Gamepads
import dev.nextftc.ftc.NextFTCOpMode
import dev.nextftc.ftc.components.BulkReadComponent
import dev.nextftc.hardware.driving.Drivetrain
import org.firstinspires.ftc.teamcode.ILT.Next.Data.Alliance
import org.firstinspires.ftc.teamcode.ILT.Next.Subsystems.Shooter.Turret
import org.firstinspires.ftc.teamcode.ILT.Next.Subsystems.Drive.currentHeading
import org.firstinspires.ftc.teamcode.ILT.Next.Subsystems.Drive.currentX
import org.firstinspires.ftc.teamcode.ILT.Next.Subsystems.Drive.currentY
import org.firstinspires.ftc.teamcode.ILT.Next.Subsystems.Gate
import org.firstinspires.ftc.teamcode.ILT.Next.Subsystems.Intake
import org.firstinspires.ftc.teamcode.ILT.Next.Subsystems.Shooter.FlyWheel
import org.firstinspires.ftc.teamcode.ILT.Next.Subsystems.Shooter.Hood

import org.firstinspires.ftc.teamcode.pedroPathing.Constants

@TeleOp(name = "Main TeleOp", group = "Competition")
class MainTeleOp : NextFTCOpMode() {

    private val panelsTelemetry = PanelsTelemetry.ftcTelemetry
    private val joinedTelemetry = JoinedTelemetry(telemetry, panelsTelemetry)
    private enum class AimModeTele { OFF, ODO }
    private var currentMode = AimModeTele.OFF
    init {
        addComponents(
            PedroComponent(Constants::createFollower),
            SubsystemComponent(
                FlyWheel, Drivetrain, Hood, Gate, Intake, Turret
            ),
            BulkReadComponent, BindingsComponent
        )
    }


    override fun onInit() {
        Alliance.BLUE
        follower.pose = Pose(72.0, 72.0, 0.0)
    }

    override fun onStartButtonPressed() {
       PedroDriverControlled(
            -Gamepads.gamepad1.leftStickY,
            -Gamepads.gamepad1.leftStickX,
            -Gamepads.gamepad1.rightStickX,
            true  // false = field centric, true = robot centric
        ).schedule()
        bindControls()
    }

    private fun bindControls() {
        // --- DRIVER (GP1) ---
        Gamepads.gamepad1.leftTrigger.greaterThan(0.5) whenBecomesTrue(Intake.run) whenBecomesFalse(Intake.stop)
        Gamepads.gamepad1.leftBumper whenBecomesTrue(Intake.reverse) whenBecomesFalse(Intake.stop)

        Gamepads.gamepad1.rightBumper whenBecomesTrue Gate.open whenBecomesFalse Gate.close
        Gamepads.gamepad1.rightTrigger greaterThan(0.5) whenBecomesTrue {  }


        Gamepads.gamepad1.circle whenBecomesTrue {reset}
        Gamepads.gamepad1.dpadUp whenBecomesTrue Hood.open
        Gamepads.gamepad1.dpadLeft whenBecomesTrue Hood.half
        Gamepads.gamepad1.dpadDown whenBecomesTrue Hood.close

        Gamepads.gamepad1.square whenBecomesTrue { currentMode = AimModeTele.ODO }
        Gamepads.gamepad1.triangle whenBecomesTrue {currentMode = AimModeTele.OFF}



    }

    override fun onUpdate() {
       currentX = follower.pose.x
        currentY = follower.pose.y
        currentHeading = follower.pose.heading

        when (currentMode) {
            AimModeTele.OFF -> Turret.stop()
            AimModeTele.ODO -> Turret.aimWithOdometryOnly()
        }



    }
    val reset = InstantCommand{
        ParallelGroup(
            Hood.close,
            Gate.close,
            FlyWheel.off,
            Intake.stop,

        )
    }
}