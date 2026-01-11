package org.firstinspires.ftc.teamcode.next.tuning

import com.bylazar.telemetry.JoinedTelemetry
import com.bylazar.telemetry.PanelsTelemetry
import com.pedropathing.geometry.Pose
import com.qualcomm.robotcore.eventloop.opmode.Disabled
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import dev.nextftc.core.components.BindingsComponent
import dev.nextftc.core.components.SubsystemComponent
import dev.nextftc.extensions.pedro.PedroComponent
import dev.nextftc.extensions.pedro.PedroComponent.Companion.follower
import dev.nextftc.ftc.Gamepads
import dev.nextftc.ftc.NextFTCOpMode
import dev.nextftc.ftc.components.BulkReadComponent
import org.firstinspires.ftc.teamcode.ILT.Next.Subsystem.DriveTrain
import org.firstinspires.ftc.teamcode.ILT.Next.Subsystem.Intake
import org.firstinspires.ftc.teamcode.ILT.Next.Subsystem.Outtake.ImprovedOuttake
import org.firstinspires.ftc.teamcode.ILT.Next.Subsystem.Outtake.Shooter.FlyWheel
import org.firstinspires.ftc.teamcode.ILT.Next.Subsystem.Outtake.Shooter.Hood
import org.firstinspires.ftc.teamcode.ILT.Next.Subsystem.Outtake.Shooter.Turret
import org.firstinspires.ftc.teamcode.ILT.Next.Subsystem.Outtake.Shooter.Turret.turretController
import org.firstinspires.ftc.teamcode.pedroPathing.Constants

@TeleOp
class FlywheelTuning : NextFTCOpMode() {

    private val tele = JoinedTelemetry(PanelsTelemetry.ftcTelemetry, telemetry)

    init {
        addComponents(
            SubsystemComponent(Intake, ImprovedOuttake),
            PedroComponent(Constants::createFollower),
            BulkReadComponent,
            BindingsComponent,
        )
    }

    override fun onInit() {
        follower.setStartingPose(Pose(144 - 36.0, 6.5, Math.PI / 2))

        // Make sure there is a nonzero target and gains while tuning
        FlyWheel.targetVelocity = 1500.0      // example units (ticks/s or rad/s – match your system)

    }

    override fun onStartButtonPressed() {
        // Make sure these are Commands that flip flywheelsOn / targetOnVelo
        Gamepads.gamepad1.x whenBecomesTrue FlyWheel.spin
        Gamepads.gamepad1.y whenBecomesTrue FlyWheel.stop
        Gamepads.gamepad1.cross whenBecomesTrue FlyWheel.backOut
        Gamepads.gamepad1.dpadUp whenBecomesTrue Turret.zeroMotor

        Gamepads.gamepad2.rightBumper whenBecomesTrue Turret.spinGearRight whenBecomesFalse Turret.stopGear
        Gamepads.gamepad2.leftBumper  whenBecomesTrue Turret.spinGearLeft  whenBecomesFalse Turret.stopGear
    }

    override fun onUpdate() {
        tele.run {
            addData("current X", DriveTrain.currentX)
            addData("current Y", DriveTrain.currentY)
            addData("current H", DriveTrain.currentHeading)

            addData("f1 power", FlyWheel.f1.power)
            addData("f1 vel", FlyWheel.f1.velocity)
            addData("f2 vel", FlyWheel.f2.velocity)

            addData("kinetic state", FlyWheel.f1.state)
            addData("controller", FlyWheel.flywheelController)
            addData("ctrl output", FlyWheel.flywheelController.calculate(FlyWheel.f1.state))

            addData("target vel", FlyWheel.targetVelocity)

            addData("gear pos", Turret.gP)
            addData("intake pos", Intake.iM.power)

            addData("spin power", Turret.turret.power)
            addData("spin vel", Turret.turret.velocity)
            addData("spin pos", Turret.turret.currentPosition)
            addData("target", Turret.deltaHeading2)
            addData("target x", ImprovedOuttake.goalX)
            addData("target y", ImprovedOuttake.goalY)
            addData("goal",  turretController.goal)
            addData("DistM", ImprovedOuttake.distM)
            addData("DistLL", ImprovedOuttake.distLL)
            addData("flap pos", Hood.hP)
            addData("test", "true")
            addData("distance Difference", ImprovedOuttake.distDiff)

            update()
        }
    }
}
