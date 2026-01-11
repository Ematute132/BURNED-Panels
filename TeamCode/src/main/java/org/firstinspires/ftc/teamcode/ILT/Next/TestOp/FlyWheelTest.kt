package org.firstinspires.ftc.teamcode.ILT.Next.TestOp

import com.bylazar.telemetry.JoinedTelemetry
import com.bylazar.telemetry.PanelsTelemetry
import com.pedropathing.geometry.Pose
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import dev.nextftc.core.components.BindingsComponent
import dev.nextftc.core.components.SubsystemComponent
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


@TeleOp(name = "Flywheel Test")

class FlywheelTest: NextFTCOpMode() {
    init {
        addComponents(SubsystemComponent(ImprovedOuttake),
            BindingsComponent,
            BulkReadComponent, )
    }
    var tele = JoinedTelemetry(PanelsTelemetry.ftcTelemetry, telemetry)


    override fun onInit() {
        follower.setStartingPose(Pose(144 - 36.0, 6.5, Math.PI / 2))
    }

    override fun onStartButtonPressed() {
        Gamepads.gamepad1.x whenBecomesTrue FlyWheel.spin
        Gamepads.gamepad1.y whenBecomesTrue FlyWheel.stop
        Gamepads.gamepad1.cross whenBecomesTrue FlyWheel.Shoot
        Gamepads.gamepad1.dpadUp whenBecomesTrue Turret.zeroMotor
        Gamepads.gamepad2.rightBumper whenBecomesTrue  Turret.spinGearRight whenBecomesFalse Turret.stopGear
        Gamepads.gamepad2.leftBumper whenBecomesTrue Turret.spinGearLeft whenBecomesFalse Turret.stopGear

    }
    override fun onUpdate() {
 tele.run {
    addData("current X", DriveTrain.currentX)
    addData("current y", DriveTrain.currentY)
    addData("current H", DriveTrain.currentHeading)
    addData("f1P", FlyWheel.f1.power)
    addData("f1V", FlyWheel.f1.velocity)
    addData("f2V", FlyWheel.f2.velocity)
    addData("kinetic state", FlyWheel.f1.state)
    addData("controller", FlyWheel.flywheelController)
    addData("controller value", FlyWheel.flywheelController.calculate(FlyWheel.f1.state))
    addData("targetV", FlyWheel.targetVelocity)
    addData("gear pos", Turret.gP)
    addData("iP", Intake.iP)
    addData("spin power", Turret.turret.power)
    addData("spin velo", Turret.turret.velocity)
    addData("spijn pos", Turret.turret.currentPosition)
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

