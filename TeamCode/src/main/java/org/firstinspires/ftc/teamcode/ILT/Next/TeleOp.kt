@file:Suppress("PackageName", "unused")

package org.firstinspires.ftc.teamcode.TeleOp

import com.pedropathing.geometry.Pose
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import dev.nextftc.extensions.pedro.PedroComponent.Companion.follower
import dev.nextftc.extensions.pedro.PedroDriverControlled
import dev.nextftc.ftc.Gamepads
import dev.nextftc.ftc.NextFTCOpMode
import dev.nextftc.hardware.driving.DriverControlledCommand
import org.firstinspires.ftc.teamcode.ILT.Next.Subsystems.Drive
import org.firstinspires.ftc.teamcode.ILT.Next.Subsystems.Intake
import org.firstinspires.ftc.teamcode.Systems.Shoot
import org.firstinspires.ftc.teamcode.Systems.Shooter
import org.firstinspires.ftc.teamcode.Systems.ShooterSubsystems.Flywheel


import org.firstinspires.ftc.teamcode.Systems.ShooterSubsystems.FlywheelState
import org.firstinspires.ftc.teamcode.Util.Alliance
import org.firstinspires.ftc.teamcode.Util.ROBOT
import org.firstinspires.ftc.teamcode.Util.Stage
import org.firstinspires.ftc.teamcode.Util.addSubsystems
import org.firstinspires.ftc.teamcode.Util.includePedro
import org.firstinspires.ftc.teamcode.nextFtc.Subsystem.Shooter.Hood
import org.firstinspires.ftc.teamcode.nextFtc.Subsystem.Shooter.TurretMech.Turret
import org.firstinspires.ftc.teamcode.pedroPathing.Constants

import kotlin.math.PI

@TeleOp(name = "Red TeleOp", group = "TeleOp")
class TeleOpRed: NextFTCOpMode() {
    init {
        addSubsystems(Drive,Flywheel, Turret, Intake, Hood,)
        includePedro(Constants::createFollower)
    }

    val drivetrain: DriverControlledCommand by lazy {
        PedroDriverControlled(
            -Gamepads.gamepad1.leftStickY,
            -Gamepads.gamepad1.leftStickX,
            -Gamepads.gamepad1.rightStickX,
            true
        )
    }

    override fun onStartButtonPressed() {
        ROBOT.currAlliance = Alliance.RED
        ROBOT.currStage = Stage.TELEOP
        ROBOT.currStage.useFlywheelVel = false
        follower.setStartingPose(ROBOT.currTeleOpStartPose)
        drivetrain.schedule()

        Shooter.flywheelState = FlywheelState.AUTO_AIM

        Gamepads.gamepad1.rightTrigger.greaterThan(0.0)
            .whenBecomesTrue { Intake.run }
            .whenBecomesFalse { Intake.stop }
        Gamepads.gamepad1.rightBumper
            .whenBecomesTrue (Shoot.shootTripleCommand )
        Gamepads.gamepad1.leftBumper
            .toggleOnBecomesTrue()
            .whenBecomesTrue { drivetrain.scalar = 0.2 }
            .whenBecomesFalse { drivetrain.scalar = 1.0 }
        Gamepads.gamepad1.circle.or(Gamepads.gamepad2.rightBumper)
            .whenBecomesTrue {
                if (Shooter.flywheelState == FlywheelState.AUTO_AIM) { Shooter.flywheelState = FlywheelState.IDLE }
                else { Shooter.flywheelState = FlywheelState.AUTO_AIM }
            }
        Gamepads.gamepad1.dpadUp.or(Gamepads.gamepad2.dpadUp)
            .whenBecomesTrue { Shooter.flywheelState = FlywheelState.STOPPED }
        Gamepads.gamepad1.dpadDown.or(Gamepads.gamepad2.dpadDown)
            .whenBecomesTrue {  Shooter.flywheelState = FlywheelState.AUTO_AIM }
        Gamepads.gamepad2.triangle
            .whenBecomesTrue { follower.pose = ROBOT.currAlliance.resetPoses.resetPose2 }
        Gamepads.gamepad2.square
            .whenBecomesTrue { follower.pose = ROBOT.currAlliance.resetPoses.resetPose3 }
        Gamepads.gamepad1.cross
            .whenBecomesTrue { follower.pose = ROBOT.currAlliance.resetPoses.resetPose1 }
    }

    override fun onUpdate() {
        Shooter.update()
        telemetry.update()
    }
}