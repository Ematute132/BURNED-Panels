package org.firstinspires.ftc.teamcode.ILT.Next

import com.bylazar.telemetry.JoinedTelemetry

import com.bylazar.telemetry.PanelsTelemetry
import com.pedropathing.geometry.Pose
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import dev.nextftc.control.KineticState
import dev.nextftc.core.commands.groups.ParallelGroup
import dev.nextftc.core.commands.groups.SequentialGroup
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
import org.firstinspires.ftc.teamcode.ILT.Next.Subsystems.Drive.poseValid
import org.firstinspires.ftc.teamcode.ILT.Next.Subsystems.Gate
import org.firstinspires.ftc.teamcode.ILT.Next.Subsystems.Intake
import org.firstinspires.ftc.teamcode.ILT.Next.Subsystems.Shooter.FlyWheel
import org.firstinspires.ftc.teamcode.ILT.Next.Subsystems.Shooter.FlyWheel.controller
import org.firstinspires.ftc.teamcode.ILT.Next.Subsystems.Shooter.Hood
import org.firstinspires.ftc.teamcode.ILT.Next.Subsystems.Shooter.Turret.alliance
import org.firstinspires.ftc.teamcode.ILT.Next.Subsystems.Shooter.Turret.goalX
import org.firstinspires.ftc.teamcode.ILT.Next.Subsystems.Shooter.Turret.goalY
import org.firstinspires.ftc.teamcode.ILT.Next.TestOp.TurretAimingTestOpMode.AimMode
import org.firstinspires.ftc.teamcode.Systems.ShooterSubsystems.Flywheel
import org.firstinspires.ftc.teamcode.nextFtc.Subsystem.Shooter.Hood

import org.firstinspires.ftc.teamcode.pedroPathing.Constants

@TeleOp(name = "Main TeleOp- BLUE", group = "Competition")
class BlueTeleOp : NextFTCOpMode() {

    private val panelsTelemetry = PanelsTelemetry.ftcTelemetry
    private val joinedTelemetry = JoinedTelemetry(telemetry, panelsTelemetry)
    private enum class AimModeTele { OFF, ODO }
    private enum class FlyModeTele{IDLE,CLOSE,FAR,MID}
    private var currentMode = AimModeTele.OFF
    init {
        addComponents(
            PedroComponent(Constants::createFollower),
            SubsystemComponent(
                Flywheel, Drivetrain, Hood, Gate, Intake, Turret
            ),
            BulkReadComponent, BindingsComponent
        )
    }


    override fun onInit() {
        follower.pose = Pose(72.0, 72.0, 0.0)
    }

    override fun onStartButtonPressed() {
        PedroDriverControlled(
            Gamepads.gamepad1.leftStickY,
            Gamepads.gamepad1.leftStickX,
            -Gamepads.gamepad1.rightStickX,
            false  // false = field centric, true = robot centric
        ).schedule()
        currentMode = AimModeTele.OFF
        alliance = Alliance.BLUE
        bindControls()
    }

    private fun bindControls() {
        // --- DRIVER (GP1) ---
        Gamepads.gamepad1.leftTrigger.greaterThan(0.5) whenBecomesTrue(Intake.run) whenBecomesFalse(Intake.stop)
        Gamepads.gamepad1.leftBumper whenBecomesTrue(Intake.reverse) whenBecomesFalse(Intake.stop)

        Gamepads.gamepad1.rightBumper whenBecomesTrue Gate.open whenBecomesFalse Gate.close

        //clsoe shooting is 1000.0 hood all the way down
        // mid shooting is 1250 hood inbetween up and half / gotta tune at school
        // far shooting is 1500 hood up all the way/ may have to change velocity down.





        Gamepads.gamepad1.square whenBecomesTrue {FlyWheel.setVelocity(1000.0)}
        Gamepads.gamepad1.triangle whenBecomesTrue {FlyWheel.setVelocity(1300.0)}
        Gamepads.gamepad1.cross whenBecomesTrue { FlyWheel.setVelocity(-500.0) }
        Gamepads.gamepad1.circle whenBecomesTrue { FlyWheel.setVelocity(1500.0) }

        Gamepads.gamepad2.triangle whenBecomesTrue { follower.pose = Pose(144.0,0.0,180.0) }





    }

    override fun onUpdate() {
        poseValid = true
        currentMode = AimModeTele.ODO
        currentX = follower.pose.x
        currentY = follower.pose.y
        currentHeading = follower.pose.heading

        when (currentMode) {
            AimModeTele.OFF -> Turret.stop()
            AimModeTele.ODO -> Turret.aimWithOdometry()
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