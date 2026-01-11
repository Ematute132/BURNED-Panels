package org.firstinspires.ftc.teamcode.next

import com.bylazar.telemetry.JoinedTelemetry
import com.bylazar.telemetry.PanelsTelemetry
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import dev.nextftc.core.commands.groups.SequentialGroup
import dev.nextftc.core.commands.utility.InstantCommand
import dev.nextftc.core.components.BindingsComponent
import dev.nextftc.core.components.SubsystemComponent
import dev.nextftc.extensions.pedro.PedroComponent
import dev.nextftc.extensions.pedro.PedroComponent.Companion.follower
import dev.nextftc.ftc.Gamepads
import dev.nextftc.ftc.NextFTCOpMode
import dev.nextftc.ftc.components.BulkReadComponent
import org.firstinspires.ftc.teamcode.ILT.Next.Subsystem.Data.Alliance
import org.firstinspires.ftc.teamcode.ILT.Next.Subsystem.DriveTrain
import org.firstinspires.ftc.teamcode.ILT.Next.Subsystem.Intake
import org.firstinspires.ftc.teamcode.ILT.Next.Subsystem.Outtake.ImprovedOuttake
import org.firstinspires.ftc.teamcode.ILT.Next.Subsystem.Outtake.Shooter.FlyWheel
import org.firstinspires.ftc.teamcode.ILT.Next.Subsystem.Outtake.Shooter.Hood
import org.firstinspires.ftc.teamcode.ILT.Next.Subsystem.Outtake.Shooter.Turret
import org.firstinspires.ftc.teamcode.ILT.Next.TestOp.HoodTestOp
import org.firstinspires.ftc.teamcode.pedroPathing.Constants


@TeleOp(name = "First Try")
class TeleOP: NextFTCOpMode() {
    var tele = JoinedTelemetry(PanelsTelemetry.ftcTelemetry, telemetry)

    init {
        addComponents(
            PedroComponent(Constants::createFollower),
            SubsystemComponent(Intake, ImprovedOuttake, DriveTrain),
            BulkReadComponent,
            BindingsComponent,
        )
    }

    override fun onInit() {
        //when (DriveTrain.alliance) {
            // got to add auto
            //Alliance.RED -> follower.setStartingPose(Far12.park)
            //Alliance.BLUE -> follower.setStartingPose(Far12.park.mirror())
       // }
    }



    override fun onStartButtonPressed() {
        // Intake Controls
        Gamepads.gamepad1.rightTrigger.greaterThan(0.3) whenBecomesTrue Intake.runIntake whenBecomesFalse Intake.stopIntake
        Gamepads.gamepad1.leftTrigger.greaterThan(0.3) whenBecomesTrue Intake.reverseIntake whenBecomesFalse Intake.stopIntake

        // Gamepad 2
        Gamepads.gamepad2.rightTrigger.greaterThan(0.3) whenBecomesTrue ImprovedOuttake.aimUp
        Gamepads.gamepad2.leftTrigger.greaterThan(0.3) whenBecomesTrue ImprovedOuttake.aimDown

        // Flywheel Controls
        Gamepads.gamepad2.a whenBecomesTrue SequentialGroup(InstantCommand{ ImprovedOuttake.canSpin = true},
            FlyWheel.spin)
        Gamepads.gamepad2.b whenBecomesTrue SequentialGroup(InstantCommand { ImprovedOuttake.canSpin = false },
            FlyWheel.stop)
        Gamepads.gamepad2.x whenBecomesTrue SequentialGroup(FlyWheel.backOut)

        // Gear Controls
        Gamepads.gamepad2.rightBumper whenBecomesTrue Turret.spinGearRight whenBecomesFalse Turret.stopGear
        Gamepads.gamepad2.leftBumper whenBecomesTrue Turret.spinGearLeft whenBecomesFalse Turret.stopGear
        Gamepads.gamepad2.dpadRight whenBecomesTrue Turret.gearAlittleLeft whenBecomesFalse Turret.stopGear
        Gamepads.gamepad2.dpadLeft whenBecomesTrue Turret.gearAlittleRight whenBecomesFalse Turret.stopGear

        // Flap Controls
        Gamepads.gamepad2.dpadUp whenBecomesTrue Hood.FlapDown
        Gamepads.gamepad2.dpadDown whenBecomesTrue Hood.FlapUp

        // Aimbot Controls
    }

    override fun onUpdate() {
        tele.run {
            addData("Hood Position ", Hood.hP)
            addData("Power ", FlyWheel.targetVelocity)
            //replace this with LL instead of manualAim
            addData("Distance in Tiles ", ImprovedOuttake.manualAim/24.0)
            addData("Manual Mode ", ImprovedOuttake.fullManual)
            addData("Can Shoot", ImprovedOuttake.canSpin)
            update()
        }
    }
}