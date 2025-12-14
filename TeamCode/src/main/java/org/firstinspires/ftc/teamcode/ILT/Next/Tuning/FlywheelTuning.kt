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
import org.firstinspires.ftc.teamcode.ILT.Next.Subsystem.Intake
import org.firstinspires.ftc.teamcode.ILT.Next.Subsystem.Outtake.Outtake
import org.firstinspires.ftc.teamcode.pedroPathing.Constants

@TeleOp
class FlywheelTuning : NextFTCOpMode() {

    private val tele = JoinedTelemetry(PanelsTelemetry.ftcTelemetry, telemetry)

    init {
        addComponents(
            SubsystemComponent(Intake, Outtake),
            PedroComponent(Constants::createFollower),
            BulkReadComponent,
            BindingsComponent,
        )
    }

    override fun onInit() {
        follower.setStartingPose(Pose(144 - 36.0, 6.5, Math.PI / 2))

        // Make sure there is a nonzero target and gains while tuning
        Outtake.targetOnVelo = 1500.0      // example units (ticks/s or rad/s – match your system)

    }

    override fun onStartButtonPressed() {
        // Make sure these are Commands that flip flywheelsOn / targetOnVelo
        Gamepads.gamepad1.x whenBecomesTrue Outtake.flywheelOn
        Gamepads.gamepad1.y whenBecomesTrue Outtake.flywheelOff
        Gamepads.gamepad1.cross whenBecomesTrue Outtake.outtakeBalls
        Gamepads.gamepad1.dpadUp whenBecomesTrue Outtake.zeroMotor

        Gamepads.gamepad2.rightBumper whenBecomesTrue Outtake.spinGearRight whenBecomesFalse Outtake.stopGear
        Gamepads.gamepad2.leftBumper  whenBecomesTrue Outtake.spinGearLeft  whenBecomesFalse Outtake.stopGear
    }

    override fun onUpdate() {
        tele.run {
            addData("current X", Outtake.currentX)
            addData("current Y", Outtake.currentY)
            addData("current H", Outtake.currentHeading)

            addData("f1 power", Outtake.f1.power)
            addData("f1 vel", Outtake.f1.velocity)
            addData("f2 vel", Outtake.f2.velocity)

            addData("kinetic state", Outtake.f1.state)
            addData("controller", Outtake.controller)
            addData("ctrl output", Outtake.controller.calculate(Outtake.f1.state))

            addData("target vel", Outtake.targetOnVelo)

            addData("gear pos", Outtake.gP)
            addData("intake pos", Intake.iP)

            addData("spin power", Outtake.spin.power)
            addData("spin vel", Outtake.spin.velocity)
            addData("spin pos", Outtake.spin.currentPosition)

            addData("currentAngle", Outtake.turrentAngle)
            addData("prev angle", Outtake.prevAngle)
            addData("total angle", Outtake.totalAngle)
            addData("d heading", Outtake.dHeading)
            addData("turret heading", Outtake.turretHeading)
            addData("target heading", Outtake.targetHeading)
            addData("target x", Outtake.xcord)
            addData("target y", Outtake.ycord)

            addData("flywheel goal", Outtake.sC.goal)
            addData("dist", Outtake.dist)
            addData("flap pos", Outtake.hP)

            update()
        }
    }
}
