package org.firstinspires.ftc.teamcode.next.kotlin

import com.bylazar.telemetry.PanelsTelemetry
import com.bylazar.telemetry.JoinedTelemetry
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import dev.nextftc.core.components.BindingsComponent
import dev.nextftc.core.components.SubsystemComponent
import dev.nextftc.extensions.pedro.PedroComponent
import dev.nextftc.ftc.Gamepads
import dev.nextftc.ftc.NextFTCOpMode
import dev.nextftc.ftc.components.BulkReadComponent
import dev.nextftc.hardware.impl.Direction
import dev.nextftc.hardware.impl.IMUEx
import kotlinx.coroutines.flow.DEFAULT_CONCURRENCY
import org.firstinspires.ftc.teamcode.next.kotlin.subsystems.Intake
import org.firstinspires.ftc.teamcode.next.kotlin.subsystems.Outtake
import org.firstinspires.ftc.teamcode.next.kotlin.subsystems.Outtake.autoAdjustVelocity
import org.firstinspires.ftc.teamcode.next.kotlin.subsystems.Limelight.limeLight
import org.firstinspires.ftc.teamcode.next.kotlin.subsystems.Limelight.limeLight.distanceToGoal
import org.firstinspires.ftc.teamcode.next.subsystems.DriveTrain
import org.firstinspires.ftc.teamcode.next.subsystems.DriveTrain.ResetImu
import org.firstinspires.ftc.teamcode.next.subsystems.DriveTrain.imu
import org.firstinspires.ftc.teamcode.pedroPathing.Constants

@TeleOp(name = "Meet2-3:28am")
class Meet2: NextFTCOpMode() {
    var tele = JoinedTelemetry(PanelsTelemetry.ftcTelemetry, telemetry)




    init {
        addComponents(
           SubsystemComponent(Intake, Outtake, DriveTrain, limeLight),
           BulkReadComponent,
           BindingsComponent,
        )
    }

    override fun onStartButtonPressed() {
        // Intake Controls
        Gamepads.gamepad1.leftBumper whenBecomesTrue Intake.reverseIntake whenBecomesFalse Intake.stopIntake
        Gamepads.gamepad1.leftTrigger.greaterThan(0.3) whenBecomesTrue Intake.runIntake whenBecomesFalse Intake.stopIntake
        Gamepads.gamepad1.rightBumper whenBecomesTrue Intake.reverseIntakeSlow whenBecomesFalse Intake.stopIntake

        Gamepads.gamepad1.rightTrigger.greaterThan(0.3) whenBecomesTrue {
            autoAdjustVelocity = true
            Outtake.velocityTrue = true
        } whenBecomesFalse {
            autoAdjustVelocity = false
        }

        Gamepads.gamepad1.a whenBecomesTrue {
            autoAdjustVelocity = false
            Outtake.flywheelOff.run()
        }
        Gamepads.gamepad1.b whenBecomesTrue Outtake.flywheelBack

        Gamepads.gamepad1.triangle whenBecomesTrue {ResetImu()}

        // Limelight Auto-Align Controls
        //Gamepads.gamepad1.triangle whenBecomesTrue { limeLight.autoAlignEnabled() }
       // Gamepads.gamepad1.square whenBecomesTrue { limeLight.disableAutoAlign() }
    }

    override fun onUpdate() {
        tele.run {


            //addLine(limeLight.getTelemetryString())
            //addLine(limeLight.getDistanceDebugInfo())
            //addLine(Outtake.getTelemetryString())
            //addLine(Outtake.getDebugInfo())


            // Optional: Add debug info
            // addLine(limeLight.getDistanceDebugInfo())

            update()
        }

        // Continuously update velocity based on distance when auto-adjust is enabled

    }
}