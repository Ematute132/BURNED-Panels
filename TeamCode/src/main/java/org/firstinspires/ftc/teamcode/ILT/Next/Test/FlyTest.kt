package org.firstinspires.ftc.teamcode.opmodes.testing.baseSubsystems

import com.bylazar.configurables.annotations.Configurable
import com.bylazar.telemetry.PanelsTelemetry
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import dev.nextftc.core.components.BindingsComponent
import dev.nextftc.core.components.SubsystemComponent
import dev.nextftc.ftc.NextFTCOpMode
import dev.nextftc.ftc.components.BulkReadComponent
import org.firstinspires.ftc.teamcode.ILT.Next.Subsystems.Shooter.FlyWheel
import java.util.Collections.frequency

@Configurable
@TeleOp(name = "Shooter1 F Test", group = "Base Subsystem Tests")
class ShooterFTest : NextFTCOpMode() {
    companion object {
        @JvmField var speed = 0.0
    }

    init {
        addComponents(
            SubsystemComponent(FlyWheel),
            BulkReadComponent,
            BindingsComponent
        )
    }

    override fun onUpdate() {
        // Logic for button presses needs to be in onUpdate to be checked every frame
        // Note: NextFTC gamepad wrappers usually handle this, but here is the standard way:
        if (gamepad1.aWasPressed()) {
            FlyWheel.setVelocity(2000.0)
        } else if (gamepad1.bWasPressed()) {
            FlyWheel.setVelocity(0.0)
        } else if (gamepad1.xWasPressed()) {
            FlyWheel.setVelocity(1000.0)
        }

        // Add Telemetry to the Driver Station
        telemetry.addLine("--- Shooter Test ---")

        telemetry.addData("Target Velocity", speed)

        // Updates the Driver Station display
        telemetry.update()
    }
}