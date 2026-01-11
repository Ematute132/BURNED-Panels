package org.firstinspires.ftc.teamcode.ILT.Next.TestOp

import com.qualcomm.robotcore.eventloop.opmode.OpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.ILT.Next.Subsystem.Outtake.Shooter.Hood

@TeleOp(name = "Hood Test", group = "Test")
class HoodTestOp : OpMode() {

    override fun init() {
        telemetry.addLine("Hood Test Initialized")
        telemetry.addLine("Use left stick Y to adjust hood position")
        telemetry.addLine("Dpad up/down for fine adjustments")
        telemetry.update()
    }

    override fun loop() {
        // Left stick Y controls hood position continuously (0.0 to 1.0)
        val stickInput = -gamepad1.left_stick_y * 0.005  // Small multiplier for smooth control
        Hood.updatePosition((Hood.hP + stickInput).coerceIn(0.0, 1.0))

        // Dpad for incremental adjustments
        if (gamepad1.dpad_up) {
            Hood.hP = (Hood.hP + 0.01).coerceIn(0.0, 1.0)
        }
        if (gamepad1.dpad_down) {
            Hood.hP = (Hood.hP - 0.01).coerceIn(0.0, 1.0)
        }

        // Telemetry for monitoring
        telemetry.addData("Target Hood Pos", "%.3f".format(Hood.hP))
        telemetry.addData("Servo Pos", "%.3f".format(Hood.hS.position))
        telemetry.addData("Left Stick Y", "%.3f".format(gamepad1.left_stick_y))
        telemetry.update()
    }
}
