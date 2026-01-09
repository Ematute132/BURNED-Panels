package org.firstinspires.ftc.teamcode.ILT.Next.TestOp

import com.qualcomm.robotcore.eventloop.opmode.OpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.ILT.Next.Subsystem.Outtake.Shooter.Turret
import kotlin.math.PI

@TeleOp(name = "Turret Test (OpMode)", group = "Test")
class TurretTestOpMode : OpMode() {

    // Current commanded yaw (radians)
    private var targetYaw = 0.0

    // Scale from stick input to radians per loop
    private val stickScale = 0.02   // smaller = slower

    override fun init() {
        telemetry.addLine("Turret Test OpMode initialized")
    }

    override fun loop() {
        // Left stick X: negative -> move turret right, positive -> move turret left
        val stickX = gamepad1.left_stick_x.toDouble()

        // Note the minus sign: positive stickX moves yaw negative (left visually)
        targetYaw -= stickX * stickScale

        // Clamp to ±90 degrees
        val maxYaw = PI / 2
        if (targetYaw > maxYaw) targetYaw = maxYaw
        if (targetYaw < -maxYaw) targetYaw = -maxYaw

        // Send command to turret
        Turret.goToYaw(targetYaw)

        // Optional buttons still work if you want:
        if (gamepad1.a) {
            targetYaw = 0.0
            Turret.goToYaw(targetYaw)
        }

        // Run turret control loop
        Turret.periodic()

        telemetry.addData("targetYaw", targetYaw)
        telemetry.addData("turretYaw", Turret.getYaw())
        telemetry.update()
    }
}
