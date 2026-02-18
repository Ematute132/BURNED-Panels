package org.firstinspires.ftc.teamcode.ILT.Next.Subsystems

import dev.nextftc.core.subsystems.Subsystem
import dev.nextftc.extensions.pedro.PedroComponent.Companion.follower
import dev.nextftc.ftc.ActiveOpMode

object Drive : Subsystem {

    var currentX: Double = 0.0
    var currentY: Double = 0.0
    var currentHeading: Double = 0.0

    override fun initialize() {
        follower.startTeleopDrive()
    }

    override fun periodic() {
        follower.update()  // Must be called every tick for pose to update

        val pose = follower.pose
        currentX = pose.x
        currentY = pose.y
        currentHeading = pose.heading

        ActiveOpMode.telemetry.run {
            addData("=== DRIVETRAIN ===", "")
            addData("X", "%.1f".format(currentX))
            addData("Y", "%.1f".format(currentY))
            addData("Heading", "%.1f°".format(Math.toDegrees(currentHeading)))
        }
    }
}