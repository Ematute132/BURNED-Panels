package org.firstinspires.ftc.teamcode.ILT.Next.Subsystems

import com.pedropathing.geometry.Pose
import dev.nextftc.core.commands.utility.InstantCommand
import dev.nextftc.core.subsystems.Subsystem
import dev.nextftc.extensions.pedro.PedroComponent.Companion.follower
import dev.nextftc.ftc.ActiveOpMode
import kotlin.math.sqrt

object Drive : Subsystem{
        private var isInitialized = false
    var currentX: Double = 0.0
    var currentY: Double = 0.0
    var currentHeading: Double = 0.0
    var poseValid = false

        // The Pedro driver controlled command - created once, called every loop


        // ==================== INITIALIZATION ====================
        override fun initialize() {
            try {
                // Create the driver controlled command
                // This uses Gamepads which wraps the gamepad inputs
                isInitialized = true
                ActiveOpMode.telemetry.addData("DriveTrain", "Initialized OK")
            } catch (e: Exception) {
                ActiveOpMode.telemetry.addData("DriveTrain Error", e.message)
                isInitialized = false
            }
        }


        // ==================== PERIODIC - MUST CALL driverControlled() HERE ====================
        override fun periodic() {

            // THIS IS THE KEY LINE - actually run the Pedro driving!
            // The () invokes the command's update logic

            // Update pose from Pedro follower
            try {
                val pose = follower.pose
                currentX = pose.x
                currentY = pose.y
                currentHeading = pose.heading
                poseValid = true

                // Calculate distance to goal
            } catch (e: Exception) {
               poseValid = false
            }

            // Telemetry
            ActiveOpMode.telemetry.run {
                addData("=== DRIVETRAIN ===", "")
                addData("Pose Valid", poseValid)
                if (poseValid) {
                    addData("X", "%.1f".format(currentX))
                    addData("Y", "%.1f".format(currentY))
                    addData("Heading", "%.1f°".format(Math.toDegrees(currentHeading)))
                }
            }
        }



    }
