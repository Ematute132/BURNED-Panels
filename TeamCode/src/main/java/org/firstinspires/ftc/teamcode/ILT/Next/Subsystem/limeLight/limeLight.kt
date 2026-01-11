package org.firstinspires.ftc.teamcode.ILT.Next.Subsystem.limeLight

import com.pedropathing.geometry.Pose
import com.qualcomm.hardware.limelightvision.LLResult
import com.qualcomm.hardware.limelightvision.Limelight3A
import dev.nextftc.core.subsystems.Subsystem
import dev.nextftc.ftc.ActiveOpMode
import org.firstinspires.ftc.teamcode.ILT.Next.Subsystem.DriveTrain
import org.firstinspires.ftc.teamcode.next.subsystems.data.Motif

object limeLight : Subsystem {

    // So this variable holds a reference to our Limelight3A camera.
    // We mark it as "lateinit" because we’ll set it up later in initialize().
    lateinit var ll: Limelight3A

    // These are the values the Limelight gives us.
    // TX, TY, and TA come directly from its built-in processing.
    // TX = horizontal angle, TY = vertical angle, TA = target area percent on screen.
    var currentTx: Double = 0.0
    var currentTy: Double = 0.0
    var currentTa: Double = 0.0
    var hasValidTarget: Boolean = false // This just says whether it sees anything or not.

    // This one tracks which “motif” (basically a tag encoding) the Limelight sees.
    // If there’s no recognizable one, we set it to NONE.
    var detectedMotif: Motif = Motif.NONE

    // These are for fiducial (tag) detection.
    // “fiducials” are basically AprilTags that have position and orientation data.
    var fiducialCount: Int = 0 // How many tags we see this frame.
    var fiducialData: String = "No fiducials" // String we use for telemetry output.

    // This runs once when the subsystem starts. It's like the setup part.
    override fun initialize() {
        // We grab the Limelight from the hardware map (this tells the code where our camera is).
        ll = ActiveOpMode.hardwareMap.get(Limelight3A::class.java, "ll")

        // Set how often the Limelight updates data (100 times per second).
        ll.setPollRateHz(100)

        // Switch it to pipeline 0 (we can make different vision modes if we wanted).
        ll.pipelineSwitch(0)

        // Start the Limelight camera feed so it begins collecting data.
        ll.start()
    }

    // This runs repeatedly while the robot is running (about every frame).
    override fun periodic() {
        // These three methods keep all the data up to date.
        updateBasicData()
        updateFiducialData()
        updateMotif()
    }

    // Here I grab the basic tracking information like tx (horizontal angle), ty (vertical), and ta (target area)
    private fun updateBasicData() {
        val result = ll.latestResult // Get the latest frame interpretation from the Limelight
        if (result != null && result.isValid) {
            // If there’s a valid target, we store those numbers
            hasValidTarget = true
            currentTx = result.tx
            currentTy = result.ty
            currentTa = result.ta
        } else {
            // Otherwise, we reset everything because there’s nothing visible
            hasValidTarget = false
            currentTx = 0.0
            currentTy = 0.0
            currentTa = 0.0
        }
    }

    // This one looks specifically for fiducials (AprilTags).
    // We collect how many we find and their IDs and positions for debugging and telemetry.
    private fun updateFiducialData() {
        val result = ll.latestResult
        if (result != null && result.isValid) {
            val fiducials = result.fiducialResults
            fiducialCount = fiducials.size // Save how many tags we found

            if (fiducials.isNotEmpty()) {
                val sb = StringBuilder() // Just makes it easier to build a long multi-line string
                for (fr in fiducials) {
                    // For each fiducial, we print its ID and position info.
                    sb.append("ID: ${fr.fiducialId}, ")
                    sb.append("X: ${"%.2f".format(fr.targetXDegrees)}°, ")
                    sb.append("Strafe: ${"%.2f".format(fr.robotPoseTargetSpace.position.x)}\n")
                }
                fiducialData = sb.toString().trim()
            } else {
                fiducialData = "No fiducials detected" // No tags were picked up, so we just say that.
            }
        } else {
            // If the Limelight didn’t return a valid result at all, that usually means it has no frame.
            fiducialCount = 0
            fiducialData = "No valid result"
        }
    }

    // This method checks for specific tag IDs and figures out which Motif it matches.
    // Basically, the robot uses these IDs to understand certain field states.
    private fun updateMotif() {
        val result = ll.latestResult
        if (result != null && result.isValid) {
            val fR = result.fiducialResults
            if (fR.isNotEmpty()) {
                // We only look at the first fiducial in the list for now.
                val f = fR[0]
                // Then we assign a motif based on the tag ID number.
                detectedMotif = when (f.fiducialId) {
                    21 -> Motif.GPP
                    22 -> Motif.PGP
                    else -> Motif.PPG
                }
            } else {
                detectedMotif = Motif.NONE // If there aren’t any tags, there’s no motif.
            }
        } else {
            detectedMotif = Motif.NONE // Again, if no valid frame, no motif.
        }
    }

    // This is kind of like a “raw access” function.
    // It just returns the actual Limelight result if we need to manually check data somewhere else.
    fun grabResultData(): LLResult? {
        val lR = ll.latestResult
        if (lR != null && lR.isValid) {
            return lR
        }
        return null
    }

    // This method uses the MegaTag pose feature.
    // The Limelight can figure out the robot’s position relative to the tags using 3D space.
    fun megaTag(): Pose? {
        val lR = ll.latestResult
        val yaw = DriveTrain.currentHeading // This is the robot’s current rotation from the drivetrain.
        ll.updateRobotOrientation(yaw)      // We tell the Limelight what direction the robot is facing.

        if (lR != null && lR.isValid) {
            val botpose_mt2 = lR.botpose_MT2 // This is the Limelight’s pose data for the robot.
            if (botpose_mt2 != null) {
                // Then we return that as a Pose object so we can use it in path planning.
                return Pose(botpose_mt2.position.x, botpose_mt2.position.y, yaw)
            }
        }
        return null // If there’s nothing valid, return null to show no position was found.
    }

    // This last method is just for pretty printing telemetry to the driver hub or console.
    // It’s helpful when debugging so we can see everything at a glance.
    fun getTelemetryString(): String {
        return buildString {
            appendLine("=== LIMELIGHT STATUS ===")
            appendLine("Valid Target: $hasValidTarget")
            if (hasValidTarget) {
                appendLine("TX: ${"%.2f".format(currentTx)}°")
                appendLine("TY: ${"%.2f".format(currentTy)}°")
                appendLine("TA: ${"%.2f".format(currentTa)}%")
            }
            appendLine("Fiducials: $fiducialCount")
            if (fiducialCount > 0) {
                appendLine(fiducialData)
            }
            appendLine("Motif: $detectedMotif")
        }
    }
}
