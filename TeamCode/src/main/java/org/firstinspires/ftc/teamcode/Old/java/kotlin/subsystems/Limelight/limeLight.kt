package org.firstinspires.ftc.teamcode.next.kotlin.subsystems

import com.pedropathing.geometry.Pose
import com.qualcomm.hardware.limelightvision.LLResult
import com.qualcomm.hardware.limelightvision.Limelight3A
import dev.nextftc.core.subsystems.Subsystem
import dev.nextftc.ftc.ActiveOpMode
import org.firstinspires.ftc.teamcode.Old.java.kotlin.data.Motif
import org.firstinspires.ftc.teamcode.next.subsystems.DriveTrain

object limeLight : Subsystem {

    lateinit var ll: Limelight3A

    // Exposed telemetry values
    var currentTx: Double = 0.0
    var currentTy: Double = 0.0
    var currentTa: Double = 0.0
    var hasValidTarget: Boolean = false

    // Motif detection
    var detectedMotif: Motif = Motif.NONE

    // Fiducial data
    var fiducialCount: Int = 0
    var fiducialData: String = "No fiducials"

    override fun initialize() {
        ll = ActiveOpMode.hardwareMap.get(Limelight3A::class.java, "ll")
        ll.setPollRateHz(100)
        ll.pipelineSwitch(0)
        ll.start()
    }

    override fun periodic() {
        updateBasicData()
        updateFiducialData()
        updateMotif()
    }

    // Update basic limelight data
    private fun updateBasicData() {
        val result = ll.latestResult
        if (result != null && result.isValid) {
            hasValidTarget = true
            currentTx = result.tx
            currentTy = result.ty
            currentTa = result.ta
        } else {
            hasValidTarget = false
            currentTx = 0.0
            currentTy = 0.0
            currentTa = 0.0
        }
    }

    // Update fiducial detection data
    private fun updateFiducialData() {
        val result = ll.latestResult
        if (result != null && result.isValid) {
            val fiducials = result.fiducialResults
            fiducialCount = fiducials.size

            if (fiducials.isNotEmpty()) {
                val sb = StringBuilder()
                for (fr in fiducials) {
                    sb.append("ID: ${fr.fiducialId}, ")
                    sb.append("X: ${"%.2f".format(fr.targetXDegrees)}°, ")
                    sb.append("Strafe: ${"%.2f".format(fr.robotPoseTargetSpace.position.x)}\n")
                }
                fiducialData = sb.toString().trim()
            } else {
                fiducialData = "No fiducials detected"
            }
        } else {
            fiducialCount = 0
            fiducialData = "No valid result"
        }
    }

    // Update motif detection
    private fun updateMotif() {
        val result = ll.latestResult
        if (result != null && result.isValid) {
            val fR = result.fiducialResults
            if (fR.isNotEmpty()) {
                val f = fR[0]
                detectedMotif = when (f.fiducialId) {
                    21 -> Motif.GPP
                    22 -> Motif.PGP
                    else -> Motif.PPG
                }
            } else {
                detectedMotif = Motif.NONE
            }
        } else {
            detectedMotif = Motif.NONE
        }
    }

    // Get raw result (for manual processing if needed)
    fun grabResultData(): LLResult? {
        val lR = ll.latestResult
        if (lR != null && lR.isValid) {
            return lR
        }
        return null
    }

    // Get MegaTag pose for localization
    fun megaTag(): Pose? {
        val lR = ll.latestResult
        val yaw = DriveTrain.currentPose.heading
        ll.updateRobotOrientation(yaw)

        if (lR != null && lR.isValid) {
            val botpose_mt2 = lR.botpose_MT2
            if (botpose_mt2 != null) {
                return Pose(botpose_mt2.position.x, botpose_mt2.position.y, yaw)
            }
        }
        return null
    }

    // Get all telemetry as a formatted string
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