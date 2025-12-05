package org.firstinspires.ftc.teamcode.next.kotlin.subsystems

import com.pedropathing.geometry.Pose
import com.qualcomm.hardware.limelightvision.LLResult
import com.qualcomm.hardware.limelightvision.LLResultTypes.FiducialResult
import com.qualcomm.hardware.limelightvision.Limelight3A
import dev.nextftc.core.commands.utility.InstantCommand
import dev.nextftc.core.subsystems.Subsystem
import dev.nextftc.ftc.ActiveOpMode
import org.firstinspires.ftc.teamcode.next.subsystems.DriveTrain
import org.firstinspires.ftc.teamcode.next.kotlin.data.Motif
import kotlin.math.abs

object limeLight : Subsystem {

    lateinit var ll: Limelight3A

    // Configuration
    var limelightOn: Boolean = true
    var grabMegaTag = false

    // Distance calculation constants (from previous info)
    var llAngle = 9.895942 // Limelight mounting angle in degrees (was 0 in your original code)
    var llLensHeight = 10.2756  // Height of limelight lens in inches (from your original)
    var goalHeight = 29.5  // Height of target in inches (from your original)

    // Auto-alignment settings
    var autoAlignEnabled: Boolean = false
    var targetDistance: Double = 24.0  // Target distance in inches
    var distanceTolerance: Double = 3.0  // Tolerance for distance (inches)
    var angleTolerance: Double = 2.0  // Tolerance for angle alignment (degrees)
    var alignmentKp: Double = 0.02  // Proportional gain for heading correction
    var maxTurnPower: Double = 0.3  // Maximum turn power for alignment

    // Exposed telemetry values
    var angleToGoalDegrees: Double = 0.0
    var angleToGoalRadians: Double = 0.0
    var distanceToGoal: Double? = null
    var currentTx: Double = 0.0
    var currentTy: Double = 0.0
    var currentTa: Double = 0.0
    var hasValidTarget: Boolean = false
    var isAligned: Boolean = false
    var isAtTargetDistance: Boolean = false

    // Motif detection
    var detectedMotif: Motif = Motif.NONE

    // Fiducial data
    var fiducialCount: Int = 0
    var fiducialData: String = "No fiducials"

    // Alignment output
    var alignmentTurnPower: Double = 0.0

    override fun initialize() {
        ll = ActiveOpMode.hardwareMap.get(Limelight3A::class.java, "ll")
        ll.setPollRateHz(100)
        ll.pipelineSwitch(0)
        ll.start()
    }

    override fun periodic() {
        // Update all values every loop
        updateBasicData()
        updateDistanceCalculation()
        updateFiducialData()
        updateMotif()
        updateAlignment()
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

    // Update distance calculation (using correct formula from Limelight docs)
    private fun updateDistanceCalculation() {
        val result = ll.latestResult
        if (result != null && result.isValid) {
            val targetOffsetAngleVertical = result.ty

            // Calculate angles
            angleToGoalDegrees = llAngle + targetOffsetAngleVertical
            angleToGoalRadians = angleToGoalDegrees * (Math.PI / 180.0)

            // Calculate distance
            distanceToGoal = (goalHeight - llLensHeight) / Math.tan(angleToGoalRadians)

            // Check if at target distance
            distanceToGoal?.let { dist ->
                isAtTargetDistance = abs(dist - targetDistance) <= distanceTolerance
            } ?: run {
                isAtTargetDistance = false
            }
        } else {
            distanceToGoal = null
            angleToGoalDegrees = 0.0
            angleToGoalRadians = 0.0
            isAtTargetDistance = false
        }
    }
    // Update alignment status and calculate turn power
    private fun updateAlignment() {
        if (!hasValidTarget || !autoAlignEnabled) {
            alignmentTurnPower = 0.0
            isAligned = false
            return
        }

        // Check if aligned (tx is close to 0)
        isAligned = abs(currentTx) <= angleTolerance

        if (isAligned) {
            alignmentTurnPower = 0.0
        } else {
            // Calculate proportional turn power based on tx error
            alignmentTurnPower = -currentTx * alignmentKp

            // Clamp to max turn power
            alignmentTurnPower = alignmentTurnPower.coerceIn(-maxTurnPower, maxTurnPower)
        }
    }

    // Apply alignment correction to drivetrain
   /* fun applyAlignment() {
        if (autoAlignEnabled && hasValidTarget && isAtTargetDistance) {
            // Apply turn correction while maintaining forward/strafe
            DriveTrain.setDrivePowers(
                forward = 0.0,  // Can be controlled by driver
                strafe = 0.0,   // Can be controlled by driver
                turn = alignmentTurnPower
            )
        }
    }

    */
    fun getDistanceToTarget(): Double? {
        return distanceToGoal
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

    // Get MegaTag pose
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

    // Commands for enabling/disabling auto-align
    val enableAutoAlign = InstantCommand {
        autoAlignEnabled = true
    }

    val disableAutoAlign = InstantCommand {
        autoAlignEnabled = false
        alignmentTurnPower = 0.0
    }

    val toggleAutoAlign = InstantCommand {
        autoAlignEnabled = !autoAlignEnabled
        if (!autoAlignEnabled) {
            alignmentTurnPower = 0.0
        }
    }

    // Manual motif command
    val motifCommand = InstantCommand {
        updateMotif()
    }

    // Get all telemetry as a formatted string
    fun getTelemetryString(): String {
        return buildString {
            appendLine("=== LIMELIGHT STATUS ===")
            appendLine("Valid Target: $hasValidTarget")
            appendLine("Auto-Align: ${if (autoAlignEnabled) "ENABLED" else "DISABLED"}")
            if (hasValidTarget) {
                appendLine("TX: ${"%.2f".format(currentTx)}°")
                appendLine("TY: ${"%.2f".format(currentTy)}°")
                appendLine("TA: ${"%.2f".format(currentTa)}%")
                appendLine("Angle to Goal: ${"%.2f".format(angleToGoalDegrees)}°")
                appendLine("Distance: ${distanceToGoal?.let { "%.2f".format(it) } ?: "N/A"} inches")
                appendLine("At Target Distance: $isAtTargetDistance")
                appendLine("Aligned: $isAligned")
                if (autoAlignEnabled) {
                    appendLine("Turn Power: ${"%.3f".format(alignmentTurnPower)}")
                }
            }
            appendLine("Fiducials: $fiducialCount")
            if (fiducialCount > 0) {
                appendLine(fiducialData)
            }
            appendLine("Motif: $detectedMotif")
        }
    }

    // Debug info for distance calculation
    fun getDistanceDebugInfo(): String {
        return buildString {
            appendLine("=== DISTANCE DEBUG ===")
            appendLine("llAngle: $llAngle°")
            appendLine("TY: ${"%.2f".format(currentTy)}°")
            appendLine("Angle to Goal: ${"%.2f".format(angleToGoalDegrees)}°")
            appendLine("Angle (radians): ${"%.4f".format(angleToGoalRadians)}")
            appendLine("Goal Height: $goalHeight\"")
            appendLine("Lens Height: $llLensHeight\"")
            appendLine("Height Diff: ${goalHeight - llLensHeight}\"")
            appendLine("tan(angle): ${"%.4f".format(Math.tan(angleToGoalRadians))}")
            appendLine("Distance: ${distanceToGoal?.let { "%.2f".format(it) } ?: "N/A"}\"")
        }
    }
}