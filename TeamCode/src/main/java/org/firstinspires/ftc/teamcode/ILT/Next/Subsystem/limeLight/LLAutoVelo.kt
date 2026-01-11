package org.firstinspires.ftc.teamcode.next.kotlin.subsystems

import com.qualcomm.hardware.limelightvision.Limelight3A
import dev.nextftc.core.subsystems.Subsystem
import dev.nextftc.ftc.ActiveOpMode
import kotlin.math.*

object LLAutoVelo : Subsystem {

    // Here I’m declaring the Limelight camera object.
    // I use “lateinit” because I’ll hook it up once the subsystem initializes.
    lateinit var ll: Limelight3A

    // These are all the basic physical configuration values for the Limelight.
    // They describe how and where it's mounted on the robot.
    var llAngle = 9.895942           // This is the tilt angle of the Limelight in degrees.
    var llLensHeight = 10.2756       // The height of the Limelight’s lens from the ground (in inches).
    var goalHeight = 29.5            // The height of the scoring target we’re aiming for (in inches).

    // These are constants for the physics calculations that estimate launch velocity.
    private const val LAUNCH_ANGLE_DEG = 34.36        // The angle the ball leaves the shooter at.
    private const val SHOOTER_HEIGHT_IN = 12.9774972441 // How high the shooter is from the ground.
    private const val GOAL_HEIGHT_IN = 37.85           // The target goal height (probably top of the goal).
    private const val GRAVITY_IN_PER_S2 = 386.0        // Gravity’s acceleration in inches per second squared.
    private const val SHOOTER_DIAMETER_IN = 2.83465    // Diameter of the projectile.
    private const val SHOOTER_RADIUS_IN = SHOOTER_DIAMETER_IN / 2.0  // Radius for velocity conversion.

    // This is for the motor setup we’re using.
    // For example, goBILDA 6000 RPM motors have 28 encoder ticks per revolution.
    var motorTicksPerRev = 28.0

    // These variables help us calculate and track the robot's distance to the target.
    var targetDistance = 24.0                   // The distance we want to shoot from.
    var distanceTolerance = 3.0                 // Allowed distance difference margin (±3 inches).
    var angleToGoalDegrees: Double = 0.0        // Vertical angle from Limelight in degrees.
    var angleToGoalRadians: Double = 0.0        // Same angle converted to radians for trig math.
    var distanceToGoal: Double? = null          // Computed distance (null if not detected yet).
    var isAtTargetDistance: Boolean = false     // Whether we’re close enough to shoot from here.
    var currentTy: Double = 0.0                 // Raw vertical offset from Limelight.
    var hasValidTarget: Boolean = false         // Whether the Limelight sees a valid target.

    // The calculated results go here once we compute how fast to spin the wheel.
    var calculatedVelocity: Double = 0.0        // This is in ticks per second (for motor control).
    var calculatedRPM: Double = 0.0             // This is the flywheel’s required RPM.

    // This sets up the Limelight when the subsystem starts running.
    override fun initialize() {
        // We grab the Limelight from the hardware map so it connects to the physical camera.
        ll = ActiveOpMode.hardwareMap.get(Limelight3A::class.java, "ll")
        ll.setPollRateHz(100)   // Tells it to refresh data 100 times per second.
        ll.pipelineSwitch(0)    // Uses pipeline 0 (could switch if more vision modes existed).
        ll.start()              // Starts the Limelight feed.
    }

    // This runs repeatedly while the robot code is active.
    // It updates both the measured distance and the shooter speed calculation.
    override fun periodic() {
        updateDistanceCalculation()
        updateVelocityCalculation()
    }

    // This function figures out how far the robot is from the target using the camera angle.
    private fun updateDistanceCalculation() {
        val result = ll.latestResult
        if (result != null && result.isValid) {
            hasValidTarget = true
            val targetOffsetAngleVertical = result.ty // ty is the vertical aim offset.
            currentTy = targetOffsetAngleVertical

            // The total vertical angle to the goal combines the Limelight mount angle and offset from TY.
            angleToGoalDegrees = llAngle + targetOffsetAngleVertical
            angleToGoalRadians = angleToGoalDegrees * (PI / 180.0)

            // Use trigonometry to find the distance from the Limelight to the goal.
            // tan(θ) = (height difference) / (distance)
            distanceToGoal = (goalHeight - llLensHeight) / tan(angleToGoalRadians)

            // Check if that measured distance is close enough to our desired target distance.
            distanceToGoal?.let { dist ->
                isAtTargetDistance = abs(dist - targetDistance) <= distanceTolerance
            } ?: run {
                isAtTargetDistance = false
            }
        } else {
            // If the Limelight doesn’t see a valid target, reset everything.
            hasValidTarget = false
            distanceToGoal = null
            angleToGoalDegrees = 0.0
            angleToGoalRadians = 0.0
            isAtTargetDistance = false
            currentTy = 0.0
        }
    }

    // Once we know the distance, this uses physics to compute what velocity and RPM we need.
    private fun updateVelocityCalculation() {
        val distance = distanceToGoal
        if (distance != null && distance > 0) {
            // If the distance makes sense, calculate how fast the shooter must spin.
            calculatedVelocity = calculateMotorVelocity(distance)
            calculatedRPM = calculateRPM(distance)
        } else {
            // Otherwise, set them to zero.
            calculatedVelocity = 0.0
            calculatedRPM = 0.0
        }
    }

    // This one specifically calculates the *shooter's required RPM* to make the shot.
    // It uses some kinematic math based on the projectile motion formula.
    fun calculateRPM(distanceToTarget: Double): Double {
        val theta = Math.toRadians(LAUNCH_ANGLE_DEG)
        val cosTheta = cos(theta)
        val tanTheta = tan(theta)

        val heightDiff = GOAL_HEIGHT_IN - SHOOTER_HEIGHT_IN

        // The formula below comes from projectile motion:
        // v = sqrt(g * d^2 / (2 * cos^2(θ) * (d * tan(θ) - h)))
        val numerator = GRAVITY_IN_PER_S2 * distanceToTarget * distanceToTarget
        val denominator = 2.0 * cosTheta * cosTheta * (distanceToTarget * tanTheta - heightDiff)

        // If denominator <= 0, that means the shot path is impossible.
        if (denominator <= 0) return 0.0

        // Finds projectile exit speed needed (inches per second).
        val velocityInPerSec = sqrt(numerator / denominator)

        // Convert that linear velocity to rotational speed in RPM.
        // RPM = (velocity / (2πr)) * 60
        val rpm = 60.0 * velocityInPerSec / (2.0 * PI * SHOOTER_RADIUS_IN)

        return rpm
    }

    // Now this turns the RPM into motor ticks per second (what the robot actually controls).
    fun calculateMotorVelocity(distanceToTarget: Double): Double {
        val rpm = calculateRPM(distanceToTarget)
        if (rpm == 0.0) return 0.0

        // (Revs per second) * ticks per rev = ticks per second
        val ticksPerSec = (rpm / 60.0) * motorTicksPerRev

        return ticksPerSec
    }

    // These next few are just simple getter functions so other parts of the robot can read the data.
    fun getDistanceToTarget(): Double? {
        return distanceToGoal
    }

    fun getCalculatedVelocity(): Double {
        return calculatedVelocity
    }

    fun getCalculatedRPM(): Double {
        return calculatedRPM
    }

    // This prints detailed information for debugging — useful when tuning.
    fun getDistanceDebugInfo(): String {
        return buildString {
            appendLine("=== DISTANCE DEBUG ===")
            appendLine("Valid Target: $hasValidTarget")
            appendLine("llAngle: $llAngle°")
            appendLine("TY: ${"%.2f".format(currentTy)}°")
            appendLine("Angle to Goal: ${"%.2f".format(angleToGoalDegrees)}°")
            appendLine("Angle (radians): ${"%.4f".format(angleToGoalRadians)}")
            appendLine("Goal Height: $goalHeight\"")
            appendLine("Lens Height: $llLensHeight\"")
            appendLine("Height Diff: ${goalHeight - llLensHeight}\"")
            appendLine("tan(angle): ${"%.4f".format(Math.tan(angleToGoalRadians))}")
            appendLine("Distance: ${distanceToGoal?.let { "%.2f".format(it) } ?: "N/A"}\"")
            appendLine()
            appendLine("=== VELOCITY CALCULATION ===")
            appendLine("Calculated RPM: ${"%.0f".format(calculatedRPM)}")
            appendLine("Calculated Velocity: ${"%.0f".format(calculatedVelocity)} ticks/sec")
            appendLine("Motor TPR: $motorTicksPerRev")
            appendLine("Launch Angle: $LAUNCH_ANGLE_DEG°")
            appendLine("Shooter Height: ${"%.2f".format(SHOOTER_HEIGHT_IN)}\"")
            appendLine("Target Height: $GOAL_HEIGHT_IN\"")
        }
    }

    // This is what I’d show on the driver hub or telemetry to summarize all the info at a glance.
    fun getTelemetryString(): String {
        return buildString {
            appendLine("=== LIMELIGHT AUTO VELO ===")
            appendLine("Valid Target: $hasValidTarget")
            if (hasValidTarget) {
                appendLine("Distance: ${distanceToGoal?.let { "%.2f".format(it) } ?: "N/A"} inches")
                appendLine("Required RPM: ${"%.0f".format(calculatedRPM)}")
                appendLine("Required Velocity: ${"%.0f".format(calculatedVelocity)} ticks/sec")
                appendLine("At Target Distance: $isAtTargetDistance")
            } else {
                appendLine("No target detected")
            }
        }
    }
}
