package org.firstinspires.ftc.teamcode.ILT.Java.LimeLightJava;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.ftc.ActiveOpMode;

public class LLAutoVeloJava implements Subsystem {

    // Here I'm declaring the Limelight camera object.
    // It will be initialized once the subsystem starts.
    private static Limelight3A ll;

    // These are all the basic physical configuration values for the Limelight.
    // They describe how and where it's mounted on the robot.
    public static double llAngle = 9.895942;           // This is the tilt angle of the Limelight in degrees.
    public static double llLensHeight = 10.2756;       // The height of the Limelight's lens from the ground (in inches).
    public static double goalHeight = 29.5;            // The height of the scoring target we're aiming for (in inches).

    // These are constants for the physics calculations that estimate launch velocity.
    private static final double LAUNCH_ANGLE_DEG = 34.36;        // The angle the ball leaves the shooter at.
    private static final double SHOOTER_HEIGHT_IN = 12.9774972441; // How high the shooter is from the ground.
    private static final double GOAL_HEIGHT_IN = 37.85;           // The target goal height (probably top of the goal).
    private static final double GRAVITY_IN_PER_S2 = 386.0;        // Gravity's acceleration in inches per second squared.
    private static final double SHOOTER_DIAMETER_IN = 2.83465;    // Diameter of the projectile.
    private static final double SHOOTER_RADIUS_IN = SHOOTER_DIAMETER_IN / 2.0;  // Radius for velocity conversion.

    // This is for the motor setup we're using.
    // For example, goBILDA 6000 RPM motors have 28 encoder ticks per revolution.
    public static double motorTicksPerRev = 28.0;

    // These variables help us calculate and track the robot's distance to the target.
    public static double targetDistance = 24.0;                   // The distance we want to shoot from.
    public static double distanceTolerance = 3.0;                 // Allowed distance difference margin (±3 inches).
    private static double angleToGoalDegrees = 0.0;        // Vertical angle from Limelight in degrees.
    private static double angleToGoalRadians = 0.0;        // Same angle converted to radians for trig math.
    private static Double distanceToGoal = null;          // Computed distance (null if not detected yet).
    private static boolean isAtTargetDistance = false;     // Whether we're close enough to shoot from here.
    private static double currentTy = 0.0;                 // Raw vertical offset from Limelight.
    private static boolean hasValidTarget = false;         // Whether the Limelight sees a valid target.

    // The calculated results go here once we compute how fast to spin the wheel.
    private static double calculatedVelocity = 0.0;        // This is in ticks per second (for motor control).
    private static double calculatedRPM = 0.0;             // This is the flywheel's required RPM.

    // This sets up the Limelight when the subsystem starts running.
    @Override
    public void initialize() {
        // We grab the Limelight from the hardware map so it connects to the physical camera.
        ll = ActiveOpMode.hardwareMap().get(Limelight3A.class, "ll");
        ll.setPollRateHz(100);   // Tells it to refresh data 100 times per second.
        ll.pipelineSwitch(0);    // Uses pipeline 0 (could switch if more vision modes existed).
        ll.start();              // Starts the Limelight feed.
    }

    // This runs repeatedly while the robot code is active.
    // It updates both the measured distance and the shooter speed calculation.
    @Override
    public void periodic() {
        updateDistanceCalculation();
        updateVelocityCalculation();
    }

    // This function figures out how far the robot is from the target using the camera angle.
    private static void updateDistanceCalculation() {
        if (ll == null) return;

        LLResult result = ll.getLatestResult();
        if (result != null && result.isValid()) {
            hasValidTarget = true;
            double targetOffsetAngleVertical = result.getTy(); // ty is the vertical aim offset.
            currentTy = targetOffsetAngleVertical;

            // The total vertical angle to the goal combines the Limelight mount angle and offset from TY.
            angleToGoalDegrees = llAngle + targetOffsetAngleVertical;
            angleToGoalRadians = angleToGoalDegrees * (Math.PI / 180.0);

            // Use trigonometry to find the distance from the Limelight to the goal.
            // tan(θ) = (height difference) / (distance)
            double tanAngle = Math.tan(angleToGoalRadians);
            if (tanAngle != 0.0) {
                distanceToGoal = (goalHeight - llLensHeight) / tanAngle;
            } else {
                distanceToGoal = null;
            }

            // Check if that measured distance is close enough to our desired target distance.
            if (distanceToGoal != null) {
                isAtTargetDistance = Math.abs(distanceToGoal - targetDistance) <= distanceTolerance;
            } else {
                isAtTargetDistance = false;
            }
        } else {
            // If the Limelight doesn't see a valid target, reset everything.
            hasValidTarget = false;
            distanceToGoal = null;
            angleToGoalDegrees = 0.0;
            angleToGoalRadians = 0.0;
            isAtTargetDistance = false;
            currentTy = 0.0;
        }
    }

    // Once we know the distance, this uses physics to compute what velocity and RPM we need.
    private static void updateVelocityCalculation() {
        if (distanceToGoal != null && distanceToGoal > 0) {
            // If the distance makes sense, calculate how fast the shooter must spin.
            calculatedVelocity = calculateMotorVelocity(distanceToGoal);
            calculatedRPM = calculateRPM(distanceToGoal);
        } else {
            // Otherwise, set them to zero.
            calculatedVelocity = 0.0;
            calculatedRPM = 0.0;
        }
    }

    // This one specifically calculates the *shooter's required RPM* to make the shot.
    // It uses some kinematic math based on the projectile motion formula.
    private static double calculateRPM(double distanceToTarget) {
        double theta = Math.toRadians(LAUNCH_ANGLE_DEG);
        double cosTheta = Math.cos(theta);
        double tanTheta = Math.tan(theta);

        double heightDiff = GOAL_HEIGHT_IN - SHOOTER_HEIGHT_IN;

        // The formula below comes from projectile motion:
        // v = sqrt(g * d^2 / (2 * cos^2(θ) * (d * tan(θ) - h)))
        double numerator = GRAVITY_IN_PER_S2 * distanceToTarget * distanceToTarget;
        double denominator = 2.0 * cosTheta * cosTheta * (distanceToTarget * tanTheta - heightDiff);

        // If denominator <= 0, that means the shot path is impossible.
        if (denominator <= 0) return 0.0;

        // Finds projectile exit speed needed (inches per second).
        double velocityInPerSec = Math.sqrt(numerator / denominator);

        // Convert that linear velocity to rotational speed in RPM.
        // RPM = (velocity / (2πr)) * 60
        double rpm = 60.0 * velocityInPerSec / (2.0 * Math.PI * SHOOTER_RADIUS_IN);

        return rpm;
    }

    // Now this turns the RPM into motor ticks per second (what the robot actually controls).
    private static double calculateMotorVelocity(double distanceToTarget) {
        double rpm = calculateRPM(distanceToTarget);
        if (rpm == 0.0) return 0.0;

        // (Revs per second) * ticks per rev = ticks per second
        double ticksPerSec = (rpm / 60.0) * motorTicksPerRev;

        return ticksPerSec;
    }

    // These next few are just simple getter functions so other parts of the robot can read the data.


    // This prints detailed information for debugging — useful when tuning.
    public static String getDistanceDebugInfo() {
        StringBuilder sb = new StringBuilder();
        sb.append("=== DISTANCE DEBUG ===").append("\n");
        sb.append("Valid Target: ").append(hasValidTarget).append("\n");
        sb.append("llAngle: ").append(llAngle).append("°").append("\n");
        sb.append("TY: ").append(String.format("%.2f", currentTy)).append("°").append("\n");
        sb.append("Angle to Goal: ").append(String.format("%.2f", angleToGoalDegrees)).append("°").append("\n");
        sb.append("Angle (radians): ").append(String.format("%.4f", angleToGoalRadians)).append("\n");
        sb.append("Goal Height: ").append(goalHeight).append("\"").append("\n");
        sb.append("Lens Height: ").append(llLensHeight).append("\"").append("\n");
        sb.append("Height Diff: ").append(goalHeight - llLensHeight).append("\"").append("\n");
        sb.append("tan(angle): ").append(String.format("%.4f", Math.tan(angleToGoalRadians))).append("\n");
        sb.append("Distance: ").append(distanceToGoal != null ? String.format("%.2f", distanceToGoal) : "N/A").append("\"").append("\n");
        sb.append("\n");
        sb.append("=== VELOCITY CALCULATION ===").append("\n");
        sb.append("Calculated RPM: ").append(String.format("%.0f", calculatedRPM)).append("\n");
        sb.append("Calculated Velocity: ").append(String.format("%.0f", calculatedVelocity)).append(" ticks/sec").append("\n");
        sb.append("Motor TPR: ").append(motorTicksPerRev).append("\n");
        sb.append("Launch Angle: ").append(LAUNCH_ANGLE_DEG).append("°").append("\n");
        sb.append("Shooter Height: ").append(String.format("%.2f", SHOOTER_HEIGHT_IN)).append("\"").append("\n");
        sb.append("Target Height: ").append(GOAL_HEIGHT_IN).append("\"");
        return sb.toString();
    }

    // This is what I'd show on the driver hub or telemetry to summarize all the info at a glance.
    public static String getTelemetryString() {
        StringBuilder sb = new StringBuilder();
        sb.append("=== LIMELIGHT AUTO VELO ===").append("\n");
        sb.append("Valid Target: ").append(hasValidTarget).append("\n");
        if (hasValidTarget) {
            sb.append("Distance: ").append(distanceToGoal != null ? String.format("%.2f", distanceToGoal) : "N/A").append(" inches").append("\n");
            sb.append("Required RPM: ").append(String.format("%.0f", calculatedRPM)).append("\n");
            sb.append("Required Velocity: ").append(String.format("%.0f", calculatedVelocity)).append(" ticks/sec").append("\n");
            sb.append("At Target Distance: ").append(isAtTargetDistance);
        } else {
            sb.append("No target detected");
        }
        return sb.toString();
    }
}