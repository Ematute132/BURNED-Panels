package org.firstinspires.ftc.teamcode.ILT.Next.Subsystem

import com.bylazar.configurables.annotations.Configurable
import com.pedropathing.geometry.Pose
import dev.nextftc.core.commands.Command
import dev.nextftc.core.subsystems.Subsystem
import dev.nextftc.extensions.pedro.PedroComponent.Companion.follower
import dev.nextftc.ftc.Gamepads
import dev.nextftc.hardware.driving.FieldCentric
import dev.nextftc.hardware.driving.MecanumDriverControlled
import dev.nextftc.hardware.impl.Direction
import dev.nextftc.hardware.impl.IMUEx
import dev.nextftc.hardware.impl.MotorEx
import kotlin.math.PI
import org.firstinspires.ftc.teamcode.ILT.Next.Subsystem.Data.Alliance
import kotlin.math.abs

@Configurable
object DriveTrain: Subsystem {
    // Here I declare the four mecanum motors and the IMU (gyro) for heading awareness.
    val fL = MotorEx("fl")
    val fR = MotorEx("fr")
    val bL = MotorEx("bl")
    val bR = MotorEx("br")
    val imu = IMUEx("imu", Direction.RIGHT, Direction.UP)

    // alliance tells the robot which side of the field we’re on (affects goal targeting).
    @JvmField var alliance = Alliance.RED

    // sensitivity scales the driver’s joystick inputs (useful for precision driving).
    // lower sensitivity means robot drives slower
    @JvmField var sensitivity = 1.0

    // These three track the robot’s current position and heading on the field.
    var currentX = 0.0
    var currentY = 0.0
    var currentHeading = 0.0

    // HEADING LOCK STATE
    // These variables control the “heading lock” feature, where the robot holds a specific angle.
    private var targetHeadingLock: Double? = null
    private var headingLockActive = false

    // This is the default driving command that runs when no other commands override it.
    override val defaultCommand: Command
        get() = MecanumDriverControlled(
            fL, fR, bL, bR,
            -Gamepads.gamepad1.leftStickY.map { it * sensitivity }, // Forward/back with sensitivity
            Gamepads.gamepad1.leftStickX.map { it * sensitivity },  // Strafe left/right
            Gamepads.gamepad1.rightStickX.map { it * sensitivity }, // Rotate
            FieldCentric(imu) // Makes driving relative to the field orientation using the IMU
        )

    override fun periodic() {
        // Every loop, I update the robot’s position and heading from the path follower.
        currentX = follower.pose.x
        currentY = follower.pose.y
        currentHeading = follower.heading

        // HEADING LOCK - Direct motor control
        if (headingLockActive && targetHeadingLock != null) {
            // If heading lock is active, I calculate how far off we are from the target angle.
            val targetHeading = targetHeadingLock!!
            val headingError = normalizeAngle(targetHeading - currentHeading)
            val turnPower = (headingError * 0.4).coerceIn(-0.8, 0.8)  // Clamp power

            // MECHANUM TURN: Opposite motors for rotation
            // This applies rotation power directly to the motors to hold the heading.
            // Front-left and back-left spin one way, front-right and back-right spin the opposite.
            fL.power = -turnPower
            fR.power = turnPower
            bL.power = -turnPower
            bR.power = turnPower
        }
        // defaultCommand handles manual drive when no heading lock
    }

    // LLAutoTurn INTERFACE
    // These functions let other subsystems (like turret auto-aim) control the robot’s heading.
    fun setTargetHeading(target: Double) {
        // This gets called when we want the robot to point at a specific angle.
        // For example, for shooting, the turret might want the robot to face the goal.
        targetHeadingLock = target
        headingLockActive = true
    }

    fun clearTargetHeading() {
        // This releases the heading lock and stops all motors.
        targetHeadingLock = null
        headingLockActive = false
        // Stop motors when releasing
        fL.power = 0.0
        fR.power = 0.0
        bL.power = 0.0
        bR.power = 0.0
    }

    // This helper function converts any angle into the range [-π, π] (standard robotics convention).
    private fun normalizeAngle(angle: Double): Double {
        var normalized = angle
        while (normalized > PI) normalized -= 2 * PI
        while (normalized < -PI) normalized += 2 * PI
        return normalized
    }

    // Your existing methods unchanged...
    // This function checks if a point (Pose) is inside a triangle defined by three other points.
    // It uses barycentric coordinates — a math trick to test if a point is inside a triangle.
    fun PoseInTriangle(p: Pose, a: Pose, b: Pose, c: Pose): Boolean {
        val det = (b.y - c.y) * (a.x - c.x) + (c.x - b.x) * (a.y - c.y)
        if (abs(det) < 1e-6) return false
        val u = ((b.y - c.y) * (p.x - c.x) + (c.x - b.x) * (p.y - c.y)) / det
        val v = ((c.y - a.y) * (p.x - c.x) + (a.x - c.x) * (p.y - c.y)) / det
        val w = 1 - u - v
        return u >= 0 && v >= 0 && w >= 0
    }

    // This checks if the robot is in a safe shooting zone.
    // It models the field with triangles for safe zones and obstacles, then checks if any corner of the robot overlaps.
    fun inShootZone(): Boolean {
        // These define the triangles for the upper safe zone, lower safe zone, and obstacle zone.
        val obstacle = listOf(Pose(0.0, 115.0), Pose(25.0, 144.0), Pose(0.0, 141.0))
        val upper = listOf(Pose(0.0, 115.0), Pose(25.0, 144.0), Pose(72.0, 72.0))
        val lower = listOf(Pose(48.0, 0.0), Pose(72.0, 24.0), Pose(72.0, 0.0))

        // The robot’s bounding box corners (assuming it’s about 13" x 13").
        val hw = 13.0 / 2.0
        val hl = 13.0 / 2.0
        val corners = listOf(
            Pose(currentX - hw, currentY - hl),
            Pose(currentX + hw, currentY - hl),
            Pose(currentX + hw, currentY + hl),
            Pose(currentX - hw, currentY + hl)
        )

        // Helper function to check if any robot corner is inside a triangle.
        fun overlaps(tri: List<Pose>): Boolean {
            return corners.any { PoseInTriangle(it, tri[0], tri[1], tri[2]) }
        }

        // We’re good to shoot if we’re in an upper OR lower safe zone AND NOT in the obstacle zone.
        val inUpper = overlaps(upper)
        val inLower = overlaps(lower)
        val inObstacle = overlaps(obstacle)
        return (inUpper || inLower) && !inObstacle
    }
}
