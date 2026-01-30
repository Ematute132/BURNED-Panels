package org.firstinspires.ftc.teamcode.ILT.Next.Subsystems.Shooter

import com.pedropathing.math.MathFunctions
import dev.nextftc.control.KineticState
import dev.nextftc.control.builder.controlSystem
import dev.nextftc.control.feedback.PIDCoefficients
import dev.nextftc.core.subsystems.Subsystem
import dev.nextftc.hardware.impl.MotorEx
import org.firstinspires.ftc.teamcode.ILT.Next.Data.Alliance
import org.firstinspires.ftc.teamcode.ILT.Next.Subsystems.Drive
import kotlin.math.PI
import kotlin.math.atan2

object Turret : Subsystem {
    private var turret = MotorEx("turret")

    @JvmField var alliance = Alliance.RED
    @JvmField var goalY = 132.0
    @JvmField var RED_GOAL_X = 9.25
    @JvmField var BLUE_GOAL_X = 134.75
    val goalX: Double
        get() = if (alliance == Alliance.RED) {
            RED_GOAL_X
        } else {
            BLUE_GOAL_X
        }


    private const val RADIANS_PER_TICK = (2.0 * PI) / (8192 * 1.0)
    @JvmField var turretPID = PIDCoefficients(0.2, 0.0, 0.0)

    var controller = controlSystem {
        posPid(turretPID)
    }


    override fun periodic() {
// 1. Calculate the angle from robot to the field goal (X, Y)
        val deltaX = goalX - Drive.currentX
        val deltaY = goalY - Drive.currentY
        val fieldAngleToGoal = atan2(deltaY, deltaX)

        // 2. Subtract robot heading to find target relative to the robot
        val robotHeading = Drive.currentHeading
        val targetTurretYaw = MathFunctions.normalizeAngle(fieldAngleToGoal - robotHeading)

        // 3. Apply safety limits (clamping)
        val clampedTarget = targetTurretYaw.coerceIn(-2.35, 2.35) // Approx ±135 degrees

        // 4. Update the controller and move
        val currentYaw = getYaw()
        controller.goal = KineticState(position = clampedTarget)

        val power = controller.calculate(KineticState(position = currentYaw))
        turret.power = power.coerceIn(-0.85, 0.85)

    }
    fun getYaw(): Double = MathFunctions.normalizeAngle(turret.currentPosition * RADIANS_PER_TICK)


}