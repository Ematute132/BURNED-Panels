package org.firstinspires.ftc.teamcode.ILT.Next.Subsystem.Outtake

import dev.nextftc.control.KineticState
import dev.nextftc.control.builder.controlSystem
import dev.nextftc.control.feedback.PIDCoefficients
import dev.nextftc.core.subsystems.Subsystem
import dev.nextftc.ftc.ActiveOpMode
import dev.nextftc.hardware.impl.MotorEx

import kotlin.math.PI
import kotlin.math.atan2

object Turret: Subsystem {
    //convert this tele into Panels
    //private val tele = MultipleTelemetry(FtcDashboard.getInstance().telemetry, ActiveOpMode.telemetry)
    private val turret = MotorEx("turret")
    //ask will what the gear ratio is cause its needed for math
    private val gearRatio = 3.47

    @JvmField
    var autoTurret = true

    // change values when u tune the robot
    @JvmField
    var turretPID = PIDCoefficients(2.0, 0.0, 0.2)
    var turretController = controlSystem {
        posPid(turretPID)
    }

    private val ppr = 537.7 // The resolution of our motor encoder on the goBilda site
    private val rpt = 2* PI /(ppr * gearRatio) // The amount of radians per turn of the motor

    override fun periodic() {
        if(autoTurret) {
            //need to fix auto aim function //boolean
            autoAim()
        }

        ActiveOpMode.telemetry.run {
            addData("goal", turretController.goal.position)
            addData("turret Pos", getYaw())
        }
    }

    private fun autoAim() {
        //create the values in LL
        val mu = atan2(goalY - currentY, goalX - currentX)
        val deltaHeading = normalizeAngle(imu - currentHeading)
        // create function to get current heading, delta heading and clamp heading
       val clampedHeading = deltaHeading.coerceIn(-PI, PI)
        //turretController.goal = KineticState(clampedHeading, 0.0)
        turret.power = turretController.calculate(KineticState(getYaw(), 0.0))
    }

    fun goToYaw(yaw:Double) { // Go to a specific position
        turretController.goal = KineticState(yaw, 0.0)
    }

    fun getYaw(): Double { // Get the current yaw of the turret from [-pi, pi]
        return normalizeAngle(turret.currentPosition * rpt)
    }


    fun normalizeAngle(angleRadians: Double): Double { // Returns a normalized angle between [-pi, pi]
        var angle = angleRadians % (2.0 * PI)
        if (angle <= -PI) {
            angle += 2.0 * PI
        }
        if (angle > PI) {
            angle -= 2.0 * PI
        }
        return angle
    }
}