package org.firstinspires.ftc.teamcode.nextFtc.Subsystem.Shooter.TurretMech

import com.pedropathing.geometry.Pose
import dev.nextftc.core.commands.Command

class OdometryAim(
    private val goalX: Double,
    private val goalY: Double,
    private val poseX: () -> Double,
    private val poseY: () -> Double,
    private val poseH: () -> Double,
    private val ofsTurret: Double = 0.0
) : Command() {

    override val isDone = false

    override fun start() {
        Turret.currentState = Turret.State.AIMING
        Turret.registerCommand(this)
    }

    override fun update() {
        if (!Turret.hasRelocalized) return  // don't aim until pose is trusted
        val botPose = Pose(poseX(), poseY(), poseH() + ofsTurret)
        val targetPose = Pose(goalX, goalY)
        Turret.aimAt(targetPose, botPose)
    }
    override fun stop(interrupted: Boolean) {
        if (!interrupted) Turret.stop()
    }
}