@file:Suppress("PackageName")

package org.firstinspires.ftc.teamcode.Util

import com.pedropathing.geometry.Pose
import dev.nextftc.extensions.pedro.PedroComponent
import dev.nextftc.extensions.pedro.PedroComponent.Companion.follower
import kotlin.math.PI
import kotlin.math.cos
import kotlin.math.sin

data object ROBOT {
    private const val FIELD_WIDTH: Double = 141.5
    private const val FIELD_LENGTH: Double = 141.5
    private const val TURRET_Y_OFFSET: Double = -1.774
    private const val FAR_ZONE_BUFFER: Double = 12.0
    private const val CLOSE_ZONE_BUFFER: Double = 11.0

    internal var currAlliance: Alliance = Alliance.BLUE
    internal var currStage: Stage = Stage.TELEOP
    internal var currTeleOpStartPose: Pose = Pose(79.0,7.5,PI/2)

    internal fun correctedPose(lVScalar: Double, angVScalar: Double): Pose {
        val v = follower.velocity.times(lVScalar)
        return (shooterPose() + Pose(v.xComponent, v.yComponent, follower.heading * angVScalar))
    }

    internal fun shooterPose(): Pose {
        val a: Double = PedroComponent.Companion.follower.heading
        return PedroComponent.Companion.follower.pose + Pose(
            TURRET_Y_OFFSET * cos(a),
            TURRET_Y_OFFSET * sin(a)
        )
    }

    internal fun getDistanceFromGoal(): Double = shooterPose().distanceFrom(currAlliance.goalPoses.flywheelGoalPose)
    internal fun inFarZone(): Boolean {
        val p: Pose = shooterPose()
        return ((p.y >= 0.0) && (p.y < (-p.x + (FIELD_LENGTH * (2 / 3.0)) + FAR_ZONE_BUFFER)) && (p.y < (p.x - FIELD_LENGTH / 3.0 + FAR_ZONE_BUFFER)))
    }
    internal fun inCloseZone(): Boolean {
        val p: Pose = shooterPose()
        return ((p.y <= FIELD_LENGTH) && (p.y > (-p.x + FIELD_LENGTH - CLOSE_ZONE_BUFFER)) && (p.y > (p.x - CLOSE_ZONE_BUFFER)))
    }
}

enum class Alliance {
    BLUE {
        override val resetPoses: ResetPoses  = ResetPoses(Pose(9.5,8.9,-PI).mirror(), Pose(117.0,129.0,-2.448).mirror(), Pose(116.0,130.0,-0.939).mirror())
        override val goalPoses: GoalPoses = GoalPoses(Pose(141.0,140.0).mirror(),Pose(141.0,140.0).mirror(),Pose(140.0,140.0).mirror())
    },
    RED{
        override val resetPoses: ResetPoses  = ResetPoses(Pose(9.5,8.9,-PI), Pose(117.0,129.0,-2.448), Pose(116.0,130.0,-0.939))
        override val goalPoses: GoalPoses = GoalPoses(Pose(136.0,140.0),Pose(136.0,140.0),Pose(132.0,140.0))
    };
    abstract val resetPoses: ResetPoses
    abstract val goalPoses: GoalPoses
}

enum class Motif {
    UNKNOWN { override val tagID: Int = 0 },
    PPG { override val tagID: Int = 23 },
    PGP { override val tagID: Int = 22 },
    GPP { override val tagID: Int = 21 };
    abstract val tagID: Int
}

enum class Stage {
    TELEOP,
    AUTONOMOUS;
    open var currMotif: Motif = Motif.UNKNOWN
    open var useFlywheelVel: Boolean = false
}

data class GoalPoses(val turretGoalPoseFar: Pose, val turretGoalPoseClose: Pose, val flywheelGoalPose: Pose)

data class ResetPoses(val resetPose1: Pose, val resetPose2: Pose, val resetPose3: Pose)