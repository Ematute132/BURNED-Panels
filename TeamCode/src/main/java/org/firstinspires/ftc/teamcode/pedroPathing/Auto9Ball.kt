package org.firstinspires.ftc.teamcode.next.kotlin

import com.bylazar.configurables.annotations.Configurable
import com.bylazar.telemetry.JoinedTelemetry
import com.bylazar.telemetry.PanelsTelemetry
import com.pedropathing.geometry.BezierLine
import com.pedropathing.geometry.Pose
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import dev.nextftc.core.commands.groups.ParallelGroup
import dev.nextftc.core.commands.groups.SequentialGroup
import dev.nextftc.core.commands.utility.InstantCommand
import dev.nextftc.core.components.BindingsComponent
import dev.nextftc.core.components.SubsystemComponent
import dev.nextftc.extensions.pedro.FollowPath
import dev.nextftc.extensions.pedro.PedroComponent
import dev.nextftc.extensions.pedro.PedroComponent.Companion.follower
import dev.nextftc.ftc.NextFTCOpMode
import dev.nextftc.ftc.components.BulkReadComponent
import org.firstinspires.ftc.teamcode.Old.java.kotlin.subsystems.Intake
import org.firstinspires.ftc.teamcode.Old.java.kotlin.subsystems.Outtake
import org.firstinspires.ftc.teamcode.next.kotlin.subsystems.Limelight.limeLight
import org.firstinspires.ftc.teamcode.next.subsystems.DriveTrain
import org.firstinspires.ftc.teamcode.pedroPathing.Constants
import java.lang.StrictMath.toRadians


@Autonomous(name = "Pedro Auto 3+3")
@Configurable
class PedroAutonomous : NextFTCOpMode() {

    private var tele = JoinedTelemetry(PanelsTelemetry.ftcTelemetry, telemetry)
    private lateinit var autoPath: AutoPath
    private var index = 0

    init {
        addComponents(
            SubsystemComponent(Intake, Outtake, DriveTrain, limeLight),
            PedroComponent(Constants::createFollower),
            BulkReadComponent,
            BindingsComponent
        )
    }

    override fun onInit() {
        follower.setStartingPose(Pose(21.084, 122.019, toRadians(143.0)))
        autoPath = AutoPath()

        tele.run {
            addLine("Status: Initialized")
            addLine("Ready to run 3+3 autonomous")
            update()
        }
    }

    override fun onStartButtonPressed() {
        // Schedule the first path
        autoPath.next().schedule()
    }

    override fun onUpdate() {
        follower.update()

        tele.run {
            addLine("Path Index: $index / ${autoPath.pathCount}")
            addLine("X: ${"%.2f".format(follower.pose.x)}")
            addLine("Y: ${"%.2f".format(follower.pose.y)}")
            addLine("Heading: ${"%.2f".format(Math.toDegrees(follower.pose.heading))}°")
            addLine()
            //addLine(Outtake.getTelemetryString())
            update()
        }
    }

    inner class AutoPath {

            // Poses for autonomous
            val start = Pose(21.084, 122.019, toRadians(143.0))
            val alignWithBalls = Pose(44.187, 98.916, toRadians(180.0))
            val firstSetStart = Pose(44.187, 83.888, toRadians(180.0))
            val firstSetEnd = Pose(16.150, 83.888, toRadians(180.0))
            val launchPoint = Pose(33.869, 109.009, toRadians(138.0))
            val secondSetStart = Pose(45.187, 60.336, toRadians(180.0))
            val secondSetEnd = Pose(19.074, 60.336, toRadians(180.0))
            val thirdSetStart = Pose(44.187, 35.664, toRadians(180.0))
            val thirdSetEnd = Pose(17.047, 35.664, toRadians(180.0))


        val pathCount = 7

        // Helper command to shoot with distance-based velocity
        private val shootWithDistance = InstantCommand {
            val distance = limeLight.getDistanceToTarget()
            if (distance != null && distance > 0) {
                //Outtake.setVelocityForDistance(distance)
            } else {
                Outtake.flywheelOn.run()
            }
        }

        // Path sequences
        val startToLaunch = SequentialGroup(
            ParallelGroup(
                Intake.reverseIntakeSlow, // Clear clogged balls
                shootWithDistance, // Prep flywheel
                FollowPath(
                    follower.pathBuilder()
                        .addPath(BezierLine(start, alignWithBalls))
                        .addPath(BezierLine(alignWithBalls, launchPoint))
                        .setLinearHeadingInterpolation(toRadians(143.0), toRadians(138.0))
                        .build()
                )
            ),
            Intake.reverseIntakeSlow // Shoot once at launch point
        )

        val launchToFirstSet = SequentialGroup(
            Outtake.flywheelOff,
            Intake.runIntake, // Start intaking
            FollowPath(
                follower.pathBuilder()
                    .addPath(BezierLine(launchPoint, firstSetStart))
                    .setLinearHeadingInterpolation(toRadians(138.0), toRadians(180.0))
                    .build()
            )
        )

        val firstSetIntakeToLaunch = SequentialGroup(
            ParallelGroup(
                Intake.reverseIntakeSlow, // Clear clogged balls
                shootWithDistance, // Prep flywheel
                FollowPath(
                    follower.pathBuilder()
                        .addPath(BezierLine(firstSetStart, firstSetEnd))
                        .addPath(BezierLine(firstSetEnd, launchPoint))
                        .setTangentHeadingInterpolation()
                        .build()
                )
            ),
            Intake.reverseIntakeSlow // Shoot once at launch point
        )

        val launchToSecondSet = SequentialGroup(
            Outtake.flywheelOff,
            Intake.runIntake, // Start intaking
            FollowPath(
                follower.pathBuilder()
                    .addPath(BezierLine(launchPoint, secondSetStart))
                    .setLinearHeadingInterpolation(toRadians(138.0), toRadians(180.0))
                    .build()
            )
        )

        val secondSetIntakeToLaunch = SequentialGroup(
            ParallelGroup(
                Intake.reverseIntakeSlow, // Clear clogged balls
                shootWithDistance, // Prep flywheel
                FollowPath(
                    follower.pathBuilder()
                        .addPath(BezierLine(secondSetStart, secondSetEnd))
                        .addPath(BezierLine(secondSetEnd, launchPoint))
                        .setTangentHeadingInterpolation()
                        .build()
                )
            ),
            Intake.reverseIntakeSlow // Shoot once at launch point
        )

        val launchToThirdSet = SequentialGroup(
            Outtake.flywheelOff,
            Intake.runIntake, // Start intaking
            FollowPath(
                follower.pathBuilder()
                    .addPath(BezierLine(launchPoint, thirdSetStart))
                    .setLinearHeadingInterpolation(toRadians(138.0), toRadians(180.0))
                    .build()
            )
        )

        val thirdSetIntakeToLaunch = SequentialGroup(
            ParallelGroup(
                Intake.reverseIntakeSlow, // Clear clogged balls
                shootWithDistance, // Prep flywheel
                FollowPath(
                    follower.pathBuilder()
                        .addPath(BezierLine(thirdSetStart, thirdSetEnd))
                        .addPath(BezierLine(thirdSetEnd, launchPoint))
                        .setTangentHeadingInterpolation()
                        .build()
                )
            ),
            Intake.reverseIntakeSlow, // Shoot once at launch point
            Outtake.flywheelOff,
            Intake.stopIntake
        )

        fun next(): SequentialGroup {
            return when(index++) {
                0 -> startToLaunch
                1 -> launchToFirstSet
                2 -> firstSetIntakeToLaunch
                3 -> launchToSecondSet
                4 -> secondSetIntakeToLaunch
                5 -> launchToThirdSet
                6 -> thirdSetIntakeToLaunch
                else -> SequentialGroup() // Done
            }
        }
    }
}