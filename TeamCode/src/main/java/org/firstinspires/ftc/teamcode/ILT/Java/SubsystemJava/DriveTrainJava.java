package org.firstinspires.ftc.teamcode.ILT.Java.SubsystemJava;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.geometry.Pose;
import dev.nextftc.core.commands.Command;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.extensions.pedro.PedroComponent;
import dev.nextftc.ftc.Gamepads;
import dev.nextftc.hardware.driving.FieldCentric;
import dev.nextftc.hardware.driving.MecanumDriverControlled;
import dev.nextftc.hardware.impl.Direction;
import dev.nextftc.hardware.impl.IMUEx;
import dev.nextftc.hardware.impl.MotorEx;
import org.firstinspires.ftc.teamcode.ILT.Next.Subsystem.Data.Alliance;

@Configurable
public class DriveTrainJava implements Subsystem {
    public static final DriveTrainJava INSTANCE = new DriveTrainJava();
    // Here I declare the four mecanum motors and the IMU (gyro) for heading awareness.
    public static final MotorEx fL = new MotorEx("fl");
    public static final MotorEx fR = new MotorEx("fr");
    public static final MotorEx bL = new MotorEx("bl");
    public static final MotorEx bR = new MotorEx("br");
    public static final IMUEx imu = new IMUEx("imu", Direction.RIGHT, Direction.UP);

    // alliance tells the robot which side of the field we’re on (affects goal targeting).
    public static Alliance alliance = Alliance.RED;

    // sensitivity scales the driver’s joystick inputs (useful for precision driving).
    // lower sensitivity means robot drives slower
    public static double sensitivity = 1.0;

    // These three track the robot’s current position and heading on the field.
    public static double currentX = 0.0;
    public static double currentY = 0.0;
    public static double currentHeading = 0.0;

    // HEADING LOCK STATE
    // These variables control the “heading lock” feature, where the robot holds a specific angle.
    private static Double targetHeadingLock = null;
    private static boolean headingLockActive = false;

    // This is the default driving command that runs when no other commands override it.
    @Override
    public Command getDefaultCommand() {
        return new MecanumDriverControlled(
                fL, fR, bL, bR,
                Gamepads.gamepad1().leftStickY().map(it -> -it * sensitivity), // Forward/back with sensitivity
                Gamepads.gamepad1().leftStickX().map(it -> it * sensitivity),  // Strafe left/right
                Gamepads.gamepad1().rightStickX().map(it -> it * sensitivity), // Rotate
                new FieldCentric(imu) // Makes driving relative to the field orientation using the IMU
        );
    }

    @Override
    public void periodic() {
        // Every loop, I update the robot’s position and heading from the path follower.
        currentX = PedroComponent.Companion.follower().getPose().getX();
        currentY = PedroComponent.Companion.follower().getPose().getY();
        currentHeading = PedroComponent.Companion.follower().getHeading();

        // HEADING LOCK - Direct motor control
        if (headingLockActive && targetHeadingLock != null) {
            // If heading lock is active, I calculate how far off we are from the target angle.
            double targetHeading = targetHeadingLock;
            double headingError = normalizeAngle(targetHeading - currentHeading);
            double turnPower = headingError * 0.4;

            // Clamp power to [-0.8, 0.8].
            if (turnPower > 0.8) turnPower = 0.8;
            if (turnPower < -0.8) turnPower = -0.8;

            // MECHANUM TURN: Opposite motors for rotation
            // This applies rotation power directly to the motors to hold the heading.
            // Front-left and back-left spin one way, front-right and back-right spin the opposite.
            fL.setPower(-turnPower);
            fR.setPower(turnPower);
            bL.setPower(-turnPower);
            bR.setPower(turnPower);
        }
        // defaultCommand handles manual drive when no heading lock
    }

    // LLAutoTurn INTERFACE
    // These functions let other subsystems (like turret auto-aim) control the robot’s heading.
    public static void setTargetHeading(double target) {
        // This gets called when we want the robot to point at a specific angle.
        // For example, for shooting, the turret might want the robot to face the goal.
        targetHeadingLock = target;
        headingLockActive = true;
    }

    public static void clearTargetHeading() {
        // This releases the heading lock and stops all motors.
        targetHeadingLock = null;
        headingLockActive = false;
        // Stop motors when releasing
        fL.setPower(0.0);
        fR.setPower(0.0);
        bL.setPower(0.0);
        bR.setPower(0.0);
    }

    // This helper function converts any angle into the range [-π, π] (standard robotics convention).
    private static double normalizeAngle(double angle) {
        double normalized = angle;
        while (normalized > Math.PI) normalized -= 2 * Math.PI;
        while (normalized < -Math.PI) normalized += 2 * Math.PI;
        return normalized;
    }

    // This function checks if a point (Pose) is inside a triangle defined by three other points.
    // It uses barycentric coordinates — a math trick to test if a point is inside a triangle.
    public static boolean PoseInTriangle(Pose p, Pose a, Pose b, Pose c) {
        double det =
                (b.getY() - c.getY()) * (a.getX() - c.getX()) +
                        (c.getX() - b.getX()) * (a.getY() - c.getY());
        if (Math.abs(det) < 1e-6) return false;

        double u =
                ((b.getY() - c.getY()) * (p.getX() - c.getX()) +
                        (c.getX() - b.getX()) * (p.getY() - c.getY())) / det;
        double v =
                ((c.getY() - a.getY()) * (p.getX() - c.getX()) +
                        (a.getX() - c.getX()) * (p.getY() - c.getY())) / det;
        double w = 1 - u - v;


        return u >= 0 && v >= 0 && w >= 0;
    }

    // This checks if the robot is in a safe shooting zone.
    // It models the field with triangles for safe zones and obstacles, then checks if any corner of the robot overlaps.
    public static boolean inShootZone() {
        // These define the triangles for the upper safe zone, lower safe zone, and obstacle zone.
        Pose[] obstacle = new Pose[] {
                new Pose(0.0, 115.0),
                new Pose(25.0, 144.0),
                new Pose(0.0, 141.0)
        };
        Pose[] upper = new Pose[] {
                new Pose(0.0, 115.0),
                new Pose(25.0, 144.0),
                new Pose(72.0, 72.0)
        };
        Pose[] lower = new Pose[] {
                new Pose(48.0, 0.0),
                new Pose(72.0, 24.0),
                new Pose(72.0, 0.0)
        };

        // The robot’s bounding box corners (assuming it’s about 13" x 13").
        double hw = 13.0 / 2.0;
        double hl = 13.0 / 2.0;
        Pose[] corners = new Pose[] {
                new Pose(currentX - hw, currentY - hl),
                new Pose(currentX + hw, currentY - hl),
                new Pose(currentX + hw, currentY + hl),
                new Pose(currentX - hw, currentY + hl)
        };

        // Helper function to check if any robot corner is inside a triangle.
        java.util.function.Function<Pose[], Boolean> overlaps = tri -> {
            for (Pose corner : corners) {
                if (PoseInTriangle(corner, tri[0], tri[1], tri[2])) {
                    return true;
                }
            }
            return false;
        };

        // We’re good to shoot if we’re in an upper OR lower safe zone AND NOT in the obstacle zone.
        boolean inUpper = overlaps.apply(upper);
        boolean inLower = overlaps.apply(lower);
        boolean inObstacle = overlaps.apply(obstacle);

        return (inUpper || inLower) && !inObstacle;
    }
}
