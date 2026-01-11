package org.firstinspires.ftc.teamcode.ILT.Java.SubsystemJava.OuttakeJava;

import dev.nextftc.core.commands.utility.InstantCommand;
import dev.nextftc.core.subsystems.SubsystemGroup;

import org.firstinspires.ftc.teamcode.ILT.Java.SubsystemJava.DriveTrainJava;
import org.firstinspires.ftc.teamcode.ILT.Java.SubsystemJava.OuttakeJava.ShooterJava.FlyWheelJava;
import org.firstinspires.ftc.teamcode.ILT.Java.SubsystemJava.OuttakeJava.ShooterJava.HoodJava;
import org.firstinspires.ftc.teamcode.ILT.Java.SubsystemJava.OuttakeJava.ShooterJava.TurretJava;
import org.firstinspires.ftc.teamcode.ILT.Next.Subsystem.Data.Aimbot;
import org.firstinspires.ftc.teamcode.ILT.Next.Subsystem.Data.Alliance;
import org.firstinspires.ftc.teamcode.ILT.Next.Subsystem.DriveTrain;
import org.firstinspires.ftc.teamcode.ILT.Next.Subsystem.Outtake.ImprovedOuttake;
import org.firstinspires.ftc.teamcode.ILT.Next.Subsystem.Outtake.Shooter.FlyWheel;
import org.firstinspires.ftc.teamcode.ILT.Next.Subsystem.Outtake.Shooter.Hood;
import org.firstinspires.ftc.teamcode.ILT.Next.Subsystem.Outtake.Shooter.Turret;
import org.firstinspires.ftc.teamcode.ILT.Next.Subsystem.Outtake.Shooter.Hood; // for hP, hS
import org.firstinspires.ftc.teamcode.ILT.Next.Subsystem.Outtake.Shooter.Turret; // for gP, turret
import org.firstinspires.ftc.teamcode.next.kotlin.subsystems.LLAutoVelo;


public class ImprovedOuttakeJava extends SubsystemGroup {
    public static final ImprovedOuttakeJava INSTANCE = new ImprovedOuttakeJava();

    // If fullManual is true, the driver does all aiming and shooting themselves.
    // When it’s false, we use the auto-aim and auto-settings logic below.
    public static boolean fullManual = false;

    // When true, system triggers automated shooting routine (when conditions are met).
    public static boolean autoShoot = false;

    // Field X-coordinate of the scoring goal; set dynamically based on alliance.
    public static double goalX = 0.0;
    // Field Y-coordinate of the scoring goal; constant across alliances in this setup.
    public static final double goalY = 144.0 - 8.0;

    // distance metrics
    public static double distM = 0.0;
    public static double distLL = 0.0;
    public static double distDiff = 0.0;

    // values[0] and values[1] are the recommended hood and velocity settings for that distance.
    public static double[] values = new double[2];

    // Manual aim state
    public static double targetVelo = 0.0;
    public static int manualAim = 0;
    public static final InstantCommand aimUp   = new InstantCommand(() -> manualAim += 12);
    public static final InstantCommand aimDown = new InstantCommand(() -> manualAim -= 12);
    public static boolean canSpin = true;


    // Initialize per-alliance goal position so the turret can auto-aim correctly.
    @Override
    public void initialize() {
        // In init, I decide where the goal is on the field depending on which alliance we’re on.
        // Red goal is on one side of the field, blue on the opposite.
        if (DriveTrain.alliance == Alliance.RED) {
            goalX = 144.0 - 6.0;   // Red alliance goal X
        } else {
            goalX = 6.0;           // Blue alliance goal X
        }
    }

    // Main loop: choose manual vs auto aiming and optionally perform auto-shoot.
    @Override
    public void periodic() {
        // Update distances every loop using current drivetrain pose and LL reading.
        distM = Math.sqrt(
                Math.pow(goalX - DriveTrainJava.currentX, 2) +
                        Math.pow(goalY - DriveTrainJava.currentY, 2)
        );

        distLL = LLAutoVelo.INSTANCE.getDistanceToGoal(); // Kotlin `!!` becomes non-null getter
        distDiff = distM - distLL;

        values = Aimbot.getValues(distLL);

        // Every loop, decide whether we’re in full manual or assisted mode.
        if (fullManual) {
            Turret.autoTurret = false;   // Disable auto aim when in full manual
            // manualAim();              // Call if you want manual routine active each loop
        } else {
            Turret.autoTurret = true;    // Enable turret auto-aim to track goal
            auto();                      // Auto hood/flywheel tuning based on distance
        }

        // If autoShoot is enabled, then run the auto shooting routine.
        if (autoShoot) {
            autoShoot();
            auto();
        }
    }

    public static void auto() {
        // In auto, let the Aimbot table pick a hood angle and flywheel velocity based on distance.
        // Then apply small offsets to tune shots on the real robot.
        HoodJava.updatePosition(values[0] + 0.06);  // Hood offset tweak
        FlyWheelJava.updatePid(values[1] + 100);    // Velocity bump for consistency
    }

    // Automated shooting sequence gate-kept by shoot zone check.
    public static void autoShoot() {
        // Only allow autoShoot to run if drivetrain says we’re inside a good shooting zone.
        if (DriveTrainJava.inShootZone()) {
            // Placeholder: spin up flywheel, set hood, align turret, and fire.
            // assuming Shoot is a @JvmStatic function; adjust if it's a Command
            FlyWheelJava.Shoot.run();
            HoodJava.updatePosition(values[0] + 0.06);  // Hood offset tweak
            FlyWheelJava.updatePid(values[1] + 100);
        }
    }

    // ManualAim
    public static void manualAim() {
        // TODO: Implement operator-driven aiming (e.g., stick inputs → Turret.goToYaw).
        // In full manual, use the aimDistance mapping for hood and velocity and drive actuators.
        if (fullManual) {
            aimDistance();
            HoodJava.hS.setPosition(Hood.hP);      // hS.position = hP
            TurretJava.turret.setPower(TurretJava.gP); // turret.power = gP
        }
        // Student work: wire to gamepad bindings and refine the mapping table.
    }

    public static void aimDistance() {
        if (canSpin) {
            switch (manualAim) {
                case 12:  targetVelo = 835.0;  break;
                case 24:  targetVelo = 862.0;  break;
                case 36:  targetVelo = 844.0;  break;
                case 48:  targetVelo = 848.0;  break;
                case 60:  targetVelo = 908.0;  break;
                case 72:  targetVelo = 1025.0; break;
                case 84:  targetVelo = 1165.0; break;
                case 96:  targetVelo = 1230.0; break;
                case 108: targetVelo = 1070.0; break;
                case 120: targetVelo = 1112.0; break;
                case 132: targetVelo = 1150.0; break;
                case 144: targetVelo = 1250.0; break;
                default:  targetVelo = 0.0;    break;
            }
        }

        // Hood servo position matching angle/distance
        switch (manualAim) {
            case 12:  Hood.hP = 0.81; break;
            case 24:  Hood.hP = 0.93; break;
            case 36:  Hood.hP = 0.71; break;
            case 48:  Hood.hP = 0.60; break;
            case 60:  Hood.hP = 0.62; break;
            case 72:  Hood.hP = 0.65; break;
            case 84:  Hood.hP = 0.70; break;
            case 96:  Hood.hP = 0.70; break;
            case 108: Hood.hP = 0.42; break;
            case 120: Hood.hP = 0.43; break;
            case 134: Hood.hP = 0.44; break;
            case 146: Hood.hP = 0.45; break;
            default:  Hood.hP = 0.0;  break;
        }

        // Limit aiming range
        if (manualAim > 146) manualAim = 146;
        else if (manualAim < 12) manualAim = 12;

        // Round to nearest 12-increment for valid lookup values
        if (manualAim % 12 != 0) manualAim -= manualAim % 12;
    }
}
