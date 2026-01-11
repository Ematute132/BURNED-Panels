package org.firstinspires.ftc.teamcode.ILT.Next.Subsystem.Outtake

import dev.nextftc.core.commands.utility.InstantCommand
import dev.nextftc.core.subsystems.SubsystemGroup
import org.firstinspires.ftc.teamcode.ILT.Next.Subsystem.Data.Aimbot
import org.firstinspires.ftc.teamcode.ILT.Next.Subsystem.Data.Alliance
import org.firstinspires.ftc.teamcode.ILT.Next.Subsystem.Outtake.Shooter.FlyWheel
import org.firstinspires.ftc.teamcode.ILT.Next.Subsystem.Outtake.Shooter.Hood
import org.firstinspires.ftc.teamcode.ILT.Next.Subsystem.Outtake.Shooter.Turret
import org.firstinspires.ftc.teamcode.ILT.Next.Subsystem.DriveTrain
import org.firstinspires.ftc.teamcode.ILT.Next.Subsystem.DriveTrain.currentX
import org.firstinspires.ftc.teamcode.ILT.Next.Subsystem.DriveTrain.currentY
import org.firstinspires.ftc.teamcode.ILT.Next.Subsystem.Outtake.Shooter.Hood.hP
import org.firstinspires.ftc.teamcode.ILT.Next.Subsystem.Outtake.Shooter.Hood.hS
import org.firstinspires.ftc.teamcode.ILT.Next.Subsystem.Outtake.Shooter.Turret.gP
import org.firstinspires.ftc.teamcode.ILT.Next.Subsystem.Outtake.Shooter.Turret.turret
import org.firstinspires.ftc.teamcode.next.kotlin.subsystems.LLAutoVelo
import kotlin.math.pow
import kotlin.math.sqrt


// Here I’m creating a SubsystemGroup which bundles together the FlyWheel, Hood, and Turret.
// This lets me treat the whole outtake as one logical unit while still keeping each piece separate.
object ImprovedOuttake: SubsystemGroup(FlyWheel, Hood, Turret){

    // If fullManual is true, the driver does all aiming and shooting themselves.
    // When it’s false, we use the auto-aim and auto-settings logic below.
    @JvmField var fullManual = false

    // When true, system triggers automated shooting routine (when conditions are met).
    @JvmField var autoShoot = false


    // Field X-coordinate of the scoring goal; set dynamically based on alliance.
    var goalX = 0.0
    // Field Y-coordinate of the scoring goal; constant across alliances in this setup.
    val goalY = 144-8.0


    // Initialize per-alliance goal position so the turret can auto-aim correctly.
    override fun initialize() {
        // In init, I decide where the goal is on the field depending on which alliance we’re on.
        // Red goal is on one side of the field, blue on the opposite.
        goalX = if (DriveTrain.alliance == Alliance.RED) {
            144-6.0   // Red alliance goal X
        } else {
            6.0       // Blue alliance goal X
        }
    }

    // Main loop: choose manual vs auto aiming and optionally perform auto-shoot.
    override fun periodic() {
        // Every loop, I first decide whether we’re in full manual or assisted mode.
        if (fullManual) {
            Turret.autoTurret = false   // Disable auto aim when in full manual
            // manualAim()                 // Placeholder for operator-controlled aiming
        } else {
            Turret.autoTurret = true    // Enable turret auto-aim to track goal
            auto()                     // Optional: run auto hood/flywheel tuning based on distance
        }

        // If autoShoot is enabled, then we also run the auto shooting routine.
        if(autoShoot) {
            autoShoot()
            auto()
        }
    }



    // Auto Functions that i need to do
    // - Apply minor offsets for calibration

    //get distance Manually
    // Here I calculate the distance to the goal using the robot’s field position (from DriveTrain).
    // It uses the Pythagorean theorem on the difference between robot position and goal position.
    val distM: Double = sqrt((goalX-currentX).pow(2) + (goalY-currentY).pow(2))

    // get distance with LL
    // This one uses the distance estimated by the Limelight + physics subsystem (LLAutoVelo).
    // Because of the !!, I’m assuming distanceToGoal is never null once things are running.
    val distLL: Double = LLAutoVelo.distanceToGoal!!

    //difference in distance
    // This is just comparing the manual field distance vs the Limelight-estimated distance.
    // It could be used for calibration or debugging.
    val distDiff = distM - distLL

    // values[0] and values[1] are the recommended hood and velocity settings for that distance.
    val values: DoubleArray = Aimbot.getValues(distLL)

    fun auto() {
        // In auto, I let the Aimbot table pick a hood angle and flywheel velocity based on distance.
        // Then I apply small offsets to tune shots on the real robot.

        Hood.updatePosition(values[0] + 0.06)  // Hood offset tweak
        FlyWheel.updatePid(values[1] + 100)    // Velocity bump for consistency
    }


    // Automated shooting sequence gate-kept by shoot zone check.
    fun autoShoot() {
        // Here I only allow autoShoot to run if our drivetrain says we’re inside a good shooting zone.
        if(DriveTrain.inShootZone()) {
            // TODO: Build command chain to spin up flywheels, set hood, align turret, and fire.
            // This can be student work; keep API simple and testable.

            // Right now this is mostly a placeholder: it triggers the flywheel "Shoot" routine
            // and re-applies our auto hood and velocity settings.
            FlyWheel.Shoot
            Hood.updatePosition(values[0] + 0.06)  // Hood offset tweak
            FlyWheel.updatePid(values[1] + 100)
        }
    }
     //ManualAim

    var targetVelo = 0.0
   var manualAim = 0
    val aimUp = InstantCommand { manualAim += 12 }
    val aimDown = InstantCommand { manualAim -= 12 }
    @JvmField var canSpin = true


    fun manualAim() {
        // TODO: Implement operator-driven aiming (e.g., stick inputs → Turret.goToYaw).
        // This is left for student work; integrate with NextFTC command bindings.
          if (fullManual) {
                   aimDistance()
                  hS.position = hP
                  turret.power = gP
              }
             // Mr barrera this is smth a student could do
              // note to self talk aobut the aim distance function that is just hard values
              // helpful code that i wrote



    }
    fun aimDistance() {
        if (canSpin) {
            when (manualAim) {
                12 -> targetVelo = 835.0
                24 -> targetVelo = 862.0
                36 -> targetVelo = 844.0
                48 -> targetVelo = 848.0
                60 -> targetVelo = 908.0
                72 -> targetVelo = 1025.0
                84 -> targetVelo = 1165.0
                96 -> targetVelo = 1230.0
                108 -> targetVelo = 1070.0
                120 -> targetVelo = 1112.0
                132 -> targetVelo = 1150.0
                144 -> targetVelo = 1250.0
                else -> targetVelo = 0.0
            }
        }

        // Hood servo position matching angle/distance
        when (manualAim) {
            12 -> hP = 0.81
            24 -> hP = 0.93
            36 -> hP = 0.71
            48 -> hP = 0.6
            60 -> hP = 0.62
            72 -> hP = 0.65
            84 -> hP = 0.7
            96 -> hP = 0.7
            108 -> hP = 0.42
            120 -> hP = 0.43
            134 -> hP = 0.44
            146 -> hP = 0.45
            else -> hP = 0.0
        }

        // Limit aiming range
        if (manualAim > 146) manualAim = 146
        else if (manualAim < 12) manualAim = 12

        // Round to nearest 12-increment for valid lookup values
        if (manualAim % 12 != 0) manualAim -= manualAim % 12
    }


}
