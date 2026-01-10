package org.firstinspires.ftc.teamcode.ILT.Next.Subsystem.Outtake

import dev.nextftc.core.subsystems.SubsystemGroup
import org.firstinspires.ftc.teamcode.ILT.Next.Subsystem.Data.Aimbot
import org.firstinspires.ftc.teamcode.ILT.Next.Subsystem.Data.Alliance
import org.firstinspires.ftc.teamcode.ILT.Next.Subsystem.Outtake.Shooter.FlyWheel
import org.firstinspires.ftc.teamcode.ILT.Next.Subsystem.Outtake.Shooter.Hood
import org.firstinspires.ftc.teamcode.ILT.Next.Subsystem.Outtake.Shooter.Turret
import org.firstinspires.ftc.teamcode.ILT.Next.Subsystem.DriveTrain
import org.firstinspires.ftc.teamcode.ILT.Next.Subsystem.DriveTrain.currentX
import org.firstinspires.ftc.teamcode.ILT.Next.Subsystem.DriveTrain.currentY
import kotlin.math.pow
import kotlin.math.sqrt


object ImprovedOuttake: SubsystemGroup(FlyWheel, Hood, Turret){

    // When true, operator fully controls aiming; disables auto turret aiming.
    @JvmField var fullManual = false
    // When true, system triggers automated shooting routine (when conditions are met).
    @JvmField var autoShoot = false

    // Field X-coordinate of the scoring goal; set dynamically based on alliance.
    var goalX = 0.0
    // Field Y-coordinate of the scoring goal; constant across alliances in this setup.
    val goalY = 144-8.0

    // Initialize per-alliance goal position so the turret can auto-aim correctly.
    override fun initialize() {
        goalX = if (DriveTrain.alliance == Alliance.RED) {
            144-6.0   // Red alliance goal X
        } else {
            6.0       // Blue alliance goal X
        }
    }

    // Main loop: choose manual vs auto aiming and optionally perform auto-shoot.
    override fun periodic() {
        if (fullManual) {
            Turret.autoTurret = false   // Disable auto aim when in full manual
            manualAim()                 // Placeholder for operator-controlled aiming
        } else {
            Turret.autoTurret = true    // Enable turret auto-aim to track goal
            //auto()                     // Optional: run auto hood/flywheel tuning based on distance
        }

        if(autoShoot) {
            autoShoot()                 // Attempt automated firing logic if enabled
        }
    }

    // Manual Aim
    fun manualAim() {
        // TODO: Implement operator-driven aiming (e.g., stick inputs → Turret.goToYaw).
        // This is left for student work; integrate with NextFTC command bindings.
    }

    // Auto Functions that i need to do

    // Example auto routine:
    // - Compute distance to goal from robot pose
    // - Query a model/lookup to get hood angle and wheel velocity
    // - Apply minor offsets for calibration


    fun auto() {
        val dist: Double = sqrt((goalX-currentX).pow(2) + (goalY-currentY).pow(2))
        val values: DoubleArray = Aimbot.getValues(dist)

        Hood.updatePosition(values[0] + 0.06)  // Hood offset tweak
        FlyWheel.updatePid(values[1] + 100)    // Velocity bump for consistency
    }

    // Automated shooting sequence gate-kept by shoot zone check.
    fun autoShoot() {
        if(DriveTrain.inShootZone()) {
            // TODO: Build command chain to spin up flywheels, set hood, align turret, and fire.
            // Prefer NextFTC commands: schedule spin, wait for velocity within tolerance, actuate feeder.
            // This can be student work; keep API simple and testable.
        }
    }
}