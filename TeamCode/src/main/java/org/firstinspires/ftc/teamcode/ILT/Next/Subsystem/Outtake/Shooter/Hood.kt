package org.firstinspires.ftc.teamcode.ILT.Next.Subsystem.Outtake.Shooter

import com.bylazar.configurables.annotations.Configurable
import dev.nextftc.core.commands.utility.InstantCommand
import dev.nextftc.core.subsystems.Subsystem
import dev.nextftc.hardware.impl.ServoEx


@Configurable
// air sorting
// move hood
object Hood: Subsystem {
    // Servo driving the hood/flap that adjusts shot angle/trajectory
     val hS = ServoEx("flap")

    // Desired hood position (0.0–1.0). Tune based on distance and projectile velocity
    @JvmField var hP = 0.0
    val FlapDown = InstantCommand { hP += 0.05 }
    val FlapUp = InstantCommand { hP -= 0.05 }
    fun getHoodPosition(){
        // {{{{{{this is already done but the long way so someone can do it the short way by using math.}}}}}}
        // Intended: compute hood position from distance to target and launch velocity.
        // Steps typically include:
        // 1) Measure robot-to-goal distance (field coordinates or sensor).
        // 2) Use a mapping/model (lookup table or ballistic equation) to convert distance to servo angle.
        // 3) Convert angle to servo normalized position and assign to hoodPosition.
        // Note: Keep as pure function or update hoodPosition inside here once model exists.
    }

    override fun periodic() {
        // Apply the commanded hood position every loop to the servo
        hS.position = hP
    }

    fun updatePosition(position: Double) {
        // External setter to update desired hood position (e.g., from auto-aim or operator input)
        hP = position
    }
}