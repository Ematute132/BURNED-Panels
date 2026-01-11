package org.firstinspires.ftc.teamcode.ILT.Java.SubsystemJava.OuttakeJava.ShooterJava;

import com.bylazar.configurables.annotations.Configurable;
import dev.nextftc.core.commands.utility.InstantCommand;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.hardware.impl.ServoEx;

@Configurable
public class HoodJava implements Subsystem {
    // Servo driving the hood/flap that adjusts shot angle/trajectory
    public static final ServoEx hS = new ServoEx("flap");

    // Desired hood position (0.0–1.0). Tune based on distance and projectile velocity
    public static double hP = 0.0;

    public static final InstantCommand FlapDown = new InstantCommand(() -> hP += 0.05);
    public static final InstantCommand FlapUp   = new InstantCommand(() -> hP -= 0.05);

    public static void getHoodPosition() {
        // {{{{{{this is already done but the long way so someone can do it the short way by using math.}}}}}}
        // Intended: compute hood position from distance to target and launch velocity.
        // Steps typically include:
        // 1) Measure robot-to-goal distance (field coordinates or sensor).
        // 2) Use a mapping/model (lookup table or ballistic equation) to convert distance to servo angle.
        // 3) Convert angle to servo normalized position and assign to hP.
        // Note: Keep as pure function or update hP inside here once model exists.
    }

    @Override
    public void periodic() {
        // Apply the commanded hood position every loop to the servo
        hS.setPosition(hP);
    }

    public static void updatePosition(double position) {
        // External setter to update desired hood position (e.g., from auto-aim or operator input)
        hP = position;
    }
}
