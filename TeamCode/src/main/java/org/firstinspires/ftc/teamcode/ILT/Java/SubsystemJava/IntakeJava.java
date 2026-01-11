package org.firstinspires.ftc.teamcode.ILT.Java.SubsystemJava;

import com.bylazar.configurables.annotations.Configurable;
import dev.nextftc.core.commands.utility.InstantCommand;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.hardware.controllable.MotorGroup;
import dev.nextftc.hardware.impl.MotorEx;

@Configurable
public class IntakeJava implements Subsystem {
    public static final IntakeJava INSTANCE = new IntakeJava();
    // Here I declare two intake motors (left and right) and group them together.
    // The MotorGroup makes it easy to control both at once.
    public static final MotorEx iMR = new MotorEx("iMR");
    public static final MotorEx iML = new MotorEx("iML");
    public static final MotorGroup iM = new MotorGroup(iML, iMR);

    // Driver-requested intake power (what you WANT the intake to do)
    // iP is the power level we want to apply — positive sucks balls in, negative ejects them.
    public static double iP = 0.0;

    @Override
    public void periodic() {
        // Every loop, I just apply whatever power level the driver requested.
        // It's super simple — no PID or fancy control, just direct motor power.
        iM.setPower(iP);
    }

    // These InstantCommands are designed to be bound to gamepad buttons.
    // They're one-press actions to quickly change intake behavior.
    public static final InstantCommand runIntake = new InstantCommand(() -> iP = 1.0);

    public static final InstantCommand reverseIntake = new InstantCommand(() -> iP = -1.0);

    public static final InstantCommand reverseIntakeSlow = new InstantCommand(() -> iP = -0.5);

    public static final InstantCommand reverseIntakeVerySlow = new InstantCommand(() -> iP = -0.2);

    public static final InstantCommand stopIntake = new InstantCommand(() -> iP = 0.0);
}
