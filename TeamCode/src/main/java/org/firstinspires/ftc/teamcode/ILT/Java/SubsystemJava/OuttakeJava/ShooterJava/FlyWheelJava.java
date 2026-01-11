package org.firstinspires.ftc.teamcode.ILT.Java.SubsystemJava.OuttakeJava.ShooterJava;

import dev.nextftc.control.KineticState;
import dev.nextftc.control.builder.ControlSystemBuilderKt;
import dev.nextftc.control.feedback.PIDCoefficients;
import dev.nextftc.control.feedforward.BasicFeedforwardParameters;
import dev.nextftc.core.commands.delays.Delay;
import dev.nextftc.core.commands.groups.SequentialGroup;
import dev.nextftc.core.commands.utility.InstantCommand;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.ftc.ActiveOpMode;
import dev.nextftc.hardware.impl.MotorEx;

import org.firstinspires.ftc.teamcode.ILT.Java.SubsystemJava.IntakeJava;
import org.firstinspires.ftc.teamcode.ILT.Next.Subsystem.Intake;

public class FlyWheelJava implements Subsystem {
    // Primary flywheel motor.
    public static final MotorEx f1 = new MotorEx("f1M");

    // Secondary flywheel motor, reversed to match mechanical orientation.
    public static final MotorEx f2 = new MotorEx("f2M").reversed();

    // Velocity PID coefficients (tune for your drivetrain and inertia).
    public static PIDCoefficients flywheelPID = new PIDCoefficients(0.0033, 0.0, 0.0);

    // Basic feedforward parameters: kV (per-tick), kA, kS (static). Tune to reduce error and improve spin-up.
    public static BasicFeedforwardParameters flywheelFF =
            new BasicFeedforwardParameters(1.66667E-4, 0.0, 0.003);

    // Controller combining velocity PID and feedforward for stable target tracking.
    public static final dev.nextftc.control.ControlSystem flywheelController =
            ControlSystemBuilderKt.controlSystem(builder -> {
                builder.velPid(flywheelPID);   // Use velocity PID loop
                builder.basicFF(flywheelFF);   // Add simple feedforward model
                return null;
            });

    // Desired wheel linear velocity in ticks/sec (controller uses this as goal velocity).
    public static double targetVelocity = 1500.0;
    // have to change target velo to the LL
    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

    // On/off state for flywheels; when off, the controller targets zero velocity.
    public static boolean flywheelsOn = false;

    // Convenience metric for driver feedback: estimated motor RPM.
    public static double motorRpm = 0.0;

    // Command: enable flywheels (controller will pursue targetVelocity).
    public static final InstantCommand spin = new InstantCommand(() -> flywheelsOn = true);

    // Command: disable flywheels (controller will target zero).
    public static final InstantCommand stop = new InstantCommand(() -> flywheelsOn = false);

    // Command: briefly reverse to clear jams; schedules stop first, then sets negative power.
    public static final InstantCommand backOutSlow = new InstantCommand(() -> {
        stop.schedule();      // Ensure controller is not trying to maintain positive velocity.
        f1.setPower(-0.5);    // Manual reverse power on primary motor.
        f2.setPower(f1.getPower()); // Mirror reverse power on secondary motor.
    });

    public static final InstantCommand backOut = new InstantCommand(() -> {
        stop.schedule();      // Ensure controller is not trying to maintain positive velocity.
        f1.setPower(-1.0);    // Manual reverse power on primary motor.
        f2.setPower(f1.getPower()); // Mirror reverse power on secondary motor.
    });

    // SequentialCommand to automate shooting sequence
    // might need to change the seconds
    public static final SequentialGroup Shoot = new SequentialGroup(
            spin,
            new Delay(0.1),          // 0.1.seconds in Kotlin → 0.1 seconds double in Java
            IntakeJava.runIntake,
            new Delay(0.5),          // 0.5.seconds
            stop
    );

    @Override
    public void periodic() {
        // Convert measured motor velocity (ticks/sec) to RPM; 60 sec/min divided by 28 ticks per motor rev.
        motorRpm = f1.getVelocity() * 60.0 / 28.0;

        // Compute power from controller using current measured motor state.
        double power = flywheelController.calculate(f1.getState());
        f1.setPower(power);
        // Mirror power to second motor to keep both wheels synchronized.
        f2.setPower(power);

        // Update controller goal based on flywheelsOn flag.
        if (flywheelsOn) {
            // Track the desired target velocity when enabled.
            flywheelController.setGoal(new KineticState(0.0, targetVelocity));
        } else {
            // Stop the wheels by commanding zero velocity.
            flywheelController.setGoal(new KineticState(0.0, 0.0));
        }

        // Driver-station telemetry: show targets and actuals for tuning and match awareness.
        ActiveOpMode.telemetry().addData("targetVelo", targetVelocity);                 // Controller target (ticks/sec)
        ActiveOpMode.telemetry().addData("Current RPM", motorRpm);                     // Estimated actual RPM
        ActiveOpMode.telemetry().addData("RPM target", targetVelocity * 60.0 / 28.0);  // Target expressed in RPM
        ActiveOpMode.telemetry().addData("flywheel goal", flywheelController.getGoal());// Full KineticState goal
        ActiveOpMode.telemetry().update();

    }

    // External API to set a new velocity target (ticks/sec). Does not auto-enable the wheels.
    public static void updatePid(double velocity) {
        targetVelocity = velocity;
    }
}
