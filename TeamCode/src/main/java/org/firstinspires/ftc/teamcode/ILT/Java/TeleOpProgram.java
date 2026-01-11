package org.firstinspires.ftc.teamcode.ILT.Java;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import dev.nextftc.core.commands.groups.SequentialGroup;
import dev.nextftc.core.commands.utility.InstantCommand;
import dev.nextftc.core.components.BindingsComponent;
import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.extensions.pedro.PedroComponent;
import dev.nextftc.ftc.Gamepads;
import dev.nextftc.ftc.NextFTCOpMode;
import dev.nextftc.ftc.components.BulkReadComponent;

import org.firstinspires.ftc.teamcode.ILT.Java.SubsystemJava.DriveTrainJava;
import org.firstinspires.ftc.teamcode.ILT.Java.SubsystemJava.OuttakeJava.ImprovedOuttakeJava;
import org.firstinspires.ftc.teamcode.ILT.Java.SubsystemJava.OuttakeJava.ShooterJava.FlyWheelJava;
import org.firstinspires.ftc.teamcode.ILT.Java.SubsystemJava.OuttakeJava.ShooterJava.HoodJava;
import org.firstinspires.ftc.teamcode.ILT.Java.SubsystemJava.OuttakeJava.ShooterJava.TurretJava;

import org.firstinspires.ftc.teamcode.ILT.Java.SubsystemJava.IntakeJava;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@TeleOp(name = "First Try")
public class TeleOpProgram extends NextFTCOpMode {

    public TeleOpProgram() {
        addComponents(
                new PedroComponent(Constants::createFollower),
                new SubsystemComponent(IntakeJava.INSTANCE, ImprovedOuttakeJava.INSTANCE, DriveTrainJava.INSTANCE),
                BulkReadComponent.INSTANCE,
                BindingsComponent.INSTANCE
        );
    }

    @Override
    public void onInit() {
        // when (DriveTrain.INSTANCE.alliance) {
        //     // got to add auto
        //     // Alliance.RED -> PedroComponent.follower.setStartingPose(Far12.park)
        //     // Alliance.BLUE -> PedroComponent.follower.setStartingPose(Far12.park.mirror())
        // }
    }

    @Override
    public void onStartButtonPressed() {

        // Intake Controls
        Gamepads.gamepad1().rightTrigger().greaterThan(0.3)
                .whenBecomesTrue(IntakeJava.INSTANCE.runIntake)
                .whenBecomesFalse(IntakeJava.INSTANCE.stopIntake);

        Gamepads.gamepad1().leftTrigger().greaterThan(0.3)
                .whenBecomesTrue(IntakeJava.reverseIntake)
                .whenBecomesFalse(IntakeJava.stopIntake);

        // Gamepad 2 - Outtake aim
        Gamepads.gamepad2().rightTrigger().greaterThan(0.3)
                .whenBecomesTrue(ImprovedOuttakeJava.aimUp);

        Gamepads.gamepad2().leftTrigger().greaterThan(0.3)
                .whenBecomesTrue(ImprovedOuttakeJava.aimDown);

        // Flywheel Controls
        Gamepads.gamepad2().a()
                .whenBecomesTrue(new SequentialGroup(
                        new InstantCommand(() -> ImprovedOuttakeJava.canSpin = true),
                        FlyWheelJava.spin
                ));

        Gamepads.gamepad2().b()
                .whenBecomesTrue(new SequentialGroup(
                        new InstantCommand(() -> ImprovedOuttakeJava.canSpin = false),
                        FlyWheelJava.stop
                ));

        Gamepads.gamepad2().x()
                .whenBecomesTrue(new SequentialGroup(FlyWheelJava.backOut));

        // Gear Controls
        Gamepads.gamepad2().rightBumper()
                .whenBecomesTrue(TurretJava.spinGearRight)
                .whenBecomesFalse(TurretJava.stopGear);

        Gamepads.gamepad2().leftBumper()
                .whenBecomesTrue(TurretJava.spinGearLeft)
                .whenBecomesFalse(TurretJava.stopGear);

        Gamepads.gamepad2().dpadRight()
                .whenBecomesTrue(TurretJava.gearAlittleLeft)
                .whenBecomesFalse(TurretJava.stopGear);

        Gamepads.gamepad2().dpadLeft()
                .whenBecomesTrue(TurretJava.gearAlittleRight)
                .whenBecomesFalse(TurretJava.stopGear);

        // Flap Controls
        Gamepads.gamepad2().dpadUp()
                .whenBecomesTrue(HoodJava.FlapDown);

        Gamepads.gamepad2().dpadDown()
                .whenBecomesTrue(HoodJava.FlapUp);
    }

    @Override
    public void onUpdate() {
        // Note: Java equivalent of Kotlin telemetry DSL needs your telemetry setup
        // telemetry.addData("Hood Position", Hood.INSTANCE.hP);
        // telemetry.addData("Power", FlyWheel.INSTANCE.targetVelocity);
        // telemetry.addData("Distance in Tiles", ImprovedOuttake.INSTANCE.manualAim/24.0);
        // telemetry.addData("Manual Mode", ImprovedOuttake.INSTANCE.fullManual);
        // telemetry.addData("Can Shoot", ImprovedOuttake.INSTANCE.canSpin);
        // telemetry.update();
    }
}
