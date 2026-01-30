package org.firstinspires.ftc.teamcode.ILT.Next

import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import dev.nextftc.core.components.BindingsComponent
import dev.nextftc.ftc.NextFTCOpMode
import dev.nextftc.ftc.components.BulkReadComponent
import dev.nextftc.hardware.impl.MotorEx
import com.qualcomm.robotcore.hardware.DcMotor
import dev.nextftc.ftc.Gamepads

@TeleOp(name = "Motor Test - 0.1 Power", group = "Test")
class TeleOpBurn : NextFTCOpMode() {

    // Replace "motor1" and "motor2" with your actual motor names from the config
    private  var motor1 = MotorEx("Fly1")
    private  var motor2 =  MotorEx("Fly2")

    init {
        addComponents(
            BulkReadComponent,
            BindingsComponent,
        )
    }

    override fun onInit() {
        // Initialize motors - replace these strings with your actual hardware map names


        // Set motors to run without encoders (or change if needed)
        motor1.motor.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
        motor2.motor.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER

        telemetry.addLine("Motors initialized")
        telemetry.addLine("Ready to test at 0.1 power")
        telemetry.update()
    }

    override fun onStartButtonPressed() {

        Gamepads.gamepad1.square whenBecomesTrue {motor1.power = 1.0} whenBecomesFalse {motor1.power = 0.0}
        Gamepads.gamepad1.circle whenBecomesTrue { motor2.power = 1.0}whenBecomesFalse {motor2.power = 0.0}
        Gamepads.gamepad1.leftBumper whenBecomesTrue {motor1.power = -1.0} whenBecomesFalse {motor1.power = 0.0}
        Gamepads.gamepad1.rightBumper whenBecomesTrue { motor2.power = -1.0}whenBecomesFalse {motor2.power = 0.0}


    }

    override fun onUpdate() {
        // Display motor info
        telemetry.addData("Motor 1 Power", motor1.power)
        telemetry.addData("Motor 1 Position", motor1.currentPosition)
        telemetry.addLine()
        telemetry.addData("Motor 2 Power", motor2.power)
        telemetry.addData("Motor 2 Position", motor2.currentPosition)
        telemetry.addLine()
        telemetry.addLine("Press STOP to end test")
        telemetry.update()
    }

    override fun onStop() {
        // Stop motors when OpMode ends
        motor1.power = 0.0
        motor2.power = 0.0
    }
}