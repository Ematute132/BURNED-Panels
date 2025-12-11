package org.firstinspires.ftc.teamcode.ILT.Next.Subsystem.Outtake

import com.bylazar.configurables.annotations.Configurable
import dev.nextftc.core.subsystems.Subsystem
import dev.nextftc.hardware.impl.ServoEx

@Configurable
object Hood: Subsystem {
    private val hoodServo = ServoEx("flap")
    //change this hoodPostiion value based on distance and velo
    @JvmField var hoodPosition = 0.0

    fun getHoodPosition(){
        //what goes inside will be the distance from goal and using that we will find the hood servo angle
        // set that angle to servo hood position
    }

    override fun periodic() {
        hoodServo.position = hoodPosition
    }

    fun updatePosition(position: Double) {
        hoodPosition = position
    }
}