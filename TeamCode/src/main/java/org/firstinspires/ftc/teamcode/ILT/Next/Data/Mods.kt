@file:Suppress("PackageName")

package org.firstinspires.ftc.teamcode.Util

import com.pedropathing.follower.Follower
import com.pedropathing.geometry.Pose
import com.pedropathing.math.Vector
import com.qualcomm.robotcore.hardware.HardwareMap
import dev.nextftc.core.components.BindingsComponent
import dev.nextftc.core.components.SubsystemComponent
import dev.nextftc.core.subsystems.Subsystem
import dev.nextftc.extensions.pedro.PedroComponent
import dev.nextftc.ftc.NextFTCOpMode
import dev.nextftc.ftc.components.BulkReadComponent
import dev.nextftc.ftc.components.LoopTimeComponent

fun NextFTCOpMode.addSubsystems(vararg subsystems: Subsystem) {
    addComponents(BindingsComponent, BulkReadComponent, LoopTimeComponent(), SubsystemComponent(*subsystems))
}
fun NextFTCOpMode.includePedro(followerFactory: (HardwareMap) -> Follower) {
    addComponents(PedroComponent(followerFactory))
}
fun Vector.toPose(): Pose {
    return Pose(this.xComponent,this.yComponent)
}