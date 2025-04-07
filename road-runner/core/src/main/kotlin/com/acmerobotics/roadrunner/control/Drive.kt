@file:JvmName("Drive")
package com.acmerobotics.roadrunner.control

import com.acmerobotics.roadrunner.geometry.PoseVelocity2d
import com.acmerobotics.roadrunner.geometry.PoseVelocity2dDual
import com.acmerobotics.roadrunner.geometry.Time
import com.acmerobotics.roadrunner.paths.AccelConstraint
import com.acmerobotics.roadrunner.paths.ProfileParams
import com.acmerobotics.roadrunner.paths.VelConstraint
import com.acmerobotics.roadrunner.trajectories.TurnConstraints

interface Drive<WI: WheelIncrements<Time>, WV: WheelVelocities<Time>> {
    val localizer: Localizer
    val controller: RobotPosVelController
    val kinematics: RobotKinematics<WI, WV>

    val profileParams: ProfileParams
    val defaultVelConstraint: VelConstraint
    val defaultAccelConstraint: AccelConstraint
    val defaultTurnConstraints: TurnConstraints

    fun setDrivePowers(powers: PoseVelocity2dDual<Time>)
    fun setDrivePowersWithFF(powers: PoseVelocity2dDual<Time>)

    fun setDrivePowers(powers: PoseVelocity2d) =
        setDrivePowers(PoseVelocity2dDual.constant(powers, 3))
    fun setDrivePowersWithFF(powers: PoseVelocity2d) =
        setDrivePowersWithFF(PoseVelocity2dDual.constant(powers, 3))

    fun updatePoseEstimate(): PoseVelocity2d {
        return localizer.update()
    }
}


