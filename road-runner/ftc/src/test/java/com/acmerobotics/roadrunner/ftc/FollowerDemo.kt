package com.acmerobotics.roadrunner.ftc

import com.acmerobotics.roadrunner.actions.TrajectoryActionBuilder
import com.acmerobotics.roadrunner.control.Drive
import com.acmerobotics.roadrunner.geometry.Pose2d
import com.acmerobotics.roadrunner.trajectories.TrajectoryBuilderParams

class FollowerDemo {
    fun Drive<*, *>.actionBuilderDisp(startPose: Pose2d) = TrajectoryActionBuilder(
        { TurnAction(it, this) },
        { FollowTrajectoryAction(
            DisplacementFollower(it, this),
            this
        )},
        TrajectoryBuilderParams(
            1e-6,
            followerParams.profileParams
        ),
        startPose,
        0.0,
        defaultTurnConstraints,
        defaultVelConstraint,
        defaultAccelConstraint
    )

    fun Drive<*, *>.actionBuilderTime(startPose: Pose2d) = TrajectoryActionBuilder(
        { TurnAction(it, this) },
        { FollowTrajectoryAction(
            TimeFollower(it, this),
            this
        )},
        TrajectoryBuilderParams(
            1e-6,
            followerParams.profileParams
        ),
        startPose,
        0.0,
        defaultTurnConstraints,
        defaultVelConstraint,
        defaultAccelConstraint
    )
}