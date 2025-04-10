package com.acmerobotics.roadrunner.control

import com.acmerobotics.roadrunner.geometry.Pose2d
import com.acmerobotics.roadrunner.geometry.PoseVelocity2d
import com.acmerobotics.roadrunner.geometry.PoseVelocity2dDual
import com.acmerobotics.roadrunner.geometry.Time
import com.acmerobotics.roadrunner.geometry.Vector2d
import com.acmerobotics.roadrunner.paths.AccelConstraint
import com.acmerobotics.roadrunner.paths.PosePath
import com.acmerobotics.roadrunner.paths.ProfileParams
import com.acmerobotics.roadrunner.paths.VelConstraint
import com.acmerobotics.roadrunner.paths.forwardProfile
import com.acmerobotics.roadrunner.trajectories.DisplacementTrajectory

data class FollowerParams(
    @JvmField
    val profileParams: ProfileParams,
    @JvmField
    val velConstraint: VelConstraint,
    @JvmField
    val accelConstraint: AccelConstraint
)

class Follower(
    @JvmField val traj: DisplacementTrajectory,
    @JvmField val drive: Drive<*, *>
    ) {
    @JvmOverloads
    constructor(
        path: PosePath,
        drive: Drive<*, *>,
        velConstraintOverride: VelConstraint = drive.followerParams.velConstraint,
        accelConstraintOverride: AccelConstraint = drive.followerParams.accelConstraint
    ) : this(
        DisplacementTrajectory(
            path,
            forwardProfile(
                drive.followerParams.profileParams,
                path,
                0.0,
                velConstraintOverride,
                accelConstraintOverride
            )
        ),
        drive
    )

    var disp = 0.0
    var isDone = false

    fun follow() : PoseVelocity2dDual<Time> {
        val robotVel = drive.updatePoseEstimate()
        val robotPose = drive.localizer.pose

        disp = traj.project(robotPose.position, disp)

        val error = traj.path.end(1).value() - robotPose
        if (disp >= traj.length() || (error.line.norm() < 1.0 && error.angle < Math.toDegrees(5.0))) {
            isDone = true
            return PoseVelocity2dDual.constant(PoseVelocity2d(Vector2d(0.0, 0.0), 0.0), 1)
        }

        val target = traj[disp]
        return drive.controller.compute(
            target,
            robotPose,
            robotVel
        )
    }
}