package com.acmerobotics.roadrunner.ftc

import com.acmerobotics.roadrunner.actions.now
import com.acmerobotics.roadrunner.control.Drive
import com.acmerobotics.roadrunner.control.Localizer
import com.acmerobotics.roadrunner.control.RobotPosVelController
import com.acmerobotics.roadrunner.geometry.Pose2d
import com.acmerobotics.roadrunner.geometry.PoseVelocity2d
import com.acmerobotics.roadrunner.geometry.PoseVelocity2dDual
import com.acmerobotics.roadrunner.geometry.Time
import com.acmerobotics.roadrunner.geometry.Vector2d
import com.acmerobotics.roadrunner.paths.PosePath
import com.acmerobotics.roadrunner.profiles.AccelConstraint
import com.acmerobotics.roadrunner.profiles.VelConstraint
import com.acmerobotics.roadrunner.profiles.forwardProfile
import com.acmerobotics.roadrunner.profiles.wrtTime
import com.acmerobotics.roadrunner.trajectories.DisplacementTrajectory
import com.acmerobotics.roadrunner.trajectories.TimeTrajectory
import com.acmerobotics.roadrunner.trajectories.Trajectory
import com.acmerobotics.roadrunner.trajectories.wrtDisp
import com.acmerobotics.roadrunner.trajectories.wrtTime

interface Follower {
    val trajectory: Trajectory
    val currentTarget: Pose2d

    val isDone: Boolean

    fun follow() : PoseVelocity2dDual<Time>
}

class DisplacementFollower(
    override val trajectory: DisplacementTrajectory,
    @JvmField val controller: RobotPosVelController,
    @JvmField val localizer: Localizer
) : Follower {
    constructor(
        traj: Trajectory,
        drive: Drive<*, *>
    ) : this(
        traj.wrtDisp!!,
        drive.controller,
        drive.localizer
    )

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

    var ds = 0.0
        private set

    override var isDone = false
        private set

    override var currentTarget = trajectory.path.begin(1).value()
        private set

    override fun follow() : PoseVelocity2dDual<Time> {
        val robotVel = localizer.update()
        val robotPose = localizer.pose

        ds = trajectory.project(robotPose.position, ds)

        val error = trajectory.path.end(1).value() - robotPose
        if (ds >= trajectory.length() || (error.line.norm() < 1.0 && error.angle < Math.toDegrees(5.0))) {
            isDone = true
            return PoseVelocity2dDual.constant(PoseVelocity2d(Vector2d(0.0, 0.0), 0.0), 1)
        }

        val target = trajectory[ds]
        currentTarget = target.value()

        return controller.compute(
            target,
            robotPose,
            robotVel
        )
    }
}

class TimeFollower(
    override val trajectory: TimeTrajectory,
    @JvmField val controller: RobotPosVelController,
    @JvmField val localizer: Localizer
) : Follower {
    constructor(
        traj: Trajectory,
        drive: Drive<*, *>
    ) : this(
        traj.wrtTime!!,
        drive.controller,
        drive.localizer
    )

    @JvmOverloads
    constructor(
        path: PosePath,
        drive: Drive<*, *>,
        velConstraintOverride: VelConstraint = drive.followerParams.velConstraint,
        accelConstraintOverride: AccelConstraint = drive.followerParams.accelConstraint
    ) : this(
        TimeTrajectory(
            path,
            forwardProfile(
                drive.followerParams.profileParams,
                path,
                0.0,
                velConstraintOverride,
                accelConstraintOverride
            ).wrtTime!!
        ),
        drive
    )

    override var currentTarget = trajectory.path.begin(1).value()
        private set

    var startTime = -1.0
    var dt = 0.0

    override var isDone = false
        private set

    override fun follow(): PoseVelocity2dDual<Time> {
        if (startTime < 0) {
            startTime = now()
        } else {
            dt = now() - startTime
        }

        if (dt >= trajectory.duration) {
            isDone = true
            return PoseVelocity2dDual.constant(PoseVelocity2d(Vector2d(0.0, 0.0), 0.0), 1)
        }

        val target = trajectory[dt]
        val robotVel = localizer.update()

        return controller.compute(
            target,
            localizer.pose,
            robotVel
        )
    }
}