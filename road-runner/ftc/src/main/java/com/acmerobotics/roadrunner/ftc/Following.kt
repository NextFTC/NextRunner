package com.acmerobotics.roadrunner.ftc

import com.acmerobotics.roadrunner.actions.now
import com.acmerobotics.roadrunner.geometry.Arclength
import com.acmerobotics.roadrunner.geometry.Pose2d
import com.acmerobotics.roadrunner.geometry.Pose2dDual
import com.acmerobotics.roadrunner.geometry.PoseVelocity2d
import com.acmerobotics.roadrunner.geometry.PoseVelocity2dDual
import com.acmerobotics.roadrunner.geometry.Time
import com.acmerobotics.roadrunner.geometry.Vector2d
import com.acmerobotics.roadrunner.geometry.range
import com.acmerobotics.roadrunner.paths.PosePath
import com.acmerobotics.roadrunner.profiles.AccelConstraint
import com.acmerobotics.roadrunner.profiles.ProfileParams
import com.acmerobotics.roadrunner.profiles.TimeProfile
import com.acmerobotics.roadrunner.profiles.VelConstraint
import com.acmerobotics.roadrunner.profiles.forwardProfile
import com.acmerobotics.roadrunner.trajectories.DisplacementTrajectory
import com.acmerobotics.roadrunner.trajectories.TimeTrajectory
import com.acmerobotics.roadrunner.trajectories.Trajectory
import com.acmerobotics.roadrunner.trajectories.begin
import com.acmerobotics.roadrunner.trajectories.duration
import com.acmerobotics.roadrunner.trajectories.end
import kotlin.math.ceil
import kotlin.math.max

data class FollowerParams(
    @JvmField
    val profileParams: ProfileParams,
    @JvmField
    val velConstraint: VelConstraint,
    @JvmField
    val accelConstraint: AccelConstraint
)

interface Follower {
    val currentTarget: Pose2d
    val lastCommand: PoseVelocity2dDual<Time>

    val isDone: Boolean

    fun follow()

    val points: List<Vector2d> get() = listOf(Vector2d.zero)
}

class DisplacementFollower(
    val trajectory: Trajectory<Arclength>,
    val drive: Drive
) : Follower {
    constructor(
        traj: Trajectory<*>,
        drive: Drive
    ) : this(
        traj.wrtDisp(),
        drive
    )

    @JvmOverloads
    constructor(
        path: PosePath,
        drive: Drive,
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

    override var currentTarget = trajectory.begin.value()
        private set

    override var lastCommand = PoseVelocity2dDual.constant<Time>(PoseVelocity2d(Vector2d(0.0, 0.0), 0.0), 1)
        private set

    fun getDriveCommand() : PoseVelocity2dDual<Time> {
        val robotVel = drive.localizer.update()
        val robotPose = drive.localizer.pose

        ds = trajectory.project(robotPose.position, ds)

        val error = trajectory.end.value() - robotPose
        if (ds >= trajectory.length() || (error.line.norm() < 1.0 && error.angle < Math.toDegrees(5.0))) {
            isDone = true
            return PoseVelocity2dDual.constant(PoseVelocity2d(Vector2d(0.0, 0.0), 0.0), 1)
        }

        val target = trajectory[ds]
        currentTarget = target.value()

        return drive.controller.compute(
            target,
            robotPose,
            robotVel
        )
    }

    override fun follow() {
        lastCommand = getDriveCommand()
        drive.setDrivePowersWithFF(lastCommand)
    }

    override val points = range(
        0.0,
        trajectory.length(),
        max(2, ceil(trajectory.length() / 2 ).toInt())
    ).let {
        List<Vector2d>(it.size) { i ->
            trajectory[it[i]].value().position
        }
    }
}

class TimeFollower(
    val trajectory: Trajectory<Time>,
    val drive: Drive
) : Follower {
    constructor(
        traj: Trajectory<*>,
        drive: Drive
    ) : this(
        traj.wrtTime(),
        drive
    )

    @JvmOverloads
    constructor(
        path: PosePath,
        drive: Drive,
        velConstraintOverride: VelConstraint = drive.followerParams.velConstraint,
        accelConstraintOverride: AccelConstraint = drive.followerParams.accelConstraint
    ) : this(
        TimeTrajectory(
            path,
            TimeProfile(forwardProfile(
                drive.followerParams.profileParams,
                path,
                0.0,
                velConstraintOverride,
                accelConstraintOverride
            ))
        ),
        drive
    )

    override var currentTarget = trajectory.begin.value()
        private set

    val duration = trajectory.duration
    var startTime = -1.0
    var dt = 0.0

    override var isDone = false
        private set

    override var lastCommand = PoseVelocity2dDual.constant<Time>(PoseVelocity2d(Vector2d(0.0, 0.0), 0.0), 1)
        private set

    fun getDriveCommand(): PoseVelocity2dDual<Time> {
        if (startTime < 0) {
            startTime = now()
        } else {
            dt = now() - startTime
        }

        if (dt >= duration) {
            isDone = true
            return PoseVelocity2dDual.constant(PoseVelocity2d(Vector2d(0.0, 0.0), 0.0), 1)
        }

        val target = trajectory[dt]
        val robotVel = drive.localizer.update()

        return drive.controller.compute(
            target,
            drive.localizer.pose,
            robotVel
        )
    }

    override fun follow() {
        lastCommand = getDriveCommand()
        drive.setDrivePowersWithFF(lastCommand)
    }

    override val points = range(
        0.0,
        trajectory.length(),
        max(2, ceil(trajectory.length() / 2 ).toInt())
    ).let {
        List<Vector2d>(it.size) { i ->
            trajectory[it[i]].value().position
        }
    }
}

class HoldFollower(pose: Pose2d, val drive: Drive) : Follower {
    val pose = Pose2dDual.constant<Time>(pose, 3)

    override val currentTarget: Pose2d = pose

    override var lastCommand: PoseVelocity2dDual<Time> = PoseVelocity2dDual.zero()
        private set

    override var isDone: Boolean = false

    override fun follow() {
        val vel = drive.localizer.update()
        lastCommand = drive.controller.compute(pose, drive.localizer.pose, vel)
        drive.setDrivePowersWithFF(lastCommand)
    }
}