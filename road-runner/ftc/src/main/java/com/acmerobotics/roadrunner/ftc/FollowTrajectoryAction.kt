package com.acmerobotics.roadrunner.ftc

import com.acmerobotics.dashboard.telemetry.TelemetryPacket
import com.acmerobotics.roadrunner.actions.Action
import com.acmerobotics.roadrunner.control.Drive
import com.acmerobotics.roadrunner.geometry.Vector2d
import com.acmerobotics.roadrunner.geometry.range
import com.acmerobotics.roadrunner.geometry.xs
import com.acmerobotics.roadrunner.geometry.ys
import com.acmerobotics.roadrunner.paths.PosePath
import com.acmerobotics.roadrunner.profiles.AccelConstraint
import com.acmerobotics.roadrunner.profiles.VelConstraint
import com.acmerobotics.roadrunner.trajectories.DisplacementTrajectory
import com.acmerobotics.roadrunner.trajectories.Trajectory
import java.lang.Math.toDegrees
import kotlin.math.ceil
import kotlin.math.max

class FollowTrajectoryAction(
    @JvmField
    val follower: Follower,
    @JvmField
    val drive: Drive<*, *>
) : Action {
    constructor(
        traj: Trajectory,
        drive: Drive<*, *>
    ) : this(
        DisplacementFollower(traj, drive),
        drive
    )

    constructor(
        path: PosePath,
        drive: Drive<*, *>,
        velConstraintOverride: VelConstraint = drive.followerParams.velConstraint,
        accelConstraintOverride: AccelConstraint = drive.followerParams.accelConstraint
    ) : this(
        DisplacementFollower(path, drive, velConstraintOverride, accelConstraintOverride),
        drive
    )

    val points = range(
        0.0,
        follower.trajectory.length(),
        max(2, ceil(follower.trajectory.length() / 2 ).toInt())
    ).let {
        List<Vector2d>(it.size) { i ->
            follower.trajectory[it[i]].value().position
        }
    }

    val xPoints = points.xs().toDoubleArray()
    val yPoints = points.ys().toDoubleArray()

    override fun run(p: TelemetryPacket): Boolean {
        drive.setDrivePowersWithFF(follower.follow())

        p.put("x", drive.localizer.pose.position.x)
        p.put("y", drive.localizer.pose.position.y)
        p.put("heading (deg)", toDegrees(drive.localizer.pose.heading.toDouble()))

        val error = follower.currentTarget.minusExp(drive.localizer.pose)
        p.put("xError", error.position.x)
        p.put("yError", error.position.y)
        p.put("headingError (deg)", toDegrees(error.heading.toDouble()))

        // only draw when active; only one drive action should be active at a time
        val c = p.fieldOverlay()
        drive.drawPoseHistory(c)

        c.setStroke("#4CAF50")
        drawRobot(c, follower.currentTarget)

        c.setStroke("#3F51B5")
        drawRobot(c, drive.localizer.pose)

        c.setStroke("#4CAF50FF")
        c.setStrokeWidth(1)
        c.strokePolyline(xPoints, yPoints)

        return follower.isDone
    }
}