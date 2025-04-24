package com.acmerobotics.roadrunner.trajectories

import com.acmerobotics.roadrunner.profiles.CancelableProfile
import com.acmerobotics.roadrunner.profiles.DisplacementProfile
import com.acmerobotics.roadrunner.profiles.TimeProfile
import com.acmerobotics.roadrunner.geometry.Pose2dDual
import com.acmerobotics.roadrunner.geometry.Time
import com.acmerobotics.roadrunner.geometry.Vector2d
import com.acmerobotics.roadrunner.paths.PosePath

interface Trajectory {fun wrtDisp(): DisplacementTrajectory
    fun wrtTime(): TimeTrajectory

    fun length(): Double

    operator fun get(t: Double): Pose2dDual<Time>
}

class CancelableTrajectory(
    val path: MappedPosePath,
    val profile: CancelableProfile,
    @JvmField
    val offsets: List<Double>
) : Trajectory {
    fun cancel(s: Double): DisplacementTrajectory {
        val offset = s
        return DisplacementTrajectory(
            object : PosePath {
                override fun length() = path.length() - offset
                override fun get(s: Double, n: Int) = path[s + offset, n]
            },
            profile.cancel(s)
        )
    }

    override fun wrtDisp() = DisplacementTrajectory(this)
    override fun wrtTime() = TimeTrajectory(this)

    override fun length(): Double = path.length()

    override fun get(s: Double) = path[s, 3].reparam(profile[s])
}

class DisplacementTrajectory(
    val path: PosePath,
    val profile: DisplacementProfile
) : Trajectory {
    constructor(t: CancelableTrajectory) : this(t.path, t.profile.baseProfile)

    override fun wrtDisp() = this
    override fun wrtTime() = TimeTrajectory(this)

    override fun length() = path.length()

    fun project(query: Vector2d, init: Double) = com.acmerobotics.roadrunner.paths.project(path, query, init)

    override operator fun get(s: Double): Pose2dDual<Time> = path[s, 3].reparam(profile[s])
}

class TimeTrajectory(
    val path: PosePath,
    val profile: TimeProfile
) : Trajectory {
    @JvmField val duration = profile.duration

    constructor(t: CancelableTrajectory) : this(t.path, TimeProfile(t.profile.baseProfile))

    constructor(t: DisplacementTrajectory) : this(t.path, TimeProfile(t.profile))

    override fun wrtDisp() = DisplacementTrajectory(this.path, this.profile.dispProfile)
    override fun wrtTime() = this

    override fun length(): Double = path.length()

    override operator fun get(t: Double): Pose2dDual<Time> {
        val s = profile[t]
        return path[s.value(), 3].reparam(s)
    }
}
