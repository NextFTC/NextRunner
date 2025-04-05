package com.acmerobotics.roadrunner

interface Trajectory {
    fun length(): Double

    operator fun get(t: Double): Pose2dDual<Time>
}

class CancelableTrajcetory(
    @JvmField
    val path: MappedPosePath,
    @JvmField
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

    override fun length(): Double = path.length()

    override fun get(s: Double) = path[s, 3].reparam(profile[s])
}

class DisplacementTrajectory(
    @JvmField
    val path: PosePath,
    @JvmField
    val profile: DisplacementProfile
) : Trajectory {
    constructor(t: CancelableTrajcetory) : this(t.path, t.profile.baseProfile)

    override fun length() = path.length()

    fun project(query: Vector2d, init: Double) = project(path, query, init)

    override operator fun get(s: Double): Pose2dDual<Time> = path[s, 3].reparam(profile[s])
}

class TimeTrajectory(
    @JvmField
    val path: PosePath,
    @JvmField
    val profile: TimeProfile
) : Trajectory {
    @JvmField
    val duration = profile.duration

    constructor(t: CancelableTrajcetory) : this(t.path, TimeProfile(t.profile.baseProfile))

    constructor(t: DisplacementTrajectory) : this(t.path, TimeProfile(t.profile))

    override fun length(): Double = path.length()

    override operator fun get(t: Double): Pose2dDual<Time> {
        val s = profile[t]
        return path[s.value(), 3].reparam(s)
    }
}
