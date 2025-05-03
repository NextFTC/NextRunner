@file:JvmName("Trajectories")
package com.acmerobotics.roadrunner.trajectories

import com.acmerobotics.roadrunner.geometry.Arclength
import com.acmerobotics.roadrunner.profiles.CancelableProfile
import com.acmerobotics.roadrunner.profiles.DisplacementProfile
import com.acmerobotics.roadrunner.profiles.TimeProfile
import com.acmerobotics.roadrunner.geometry.Pose2dDual
import com.acmerobotics.roadrunner.geometry.Time
import com.acmerobotics.roadrunner.geometry.Vector2d
import com.acmerobotics.roadrunner.paths.CompositePosePath
import com.acmerobotics.roadrunner.paths.PosePath
import com.acmerobotics.roadrunner.profiles.plus

interface Trajectory<Param> {
    fun length(): Double

    operator fun get(param: Double): Pose2dDual<Time>

    fun project(query: Vector2d, init: Double): Double

    fun wrtDisp(): DisplacementTrajectory
    fun wrtTime(): TimeTrajectory
}

val Trajectory<*>.duration get() = this.wrtTime().duration

val Trajectory<Arclength>.begin get() = get(0.0)
val Trajectory<Arclength>.end get() = get(length())

val Trajectory<Time>.begin get() = get(0.0)
val Trajectory<Time>.end get() = get(duration)

class CancelableTrajectory(
    @JvmField
    val path: PosePath,
    @JvmField
    val profile: CancelableProfile,
    @JvmField
    val offsets: List<Double>
) : Trajectory<Arclength> {
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

    override fun project(query: Vector2d, init: Double): Double = path.project(query, init)
}

class DisplacementTrajectory(
    @JvmField
    val path: PosePath,
    @JvmField
    val profile: DisplacementProfile
) : Trajectory<Arclength> {
    constructor(t: CancelableTrajectory) : this(t.path, t.profile.baseProfile)

    override fun wrtDisp() = this
    override fun wrtTime() = TimeTrajectory(this)

    override fun length() = path.length()

    override fun project(query: Vector2d, init: Double) = path.project(query, init)

    override operator fun get(s: Double): Pose2dDual<Time> = path[s, 3].reparam(profile[s])
}

class TimeTrajectory(
    @JvmField
    val path: PosePath,
    @JvmField
    val profile: TimeProfile
) : Trajectory<Time> {
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

    override fun project(query: Vector2d, init: Double): Double = path.project(query, init)
}

class CompositeTrajectory @JvmOverloads constructor(
    @JvmField
    val trajectories: List<DisplacementTrajectory>,
    @JvmField
    val offsets: List<Double> = trajectories.scan(0.0) { acc, t -> acc + t.length() }
) : Trajectory<Arclength> {
    constructor(trajectories: Collection<Trajectory<*>>) : this(trajectories.map { it.wrtDisp() })
    constructor(vararg trajectories: DisplacementTrajectory) : this(trajectories.toList())
    constructor(vararg trajectories: Trajectory<*>) : this(trajectories.map { it.wrtDisp() })

    @JvmField
    val length = offsets.last()
    @JvmField
    val path = CompositePosePath(trajectories.map { it.path }, offsets)
    @JvmField
    val profile = trajectories.map { it.profile }.reduce { acc, profile -> acc + profile }

    init {
        require(trajectories.size + 1 == offsets.size) {
            "trajectories.size (${trajectories.size}) + 1 != offsets.size (${offsets.size})"
        }
    }

    override fun get(s: Double): Pose2dDual<Time> {
        if (s > length) {
            return Pose2dDual.Companion.constant(trajectories.last().path.end(1).value(), 3)
        }

        for ((offset, traj) in offsets.zip(trajectories).reversed()) {
            if (s >= offset) {
                return traj[s - offset]
            }
        }

        return trajectories.first()[0.0]
    }

    override fun length() = length

    override fun wrtDisp() = DisplacementTrajectory(path, profile)
    override fun wrtTime() = wrtDisp().wrtTime()

    override fun project(query: Vector2d, init: Double): Double = wrtDisp().project(query, init)
}

fun compose(vararg trajectories: Trajectory<*>) = CompositeTrajectory(*trajectories)
fun List<Trajectory<*>>.compose() = CompositeTrajectory(this)
