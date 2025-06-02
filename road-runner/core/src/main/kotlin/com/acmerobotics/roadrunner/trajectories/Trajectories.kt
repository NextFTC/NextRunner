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
import com.acmerobotics.roadrunner.paths.MappedPosePath
import com.acmerobotics.roadrunner.paths.PoseMap
import com.acmerobotics.roadrunner.paths.PosePath
import com.acmerobotics.roadrunner.profiles.plus

interface Trajectory<Param> {
    fun length(): Double
    fun duration() = wrtTime().duration


    operator fun get(param: Double): Pose2dDual<Time>
    fun start() = get(0.0)
    fun endWrtDisp() = wrtDisp()[length()]
    fun endWrtTime() = wrtTime()[duration()]

    fun project(query: Vector2d, init: Double): Double

    fun wrtDisp(): DisplacementTrajectory
    fun wrtTime(): TimeTrajectory

    operator fun plus(other: Trajectory<Param>) = CompositeTrajectory(this, other)

    fun map(map: PoseMap) = wrtDisp().let {
        DisplacementTrajectory(
            MappedPosePath(it.path, map),
            it.profile
        )
    }
}

val Trajectory<*>.duration get() = this.wrtTime().duration

val Trajectory<Arclength>.endWrtDisp get() = get(length())
val Trajectory<Time>.endWrtTime get() = get(duration)

class CancelableTrajectory(
    path: PosePath,
    @JvmField
    val cProfile: CancelableProfile,
    @JvmField
    val offsets: List<Double>
) : DisplacementTrajectory(path, cProfile.baseProfile) {
    fun cancel(s: Double): DisplacementTrajectory {
        val offset = s
        return DisplacementTrajectory(
            object : PosePath {
                override fun length() = path.length() - offset
                override fun get(s: Double, n: Int) = path[s + offset, n]
            },
            cProfile.cancel(s)
        )
    }
}

open class DisplacementTrajectory(
    @JvmField
    val path: PosePath,
    @JvmField
    val profile: DisplacementProfile
) : Trajectory<Arclength> {
    constructor(t: CancelableTrajectory) : this(t.path, t.cProfile.baseProfile)

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

    constructor(t: CancelableTrajectory) : this(t.path, TimeProfile(t.cProfile.baseProfile))

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
) : DisplacementTrajectory(
    CompositePosePath(trajectories.map { it.path }, offsets),
    trajectories.map { it.profile }.reduce { acc, profile -> acc + profile }
    ) {

    constructor(trajectories: Collection<Trajectory<*>>) : this(trajectories.map { it.wrtDisp() })
    constructor(vararg trajectories: DisplacementTrajectory) : this(trajectories.toList())
    constructor(vararg trajectories: Trajectory<*>) : this(trajectories.map { it.wrtDisp() })

    @JvmField
    val length = offsets.last()

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
}

fun compose(vararg trajectories: Trajectory<*>) = CompositeTrajectory(*trajectories)
fun List<Trajectory<*>>.compose() = CompositeTrajectory(this)
