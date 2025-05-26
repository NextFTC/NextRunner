package com.acmerobotics.roadrunner

import com.acmerobotics.roadrunner.geometry.Pose2d
import com.acmerobotics.roadrunner.geometry.Rotation2d
import com.acmerobotics.roadrunner.geometry.Vector2d
import com.acmerobotics.roadrunner.profiles.AccelConstraint
import com.acmerobotics.roadrunner.profiles.AngularVelConstraint
import com.acmerobotics.roadrunner.profiles.CancelableProfile
import com.acmerobotics.roadrunner.profiles.MinVelConstraint
import com.acmerobotics.roadrunner.profiles.ProfileAccelConstraint
import com.acmerobotics.roadrunner.profiles.ProfileParams
import com.acmerobotics.roadrunner.profiles.TimeProfile
import com.acmerobotics.roadrunner.profiles.TranslationalVelConstraint
import com.acmerobotics.roadrunner.profiles.VelConstraint
import com.acmerobotics.roadrunner.trajectories.TrajectoryBuilderParams
import kotlin.random.Random

val TEST_PROFILE_PARAMS = ProfileParams(
    0.25,
    0.1,
    1e-4,
)

val TEST_TRAJECTORY_BUILDER_PARAMS = TrajectoryBuilderParams(
    1e-6,
    TEST_PROFILE_PARAMS
)

val TEST_VEL_CONSTRAINT: VelConstraint = MinVelConstraint(
    listOf(
        TranslationalVelConstraint(50.0),
        AngularVelConstraint(Math.PI)
    )
)
val TEST_ACCEL_CONSTRAINT: AccelConstraint =
    ProfileAccelConstraint(-10.0, 30.0)


fun randomPoint(): Vector2d = Vector2d(
    Random.Default.nextDouble(-72.0, 72.0),
    Random.Default.nextDouble(-72.0, 72.0)
)

fun randomAngle(): Rotation2d =
    Rotation2d.exp(Random.Default.nextDouble(-Math.PI, Math.PI))

fun randomPose(): Pose2d = Pose2d(randomPoint(), randomAngle())

fun CancelableProfile.duration() = TimeProfile(baseProfile).duration

val Rotation2d.deg get() = Math.toDegrees(log())
val Rotation2d.repr get() = "$deg°"
val Pose2d.repr get() = "(${position.x}, ${position.y}, ${heading.repr})"