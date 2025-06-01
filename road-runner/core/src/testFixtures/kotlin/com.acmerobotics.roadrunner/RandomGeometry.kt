package com.acmerobotics.roadrunner

import com.acmerobotics.roadrunner.geometry.Pose2d
import com.acmerobotics.roadrunner.geometry.Rotation2d
import com.acmerobotics.roadrunner.geometry.Vector2d
import io.kotest.property.Arb
import kotlin.random.Random
import io.kotest.property.arbitrary.arbitrary
import io.kotest.property.arbitrary.bind
import io.kotest.property.arbitrary.double
import io.kotest.property.arbitrary.map

fun Random.randomPoint(): Vector2d = Vector2d(
    nextDouble(-72.0, 72.0),
    nextDouble(-72.0, 72.0)
)

fun randomPoint() = Random.Default.randomPoint()

val randomPointGen = arbitrary { rs -> rs.random.randomPoint() }

fun Random.randomAngle(): Rotation2d =
    Rotation2d.exp(nextDouble(-Math.PI, Math.PI))

fun randomAngle() = Random.Default.randomAngle()

val randomAngleGen = arbitrary { rs -> rs.random.randomAngle() }

fun Random.randomPose(): Pose2d = Pose2d(randomPoint(), randomAngle())

fun randomPose() = Random.Default.randomPose()

val randomPoseGen = Arb.bind(
    randomPointGen,
    randomAngleGen
) {
    pos, heading -> Pose2d(pos, heading)
}