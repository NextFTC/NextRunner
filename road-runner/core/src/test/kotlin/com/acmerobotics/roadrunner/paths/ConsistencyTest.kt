package com.acmerobotics.roadrunner.paths

import com.acmerobotics.roadrunner.TEST_ACCEL_CONSTRAINT
import com.acmerobotics.roadrunner.TEST_PROFILE_PARAMS
import com.acmerobotics.roadrunner.TEST_TRAJECTORY_BUILDER_PARAMS
import com.acmerobotics.roadrunner.TEST_VEL_CONSTRAINT
import com.acmerobotics.roadrunner.dualEqual
import com.acmerobotics.roadrunner.dualPoseEqual
import com.acmerobotics.roadrunner.geometry.Rotation2d
import com.acmerobotics.roadrunner.geometry.Vector2d
import com.acmerobotics.roadrunner.geometry.range
import com.acmerobotics.roadrunner.posPathSeqBuilder
import com.acmerobotics.roadrunner.profiles.forwardProfile
import com.acmerobotics.roadrunner.randomAngle
import com.acmerobotics.roadrunner.randomAngleGen
import com.acmerobotics.roadrunner.randomPoint
import com.acmerobotics.roadrunner.randomPointGen
import com.acmerobotics.roadrunner.trajectories.PositionPathSeqBuilder
import io.kotest.core.spec.style.FunSpec
import io.kotest.matchers.should
import io.kotest.matchers.shouldBe
import io.kotest.property.arbitrary.arbitrary
import io.kotest.property.checkAll

data class SplineTarget(val pos: Vector2d, val tangent: Rotation2d)

val targetArb = arbitrary {
    val pos = randomPointGen.bind()
    val tangent = randomAngleGen.bind()
    SplineTarget(pos, tangent)
}

val random3SplinePath = arbitrary { rs ->
    PositionPathSeqBuilder(
        Vector2d(0.0, 0.0),
        0.0,
        TEST_TRAJECTORY_BUILDER_PARAMS.arcLengthSamplingEps
    ).splineTo(rs.random.randomPoint(), rs.random.randomAngle())
        .splineTo(rs.random.randomPoint(), rs.random.randomAngle())
        .splineTo(rs.random.randomPoint(), rs.random.randomAngle())
}

fun PositionPathSeqBuilder.buildToComposite() =
    CompositePosePath(build().map { TangentPath(it, 0.0 ) })

class ConsistencyTest : FunSpec(
    {
        context("given the same inputs") {
            test("multiple path builds are the same") {
                checkAll(targetArb, targetArb, targetArb) { tA, tB, tC ->
                    val path1 = posPathSeqBuilder()
                        .splineTo(tA.pos, tA.tangent)
                        .splineTo(tB.pos, tB.tangent)
                        .splineTo(tC.pos, tC.tangent)
                        .buildToComposite()

                    val path2 = posPathSeqBuilder()
                        .splineTo(tA.pos, tA.tangent)
                        .splineTo(tB.pos, tB.tangent)
                        .splineTo(tC.pos, tC.tangent)
                        .buildToComposite()

                    path1.length() shouldBe path2.length()

                    val samples = range(0.0, path1.length(), 100)

                    samples.forEach {
                        path1[it, 3] should dualPoseEqual(path2[it, 3])
                    }
                }
            }


            test("multiple profile builds are the same") {
                checkAll(targetArb, targetArb, targetArb) { tA, tB, tC ->
                    val profile1 = forwardProfile(
                        TEST_PROFILE_PARAMS,
                        posPathSeqBuilder()
                            .splineTo(tA.pos, tA.tangent)
                            .splineTo(tB.pos, tB.tangent)
                            .splineTo(tC.pos, tC.tangent)
                            .buildToComposite(),
                        0.0,
                        TEST_VEL_CONSTRAINT,
                        TEST_ACCEL_CONSTRAINT
                    )

                    val profile2 = forwardProfile(
                        TEST_PROFILE_PARAMS,
                        posPathSeqBuilder()
                            .splineTo(tA.pos, tA.tangent)
                            .splineTo(tB.pos, tB.tangent)
                            .splineTo(tC.pos, tC.tangent)
                            .buildToComposite(),
                        0.0,
                        TEST_VEL_CONSTRAINT,
                        TEST_ACCEL_CONSTRAINT
                    )

                    profile1.length shouldBe profile2.length

                    val samples = range(0.0, profile1.length, 100)

                    samples.forEach {
                        profile1[it] should dualEqual(profile2[it])
                    }
                }
            }
        }
    }
)