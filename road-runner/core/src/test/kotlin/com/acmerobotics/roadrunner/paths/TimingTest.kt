package com.acmerobotics.roadrunner.paths

import com.acmerobotics.roadrunner.geometry.Vector2d
import com.acmerobotics.roadrunner.trajectories.PositionPathSeqBuilder
import com.acmerobotics.roadrunner.trajectories.TrajectoryBuilderParams
import kotlin.random.Random
import kotlin.system.measureTimeMillis
import kotlin.test.Test
import kotlin.time.measureTimedValue


val params = TrajectoryBuilderParams(
    1e-6,
    ProfileParams(
        0.25, 0.1, 1e-2,
    ),
)

val velConstraint: VelConstraint = MinVelConstraint(
    listOf(
        TranslationalVelConstraint(50.0),
        AngularVelConstraint(Math.PI)
    )
)
val accelConstraint: AccelConstraint =
    ProfileAccelConstraint(-10.0, 30.0)

class TimingTests {
    @Test
    fun `profile timing test`() {
        val posPaths = PositionPathSeqBuilder(
            Vector2d(0.0, 0.0),
            0.0,
            params.arcLengthSamplingEps
        ).splineTo(Vector2d(10.0, 10.0), 0.0)
            .splineTo(Vector2d(-10.0, 0.0), Math.PI)
            .splineTo(Vector2d(0.0, 0.0), Math.PI/2)
            .build()

        val posePaths = posPaths.map {
            TangentPath(it, 0.0)
        }

        val times = posePaths.mapIndexed { i, it ->
            i to
            measureTimeMillis {
                profile(params.profileParams, it, 0.0, velConstraint, accelConstraint)
            }
        }

        times.forEach {
            assert(it.second < 25) { "path ${it.first} took ${it.second} to profile" }
        }
    }

    @Test
    fun `bezier timing test`() {
        val pointList = (0..9).map {
            listOf(randomPoint(), randomPoint(), randomPoint())
        }

        val times = (0..9).map {
            measureTimedValue {
                val bezier = ArclengthReparamCurve2d(
                    BezierCurve2dInternal(pointList[it]),
                    params.arcLengthSamplingEps,
                )

                val path = TangentPath(bezier, 0.0)
                val profile = profile(params.profileParams, path, 0.0, velConstraint, accelConstraint)

                path to profile
            }
        }

        times.forEachIndexed { i, it ->
            println("time $i: ${it.duration.inWholeMilliseconds} ms")
            println("length $i: ${it.value.first.path.length()}")
            assert(it.duration.inWholeMilliseconds < 100)
        }

        val totalTime = times.sumOf { it.duration.inWholeMilliseconds }
        val avgTime = totalTime / times.size.toDouble()

        val totalLen = times.sumOf { it.value.first.path.length() }
        val avgLen = totalLen / times.size.toDouble()

        println("total time $totalTime, avg time $avgTime")
        println("total len $totalLen, avg len $avgLen")
    }

    fun randomPoint(): Vector2d = Vector2d(
            Random.Default.nextDouble(-72.0, 72.00),
            Random.Default.nextDouble(-72.0, 72.0)
        )
}