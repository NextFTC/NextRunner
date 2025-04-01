@file:JvmName("PosePaths")

package com.acmerobotics.roadrunner

/**
 * @usesMathJax
 *
 * Path heading \(\theta(s)\)
 */
interface HeadingPath {
    operator fun get(s: Double, n: Int): Rotation2dDual<Arclength>

    fun length(): Double

    fun begin(n: Int) = get(0.0, n)
    fun end(n: Int) = get(length(), n)
}

data class ConstantHeadingPath(
    @JvmField
    val heading: Rotation2d,
    @JvmField
    val length: Double,
) : HeadingPath {
    override fun get(s: Double, n: Int) = Rotation2dDual.constant<Arclength>(heading, n)

    override fun length() = length
}

data class LinearHeadingPath(
    @JvmField
    val begin: Rotation2d,
    @JvmField
    val angle: Double,
    @JvmField
    val length: Double,
) : HeadingPath {
    override fun get(s: Double, n: Int) =
        Rotation2dDual.exp(DualNum.variable<Arclength>(s, n) / length * angle) * begin

    override fun length() = length
}

data class SplineHeadingPath(
    @JvmField
    val begin: Rotation2d,
    @JvmField
    val spline: QuinticSpline1d,
    @JvmField
    val length: Double,
) : HeadingPath {
    constructor(
        begin: Rotation2dDual<Arclength>,
        end: Rotation2dDual<Arclength>,
        length: Double,
    ) : this(
        begin.value(),
        require(begin.size() >= 3).let {
            require(end.size() >= 3).let {
                // s(t) = t * len
                (DualNum.variable<Internal>(1.0, 3) * length).let { s ->
                    QuinticSpline1d(
                        DualNum.cons(0.0, begin.velocity()).reparam(s),
                        DualNum.cons(end.value() - begin.value(), end.velocity()).reparam(s),
                    )
                }
            }
        },
        length
    )

    override fun get(s: Double, n: Int) =
        Rotation2dDual.exp(
            spline[s / length, n]
                .reparam(
                    // t(s) = s / len
                    DualNum.variable<Arclength>(s, n) / length
                )
        ) * begin

    override fun length() = length
}

/**
 * @usesMathJax
 *
 * Pose path \((x(s), y(s), \theta(s))\)
 */
interface PosePath {
    operator fun get(s: Double, n: Int): Pose2dDual<Arclength>

    fun length(): Double

    fun begin(n: Int) = get(0.0, n)
    fun end(n: Int) = get(length(), n)

    fun project(query: Vector2d, init: Double = 0.0) = (1..10).fold(init) { s, _ ->
        val guess = this[s, 3].position.bind()
        val ds = (query - guess.value()) dot guess.drop(1).value()
        clamp(s + ds, 0.0, this.length())
    }
}

data class TangentPath(
    @JvmField
    val path: PositionPath<Arclength>,
    @JvmField
    val offset: Double
) : PosePath {
    // NOTE: n+1 guarantees enough derivatives for tangent
    override operator fun get(s: Double, n: Int) = path[s, n + 1].let {
        Pose2dDual(it, it.drop(1).angleCast() + offset)
    }

    override fun length() = path.length()
}

data class HeadingPosePath(
    @JvmField
    val posPath: PositionPath<Arclength>,
    @JvmField
    val headingPath: HeadingPath,
) : PosePath {
    init {
        require(posPath.length() == headingPath.length()) {
            "posPath.length (${posPath.length()}) != headingPath.length (${headingPath.length()})"
        }
    }

    override fun get(s: Double, n: Int) =
        Pose2dDual(posPath[s, n], headingPath[s, n])

    override fun length() = posPath.length()
}

data class CompositePosePath(
    @JvmField
    val paths: List<PosePath>,
    @JvmField
    val offsets: List<Double> = paths.scan(0.0) { acc, path -> acc + path.length() },
) : PosePath {
    init {
        require(paths.size + 1 == offsets.size) {
            "paths.size (${paths.size}) + 1 != offsets.size (${offsets.size})"
        }
    }

    @JvmField
    val length = offsets.last()

    override fun get(s: Double, n: Int): Pose2dDual<Arclength> {
        if (s > length) {
            return Pose2dDual.constant(paths.last().end(1).value(), n)
        }

        for ((offset, path) in offsets.zip(paths).reversed()) {
            if (s >= offset) {
                return path[s - offset, n]
            }
        }

        return Pose2dDual.constant(paths.first()[0.0, 1].value(), n)
    }

    override fun length() = length
}

class OffsetPosePath(
    @JvmField
    val path: PosePath,
    @JvmField
    val offsets: List<Double> = listOf(0.0, path.length())
) : PosePath {
    init {
        require(offsets.size == 2) {
            "offsets.size (${offsets.size}) != 2"
        }
    }

    override fun get(s: Double, n: Int): Pose2dDual<Arclength> {
        val offset = offsets[0]
        val length = offsets[1] - offsets[0]
        return path[s * length + offset, n]
    }

    override fun length() = offsets[1] - offsets[0]
}

/**
 * Project position [query] onto position path [path] starting with initial guess [init].
 */
fun project(path: PosePath, query: Vector2d, init: Double): Double = path.project(query, init)
