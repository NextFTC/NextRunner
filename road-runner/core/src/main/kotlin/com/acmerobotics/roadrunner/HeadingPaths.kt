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

data class TangentHeadingPath(
    @JvmField
    val posPath: PositionPath<Arclength>,
    @JvmField
    val offset: Double = 0.0
) : HeadingPath {
    override fun get(s: Double, n: Int): Rotation2dDual<Arclength> = posPath[s, n+1].drop(1).angleCast() + offset

    override fun length(): Double = posPath.length()
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