@file:Suppress("LocalVariableName")

package com.acmerobotics.roadrunner.control

import com.acmerobotics.roadrunner.geometry.DiagonalElement
import com.acmerobotics.roadrunner.geometry.DualNum
import com.acmerobotics.roadrunner.geometry.Matrix
import com.acmerobotics.roadrunner.geometry.Pose2d
import com.acmerobotics.roadrunner.geometry.Pose2dDual
import com.acmerobotics.roadrunner.geometry.PoseVelocity2d
import com.acmerobotics.roadrunner.geometry.PoseVelocity2dDual
import com.acmerobotics.roadrunner.geometry.Rotation2d
import com.acmerobotics.roadrunner.geometry.Time
import com.acmerobotics.roadrunner.geometry.Twist2d
import com.acmerobotics.roadrunner.geometry.Twist2dDual
import com.acmerobotics.roadrunner.geometry.Vector2d
import com.acmerobotics.roadrunner.geometry.Vector2dDual
import com.acmerobotics.roadrunner.geometry.times
import com.acmerobotics.roadrunner.geometry.unaryMinus
import org.ejml.dense.row.decomposition.lu.LUDecompositionAlt_DDRM
import org.ejml.simple.SimpleMatrix
import kotlin.math.pow

/**
 * Parameters class for linear quadratic regulator (LQR) control
 * for use with holonomic robots.
 *
 * This constructor assumes a kinematic acceleration model where the state error `x` is
 * `[xError, yError, headingError, vxError, vyError, omegaError]` and the control input `u`
 * is the desired robot acceleration `[ax, ay, alpha]`.
 * NextRunner converts the acceleration command to a velocity+acceleration command
 * when the LQR is used as a [RobotPosVelController]
 *
 * The system dynamics are approximated as:
 * `dx_dot = vx`
 * `dy_dot = vy`
 * `dtheta_dot = omega`
 * `dvx_dot = ax`
 * `dvy_dot = ay`
 * `domega_dot = alpha`
 *
 * @property qX The penalty for error in the x-position. Higher values prioritize correcting x-error.
 * @property qY The penalty for error in the y-position. Higher values prioritize correcting y-error.
 * @property qHeading The penalty for error in the heading. Higher values prioritize correcting heading error.
 * @property qVx The penalty for error in the x-velocity. Higher values prioritize correcting x-velocity error.
 * @property qVy The penalty for error in the y-velocity. Higher values prioritize correcting y-velocity error.
 * @property qOmega The penalty for error in the angular velocity. Higher values prioritize correcting angular velocity error.
 * @property rAx The penalty for using linear acceleration in the x-direction. Higher values prioritize using less `ax`.
 * @property rAy The penalty for using linear acceleration in the y-direction. Higher values prioritize using less `ay`.
 * @property rAlpha The penalty for using angular acceleration. Higher values prioritize using less `alpha`.
 */
class LQRParameters(
    qX: Double, qY: Double, qHeading: Double,
    qVx: Double, qVy: Double, qOmega: Double,
    rAx: Double, rAy: Double, rAlpha: Double,
) {
    init {
        require(qX >= 0 && qY >= 0 && qHeading >= 0) { "Position state costs (Q) must be non-negative" }
        require(qVx >= 0 && qVy >= 0 && qOmega >= 0) { "Velocity state costs (Q) must be non-negative" }
        require(rAx > 0 && rAy > 0 && rAlpha > 0) { "Control costs (R) must be positive" }
    }

    val Q = Matrix(SimpleMatrix.diag(qX, qY, qHeading, qVx, qVy, qOmega))
    val R = Matrix(SimpleMatrix.diag(rAx, rAy, rAlpha))

    var qX by DiagonalElement(Q, 0)
    var qY by DiagonalElement(Q, 1)
    var qHeading by DiagonalElement(Q, 2)
    var qVx by DiagonalElement(Q, 3)
    var qVy by DiagonalElement(Q, 4)
    var qOmega by DiagonalElement(Q, 5)

    var rAx by DiagonalElement(R, 0)
    var rAy by DiagonalElement(R, 1)
    var rAlpha by DiagonalElement(R, 2)
}

/**
 * A Linear Quadratic Regulator (LQR) for controlling a system modeled by state-space equations.
 *
 * LQR is a form of optimal control that finds the best control input to apply to a system
 * by minimizing a quadratic cost function. The cost function balances two competing goals:
 * 1.  **State Error**: How far the system is from its desired target state (penalized by the `Q` matrix).
 * 2.  **Control Effort**: How much energy or effort is used to control the system (penalized by the `R` matrix).
 *
 * The controller computes the optimal control input `u` using a simple state-feedback law: `u = -Kx`,
 * where `x` is the system's state error and `K` is the optimal gain matrix.
 *
 * This should be used by holonomic robots only.
 *
 * Thank you to Tyler Veness and WPILib!
 *
 * @see <a href="https://en.wikipedia.org/wiki/Linear%E2%80%93quadratic_regulator">LQR on Wikipedia</a>
 */
class LQRController : RobotPosVelController{
    private val k: SimpleMatrix
    private val dt: Double

    /**
     * Constructor for the common holonomic robot control use case.
     *
     * This constructor assumes a kinematic acceleration model where the state error `x` is
     * `[xError, yError, headingError, vxError, vyError, omegaError]` and the control input `u`
     * is the desired robot acceleration `[ax, ay, alpha]`.
     *
     * The system dynamics are approximated as:
     * `dx_dot = vx`
     * `dy_dot = vy`
     * `dtheta_dot = omega`
     * `dvx_dot = ax`
     * `dvy_dot = ay`
     * `domega_dot = alpha`
     *
     * @param params The parameters for the LQR controller; see [LQRParameters] constructor.
     * @param dt The discrete time step (control loop period) in seconds.
     */
    constructor(
        params: LQRParameters,
        dt: Double = 0.0303
    ) {
        require(dt > 0) { "Time step (dt) must be positive" }

        val A_cont = SimpleMatrix(6, 6)
        A_cont.set(0, 3, 1.0) // d(delta_x)/dt = delta_vx
        A_cont.set(1, 4, 1.0) // d(delta_y)/dt = delta_vy
        A_cont.set(2, 5, 1.0) // d(delta_theta)/dt = delta_omega

        val B_cont = SimpleMatrix(6, 3)
        B_cont.set(3, 0, 1.0) // d(delta_vx)/dt = ax_cmd
        B_cont.set(4, 1, 1.0) // d(delta_vy)/dt = ay_cmd
        B_cont.set(5, 2, 1.0) // d(delta_omega)/dt = alpha_cmd

        // discretize continuous A and B matrices for DARE
        val (Ad, Bd) = discretizeSystem(A_cont, B_cont, dt)

        this.k = solveDARE(Ad, Bd, params.Q.simple, params.R.simple)
        this.dt = dt
    }

    /**
     * Constructor for the common holonomic robot control use case.
     *
     * This constructor assumes a kinematic acceleration model where the state error `x` is
     * `[xError, yError, headingError, vxError, vyError, omegaError]` and the control input `u`
     * is the desired robot acceleration `[ax, ay, alpha]`.
     *
     * The system dynamics are approximated as:
     * `dx_dot = vx`
     * `dy_dot = vy`
     * `dtheta_dot = omega`
     * `dvx_dot = ax`
     * `dvy_dot = ay`
     * `domega_dot = alpha`
     *
     * @param qX The penalty for error in the x-position. Higher values prioritize correcting x-error.
     * @param qY The penalty for error in the y-position. Higher values prioritize correcting y-error.
     * @param qHeading The penalty for error in the heading. Higher values prioritize correcting heading error.
     * @param qVx The penalty for error in the x-velocity. Higher values prioritize correcting x-velocity error.
     * @param qVy The penalty for error in the y-velocity. Higher values prioritize correcting y-velocity error.
     * @param qOmega The penalty for error in the angular velocity. Higher values prioritize correcting angular velocity error.
     * @param rAx The penalty for using linear acceleration in the x-direction. Higher values prioritize using less `ax`.
     * @param rAy The penalty for using linear acceleration in the y-direction. Higher values prioritize using less `ay`.
     * @param rAlpha The penalty for using angular acceleration. Higher values prioritize using less `alpha`.
     * @param dt The discrete time step (control loop period) in seconds.
     */
    constructor(
        qX: Double, qY: Double, qHeading: Double,
        qVx: Double, qVy: Double, qOmega: Double,
        rAx: Double, rAy: Double, rAlpha: Double,
        dt: Double = 0.0303
    ) : this(
        LQRParameters(qX, qY, qHeading, qVx, qVy, qOmega, rAx, rAy, rAlpha),
        dt
    )

    /**
     * General-purpose constructor for any linear time-invariant (LTI) system.
     *
     * This constructor is for advanced users who have a full state-space model of their system
     * in the form `ẋ = Ax + Bu`. It computes the optimal gain matrix `K` by iteratively
     * solving the discrete-time Algebraic Riccati Equation (ARE) until convergence.
     *
     * @param A The state matrix.
     * @param B The input matrix.
     * @param Q The state cost matrix.
     * @param R The control cost matrix.
     * @param dt The time step for the discrete-time model (your loop time)
     * @param maxIter The maximum number of iterations for the DARE solver.
     * @param epsilon The convergence tolerance for the DARE solver.
     */
    @JvmOverloads
    constructor(
        A: Matrix,
        B: Matrix,
        Q: Matrix,
        R: Matrix,
        dt: Double = 0.0303,
        maxIter: Int = 100,
        epsilon: Double = 1e-6
    ) {
        require(dt > 0) { "Time step (dt) must be positive" }
        require(A.numRows == A.numCols && A.numRows == Q.numRows && Q.numRows == Q.numCols) { "A and Q must be square and match dimensions." }
        require(B.numRows == A.numRows && B.numCols == R.numRows && R.numRows == R.numCols) { "B and R must have compatible dimensions with A and each other." }

        val (Ad, Bd) = discretizeSystem(A.simple, B.simple, dt)

        this.k = solveDARE(Ad, Bd, Q.simple, R.simple, maxIter, epsilon)
        this.dt = dt
    }

    /**
     * Calculates the optimal control input (accelerations) to correct for the given state error.
     *
     * @param error The current state error of the system, represented as a Matrix.
     * Expected dimensions: num_states x 1.
     * Order of states MUST match the LQR's design: [dx, dy, dtheta, dvx, dvy, domega].
     * @return The calculated optimal control accelerations as a Matrix.
     * The matrix will have dimensions 3 x 1: [ax, ay, alpha].
     * */
    fun update(error: Matrix): Matrix {
        require(error.numCols == k.numCols)

        return Matrix(-k * error.simple)
    }

    /**
     * Calculates the optimal control input to correct for the given state error.
     *
     * @param error The current state error of the system, represented as a dual twist
     *              (position error AND velocity error).
     * @return The calculated optimal control acceleration (`PoseVelocity2dDual`).
     *              Note that despite being a `PoseVelocity2dDual` object, it is an acceleration.
     */
    fun <Param> update(error: Twist2dDual<Param>): PoseVelocity2d {
        require(error.size() >= 2) { "Position and velocity errors must be provided" }

        val posError = error.value()
        val velError = error.velocity().value()

        val input = Matrix(doubleArrayOf(
            posError.line.x, posError.line.y, posError.angle,
            velError.linearVel.x, velError.linearVel.y, velError.angVel
        ))

        val output = update(input)

        return PoseVelocity2d(Vector2d(output[0, 0], output[1, 0]), output[2, 0])
    }

    override fun compute(
        targetPose: Pose2dDual<Time>,
        actualPose: Pose2d,
        actualVelActual: PoseVelocity2d,
    ): PoseVelocity2dDual<Time> {
        val posError = targetPose.value() - actualPose
        val velError = (targetPose.velocity().value() - actualVelActual).let {
            Twist2d(it.linearVel, it.angVel)
        }

        val acc = update(posError.concat<Time>(velError))
        val vel = integrateAccel(acc, actualVelActual, dt)

        return vel.concat(acc)
    }
}

/**
 * Solves the Discrete-Time Algebraic Riccati Equation (DARE) using iterative method.
 * P = A'PA - (A'PB)(R + B'PB)⁻¹(B'PA) + Q
 *
 * @author Tyler Veness (C++ implementation)
 * @author Zach Harel (Kotlin implementation)
 */
internal fun solveDARE(Ad: SimpleMatrix, Bd: SimpleMatrix, Q: SimpleMatrix, R: SimpleMatrix, maxIter: Int = 100, epsilon: Double = 1e-6): SimpleMatrix {
    var A = Ad.copy()
    var G = Bd * R.solve(Bd.transpose())
    var P = Q.copy() // Initial guess P_0 = Q
    var H: SimpleMatrix
    var i = 0

    do {
        H = P.copy()

        val W = SimpleMatrix.identity(Q.numRows) + G * H

        val LU = LUDecompositionAlt_DDRM().let {
            it.decompose(W.ddrm)
            SimpleMatrix.wrap(it.lu)
        }

        val V1 = LU.solve(A)
        val V2 = W.solve(G.transpose()).transpose()

        G += A * V2 * A.transpose()
        P = H + V1.transpose() * H * A
        A *= V1

        i++
    } while (i < maxIter && (P - H).normF() > epsilon * P.normF())


    // K = (R + B'PB)⁻¹(B'PA)
    return (R + Bd.transpose() * P * Bd).invert() * Bd.transpose() * P * Ad
}

/**
 * Discretizes a continuous-time system (A, B) to a discrete-time system (Ad, Bd).
 * Uses the matrix exponential (approximated with a Taylor series).
 * Ad = e^(A*dt)
 * Bd = (∫ e^(Aτ) dτ from 0 to dt) * B ≈ (I*dt + A*dt²/2! + ...) * B
 */
internal fun discretizeSystem(A: SimpleMatrix, B: SimpleMatrix, dt: Double, taylorTerms: Int = 10): Pair<SimpleMatrix, SimpleMatrix> {
    val n = A.numRows
    var Ad = SimpleMatrix.identity(n)
    var Bd_integral = SimpleMatrix.identity(n).times(dt) // Start with I*dt

    var APowerDt = A.times(dt)
    var dtPower = dt
    var factorial = 1.0

    // Taylor series for e^(A*dt) and its integral
    for (i in 1..taylorTerms) {
        // Ad term: (A*dt)^i / i!
        Ad = Ad.plus(APowerDt.times(1.0 / factorial))

        // Bd integral term: A^(i-1) * dt^(i+1) / (i+1)!
        dtPower *= dt
        factorial *= (i + 1)
        Bd_integral = Bd_integral.plus(APowerDt.times(dt / (i + 1)))

        // Prepare for next iteration
        APowerDt = APowerDt.times(A.times(dt))
        factorial *= (i + 1)
    }

    val Bd = Bd_integral.times(B)
    return Pair(Ad, Bd)
}

internal fun <Param> Twist2d.concat(other: Twist2d) = Twist2dDual<Param>(
    this.line.concat(other.line),
    DualNum(doubleArrayOf(angle, other.angle))
)

internal fun <Param> Vector2d.concat(other: Vector2d) = Vector2dDual<Param>(
    DualNum(doubleArrayOf(this.x, other.x)),
    DualNum(doubleArrayOf(this.y, other.y))
)

internal fun <Param> Vector2dDual<Param>.concat(other: Vector2dDual<Param>) = Vector2dDual<Param>(
    DualNum(x.values() + y.values()),
    DualNum(x.values() + y.values())
)

internal fun <Param> PoseVelocity2d.concat(other: PoseVelocity2d) = PoseVelocity2dDual<Param>(
    this.linearVel.concat(other.linearVel),
    DualNum(doubleArrayOf(this.angVel, other.angVel))
)

internal fun <Param> Twist2dDual<Param>.size() = angle.size()

fun integrateAccel(accel: PoseVelocity2d, vi: PoseVelocity2d, dt: Double) = PoseVelocity2d(
    Vector2d(
        vi.linearVel.x + accel.linearVel.x * dt,
        vi.linearVel.y + accel.linearVel.y * dt
    ),
        vi.angVel + accel.angVel * dt
    )

fun integrateVel(a: PoseVelocity2d, v: PoseVelocity2d, x: Pose2d, dt: Double) = Pose2d(
    Vector2d(
        0.5 * a.linearVel.x * dt.pow(2) + v.linearVel.x * dt + x.position.x,
        0.5 * a.linearVel.y * dt.pow(2) + v.linearVel.y * dt + x.position.y,
    ),
    Rotation2d.exp(0.5 * a.angVel * dt.pow(2) + v.angVel * dt + x.heading.log())
)