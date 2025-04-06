package com.acmerobotics.roadrunner.control

import com.acmerobotics.roadrunner.geometry.Arclength
import com.acmerobotics.roadrunner.geometry.DualNum
import com.acmerobotics.roadrunner.geometry.Pose2dDual
import com.acmerobotics.roadrunner.geometry.PoseVelocity2dDual
import com.acmerobotics.roadrunner.geometry.Twist2dDual
import com.acmerobotics.roadrunner.geometry.Vector2dDual
import com.acmerobotics.roadrunner.paths.PosePath
import com.acmerobotics.roadrunner.paths.VelConstraint
import kotlin.math.abs

interface WheelIncrements<Param>

/**
 * Represents the velocities of the individual drive wheels.
 */
interface WheelVelocities<Param> {
    /**
     * Returns a list of all wheel velocities.
     * The order should be consistent for a given implementation.
     */
    fun all(): List<DualNum<Param>>
}

/**
 * Represents the kinematics of a robot drive train, providing methods for
 * inverse kinematics and velocity constraints based on wheel speeds.
 */
interface RobotKinematics<in WI: WheelIncrements<*>, out WV: WheelVelocities<*>> {
    fun <Param> forward(w: WI): Twist2dDual<Param>

    /**
     * Performs inverse kinematics: computes wheel velocities required to achieve
     * the desired robot velocity.
     *
     * @param t Robot velocity in the robot's local frame.
     * @return Wheel velocities.
     */
    fun <Param> inverse(t: PoseVelocity2dDual<Param>): WheelVelocities<Param>
}

/**
 * @param[trackWidth] distance between wheels on opposite sides; see the diagram below
 * ![Wheelbase and track width diagram](https://upload.wikimedia.org/wikipedia/commons/5/52/Wheelbase_and_Track.png)
 * @param[lateralMultiplier] factor that multiplies strafe velocity to compensate for slip; increase it to boost the
 * distance traveled in the strafe direction
 */
data class MecanumKinematics @JvmOverloads constructor(
    @JvmField
    val trackWidth: Double,
    @JvmField
    val lateralMultiplier: Double = 1.0
) : RobotKinematics<MecanumKinematics.MecanumWheelIncrements<*>, WheelVelocities<*>> {
    /**
     * @param[wheelbase] distance between wheels on the same side; see the diagram in [MecanumKinematics]
     */
    constructor(
        trackWidth: Double,
        wheelbase: Double,
        lateralMultiplier: Double = 1.0
    ) : this((trackWidth + wheelbase) / 2, lateralMultiplier)

    data class MecanumWheelIncrements<Param>(
        @JvmField
        val leftFront: DualNum<Param>,
        @JvmField
        val leftBack: DualNum<Param>,
        @JvmField
        val rightBack: DualNum<Param>,
        @JvmField
        val rightFront: DualNum<Param>,
    ) : WheelIncrements<Param>

    override fun <Param> forward(w: MecanumWheelIncrements<*>): Twist2dDual<Param> {
        w as MecanumWheelIncrements<Param>

        return Twist2dDual(
            Vector2dDual(
                (w.leftFront + w.leftBack + w.rightBack + w.rightFront) * 0.25,
                (-w.leftFront + w.leftBack - w.rightBack + w.rightFront) * (0.25 / lateralMultiplier),
            ),
            (-w.leftFront - w.leftBack + w.rightBack + w.rightFront) * (0.25 / trackWidth),
        )
    }

    data class MecanumWheelVelocities<Param>(
        @JvmField
        val leftFront: DualNum<Param>,
        @JvmField
        val leftBack: DualNum<Param>,
        @JvmField
        val rightBack: DualNum<Param>,
        @JvmField
        val rightFront: DualNum<Param>,
    ) : WheelVelocities<Param> {
        override fun all() = listOf(leftFront, leftBack, rightBack, rightFront)
    }

    override fun <Param> inverse(t: PoseVelocity2dDual<Param>) = MecanumWheelVelocities(
        t.linearVel.x - t.linearVel.y * lateralMultiplier - t.angVel * trackWidth,
        t.linearVel.x + t.linearVel.y * lateralMultiplier - t.angVel * trackWidth,
        t.linearVel.x - t.linearVel.y * lateralMultiplier + t.angVel * trackWidth,
        t.linearVel.x + t.linearVel.y * lateralMultiplier + t.angVel * trackWidth,
    )
}

/**
 * @param[trackWidth] distance between wheels on opposite sides; see the diagram below
 * ![Wheelbase and track width diagram](https://upload.wikimedia.org/wikipedia/commons/5/52/Wheelbase_and_Track.png)
 */
data class TankKinematics(@JvmField val trackWidth: Double) :
    RobotKinematics<TankKinematics.TankWheelIncrements<*>, TankKinematics.TankWheelVelocities<*>> {

    data class TankWheelIncrements<Param>(
        @JvmField
        val left: DualNum<Param>,
        @JvmField
        val right: DualNum<Param>,
    ) : WheelIncrements<Param>

    override fun <Param> forward(w: TankWheelIncrements<*>): Twist2dDual<Param> {
        w as TankWheelIncrements<Param>

        return Twist2dDual(
            Vector2dDual(
                (w.left + w.right) * 0.5,
                DualNum.constant(0.0, w.left.size()),
            ),
            (-w.left + w.right) / trackWidth,
        )
    }

    data class TankWheelVelocities<Param>(
        @JvmField
        val left: DualNum<Param>,
        @JvmField
        val right: DualNum<Param>,
    ) : WheelVelocities<Param> {
        override fun all() = listOf(left, right)
    }

    override fun <Param> inverse(t: PoseVelocity2dDual<Param>): TankWheelVelocities<Param> {
        require(t.linearVel.y.values().all { abs(it) < 1e-6 }) {
            "Tank drive does not support lateral motion"
        }

        return TankWheelVelocities(
            t.linearVel.x - t.angVel * 0.5 * trackWidth,
            t.linearVel.x + t.angVel * 0.5 * trackWidth,
        )
    }
}

class WheelVelConstraint(
    @JvmField
    val kinematics: RobotKinematics<*, *>,
    @JvmField
    val maxWheelVel: Double
) : VelConstraint {
    override fun maxRobotVel(robotPose: Pose2dDual<Arclength>, path: PosePath, s: Double): Double {
        val txRobotWorld = robotPose.value().inverse()
        val robotVelWorld = robotPose.velocity().value()
        val robotVelRobot = txRobotWorld * robotVelWorld

        return kinematics.inverse(PoseVelocity2dDual.constant<Arclength>(robotVelRobot, 1))
            .all()
            .minOf { abs(maxWheelVel / it.value()) }
    }
}
