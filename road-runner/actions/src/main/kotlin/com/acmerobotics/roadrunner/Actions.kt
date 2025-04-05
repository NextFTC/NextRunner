@file:JvmName("Actions")

package com.acmerobotics.roadrunner

import com.acmerobotics.dashboard.canvas.Canvas
import com.acmerobotics.dashboard.telemetry.TelemetryPacket

/**
 * Concurrent task for cooperative multitasking with some FTC dashboard hooks. Actions may have mutable state.
 */
@JvmDefaultWithoutCompatibility
fun interface Action {
    /**
     * Runs a single uninterruptible block. Returns true if the action should run again and false if it has completed.
     * A telemetry packet [p] is provided to record any information on the action's progress.
     */
    fun run(p: TelemetryPacket): Boolean

    /**
     * Draws a preview of the action on canvas [fieldOverlay].
     */
    fun preview(fieldOverlay: Canvas) {}
}

/**
 * Action combinator that executes the action group [initialActions] in series. Each action is run one after the other.
 * When an action completes, the next one is immediately run. This action completes when the last action completes.
 */
data class SequentialAction(
    val initialActions: List<Action>
) : Action {
    private var actions = initialActions

    constructor(vararg actions: Action) : this(actions.asList())

    override tailrec fun run(p: TelemetryPacket): Boolean {
        if (actions.isEmpty()) {
            return false
        }

        return if (actions.first().run(p)) {
            true
        } else {
            actions = actions.drop(1)
            run(p)
        }
    }

    override fun preview(fieldOverlay: Canvas) {
        for (a in initialActions) {
            a.preview(fieldOverlay)
        }
    }
}

/**
 * Action combinator that executes the action group [initialActions] in parallel. Each call to [run] on this action
 * calls [run] on _every_ live child action in the order provided. Completed actions are removed from the rotation
 * and _do not_ prevent the completion of other actions. This action completes when all of [initialActions] have.
 */
data class ParallelAction(
    val initialActions: List<Action>
) : Action {
    private var actions = initialActions

    constructor(vararg actions: Action) : this(actions.asList())

    override fun run(p: TelemetryPacket): Boolean {
        actions = actions.filter { it.run(p) }
        return actions.isNotEmpty()
    }

    override fun preview(fieldOverlay: Canvas) {
        for (a in initialActions) {
            a.preview(fieldOverlay)
        }
    }
}

/**
 * Action combinator that executes the action group [actions] in parallel. Each call to [run] on this action
 * calls [run] on _every_ live child action in the order provided. Once one action ends, all other actions are ended.
*/
data class RaceAction(
    val actions: List<Action>
) : Action {

    constructor(vararg actions: Action) : this(actions.asList())

    override fun run(p: TelemetryPacket): Boolean = !actions.any { !it.run(p) }

    override fun preview(fieldOverlay: Canvas) {
        for (a in actions) {
            a.preview(fieldOverlay)
        }
    }
}

/**
 * Returns [System.nanoTime] in seconds.
 */
fun now() = System.nanoTime() * 1e-9

/**
 * Primitive sleep action that stalls for [dt] seconds.
 */
data class SleepAction(val dt: Double) : Action {
    private var beginTs = -1.0

    override fun run(p: TelemetryPacket): Boolean {
        val t = if (beginTs < 0) {
            beginTs = now()
            0.0
        } else {
            now() - beginTs
        }

        return t < dt
    }

    override fun preview(fieldOverlay: Canvas) {}
}
fun interface InstantFunction {
    fun run()
}

/**
 * Instant action that executes [f] immediately.
 */
class InstantAction(val f: InstantFunction) : Action {
    override fun run(p: TelemetryPacket): Boolean {
        f.run()
        return false
    }
}

/**
 * Null action that does nothing.
 */
class NullAction : Action {
    override fun run(p: TelemetryPacket) = false
}

