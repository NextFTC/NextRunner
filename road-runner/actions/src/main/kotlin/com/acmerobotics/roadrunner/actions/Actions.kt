@file:JvmName("Actions")

package com.acmerobotics.roadrunner.actions

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

    /**
     * Returns a new action that executes this action followed by [a].
     */
    fun then(a: Action) = seqCons(this, a)
    fun then(f: InstantFunction) = seqCons(this, InstantAction(f))
    fun then(a: () -> Action) = seqCons(this, a())

    /**
     * Returns a new action that executes this action in parallel with [a].
     */
    fun with(a: Action) = parCons(this, a)
    fun with(f: InstantFunction) = parCons(this, InstantAction(f))
    fun with(a: () -> Action) = parCons(this, a())

    /**
     * Returns a new action that executes this action in parallel with [a].
     */
    fun race(a: Action) = RaceAction(this, a)
    fun race(f: InstantFunction) = RaceAction(this, InstantAction(f))
    fun race(a: () -> Action) = RaceAction(this, a())

    /**
     * Returns a new action that waits [dt] seconds before executing this action.
     */
    fun delay(dt: Double) = SleepAction(dt).then(this)
}

open class ActionEx @JvmOverloads constructor(
    private val initBlock: (TelemetryPacket) -> Unit = { },
    private val loopBlock: (TelemetryPacket) -> Boolean = { false },
    private val endBlock: (TelemetryPacket) -> Unit = { }
) : Action {

    open fun init(packet: TelemetryPacket) = initBlock(packet)
    open fun loop(packet: TelemetryPacket) = loopBlock(packet)
    open fun end(packet: TelemetryPacket) = endBlock(packet)

    private val sequential = SequentialAction(
        Action { init(it).let { false } },
        Action { loop(it) },
        Action { end(it).let { false } }
    )

    final override fun run(packet: TelemetryPacket) = sequential.run(packet)

    fun withInit(initBlock: (TelemetryPacket) -> Unit) = ActionEx(initBlock, loopBlock, endBlock)
    fun withLoop(loopBlock: (TelemetryPacket) -> Boolean) = ActionEx(initBlock, loopBlock, endBlock)
    fun withEnd(endBlock: (TelemetryPacket) -> Unit) = ActionEx(initBlock, loopBlock, endBlock)
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

internal fun seqCons(hd: Action, tl: Action): Action =
    when (tl) {
        is NullAction -> hd
        is SequentialAction -> SequentialAction(listOf(hd) + tl.initialActions)
        else -> SequentialAction(hd, tl)
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

internal fun parCons(hd: Action, tl: Action): Action =
    when (tl) {
        is NullAction -> hd
        is ParallelAction -> ParallelAction(listOf(hd) + tl.initialActions)
        else -> ParallelAction(hd, tl)
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

