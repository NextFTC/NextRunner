@file:JvmName("Actions")

package com.acmerobotics.roadrunner.ftc

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.canvas.Canvas
import com.acmerobotics.dashboard.telemetry.TelemetryPacket
import com.acmerobotics.roadrunner.actions.Action
import com.qualcomm.robotcore.eventloop.opmode.OpMode
import com.qualcomm.robotcore.eventloop.opmode.OpModeManagerNotifier

/**
 * Run [a] to completion in a blocking loop.
 */
fun runBlocking(a: Action) {
    val dash = FtcDashboard.getInstance()
    val c = Canvas()
    a.preview(c)

    var b = true
    while (b && !Thread.currentThread().isInterrupted) {
        val p = TelemetryPacket()
        p.fieldOverlay().operations.addAll(c.operations)

        b = a.run(p)

        dash.sendTelemetryPacket(p)
    }
}

/**
 * Singleton object responsible for managing and updating a queue of concurrent asynchronous tasks (actions).
 * Provides methods to enqueue actions and process the queue during the robot's main loop.
 *
 * Implements `OpModeManagerNotifier.Notifications` to manage the actions' lifecycle across different OpMode stages.
 */
object ActionRunner : OpModeManagerNotifier.Notifications {
    private val dash = lazy { FtcDashboard.getInstance() }
    private val _actions = ArrayDeque<Action>()

    /**
     * The actions currently being run.
     */
    @JvmStatic
    @get:JvmName("actions")
    val actions: List<Action>
        get() = _actions

    /**
     * Adds [action] to the run queue.
     */
    @JvmStatic
    fun run(action: Action) {
        _actions.addLast(action)
    }

    /**
     * Adds all actions in [actions] to the run queue.
     */
    @JvmStatic
    fun run(actions: Collection<Action>) {
        _actions.addAll(actions)
    }

    /**
     * Adds all actions in [actions] to the run queue.
     */
    @JvmStatic
    fun run(vararg actions: Action) {
        _actions.addAll(actions)
    }

    /**
     * Updates the run queue.
     * MUST be called at the end of every loop.
     */
    @JvmStatic
    fun update() {
        val p = TelemetryPacket()
        _actions.retainAll {
            it.run(p)
        }
        dash.value.sendTelemetryPacket(p)
    }

    override fun onOpModePreInit(p0: OpMode?) {
        require(dash.isInitialized())
        require(_actions.isEmpty())
    }

    override fun onOpModePreStart(p0: OpMode?) {}

    override fun onOpModePostStop(p0: OpMode?) {
        _actions.clear()
    }
}
