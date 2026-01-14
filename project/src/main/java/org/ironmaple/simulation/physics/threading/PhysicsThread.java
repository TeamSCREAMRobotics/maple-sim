package org.ironmaple.simulation.physics.threading;

import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Time;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.Optional;
import java.util.concurrent.atomic.AtomicReference;
import java.util.concurrent.locks.LockSupport;
import org.ironmaple.simulation.debugging.SimDebugLogger;
import org.ironmaple.simulation.physics.PhysicsBody;
import org.ironmaple.simulation.physics.PhysicsEngine;
import org.ironmaple.simulation.physics.bullet.BulletBody;
import org.ironmaple.simulation.physics.bullet.BulletPhysicsEngine;

/**
 * Dedicated thread for running Bullet physics simulation.
 *
 * <h2>Thread Safety</h2>
 *
 * <p>All Bullet API calls are confined to this thread. Communication with the main thread uses lock-free
 * AtomicReferences:
 *
 * <ul>
 *   <li>{@code pendingInputs} - Main thread writes, physics thread reads
 *   <li>{@code currentState} - Physics thread writes, main thread reads
 * </ul>
 *
 * <h2>Timing</h2>
 *
 * <p>Uses {@link LockSupport#parkNanos} for precise timing. If physics falls behind, it will run catch-up ticks up to
 * {@link PhysicsThreadConfig#maxCatchupTicks()}.
 */
public class PhysicsThread extends Thread {
    private final BulletPhysicsEngine engine;
    private final PhysicsThreadConfig config;

    // Lock-free communication channels
    private final AtomicReference<SimulationInputs> pendingInputs = new AtomicReference<>(SimulationInputs.EMPTY);
    private final AtomicReference<SimulationState> currentState = new AtomicReference<>(SimulationState.EMPTY);

    // Body ID tracking
    private final Map<Integer, PhysicsBody> bodyById = new HashMap<>();

    // Physics calculators that run each tick (e.g., swerve suspension/traction)
    private final List<PhysicsCalculator> calculators = new java.util.concurrent.CopyOnWriteArrayList<>();

    // Timing state
    private volatile boolean running = true;
    private long tickCount = 0;
    private double simulationTimeSeconds = 0.0;
    private double lastTickDurationSeconds = 0.0;

    // NetworkTables metrics
    private final DoublePublisher tickTimePublisher;
    private final DoublePublisher targetTickTimePublisher;
    private final DoublePublisher inputQueueDepthPublisher;

    /**
     * Creates a new physics thread.
     *
     * @param engine The Bullet physics engine (must be initialized)
     * @param config Threading configuration
     */
    public PhysicsThread(BulletPhysicsEngine engine, PhysicsThreadConfig config) {
        super("MapleSim-PhysicsThread");
        this.engine = engine;
        this.config = config;
        setDaemon(true); // Don't prevent JVM shutdown

        // Setup NetworkTables metrics
        var table = NetworkTableInstance.getDefault().getTable("SmartDashboard/MapleSim3D/Physics");
        tickTimePublisher = table.getDoubleTopic("TickTimeMs").publish();
        targetTickTimePublisher = table.getDoubleTopic("TargetTickTimeMs").publish();
        inputQueueDepthPublisher = table.getDoubleTopic("InputQueueDepth").publish();

        targetTickTimePublisher.set(config.tickPeriodNanos() / 1_000_000.0);
    }

    @Override
    public void run() {
        long nextTickNanos = System.nanoTime();
        final long tickPeriodNanos = config.tickPeriodNanos();
        final Time tickDt = Units.Seconds.of(config.tickPeriodSeconds());

        // Initialize engine on this thread if needed
        engine.initialize();

        while (running) {
            long tickStart = System.nanoTime();

            // Sync bodies from engine (captures bodies added by main thread)
            syncBodies();

            // Process pending inputs
            SimulationInputs inputs = pendingInputs.getAndSet(null);
            if (inputs != null) {
                applyInputs(inputs);
            }

            // Run physics calculators (suspension, traction, etc.) with CURRENT state
            for (PhysicsCalculator calc : calculators) {
                calc.applyForces(engine);
            }

            // Step physics
            engine.step(tickDt);
            tickCount++;
            simulationTimeSeconds += config.tickPeriodSeconds();

            // Capture and publish state (using previous tick's duration)
            SimulationState state = captureState(inputs);
            currentState.set(state);

            // Calculate timing
            long elapsed = System.nanoTime() - tickStart;
            double elapsedMs = elapsed / 1_000_000.0;
            lastTickDurationSeconds = elapsed / 1_000_000_000.0;
            tickTimePublisher.set(elapsedMs);

            if (tickCount % 120 == 0) {
                SimDebugLogger.logPerformance(String.format(
                        "Physics Thread Tick: %d, Bodies: %d, Time: %.3f ms", tickCount, bodyById.size(), elapsedMs));
            }

            // Wait for next tick
            nextTickNanos += tickPeriodNanos;
            long waitNanos = nextTickNanos - System.nanoTime();

            if (waitNanos > 0) {
                LockSupport.parkNanos(waitNanos);
            } else {
                // Falling behind - try to catch up, but limit
                int ticksBehind = (int) (-waitNanos / tickPeriodNanos);
                if (ticksBehind > config.maxCatchupTicks()) {
                    // Reset timing - we're too far behind
                    nextTickNanos = System.nanoTime() + tickPeriodNanos;
                }
            }
        }
    }

    /** Synchronizes local body map with engine's live bodies. */
    private void syncBodies() {
        for (PhysicsBody body : engine.getBodies()) {
            if (body instanceof BulletBody bb) {
                bodyById.put(bb.getBodyId(), body);
            }
        }
    }

    /** Applies input commands to the physics world. */
    private void applyInputs(SimulationInputs inputs) {
        inputQueueDepthPublisher.set(inputs.forces().size()
                + inputs.torques().size()
                + inputs.spawns().size()
                + inputs.removals().size()
                + inputs.bodiesToAdd().size());

        // Process existing body additions FIRST and add to bodyById immediately
        // This allows forces queued in the same batch to find these bodies
        for (PhysicsBody body : inputs.bodiesToAdd()) {
            engine.addBody(body);
            if (body instanceof BulletBody bb) {
                bodyById.put(bb.getBodyId(), body);
            }
        }

        // Process body spawns (also add to bodyById)
        for (var spawn : inputs.spawns()) {
            PhysicsBody newBody = engine.createDynamicBody(spawn.shape(), spawn.massKg(), spawn.initialPose());
            if (newBody instanceof BulletBody bb) {
                bodyById.put(bb.getBodyId(), newBody);
            }
        }

        // Process body removals
        for (int bodyId : inputs.removals()) {
            PhysicsBody body = bodyById.remove(bodyId);
            if (body != null) {
                engine.removeBody(body);
            }
        }

        // Process pose resets (teleports)
        for (var reset : inputs.poseResets()) {
            PhysicsBody body = bodyById.get(reset.bodyId());
            if (body != null) {
                body.setPose3d(reset.newPose());
            }
        }

        // Process velocity resets
        for (var reset : inputs.velocityResets()) {
            PhysicsBody body = bodyById.get(reset.bodyId());
            if (body != null) {
                body.setLinearVelocityMPS(reset.linearVelocity());
                body.setAngularVelocityRadPerSec(reset.angularVelocity());
            }
        }

        // Apply forces (now can find bodies added in the same batch)
        for (var force : inputs.forces()) {
            PhysicsBody body = bodyById.get(force.bodyId());
            if (body != null) {
                if (force.point() != null) {
                    body.applyForceAtPoint(force.force(), force.point());
                } else {
                    body.applyForce(force.force());
                }
            }
        }

        // Apply torques
        for (var torque : inputs.torques()) {
            PhysicsBody body = bodyById.get(torque.bodyId());
            if (body != null) {
                body.applyTorque(torque.torque());
            }
        }

        // Apply global gravity update
        inputs.newGravity().ifPresent(engine::setGravity);
    }

    /** Captures the current physics state as an immutable snapshot. */
    private SimulationState captureState(SimulationInputs inputs) {
        SimulationState.Builder builder = SimulationState.builder()
                .tickNumber(tickCount)
                .simulationTime(simulationTimeSeconds)
                .lastTickDuration(lastTickDurationSeconds);

        // Capture all body states
        for (var entry : bodyById.entrySet()) {
            int bodyId = entry.getKey();
            PhysicsBody body = entry.getValue();

            builder.addBodyState(new SimulationState.BodyState(
                    bodyId, body.getPose3d(), body.getLinearVelocityMPS(), body.getAngularVelocityRadPerSec()));
        }

        // Process raycast requests
        if (inputs != null) {
            for (var request : inputs.raycastRequests()) {
                Optional<PhysicsEngine.RaycastResult> result =
                        engine.raycast(request.origin(), request.direction(), request.maxDistance());
                if (result.isPresent()) {
                    builder.addRaycastResult(request.requestId(), result.get());
                }
            }
        }

        return builder.build();
    }

    /**
     * Submits inputs to be applied on the next physics tick.
     *
     * <p>If there are already pending inputs, they are replaced (not merged). This is intentional - the latest state
     * should drive the simulation.
     *
     * @param inputs The inputs to apply
     */
    public void submitInputs(SimulationInputs inputs) {
        pendingInputs.set(inputs);
    }

    /**
     * Gets the latest physics state snapshot.
     *
     * @return The most recent state, never null (may be EMPTY initially)
     */
    public SimulationState getLatestState() {
        return currentState.get();
    }

    /**
     * Registers an existing body with the thread's tracking.
     *
     * <p>Used for bodies created during initialization (before thread starts).
     *
     * @param body The body to register
     */
    public void registerBody(PhysicsBody body) {
        if (body instanceof BulletBody bb) {
            bodyById.put(bb.getBodyId(), body);
        }
    }

    /**
     * Registers a physics calculator to run each tick.
     *
     * <p>Calculators are called in registration order, before engine.step(). They receive the physics engine and can
     * apply forces using current state.
     *
     * @param calculator The calculator to register
     */
    public void registerCalculator(PhysicsCalculator calculator) {
        calculators.add(calculator);
    }

    /** Requests graceful shutdown. */
    public void shutdown() {
        running = false;
        LockSupport.unpark(this); // Wake from any sleep
    }

    /**
     * Checks if the physics thread is currently running.
     *
     * @return true if running
     */
    public boolean isRunning() {
        return running && isAlive();
    }

    /**
     * Gets the current tick count.
     *
     * @return Number of physics ticks completed
     */
    public long getTickCount() {
        return tickCount;
    }

    /**
     * Gets the simulated time in seconds.
     *
     * @return Total simulated time
     */
    public double getSimulationTimeSeconds() {
        return simulationTimeSeconds;
    }
}
