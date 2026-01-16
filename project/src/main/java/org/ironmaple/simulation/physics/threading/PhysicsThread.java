package org.ironmaple.simulation.physics.threading;

import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Time;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.Optional;
import java.util.concurrent.CountDownLatch;
import java.util.concurrent.Semaphore;
import java.util.concurrent.atomic.AtomicReference;
import java.util.concurrent.locks.LockSupport;
import org.ironmaple.simulation.debugging.SimDebugLogger;
import org.ironmaple.simulation.physics.PhysicsBody;
import org.ironmaple.simulation.physics.PhysicsEngine;

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
    private final PhysicsEngine engine;
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
    private double accumulatedTime = 0.0;

    // Debug: Enable to log frame IDs for pipeline verification
    private static final boolean DEBUG_FRAME_IDS = true;

    // Lock-step synchronization for deterministic physics
    // frameReadySemaphore: Main thread releases to signal "process this frame"
    // frameCompleteSemaphore: Physics thread releases when frame is done
    private final Semaphore frameReadySemaphore = new Semaphore(0);
    private final Semaphore frameCompleteSemaphore = new Semaphore(0);
    private volatile boolean lockStepMode = true; // Default to deterministic mode

    // Initialization synchronization - signals when engine is ready
    private final CountDownLatch initializationLatch = new CountDownLatch(1);

    // NetworkTables metrics
    private final DoublePublisher tickTimePublisher;
    private final DoublePublisher targetTickTimePublisher;
    private final DoublePublisher inputQueueDepthPublisher;

    /**
     * Creates a new physics thread.
     *
     * @param engine The physics engine (must be initialized)
     * @param config Threading configuration
     */
    public PhysicsThread(PhysicsEngine engine, PhysicsThreadConfig config) {
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
        // Initialize engine on this thread to maintain thread affinity
        engine.initialize();
        initializationLatch.countDown(); // Signal that initialization is complete

        final int subTicks = config.subTicksPerFrame();
        final long tickPeriodNanos = config.tickPeriodNanos();
        // Do NOT divide by subTicks. checking rateHz is the PHYSICS tick rate (e.g.
        // 100Hz -> 0.01s).
        // We want to run 'subTicks' steps of this size per frame.
        final Time subTickDt = Units.Seconds.of(config.tickPeriodSeconds());
        final double subTickDtSeconds = config.tickPeriodSeconds();

        // For free-running mode fallback
        long nextTickNanos = System.nanoTime();
        long lastLoopTime = System.nanoTime();

        while (running) {
            // ==================== LOCK-STEP MODE ====================
            // Wait for main thread to signal that a frame is ready to process.
            // This ensures deterministic timing: physics only runs when the main
            // thread has prepared inputs, eliminating wall-clock jitter.
            if (lockStepMode) {
                try {
                    frameReadySemaphore.acquire();
                } catch (InterruptedException e) {
                    if (!running) break;
                    Thread.currentThread().interrupt();
                    continue;
                }
                if (!running) break;
            }

            long tickStart = System.nanoTime();

            // Sync bodies from engine (captures bodies added by main thread)
            syncBodies();

            // Process pending inputs (guaranteed to be present in lock-step mode)
            SimulationInputs inputs = pendingInputs.getAndSet(null);
            if (inputs != null) {
                if (DEBUG_FRAME_IDS) {
                    System.out.println("[PhysicsThread] Processing frame " + inputs.frameNumber() + " at physics tick "
                            + tickCount);
                }
                applyInputs(inputs);
            } else if (DEBUG_FRAME_IDS && lockStepMode) {
                System.err.println("[PhysicsThread] WARNING: No inputs in lock-step mode at tick " + tickCount);
            }

            // --- Physics Application ---
            if (lockStepMode) {
                // STRICT MODE: Always run exactly 'subTicks' steps.
                // This assumes the main thread is running at 50Hz and we are configured for
                // 100Hz (2 steps).
                // We ignore wall clock time completely to ensure determinism.
                for (int i = 0; i < subTicks; i++) {
                    // Run physics calculators with CURRENT state
                    for (PhysicsCalculator calc : calculators) {
                        calc.applyForces(engine);
                    }

                    engine.step(subTickDt);
                    simulationTimeSeconds += subTickDtSeconds;
                }
            } else {
                // Free-running mode (Accumulator fallback)
                long now = System.nanoTime();
                double deltaTimeSeconds = (now - lastLoopTime) / 1_000_000_000.0;
                lastLoopTime = now;
                accumulatedTime += deltaTimeSeconds;

                int stepsRun = 0;
                while (accumulatedTime >= subTickDtSeconds) {
                    for (PhysicsCalculator calc : calculators) {
                        calc.applyForces(engine);
                    }
                    engine.step(subTickDt);
                    simulationTimeSeconds += subTickDtSeconds;
                    accumulatedTime -= subTickDtSeconds;
                    stepsRun++;

                    if (stepsRun >= config.maxCatchupTicks()) {
                        accumulatedTime = 0;
                        break;
                    }
                }
            }

            tickCount++;

            // Capture and publish state
            SimulationState state = captureState(inputs);
            currentState.set(state);

            // Calculate timing metrics
            long elapsed = System.nanoTime() - tickStart;
            double elapsedMs = elapsed / 1_000_000.0;
            lastTickDurationSeconds = elapsed / 1_000_000_000.0;
            tickTimePublisher.set(elapsedMs);

            if (tickCount % 120 == 0) {
                SimDebugLogger.logPerformance(String.format(
                        "Physics Thread Tick: %d, Bodies: %d, Time: %.3f ms", tickCount, bodyById.size(), elapsedMs));
            }

            // ==================== SIGNAL COMPLETION ====================
            if (lockStepMode) {
                // Signal main thread that this frame is complete
                // In Pipelined mode (Option C), Main thread might not be waiting,
                // but we signal anyway for the N-1 handshake.
                frameCompleteSemaphore.release();
            } else {
                // Free-running mode pacing
                nextTickNanos += tickPeriodNanos;
                long waitNanos = nextTickNanos - System.nanoTime();

                if (waitNanos > 0) {
                    LockSupport.parkNanos(waitNanos);
                } else {
                    nextTickNanos = System.nanoTime();
                }
            }
        }
    }

    /** Synchronizes local body map with engine's live bodies. */
    private void syncBodies() {
        // This is inefficient if getBodies returns a new list every time
        // But needed if main thread adds bodies directly
        for (PhysicsBody body : engine.getBodies()) {
            bodyById.put(body.getBodyId(), body);
        }
    }

    /** Applies input commands to the physics world. */
    private void applyInputs(SimulationInputs inputs) {
        inputQueueDepthPublisher.set(inputs.forces().size()
                + inputs.torques().size()
                + inputs.spawns().size()
                + inputs.removals().size()
                + inputs.bodiesToAdd().size()
                + inputs.newCalculators().size());

        // Process existing body additions FIRST and add to bodyById immediately
        // This allows forces queued in the same batch to find these bodies
        for (PhysicsBody body : inputs.bodiesToAdd()) {
            engine.addBody(body);
            bodyById.put(body.getBodyId(), body);
        }

        // Process body spawns (also add to bodyById)
        for (var spawn : inputs.spawns()) {
            PhysicsBody newBody = engine.createDynamicBody(spawn.shape(), spawn.massKg(), spawn.initialPose());
            bodyById.put(newBody.getBodyId(), newBody);
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
                if (reset.linearVelocity() != null) {
                    body.setLinearVelocityMPS(reset.linearVelocity());
                }
                if (reset.angularVelocity() != null) {
                    body.setAngularVelocityRadPerSec(reset.angularVelocity());
                }
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

        // Register new calculators FIRST, before applying swerve inputs
        // This ensures newly registered calculators receive their swerve input
        // on the same frame they're added (critical for determinism on first frame)
        for (PhysicsCalculator calculator : inputs.newCalculators()) {
            calculators.add(calculator);
        }

        // Update physics calculators with swerve inputs AFTER adding new calculators
        // This ensures the new calculator gets its capturedStates set immediately,
        // avoiding the fallback path that reads live (non-deterministic) state.
        inputs.swerveInput().ifPresent(input -> {
            for (PhysicsCalculator calc : calculators) {
                if (calc instanceof ThreadedSwerveCalculator swerveCalc) {
                    swerveCalc.setCapturedStates(input.moduleStates());
                }
            }
        });
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
        bodyById.put(body.getBodyId(), body);
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

    /**
     * Waits for the physics engine to complete initialization.
     *
     * <p>Call this after starting the thread to ensure the engine is ready before creating bodies.
     *
     * @throws InterruptedException if interrupted while waiting
     */
    public void waitForInitialization() throws InterruptedException {
        initializationLatch.await();
    }

    /** Requests graceful shutdown. */
    public void shutdown() {
        running = false;
        // Wake thread from any blocking state
        frameReadySemaphore.release();
        LockSupport.unpark(this);
    }

    /**
     * Signals the physics thread to process a frame.
     *
     * <p>In lock-step mode, this wakes the physics thread to process the pending inputs. The physics thread will run
     * exactly {@code subTicksPerFrame} sub-steps and then signal completion.
     *
     * <p>Call this AFTER submitting inputs via {@link #submitInputs(SimulationInputs)}.
     */
    public void signalFrameReady() {
        frameReadySemaphore.release();
    }

    /**
     * Waits for the physics thread to complete the current frame.
     *
     * <p>In lock-step mode, blocks until the physics thread has finished processing the frame that was signaled by
     * {@link #signalFrameReady()}. This ensures the latest state is available before the main thread continues.
     *
     * <p>For pipelining: you can choose NOT to wait, allowing the main thread to continue with the previous frame's
     * state while physics processes in parallel. Call this method if you need the latest state immediately.
     *
     * @return true if the frame completed, false if interrupted
     */
    public boolean waitForFrameComplete() {
        try {
            frameCompleteSemaphore.acquire();
            return true;
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
            return false;
        }
    }

    /**
     * Waits for frame completion with a timeout.
     *
     * @param timeoutMs Maximum time to wait in milliseconds
     * @return true if the frame completed, false if timeout or interrupted
     */
    public boolean waitForFrameComplete(long timeoutMs) {
        try {
            return frameCompleteSemaphore.tryAcquire(timeoutMs, java.util.concurrent.TimeUnit.MILLISECONDS);
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
            return false;
        }
    }

    /**
     * Checks if a frame has completed without blocking.
     *
     * @return true if a frame completion signal is available
     */
    public boolean isFrameComplete() {
        return frameCompleteSemaphore.tryAcquire();
    }

    /**
     * Sets whether to use lock-step mode (deterministic) or free-running mode.
     *
     * <p>Lock-step mode (default): Physics waits for explicit frame signals from the main thread. This ensures perfect
     * determinism but requires the main thread to drive the physics timing.
     *
     * <p>Free-running mode: Physics runs at its configured tick rate using wall-clock timing. This allows physics to
     * run faster than the main thread but introduces timing jitter that can cause non-deterministic behavior.
     *
     * @param lockStep true for deterministic lock-step, false for free-running
     */
    public void setLockStepMode(boolean lockStep) {
        this.lockStepMode = lockStep;
    }

    /**
     * Returns whether lock-step mode is enabled.
     *
     * @return true if in lock-step mode
     */
    public boolean isLockStepMode() {
        return lockStepMode;
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
