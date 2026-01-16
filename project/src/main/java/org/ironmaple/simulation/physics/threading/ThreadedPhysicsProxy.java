package org.ironmaple.simulation.physics.threading;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Translation3d;
import java.util.HashMap;
import java.util.Map;
import java.util.Optional;
import org.ironmaple.simulation.physics.PhysicsBody;
import org.ironmaple.simulation.physics.PhysicsEngine;
import org.ironmaple.simulation.physics.PhysicsShape;

/**
 * Thread-safe proxy for physics operations in threaded mode.
 *
 * <h2>Design</h2>
 *
 * <p>Instead of directly calling the physics engine, this proxy:
 *
 * <ul>
 *   <li>Queues force/torque/spawn commands in a builder
 *   <li>Returns cached state from the last physics tick
 *   <li>Batches all commands and submits them atomically
 * </ul>
 *
 * <h2>Usage Pattern</h2>
 *
 * <pre>
 * // Each simulation period:
 * proxy.pullLatestState(); // Get latest from physics thread
 * // ... run swerve simulation with cached state ...
 * proxy.flushInputs(); // Submit accumulated commands
 * </pre>
 */
public class ThreadedPhysicsProxy {
    private final PhysicsThread physicsThread;
    private final SimulationInputs.Builder inputBuilder = SimulationInputs.builder();

    private SimulationState cachedState = SimulationState.EMPTY;
    private long frameNumber = 0;

    // Raycast request ID counter (per frame)
    private int nextRaycastId = 0;

    // Map of pending raycast requests for result retrieval
    private final Map<Integer, RaycastRequest> pendingRaycasts = new HashMap<>();

    /**
     * Creates a new threaded physics proxy.
     *
     * @param physicsThread The physics thread to communicate with
     */
    public ThreadedPhysicsProxy(PhysicsThread physicsThread) {
        this.physicsThread = physicsThread;
    }

    // ==================== State Access ====================

    /**
     * Pulls the latest state from the physics thread.
     *
     * <p>Call this at the START of each simulation period.
     */
    public void pullLatestState() {
        cachedState = physicsThread.getLatestState();
        nextRaycastId = 0;
        pendingRaycasts.clear();
    }

    /**
     * Gets the cached pose of a body.
     *
     * @param bodyId The body ID
     * @return The pose, or identity if not found
     */
    public Pose3d getCachedBodyPose(int bodyId) {
        SimulationState.BodyState state = cachedState.getBodyState(bodyId);
        return state != null ? state.pose() : new Pose3d();
    }

    /**
     * Gets the cached linear velocity of a body.
     *
     * @param bodyId The body ID
     * @return The velocity, or zero if not found
     */
    public Translation3d getCachedBodyLinearVelocity(int bodyId) {
        SimulationState.BodyState state = cachedState.getBodyState(bodyId);
        return state != null ? state.linearVelocity() : new Translation3d();
    }

    /**
     * Gets the cached angular velocity of a body.
     *
     * @param bodyId The body ID
     * @return The angular velocity, or zero if not found
     */
    public Translation3d getCachedBodyAngularVelocity(int bodyId) {
        SimulationState.BodyState state = cachedState.getBodyState(bodyId);
        return state != null ? state.angularVelocity() : new Translation3d();
    }

    /**
     * Calculates velocity at a point using cached state.
     *
     * @param bodyId The body ID
     * @param pointWorld The point in world coordinates
     * @return The velocity at that point
     */
    public Translation3d getCachedVelocityAtPoint(int bodyId, Translation3d pointWorld) {
        SimulationState.BodyState state = cachedState.getBodyState(bodyId);
        if (state == null) return new Translation3d();

        // v_point = v_com + omega x (point - com)
        Translation3d comWorld = state.pose().getTranslation();
        Translation3d relOffset = pointWorld.minus(comWorld);

        Translation3d linearVel = state.linearVelocity();
        Translation3d angVel = state.angularVelocity();

        // Cross product: omega x r
        double ox = angVel.getX(), oy = angVel.getY(), oz = angVel.getZ();
        double rx = relOffset.getX(), ry = relOffset.getY(), rz = relOffset.getZ();

        Translation3d tangential = new Translation3d(oy * rz - oz * ry, oz * rx - ox * rz, ox * ry - oy * rx);

        return linearVel.plus(tangential);
    }

    /**
     * Gets a cached raycast result.
     *
     * @param requestId The request ID from queueRaycast
     * @return The result, or empty if not found
     */
    public Optional<PhysicsEngine.RaycastResult> getCachedRaycast(int requestId) {
        PhysicsEngine.RaycastResult result = cachedState.getCachedRaycast(requestId);
        return Optional.ofNullable(result);
    }

    /**
     * Gets the current state snapshot.
     *
     * @return The cached simulation state
     */
    public SimulationState getCachedState() {
        return cachedState;
    }

    // ==================== Input Queueing ====================

    /**
     * Queues a force to be applied at a point.
     *
     * @param bodyId The body ID
     * @param force The force in Newtons
     * @param point The application point in world coordinates
     */
    public void queueForceAtPoint(int bodyId, Translation3d force, Translation3d point) {
        inputBuilder.addForce(bodyId, force, point);
    }

    /**
     * Queues a central force to be applied.
     *
     * @param bodyId The body ID
     * @param force The force in Newtons
     */
    public void queueCentralForce(int bodyId, Translation3d force) {
        inputBuilder.addCentralForce(bodyId, force);
    }

    /**
     * Queues a torque to be applied.
     *
     * @param bodyId The body ID
     * @param torque The torque in Newton-meters
     */
    public void queueTorque(int bodyId, Translation3d torque) {
        inputBuilder.addTorque(bodyId, torque);
    }

    /**
     * Queues a body spawn request.
     *
     * @param shape The collision shape
     * @param massKg The mass in kg
     * @param pose The initial pose
     * @return The request ID for tracking
     */
    public int queueBodySpawn(PhysicsShape shape, double massKg, Pose3d pose) {
        return inputBuilder.addSpawn(shape, massKg, pose);
    }

    /**
     * Queues a body removal.
     *
     * @param bodyId The body ID to remove
     */
    public void queueBodyRemoval(int bodyId) {
        inputBuilder.addRemoval(bodyId);
    }

    /**
     * Queues a pose reset (teleport).
     *
     * @param bodyId The body ID
     * @param newPose The new pose
     */
    public void queuePoseReset(int bodyId, Pose3d newPose) {
        inputBuilder.addPoseReset(bodyId, newPose);
    }

    /**
     * Queues a velocity reset.
     *
     * @param bodyId The body ID
     * @param linear New linear velocity
     * @param angular New angular velocity
     */
    public void queueVelocityReset(int bodyId, Translation3d linear, Translation3d angular) {
        inputBuilder.addVelocityReset(bodyId, linear, angular);
    }

    /**
     * Queues a raycast request.
     *
     * @param origin Ray origin
     * @param direction Ray direction
     * @param maxDistance Maximum distance
     * @return The request ID for result retrieval in next frame
     */
    public int queueRaycast(Translation3d origin, Translation3d direction, double maxDistance) {
        int requestId = nextRaycastId++;
        inputBuilder.addRaycast(requestId, origin, direction, maxDistance);
        pendingRaycasts.put(requestId, new RaycastRequest(origin, direction, maxDistance));

        return requestId;
    }

    /**
     * Flushes all queued inputs to the physics thread.
     *
     * <p>Call this at the END of each simulation period. In lock-step mode, this also signals the physics thread to
     * process the frame.
     */
    public void flushInputs() {
        frameNumber++;
        inputBuilder.frameNumber(frameNumber);

        // Always build and submit, even if empty (physics still needs to run sub-ticks)
        SimulationInputs built = inputBuilder.build();
        physicsThread.submitInputs(built);

        // In lock-step mode, signal the physics thread to wake up and process
        if (physicsThread.isLockStepMode()) {
            physicsThread.signalFrameReady();
        }

        inputBuilder.reset();
    }

    /**
     * Flushes inputs and waits for the physics frame to complete.
     *
     * <p>This is the synchronous version of {@link #flushInputs()} that blocks until the physics thread has finished
     * processing the frame. Use this when you need the latest physics state immediately.
     *
     * <p>In lock-step mode, this ensures deterministic behavior by waiting for the physics thread to complete before
     * continuing.
     *
     * @return true if the frame completed successfully, false if interrupted
     */
    public boolean flushInputsAndWait() {
        flushInputs();

        if (physicsThread.isLockStepMode()) {
            boolean completed = physicsThread.waitForFrameComplete();
            if (completed) {
                // Pull the fresh state immediately
                pullLatestState();
            }
            return completed;
        }
        return true;
    }

    /**
     * Flushes inputs and waits for frame completion with a timeout.
     *
     * @param timeoutMs Maximum time to wait in milliseconds
     * @return true if completed within timeout, false otherwise
     */
    public boolean flushInputsAndWait(long timeoutMs) {
        flushInputs();

        if (physicsThread.isLockStepMode()) {
            boolean completed = physicsThread.waitForFrameComplete(timeoutMs);
            if (completed) {
                pullLatestState();
            }
            return completed;
        }
        return true;
    }

    /**
     * Waits for the current physics frame to complete without flushing new inputs.
     *
     * <p>Use this if you've already called {@link #flushInputs()} and want to wait for completion later.
     *
     * @return true if completed, false if interrupted
     */
    public boolean waitForFrameComplete() {
        if (physicsThread.isLockStepMode()) {
            boolean completed = physicsThread.waitForFrameComplete();
            if (completed) {
                pullLatestState();
            }
            return completed;
        }
        return true;
    }

    /**
     * Checks if the physics thread is in lock-step mode.
     *
     * @return true if deterministic lock-step mode is enabled
     */
    public boolean isLockStepMode() {
        return physicsThread.isLockStepMode();
    }

    /**
     * Sets the physics thread's lock-step mode.
     *
     * @param lockStep true for deterministic lock-step, false for free-running
     */
    public void setLockStepMode(boolean lockStep) {
        physicsThread.setLockStepMode(lockStep);
    }

    /**
     * Registers a body created during initialization.
     *
     * @param body The body to register
     */
    public void registerBody(PhysicsBody body) {
        physicsThread.registerBody(body);
    }

    /**
     * Queues an existing body to be added to the physics space.
     *
     * @param body The body to add
     */
    public void queueBodyAdd(PhysicsBody body) {
        inputBuilder.addBody(body);
    }

    /**
     * Queues a global gravity update.
     *
     * @param gravity The new gravity vector
     */
    public void queueGravity(Translation3d gravity) {
        inputBuilder.setGravity(gravity);
    }

    /**
     * Queues swerve module states for the next frame.
     *
     * @param moduleStates The states of all swerve modules
     */
    public void queueSwerveInput(edu.wpi.first.math.kinematics.SwerveModuleState[] moduleStates) {
        inputBuilder.setSwerveInput(moduleStates);
    }

    /**
     * Checks if the physics thread is running.
     *
     * @return true if running
     */
    public boolean isRunning() {
        return physicsThread.isRunning();
    }

    /**
     * Gets the physics thread's tick count.
     *
     * @return Number of physics ticks completed
     */
    public long getPhysicsTickCount() {
        return physicsThread.getTickCount();
    }

    /**
     * Registers a physics calculator to run each physics tick.
     *
     * <p>Calculators run on the physics thread with current state, eliminating stale data issues.
     *
     * @param calculator The calculator to register
     */
    public void registerCalculator(PhysicsCalculator calculator) {
        inputBuilder.addCalculator(calculator);
    }

    /**
     * Queues arbitrary inputs using the builder.
     *
     * @param action Action to perform on the builder
     */
    public void queueInput(java.util.function.Consumer<SimulationInputs.Builder> action) {
        action.accept(inputBuilder);
    }

    // ==================== ThreadedJoltBody Support Methods ====================

    private final Map<Integer, Object> bodyUserData = new java.util.concurrent.ConcurrentHashMap<>();

    /** Gets the latest state from physics thread. */
    public SimulationState getLatestState() {
        return cachedState;
    }

    /** Queues a body pose update. */
    public void queueBodyPoseUpdate(int bodyId, Pose3d pose) {
        queuePoseReset(bodyId, pose);
    }

    /** Queues a body velocity update. */
    public void queueBodyVelocityUpdate(int bodyId, Translation3d linear, Translation3d angular) {
        queueVelocityReset(bodyId, linear, angular);
    }

    /** Queues a body force. */
    public void queueBodyForce(int bodyId, Translation3d force, Translation3d point) {
        if (point != null) {
            queueForceAtPoint(bodyId, force, point);
        } else {
            queueCentralForce(bodyId, force);
        }
    }

    /** Queues a body torque. */
    public void queueBodyTorque(int bodyId, Translation3d torque) {
        queueTorque(bodyId, torque);
    }

    /** Queues body damping update. */
    public void queueBodyDamping(int bodyId, double linearDamping, double angularDamping) {
        inputBuilder.addDamping(bodyId, linearDamping, angularDamping);
    }

    /** Sets user data for a body (stored locally in proxy). */
    public void setBodyUserData(int bodyId, Object data) {
        if (data == null) {
            bodyUserData.remove(bodyId);
        } else {
            bodyUserData.put(bodyId, data);
        }
    }

    /** Gets user data for a body. */
    public Object getBodyUserData(int bodyId) {
        return bodyUserData.get(bodyId);
    }

    private record RaycastRequest(Translation3d origin, Translation3d direction, double maxDistance) {}
}
