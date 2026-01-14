package org.ironmaple.simulation.physics.threading;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Translation3d;
import java.util.Collections;
import java.util.HashMap;
import java.util.Map;
import org.ironmaple.simulation.physics.PhysicsEngine;

/**
 * Immutable snapshot of the physics simulation state.
 *
 * <p>Published by the physics thread after each tick. Main thread reads this to get body poses and velocities without
 * locking.
 *
 * @param tickNumber Monotonically increasing tick counter
 * @param simulationTimeSeconds Total simulated time in seconds
 * @param bodyStates Map of body ID to state snapshot
 * @param cachedRaycasts Map of raycast request ID to result
 */
public record SimulationState(
        long tickNumber,
        double simulationTimeSeconds,
        double lastTickDurationSeconds,
        Map<Integer, BodyState> bodyStates,
        Map<Integer, PhysicsEngine.RaycastResult> cachedRaycasts) {
    /** Empty state for initialization. */
    public static final SimulationState EMPTY =
            new SimulationState(0, 0.0, 0.0, Collections.emptyMap(), Collections.emptyMap());

    /**
     * Gets the state of a specific body.
     *
     * @param bodyId The body's unique ID
     * @return The body state, or null if not found
     */
    public BodyState getBodyState(int bodyId) {
        return bodyStates.get(bodyId);
    }

    /**
     * Gets a cached raycast result.
     *
     * @param requestId The raycast request ID
     * @return The raycast result, or null if not found
     */
    public PhysicsEngine.RaycastResult getCachedRaycast(int requestId) {
        return cachedRaycasts.get(requestId);
    }

    /**
     * Immutable snapshot of a single body's state.
     *
     * @param bodyId The body's unique ID
     * @param pose The body's 3D pose
     * @param linearVelocity Linear velocity in m/s
     * @param angularVelocity Angular velocity in rad/s
     */
    public record BodyState(int bodyId, Pose3d pose, Translation3d linearVelocity, Translation3d angularVelocity) {}

    /** Builder for creating SimulationState instances. */
    public static class Builder {
        private long tickNumber;
        private double simulationTimeSeconds;
        private double lastTickDurationSeconds;
        private final Map<Integer, BodyState> bodyStates = new HashMap<>();
        private final Map<Integer, PhysicsEngine.RaycastResult> cachedRaycasts = new HashMap<>();

        public Builder tickNumber(long tickNumber) {
            this.tickNumber = tickNumber;
            return this;
        }

        public Builder simulationTime(double seconds) {
            this.simulationTimeSeconds = seconds;
            return this;
        }

        public Builder lastTickDuration(double seconds) {
            this.lastTickDurationSeconds = seconds;
            return this;
        }

        public Builder addBodyState(BodyState state) {
            bodyStates.put(state.bodyId(), state);
            return this;
        }

        public Builder addRaycastResult(int requestId, PhysicsEngine.RaycastResult result) {
            cachedRaycasts.put(requestId, result);
            return this;
        }

        public SimulationState build() {
            return new SimulationState(
                    tickNumber,
                    simulationTimeSeconds,
                    lastTickDurationSeconds,
                    Collections.unmodifiableMap(new HashMap<>(bodyStates)),
                    Collections.unmodifiableMap(new HashMap<>(cachedRaycasts)));
        }
    }

    public static Builder builder() {
        return new Builder();
    }
}
