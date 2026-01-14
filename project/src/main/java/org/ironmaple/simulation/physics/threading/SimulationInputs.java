package org.ironmaple.simulation.physics.threading;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Translation3d;
import java.util.ArrayList;
import java.util.Collections;
import java.util.List;
import java.util.Optional;
import org.ironmaple.simulation.physics.PhysicsBody;
import org.ironmaple.simulation.physics.PhysicsShape;

/**
 * Commands to be applied by the physics thread on the next tick.
 *
 * <p>Main thread builds this and submits atomically to physics thread. All commands are applied at the start of the
 * next physics tick.
 *
 * @param frameNumber Main thread frame number for debugging
 * @param forces Force commands to apply
 * @param torques Torque commands to apply
 * @param spawns New bodies to create
 * @param removals Body IDs to remove
 * @param poseResets Bodies to teleport
 * @param velocityResets Bodies to reset velocity
 * @param raycastRequests Raycasts to perform and cache
 */
public record SimulationInputs(
        long frameNumber,
        List<ForceCommand> forces,
        List<TorqueCommand> torques,
        List<BodySpawnCommand> spawns,
        List<Integer> removals,
        List<PoseResetCommand> poseResets,
        List<VelocityResetCommand> velocityResets,
        List<RaycastRequest> raycastRequests,
        List<PhysicsBody> bodiesToAdd,
        Optional<Translation3d> newGravity) {
    /** Empty inputs for initialization. */
    public static final SimulationInputs EMPTY = new SimulationInputs(
            0,
            Collections.emptyList(),
            Collections.emptyList(),
            Collections.emptyList(),
            Collections.emptyList(),
            Collections.emptyList(),
            Collections.emptyList(),
            Collections.emptyList(),
            Collections.emptyList(),
            Optional.empty());

    /**
     * Force to apply at a point on a body.
     *
     * @param bodyId Target body ID
     * @param force Force vector in Newtons (world space)
     * @param point Application point (world space), null for center of mass
     */
    public record ForceCommand(int bodyId, Translation3d force, Translation3d point) {}

    /**
     * Torque to apply to a body's center of mass.
     *
     * @param bodyId Target body ID
     * @param torque Torque vector in Newton-meters
     */
    public record TorqueCommand(int bodyId, Translation3d torque) {}

    /**
     * Request to spawn a new dynamic body.
     *
     * @param shape The collision shape
     * @param massKg Mass in kilograms
     * @param initialPose Initial pose
     * @param requestId Assigned ID for tracking (set by builder)
     */
    public record BodySpawnCommand(PhysicsShape shape, double massKg, Pose3d initialPose, int requestId) {}

    /**
     * Request to teleport a body to a new pose.
     *
     * @param bodyId Target body ID
     * @param newPose New pose to set
     */
    public record PoseResetCommand(int bodyId, Pose3d newPose) {}

    /**
     * Request to reset a body's velocities.
     *
     * @param bodyId Target body ID
     * @param linearVelocity New linear velocity
     * @param angularVelocity New angular velocity
     */
    public record VelocityResetCommand(int bodyId, Translation3d linearVelocity, Translation3d angularVelocity) {}

    /**
     * Raycast request for suspension or other queries.
     *
     * @param requestId Unique ID for result retrieval
     * @param origin Ray origin
     * @param direction Ray direction (will be normalized)
     * @param maxDistance Maximum ray distance
     */
    public record RaycastRequest(int requestId, Translation3d origin, Translation3d direction, double maxDistance) {}

    /** Builder for constructing SimulationInputs. */
    public static class Builder {
        private long frameNumber;
        private final List<ForceCommand> forces = new ArrayList<>();
        private final List<TorqueCommand> torques = new ArrayList<>();
        private final List<BodySpawnCommand> spawns = new ArrayList<>();
        private final List<Integer> removals = new ArrayList<>();
        private final List<PoseResetCommand> poseResets = new ArrayList<>();
        private final List<VelocityResetCommand> velocityResets = new ArrayList<>();
        private final List<RaycastRequest> raycastRequests = new ArrayList<>();
        private final List<PhysicsBody> bodiesToAdd = new ArrayList<>();
        private Optional<Translation3d> newGravity = Optional.empty();
        private int nextSpawnId = 0;

        public Builder frameNumber(long frameNumber) {
            this.frameNumber = frameNumber;
            return this;
        }

        public Builder addForce(int bodyId, Translation3d force, Translation3d point) {
            forces.add(new ForceCommand(bodyId, force, point));
            return this;
        }

        public Builder addCentralForce(int bodyId, Translation3d force) {
            forces.add(new ForceCommand(bodyId, force, null));
            return this;
        }

        public Builder addTorque(int bodyId, Translation3d torque) {
            torques.add(new TorqueCommand(bodyId, torque));
            return this;
        }

        /**
         * Adds a body spawn request.
         *
         * @return The request ID that will be assigned to the new body
         */
        public int addSpawn(PhysicsShape shape, double massKg, Pose3d pose) {
            int requestId = nextSpawnId++;
            spawns.add(new BodySpawnCommand(shape, massKg, pose, requestId));
            return requestId;
        }

        public Builder addRemoval(int bodyId) {
            removals.add(bodyId);
            return this;
        }

        public Builder addPoseReset(int bodyId, Pose3d newPose) {
            poseResets.add(new PoseResetCommand(bodyId, newPose));
            return this;
        }

        public Builder addVelocityReset(int bodyId, Translation3d linear, Translation3d angular) {
            velocityResets.add(new VelocityResetCommand(bodyId, linear, angular));
            return this;
        }

        public Builder addRaycast(int requestId, Translation3d origin, Translation3d direction, double maxDist) {
            raycastRequests.add(new RaycastRequest(requestId, origin, direction, maxDist));
            return this;
        }

        public Builder addBody(PhysicsBody body) {
            bodiesToAdd.add(body);
            return this;
        }

        public Builder setGravity(Translation3d gravity) {
            this.newGravity = Optional.of(gravity);
            return this;
        }

        public SimulationInputs build() {
            return new SimulationInputs(
                    frameNumber,
                    List.copyOf(forces),
                    List.copyOf(torques),
                    List.copyOf(spawns),
                    List.copyOf(removals),
                    List.copyOf(poseResets),
                    List.copyOf(velocityResets),
                    List.copyOf(raycastRequests),
                    List.copyOf(bodiesToAdd),
                    newGravity);
        }

        public void reset() {
            forces.clear();
            torques.clear();
            spawns.clear();
            removals.clear();
            poseResets.clear();
            velocityResets.clear();
            raycastRequests.clear();
            bodiesToAdd.clear();
            newGravity = Optional.empty();
            nextSpawnId = 0;
        }

        public boolean isEmpty() {
            return forces.isEmpty()
                    && torques.isEmpty()
                    && spawns.isEmpty()
                    && removals.isEmpty()
                    && poseResets.isEmpty()
                    && velocityResets.isEmpty()
                    && raycastRequests.isEmpty()
                    && bodiesToAdd.isEmpty()
                    && newGravity.isEmpty();
        }
    }

    public static Builder builder() {
        return new Builder();
    }
}
