package org.ironmaple.simulation.physics.threading;

import org.ironmaple.simulation.physics.PhysicsEngine;

/**
 * Interface for physics calculations that run on the physics thread.
 *
 * <p>Implementations of this interface are registered with {@link PhysicsThread} and have their
 * {@link #applyForces(PhysicsEngine)} method called each tick BEFORE the physics step.
 *
 * <p>This ensures that force calculations use the CURRENT physics state, not stale cached state, eliminating feedback
 * delay that causes oscillation.
 */
public interface PhysicsCalculator {

    /**
     * Apply forces to physics bodies.
     *
     * <p>Called on the physics thread each tick, immediately before {@code engine.step()}. Implementations should:
     *
     * <ol>
     *   <li>Query current state directly from physics bodies
     *   <li>Calculate forces based on that current state
     *   <li>Apply forces to bodies
     * </ol>
     *
     * @param engine The physics engine (for raycasting, etc.)
     */
    void applyForces(PhysicsEngine engine);
}
