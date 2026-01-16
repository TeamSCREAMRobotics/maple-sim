package org.ironmaple.simulation.physics.threading;

import edu.wpi.first.math.kinematics.SwerveModuleState;

/**
 * Interface for physics calculators that need to receive swerve module states from the main thread.
 *
 * <p>Implementing this interface allows the {@link PhysicsThread} to automatically update the calculator with the
 * captured state from {@link SimulationInputs}, ensuring thread-safe synchronization.
 */
public interface ThreadedSwerveCalculator extends PhysicsCalculator {
    /**
     * Updates the captured states for the next tick.
     *
     * @param states The swerve module states from the beginning of the frame.
     */
    void setCapturedStates(SwerveModuleState[] states);
}
