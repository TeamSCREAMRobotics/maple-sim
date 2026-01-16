package org.ironmaple.simulation.physics.threading;

/**
 * Configuration for multithreaded physics simulation.
 *
 * <p>When enabled, physics runs on a dedicated background thread at the configured tick rate, decoupled from the main
 * robot periodic loop.
 *
 * @param enabled Whether to enable threaded physics (default: false)
 * @param tickRateHz Physics simulation tick rate in Hz (default: 60)
 * @param warmStart Whether to use warm starting for physics constraints
 * @param maxCatchupTicks Maximum physics ticks to run if falling behind schedule
 * @param subTicksPerFrame Number of physics sub-steps to run per frame (default: 5). Higher = more stable suspension.
 */
public record PhysicsThreadConfig(
        boolean enabled, int tickRateHz, boolean warmStart, int maxCatchupTicks, int subTicksPerFrame) {
    /** Default configuration with threading enabled (100Hz, 2 sub-ticks). */
    public static final PhysicsThreadConfig DEFAULT = new PhysicsThreadConfig(false, 100, true, 0, 2);

    /**
     * Creates a configuration with threading enabled at the specified tick rate.
     *
     * @param tickRateHz The physics tick rate in Hz (e.g., 100 for 10ms ticks)
     * @return A new configuration with threading enabled
     */
    public static PhysicsThreadConfig enabled(int tickRateHz) {
        return new PhysicsThreadConfig(true, tickRateHz, true, 0, 2);
    }

    public static PhysicsThreadConfig enabled(int tickRateHz, int subTicksPerFrame) {
        return new PhysicsThreadConfig(true, tickRateHz, true, 0, subTicksPerFrame);
    }

    /**
     * Gets the tick period in nanoseconds.
     *
     * @return The period between physics ticks in nanoseconds
     */
    public long tickPeriodNanos() {
        return 1_000_000_000L / tickRateHz;
    }

    /**
     * Gets the tick period in seconds.
     *
     * @return The period between physics ticks in seconds
     */
    public double tickPeriodSeconds() {
        return 1.0 / tickRateHz;
    }
}
