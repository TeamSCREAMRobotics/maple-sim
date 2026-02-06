package org.ironmaple.simulation.physics.jolt;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.units.measure.Time;
import java.util.Optional;
import org.ironmaple.simulation.physics.PhysicsBackend;
import org.ironmaple.simulation.physics.PhysicsBody;
import org.ironmaple.simulation.physics.PhysicsEngine;
import org.ironmaple.simulation.physics.PhysicsShape;

/**
 *
 *
 * <h1>Jolt Physics Backend Implementation</h1>
 *
 * <p>Implements the {@link PhysicsBackend} interface using Jolt Physics.
 *
 * <p>This provides a simplified interface for obstacle management while leveraging Jolt's 3D physics.
 */
public class JoltBackend implements PhysicsBackend {
    private final JoltPhysicsEngine engine;
    // The engine exposed to external users (wrapped if threaded)
    private PhysicsEngine exposedEngine;
    private final org.ironmaple.simulation.physics.threading.PhysicsThreadConfig threadConfig;
    private org.ironmaple.simulation.physics.threading.PhysicsThread physicsThread;
    private org.ironmaple.simulation.physics.threading.ThreadedPhysicsProxy threadedProxy;
    private boolean initialized = false;

    /** Creates a Jolt backend with default configuration (threading disabled). */
    public JoltBackend() {
        this(org.ironmaple.simulation.physics.threading.PhysicsThreadConfig.DEFAULT);
    }

    /**
     * Creates a Jolt backend with the specified threading configuration.
     *
     * @param threadConfig Threading configuration
     */
    public JoltBackend(org.ironmaple.simulation.physics.threading.PhysicsThreadConfig threadConfig) {
        // Configure Jolt engine based on thread config if needed (e.g. max bodies)
        // For now use default Jolt config
        this.engine = new JoltPhysicsEngine();
        this.threadConfig = threadConfig;
    }

    @Override
    public boolean is3D() {
        return true;
    }

    @Override
    public void initialize() {
        if (initialized) return;

        // Ensure native library is loaded so we can create shapes on main thread
        JoltPhysicsEngine.loadLibrary();

        if (threadConfig.enabled()) {
            // Threaded mode: Initialize engine on physics thread to maintain thread affinity
            physicsThread = new org.ironmaple.simulation.physics.threading.PhysicsThread(engine, threadConfig);
            threadedProxy = new org.ironmaple.simulation.physics.threading.ThreadedPhysicsProxy(physicsThread);
            // Wrap the engine
            exposedEngine = new org.ironmaple.simulation.physics.threading.ThreadedPhysicsEngine(engine, threadedProxy);
            physicsThread.start();

            // Wait for physics thread to complete initialization before returning
            try {
                physicsThread.waitForInitialization();
            } catch (InterruptedException e) {
                Thread.currentThread().interrupt();
                throw new RuntimeException("Interrupted while waiting for physics initialization", e);
            }
        } else {
            // Sync mode: Initialize engine on main thread
            engine.initialize();
            exposedEngine = engine;
        }

        initialized = true;
    }

    @Override
    public void shutdown() {
        if (!initialized) return;

        if (physicsThread != null) {
            physicsThread.shutdown();
            try {
                physicsThread.join(1000); // Wait up to 1 second
            } catch (InterruptedException e) {
                Thread.currentThread().interrupt();
            }
            physicsThread = null;
            threadedProxy = null;
        }

        engine.shutdown();
        initialized = false;
    }

    @Override
    public void step(Time deltaTime) {
        ensureInitialized();
        if (!isThreaded()) {
            // Synchronous mode: step the engine directly
            engine.step(deltaTime);
        }
        // In threaded mode, physics runs continuously on its own thread
    }

    @Override
    public Object addStaticBox(Translation3d halfExtents, Pose3d pose) {
        ensureInitialized();
        PhysicsShape shape = exposedEngine.createBoxShape(halfExtents);
        return exposedEngine.createStaticBody(shape, pose);
    }

    @Override
    public Object addStaticLine(Translation2d start, Translation2d end) {
        ensureInitialized();
        // Create a thin box to approximate a 2D line in 3D
        double dx = end.getX() - start.getX();
        double dy = end.getY() - start.getY();
        double length = Math.sqrt(dx * dx + dy * dy);
        double angle = Math.atan2(dy, dx);

        // Create box shape (thin wall)
        double thickness = 0.02; // 2 cm thick
        double height = 0.5; // 50 cm tall
        Translation3d halfExtents = new Translation3d(length / 2, thickness / 2, height / 2);
        PhysicsShape shape = exposedEngine.createBoxShape(halfExtents);

        // Position at midpoint
        double midX = (start.getX() + end.getX()) / 2;
        double midY = (start.getY() + end.getY()) / 2;
        // Jolt rotation is Z-up so this is standard Z rotation
        Pose3d pose = new Pose3d(new Translation3d(midX, midY, height / 2), new Rotation3d(0, 0, angle));

        return exposedEngine.createStaticBody(shape, pose);
    }

    @Override
    public void removeBody(Object bodyHandle) {
        ensureInitialized();
        if (bodyHandle instanceof PhysicsBody body) {
            exposedEngine.removeBody(body);
        }
    }

    @Override
    public void removeAllBodies() {
        ensureInitialized();
        exposedEngine.removeAllBodies();
    }

    @Override
    public Optional<PhysicsEngine.RaycastResult> raycast(
            Translation3d origin, Translation3d direction, double maxDistance) {
        ensureInitialized();
        return exposedEngine.raycast(origin, direction, maxDistance);
    }

    @Override
    public void setGravity(Translation3d gravity) {
        ensureInitialized();
        exposedEngine.setGravity(gravity);
    }

    /**
     * Gets the underlying Jolt physics engine.
     *
     * @return the Jolt physics engine (wrapped if threaded, or raw if sync)
     */
    public PhysicsEngine getEngine() {
        ensureInitialized();
        return exposedEngine;
    }

    @Override
    public boolean isThreaded() {
        return threadConfig.enabled() && physicsThread != null && physicsThread.isRunning();
    }

    @Override
    public org.ironmaple.simulation.physics.threading.ThreadedPhysicsProxy getThreadedProxy() {
        return threadedProxy;
    }

    @Override
    public void flushInputs() {
        if (isThreaded() && threadedProxy != null) {
            threadedProxy.flushInputs();
        }
    }

    @Override
    public void pullLatestState() {
        if (isThreaded() && threadedProxy != null) {
            threadedProxy.pullLatestState();
        }
    }

    /**
     * Gets the threading configuration.
     *
     * @return the thread config
     */
    public org.ironmaple.simulation.physics.threading.PhysicsThreadConfig getThreadConfig() {
        return threadConfig;
    }

    private void ensureInitialized() {
        if (!initialized) {
            throw new IllegalStateException("JoltBackend has not been initialized. Call initialize() first.");
        }
    }
}
