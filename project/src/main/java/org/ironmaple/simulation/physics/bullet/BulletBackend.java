package org.ironmaple.simulation.physics.bullet;

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
import org.ironmaple.simulation.physics.threading.PhysicsThread;
import org.ironmaple.simulation.physics.threading.PhysicsThreadConfig;
import org.ironmaple.simulation.physics.threading.ThreadedPhysicsProxy;

/**
 *
 *
 * <h1>Bullet Physics Backend (3D)</h1>
 *
 * <p>Implements the {@link PhysicsBackend} interface using Bullet Physics via Libbulletjme. This provides high-fidelity
 * 3D physics simulation.
 *
 * <h2>Threading Support</h2>
 *
 * <p>When constructed with a {@link PhysicsThreadConfig} that has threading enabled, physics runs on a dedicated
 * background thread at the configured tick rate (default 120Hz).
 */
public class BulletBackend implements PhysicsBackend {
    private final BulletPhysicsEngine engine;
    // The engine exposed to external users (wrapped if threaded)
    private PhysicsEngine exposedEngine;
    private final PhysicsThreadConfig threadConfig;
    private PhysicsThread physicsThread;
    private ThreadedPhysicsProxy threadedProxy;
    private boolean initialized = false;

    /** Creates a backend with default configuration (threading disabled). */
    public BulletBackend() {
        this(PhysicsThreadConfig.DEFAULT);
    }

    /**
     * Creates a backend with the specified threading configuration.
     *
     * @param threadConfig Threading configuration
     */
    public BulletBackend(PhysicsThreadConfig threadConfig) {
        this.engine = new BulletPhysicsEngine();
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
        BulletPhysicsEngine.loadLibrary();

        if (threadConfig.enabled()) {
            physicsThread = new PhysicsThread(engine, threadConfig);
            threadedProxy = new ThreadedPhysicsProxy(physicsThread);
            // Wrap the engine
            exposedEngine = new org.ironmaple.simulation.physics.threading.ThreadedPhysicsEngine(engine, threadedProxy);
            physicsThread.start();
            // System.out.println("[MapleSim3D] Physics thread started at " +
            // threadConfig.tickRateHz() + " Hz");
        } else {
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
            // System.out.println("[MapleSim3D] Physics thread stopped");
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
        // Create a thin box approximation of the line segment
        double length = start.getDistance(end);
        double thickness = 0.05; // 5cm thick wall
        double height = 1.0; // 1m tall wall

        Translation3d halfExtents = new Translation3d(length / 2, thickness / 2, height / 2);

        // Calculate the center and rotation of the segment
        Translation2d center = start.plus(end).div(2);
        double angle = Math.atan2(end.getY() - start.getY(), end.getX() - start.getX());

        Pose3d pose =
                new Pose3d(new Translation3d(center.getX(), center.getY(), height / 2), new Rotation3d(0, 0, angle));

        PhysicsShape shape = exposedEngine.createBoxShape(halfExtents);
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
     *
     *
     * <h2>Gets the Underlying Physics Engine.</h2>
     *
     * @return the physics engine (wrapped if threaded)
     */
    public PhysicsEngine getEngine() {
        ensureInitialized();
        return exposedEngine;
    }

    /**
     * Checks if this backend is running in threaded mode.
     *
     * @return true if physics runs on a dedicated thread
     */
    public boolean isThreaded() {
        return threadConfig.enabled() && physicsThread != null && physicsThread.isRunning();
    }

    /**
     * Gets the threaded physics proxy for async operations.
     *
     * @return the proxy, or null if not in threaded mode
     */
    public ThreadedPhysicsProxy getThreadedProxy() {
        return threadedProxy;
    }

    /**
     * Flushes queued inputs to the physics thread.
     *
     * <p>Only has effect in threaded mode. Call at end of simulation period.
     */
    public void flushInputs() {
        if (isThreaded() && threadedProxy != null) {
            threadedProxy.flushInputs();
        }
    }

    /**
     * Pulls the latest state from the physics thread.
     *
     * <p>Only has effect in threaded mode. Call at start of simulation period.
     */
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
    public PhysicsThreadConfig getThreadConfig() {
        return threadConfig;
    }

    private void ensureInitialized() {
        if (!initialized) {
            throw new IllegalStateException("BulletBackend has not been initialized. Call initialize() first.");
        }
    }
}
