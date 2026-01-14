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

/**
 *
 *
 * <h1>Bullet Physics Backend (3D)</h1>
 *
 * <p>Implements the {@link PhysicsBackend} interface using Bullet Physics via Libbulletjme. This provides high-fidelity
 * 3D physics simulation.
 */
public class BulletBackend implements PhysicsBackend {
    private final BulletPhysicsEngine engine;
    private boolean initialized = false;

    public BulletBackend() {
        this.engine = new BulletPhysicsEngine();
    }

    @Override
    public boolean is3D() {
        return true;
    }

    @Override
    public void initialize() {
        if (initialized) return;
        engine.initialize();
        initialized = true;
    }

    @Override
    public void shutdown() {
        if (!initialized) return;
        engine.shutdown();
        initialized = false;
    }

    @Override
    public void step(Time deltaTime) {
        ensureInitialized();
        engine.step(deltaTime);
    }

    @Override
    public Object addStaticBox(Translation3d halfExtents, Pose3d pose) {
        ensureInitialized();
        PhysicsShape shape = engine.createBoxShape(halfExtents);
        return engine.createStaticBody(shape, pose);
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

        PhysicsShape shape = engine.createBoxShape(halfExtents);
        return engine.createStaticBody(shape, pose);
    }

    @Override
    public void removeBody(Object bodyHandle) {
        ensureInitialized();
        if (bodyHandle instanceof PhysicsBody body) {
            engine.removeBody(body);
        }
    }

    @Override
    public void removeAllBodies() {
        ensureInitialized();
        engine.removeAllBodies();
    }

    @Override
    public Optional<PhysicsEngine.RaycastResult> raycast(
            Translation3d origin, Translation3d direction, double maxDistance) {
        ensureInitialized();
        return engine.raycast(origin, direction, maxDistance);
    }

    @Override
    public void setGravity(Translation3d gravity) {
        ensureInitialized();
        engine.setGravity(gravity);
    }

    /**
     *
     *
     * <h2>Gets the Underlying Bullet Physics Engine.</h2>
     *
     * @return the bullet physics engine
     */
    public BulletPhysicsEngine getEngine() {
        ensureInitialized();
        return engine;
    }

    private void ensureInitialized() {
        if (!initialized) {
            throw new IllegalStateException("BulletBackend has not been initialized. Call initialize() first.");
        }
    }
}
