package org.ironmaple.simulation.physics.threading;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.units.measure.Time;
import java.util.List;
import java.util.Optional;
import java.util.stream.Collectors;
import org.ironmaple.simulation.physics.PhysicsBody;
import org.ironmaple.simulation.physics.PhysicsEngine;
import org.ironmaple.simulation.physics.PhysicsShape;
import org.ironmaple.simulation.physics.bullet.BulletBody;

/**
 * A wrapper around a {@link PhysicsEngine} that wraps created bodies in {@link ThreadedBulletBody}.
 *
 * <p>This ensures that all bodies returned to the user are safe to use from the main thread, delegating operations to
 * the {@link ThreadedPhysicsProxy}.
 */
public class ThreadedPhysicsEngine implements PhysicsEngine {
    private final PhysicsEngine realEngine;
    private final ThreadedPhysicsProxy proxy;

    public ThreadedPhysicsEngine(PhysicsEngine realEngine, ThreadedPhysicsProxy proxy) {
        this.realEngine = realEngine;
        this.proxy = proxy;
    }

    @Override
    public void initialize() {
        // Initialization handled by backend/thread management
        realEngine.initialize();
    }

    @Override
    public void shutdown() {
        realEngine.shutdown();
    }

    @Override
    public void step(Time deltaTime) {
        // Step is handled by PhysicsThread, but if called here (e.g. sync fallback),
        // forward it.
        // In threaded mode, this shouldn't really be called by user.
        realEngine.step(deltaTime);
    }

    @Override
    public PhysicsBody createDynamicBody(PhysicsShape shape, double massKg, Pose3d initialPose) {
        // We need to create the body without adding it to the space immediately (which
        // is unsafe)
        PhysicsBody realBody;
        if (realEngine instanceof org.ironmaple.simulation.physics.bullet.BulletPhysicsEngine bulletEngine) {
            realBody = bulletEngine.createDynamicBodyNoAdd(shape, massKg, initialPose);
        } else {
            // Fallback (unsafe if engine adds immediately, but we have no choice without
            // interface change)
            realBody = realEngine.createDynamicBody(shape, massKg, initialPose);
        }

        // Queue addition of the REAL body
        proxy.queueBodyAdd(realBody);

        // Return the wrapper
        return new ThreadedBulletBody((BulletBody) realBody, proxy);
    }

    @Override
    public PhysicsBody createStaticBody(PhysicsShape shape, Pose3d pose) {
        PhysicsBody realBody;
        if (realEngine instanceof org.ironmaple.simulation.physics.bullet.BulletPhysicsEngine bulletEngine) {
            realBody = bulletEngine.createStaticBodyNoAdd(shape, pose);
        } else {
            realBody = realEngine.createStaticBody(shape, pose);
        }

        // Queue addition of the REAL body
        proxy.queueBodyAdd(realBody);

        return new ThreadedBulletBody((BulletBody) realBody, proxy);
    }

    @Override
    public void removeBody(PhysicsBody body) {
        if (body instanceof ThreadedBulletBody wrapper) {
            proxy.queueBodyRemoval(wrapper.getRealBody().getBodyId());
        } else {
            // Fallback for non-wrapped bodies (shouldn't happen if created via this engine)
            realEngine.removeBody(body);
        }
    }

    @Override
    public void removeAllBodies() {
        realEngine.removeAllBodies();
    }

    @Override
    public PhysicsShape createBoxShape(Translation3d halfExtentsMeters) {
        return realEngine.createBoxShape(halfExtentsMeters);
    }

    @Override
    public PhysicsShape createSphereShape(double radiusMeters) {
        return realEngine.createSphereShape(radiusMeters);
    }

    @Override
    public PhysicsShape createCylinderShape(double radiusMeters, double heightMeters) {
        return realEngine.createCylinderShape(radiusMeters, heightMeters);
    }

    @Override
    public PhysicsShape createConvexHullShape(Translation3d[] vertices) {
        return realEngine.createConvexHullShape(vertices);
    }

    @Override
    public PhysicsShape createCompoundShapeFromMesh(String resourcePath) {
        return realEngine.createCompoundShapeFromMesh(resourcePath);
    }

    @Override
    public Optional<RaycastResult> raycast(Translation3d origin, Translation3d direction, double maxDistance) {
        // Call real raycast (synchronized)
        Optional<RaycastResult> realResult = realEngine.raycast(origin, direction, maxDistance);

        // Unwrap/Wrap the hit body
        return realResult.map(r -> {
            PhysicsBody hitBody = r.hitBody();
            // Check if there is a wrapper linked
            if (hitBody.getUserData() instanceof ThreadedBulletBody wrapper) {
                return new RaycastResult(r.hitPoint(), r.hitNormal(), r.hitDistance(), wrapper);
            }
            // If no wrapper found (e.g. internal body?), fallback to real body
            return r;
        });
    }

    @Override
    public void setGravity(Translation3d gravityMPS2) {
        proxy.queueGravity(gravityMPS2);
    }

    @Override
    public List<PhysicsBody> getOverlappingBodies(PhysicsShape shape, Pose3d pose) {
        List<PhysicsBody> overlaps = realEngine.getOverlappingBodies(shape, pose);
        return overlaps.stream()
                .map(b -> {
                    if (b.getUserData() instanceof ThreadedBulletBody wrapper) {
                        return wrapper;
                    }
                    return b;
                })
                .collect(Collectors.toList());
    }
}
