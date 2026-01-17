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
        PhysicsBody wrapperBody;

        if (realEngine instanceof org.ironmaple.simulation.physics.bullet.BulletPhysicsEngine bulletEngine) {
            realBody = bulletEngine.createDynamicBodyNoAdd(shape, massKg, initialPose);
            wrapperBody = new ThreadedBulletBody((BulletBody) realBody, proxy);
        } else if (realEngine instanceof org.ironmaple.simulation.physics.jolt.JoltPhysicsEngine joltEngine) {
            realBody = joltEngine.createDynamicBodyNoAdd(shape, massKg, initialPose);
            wrapperBody = new org.ironmaple.simulation.physics.threading.ThreadedJoltBody(
                    (org.ironmaple.simulation.physics.jolt.JoltBody) realBody, proxy);
        } else {
            // Fallback (unsafe if engine adds immediately)
            realBody = realEngine.createDynamicBody(shape, massKg, initialPose);
            // We don't have a generic wrapper, so we can't wrap it properly if unknown type
            // But we can try to wrap it if it implements PhysicsBody, though threading
            // support might fail
            return realBody;
        }

        // Link wrapper to real body for raycast identification
        realBody.setUserData(wrapperBody);

        // Queue addition of the REAL body
        proxy.queueBodyAdd(realBody);

        return wrapperBody;
    }

    @Override
    public PhysicsBody createDynamicBody(
            PhysicsShape shape,
            double massKg,
            double friction,
            double restitution,
            double linearDamping,
            double angularDamping,
            Pose3d initialPose) {
        // We need to create the body without adding it to the space immediately (which
        // is unsafe)
        PhysicsBody realBody;
        PhysicsBody wrapperBody;

        if (realEngine instanceof org.ironmaple.simulation.physics.bullet.BulletPhysicsEngine bulletEngine) {
            realBody = bulletEngine.createDynamicBodyNoAdd(
                    shape, massKg, friction, restitution, linearDamping, angularDamping, initialPose);
            wrapperBody = new ThreadedBulletBody((BulletBody) realBody, proxy);
        } else if (realEngine instanceof org.ironmaple.simulation.physics.jolt.JoltPhysicsEngine joltEngine) {
            realBody = joltEngine.createDynamicBodyNoAdd(
                    shape, massKg, friction, restitution, linearDamping, angularDamping, initialPose);
            wrapperBody = new org.ironmaple.simulation.physics.threading.ThreadedJoltBody(
                    (org.ironmaple.simulation.physics.jolt.JoltBody) realBody, proxy);
        } else {
            // Fallback (unsafe if engine adds immediately)
            realBody = realEngine.createDynamicBody(
                    shape, massKg, friction, restitution, linearDamping, angularDamping, initialPose);
            // We don't have a generic wrapper, so we can't wrap it properly if unknown type
            // But we can try to wrap it if it implements PhysicsBody, though threading
            // support might fail
            return realBody;
        }

        // Link wrapper to real body for raycast identification
        realBody.setUserData(wrapperBody);

        // Queue addition of the REAL body
        proxy.queueBodyAdd(realBody);

        return wrapperBody;
    }

    @Override
    public PhysicsBody createStaticBody(PhysicsShape shape, Pose3d pose) {
        PhysicsBody realBody;
        PhysicsBody wrapperBody;

        if (realEngine instanceof org.ironmaple.simulation.physics.bullet.BulletPhysicsEngine bulletEngine) {
            realBody = bulletEngine.createStaticBodyNoAdd(shape, pose);
            wrapperBody = new ThreadedBulletBody((BulletBody) realBody, proxy);
        } else if (realEngine instanceof org.ironmaple.simulation.physics.jolt.JoltPhysicsEngine joltEngine) {
            realBody = joltEngine.createStaticBodyNoAdd(shape, pose);
            wrapperBody = new org.ironmaple.simulation.physics.threading.ThreadedJoltBody(
                    (org.ironmaple.simulation.physics.jolt.JoltBody) realBody, proxy);
        } else {
            realBody = realEngine.createStaticBody(shape, pose);
            return realBody;
        }

        // Link wrapper to real body for raycast identification
        realBody.setUserData(wrapperBody);

        // Queue addition of the REAL body
        proxy.queueBodyAdd(realBody);

        return wrapperBody;
    }

    @Override
    public PhysicsShape createOffsetShape(PhysicsShape shape, Translation3d offset) {
        return realEngine.createOffsetShape(shape, offset);
    }

    @Override
    public void removeBody(PhysicsBody body) {
        if (body instanceof ThreadedBulletBody wrapper) {
            proxy.queueBodyRemoval(wrapper.getRealBody().getBodyId());
        } else if (body instanceof org.ironmaple.simulation.physics.threading.ThreadedJoltBody wrapper) {
            proxy.queueBodyRemoval(wrapper.getRealBody().getTrackingId());
        } else {
            // Fallback for non-wrapped bodies
            realEngine.removeBody(body);
        }
    }

    @Override
    public void removeAllBodies() {
        realEngine.removeAllBodies();
    }

    @Override
    public void addBody(PhysicsBody body) {
        if (body != null) {
            PhysicsBody realBody = body;
            if (body instanceof ThreadedBulletBody wrapper) {
                realBody = wrapper.getRealBody();
            } else if (body instanceof org.ironmaple.simulation.physics.threading.ThreadedJoltBody wrapper) {
                realBody = wrapper.getRealBody();
            }
            proxy.queueBodyAdd(realBody);
        }
    }

    @Override
    public List<PhysicsBody> getBodies() {
        return realEngine.getBodies().stream()
                .map(b -> {
                    if (b.getUserData() instanceof PhysicsBody wrapper) {
                        return wrapper;
                    }
                    return b;
                })
                .collect(Collectors.toList());
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
            // Check if there is a wrapper linked (userData)
            if (hitBody != null && hitBody.getUserData() instanceof PhysicsBody wrapper) {
                return new RaycastResult(r.hitPoint(), r.hitNormal(), r.hitDistance(), wrapper);
            }
            // If no wrapper found, return original result
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
                    if (b.getUserData() instanceof PhysicsBody wrapper) {
                        return wrapper;
                    }
                    return b;
                })
                .collect(Collectors.toList());
    }
}
