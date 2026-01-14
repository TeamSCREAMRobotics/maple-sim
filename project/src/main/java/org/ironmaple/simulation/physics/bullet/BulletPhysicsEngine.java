package org.ironmaple.simulation.physics.bullet;

import com.jme3.bullet.PhysicsSpace;
import com.jme3.bullet.collision.shapes.*;
import com.jme3.bullet.objects.PhysicsRigidBody;
import com.jme3.math.Quaternion;
import com.jme3.math.Vector3f;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.units.measure.Time;
import electrostatic4j.snaploader.LibraryInfo;
import electrostatic4j.snaploader.LoadingCriterion;
import electrostatic4j.snaploader.NativeBinaryLoader;
import electrostatic4j.snaploader.filesystem.DirectoryPath;
import electrostatic4j.snaploader.platform.NativeDynamicLibrary;
import electrostatic4j.snaploader.platform.util.PlatformPredicate;
import java.util.Optional;
import org.ironmaple.simulation.physics.PhysicsBody;
import org.ironmaple.simulation.physics.PhysicsEngine;
import org.ironmaple.simulation.physics.PhysicsShape;

/**
 *
 *
 * <h1>Bullet Physics Engine Implementation</h1>
 *
 * <p>Implements the {@link PhysicsEngine} interface using Bullet Physics via the Libbulletjme JNI wrapper.
 *
 * <p>This provides high-performance 3D rigid body dynamics, collision detection, and raycasting.
 */
public class BulletPhysicsEngine implements PhysicsEngine {
    private PhysicsSpace physicsSpace;
    private boolean initialized = false;
    private final java.util.Map<com.jme3.bullet.collision.PhysicsCollisionObject, PhysicsBody> bodyMap =
            new java.util.HashMap<>();

    @Override
    public void initialize() {
        if (initialized) return;

        // Load the native Bullet library
        // Load the native Bullet library
        try {
            LibraryInfo info = new LibraryInfo(null, "bulletjme", DirectoryPath.USER_DIR);
            NativeBinaryLoader loader = new NativeBinaryLoader(info);

            NativeDynamicLibrary[] libraries = {
                new NativeDynamicLibrary("native/linux/arm64", PlatformPredicate.LINUX_ARM_64),
                new NativeDynamicLibrary("native/linux/arm32", PlatformPredicate.LINUX_ARM_32),
                new NativeDynamicLibrary("native/linux/x86_64", PlatformPredicate.LINUX_X86_64),
                new NativeDynamicLibrary("native/osx/arm64", PlatformPredicate.MACOS_ARM_64),
                new NativeDynamicLibrary("native/osx/x86_64", PlatformPredicate.MACOS_X86_64),
                new NativeDynamicLibrary("native/windows/x86_64", PlatformPredicate.WIN_X86_64)
            };
            loader.registerNativeLibraries(libraries).initPlatformLibrary();
            loader.loadLibrary(LoadingCriterion.CLEAN_EXTRACTION);
        } catch (Exception e) {
            throw new RuntimeException("Failed to load Bullet native library", e);
        }

        // Create the physics space with DBVT broadphase (good general-purpose)
        physicsSpace = new PhysicsSpace(PhysicsSpace.BroadphaseType.DBVT);

        // Default gravity (can be overridden)
        physicsSpace.setGravity(new Vector3f(0, -9.81f, 0));

        initialized = true;
    }

    @Override
    public void shutdown() {
        if (!initialized) return;
        physicsSpace.destroy();
        physicsSpace = null;
        initialized = false;
    }

    @Override
    public void step(Time deltaTime) {
        ensureInitialized();
        float dt = (float) deltaTime.baseUnitMagnitude();
        // Use a fixed sub-step count for stability
        int maxSubSteps = 5;
        physicsSpace.update(dt, maxSubSteps, false, true, false);
    }

    @Override
    public PhysicsBody createDynamicBody(PhysicsShape shape, double massKg, Pose3d initialPose) {
        ensureInitialized();
        BulletShape bulletShape = (BulletShape) shape;
        PhysicsRigidBody rigidBody = new PhysicsRigidBody(bulletShape.getCollisionShape(), (float) massKg);

        // Set initial pose
        rigidBody.setPhysicsLocation(toVector3f(initialPose.getTranslation()));
        rigidBody.setPhysicsRotation(toQuaternion(initialPose.getRotation()));

        physicsSpace.addCollisionObject(rigidBody);
        BulletBody body = new BulletBody(rigidBody, false);
        bodyMap.put(rigidBody, body);

        return body;
    }

    @Override
    public PhysicsBody createStaticBody(PhysicsShape shape, Pose3d pose) {
        ensureInitialized();
        BulletShape bulletShape = (BulletShape) shape;
        // Mass of 0 = static body in Bullet
        PhysicsRigidBody rigidBody = new PhysicsRigidBody(bulletShape.getCollisionShape(), 0f);

        rigidBody.setPhysicsLocation(toVector3f(pose.getTranslation()));
        rigidBody.setPhysicsRotation(toQuaternion(pose.getRotation()));

        physicsSpace.addCollisionObject(rigidBody);
        BulletBody body = new BulletBody(rigidBody, true);
        bodyMap.put(rigidBody, body);

        return body;
    }

    @Override
    public void removeBody(PhysicsBody body) {
        ensureInitialized();
        BulletBody bulletBody = (BulletBody) body;
        physicsSpace.removeCollisionObject(bulletBody.getRigidBody());
        bodyMap.remove(bulletBody.getRigidBody());
    }

    @Override
    public void removeAllBodies() {
        ensureInitialized();
        for (PhysicsRigidBody body : physicsSpace.getRigidBodyList()) {
            physicsSpace.removeCollisionObject(body);
        }
        bodyMap.clear();
    }

    @Override
    public PhysicsShape createBoxShape(Translation3d halfExtentsMeters) {
        BoxCollisionShape box = new BoxCollisionShape(
                (float) halfExtentsMeters.getX(), (float) halfExtentsMeters.getY(), (float) halfExtentsMeters.getZ());
        return new BulletShape(box, PhysicsShape.ShapeType.BOX);
    }

    @Override
    public PhysicsShape createSphereShape(double radiusMeters) {
        SphereCollisionShape sphere = new SphereCollisionShape((float) radiusMeters);
        return new BulletShape(sphere, PhysicsShape.ShapeType.SPHERE);
    }

    @Override
    public PhysicsShape createCylinderShape(double radiusMeters, double heightMeters) {
        // Bullet cylinders are aligned along Y axis by default
        CylinderCollisionShape cylinder =
                new CylinderCollisionShape((float) radiusMeters, (float) heightMeters, PhysicsSpace.AXIS_Y);
        return new BulletShape(cylinder, PhysicsShape.ShapeType.CYLINDER);
    }

    @Override
    public PhysicsShape createConvexHullShape(Translation3d[] vertices) {
        // Converts to float buffer
        float[] points = new float[vertices.length * 3];
        for (int i = 0; i < vertices.length; i++) {
            points[i * 3] = (float) vertices[i].getX();
            points[i * 3 + 1] = (float) vertices[i].getY();
            points[i * 3 + 2] = (float) vertices[i].getZ();
        }
        HullCollisionShape hull = new HullCollisionShape(points);
        return new BulletShape(hull, PhysicsShape.ShapeType.CONVEX_HULL);
    }

    @Override
    public PhysicsShape createCompoundShapeFromMesh(String resourcePath) {
        try {
            java.util.List<Translation3d[]> hulls = org.ironmaple.utils.ObjLoader.loadConvexHulls(resourcePath);

            CompoundCollisionShape compound = new CompoundCollisionShape(hulls.size());

            for (Translation3d[] hullVertices : hulls) {
                // Convert to float buffer
                float[] points = new float[hullVertices.length * 3];
                for (int i = 0; i < hullVertices.length; i++) {
                    points[i * 3] = (float) hullVertices[i].getX();
                    points[i * 3 + 1] = (float) hullVertices[i].getY();
                    points[i * 3 + 2] = (float) hullVertices[i].getZ();
                }

                HullCollisionShape hullShape = new HullCollisionShape(points);
                // Child shape, local transform (Identity)
                compound.addChildShape(hullShape, 0, 0, 0);
            }

            return new BulletShape(compound, PhysicsShape.ShapeType.COMPOUND);
        } catch (java.io.IOException e) {
            throw new RuntimeException("Failed to load generic collision mesh: " + resourcePath, e);
        }
    }

    @Override
    public Optional<RaycastResult> raycast(Translation3d origin, Translation3d direction, double maxDistance) {
        ensureInitialized();

        Vector3f from = toVector3f(origin);
        Translation3d normalizedDir = direction.div(direction.getNorm());
        Vector3f to = toVector3f(origin.plus(normalizedDir.times(maxDistance)));

        // Perform raycast
        com.jme3.bullet.collision.PhysicsRayTestResult closest = null;
        for (com.jme3.bullet.collision.PhysicsRayTestResult result : physicsSpace.rayTest(from, to)) {
            if (closest == null || result.getHitFraction() < closest.getHitFraction()) {
                closest = result;
            }
        }

        if (closest == null) {
            return Optional.empty();
        }

        Vector3f hitPoint = new Vector3f();
        float t = closest.getHitFraction();
        hitPoint.x = from.x + t * (to.x - from.x);
        hitPoint.y = from.y + t * (to.y - from.y);
        hitPoint.z = from.z + t * (to.z - from.z);

        Vector3f hitNormal = closest.getHitNormalLocal(null);

        double hitDistance = maxDistance * closest.getHitFraction();

        // Find the PhysicsBody wrapper for the hit object
        // Find the PhysicsBody wrapper for the hit object
        PhysicsRigidBody hitRigidBody = (PhysicsRigidBody) closest.getCollisionObject();
        PhysicsBody hitBody = bodyMap.get(hitRigidBody);
        if (hitBody == null) {
            // Fallback if not found (shouldn't happen for managed bodies)
            hitBody = new BulletBody(hitRigidBody, hitRigidBody.getMass() == 0);
        }

        return Optional.of(
                new RaycastResult(toTranslation3d(hitPoint), toTranslation3d(hitNormal), hitDistance, hitBody));
    }

    @Override
    public void setGravity(Translation3d gravityMPS2) {
        ensureInitialized();
        physicsSpace.setGravity(toVector3f(gravityMPS2));
    }

    @Override
    public java.util.List<PhysicsBody> getOverlappingBodies(PhysicsShape shape, Pose3d pose) {
        ensureInitialized();
        BulletShape bulletShape = (BulletShape) shape;
        com.jme3.bullet.objects.PhysicsGhostObject ghost =
                new com.jme3.bullet.objects.PhysicsGhostObject(bulletShape.getCollisionShape());

        ghost.setPhysicsLocation(toVector3f(pose.getTranslation()));
        ghost.setPhysicsRotation(toQuaternion(pose.getRotation()));

        // We need to add it to space to check?
        // Actually PhysicsSpace.contactTest checks against everything in space.
        // But the ghost object itself doesn't need to be added if we just use it as a
        // query shape?
        // Let's safe-guard by not adding it, passing it directly to contactTest.

        java.util.List<PhysicsBody> overlaps = new java.util.ArrayList<>();

        physicsSpace.contactTest(ghost, new com.jme3.bullet.collision.PhysicsCollisionListener() {
            public void collision(com.jme3.bullet.collision.PhysicsCollisionEvent event) {
                com.jme3.bullet.collision.PhysicsCollisionObject other;
                if (event.getObjectA() == ghost) {
                    other = event.getObjectB();
                } else if (event.getObjectB() == ghost) {
                    other = event.getObjectA();
                } else {
                    return;
                }

                PhysicsBody body = bodyMap.get(other);
                if (body != null) {
                    overlaps.add(body);
                }
            }
        });

        // Ensure we don't leak anything if we didn't add it.
        // If we added it, remove it. Here we didn't add it.

        return overlaps;
    }

    /**
     *
     *
     * <h2>Gets the Underlying Bullet PhysicsSpace.</h2>
     *
     * <p>For advanced use cases that need direct access to Bullet features.
     *
     * @return the physics space
     */
    public PhysicsSpace getPhysicsSpace() {
        ensureInitialized();
        return physicsSpace;
    }

    private void ensureInitialized() {
        if (!initialized) {
            throw new IllegalStateException("BulletPhysicsEngine has not been initialized. Call initialize() first.");
        }
    }

    // ==================== Geometry Conversion Utilities ====================

    /** Converts WPILib Translation3d to Bullet Vector3f. */
    public static Vector3f toVector3f(Translation3d translation) {
        return new Vector3f((float) translation.getX(), (float) translation.getY(), (float) translation.getZ());
    }

    /** Converts Bullet Vector3f to WPILib Translation3d. */
    public static Translation3d toTranslation3d(Vector3f vector) {
        return new Translation3d(vector.x, vector.y, vector.z);
    }

    /** Converts WPILib Rotation3d to Bullet Quaternion. */
    public static Quaternion toQuaternion(Rotation3d rotation) {
        edu.wpi.first.math.geometry.Quaternion wpilibQuat = rotation.getQuaternion();
        return new Quaternion((float) wpilibQuat.getX(), (float) wpilibQuat.getY(), (float) wpilibQuat.getZ(), (float)
                wpilibQuat.getW());
    }

    /** Converts Bullet Quaternion to WPILib Rotation3d. */
    public static Rotation3d toRotation3d(Quaternion quaternion) {
        return new Rotation3d(new edu.wpi.first.math.geometry.Quaternion(
                quaternion.getW(), quaternion.getX(), quaternion.getY(), quaternion.getZ()));
    }

    /** Converts WPILib Pose3d to Bullet Transform components. */
    public static void toPose(Pose3d pose, Vector3f outLocation, Quaternion outRotation) {
        outLocation.set(toVector3f(pose.getTranslation()));
        outRotation.set(toQuaternion(pose.getRotation()));
    }

    /** Converts Bullet Transform components to WPILib Pose3d. */
    public static Pose3d toPose3d(Vector3f location, Quaternion rotation) {
        return new Pose3d(toTranslation3d(location), toRotation3d(rotation));
    }
}
