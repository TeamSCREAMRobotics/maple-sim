package org.ironmaple.simulation.physics.threading;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import java.util.Optional;
import java.util.concurrent.atomic.AtomicReference;
import org.ironmaple.simulation.drivesims.SwerveModuleSimulation;
import org.ironmaple.simulation.physics.PhysicsBody;
import org.ironmaple.simulation.physics.PhysicsEngine;

/**
 * Physics-thread-side calculator for swerve drive suspension and traction forces.
 *
 * <p>This class runs ON the physics thread each tick, calculating and applying forces using the CURRENT physics state
 * (not stale cached state). This eliminates the feedback delay that causes oscillation when forces are calculated on
 * the WPILib thread.
 *
 * <h2>Thread Safety</h2>
 *
 * <ul>
 *   <li>{@link #applyForces} runs only on the physics thread
 *   <li>Module states are updated atomically from the WPILib thread
 * </ul>
 */
public class SwervePhysicsCalculator implements PhysicsCalculator {

    // ==================== Suspension Parameters ====================
    private static final double SUSPENSION_STIFFNESS = 10000.0;
    private static final double TARGET_DAMPING_RATIO = 0.7;
    private static final double COMPRESSION_DAMPING_MULT = 1.0;
    private static final double EXPANSION_DAMPING_MULT = 0.5;
    private static final double SUSPENSION_REST_LENGTH = 0.05;
    private static final double SUSPENSION_MAX_TRAVEL = 0.1;
    private static final double WHEEL_RADIUS = 0.0508;
    private static final double CHASSIS_HEIGHT = 0.1;
    private static final double MAX_SUSPENSION_FORCE = 1000.0;
    private static final double RAYCAST_ORIGIN_OFFSET = 0.5;

    private final PhysicsBody chassisBody;
    private final Translation2d[] moduleOffsets;
    private final SwerveModuleSimulation[] modules;
    private final double robotMassKg;

    // Thread-safe storage for module states (updated by WPILib thread)
    private final AtomicReference<ModuleSetpoint[]> currentSetpoints = new AtomicReference<>(null);

    /** Module setpoint record for thread-safe communication. */
    public record ModuleSetpoint(Rotation2d angle, double speedMps) {}

    /**
     * Creates a swerve physics calculator.
     *
     * @param chassisBody The chassis physics body
     * @param moduleOffsets Translation2d positions of each module relative to chassis center
     * @param modules The swerve module simulations
     * @param robotMassKg Robot mass in kilograms
     */
    public SwervePhysicsCalculator(
            PhysicsBody chassisBody,
            Translation2d[] moduleOffsets,
            SwerveModuleSimulation[] modules,
            double robotMassKg) {
        // Unwrap ThreadedBulletBody to get the real body for direct physics thread
        // access
        if (chassisBody instanceof ThreadedBulletBody tbb) {
            this.chassisBody = tbb.getRealBody();
        } else {
            this.chassisBody = chassisBody;
        }
        this.moduleOffsets = moduleOffsets.clone();
        this.modules = modules;
        this.robotMassKg = robotMassKg;
    }

    /**
     * Updates module setpoints from the WPILib thread.
     *
     * <p>This is the ONLY method that should be called from outside the physics thread.
     *
     * @param setpoints Current module setpoints (angles and speeds)
     */
    public void updateSetpoints(ModuleSetpoint[] setpoints) {
        currentSetpoints.set(setpoints.clone());
    }

    @Override
    public void applyForces(PhysicsEngine engine) {
        if (chassisBody == null) return;

        // Get CURRENT state directly from physics body (not cached!)
        Pose3d pose3d = chassisBody.getPose3d();
        Rotation3d rotation = pose3d.getRotation();
        Rotation2d robotHeading = rotation.toRotation2d();

        // Calculate damping parameters
        double massPerWheel = robotMassKg / moduleOffsets.length;
        double criticalDamping = 2.0 * Math.sqrt(SUSPENSION_STIFFNESS * massPerWheel);
        double baseDamping = TARGET_DAMPING_RATIO * criticalDamping;

        for (int i = 0; i < moduleOffsets.length; i++) {
            Translation2d moduleOffset = moduleOffsets[i];
            SwerveModuleSimulation module = modules[i];

            // --- 1. Calculate World Mount Point ---
            Translation3d localMountPoint =
                    new Translation3d(moduleOffset.getX(), moduleOffset.getY(), -CHASSIS_HEIGHT / 2);
            Translation3d worldMountPoint =
                    rotatePoint(localMountPoint, rotation).plus(pose3d.getTranslation());

            // --- 2. Raycast for Ground Detection ---
            Translation3d rayDirection = new Translation3d(0, 0, -1);
            Translation3d rayOrigin = worldMountPoint.plus(new Translation3d(0, 0, RAYCAST_ORIGIN_OFFSET));
            double maxRayDistance =
                    SUSPENSION_REST_LENGTH + SUSPENSION_MAX_TRAVEL + WHEEL_RADIUS + RAYCAST_ORIGIN_OFFSET;

            // Exclude chassisBody from raycast to prevent self-collision
            Optional<PhysicsEngine.RaycastResult> hitOpt =
                    engine.raycast(rayOrigin, rayDirection, maxRayDistance, chassisBody);
            PhysicsEngine.RaycastResult hitResult = hitOpt.orElse(null);

            double normalForceNewtons = 0.0;
            Translation3d suspensionForce = Translation3d.kZero;

            if (hitResult != null) {
                double groundDistance = hitResult.hitDistance() - RAYCAST_ORIGIN_OFFSET;
                double suspensionCompression = SUSPENSION_REST_LENGTH + WHEEL_RADIUS - groundDistance;

                if (suspensionCompression > 0) {
                    // Get ACTUAL velocity at mount point (not approximation!)
                    Translation3d mountVelocity = chassisBody.getLinearVelocityAtPointMPS(worldMountPoint);
                    double compressionVelocity = -mountVelocity.getZ();

                    double dampingCoeff = compressionVelocity > 0
                            ? baseDamping * COMPRESSION_DAMPING_MULT
                            : baseDamping * EXPANSION_DAMPING_MULT;

                    double springForce = SUSPENSION_STIFFNESS * suspensionCompression;
                    double damperForce = dampingCoeff * compressionVelocity;

                    normalForceNewtons = Math.max(0, springForce + damperForce);
                    normalForceNewtons = Math.min(normalForceNewtons, MAX_SUSPENSION_FORCE);

                    Translation3d normal = hitResult.hitNormal();
                    suspensionForce = normal.times(normalForceNewtons);
                }
            }

            // --- 3. Traction/Propulsion Calculation ---
            // Get ACTUAL velocity at wheel contact point
            Translation3d pointVelocity3d = chassisBody.getLinearVelocityAtPointMPS(worldMountPoint);
            org.dyn4j.geometry.Vector2 groundVelocity2d =
                    new org.dyn4j.geometry.Vector2(pointVelocity3d.getX(), pointVelocity3d.getY());

            org.dyn4j.geometry.Vector2 tractionForce2d =
                    module.updateSimulationSubTickGetModuleForce(groundVelocity2d, robotHeading, normalForceNewtons);

            Translation3d tractionForce3d = new Translation3d(tractionForce2d.x, tractionForce2d.y, 0);

            // DEBUG: Log forces for module 0 once per second
            if (i == 0 && (System.currentTimeMillis() % 1000) < 15) {
                System.out.printf(
                        "[SwervePhysicsCalc] Module0: pos=(%.2f,%.2f,%.2f) hit=%s dist=%.3f norm=(%.2f,%.2f,%.2f) suspForce=(%.1f,%.1f,%.1f) tractForce=(%.1f,%.1f)%n",
                        worldMountPoint.getX(),
                        worldMountPoint.getY(),
                        worldMountPoint.getZ(),
                        hitResult != null ? "YES" : "NO",
                        hitResult != null ? hitResult.hitDistance() - RAYCAST_ORIGIN_OFFSET : -1,
                        hitResult != null ? hitResult.hitNormal().getX() : 0,
                        hitResult != null ? hitResult.hitNormal().getY() : 0,
                        hitResult != null ? hitResult.hitNormal().getZ() : 0,
                        suspensionForce.getX(),
                        suspensionForce.getY(),
                        suspensionForce.getZ(),
                        tractionForce3d.getX(),
                        tractionForce3d.getY());
            }

            // --- 4. Apply Forces Directly ---
            if (normalForceNewtons > 0) {
                chassisBody.applyForceAtPoint(suspensionForce, worldMountPoint);
            }
            chassisBody.applyForceAtPoint(tractionForce3d, worldMountPoint);
        }
    }

    /** Rotates a point by a 3D rotation. */
    private static Translation3d rotatePoint(Translation3d point, Rotation3d rotation) {
        // Convert to rotation matrix multiplication
        double roll = rotation.getX();
        double pitch = rotation.getY();
        double yaw = rotation.getZ();

        double cosYaw = Math.cos(yaw);
        double sinYaw = Math.sin(yaw);
        double cosPitch = Math.cos(pitch);
        double sinPitch = Math.sin(pitch);
        double cosRoll = Math.cos(roll);
        double sinRoll = Math.sin(roll);

        double x = point.getX();
        double y = point.getY();
        double z = point.getZ();

        // Rotation matrix: Rz(yaw) * Ry(pitch) * Rx(roll)
        double newX = x * (cosYaw * cosPitch)
                + y * (cosYaw * sinPitch * sinRoll - sinYaw * cosRoll)
                + z * (cosYaw * sinPitch * cosRoll + sinYaw * sinRoll);
        double newY = x * (sinYaw * cosPitch)
                + y * (sinYaw * sinPitch * sinRoll + cosYaw * cosRoll)
                + z * (sinYaw * sinPitch * cosRoll - cosYaw * sinRoll);
        double newZ = x * (-sinPitch) + y * (cosPitch * sinRoll) + z * (cosPitch * cosRoll);

        return new Translation3d(newX, newY, newZ);
    }
}
