package org.ironmaple.simulation.physics.threading;

import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.units.measure.Time;
import java.util.Optional;
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
public class SwervePhysicsCalculator implements ThreadedSwerveCalculator {

    // ==================== Suspension Parameters ====================
    // ==================== Suspension Parameters ====================
    private static final double SUSPENSION_STIFFNESS = 12000.0;
    private static final double TARGET_DAMPING_RATIO = 1.2;
    private static final double COMPRESSION_DAMPING_MULT = 1.5;
    private static final double EXPANSION_DAMPING_MULT = 1.2;
    private static final double SUSPENSION_MAX_TRAVEL = 0.1;
    private static final double MAX_SUSPENSION_FORCE = 1000.0;
    private static final double RAYCAST_ORIGIN_OFFSET = 0.5;

    private final PhysicsBody chassisBody;
    private final Translation2d[] moduleOffsets;
    private final SwerveModuleSimulation[] modules;
    private final double robotMassKg;
    private final Time tickPeriod;
    private volatile SwerveModuleState[] capturedStates = null;

    private final double suspensionRestLength;
    private final double wheelRadius;
    private final double chassisHeight;

    /**
     * Creates a swerve physics calculator.
     *
     * @param chassisBody The chassis physics body
     * @param config The drivetrain simulation configuration
     * @param tickPeriodSeconds The physics tick period in seconds (e.g., 1.0/120 for 120Hz)
     */
    public SwervePhysicsCalculator(
            PhysicsBody chassisBody,
            org.ironmaple.simulation.drivesims.configs.DriveTrainSimulationConfig config,
            double tickPeriodSeconds) {
        // Unwrap threaded wrappers to get the real body for direct physics thread
        // access
        if (chassisBody instanceof ThreadedBulletBody tbb) {
            this.chassisBody = tbb.getRealBody();
        } else if (chassisBody instanceof ThreadedJoltBody tjb) {
            this.chassisBody = tjb.getRealBody();
        } else {
            this.chassisBody = chassisBody;
        }
        this.moduleOffsets = config.moduleTranslations.clone();
        this.modules = java.util.Arrays.stream(config.swerveModuleSimulationFactories)
                .map(java.util.function.Supplier::get)
                .toArray(SwerveModuleSimulation[]::new);
        this.robotMassKg = config.robotMass.in(edu.wpi.first.units.Units.Kilograms);
        this.tickPeriod = Seconds.of(tickPeriodSeconds);

        double groundClearance = config.groundClearance.in(edu.wpi.first.units.Units.Meters);

        // Calculate rest length
        double massPerWheel = robotMassKg / moduleOffsets.length;
        double weightPerWheel = massPerWheel * 9.81;
        double staticCompression = weightPerWheel / SUSPENSION_STIFFNESS;
        this.suspensionRestLength = groundClearance + staticCompression;

        this.wheelRadius = config.wheelRadius.in(edu.wpi.first.units.Units.Meters);
        this.chassisHeight = config.chassisHeight.in(edu.wpi.first.units.Units.Meters);

        // Determine COM height (same logic as AbstractDriveTrainSimulation3D)
        double geometricCenterZ = groundClearance + this.chassisHeight / 2.0;
        double targetComZ = config.centerOfMass.getZ();
        if (targetComZ < 0.01 || targetComZ > 1.0) {
            targetComZ = geometricCenterZ;
        }
        this.comHeightAboveGround = targetComZ;
        double wheelRadius = config.wheelRadius.in(edu.wpi.first.units.Units.Meters);
        this.mountPointZ = groundClearance + wheelRadius - comHeightAboveGround;
    }

    private final double comHeightAboveGround;
    private final double mountPointZ;

    /**
     * Updates the captured states for the next tick.
     *
     * @param states The swerve module states from the beginning of the frame.
     */
    public void setCapturedStates(SwerveModuleState[] states) {
        this.capturedStates = states;
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
            Translation3d localMountPoint = new Translation3d(moduleOffset.getX(), moduleOffset.getY(), mountPointZ);
            Translation3d worldMountPoint =
                    rotatePoint(localMountPoint, rotation).plus(pose3d.getTranslation());

            // --- 2. Raycast for Ground Detection ---
            Translation3d rayDirection = new Translation3d(0, 0, -1);
            Translation3d rayOrigin = worldMountPoint.plus(new Translation3d(0, 0, RAYCAST_ORIGIN_OFFSET));
            double maxRayDistance = suspensionRestLength + SUSPENSION_MAX_TRAVEL + wheelRadius + RAYCAST_ORIGIN_OFFSET;

            // Exclude chassisBody from raycast to prevent self-collision
            Optional<PhysicsEngine.RaycastResult> hitOpt =
                    engine.raycast(rayOrigin, rayDirection, maxRayDistance, chassisBody);
            PhysicsEngine.RaycastResult hitResult = hitOpt.orElse(null);

            double normalForceNewtons = 0.0;
            Translation3d suspensionForce = Translation3d.kZero;

            if (hitResult != null) {
                double groundDistance = hitResult.hitDistance() - RAYCAST_ORIGIN_OFFSET;
                double suspensionCompression = suspensionRestLength + wheelRadius - groundDistance;

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

            org.dyn4j.geometry.Vector2 tractionForce2d;
            if (capturedStates != null && i < capturedStates.length) {
                // Use captured state for determinism (Physics Thread does NOT mutate module)
                tractionForce2d = module.getModuleForceFromState(
                        capturedStates[i], groundVelocity2d, robotHeading, normalForceNewtons);
            } else {
                // DETERMINISM WARNING: This fallback path reads live (mutable) state from
                // the module, which breaks determinism in threaded mode. This should NEVER
                // execute if lock-step mode is working correctly.
                // If you see this warning, there's a bug in the input synchronization.
                System.err.println("[SwervePhysicsCalculator] WARNING: capturedStates is null! "
                        + "Falling back to live state - DETERMINISM BROKEN. "
                        + "Module index: " + i);
                tractionForce2d = module.updateSimulationSubTickGetModuleForce(
                        groundVelocity2d, robotHeading, normalForceNewtons, tickPeriod);
            }

            Translation3d tractionForce3d = new Translation3d(tractionForce2d.x, tractionForce2d.y, 0);

            // DEBUG: Log forces for module 0 once per second
            // if (i == 0 && (System.currentTimeMillis() % 1000) < 15) {
            // System.out.printf(
            // "[SwervePhysicsCalc] Module0: pos=(%.2f,%.2f,%.2f) hit=%s dist=%.3f
            // norm=(%.2f,%.2f,%.2f) suspForce=(%.1f,%.1f,%.1f) tractForce=(%.1f,%.1f)%n",
            // worldMountPoint.getX(),
            // worldMountPoint.getY(),
            // worldMountPoint.getZ(),
            // hitResult != null ? "YES" : "NO",
            // hitResult != null ? hitResult.hitDistance() - RAYCAST_ORIGIN_OFFSET : -1,
            // hitResult != null ? hitResult.hitNormal().getX() : 0,
            // hitResult != null ? hitResult.hitNormal().getY() : 0,
            // hitResult != null ? hitResult.hitNormal().getZ() : 0,
            // suspensionForce.getX(),
            // suspensionForce.getY(),
            // suspensionForce.getZ(),
            // tractionForce3d.getX(),
            // tractionForce3d.getY());
            // }

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
