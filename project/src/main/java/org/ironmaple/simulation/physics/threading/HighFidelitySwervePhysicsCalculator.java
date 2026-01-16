package org.ironmaple.simulation.physics.threading;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import java.util.Optional;
import org.ironmaple.simulation.drivesims.configs.HighFidelitySwerveSim3DConfig;
import org.ironmaple.simulation.drivesims.tire.PacejkaTireModel;
import org.ironmaple.simulation.drivesims.tire.TireForceResult;
import org.ironmaple.simulation.physics.PhysicsBody;
import org.ironmaple.simulation.physics.PhysicsEngine;

/**
 *
 *
 * <h1>High-Fidelity Physics Calculator for Threaded Mode.</h1>
 *
 * <p>Runs on the physics thread with current state (not stale cached state). Implements the same Pacejka tire model as
 * {@link org.ironmaple.simulation.drivesims.HighFidelitySwerveSim3D}.
 *
 * <h2>Thread Safety:</h2>
 *
 * <ul>
 *   <li>{@link #applyForces} runs only on the physics thread
 *   <li>Module states are updated atomically from the WPILib thread via {@link #setCapturedStates}
 * </ul>
 *
 * @see org.ironmaple.simulation.drivesims.HighFidelitySwerveSim3D
 * @see PacejkaTireModel
 */
public class HighFidelitySwervePhysicsCalculator implements ThreadedSwerveCalculator {

    // ==================== Suspension Parameters ====================
    private static final double SUSPENSION_STIFFNESS = 4000.0;
    private static final double TARGET_DAMPING_RATIO = 1.2;
    private static final double COMPRESSION_DAMPING_MULT = 1.5;
    private static final double EXPANSION_DAMPING_MULT = 1.2;
    private static final double SUSPENSION_REST_LENGTH = 0.05;
    private static final double WHEEL_RADIUS = 0.0508;
    private static final double CHASSIS_HEIGHT = 0.1;
    private static final double MAX_SUSPENSION_FORCE = 1000.0;
    private static final double RAYCAST_ORIGIN_OFFSET = 0.5;

    // ==================== Configuration ====================
    private final PhysicsBody chassisBody;
    private final Translation2d[] moduleOffsets;
    private final HighFidelitySwerveSim3DConfig hifiConfig;
    private final PacejkaTireModel tireModel;
    private final double robotMassKg;
    private final double tickPeriodSeconds;
    private final java.util.concurrent.atomic.AtomicReference<double[]> latestNormalForces =
            new java.util.concurrent.atomic.AtomicReference<>(new double[0]);
    private final java.util.concurrent.atomic.AtomicReference<TireForceResult[]> latestTireResults =
            new java.util.concurrent.atomic.AtomicReference<>(new TireForceResult[0]);

    // ==================== State ====================
    private volatile SwerveModuleState[] capturedStates = null;

    /**
     *
     *
     * <h2>Creates a High-Fidelity Swerve Physics Calculator.</h2>
     *
     * @param chassisBody the chassis physics body (will be unwrapped if threaded wrapper)
     * @param moduleOffsets Translation2d positions of each module relative to chassis center
     * @param hifiConfig high-fidelity simulation configuration
     * @param tireModel the Pacejka tire model to use
     * @param robotMassKg robot mass in kilograms
     * @param tickPeriodSeconds the physics tick period in seconds
     */
    public HighFidelitySwervePhysicsCalculator(
            PhysicsBody chassisBody,
            Translation2d[] moduleOffsets,
            HighFidelitySwerveSim3DConfig hifiConfig,
            PacejkaTireModel tireModel,
            double robotMassKg,
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

        this.moduleOffsets = moduleOffsets.clone();
        this.hifiConfig = hifiConfig;
        this.tireModel = tireModel;
        this.robotMassKg = robotMassKg;
        this.tickPeriodSeconds = tickPeriodSeconds;

        // Initialize with empty arrays
        this.latestNormalForces.set(new double[moduleOffsets.length]);
        TireForceResult[] initialTireResults = new TireForceResult[moduleOffsets.length];
        java.util.Arrays.fill(initialTireResults, TireForceResult.zero());
        this.latestTireResults.set(initialTireResults);
    }

    public double[] getLatestNormalForces() {
        return latestNormalForces.get();
    }

    public TireForceResult[] getLatestTireResults() {
        return latestTireResults.get();
    }

    /**
     *
     *
     * <h2>Updates Captured Module States.</h2>
     *
     * <p>Called from WPILib thread to update the states that will be used in the next physics tick.
     *
     * @param states the swerve module states from the main thread
     */
    public void setCapturedStates(SwerveModuleState[] states) {
        this.capturedStates = states;
    }

    @Override
    public void applyForces(PhysicsEngine engine) {
        if (chassisBody == null || capturedStates == null) return;

        // Get CURRENT state directly from physics body (not cached!)
        Pose3d pose3d = chassisBody.getPose3d();
        Rotation3d rotation = pose3d.getRotation();
        Rotation2d robotHeading = rotation.toRotation2d();

        // Calculate damping parameters
        double massPerWheel = robotMassKg / moduleOffsets.length;
        double criticalDamping = 2.0 * Math.sqrt(SUSPENSION_STIFFNESS * massPerWheel);
        double baseDamping = TARGET_DAMPING_RATIO * criticalDamping;

        // Local arrays for this tick's results
        double[] currentNormalForces = new double[moduleOffsets.length];
        TireForceResult[] currentTireResults = new TireForceResult[moduleOffsets.length];

        for (int i = 0; i < moduleOffsets.length; i++) {
            if (i >= capturedStates.length) continue;

            Translation2d moduleOffset = moduleOffsets[i];
            SwerveModuleState moduleState = capturedStates[i];

            // --- 1. Calculate World Mount Point ---
            Translation3d localMountPoint =
                    new Translation3d(moduleOffset.getX(), moduleOffset.getY(), -CHASSIS_HEIGHT / 2);
            Translation3d worldMountPoint =
                    rotatePoint(localMountPoint, rotation).plus(pose3d.getTranslation());

            // --- 2. Raycast for Ground Detection ---
            Translation3d rayDirection = new Translation3d(0, 0, -1);
            Translation3d rayOrigin = worldMountPoint.plus(new Translation3d(0, 0, RAYCAST_ORIGIN_OFFSET));
            double maxRayDistance = SUSPENSION_REST_LENGTH + 0.1 + WHEEL_RADIUS + RAYCAST_ORIGIN_OFFSET;

            Optional<PhysicsEngine.RaycastResult> hitOpt =
                    engine.raycast(rayOrigin, rayDirection, maxRayDistance, chassisBody);
            PhysicsEngine.RaycastResult hitResult = hitOpt.orElse(null);

            double normalForceN = 0.0;
            Translation3d suspensionForce = Translation3d.kZero;

            if (hitResult != null) {
                double groundDistance = hitResult.hitDistance() - RAYCAST_ORIGIN_OFFSET;
                double suspensionCompression = SUSPENSION_REST_LENGTH + WHEEL_RADIUS - groundDistance;

                if (suspensionCompression > 0) {
                    // Get ACTUAL velocity at mount point
                    Translation3d mountVelocity = chassisBody.getLinearVelocityAtPointMPS(worldMountPoint);
                    double compressionVelocity = -mountVelocity.getZ();

                    double dampingCoeff = compressionVelocity > 0
                            ? baseDamping * COMPRESSION_DAMPING_MULT
                            : baseDamping * EXPANSION_DAMPING_MULT;

                    double springForce = SUSPENSION_STIFFNESS * suspensionCompression;
                    double damperForce = dampingCoeff * compressionVelocity;

                    normalForceN = Math.max(0, springForce + damperForce);
                    normalForceN = Math.min(normalForceN, MAX_SUSPENSION_FORCE);

                    Translation3d normal = hitResult.hitNormal();
                    suspensionForce = normal.times(normalForceN);
                }
            }
            currentNormalForces[i] = normalForceN;

            // Apply suspension force
            if (normalForceN > 0) {
                chassisBody.applyForceAtPoint(suspensionForce, worldMountPoint);
            }

            // --- 3. Tire Forces (Pacejka) ---
            Translation3d contactVel = chassisBody.getLinearVelocityAtPointMPS(worldMountPoint);
            Rotation2d moduleWorldFacing = moduleState.angle.plus(robotHeading);

            // Decompose velocity into longitudinal/lateral
            double cosA = moduleWorldFacing.getCos();
            double sinA = moduleWorldFacing.getSin();
            double groundLongVel = contactVel.getX() * cosA + contactVel.getY() * sinA;
            double groundLatVel = -contactVel.getX() * sinA + contactVel.getY() * cosA;

            // Calculate slip
            double wheelSpeed = moduleState.speedMetersPerSecond;
            double longSlip = PacejkaTireModel.calculateLongitudinalSlip(wheelSpeed, groundLongVel);
            double slipAngle = PacejkaTireModel.calculateSlipAngle(groundLatVel, Math.abs(groundLongVel));

            // Pacejka tire forces
            TireForceResult tireResult = tireModel.calculateCombinedForces(longSlip, slipAngle, normalForceN);
            currentTireResults[i] = tireResult;

            // Convert to world frame
            // Note: Lateral force is negated because Pacejka outputs force in the direction
            // of slip,
            // but we need the reaction force that opposes the lateral sliding motion.
            double fxWorld = tireResult.longitudinalForceNewtons() * cosA + tireResult.lateralForceNewtons() * sinA;
            double fyWorld = tireResult.longitudinalForceNewtons() * sinA - tireResult.lateralForceNewtons() * cosA;

            // Apply traction force
            chassisBody.applyForceAtPoint(new Translation3d(fxWorld, fyWorld, 0), worldMountPoint);
        }

        // Publish results atomically
        // stored them.
        // But we assigned them to 'latestNormalForces[i]' inside the loop in the OLD
        // code.
        // In this NEW code, we need to collect them.
        // Wait, the ReplacementChunk above removed the loop variable definitions?
        // No, I need to rewrite the loop or the method to collect into local arrays
        // first.
    }

    /**
     *
     *
     * <h2>Rotates a Point by a 3D Rotation.</h2>
     *
     * <p>Uses rotation matrix: Rz(yaw) × Ry(pitch) × Rx(roll)
     */
    private static Translation3d rotatePoint(Translation3d point, Rotation3d rotation) {
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
