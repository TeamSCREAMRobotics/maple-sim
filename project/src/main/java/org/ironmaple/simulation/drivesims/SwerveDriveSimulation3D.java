package org.ironmaple.simulation.drivesims;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.units.measure.*;
import java.util.Arrays;
import java.util.Optional;
import java.util.function.Supplier;
import org.ironmaple.simulation.debugging.SimDebugLogger;
import org.ironmaple.simulation.drivesims.configs.DriveTrainSimulationConfig;
import org.ironmaple.simulation.physics.PhysicsEngine;
import org.ironmaple.simulation.physics.bullet.BulletBody;

/**
 *
 *
 * <h1>Simulates a Swerve Drivetrain in 3D.</h1>
 *
 * <p>This class provides high-fidelity 3D simulation of a swerve drivetrain with:
 *
 * <ul>
 *   <li>Raycast-based wheel contact detection for proper ground interaction
 *   <li>3D friction forces calculated relative to ground surface normals
 *   <li>Support for driving on inclined surfaces (ramps, charge stations)
 *   <li>Realistic wheel slip and traction limiting
 * </ul>
 */
public class SwerveDriveSimulation3D extends AbstractDriveTrainSimulation3D {
    private final SwerveModuleSimulation[] moduleSimulations;
    protected final GyroSimulation gyroSimulation;
    protected final Translation2d[] moduleTranslations;
    protected final SwerveDriveKinematics kinematics;

    // ==================== Suspension Parameters ====================

    /** Suspension spring constant (N/m). Reduced for softer ride and less oscillation. */
    private static final double SUSPENSION_STIFFNESS = 10000.0;

    /** Target damping ratio. 0.7 = slightly underdamped (responsive feel). */
    private static final double TARGET_DAMPING_RATIO = 0.7;

    /** Compression damping multiplier (full damping on bump). */
    private static final double COMPRESSION_DAMPING_MULT = 1.0;

    /** Expansion damping multiplier (less damping on rebound for responsiveness). */
    private static final double EXPANSION_DAMPING_MULT = 0.5;

    /**
     * Rest length of suspension (meters). Equilibrium Z = chassisHeight + restLength + wheelRadius - compression at
     * rest
     */
    private static final double SUSPENSION_REST_LENGTH = 0.05;

    /** Maximum suspension compression/extension (meters). */
    private static final double SUSPENSION_MAX_TRAVEL = 0.1;

    /** Wheel radius (meters). */
    private static final double WHEEL_RADIUS = 0.0508; // 2 inch wheel

    /** Height of chassis collision box (meters). Low = stable. */
    private static final double CHASSIS_HEIGHT = 0.1;

    /** Maximum force per wheel to prevent physics explosions (N). */
    private static final double MAX_SUSPENSION_FORCE = 1000.0; // 4 wheels * 1000N = 4000N = 400kg support

    /**
     *
     *
     * <h2>Creates a 3D Swerve Drive Simulation.</h2>
     *
     * @param config the drivetrain configuration
     * @param initialPoseOnField the initial 2D pose
     */
    public SwerveDriveSimulation3D(DriveTrainSimulationConfig config, Pose2d initialPoseOnField) {
        super(config, initialPoseOnField);

        this.moduleTranslations = config.moduleTranslations;
        this.moduleSimulations = Arrays.stream(config.swerveModuleSimulationFactories)
                .map(Supplier::get)
                .toArray(SwerveModuleSimulation[]::new);
        this.gyroSimulation = config.gyroSimulationFactory.get();

        this.kinematics = new SwerveDriveKinematics(moduleTranslations);
    }

    @Override
    public void simulationSubTick(int subTickNum) {
        if (physicsBody == null || physicsEngine == null) return;

        SimDebugLogger.incrementTick();

        // Get current pose and velocities for logging
        var pose3d = physicsBody.getPose3d();
        var rotation = pose3d.getRotation();
        Rotation2d heading2d = rotation.toRotation2d();
        Translation3d linearVel = physicsBody.getLinearVelocityMPS();

        // Get yaw rate - IMPORTANT: Use raw Z component, not Rotation3d.getZ() which is
        // corrupted!
        double yawRateOld = physicsBody.getAngularVelocityRadPerSec().getZ(); // Old (potentially wrong) method
        double yawRateNew =
                (physicsBody instanceof BulletBody) ? ((BulletBody) physicsBody).getRawAngularVelocityZ() : yawRateOld;

        // Log every 50 ticks to avoid spam (10x per second at 500Hz sim rate)
        if (subTickNum == 0) {
            SimDebugLogger.logPose(String.format(
                    "X=%.3f Y=%.3f Z=%.3f Yaw=%.1f° Roll=%.1f° Pitch=%.1f°",
                    pose3d.getX(),
                    pose3d.getY(),
                    pose3d.getZ(),
                    Math.toDegrees(heading2d.getRadians()),
                    Math.toDegrees(rotation.getX()),
                    Math.toDegrees(rotation.getY())));

            SimDebugLogger.logVelocity(String.format(
                    "Linear: vx=%.3f vy=%.3f vz=%.3f | YawRate: old=%.3f new=%.3f rad/s",
                    linearVel.getX(), linearVel.getY(), linearVel.getZ(), yawRateOld, yawRateNew));

            SimDebugLogger.logHeading(
                    String.format("Heading2d=%.1f° (from toRotation2d)", Math.toDegrees(heading2d.getRadians())));
        }

        // Apply suspension and traction forces
        simulateModules();

        // Update gyro with yaw rate - USE THE NEW RAW VALUE
        gyroSimulation.updateSimulationSubTick(yawRateNew);
    }

    /** Calculates the critical damping coefficient for a given mass per wheel. c_crit = 2 * sqrt(k * m) */
    private double calculateCriticalDamping(double massPerWheelKg) {
        return 2.0 * Math.sqrt(SUSPENSION_STIFFNESS * massPerWheelKg);
    }

    /**
     *
     *
     * <h2>Rotates a Point by a 3D Rotation.</h2>
     */
    private Translation3d rotatePoint(Translation3d point, edu.wpi.first.math.geometry.Rotation3d rotation) {
        // Use quaternion rotation
        var quaternion = rotation.getQuaternion();
        double qw = quaternion.getW(), qx = quaternion.getX(), qy = quaternion.getY(), qz = quaternion.getZ();
        double px = point.getX(), py = point.getY(), pz = point.getZ();

        // Quaternion rotation: q * p * q^-1
        double rx = qw * qw * px
                + 2 * qy * qw * pz
                - 2 * qz * qw * py
                + qx * qx * px
                + 2 * qy * qx * py
                + 2 * qz * qx * pz
                - qz * qz * px
                - qy * qy * px;
        double ry = 2 * qx * qy * px
                + qy * qy * py
                + 2 * qz * qy * pz
                + 2 * qw * qz * px
                - qz * qz * py
                + qw * qw * py
                - 2 * qx * qw * pz
                - qx * qx * py;
        double rz = 2 * qx * qz * px
                + 2 * qy * qz * py
                + qz * qz * pz
                - 2 * qw * qy * px
                - qy * qy * pz
                + 2 * qw * qx * py
                - qx * qx * pz
                + qw * qw * pz;

        return new Translation3d(rx, ry, rz);
    }

    /**
     *
     *
     * <h2>Gets the Maximum Linear Velocity.</h2>
     */
    public LinearVelocity maxLinearVelocity() {
        return moduleSimulations[0].config.maximumGroundSpeed();
    }

    /**
     *
     *
     * <h2>Gets the Maximum Angular Velocity.</h2>
     */
    public AngularVelocity maxAngularVelocity() {
        return RadiansPerSecond.of(maxLinearVelocity().in(MetersPerSecond)
                / config.driveBaseRadius().in(Meters));
    }

    /**
     *
     *
     * <h2>Gets the 3D Pose Adjusted for Ground-Level Visualization.</h2>
     *
     * <p>Overridden to account for suspension rest length and wheel radius.
     *
     * @return the 3D pose such that Z=0 corresponds to the bottom of the wheels at rest
     */
    @Override
    public Pose3d getSimulatedDriveTrainPose3dGroundRelative() {
        if (physicsBody == null) return new Pose3d();
        Pose3d physicsPose = physicsBody.getPose3d();

        // Calculate offset from center of mass to bottom of wheels at rest
        // zOffset = (chassisBodyHeight / 2) + SUSPENSION_REST_LENGTH + WHEEL_RADIUS
        // Note: chassisBodyHeight is in the superclass, we assume CHASSIS_HEIGHT is
        // consistent
        double zOffset = (CHASSIS_HEIGHT / 2.0) + SUSPENSION_REST_LENGTH + WHEEL_RADIUS;

        return new Pose3d(
                physicsPose.getX(), physicsPose.getY(), physicsPose.getZ() - zOffset, physicsPose.getRotation());
    }

    /**
     *
     *
     * <h2>Simulates All Swerve Modules (Suspension + Traction).</h2>
     *
     * <p>Iterates through each module to:
     *
     * <ul>
     *   <li>Calculate suspension force (raycast)
     *   <li>Calculate ground velocity
     *   <li>Update module simulation with dynamic normal force
     *   <li>Apply resulting suspension and traction forces to the body
     * </ul>
     */
    private void simulateModules() {
        var pose3d = physicsBody.getPose3d();
        var rotation = pose3d.getRotation();
        // CRITICAL: Use toRotation2d() to properly extract yaw from 3D rotation
        // rotation.getZ() returns rotation vector Z component, NOT yaw angle!
        Rotation2d robotHeading = rotation.toRotation2d();

        // Calculate per-wheel mass for damping
        double robotMassKg = config.robotMass.in(edu.wpi.first.units.Units.Kilograms);
        double massPerWheel = robotMassKg / moduleTranslations.length;
        double criticalDamping = calculateCriticalDamping(massPerWheel);
        double baseDamping = TARGET_DAMPING_RATIO * criticalDamping;

        for (int i = 0; i < moduleTranslations.length; i++) {
            Translation2d moduleOffset = moduleTranslations[i];
            SwerveModuleSimulation module = moduleSimulations[i];

            // --- 1. Suspension Calculation ---
            // Mount point is at bottom of chassis (relative to chassis center)
            Translation3d localMountPoint =
                    new Translation3d(moduleOffset.getX(), moduleOffset.getY(), -CHASSIS_HEIGHT / 2);
            Translation3d worldMountPoint =
                    rotatePoint(localMountPoint, rotation).plus(pose3d.getTranslation());

            // Raycast downward from mount point
            Translation3d rayDirection = new Translation3d(0, 0, -1);
            double maxRayDistance = SUSPENSION_REST_LENGTH + SUSPENSION_MAX_TRAVEL + WHEEL_RADIUS;
            Optional<PhysicsEngine.RaycastResult> hit =
                    physicsEngine.raycast(worldMountPoint, rayDirection, maxRayDistance);

            double normalForceNewtons = 0.0;

            if (hit.isPresent()) {
                double groundDistance = hit.get().hitDistance();
                double suspensionCompression = SUSPENSION_REST_LENGTH + WHEEL_RADIUS - groundDistance;

                if (suspensionCompression > 0) {
                    // Get velocity of mount point (not ground point!)
                    Translation3d mountVelocity = physicsBody.getLinearVelocityAtPointMPS(worldMountPoint);
                    double compressionVelocity = -mountVelocity.getZ(); // Positive = compressing

                    // Select damping based on compression vs expansion
                    double dampingCoeff = compressionVelocity > 0
                            ? baseDamping * COMPRESSION_DAMPING_MULT
                            : baseDamping * EXPANSION_DAMPING_MULT;

                    // Spring + Damper force
                    double springForce = SUSPENSION_STIFFNESS * suspensionCompression;
                    double damperForce = dampingCoeff * compressionVelocity;
                    normalForceNewtons = Math.max(0, springForce + damperForce);

                    // Cap force to prevent physics explosions
                    normalForceNewtons = Math.min(normalForceNewtons, MAX_SUSPENSION_FORCE);

                    // Apply suspension force at MOUNT POINT (chassis attachment), not ground
                    // Force direction is along ground normal (usually straight up)
                    Translation3d normal = hit.get().hitNormal();
                    Translation3d suspensionForce = normal.times(normalForceNewtons);
                    physicsBody.applyForceAtPoint(suspensionForce, worldMountPoint);
                }
            }

            // --- 2. Traction/Propulsion Calculation ---
            // Get ground velocity at mount point
            Translation3d pointVelocity3d = physicsBody.getLinearVelocityAtPointMPS(worldMountPoint);
            org.dyn4j.geometry.Vector2 groundVelocity2d =
                    new org.dyn4j.geometry.Vector2(pointVelocity3d.getX(), pointVelocity3d.getY());

            // Update module simulation with dynamic normal force
            // If airborne, normalForceNewtons is 0, so grip is 0.
            org.dyn4j.geometry.Vector2 tractionForce2d =
                    module.updateSimulationSubTickGetModuleForce(groundVelocity2d, robotHeading, normalForceNewtons);

            // Apply traction force in 3D (XY plane for flat ground)
            Translation3d tractionForce3d = new Translation3d(tractionForce2d.x, tractionForce2d.y, 0);
            physicsBody.applyForceAtPoint(tractionForce3d, worldMountPoint);

            // Debug logging for forces (only log module 0 to reduce spam)
            if (i == 0) {
                SimDebugLogger.logForce(String.format(
                        "Mod0: suspension=%.1fN traction=(%.1f,%.1f)N groundVel=(%.3f,%.3f)m/s heading=%.1f°",
                        normalForceNewtons,
                        tractionForce2d.x,
                        tractionForce2d.y,
                        groundVelocity2d.x,
                        groundVelocity2d.y,
                        Math.toDegrees(robotHeading.getRadians())));
            }
        }
    }

    /**
     *
     *
     * <h2>Gets the Swerve Module Simulations.</h2>
     */
    public SwerveModuleSimulation[] getModules() {
        return moduleSimulations;
    }

    /**
     *
     *
     * <h2>Gets the Gyro Simulation.</h2>
     */
    public GyroSimulation getGyroSimulation() {
        return gyroSimulation;
    }

    /**
     *
     *
     * <h2>Gets the Drive Base Radius.</h2>
     */
    public Distance driveBaseRadius() {
        return config.driveBaseRadius();
    }
}
