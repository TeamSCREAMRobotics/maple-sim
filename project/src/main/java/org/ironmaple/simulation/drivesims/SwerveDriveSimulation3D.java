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
import org.ironmaple.simulation.physics.threading.SimulationState;
import org.ironmaple.simulation.physics.threading.ThreadedPhysicsProxy;

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

    // Track raycast IDs from previous frame (for threaded mode)
    private int[] lastQueuedRaycastIds;

    // Physics calculator for threaded mode (runs on physics thread)
    private org.ironmaple.simulation.physics.threading.SwervePhysicsCalculator physicsCalculator;
    private boolean calculatorRegistered = false;

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
        this.lastQueuedRaycastIds = new int[moduleTranslations.length];
        java.util.Arrays.fill(lastQueuedRaycastIds, -1); // Initialize to invalid
    }

    @Override
    public void simulationSubTick(int subTickNum) {
        if (physicsBody == null || physicsEngine == null) return;

        SimDebugLogger.incrementTick();

        // Determine threading mode
        boolean isThreaded = arena != null && arena.isThreaded();
        ThreadedPhysicsProxy proxy = isThreaded ? arena.getThreadedProxy() : null;
        SimulationState state = isThreaded ? proxy.getCachedState() : null;

        // Register physics calculator on first tick (threaded mode only)
        if (isThreaded && !calculatorRegistered && proxy != null) {
            double robotMassKg = config.robotMass.in(edu.wpi.first.units.Units.Kilograms);
            physicsCalculator = new org.ironmaple.simulation.physics.threading.SwervePhysicsCalculator(
                    physicsBody, moduleTranslations, moduleSimulations, robotMassKg);
            proxy.registerCalculator(physicsCalculator);
            calculatorRegistered = true;
            System.out.println("[MapleSim3D] Registered SwervePhysicsCalculator with physics thread");
        }

        // Get body ID - works for both BulletBody and ThreadedBulletBody
        int bodyId = -1;
        if (physicsBody instanceof BulletBody bb) {
            bodyId = bb.getBodyId();
        } else if (physicsBody instanceof org.ironmaple.simulation.physics.threading.ThreadedBulletBody tbb) {
            bodyId = tbb.getBodyId();
        }

        // Get current pose and velocities
        Pose3d pose3d;
        Translation3d linearVel;
        double yawRate;

        if (isThreaded && state != null && bodyId >= 0) {
            // Threaded mode: use cached state from physics thread
            SimulationState.BodyState bodyState = state.getBodyState(bodyId);
            if (bodyState != null) {
                pose3d = bodyState.pose();
                linearVel = bodyState.linearVelocity();
                yawRate = bodyState.angularVelocity().getZ();
            } else {
                // Body not yet in state (first tick), use direct access
                pose3d = physicsBody.getPose3d();
                linearVel = physicsBody.getLinearVelocityMPS();
                yawRate = getYawRate(physicsBody);
            }
        } else {
            // Sync mode: direct physics access
            pose3d = physicsBody.getPose3d();
            linearVel = physicsBody.getLinearVelocityMPS();
            yawRate = getYawRate(physicsBody);
        }

        var rotation = pose3d.getRotation();
        Rotation2d heading2d = rotation.toRotation2d();

        // Log every 50 ticks to avoid spam
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
                    "Linear: vx=%.3f vy=%.3f vz=%.3f | YawRate=%.3f rad/s | Threaded=%b",
                    linearVel.getX(), linearVel.getY(), linearVel.getZ(), yawRate, isThreaded));

            SimDebugLogger.logHeading(
                    String.format("Heading2d=%.1f° (from toRotation2d)", Math.toDegrees(heading2d.getRadians())));
        }

        // Apply suspension and traction forces
        // In threaded mode, forces are calculated on the physics thread by
        // SwervePhysicsCalculator
        // In sync mode, we calculate and apply them directly here
        if (!isThreaded) {
            simulateModules(pose3d, linearVel, bodyId, proxy, state);
        }
        // Note: In threaded mode, SwervePhysicsCalculator.applyForces() is called
        // by PhysicsThread each tick with CURRENT state, not stale cached state

        // Update gyro with yaw rate
        gyroSimulation.updateSimulationSubTick(yawRate);
    }

    /** Gets the yaw rate from a physics body, handling different wrapper types. */
    private double getYawRate(org.ironmaple.simulation.physics.PhysicsBody body) {
        if (body instanceof BulletBody bb) {
            return bb.getRawAngularVelocityZ();
        } else if (body instanceof org.ironmaple.simulation.physics.threading.ThreadedBulletBody tbb) {
            return tbb.getRawAngularVelocityZ();
        } else {
            return body.getAngularVelocityRadPerSec().getZ();
        }
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
     *
     * @param pose3d Current robot pose (from physics or cached state)
     * @param linearVel Current linear velocity
     * @param bodyId Body ID for threaded proxy (-1 if sync)
     * @param proxy Threaded proxy (null if sync mode)
     * @param state Cached simulation state (null if sync mode)
     */
    private void simulateModules(
            Pose3d pose3d, Translation3d linearVel, int bodyId, ThreadedPhysicsProxy proxy, SimulationState state) {
        var rotation = pose3d.getRotation();
        Rotation2d robotHeading = rotation.toRotation2d();
        boolean isThreaded = proxy != null;

        // Calculate per-wheel mass for damping
        double robotMassKg = config.robotMass.in(edu.wpi.first.units.Units.Kilograms);
        double massPerWheel = robotMassKg / moduleTranslations.length;
        double criticalDamping = calculateCriticalDamping(massPerWheel);
        double baseDamping = TARGET_DAMPING_RATIO * criticalDamping;

        for (int i = 0; i < moduleTranslations.length; i++) {
            Translation2d moduleOffset = moduleTranslations[i];
            SwerveModuleSimulation module = moduleSimulations[i];

            // --- 1. Suspension Calculation ---
            Translation3d localMountPoint =
                    new Translation3d(moduleOffset.getX(), moduleOffset.getY(), -CHASSIS_HEIGHT / 2);
            Translation3d worldMountPoint =
                    rotatePoint(localMountPoint, rotation).plus(pose3d.getTranslation());

            Translation3d rayDirection = new Translation3d(0, 0, -1);
            // Start raycast higher to avoid starting inside the ground if the robot sinks
            double rayOriginOffset = 0.5;
            Translation3d rayOrigin = worldMountPoint.plus(new Translation3d(0, 0, rayOriginOffset));
            double maxRayDistance = SUSPENSION_REST_LENGTH + SUSPENSION_MAX_TRAVEL + WHEEL_RADIUS + rayOriginOffset;

            // Get raycast result (sync: direct, threaded: from cached state)
            PhysicsEngine.RaycastResult hitResult = null;
            if (isThreaded && state != null) {
                // Look up result from PREVIOUS tick using stored ID
                int prevRaycastId = lastQueuedRaycastIds[i];
                if (prevRaycastId >= 0) {
                    var optResult = proxy.getCachedRaycast(prevRaycastId);
                    hitResult = optResult.orElse(null);
                }

                // Queue raycast for NEXT tick (store ID for use next frame)
                int newId = proxy.queueRaycast(rayOrigin, rayDirection, maxRayDistance);
                lastQueuedRaycastIds[i] = newId;
            } else {
                // Sync mode: direct raycast
                Optional<PhysicsEngine.RaycastResult> hit =
                        physicsEngine.raycast(rayOrigin, rayDirection, maxRayDistance);
                hitResult = hit.orElse(null);
            }

            double normalForceNewtons = 0.0;
            Translation3d suspensionForce = Translation3d.kZero;

            if (hitResult != null) {
                double groundDistance = hitResult.hitDistance() - rayOriginOffset;
                double suspensionCompression = SUSPENSION_REST_LENGTH + WHEEL_RADIUS - groundDistance;

                if (suspensionCompression > 0) {
                    // Get velocity of mount point
                    Translation3d mountVelocity;
                    if (isThreaded) {
                        // Approximate: use linear velocity (ignoring angular contribution for
                        // simplicity)
                        mountVelocity = linearVel;
                    } else {
                        mountVelocity = physicsBody.getLinearVelocityAtPointMPS(worldMountPoint);
                    }
                    double compressionVelocity = -mountVelocity.getZ();

                    double dampingCoeff = compressionVelocity > 0
                            ? baseDamping * COMPRESSION_DAMPING_MULT
                            : baseDamping * EXPANSION_DAMPING_MULT;

                    double springForce = SUSPENSION_STIFFNESS * suspensionCompression;
                    double damperForce = dampingCoeff * compressionVelocity;

                    // Ensure normal force is non-negative (can't pull ground)
                    normalForceNewtons = Math.max(0, springForce + damperForce);
                    normalForceNewtons = Math.min(normalForceNewtons, MAX_SUSPENSION_FORCE);

                    Translation3d normal = hitResult.hitNormal();
                    suspensionForce = normal.times(normalForceNewtons);
                }
            }

            // --- 2. Traction/Propulsion Calculation ---
            Translation3d pointVelocity3d;
            if (isThreaded) {
                pointVelocity3d = linearVel; // Approximation for threaded mode
            } else {
                pointVelocity3d = physicsBody.getLinearVelocityAtPointMPS(worldMountPoint);
            }
            org.dyn4j.geometry.Vector2 groundVelocity2d =
                    new org.dyn4j.geometry.Vector2(pointVelocity3d.getX(), pointVelocity3d.getY());

            org.dyn4j.geometry.Vector2 tractionForce2d =
                    module.updateSimulationSubTickGetModuleForce(groundVelocity2d, robotHeading, normalForceNewtons);

            Translation3d tractionForce3d = new Translation3d(tractionForce2d.x, tractionForce2d.y, 0);

            // Apply forces (sync: direct, threaded: queue)
            if (isThreaded) {
                if (normalForceNewtons > 0) {
                    proxy.queueForceAtPoint(bodyId, suspensionForce, worldMountPoint);
                }
                proxy.queueForceAtPoint(bodyId, tractionForce3d, worldMountPoint);
            } else {
                if (normalForceNewtons > 0) {
                    physicsBody.applyForceAtPoint(suspensionForce, worldMountPoint);
                }
                physicsBody.applyForceAtPoint(tractionForce3d, worldMountPoint);
            }

            // Debug logging (only module 0)
            if (i == 0) {
                SimDebugLogger.logForce(String.format(
                        "Mod0: suspension=%.1fN traction=(%.1f,%.1f)N groundVel=(%.3f,%.3f)m/s heading=%.1f° threaded=%b",
                        normalForceNewtons,
                        tractionForce2d.x,
                        tractionForce2d.y,
                        groundVelocity2d.x,
                        groundVelocity2d.y,
                        Math.toDegrees(robotHeading.getRadians()),
                        isThreaded));
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
