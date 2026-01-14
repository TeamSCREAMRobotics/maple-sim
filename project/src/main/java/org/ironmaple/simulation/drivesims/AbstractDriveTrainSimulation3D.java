package org.ironmaple.simulation.drivesims;

import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import org.ironmaple.simulation.SimulatedArena3D;
import org.ironmaple.simulation.drivesims.configs.DriveTrainSimulationConfig;
import org.ironmaple.simulation.physics.PhysicsBody;
import org.ironmaple.simulation.physics.PhysicsEngine;
import org.ironmaple.simulation.physics.PhysicsShape;

/**
 *
 *
 * <h1>Represents an Abstract 3D Drivetrain Simulation.</h1>
 *
 * <h3>Simulates the Mass, Collision Space, and Physics of the Drivetrain in 3D.</h3>
 *
 * <p>This class models the physical properties of a drivetrain in a full 3D physics environment, including:
 *
 * <ul>
 *   <li>Mass and inertia tensor
 *   <li>3D collision box
 *   <li>Full 3D forces and torques
 *   <li>Raycast-based ground contact detection
 * </ul>
 *
 * <p>Unlike the 2D {@link AbstractDriveTrainSimulation}, this class does not extend dyn4j Body. Instead, it wraps a
 * {@link PhysicsBody} from the Bullet physics engine.
 */
public abstract class AbstractDriveTrainSimulation3D implements SimulatedArena3D.Simulatable {
    public static final double
            BUMPER_COEFFICIENT_OF_FRICTION = 0.65, // https://en.wikipedia.org/wiki/Friction#Coefficient_of_friction
            BUMPER_COEFFICIENT_OF_RESTITUTION = 0.08; // https://simple.wikipedia.org/wiki/Coefficient_of_restitution

    public final DriveTrainSimulationConfig config;
    protected PhysicsBody physicsBody;
    protected PhysicsEngine physicsEngine;
    protected SimulatedArena3D arena;

    /**
     * Height of the robot chassis BOTTOM above ground at spawn (meters). This should be approximately the suspension
     * equilibrium height = WHEEL_RADIUS + SUSPENSION_REST_LENGTH ≈ 0.08m. The robot will settle to the correct height
     * from gravity.
     */
    protected double chassisHeightMeters = 0.08;

    /** Height of the robot chassis body/collision box (meters). */
    protected double chassisBodyHeight = 0.1;

    /**
     *
     *
     * <h2>Creates a 3D Simulation of a Drivetrain.</h2>
     *
     * <p>Sets up the collision space and mass of the chassis in the 3D physics world.
     *
     * @param config the drivetrain configuration
     * @param initialPoseOnField the initial pose (X, Y, heading) - Z is calculated from chassis height
     */
    protected AbstractDriveTrainSimulation3D(DriveTrainSimulationConfig config, Pose2d initialPoseOnField) {
        this.config = config;
    }

    /**
     *
     *
     * <h2>Registers This Drivetrain with a 3D Arena.</h2>
     *
     * <p>Must be called after construction to add the drivetrain to the physics world.
     *
     * @param arena the 3D simulation arena
     * @param initialPose the initial 2D pose (converted to 3D with ground height)
     */
    public void registerWithArena(SimulatedArena3D arena, Pose2d initialPose) {
        this.arena = arena;
        this.physicsEngine = arena.getPhysicsEngine();

        // Create collision shape (box)
        double lengthX = config.bumperLengthX.in(Meters);
        double widthY = config.bumperWidthY.in(Meters);
        // Robot height in meters is now using class field chassisBodyHeight

        Translation3d halfExtents = new Translation3d(lengthX / 2, widthY / 2, chassisBodyHeight / 2);
        PhysicsShape shape = physicsEngine.createBoxShape(halfExtents);

        // Create initial 3D pose
        Pose3d pose3d = new Pose3d(
                new Translation3d(initialPose.getX(), initialPose.getY(), chassisHeightMeters + chassisBodyHeight / 2),
                new Rotation3d(0, 0, initialPose.getRotation().getRadians()));

        // Create the dynamic body
        double massKg = config.robotMass.in(edu.wpi.first.units.Units.Kilograms);
        this.physicsBody = physicsEngine.createDynamicBody(shape, massKg, pose3d);

        // Apply light damping to prevent drift (0.1 is standard value)
        // Bullet damping range is [0, 1] where 1 stops object immediately.
        physicsBody.setDamping(0.1, 0.1);

        // Register as a custom simulation for sub-tick updates
        arena.addCustomSimulation(this);
    }

    /**
     *
     *
     * <h2>Sets the Robot's Current Pose in the Simulation World.</h2>
     *
     * @param robotPose the desired robot pose (2D)
     */
    public void setSimulationWorldPose(Pose2d robotPose) {
        if (physicsBody == null) return;

        Pose3d currentPose = physicsBody.getPose3d();
        Pose3d newPose = new Pose3d(
                new Translation3d(robotPose.getX(), robotPose.getY(), currentPose.getZ()),
                new Rotation3d(0, 0, robotPose.getRotation().getRadians()));
        physicsBody.setPose3d(newPose);
        physicsBody.setLinearVelocityMPS(new Translation3d());
        physicsBody.setAngularVelocityRadPerSec(new Translation3d());
    }

    /**
     *
     *
     * <h2>Sets the Robot's Speeds.</h2>
     *
     * @param givenSpeeds the desired chassis speeds
     */
    public void setRobotSpeeds(ChassisSpeeds givenSpeeds) {
        if (physicsBody == null) return;

        // Convert robot-relative speeds to field-relative for the physics body
        Pose3d pose = physicsBody.getPose3d();
        double heading = pose.getRotation().getZ();

        double vx =
                givenSpeeds.vxMetersPerSecond * Math.cos(heading) - givenSpeeds.vyMetersPerSecond * Math.sin(heading);
        double vy =
                givenSpeeds.vxMetersPerSecond * Math.sin(heading) + givenSpeeds.vyMetersPerSecond * Math.cos(heading);

        physicsBody.setLinearVelocityMPS(new Translation3d(vx, vy, 0));
        physicsBody.setAngularVelocityRadPerSec(new Translation3d(0, 0, givenSpeeds.omegaRadiansPerSecond));
    }

    /**
     *
     *
     * <h2>Gets the Actual 3D Pose of the Drivetrain.</h2>
     *
     * <p>Returns the physics body center of mass pose. For visualization where Z=0 is ground, use
     * {@link #getSimulatedDriveTrainPose3dGroundRelative()} instead.
     *
     * @return the 3D pose in the simulation (center of mass)
     */
    public Pose3d getSimulatedDriveTrainPose3d() {
        if (physicsBody == null) return new Pose3d();
        return physicsBody.getPose3d();
    }

    /**
     *
     *
     * <h2>Gets the 3D Pose Adjusted for Ground-Level Visualization.</h2>
     *
     * <p>For AdvantageScope and similar tools where robot models have their origin at the bottom (wheel contact point),
     * this method returns a pose with Z adjusted so Z=0 represents the bottom of the robot at rest on flat ground.
     *
     * @return the 3D pose adjusted for ground-relative visualization
     */
    public Pose3d getSimulatedDriveTrainPose3dGroundRelative() {
        if (physicsBody == null) return new Pose3d();
        Pose3d physicsPose = physicsBody.getPose3d();
        // Subtract the chassis center height above ground
        // At rest: Z_center = chassisBodyHeight/2 + (wheel radius + suspension)
        // For visualization, we want the bottom of the robot at Z=0
        double zOffset = chassisBodyHeight / 2;
        return new Pose3d(
                physicsPose.getX(), physicsPose.getY(), physicsPose.getZ() - zOffset, physicsPose.getRotation());
    }

    /**
     *
     *
     * <h2>Gets the Actual 2D Pose of the Drivetrain.</h2>
     *
     * <p>Projects the 3D pose to 2D (ignoring Z and pitch/roll).
     *
     * @return the 2D pose
     */
    public Pose2d getSimulatedDriveTrainPose() {
        Pose3d pose3d = getSimulatedDriveTrainPose3d();
        return new Pose2d(
                pose3d.getX(),
                pose3d.getY(),
                new Rotation2d(pose3d.getRotation().getZ()));
    }

    /**
     *
     *
     * <h2>Gets the Actual Robot-Relative Chassis Speeds.</h2>
     *
     * @return the chassis speeds, robot-relative
     */
    public ChassisSpeeds getDriveTrainSimulatedChassisSpeedsRobotRelative() {
        ChassisSpeeds fieldRelative = getDriveTrainSimulatedChassisSpeedsFieldRelative();
        return ChassisSpeeds.fromFieldRelativeSpeeds(
                fieldRelative, getSimulatedDriveTrainPose().getRotation());
    }

    /**
     *
     *
     * <h2>Gets the Actual Field-Relative Chassis Speeds.</h2>
     *
     * @return the chassis speeds, field-relative
     */
    public ChassisSpeeds getDriveTrainSimulatedChassisSpeedsFieldRelative() {
        if (physicsBody == null) return new ChassisSpeeds();

        Translation3d linearVel = physicsBody.getLinearVelocityMPS();
        Translation3d angularVel = physicsBody.getAngularVelocityRadPerSec();

        return new ChassisSpeeds(linearVel.getX(), linearVel.getY(), angularVel.getZ());
    }

    /**
     *
     *
     * <h2>Gets the Underlying Physics Body.</h2>
     *
     * @return the physics body, or null if not registered
     */
    public PhysicsBody getPhysicsBody() {
        return physicsBody;
    }

    /**
     *
     *
     * <h2>Applies a Force at a Point on the Robot.</h2>
     *
     * @param force the force vector in world coordinates
     * @param pointWorld the application point in world coordinates
     */
    protected void applyForceAtPoint(Translation3d force, Translation3d pointWorld) {
        if (physicsBody != null) {
            physicsBody.applyForceAtPoint(force, pointWorld);
        }
    }

    /**
     *
     *
     * <h2>Applies a Central Force to the Robot.</h2>
     *
     * @param force the force vector in world coordinates
     */
    protected void applyForce(Translation3d force) {
        if (physicsBody != null) {
            physicsBody.applyForce(force);
        }
    }

    /**
     *
     *
     * <h2>Applies a Torque to the Robot.</h2>
     *
     * @param torque the torque vector
     */
    protected void applyTorque(Translation3d torque) {
        if (physicsBody != null) {
            physicsBody.applyTorque(torque);
        }
    }

    /**
     *
     *
     * <h2>Gets the Linear Velocity at a Specific Point.</h2>
     *
     * @param pointWorld the point in world coordinates
     * @return the velocity at that point
     */
    protected Translation3d getLinearVelocityAtPoint(Translation3d pointWorld) {
        if (physicsBody == null) return new Translation3d();
        return physicsBody.getLinearVelocityAtPointMPS(pointWorld);
    }

    /**
     *
     *
     * <h2>Abstract Simulation Sub-Tick Method.</h2>
     *
     * <p>Implemented by subclasses to apply propelling forces.
     */
    @Override
    public abstract void simulationSubTick(int subTickNum);
}
