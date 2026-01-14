package org.ironmaple.simulation;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import java.util.ArrayDeque;
import java.util.List;
import java.util.Queue;
import java.util.function.Predicate;
import org.ironmaple.simulation.drivesims.AbstractDriveTrainSimulation3D;
import org.ironmaple.simulation.gamepieces.GamePieceOnFieldSimulation3D;
import org.ironmaple.simulation.physics.PhysicsBody;
import org.ironmaple.simulation.physics.PhysicsEngine;
import org.ironmaple.simulation.physics.PhysicsShape;

/**
 *
 *
 * <h1>Simulates an Intake Mechanism in 3D.</h1>
 *
 * <p>This class simulates a 3D intake attached to a 3D drivetrain. It detects game pieces within its volume and
 * "collects" them if enabled.
 */
public class IntakeSimulation3D implements SimulatedArena3D.Simulatable {
    private final String targetedGamePieceType;
    private final AbstractDriveTrainSimulation3D drivetrain;
    private final PhysicsShape intakeShape;
    private final Transform3d intakeOffset;
    private final int capacity;
    private int gamePiecesInIntakeCount;
    private boolean intakeRunning;
    private final Queue<GamePieceOnFieldSimulation3D> gamePiecesToRemove;
    private Predicate<GamePieceOnFieldSimulation3D> customIntakeCondition = gp -> true;

    private final PhysicsEngine physicsEngine;

    /**
     *
     *
     * <h2>Creates a 3D Intake Simulation.</h2>
     *
     * @param targetedGamePieceType the type of game piece to collect
     * @param drivetrain the drivetrain this intake is attached to
     * @param intakeShape the 3D shape of the intake volume
     * @param intakeOffset the offset of the intake center from the robot center
     * @param capacity the maximum number of game pieces
     * @param arena the simulation arena
     */
    public IntakeSimulation3D(
            String targetedGamePieceType,
            AbstractDriveTrainSimulation3D drivetrain,
            PhysicsShape intakeShape,
            Transform3d intakeOffset,
            int capacity,
            SimulatedArena3D arena) {
        this.targetedGamePieceType = targetedGamePieceType;
        this.drivetrain = drivetrain;
        this.intakeShape = intakeShape;
        this.intakeOffset = intakeOffset;
        this.capacity = capacity;
        this.gamePiecesInIntakeCount = 0;
        this.gamePiecesToRemove = new ArrayDeque<>();
        this.physicsEngine = arena.getPhysicsEngine();

        arena.addCustomSimulation(this);
    }

    /**
     *
     *
     * <h2>Turns the Intake On.</h2>
     */
    public void startIntake() {
        this.intakeRunning = true;
    }

    /**
     *
     *
     * <h2>Turns the Intake Off.</h2>
     */
    public void stopIntake() {
        this.intakeRunning = false;
    }

    @Override
    public void simulationSubTick(int subTickNum) {
        if (!intakeRunning || gamePiecesInIntakeCount >= capacity) return;

        // Calculate current intake pose
        Pose3d robotPose = drivetrain.getSimulatedDriveTrainPose3d();
        Pose3d intakePose = robotPose.plus(intakeOffset);

        // Check for overlaps
        List<PhysicsBody> overlaps = physicsEngine.getOverlappingBodies(intakeShape, intakePose);

        for (PhysicsBody body : overlaps) {
            Object userData = body.getUserData();
            if (userData instanceof GamePieceOnFieldSimulation3D gp) {
                if (gp.getType().equals(targetedGamePieceType) && customIntakeCondition.test(gp)) {
                    if (!gamePiecesToRemove.contains(gp)) {
                        gamePiecesToRemove.add(gp);
                        gamePiecesInIntakeCount++;
                        if (gamePiecesInIntakeCount >= capacity) break;
                    }
                }
            }
        }
    }

    /**
     *
     *
     * <h2>Removes obtained game pieces.</h2>
     *
     * <p>Called by arena or manually.
     */
    public void removeObtainedGamePieces(SimulatedArena3D arena) {
        while (!gamePiecesToRemove.isEmpty()) {
            GamePieceOnFieldSimulation3D gp = gamePiecesToRemove.poll();
            arena.getPhysicsEngine().removeBody(gp.getPhysicsBody());
            // Add logic to move to internal storage or notify manager
        }
    }
}
