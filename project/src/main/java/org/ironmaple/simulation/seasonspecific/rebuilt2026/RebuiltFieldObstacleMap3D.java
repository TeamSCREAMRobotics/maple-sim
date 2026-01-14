package org.ironmaple.simulation.seasonspecific.rebuilt2026;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import org.ironmaple.simulation.SimulatedArena3D;
import org.ironmaple.utils.FieldMirroringUtils;

public class RebuiltFieldObstacleMap3D extends SimulatedArena3D.FieldMap3D {

    private static final double FIELD_WIDTH = 17.548;
    private static final double FIELD_HEIGHT = 8.052;

    public RebuiltFieldObstacleMap3D() {
        super();

        // Field Perimeter Walls (assume 50cm height)
        double wallHeight = 0.5;
        double wallThickness = 0.1;

        // Floor (Ground)
        // Create a large box for the floor, positioned so its top surface is at Z=0
        double floorThickness = 1.0;
        this.addBox(
                new Translation3d(FIELD_WIDTH * 1.5 / 2, FIELD_HEIGHT * 1.5 / 2, floorThickness / 2),
                new Pose3d(FIELD_WIDTH / 2, FIELD_HEIGHT / 2, -floorThickness / 2, new Rotation3d()));

        // Top Wall
        this.addBorderLine(
                new Translation3d(FIELD_WIDTH / 2, FIELD_HEIGHT + wallThickness / 2, wallHeight / 2),
                new Translation3d(FIELD_WIDTH, wallThickness, wallHeight));

        // Bottom Wall (0 y is bottom?) No, 0,0 is usually bottom left.
        // 2D map: addBorderLine(new Translation2d(0, 0), new Translation2d(FIELD_WIDTH,
        // 0));
        this.addBorderLine(
                new Translation3d(FIELD_WIDTH / 2, -wallThickness / 2, wallHeight / 2),
                new Translation3d(FIELD_WIDTH, wallThickness, wallHeight));

        // Left Wall
        this.addBorderLine(
                new Translation3d(-wallThickness / 2, FIELD_HEIGHT / 2, wallHeight / 2),
                new Translation3d(wallThickness, FIELD_HEIGHT, wallHeight));

        // Right Wall
        this.addBorderLine(
                new Translation3d(FIELD_WIDTH + wallThickness / 2, FIELD_HEIGHT / 2, wallHeight / 2),
                new Translation3d(wallThickness, FIELD_HEIGHT, wallHeight));

        // --- Obstacles from RebuiltFieldObstacleMap ---

        // Blue Trench Right
        var blueTrenchRightPos = new Translation2d(4.626, 1.431);
        addBox(
                new Translation3d(1.194 / 2, 0.305 / 2, 0.5),
                new Pose3d(blueTrenchRightPos.getX(), blueTrenchRightPos.getY(), 0.5, new Rotation3d()));

        // Blue Trench Left
        var blueTrenchLeftPos = new Translation2d(4.626, FIELD_HEIGHT - 1.431); // 1.431 is Y from 2D map
        addBox(
                new Translation3d(1.19 / 2, 0.305 / 2, 0.5),
                new Pose3d(blueTrenchLeftPos.getX(), blueTrenchLeftPos.getY(), 0.5, new Rotation3d()));

        // Red Trench Left (Mirror of Blue Right)
        Translation2d redTrenchLeftPos = FieldMirroringUtils.flip(blueTrenchRightPos);
        addBox(
                new Translation3d(1.194 / 2, 0.305 / 2, 0.5),
                new Pose3d(redTrenchLeftPos.getX(), redTrenchLeftPos.getY(), 0.5, new Rotation3d()));

        // Red Trench Right (Mirror of Blue Left)
        Translation2d redTrenchRightPos = FieldMirroringUtils.flip(blueTrenchLeftPos);
        addBox(
                new Translation3d(1.19 / 2, 0.305 / 2, 0.5),
                new Pose3d(redTrenchRightPos.getX(), redTrenchRightPos.getY(), 0.5, new Rotation3d()));

        // Blue Hub
        Translation2d blueHubPos = RebuiltHub.BLUE_HUB_POS.toTranslation2d();
        addObstacle("meshes/hub.obj", new Pose3d(blueHubPos.getX(), blueHubPos.getY(), 0, new Rotation3d()));

        // Red Hub
        Translation2d redHubPos = RebuiltHub.RED_HUB_POS.toTranslation2d();
        addObstacle("meshes/hub.obj", new Pose3d(redHubPos.getX(), redHubPos.getY(), 0, new Rotation3d(0, 0, Math.PI)));

        // Blue Tower Poles
        // 2D: size 2" x 47", pos (42", 159")
        double polesWidth = Inches.of(2).in(Meters);
        double polesHeight = Inches.of(47).in(Meters); // Length in 2D
        double poleZHeight = 2.0; // Assume tall poles
        Translation2d bluePolePos =
                new Translation2d(Inches.of(42).in(Meters), Inches.of(159).in(Meters));
        addBox(
                new Translation3d(polesWidth / 2, polesHeight / 2, poleZHeight / 2),
                new Pose3d(bluePolePos.getX(), bluePolePos.getY(), poleZHeight / 2, new Rotation3d()));

        // Red Tower Poles
        Translation2d redPolePos =
                new Translation2d(Inches.of(651 - 42).in(Meters), Inches.of(170).in(Meters));
        addBox(
                new Translation3d(polesWidth / 2, polesHeight / 2, poleZHeight / 2),
                new Pose3d(redPolePos.getX(), redPolePos.getY(), poleZHeight / 2, new Rotation3d()));

        // --- Custom V-HACD Meshes ---
        // Placeholders - User to provide actual paths and poses
        // To use: ensure meshes are in src/main/resources/meshes/
        // Pose3d corresponds to the (0,0,0) origin of the OBJ file.

        /*
         * Example:
         * addObstacle("meshes/reef.obj", new Pose3d(0, 0, 0, new Rotation3d()));
         * addObstacle("meshes/coral_station.obj", new Pose3d(0, 0, 0, new
         * Rotation3d()));
         */
    }

    public void addObstacle(String meshResourcePath, Pose3d pose) {
        getObstacles().add(new Obstacle(meshResourcePath, pose));
    }

    private void addBorderLine(Translation3d center, Translation3d size) {
        addBox(
                new Translation3d(size.getX() / 2, size.getY() / 2, size.getZ() / 2),
                new Pose3d(center, new Rotation3d()));
    }
}
