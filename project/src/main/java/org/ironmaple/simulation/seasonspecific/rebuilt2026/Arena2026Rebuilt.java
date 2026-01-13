package org.ironmaple.simulation.seasonspecific.rebuilt2026;

import static org.ironmaple.utils.FieldMirroringUtils.*;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import java.util.List;
import org.ironmaple.simulation.SimulatedArena;

public class Arena2026Rebuilt extends SimulatedArena {
    public static final class RebuiltFieldObstacleMap extends FieldMap {

        public RebuiltFieldObstacleMap() {
            super();

            // blue wall
            super.addBorderLine(new Translation2d(0, 0), new Translation2d(0, FIELD_HEIGHT));

            // red wall
            super.addBorderLine(new Translation2d(FIELD_WIDTH, 0), new Translation2d(FIELD_WIDTH, FIELD_HEIGHT));

            // side walls
            super.addBorderLine(new Translation2d(0, 0), new Translation2d(FIELD_WIDTH, 0));
            super.addBorderLine(new Translation2d(0, FIELD_HEIGHT), new Translation2d(FIELD_WIDTH, FIELD_HEIGHT));

            var blueTrenchRightPos = new Translation2d(4.626, 1.431);
            var blueTrenchLeftPos = new Translation2d(4.626, FIELD_HEIGHT - blueTrenchRightPos.getY());
            var redTrenchLeftPos = flip(blueTrenchRightPos);
            var redTrenchRightPos = flip(blueTrenchLeftPos);
            var blueTowerPos = new Translation2d(0.4935, 3.711);
            var redTowerPos = flip(blueTowerPos);

            // blue hub
            super.addRectangularObstacle(1.19, 1.19, new Pose2d(blueHubPos, Rotation2d.kZero));

            // blue trench barrier right
            super.addRectangularObstacle(1.194, 0.305, new Pose2d(blueTrenchRightPos, Rotation2d.kZero));

            // blue trench barrier left
            super.addRectangularObstacle(1.19, 0.305, new Pose2d(blueTrenchLeftPos, Rotation2d.kZero));

            // blue tower
            super.addRectangularObstacle(1.146, 0.987, new Pose2d(blueTowerPos, Rotation2d.kZero));

            // red hub
            super.addRectangularObstacle(1.19, 1.19, new Pose2d(redHubPos, Rotation2d.kZero));

            // red trench barrier right
            super.addRectangularObstacle(1.19, 0.305, new Pose2d(redTrenchRightPos, Rotation2d.kZero));

            // red trench barrier left
            super.addRectangularObstacle(1.19, 0.305, new Pose2d(redTrenchLeftPos, Rotation2d.kZero));

            /// red tower
            super.addRectangularObstacle(1.146, 0.987, new Pose2d(redTowerPos, Rotation2d.kZero));
        }
    }

    /*
     * field element simulation inits
     */

    public Arena2026Rebuilt() {
        super(new RebuiltFieldObstacleMap());

        /* add field element simulations */
    }

    private Translation2d fuelZoneCenter = new Translation2d(FIELD_WIDTH / 2, FIELD_HEIGHT / 2);
    private int fuelCount = 408;
    private double fuelSeparationGap = 0.001; // 1mm

    public void setFuelCount(int count) {
        this.fuelCount = Math.max(360, Math.min(600, count));
    }

    public int getFuelCount() {
        return fuelCount;
    }

    @Override
    public void placeGamePiecesOnField() {
        double fuelDiameter = ((org.dyn4j.geometry.Circle) RebuiltFuelOnField.REBUILT_FUEL_INFO.shape()).getRadius()
                * 2;
        double fuelRadius = fuelDiameter / 2;
        double spacing = fuelDiameter + fuelSeparationGap;

        // Dimensions from graphic
        double boundingBoxWidth = edu.wpi.first.units.Units.Inches.of(206).in(edu.wpi.first.units.Units.Meters);
        double boundingBoxDepth = edu.wpi.first.units.Units.Inches.of(72).in(edu.wpi.first.units.Units.Meters);

        java.util.List<Translation2d> q1 = new java.util.ArrayList<>();
        java.util.List<Translation2d> q2 = new java.util.ArrayList<>();
        java.util.List<Translation2d> q3 = new java.util.ArrayList<>();
        java.util.List<Translation2d> q4 = new java.util.ArrayList<>();

        // Generate points for one quadrant (Q1) starting from the center outward
        // We want uniform spacing of 'fuelSeparationGap' between all pieces, including
        // across the axes.
        // So the first piece center should be at (radius + gap/2) from the axis.
        double startX = fuelRadius + fuelSeparationGap / 2.0;
        double startY = fuelRadius + fuelSeparationGap / 2.0;

        for (double x = startX; x + fuelRadius <= boundingBoxDepth / 2; x += spacing) {
            for (double y = startY; y + fuelRadius <= boundingBoxWidth / 2; y += spacing) {
                // Add mirrored positions to all quadrants
                q1.add(fuelZoneCenter.plus(new Translation2d(x, y))); // Q1
                q2.add(fuelZoneCenter.plus(new Translation2d(-x, y))); // Q2
                q3.add(fuelZoneCenter.plus(new Translation2d(-x, -y))); // Q3
                q4.add(fuelZoneCenter.plus(new Translation2d(x, -y))); // Q4
            }
        }

        // Round robin placement
        java.util.List<java.util.List<Translation2d>> quadrants = java.util.List.of(q1, q2, q3, q4);
        int added = 0;
        boolean piecesAvailable = true;

        // Sort lists to ensure deterministic deterministic filling order (e.g. from
        // center out)
        // Distance to center ascending
        java.util.Comparator<Translation2d> distComparator = java.util.Comparator
                .comparingDouble(p -> p.getDistance(fuelZoneCenter));
        for (java.util.List<Translation2d> q : quadrants) {
            q.sort(distComparator);
        }

        // Indices for each quadrant list
        int[] indices = new int[4];

        while (added < fuelCount && piecesAvailable) {
            piecesAvailable = false;
            for (int i = 0; i < 4; i++) {
                if (added >= fuelCount)
                    break;

                java.util.List<Translation2d> q = quadrants.get(i);
                if (indices[i] < q.size()) {
                    super.addGamePiece(new RebuiltFuelOnField(q.get(indices[i])));
                    indices[i]++;
                    added++;
                    piecesAvailable = true;
                }
            }
        }

        // Depot Placement
        // Origin: (0.071, 5.584)
        // 4 rows in +y, 6 cols in +x
        Translation2d depotOrigin = new Translation2d(0.071, 5.584);
        for (int i = 0; i < 4; i++) {
            for (int j = 0; j < 6; j++) {
                double x = depotOrigin.getX() + (i * spacing);
                double y = depotOrigin.getY() + (j * spacing);

                // Blue side
                Translation2d bluePos = new Translation2d(x, y);
                super.addGamePiece(new RebuiltFuelOnField(bluePos));

                // Red side (mirrored)
                Translation2d redPos = flip(bluePos);
                super.addGamePiece(new RebuiltFuelOnField(redPos));
            }
        }
    }

    @Override
    public synchronized List<Pose3d> getGamePiecesPosesByType(String type) {
        List<Pose3d> poses = super.getGamePiecesPosesByType(type);

        return poses;
    }

    @Override
    public synchronized void clearGamePieces() {
        super.clearGamePieces();
        /* clear simulations */
    }
}
