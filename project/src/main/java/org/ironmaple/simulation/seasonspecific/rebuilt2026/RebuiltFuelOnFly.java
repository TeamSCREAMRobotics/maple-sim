package org.ironmaple.simulation.seasonspecific.rebuilt2026;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import org.ironmaple.simulation.gamepieces.GamePieceProjectile;

public class RebuiltFuelOnFly extends GamePieceProjectile {

    public RebuiltFuelOnFly(
            Translation2d robotPosition,
            Translation2d shooterPositionOnRobot,
            ChassisSpeeds chassisSpeeds,
            Rotation2d shooterFacing,
            Distance initialHeight,
            LinearVelocity launchSpeed,
            Angle shooterAngle) {
        super(
                RebuiltFuelOnField.REBUILT_FUEL_INFO,
                robotPosition,
                shooterPositionOnRobot,
                chassisSpeeds,
                shooterFacing,
                initialHeight,
                launchSpeed,
                shooterAngle);

        super.enableBecomesGamePieceOnFieldAfterTouchGround();
        super.withTouchGroundHeight(0.16); // diameter + tolerance?
    }
}
