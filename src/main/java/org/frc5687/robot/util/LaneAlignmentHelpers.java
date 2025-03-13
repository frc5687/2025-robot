package org.frc5687.robot.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

public class LaneAlignmentHelpers {

    public static Pose2d calculateBestLanePose(Pose2d robotPose) {
        Translation2d robotPosition = robotPose.getTranslation();
        Translation2d farCage = FieldConstants.Barge.farCage;
        Translation2d middleCage = FieldConstants.Barge.middleCage;
        Translation2d closeCage = FieldConstants.Barge.closeCage;

        double distanceFromFarCage = Math.abs(robotPosition.getY() - farCage.getY());
        double distanceFromMiddleCage = Math.abs(robotPosition.getY() - middleCage.getY());
        double distanceFromCloseCage = Math.abs(robotPosition.getY() - closeCage.getY());

        double targetY = robotPose.getY();

        if (distanceFromCloseCage < distanceFromMiddleCage
                && distanceFromCloseCage < distanceFromFarCage) {
            targetY = closeCage.getY();
        }

        if (distanceFromMiddleCage < distanceFromCloseCage
                && distanceFromMiddleCage < distanceFromFarCage) {
            targetY = middleCage.getY();
        }

        if (distanceFromFarCage < distanceFromMiddleCage
                && distanceFromFarCage < distanceFromCloseCage) {
            targetY = farCage.getY();
        }

        Translation2d targetTranslation = new Translation2d(robotPosition.getX(), targetY + .03);

        return new Pose2d(targetTranslation, Rotation2d.fromDegrees(180));
    }
}
