package org.frc5687.robot.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

public class ReefAlignmentHelpers {
    public static final TunableDouble LEFT_OFFSET =
            new TunableDouble("ReefAlignmentHelpers", "LEFT_OFFSET", 0.06);
    public static final TunableDouble RIGHT_OFFSET =
            new TunableDouble("ReefAlignmentHelpers", "RIGHT_OFFSET", -0.27);
    public static final double SAFE_APPROACH_DISTANCE = 0.5;

    public enum ReefSide {
        LEFT,
        RIGHT
    }

    public static int calculateBestFace(Pose2d robotPose) {
        Translation2d reefCenter = FieldConstants.Reef.center;
        Translation2d robotPosition = robotPose.getTranslation();

        double angleToRobot =
                Math.atan2(
                        robotPosition.getY() - reefCenter.getY(), robotPosition.getX() - reefCenter.getX());

        if (angleToRobot < 0) {
            angleToRobot += 2 * Math.PI;
        }

        double normalizedAngle = (3 * Math.PI - angleToRobot + Math.PI / 6) % (2 * Math.PI);

        int faceNumber = (int) (normalizedAngle / (Math.PI / 3)) + 1;

        if (faceNumber < 1) {
            faceNumber += 6;
        } else if (faceNumber > 6) {
            faceNumber = 1;
        }

        return faceNumber;
    }

    public static int getTagIdFromBestFace(Pose2d robotPose, boolean isRedAlliance) {
        int faceNumber = calculateBestFace(robotPose);
        int faceIndex = faceNumber - 1;
        return isRedAlliance
                ? FieldConstants.Reef.redAllianceTagIds[faceIndex]
                : FieldConstants.Reef.blueAllianceTagIds[faceIndex];
    }

    public static Pose2d getFaceAlignedPose(int faceIndex) {
        Pose2d facePose = FieldConstants.Reef.centerFaces[faceIndex];

        // Face rotation in field coordinates
        Rotation2d faceAngle = facePose.getRotation();

        // We should face opposite direction of the face angle
        Rotation2d robotRotation = faceAngle.plus(new Rotation2d(Math.PI));

        Translation2d facePosition = facePose.getTranslation();
        Translation2d outwardOffset =
                new Translation2d(
                        SAFE_APPROACH_DISTANCE * Math.cos(faceAngle.getRadians()),
                        SAFE_APPROACH_DISTANCE * Math.sin(faceAngle.getRadians()));

        return new Pose2d(facePosition.plus(outwardOffset), robotRotation);
    }

    public static Pose2d calculateTargetPose(int faceNumber, ReefSide side) {
        if (faceNumber < 1 || faceNumber > 6) {
            throw new IllegalArgumentException("Face number must be between 1 and 6");
        }

        int faceIndex = faceNumber - 1;

        Pose2d basePose = getFaceAlignedPose(faceIndex);

        double lateralOffset = (side == ReefSide.LEFT) ? LEFT_OFFSET.get() : RIGHT_OFFSET.get();

        double lateralAngle = basePose.getRotation().getRadians() + Math.PI / 2;
        Translation2d lateralMove =
                new Translation2d(
                        lateralOffset * Math.cos(lateralAngle), lateralOffset * Math.sin(lateralAngle));

        Translation2d finalPosition = basePose.getTranslation().plus(lateralMove);

        return new Pose2d(finalPosition, basePose.getRotation());
    }
}
