package org.frc5687.robot.util.vision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.util.Units;
import java.util.Optional;
import org.frc5687.robot.util.EpilogueLog;
import org.frc5687.robot.util.TunableDouble;
import org.frc5687.robot.util.vision.LimelightHelpers.RawDetection;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonTrackedTarget;

public class NeuralPipelineObservation implements EpilogueLog {

    private final int classId;
    private final double x;
    private final double y;

    private static final TunableDouble pitchFlipped =
            new TunableDouble("Neural", "pitchFlipped (-1 or 1)", -1);
    private static final TunableDouble yawFlipped =
            new TunableDouble("Neural", "yawFlipped (-1 or 1)", 1);

    private static final InterpolatingDoubleTreeMap CORAL_PITCH_TO_DIST =
            new InterpolatingDoubleTreeMap();

    private static final double MIN_PITCH = -9.4;
    private static final double MAX_PITCH = 12.53;

    static {
        CORAL_PITCH_TO_DIST.put(MIN_PITCH, Units.inchesToMeters(39));
        CORAL_PITCH_TO_DIST.put(-4.8, Units.inchesToMeters(46.5));
        CORAL_PITCH_TO_DIST.put(-1.0, Units.inchesToMeters(57));
        CORAL_PITCH_TO_DIST.put(4.2, Units.inchesToMeters(74));
        CORAL_PITCH_TO_DIST.put(7.75, Units.inchesToMeters(91));
        CORAL_PITCH_TO_DIST.put(MAX_PITCH, Units.inchesToMeters(154));
    }

    public NeuralPipelineObservation(int classId, double x, double y) {
        this.classId = classId;
        this.x = x;
        this.y = y;
    }

    public static NeuralPipelineObservation fromLimelight(
            RawDetection rawDetection, Transform3d robotToCamera) {
        if (robotToCamera.getRotation().getX() != 0.0)
            throw new IllegalArgumentException(
                    "robotToCamera can not have a roll because xavier did not want to implement it.");

        int classId = rawDetection.classId;
        if (classId != 0) {
            return null;
        }

        Transform2d robotToCamera2d =
                new Transform2d(
                        robotToCamera.getX(), robotToCamera.getY(), robotToCamera.getRotation().toRotation2d());

        double detectionWidth = rawDetection.corner2_X - rawDetection.corner0_X;
        double detectionHeight = rawDetection.corner2_Y - rawDetection.corner0_Y;
        double detectionSizePixels = Math.max(detectionWidth, detectionHeight);
        double radiansPerPixel = Units.degreesToRadians(81.984) / 640;
        double detectionSizeAngle = detectionSizePixels * radiansPerPixel;
        double distanceMeters = Units.inchesToMeters(17) / Math.tan(detectionSizeAngle);
        double cameraX = distanceMeters * Math.cos(Units.degreesToRadians(rawDetection.txnc));
        double cameraY = -distanceMeters * Math.sin(Units.degreesToRadians(rawDetection.txnc));

        Transform2d camToGamePiece = new Transform2d(cameraX, cameraY, new Rotation2d());
        Transform2d robotToGamePiece = robotToCamera2d.plus(camToGamePiece);

        // worldToRobot + robotToCam + camToGamePiece

        return new NeuralPipelineObservation(
                rawDetection.classId, robotToGamePiece.getX(), robotToGamePiece.getY());
    }

    public static Optional<NeuralPipelineObservation> fromPhotonVision(
            PhotonCamera cam, Transform3d robotToCam, PhotonTrackedTarget target, double timestamp) {
        if (target.objDetectId != 1) {
            return Optional.empty();
        }

        if (target.pitch < MIN_PITCH || target.pitch > MAX_PITCH) {
            return Optional.empty();
        }

        Rotation2d yaw =
                Rotation2d.fromDegrees(target.yaw).minus(robotToCam.getRotation().toRotation2d());
        double xDistance = CORAL_PITCH_TO_DIST.get(target.pitch);

        Pose2d robotToCameraRay = new Pose2d(robotToCam.getX(), robotToCam.getY(), yaw);
        double unitVectorX = yaw.getCos();
        double unitVectorY = yaw.getCos();

        // Pose2d robotToDetection = robotToCameraRay.plus(cameraRayToDetection);

        // return Optional.of(
        // new NeuralPipelineObservation(1, robotToDetection.getX(), robotToDetection.getY()));
        return Optional.of(new NeuralPipelineObservation(1, unitVectorX, unitVectorY));
    }

    @Override
    public String getLogBase() {
        return "NeuralPipelineObservation";
    }

    public int getClassId() {
        return classId;
    }

    public double getX() {
        return x;
    }

    public double getY() {
        return y;
    }

    public double getDistance() {
        return Math.hypot(x, y);
    }

    public double getAngle() {
        return Math.atan2(y, x);
    }
}
