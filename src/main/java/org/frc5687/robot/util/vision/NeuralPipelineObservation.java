package org.frc5687.robot.util.vision;

import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.util.Units;
import org.frc5687.robot.util.EpilogueLog;
import org.frc5687.robot.util.TunableDouble;
import org.frc5687.robot.util.vision.LimelightHelpers.RawDetection;
import org.photonvision.targeting.PhotonTrackedTarget;

public class NeuralPipelineObservation implements EpilogueLog {

    private final int classId;
    private final double x;
    private final double y;

    private static final TunableDouble pitchFlipped =
            new TunableDouble("Neural", "pitchFlipped (-1 or 1)", -1);
    private static final TunableDouble yawFlipped =
            new TunableDouble("Neural", "yawFlipped (-1 or 1)", 1);

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

        double detectionWidth = rawDetection.corner2_X - rawDetection.corner0_X;
        double detectionHeight = rawDetection.corner2_Y - rawDetection.corner0_Y;
        double detectionSizePixels = Math.max(detectionWidth, detectionHeight);
        double radiansPerPixel = Units.degreesToRadians(81.984) / 640;
        double detectionSizeAngle = detectionSizePixels * radiansPerPixel;
        double distanceMeters = Units.inchesToMeters(17) / Math.tan(detectionSizeAngle);
        double x =
                robotToCamera.getX() + distanceMeters * Math.cos(Units.degreesToRadians(rawDetection.txnc));
        double y =
                robotToCamera.getY() + distanceMeters * Math.sin(Units.degreesToRadians(rawDetection.txnc));

        return new NeuralPipelineObservation(rawDetection.classId, x, y);
    }

    public static AprilTagObservation fromPhotonVision(PhotonTrackedTarget target, double timestamp) {
        return null; // TODO
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
