package org.frc5687.robot.util;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.interpolation.TimeInterpolatableBuffer;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.Timer;
import java.util.HashMap;
import java.util.Map;
import java.util.Optional;
import org.photonvision.EstimatedRobotPose;

public class PoseEstimator {
    private static final double POSE_BUFFER_SECONDS = 1.5;

    private final OdometrySource _odometrySource;

    private final TimeInterpolatableBuffer<Pose2d> _poseBuffer =
            TimeInterpolatableBuffer.createBuffer(POSE_BUFFER_SECONDS);
    private Pose2d _odometryPose = new Pose2d();
    private Pose2d _estimatedPose = new Pose2d();

    private final Matrix<N3, N1> _qStdDevs =
            VecBuilder.fill(0.003 * 0.003, 0.003 * 0.003, 0.002 * 0.002);

    private final Map<Integer, Pose2d> _tagPoses = new HashMap<>();

    public PoseEstimator(OdometrySource odometrySource) {
        FieldConstants.aprilTagLayout
                .getTags()
                .forEach(tag -> _tagPoses.put(tag.ID, tag.pose.toPose2d()));

        _odometrySource = odometrySource;
    }

    public void resetPose(Pose2d pose) {
        _estimatedPose = pose;
        _odometryPose = pose;
        _poseBuffer.clear();
    }

    public void addVisionMeasurement(
            EstimatedRobotPose visionPose, Matrix<N3, N1> stdDevs, double timestamp) {
        processMeasurement(visionPose.estimatedPose.toPose2d(), stdDevs, timestamp);
    }

    public void updateOdometry() {
        double timestamp = Timer.getFPGATimestamp();
        Pose2d measuredOdometryPose = _odometrySource.getOdometryPose();
        Transform2d diff = new Transform2d(_odometryPose, measuredOdometryPose);
        _odometryPose = measuredOdometryPose;
        _poseBuffer.addSample(timestamp, measuredOdometryPose);
        _estimatedPose = _estimatedPose.plus(diff);
    }

    private void processMeasurement(
            Pose2d measurementPose, Matrix<N3, N1> stdDevs, double timestamp) {
        // Dont add stale measurements
        Optional<Pose2d> sample = _poseBuffer.getSample(timestamp); // interpolates
        if (sample.isEmpty()
                || _poseBuffer.getInternalBuffer().lastKey() - POSE_BUFFER_SECONDS > timestamp) {
            return;
        }

        // Odometry pose at measurement time
        Pose2d odometryAtTime = sample.get();

        // Transform from odometryAtTime to current odometry
        Transform2d odometryTransform = new Transform2d(odometryAtTime, _odometryPose);

        // Estimated pose at measurement time
        Pose2d estimateAtTime = _estimatedPose.plus(odometryTransform.inverse());

        Transform2d measurementTransform = new Transform2d(estimateAtTime, measurementPose); // diff

        // Calculate Kalman gain (6328)
        Matrix<N3, N3> kalmanGain = new Matrix<>(Nat.N3(), Nat.N3());
        for (int i = 0; i < 3; i++) {
            double q = _qStdDevs.get(i, 0);
            double r = stdDevs.get(i, 0) * stdDevs.get(i, 0);
            kalmanGain.set(i, i, q / (q + Math.sqrt(q * r)));
        }

        // Scaled correction
        Matrix<N3, N1> kTimesTransform =
                kalmanGain.times(
                        VecBuilder.fill(
                                measurementTransform.getX(),
                                measurementTransform.getY(),
                                measurementTransform.getRotation().getRadians()));

        Transform2d scaledTransform =
                new Transform2d(
                        kTimesTransform.get(0, 0),
                        kTimesTransform.get(1, 0),
                        Rotation2d.fromRadians(kTimesTransform.get(2, 0)));

        _estimatedPose = estimateAtTime.plus(scaledTransform).plus(odometryTransform);
    }

    public Pose2d getEstimatedPose() {
        return _estimatedPose;
    }
}
