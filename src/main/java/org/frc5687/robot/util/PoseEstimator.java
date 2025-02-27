package org.frc5687.robot.util;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.interpolation.TimeInterpolatableBuffer;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.Timer;
import java.util.HashMap;
import java.util.Map;
import java.util.Optional;
import org.frc5687.robot.subsystems.vision.RobotPoseEstimate;

public class PoseEstimator implements EpilogueLog {
    private static final double POSE_BUFFER_SECONDS = 1.5;

    private final OdometrySource _odometrySource;
    private final VisionSTDFilter _visionStdFilter;

    private final TimeInterpolatableBuffer<Pose2d> _poseBuffer =
            TimeInterpolatableBuffer.createBuffer(POSE_BUFFER_SECONDS);
    private Pose2d _odometryPose = new Pose2d();
    private Pose2d _estimatedPose = new Pose2d();
    private Pose2d _lastPose = new Pose2d();
    private double _lastTimestamp = Timer.getFPGATimestamp();
    private double _currentVelocity = 0.0;

    // We probably want to seperate this for the different estimators
    private final Matrix<N3, N1> _qStdDevs =
            VecBuilder.fill(0.003 * 0.003, 0.003 * 0.003, 0.002 * 0.002);

    private final Map<Integer, Pose2d> _tagPoses = new HashMap<>();

    public PoseEstimator(OdometrySource odometrySource) {
        FieldConstants.aprilTagLayout
                .getTags()
                .forEach(tag -> _tagPoses.put(tag.ID, tag.pose.toPose2d()));

        _odometrySource = odometrySource;
        _visionStdFilter = new VisionSTDFilter();
        _lastPose = _odometrySource.getOdometryPose();
    }

    public void resetPose(Pose2d pose) {
        _odometrySource.resetPose(pose);
        _estimatedPose = pose;
        _odometryPose = pose;
        _lastPose = pose;
        _currentVelocity = 0.0;
        _poseBuffer.clear();
    }

    /**
     * Add a vision measurement from our unified RobotPoseEstimate format
     *
     * @param poseEstimate The robot pose estimate from vision
     */
    public void addVisionMeasurement(RobotPoseEstimate poseEstimate) {
        // Calculate standard deviations based on vision data
        Matrix<N3, N1> stdDevs = calculateVisionStdDevs(poseEstimate);

        // Log vision pose
        log("Vision Pose", poseEstimate.getPose3d(), Pose3d.struct);

        // Process measurement with timestamp
        processMeasurement(poseEstimate.pose, stdDevs, poseEstimate.timestampSeconds);
    }

    /**
     * Calculate vision standard deviations based on our RobotPoseEstimate Larger deviation = less
     * trust in the measurement
     */
    private Matrix<N3, N1> calculateVisionStdDevs(RobotPoseEstimate poseEstimate) {
        // Base std devs - higher values = less weight given to vision
        double xyStdDev = 0.1; // meters
        double thetaStdDev = 0.05; // radians

        // Decrease weight for single-tag measurements
        if (poseEstimate.tagCount <= 1) {
            xyStdDev *= 2.0;
            thetaStdDev *= 1.5;
        }

        // Decrease weight for distant tags
        if (poseEstimate.avgTagDist > 3.0) {
            xyStdDev *= 1.0 + (poseEstimate.avgTagDist - 3.0) * 0.5;
            thetaStdDev *= 1.0 + (poseEstimate.avgTagDist - 3.0) * 0.3;
        }

        // Decrease weight for low confidence
        if (poseEstimate.confidence < 0.9) {
            double confidenceFactor = 1.0 + (0.9 - poseEstimate.confidence) * 4.0;
            xyStdDev *= confidenceFactor;
            thetaStdDev *= confidenceFactor;
        }

        // Apply minimums
        xyStdDev = Math.max(0.05, xyStdDev);
        thetaStdDev = Math.max(0.02, thetaStdDev);

        // Square values for matrix (variance)
        return VecBuilder.fill(xyStdDev * xyStdDev, xyStdDev * xyStdDev, thetaStdDev * thetaStdDev);
    }

    public void updateOdometry() {
        double currentTimestamp = Timer.getFPGATimestamp();
        Pose2d measuredOdometryPose = _odometrySource.getOdometryPose();

        double dt = currentTimestamp - _lastTimestamp;
        if (dt > 0) {
            Transform2d poseDiff = new Transform2d(_lastPose, measuredOdometryPose);
            double distance =
                    Math.sqrt(poseDiff.getX() * poseDiff.getX() + poseDiff.getY() * poseDiff.getY());
            _currentVelocity = distance / dt;
        }

        Transform2d diff = new Transform2d(_odometryPose, measuredOdometryPose);
        _odometryPose = measuredOdometryPose;
        _poseBuffer.addSample(currentTimestamp, measuredOdometryPose);
        _estimatedPose = _estimatedPose.plus(diff);

        _lastPose = measuredOdometryPose;
        _lastTimestamp = currentTimestamp;
    }

    private void processMeasurement(
            Pose2d measurementPose, Matrix<N3, N1> stdDevs, double timestamp) {
        // Don't add stale measurements
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

        // Calculate Kalman gain (6328) and now
        // https://github.com/wpilibsuite/allwpilib/blob/main/wpimath/algorithms.md#closed-form-kalman-gain-for-continuous-kalman-filter-with-a--0-and-c--i
        Matrix<N3, N3> kalmanGain = new Matrix<>(Nat.N3(), Nat.N3());
        for (int i = 0; i < 3; i++) {
            double q = _qStdDevs.get(i, 0);
            double r = stdDevs.get(i, 0);

            // edge cases from derivation
            if (q == 0 && r != 0) {
                kalmanGain.set(i, i, 0.0);
            } else if (q != 0 && r == 0) {
                kalmanGain.set(i, i, 1.0);
            } else {
                kalmanGain.set(i, i, q / (q + Math.sqrt(q * r)));
            }
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

    public double getCurrentVelocity() {
        return _currentVelocity;
    }

    @Override
    public String getLogBase() {
        return "PoseEstimator";
    }
}
