package org.frc5687.robot;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.StateSpaceUtil;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.ExtendedKalmanFilter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.interpolation.TimeInterpolatableBuffer;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.Timer;
import java.util.Optional;
import java.util.TreeMap;
import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;
import org.littletonrobotics.junction.Logger;

public class RobotState {
    private static RobotState _instance;
    private final Lock _stateLock = new ReentrantLock();

    // Kalman Filter Configuration
    private static final Matrix<N3, N1> _stateStdDevs = VecBuilder.fill(0.1, 0.1, 0.1);
    private static final Matrix<N3, N1> _encoderStdDevs = VecBuilder.fill(0.1, 0.1, 0.05);
    private static final Matrix<N3, N1> _visualOdomStdDevs = VecBuilder.fill(0.05, 0.05, 0.03);
    private static final Matrix<N3, N1> _visionStdDevs = VecBuilder.fill(0.9, 0.9, 0.5);

    // Odometry and State Estimation
    private final SwerveDriveKinematics _kinematics;
    private final SwerveDriveOdometry _odometry;
    private final ExtendedKalmanFilter<N3, N3, N3> _ekf;
    private final TimeInterpolatableBuffer<Pose2d> _poseBuffer;
    private final TreeMap<Double, VisionUpdate> _visionUpdates;
    private Optional<Pose2d> _latestVisualOdomPose = Optional.empty();

    private ChassisSpeeds _measuredSpeeds = new ChassisSpeeds();
    private boolean _hasVisualOdomData = false;

    private static final double BUFFER_DURATION = 1.5; // seconds

    private RobotState(
            SwerveDriveKinematics kinematics,
            Rotation2d gyroAngle,
            SwerveModulePosition[] initialPositions,
            Pose2d initialPose) {
        _kinematics = kinematics;
        _odometry = new SwerveDriveOdometry(kinematics, gyroAngle, initialPositions, initialPose);

        _poseBuffer = TimeInterpolatableBuffer.createBuffer(BUFFER_DURATION);
        _visionUpdates = new TreeMap<>();

        // Initialize EKF
        _ekf =
                new ExtendedKalmanFilter<>(
                        Nat.N3(),
                        Nat.N3(),
                        Nat.N3(),
                        (x, u) -> u, // State transition is just the twist
                        (x, u) -> x, // Measurement model is direct state observation
                        _stateStdDevs,
                        _encoderStdDevs,
                        Constants.UPDATE_PERIOD);

        reset(Timer.getFPGATimestamp(), initialPose);
    }

    public static void createInstance(
            SwerveDriveKinematics kinematics,
            Rotation2d gyroAngle,
            SwerveModulePosition[] initialPositions,
            Pose2d initialPose) {
        _instance = new RobotState(kinematics, gyroAngle, initialPositions, initialPose);
    }

    public static RobotState getInstance() {
        if (_instance == null) {
            throw new RuntimeException("RobotState must be initialized first!");
        }
        return _instance;
    }

    public void reset(double timestamp, Pose2d pose) {
        _stateLock.lock();
        try {
            _poseBuffer.clear();
            _visionUpdates.clear();
            _ekf.setXhat(VecBuilder.fill(pose.getX(), pose.getY(), pose.getRotation().getRadians()));
            _measuredSpeeds = new ChassisSpeeds();
            _latestVisualOdomPose = Optional.empty();
            _hasVisualOdomData = false;

            Logger.recordOutput("RobotState/Reset", pose);
        } finally {
            _stateLock.unlock();
        }
    }

    /** Updates state estimate with new odometry and IMU data */
    public void updateOdometry(Rotation2d gyroAngle, SwerveModulePosition[] modulePositions) {
        _stateLock.lock();
        try {
            // Update odometry
            Pose2d odometryPose = _odometry.update(gyroAngle, modulePositions);
            _poseBuffer.addSample(Timer.getFPGATimestamp(), odometryPose);

            // Update EKF with odometry measurement
            _ekf.correct(
                    VecBuilder.fill(0, 0, 0),
                    VecBuilder.fill(
                            odometryPose.getX(), odometryPose.getY(), odometryPose.getRotation().getRadians()),
                    StateSpaceUtil.makeCovarianceMatrix(Nat.N3(), _encoderStdDevs));

            // Update measurement speeds
            _measuredSpeeds = _kinematics.toChassisSpeeds(getModuleStates(modulePositions));

            // Predict next state
            _ekf.predict(
                    VecBuilder.fill(
                            _measuredSpeeds.vxMetersPerSecond * Constants.UPDATE_PERIOD,
                            _measuredSpeeds.vyMetersPerSecond * Constants.UPDATE_PERIOD,
                            _measuredSpeeds.omegaRadiansPerSecond * Constants.UPDATE_PERIOD),
                    Constants.UPDATE_PERIOD);

            Logger.recordOutput("RobotState/OdometryPose", odometryPose);
        } finally {
            _stateLock.unlock();
        }
    }

    /** Adds a visual odometry measurement from Oculus */
    public void addVisualOdometryMeasurement(double timestamp, Pose2d visualOdomPose) {
        _stateLock.lock();
        try {
            // First visual odometry measurement
            if (_latestVisualOdomPose.isEmpty()) {
                _latestVisualOdomPose = Optional.of(visualOdomPose);
                _hasVisualOdomData = true;
                return;
            }

            // Calculate relative movement
            Transform2d relativePose = _latestVisualOdomPose.get().minus(visualOdomPose);

            // Update EKF with visual odometry measurement
            _ekf.correct(
                    VecBuilder.fill(
                            relativePose.getX(), relativePose.getY(), relativePose.getRotation().getRadians()),
                    VecBuilder.fill(
                            visualOdomPose.getX(),
                            visualOdomPose.getY(),
                            visualOdomPose.getRotation().getRadians()),
                    StateSpaceUtil.makeCovarianceMatrix(Nat.N3(), _visualOdomStdDevs));

            _latestVisualOdomPose = Optional.of(visualOdomPose);
            Logger.recordOutput("RobotState/VisualOdomPose", visualOdomPose);
        } finally {
            _stateLock.unlock();
        }
    }

    /** Adds a vision measurement (like AprilTags) */
    public void addVisionMeasurement(double timestamp, Pose2d visionPose, double confidence) {
        _stateLock.lock();
        try {
            // Get odometry pose at vision timestamp for comparison
            var odometryPose = _poseBuffer.getSample(timestamp);
            if (odometryPose.isEmpty()) return;

            // Basic acceptance filtering
            Pose2d currentPose = getEstimatedPose();
            Transform2d difference = currentPose.minus(visionPose);
            if (difference.getTranslation().getNorm() > 1.0
                    || Math.abs(difference.getRotation().getDegrees()) > 20) {
                return;
            }

            // Create vision measurement with confidence-adjusted standard deviations
            Matrix<N3, N1> stdDevs =
                    VecBuilder.fill(
                            _visionStdDevs.get(0, 0) / confidence,
                            _visionStdDevs.get(1, 0) / confidence,
                            _visionStdDevs.get(2, 0) / confidence);

            // Update EKF with vision measurement
            _ekf.correct(
                    VecBuilder.fill(0, 0, 0),
                    VecBuilder.fill(
                            visionPose.getX(), visionPose.getY(), visionPose.getRotation().getRadians()),
                    StateSpaceUtil.makeCovarianceMatrix(Nat.N3(), stdDevs));

            _visionUpdates.put(timestamp, new VisionUpdate(visionPose, odometryPose.get()));
            Logger.recordOutput("RobotState/VisionPose", visionPose);

            cleanVisionUpdates();
        } finally {
            _stateLock.unlock();
        }
    }

    private void cleanVisionUpdates() {
        if (_poseBuffer.getInternalBuffer().isEmpty()) return;
        double oldestTimestamp = _poseBuffer.getInternalBuffer().firstKey();
        _visionUpdates.headMap(oldestTimestamp).clear();
    }

    public Pose2d getEstimatedPose() {
        _stateLock.lock();
        try {
            var state = _ekf.getXhat();
            return new Pose2d(state.get(0, 0), state.get(1, 0), new Rotation2d(state.get(2, 0)));
        } finally {
            _stateLock.unlock();
        }
    }

    public ChassisSpeeds getMeasuredSpeeds() {
        return _measuredSpeeds;
    }

    public boolean hasVisualOdometryData() {
        return _hasVisualOdomData;
    }

    private static class VisionUpdate {
        public final Pose2d visionPose;
        public final Pose2d odometryPose;

        public VisionUpdate(Pose2d visionPose, Pose2d odometryPose) {
            this.visionPose = visionPose;
            this.odometryPose = odometryPose;
        }
    }

    // Helper method to get module states from positions
    private SwerveModuleState[] getModuleStates(SwerveModulePosition[] positions) {
        SwerveModuleState[] states = new SwerveModuleState[positions.length];
        for (int i = 0; i < positions.length; i++) {
            states[i] = new SwerveModuleState(0.0, positions[i].angle); // Velocity not needed
        }
        return states;
    }

    public Lock getWriteLock() {
        return _stateLock;
    }
}
