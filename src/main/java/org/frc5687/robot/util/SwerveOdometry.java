package org.frc5687.robot.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;
import org.frc5687.robot.subsystems.drive.DriveInputs;

// Ideas from 1678 WheelTracker and 1690s presentation
public class SwerveOdometry {
    private final ModuleState[] _moduleStates;
    private Pose2d _robotPose = new Pose2d();
    private Translation2d _robotVelocity = new Translation2d();
    private double _lastTimestamp = 0.0;

    private static final double VELOCITY_SAMPLE_TIME = 0.02;
    private static final double MODULE_TRUST_THRESHOLD = 0.01;
    private static final double SKID_RATIO_THRESHOLD = 1.2;

    private class ModuleState {
        private double _previousDistance = 0;
        private Translation2d _position;
        private final Translation2d _offset;
        private Pose2d _estimatedRobotPose = new Pose2d();
        private double _trustFactor = 1.0;
        private double _lastVelocity = 0.0;

        public ModuleState(Translation2d moduleOffset) {
            _offset = moduleOffset;
            _position = moduleOffset;
        }

        public void updateState(
                double newDistance, Rotation2d angle, Rotation2d robotYaw, double currentTime) {
            double dt = currentTime - _lastTimestamp;
            double deltaDistance = newDistance - _previousDistance;
            Rotation2d globalAngle = angle.rotateBy(robotYaw);

            Translation2d deltaPosition =
                    new Translation2d(
                            globalAngle.getCos() * deltaDistance, globalAngle.getSin() * deltaDistance);

            _position = _position.plus(deltaPosition);
            _estimatedRobotPose =
                    new Pose2d(_position, robotYaw)
                            .transformBy(new Transform2d(_offset.unaryMinus(), new Rotation2d()));

            // TODO: Epsilon
            double velocity = dt > 1e-5 ? deltaDistance / dt : 0.0;
            updateTrustFactor(velocity);

            _previousDistance = newDistance;
            _lastVelocity = velocity;
            _lastTimestamp = currentTime;
        }

        private void updateTrustFactor(double currentVelocity) {
            double velocityDelta = Math.abs(currentVelocity - _lastVelocity);
            if (velocityDelta > MODULE_TRUST_THRESHOLD) {
                _trustFactor = Math.max(_trustFactor * 0.8, 0.1);
            } else {
                _trustFactor = Math.min(_trustFactor * 1.2, 1.0);
            }
        }
    }

    public SwerveOdometry(Translation2d[] moduleLocations) {
        _moduleStates = new ModuleState[moduleLocations.length];
        for (int i = 0; i < moduleLocations.length; i++) {
            _moduleStates[i] = new ModuleState(moduleLocations[i]);
        }
        resetModulePositions(_robotPose);
    }

    public void update(DriveInputs inputs) {
        double timestamp = Timer.getFPGATimestamp();
        Translation2d averageTranslation = new Translation2d();
        double totalTrust = 0.0;

        double[] translationalVelocities = new double[_moduleStates.length];
        double minVelocity = Double.MAX_VALUE;
        double maxVelocity = Double.MIN_VALUE;

        for (int i = 0; i < _moduleStates.length; i++) {
            ModuleState state = _moduleStates[i];
            state.updateState(
                    inputs.modulePositions[i].distanceMeters,
                    inputs.measuredStates[i].angle,
                    inputs.yawPosition,
                    timestamp);

            translationalVelocities[i] = Math.abs(state._lastVelocity);
            minVelocity = Math.min(minVelocity, translationalVelocities[i]);
            maxVelocity = Math.max(maxVelocity, translationalVelocities[i]);
        }

        boolean skidding = (maxVelocity / Math.max(minVelocity, 1e-5)) > SKID_RATIO_THRESHOLD;

        for (int i = 0; i < _moduleStates.length; i++) {
            ModuleState state = _moduleStates[i];

            if (!skidding || translationalVelocities[i] <= (minVelocity * SKID_RATIO_THRESHOLD)) {
                averageTranslation =
                        averageTranslation.plus(
                                state._estimatedRobotPose.getTranslation().times(state._trustFactor));
                totalTrust += state._trustFactor;
            }
        }

        Pose2d newPose = new Pose2d(averageTranslation.times(1.0 / totalTrust), inputs.yawPosition);

        double dt = timestamp - _lastTimestamp;
        if (dt > VELOCITY_SAMPLE_TIME) {
            _robotVelocity = newPose.getTranslation().minus(_robotPose.getTranslation()).times(1.0 / dt);
            _lastTimestamp = timestamp;
        }

        _robotPose = newPose;
        resetModulePositions(_robotPose);
    }

    private void resetModulePositions(Pose2d pose) {
        for (ModuleState state : _moduleStates) {
            state._position =
                    pose.transformBy(new Transform2d(state._offset, new Rotation2d())).getTranslation();
        }
    }

    public synchronized Pose2d getPose() {
        return _robotPose;
    }

    public Translation2d getVelocity() {
        return _robotVelocity;
    }

    public void resetPose(Pose2d pose) {
        _robotPose = pose;
        _lastTimestamp = Timer.getFPGATimestamp();
        resetModulePositions(pose);
        for (ModuleState state : _moduleStates) {
            state._trustFactor = 1.0;
        }
    }
}
