package org.frc5687.robot.util.estimator;

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
    private Pose2d _previousRobotPose = new Pose2d();
    private Translation2d _robotVelocity = new Translation2d();
    private double _lastTimestamp = 0.0;
    private boolean _firstCycle = true;

    // Constants
    private static final double VELOCITY_SAMPLE_TIME = 0.02;
    private static final double MODULE_TRUST_THRESHOLD = 0.01;
    private static final double SKID_RATIO_THRESHOLD = 1.2;
    private static final double MIN_VELOCITY_FOR_SKID_DETECTION = 0.05; // m/s
    private static final double MAX_POSITION_CHANGE =
            0.1; // Maximum position change per cycle in meters

    // Do not reset module positions every cycle - this is CRITICAL
    // Only reset on initialization and when explicitly told to
    private static final boolean RESET_MODULES_EVERY_CYCLE = false;

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

        public Pose2d updateState(
                double newDistance,
                Rotation2d angle,
                Rotation2d robotYaw,
                double currentTime,
                double lastTime) {
            double dt = currentTime - lastTime;
            double deltaDistance = newDistance - _previousDistance;
            Rotation2d globalAngle = angle.rotateBy(robotYaw);

            Translation2d deltaPosition =
                    new Translation2d(
                            globalAngle.getCos() * deltaDistance, globalAngle.getSin() * deltaDistance);

            _position = _position.plus(deltaPosition);
            _estimatedRobotPose =
                    new Pose2d(_position, robotYaw)
                            .transformBy(new Transform2d(_offset.unaryMinus(), new Rotation2d()));

            // Calculate velocity with protection against division by zero
            double velocity = dt > 1e-5 ? deltaDistance / dt : 0.0;
            updateTrustFactor(velocity);

            _previousDistance = newDistance;
            _lastVelocity = velocity;

            return _estimatedRobotPose;
        }

        private void updateTrustFactor(double currentVelocity) {
            double velocityDelta = Math.abs(currentVelocity - _lastVelocity);
            if (velocityDelta > MODULE_TRUST_THRESHOLD) {
                _trustFactor = Math.max(_trustFactor * 0.8, 0.1);
            } else {
                _trustFactor = Math.min(_trustFactor * 1.2, 1.0);
            }
        }

        public double getTrustFactor() {
            return _trustFactor;
        }

        public double getVelocity() {
            return _lastVelocity;
        }

        public Pose2d getEstimatedRobotPose() {
            return _estimatedRobotPose;
        }

        public void resetPosition(Pose2d robotPose) {
            _position =
                    robotPose.transformBy(new Transform2d(_offset, new Rotation2d())).getTranslation();
            _previousDistance = 0.0; // Optional: reset distance when explicitly resetting
        }
    }

    public SwerveOdometry(Translation2d[] moduleLocations) {
        _moduleStates = new ModuleState[moduleLocations.length];
        for (int i = 0; i < moduleLocations.length; i++) {
            _moduleStates[i] = new ModuleState(moduleLocations[i]);
        }
        resetModulePositions(_robotPose);
        _lastTimestamp = Timer.getFPGATimestamp();
    }

    public void update(DriveInputs inputs) {
        double currentTime = Timer.getFPGATimestamp();
        double dt = currentTime - _lastTimestamp;

        if (_firstCycle) {
            _firstCycle = false;
            _lastTimestamp = currentTime;
            return;
        }

        _previousRobotPose = _robotPose;

        Translation2d weightedSum = new Translation2d();
        double totalWeight = 0.0;

        double[] moduleVelocities = new double[_moduleStates.length];
        Pose2d[] modulePoses = new Pose2d[_moduleStates.length];

        for (int i = 0; i < _moduleStates.length; i++) {
            modulePoses[i] =
                    _moduleStates[i].updateState(
                            inputs.modulePositions[i].distanceMeters,
                            inputs.measuredStates[i].angle,
                            inputs.yawPosition,
                            currentTime,
                            _lastTimestamp);

            moduleVelocities[i] = Math.abs(_moduleStates[i].getVelocity());
        }

        double averageVelocity = 0;
        for (double velocity : moduleVelocities) {
            averageVelocity += velocity;
        }
        averageVelocity /= moduleVelocities.length;

        boolean skidding = false;
        if (averageVelocity > MIN_VELOCITY_FOR_SKID_DETECTION) {
            // Find min and max velocities among modules that are moving
            double minVelocity = Double.MAX_VALUE;
            double maxVelocity = 0.0;

            for (double velocity : moduleVelocities) {
                if (velocity > MIN_VELOCITY_FOR_SKID_DETECTION) {
                    minVelocity = Math.min(minVelocity, velocity);
                    maxVelocity = Math.max(maxVelocity, velocity);
                }
            }

            // Protect against division by zero
            if (minVelocity > 0) {
                skidding = (maxVelocity / minVelocity) > SKID_RATIO_THRESHOLD;
            }
        }

        for (int i = 0; i < _moduleStates.length; i++) {
            if (skidding && moduleVelocities[i] > MIN_VELOCITY_FOR_SKID_DETECTION) {
                double velocityRatio = moduleVelocities[i] / Math.max(averageVelocity, 0.01);
                if (velocityRatio > 1.1) { // Module moving significantly faster than average
                    continue; // Skip this module for position calculation
                }
            }

            double trust = _moduleStates[i].getTrustFactor();
            weightedSum = weightedSum.plus(modulePoses[i].getTranslation().times(trust));
            totalWeight += trust;
        }

        if (totalWeight > 0) {
            Translation2d newTranslation = weightedSum.times(1.0 / totalWeight);

            Translation2d positionDelta = newTranslation.minus(_previousRobotPose.getTranslation());
            double distanceMoved = positionDelta.getNorm();

            if (distanceMoved > MAX_POSITION_CHANGE && !_firstCycle) {
                positionDelta = positionDelta.times(MAX_POSITION_CHANGE / distanceMoved);
                newTranslation = _previousRobotPose.getTranslation().plus(positionDelta);
            }

            _robotPose = new Pose2d(newTranslation, inputs.yawPosition);

            if (dt > VELOCITY_SAMPLE_TIME) {
                _robotVelocity = newTranslation.minus(_previousRobotPose.getTranslation()).times(1.0 / dt);
            }
        }

        if (RESET_MODULES_EVERY_CYCLE) {
            resetModulePositions(_robotPose);
        }

        _lastTimestamp = currentTime;
    }

    private void resetModulePositions(Pose2d pose) {
        for (ModuleState state : _moduleStates) {
            state.resetPosition(pose);
        }
    }

    public synchronized Pose2d getPose() {
        return _robotPose;
    }

    public Translation2d getVelocity() {
        return _robotVelocity;
    }

    public void resetPose(Pose2d pose) {
        _previousRobotPose = pose;
        _robotPose = pose;
        _lastTimestamp = Timer.getFPGATimestamp();
        resetModulePositions(pose);
        _firstCycle = true;

        for (ModuleState state : _moduleStates) {
            state._trustFactor = 1.0;
        }
    }
}
