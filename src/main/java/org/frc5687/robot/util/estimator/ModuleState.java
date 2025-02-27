package org.frc5687.robot.util.estimator;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;

public class ModuleState implements EstimatorLoggable {
    // Constants
    private static final double MODULE_TRUST_THRESHOLD = 0.01;
    private static final double MIN_TRUST_FACTOR = 0.1;
    private static final double MAX_TRUST_FACTOR = 1.0;
    private static final double TRUST_DECREASE_RATE = 0.8;
    private static final double TRUST_INCREASE_RATE = 1.2;

    private double _previousDistance = 0.0;
    private Translation2d _position;
    private final Translation2d _offset;
    private Pose2d _estimatedRobotPose = new Pose2d();
    private double _trustFactor = 1.0;
    private double _lastVelocity = 0.0;

    public ModuleState(Translation2d moduleOffset) {
        _offset = moduleOffset;
        _position = moduleOffset;
    }

    public Pose2d update(
            double newDistance,
            Rotation2d angle,
            Rotation2d robotYaw,
            double currentTime,
            double lastTimestamp) {

        double dt = currentTime - lastTimestamp;
        double deltaDistance = newDistance - _previousDistance;

        if (dt <= 0) {
            return _estimatedRobotPose;
        }

        Rotation2d globalAngle = angle.rotateBy(robotYaw);

        Translation2d deltaPosition =
                new Translation2d(
                        globalAngle.getCos() * deltaDistance, globalAngle.getSin() * deltaDistance);

        _position = _position.plus(deltaPosition);

        _estimatedRobotPose =
                new Pose2d(_position, robotYaw)
                        .transformBy(new Transform2d(_offset.unaryMinus(), new Rotation2d()));

        double velocity = deltaDistance / dt;
        updateTrustFactor(velocity);

        _previousDistance = newDistance;
        _lastVelocity = velocity;

        return _estimatedRobotPose;
    }

    private void updateTrustFactor(double currentVelocity) {
        double velocityDelta = Math.abs(currentVelocity - _lastVelocity);

        if (velocityDelta > MODULE_TRUST_THRESHOLD) {
            _trustFactor = Math.max(_trustFactor * TRUST_DECREASE_RATE, MIN_TRUST_FACTOR);
        } else {
            _trustFactor = Math.min(_trustFactor * TRUST_INCREASE_RATE, MAX_TRUST_FACTOR);
        }
    }

    public double getTrustFactor() {
        return _trustFactor;
    }

    public Pose2d getEstimatedRobotPose() {
        return _estimatedRobotPose;
    }

    public double getVelocity() {
        return _lastVelocity;
    }

    public void reset(Pose2d robotPose) {
        _position = robotPose.transformBy(new Transform2d(_offset, new Rotation2d())).getTranslation();

        _estimatedRobotPose = robotPose;
        _previousDistance = 0.0;
        _lastVelocity = 0.0;
        _trustFactor = MAX_TRUST_FACTOR;
    }

    @Override
    public String getLogBase() {
        return EstimatorLoggable.super.getLogBase() + "/" + _offset.getX() + "_" + _offset.getY();
    }
}
