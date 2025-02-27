package org.frc5687.robot.util.estimator;

import edu.wpi.first.math.kinematics.SwerveModuleState;

public class SkidDetector implements EstimatorLoggable {
    private static final double SKID_RATIO_THRESHOLD = 1.2;
    private static final double MIN_VELOCITY_THRESHOLD = 0.1;

    private boolean _isSkidding = false;
    private final double[] _moduleVelocities;

    public SkidDetector(int moduleCount) {
        _moduleVelocities = new double[moduleCount];
    }

    public boolean detect(SwerveModuleState[] moduleStates) {
        for (int i = 0; i < moduleStates.length; i++) {
            _moduleVelocities[i] = Math.abs(moduleStates[i].speedMetersPerSecond);
        }

        return detect(_moduleVelocities);
    }

    public boolean detect(double[] moduleVelocities) {
        System.arraycopy(
                moduleVelocities,
                0,
                _moduleVelocities,
                0,
                Math.min(moduleVelocities.length, _moduleVelocities.length));

        double minVelocity = Double.MAX_VALUE;
        double maxVelocity = 0.0;
        int movingModules = 0;

        for (double velocity : _moduleVelocities) {
            if (velocity > MIN_VELOCITY_THRESHOLD) {
                minVelocity = Math.min(minVelocity, velocity);
                maxVelocity = Math.max(maxVelocity, velocity);
                movingModules++;
            }
        }

        if (movingModules < 2 || minVelocity == Double.MAX_VALUE) {
            _isSkidding = false;

            log("MovingModules", movingModules);
            log("VelocityRatio", 1.0);
            log("Skidding", false);

            return false;
        }

        double velocityRatio = maxVelocity / minVelocity;

        _isSkidding = velocityRatio > SKID_RATIO_THRESHOLD;

        log("MinVelocity", minVelocity);
        log("MaxVelocity", maxVelocity);
        log("MovingModules", movingModules);
        log("VelocityRatio", velocityRatio);
        log("Skidding", _isSkidding);

        return _isSkidding;
    }

    public boolean isSkidding() {
        return _isSkidding;
    }

    public double getSkidRatioThreshold() {
        return SKID_RATIO_THRESHOLD;
    }

    public void reset() {
        _isSkidding = false;

        for (int i = 0; i < _moduleVelocities.length; i++) {
            _moduleVelocities[i] = 0.0;
        }
    }
}
