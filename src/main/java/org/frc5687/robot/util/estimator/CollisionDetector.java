package org.frc5687.robot.util.estimator;

import edu.wpi.first.wpilibj.Timer;

public class CollisionDetector implements EstimatorLoggable {
    private static final double COLLISION_THRESHOLD_G = 2.0;
    private static final double COLLISION_RESET_TIME = 0.5;

    private double _lastAccelX = 0.0;
    private double _lastAccelY = 0.0;
    private boolean _inCollision = false;
    private double _collisionStartTime = 0.0;
    private double _lastCollisionMagnitude = 0.0;

    public CollisionDetector() {}

    public boolean detect(double accelX, double accelY) {
        double currentTime = Timer.getFPGATimestamp();

        double deltaAccelX = Math.abs(accelX - _lastAccelX);
        double deltaAccelY = Math.abs(accelY - _lastAccelY);

        double magnitude = Math.sqrt(deltaAccelX * deltaAccelX + deltaAccelY * deltaAccelY);

        _lastAccelX = accelX;
        _lastAccelY = accelY;

        boolean collisionDetected = (magnitude > COLLISION_THRESHOLD_G);

        if (_inCollision && (currentTime - _collisionStartTime > COLLISION_RESET_TIME)) {
            _inCollision = false;
        }

        if (collisionDetected && !_inCollision) {
            _inCollision = true;
            _collisionStartTime = currentTime;
            _lastCollisionMagnitude = magnitude;

            log("Collision", true);
            log("Magnitude", magnitude);
            log("DeltaX", deltaAccelX);
            log("DeltaY", deltaAccelY);
            log("Time", currentTime);

            return true;
        }

        log("Collision", _inCollision);
        log("CurrentMagnitude", magnitude);

        return false;
    }

    public boolean isInCollisionState() {
        return _inCollision;
    }

    public double getLastCollisionMagnitude() {
        return _lastCollisionMagnitude;
    }

    public double getTimeSinceCollision() {
        if (!_inCollision) {
            return -1;
        }
        return Timer.getFPGATimestamp() - _collisionStartTime;
    }

    public void reset() {
        _inCollision = false;
        _lastAccelX = 0.0;
        _lastAccelY = 0.0;
        _lastCollisionMagnitude = 0.0;
    }

    public double getCollisionThreshold() {
        return COLLISION_THRESHOLD_G;
    }
}
