package org.frc5687.robot.util.estimator;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform2d;
import org.frc5687.robot.subsystems.vision.RobotPoseEstimate;

/**
 * Calculates and tracks Figure of Merit (FOM) values for different pose sources. Lower FOM values
 * indicate higher confidence.
 */
public class FOMCalculator implements EstimatorLoggable {
    private static final double BASE_ODOMETRY_FOM = 0.1;
    private static final double MIN_FOM = 0.1;
    private static final double MAX_FOM = 10.0;
    private static final double COLLISION_FOM_MULTIPLIER = 1.5; // Reduced from 2.0
    private static final double SKID_FOM_MULTIPLIER = 1.2; // Reduced from 1.5
    private static final double FOM_RECOVERY_RATE = 0.9; // Faster recovery (was 0.95)

    private static final double MAX_SKID_FOM = 1.0; // Cap FOM during skidding
    private static final double MAX_COLLISION_FOM = 2.0; // Cap FOM during collision
    private static final int CONSECUTIVE_SKID_THRESHOLD =
            5; // Only increase FOM for persistent skidding

    private double _odometryFOM = BASE_ODOMETRY_FOM;
    private double _visionFOM = MAX_FOM;
    private double _robotFOM = BASE_ODOMETRY_FOM;
    private int _consecutiveSkidCount = 0;
    private int _consecutiveCollisionCount = 0;

    public FOMCalculator() {}

    public double updateOdometryFOM(boolean skidding, boolean collision) {
        if (collision) {
            _consecutiveCollisionCount++;

            if (_consecutiveCollisionCount >= 2) {
                _odometryFOM = Math.min(_odometryFOM * COLLISION_FOM_MULTIPLIER, MAX_COLLISION_FOM);
                log("CollisionFOMIncrease", _odometryFOM);
            }
        } else {
            _consecutiveCollisionCount = 0;
        }

        if (skidding) {
            _consecutiveSkidCount++;

            if (_consecutiveSkidCount >= CONSECUTIVE_SKID_THRESHOLD) {
                _odometryFOM = Math.min(_odometryFOM * SKID_FOM_MULTIPLIER, MAX_SKID_FOM);
                log("SkidFOMIncrease", _odometryFOM);
            } else if (_consecutiveSkidCount >= 2) {
                _odometryFOM = Math.min(_odometryFOM * 1.05, MAX_SKID_FOM);
            }
        } else {
            _consecutiveSkidCount = 0;

            _odometryFOM = Math.max(_odometryFOM * FOM_RECOVERY_RATE, BASE_ODOMETRY_FOM);
        }

        _odometryFOM = Math.max(Math.min(_odometryFOM, MAX_FOM), MIN_FOM);

        updateRobotFOM();

        log("Odometry", _odometryFOM);
        log("Robot", _robotFOM);
        log("SkidCount", _consecutiveSkidCount);

        return _odometryFOM;
    }

    public double calculateVisionFOM(RobotPoseEstimate poseEstimate, Pose2d currentPose) {
        double visionFOM = 0.5;

        if (poseEstimate == null) {
            _visionFOM = MAX_FOM;
            updateRobotFOM();
            return _visionFOM;
        }

        if (poseEstimate.tagCount <= 0) {
            // No tags means no confidence
            _visionFOM = MAX_FOM;
            updateRobotFOM();
            return _visionFOM;
        } else if (poseEstimate.tagCount >= 3) {
            // Multiple tags significantly improve confidence
            visionFOM *= 0.4;
        } else if (poseEstimate.tagCount == 2) {
            // Two tags are better than one
            visionFOM *= 0.6;
        }

        if (poseEstimate.avgTagDist > 3.0) {
            visionFOM *= 1.0 + ((poseEstimate.avgTagDist - 3.0) * 0.2);
        } else if (poseEstimate.avgTagDist < 1.5) {
            visionFOM *= 0.8; // Close tags are reliable
        }

        // if (poseEstimate.confidence < 0.8) {
        //     visionFOM *= 1.0 + ((0.8 - poseEstimate.confidence) * 2.0);
        // } else if (poseEstimate.confidence > 0.95) {
        //     visionFOM *= 0.9; // Very high confidence is good
        // }

        if (poseEstimate.tagCount >= 2 && poseEstimate.tagSpan > 0.5) {
            double spanFactor = Math.min(poseEstimate.tagSpan, 3.0) / 3.0;
            visionFOM *= 1.0 - (spanFactor * 0.2);
        }

        double latency = poseEstimate.getLatency();
        if (latency > 0.1) {
            visionFOM *= 1.0 + (latency * 0.5);
        }

        if (currentPose != null && _robotFOM < 0.5) {
            Transform2d poseJump = new Transform2d(currentPose, poseEstimate.pose);
            double jumpDistance = poseJump.getTranslation().getNorm();

            if (jumpDistance > 0.3) {
                double jumpFactor = jumpDistance / 0.3;
                visionFOM *= Math.min(jumpFactor, 3.0);
            }
        }

        _visionFOM = Math.max(Math.min(visionFOM, MAX_FOM), MIN_FOM);

        updateRobotFOM();

        log("Vision", _visionFOM);
        log("Robot", _robotFOM);
        log("Vision/TagCount", poseEstimate.tagCount);
        log("Vision/AvgDist", poseEstimate.avgTagDist);
        log("Vision/Confidence", poseEstimate.confidence);
        log("Vision/Latency", latency);

        return _visionFOM;
    }

    private void updateRobotFOM() {
        _robotFOM = Math.min(_odometryFOM, _visionFOM);
    }

    public double getOdometryFOM() {
        return _odometryFOM;
    }

    public double getVisionFOM() {
        return _visionFOM;
    }

    public double getRobotFOM() {
        return _robotFOM;
    }

    public boolean isPoseReliable() {
        return _robotFOM < 0.5;
    }

    public void reset() {
        _odometryFOM = BASE_ODOMETRY_FOM;
        _visionFOM = MAX_FOM;
        _robotFOM = BASE_ODOMETRY_FOM;
        _consecutiveSkidCount = 0;
        _consecutiveCollisionCount = 0;
    }

    public static double[] calculateBlendWeights(double fom1, double fom2) {
        double weight1 = 1.0 / (fom1 * fom1);
        double weight2 = 1.0 / (fom2 * fom2);

        double totalWeight = weight1 + weight2;

        if (totalWeight < 1e-6) {
            return new double[] {0.5, 0.5};
        }

        return new double[] {weight1 / totalWeight, weight2 / totalWeight};
    }
}
