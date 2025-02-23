package org.frc5687.robot.util;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import java.util.ArrayList;
import java.util.List;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.targeting.PhotonTrackedTarget;

public class VisionSTDFilter {
    private static final double BASE_XY_STD = 0.5;
    private static final double BASE_THETA_STD = 0.3;
    private static final double MIN_XY_STD = 0.1;
    private static final double MIN_THETA_STD = 0.02;
    private static final double MAX_XY_STD = 2.0;
    private static final double MAX_THETA_STD = 1.0;

    private static final double DISTANCE_SCALE_FACTOR = 0.009;
    private static final double AMBIGUITY_SCALE_FACTOR = 0.03;
    private static final double TIME_SCALE_FACTOR = 0.001;
    private static final double VELOCITY_SCALE_FACTOR = 0.3;
    private static final double VELOCITY_EXPONENT = 2.0;

    private static final double MIN_TAG_AREA_PERCENT = 0.1;
    private static final double GOOD_TAG_AREA_PERCENT = 1.0;
    private static final double MAX_TAG_SKEW = 0.8;

    private static final int HISTORY_SIZE = 10;
    private final List<Pose2d> poseHistory = new ArrayList<>();
    private final List<Double> timestampHistory = new ArrayList<>();
    private final List<Double> confidenceHistory = new ArrayList<>();

    private boolean _hasInitialLocalization = false;
    private double _lastGoodPoseTimestamp = 0;
    private Pose2d _lastGoodPose = new Pose2d();
    private double _robotVelocity = 0.0;

    public Matrix<N3, N1> calculateVisionStdDevs(
            EstimatedRobotPose visionPose,
            Pose2d currentEstimatedPose,
            double currentTimestamp,
            double currentVelocity) {

        _robotVelocity = currentVelocity;
        double xyStd = _hasInitialLocalization ? MIN_XY_STD : BASE_XY_STD;
        double thetaStd = _hasInitialLocalization ? MIN_THETA_STD : BASE_THETA_STD;

        double avgTagQuality = calculateTagQuality(visionPose.targetsUsed);
        double minTagDistance = calculateMinTagDistance(visionPose.targetsUsed);

        double distanceMultiplier = calculateDistanceMultiplier(minTagDistance);
        double ambiguityMultiplier = calculateAmbiguityMultiplier(visionPose.targetsUsed.size());
        double timeMultiplier = calculateTimeMultiplier(currentTimestamp);
        double velocityMultiplier = calculateVelocityMultiplier();
        double qualityMultiplier = calculateQualityMultiplier(avgTagQuality);

        double combinedMultiplier =
                distanceMultiplier
                        * ambiguityMultiplier
                        * timeMultiplier
                        * velocityMultiplier
                        * qualityMultiplier;

        xyStd *= Math.max(1.0, combinedMultiplier);
        thetaStd *= Math.max(1.0, combinedMultiplier);

        if (detectPoseJump(visionPose.estimatedPose.toPose2d(), currentTimestamp)) {
            xyStd *= 2.0;
            thetaStd *= 2.0;
        }

        updateHistory(
                visionPose.estimatedPose.toPose2d(),
                1.0 / (qualityMultiplier * distanceMultiplier),
                currentTimestamp);

        if (!_hasInitialLocalization) {
            handleInitialLocalization(visionPose, currentEstimatedPose, currentTimestamp);
        }

        xyStd = MathUtil.clamp(xyStd, MIN_XY_STD, MAX_XY_STD);
        thetaStd = MathUtil.clamp(thetaStd, MIN_THETA_STD, MAX_THETA_STD);
        System.out.println("xy: " + xyStd);

        return VecBuilder.fill(xyStd, xyStd, thetaStd);
    }

    private double calculateTagQuality(List<PhotonTrackedTarget> targets) {
        if (targets.isEmpty()) return 0.0;

        double totalQuality = 0.0;
        for (PhotonTrackedTarget target : targets) {
            double areaQuality =
                    Math.min(
                            1.0,
                            (target.getArea() - MIN_TAG_AREA_PERCENT)
                                    / (GOOD_TAG_AREA_PERCENT - MIN_TAG_AREA_PERCENT));

            double skewQuality = 1.0 - (target.getSkew() / MAX_TAG_SKEW);
            double poseAmbiguity = 1.0 - Math.min(1.0, target.getPoseAmbiguity());
            totalQuality += (areaQuality + skewQuality + poseAmbiguity) / 3.0;
        }
        return totalQuality / targets.size();
    }

    private double calculateDistanceMultiplier(double minTagDistance) {
        return 1.0 + (minTagDistance * DISTANCE_SCALE_FACTOR);
    }

    private double calculateAmbiguityMultiplier(int numTags) {
        return 1.0 + (AMBIGUITY_SCALE_FACTOR * (3.0 / Math.max(1, numTags)));
    }

    private double calculateTimeMultiplier(double currentTimestamp) {
        return 1.0 + ((currentTimestamp - _lastGoodPoseTimestamp) * TIME_SCALE_FACTOR);
    }

    private double calculateVelocityMultiplier() {
        return 1.0 + Math.pow(_robotVelocity * VELOCITY_SCALE_FACTOR, VELOCITY_EXPONENT);
    }

    private double calculateQualityMultiplier(double tagQuality) {
        return 2.0 - Math.min(1.0, tagQuality);
    }

    private boolean detectPoseJump(Pose2d newPose, double newTimestamp) {
        if (poseHistory.isEmpty()) return false;

        Pose2d lastPose = poseHistory.get(poseHistory.size() - 1);
        double lastTimestamp = timestampHistory.get(timestampHistory.size() - 1);
        double dt = newTimestamp - lastTimestamp;

        double translation = lastPose.getTranslation().getDistance(newPose.getTranslation());
        double rotation = Math.abs(lastPose.getRotation().minus(newPose.getRotation()).getDegrees());

        double maxPossibleTranslation = _robotVelocity * dt * 1.5;
        double maxPossibleRotation = 45.0 * dt;

        return translation > maxPossibleTranslation || rotation > maxPossibleRotation;
    }

    private void updateHistory(Pose2d pose, double confidence, double timestamp) {
        poseHistory.add(pose);
        timestampHistory.add(timestamp);
        confidenceHistory.add(confidence);

        if (poseHistory.size() > HISTORY_SIZE) {
            poseHistory.remove(0);
            timestampHistory.remove(0);
            confidenceHistory.remove(0);
        }
    }

    private void handleInitialLocalization(
            EstimatedRobotPose visionPose, Pose2d currentEstimatedPose, double currentTimestamp) {

        double poseDifference =
                currentEstimatedPose
                        .getTranslation()
                        .getDistance(visionPose.estimatedPose.toPose2d().getTranslation());

        boolean hasMultipleTags = visionPose.targetsUsed.size() >= 2;
        boolean hasGoodQuality = calculateTagQuality(visionPose.targetsUsed) > 0.7;
        boolean hasReasonableDistance = calculateMinTagDistance(visionPose.targetsUsed) < 4.0;

        if (poseDifference > 1.0 && hasMultipleTags && hasGoodQuality && hasReasonableDistance) {
            _hasInitialLocalization = true;
            _lastGoodPoseTimestamp = currentTimestamp;
            _lastGoodPose = visionPose.estimatedPose.toPose2d();
        }
    }

    private double calculateMinTagDistance(List<PhotonTrackedTarget> targets) {
        double minDistance = Double.MAX_VALUE;
        for (PhotonTrackedTarget target : targets) {
            Transform3d camToTarget = target.getBestCameraToTarget();
            double distance =
                    Math.sqrt(
                            camToTarget.getX() * camToTarget.getX() + camToTarget.getY() * camToTarget.getY());
            minDistance = Math.min(minDistance, distance);
        }
        return minDistance;
    }

    public void reset() {
        _hasInitialLocalization = false;
        _lastGoodPoseTimestamp = 0;
        _lastGoodPose = new Pose2d();
        poseHistory.clear();
        timestampHistory.clear();
        confidenceHistory.clear();
    }
}
