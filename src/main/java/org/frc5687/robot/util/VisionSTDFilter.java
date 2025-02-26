package org.frc5687.robot.util;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import java.util.List;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.targeting.PhotonTrackedTarget;

public class VisionSTDFilter {
    private static final TunableDouble BASE_STD =
            new TunableDouble("VisionSTDFilter", "BASE_STD", 0.3);
    private static final double MIN_STD = 0.1;
    private static final TunableDouble TAG_DISTANCE_FACTOR =
            new TunableDouble("VisionSTDFilter", "TAG_DISTANCE_FACTOR", 0.05);
    private static final TunableDouble MULTI_TAG_BOOST =
            new TunableDouble("VisionSTDFilter", "MULTI_TAG_BOOST", 0.3);

    // reworked to be WAY simpler, start adding functionality when needed.
    public static Matrix<N3, N1> calculateVisionStdDevs(EstimatedRobotPose visionPose) {
        double minDistance = getClosestTagDistance(visionPose.targetsUsed);
        int tagCount = visionPose.targetsUsed.size();

        double distanceStd = BASE_STD.get() + (minDistance * TAG_DISTANCE_FACTOR.get());

        double multiTagFactor = tagCount > 1 ? MULTI_TAG_BOOST.get() : 1.0;
        double xyStd = MathUtil.clamp(distanceStd * multiTagFactor, MIN_STD, BASE_STD.get());
        double thetaStd = Math.max(distanceStd * multiTagFactor, 0.4);
        // System.out.println("xy: " + xyStd);

        return VecBuilder.fill(xyStd, xyStd, thetaStd);
    }

    private static double getClosestTagDistance(List<PhotonTrackedTarget> targets) {
        double minDistance = Double.MAX_VALUE;
        for (PhotonTrackedTarget target : targets) {
            Transform3d transform = target.getBestCameraToTarget();
            double distance = Math.hypot(transform.getX(), transform.getY());
            minDistance = Math.min(minDistance, distance);
        }
        return minDistance;
    }
}
