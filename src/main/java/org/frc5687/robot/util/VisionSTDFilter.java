package org.frc5687.robot.util;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import java.util.List;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.targeting.PhotonTrackedTarget;

public class VisionSTDFilter {
    private static final TunableDouble MIN_STD = new TunableDouble("VisionSTDFilter", "MIN_STD", 0.1);
    private static final TunableDouble TAG_DISTANCE_FACTOR =
            new TunableDouble("VisionSTDFilter", "TAG_DISTANCE_FACTOR", 0.05); // 5cm per meter stddev
    private static final TunableDouble MULTI_TAG_BOOST =
            new TunableDouble(
                    "VisionSTDFilter", "MULTI_TAG_BOOST", 0.2); // 1/5 as high stddev if multitag used

    // reworked to be WAY simpler, start adding functionality when needed.
    public static Matrix<N3, N1> calculateVisionStdDevs(EstimatedRobotPose visionPose) {
        double minDistance = getClosestTagDistance(visionPose.targetsUsed);
        int tagCount = visionPose.targetsUsed.size();

        double distanceStd = minDistance * TAG_DISTANCE_FACTOR.get();

        double multiTagFactor = tagCount > 1 ? MULTI_TAG_BOOST.get() : 1.0;
        double xyStd = MIN_STD.get() + distanceStd * multiTagFactor;
        double thetaStd = MIN_STD.get() + multiTagFactor;
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

    // private static double getWorstAmbiguity(List<PhotonTrackedTarget> targets) {
    //     double worstAmbiguity = 0.0;
    //     for (PhotonTrackedTarget target : targets) {
    //         worstAmbiguity = Math.max(worstAmbiguity, target.getPoseAmbiguity());
    //     }
    //     return worstAmbiguity;
    // }
}
