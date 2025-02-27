package org.frc5687.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.util.struct.Struct;
import edu.wpi.first.util.struct.StructSerializable;
import edu.wpi.first.wpilibj.Timer;
import java.nio.ByteBuffer;
import java.util.Arrays;
import java.util.List;
import org.frc5687.robot.util.vision.LimelightHelpers;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

/**
 * RobotPoseEstimate - A unified data class for robot pose estimates from any vision system Works
 * with both PhotonVision and Limelight (including MegaTag2)
 */
public class RobotPoseEstimate implements StructSerializable {
    public static class RobotPoseEstimateStruct implements Struct<RobotPoseEstimate> {
        @Override
        public Class<RobotPoseEstimate> getTypeClass() {
            return RobotPoseEstimate.class;
        }

        @Override
        public String getTypeName() {
            return "RobotPoseEstimate";
        }

        @Override
        public int getSize() {
            return Pose2d.struct.getSize()
                    + kSizeDouble
                    + kSizeInt32
                    + 3 * kSizeDouble
                    + kSizeDouble
                    +
                    // We'll specify tag IDs as int32[8] with a size prefix for dynamic count
                    kSizeInt32
                    + 8 * kSizeInt32
                    +
                    // Source string as char[32]
                    32 * 1;
        }

        @Override
        public String getSchema() {
            return "Pose2d pose;double timestampSeconds;int32 tagCount;double tagSpan;"
                    + "double avgTagDist;double avgTagArea;double confidence;"
                    + "int32 tagIdsSize;int32 tagIds[8];"
                    + "char source[32]";
        }

        @Override
        public Struct<?>[] getNested() {
            return new Struct<?>[] {Pose2d.struct};
        }

        @Override
        public RobotPoseEstimate unpack(ByteBuffer bb) {
            Pose2d pose = Pose2d.struct.unpack(bb);
            double timestampSeconds = bb.getDouble();
            int tagCount = bb.getInt();
            double tagSpan = bb.getDouble();
            double avgTagDist = bb.getDouble();
            double avgTagArea = bb.getDouble();
            double confidence = bb.getDouble();

            int tagIdsSize = bb.getInt();
            int[] tagIds = new int[tagIdsSize];
            for (int i = 0; i < tagIdsSize && i < 8; i++) {
                tagIds[i] = bb.getInt();
            }

            for (int i = tagIdsSize; i < 8; i++) {
                bb.getInt(); // Skip
            }

            byte[] sourceBytes = new byte[32];
            bb.get(sourceBytes);
            String source = new String(sourceBytes).trim(); // Trim trailing zeros

            return new RobotPoseEstimate(
                    pose,
                    timestampSeconds,
                    tagCount,
                    tagSpan,
                    avgTagDist,
                    avgTagArea,
                    confidence,
                    tagIds,
                    source);
        }

        @Override
        public void pack(ByteBuffer bb, RobotPoseEstimate value) {
            Pose2d.struct.pack(bb, value.pose);
            bb.putDouble(value.timestampSeconds);
            bb.putInt(value.tagCount);
            bb.putDouble(value.tagSpan);
            bb.putDouble(value.avgTagDist);
            bb.putDouble(value.avgTagArea);
            bb.putDouble(value.confidence);

            int tagIdsSize = Math.min(value.tagIds.length, 8);
            bb.putInt(tagIdsSize);
            for (int i = 0; i < tagIdsSize; i++) {
                bb.putInt(value.tagIds[i]);
            }

            for (int i = tagIdsSize; i < 8; i++) {
                bb.putInt(0);
            }

            byte[] sourceBytes = new byte[32];
            byte[] sourceStringBytes = value.source.getBytes();
            int copyLen = Math.min(sourceStringBytes.length, 32);
            System.arraycopy(sourceStringBytes, 0, sourceBytes, 0, copyLen);
            bb.put(sourceBytes);
        }

        @Override
        public boolean isImmutable() {
            return true;
        }
    }

    public final Pose2d pose;

    public final double timestampSeconds;

    public final int tagCount;
    public final double tagSpan;
    public final double avgTagDist;
    public final double avgTagArea;

    public final double confidence;

    public final int[] tagIds;

    public final String source;

    public RobotPoseEstimate(
            Pose2d pose,
            double timestampSeconds,
            int tagCount,
            double tagSpan,
            double avgTagDist,
            double avgTagArea,
            double confidence,
            int[] tagIds,
            String source) {
        this.pose = pose;
        this.timestampSeconds = timestampSeconds;
        this.tagCount = tagCount;
        this.tagSpan = tagSpan;
        this.avgTagDist = avgTagDist;
        this.avgTagArea = avgTagArea;
        this.confidence = confidence;
        this.tagIds = tagIds != null ? tagIds : new int[0];
        this.source = source != null ? source : "";
    }

    public Pose3d getPose3d() {
        return new Pose3d(pose);
    }

    public double getLatency() {
        return Timer.getFPGATimestamp() - timestampSeconds;
    }

    public double getQualityScore() {
        double score = confidence * 40;

        score += Math.min(tagCount * 10, 30);

        if (tagCount > 1) {
            score += Math.min(tagSpan * 5, 15);
        }

        double distanceDeduction = Math.min(avgTagDist, 6) * 2;
        score -= distanceDeduction;

        score += Math.min(avgTagArea * 3, 15);

        return Math.max(0, score);
    }

    public static RobotPoseEstimate fromLimelight(
            LimelightHelpers.PoseEstimate poseEstimate, String source) {

        int[] tagIds = new int[poseEstimate.tagCount];
        if (poseEstimate.rawFiducials != null) {
            for (int i = 0; i < Math.min(poseEstimate.rawFiducials.length, tagIds.length); i++) {
                tagIds[i] = poseEstimate.rawFiducials[i].id;
            }
        }

        double confidence = 0.7; // Base confidence for MegaTag2

        if (poseEstimate.tagCount >= 3) {
            confidence += 0.2; // High confidence with 3+ tags
        } else if (poseEstimate.tagCount == 2) {
            confidence += 0.1; // Medium confidence with 2 tags
        }

        if (poseEstimate.avgTagDist < 3.0) {
            confidence += 0.1; // Boost confidence for close tags
        }

        confidence = Math.min(confidence, 1.0);

        return new RobotPoseEstimate(
                poseEstimate.pose,
                poseEstimate.timestampSeconds,
                poseEstimate.tagCount,
                poseEstimate.tagSpan,
                poseEstimate.avgTagDist,
                poseEstimate.avgTagArea,
                confidence,
                tagIds,
                source);
    }

    public static RobotPoseEstimate fromPhotonVision(
            EstimatedRobotPose photonPose, List<PhotonTrackedTarget> targets, String source) {

        Pose2d pose = photonPose.estimatedPose.toPose2d();
        double timestamp = photonPose.timestampSeconds;
        int tagCount = targets.size();

        int[] tagIds = new int[tagCount];
        for (int i = 0; i < tagCount; i++) {
            tagIds[i] = targets.get(i).getFiducialId();
        }

        double tagSpan = 0.0;
        if (tagCount > 1) {
            for (int i = 0; i < tagCount; i++) {
                for (int j = i + 1; j < tagCount; j++) {
                    var transform1 = targets.get(i).getBestCameraToTarget();
                    var transform2 = targets.get(j).getBestCameraToTarget();
                    if (transform1 != null && transform2 != null) {
                        double distance =
                                transform1.getTranslation().minus(transform2.getTranslation()).getNorm();
                        tagSpan = Math.max(tagSpan, distance);
                    }
                }
            }
        }

        double avgTagDist = 0.0;
        double avgTagArea = 0.0;
        for (int i = 0; i < tagCount; i++) {
            var transform = targets.get(i).getBestCameraToTarget();
            if (transform != null) {
                avgTagDist += transform.getTranslation().getNorm();
            }
            avgTagArea += targets.get(i).getArea();
        }

        if (tagCount > 0) {
            avgTagDist /= tagCount;
            avgTagArea /= tagCount;
        }

        double confidence = 0.6; // Base confidence

        if (tagCount >= 3) {
            confidence += 0.3; // High confidence with 3+ tags
        } else if (tagCount == 2) {
            confidence += 0.2; // Medium confidence with 2 tags
        }

        if (avgTagDist < 3.0) {
            confidence += 0.1; // Boost confidence for close tags
        }

        confidence = Math.min(confidence, 1.0);

        return new RobotPoseEstimate(
                pose, timestamp, tagCount, tagSpan, avgTagDist, avgTagArea, confidence, tagIds, source);
    }

    public static RobotPoseEstimate fromPhotonVision(EstimatedRobotPose photonPose, String source) {
        return fromPhotonVision(photonPose, photonPose.targetsUsed, source);
    }

    public static RobotPoseEstimate fromPhotonVision(
            EstimatedRobotPose photonPose, PhotonPipelineResult result, String source) {
        return fromPhotonVision(photonPose, photonPose.targetsUsed, source);
    }

    @Override
    public String toString() {
        return String.format(
                "RobotPoseEstimate[pose=(%s), tags=%d, span=%.2fm, dist=%.2fm, area=%.2f%%, conf=%.2f, "
                        + "tagIds=%s, latency=%.1fms, source=%s]",
                pose,
                tagCount,
                tagSpan,
                avgTagDist,
                avgTagArea,
                confidence,
                Arrays.toString(tagIds),
                getLatency() * 1000,
                source);
    }

    public static final RobotPoseEstimateStruct struct = new RobotPoseEstimateStruct();
}
