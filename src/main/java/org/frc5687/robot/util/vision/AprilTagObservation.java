package org.frc5687.robot.util.vision;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.util.struct.Struct;
import edu.wpi.first.util.struct.StructSerializable;
import edu.wpi.first.wpilibj.Timer;
import java.nio.ByteBuffer;
import java.util.Optional;
import org.frc5687.robot.util.vision.LimelightHelpers.LimelightTarget_Fiducial;
import org.frc5687.robot.util.vision.LimelightHelpers.RawFiducial;
import org.photonvision.targeting.PhotonTrackedTarget;

public class AprilTagObservation implements StructSerializable {
    public static class AprilTagObservationStruct implements Struct<AprilTagObservation> {
        @Override
        public Class<AprilTagObservation> getTypeClass() {
            return AprilTagObservation.class;
        }

        @Override
        public String getTypeName() {
            return "AprilTagObservation";
        }

        @Override
        public int getSize() {
            return kSizeInt32
                    + Transform3d.struct.getSize()
                    + 4 * kSizeDouble
                    + TargetCorners.struct.getSize()
                    + kSizeDouble
                    + kSizeDouble
                    + kSizeDouble;
        }

        @Override
        public String getSchema() {
            return "int32 id;Transform3d transform;double area;double ambiguity;double tx;double ty;"
                    + "TargetCorners corners;double timestamp;double confidence;double poseError";
        }

        @Override
        public Struct<?>[] getNested() {
            return new Struct<?>[] {Transform3d.struct, TargetCorners.struct};
        }

        @Override
        public AprilTagObservation unpack(ByteBuffer bb) {
            int id = bb.getInt();
            Transform3d transform = Transform3d.struct.unpack(bb);
            double area = bb.getDouble();
            double ambiguity = bb.getDouble();
            double tx = bb.getDouble();
            double ty = bb.getDouble();
            TargetCorners corners = TargetCorners.struct.unpack(bb);
            double timestamp = bb.getDouble();
            double confidence = bb.getDouble();
            double poseError = bb.getDouble();

            return new AprilTagObservation(
                    id, transform, area, ambiguity, tx, ty, corners, timestamp, confidence, poseError);
        }

        @Override
        public void pack(ByteBuffer bb, AprilTagObservation value) {
            bb.putInt(value.getId());
            Transform3d.struct.pack(bb, value.getTransform());
            bb.putDouble(value.getArea());
            bb.putDouble(value.getAmbiguity());
            bb.putDouble(value.getTx());
            bb.putDouble(value.getTy());
            TargetCorners.struct.pack(bb, value.getCorners());
            bb.putDouble(value.getTimestamp());
            bb.putDouble(value.getConfidence());
            bb.putDouble(value.getPoseError());
        }

        @Override
        public boolean isImmutable() {
            return true;
        }
    }

    private final int id;
    private final Transform3d transform;
    private final double area;
    private final double ambiguity;
    private final double tx;
    private final double ty;
    private final TargetCorners corners;
    private final double timestamp;
    private final double confidence;
    private final double poseError;

    private Double distance = null;
    private Double distance2d = null;

    public AprilTagObservation(
            int id,
            Transform3d transform,
            double area,
            double ambiguity,
            double tx,
            double ty,
            TargetCorners corners,
            double timestamp,
            double confidence,
            double poseError) {
        this.id = id;
        this.transform = transform;
        this.area = area;
        this.ambiguity = ambiguity;
        this.tx = tx;
        this.ty = ty;
        this.corners = corners;
        this.timestamp = timestamp;
        this.confidence = confidence;
        this.poseError = poseError;
    }

    public AprilTagObservation(
            int id,
            Transform3d transform,
            double area,
            double ambiguity,
            double tx,
            double ty,
            Translation2d[] corners,
            double timestamp,
            double confidence,
            double poseError) {
        this(
                id,
                transform,
                area,
                ambiguity,
                tx,
                ty,
                TargetCorners.fromTranslation2dArray(corners),
                timestamp,
                confidence,
                poseError);
    }

    public AprilTagObservation(
            int id,
            Transform3d transform,
            double area,
            double ambiguity,
            double tx,
            double ty,
            Translation2d[] corners,
            double timestamp) {
        this(
                id,
                transform,
                area,
                ambiguity,
                tx,
                ty,
                TargetCorners.fromTranslation2dArray(corners),
                timestamp,
                1.0,
                0.0);
    }

    public AprilTagObservation(
            int id,
            Transform3d transform,
            double area,
            double ambiguity,
            double tx,
            double ty,
            TargetCorner[] corners,
            double timestamp,
            double confidence,
            double poseError) {
        this(
                id,
                transform,
                area,
                ambiguity,
                tx,
                ty,
                TargetCorners.fromArray(corners),
                timestamp,
                confidence,
                poseError);
    }

    public static AprilTagObservation fromLimelight(
            LimelightTarget_Fiducial target, double timestamp) {
        Transform3d transform =
                new Transform3d(
                        target.getTargetPose_CameraSpace().getTranslation(),
                        target.getTargetPose_CameraSpace().getRotation());

        TargetCorner[] cornersArray = new TargetCorner[4];
        for (int i = 0; i < 4; i++) {
            cornersArray[i] = new TargetCorner();
        }
        TargetCorners corners = TargetCorners.fromArray(cornersArray);

        double ambiguity = 0.1;

        double area = target.ta;

        double tx = target.tx;
        double ty = target.ty;

        double confidence = 0.9;

        double poseError = 0.05;
        double distance = transform.getTranslation().getNorm();
        poseError += distance * 0.01;

        return new AprilTagObservation(
                (int) target.fiducialID,
                transform,
                area,
                ambiguity,
                tx,
                ty,
                corners,
                timestamp,
                confidence,
                poseError);
    }

    public static AprilTagObservation fromLimelight(RawFiducial fiducial, double timestamp) {
        double confidence = Math.max(0.0, 1.0 - fiducial.ambiguity);

        Transform3d estimatedTransform = null;
        if (fiducial.distToCamera > 0) {
            Translation3d translation = new Translation3d(0, 0, fiducial.distToCamera);
            estimatedTransform =
                    new Transform3d(translation, new edu.wpi.first.math.geometry.Rotation3d());
        }

        TargetCorner[] cornersArray = new TargetCorner[4];
        for (int i = 0; i < 4; i++) {
            cornersArray[i] = new TargetCorner();
        }
        TargetCorners corners = TargetCorners.fromArray(cornersArray);

        return new AprilTagObservation(
                fiducial.id,
                estimatedTransform,
                fiducial.ta,
                fiducial.ambiguity,
                fiducial.txnc,
                fiducial.tync,
                corners,
                timestamp,
                confidence,
                fiducial.ambiguity * 0.1);
    }

    public static AprilTagObservation fromLimelight(
            LimelightHelpers.PoseEstimate poseEstimate, int tagId) {
        if (poseEstimate == null || poseEstimate.rawFiducials == null) {
            return null;
        }

        for (RawFiducial fiducial : poseEstimate.rawFiducials) {
            if (fiducial.id == tagId) {
                return fromLimelight(fiducial, poseEstimate.timestampSeconds);
            }
        }

        return null;
    }

    public static AprilTagObservation fromPhotonVision(PhotonTrackedTarget target, double timestamp) {
        Transform3d transform = target.getBestCameraToTarget();

        TargetCorner[] cornersArray = new TargetCorner[4];
        var photonCorners = target.getDetectedCorners();
        for (int i = 0; i < Math.min(4, photonCorners.size()); i++) {
            var corner = photonCorners.get(i);
            cornersArray[i] = new TargetCorner(corner.x, corner.y);
        }
        for (int i = photonCorners.size(); i < 4; i++) {
            cornersArray[i] = new TargetCorner();
        }
        TargetCorners corners = TargetCorners.fromArray(cornersArray);

        double ambiguity = target.getPoseAmbiguity() != -1 ? target.getPoseAmbiguity() : 0.0;

        double area = target.getArea();

        double tx = target.getYaw();
        double ty = target.getPitch();

        double confidence = Math.max(0.0, 1.0 - ambiguity);

        double poseError = ambiguity * 0.1;
        if (transform != null) {
            double distance = transform.getTranslation().getNorm();
            poseError += distance * 0.01;
        }

        return new AprilTagObservation(
                target.getFiducialId(),
                transform,
                area,
                ambiguity,
                tx,
                ty,
                corners,
                timestamp,
                confidence,
                poseError);
    }

    public int getId() {
        return id;
    }

    public Transform3d getTransform() {
        return transform;
    }

    public double getArea() {
        return area;
    }

    public double getAmbiguity() {
        return ambiguity;
    }

    public double getTx() {
        return tx;
    }

    public double getTy() {
        return ty;
    }

    public TargetCorners getCorners() {
        return corners;
    }

    public Translation2d[] getCornersAsTranslation2dArray() {
        return corners.toTranslation2dArray();
    }

    public TargetCorner[] getCornersArray() {
        return corners.toArray();
    }

    public double getTimestamp() {
        return timestamp;
    }

    public double getConfidence() {
        return confidence;
    }

    public double getPoseError() {
        return poseError;
    }

    public double getDistance() {
        if (distance == null) {
            distance = transform != null ? transform.getTranslation().getNorm() : 0.0;
        }
        return distance;
    }

    public double get2dDistance() {
        if (distance2d == null) {
            distance2d = transform != null ? transform.getTranslation().toTranslation2d().getNorm() : 0.0;
        }
        return distance2d;
    }

    public double getLatency() {
        return Timer.getFPGATimestamp() - timestamp;
    }

    public Optional<Pose3d> getTagPose() {
        return transform != null
                ? Optional.of(new Pose3d(transform.getTranslation(), transform.getRotation()))
                : Optional.empty();
    }

    public AprilTagObservation withTimestamp(double newTimestamp) {
        return new AprilTagObservation(
                id, transform, area, ambiguity, tx, ty, corners, newTimestamp, confidence, poseError);
    }

    @Override
    public String toString() {
        return String.format(
                "AprilTagObservation[id=%d, tx=%.2f°, ty=%.2f°, area=%.2f%%, dist=%.2fm, ambiguity=%.3f, "
                        + "confidence=%.2f, poseError=%.3fm, latency=%.1fms]",
                id, tx, ty, area, getDistance(), ambiguity, confidence, poseError, getLatency() * 1000);
    }

    public static final AprilTagObservationStruct struct = new AprilTagObservationStruct();
}