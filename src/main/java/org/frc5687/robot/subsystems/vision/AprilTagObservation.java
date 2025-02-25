package org.frc5687.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.util.struct.Struct;
import edu.wpi.first.util.struct.StructSerializable;
import edu.wpi.first.wpilibj.Timer;
import java.nio.ByteBuffer;
import java.util.List;
import org.photonvision.targeting.PhotonTrackedTarget;
import org.photonvision.targeting.TargetCorner;

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
            return kSizeInt32 + Transform3d.struct.getSize() + 5 * kSizeDouble + 8 * kSizeDouble;
        }

        @Override
        public String getSchema() {
            return "int32 id;Transform3d transform;double area;double ambiguity;double yaw;double pitch;double corners[8];double timestamp";
        }

        @Override
        public Struct<?>[] getNested() {
            return new Struct<?>[] {Transform3d.struct};
        }

        @Override
        public AprilTagObservation unpack(ByteBuffer bb) {
            int id = bb.getInt();
            Transform3d transform = Transform3d.struct.unpack(bb);
            double area = bb.getDouble();
            double ambiguity = bb.getDouble();
            double yaw = bb.getDouble();
            double pitch = bb.getDouble();

            Translation2d[] corners = new Translation2d[4];
            for (int i = 0; i < 4; i++) {
                double x = bb.getDouble();
                double y = bb.getDouble();
                corners[i] = new Translation2d(x, y);
            }

            double timestamp = bb.getDouble();

            return new AprilTagObservation(
                    id, transform, area, ambiguity, yaw, pitch, corners, timestamp);
        }

        @Override
        public void pack(ByteBuffer bb, AprilTagObservation value) {
            bb.putInt(value.getId());
            Transform3d.struct.pack(bb, value.getTransform());
            bb.putDouble(value.getArea());
            bb.putDouble(value.getAmbiguity());
            bb.putDouble(value.getYaw());
            bb.putDouble(value.getPitch());

            Translation2d[] corners = value.getCorners();
            for (int i = 0; i < 4; i++) {
                if (corners != null && i < corners.length) {
                    bb.putDouble(corners[i].getX());
                    bb.putDouble(corners[i].getY());
                } else {
                    bb.putDouble(0.0);
                    bb.putDouble(0.0);
                }
            }

            bb.putDouble(value.getTimestamp());
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
    private final double yaw;
    private final double pitch;
    private final Translation2d[] corners;
    private final double timestamp;

    public AprilTagObservation(
            int id,
            Transform3d transform,
            double area,
            double ambiguity,
            double yaw,
            double pitch,
            Translation2d[] corners,
            double timestamp) {
        this.id = id;
        this.transform = transform;
        this.area = area;
        this.ambiguity = ambiguity;
        this.yaw = yaw;
        this.pitch = pitch;
        this.corners = corners;
        this.timestamp = timestamp;
    }

    public static AprilTagObservation fromPhotonVision(PhotonTrackedTarget target, double timestamp) {
        List<TargetCorner> corners = target.getDetectedCorners();
        Translation2d[] newCorners = new Translation2d[4];
        for (int i = 0; i < Math.min(corners.size(), 4); i++) {
            TargetCorner corner = corners.get(i);
            newCorners[i] = new Translation2d(corner.x, corner.y);
        }

        return new AprilTagObservation(
                target.getFiducialId(),
                target.getBestCameraToTarget(),
                target.getArea(),
                target.getPoseAmbiguity(),
                target.getYaw(),
                target.getPitch(),
                newCorners,
                timestamp);
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

    public double getYaw() {
        return yaw;
    }

    public double getPitch() {
        return pitch;
    }

    public Translation2d[] getCorners() {
        return corners;
    }

    public double getTimestamp() {
        return timestamp;
    }

    public double getDistance() {
        return getTransform().getTranslation().getDistance(Translation3d.kZero);
    }

    public double get2dDistance() {
        return getTransform().getTranslation().toTranslation2d().getDistance(Translation2d.kZero);
    }

    public double getLatency() {
        return Timer.getFPGATimestamp() - timestamp;
    }

    public static final AprilTagObservationStruct struct = new AprilTagObservationStruct();
}
