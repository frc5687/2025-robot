package org.frc5687.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.util.struct.Struct;
import edu.wpi.first.util.struct.StructSerializable;
import edu.wpi.first.wpilibj.Timer;
import java.nio.ByteBuffer;
import org.photonvision.targeting.PhotonTrackedTarget;

// This is to mimic how WPILib create struct seriablizables Like Pose3d, Pose2d etc
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
            return kSizeInt32 + 5 * kSizeDouble + Transform3d.struct.getSize();
        }

        @Override
        public String getSchema() {
            return "int32 id;Transform3d transform;double area;double ambiguity;double yaw;double pitch;double timestamp";
        }

        @Override
        public Struct<?>[] getNested() {
            return new Struct<?>[] {Transform3d.struct};
        }

        @Override
        public AprilTagObservation unpack(ByteBuffer bb) {
            return new AprilTagObservation(
                    bb.getInt(),
                    Transform3d.struct.unpack(bb),
                    bb.getDouble(),
                    bb.getDouble(),
                    bb.getDouble(),
                    bb.getDouble(),
                    bb.getDouble());
        }

        @Override
        public void pack(ByteBuffer bb, AprilTagObservation value) {
            bb.putInt(value.getId());
            Transform3d.struct.pack(bb, value.getTransform());
            bb.putDouble(value.getArea());
            bb.putDouble(value.getAmbiguity());
            bb.putDouble(value.getYaw());
            bb.putDouble(value.getPitch());
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
    private final double timestamp;

    public AprilTagObservation(
            int id,
            Transform3d transform,
            double area,
            double ambiguity,
            double yaw,
            double pitch,
            double timestamp) {
        this.id = id;
        this.transform = transform;
        this.area = area;
        this.ambiguity = ambiguity;
        this.yaw = yaw;
        this.pitch = pitch;
        this.timestamp = timestamp;
    }

    public static AprilTagObservation fromPhotonVision(PhotonTrackedTarget target, double timestamp) {
        return new AprilTagObservation(
                target.getFiducialId(),
                target.getBestCameraToTarget(),
                target.getArea(),
                target.getPoseAmbiguity(),
                target.getYaw(),
                target.getPitch(),
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
