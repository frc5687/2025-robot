package org.frc5687.robot.util.vision;

import edu.wpi.first.util.struct.Struct;
import edu.wpi.first.util.struct.StructSerializable;
import java.nio.ByteBuffer;

public class TargetCorner implements StructSerializable {
    public static class TargetCornerStruct implements Struct<TargetCorner> {
        @Override
        public Class<TargetCorner> getTypeClass() {
            return TargetCorner.class;
        }

        @Override
        public String getTypeName() {
            return "TargetCorner";
        }

        @Override
        public int getSize() {
            return 2 * kSizeDouble;
        }

        @Override
        public String getSchema() {
            return "double x;double y";
        }

        @Override
        public Struct<?>[] getNested() {
            return new Struct<?>[0];
        }

        @Override
        public TargetCorner unpack(ByteBuffer bb) {
            double x = bb.getDouble();
            double y = bb.getDouble();
            return new TargetCorner(x, y);
        }

        @Override
        public void pack(ByteBuffer bb, TargetCorner value) {
            bb.putDouble(value.getX());
            bb.putDouble(value.getY());
        }

        @Override
        public boolean isImmutable() {
            return true;
        }
    }

    private final double x;
    private final double y;

    public TargetCorner(double x, double y) {
        this.x = x;
        this.y = y;
    }

    public TargetCorner() {
        this(0.0, 0.0);
    }

    public double getX() {
        return x;
    }

    public double getY() {
        return y;
    }

    public double getDistance(TargetCorner other) {
        double dx = x - other.x;
        double dy = y - other.y;
        return Math.sqrt(dx * dx + dy * dy);
    }

    @Override
    public String toString() {
        return String.format("TargetCorner[x=%.2f, y=%.2f]", x, y);
    }

    public static final TargetCornerStruct struct = new TargetCornerStruct();
}
