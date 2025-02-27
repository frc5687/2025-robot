package org.frc5687.robot.util.vision;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.util.struct.Struct;
import edu.wpi.first.util.struct.StructSerializable;
import java.nio.ByteBuffer;

public class TargetCorners implements StructSerializable {

    public static class TargetCornersStruct implements Struct<TargetCorners> {
        @Override
        public Class<TargetCorners> getTypeClass() {
            return TargetCorners.class;
        }

        @Override
        public String getTypeName() {
            return "TargetCorners";
        }

        @Override
        public int getSize() {
            return 4 * TargetCorner.struct.getSize();
        }

        @Override
        public String getSchema() {
            return "TargetCorner topLeft;TargetCorner topRight;"
                    + "TargetCorner bottomRight;TargetCorner bottomLeft";
        }

        @Override
        public Struct<?>[] getNested() {
            return new Struct<?>[] {TargetCorner.struct};
        }

        @Override
        public TargetCorners unpack(ByteBuffer bb) {
            TargetCorner topLeft = TargetCorner.struct.unpack(bb);
            TargetCorner topRight = TargetCorner.struct.unpack(bb);
            TargetCorner bottomRight = TargetCorner.struct.unpack(bb);
            TargetCorner bottomLeft = TargetCorner.struct.unpack(bb);

            return new TargetCorners(topLeft, topRight, bottomRight, bottomLeft);
        }

        @Override
        public void pack(ByteBuffer bb, TargetCorners value) {
            TargetCorner.struct.pack(bb, value.topLeft);
            TargetCorner.struct.pack(bb, value.topRight);
            TargetCorner.struct.pack(bb, value.bottomRight);
            TargetCorner.struct.pack(bb, value.bottomLeft);
        }

        @Override
        public boolean isImmutable() {
            return true;
        }
    }

    public final TargetCorner topLeft;
    public final TargetCorner topRight;
    public final TargetCorner bottomRight;
    public final TargetCorner bottomLeft;

    public TargetCorners(
            TargetCorner topLeft,
            TargetCorner topRight,
            TargetCorner bottomRight,
            TargetCorner bottomLeft) {
        this.topLeft = topLeft;
        this.topRight = topRight;
        this.bottomRight = bottomRight;
        this.bottomLeft = bottomLeft;
    }

    public static TargetCorners fromTranslation2dArray(Translation2d[] corners) {
        if (corners == null || corners.length < 4) {
            return new TargetCorners(
                    new TargetCorner(), new TargetCorner(), new TargetCorner(), new TargetCorner());
        }

        return new TargetCorners(
                new TargetCorner(corners[0].getX(), corners[0].getY()),
                new TargetCorner(corners[1].getX(), corners[1].getY()),
                new TargetCorner(corners[2].getX(), corners[2].getY()),
                new TargetCorner(corners[3].getX(), corners[3].getY()));
    }

    public static TargetCorners fromArray(TargetCorner[] corners) {
        if (corners == null || corners.length < 4) {
            return new TargetCorners(
                    new TargetCorner(), new TargetCorner(), new TargetCorner(), new TargetCorner());
        }

        return new TargetCorners(corners[0], corners[1], corners[2], corners[3]);
    }

    public TargetCorner[] toArray() {
        return new TargetCorner[] {topLeft, topRight, bottomRight, bottomLeft};
    }

    public Translation2d[] toTranslation2dArray() {
        return new Translation2d[] {
            new Translation2d(topLeft.getX(), topLeft.getY()),
            new Translation2d(topRight.getX(), topRight.getY()),
            new Translation2d(bottomRight.getX(), bottomRight.getY()),
            new Translation2d(bottomLeft.getX(), bottomLeft.getY())
        };
    }

    public TargetCorner getCenter() {
        double centerX =
                (topLeft.getX() + topRight.getX() + bottomRight.getX() + bottomLeft.getX()) / 4.0;
        double centerY =
                (topLeft.getY() + topRight.getY() + bottomRight.getY() + bottomLeft.getY()) / 4.0;
        return new TargetCorner(centerX, centerY);
    }

    public double getWidth() {
        double topWidth = topLeft.getDistance(topRight);
        double bottomWidth = bottomLeft.getDistance(bottomRight);
        return (topWidth + bottomWidth) / 2.0;
    }

    public double getHeight() {
        double leftHeight = topLeft.getDistance(bottomLeft);
        double rightHeight = topRight.getDistance(bottomRight);
        return (leftHeight + rightHeight) / 2.0;
    }

    public static final TargetCornersStruct struct = new TargetCornersStruct();
}