package org.frc5687.robot;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.util.Units;
import java.util.EnumMap;

public class RobotStateManager {
    public enum RobotCoordinate {
        ROBOT_BASE,
        ELEVATOR_BASE,
        ELEVATOR_STAGE,
        ELEVATOR_TOP,
        CORAL_ARM_BASE,
        CORAL_ARM_END,
        ALGAE_ARM_BASE,
        ALGAE_ARM_END
    }

    // https://docs.wpilib.org/en/stable/docs/software/basic-programming/coordinate-system.html#wpilib-coordinate-system
    // Read for rotaions

    public static final class Geometry {
        private Geometry() {}

        public static final double ELEVATOR_X_OFFSET = 0.0;
        public static final double ELEVATOR_Y_OFFSET = 0.0;

        private static final double NW_Y_OFFSET = 0.247650;
        private static final double NW_X_OFFSET = 0.184150;

        private static final double SW_Y_OFFSET = 0.247650;
        private static final double SW_X_OFFSET = -0.184150;

        private static final double NE_Y_OFFSET = -0.247650;
        private static final double NE_X_OFFSET = 0.184150;

        public static final double ELEVATOR_STAGE_ONE_HEIGHT = 0.193675; // m
        public static final double ELEVATOR_STAGE_TWO_HEIGHT = 0.253429; // m
        public static final double ELEVATOR_MAX_HEIGHT = 1.459103;

        public static final double CORAL_ARM_X_OFFSET = 0.171450;
        public static final double CORAL_ARM_Y_OFFSET = -0.0762;
        public static final double CORAL_ARM_Z_OFFSET = 0.178689;
        public static final double CORAL_ARM_LENGTH = Units.inchesToMeters(12);

        public static final double ALGAE_ARM_X_OFFSET = 0.171450;
        public static final double ALGAE_ARM_Y_OFFSET = 0.0762;
        public static final double ALGAE_ARM_Z_OFFSET = 0.178689;
        public static final double ALGAE_ARM_LENGTH = Units.inchesToMeters(12);
    }

    private static final double EPSILON = 1e-6;
    private static RobotStateManager _instance;
    private final EnumMap<RobotCoordinate, Pose3d> _poses;

    private RobotStateManager() {
        _poses = new EnumMap<>(RobotCoordinate.class);
        _poses.put(RobotCoordinate.ROBOT_BASE, new Pose3d());
        _poses.put(
                RobotCoordinate.ELEVATOR_BASE,
                new Pose3d(
                        Geometry.ELEVATOR_X_OFFSET,
                        Geometry.ELEVATOR_Y_OFFSET,
                        Geometry.ELEVATOR_STAGE_ONE_HEIGHT,
                        new Rotation3d()));
        updatePlatform(0.0, 0.0, 0.0);
    }

    public static synchronized RobotStateManager getInstance() {
        if (_instance == null) {
            _instance = new RobotStateManager();
        }
        return _instance;
    }

    public synchronized void updateRobotBase(Pose2d robotPose) {
        _poses.put(RobotCoordinate.ROBOT_BASE, new Pose3d(robotPose));
    }

    public synchronized void updatePlatform(double nwHeight, double neHeight, double swHeight) {
        double centerHeight = (nwHeight + neHeight + swHeight) / 3.0;

        var northWestToNorthEast =
                VecBuilder.fill(
                        Geometry.NE_X_OFFSET - Geometry.NW_X_OFFSET,
                        Geometry.NE_Y_OFFSET - Geometry.NW_Y_OFFSET,
                        neHeight - nwHeight);

        var northWestToSouthWest =
                VecBuilder.fill(
                        Geometry.SW_X_OFFSET - Geometry.NW_X_OFFSET,
                        Geometry.SW_Y_OFFSET - Geometry.NW_Y_OFFSET,
                        swHeight - nwHeight);

        var normalVec = Vector.cross(northWestToNorthEast, northWestToSouthWest);
        normalVec = normalVec.div(normalVec.norm());
        Rotation3d rotation = new Rotation3d(VecBuilder.fill(0, 0, 0), normalVec);

        updateElevatorTransforms(centerHeight, rotation);
        // Update dependent mechanisms
        Pose3d currentCoralArm = _poses.get(RobotCoordinate.CORAL_ARM_END);
        if (currentCoralArm != null) {
            updateCoralArmTransforms(currentCoralArm.getRotation().getY());
        }

        Pose3d currentAlgaeArm = _poses.get(RobotCoordinate.ALGAE_ARM_END);
        if (currentAlgaeArm != null) {
            updateAlgaeArmTransforms(currentAlgaeArm.getRotation().getY());
        }
    }

    // public synchronized void updateElevator(double heightMeters) {
    //     Pose3d currentStage = _poses.get(RobotCoordinate.ELEVATOR_STAGE);
    //     if (currentStage != null && Math.abs(currentStage.getZ() - heightMeters) < EPSILON) {
    //         return;
    //     }

    //     updateElevatorTransforms(heightMeters);

    //     Pose3d currentCoralArm = _poses.get(RobotCoordinate.CORAL_ARM_END);
    //     if (currentCoralArm != null) {
    //         updateCoralArmTransforms(currentCoralArm.getRotation().getY());
    //     }

    //     Pose3d currentAlgaeArm = _poses.get(RobotCoordinate.ALGAE_ARM_END);
    //     if (currentAlgaeArm != null) {
    //         updateAlgaeArmTransforms(currentCoralArm.getRotation().getY());
    //     }
    // }

    public synchronized void updateCoralArm(double angleRadians) {
        Pose3d currentArm = _poses.get(RobotCoordinate.CORAL_ARM_END);
        if (currentArm != null && Math.abs(currentArm.getRotation().getY() - angleRadians) < EPSILON) {
            return;
        }
        updateCoralArmTransforms(angleRadians);
    }

    public synchronized void updateAlgaeArm(double angleRadians) {
        Pose3d currentArm = _poses.get(RobotCoordinate.ALGAE_ARM_END);
        if (currentArm != null && Math.abs(currentArm.getRotation().getY() - angleRadians) < EPSILON) {
            return;
        }
        updateAlgaeArmTransforms(angleRadians);
    }

    private void updateElevatorTransforms(double heightMeters, Rotation3d rot) {
        _poses.put(
                RobotCoordinate.ELEVATOR_STAGE,
                new Pose3d(
                        Geometry.ELEVATOR_X_OFFSET,
                        Geometry.ELEVATOR_Y_OFFSET,
                        Geometry.ELEVATOR_STAGE_ONE_HEIGHT + heightMeters,
                        rot));

        _poses.put(
                RobotCoordinate.ELEVATOR_TOP,
                new Pose3d(
                        Geometry.ELEVATOR_X_OFFSET,
                        Geometry.ELEVATOR_Y_OFFSET,
                        Geometry.ELEVATOR_STAGE_TWO_HEIGHT + (heightMeters * 2.0),
                        rot));
    }

    private void updateCoralArmTransforms(double angleRadians) {
        double cosAngle = Math.cos(angleRadians);
        double sinAngle = Math.sin(angleRadians);
        Rotation3d rotation = new Rotation3d(0, angleRadians, 0);

        Pose3d elevatorTop = _poses.get(RobotCoordinate.ELEVATOR_TOP);
        double elevatorZ = elevatorTop.getZ();

        _poses.put(
                RobotCoordinate.CORAL_ARM_BASE,
                new Pose3d(
                        Geometry.ELEVATOR_X_OFFSET + Geometry.CORAL_ARM_X_OFFSET,
                        Geometry.ELEVATOR_Y_OFFSET + Geometry.CORAL_ARM_Y_OFFSET,
                        elevatorZ + Geometry.CORAL_ARM_Z_OFFSET,
                        rotation));

        _poses.put(
                RobotCoordinate.CORAL_ARM_END,
                new Pose3d(
                        Geometry.ELEVATOR_X_OFFSET
                                + Geometry.CORAL_ARM_X_OFFSET
                                + Geometry.CORAL_ARM_LENGTH * cosAngle,
                        Geometry.ELEVATOR_Y_OFFSET + Geometry.CORAL_ARM_Y_OFFSET,
                        elevatorZ + Geometry.CORAL_ARM_Z_OFFSET + Geometry.CORAL_ARM_LENGTH * sinAngle,
                        rotation));
    }

    private void updateAlgaeArmTransforms(double angleRadians) {
        double cosAngle = Math.cos(angleRadians);
        double sinAngle = Math.sin(angleRadians);
        Rotation3d rotation = new Rotation3d(0, angleRadians, 0);

        Pose3d elevatorTop = _poses.get(RobotCoordinate.ELEVATOR_TOP);
        double elevatorZ = elevatorTop.getZ();

        _poses.put(
                RobotCoordinate.ALGAE_ARM_BASE,
                new Pose3d(
                        Geometry.ELEVATOR_X_OFFSET + Geometry.ALGAE_ARM_X_OFFSET,
                        Geometry.ELEVATOR_Y_OFFSET + Geometry.ALGAE_ARM_Y_OFFSET,
                        elevatorZ + Geometry.ALGAE_ARM_Z_OFFSET,
                        rotation));

        _poses.put(
                RobotCoordinate.ALGAE_ARM_END,
                new Pose3d(
                        Geometry.ELEVATOR_X_OFFSET
                                + Geometry.ALGAE_ARM_X_OFFSET
                                + Geometry.ALGAE_ARM_LENGTH * cosAngle,
                        Geometry.ELEVATOR_Y_OFFSET + Geometry.ALGAE_ARM_Y_OFFSET,
                        elevatorZ + Geometry.ALGAE_ARM_Z_OFFSET + Geometry.ALGAE_ARM_LENGTH * sinAngle,
                        rotation));
    }

    public Pose3d getPose(RobotCoordinate coordinate) {
        return _poses.get(coordinate);
    }

    public Transform3d getTransform(RobotCoordinate fromFrame, RobotCoordinate toFrame) {
        if (fromFrame == toFrame) {
            return new Transform3d();
        }
        return new Transform3d(_poses.get(fromFrame), _poses.get(toFrame));
    }
}
