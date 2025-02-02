package org.frc5687.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.util.Units;
import java.util.EnumMap;
import org.frc5687.robot.util.EpilogueLog;

public class RobotStateManager implements EpilogueLog {
    @Override
    public String getLogBase() {
        return "RobotStateManager";
    }

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

    public static final class Geometry {
        private Geometry() {}

        public static final double ELEVATOR_X_OFFSET = 0.0;
        public static final double ELEVATOR_Y_OFFSET = 0.0;

        public static final double NW_Y_OFFSET = 0.247650;
        public static final double NW_X_OFFSET = 0.184150;

        public static final double SW_Y_OFFSET = 0.247650;
        public static final double SW_X_OFFSET = -0.184150;

        public static final double NE_Y_OFFSET = -0.247650;
        public static final double NE_X_OFFSET = 0.184150;

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

        public static final double INTAKE_ARM_X_OFFSET = 0.0;
        public static final double INTAKE_ARM_Y_OFFSET = 0.0;
        public static final double INTAKE_ARM_Z_OFFSET = 0.0;
    }

    private static final double EPSILON = 1e-6;
    private static RobotStateManager _instance;
    private final EnumMap<RobotCoordinate, Pose3d> _poses;
    private Rotation3d _currentPlatformRotation = new Rotation3d();

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

    public synchronized void updateRobotBase(Pose2d robotPose, Rotation3d rot) {
        _poses.put(RobotCoordinate.ROBOT_BASE, new Pose3d(robotPose.getX(), robotPose.getY(), 0, rot));
    }

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

    public synchronized void updatePlatform(double centerHeight, double pitch, double roll) {
        _currentPlatformRotation = new Rotation3d(roll, pitch, 0);
        updateElevatorTransforms(centerHeight, _currentPlatformRotation);

        Pose3d currentCoralArm = _poses.get(RobotCoordinate.CORAL_ARM_END);
        if (currentCoralArm != null) {
            updateCoralArmTransforms(currentCoralArm.getRotation().getY());
        }

        Pose3d currentAlgaeArm = _poses.get(RobotCoordinate.ALGAE_ARM_END);
        if (currentAlgaeArm != null) {
            updateAlgaeArmTransforms(currentAlgaeArm.getRotation().getY());
        }
    }

    private void updateElevatorTransforms(double heightMeters, Rotation3d platformRotation) {
        // Update stage position with platform rotation
        _poses.put(
                RobotCoordinate.ELEVATOR_STAGE,
                new Pose3d(
                        Geometry.ELEVATOR_X_OFFSET,
                        Geometry.ELEVATOR_Y_OFFSET,
                        Geometry.ELEVATOR_STAGE_ONE_HEIGHT + heightMeters,
                        platformRotation));

        // Update top position, maintaining platform rotation
        _poses.put(
                RobotCoordinate.ELEVATOR_TOP,
                new Pose3d(
                        Geometry.ELEVATOR_X_OFFSET,
                        Geometry.ELEVATOR_Y_OFFSET,
                        Geometry.ELEVATOR_STAGE_TWO_HEIGHT + (heightMeters * 2.0),
                        platformRotation));
    }

    private void updateCoralArmTransforms(double armAngleRadians) {
        // Combine platform rotation with arm angle
        Rotation3d armRotation =
                new Rotation3d(
                        _currentPlatformRotation.getX(), armAngleRadians, _currentPlatformRotation.getZ());

        double cosAngle = Math.cos(armAngleRadians);
        double sinAngle = Math.sin(armAngleRadians);

        Pose3d elevatorTop = _poses.get(RobotCoordinate.ELEVATOR_TOP);
        double elevatorZ = elevatorTop.getZ();

        // Update arm base with platform rotation
        _poses.put(
                RobotCoordinate.CORAL_ARM_BASE,
                new Pose3d(
                        Geometry.ELEVATOR_X_OFFSET + Geometry.CORAL_ARM_X_OFFSET,
                        Geometry.ELEVATOR_Y_OFFSET + Geometry.CORAL_ARM_Y_OFFSET,
                        elevatorZ + Geometry.CORAL_ARM_Z_OFFSET,
                        armRotation));

        // Update arm end with combined rotation
        _poses.put(
                RobotCoordinate.CORAL_ARM_END,
                new Pose3d(
                        Geometry.ELEVATOR_X_OFFSET
                                + Geometry.CORAL_ARM_X_OFFSET
                                + Geometry.CORAL_ARM_LENGTH * cosAngle,
                        Geometry.ELEVATOR_Y_OFFSET + Geometry.CORAL_ARM_Y_OFFSET,
                        elevatorZ + Geometry.CORAL_ARM_Z_OFFSET + Geometry.CORAL_ARM_LENGTH * sinAngle,
                        armRotation));
    }

    private void updateAlgaeArmTransforms(double armAngleRadians) {
        // Combine platform rotation with arm angle
        Rotation3d armRotation =
                new Rotation3d(
                        _currentPlatformRotation.getX(), armAngleRadians, _currentPlatformRotation.getZ());

        double cosAngle = Math.cos(armAngleRadians);
        double sinAngle = Math.sin(armAngleRadians);

        Pose3d elevatorTop = _poses.get(RobotCoordinate.ELEVATOR_TOP);
        double elevatorZ = elevatorTop.getZ();

        // Update arm base with platform rotation
        _poses.put(
                RobotCoordinate.ALGAE_ARM_BASE,
                new Pose3d(
                        Geometry.ELEVATOR_X_OFFSET + Geometry.ALGAE_ARM_X_OFFSET,
                        Geometry.ELEVATOR_Y_OFFSET + Geometry.ALGAE_ARM_Y_OFFSET,
                        elevatorZ + Geometry.ALGAE_ARM_Z_OFFSET,
                        armRotation));

        // Update arm end with combined rotation
        _poses.put(
                RobotCoordinate.ALGAE_ARM_END,
                new Pose3d(
                        Geometry.ELEVATOR_X_OFFSET
                                + Geometry.ALGAE_ARM_X_OFFSET
                                + Geometry.ALGAE_ARM_LENGTH * cosAngle,
                        Geometry.ELEVATOR_Y_OFFSET + Geometry.ALGAE_ARM_Y_OFFSET,
                        elevatorZ + Geometry.ALGAE_ARM_Z_OFFSET + Geometry.ALGAE_ARM_LENGTH * sinAngle,
                        armRotation));
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

    public Rotation3d getPlatformRotation() {
        return _currentPlatformRotation;
    }

    public void logComponentPoses() {
        Pose3d[] componentPoses = {
            getPose(RobotCoordinate.ELEVATOR_STAGE),
            getPose(RobotCoordinate.ELEVATOR_TOP),
            getPose(RobotCoordinate.CORAL_ARM_BASE),
            getPose(RobotCoordinate.ALGAE_ARM_BASE)
        };
        log("Components", componentPoses, Pose3d.struct);
    }
}
