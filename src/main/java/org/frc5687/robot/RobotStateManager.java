package org.frc5687.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotBase;
import java.util.EnumMap;
import java.util.function.Supplier;
import org.frc5687.robot.subsystems.drive.DriveInputs;
import org.frc5687.robot.subsystems.vision.RobotPoseEstimate;
import org.frc5687.robot.util.EpilogueLog;
import org.frc5687.robot.util.PoseEstimator;
import org.frc5687.robot.util.QuestNav;
import org.frc5687.robot.util.WheelOdometrySource;
import org.frc5687.robot.util.estimator.PoseTracker;
import org.frc5687.robot.util.estimator.SwerveOdometry;

public class RobotStateManager implements EpilogueLog {
    @Override
    public String getLogBase() {
        return "RobotStateManager";
    }

    public enum RobotCoordinate {
        ROBOT_BASE_SWERVE,
        ROBOT_BASE_QUESTNAV,
        ROBOT_BASE_SIM_ODOM,
        ELEVATOR_BASE,
        ELEVATOR_STAGE,
        ELEVATOR_TOP,
        CORAL_ARM_BASE,
        CORAL_ARM_END,
        ALGAE_ARM_BASE,
        ALGAE_ARM_END,
        INTAKE_ARM_BASE,
        INTAKE_ARM_END
    }

    public static final class Geometry {
        private Geometry() {}

        public static final double ELEVATOR_X_OFFSET = 0.127000;
        public static final double ELEVATOR_Y_OFFSET = 0.0;

        public static final double ELEVATOR_STAGE_ONE_HEIGHT = 0.165639;
        public static final double ELEVATOR_STAGE_ONE_MAX_HEIGHT = 0.851439;
        public static final double ELEVATOR_STAGE_TWO_HEIGHT = 0.191039; // m
        public static final double ELEVATOR_STAGE_TWO_OFFSET = Units.inchesToMeters(6.0);

        public static final double CORAL_ARM_X_OFFSET = 0.260350;
        public static final double CORAL_ARM_Y_OFFSET = 0.145;
        public static final double CORAL_ARM_Z_OFFSET = 0.381000;
        public static final double CORAL_ARM_LENGTH = Units.inchesToMeters(7.0);

        public static final double ALGAE_ARM_X_OFFSET = 0.260350;
        public static final double ALGAE_ARM_Y_OFFSET = -0.108600; // probably close to real
        public static final double ALGAE_ARM_Z_OFFSET = 0.381; // this is from elevator stage 2 height
        public static final double ALGAE_ARM_LENGTH = Units.inchesToMeters(12);

        public static final double INTAKE_ARM_X_OFFSET = -0.330200;
        public static final double INTAKE_ARM_Y_OFFSET = 0.03;
        public static final double INTAKE_ARM_Z_OFFSET = 0.193675;
        public static final double INTAKE_ARM_LENGTH = Units.inchesToMeters(12);
    }

    private static final double EPSILON = 1e-6;
    private static RobotStateManager _instance;
    private final EnumMap<RobotCoordinate, Pose3d> _poses;
    private Rotation3d _currentPlatformRotation = new Rotation3d();

    // New pose trackers
    private PoseTracker _swervePoseTracker;
    private PoseTracker _simPoseTracker;
    private PoseEstimator _estimator;

    private final DriveInputs _swerveInputs = new DriveInputs();
    private final DriveInputs _simInputs = new DriveInputs();

    private Supplier<Rotation2d> _imuRotation;
    private Supplier<SwerveModulePosition[]> _modulePositionSupplier;

    private SwerveOdometry _odom;
    private QuestNav _nav;

    public boolean _questVisionUpdatesOn;

    private RobotStateManager() {
        _poses = new EnumMap<>(RobotCoordinate.class);
        _poses.put(RobotCoordinate.ROBOT_BASE_SWERVE, new Pose3d());
        _poses.put(RobotCoordinate.ROBOT_BASE_QUESTNAV, new Pose3d());
        _poses.put(RobotCoordinate.ROBOT_BASE_SIM_ODOM, new Pose3d());
        _poses.put(
                RobotCoordinate.ELEVATOR_BASE,
                new Pose3d(
                        Geometry.ELEVATOR_X_OFFSET,
                        Geometry.ELEVATOR_Y_OFFSET,
                        Geometry.ELEVATOR_STAGE_ONE_HEIGHT,
                        new Rotation3d()));
        updatePlatform(0.0);
        _questVisionUpdatesOn = false;
    }

    public static synchronized RobotStateManager getInstance() {
        if (_instance == null) {
            _instance = new RobotStateManager();
        }
        return _instance;
    }

    public void initEstimators(
            Supplier<SwerveModulePosition[]> positionSupplier,
            Supplier<Rotation2d> headingSupplier,
            Supplier<ChassisSpeeds> chassisSpeedsSupplier,
            QuestNav nav) {

        _imuRotation = headingSupplier;
        _modulePositionSupplier = positionSupplier;
        _nav = nav;

        _odom = new SwerveOdometry(Constants.DriveTrain.MODULE_LOCATIONS);
        _estimator = new PoseEstimator(new WheelOdometrySource(positionSupplier, headingSupplier));
        // Initialize the swerve pose tracker
        _swervePoseTracker = new PoseTracker(Constants.DriveTrain.MODULE_LOCATIONS, _swerveInputs);

        // For simulation, create a separate tracker
        if (RobotBase.isSimulation()) {
            _simPoseTracker = new PoseTracker(Constants.DriveTrain.MODULE_LOCATIONS, _simInputs);
        }
    }

    public synchronized void updateOdometry() {
        // Update swerve inputs from suppliers
        _odom.update(_swerveInputs);
        updateInputs(_swerveInputs);
        log("SkidOdom", _odom.getPose(), Pose2d.struct);
        Pose2d robotPose = _estimator.getEstimatedPose();
        _estimator.updateOdometry();
        _poses.put(
                RobotCoordinate.ROBOT_BASE_SWERVE,
                new Pose3d(
                        robotPose.getX(),
                        robotPose.getY(),
                        0,
                        new Rotation3d(0, 0, robotPose.getRotation().getRadians())));
        log("Old Estimator", _estimator.getEstimatedPose(), Pose2d.struct);

        // if (_swervePoseTracker != null) {
        //     _swervePoseTracker.update();
        //     Pose2d robotPose = _swervePoseTracker.getPose();
        //     _poses.put(
        //             RobotCoordinate.ROBOT_BASE_SWERVE,
        //             new Pose3d(
        //                     robotPose.getX(),
        //                     robotPose.getY(),
        //                     0,
        //                     new Rotation3d(0, 0, robotPose.getRotation().getRadians())));
        // }

        // if (_nav != null) {
        //     Pose2d questNavPose = _nav.getEstimatedPose();
        //     _poses.put(
        //             RobotCoordinate.ROBOT_BASE_QUESTNAV,
        //             new Pose3d(
        //                     questNavPose.getX(),
        //                     questNavPose.getY(),
        //                     0,
        //                     new Rotation3d(0, 0, questNavPose.getRotation().getRadians())));
        // }

        // Update sim pose tracker if in simulation
        if (RobotBase.isSimulation() && _simPoseTracker != null) {
            updateInputs(_simInputs);
            _simPoseTracker.update();
            robotPose = _simPoseTracker.getPose();
            _poses.put(
                    RobotCoordinate.ROBOT_BASE_SIM_ODOM,
                    new Pose3d(
                            robotPose.getX(),
                            robotPose.getY(),
                            0,
                            new Rotation3d(0, 0, robotPose.getRotation().getRadians())));
        }
    }

    private void updateInputs(DriveInputs inputs) {
        SwerveModulePosition[] positions = _modulePositionSupplier.get();
        System.arraycopy(
                positions,
                0,
                inputs.modulePositions,
                0,
                Math.min(positions.length, inputs.modulePositions.length));

        for (int i = 0; i < inputs.measuredStates.length && i < positions.length; i++) {
            inputs.measuredStates[i].angle = positions[i].angle;
        }

        Rotation2d heading = _imuRotation.get();
        inputs.yawPosition = heading;
    }

    public synchronized void updateVision(RobotPoseEstimate estimatedRobotPose) {
        // Update the main swerve tracker with vision data
        // if (_swervePoseTracker != null) {
        //     _swervePoseTracker.processVision(estimatedRobotPose);
        // }

        if (_estimator != null) {
            _estimator.addVisionMeasurement(estimatedRobotPose);
        }

        // if (_nav != null && _questVisionUpdatesOn) {
        //     _nav.updateWithVision(estimatedRobotPose);
        // }

        // Update simulation tracker if needed
        if (RobotBase.isSimulation() && _simPoseTracker != null) {
            //    _simPoseTracker.processVision(estimatedRobotPose);
        }
    }

    public synchronized void resetEstimatedPose(Pose2d pose) {
        // Reset the main swerve tracker
        // if (_swervePoseTracker != null) {
        //     _swervePoseTracker.resetPose(pose);
        //     _poses.put(RobotCoordinate.ROBOT_BASE_SWERVE, new Pose3d(pose));
        // }

        if (_estimator != null) {
            _estimator.resetPose(pose);
            _poses.put(RobotCoordinate.ROBOT_BASE_SWERVE, new Pose3d(pose));
        }

        // if (_nav != null) {
        //     _nav.resetPose(pose);
        //     _poses.put(RobotCoordinate.ROBOT_BASE_QUESTNAV, new Pose3d(pose));
        // }

        // Reset simulation if needed
        if (RobotBase.isSimulation() && _simPoseTracker != null) {
            _simPoseTracker.resetPose(pose);
            _poses.put(RobotCoordinate.ROBOT_BASE_SIM_ODOM, new Pose3d(pose));
        }
    }

    public synchronized void updateCoralArm(double angleRadians) {
        Pose3d currentArm = _poses.get(RobotCoordinate.CORAL_ARM_BASE);
        if (currentArm != null && Math.abs(currentArm.getRotation().getY() - angleRadians) < EPSILON) {
            return;
        }
        updateCoralArmTransforms(angleRadians);
    }

    public synchronized void updateAlgaeArm(double angleRadians) {
        Pose3d currentArm = _poses.get(RobotCoordinate.ALGAE_ARM_BASE);
        if (currentArm != null && Math.abs(currentArm.getRotation().getY() - angleRadians) < EPSILON) {
            return;
        }
        updateAlgaeArmTransforms(angleRadians);
    }

    public synchronized void updateIntakeArm(double angleRadians) {
        Pose3d currentArm = _poses.get(RobotCoordinate.INTAKE_ARM_BASE);
        if (currentArm != null && Math.abs(currentArm.getRotation().getY() - angleRadians) < EPSILON) {
            return;
        }
        updateIntakeArmTransforms(angleRadians);
    }

    public synchronized void updatePlatform(double centerHeight) {
        updateElevatorTransforms(centerHeight);

        Pose3d currentCoralArm = _poses.get(RobotCoordinate.CORAL_ARM_END);
        if (currentCoralArm != null) {
            updateCoralArmTransforms(currentCoralArm.getRotation().getY());
        }

        Pose3d currentAlgaeArm = _poses.get(RobotCoordinate.ALGAE_ARM_END);
        if (currentAlgaeArm != null) {
            updateAlgaeArmTransforms(currentAlgaeArm.getRotation().getY());
        }
    }

    private void updateElevatorTransforms(double heightMeters) {
        // Update stage position with platform rotation
        _poses.put(
                RobotCoordinate.ELEVATOR_STAGE,
                new Pose3d(
                        Geometry.ELEVATOR_X_OFFSET,
                        Geometry.ELEVATOR_Y_OFFSET,
                        Geometry.ELEVATOR_STAGE_ONE_HEIGHT + heightMeters,
                        new Rotation3d()));
        _poses.put(
                RobotCoordinate.ELEVATOR_TOP,
                new Pose3d(
                        Geometry.ELEVATOR_X_OFFSET,
                        Geometry.ELEVATOR_Y_OFFSET,
                        Geometry.ELEVATOR_STAGE_TWO_HEIGHT + (heightMeters * 2.0),
                        new Rotation3d()));
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
                        Geometry.CORAL_ARM_X_OFFSET,
                        Geometry.CORAL_ARM_Y_OFFSET,
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
        Rotation3d armRotation = new Rotation3d(0, armAngleRadians, 0);

        double cosAngle = Math.cos(armAngleRadians);
        double sinAngle = Math.sin(armAngleRadians);

        Pose3d elevatorTop = _poses.get(RobotCoordinate.ELEVATOR_TOP);
        double elevatorZ = elevatorTop.getZ();

        // Update arm base with platform rotation
        _poses.put(
                RobotCoordinate.ALGAE_ARM_BASE,
                new Pose3d(
                        Geometry.ALGAE_ARM_X_OFFSET,
                        Geometry.ALGAE_ARM_Y_OFFSET,
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

    private void updateIntakeArmTransforms(double armAngleRadians) {
        Rotation3d armRotation = new Rotation3d(0, armAngleRadians, 0);
        double cosAngle = Math.cos(armAngleRadians);
        double sinAngle = Math.sin(armAngleRadians);

        _poses.put(
                RobotCoordinate.INTAKE_ARM_BASE,
                new Pose3d(
                        Geometry.INTAKE_ARM_X_OFFSET,
                        Geometry.INTAKE_ARM_Y_OFFSET,
                        Geometry.INTAKE_ARM_Z_OFFSET,
                        armRotation));
        _poses.put(
                RobotCoordinate.INTAKE_ARM_END,
                new Pose3d(
                        Geometry.INTAKE_ARM_X_OFFSET + Geometry.INTAKE_ARM_LENGTH * cosAngle,
                        Geometry.INTAKE_ARM_Y_OFFSET,
                        Geometry.INTAKE_ARM_Z_OFFSET + Geometry.INTAKE_ARM_LENGTH * sinAngle,
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

    public Rotation2d getRawIMURotation() {
        return _imuRotation.get();
    }

    public Rotation3d getPlatformRotation() {
        return _currentPlatformRotation;
    }

    public double getPoseFOM() {
        if (_swervePoseTracker != null) {
            return _swervePoseTracker.getFOM();
        }
        return Double.MAX_VALUE;
    }

    public boolean isPoseReliable() {
        if (_swervePoseTracker != null) {
            return _swervePoseTracker.isPoseReliable();
        }
        return false;
    }

    public boolean isSkidding() {
        if (_swervePoseTracker != null) {
            return _swervePoseTracker.isSkidding();
        }
        return false;
    }

    public boolean isInCollision() {
        if (_swervePoseTracker != null) {
            return _swervePoseTracker.isInCollisionState();
        }
        return false;
    }

    public void logComponentPoses() {
        Pose3d[] componentPoses = {
            getPose(RobotCoordinate.ELEVATOR_STAGE),
            getPose(RobotCoordinate.ELEVATOR_TOP),
            getPose(RobotCoordinate.CORAL_ARM_BASE),
            getPose(RobotCoordinate.ALGAE_ARM_BASE),
            getPose(RobotCoordinate.INTAKE_ARM_BASE)
        };
        log("Components", componentPoses, Pose3d.struct);
    }

    public void logEstimatedPoses() {
        log("Quest Estimator Pose", getPose(RobotCoordinate.ROBOT_BASE_QUESTNAV), Pose3d.struct);
        log("Swerve Estimator Pose", getPose(RobotCoordinate.ROBOT_BASE_SWERVE), Pose3d.struct);
        log("Sim Odom Pose", getPose(RobotCoordinate.ROBOT_BASE_SIM_ODOM), Pose3d.struct);

        // Log additional metrics
        if (_swervePoseTracker != null) {
            log("Swerve FOM", getPoseFOM());
            log("Swerve Reliable", isPoseReliable());
            log("Swerve Skidding", isSkidding());
            log("Swerve Collision", isInCollision());
        }
    }
}
