package org.frc5687.robot.commands.drive;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import org.frc5687.robot.RobotStateManager;
import org.frc5687.robot.RobotStateManager.RobotCoordinate;
import org.frc5687.robot.subsystems.drive.DriveSubsystem;
import org.frc5687.robot.subsystems.vision.VisionSubsystem;
import org.frc5687.robot.util.TunableDouble;
import org.frc5687.robot.util.vision.AlgaeTracker;

public class DriveToGroundAlgae extends DriveToPoseSmooth {
    private final DriveSubsystem _drive;
    private final VisionSubsystem _vision;

    private static final TunableDouble xOffset =
            new TunableDouble("DriveToGroundAlgae", "xOffset", 0);
    private static final TunableDouble yOffset =
            new TunableDouble("DriveToGroundAlgae", "yOffset", 0);

    public DriveToGroundAlgae(DriveSubsystem drive, VisionSubsystem vision) {
        super(
                drive,
                () -> {
                    var robotPose =
                            RobotStateManager.getInstance().getPose(RobotCoordinate.ROBOT_BASE_SWERVE).toPose2d();
                    var detection = AlgaeTracker.getInstance().getClosestAlgae(robotPose.getTranslation());
                    // var detection = Optional.of(Translation2d.kZero); // FIXME remove
                    if (detection.isEmpty()) {
                        return robotPose;
                    }
                    double xErr = detection.get().getX() - robotPose.getX();
                    double yErr = detection.get().getY() - robotPose.getY();
                    Rotation2d theta = new Rotation2d(xErr, yErr);
                    Translation2d robotToAlgaeArm = new Translation2d(xOffset.get(), yOffset.get());

                    Translation2d robotTargetTranslation =
                            detection.get().minus(robotToAlgaeArm.rotateBy(theta));

                    return new Pose2d(robotTargetTranslation, theta);
                });
        _rotationController.setPID(8, 0, 0);
        _drive = drive;
        _vision = vision;
        addRequirements(_drive);
    }

    @Override
    public void initialize() {
        _vision.setPipelineIndex("limelight-center", 1);
        super.initialize();
    }

    @Override
    public void end(boolean interrupted) {
        _vision.setPipelineIndex("limelight-center", 0);
        super.end(interrupted);
    }
}
