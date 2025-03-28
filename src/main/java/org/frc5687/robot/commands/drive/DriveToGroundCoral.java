package org.frc5687.robot.commands.drive;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import java.util.Optional;
import org.frc5687.robot.RobotStateManager;
import org.frc5687.robot.RobotStateManager.RobotCoordinate;
import org.frc5687.robot.subsystems.drive.DriveSubsystem;
import org.frc5687.robot.subsystems.vision.VisionSubsystem;
import org.frc5687.robot.util.FieldConstants;
import org.frc5687.robot.util.TunableDouble;

public class DriveToGroundCoral extends DriveToPoseSmooth {
    private final DriveSubsystem _drive;
    private final VisionSubsystem _vision;

    private static final TunableDouble xOffset =
            new TunableDouble("DriveToGroundCoral", "xOffset", -0.6);
    private static final TunableDouble yOffset =
            new TunableDouble("DriveToGroundCoral", "yOffset", Units.inchesToMeters(5));

    public DriveToGroundCoral(DriveSubsystem drive, VisionSubsystem vision) {
        super(
                drive,
                () -> {
                    var robotPose =
                            RobotStateManager.getInstance().getPose(RobotCoordinate.ROBOT_BASE_SWERVE).toPose2d();
                    // var detection = CoralTracker.getInstance().getClosestCoral(robotPose.getTranslation());
                    var detection =
                            Optional.of(FieldConstants.StagingPositions.rightIceCream.getTranslation());
                    if (detection.isEmpty()) {
                        return robotPose;
                    }
                    double xErr = detection.get().getX() - robotPose.getX();
                    double yErr = detection.get().getY() - robotPose.getY();
                    Rotation2d theta = new Rotation2d(xErr, yErr).plus(Rotation2d.k180deg);
                    Translation2d robotToAlgaeArm = new Translation2d(xOffset.get(), yOffset.get());

                    Translation2d robotTargetTranslation =
                            detection.get().minus(robotToAlgaeArm.rotateBy(theta));

                    return new Pose2d(robotTargetTranslation, theta);
                });
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
