package org.frc5687.robot.commands.drive;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import org.frc5687.robot.RobotStateManager;
import org.frc5687.robot.RobotStateManager.RobotCoordinate;
import org.frc5687.robot.subsystems.drive.DriveSubsystem;
import org.frc5687.robot.subsystems.vision.VisionSubsystem;
import org.frc5687.robot.util.vision.AlgaeTracker;

public class DriveToGroundAlgae extends DriveToPose {
    private final DriveSubsystem _drive;
    private final VisionSubsystem _vision;

    public DriveToGroundAlgae(DriveSubsystem drive, VisionSubsystem vision) {
        super(drive, ()->{
            var robotPose =
                RobotStateManager.getInstance().getPose(RobotCoordinate.ROBOT_BASE_SWERVE).toPose2d();
            var detection = AlgaeTracker.getInstance().getClosestAlgae(robotPose.getTranslation());
            if (detection.isEmpty()) {
                return robotPose;
            }
            return new Pose2d(detection.get(), detection.get().minus(robotPose.getTranslation()).getAngle());
        });
        _thetaController.setPID(8, 0, 0);
        _drive = drive;
        _vision = vision;
        addRequirements(_drive);
    }

    @Override
    public void initialize() {
        _vision.setPipelineIndex("limelight-center", 1);
    }

    @Override
    public void end(boolean interrupted) {
        _vision.setPipelineIndex("limelight-center", 0);
    }
}
