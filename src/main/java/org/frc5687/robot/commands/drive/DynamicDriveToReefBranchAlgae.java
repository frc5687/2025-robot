package org.frc5687.robot.commands.drive;

import com.pathplanner.lib.util.FlippingUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import org.frc5687.robot.subsystems.drive.DriveSubsystem;
import org.frc5687.robot.util.ReefAlignmentHelpers;
import org.frc5687.robot.util.ReefAlignmentHelpers.ReefSide;

public class DynamicDriveToReefBranchAlgae extends DriveWithNormalVectorAlignment {
    public DynamicDriveToReefBranchAlgae(DriveSubsystem drive, ReefSide side) {
        super(
                drive,
                () -> {
                    Pose2d robotPose = drive.getPose();

                    var alliance = DriverStation.getAlliance();
                    if (alliance.isPresent() && alliance.get() == Alliance.Red) {
                        Pose2d mirroredPose = FlippingUtil.flipFieldPose(robotPose);
                        int currentFace = ReefAlignmentHelpers.calculateBestFace(mirroredPose);
                        Pose2d mirroredTargetPose = ReefAlignmentHelpers.calculateTargetPose(currentFace, side);
                        return FlippingUtil.flipFieldPose(mirroredTargetPose);
                    } else {
                        int currentFace = ReefAlignmentHelpers.calculateBestFace(robotPose);
                        return ReefAlignmentHelpers.calculateTargetPose(currentFace, side);
                    }
                });
    }
}
