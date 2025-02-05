package org.frc5687.robot.commands.drive;

import org.frc5687.robot.subsystems.drive.DriveSubsystem;
import org.frc5687.robot.util.FieldConstants.ReefHeight;
import org.frc5687.robot.util.ReefAlignmentHelpers;
import org.frc5687.robot.util.ReefAlignmentHelpers.ReefSide;

public class DynamicDriveToReefBranch extends DriveToPose {

    public DynamicDriveToReefBranch(DriveSubsystem drive, ReefSide side, ReefHeight height) {
        super(
                drive,
                () -> {
                    int currentFace = ReefAlignmentHelpers.calculateBestFace(drive.getPose());
                    return ReefAlignmentHelpers.calculateTargetPose(currentFace, side, height);
                });
    }
}
