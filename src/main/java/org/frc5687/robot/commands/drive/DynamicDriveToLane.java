package org.frc5687.robot.commands.drive;

import com.pathplanner.lib.util.FlippingUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import org.frc5687.robot.subsystems.drive.DriveSubsystem;
import org.frc5687.robot.util.LaneAlignmentHelpers;

public class DynamicDriveToLane extends DriveToPoseSmooth {

    public DynamicDriveToLane(DriveSubsystem drive) {
        super(
                drive,
                () -> {
                    Pose2d rawPose = drive.getPose();
                    var alliance = DriverStation.getAlliance();
                    if (alliance.isPresent() && alliance.get() == Alliance.Red) {
                        Pose2d mirroredPose = FlippingUtil.flipFieldPose(rawPose);
                        Pose2d mirroredTargetPose = LaneAlignmentHelpers.calculateBestLanePose(mirroredPose);
                        System.out.println(FlippingUtil.flipFieldPose(mirroredTargetPose));
                        return FlippingUtil.flipFieldPose(mirroredTargetPose);
                    } else {
                        System.out.println(LaneAlignmentHelpers.calculateBestLanePose(rawPose));
                        return LaneAlignmentHelpers.calculateBestLanePose(rawPose);
                    }
                });
    }
}
