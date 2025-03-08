// package org.frc5687.robot.commands.drive;

// import com.pathplanner.lib.util.FlippingUtil;
// import edu.wpi.first.math.geometry.Pose2d;
// import edu.wpi.first.wpilibj.DriverStation;
// import edu.wpi.first.wpilibj.DriverStation.Alliance;
// import org.frc5687.robot.subsystems.drive.DriveSubsystem;
// import org.frc5687.robot.util.ReefAlignmentHelpers;
// import org.frc5687.robot.util.ReefAlignmentHelpers.ReefSide;

// public class DriveToAlgaeOffset extends DriveToPoseSmooth {
//     public DriveToAlgaeOffset(DriveSubsystem drive) {
//         super(
//                 drive,
//                 () -> {
//                     Pose2d rawPose = drive.getPose();
//                     var alliance = DriverStation.getAlliance();
//                     if (alliance.isPresent() && alliance.get() == Alliance.Red) {
//                         Pose2d mirroredPose = FlippingUtil.flipFieldPose(rawPose);
//                         int currentFace = ReefAlignmentHelpers.calculateBestFace(mirroredPose);
//                         return ReefAlignmentHelpers.calculateTargetAlgaePose(currentFace,
// ReefSide.ALGAE);
//                     } else {
//                         int currentFace = ReefAlignmentHelpers.calculateBestFace(rawPose);
//                         return ReefAlignmentHelpers.calculateTargetAlgaePose(currentFace,
// ReefSide.ALGAE);
//                     }
//                 });
//     }
// }
