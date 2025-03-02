package org.frc5687.robot.commands.drive;

import com.pathplanner.lib.util.FlippingUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import org.frc5687.robot.subsystems.drive.DriveSubsystem;
import org.frc5687.robot.util.FieldConstants;

public class DriveToHP extends DriveToPoseSmooth {

    private static final double TARGET_X = 1.1;
    private static final double TARGET_Y_LOWER = 1.0;
    private static final double TARGET_Y_UPPER = 7.0;
    private static final double TARGET_ANGLE_LOWER = 53.0;
    private static final double TARGET_ANGLE_UPPER = -53.0;

    public DriveToHP(DriveSubsystem drive) {
        super(
                drive,
                () -> {
                    Pose2d currentPose = drive.getPose();
                    double fieldMidpointY = FieldConstants.fieldWidth / 2.0;
                    Pose2d targetPose;

                    var alliance = DriverStation.getAlliance();
                    boolean isRedAlliance = alliance.isPresent() && alliance.get() == Alliance.Red;

                    if (isRedAlliance) {
                        if (currentPose.getY() > fieldMidpointY) {
                            targetPose =
                                    new Pose2d(TARGET_X, TARGET_Y_LOWER, Rotation2d.fromDegrees(TARGET_ANGLE_LOWER));
                        } else {
                            targetPose =
                                    new Pose2d(TARGET_X, TARGET_Y_UPPER, Rotation2d.fromDegrees(TARGET_ANGLE_UPPER));
                        }
                        return FlippingUtil.flipFieldPose(targetPose);
                    } else {
                        if (currentPose.getY() < fieldMidpointY) {
                            targetPose =
                                    new Pose2d(TARGET_X, TARGET_Y_LOWER, Rotation2d.fromDegrees(TARGET_ANGLE_LOWER));
                        } else {
                            targetPose =
                                    new Pose2d(TARGET_X, TARGET_Y_UPPER, Rotation2d.fromDegrees(TARGET_ANGLE_UPPER));
                        }
                        return targetPose;
                    }
                });
    }
}
