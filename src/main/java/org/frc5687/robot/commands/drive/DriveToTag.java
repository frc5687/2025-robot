package org.frc5687.robot.commands.drive;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import java.util.List;
import java.util.Optional;
import org.frc5687.robot.commands.OutliersCommand;
import org.frc5687.robot.subsystems.drive.DriveSubsystem;
import org.frc5687.robot.subsystems.vision.AprilTagObservation;
import org.frc5687.robot.subsystems.vision.VisionSubsystem;
import org.frc5687.robot.util.FieldConstants;
import org.frc5687.robot.util.ReefAlignmentHelpers;
import org.frc5687.robot.util.TunableDouble;
import org.frc5687.robot.util.ReefAlignmentHelpers.ReefSide;

public class DriveToTag extends OutliersCommand {
    private final DriveSubsystem _drive;
    private final VisionSubsystem _vision;
    private final ReefSide _reefSide;

    private final ProfiledPIDController _driveController;
    private final ProfiledPIDController _thetaController;
    private final ProfiledPIDController _lateralController;
    private final TunableDouble _distanceOffset = new TunableDouble("DriveToTag", "Distance Offset", 0.2);

    public DriveToTag(DriveSubsystem drive, VisionSubsystem vision, ReefSide reefSide) {
        _drive = drive;
        _vision = vision;
        _reefSide = reefSide;

        _driveController =
                new ProfiledPIDController(2.0, 0.0, 0.0, new TrapezoidProfile.Constraints(4.0, 5.0), 0.02);

        _thetaController =
                new ProfiledPIDController(
                        3.0, 0.0, 0.0, new TrapezoidProfile.Constraints(Math.PI, Math.PI / 2), 0.02);
                        
        _lateralController = 
                new ProfiledPIDController(
                        1.5, 0.0, 0.0, new TrapezoidProfile.Constraints(2.0, 3.0), 0.02);

        _thetaController.enableContinuousInput(-Math.PI, Math.PI);

        addRequirements(drive, vision);
    }

    @Override
    public void initialize() {
        Pose2d currentPose = _drive.getPose();
        _driveController.reset(0, 0);
        _lateralController.reset(0, 0);
        _thetaController.reset(currentPose.getRotation().getRadians(), 0);
        _vision.setPipeline(1); // remove magic number later -Dennis
    }

    @Override
    public void execute(double timestamp) {
        boolean isRedAlliance = false;
        var alliance = DriverStation.getAlliance();
        if (alliance.isPresent()) {
            isRedAlliance = alliance.get() == Alliance.Red;
        }
        int tagId = ReefAlignmentHelpers.getTagIdFromBestFace(_drive.getPose(), isRedAlliance);
        
        String cameraName;
        List<AprilTagObservation> observations;
        
        if (_reefSide == ReefSide.RIGHT) {
            cameraName = "North_West_Camera";
            observations = _vision.getNorthWestCameraObservations();
        } else {
            cameraName = "North_Camera";
            observations = _vision.getNorthCameraObservations();
        }

        Optional<AprilTagObservation> observation = _vision.getTagFromObservations(observations, tagId);
        if (observation.isEmpty()) {
            return;
        }

        Optional<Pose3d> tagPose = FieldConstants.aprilTagLayout.getTagPose(tagId);
        if (tagPose.isEmpty()) {
            return;
        }
        log("desired tag id", tagId);

        // I tried distance calculation with just pitch, corners with the focal length from simulated calibration seems better.
        double distance = _vision.calculateDistanceWithCalibration(
                observation.get(), cameraName);
        
        log("Distance", distance);

        Rotation2d targetHeading =
                tagPose.get().getRotation().toRotation2d().rotateBy(new Rotation2d(Math.PI));

        // lateral offset to align directly with tag
        double lateralOffset = _vision.calculateLateralOffset(observation.get(), cameraName);
        log("Lateral Offset", lateralOffset);
        
        // negative as the error will be negative since distance is always positive away from tag.
        double vxDesired = -_driveController.calculate(distance, _distanceOffset.get());
        double vyDesired = _lateralController.calculate(lateralOffset, 0);
        
        log("vxDesired", vxDesired);
        log("vyDesired", vyDesired);

        ChassisSpeeds speeds = new ChassisSpeeds(
                vxDesired,
                vyDesired,
                _thetaController.calculate(
                        _drive.getPose().getRotation().getRadians(), targetHeading.getRadians()));

        _drive.setDesiredChassisSpeeds(speeds);
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        _vision.setPipelineIndex(0);
        _drive.setDesiredChassisSpeeds(new ChassisSpeeds());
    }
}