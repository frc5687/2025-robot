package org.frc5687.robot.commands.drive;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.filter.MedianFilter;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import java.util.ArrayDeque;
import java.util.Queue;
import org.frc5687.robot.commands.OutliersCommand;
import org.frc5687.robot.subsystems.drive.DriveSubsystem;
import org.frc5687.robot.subsystems.vision.AprilTagObservation;
import org.frc5687.robot.subsystems.vision.VisionSubsystem;

public class DriveToTag extends OutliersCommand {
    private final DriveSubsystem _drive;
    private final VisionSubsystem _vision;
    private final Transform3d _robotToCam;
    private final double DESIRED_DISTANCE = 0.5;

    private final ProfiledPIDController _driveController;
    private final ProfiledPIDController _thetaController;

    private static final int FILTER_WINDOW = 5;
    private static final int POSE_HISTORY_SIZE = 10;
    private static final double MAX_POSE_JUMP = 0.5; // meters
    private static final double MAX_ROTATION_JUMP = Math.PI / 2; // 90 degrees

    private double ffMinRadius = 0.2;
    private double ffMaxRadius = 0.8;

    private final MedianFilter _xFilter = new MedianFilter(FILTER_WINDOW);
    private final MedianFilter _yFilter = new MedianFilter(FILTER_WINDOW);
    private final MedianFilter _rotFilter = new MedianFilter(FILTER_WINDOW);

    private final Queue<Pose2d> _recentPoses = new ArrayDeque<>(POSE_HISTORY_SIZE);
    private Pose2d _lastValidPose = null;
    private Pose2d _targetPose = null;
    private Translation2d _lastSetpointTranslation;
    private int _tagId = -1;
    private long _lastValidTimestamp = 0;
    private static final long MAX_TAG_AGE_MS = 500;

    public DriveToTag(
            DriveSubsystem drive, VisionSubsystem vision, Transform3d robotToCam) {
        _drive = drive;
        _vision = vision;
        _robotToCam = robotToCam;

        _driveController =
                new ProfiledPIDController(5.0, 0.0, 0.0, new TrapezoidProfile.Constraints(4.0, 5.0), 0.02);

        _thetaController =
                new ProfiledPIDController(
                        3.0, 0.0, 0.0, new TrapezoidProfile.Constraints(Math.PI, Math.PI / 2), 0.02);

        _thetaController.enableContinuousInput(-Math.PI, Math.PI);

        addRequirements(drive, vision);
    }

    @Override
    public void initialize() {
        Pose2d currentPose = _drive.getPose();
        _driveController.reset(0, 0);
        _thetaController.reset(currentPose.getRotation().getRadians(), 0);
        _lastSetpointTranslation = currentPose.getTranslation();
        _targetPose = null;
        _lastValidPose = null;
        _recentPoses.clear();
        _tagId = -1;
    }

    private boolean isPoseValid(Pose2d newPose) {
        if (_lastValidPose == null) return true;

        Transform2d difference = newPose.minus(_lastValidPose);
        return Math.hypot(difference.getX(), difference.getY()) < MAX_POSE_JUMP
                && Math.abs(difference.getRotation().getRadians()) < MAX_ROTATION_JUMP;
    }

    private void updateTargetPose(AprilTagObservation tag) {
        if (tag.getId() != _tagId) {
            _xFilter.reset();
            _yFilter.reset();
            _rotFilter.reset();
            _recentPoses.clear();
            _tagId = tag.getId();
        }

        Pose3d robotPose = new Pose3d(_drive.getPose());
        Pose3d cameraPose = robotPose.transformBy(_robotToCam);
        Pose3d tagPose = cameraPose.transformBy(tag.getTransform());

        Rotation2d tagRotation2d = tagPose.getRotation().toRotation2d();
        double filteredRot = _rotFilter.calculate(tagRotation2d.getRadians());
        double filteredX = _xFilter.calculate(tagPose.getX());
        double filteredY = _yFilter.calculate(tagPose.getY());

        Translation2d approachPoint =
                new Translation2d(filteredX, filteredY)
                        .plus(
                                new Translation2d(
                                        DESIRED_DISTANCE * Math.cos(filteredRot),
                                        DESIRED_DISTANCE * Math.sin(filteredRot)));

        Pose2d newTargetPose =
                new Pose2d(
                        approachPoint,
                        new Rotation2d(
                                Math.atan2(filteredY - approachPoint.getY(), filteredX - approachPoint.getX())));

        if (isPoseValid(newTargetPose)) {
            _targetPose = newTargetPose;
            _lastValidPose = newTargetPose;
            _lastValidTimestamp = System.currentTimeMillis();

            _recentPoses.add(newTargetPose);
            if (_recentPoses.size() > POSE_HISTORY_SIZE) {
                _recentPoses.remove();
            }
        }
    }

    @Override
    public void execute(double timestamp) {
        var tag = _vision.getClosestTag();

        if (tag != null) {
            updateTargetPose(tag);
        }

        if (_lastValidPose == null
                || System.currentTimeMillis() - _lastValidTimestamp > MAX_TAG_AGE_MS) {
            _drive.setDesiredChassisSpeeds(new ChassisSpeeds());
            return;
        }

        Pose2d currentPose = _drive.getPose();

        double currentDistance = currentPose.getTranslation().getDistance(_targetPose.getTranslation());
        double ffScaler =
                MathUtil.clamp((currentDistance - ffMinRadius) / (ffMaxRadius - ffMinRadius), 0.0, 1.0);

        _driveController.reset(
                _lastSetpointTranslation.getDistance(_targetPose.getTranslation()),
                _driveController.getSetpoint().velocity);

        double driveVelocityScalar =
                _driveController.getSetpoint().velocity * ffScaler
                        + _driveController.calculate(currentDistance, 0.0);

        if (currentDistance < _driveController.getPositionTolerance()) {
            driveVelocityScalar = 0.0;
        }

        _lastSetpointTranslation =
                new Pose2d(
                                _targetPose.getTranslation(),
                                currentPose.getTranslation().minus(_targetPose.getTranslation()).getAngle())
                        .transformBy(
                                new Transform2d(
                                        new Translation2d(_driveController.getSetpoint().position, 0.0),
                                        new Rotation2d()))
                        .getTranslation();

        double thetaVelocity =
                _thetaController.getSetpoint().velocity * ffScaler
                        + _thetaController.calculate(
                                currentPose.getRotation().getRadians(), _targetPose.getRotation().getRadians());

        double thetaError =
                Math.abs(currentPose.getRotation().minus(_targetPose.getRotation()).getRadians());

        if (thetaError < _thetaController.getPositionTolerance()) {
            thetaVelocity = 0.0;
        }

        var driveVelocity =
                new Pose2d(
                                new Translation2d(),
                                currentPose.getTranslation().minus(_targetPose.getTranslation()).getAngle())
                        .transformBy(
                                new Transform2d(new Translation2d(driveVelocityScalar, 0.0), new Rotation2d()))
                        .getTranslation();

        _drive.setDesiredChassisSpeeds(
                ChassisSpeeds.fromFieldRelativeSpeeds(
                        driveVelocity.getX(), driveVelocity.getY(), thetaVelocity, currentPose.getRotation()));
    }

    @Override
    public boolean isFinished() {
        return _targetPose == null || (_driveController.atGoal() && _thetaController.atGoal());
    }

    @Override
    public void end(boolean interrupted) {
        _drive.setDesiredChassisSpeeds(new ChassisSpeeds());
    }
}
