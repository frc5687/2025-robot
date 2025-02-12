package org.frc5687.robot.commands.drive;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import java.util.function.Supplier;
import org.frc5687.robot.commands.OutliersCommand;
import org.frc5687.robot.subsystems.drive.DriveSubsystem;

public class DriveToPose extends OutliersCommand {
    private final DriveSubsystem _drive;
    private final Supplier<Pose2d> _poseSupplier;

    private final ProfiledPIDController _driveController;
    private final ProfiledPIDController _thetaController;

    private Translation2d _lastSetpointTranslation;

    private double ffMinRadius = 0.2;
    private double ffMaxRadius = 0.8;

    public DriveToPose(DriveSubsystem drive, Supplier<Pose2d> poseSupplier) {
        _drive = drive;
        _poseSupplier = poseSupplier;

        _driveController =
                new ProfiledPIDController(4, 0.0, 0.0, new TrapezoidProfile.Constraints(3.0, 5.0));

        _thetaController =
                new ProfiledPIDController(
                        4.0, 0.0, 0.0, new TrapezoidProfile.Constraints(2.0 * Math.PI, 4 * Math.PI));

        _thetaController.enableContinuousInput(-Math.PI, Math.PI);

        _driveController.setTolerance(0.001);
        _thetaController.setTolerance(0.001);

        addRequirements(drive);
    }

    @Override
    public void initialize() {
        Pose2d currentPose = _drive.getPose();
        _driveController.reset(0, 0);
        _thetaController.reset(currentPose.getRotation().getRadians(), 0);
        _lastSetpointTranslation = currentPose.getTranslation();
    }

    @Override
    public void execute(double timestamp) {
        Pose2d currentPose = _drive.getPose();
        Pose2d targetPose = _poseSupplier.get();

        double currentDistance = currentPose.getTranslation().getDistance(targetPose.getTranslation());
        double ffScaler =
                MathUtil.clamp((currentDistance - ffMinRadius) / (ffMaxRadius - ffMinRadius), 0.0, 1.0);

        _driveController.reset(
                _lastSetpointTranslation.getDistance(targetPose.getTranslation()),
                _driveController.getSetpoint().velocity);

        double driveVelocityScalar =
                _driveController.getSetpoint().velocity * ffScaler
                        + _driveController.calculate(currentDistance, 0.0);

        if (currentDistance < _driveController.getPositionTolerance()) {
            driveVelocityScalar = 0.0;
        }

        _lastSetpointTranslation =
                new Pose2d(
                                targetPose.getTranslation(),
                                currentPose.getTranslation().minus(targetPose.getTranslation()).getAngle())
                        .transformBy(
                                new Transform2d(
                                        new Translation2d(_driveController.getSetpoint().position, 0.0),
                                        new Rotation2d()))
                        .getTranslation();

        double thetaVelocity =
                _thetaController.getSetpoint().velocity * ffScaler
                        + _thetaController.calculate(
                                currentPose.getRotation().getRadians(), targetPose.getRotation().getRadians());

        double thetaError =
                Math.abs(currentPose.getRotation().minus(targetPose.getRotation()).getRadians());

        if (thetaError < _thetaController.getPositionTolerance()) {
            thetaVelocity = 0.0;
        }

        var driveVelocity =
                new Pose2d(
                                new Translation2d(),
                                currentPose.getTranslation().minus(targetPose.getTranslation()).getAngle())
                        .transformBy(
                                new Transform2d(new Translation2d(driveVelocityScalar, 0.0), new Rotation2d()))
                        .getTranslation();

        _drive.setDesiredChassisSpeeds(
                ChassisSpeeds.fromFieldRelativeSpeeds(
                        driveVelocity.getX(), driveVelocity.getY(), thetaVelocity, currentPose.getRotation()));
    }

    @Override
    public boolean isFinished() {
        return _driveController.atGoal() && _thetaController.atGoal();
    }

    @Override
    public void end(boolean interrupted) {
        _drive.setDesiredChassisSpeeds(new ChassisSpeeds());
    }
}
