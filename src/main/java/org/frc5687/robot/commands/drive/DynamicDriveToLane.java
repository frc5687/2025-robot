package org.frc5687.robot.commands.drive;

import com.pathplanner.lib.util.FlippingUtil;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import java.util.Optional;
import java.util.function.DoubleSupplier;
import org.frc5687.robot.commands.OutliersCommand;
import org.frc5687.robot.subsystems.drive.DriveSubsystem;
import org.frc5687.robot.util.LaneAlignmentHelpers;

public class DynamicDriveToLane extends OutliersCommand {
    private final DriveSubsystem _drive;
    private final DoubleSupplier _xSupplier;

    private final ProfiledPIDController _driveController;
    private final ProfiledPIDController _thetaController;

    private Translation2d _lastSetpointTranslation;

    private Pose2d _lanePose;
    private double ffMinRadius = 0.2;
    private double ffMaxRadius = 0.8;

    public DynamicDriveToLane(DriveSubsystem drive, DoubleSupplier xSupplier) {
        _drive = drive;
        _xSupplier = xSupplier;

        _lanePose = new Pose2d();

        _driveController =
                new ProfiledPIDController(3, 0.0, 0.001, new TrapezoidProfile.Constraints(3.0, 3.0));

        _thetaController =
                new ProfiledPIDController(
                        4.0, 0.0, 0.0, new TrapezoidProfile.Constraints(2.0 * Math.PI, 4 * Math.PI));

        _thetaController.enableContinuousInput(-Math.PI, Math.PI);

        _driveController.setTolerance(0.005);
        _thetaController.setTolerance(0.02);

        addRequirements(drive);
    }

    @Override
    public void initialize() {
        Pose2d currentPose = _drive.getPose();
        var alliance = DriverStation.getAlliance();
        if (alliance.isPresent() && alliance.get() == Alliance.Red) {
            Pose2d mirroredPose = FlippingUtil.flipFieldPose(currentPose);
            Pose2d mirroredTargetPose = LaneAlignmentHelpers.calculateBestLanePose(mirroredPose);
            System.out.println(FlippingUtil.flipFieldPose(mirroredTargetPose));
            _lanePose = FlippingUtil.flipFieldPose(mirroredTargetPose);
        } else {
            System.out.println(LaneAlignmentHelpers.calculateBestLanePose(currentPose));
            _lanePose = LaneAlignmentHelpers.calculateBestLanePose(currentPose);
        }
        _driveController.reset(0, 0);
        _thetaController.reset(currentPose.getRotation().getRadians(), 0);
        _lastSetpointTranslation = currentPose.getTranslation();
    }

    @Override
    public void execute(double timestamp) {
        Pose2d currentPose = _drive.getPose();

        Pose2d targetPose = _lanePose;
        log("DriveToPose Target", targetPose, Pose2d.struct);

        Optional<Alliance> alliance = DriverStation.getAlliance();
        if (alliance.isPresent()) {
            if (alliance.get() == Alliance.Red) {
                currentPose =
                        new Pose2d(
                                currentPose.getTranslation(),
                                currentPose.getRotation().rotateBy(Rotation2d.fromDegrees(180)));
            }
        }

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
                        _xSupplier.getAsDouble(),
                        driveVelocity.getY(),
                        thetaVelocity,
                        currentPose.getRotation()));
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
