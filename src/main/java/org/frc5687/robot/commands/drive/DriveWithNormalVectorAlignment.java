package org.frc5687.robot.commands.drive;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import java.util.Optional;
import java.util.function.Supplier;
import org.frc5687.robot.Constants;
import org.frc5687.robot.commands.OutliersCommand;
import org.frc5687.robot.subsystems.drive.DriveSubsystem;
import org.frc5687.robot.subsystems.superstructure.SuperstructureManager;
import org.frc5687.robot.util.TunableDouble;

public class DriveWithNormalVectorAlignment extends OutliersCommand {
    private final DriveSubsystem _drive;
    private final SuperstructureManager _manager;
    private final Supplier<Pose2d> _finalPoseSupplier;

    private final TunableDouble _normalVectorOffset =
            new TunableDouble("NormalVectorAlignment", "offset", 0.5);

    // We blend the start and end for smooth transition
    private final TunableDouble _blendStartDistance =
            new TunableDouble("NormalVectorAlignment", "blendStart", 0.3);
    private final TunableDouble _blendEndDistance =
            new TunableDouble("NormalVectorAlignment", "blendEnd", 0.05);

    private Pose2d _alignmentPose = null;
    private Pose2d _finalPose = null;

    private ChassisSpeeds _fieldRelativeVelocity;
    private final PIDController _xController;
    private final PIDController _yController;
    private final PIDController _rotationController;

    private final TunableDouble _smoothingFactor =
            new TunableDouble("DriveToPose", "smoothingFactor", 0.6);
    private final TunableDouble _maxVelocity = new TunableDouble("DriveToPose", "maxVelocity", 3.0);
    private final TunableDouble _maxAcceleration =
            new TunableDouble("DriveToPose", "maxAcceleration", 9.0);
    private final TunableDouble _positionTolerance =
            new TunableDouble("DriveToPose", "positionTolerance", 0.01);
    private final TunableDouble _velocityTolerance =
            new TunableDouble("DriveToPose", "velocityTolerance", 0.1);
    private final TunableDouble _minOutput = new TunableDouble("DriveToPose", "minOutput", 0.1);
    private final TunableDouble _counteractGain =
            new TunableDouble("DriveToPose", "counteractGain", 0.5);
    private final TunableDouble _aggressiveAccelMultiplier =
            new TunableDouble("DriveToPose", "aggressiveAccelMultiplier", 2.0);

    private final TunableDouble _driveKp = new TunableDouble("DriveToPose", "driveKp", 3.0);
    private final TunableDouble _driveKi = new TunableDouble("DriveToPose", "driveKi", 0.0);
    private final TunableDouble _driveKd = new TunableDouble("DriveToPose", "driveKd", 0.1);
    private final TunableDouble _rotKp = new TunableDouble("DriveToPose", "rotKp", 3.0);
    private final TunableDouble _rotKi = new TunableDouble("DriveToPose", "rotKi", 0.0);
    private final TunableDouble _rotKd = new TunableDouble("DriveToPose", "rotKd", 0.1);

    public DriveWithNormalVectorAlignment(
            DriveSubsystem drive, SuperstructureManager manager, Supplier<Pose2d> finalPoseSupplier) {
        _drive = drive;
        _manager = manager;
        _finalPoseSupplier = finalPoseSupplier;

        _xController = new PIDController(_driveKp.get(), _driveKi.get(), _driveKd.get());
        _yController = new PIDController(_driveKp.get(), _driveKi.get(), _driveKd.get());
        _rotationController = new PIDController(_rotKp.get(), _rotKi.get(), _rotKd.get());

        _rotationController.enableContinuousInput(-Math.PI, Math.PI);

        addRequirements(drive);
    }

    @Override
    public void initialize() {
        ChassisSpeeds robotRelativeSpeeds = _drive.getMeasuredChassisSpeeds();
        _fieldRelativeVelocity =
                ChassisSpeeds.fromRobotRelativeSpeeds(robotRelativeSpeeds, _drive.getPose().getRotation());

        _xController.reset();
        _yController.reset();
        _rotationController.reset();

        updatePIDTolerances();
        updateTargetPoses();
    }

    private void updateTargetPoses() {
        _finalPose = _finalPoseSupplier.get();

        if (_finalPose == null) {
            return;
        }

        // alignment pose direction opposite to normal vector
        double offsetX = -_normalVectorOffset.get() * _finalPose.getRotation().getCos();
        double offsetY = -_normalVectorOffset.get() * _finalPose.getRotation().getSin();
        _alignmentPose =
                new Pose2d(
                        _finalPose.getX() + offsetX, _finalPose.getY() + offsetY, _finalPose.getRotation());
        // Set in intialize but also in execute for reference
        _manager.setGoalPose(Optional.of(_alignmentPose));
    }

    private Pose2d getCurrentTargetPose() {
        updateTargetPoses();

        Pose2d robotPose = _drive.getPose();

        Translation2d robotToAlignment =
                new Translation2d(
                        _alignmentPose.getX() - robotPose.getX(), _alignmentPose.getY() - robotPose.getY());

        // alignment pose to final pose
        Translation2d approachVector =
                new Translation2d(
                        _finalPose.getX() - _alignmentPose.getX(), _finalPose.getY() - _alignmentPose.getY());
        double approachMagnitude = approachVector.getNorm();
        Translation2d approachDirection = approachVector.div(approachMagnitude);

        // Project robot position onto alignment line to find closest point
        double robotProjection =
                (robotToAlignment.getX() * -approachDirection.getX()
                        + robotToAlignment.getY() * -approachDirection.getY());

        Translation2d projectedPoint =
                new Translation2d(
                        robotPose.getX() + robotProjection * -approachDirection.getX(),
                        robotPose.getY() + robotProjection * -approachDirection.getY());

        double perpendicularDistance =
                new Translation2d(
                                projectedPoint.getX() - _alignmentPose.getX(),
                                projectedPoint.getY() - _alignmentPose.getY())
                        .getNorm();

        double blendFactor = 0.0;

        // If we go past the alignment pose along approach vector, we want to blend based on distance
        if (perpendicularDistance > 0) {
            blendFactor =
                    MathUtil.clamp(
                            1.0
                                    - (perpendicularDistance - _blendEndDistance.get())
                                            / (_blendStartDistance.get() - _blendEndDistance.get()),
                            0.0,
                            1.0);
        }

        double targetX = _alignmentPose.getX() + blendFactor * approachVector.getX();
        double targetY = _alignmentPose.getY() + blendFactor * approachVector.getY();
        log("Perpendicular Distance", perpendicularDistance);
        log("Blend Factor", blendFactor);

        return new Pose2d(targetX, targetY, _finalPose.getRotation());
    }

    private void updatePIDGains() {
        _xController.setPID(_driveKp.get(), _driveKi.get(), _driveKd.get());
        _yController.setPID(_driveKp.get(), _driveKi.get(), _driveKd.get());
        _rotationController.setPID(_rotKp.get(), _rotKi.get(), _rotKd.get());
    }

    private void updatePIDTolerances() {
        _xController.setTolerance(_positionTolerance.get(), _velocityTolerance.get());
        _yController.setTolerance(_positionTolerance.get(), _velocityTolerance.get());
        _rotationController.setTolerance(Math.toRadians(2), Math.toRadians(2));
    }

    @Override
    public void execute(double timestamp) {
        Pose2d targetPose = getCurrentTargetPose();

        log("Target Pose", targetPose, Pose2d.struct);
        log("Alignment Pose", _alignmentPose, Pose2d.struct);
        log("Final Pose", _finalPose, Pose2d.struct);

        if (_driveKp.hasChanged()
                || _driveKi.hasChanged()
                || _driveKd.hasChanged()
                || _rotKp.hasChanged()
                || _rotKi.hasChanged()
                || _rotKd.hasChanged()) {
            updatePIDGains();
        }
        if (_positionTolerance.hasChanged() || _velocityTolerance.hasChanged()) {
            updatePIDTolerances();
        }

        Pose2d currentPose = _drive.getPose();

        Translation2d errorVec =
                new Translation2d(
                        targetPose.getX() - currentPose.getX(), targetPose.getY() - currentPose.getY());
        Translation2d currentVel =
                new Translation2d(
                        _fieldRelativeVelocity.vxMetersPerSecond, _fieldRelativeVelocity.vyMetersPerSecond);

        // find direction vec
        double velocityDot = errorVec.getX() * currentVel.getX() + errorVec.getY() * currentVel.getY();
        boolean movingAway = velocityDot < 0;

        double vxDesired = _xController.calculate(currentPose.getX(), targetPose.getX());
        double vyDesired = _yController.calculate(currentPose.getY(), targetPose.getY());

        // If we are moving away from the goal we want the control to be more aggresive
        if (movingAway) {
            Translation2d counteract = currentVel.times(-_counteractGain.get());
            vxDesired += counteract.getX();
            vyDesired += counteract.getY();
        }

        Translation2d desiredVel = new Translation2d(vxDesired, vyDesired);
        double currentDistance = errorVec.getNorm();

        // limit our speed based on distance
        double maxAllowedSpeed = Math.sqrt(2 * _maxAcceleration.get() * currentDistance);
        maxAllowedSpeed = Math.min(maxAllowedSpeed, _maxVelocity.get());

        if (desiredVel.getNorm() > maxAllowedSpeed) {
            desiredVel = desiredVel.times(maxAllowedSpeed / desiredVel.getNorm());
            vxDesired = desiredVel.getX();
            vyDesired = desiredVel.getY();
        }

        // Maintain translational velocity mostly for the start
        if (currentDistance > _positionTolerance.get() * 2) {
            double currentSpeed = desiredVel.getNorm();
            if (currentSpeed < _minOutput.get() * _maxVelocity.get()) {
                desiredVel = errorVec.times(_minOutput.get() * _maxVelocity.get() / currentDistance);
                vxDesired = desiredVel.getX();
                vyDesired = desiredVel.getY();
            }
        }

        double omegaDesired =
                _rotationController.calculate(
                        currentPose.getRotation().getRadians(), targetPose.getRotation().getRadians());

        double omega =
                MathUtil.interpolate(
                        _fieldRelativeVelocity.omegaRadiansPerSecond, omegaDesired, _smoothingFactor.get());

        double dt = Constants.UPDATE_PERIOD;
        Translation2d deltaV =
                new Translation2d(
                        (vxDesired - _fieldRelativeVelocity.vxMetersPerSecond) * _smoothingFactor.get(),
                        (vyDesired - _fieldRelativeVelocity.vyMetersPerSecond) * _smoothingFactor.get());

        double effectiveMaxAccel =
                movingAway
                        ? _maxAcceleration.get() * _aggressiveAccelMultiplier.get()
                        : _maxAcceleration.get();

        double maxDv = effectiveMaxAccel * dt;
        if (deltaV.getNorm() > maxDv) {
            deltaV = deltaV.times(maxDv / deltaV.getNorm());
        }

        _fieldRelativeVelocity =
                new ChassisSpeeds(
                        _fieldRelativeVelocity.vxMetersPerSecond + deltaV.getX(),
                        _fieldRelativeVelocity.vyMetersPerSecond + deltaV.getY(),
                        omega);

        // Convert to robot-relative speeds
        _drive.setDesiredChassisSpeeds(
                ChassisSpeeds.fromFieldRelativeSpeeds(
                        _fieldRelativeVelocity.vxMetersPerSecond,
                        _fieldRelativeVelocity.vyMetersPerSecond,
                        _fieldRelativeVelocity.omegaRadiansPerSecond,
                        _drive.getPose().getRotation()));
    }

    @Override
    public void end(boolean interrupted) {
        _drive.setDesiredChassisSpeeds(new ChassisSpeeds());
        _manager.setGoalPose(Optional.empty());
    }

    @Override
    public boolean isFinished() {
        boolean atFinalPose =
                _xController.atSetpoint()
                        && _yController.atSetpoint()
                        && _rotationController.atSetpoint()
                        && Math.abs(_fieldRelativeVelocity.vxMetersPerSecond) < _velocityTolerance.get()
                        && Math.abs(_fieldRelativeVelocity.vyMetersPerSecond) < _velocityTolerance.get()
                        && Math.abs(_fieldRelativeVelocity.omegaRadiansPerSecond) < Math.toRadians(2);

        Pose2d robotPose = _drive.getPose();
        double distanceToFinalPose =
                new Translation2d(
                                robotPose.getX() - _finalPose.getX(), robotPose.getY() - _finalPose.getY())
                        .getNorm();

        return distanceToFinalPose < _positionTolerance.get() * 3 && atFinalPose;
    }
}
