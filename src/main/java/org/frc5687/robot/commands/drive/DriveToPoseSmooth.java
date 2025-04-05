package org.frc5687.robot.commands.drive;

import edu.wpi.first.epilogue.Logged.Importance;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import java.util.function.Supplier;
import org.frc5687.robot.Constants;
import org.frc5687.robot.commands.OutliersCommand;
import org.frc5687.robot.subsystems.drive.DriveSubsystem;
import org.frc5687.robot.util.TunableDouble;

public class DriveToPoseSmooth extends OutliersCommand {
    protected final DriveSubsystem _drive;
    private final Supplier<Pose2d> _poseSupplier;

    private ChassisSpeeds _fieldRelativeVelocity;
    protected final PIDController _xController;
    protected final PIDController _yController;
    protected final PIDController _rotationController;

    private final TunableDouble _smoothingFactor =
            new TunableDouble("DriveToPose", "smoothingFactor", Constants.DriveToPose.SMOOTHING_FACTOR);
    private final TunableDouble _maxVelocity =
            new TunableDouble("DriveToPose", "maxVelocity", Constants.DriveToPose.MAX_VELOCITY);
    private final TunableDouble _maxAcceleration =
            new TunableDouble("DriveToPose", "maxAcceleration", Constants.DriveToPose.MAX_ACCELERATION);
    private final TunableDouble _positionTolerance =
            new TunableDouble(
                    "DriveToPose", "positionTolerance", Constants.DriveToPose.POSITION_TOLERANCE);
    private final TunableDouble _velocityTolerance =
            new TunableDouble(
                    "DriveToPose", "velocityTolerance", Constants.DriveToPose.VELOCITY_TOLERANCE);
    private final TunableDouble _minOutput =
            new TunableDouble("DriveToPose", "minOutput", Constants.DriveToPose.MIN_OUTPUT);
    private final TunableDouble _counteractGain =
            new TunableDouble("DriveToPose", "counteractGain", Constants.DriveToPose.COUNTERACT_GAIN);
    private final TunableDouble _aggressiveAccelMultiplier =
            new TunableDouble(
                    "DriveToPose",
                    "aggressiveAccelMultiplier",
                    Constants.DriveToPose.AGGRESSIVE_ACCEL_MULTIPLIER);

    private final TunableDouble _driveKp =
            new TunableDouble("DriveToPose", "driveKp", Constants.DriveToPose.DRIVE_KP);
    private final TunableDouble _driveKi =
            new TunableDouble("DriveToPose", "driveKi", Constants.DriveToPose.DRIVE_KI);
    private final TunableDouble _driveKd =
            new TunableDouble("DriveToPose", "driveKd", Constants.DriveToPose.DRIVE_KD);
    private final TunableDouble _rotKp =
            new TunableDouble("DriveToPose", "rotKp", Constants.DriveToPose.ROT_KP);
    private final TunableDouble _rotKi =
            new TunableDouble("DriveToPose", "rotKi", Constants.DriveToPose.ROT_KI);
    private final TunableDouble _rotKd =
            new TunableDouble("DriveToPose", "rotKd", Constants.DriveToPose.ROT_KD);

    public DriveToPoseSmooth(DriveSubsystem drive, Supplier<Pose2d> poseSupplier) {
        _drive = drive;
        _poseSupplier = poseSupplier;

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
        log("Target Pose", _poseSupplier.get(), Pose2d.struct, Importance.CRITICAL);
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
        Pose2d targetPose = _poseSupplier.get();

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
                        _fieldRelativeVelocity.omegaRadiansPerSecond,
                        omegaDesired,
                        _smoothingFactor.get()); // FIXME dennis do your thing

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
    }

    @Override
    public boolean isFinished() {
        return _xController.atSetpoint()
                && _yController.atSetpoint()
                && _rotationController.atSetpoint()
                && Math.abs(_fieldRelativeVelocity.vxMetersPerSecond) < _velocityTolerance.get()
                && Math.abs(_fieldRelativeVelocity.vyMetersPerSecond) < _velocityTolerance.get()
                && Math.abs(_fieldRelativeVelocity.omegaRadiansPerSecond) < Math.toRadians(2);
    }
}
