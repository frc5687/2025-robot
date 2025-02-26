package org.frc5687.robot.commands.drive;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import org.frc5687.robot.Constants;
import org.frc5687.robot.commands.OutliersCommand;
import org.frc5687.robot.subsystems.drive.DriveSubsystem;

public class TeleopDriveWithSnapTo extends OutliersCommand {
    private final DriveSubsystem _drive;
    private final DoubleSupplier _xSupplier;
    private final DoubleSupplier _ySupplier;
    private final DoubleSupplier _rotationSupplier;
    private final BooleanSupplier _fieldRelativeSupplier;
    private final double _angle;

    private boolean _rightStickCentered;

    private final PIDController _headingController;

    public TeleopDriveWithSnapTo(
            double angle,
            DriveSubsystem drive,
            DoubleSupplier xSupplier,
            DoubleSupplier ySupplier,
            DoubleSupplier rotationSupplier,
            BooleanSupplier fieldRelativeSupplier) {
        _drive = drive;
        _xSupplier = xSupplier;
        _ySupplier = ySupplier;
        _rotationSupplier = rotationSupplier;
        _fieldRelativeSupplier = fieldRelativeSupplier;
        _rightStickCentered = true;
        _angle = angle;

        _headingController =
                new PIDController(
                        Constants.DriveTrain.HEADING_kP,
                        Constants.DriveTrain.HEADING_kI,
                        Constants.DriveTrain.HEADING_kD);
        _headingController.enableContinuousInput(-Math.PI, Math.PI);
        _headingController.setTolerance(0.0175 * 5);
        addRequirements(drive);
    }

    @Override
    public void execute(double timestamp) {

        // Calculate chassis speeds
        ChassisSpeeds chassisSpeeds =
                _fieldRelativeSupplier.getAsBoolean()
                        ? ChassisSpeeds.fromFieldRelativeSpeeds(
                                _xSupplier.getAsDouble(),
                                _ySupplier.getAsDouble(),
                                _rotationSupplier.getAsDouble(),
                                _drive.getHeading())
                        : new ChassisSpeeds(
                                _xSupplier.getAsDouble(),
                                _ySupplier.getAsDouble(),
                                _rotationSupplier.getAsDouble());

        double headingControllerOutput =
                _headingController.calculate(_drive.getHeading().getRadians(), _angle);
        chassisSpeeds.omegaRadiansPerSecond += headingControllerOutput;
        _drive.setDesiredChassisSpeeds(chassisSpeeds);
    }

    @Override
    public void end(boolean interrupted) {
        _drive.setDesiredChassisSpeeds(new ChassisSpeeds());
    }

    @Override
    public boolean isFinished() {
        return Math.abs(_drive.getHeading().minus(new Rotation2d(_angle)).getDegrees()) < 5.0;
    }
}
