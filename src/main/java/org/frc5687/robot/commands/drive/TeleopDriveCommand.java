package org.frc5687.robot.commands.drive;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import java.util.Optional;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import org.frc5687.robot.commands.OutliersCommand;
import org.frc5687.robot.subsystems.drive.DriveSubsystem;

public class TeleopDriveCommand extends OutliersCommand {
    private final DriveSubsystem _drive;
    private final DoubleSupplier _xSupplier;
    private final DoubleSupplier _ySupplier;
    private final DoubleSupplier _rotationSupplier;
    private final BooleanSupplier _fieldRelativeSupplier;
    private static final double ROSIE_THRESHOLD = 10; // 10deg

    private boolean _rightStickCentered;

    public TeleopDriveCommand(
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
        addRequirements(drive);
    }

    @Override
    public void execute(double timestamp) {
        if (_drive.rosieEnabled
                && (Math.abs(_drive.getPitch().getDegrees()) > ROSIE_THRESHOLD
                        || Math.abs(_drive.getRoll().getDegrees()) > ROSIE_THRESHOLD)) {
            var chassisSpeeds =
                    new ChassisSpeeds(
                            0.5 * _drive.getPitch().getDegrees(), -0.5 * _drive.getRoll().getDegrees(), 0.0);
            _drive.setDesiredChassisSpeeds(chassisSpeeds);
            return;
        }

        // Calculate chassis speeds
        Optional<Alliance> alliance = DriverStation.getAlliance();
        Rotation2d relativeHeading = _drive.getHeading();
        if (alliance.isPresent() && alliance.get() == Alliance.Red) {
            relativeHeading = _drive.getHeading().rotateBy(Rotation2d.fromDegrees(180));
        }
        ChassisSpeeds chassisSpeeds =
                _fieldRelativeSupplier.getAsBoolean()
                        ? ChassisSpeeds.fromFieldRelativeSpeeds(
                                _xSupplier.getAsDouble(),
                                _ySupplier.getAsDouble(),
                                _rotationSupplier.getAsDouble(),
                                relativeHeading)
                        : new ChassisSpeeds(
                                _xSupplier.getAsDouble(),
                                _ySupplier.getAsDouble(),
                                _rotationSupplier.getAsDouble());

        _drive.setDesiredChassisSpeeds(chassisSpeeds);
    }

    @Override
    public void end(boolean interrupted) {
        _drive.setDesiredChassisSpeeds(new ChassisSpeeds());
    }
}
