package org.frc5687.robot.commands.drive;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import org.frc5687.robot.commands.OutliersCommand;
import org.frc5687.robot.subsystems.drive.DriveSubsystem;

import edu.wpi.first.math.kinematics.ChassisSpeeds;

public class TeleopDriveCommand extends OutliersCommand {
    private final DriveSubsystem _drive;
    private final DoubleSupplier _xSupplier;
    private final DoubleSupplier _ySupplier;
    private final DoubleSupplier _rotationSupplier;
    private final BooleanSupplier _fieldRelativeSupplier;

    public TeleopDriveCommand(
        DriveSubsystem drive,
        DoubleSupplier xSupplier,
        DoubleSupplier ySupplier,
        DoubleSupplier rotationSupplier,
        BooleanSupplier fieldRelativeSupplier
    ) {
        _drive = drive;
        _xSupplier = xSupplier;
        _ySupplier = ySupplier;
        _rotationSupplier = rotationSupplier;
        _fieldRelativeSupplier = fieldRelativeSupplier;
        addRequirements(drive);
    }

    @Override
    public void execute(double timestamp) {
        // Calculate chassis speeds
        ChassisSpeeds chassisSpeeds = _fieldRelativeSupplier.getAsBoolean()
            ? ChassisSpeeds.fromFieldRelativeSpeeds(
                _xSupplier.getAsDouble(),
                _ySupplier.getAsDouble(),
                _rotationSupplier.getAsDouble(),
                _drive.getHeading()
            )
            : new ChassisSpeeds(
                _xSupplier.getAsDouble(),
                _ySupplier.getAsDouble(),
                _rotationSupplier.getAsDouble()
            );

        // Set desired chassis speeds
        _drive.setDesiredChassisSpeeds(chassisSpeeds);
    }

    @Override
    public void end(boolean interrupted) {
        _drive.setDesiredChassisSpeeds(new ChassisSpeeds());
    }
}
