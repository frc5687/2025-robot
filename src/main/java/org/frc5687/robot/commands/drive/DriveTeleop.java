package org.frc5687.robot.commands.drive;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj.Timer;

import java.util.Optional;
import java.util.function.DoubleSupplier;

import org.frc5687.robot.Constants;
import org.frc5687.robot.util.Helpers;
import org.frc5687.robot.subsystems.drive.DriveSubsystem;
import org.frc5687.robot.util.math.Vector2d;

public class DriveTeleop extends Command {
    private final DriveSubsystem _drive;
    private final DoubleSupplier _throttle;
    private final DoubleSupplier _strafe;
    private final DoubleSupplier _rotation;
    private final boolean _fieldRelative;
    
    private final int[] _segmentationArray;
    private Optional<Rotation2d> _headingSetpoint = Optional.empty();
    private double _joystickLastTouched = -1;

    public DriveTeleop(
        DriveSubsystem drive, 
        DoubleSupplier throttle,
        DoubleSupplier strafe,
        DoubleSupplier rotation,
        boolean fieldRelative
    ) {
        _drive = drive;
        _throttle = throttle;
        _strafe = strafe;
        _rotation = rotation;
        _fieldRelative = fieldRelative;
        
        _segmentationArray = new int[360 / 5];
        for (int i = 0; i < _segmentationArray.length; i++) {
            _segmentationArray[i] = (360 / _segmentationArray.length) * i;
        }
        
        addRequirements(drive);
    }

    @Override
    public void initialize() {
        _headingSetpoint = Optional.empty();
    }

    @Override
    public void execute() {
        double throttle = _throttle.getAsDouble();
        double strafe = _strafe.getAsDouble();
        double rotation = _rotation.getAsDouble();
        
        throttle = MathUtil.applyDeadband(throttle, Constants.DriveTrain.TRANSLATION_DEADBAND);
        strafe = MathUtil.applyDeadband(strafe, Constants.DriveTrain.TRANSLATION_DEADBAND);
        rotation = MathUtil.applyDeadband(rotation, Constants.DriveTrain.ROTATION_DEADBAND);

        rotation = Math.signum(rotation) * rotation * rotation;

        if (Math.abs(rotation) > Constants.DriveTrain.ROTATION_DEADBAND) {
            _joystickLastTouched = Timer.getFPGATimestamp();
            _headingSetpoint = Optional.empty();
        }

        Vector2d vec = Helpers.axisToSegmentedUnitCircleRadians(throttle, strafe, _segmentationArray);
        
        double vx = vec.x() * Constants.DriveTrain.MAX_MPS;
        double vy = vec.y() * Constants.DriveTrain.MAX_MPS;
        double omega = rotation * Constants.DriveTrain.MAX_ANG_VEL;

        Rotation2d heading = _drive.getHeading();

        if (isRedAlliance()) {
            heading = heading.plus(new Rotation2d(Math.PI));
        }

        ChassisSpeeds speeds;
        if (_fieldRelative) {
            if (Math.abs(rotation) > Constants.DriveTrain.ROTATION_DEADBAND) {
                speeds = ChassisSpeeds.fromFieldRelativeSpeeds(vx, vy, omega, heading);
            } else {
                if (_headingSetpoint.isEmpty()) {
                    _headingSetpoint = Optional.of(heading);
                }
                speeds = ChassisSpeeds.fromFieldRelativeSpeeds(vx, vy, 0, heading);
            }
        } else {
            speeds = new ChassisSpeeds(vx, vy, omega);
        }

        _drive.setVelocity(speeds);
    }

    @Override
    public void end(boolean interrupted) {
        _drive.setVelocity(new ChassisSpeeds());
    }

    private boolean isRedAlliance() {
        var alliance = DriverStation.getAlliance();
        return alliance.isPresent() && alliance.get() == Alliance.Red;
    }
}