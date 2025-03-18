package org.frc5687.robot.commands.drive;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import java.util.function.DoubleSupplier;
import org.frc5687.robot.Constants;
import org.frc5687.robot.RobotContainer;
import org.frc5687.robot.commands.OutliersCommand;
import org.frc5687.robot.subsystems.drive.DriveSubsystem;
import org.frc5687.robot.util.FieldConstants;

public class DynamicDriveToNet extends OutliersCommand {
    private final DriveSubsystem _drive;
    private final DoubleSupplier _joystickSupplier;

    private final ProfiledPIDController _xController;
    private final ProfiledPIDController _thetaController;

    private Alliance _alliance;

    public DynamicDriveToNet(DriveSubsystem drive, DoubleSupplier joystickSupplier) {
        _drive = drive;
        _joystickSupplier = joystickSupplier;
        _xController =
                new ProfiledPIDController(3, 0.0, 0.001, new TrapezoidProfile.Constraints(3.0, 1.0));
        _thetaController =
                new ProfiledPIDController(
                        4.0, 0.0, 0.0, new TrapezoidProfile.Constraints(2.0 * Math.PI, 4 * Math.PI));
        _thetaController.enableContinuousInput(-Math.PI, Math.PI);

        addRequirements(drive);
    }

    @Override
    public void initialize() {
        Pose2d currentPose = _drive.getPose();
        var alliance = DriverStation.getAlliance();
        if (alliance.isPresent() && alliance.get() == Alliance.Red) {
            _xController.setGoal(FieldConstants.fieldLength / 2.0 + Units.inchesToMeters(40));
            _thetaController.setGoal(Math.PI);
            _alliance = Alliance.Red;
        } else {
            _xController.setGoal(FieldConstants.fieldLength / 2.0 - Units.inchesToMeters(40));
            _thetaController.setGoal(0.0);
            _alliance = Alliance.Blue;
        }

        _xController.reset(currentPose.getX(), 0.0); // FIXME add measured x velocity
        _thetaController.reset(_drive.getHeading().getRadians(), _drive.getAngularVelocityYaw());
    }

    @Override
    public void execute(double timestamp) {
        double currentX = _drive.getPose().getX();
        Rotation2d currentTheta = _drive.getHeading();
        double yVel =
                RobotContainer.modifyAxis(_joystickSupplier.getAsDouble()) * Constants.DriveTrain.MAX_MPS;
        log(
                "DriveToPose Target",
                new Pose2d(
                        _xController.getSetpoint().position,
                        _drive.getPose().getY(),
                        new Rotation2d(_thetaController.getSetpoint().position)),
                Pose2d.struct);

        _drive.setDesiredChassisSpeeds(
                ChassisSpeeds.fromFieldRelativeSpeeds(
                        _xController.calculate(currentX),
                        _alliance == Alliance.Red ? yVel : -yVel,
                        _thetaController.calculate(currentTheta.getRadians()),
                        currentTheta));
    }

    @Override
    public boolean isFinished() {
        return _xController.atGoal() && _thetaController.atGoal();
    }

    @Override
    public void end(boolean interrupted) {
        _drive.setDesiredChassisSpeeds(new ChassisSpeeds());
    }
}
