package org.frc5687.robot;

import com.pathplanner.lib.commands.PathPlannerAuto;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.NotLogged;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import org.frc5687.robot.commands.drive.TeleopDriveCommand;
import org.frc5687.robot.subsystems.drive.*;
import org.frc5687.robot.subsystems.drive.modules.SwerveModule;
import org.frc5687.robot.util.Helpers;

public class RobotContainer {

    private final OperatorInterface _oi;
    @Logged private final DriveSubsystem _drive;
    @NotLogged private final SwerveModule[] _modules;

    private final Field2d _field;

    public RobotContainer() {
        _oi = new OperatorInterface();
        _field = new Field2d();
        _modules = new SwerveModule[Constants.SwerveModule.NUM_MODULES];

        DriveIO driveIO =
                RobotBase.isSimulation()
                        ? new SimDriveIO(RobotMap.CAN.PIGEON.PIGEON)
                        : new CTREDriveIO(RobotMap.CAN.PIGEON.PIGEON, Constants.SwerveModule.CAN_BUS);

        _drive = new DriveSubsystem(driveIO, Constants.DriveTrain.MODULE_LOCATIONS);

        if (RobotBase.isSimulation()) {
            SmartDashboard.putData("Field", _field);
        }

        configureDefaultCommands();
        _oi.configureCommandMapping(_drive);
    }

    private void configureDefaultCommands() {
        _drive.setDefaultCommand(
                new TeleopDriveCommand(
                        _drive,
                        () ->
                                -modifyAxis(_oi.getDriverController().getLeftY())
                                        * Constants.SwerveModule.MAX_LINEAR_SPEED,
                        () ->
                                -modifyAxis(_oi.getDriverController().getLeftX())
                                        * Constants.SwerveModule.MAX_LINEAR_SPEED,
                        () ->
                                -modifyAxis(_oi.getDriverController().getRightX())
                                        * Constants.SwerveModule.MAX_ANGULAR_SPEED,
                        () -> true // Always field relative
                        ));
    }

    public Command getAutonomousCommand() {
        return new PathPlannerAuto("Test Auto");
    }

    public void periodic() {
        if (RobotBase.isSimulation()) {
            _field.setRobotPose(_drive.getPose());
        }
    }

    // for not be lazy and just square input TODO: DONT
    private static double modifyAxis(double value) {
        value = Helpers.applyDeadband(value, 0.1);
        value = Math.copySign(value * value, value);

        return value;
    }
}
