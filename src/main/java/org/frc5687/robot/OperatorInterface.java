package org.frc5687.robot;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import org.frc5687.robot.subsystems.drive.DriveSubsystem;

public class OperatorInterface {
    private final CommandXboxController _driverController;

    public OperatorInterface() {
        _driverController = new CommandXboxController(0);
    }

    public void configureCommandMapping(DriveSubsystem drive) {
        // _driverController.povDown().onTrue(Commands.runOnce(drive::zeroGyroscope));
        // _driverController.a().onTrue(drive.sysIdDynamic(SysIdRoutine.Direction.kForward));
        // _driverController.b().onTrue(drive.sysIdDynamic(SysIdRoutine.Direction.kReverse));
        // _driverController.y().onTrue(drive.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
        // _driverController.x().onTrue(drive.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    }

    public CommandXboxController getDriverController() {
        return _driverController;
    }
}
