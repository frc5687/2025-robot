package org.frc5687.robot;

import org.frc5687.robot.subsystems.drive.DriveSubsystem;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public class OperatorInterface {
    private final CommandXboxController _driverController;

    public OperatorInterface() {
        _driverController = new CommandXboxController(0);
    }

    public void configureCommandMapping(DriveSubsystem drive) {
        _driverController.povDown().onTrue(Commands.runOnce(drive::zeroGyroscope));
    }

    public CommandXboxController getDriverController()  {
        return _driverController;
    }
}
