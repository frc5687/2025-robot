package org.frc5687.robot;

import org.frc5687.robot.subsystems.drive.DriveSubsystem;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandPS4Controller;

public class OperatorInterface {
    private final CommandPS4Controller _driverController;

    public OperatorInterface() {
        _driverController = new CommandPS4Controller(0);
    }

    public void configureCommandMapping(DriveSubsystem drive) {
        _driverController.povDown().onTrue(Commands.runOnce(drive::zeroGyroscope));
    }

    public CommandPS4Controller getDriverController()  {
        return _driverController;
    }
}
