package org.frc5687.robot;

import org.frc5687.robot.commands.ServoCommand;
import org.frc5687.robot.commands.intake.RunIntake;
import org.frc5687.robot.subsystems.ServoTest;
import org.frc5687.robot.subsystems.drive.DriveSubsystem;
import org.frc5687.robot.subsystems.intake.IntakeSubsystem;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

public class OperatorInterface {
    private final CommandXboxController _driverController;

    public OperatorInterface() {
        _driverController = new CommandXboxController(0);
    }

    public void configureCommandMapping(IntakeSubsystem intake, ServoTest servoTest) {
        //_driverController.povDown().onTrue(Commands.runOnce(drive::zeroGyroscope));
        // _driverController.a().onTrue(drive.sysIdDynamic(SysIdRoutine.Direction.kForward));
        // _driverController.b().onTrue(drive.sysIdDynamic(SysIdRoutine.Direction.kReverse));
        // _driverController.y().onTrue(drive.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
        // _driverController.x().onTrue(drive.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));

        _driverController.leftTrigger().whileTrue(new RunIntake(intake, 6, 6));
        _driverController.povLeft().whileTrue(new ServoCommand(servoTest, 1));
        _driverController.povRight().whileTrue(new ServoCommand(servoTest, -1.0));
    }

    public CommandXboxController getDriverController()  {
        return _driverController;
    }
}
