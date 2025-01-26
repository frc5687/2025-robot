package org.frc5687.robot;

import org.frc5687.robot.commands.elevator.SetElevatorPosition;
import org.frc5687.robot.commands.intake.RunIntake;
import org.frc5687.robot.subsystems.drive.DriveSubsystem;
import org.frc5687.robot.subsystems.elevator.ElevatorSubsystem;
import org.frc5687.robot.subsystems.intake.IntakeSubsystem;

import org.frc5687.robot.util.Helpers;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

public class OperatorInterface {
    private final CommandXboxController _driverController;

    public OperatorInterface() {
        _driverController = new CommandXboxController(0);
    }

    public void configureCommandMapping(DriveSubsystem drive, ElevatorSubsystem elevator) {
        _driverController.a().onTrue(new SetElevatorPosition(elevator, () -> {return 10;}));
        _driverController.b().onTrue(new SetElevatorPosition(elevator, () -> {return 15;}));

        _driverController.x().onTrue(new SetElevatorPosition(elevator, () -> {return 30;}));

        _driverController.y().onTrue(new SetElevatorPosition(elevator, () -> {return 35;}));
        //_driverController.povDown().onTrue(Commands.runOnce(drive::zeroGyroscope));
        // _driverController.a().onTrue(drive.sysIdDynamic(SysIdRoutine.Direction.kForward));
        // _driverController.b().onTrue(drive.sysIdDynamic(SysIdRoutine.Direction.kReverse));
        // _driverController.y().onTrue(drive.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
        // _driverController.x().onTrue(drive.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));

        //_driverController.leftTrigger().whileTrue(new RunIntake(intake, 6, 6));
        
    }

    public double getAxisY() {
        double speed = _driverController.getLeftY();
        speed = Helpers.applyDeadband(speed, Constants.DriveTrain.TRANSLATION_DEADBAND);
        return speed;
    }

    public double getAxisX() {
        double speed = _driverController.getRightX();
        speed = Helpers.applyDeadband(speed, Constants.DriveTrain.TRANSLATION_DEADBAND);
        return speed;
    }

    
    
    public CommandXboxController getDriverController()  {
        return _driverController;
    }
}
