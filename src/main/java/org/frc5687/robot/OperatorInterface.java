package org.frc5687.robot;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import org.frc5687.robot.commands.superstructure.SuperstructureFactory;
import org.frc5687.robot.util.Helpers;

public class OperatorInterface {
    private final CommandXboxController _driverController;

    public OperatorInterface() {
        _driverController = new CommandXboxController(0);
    }

    public void configureCommandMapping(RobotContainer container) {
        // _driverController
        //         .y()
        //         .onTrue(new ElevatorSetState(container.getElevator(),
        // ElevatorState.L3_CORAL_PLACING));
        // _driverController
        //         .x()
        //         .onTrue(new ElevatorSetState(container.getElevator(),
        // ElevatorState.L4_CORAL_PLACING));
        // _driverController
        //         .b()
        //         .onTrue(new ElevatorSetState(container.getElevator(),
        // ElevatorState.L2_CORAL_PLACING));
        // _driverController
        //         .a()
        //         .onTrue(new ElevatorSetState(container.getElevator(), ElevatorState.STOWED));

        _driverController.y().onTrue(SuperstructureFactory.placeCoralL4(container, false));
        _driverController.x().onTrue(SuperstructureFactory.receiveFromFunnel(container));
        _driverController.a().onTrue(SuperstructureFactory.placeCoralL3(container, true));
        _driverController.b().onTrue(SuperstructureFactory.receiveFromIntake(container));
        // _driverController.povDown().onTrue(Commands.runOnce(drive::zeroGyroscope));
        // _driverController.a().onTrue(drive.sysIdDynamic(SysIdRoutine.Direction.kForward));
        // _driverController.b().onTrue(drive.sysIdDynamic(SysIdRoutine.Direction.kReverse));
        // _driverController.y().onTrue(drive.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
        // _driverController.x().onTrue(drive.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));

        // _driverController.leftTrigger().whileTrue(new RunIntake(intake, 6, 6));

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

    public CommandXboxController getDriverController() {
        return _driverController;
    }
}
