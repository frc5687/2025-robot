package org.frc5687.robot;

import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import org.frc5687.robot.commands.drive.DynamicDriveToReefBranch;
import org.frc5687.robot.commands.elevator.ElevatorSetState;
import org.frc5687.robot.commands.superstructure.SuperstructureFactory;
import org.frc5687.robot.subsystems.elevator.ElevatorState;
import org.frc5687.robot.util.FieldConstants.ReefHeight;
import org.frc5687.robot.util.Helpers;
import org.frc5687.robot.util.ReefAlignmentHelpers.ReefSide;

public class OperatorInterface {
    private final CommandXboxController _driverController;
    private final CommandXboxController _operatorController;

    public OperatorInterface() {
        _driverController = new CommandXboxController(0);
        _operatorController = new CommandXboxController(1);
    }

    public void configureCommandMapping(RobotContainer container) {
        _driverController.x().onTrue(new InstantCommand(()->{}));
        _driverController.x().onTrue(SuperstructureFactory.receiveFromFunnel(container));
        _driverController.b().onTrue(new InstantCommand(()->{}));
        _driverController.b().onTrue(SuperstructureFactory.receiveFromFunnel(container));

        _driverController.leftBumper().whileTrue(new DynamicDriveToReefBranch(container.getDrive(), ReefSide.LEFT, ReefHeight.L4));
        _driverController.rightBumper().whileTrue(new DynamicDriveToReefBranch(container.getDrive(), ReefSide.RIGHT, ReefHeight.L4));

        _driverController.leftTrigger().onTrue(SuperstructureFactory.place(container)); // TODO place based on held
        _driverController.rightTrigger().whileTrue(new ConditionalCommand(null, null, null)); // TODO intake based on mode


        /**
         * OPERATOR CONTROLS:
         * Coral Mode
         * Algae Mode
         * L1
         * L2
         * L3
         * L4
         * Place Reef
         * Place Processor 
         */

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
