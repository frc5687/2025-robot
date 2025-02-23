package org.frc5687.robot;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import org.frc5687.robot.commands.drive.DynamicDriveToReefBranch;
import org.frc5687.robot.subsystems.superstructure.RequestType;
import org.frc5687.robot.subsystems.superstructure.SuperstructureGoals;
import org.frc5687.robot.subsystems.superstructure.SuperstructureManager;
import org.frc5687.robot.util.FieldConstants.ReefHeight;
import org.frc5687.robot.util.OutliersController;
import org.frc5687.robot.util.ReefAlignmentHelpers.ReefSide;

public class OperatorInterface {
    private final OutliersController _driverController;
    private final OutliersController _operatorController;

    public OperatorInterface() {
        if (RobotBase.isReal()) {
            _driverController = new OutliersController(new CommandXboxController(0));
            _operatorController = new OutliersController(new CommandXboxController(1));
        } else {
            _driverController = new OutliersController(new CommandPS5Controller(0));
            _operatorController = new OutliersController(new CommandPS5Controller(1));
        }
    }

    public void configureCommandMapping(RobotContainer container) {
        SuperstructureManager manager = container.getSuperstructureManager();
        configureDriverControls(container, manager);
        configureOperatorControls(container, manager);
    }

    private void configureDriverControls(RobotContainer container, SuperstructureManager manager) {

        // Face angles with funnel receive
        _driverController
                .x()
                .onTrue(
                        new InstantCommand(
                                        () -> container.getDrive().enableHeadingController(Units.degreesToRadians(-60)))
                                .andThen(manager.receiveFunnel(RequestType.IMMEDIATE)));

        _driverController
                .b()
                .onTrue(
                        new InstantCommand(
                                        () -> container.getDrive().enableHeadingController(Units.degreesToRadians(60)))
                                .andThen(manager.receiveFunnel(RequestType.IMMEDIATE)));

        _driverController
                .leftBumper()
                .whileTrue(
                        new DynamicDriveToReefBranch(container.getDrive(), ReefSide.LEFT, ReefHeight.L4));
        _driverController
                .rightBumper()
                .whileTrue(
                        new DynamicDriveToReefBranch(container.getDrive(), ReefSide.RIGHT, ReefHeight.L4));

        _driverController.leftTrigger().whileTrue(manager.groundIntake(RequestType.IMMEDIATE));

        _driverController
                .rightTrigger()
                .onTrue(manager.placeAtCurrentHeight(RequestType.IMMEDIATE));

        _driverController.rightMiddleButton().onTrue(new InstantCommand(container.getDrive()::zeroIMU));
        _driverController
                .leftMiddleButton()
                .onTrue(new InstantCommand(container.getClimber()::toggleClimberSetpoint));
    }

    /** OPERATOR CONTROLS: Coral Mode Algae Mode L1 L2 L3 L4 Place Reef Place Processor */
    private void configureOperatorControls(RobotContainer container, SuperstructureManager manager) {
        // override queue
        _operatorController
                .back()
                .onTrue(new InstantCommand(manager::forceQueueExecution))
                .onFalse(new InstantCommand(manager::releaseQueueExecution));

        _operatorController
                .a()
                .onTrue(
                        manager.setToPlaceHeight(
                                SuperstructureGoals.PLACE_CORAL_L1, RequestType.QUEUED));
        _operatorController
                .b()
                .onTrue(
                        manager.setToPlaceHeight(
                                SuperstructureGoals.PLACE_CORAL_L2, RequestType.QUEUED));
        _operatorController
                .x()
                .onTrue(
                        manager.setToPlaceHeight(
                                SuperstructureGoals.PLACE_CORAL_L3, RequestType.QUEUED));
        _operatorController
                .y()
                .onTrue(
                        manager.setToPlaceHeight(
                                SuperstructureGoals.PLACE_CORAL_L4, RequestType.QUEUED));

        _operatorController
                .leftBumper()
                .whileTrue(
                        manager.grabAlgae(SuperstructureGoals.L2_ALGAE_GRAB, RequestType.QUEUED));
        _operatorController
                .rightBumper()
                .whileTrue(
                        manager.grabAlgae(SuperstructureGoals.L1_ALGAE_GRAB, RequestType.QUEUED));

        _operatorController
                .rightTrigger()
                .onTrue(manager.receiveFunnel(RequestType.IMMEDIATE));
    }

    public OutliersController getDriverController() {
        return _driverController;
    }

    public OutliersController getOperatorController() {
        return _operatorController;
    }
}
