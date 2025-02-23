package org.frc5687.robot;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandPS4Controller;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import org.frc5687.robot.commands.coral.EjectCoral;
import org.frc5687.robot.commands.drive.DynamicDriveToReefBranch;
import org.frc5687.robot.commands.superstructure.SetSuperstructure;
import org.frc5687.robot.commands.superstructure.SuperstructureFactory;
import org.frc5687.robot.subsystems.superstructure.SuperstructureGoals;
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
            _operatorController = new OutliersController(new CommandPS4Controller(1));
        }
    }

    public void configureDriverMapping(RobotContainer container) {
        var receiveFromFunnel = SuperstructureFactory.receiveFromFunnel(container);
        _driverController
                .x()
                .onTrue(
                        new InstantCommand(
                                () -> {
                                    container.getDrive().enableHeadingController(Units.degreesToRadians(-60));
                                }));
        _driverController.x().onTrue(receiveFromFunnel);
        _driverController
                .b()
                .onTrue(
                        new InstantCommand(
                                () -> {
                                    container.getDrive().enableHeadingController(Units.degreesToRadians(60));
                                }));
        _driverController.b().onTrue(receiveFromFunnel);

        _driverController
                .leftBumper()
                .whileTrue(
                        new DynamicDriveToReefBranch(container.getDrive(), ReefSide.LEFT, ReefHeight.L4));
        _driverController
                .rightBumper()
                .whileTrue(
                        new DynamicDriveToReefBranch(container.getDrive(), ReefSide.RIGHT, ReefHeight.L4));
        _driverController.rightTrigger().whileTrue(new EjectCoral(container));

        _driverController
                .leftMiddleButton()
                .onTrue(new InstantCommand(container.getClimber()::toggleClimberSetpoint));
        _driverController.rightMiddleButton().onTrue(new InstantCommand(container.getDrive()::zeroIMU));
    }

    public void configureOperatorMapping(RobotContainer container) {
        /** OPERATOR CONTROLS: Coral Mode Algae Mode L1 L2 L3 L4 Place Reef Place Processor */
        _operatorController
                .a()
                .onTrue(new SetSuperstructure(container, SuperstructureGoals.PLACE_CORAL_L1));
        _operatorController
                .b()
                .onTrue(new SetSuperstructure(container, SuperstructureGoals.PLACE_CORAL_L2));
        _operatorController
                .x()
                .onTrue(new SetSuperstructure(container, SuperstructureGoals.PLACE_CORAL_L3));
        _operatorController
                .y()
                .onTrue(new SetSuperstructure(container, SuperstructureGoals.PLACE_CORAL_L4));

        // _operatorController.x().onTrue(new SetSuperstructure(container,
        // Optional.empty(),Optional.empty(),Optional.empty(),Optional.of(CoralState.STOWED)));
        // _operatorController.y().onTrue(new SetSuperstructure(container,
        // Optional.empty(),Optional.empty(),Optional.empty(),Optional.of(CoralState.)));

        _operatorController.leftBumper().whileTrue(SuperstructureFactory.grabAlgaeL2(container));
        _operatorController.rightBumper().whileTrue(SuperstructureFactory.grabAlgaeL1(container));
    }

    public OutliersController getDriverController() {
        return _driverController;
    }

    public OutliersController getOperatorController() {
        return _operatorController;
    }
}
