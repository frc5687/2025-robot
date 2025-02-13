package org.frc5687.robot;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import org.frc5687.robot.commands.drive.DynamicDriveToReefBranch;
import org.frc5687.robot.commands.intake.IntakeSetState;
import org.frc5687.robot.commands.superstructure.SuperstructureFactory;
import org.frc5687.robot.subsystems.intake.IntakeState;
import org.frc5687.robot.util.FieldConstants.ReefHeight;
import org.frc5687.robot.util.ReefAlignmentHelpers.ReefSide;

public class OperatorInterface {
    private final CommandXboxController _driverController;
    private final CommandXboxController _operatorController;

    public OperatorInterface() {
        _driverController = new CommandXboxController(0);
        _operatorController = new CommandXboxController(1);
    }

    public void configureCommandMapping(RobotContainer container) {
        _driverController
                .x()
                .onTrue(
                        new InstantCommand(
                                () -> {
                                    container.getDrive().enableHeadingController(Units.degreesToRadians(180 - 60));
                                }));
        _driverController.x().onTrue(SuperstructureFactory.receiveFromFunnel(container));
        _driverController
                .b()
                .onTrue(
                        new InstantCommand(
                                () -> {
                                    container.getDrive().enableHeadingController(Units.degreesToRadians(180 + 60));
                                }));
        _driverController.b().onTrue(SuperstructureFactory.receiveFromFunnel(container));

        _driverController
                .leftBumper()
                .whileTrue(
                        new DynamicDriveToReefBranch(container.getDrive(), ReefSide.LEFT, ReefHeight.L4));
        _driverController
                .rightBumper()
                .whileTrue(
                        new DynamicDriveToReefBranch(container.getDrive(), ReefSide.RIGHT, ReefHeight.L4));
        _driverController
                .leftBumper()
                .whileTrue(
                        new DynamicDriveToReefBranch(container.getDrive(), ReefSide.LEFT, ReefHeight.L4));
        _driverController
                .rightBumper()
                .whileTrue(
                        new DynamicDriveToReefBranch(container.getDrive(), ReefSide.RIGHT, ReefHeight.L4));

        _driverController
                .leftTrigger()
                .onTrue(SuperstructureFactory.place(container)); // TODO place based on held
        _driverController
                .rightTrigger()
                .onTrue(new IntakeSetState(container.getIntake(), IntakeState.DEPLOYED));
        _driverController.rightTrigger().onTrue(SuperstructureFactory.groundIntakeHandoff(container));
        // _driverController
        //         .rightTrigger()
        //         .whileTrue(new ConditionalCommand(null, null, false)); // TODO intake based on mode

        /** OPERATOR CONTROLS: Coral Mode Algae Mode L1 L2 L3 L4 Place Reef Place Processor */
    }

    public CommandXboxController getDriverController() {
        return _driverController;
    }

    public CommandXboxController getOperatorController() {
        return _operatorController;
    }
}
