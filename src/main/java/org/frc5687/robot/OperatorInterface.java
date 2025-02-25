package org.frc5687.robot;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.button.CommandPS4Controller;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import java.util.Optional;
import org.frc5687.robot.commands.algae.EjectAlgae;
import org.frc5687.robot.commands.algae.IntakeAlgae;
import org.frc5687.robot.commands.coral.EjectCoral;
import org.frc5687.robot.commands.drive.DynamicDriveToReefBranch;
import org.frc5687.robot.subsystems.algaearm.AlgaeState;
import org.frc5687.robot.subsystems.superstructure.RequestType;
import org.frc5687.robot.subsystems.superstructure.SuperstructureManager;
import org.frc5687.robot.subsystems.superstructure.SuperstructureState;
import org.frc5687.robot.util.FieldConstants;
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

    public void configureCommandMapping(RobotContainer container) {
        SuperstructureManager manager = container.getSuperstructureManager();
        configureDriverControls(container, manager);
        configureOperatorControls(container, manager);
    }

    private void configureDriverControls(RobotContainer container, SuperstructureManager manager) {

        // TODO when the elevator is L1 with no coral it should automatically intake (might have to
        // queue this so it doesn't collide with the reef)

        // Face angles with funnel receive
        _driverController
                .x()
                .onTrue(
                        new InstantCommand(
                                () -> container.getDrive().enableHeadingController(Units.degreesToRadians(-60))));

        _driverController
                .b()
                .onTrue(
                        new InstantCommand(
                                () -> container.getDrive().enableHeadingController(Units.degreesToRadians(60))));

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
                .whileTrue(
                        new SequentialCommandGroup(
                                manager.grabAlgae(
                                        Constants.SuperstructureGoals.GROUND_PICKUP, RequestType.IMMEDIATE),
                                new IntakeAlgae(container.getAlgae()),
                                manager.createRequest(
                                        new SuperstructureState(
                                                Optional.empty(),
                                                Optional.empty(),
                                                Optional.of(AlgaeState.IDLE),
                                                Optional.empty()),
                                        RequestType.IMMEDIATE),
                                new InstantCommand(() -> container.getAlgae().setWheelMotorVoltage(0))));

        _driverController.rightTrigger().whileTrue(new EjectCoral(container.getCoral()));

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
                        manager.createRequest(
                                Constants.SuperstructureGoals.PLACE_CORAL_L1, RequestType.QUEUED));
        _operatorController
                .b()
                .onTrue(
                        manager.createRequest(
                                Constants.SuperstructureGoals.PLACE_CORAL_L2, RequestType.QUEUED));
        _operatorController
                .x()
                .onTrue(
                        manager.createRequest(
                                Constants.SuperstructureGoals.PLACE_CORAL_L3, RequestType.QUEUED));
        _operatorController
                .y()
                .onTrue(
                        manager.createRequest(
                                Constants.SuperstructureGoals.PLACE_CORAL_L4, RequestType.QUEUED));

        _operatorController
                .leftBumper()
                .whileTrue(
                        new SequentialCommandGroup(
                                manager.grabAlgae(
                                        Constants.SuperstructureGoals.LOW_ALGAE_GRAB, RequestType.IMMEDIATE),
                                new IntakeAlgae(container.getAlgae()),
                                new WaitUntilCommand(
                                        () ->
                                                container
                                                                .getDrive()
                                                                .getPose()
                                                                .getTranslation()
                                                                .getDistance(FieldConstants.getAllianceSpecificReefCenter())
                                                        > 2),
                                manager.createRequest(
                                        new SuperstructureState(
                                                Optional.empty(),
                                                Optional.empty(),
                                                Optional.of(AlgaeState.IDLE),
                                                Optional.empty()),
                                        RequestType.IMMEDIATE),
                                new InstantCommand(() -> container.getAlgae().setWheelMotorVoltage(0))));
        _operatorController
                .rightBumper()
                .whileTrue(
                        new SequentialCommandGroup(
                                manager.grabAlgae(
                                        Constants.SuperstructureGoals.HIGH_ALGAE_GRAB, RequestType.IMMEDIATE),
                                new IntakeAlgae(container.getAlgae()),
                                new WaitUntilCommand(
                                        () ->
                                                container
                                                                .getDrive()
                                                                .getPose()
                                                                .getTranslation()
                                                                .getDistance(FieldConstants.getAllianceSpecificReefCenter())
                                                        > 2),
                                manager.createRequest(
                                        new SuperstructureState(
                                                Optional.empty(),
                                                Optional.empty(),
                                                Optional.of(AlgaeState.IDLE),
                                                Optional.empty()),
                                        RequestType.IMMEDIATE),
                                new InstantCommand(() -> container.getAlgae().setWheelMotorVoltage(0))));

        _operatorController.povDown().onTrue(manager.receiveFunnel(RequestType.IMMEDIATE));
        _operatorController
                .rightTrigger()
                .whileTrue(
                        new SequentialCommandGroup(
                                manager.createRequest(
                                        Constants.SuperstructureGoals.BARGE_HELD, RequestType.IMMEDIATE),
                                manager.createRequest(
                                        Constants.SuperstructureGoals.BARGE_DROPOFF, RequestType.IMMEDIATE),
                                new EjectAlgae(container.getAlgae())));
                        _operatorController
                                .rightTrigger()
                                .whileTrue(
                                        manager.createRequest(
                                                Constants.SuperstructureGoals.PROCESSOR_DROPOFF, RequestType.IMMEDIATE)).onFalse(
                                                new EjectAlgae(container.getAlgae()));
        
    }

    public OutliersController getDriverController() {
        return _driverController;
    }

    public OutliersController getOperatorController() {
        return _operatorController;
    }
}
