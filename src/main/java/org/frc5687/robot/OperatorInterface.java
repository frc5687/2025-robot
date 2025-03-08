package org.frc5687.robot;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Radians;

import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import java.util.Optional;
import org.frc5687.robot.commands.algae.EjectAlgae;
import org.frc5687.robot.commands.algae.EmergencyEjectAlgae;
import org.frc5687.robot.commands.algae.IntakeAlgae;
import org.frc5687.robot.commands.coral.EjectCoral;
import org.frc5687.robot.commands.drive.DriveToHP;
import org.frc5687.robot.commands.drive.DynamicDriveToReefBranch;
import org.frc5687.robot.commands.drive.TeleopDriveWithSnapTo;
import org.frc5687.robot.subsystems.algaearm.AlgaeState;
import org.frc5687.robot.subsystems.superstructure.RequestType;
import org.frc5687.robot.subsystems.superstructure.SuperstructureManager;
import org.frc5687.robot.subsystems.superstructure.SuperstructureState;
import org.frc5687.robot.util.FieldConstants;
import org.frc5687.robot.util.Helpers;
import org.frc5687.robot.util.OutliersController;
import org.frc5687.robot.util.ReefAlignmentHelpers.ReefSide;

public class OperatorInterface {
    private final OutliersController _driverController;
    private final OutliersController _operatorController;

    public OperatorInterface() {
        _driverController = new OutliersController(new CommandPS5Controller(0));
        _operatorController = new OutliersController(new CommandPS5Controller(1));
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
                        new TeleopDriveWithSnapTo(
                                Degrees.of(-53).in(Radians),
                                container.getDrive(),
                                () ->
                                        -RobotContainer.modifyAxis(getDriverController().getLeftY())
                                                * Constants.SwerveModule.MAX_LINEAR_SPEED,
                                () ->
                                        -RobotContainer.modifyAxis(getDriverController().getLeftX())
                                                * Constants.SwerveModule.MAX_LINEAR_SPEED,
                                () ->
                                        -RobotContainer.modifyAxis(getDriverController().getRightX())
                                                * Constants.SwerveModule.MAX_ANGULAR_SPEED,
                                () -> true)); // Always field relative

        _driverController
                .b()
                .onTrue(
                        new TeleopDriveWithSnapTo(
                                Degrees.of(53).in(Radians),
                                container.getDrive(),
                                () ->
                                        -RobotContainer.modifyAxis(getDriverController().getLeftY())
                                                * Constants.SwerveModule.MAX_LINEAR_SPEED,
                                () ->
                                        -RobotContainer.modifyAxis(getDriverController().getLeftX())
                                                * Constants.SwerveModule.MAX_LINEAR_SPEED,
                                () ->
                                        -RobotContainer.modifyAxis(getDriverController().getRightX())
                                                * Constants.SwerveModule.MAX_ANGULAR_SPEED,
                                () -> true)); // Always field relative

        _driverController
                .leftBumper()
                .whileTrue(
                        new ConditionalCommand(
                                new DynamicDriveToReefBranch(container.getDrive(), ReefSide.ALGAE),
                                new DynamicDriveToReefBranch(container.getDrive(), ReefSide.LEFT),
                                manager::isAlgaeMode));

        _driverController
                .rightBumper()
                .whileTrue(
                        new ConditionalCommand(
                                new DynamicDriveToReefBranch(container.getDrive(), ReefSide.ALGAE),
                                new DynamicDriveToReefBranch(container.getDrive(), ReefSide.RIGHT),
                                manager::isAlgaeMode));

        _driverController
                .leftJoystickButton()
                .whileTrue(new DynamicDriveToReefBranch(container.getDrive(), ReefSide.ALGAE));
        _driverController.rightJoystickButton().whileTrue(new DriveToHP(container.getDrive()));

        // _driverController.leftJoystickButton().whileTrue(new
        // EmergencyEjectAlgae(container.getAlgae()));
        _driverController.povUp().whileTrue(new EmergencyEjectAlgae(container.getAlgae()));
        _driverController.povUpLeft().whileTrue(new EmergencyEjectAlgae(container.getAlgae()));
        _driverController.povUpRight().whileTrue(new EmergencyEjectAlgae(container.getAlgae()));

        _driverController
                .leftTrigger()
                .whileTrue(
                        new ConditionalCommand(
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
                                        new InstantCommand(() -> container.getAlgae().setWheelMotorVoltage(0))),
                                new InstantCommand(),
                                manager::isAlgaeMode));

        _driverController
                .rightTrigger()
                .whileTrue(
                        new ConditionalCommand(
                                // new ConditionalCommand(
                                new EjectAlgae(container.getAlgae()),
                                // manager.createRequest(
                                //         Constants.SuperstructureGoals.BARGE_DROPOFF, RequestType.IMMEDIATE),
                                // () ->
                                //         container.getElevator().getElevatorHeight()
                                //                 < ElevatorState.L3_CORAL_PLACING.getHeight()),
                                new EjectCoral(container.getCoral()),
                                manager::isAlgaeMode));

        _driverController.rightMiddleButton().onTrue(new InstantCommand(container.getDrive()::zeroIMU));

        _driverController
                .leftMiddleButton()
                .onTrue(new InstantCommand(container.getClimber()::toggleClimberSetpoint));

        // _driverController
        //         .povDown()
        //         .whileTrue(
        //                 new DynamicDriveToLane(
        //                         container.getDrive(),
        //                         () ->
        //                                 -modifyAxis(_driverController.getLeftY())
        //                                         * Constants.SwerveModule.MAX_LINEAR_SPEED));
    }

    /** OPERATOR CONTROLS: Coral Mode Algae Mode L1 L2 L3 L4 Place Reef Place Processor */
    private void configureOperatorControls(RobotContainer container, SuperstructureManager manager) {
        // toggle mode (just in case manual override is needed)

        _operatorController
                .start()
                .onTrue(
                        new SequentialCommandGroup(
                                        new InstantCommand(manager::toggleMode),
                                        new InstantCommand(
                                                () ->
                                                        System.out.println(
                                                                "Mode explicitly toggled to: " + manager.getCurrentMode())))
                                .withName("Mode Toggle Button"));

        _operatorController
                .back()
                .onTrue(new InstantCommand(manager::forceQueueExecution))
                .onFalse(new InstantCommand(manager::releaseQueueExecution));

        /*
         * Mode-aware L1 action (x button)
         * x() without coral should default to receiveFunnel and set CORAL mode
         * if algae held:
         *      processor dropoff
         * elif coral held:
         *      coral trough
         *      wait until (coral not held & far from reef)
         *      receive from funnel
         * else:
         *      receive from funnel
         */
        _operatorController
                .x()
                .onTrue(
                        new ConditionalCommand(
                                        manager.grabAlgae(
                                                Constants.SuperstructureGoals.PROCESSOR_DROPOFF, RequestType.IMMEDIATE),
                                        new ConditionalCommand(
                                                new SequentialCommandGroup(
                                                        manager.createRequest(
                                                                Constants.SuperstructureGoals.PLACE_CORAL_L1,
                                                                RequestType.IMMEDIATE),
                                                        new WaitUntilCommand(
                                                                () ->
                                                                        !container.getCoral().isCoralDetected()
                                                                                && container
                                                                                                .getDrive()
                                                                                                .getPose()
                                                                                                .getTranslation()
                                                                                                .getDistance(
                                                                                                        FieldConstants.getAllianceSpecificReefCenter())
                                                                                        > 2),
                                                        manager.receiveFunnel(RequestType.IMMEDIATE)),
                                                manager.receiveFunnel(RequestType.IMMEDIATE),
                                                container.getCoral()::isCoralDetected),
                                        container.getAlgae()::isAlgaeDetected)
                                .withName("L1 Action"));

        // L2 action (y button)
        _operatorController
                .y()
                .onTrue(
                        new ConditionalCommand(
                                        new InstantCommand(() -> System.out.println("ALGAE mode L2 action")),
                                        manager.createRequest(
                                                Constants.SuperstructureGoals.PLACE_CORAL_L2, RequestType.QUEUED),
                                        manager::isAlgaeMode)
                                .withName("L2 Height Action"));

        // L3 action (b button)
        _operatorController
                .b()
                .onTrue(
                        new ConditionalCommand(
                                        new InstantCommand(() -> System.out.println("ALGAE mode L3 action")),
                                        manager.createRequest(
                                                Constants.SuperstructureGoals.PLACE_CORAL_L3, RequestType.QUEUED),
                                        manager::isAlgaeMode)
                                .withName("L3 Height Action"));

        /*
         * L4/Net action (a button) - functionality depends on mode but doesn't change mode
         * if algae held:
         *      net dropoff
         * else:
         *      coral l4
         */
        _operatorController
                .a()
                .onTrue(
                        new ConditionalCommand(
                                        manager.aimAtAlgaeNet(),
                                        manager.createRequest(
                                                Constants.SuperstructureGoals.PLACE_CORAL_L4, RequestType.QUEUED),
                                        manager::isAlgaeMode)
                                .withName("L4/Net Action"));

        // Low grab (left bumper) - always performs algae operations
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
                                        new InstantCommand(() -> container.getAlgae().setWheelMotorVoltage(0)))
                                .withName("Algae Low Grab"));

        // High grab (right bumper) - always performs algae operations
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
                                        new InstantCommand(() -> container.getAlgae().setWheelMotorVoltage(0)))
                                .withName("Algae High Grab"));
    }

    public static double modifyAxis(double value) {
        value = Helpers.applyDeadband(value, 0.1);
        value = Math.copySign(value * value, value);

        return value;
    }

    public OutliersController getDriverController() {
        return _driverController;
    }

    public OutliersController getOperatorController() {
        return _operatorController;
    }
}
