package org.frc5687.robot;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Radians;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
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
import org.frc5687.robot.commands.drive.DynamicDriveToLane;
import org.frc5687.robot.commands.drive.DynamicDriveToReefBranch;
import org.frc5687.robot.commands.drive.TeleopDriveWithSnapTo;
import org.frc5687.robot.subsystems.algaearm.AlgaeState;
import org.frc5687.robot.subsystems.elevator.ElevatorState;
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
                .whileTrue(new DynamicDriveToReefBranch(container.getDrive(), ReefSide.LEFT));
        // _driverController
        //         .leftBumper()
        //         .whileTrue(new DriveToTag(container.getDrive(), container.getVision(),
        // ReefSide.LEFT));

        _driverController
                .rightBumper()
                .whileTrue(new DynamicDriveToReefBranch(container.getDrive(), ReefSide.RIGHT));

        // _driverController
        //         .rightBumper()
        //         .whileTrue(new DriveToTag(container.getDrive(), container.getVision(),
        // ReefSide.RIGHT));

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

        _driverController
                .rightTrigger()
                .whileTrue(
                        new ConditionalCommand(
                                new ConditionalCommand(
                                        new EjectAlgae(container.getAlgae()),
                                        manager.createRequest(
                                                Constants.SuperstructureGoals.BARGE_DROPOFF, RequestType.IMMEDIATE),
                                        () ->
                                                container.getElevator().getElevatorHeight()
                                                        < ElevatorState.L3_CORAL_PLACING.getHeight()),
                                new EjectCoral(container.getCoral()),
                                container.getAlgae()::isAlgaeDetected));
        _driverController.rightMiddleButton().onTrue(new InstantCommand(container.getDrive()::zeroIMU));

        // _driverController
        //         .leftMiddleButton()
        //         .onTrue(new InstantCommand(container.getClimber()::toggleClimberSetpoint));

        // _driverController
        //         .leftMiddleButton()
        //         .onTrue(
        //                 new InstantCommand(
        //                         () -> {
        //                             var questVisionUpdatesOn =
        // RobotStateManager.getInstance()._questVisionUpdatesOn;
        //                             System.out.println(
        //                                     "quest vision updates are now " + (questVisionUpdatesOn ?
        // "off" : "on"));
        //                             RobotStateManager.getInstance()._questVisionUpdatesOn =
        // !questVisionUpdatesOn;
        //                         }));

        _driverController
                .leftMiddleButton()
                .onTrue(
                        new InstantCommand(
                                () ->
                                        RobotStateManager.getInstance()
                                                .resetEstimatedPose(new Pose2d(3.169, 4.021, new Rotation2d()))));

        _driverController.a().onTrue(new InstantCommand(container.getClimber()::decreaseServoSetpoint));
        _driverController.y().onTrue(new InstantCommand(container.getClimber()::increaseServoSetpoint));
        _driverController
                .povDown()
                .whileTrue(
                        new DynamicDriveToLane(
                                container.getDrive(),
                                () ->
                                        -modifyAxis(_driverController.getLeftY())
                                                * Constants.SwerveModule.MAX_LINEAR_SPEED));
    }

    /** OPERATOR CONTROLS: Coral Mode Algae Mode L1 L2 L3 L4 Place Reef Place Processor */
    private void configureOperatorControls(RobotContainer container, SuperstructureManager manager) {
        _operatorController
                .back()
                .onTrue(new InstantCommand(manager::forceQueueExecution))
                .onFalse(new InstantCommand(manager::releaseQueueExecution));

        /*
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
                                                        Constants.SuperstructureGoals.PLACE_CORAL_L1, RequestType.IMMEDIATE),
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
                                container.getAlgae()::isAlgaeDetected));

        _operatorController
                .y()
                .onTrue(
                        manager.createRequest(
                                Constants.SuperstructureGoals.PLACE_CORAL_L2, RequestType.QUEUED));
        _operatorController
                .b()
                .onTrue(
                        manager.createRequest(
                                Constants.SuperstructureGoals.PLACE_CORAL_L3, RequestType.QUEUED));
        /*
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
                                container.getAlgae()::isAlgaeDetected));

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
