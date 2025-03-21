package org.frc5687.robot;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Radians;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import org.frc5687.robot.commands.algae.EjectAlgae;
import org.frc5687.robot.commands.algae.EmergencyEjectAlgae;
import org.frc5687.robot.commands.coral.EjectCoral;
import org.frc5687.robot.commands.drive.DriveToHP;
import org.frc5687.robot.commands.drive.DynamicDriveToLane;
import org.frc5687.robot.commands.drive.DynamicDriveToReefBranch;
import org.frc5687.robot.commands.drive.DynamicDriveToReefBranchAlgae;
import org.frc5687.robot.commands.drive.DynamicDriveToReefBranchNoNormalVector;
import org.frc5687.robot.commands.drive.TeleopDriveWithSnapTo;
import org.frc5687.robot.subsystems.superstructure.RequestType;
import org.frc5687.robot.subsystems.superstructure.SuperstructureManager;
import org.frc5687.robot.util.Helpers;
import org.frc5687.robot.util.OutliersController;
import org.frc5687.robot.util.ReefAlignmentHelpers.ReefSide;

public class OperatorInterface {
    private final OutliersController _driverController;
    private final OutliersController _operatorController;

    private Trigger _intakeCoralDetectedTrigger;

    public OperatorInterface() {
        _driverController = new OutliersController(new CommandPS5Controller(0));
        _operatorController = new OutliersController(new CommandPS5Controller(1));
    }

    public void configureCommandMapping(RobotContainer container) {
        SuperstructureManager manager = container.getSuperstructureManager();
        _intakeCoralDetectedTrigger = new Trigger(container.getIntake()::isIntakeCoralDetected);
        configureDriverControls(container, manager);
        configureOperatorControls(container, manager);
    }

    private void configureDriverControls(RobotContainer container, SuperstructureManager manager) {
        // Face angles with funnel receive
        _driverController
                .x()
                .whileTrue(
                        new DynamicDriveToReefBranchNoNormalVector(container.getDrive(), ReefSide.LEFT_L1));

        _driverController
                .b()
                .whileTrue(
                        new DynamicDriveToReefBranchNoNormalVector(container.getDrive(), ReefSide.RIGHT_L1));

        _driverController
                .y()
                .onTrue(
                        new TeleopDriveWithSnapTo(
                                Degrees.of(180).in(Radians),
                                container.getDrive(),
                                () ->
                                        -RobotContainer.modifyAxis(getDriverController().getLeftY())
                                                * Constants.DriveTrain.MAX_MPS,
                                () ->
                                        -RobotContainer.modifyAxis(getDriverController().getLeftX())
                                                * Constants.DriveTrain.MAX_MPS,
                                () ->
                                        -RobotContainer.modifyAxis(getDriverController().getRightX())
                                                * Constants.DriveTrain.MAX_MPS,
                                () -> true)); // Always field relative

        _driverController
                .leftBumper()
                .whileTrue(
                        new ConditionalCommand(
                                new DynamicDriveToReefBranchAlgae(container.getDrive(), manager, ReefSide.ALGAE),
                                new DynamicDriveToReefBranch(container.getDrive(), manager, ReefSide.LEFT),
                                manager::isAlgaeMode));

        _driverController
                .rightBumper()
                .whileTrue(
                        new ConditionalCommand(
                                new DynamicDriveToReefBranch(container.getDrive(), manager, ReefSide.ALGAE),
                                new DynamicDriveToReefBranch(container.getDrive(), manager, ReefSide.RIGHT),
                                manager::isAlgaeMode));

        // _driverController
        //         .leftJoystickButton()
        //         .whileTrue(new DynamicDriveToReefBranch(container.getDrive(), manager,
        // ReefSide.ALGAE));
        _driverController.rightJoystickButton().whileTrue(new DriveToHP(container.getDrive()));

        // _driverController.leftJoystickButton().whileTrue(new
        // EmergencyEjectAlgae(container.getAlgae()));
        _driverController.povUp().whileTrue(new EmergencyEjectAlgae(container.getAlgae()));
        _driverController.povUpLeft().whileTrue(new EmergencyEjectAlgae(container.getAlgae()));
        _driverController.povUpRight().whileTrue(new EmergencyEjectAlgae(container.getAlgae()));

        _driverController
                .leftTrigger()
                .whileTrue(
                        Commands.sequence(
                                        manager.createRequest(
                                                Constants.SuperstructureGoals.GROUND_INTAKE, RequestType.IMMEDIATE),
                                        Commands.run(() -> container.getIntake().setVoltages(-12, 4)))
                                .finallyDo(
                                        (interrupted) -> {
                                            container.getIntake().setVoltages(0, 0);

                                            if (!container.getIntake().isIntakeCoralDetected()) {
                                                manager
                                                        .createRequest(
                                                                Constants.SuperstructureGoals.STOW_INTAKE, RequestType.IMMEDIATE)
                                                        .schedule();
                                            }
                                        })
                                .handleInterrupt(
                                        () -> {
                                            container.getIntake().setVoltages(0, 0);
                                        }));

        // This was the only way I got this to work the way I would like it too.
        _intakeCoralDetectedTrigger.onTrue(
                Commands.sequence(
                        Commands.runOnce(() -> container.getIntake().setVoltages(0, 0)),
                        manager.createRequest(
                                Constants.SuperstructureGoals.RECEIVE_FROM_GROUND_INTAKE, RequestType.IMMEDIATE),
                        Commands.waitSeconds(0.3),
                        Commands.runOnce(() -> container.getIntake().setVoltages(0, -3.0)),
                        manager.indexCoral(),
                        Commands.runOnce(() -> container.getIntake().setVoltages(0, 0)),
                        manager.createRequest(
                                Constants.SuperstructureGoals.STOW_INTAKE, RequestType.IMMEDIATE)));

        // _driverController
        //         .leftTrigger()
        //         .whileTrue(
        //                 new ConditionalCommand(
        //                         new SequentialCommandGroup(
        //                                         manager.grabAlgae(
        //                                                 Constants.SuperstructureGoals.GROUND_PICKUP,
        // RequestType.IMMEDIATE),
        //                                         new IntakeAlgae(container.getAlgae()),
        //                                         manager.createRequest(
        //                                                 new SuperstructureState(
        //                                                         Optional.empty(),
        //                                                         Optional.empty(),
        //                                                         Optional.of(AlgaeState.IDLE),
        //                                                         Optional.empty()),
        //                                                 RequestType.IMMEDIATE),
        //                                         new InstantCommand(() ->
        // container.getAlgae().setWheelMotorVoltage(0)))
        //                                 .alongWith(new DriveToGroundAlgae(container.getDrive(),
        // container.getVision())),
        //                         new InstantCommand(),
        //                         manager::isAlgaeMode));
        // .onTrue(
        //         new SequentialCommandGroup(
        //                 new InstantCommand(() -> container.getIntake().setVoltages(-12, 12)),
        //                 manager.createRequest(
        //                         Constants.SuperstructureGoals.GROUND_INTAKE, RequestType.IMMEDIATE),
        //                 new WaitUntilCommand(
        //                         () ->
        //                                 container.getIntake().isIntakeCoralDetected()
        //                                         || !_driverController.leftTrigger().getAsBoolean()),
        //                 new InstantCommand(() -> container.getIntake().setVoltages(0, 3)),
        //                 new ConditionalCommand(
        //                         new SequentialCommandGroup(
        //                                 manager.createRequest(
        //
        // Constants.SuperstructureGoals.RECEIVE_FROM_GROUND_INTAKE,
        //                                         RequestType.IMMEDIATE),
        //                                 new InstantCommand(() -> container.getIntake().setVoltages(0,
        // -12)),
        //                                 manager.indexCoral(),
        //                                 new InstantCommand(() -> container.getIntake().setVoltages(0,
        // 0))),
        //                         manager.createRequest(
        //                                 Constants.SuperstructureGoals.STOW_INTAKE,
        // RequestType.IMMEDIATE),
        //                         container.getIntake()::isIntakeCoralDetected)));

        _driverController
                .rightTrigger()
                .whileTrue(
                        new ConditionalCommand(
                                new EjectAlgae(container.getAlgae(), container.getElevator()),
                                new EjectCoral(container.getCoral()),
                                manager::isAlgaeMode));

        _driverController.rightMiddleButton().onTrue(new InstantCommand(container.getDrive()::zeroIMU));

        _driverController
                .leftMiddleButton()
                .onTrue(new InstantCommand(container.getClimber()::toggleClimberSetpoint));

        // _driverController
        //         .a()
        //         .whileTrue(new DynamicDriveToNet(container.getDrive(), _driverController::getLeftX));

        // _driverController
        //         .y()
        //         .whileTrue(new DriveToGroundAlgae(container.getDrive(), container.getVision()));

        _driverController
                .a()
                .whileTrue(
                        new DynamicDriveToLane(
                                container.getDrive(),
                                () -> -modifyAxis(_driverController.getLeftY()) * Constants.DriveTrain.MAX_MPS));
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
                                                manager.createRequest(
                                                        Constants.SuperstructureGoals.PLACE_CORAL_L1, RequestType.IMMEDIATE),
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
                .whileTrue(manager.algaeIntake(Constants.SuperstructureGoals.LOW_ALGAE_GRAB));

        // High grab (right bumper) - always performs algae operations
        _operatorController
                .rightBumper()
                .whileTrue(manager.algaeIntake(Constants.SuperstructureGoals.HIGH_ALGAE_GRAB));
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
