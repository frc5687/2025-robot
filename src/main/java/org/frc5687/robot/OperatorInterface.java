package org.frc5687.robot;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandPS4Controller;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import org.frc5687.robot.commands.algae.AlgaeSetState;
import org.frc5687.robot.commands.superstructure.SuperstructureFactory;
import org.frc5687.robot.subsystems.algaearm.AlgaeState;
import org.frc5687.robot.subsystems.coralarm.CoralState;
import org.frc5687.robot.util.FieldConstants.ReefHeight;
import org.frc5687.robot.util.OutliersController;
import org.frc5687.robot.util.ReefAlignmentHelpers.ReefSide;

public class OperatorInterface {
        private final OutliersController _driverController;
        private final OutliersController _operatorController;

        public OperatorInterface() {
                _driverController = new OutliersController(new CommandPS5Controller(0));
                _operatorController = new OutliersController(new CommandPS4Controller(1));
        }

        public void configureCommandMapping(RobotContainer container) {
                // _driverController.a().onTrue(new HomeElevator(container.getElevator()));
                _driverController
                                .x()
                                .onTrue(
                                                new InstantCommand(
                                                                () -> {
                                                                        container.getDrive().enableHeadingController(
                                                                                        Units.degreesToRadians(-60));
                                                                }));
                _driverController.x().onTrue(SuperstructureFactory.receiveFromFunnel(container));
                _driverController
                                .b()
                                .onTrue(
                                                new InstantCommand(
                                                                () -> {
                                                                        container.getDrive().enableHeadingController(
                                                                                        Units.degreesToRadians(60));
                                                                }));
                _driverController.b().onTrue(SuperstructureFactory.receiveFromFunnel(container));

                // _driverController
                // .leftBumper()
                // .whileTrue(
                // new DynamicDriveToReefBranch(container.getDrive(), ReefSide.LEFT,
                // ReefHeight.L4));
                // _driverController
                // .rightBumper()
                // .whileTrue(
                // new DynamicDriveToReefBranch(container.getDrive(), ReefSide.RIGHT,
                // ReefHeight.L4));
                // _driverController.leftTrigger().whileTrue(SuperstructureFactory.groundIntakeHandoff(container));
                _driverController
                                .rightTrigger()
                                .onTrue(SuperstructureFactory.placeAndStow(container)); // TODO place based on held

                // // _driverController
                // // .rightTrigger()
                // // .whileTrue(new ConditionalCommand(null, null, false)); // TODO intake
                // based on
                // mode

                // _driverController.button(8).onTrue(new
                // InstantCommand(container.getDrive()::zeroIMU));

                // _driverController
                // .button(7)
                // .onTrue(new InstantCommand(container.getClimber()::toggleClimberSetpoint));

                _driverController.rightMiddleButton().onTrue(new InstantCommand(container.getDrive()::zeroIMU));

                _driverController
                                .leftMiddleButton()
                                .onTrue(new InstantCommand(container.getClimber()::toggleClimberSetpoint));

                /**
                 * OPERATOR CONTROLS: Coral Mode Algae Mode L1 L2 L3 L4 Place Reef Place
                 * Processor
                 */
                _operatorController.a().onTrue(new CoralSetState(container.getCoral(), CoralState.STOWED));

                _operatorController.a().onTrue(SuperstructureFactory.placeCoralL1(container));
                _operatorController.b().onTrue(SuperstructureFactory.placeCoralL2(container));
                _operatorController.x().onTrue(SuperstructureFactory.placeCoralL3(container, false));
                _operatorController.y().onTrue(SuperstructureFactory.placeCoralL4(container, false));
                // _operatorController.leftBumper().onTrue(SuperstructureFactory.grabAlgaeL2(container));
                // _operatorController.rightBumper().onTrue(SuperstructureFactory.grabAlgaeL1(container));
                _operatorController.rightTrigger().onTrue(SuperstructureFactory.processorDropoff(container));
                _operatorController
                                .leftTrigger()
                                .onTrue(new AlgaeSetState(container.getAlgae(), AlgaeState.PROCESSOR_DROPOFF_WHEEL));
                _operatorController.povDown().whileTrue(SuperstructureFactory.intakeEject(container));
                _operatorController
                                .povUp()
                                .onTrue(new AlgaeSetState(container.getAlgae(), AlgaeState.GROUND_PICKUP));
        }

        public OutliersController getDriverController() {
                return _driverController;
        }

        public OutliersController getOperatorController() {
                return _operatorController;
        }
}
