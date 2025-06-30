package org.frc5687.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import org.frc5687.robot.Constants;
import org.frc5687.robot.RobotContainer;
import org.frc5687.robot.commands.algae.EjectAlgae;
import org.frc5687.robot.commands.drive.DriveToProcessorLineup;
import org.frc5687.robot.commands.drive.DynamicDriveToReefBranch;
import org.frc5687.robot.commands.elevator.GoToAlgaeHeight;
import org.frc5687.robot.util.ReefAlignmentHelpers.ReefSide;

public class AutoActions {

    public static Command autoPlace(RobotContainer container) {
        return new SequentialCommandGroup(
                new InstantCommand(
                        () -> {
                            container
                                    .getCoral()
                                    .setWheelMotorDutyCycle(Constants.CoralArm.WHEEL_EJECT_CORAL_DUTY_CYCLE);
                        }),
                // new ParallelRaceGroup(new WaitCommand(.5), new WaitForPlaceSensor(container)),
                new WaitForPlaceSensor(container),
                new InstantCommand(
                        () -> {
                            container.getCoral().setWheelMotorDutyCycle(0);
                        }));
    }

    public static Command autoProcessorDropOff(RobotContainer container) {
        return new DriveToProcessorLineup(container.getDrive(), container.getSuperstructureManager())
                .andThen(new EjectAlgae(container.getAlgae(), container.getElevator()));
    }

    public static Command autoHybridCycle(RobotContainer container) {
        return autoPlace(container)
                .andThen(() -> container.getSuperstructureManager().toggleMode())
                .andThen(
                        new ParallelDeadlineGroup(
                                new WaitUntilCommand(() -> container.getAlgae().isAlgaeDetected()),
                                container.getSuperstructureManager().algaeIntakeAuto(),
                                new GoToAlgaeHeight(container.getElevator(), container.getDrive()),
                                new DynamicDriveToReefBranch(
                                        container.getDrive(),
                                        container.getSuperstructureManager(),
                                        ReefSide.ALGAE,
                                        true)));
    }

    public static Command autoPickupAlgaeOfReef(RobotContainer container) {
        return container.getSuperstructureManager().hybridAlgaeIntake();
    }

    public static Command autoPlaceAlgaeNet(RobotContainer container) {
        return new EjectAlgae(container.getAlgae(), container.getElevator());
    }

    public static Command autoGroundInakeCoral(RobotContainer container) {
        return new WaitCommand(0);
    }
}
