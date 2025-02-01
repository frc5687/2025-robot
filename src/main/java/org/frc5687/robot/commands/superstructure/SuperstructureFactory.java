package org.frc5687.robot.commands.superstructure;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import org.frc5687.robot.RobotContainer;
import org.frc5687.robot.commands.algae.AlgaeSetState;
import org.frc5687.robot.subsystems.superstructure.SuperstructureGoals;
import org.frc5687.robot.subsystems.superstructure.SuperstructureState;

public class SuperstructureFactory {

    private static Command withStateTracking(
            RobotContainer container, SuperstructureState state, Command command) {
        return new SequentialCommandGroup(
                new InstantCommand(() -> container.getSuperstructureTracker().setDesiredState(state)),
                command);
    }

    // I can just make a set state command to remove repetative code :)
    public static Command transitionToSafeCoralState(RobotContainer container) {
        return withStateTracking(
                        container,
                        SuperstructureGoals.SAFE_CORAL_TRANSITION,
                        new ParallelCommandGroup(
                                // new ElevatorSetState(
                                //         container.getElevator(),
                                //         SuperstructureGoals.SAFE_CORAL_TRANSITION.getElevator()),
                                // new CoralSetState(
                                //         container.getCoral(),
                                // SuperstructureGoals.SAFE_CORAL_TRANSITION.getCoral()),
                                new AlgaeSetState(
                                        container.getAlgae(), SuperstructureGoals.SAFE_CORAL_TRANSITION.getAlgae())))
                .unless(() -> !container.getSuperstructureTracker().needsSafeCoralTransition());
    }

    public static Command receiveFromFunnel(RobotContainer container) {
        return withStateTracking(
                container,
                SuperstructureGoals.RECEIVE_FROM_FUNNEL,
                new SequentialCommandGroup(
                        transitionToSafeCoralState(container),
                        new ParallelCommandGroup(
                                // new ElevatorSetState(
                                //         container.getElevator(),
                                // SuperstructureGoals.RECEIVE_FROM_FUNNEL.getElevator()),
                                // new CoralSetState(
                                //         container.getCoral(),
                                // SuperstructureGoals.RECEIVE_FROM_FUNNEL.getCoral()),
                                new AlgaeSetState(
                                        container.getAlgae(), SuperstructureGoals.RECEIVE_FROM_FUNNEL.getAlgae()))));
    }

    public static Command receiveFromIntake(RobotContainer container) {
        return withStateTracking(
                container,
                SuperstructureGoals.RECEIVE_FROM_INTAKE,
                new SequentialCommandGroup(
                        transitionToSafeCoralState(container),
                        new ParallelCommandGroup(
                                // new ElevatorSetState(
                                //         container.getElevator(),
                                // SuperstructureGoals.RECEIVE_FROM_INTAKE.getElevator()),
                                // new CoralSetState(
                                // container.getCoral(), SuperstructureGoals.RECEIVE_FROM_INTAKE.getCoral()),
                                new AlgaeSetState(
                                        container.getAlgae(), SuperstructureGoals.RECEIVE_FROM_INTAKE.getAlgae()))));
    }

    public static Command placeCoralL4(RobotContainer container, boolean withAlgaeGrab) {
        SuperstructureState targetState =
                withAlgaeGrab
                        ? SuperstructureGoals.PLACE_CORAL_L4_ALGAE_GRAB
                        : SuperstructureGoals.PLACE_CORAL_L4;

        return withStateTracking(
                container,
                targetState,
                new SequentialCommandGroup(
                        transitionToSafeCoralState(container),
                        new ParallelCommandGroup(
                                // new ElevatorSetState(container.getElevator(), targetState.getElevator()),
                                // new CoralSetState(container.getCoral(), targetState.getCoral()),
                                new AlgaeSetState(container.getAlgae(), targetState.getAlgae()))));
    }

    public static Command placeCoralL3(RobotContainer container, boolean withAlgaeGrab) {
        SuperstructureState targetState =
                withAlgaeGrab
                        ? SuperstructureGoals.PLACE_CORAL_L3_ALGAE_GRAB
                        : SuperstructureGoals.PLACE_CORAL_L3;

        return withStateTracking(
                container,
                targetState,
                new SequentialCommandGroup(
                        transitionToSafeCoralState(container),
                        new ParallelCommandGroup(
                                // new ElevatorSetState(container.getElevator(), targetState.getElevator()),
                                // new CoralSetState(container.getCoral(), targetState.getCoral()),
                                new AlgaeSetState(container.getAlgae(), targetState.getAlgae()))));
    }

    public static Command placeCoralL2(RobotContainer container) {
        return withStateTracking(
                container,
                SuperstructureGoals.PLACE_CORAL_L2,
                new SequentialCommandGroup(
                        transitionToSafeCoralState(container),
                        new ParallelCommandGroup(
                                // new ElevatorSetState(
                                //         container.getElevator(),
                                // SuperstructureGoals.PLACE_CORAL_L2.getElevator()),
                                // new CoralSetState(
                                // container.getCoral(), SuperstructureGoals.PLACE_CORAL_L2.getCoral()),
                                new AlgaeSetState(
                                        container.getAlgae(), SuperstructureGoals.PLACE_CORAL_L2.getAlgae()))));
    }

    public static Command grabAlgaeL1(RobotContainer container) {
        return withStateTracking(
                container,
                SuperstructureGoals.L1_ALGAE_GRAB,
                new ParallelCommandGroup(
                        // new ElevatorSetState(
                        //         container.getElevator(), SuperstructureGoals.L1_ALGAE_GRAB.getElevator()),
                        // new CoralSetState(container.getCoral(),
                        // SuperstructureGoals.L1_ALGAE_GRAB.getCoral()),
                        new AlgaeSetState(container.getAlgae(), SuperstructureGoals.L1_ALGAE_GRAB.getAlgae())));
    }

    public static Command grabAlgaeL2(RobotContainer container) {
        return withStateTracking(
                container,
                SuperstructureGoals.L2_ALGAE_GRAB,
                new ParallelCommandGroup(
                        // new ElevatorSetState(
                        //         container.getElevator(), SuperstructureGoals.L2_ALGAE_GRAB.getElevator()),
                        // new CoralSetState(container.getCoral(),
                        // SuperstructureGoals.L2_ALGAE_GRAB.getCoral()),
                        new AlgaeSetState(container.getAlgae(), SuperstructureGoals.L2_ALGAE_GRAB.getAlgae())));
    }

    //     public static Command place(RobotContainer container) {
    //         return new SuperstructurePlace(
    //                 container.getElevator(), container.getCoral(), container.getAlgae());
    //     }
}
