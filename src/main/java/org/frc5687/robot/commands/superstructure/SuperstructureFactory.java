package org.frc5687.robot.commands.superstructure;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import org.frc5687.robot.RobotContainer;
import org.frc5687.robot.commands.algae.AlgaeSetState;
import org.frc5687.robot.commands.coral.CoralSetState;
import org.frc5687.robot.commands.elevator.ElevatorSetState;
import org.frc5687.robot.commands.intake.IntakeSetState;
import org.frc5687.robot.subsystems.superstructure.SuperstructureGoals;
import org.frc5687.robot.subsystems.superstructure.SuperstructureState;

public class SuperstructureFactory {

    private static Command withStateTracking(
            RobotContainer container, SuperstructureState state, Command command) {
        return new SequentialCommandGroup(
                new InstantCommand(() -> container.getSuperstructureTracker().setDesiredState(state)),
                command);
    }

    private static Command setSuperstructure(RobotContainer container, SuperstructureState state) {
        return new ParallelCommandGroup(
                new ElevatorSetState(container.getElevator(), state.getElevator()),
                new CoralSetState(container.getCoral(), state.getCoral()),
                new AlgaeSetState(container.getAlgae(), state.getAlgae()),
                new IntakeSetState(container.getIntake(), state.getIntake()));
    }

    public static Command clearIntake(RobotContainer container) {
        return withStateTracking(
                container,
                SuperstructureGoals.CLEAR_INTAKE,
                setSuperstructure(container, SuperstructureGoals.CLEAR_INTAKE)
                        .unless(() -> !container.getSuperstructureTracker().needToClearIntake()));
    }

    // I can just make a set state command to remove repetative code :)
    public static Command transitionToSafeCoralState(RobotContainer container) {
        return withStateTracking(
                container,
                SuperstructureGoals.SAFE_CORAL_TRANSITION,
                setSuperstructure(container, SuperstructureGoals.SAFE_CORAL_TRANSITION)
                        .unless(() -> !container.getSuperstructureTracker().needsSafeCoralTransition()));
    }

    public static Command ensureClearance(RobotContainer container) {
        return new SequentialCommandGroup(
                clearIntake(container)/*  , transitionToSafeCoralState(container) */);
    }

    public static Command receiveFromFunnel(RobotContainer container) {
        return withStateTracking(
                container,
                SuperstructureGoals.RECEIVE_FROM_FUNNEL,
                new SequentialCommandGroup(
                        ensureClearance(container),
                        setSuperstructure(container, SuperstructureGoals.RECEIVE_FROM_FUNNEL)));
    }

    public static Command receiveFromIntake(RobotContainer container) {
        return withStateTracking(
                container,
                SuperstructureGoals.RECEIVE_FROM_INTAKE,
                new SequentialCommandGroup(
                        ensureClearance(container),
                        setSuperstructure(container, SuperstructureGoals.RECEIVE_FROM_INTAKE)));
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
                        transitionToSafeCoralState(container), setSuperstructure(container, targetState)));
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
                        transitionToSafeCoralState(container), setSuperstructure(container, targetState)));
    }

    public static Command placeCoralL2(RobotContainer container) {
        return withStateTracking(
                container,
                SuperstructureGoals.PLACE_CORAL_L2,
                new SequentialCommandGroup(
                        transitionToSafeCoralState(container),
                        setSuperstructure(container, SuperstructureGoals.PLACE_CORAL_L2)));
    }

    public static Command grabAlgaeL1(RobotContainer container) {
        return withStateTracking(
                container,
                SuperstructureGoals.L1_ALGAE_GRAB,
                setSuperstructure(container, SuperstructureGoals.L1_ALGAE_GRAB));
    }

    public static Command grabAlgaeL2(RobotContainer container) {
        return withStateTracking(
                container,
                SuperstructureGoals.L2_ALGAE_GRAB,
                setSuperstructure(container, SuperstructureGoals.L2_ALGAE_GRAB));
    }

    public static Command place(RobotContainer container) {
        return new SuperstructurePlace(
                container.getElevator(), container.getCoral(), container.getAlgae());
    }
}
