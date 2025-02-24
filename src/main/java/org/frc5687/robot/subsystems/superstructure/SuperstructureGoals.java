package org.frc5687.robot.subsystems.superstructure;

import java.util.Optional;
import org.frc5687.robot.subsystems.algaearm.AlgaeState;
import org.frc5687.robot.subsystems.coralarm.CoralState;
import org.frc5687.robot.subsystems.elevator.ElevatorState;

public class SuperstructureGoals {
    public static final SuperstructureState RECEIVE_FROM_FUNNEL =
            new SuperstructureState(
                    Optional.of(ElevatorState.FUNNEL_RECEIVE),
                    Optional.of(CoralState.RECEIVE_FROM_FUNNEL),
                    Optional.empty(),
                    Optional.empty());

    public static final SuperstructureState PLACE_CORAL_L4 =
            new SuperstructureState(
                    Optional.of(ElevatorState.L4_CORAL_PLACING), Optional.of(CoralState.PLACING_L4),
                    Optional.of(AlgaeState.IDLE), Optional.empty());

    public static final SuperstructureState PLACE_CORAL_L4_ALGAE_GRAB =
            new SuperstructureState(
                    Optional.of(ElevatorState.L4_CORAL_PLACING),
                    Optional.of(CoralState.PLACING_L4),
                    Optional.of(AlgaeState.REEF_PICKUP),
                    Optional.empty());

    public static final SuperstructureState PLACE_CORAL_L3 =
            new SuperstructureState(
                    Optional.of(ElevatorState.L3_CORAL_PLACING), Optional.of(CoralState.PLACING),
                    Optional.of(AlgaeState.IDLE), Optional.empty());
    public static final SuperstructureState PLACE_CORAL_L3_ALGAE_GRAB =
            new SuperstructureState(
                    Optional.of(ElevatorState.L3_CORAL_PLACING),
                    Optional.of(CoralState.PLACING),
                    Optional.of(AlgaeState.REEF_PICKUP),
                    Optional.empty());

    public static final SuperstructureState PLACE_CORAL_L2 =
            new SuperstructureState(
                    Optional.of(ElevatorState.L2_CORAL_PLACING), Optional.of(CoralState.PLACING),
                    Optional.of(AlgaeState.IDLE), Optional.empty());

    public static final SuperstructureState PLACE_CORAL_L1 =
            new SuperstructureState(
                    Optional.of(ElevatorState.L1_CORAL_PLACING), Optional.of(CoralState.PLACING),
                    Optional.of(AlgaeState.IDLE), Optional.empty());

    public static final SuperstructureState PROCESSOR_DROPOFF =
            new SuperstructureState(
                    Optional.of(ElevatorState.L1_CORAL_PLACING),
                    Optional.of(CoralState.IDLE),
                    Optional.of(AlgaeState.PROCESSOR_DROPOFF),
                    Optional.empty());
}
