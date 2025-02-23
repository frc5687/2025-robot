package org.frc5687.robot.subsystems.superstructure;

import java.util.Optional;
import org.frc5687.robot.subsystems.algaearm.AlgaeState;
import org.frc5687.robot.subsystems.coralarm.CoralState;
import org.frc5687.robot.subsystems.elevator.ElevatorState;
import org.frc5687.robot.subsystems.intake.IntakeState;

public class SuperstructureGoals {

    // FIXME use optionals because i can...
    // This is the sate we need to be in before we transition to and from recieiving
    // the coral from
    // either the funnel or intake
    public static final SuperstructureState SAFE_CORAL_TRANSITION =
            new SuperstructureState(
                    Optional.of(ElevatorState.FUNNEL_RECEIVE),
                    Optional.of(CoralState.IDLE_WITH_CORAL),
                    Optional.of(AlgaeState.IDLE),
                    Optional.of(IntakeState.IDLE));

    public static final SuperstructureState RECEIVE_FROM_FUNNEL_RUNNING =
            new SuperstructureState(
                    Optional.of(ElevatorState.FUNNEL_RECEIVE),
                    Optional.of(CoralState.RECEIVE_FROM_FUNNEL_RUNNING),
                    Optional.of(AlgaeState.IDLE),
                    Optional.of(IntakeState.IDLE));

    public static final SuperstructureState RECEIVE_FROM_FUNNEL =
            new SuperstructureState(
                    Optional.of(ElevatorState.FUNNEL_RECEIVE),
                    Optional.of(CoralState.RECEIVE_FROM_FUNNEL),
                    Optional.of(AlgaeState.IDLE),
                    Optional.of(IntakeState.IDLE));

    public static final SuperstructureState RECEIVE_FROM_INTAKE =
            new SuperstructureState(
                    Optional.of(ElevatorState.STOWED),
                    Optional.of(CoralState.RECEIVE_FROM_INTAKE),
                    Optional.of(AlgaeState.IDLE),
                    Optional.of(IntakeState.PASSOFF_TO_CORAL));

    public static final SuperstructureState EJECT_INTAKE =
            new SuperstructureState(
                    Optional.of(ElevatorState.STOWED), Optional.of(CoralState.IDLE_NO_CORAL),
                    Optional.of(AlgaeState.IDLE), Optional.of(IntakeState.DEPLOYED));
    public static final SuperstructureState CLEAR_INTAKE =
            new SuperstructureState(
                    Optional.of(ElevatorState.STOWED),
                    Optional.of(CoralState.STOWED),
                    Optional.of(AlgaeState.IDLE),
                    Optional.of(IntakeState.IDLE));

    public static final SuperstructureState PLACE_CORAL_L4 =
            new SuperstructureState(
                    Optional.of(ElevatorState.L4_CORAL_PLACING), Optional.of(CoralState.PLACING_L4),
                    Optional.of(AlgaeState.IDLE), Optional.of(IntakeState.IDLE));
    public static final SuperstructureState PLACE_CORAL_L4_ALGAE_GRAB =
            new SuperstructureState(
                    Optional.of(ElevatorState.L4_CORAL_PLACING),
                    Optional.of(CoralState.PLACING_L4),
                    Optional.of(AlgaeState.REEF_PICKUP_WHEEL),
                    Optional.of(IntakeState.IDLE));

    public static final SuperstructureState PLACE_CORAL_L3 =
            new SuperstructureState(
                    Optional.of(ElevatorState.L3_CORAL_PLACING), Optional.of(CoralState.PLACING),
                    Optional.of(AlgaeState.IDLE), Optional.of(IntakeState.IDLE));
    public static final SuperstructureState PLACE_CORAL_L3_ALGAE_GRAB =
            new SuperstructureState(
                    Optional.of(ElevatorState.L3_CORAL_PLACING),
                    Optional.of(CoralState.PLACING),
                    Optional.of(AlgaeState.REEF_PICKUP_WHEEL),
                    Optional.of(IntakeState.IDLE));

    public static final SuperstructureState PLACE_CORAL_L2 =
            new SuperstructureState(
                    Optional.of(ElevatorState.L2_CORAL_PLACING), Optional.of(CoralState.PLACING),
                    Optional.of(AlgaeState.IDLE), Optional.of(IntakeState.IDLE));

    public static final SuperstructureState PLACE_CORAL_L1 =
            new SuperstructureState(
                    Optional.of(ElevatorState.L1_CORAL_PLACING), Optional.of(CoralState.PLACING),
                    Optional.of(AlgaeState.IDLE), Optional.of(IntakeState.IDLE));

    public static final SuperstructureState L1_ALGAE_GRAB =
            new SuperstructureState(
                    Optional.of(ElevatorState.L1_ALGAE_GRAB),
                    Optional.of(CoralState.IDLE_NO_CORAL),
                    Optional.of(AlgaeState.REEF_PICKUP_WHEEL),
                    Optional.of(IntakeState.IDLE));
    public static final SuperstructureState L2_ALGAE_GRAB =
            new SuperstructureState(
                    Optional.of(ElevatorState.L2_ALGAE_GRAB),
                    Optional.of(CoralState.IDLE_NO_CORAL),
                    Optional.of(AlgaeState.REEF_PICKUP_WHEEL),
                    Optional.of(IntakeState.IDLE));
    public static final SuperstructureState PROCESSOR_DROPOFF =
            new SuperstructureState(
                    Optional.of(ElevatorState.L1_CORAL_PLACING),
                    Optional.of(CoralState.IDLE_NO_CORAL),
                    Optional.of(AlgaeState.PROCESSOR_DROPOFF),
                    Optional.of(IntakeState.IDLE));

    public static final SuperstructureState PROCESSOR_DROPOFF_WHEEL =
            new SuperstructureState(
                    Optional.of(ElevatorState.L1_CORAL_PLACING),
                    Optional.of(CoralState.IDLE_NO_CORAL),
                    Optional.of(AlgaeState.PROCESSOR_DROPOFF_WHEEL),
                    Optional.of(IntakeState.IDLE));

    public static final SuperstructureState GROUND_PICKUP =
            new SuperstructureState(
                    Optional.of(ElevatorState.FUNNEL_RECEIVE),
                    Optional.of(CoralState.IDLE_NO_CORAL),
                    Optional.of(AlgaeState.GROUND_PICKUP),
                    Optional.of(IntakeState.IDLE));
}
