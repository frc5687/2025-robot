package org.frc5687.robot.subsystems.superstructure;

import org.frc5687.robot.subsystems.algaearm.AlgaeState;
import org.frc5687.robot.subsystems.coralarm.CoralState;
import org.frc5687.robot.subsystems.elevator.ElevatorState;
import org.frc5687.robot.subsystems.intake.IntakeState;

public class SuperstructureGoals {
    // This is the sate we need to be in before we transition to and from recieiving the coral from
    // either the funnel or intake
    public static final SuperstructureState SAFE_CORAL_TRANSITION =
            new SuperstructureState(
                    ElevatorState.STOWED, CoralState.STOWED, AlgaeState.IDLE, IntakeState.IDLE);

    public static final SuperstructureState RECEIVE_FROM_FUNNEL =
            new SuperstructureState(
                    ElevatorState.FUNNEL_RECEIVE,
                    CoralState.RECEIVE_FROM_FUNNEL,
                    AlgaeState.IDLE,
                    IntakeState.IDLE);

    public static final SuperstructureState RECEIVE_FROM_INTAKE =
            new SuperstructureState(
                    ElevatorState.STOWED,
                    CoralState.RECEIVE_FROM_INTAKE,
                    AlgaeState.IDLE,
                    IntakeState.PASSOFF_TO_CORAL);

    public static final SuperstructureState CLEAR_INTAKE =
            new SuperstructureState(
                    ElevatorState.STOWED, CoralState.STOWED, AlgaeState.IDLE, IntakeState.IDLE);

    public static final SuperstructureState PLACE_CORAL_L4 =
            new SuperstructureState(
                    ElevatorState.L4_CORAL_PLACING, CoralState.PLACING, AlgaeState.IDLE, IntakeState.IDLE);
    public static final SuperstructureState PLACE_CORAL_L4_ALGAE_GRAB =
            new SuperstructureState(
                    ElevatorState.L4_CORAL_PLACING,
                    CoralState.PLACING,
                    AlgaeState.REEF_PICKUP,
                    IntakeState.IDLE);

    public static final SuperstructureState PLACE_CORAL_L3 =
            new SuperstructureState(
                    ElevatorState.L3_CORAL_PLACING, CoralState.PLACING, AlgaeState.IDLE, IntakeState.IDLE);
    public static final SuperstructureState PLACE_CORAL_L3_ALGAE_GRAB =
            new SuperstructureState(
                    ElevatorState.L3_CORAL_PLACING,
                    CoralState.PLACING,
                    AlgaeState.REEF_PICKUP,
                    IntakeState.IDLE);

    public static final SuperstructureState PLACE_CORAL_L2 =
            new SuperstructureState(
                    ElevatorState.L2_CORAL_PLACING, CoralState.PLACING, AlgaeState.IDLE, IntakeState.IDLE);

    public static final SuperstructureState L1_ALGAE_GRAB =
            new SuperstructureState(
                    ElevatorState.L1_ALGAE_GRAB,
                    CoralState.IDLE_NO_CORAL,
                    AlgaeState.REEF_PICKUP,
                    IntakeState.IDLE);
    public static final SuperstructureState L2_ALGAE_GRAB =
            new SuperstructureState(
                    ElevatorState.L2_ALGAE_GRAB,
                    CoralState.IDLE_NO_CORAL,
                    AlgaeState.REEF_PICKUP,
                    IntakeState.IDLE);
}
