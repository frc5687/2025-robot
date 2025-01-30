package org.frc5687.robot.subsystems.superstructure;

import org.frc5687.robot.subsystems.algaearm.AlgaeState;
import org.frc5687.robot.subsystems.coralarm.CoralState;
import org.frc5687.robot.subsystems.elevator.ElevatorState;

public class SuperstructureState {
    private ElevatorState _elevator;
    private CoralState _coral;
    private AlgaeState _algae;

    public SuperstructureState(ElevatorState elevator, CoralState coreal, AlgaeState algae) {
        _elevator = elevator;
        _coral = coreal;
        _algae = algae;
    }

    public ElevatorState getElevator() {
        return _elevator;
    }

    public CoralState getCoral() {
        return _coral;
    }

    public AlgaeState getAlgae() {
        return _algae;
    }

    @Override
    public boolean equals(Object obj) {
        if (this == obj) return true;
        if (!(obj instanceof SuperstructureState)) return false;
        SuperstructureState other = (SuperstructureState) obj;
        return _elevator == other._elevator && _coral == other._coral && _algae == other._algae;
    }

    @Override
    public String toString() {
        return String.format("Elevator: %s\nCoral: %s\nAlgae: %s\n", _elevator, _coral, _algae);
    }
}
