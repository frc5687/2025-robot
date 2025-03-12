package org.frc5687.robot.subsystems.superstructure;

import java.util.Optional;
import org.frc5687.robot.subsystems.algaearm.AlgaeState;
import org.frc5687.robot.subsystems.coralarm.CoralState;
import org.frc5687.robot.subsystems.elevator.ElevatorState;
import org.frc5687.robot.subsystems.intake.IntakeState;

public class SuperstructureState {
    private Optional<ElevatorState> _elevator;
    private Optional<CoralState> _coral;
    private Optional<AlgaeState> _algae;
    private Optional<IntakeState> _intake;

    public SuperstructureState(
            Optional<ElevatorState> elevator,
            Optional<CoralState> coral,
            Optional<AlgaeState> algae,
            Optional<IntakeState> intake) {
        _elevator = elevator;
        _coral = coral;
        _algae = algae;
        _intake = intake;
    }

    public Optional<ElevatorState> getElevator() {
        return _elevator;
    }

    public Optional<CoralState> getCoral() {
        return _coral;
    }

    public Optional<AlgaeState> getAlgae() {
        return _algae;
    }

    public Optional<IntakeState> getIntake() {
        return _intake;
    }

    @Override
    public boolean equals(Object obj) {
        if (this == obj) return true;
        if (!(obj instanceof SuperstructureState)) return false;
        SuperstructureState other = (SuperstructureState) obj;
        return _elevator.equals(other._elevator)
                && _coral.equals(other._coral)
                && _algae.equals(other._algae)
                && _intake.equals(other._intake);
    }

    @Override
    public String toString() {
        return String.format(
                "Elevator: %s\nCoral: %s\nAlgae: %s\nIntake: %s\n", _elevator, _coral, _algae, _intake);
    }
}
