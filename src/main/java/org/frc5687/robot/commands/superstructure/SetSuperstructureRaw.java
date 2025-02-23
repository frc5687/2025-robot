package org.frc5687.robot.commands.superstructure;

import java.util.Optional;
import org.frc5687.robot.RobotContainer;
import org.frc5687.robot.commands.OutliersCommand;
import org.frc5687.robot.subsystems.algaearm.AlgaeArmSubsystem;
import org.frc5687.robot.subsystems.algaearm.AlgaeState;
import org.frc5687.robot.subsystems.coralarm.CoralArmSubsystem;
import org.frc5687.robot.subsystems.coralarm.CoralState;
import org.frc5687.robot.subsystems.elevator.ElevatorState;
import org.frc5687.robot.subsystems.elevator.ElevatorSubsystem;
import org.frc5687.robot.subsystems.intake.IntakeState;
import org.frc5687.robot.subsystems.intake.IntakeSubsystem;
import org.frc5687.robot.subsystems.superstructure.SuperstructureState;

public class SetSuperstructureRaw extends OutliersCommand {
    private final ElevatorSubsystem _elevator;
    private final CoralArmSubsystem _coral;
    private final AlgaeArmSubsystem _algae;
    private final IntakeSubsystem _intake;
    private Optional<ElevatorState> _elevatorGoal;
    private Optional<AlgaeState> _algaeGoal;
    private Optional<CoralState> _coralGoal;
    private Optional<IntakeState> _intakeGoal;

    public SetSuperstructureRaw(RobotContainer container, SuperstructureState goalState) {
        this(
                container,
                Optional.of(goalState.getAlgae()),
                Optional.of(goalState.getElevator()),
                Optional.of(goalState.getIntake()),
                Optional.of(goalState.getCoral()));
    }

    public SetSuperstructureRaw(
            RobotContainer container,
            Optional<AlgaeState> algaeGoal,
            Optional<ElevatorState> elevatorGoal,
            Optional<IntakeState> intakeGoal,
            Optional<CoralState> coralGoal) {
        _elevator = container.getElevator();
        _coral = container.getCoral();
        _algae = container.getAlgae();
        _intake = container.getIntake();
        _elevatorGoal = elevatorGoal;
        _algaeGoal = algaeGoal;
        _coralGoal = coralGoal;
        _intakeGoal = intakeGoal;
        if (_elevatorGoal.isPresent()) {
            addRequirements(_elevator);
        }
        if (_coralGoal.isPresent()) {
            addRequirements(_coral);
        }
        if (_algaeGoal.isPresent()) {
            addRequirements(_algae);
        }
        if (_intakeGoal.isPresent()) {
            addRequirements(_intake);
        }
    }

    @Override
    public void initialize() {
        System.out.println(
                "Set Superstructure Raw with goals "
                        + _elevatorGoal
                        + ", "
                        + _algaeGoal
                        + ", "
                        + _coralGoal
                        + " and "
                        + _intakeGoal);
        if (_elevatorGoal.isPresent()) {
            _elevator.setDesiredState(_elevatorGoal.get());
        }
        if (_coralGoal.isPresent()) {
            _coral.setDesiredState(_coralGoal.get());
        }
        if (_algaeGoal.isPresent()) {
            _algae.setDesiredState(_algaeGoal.get());
        }
        if (_intakeGoal.isPresent()) {
            _intake.setDesiredState(_intakeGoal.get());
        }
    }

    @Override
    protected void execute(double timestamp) {}

    @Override
    public boolean isFinished() {
        return _elevator.isAtDesiredPosition()
                && _coral.isAtDesiredAngle()
                && _algae.isAtDesiredAngle();
        // && _intake.isAtDesiredAngle(); TODO add intake
    }
}
