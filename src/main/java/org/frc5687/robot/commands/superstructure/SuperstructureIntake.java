package org.frc5687.robot.commands.superstructure;

import org.frc5687.robot.commands.OutliersCommand;
import org.frc5687.robot.subsystems.algaearm.AlgaeArmSubsystem;
import org.frc5687.robot.subsystems.algaearm.AlgaeState;
import org.frc5687.robot.subsystems.coralarm.CoralArmSubsystem;
import org.frc5687.robot.subsystems.coralarm.CoralState;
import org.frc5687.robot.subsystems.elevator.ElevatorState;
import org.frc5687.robot.subsystems.elevator.ElevatorSubsystem;
import org.frc5687.robot.subsystems.intake.IntakeState;
import org.frc5687.robot.subsystems.intake.IntakeSubsystem;

public class SuperstructureIntake extends OutliersCommand {

    private final ElevatorSubsystem _elevator;
    private final CoralArmSubsystem _coral;
    private final AlgaeArmSubsystem _algae;
    private final IntakeSubsystem _intake;
    private SuperstructureIntakeState _state;
    private double _timer;

    public SuperstructureIntake(
            ElevatorSubsystem elevator,
            CoralArmSubsystem coral,
            AlgaeArmSubsystem algae,
            IntakeSubsystem intake) {
        _elevator = elevator;
        _coral = coral;
        _algae = algae;
        _intake = intake;

        addRequirements(_elevator, _coral, _algae, _intake);
    }

    @Override
    public void initialize() {
        _elevator.setDesiredState(ElevatorState.STOWED);
        _coral.setDesiredState(CoralState.RECEIVE_FROM_INTAKE);
        _algae.setDesiredState(AlgaeState.IDLE);
        _intake.setDesiredState(IntakeState.DEPLOYED);
        _state = SuperstructureIntakeState.INTAKING;
    }

    @Override
    protected void execute(double timestamp) {
        if (_state == SuperstructureIntakeState.INTAKING) {
            _intake.setDesiredState(IntakeState.DEPLOYED);
            _intake.setRollerVoltage(12);
            _intake.setIntakeVoltage(-12);
            if (_intake.isIntakeCoralDetected()) {
                _intake.setRollerVoltage(0);
                _state = SuperstructureIntakeState.HANDOFF_ANGLE;
            }
        }

        if (_state == SuperstructureIntakeState.HANDOFF_ANGLE) {
            _intake.setDesiredState(IntakeState.PASSOFF_TO_CORAL);
            _coral.setDesiredState(CoralState.RECEIVE_FROM_INTAKE);
            System.out.println(_coral.isAtDesiredAngle() + "coral desired angle");
            System.out.println(_intake.isAtDesiredAngle() + "intake desired angle");

            if (_coral.isAtDesiredAngle() && _intake.isAtDesiredAngle()) {
                _intake.setIntakeVoltage(0);
                _timer = timestamp;
                _state = SuperstructureIntakeState.HANDOFF;
            }
        }

        if (_state == SuperstructureIntakeState.HANDOFF) {
            if (Math.abs(_timer - timestamp) >= .5) {
                if (!_coral.isCoralDetected()) {
                    _intake.setRollerVoltage(-1);
                    _intake.setIntakeVoltage(3);
                    _coral.setWheelMotorDutyCycle(-1);
                    _timer = timestamp;
                }
                if (_coral.isCoralDetected()) {
                    _intake.setDesiredState(IntakeState.IDLE);
                    _intake.setRollerVoltage(0);
                    _intake.setIntakeVoltage(0);
                    _coral.setWheelMotorPosition(_coral.getWheelMotorPosition());
                }
            }
        }
    }

    public enum SuperstructureIntakeState {
        INTAKING(0),
        HANDOFF_ANGLE(1),
        HANDOFF(2);

        private final double _value;

        SuperstructureIntakeState(double value) {
            _value = value;
        }

        public double getValue() {
            return _value;
        }
    }
}
