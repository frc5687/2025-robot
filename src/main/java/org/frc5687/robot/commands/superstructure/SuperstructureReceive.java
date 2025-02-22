package org.frc5687.robot.commands.superstructure;

import org.frc5687.robot.Constants;
import org.frc5687.robot.RobotContainer;
import org.frc5687.robot.commands.OutliersCommand;
import org.frc5687.robot.subsystems.algaearm.AlgaeArmSubsystem;
import org.frc5687.robot.subsystems.coralarm.CoralArmSubsystem;
import org.frc5687.robot.subsystems.elevator.ElevatorSubsystem;
import org.frc5687.robot.subsystems.intake.IntakeSubsystem;
import org.frc5687.robot.subsystems.superstructure.SuperstructureState;
import org.frc5687.robot.util.TunableDouble;

public class SuperstructureReceive extends OutliersCommand {
    private final ElevatorSubsystem _elevator;
    private final CoralArmSubsystem _coral;
    private final AlgaeArmSubsystem _algae;
    private final IntakeSubsystem _intake;
    private final SuperstructureState _goal;
    private SuperstructureReceiveState _state;

    private static final TunableDouble _goMore = new TunableDouble("Receive", "goMore", 7.5);

    public SuperstructureReceive(RobotContainer container, SuperstructureState goal) {
        _elevator = container.getElevator();
        _coral = container.getCoral();
        _algae = container.getAlgae();
        _intake = container.getIntake();
        _goal = goal;

        addRequirements(_elevator, _coral, _algae, _intake);
    }

    @Override
    public void initialize() {
        _elevator.setDesiredState(_goal.getElevator());
        _coral.setDesiredState(_goal.getCoral());
        _algae.setDesiredState(_goal.getAlgae());
        _intake.setDesiredState(_goal.getIntake());
        _state = SuperstructureReceiveState.INTAKING;
    }

    @Override
    public void execute(double timestamp) {
        if (_state == SuperstructureReceiveState.INTAKING) {
            _coral.setWheelMotorDutyCycle(Constants.CoralArm.WHEEL_RECEIVE_CORAL_DUTY_CYCLE);
            if (_coral.isCoralDetected()) {
                _state = SuperstructureReceiveState.SAW_IT;
                _coral.setWheelMotorPosition(_coral.getWheelMotorPosition() + _goMore.get());
            }
        } else if (_state == SuperstructureReceiveState.SAW_IT) {
            if (_coral.isAtDesiredAngle()) {
                _state = SuperstructureReceiveState.DONE;
            }
        }
    }

    @Override
    public boolean isFinished() {
        return _state == SuperstructureReceiveState.DONE;
    }

    @Override
    public void end(boolean interrupted) {
        _coral.setWheelMotorPosition(_coral.getWheelMotorPosition());
    }

    public enum SuperstructureReceiveState {
        INTAKING(0),
        SAW_IT(1),
        OUTTAKING(2),
        DONE(3);

        private final double _value;

        SuperstructureReceiveState(double value) {
            _value = value;
        }

        public double getValue() {
            return _value;
        }
    }
}
