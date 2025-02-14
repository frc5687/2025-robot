package org.frc5687.robot.commands.intake;

import org.frc5687.robot.commands.OutliersCommand;
import org.frc5687.robot.subsystems.intake.IntakeState;
import org.frc5687.robot.subsystems.intake.IntakeSubsystem;

public class IntakeSetState extends OutliersCommand {
    private final IntakeSubsystem _intake;
    private final IntakeState _desiredState;

    public IntakeSetState(IntakeSubsystem intake, IntakeState state) {
        _intake = intake;
        _desiredState = state;

        addRequirements(_intake);
    }

    @Override
    public void initialize() {
        super.initialize();
        _intake.setDesiredState(_desiredState);
        _intake.setIntakeVoltage(_desiredState.getBeltVoltage());
        _intake.setRollerVoltage(_desiredState.getRollerVoltage());
    }

    @Override
    protected void execute(double timestamp) {
        // TODO Auto-generated method stub

    }

    @Override
    public boolean isFinished() {
        log("is at desired", _intake.isAtDesiredAngle());

        return _intake.isAtDesiredAngle();
    }

    @Override
    public void end(boolean interrupted) {
        if (!interrupted) {
            _intake.setCurrentState(_desiredState);
        } else {
            _intake.mapToClosestState();
        }
        super.end(interrupted);
    }
}
