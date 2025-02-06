package org.frc5687.robot.commands.coral;

import org.frc5687.robot.commands.OutliersCommand;
import org.frc5687.robot.subsystems.coralarm.CoralArmSubsystem;
import org.frc5687.robot.subsystems.coralarm.CoralState;

public class CoralSetState extends OutliersCommand {
    private final CoralArmSubsystem _coral;
    private final CoralState _desiredState;

    public CoralSetState(CoralArmSubsystem coral, CoralState state) {
        _coral = coral;
        _desiredState = state;
        addRequirements(_coral);
    }

    @Override
    public void initialize() {
        super.initialize();
        _coral.setDesiredState(_desiredState);
    }

    @Override
    protected void execute(double timestamp) {}

    @Override
    public boolean isFinished() {
        log("At desired", _coral.isAtDesiredAngle());
        return _coral.isAtDesiredAngle();
    }

    @Override
    public void end(boolean interrupted) {
        if (!interrupted) {
            _coral.setCurrentState(_desiredState);
        } else {
            _coral.mapToClosestState();
        }
        super.end(interrupted);
    }
}
