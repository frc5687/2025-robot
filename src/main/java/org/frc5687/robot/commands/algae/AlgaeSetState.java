package org.frc5687.robot.commands.algae;

import org.frc5687.robot.commands.OutliersCommand;
import org.frc5687.robot.subsystems.algaearm.AlgaeArmSubsystem;
import org.frc5687.robot.subsystems.algaearm.AlgaeState;

public class AlgaeSetState extends OutliersCommand {
    private final AlgaeArmSubsystem _algae;
    private final AlgaeState _desiredState;

    public AlgaeSetState(AlgaeArmSubsystem algae, AlgaeState state) {
        _algae = algae;
        _desiredState = state;
        addRequirements(_algae);
    }

    @Override
    public void initialize() {
        super.initialize();
        _algae.setWheelMotorVoltage(_desiredState.getWheelVoltage());
        _algae.setDesiredState(_desiredState);
    }

    @Override
    protected void execute(double timestamp) {}

    @Override
    public boolean isFinished() {
        log("is desired", _algae.isAtDesiredAngle());
        return _algae.isAtDesiredAngle();
    }

    @Override
    public void end(boolean interrupted) {
        if (!interrupted) {
            _algae.setCurrentState(_desiredState);
        } else {
            _algae.mapToClosestState();
        }
        super.end(interrupted);
    }
}
