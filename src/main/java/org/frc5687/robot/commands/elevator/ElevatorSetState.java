package org.frc5687.robot.commands.elevator;

import org.frc5687.robot.commands.OutliersCommand;
import org.frc5687.robot.subsystems.elevator.ElevatorState;
import org.frc5687.robot.subsystems.elevator.ElevatorSubsystem;

public class ElevatorSetState extends OutliersCommand {
    private final ElevatorSubsystem _elevator;
    private final ElevatorState _desiredState;

    public ElevatorSetState(ElevatorSubsystem elevator, ElevatorState state) {
        _elevator = elevator;
        _desiredState = state;
        addRequirements(_elevator);
    }

    @Override
    public void initialize() {
        _elevator.setDesiredState(_desiredState);
        log("ElevatorDesiredState", _desiredState);
    }

    @Override
    public boolean isFinished() {
        return _elevator.isAtDesiredPosition();
    }

    @Override
    protected void execute(double timestamp) {}

    @Override
    public void end(boolean interrupted) {
        if (!interrupted) {
            // System.out.println(
            //         String.format(
            //                 "Setting State: %s at height: %4.5f",
            //                 _desiredState, _elevator.getPlatformWorldHeight()));
            _elevator.setCurrentState(_desiredState);
        } else {
            _elevator.mapToClosestState();
        }
        super.end(interrupted);
    }
}
