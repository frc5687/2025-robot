package org.frc5687.robot.commands.elevator;

import org.frc5687.robot.commands.OutliersCommand;
import org.frc5687.robot.subsystems.elevator.ElevatorSubsystem;

public class SetElevatorHeight extends OutliersCommand {
    private final ElevatorSubsystem _elevator;
    private final double _heightMeters;

    public SetElevatorHeight(ElevatorSubsystem elevator, double heightMeters) {
        _elevator = elevator;
        _heightMeters = heightMeters;
        addRequirements(_elevator);
    }

    @Override
    public void initialize() {
        super.initialize();
        _elevator.setDesiredPlatformHeightWorld(_heightMeters);
    }

    @Override
    protected void execute(double timestamp) {}

    @Override
    public boolean isFinished() {
        return _elevator.isAtDesiredPosition();
    }

    @Override
    public void end(boolean interrupted) {
        super.end(interrupted);
    }
}