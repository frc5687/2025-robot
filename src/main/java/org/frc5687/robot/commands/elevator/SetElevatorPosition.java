package org.frc5687.robot.commands.elevator;

import org.frc5687.robot.commands.OutliersCommand;
import org.frc5687.robot.subsystems.elevator.ElevatorSubsystem;

public class SetElevatorPosition extends OutliersCommand{
    
        private final ElevatorSubsystem _elevator;
        private final double _heightMeters;
    public SetElevatorPosition(ElevatorSubsystem elevator, double heightMeters){
        _elevator = elevator;
        _heightMeters = heightMeters;
    }

    @Override
    public void initialize() {
        super.initialize();
    }

    @Override
    protected void execute(double timestamp) {
       
    }

    @Override
    public boolean isFinished() {
        return super.isFinished();
    }

    @Override
    public void end(boolean interrupted) {
    
    }
}
