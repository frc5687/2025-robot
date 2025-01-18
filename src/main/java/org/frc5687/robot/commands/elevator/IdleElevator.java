package org.frc5687.robot.commands.elevator;

import org.frc5687.robot.commands.OutliersCommand;
import org.frc5687.robot.subsystems.elevator.ElevatorSubsystem;

public class IdleElevator extends OutliersCommand{

    private final ElevatorSubsystem _elevator;
    
    public IdleElevator(ElevatorSubsystem elevator){
        _elevator = elevator;
        addRequirements(_elevator);
    }
    @Override
    protected void execute(double timestamp) {
    
    }
    
    @Override
    public boolean isFinished() {
        // TODO Auto-generated method stub
        return super.isFinished();

    }

    @Override
    public void end(boolean interrupted) {
        // TODO Auto-generated method stub
        super.end(interrupted);
    }
}
