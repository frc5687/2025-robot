package org.frc5687.robot.commands.elevator;

import java.util.function.DoubleSupplier;

import org.frc5687.robot.commands.OutliersCommand;
import org.frc5687.robot.subsystems.elevator.ElevatorSubsystem;

public class SetElevatorPosition extends OutliersCommand{
    
        private final ElevatorSubsystem _elevator;
        // private final double _heightMeters;
        DoubleSupplier _positionSupplier;
    public SetElevatorPosition(ElevatorSubsystem elevator, DoubleSupplier positionSupplier){
        _elevator = elevator;
        // _heightMeters = heightMeters;
        _positionSupplier = positionSupplier;

        addRequirements(_elevator);
    }

    @Override
    public void initialize() {
        super.initialize();
    }

    @Override
    protected void execute(double timestamp) {
        double height = _positionSupplier.getAsDouble();
       _elevator.setPositionMeters(height);
    //    System.out.println(height);
    }

    @Override
    public boolean isFinished() {
        return super.isFinished();
    }

    @Override
    public void end(boolean interrupted) {
    
    }
}
