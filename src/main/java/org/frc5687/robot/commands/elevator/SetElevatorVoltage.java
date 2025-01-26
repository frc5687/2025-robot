package org.frc5687.robot.commands.elevator;

import java.util.function.DoubleSupplier;

import org.frc5687.robot.OperatorInterface;
import org.frc5687.robot.commands.OutliersCommand;
import org.frc5687.robot.subsystems.elevator.ElevatorSubsystem;

public class SetElevatorVoltage extends OutliersCommand{
    
        private final ElevatorSubsystem _elevator;
        
        private final DoubleSupplier _voltageSupplier;
    public SetElevatorVoltage(
    ElevatorSubsystem elevator, 
    DoubleSupplier voltageSupplier
   ){
        _elevator = elevator;
        _voltageSupplier = voltageSupplier;
        addRequirements(_elevator);
    }

    @Override
    public void initialize() {
        super.initialize();
    }

    @Override
    protected void execute(double timestamp) {
        double voltage = _voltageSupplier.getAsDouble();
       _elevator.setVoltage(voltage);
    }

    @Override
    public boolean isFinished() {
        return super.isFinished();
    }

    @Override
    public void end(boolean interrupted) {
        _elevator.setVoltage(0);
    }
}
