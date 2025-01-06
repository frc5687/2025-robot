package org.frc5687.robot.subsystems.intake;

import org.frc5687.robot.subsystems.OutliersSubsystem;
import org.frc5687.robot.subsystems.SubsystemIO;

public class IntakeSubsystem extends OutliersSubsystem<IntakeInputs, IntakeOutputs> {


    public IntakeSubsystem(
        SubsystemIO<IntakeInputs, IntakeOutputs> io) {
            super(io, new IntakeInputs(), new IntakeOutputs());
        }
    @Override
    protected void processInputs() {
        
    }

    @Override
    protected void periodic(IntakeInputs inputs, IntakeOutputs outputs) {
       

    }
    
    public void setRollerVoltage(double voltage){
        _outputs.rollerVoltage = voltage;
    }
}
