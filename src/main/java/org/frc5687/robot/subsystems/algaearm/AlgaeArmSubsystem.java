package org.frc5687.robot.subsystems.algaearm;

import org.frc5687.robot.subsystems.OutliersSubsystem;

public class AlgaeArmSubsystem extends OutliersSubsystem<AlgaeInputs, AlgaeOutputs>{

    public AlgaeArmSubsystem(AlgaeArmIO io){
        super(io, new AlgaeInputs(), new AlgaeOutputs());
    }
    @Override
    protected void processInputs() {
        
    }

    @Override
    protected void periodic(AlgaeInputs inputs, AlgaeOutputs outputs) {
       
    }
    
    public void setArmAngle(double angle){
        _outputs.algaeArmTargetAngle = angle;
    }

    public void setAlgaeMotorVoltage(double voltage){
        _outputs.algaeMotorVoltage = voltage;
    }
}
