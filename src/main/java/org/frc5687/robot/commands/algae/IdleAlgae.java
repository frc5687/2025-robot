package org.frc5687.robot.commands.algae;

import org.frc5687.robot.commands.OutliersCommand;
import org.frc5687.robot.subsystems.algaearm.AlgaeArmSubsystem;

public class IdleAlgae extends OutliersCommand{
    AlgaeArmSubsystem _algaeArm;

    public IdleAlgae(AlgaeArmSubsystem algaeArmSubsystem){
        _algaeArm = algaeArmSubsystem;
        addRequirements(_algaeArm);
    }

    @Override
    protected void execute(double timestamp) {
        //_algaeArm.setAlgaeMotorVoltage(0);
        //_algaeArm.setArmAngle(0);//FIXME:dont run without changing
    }
}
