package org.frc5687.robot.commands.algae;

import org.frc5687.robot.commands.OutliersCommand;
import org.frc5687.robot.subsystems.algaearm.AlgaeArmSubsystem;

public class IntakeAlgae extends OutliersCommand {

    AlgaeArmSubsystem _algaeArm;

    public IntakeAlgae(AlgaeArmSubsystem algaeArmSubsystem) {
        _algaeArm = algaeArmSubsystem;
        addRequirements(_algaeArm);
    }

    @Override
    protected void execute(double timestamp) {
        _algaeArm.setWheelMotorVoltage(12);
    }

    @Override
    public boolean isFinished() {
        return _algaeArm.isAlgaeDetected();
    }
}
