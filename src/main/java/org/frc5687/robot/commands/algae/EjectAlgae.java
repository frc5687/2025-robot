package org.frc5687.robot.commands.algae;

import org.frc5687.robot.commands.OutliersCommand;
import org.frc5687.robot.subsystems.algaearm.AlgaeArmSubsystem;

public class EjectAlgae extends OutliersCommand {

    AlgaeArmSubsystem _algae;

    public EjectAlgae(AlgaeArmSubsystem algae) {
        _algae = algae;
    }

    @Override
    public void initialize() {}

    @Override
    protected void execute(double timestamp) {
        if (_algae.isSafeToEject()) {
            _algae.setWheelMotorVoltage(-12);
        } else {
            _algae.setWheelMotorVoltage(0);
        }
    }

    @Override
    public void end(boolean interrupted) {
        _algae.setWheelMotorVoltage(0);
    }
}
