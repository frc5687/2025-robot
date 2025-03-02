package org.frc5687.robot.commands.algae;

import org.frc5687.robot.commands.OutliersCommand;
import org.frc5687.robot.subsystems.algaearm.AlgaeArmSubsystem;
import org.frc5687.robot.subsystems.algaearm.AlgaeState;

public class EmergencyEjectAlgae extends OutliersCommand {

    private final AlgaeArmSubsystem _algae;

    public EmergencyEjectAlgae(AlgaeArmSubsystem algae) {
        _algae = algae;
        addRequirements(_algae);
    }

    @Override
    public void initialize() {}

    @Override
    protected void execute(double timestamp) {

        _algae.setArmAngle(AlgaeState.BARGE_DROPOFF);
        _algae.setAlgaeMotorVoltage(-12);
    }

    @Override
    public boolean isFinished() {
        return !_algae.isAlgaeDetected();
    }
}
