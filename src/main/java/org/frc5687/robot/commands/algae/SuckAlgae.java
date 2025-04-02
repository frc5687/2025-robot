package org.frc5687.robot.commands.algae;

import org.frc5687.robot.commands.OutliersCommand;
import org.frc5687.robot.subsystems.algaearm.AlgaeArmSubsystem;
import org.frc5687.robot.util.TunableDouble;

public class SuckAlgae extends OutliersCommand {
    private final AlgaeArmSubsystem _algaeArm;
    private static final TunableDouble asdf =
            new TunableDouble("SuckAlgae", "wheel oomph (volts)", 6);

    public SuckAlgae(AlgaeArmSubsystem algaeArmSubsystem) {
        _algaeArm = algaeArmSubsystem;
        addRequirements(_algaeArm);
    }

    @Override
    protected void execute(double timestamp) {
        _algaeArm.setWheelMotorVoltage(asdf.get());
    }
}
