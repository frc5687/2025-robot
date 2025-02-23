package org.frc5687.robot.commands.algae;

import edu.wpi.first.wpilibj.Timer;
import org.frc5687.robot.commands.OutliersCommand;
import org.frc5687.robot.subsystems.algaearm.AlgaeArmSubsystem;
import org.frc5687.robot.util.TunableDouble;

public class IdleAlgae extends OutliersCommand {
    private final AlgaeArmSubsystem _algaeArm;
    private static final TunableDouble asdf = new TunableDouble("AlgaeArm", "wheel oomph (volts)", 2);
    private static final TunableDouble dutyCycle =
            new TunableDouble("AlgaeArm", "duty Cycle (0-1)", 0.9);

    private static final TunableDouble period = new TunableDouble("AlgaeArm", "period", 0.2);

    public IdleAlgae(AlgaeArmSubsystem algaeArmSubsystem) {
        _algaeArm = algaeArmSubsystem;
        addRequirements(_algaeArm);
    }

    @Override
    protected void execute(double timestamp) {
        if (_algaeArm.getInputs().isAlgaeDetected && isPulseActive()) {
            _algaeArm.setWheelMotorVoltage(asdf.get());
        } else {
            _algaeArm.setWheelMotorVoltage(0);
        }
    }

    private boolean isPulseActive() {
        double fract = Timer.getFPGATimestamp() % period.get();
        return fract <= dutyCycle.get() * period.get();
    }
}
