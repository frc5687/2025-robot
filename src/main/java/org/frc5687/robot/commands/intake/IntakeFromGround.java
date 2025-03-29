package org.frc5687.robot.commands.intake;

import edu.wpi.first.wpilibj.Timer;
import org.frc5687.robot.Constants;
import org.frc5687.robot.commands.OutliersCommand;
import org.frc5687.robot.subsystems.intake.IntakeSubsystem;
import org.frc5687.robot.util.TunableDouble;

public class IntakeFromGround extends OutliersCommand {

    private static IntakeSubsystem _intake;

    private static final TunableDouble pulseStrength =
            new TunableDouble("Intake", "wheel oomph (volts)", 2);

    private static final TunableDouble dutyCycle =
            new TunableDouble("Intake", "duty Cycle (0-1)", 0.9);

    private static final TunableDouble period = new TunableDouble("Intake", "period", 0.2);

    public IntakeFromGround(IntakeSubsystem intake) {
        _intake = intake;
        addRequirements(_intake);
    }

    @Override
    protected void execute(double timestamp) {
        if (Math.abs(_intake.getBeltVelocity()) < Constants.Intake.PULSE_THRESHOLD) {
            if (isPulseActive()) {
                _intake.setVoltages(pulseStrength.get());
            } else {
                _intake.setVoltages(2);
            }
            System.out.println("pulsing intake" + timestamp);
        } else {
            _intake.setVoltages(Constants.Intake.INTAKE_VOLTAGE);
        }
    }

    private boolean isPulseActive() {
        double fract = Timer.getFPGATimestamp() % period.get();
        return fract <= dutyCycle.get() * period.get();
    }
}
