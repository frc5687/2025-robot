package org.frc5687.robot.commands.intake;

import org.frc5687.robot.commands.OutliersCommand;
import org.frc5687.robot.subsystems.intake.IntakeSubsystem;

public class RunIntake extends OutliersCommand{
    private final IntakeSubsystem _intake;
    private final double _voltage;
    public RunIntake(IntakeSubsystem intake, double voltage){
        _intake = intake;
        _voltage = voltage;

        addRequirements(_intake);
    }

    @Override
    protected void execute(double timestamp) {
        _intake.setRollerVoltage(_voltage);
    }
}
