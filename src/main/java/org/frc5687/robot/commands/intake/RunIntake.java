package org.frc5687.robot.commands.intake;

import org.frc5687.robot.commands.OutliersCommand;
import org.frc5687.robot.subsystems.intake.IntakeSubsystem;

public class RunIntake extends OutliersCommand{
    private final IntakeSubsystem _intake;
    private final double _rollerVoltage;
    private final double _intakeVoltage;

    public RunIntake(IntakeSubsystem intake, double rollerVoltage, double intakeVoltage){
        _intake = intake;
        _rollerVoltage = rollerVoltage;
        _intakeVoltage = intakeVoltage;

        addRequirements(_intake);
    }

    @Override
    protected void execute(double timestamp) {
        _intake.setRollerVoltage(_rollerVoltage);
        _intake.setIntakeVoltage(_intakeVoltage);
    }
}
