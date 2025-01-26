package org.frc5687.robot.commands.intake;

import org.frc5687.robot.commands.OutliersCommand;
import org.frc5687.robot.subsystems.intake.IntakeSubsystem;

public class IdleIntake extends OutliersCommand {
    private final IntakeSubsystem _intake;

    public IdleIntake(IntakeSubsystem intake) {
        _intake = intake;

        addRequirements(_intake);
    }

    @Override
    protected void execute(double timestamp) {
        _intake.setRollerVoltage(0);
        _intake.setIntakeVoltage(0);
    }

    @Override
    public void end(boolean interrupted) {
        _intake.setRollerVoltage(0);
        _intake.setIntakeVoltage(0);
    }
}
