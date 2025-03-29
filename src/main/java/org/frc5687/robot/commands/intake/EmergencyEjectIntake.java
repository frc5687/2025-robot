package org.frc5687.robot.commands.intake;

import org.frc5687.robot.commands.OutliersCommand;
import org.frc5687.robot.subsystems.intake.IntakeSubsystem;

public class EmergencyEjectIntake extends OutliersCommand {

    private final IntakeSubsystem _intake;

    public EmergencyEjectIntake(IntakeSubsystem intake) {
        _intake = intake;
    }

    @Override
    public void initialize() {
        super.initialize();
        _intake.setIntakeVoltage(-12);
    }

    @Override
    protected void execute(double timestamp) {}

    @Override
    public boolean isFinished() {
        return super.isFinished();
    }

    @Override
    public void end(boolean interrupted) {
        _intake.setIntakeVoltage(0);
        super.end(interrupted);
    }
}
