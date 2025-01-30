package org.frc5687.robot.commands.intake;

import java.util.function.DoubleSupplier;

import org.frc5687.robot.commands.OutliersCommand;
import org.frc5687.robot.subsystems.intake.IntakeSubsystem;

public class RunIntake extends OutliersCommand {
    private final IntakeSubsystem _intake;
    private final double _rollerVoltage;
    private final double _intakeVoltage;
    private final DoubleSupplier _angleSupplier;
    public RunIntake(IntakeSubsystem intake, double rollerVoltage, double intakeVoltage, DoubleSupplier angleSupplier) {
        _intake = intake;
        _rollerVoltage = rollerVoltage;
        _intakeVoltage = intakeVoltage;
        _angleSupplier = angleSupplier;

        addRequirements(_intake);
    }

    @Override
    protected void execute(double timestamp) {
        double targetAngle = Math.atan2(0, _angleSupplier.getAsDouble());

        _intake.setRollerVoltage(_rollerVoltage);
        _intake.setIntakeVoltage(_intakeVoltage);
        _intake.setPivotAngle(targetAngle);
    }

    @Override
    public void end(boolean interrupted){
        _intake.setRollerVoltage(0);
        _intake.setIntakeVoltage(0);
    }
}
