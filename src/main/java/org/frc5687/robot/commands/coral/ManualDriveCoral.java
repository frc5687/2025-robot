package org.frc5687.robot.commands.coral;

import java.util.function.DoubleSupplier;
import org.frc5687.robot.commands.OutliersCommand;
import org.frc5687.robot.subsystems.coralarm.CoralArmSubsystem;

public class ManualDriveCoral extends OutliersCommand {

    private CoralArmSubsystem _coralArmSubsystem;
    private DoubleSupplier _doubleSupplier;

    public ManualDriveCoral(CoralArmSubsystem coralArmSubsystem, DoubleSupplier doubleSupplier) {
        _coralArmSubsystem = coralArmSubsystem;
        _doubleSupplier = doubleSupplier;
        addRequirements(_coralArmSubsystem);
    }

    @Override
    protected void execute(double timestamp) {
        _coralArmSubsystem.setWheelVoltageCommand(_doubleSupplier.getAsDouble());
    }

    @Override
    public void end(boolean interrupted) {
        super.end(interrupted);
    }
}
